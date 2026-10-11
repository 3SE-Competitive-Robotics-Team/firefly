//! 回环推理独占会话；零容量交接，忙时丢帧，不阻塞地图定位或积压旧图像。
use crate::online::{Online, Options};
use firefly_pubsub::{
    odom::OdomMessage,
    vision::{FeatureMessage, LoopConstraint},
};
use firefly_vision_match::{
    calibration::{left_from_depth, pinhole},
    depth::{RegisteredDepth, register_depth},
};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
    mpsc,
};

struct Input {
    feat: FeatureMessage,
    odom: OdomMessage,
    depth: Arc<[f32]>,
}
pub struct Output {
    pub timestamp: f64,
    pub edge: Option<LoopConstraint>,
    pub keyframe: Option<OdomMessage>,
    pub diagnostics: [f64; 4],
}
pub struct Worker {
    input: mpsc::SyncSender<Input>,
    output: mpsc::Receiver<Output>,
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}
impl Worker {
    pub fn start(
        mut session: ort::session::Session,
        options: Options,
    ) -> Result<Self, firefly_error::Error> {
        let mut online = Online::new(options);
        Self::spawn(move |input, depth, cancel| {
            let count = online.diagnostics[0];
            let edge = match online.process(&mut session, &input.feat, input.odom, depth, || {
                cancel.load(Ordering::Relaxed)
            }) {
                Ok(edge) => edge,
                Err(e) => {
                    online.miss();
                    log::warn!("回环验证失败: {e}");
                    None
                }
            };
            Output {
                timestamp: input.feat.timestamp,
                edge,
                keyframe: (online.diagnostics[0] > count).then_some(input.odom),
                diagnostics: online.diagnostics,
            }
        })
    }

    fn spawn(
        mut process: impl FnMut(&Input, &RegisteredDepth, &AtomicBool) -> Output + Send + 'static,
    ) -> firefly_error::Result<Self> {
        let (input, rx) = mpsc::sync_channel::<Input>(0);
        let (tx, output) = mpsc::sync_channel(1);
        let stop = Arc::new(AtomicBool::new(false));
        let cancel = stop.clone();
        let thread = std::thread::Builder::new()
            .name("loop-inference".into())
            // 推理线程要容纳 256KB 的 `Input`（特征消息按值进通道）+ ORT 推理与
            // 匹配缓冲区；默认 2MiB 会溢出，显式给足。
            .stack_size(16 * 1024 * 1024)
            .spawn(move || {
                while !cancel.load(Ordering::Relaxed) {
                    let input = match rx.recv_timeout(std::time::Duration::from_millis(50)) {
                        Ok(value) => value,
                        Err(mpsc::RecvTimeoutError::Timeout) => continue,
                        Err(_) => break,
                    };
                    let root = fastrace::Span::root(
                        "online_loop",
                        fastrace::prelude::SpanContext::random(),
                    );
                    let guard = root.set_local_parent();
                    if cancel.load(Ordering::Relaxed) {
                        break;
                    }
                    let depth = match register_depth(
                        &input.depth,
                        pinhole(),
                        pinhole(),
                        &left_from_depth(),
                    ) {
                        Ok(depth) => depth,
                        Err(error) => {
                            log::warn!("回环深度注册失败: {error}");
                            drop(guard);
                            drop(root);
                            fastrace::flush();
                            continue;
                        }
                    };
                    let mut result = process(&input, &depth, &cancel);
                    // 结果必须交付；等待期间不接收新任务，退出时可以放弃尚未发布的结果。
                    while !cancel.load(Ordering::Relaxed) {
                        match tx.try_send(result) {
                            Ok(()) => break,
                            Err(mpsc::TrySendError::Full(value)) => {
                                result = value;
                                std::thread::sleep(std::time::Duration::from_millis(5));
                            }
                            Err(_) => return,
                        }
                    }
                    drop(guard);
                    drop(root);
                    fastrace::flush();
                }
            })
            .map_err(|e| {
                firefly_error::Error::new(firefly_error::ErrorKind::Internal, e.to_string())
            })?;
        Ok(Self {
            input,
            output,
            stop,
            thread: Some(thread),
        })
    }
    pub fn submit(&self, feat: &FeatureMessage, odom: OdomMessage, depth: Arc<[f32]>) -> bool {
        self.input
            .try_send(Input {
                feat: *feat,
                odom,
                depth,
            })
            .is_ok()
    }
    pub fn results(&self) -> impl Iterator<Item = Output> + '_ {
        self.output.try_iter()
    }
}
impl Drop for Worker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 调用方在显式 16 MiB 栈上构造定长 IPC 消息。
    #[allow(clippy::large_stack_arrays)]
    fn input(timestamp: f64, depth: Arc<[f32]>) -> Input {
        Input {
            feat: FeatureMessage {
                timestamp,
                count: 0,
                keypoints: [[0.; 2]; firefly_pubsub::vision::MAX_FEATURES],
                descriptors: [[0.; firefly_pubsub::vision::DESC_DIM];
                    firefly_pubsub::vision::MAX_FEATURES],
                scores: [0.; firefly_pubsub::vision::MAX_FEATURES],
            },
            odom: OdomMessage {
                timestamp,
                is_initialized: true,
                ..Default::default()
            },
            depth,
        }
    }

    #[test]
    fn rendezvous_drops_busy_frames_and_shutdown_releases_pending_output() {
        std::thread::Builder::new()
            .stack_size(16 * 1024 * 1024)
            .spawn(|| {
                let (started, starts) = mpsc::channel();
                let (resume, resumes) = mpsc::channel();
                let worker = Worker::spawn(move |input, depth, _| {
                    assert_eq!(input.feat.timestamp, input.odom.timestamp);
                    let point = depth.point_at([160., 120.]).unwrap();
                    assert!((point.z + 2.).abs() < 1e-12);
                    started.send(input.feat.timestamp).unwrap();
                    resumes.recv().unwrap();
                    Output {
                        timestamp: input.feat.timestamp,
                        edge: None,
                        keyframe: Some(input.odom),
                        diagnostics: [0.; 4],
                    }
                })
                .unwrap();
                let pixels: Arc<[f32]> = vec![2.; 320 * 240].into();
                assert!(worker.input.send(input(1., pixels.clone())).is_ok());
                assert_eq!(starts.recv().unwrap(), 1.);
                let rejected = input(2., pixels.clone());
                assert!(!worker.submit(&rejected.feat, rejected.odom, rejected.depth));
                // 工作线程停在处理回调内，第二帧必须立即被丢弃且不保留快照。
                assert_eq!(Arc::strong_count(&pixels), 2);
                resume.send(()).unwrap();
                assert!(worker.input.send(input(3., pixels.clone())).is_ok());
                assert_eq!(starts.recv().unwrap(), 3.);
                assert_eq!(worker.output.try_recv().unwrap().timestamp, 1.);
                resume.send(()).unwrap();
                // 第三次接收证明前一份结果已提交；保留该结果使下一份结果遇到背压。
                assert!(worker.input.send(input(4., pixels.clone())).is_ok());
                assert_eq!(starts.recv().unwrap(), 4.);
                let rejected = input(5., pixels.clone());
                assert!(!worker.submit(&rejected.feat, rejected.odom, rejected.depth));
                resume.send(()).unwrap();
                let stop = worker.stop.clone();
                drop(worker);
                assert!(stop.load(Ordering::Relaxed));
                assert_eq!(Arc::strong_count(&pixels), 1);
                assert!(starts.try_recv().is_err());
                // 空闲接收中的线程也须响应 Drop，无需新任务唤醒。
                let idle = Worker::spawn(|_, _, _| panic!("no task submitted")).unwrap();
                drop(idle);
            })
            .unwrap()
            .join()
            .unwrap();
    }
}
