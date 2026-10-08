//! 回环推理独占会话；零容量交接，忙时丢帧，不阻塞地图定位或积压旧图像。
use crate::online::{Online, Options};
use firefly_pubsub::{
    odom::OdomMessage,
    vision::{FeatureMessage, LoopConstraint},
};
use firefly_vision_match::depth::RegisteredDepth;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
    mpsc,
};

struct Input {
    feat: FeatureMessage,
    odom: OdomMessage,
    depth: RegisteredDepth,
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
                let mut online = Online::new(options);
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
                    let count = online.diagnostics[0];
                    let edge = match online.process(
                        &mut session,
                        &input.feat,
                        input.odom,
                        &input.depth,
                        || cancel.load(Ordering::Relaxed),
                    ) {
                        Ok(edge) => edge,
                        Err(e) => {
                            online.miss();
                            log::warn!("回环验证失败: {e}");
                            None
                        }
                    };
                    let mut result = Output {
                        timestamp: input.feat.timestamp,
                        edge,
                        keyframe: (online.diagnostics[0] > count).then_some(input.odom),
                        diagnostics: online.diagnostics,
                    };
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
    pub fn submit(&self, feat: &FeatureMessage, odom: OdomMessage, depth: RegisteredDepth) {
        let _ = self.input.try_send(Input {
            feat: *feat,
            odom,
            depth,
        });
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
