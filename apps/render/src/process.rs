//! 逐像素处理工作线程：灰度化 + 深度线性化/噪声 + 调试显示映射。
//!
//! iceoryx2 发布端口在工作线程内创建和使用；UI 只读取最新的显示结果。
//!
//! 深度退化由 `depth_noise` 配置，在线测量施加噪声；离线地图路标使用理想几何深度标签。

use crate::capture::CaptureTiming;
use std::sync::{Condvar, Mutex};
use std::time::Instant;

use crate::depth_noise::{self, DepthNoiseOptions};
use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH};
use firefly_pubsub::trace::TraceContext;

use crate::rig::{FOV_Y_DEG, SENSOR_NEAR};
use crate::sensors;

/// 一拍的原始输入（主世界装配后交给工作线程）。
#[derive(Debug)]
pub struct CaptureJob {
    /// 单调序号。
    pub seq: u64,
    pub timing: CaptureTiming,
    /// 位姿时间戳（秒，sim 时间）。
    pub stamp: f64,
    /// 上游 trace 上下文（发布时续接）。
    pub trace: TraceContext,
    /// 左目紧凑 `RGBA8`（行主序无填充）。
    pub left_rgb: Vec<u8>,
    /// 右目紧凑 `RGBA8`。
    pub right_rgb: Vec<u8>,
    /// 深度预通道原始字节（小端 `f32`）。
    pub depth_raw: Vec<u8>,
}

/// 一拍的全部输出（工作线程算好后交回主世界发布/显示）。
#[derive(Debug)]
pub struct ProcessedCapture {
    /// 单调序号。
    pub seq: u64,
    pub timing: CaptureTiming,
    /// 位姿时间戳（秒）。
    pub stamp: f64,
    /// 上游 trace 上下文。
    pub trace: TraceContext,
    /// 左目 RGB（直显用）。
    pub left_rgb: Vec<u8>,
    /// 右目 RGB（直显用）。
    pub right_rgb: Vec<u8>,
    /// 左目灰度（发布）。
    pub left_gray: Vec<u8>,
    /// 右目灰度（发布）。
    pub right_gray: Vec<u8>,
    /// 深度（米，行主序；在线含退化，离线为几何标签）。
    pub depth: Vec<f32>,
    /// 深度伪彩显示图（`RGBA8`）。
    pub depth_rgba: Vec<u8>,
}

/// 有界最新值邮箱：生产者不等待消费者；关闭唤醒接收端并丢弃未处理任务。
pub struct Latest<T> {
    state: Mutex<(Option<T>, bool)>,
    ready: Condvar,
}

impl<T> Default for Latest<T> {
    fn default() -> Self {
        Self {
            state: Mutex::new((None, false)),
            ready: Condvar::new(),
        }
    }
}

impl<T> Latest<T> {
    pub fn replace(&self, value: T) {
        let mut state = self.state.lock().expect("capture mailbox poisoned");
        if !state.1 {
            state.0 = Some(value);
            self.ready.notify_one();
        }
    }

    pub fn take(&self) -> Option<T> {
        self.state
            .lock()
            .expect("capture mailbox poisoned")
            .0
            .take()
    }

    pub fn recv(&self) -> Option<T> {
        let state = self.state.lock().expect("capture mailbox poisoned");
        let mut state = self
            .ready
            .wait_while(state, |s| s.0.is_none() && !s.1)
            .expect("capture mailbox poisoned");
        state.0.take()
    }

    pub fn close(&self) {
        *self.state.lock().expect("capture mailbox poisoned") = (None, true);
        self.ready.notify_all();
    }
}

/// 像素处理后立即发布；显示端不参与工作线程的进度与背压。
pub fn run_worker(
    jobs: &Latest<CaptureJob>,
    results: &Latest<ProcessedCapture>,
    options: &DepthNoiseOptions,
    offline: bool,
    mut publish: impl FnMut(&ProcessedCapture),
) {
    while let Some(job) = jobs.recv() {
        let root = job.trace.continue_span("render-pixels").unwrap_or_else(|| {
            fastrace::Span::root(
                "render-pixels",
                fastrace::prelude::SpanContext::random().sampled(false),
            )
        });
        let guard = root.set_local_parent();
        let result = process(job, options, offline);
        publish(&result);
        results.replace(result);
        drop(guard);
        fastrace::flush();
    }
}

/// 单拍像素管线。
pub fn process(
    mut job: CaptureJob,
    options: &DepthNoiseOptions,
    offline: bool,
) -> ProcessedCapture {
    job.timing.started = Instant::now();
    let mut left_gray = vec![0u8; IMAGE_SIZE];
    let mut right_gray = vec![0u8; IMAGE_SIZE];
    sensors::rgb_to_gray(&job.left_rgb, &mut left_gray);
    sensors::rgb_to_gray(&job.right_rgb, &mut right_gray);

    let mut depth = vec![0.0f32; IMAGE_SIZE];
    sensors::linearize_depth(&job.depth_raw, &mut depth, SENSOR_NEAR);
    if !offline {
        depth_noise::apply(
            &mut depth,
            IMAGE_WIDTH,
            IMAGE_HEIGHT,
            (IMAGE_HEIGHT as f32 / 2.0) / (FOV_Y_DEG.to_radians() / 2.0).tan(),
            job.stamp,
            options,
        );
    }

    let mut depth_rgba = vec![0u8; 4 * IMAGE_SIZE];
    sensors::depth_to_display(&depth, &mut depth_rgba, sensors::DISPLAY_DEPTH_RANGE);

    job.timing.processed = Instant::now();
    ProcessedCapture {
        seq: job.seq,
        timing: job.timing,
        stamp: job.stamp,
        trace: job.trace,
        left_rgb: job.left_rgb,
        right_rgb: job.right_rgb,
        left_gray,
        right_gray,
        depth,
        depth_rgba,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn latest_mailbox_overwrites_pending_and_close_wakes_receiver() {
        let jobs = Latest::default();
        jobs.replace(1);
        jobs.replace(2);
        assert_eq!(jobs.recv(), Some(2));
        assert_eq!(jobs.take(), None);
        std::thread::scope(|scope| {
            let worker = scope.spawn(|| jobs.recv());
            jobs.close();
            assert_eq!(worker.join().unwrap(), None);
        });
        jobs.replace(3);
        assert_eq!(jobs.recv(), None);
        let pending = Latest::default();
        pending.replace(4);
        pending.close();
        assert_eq!(pending.take(), None);
    }

    #[test]
    fn worker_publishes_without_ui_polling_and_preserves_capture_stamp_and_pixels() {
        let jobs = Latest::default();
        let results = Latest::default();
        let (published, received) = std::sync::mpsc::channel();
        std::thread::scope(|scope| {
            let worker = scope.spawn(|| {
                run_worker(
                    &jobs,
                    &results,
                    &DepthNoiseOptions::default(),
                    true,
                    |result| {
                        published
                            .send((
                                result.seq,
                                result.stamp,
                                result.left_gray[0],
                                result.right_gray[0],
                            ))
                            .unwrap();
                    },
                );
            });
            let mut observations = Vec::new();
            for seq in 1..=2 {
                jobs.replace(CaptureJob {
                    seq,
                    stamp: 7. + seq as f64 / 10.,
                    timing: CaptureTiming::default(),
                    trace: TraceContext::empty(),
                    left_rgb: vec![128; 4 * IMAGE_SIZE],
                    right_rgb: vec![64; 4 * IMAGE_SIZE],
                    depth_raw: vec![0; 4 * IMAGE_SIZE],
                });
                // 有限等待只检测线程死锁；不把处理耗时作为数值正确性门槛。
                let value = received.recv_timeout(std::time::Duration::from_secs(10));
                if value.is_err() {
                    jobs.close();
                }
                observations.push(value.unwrap());
            }
            jobs.close();
            worker.join().unwrap();
            for (index, (id, stamp, left, right)) in observations.into_iter().enumerate() {
                let seq = index as u64 + 1;
                assert_eq!(id, seq);
                assert!((stamp - (7. + seq as f64 / 10.)).abs() < f64::EPSILON);
                assert_eq!((left, right), (128, 64));
            }
        });
        assert_eq!(results.take().unwrap().seq, 2);
    }

    #[test]
    fn offline_depth_is_geometry_independent_of_sensor_range_and_noise() {
        let options = DepthNoiseOptions {
            max_depth_m: 4.,
            ..Default::default()
        };
        let job = || CaptureJob {
            seq: 1,
            timing: CaptureTiming::default(),
            stamp: 1.,
            trace: TraceContext::empty(),
            left_rgb: vec![128; 4 * IMAGE_SIZE],
            right_rgb: vec![128; 4 * IMAGE_SIZE],
            depth_raw: (0..IMAGE_SIZE)
                .flat_map(|_| (SENSOR_NEAR / 10.).to_le_bytes())
                .collect(),
        };
        let ideal = process(job(), &options, true);
        assert!(ideal.depth.iter().all(|z| (*z - 10.).abs() < 1e-5));
        let measured = process(job(), &options, false);
        assert!(measured.depth.iter().all(|z| *z == 0.));
    }

    /// 单拍管线产出各缓冲尺寸正确；全清空深度保持无效 `0.0`（显示全黑）。
    #[test]
    fn process_produces_expected_buffers() {
        let options = DepthNoiseOptions::default();
        let job = CaptureJob {
            seq: 3,
            timing: CaptureTiming::default(),
            stamp: 1.5,
            trace: TraceContext::empty(),
            left_rgb: vec![128u8; 4 * IMAGE_SIZE],
            right_rgb: vec![64u8; 4 * IMAGE_SIZE],
            depth_raw: vec![0u8; 4 * IMAGE_SIZE],
        };
        let out = process(job, &options, false);
        assert_eq!(out.seq, 3);
        assert!((out.stamp - 1.5).abs() < 1e-9);
        assert_eq!(out.left_gray.len(), IMAGE_SIZE);
        assert_eq!(out.right_gray.len(), IMAGE_SIZE);
        assert_eq!(out.depth.len(), IMAGE_SIZE);
        assert_eq!(out.depth_rgba.len(), 4 * IMAGE_SIZE);
        // 原始深度全为清空值 → 全无效，显示为黑。
        assert!(out.depth.iter().all(|z| *z == 0.0));
        assert!(
            out.depth_rgba
                .as_chunks::<4>()
                .0
                .iter()
                .all(|px| px[0] == 0)
        );
    }
}
