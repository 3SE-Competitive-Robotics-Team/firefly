//! 像素处理工作线程：接收 GPU 灰度 + 深度线性化/噪声。
//!
//! iceoryx2 发布端口在工作线程内创建和使用。
//!
//! 深度退化由 `depth_noise` 配置，在线测量施加噪声；离线地图路标使用理想几何深度标签。

use crate::buffer::PooledBuffer;
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
    /// 左目紧凑灰度（行主序无填充）。
    pub left_gray: PooledBuffer,
    /// 右目紧凑灰度。
    pub right_gray: PooledBuffer,
    /// 深度预通道原始字节（小端 `f32`）。
    pub depth_raw: PooledBuffer,
}

/// 一拍的全部输出（工作线程内发布）。
#[derive(Debug)]
pub struct ProcessedCapture<'a> {
    /// 单调序号。
    pub seq: u64,
    pub timing: CaptureTiming,
    /// 位姿时间戳（秒）。
    pub stamp: f64,
    /// 上游 trace 上下文。
    pub trace: TraceContext,
    /// 左目灰度（发布）。
    pub left_gray: PooledBuffer,
    /// 右目灰度（发布）。
    pub right_gray: PooledBuffer,
    /// 深度（米，行主序；在线含退化，离线为几何标签）。
    pub depth: &'a [f32],
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

    #[cfg(test)]
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

/// 像素处理后立即发布。
pub fn run_worker(
    jobs: &Latest<CaptureJob>,
    options: &DepthNoiseOptions,
    offline: bool,
    mut publish: impl FnMut(&ProcessedCapture),
) {
    let mut processor = PixelProcessor::default();
    while let Some(job) = jobs.recv() {
        let root = job.trace.continue_span("render-pixels").unwrap_or_else(|| {
            fastrace::Span::root(
                "render-pixels",
                fastrace::prelude::SpanContext::random().sampled(false),
            )
        });
        let guard = root.set_local_parent();
        let result = processor.process(job, options, offline);
        publish(&result);
        drop(guard);
        fastrace::flush();
    }
}

/// 像素线程自有的工作缓冲；发布完成后用于下一拍。
pub struct PixelProcessor {
    depth: Vec<f32>,
    clean_depth: Vec<f32>,
}

impl Default for PixelProcessor {
    fn default() -> Self {
        Self {
            depth: vec![0.; IMAGE_SIZE],
            clean_depth: vec![0.; IMAGE_SIZE],
        }
    }
}

impl PixelProcessor {
    /// 单拍像素管线；结果借用工作缓冲，必须在下一拍前完成发布。
    pub fn process(
        &mut self,
        mut job: CaptureJob,
        options: &DepthNoiseOptions,
        offline: bool,
    ) -> ProcessedCapture<'_> {
        job.timing.started = Instant::now();
        sensors::linearize_depth(&job.depth_raw, &mut self.depth, SENSOR_NEAR);
        if !offline {
            depth_noise::apply(
                &mut self.depth,
                IMAGE_WIDTH,
                IMAGE_HEIGHT,
                (IMAGE_HEIGHT as f32 / 2.0) / (FOV_Y_DEG.to_radians() / 2.0).tan(),
                job.stamp,
                options,
                &mut self.clean_depth,
            );
        }

        job.timing.processed = Instant::now();
        ProcessedCapture {
            seq: job.seq,
            timing: job.timing,
            stamp: job.stamp,
            trace: job.trace,
            left_gray: job.left_gray,
            right_gray: job.right_gray,
            depth: &self.depth,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::buffer::BufferPool;

    fn pixels(bytes: &[u8]) -> PooledBuffer {
        let pool = BufferPool::new(1, bytes.len());
        let mut buffer = pool.take().unwrap();
        buffer.copy_from_slice(bytes);
        buffer
    }

    fn filled_pixels(value: u8, len: usize) -> PooledBuffer {
        let pool = BufferPool::new(1, len);
        let mut buffer = pool.take().unwrap();
        buffer.fill(value);
        buffer
    }

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
    fn replaced_and_closed_jobs_return_pooled_buffers() {
        let pool = BufferPool::new(2, 8);
        let jobs = Latest::default();
        jobs.replace(pool.take().unwrap());
        jobs.replace(pool.take().unwrap());
        assert!(pool.take().is_some());
        jobs.close();
        let first = pool.take().unwrap();
        let second = pool.take().unwrap();
        assert!(pool.take().is_none());
        drop((first, second));
        jobs.replace(pool.take().unwrap());
        assert!(pool.take().is_some());
    }

    #[test]
    fn processor_reuses_depth_storage_and_releases_each_input() {
        let left = BufferPool::new(1, IMAGE_SIZE);
        let right = BufferPool::new(1, IMAGE_SIZE);
        let raw = BufferPool::new(1, 4 * IMAGE_SIZE);
        let mut processor = PixelProcessor::default();
        let mut address = None;
        for seq in 1..=2 {
            let mut bytes = raw.take().unwrap();
            let d = if seq == 1 { SENSOR_NEAR / 2.0 } else { 0.0 };
            for pixel in bytes.as_chunks_mut::<4>().0 {
                pixel.copy_from_slice(&d.to_le_bytes());
            }
            let result = processor.process(
                CaptureJob {
                    seq,
                    timing: CaptureTiming::default(),
                    stamp: seq as f64,
                    trace: TraceContext::default(),
                    left_gray: left.take().unwrap(),
                    right_gray: right.take().unwrap(),
                    depth_raw: bytes,
                },
                &DepthNoiseOptions::default(),
                true,
            );
            if let Some(previous) = address {
                assert_eq!(result.depth.as_ptr(), previous);
            }
            address = Some(result.depth.as_ptr());
            let expected = if seq == 1 { 2.0 } else { 0.0 };
            assert!(result.depth.iter().all(|z| (*z - expected).abs() < 1e-6));
            assert!(raw.take().is_some());
            assert!(left.take().is_none());
            drop(result);
            assert!(left.take().is_some());
            assert!(right.take().is_some());
        }
    }

    #[test]
    fn worker_publishes_without_ui_polling_and_preserves_capture_stamp_and_pixels() {
        let jobs = Latest::default();
        let (published, received) = std::sync::mpsc::channel();
        std::thread::scope(|scope| {
            let worker = scope.spawn(|| {
                run_worker(&jobs, &DepthNoiseOptions::default(), true, |result| {
                    published
                        .send((
                            result.seq,
                            result.stamp,
                            result.left_gray[0],
                            result.right_gray[0],
                        ))
                        .unwrap();
                });
            });
            let mut observations = Vec::new();
            for seq in 1..=2 {
                jobs.replace(CaptureJob {
                    seq,
                    stamp: 7. + seq as f64 / 10.,
                    timing: CaptureTiming::default(),
                    trace: TraceContext::default(),
                    left_gray: filled_pixels(128, IMAGE_SIZE),
                    right_gray: filled_pixels(64, IMAGE_SIZE),
                    depth_raw: filled_pixels(0, 4 * IMAGE_SIZE),
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
            trace: TraceContext::default(),
            left_gray: filled_pixels(128, IMAGE_SIZE),
            right_gray: filled_pixels(128, IMAGE_SIZE),
            depth_raw: pixels(
                &(0..IMAGE_SIZE)
                    .flat_map(|_| (SENSOR_NEAR / 10.).to_le_bytes())
                    .collect::<Vec<_>>(),
            ),
        };
        let mut processor = PixelProcessor::default();
        let ideal = processor.process(job(), &options, true);
        assert!(ideal.depth.iter().all(|z| (*z - 10.).abs() < 1e-5));
        let measured = processor.process(job(), &options, false);
        assert!(measured.depth.iter().all(|z| *z == 0.));
    }

    /// 单拍管线产出各缓冲尺寸正确；全清空深度保持无效 `0.0`。
    #[test]
    fn process_produces_expected_buffers() {
        let options = DepthNoiseOptions::default();
        let job = CaptureJob {
            seq: 3,
            timing: CaptureTiming::default(),
            stamp: 1.5,
            trace: TraceContext::default(),
            left_gray: filled_pixels(128u8, IMAGE_SIZE),
            right_gray: filled_pixels(64u8, IMAGE_SIZE),
            depth_raw: filled_pixels(0u8, 4 * IMAGE_SIZE),
        };
        let mut processor = PixelProcessor::default();
        let out = processor.process(job, &options, false);
        assert_eq!(out.seq, 3);
        assert!((out.stamp - 1.5).abs() < 1e-9);
        assert_eq!(out.left_gray.len(), IMAGE_SIZE);
        assert_eq!(out.right_gray.len(), IMAGE_SIZE);
        assert_eq!(out.depth.len(), IMAGE_SIZE);
        assert!(out.depth.iter().all(|z| *z == 0.0));
    }
}
