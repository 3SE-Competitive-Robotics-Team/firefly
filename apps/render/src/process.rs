//! IPC 发布端口在像素工作线程内创建和使用。
//! 离线数据集使用几何深度标签。

use crate::buffer::PooledBuffer;
use crate::capture::CaptureTiming;
use std::sync::{Condvar, Mutex};
use std::time::Instant;

use firefly_pubsub::camera::IMAGE_SIZE;
use firefly_pubsub::trace::TraceContext;

use crate::rig::SENSOR_NEAR;
use crate::sensors;

#[derive(Debug)]
pub struct CaptureJob {
    pub seq: u64,
    pub timing: CaptureTiming,
    /// 位姿时间戳（秒，sim 时间）。
    pub stamp: f64,
    /// 上游 trace 上下文（发布时续接）。
    pub trace: TraceContext,
    /// 左目紧凑灰度（行主序无填充）。
    pub left_gray: PooledBuffer,
    pub right_gray: PooledBuffer,
    /// 深度预通道原始字节（小端 `f32`）。
    pub depth_raw: Option<PooledBuffer>,
    pub down_gray: Option<PooledBuffer>,
}

#[derive(Debug)]
pub struct ProcessedCapture<'a> {
    pub seq: u64,
    pub timing: CaptureTiming,
    /// 位姿时间戳（秒）。
    pub stamp: f64,
    pub trace: TraceContext,
    pub left_gray: PooledBuffer,
    pub right_gray: PooledBuffer,
    /// 深度（米，行主序；离线几何标签）。
    pub depth: &'a [f32],
    pub down_gray: Option<PooledBuffer>,
}

/// 有界最新值邮箱：生产者不等待消费者；关闭唤醒接收端并丢弃未处理任务。
pub struct Latest<T> {
    state: Mutex<Mailbox<T>>,
    ready: Condvar,
}

enum Mailbox<T> {
    Open(Option<T>),
    Closed,
}

impl<T> Default for Latest<T> {
    fn default() -> Self {
        Self {
            state: Mutex::new(Mailbox::Open(None)),
            ready: Condvar::new(),
        }
    }
}

impl<T> Latest<T> {
    pub fn replace(&self, value: T) {
        let mut state = self.state.lock().expect("capture mailbox poisoned");
        if let Mailbox::Open(pending) = &mut *state {
            *pending = Some(value);
            self.ready.notify_one();
        }
    }

    #[cfg(test)]
    pub fn take(&self) -> Option<T> {
        match &mut *self.state.lock().expect("capture mailbox poisoned") {
            Mailbox::Open(pending) => pending.take(),
            Mailbox::Closed => None,
        }
    }

    pub fn recv(&self) -> Option<T> {
        let state = self.state.lock().expect("capture mailbox poisoned");
        let mut state = self
            .ready
            .wait_while(state, |state| matches!(state, Mailbox::Open(None)))
            .expect("capture mailbox poisoned");
        match &mut *state {
            Mailbox::Open(pending) => pending.take(),
            Mailbox::Closed => None,
        }
    }

    pub fn close(&self) {
        *self.state.lock().expect("capture mailbox poisoned") = Mailbox::Closed;
        self.ready.notify_all();
    }
}

pub fn run_worker(jobs: &Latest<CaptureJob>, mut publish: impl FnMut(&ProcessedCapture)) {
    let mut processor = PixelProcessor::default();
    while let Some(job) = jobs.recv() {
        let root = job.trace.continue_span("render-pixels").unwrap_or_else(|| {
            fastrace::Span::root(
                "render-pixels",
                fastrace::prelude::SpanContext::random().sampled(false),
            )
        });
        let guard = root.set_local_parent();
        let result = processor.process(job);
        publish(&result);
        drop(guard);
        fastrace::flush();
    }
}

pub struct PixelProcessor {
    depth: Vec<f32>,
}

impl Default for PixelProcessor {
    fn default() -> Self {
        Self {
            depth: vec![0.; IMAGE_SIZE],
        }
    }
}

impl PixelProcessor {
    /// 单拍像素管线；结果借用工作缓冲，必须在下一拍前完成发布。
    pub fn process(&mut self, mut job: CaptureJob) -> ProcessedCapture<'_> {
        job.timing.started = Instant::now();
        if let Some(raw) = &job.depth_raw {
            sensors::linearize_depth(raw, &mut self.depth, SENSOR_NEAR);
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
            down_gray: job.down_gray,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::buffer::BufferPool;

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
    fn closing_releases_all_waiters_and_stays_closed() {
        let jobs = Latest::<u32>::default();
        let started = std::sync::Barrier::new(5);
        std::thread::scope(|scope| {
            let waiters: Vec<_> = (0..4)
                .map(|_| {
                    scope.spawn(|| {
                        started.wait();
                        jobs.recv()
                    })
                })
                .collect();
            started.wait();
            jobs.close();
            jobs.close();
            jobs.replace(1);
            for waiter in waiters {
                assert_eq!(waiter.join().unwrap(), None);
            }
        });
        assert_eq!(jobs.recv(), None);
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
            let result = processor.process(CaptureJob {
                down_gray: None,
                seq,
                timing: CaptureTiming::default(),
                stamp: seq as f64,
                trace: TraceContext::default(),
                left_gray: left.take().unwrap(),
                right_gray: right.take().unwrap(),
                depth_raw: Some(bytes),
            });
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
                run_worker(&jobs, |result| {
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
                    down_gray: None,
                    seq,
                    stamp: 7. + seq as f64 / 10.,
                    timing: CaptureTiming::default(),
                    trace: TraceContext::default(),
                    left_gray: filled_pixels(128, IMAGE_SIZE),
                    right_gray: filled_pixels(64, IMAGE_SIZE),
                    depth_raw: None,
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
    fn process_produces_expected_buffers() {
        let job = CaptureJob {
            down_gray: None,
            seq: 3,
            timing: CaptureTiming::default(),
            stamp: 1.5,
            trace: TraceContext::default(),
            left_gray: filled_pixels(128u8, IMAGE_SIZE),
            right_gray: filled_pixels(64u8, IMAGE_SIZE),
            depth_raw: None,
        };
        let mut processor = PixelProcessor::default();
        let out = processor.process(job);
        assert_eq!(out.seq, 3);
        assert!((out.stamp - 1.5).abs() < 1e-9);
        assert_eq!(out.left_gray.len(), IMAGE_SIZE);
        assert_eq!(out.right_gray.len(), IMAGE_SIZE);
        assert_eq!(out.depth.len(), IMAGE_SIZE);
        assert!(out.depth.iter().all(|z| *z == 0.0));
    }
}
