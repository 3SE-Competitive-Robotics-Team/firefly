//! 每个进程独立订阅里程计，按本地接收时间维护有效期与测量历史。
use std::collections::VecDeque;
use std::mem::DropGuard;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::state_input::StateInput;
use firefly_error::{Error, ErrorKind, Result};
use firefly_pubsub::{
    node::create_node,
    odom::{ODOM_TOPIC, OdomMessage},
    subscriber::Subscriber,
    trace::TraceContext,
};

#[derive(Default)]
struct Shared {
    input: StateInput,
    publisher: Option<u128>,
    history: VecDeque<OdomMessage>,
    trace: TraceContext,
    error: Option<Error>,
}

impl Shared {
    fn observe(
        &mut self,
        message: &OdomMessage,
        publisher: u128,
        trace: TraceContext,
        now: Instant,
    ) {
        // A replacement odometry writer can start at another world origin with a later clock.
        if self.publisher.is_some_and(|id| id != publisher) {
            self.history.clear();
            self.input.stop();
            return;
        }
        if self.input.observe(message, now) {
            self.publisher = Some(publisher);
            self.history.push_back(*message);
            while self.history.len() > 512
                || self
                    .history
                    .front()
                    .is_some_and(|old| message.timestamp - old.timestamp > 2.)
            {
                self.history.pop_front();
            }
            self.trace = trace;
        }
    }
}

/// 同一进程内按测量时间排列的有界里程计历史。
#[derive(Clone)]
pub struct OdometryHistory {
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
}

impl OdometryHistory {
    /// 在锁内查询有效历史。停止、过期或历史为空时返回 `None`。
    ///
    /// # Errors
    /// 接收线程失败或共享状态锁中毒。
    pub fn read<T>(&self, query: impl FnOnce(&VecDeque<OdomMessage>) -> T) -> Result<Option<T>> {
        let mut state = self
            .shared
            .lock()
            .map_err(|_| internal("odometry state poisoned"))?;
        if let Some(error) = &state.error {
            return Err(error.clone());
        }
        if self.stop.load(Ordering::Relaxed) || state.input.current(Instant::now()).is_none() {
            return Ok(None);
        }
        Ok((!state.history.is_empty()).then(|| query(&state.history)))
    }
}

pub struct OdometryReceiver {
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl OdometryReceiver {
    /// 启动独立里程计线程，并等待 IPC 订阅就绪。
    ///
    /// # Errors
    /// 线程创建、IPC 订阅或启动握手失败。
    pub fn start() -> Result<Self> {
        let shared = Arc::new(Mutex::new(Shared::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let (state, stopping) = (Arc::clone(&shared), Arc::clone(&stop));
        let (ready, startup) = std::sync::mpsc::sync_channel(1);
        let thread = std::thread::Builder::new()
            .name("odometry".into())
            .spawn(move || {
                let _termination = DropGuard::new(Arc::clone(&stopping), |stop| {
                    stop.store(true, Ordering::Relaxed);
                });
                let result = (|| -> Result<()> {
                    let node = create_node()?;
                    let subscriber = Subscriber::<OdomMessage>::with_topic(&node, ODOM_TOPIC)?;
                    let _ = ready.send(Ok(()));
                    while !stopping.load(Ordering::Relaxed)
                        && node.wait(Duration::from_millis(2)).is_ok()
                    {
                        while let Some(message) = subscriber.receive()? {
                            let trace = *message.user_header();
                            let _span = trace.continue_span("recv-odom");
                            let mut state = state
                                .lock()
                                .map_err(|_| internal("odometry state poisoned"))?;
                            state.observe(
                                &message,
                                message.header().publisher_id().value(),
                                trace,
                                Instant::now(),
                            );
                        }
                        state
                            .lock()
                            .map_err(|_| internal("odometry state poisoned"))?
                            .input
                            .current(Instant::now());
                    }
                    Ok(())
                })();
                stopping.store(true, Ordering::Relaxed);
                if let Ok(mut state) = state.lock() {
                    state.input.stop();
                    if let Err(error) = result {
                        state.error = Some(error.clone());
                        log::error!("odometry receiver: {error}");
                        let _ = ready.send(Err(error));
                    }
                }
            })
            .map_err(|e| Error::from(e).with_operation("spawn odometry receiver"))?;
        let receiver = Self {
            shared,
            stop,
            thread: Some(thread),
        };
        startup.recv().map_err(|e| {
            Error::caused_by(ErrorKind::Unavailable, "start odometry receiver", e)
        })??;
        Ok(receiver)
    }

    /// 原子读取局部状态和 trace。
    ///
    /// # Errors
    /// 接收线程失败或共享状态锁中毒。
    pub fn snapshot(&self) -> Result<(StateInput, TraceContext)> {
        let mut state = self
            .shared
            .lock()
            .map_err(|_| internal("odometry state poisoned"))?;
        if let Some(error) = &state.error {
            return Err(error.clone());
        }
        if self.terminated() {
            state.input.stop();
        }
        state.input.current(Instant::now());
        let snapshot = state.input.clone();
        Ok((snapshot, state.trace))
    }

    pub fn terminated(&self) -> bool {
        self.stop.load(Ordering::Relaxed)
            || self.thread.as_ref().is_some_and(JoinHandle::is_finished)
    }

    #[must_use]
    pub fn history(&self) -> OdometryHistory {
        OdometryHistory {
            shared: Arc::clone(&self.shared),
            stop: Arc::clone(&self.stop),
        }
    }

    fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            log::error!("odometry receiver panicked");
        }
    }
}

impl Drop for OdometryReceiver {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn internal(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::Internal, message.into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn changing_writer_latches_stop_even_when_measurement_time_advances() {
        let mut state = Shared::default();
        let mut message = OdomMessage {
            timestamp: 1.,
            is_initialized: true,
            ..Default::default()
        };
        let now = Instant::now();
        state.observe(&message, 10, TraceContext::default(), now);
        assert_eq!(state.history.len(), 1);
        message.timestamp = 2.;
        state.observe(&message, 11, TraceContext::default(), now);
        assert!(state.input.stopped());
        assert!(state.history.is_empty());
        state.observe(&message, 10, TraceContext::default(), now);
        assert!(state.input.current(now).is_none());
    }

    #[test]
    fn expired_input_retires_buffered_history() {
        let mut state = Shared::default();
        let message = OdomMessage {
            timestamp: 42.,
            is_initialized: true,
            ..Default::default()
        };
        let received = Instant::now().checked_sub(Duration::from_secs(1)).unwrap();
        assert!(state.input.observe(&message, received));
        state.history.push_back(message);
        let history = OdometryHistory {
            shared: Arc::new(Mutex::new(state)),
            stop: Arc::new(AtomicBool::new(false)),
        };
        assert!(history.read(VecDeque::len).unwrap().is_none());
    }

    #[test]
    fn panicking_receiver_retires_shared_history() {
        let stop = Arc::new(AtomicBool::new(false));
        let shared = Arc::new(Mutex::new(Shared::default()));
        let message = OdomMessage {
            timestamp: 1.,
            is_initialized: true,
            ..Default::default()
        };
        {
            let mut state = shared.lock().unwrap();
            assert!(state.input.observe(&message, Instant::now()));
            state.history.push_back(message);
        }
        let history = OdometryHistory {
            shared,
            stop: Arc::clone(&stop),
        };
        assert_eq!(history.read(VecDeque::len).unwrap(), Some(1));
        let handle = std::thread::spawn(move || {
            let _termination = DropGuard::new(stop, |stop| {
                stop.store(true, Ordering::Relaxed);
            });
            panic!("receiver failure outside the state mutex");
        });
        assert!(handle.join().is_err());
        assert!(history.read(VecDeque::len).unwrap().is_none());
    }
}
