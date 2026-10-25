//! 独立接收地图系状态；重规划耗时不能改变状态流的存活判定。
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::state_input::StateInput;
use firefly_error::{Error, ErrorKind, Result};
use firefly_pubsub::{
    navigation::{NAVIGATION_TOPIC, NavigationMessage},
    node::create_node,
    subscriber::Subscriber,
    trace::TraceContext,
};

#[derive(Default)]
struct Shared {
    input: StateInput,
    trace: TraceContext,
    error: Option<String>,
}

/// 输出线程只读输入，不消费规划线程的修正版本通知。
#[derive(Clone)]
pub struct NavigationReader {
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
}
impl NavigationReader {
    pub fn snapshot(&self) -> Result<StateInput> {
        let state = self
            .shared
            .lock()
            .map_err(|_| internal("navigation state poisoned"))?;
        if let Some(error) = &state.error {
            return Err(internal(error.clone()));
        }
        let mut input = state.input.clone();
        if self.stop.load(Ordering::Relaxed) {
            input.stop();
        }
        Ok(input)
    }
}

pub struct NavigationReceiver {
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl NavigationReceiver {
    pub fn reader(&self) -> NavigationReader {
        NavigationReader {
            shared: Arc::clone(&self.shared),
            stop: Arc::clone(&self.stop),
        }
    }

    pub fn start(external_map: bool) -> Result<Self> {
        let shared = Arc::new(Mutex::new(Shared::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let (state, stopping) = (Arc::clone(&shared), Arc::clone(&stop));
        let (ready, startup) = std::sync::mpsc::sync_channel(1);
        let thread = std::thread::Builder::new()
            .name("planner-navigation".into())
            .spawn(move || {
                let result = (|| -> Result<()> {
                    let node = create_node()?;
                    let subscriber =
                        Subscriber::<NavigationMessage>::with_topic(&node, NAVIGATION_TOPIC)?;
                    let _ = ready.send(Ok(()));
                    while !stopping.load(Ordering::Relaxed)
                        && node.wait(Duration::from_millis(2)).is_ok()
                    {
                        while let Some(message) = subscriber.receive()? {
                            if external_map && message.external_map != 1 {
                                return Err(Error::new(
                                    ErrorKind::InvalidArgument,
                                    "static map requires explicit external map alignment",
                                ));
                            }
                            let trace = *message.user_header();
                            let _span = trace.continue_span("recv-map-odom");
                            let mut state = state
                                .lock()
                                .map_err(|_| internal("navigation state poisoned"))?;
                            if state.input.observe_navigation(&message, Instant::now()) {
                                state.trace = trace;
                            }
                        }
                        state
                            .lock()
                            .map_err(|_| internal("navigation state poisoned"))?
                            .input
                            .current(Instant::now());
                    }
                    Ok(())
                })();
                stopping.store(true, Ordering::Relaxed);
                if let Ok(mut state) = state.lock() {
                    state.input.stop();
                    if let Err(error) = result {
                        state.error = Some(error.to_string());
                        let _ = ready.send(Err(error.to_string()));
                        log::error!("navigation receiver: {error}");
                    }
                }
            })
            .map_err(|e| internal(e.to_string()))?;
        let mut receiver = Self {
            shared,
            stop,
            thread: Some(thread),
        };
        if let Err(error) = startup.recv().map_err(|e| e.to_string()).and_then(|v| v) {
            receiver.shutdown();
            return Err(internal(error).with_operation("start navigation receiver"));
        }
        Ok(receiver)
    }

    /// 原子读取状态、坐标版本和 trace；版本变更只在交给规划线程后确认。
    pub fn snapshot(&self) -> Result<(StateInput, TraceContext)> {
        let mut state = self
            .shared
            .lock()
            .map_err(|_| internal("navigation state poisoned"))?;
        if let Some(error) = &state.error {
            return Err(internal(error.clone()));
        }
        if self.stop.load(Ordering::Relaxed) {
            state.input.stop();
        }
        state.input.current(Instant::now());
        let snapshot = state.input.clone();
        state.input.take_revision_change();
        Ok((snapshot, state.trace))
    }

    pub fn terminated(&self) -> bool {
        self.stop.load(Ordering::Relaxed)
    }

    fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            log::error!("navigation receiver panicked");
        }
    }
}

impl Drop for NavigationReceiver {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn internal(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::Internal, message.into())
}
