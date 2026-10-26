//! A fixed camera worker with bounded request/reply channels and explicit joining.
use super::{Input, Output, Processor, error};
use firefly_error::Result;
use std::{
    sync::mpsc::{self, Receiver, SyncSender},
    thread::{self, JoinHandle},
};

pub(super) struct Worker {
    input: Option<SyncSender<(Input, fastrace::Span)>>,
    output: Receiver<Result<Output>>,
    thread: Option<JoinHandle<()>>,
}

impl Worker {
    pub(super) fn spawn(
        camera: usize,
        initialize: impl FnOnce() -> Result<Processor> + Send + 'static,
    ) -> Result<Self> {
        let (input, requests) = mpsc::sync_channel::<(Input, fastrace::Span)>(1);
        let (replies, output) = mpsc::sync_channel(1);
        let (ready, initialized) = mpsc::sync_channel(1);
        let thread = thread::Builder::new()
            .name(format!("okvis-image-{camera}"))
            .spawn(move || {
                let mut processor = match initialize() {
                    Ok(processor) => processor,
                    Err(error) => {
                        let _ = ready.send(Err(error));
                        return;
                    }
                };
                if ready.send(Ok(())).is_err() {
                    return;
                }
                while let Ok((input, span)) = requests.recv() {
                    let _parent = span.set_local_parent();
                    if replies.send(processor.process(input)).is_err() {
                        break;
                    }
                }
            })
            .map_err(|e| error(format!("camera {camera} worker spawn failed: {e}")))?;
        let mut worker = Self {
            input: Some(input),
            output,
            thread: Some(thread),
        };
        match initialized.recv() {
            Ok(Ok(())) => Ok(worker),
            result => {
                let failure = match result {
                    Ok(Err(e)) => e,
                    _ => error(format!("camera {camera} worker initialization panicked")),
                };
                let _ = worker.shutdown();
                Err(failure)
            }
        }
    }

    pub(super) fn submit(&self, input: Input) -> Result<()> {
        self.input
            .as_ref()
            .ok_or_else(|| error("worker closed"))?
            .send((
                input,
                fastrace::Span::enter_with_local_parent("image-worker"),
            ))
            .map_err(|_| error("image worker stopped before submission"))
    }

    pub(super) fn receive(&self) -> Result<Output> {
        self.output
            .recv()
            .map_err(|_| error("image worker stopped before completion"))?
    }

    pub(super) fn shutdown(&mut self) -> Result<()> {
        self.input.take();
        if let Some(thread) = self.thread.take() {
            thread.join().map_err(|_| error("image worker panicked"))?;
        }
        Ok(())
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        if let Err(error) = self.shutdown() {
            log::error!("{error}");
        }
    }
}
