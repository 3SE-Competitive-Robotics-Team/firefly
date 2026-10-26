//! One synchronous batch in flight: submit all cameras, drain all replies, return in camera order.
use super::brisk::{BriskFeatureFactory, BriskSettings};
use super::{Input, Output, Processor, error, worker::Worker};
use firefly_error::Result;
use std::sync::Arc;

/// Workers start lazily for cameras with images and retain their extractors until shutdown.
/// An execution failure closes the pipeline. No partial result is returned for installation.
pub(crate) struct Pipeline {
    workers: Vec<Option<Worker>>,
    closed: bool,
}

impl Pipeline {
    pub fn new(cameras: usize) -> Self {
        Self {
            workers: (0..cameras).map(|_| None).collect(),
            closed: false,
        }
    }

    /// Factory/settings are sampled at each camera's first submission. Calibration must retain
    /// the same Arc identity; changing the rig or detector settings requires a new pipeline.
    /// Preflight rejects duplicate/out-of-range camera IDs without submitting any work.
    /// The exclusive borrow and full reply drain prevent results crossing batch boundaries.
    #[fastrace::trace]
    pub fn process<B: BriskFeatureFactory>(
        &mut self,
        mut inputs: Vec<Input>,
        factory: &Arc<B>,
        settings: &BriskSettings,
    ) -> Result<Vec<Output>> {
        if self.closed {
            return Err(error("image pipeline closed"));
        }
        inputs.sort_by_key(|input| input.camera);
        if inputs
            .iter()
            .any(|input| input.camera >= self.workers.len())
            || inputs
                .windows(2)
                .any(|pair| pair[0].camera == pair[1].camera)
        {
            return Err(error("invalid or duplicate camera in image batch"));
        }
        let mut submitted = Vec::with_capacity(inputs.len());
        let mut failure = None;
        for input in inputs {
            let camera = input.camera;
            if self.workers[camera].is_none() {
                let factory = factory.clone();
                let geometry = input.geometry.clone();
                let settings = settings.clone();
                let direction = input.direction;
                match Worker::spawn(camera, move || {
                    Processor::new(factory.as_ref(), geometry, &settings, direction)
                }) {
                    Ok(worker) => self.workers[camera] = Some(worker),
                    Err(error) => {
                        failure = Some(error.with_context("camera", camera));
                        break;
                    }
                }
            }
            let worker = self.workers[camera].as_ref().expect("initialized worker");
            match worker.submit(input) {
                Ok(()) => submitted.push(camera),
                Err(error) => {
                    failure = Some(error.with_context("camera", camera));
                    break;
                }
            }
        }
        let mut outputs = Vec::with_capacity(submitted.len());
        for camera in submitted {
            match self.workers[camera]
                .as_ref()
                .expect("submitted worker")
                .receive()
            {
                Ok(output) => outputs.push(output),
                Err(error) => {
                    failure.get_or_insert(error.with_context("camera", camera));
                }
            }
        }
        if let Some(error) = failure {
            if let Err(shutdown) = self.shutdown() {
                log::error!("{shutdown}");
            }
            return Err(error);
        }
        Ok(outputs)
    }

    pub fn shutdown(&mut self) -> Result<()> {
        self.closed = true;
        let mut failure = None;
        for worker in self.workers.iter_mut().filter_map(Option::take) {
            let mut worker = worker;
            if let Err(error) = worker.shutdown() {
                failure.get_or_insert(error);
            }
        }
        failure.map_or(Ok(()), Err)
    }
}
