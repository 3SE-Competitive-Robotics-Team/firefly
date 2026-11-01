/*
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
//! Translation of `include/basalt/optical_flow/optical_flow.h` at official Basalt
//! revision `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
use super::{
    frame_to_frame_optical_flow::FrameToFrameOpticalFlow,
    multiscale_frame_to_frame_optical_flow::MultiscaleFrameToFrameOpticalFlow,
    patch_optical_flow::PatchOpticalFlow,
    patterns::{Pattern, Pattern24, Pattern50, Pattern51, Pattern52},
};
use crate::io::dataset_io::ImageData;
use crate::utils::vio_config::VioConfig;
use firefly_basalt_headers::calibration::Calibration;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::DMatrix;
use nalgebra::{Matrix2, Vector2};
use std::{collections::BTreeMap, sync::Arc};
use std::{
    sync::mpsc::{self, Receiver, SyncSender},
    thread::JoinHandle,
};

pub type KeypointId = usize;

/// Original `AffineCompact2f` state: preserve the accumulated matrix, without
/// renormalizing it into an SO(2) rotation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AffineCompact2 {
    pub linear: Matrix2<f32>,
    pub translation: Vector2<f32>,
}
impl Default for AffineCompact2 {
    fn default() -> Self {
        Self {
            linear: Matrix2::identity(),
            translation: Vector2::zeros(),
        }
    }
}
#[derive(Clone, Debug, Default)]
pub struct OpticalFlowInput {
    pub t_ns: i64,
    pub img_data: Vec<ImageData>,
}
#[derive(Clone, Debug, Default)]
pub struct OpticalFlowResult {
    pub t_ns: i64,
    pub observations: Vec<BTreeMap<KeypointId, AffineCompact2>>,
    pub pyramid_levels: Vec<BTreeMap<KeypointId, usize>>,
    pub input_images: Option<Arc<OpticalFlowInput>>,
}

trait OpticalFlowEngine: Send {
    fn process(&mut self, input: Arc<OpticalFlowInput>) -> Result<Option<Arc<OpticalFlowResult>>>;
    fn patch_coord(&self) -> DMatrix<f32>;
}

enum OutputQueue {
    Unbounded(mpsc::Sender<Option<Arc<OpticalFlowResult>>>),
    Bounded(SyncSender<Option<Arc<OpticalFlowResult>>>),
}
impl OutputQueue {
    fn send(
        &self,
        value: Option<Arc<OpticalFlowResult>>,
    ) -> std::result::Result<(), mpsc::SendError<Option<Arc<OpticalFlowResult>>>> {
        match self {
            Self::Unbounded(queue) => queue.send(value),
            Self::Bounded(queue) => queue.send(value),
        }
    }
}
struct OutputEndMarker<'a>(&'a OutputQueue);
impl Drop for OutputEndMarker<'_> {
    fn drop(&mut self) {
        let _ = self.0.send(None);
    }
}
macro_rules! engine {
    ($tracker:ident) => {
        impl<P: Pattern<f32, N> + Send + Sync + 'static, const N: usize> OpticalFlowEngine
            for $tracker<P, N>
        {
            fn process(
                &mut self,
                input: Arc<OpticalFlowInput>,
            ) -> Result<Option<Arc<OpticalFlowResult>>> {
                self.process_frame(input)
            }
            fn patch_coord(&self) -> DMatrix<f32> {
                DMatrix::from_column_slice(2, N, P::pattern2().as_slice())
            }
        }
    };
}
engine!(FrameToFrameOpticalFlow);
engine!(PatchOpticalFlow);
engine!(MultiscaleFrameToFrameOpticalFlow);

/// The source's bounded input queue, output queue and owned processing thread.
/// `None` is the original end-of-stream sentinel in both directions.
pub struct OpticalFlowBase {
    input: Option<SyncSender<Option<Arc<OpticalFlowInput>>>>,
    output: Option<Receiver<Option<Arc<OpticalFlowResult>>>>,
    thread: Option<JoinHandle<Result<()>>>,
    pub patch_coord: DMatrix<f32>,
}
impl OpticalFlowBase {
    /// # Panics
    /// The queue is unavailable after shutdown has begun.
    #[must_use]
    pub fn input_queue(&self) -> &SyncSender<Option<Arc<OpticalFlowInput>>> {
        self.input.as_ref().unwrap()
    }
    /// # Panics
    /// The queue is unavailable after shutdown has begun or when output is connected externally.
    #[must_use]
    pub fn output_queue(&self) -> &Receiver<Option<Arc<OpticalFlowResult>>> {
        self.output.as_ref().unwrap()
    }
    /// Stop accepting results, signal termination, and join the worker.
    /// An externally supplied output queue must remain drained or be disconnected
    /// during shutdown, matching the source's blocking producer semantics.
    /// # Errors
    /// Returns the processing error or a worker-panic error.
    pub fn shutdown(mut self) -> Result<()> {
        self.finish()
    }
    fn finish(&mut self) -> Result<()> {
        self.output.take();
        if let Some(input) = self.input.take() {
            let _ = input.send(None);
        }
        if let Some(thread) = self.thread.take() {
            thread
                .join()
                .map_err(|_| Error::new(ErrorKind::Internal, "optical-flow worker panicked"))??;
        }
        Ok(())
    }
}
impl Drop for OpticalFlowBase {
    fn drop(&mut self) {
        let _ = self.finish();
    }
}

/// Complete dispatch table from official `src/optical_flow/optical_flow.cpp`.
pub struct OpticalFlowFactory;
impl OpticalFlowFactory {
    /// # Errors
    /// Rejects unknown type/pattern names, invalid calibration or thread creation failure.
    pub fn get_optical_flow(
        config: &VioConfig,
        calib: &Calibration<f64>,
    ) -> Result<OpticalFlowBase> {
        let engine = Self::create_engine(config, calib)?;
        let (sender, receiver) = mpsc::channel();
        Self::start(engine, OutputQueue::Unbounded(sender), Some(receiver))
    }

    /// Attach the caller's existing backend vision queue directly, preserving its backpressure.
    /// # Errors
    /// Rejects invalid tracker configuration or thread creation failure.
    pub fn get_optical_flow_with_output(
        config: &VioConfig,
        calib: &Calibration<f64>,
        output: SyncSender<Option<Arc<OpticalFlowResult>>>,
    ) -> Result<OpticalFlowBase> {
        Self::start(
            Self::create_engine(config, calib)?,
            OutputQueue::Bounded(output),
            None,
        )
    }

    fn create_engine(
        config: &VioConfig,
        calib: &Calibration<f64>,
    ) -> Result<Box<dyn OpticalFlowEngine>> {
        let engine: Box<dyn OpticalFlowEngine> = match (
            config.optical_flow_type.as_str(),
            config.optical_flow_pattern,
        ) {
            ("frame_to_frame", 24) => Box::new(FrameToFrameOpticalFlow::<Pattern24, 24>::new(
                config.clone(),
                calib,
            )?),
            ("frame_to_frame", 52) => Box::new(FrameToFrameOpticalFlow::<Pattern52, 52>::new(
                config.clone(),
                calib,
            )?),
            ("frame_to_frame", 51) => Box::new(FrameToFrameOpticalFlow::<Pattern51, 52>::new(
                config.clone(),
                calib,
            )?),
            ("frame_to_frame", 50) => Box::new(FrameToFrameOpticalFlow::<Pattern50, 52>::new(
                config.clone(),
                calib,
            )?),
            ("patch", 24) => Box::new(PatchOpticalFlow::<Pattern24, 24>::new(
                config.clone(),
                calib,
            )?),
            ("patch", 52) => Box::new(PatchOpticalFlow::<Pattern52, 52>::new(
                config.clone(),
                calib,
            )?),
            ("patch", 51) => Box::new(PatchOpticalFlow::<Pattern51, 52>::new(
                config.clone(),
                calib,
            )?),
            ("patch", 50) => Box::new(PatchOpticalFlow::<Pattern50, 52>::new(
                config.clone(),
                calib,
            )?),
            ("multiscale_frame_to_frame", 24) => {
                Box::new(MultiscaleFrameToFrameOpticalFlow::<Pattern24, 24>::new(
                    config.clone(),
                    calib,
                )?)
            }
            ("multiscale_frame_to_frame", 52) => {
                Box::new(MultiscaleFrameToFrameOpticalFlow::<Pattern52, 52>::new(
                    config.clone(),
                    calib,
                )?)
            }
            ("multiscale_frame_to_frame", 51) => {
                Box::new(MultiscaleFrameToFrameOpticalFlow::<Pattern51, 52>::new(
                    config.clone(),
                    calib,
                )?)
            }
            ("multiscale_frame_to_frame", 50) => {
                Box::new(MultiscaleFrameToFrameOpticalFlow::<Pattern50, 52>::new(
                    config.clone(),
                    calib,
                )?)
            }
            _ => {
                return Err(Error::new(
                    ErrorKind::InvalidArgument,
                    "unknown optical-flow type or pattern",
                ));
            }
        };
        Ok(engine)
    }

    fn start(
        mut engine: Box<dyn OpticalFlowEngine>,
        output_sender: OutputQueue,
        output: Option<Receiver<Option<Arc<OpticalFlowResult>>>>,
    ) -> Result<OpticalFlowBase> {
        let patch_coord = engine.patch_coord();
        let (input, input_receiver) = mpsc::sync_channel(10);
        let thread = std::thread::Builder::new()
            .name("basalt-optical-flow".into())
            .spawn(move || {
                // Also releases a connected estimator on an engine panic or
                // an input-channel disconnect, not only on explicit EOS/errors.
                let _ending = OutputEndMarker(&output_sender);
                while let Ok(input) = input_receiver.recv() {
                    let Some(input) = input else {
                        break;
                    };
                    match engine.process(input) {
                        Ok(Some(result)) => {
                            if output_sender.send(Some(result)).is_err() {
                                break;
                            }
                        }
                        Ok(None) => {}
                        Err(error) => {
                            return Err(error);
                        }
                    }
                }
                Ok(())
            })
            .map_err(|e| {
                Error::new(
                    ErrorKind::ResourceExhausted,
                    "cannot start optical-flow worker",
                )
                .with_source(e)
            })?;
        Ok(OpticalFlowBase {
            input: Some(input),
            output,
            thread: Some(thread),
            patch_coord,
        })
    }
}

#[cfg(test)]
mod termination_tests {
    use super::*;
    struct PanickingEngine;
    impl OpticalFlowEngine for PanickingEngine {
        fn process(&mut self, _: Arc<OpticalFlowInput>) -> Result<Option<Arc<OpticalFlowResult>>> {
            panic!("injected engine failure")
        }
        fn patch_coord(&self) -> DMatrix<f32> {
            DMatrix::zeros(2, 0)
        }
    }
    #[test]
    fn engine_panic_sends_eos_before_join_reports_failure() {
        let (sender, receiver) = mpsc::sync_channel(1);
        let flow = OpticalFlowFactory::start(
            Box::new(PanickingEngine),
            OutputQueue::Bounded(sender),
            None,
        )
        .unwrap();
        flow.input_queue()
            .send(Some(Arc::new(OpticalFlowInput {
                t_ns: 0,
                img_data: Vec::new(),
            })))
            .unwrap();
        assert!(
            receiver
                .recv_timeout(std::time::Duration::from_secs(3))
                .unwrap()
                .is_none()
        );
        assert!(flow.shutdown().is_err());
    }
}
