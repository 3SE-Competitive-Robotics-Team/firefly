// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original `src/mapper_sim_naive.cpp`: replay simulated keyframes in one VIO window.
use crate::{
    mapper_sim::{Archive, invalid, load_calibration, load_marginalization},
    optical_flow::{AffineCompact2, OpticalFlowResult},
    utils::{common_types::TimeCamId, sim_utils::SimObservations, vio_config::VioConfig},
    vi_estimator::vio_estimator::{
        VioEstimatorBase, VioEstimatorFactory, VioVisualizationData, VioWorkerOutputs, align_svd,
    },
};
use firefly_basalt_headers::{
    calibration::Calibration,
    imu::imu_types::{ImuData, PoseVelBiasState},
    spline::se3_spline::Se3Spline,
};
use firefly_error::Result;
use nalgebra::{Isometry3, Vector3};
use std::{
    collections::BTreeMap,
    path::Path,
    sync::{Arc, mpsc},
    thread,
};

pub struct NaiveMapperSimulation {
    pub calibration: Calibration<f64>,
    pub spline: Se3Spline<f64, 5>,
    pub noisy_accel: Vec<Vector3<f64>>,
    pub noisy_gyro: Vec<Vector3<f64>>,
    pub accel: Vec<Vector3<f64>>,
    pub gyro: Vec<Vector3<f64>>,
    pub accel_bias: Vec<Vector3<f64>>,
    pub gyro_bias: Vec<Vector3<f64>>,
    pub points: Vec<Vector3<f64>>,
    pub observations: BTreeMap<TimeCamId, SimObservations>,
    pub noisy_observations: BTreeMap<TimeCamId, SimObservations>,
    pub frame_timestamps: Vec<i64>,
    pub imu_timestamps: Vec<i64>,
    pub keyframe_timestamps: Vec<i64>,
    pub gt_positions: Vec<Vector3<f64>>,
}
pub struct NaiveMapperResult {
    pub visualizations: BTreeMap<i64, Arc<VioVisualizationData>>,
    pub states: Vec<Arc<PoseVelBiasState<f64>>>,
    pub diagnostics: Vec<[f32; 13]>,
}

struct EndInput<T>(mpsc::SyncSender<Option<Arc<T>>>);
impl<T> Drop for EndInput<T> {
    fn drop(&mut self) {
        let _ = self.0.send(None);
    }
}
impl NaiveMapperSimulation {
    /// # Errors
    /// Returns calibration/archive errors or inconsistent input sample arrays.
    pub fn load(calibration_path: &Path, cache_path: &Path) -> Result<Self> {
        let calibration = load_calibration(calibration_path)?;
        let mut archive = Archive::open(&cache_path.join("gt_spline.cereal"))?;
        let spline = archive.spline()?;
        let noisy_accel = archive.vectors("noisy_accel")?;
        let noisy_gyro = archive.vectors("noisy_gyro")?;
        let accel = archive.vectors("noisy_accel")?;
        let gyro = archive.vectors("gt_gyro")?;
        let accel_bias = archive.vectors("gt_accel_bias")?;
        let gyro_bias = archive.vectors("gt_gyro_bias")?;
        let _first_points = archive.vectors("gt_points")?;
        let observations = archive.observations("gt_observations")?;
        let noisy_observations = archive.observations("noisy_observations")?;
        let points = archive.vectors("gt_points")?;
        let frame_timestamps: Vec<i64> = archive.read("gt_frame_t_ns")?;
        let imu_timestamps: Vec<i64> = archive.read("gt_imu_t_ns")?;
        let gt_positions = frame_timestamps
            .iter()
            .map(|&t| spline.pose(t, None).translation.vector)
            .collect();
        let mut poses = BTreeMap::<i64, Isometry3<f64>>::new();
        for data in load_marginalization(cache_path)? {
            for (&time, state) in &data.frame_poses {
                poses.insert(time, *state.pose());
            }
            for (&time, state) in &data.frame_states {
                if data.kfs_all.contains(&time) {
                    poses.insert(time, state.state().state.pose.t_w_i);
                }
            }
        }
        if noisy_accel.len() != imu_timestamps.len()
            || noisy_gyro.len() != imu_timestamps.len()
            || accel_bias.is_empty()
            || gyro_bias.is_empty()
            || poses.is_empty()
        {
            return Err(invalid(
                "Simulation needs paired IMU samples, biases and keyframes",
            ));
        }
        Ok(Self {
            calibration,
            spline,
            noisy_accel,
            noisy_gyro,
            accel,
            gyro,
            accel_bias,
            gyro_bias,
            points,
            observations,
            noisy_observations,
            frame_timestamps,
            imu_timestamps,
            keyframe_timestamps: poses.into_keys().collect(),
            gt_positions,
        })
    }
    /// Configures the original double-precision IMU estimator with a 10000-state window.
    /// # Errors
    /// Returns estimator construction/initialization errors or empty keyframes.
    pub fn setup_vio(&self, outputs: VioWorkerOutputs) -> Result<VioEstimatorBase> {
        let time = *self
            .keyframe_timestamps
            .first()
            .ok_or_else(|| invalid("No keyframes"))?;
        let config = VioConfig {
            vio_debug: true,
            ..VioConfig::default()
        };
        let mut vio = VioEstimatorFactory::get_vio_estimator(
            &config,
            &self.calibration,
            &Vector3::new(0., 0., -9.81),
            true,
            true,
        )?;
        vio.outputs = outputs;
        vio.initialize_state(
            time,
            self.spline.pose(time, None),
            self.spline.trans_vel_world(time),
            self.gyro_bias[0],
            self.accel_bias[0],
        )?;
        vio.set_max_states(10000);
        vio.set_max_kfs(10000);
        Ok(vio)
    }
    /// # Errors
    /// Returns absent or malformed keyframe observations.
    pub fn vision_input(&self, time: i64) -> Result<OpticalFlowResult> {
        let mut observations = Vec::new();
        for camera in 0..self.calibration.t_i_c.len() {
            let input = self
                .noisy_observations
                .get(&TimeCamId::new(time, camera))
                .ok_or_else(|| invalid("Missing simulated keyframe observations"))?;
            if input.id.len() != input.pos.len() {
                return Err(invalid("Observation IDs and pixels differ in length"));
            }
            let camera = input
                .id
                .iter()
                .zip(&input.pos)
                .map(|(&id, p)| {
                    Ok((
                        usize::try_from(id).map_err(|_| invalid("Negative feature ID"))?,
                        AffineCompact2 {
                            translation: p.cast(),
                            ..AffineCompact2::default()
                        },
                    ))
                })
                .collect::<Result<_>>()?;
            observations.push(camera);
        }
        Ok(OpticalFlowResult {
            t_ns: time,
            observations,
            ..OpticalFlowResult::default()
        })
    }
    /// Feeds both bounded inputs concurrently and drains the original unbounded outputs.
    /// # Errors
    /// Propagates estimator, worker and observation errors.
    pub fn run(&self) -> Result<NaiveMapperResult> {
        let (state_tx, state_rx) = mpsc::channel();
        let (vis_tx, vis_rx) = mpsc::channel();
        let mut vio = self.setup_vio(VioWorkerOutputs {
            state: Some(state_tx),
            vis: Some(vis_tx),
            ..VioWorkerOutputs::default()
        })?;
        let imu = vio.imu_data_queue().clone();
        let vision = vio.vision_data_queue().clone();
        let processing = thread::scope(|scope| {
            let producer = scope.spawn(move || {
                let _end = EndInput(imu.clone());
                for (i, &time) in self.imu_timestamps.iter().enumerate() {
                    if imu
                        .send(Some(Arc::new(ImuData {
                            t_ns: time,
                            accel: self.noisy_accel[i],
                            gyro: self.noisy_gyro[i],
                        })))
                        .is_err()
                    {
                        return;
                    }
                }
            });
            let frames = scope.spawn(move || -> Result<()> {
                let _end = EndInput(vision.clone());
                for &time in &self.keyframe_timestamps {
                    vision
                        .send(Some(Arc::new(self.vision_input(time)?)))
                        .map_err(|_| invalid("Estimator vision input disconnected"))?;
                }
                Ok(())
            });
            let result = vio.maybe_join();
            producer
                .join()
                .map_err(|_| invalid("IMU producer panicked"))?;
            let images = frames
                .join()
                .map_err(|_| invalid("Vision producer panicked"))?;
            result?;
            images
        });
        processing?;
        let mut states = Vec::new();
        while let Ok(Some(state)) = state_rx.recv() {
            states.push(state);
        }
        let mut visualizations = BTreeMap::new();
        while let Ok(Some(frame)) = vis_rx.recv() {
            visualizations.insert(frame.t_ns, frame);
        }
        let diagnostics = states
            .iter()
            .map(|state| {
                let p = state.state.pose.t_w_i.translation.vector;
                let v = state.state.vel_w_i;
                let bg = state.bias_gyro;
                let ba = state.bias_accel;
                [
                    state.state.pose.t_ns as f64 * 1e-9,
                    v.x,
                    v.y,
                    v.z,
                    p.x,
                    p.y,
                    p.z,
                    bg.x,
                    bg.y,
                    bg.z,
                    ba.x,
                    ba.y,
                    ba.z,
                ]
                .map(|v| v as f32)
            })
            .collect();
        Ok(NaiveMapperResult {
            visualizations,
            states,
            diagnostics,
        })
    }
    /// Uses the last visualization's complete state window, as in the original CLI.
    /// # Errors
    /// Rejects missing final output or a window that dropped keyframes.
    pub fn align(&mut self, result: &NaiveMapperResult) -> Result<f64> {
        let time = self
            .keyframe_timestamps
            .last()
            .ok_or_else(|| invalid("No keyframes"))?;
        let frame = result
            .visualizations
            .get(time)
            .ok_or_else(|| invalid("Missing final visualization"))?;
        if frame.states.len() != self.keyframe_timestamps.len() {
            return Err(invalid(
                "Final window does not contain every simulated keyframe",
            ));
        }
        let positions: Vec<_> = frame.states.iter().map(|p| p.translation.vector).collect();
        Ok(align_svd(
            &self.keyframe_timestamps,
            &positions,
            &self.frame_timestamps,
            &mut self.gt_positions,
        ))
    }
    /// Original GUI stepping state, independent of rendering.
    #[must_use]
    pub fn next_step(&self, frame: &mut usize) -> bool {
        if *frame + 1 < self.keyframe_timestamps.len() {
            *frame += 1;
            true
        } else {
            false
        }
    }
}
