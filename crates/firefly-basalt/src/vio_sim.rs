// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
//! Original `src/vio_sim.cpp` simulation, with drawing and queue consumers owned
//! by the caller. Random streams reproduce the Linux/glibc/libstdc++ reference.
use crate::{
    optical_flow::{AffineCompact2, OpticalFlowResult},
    utils::{common_types::TimeCamId, sim_utils::SimObservations, vio_config::VioConfig},
    vi_estimator::vio_estimator::{VioEstimatorBase, VioEstimatorFactory, VioWorkerOutputs},
};
use firefly_basalt_headers::{
    calibration::Calibration,
    image::image::ManagedImage,
    imu::imu_types::ImuData,
    spline::se3_spline::Se3Spline,
    utils::sophus_utils::{se3_inverse, se3_product, so3_inverse},
};
use nalgebra::{Isometry3, Vector2, Vector3, Vector4};
use serde::{Serialize, ser::SerializeMap};
use std::{
    collections::BTreeMap,
    io::{self, Write},
    sync::Arc,
};

pub const NUM_FRAMES: usize = 1000;
pub const CAM_FREQ: i64 = 20;
pub const IMU_FREQ: i64 = 200;
pub const KNOT_TIME: i64 = 3;
pub const OBS_STD_DEV: f64 = 0.5;
pub const NUM_POINTS: usize = 1000;
pub const POINT_DIST: f64 = 10.;

// MT19937 recurrence specified by the C++ standard, independent of rand crate
// engines. The f64 adaptor uses two successive 32-bit engine draws.
struct Mt19937 {
    state: [u32; 624],
    index: usize,
}
impl Mt19937 {
    fn new(seed: u32) -> Self {
        let mut state = [0; 624];
        state[0] = seed;
        for i in 1..624 {
            state[i] = 1_812_433_253_u32
                .wrapping_mul(state[i - 1] ^ (state[i - 1] >> 30))
                .wrapping_add(i as u32);
        }
        Self { state, index: 624 }
    }
    fn next(&mut self) -> u32 {
        if self.index == 624 {
            for i in 0..624 {
                let combined =
                    (self.state[i] & 0x8000_0000) | (self.state[(i + 1) % 624] & 0x7fff_ffff);
                self.state[i] = self.state[(i + 397) % 624]
                    ^ (combined >> 1)
                    ^ if combined & 1 != 0 { 0x9908_b0df } else { 0 };
            }
            self.index = 0;
        }
        let mut value = self.state[self.index];
        self.index += 1;
        value ^= value >> 11;
        value ^= (value << 7) & 0x9d2c_5680;
        value ^= (value << 15) & 0xefc6_0000;
        value ^= value >> 18;
        value
    }
    fn unit(&mut self) -> f64 {
        let low = f64::from(self.next());
        let high = f64::from(self.next());
        let value = (low + high * 4_294_967_296.) / 18_446_744_073_709_551_616.;
        if value >= 1. {
            f64::from_bits(1_f64.to_bits() - 1)
        } else {
            value
        }
    }
}
#[derive(Default)]
struct Gaussian {
    saved: Option<f64>,
}
impl Gaussian {
    fn sample(&mut self, rng: &mut Mt19937, stddev: f64) -> f64 {
        if let Some(saved) = self.saved.take() {
            return saved * stddev + 0.;
        }
        loop {
            let first = 2. * rng.unit() - 1.;
            let second = 2. * rng.unit() - 1.;
            let squared = first * first + second * second;
            if squared > 1. || squared == 0. {
                continue;
            }
            let scale = (-2. * squared.ln() / squared).sqrt();
            self.saved = Some(first * scale);
            return second * scale * stddev + 0.;
        }
    }
}
// glibc's default rand stream: Park-Miller seeding and a 31-word additive
// recurrence with lag 3. Eigen::Random() consumes this separate stream.
pub(crate) struct EigenRandom {
    state: [u32; 31],
    index: usize,
}
impl EigenRandom {
    pub(crate) fn new() -> Self {
        let mut history = [0_u32; 344];
        history[0] = 1;
        for i in 1..31 {
            history[i] = ((16_807_u64 * u64::from(history[i - 1])) % 2_147_483_647) as u32;
        }
        for i in 31..34 {
            history[i] = history[i - 31];
        }
        for i in 34..344 {
            history[i] = history[i - 31].wrapping_add(history[i - 3]);
        }
        let mut state = [0; 31];
        state.copy_from_slice(&history[313..]);
        Self { state, index: 0 }
    }
    fn next(&mut self) -> u32 {
        let value = self.state[self.index].wrapping_add(self.state[(self.index + 28) % 31]);
        self.state[self.index] = value;
        self.index = (self.index + 1) % 31;
        value >> 1
    }
    pub(crate) fn sample(&mut self) -> f64 {
        // Copyright (C) 2024 Charles Schlosser <cs.schlosser@gmail.com>
        // Eigen 5 RandomImpl fills the 52 mantissa bits using two 31-bit
        // rand() draws, then maps [2,4) to [-1,1). MPL-2.0, see crate license.
        let low = u64::from(self.next());
        let high = u64::from(self.next());
        let bits = (low | (high << 31)) & 0x000f_ffff_ffff_ffff;
        f64::from_bits(2_f64.to_bits() | bits) - 3.
    }
    fn vector(&mut self) -> Vector3<f64> {
        Vector3::new(self.sample(), self.sample(), self.sample())
    }
}

pub struct Simulation {
    pub calibration: Calibration<f64>,
    pub spline: Se3Spline<f64, 5>,
    pub points: Vec<Vector3<f64>>,
    pub frame_poses: Vec<Isometry3<f64>>,
    pub frame_timestamps: Vec<i64>,
    pub imu_timestamps: Vec<i64>,
    pub accel: Vec<Vector3<f64>>,
    pub gyro: Vec<Vector3<f64>>,
    pub velocity: Vec<Vector3<f64>>,
    pub accel_bias: Vec<Vector3<f64>>,
    pub gyro_bias: Vec<Vector3<f64>>,
    pub noisy_accel: Vec<Vector3<f64>>,
    pub noisy_gyro: Vec<Vector3<f64>>,
    pub observations: BTreeMap<TimeCamId, SimObservations>,
    pub noisy_observations: BTreeMap<TimeCamId, SimObservations>,
    pub images: Vec<ManagedImage<u8>>,
    /// Original time, accel, gyro, velocity, position, gyro bias, accel bias columns.
    pub imu_diagnostics: Vec<[f32; 19]>,
}
impl Simulation {
    /// Generate the original 1000-frame synthetic sequence using its two random streams.
    /// # Panics
    /// Requires matching camera/calibration dimensions and nonnegative image sizes.
    #[must_use]
    #[expect(
        clippy::too_many_lines,
        reason = "Preserves original generation and random-stream consumption order"
    )]
    pub fn generate(calibration: Calibration<f64>, num_points: usize, point_distance: f64) -> Self {
        assert_eq!(calibration.intrinsics.len(), calibration.t_i_c.len());
        assert_eq!(calibration.intrinsics.len(), calibration.resolution.len());
        let mut uniform = EigenRandom::new();
        let mut rng = Mt19937::new(1);
        let mut spline = Se3Spline::<f64, 5>::new(KNOT_TIME * 1_000_000_000, 0);
        let knots = (NUM_FRAMES as i64 / CAM_FREQ) / KNOT_TIME + 5;
        spline.gen_random_trajectory(knots as i32, false, || uniform.sample());
        let mut result = Self {
            calibration,
            spline,
            points: Vec::new(),
            frame_poses: Vec::new(),
            frame_timestamps: Vec::new(),
            imu_timestamps: Vec::new(),
            accel: Vec::new(),
            gyro: Vec::new(),
            velocity: Vec::new(),
            accel_bias: vec![uniform.vector() / 10.],
            gyro_bias: vec![uniform.vector() / 100.],
            noisy_accel: Vec::new(),
            noisy_gyro: Vec::new(),
            observations: BTreeMap::new(),
            noisy_observations: BTreeMap::new(),
            images: Vec::new(),
            imu_diagnostics: Vec::new(),
        };
        for resolution in &result.calibration.resolution {
            let mut image = ManagedImage::new(
                usize::try_from(resolution.x).expect("nonnegative image width"),
                usize::try_from(resolution.y).expect("nonnegative image height"),
            );
            image.data_mut().fill(200);
            result.images.push(image);
        }
        let frame_dt = 1_000_000_000 / CAM_FREQ;
        for i in 0..NUM_FRAMES {
            let time = i as i64 * frame_dt;
            result.frame_timestamps.push(time);
            result.frame_poses.push(result.spline.pose(time, None));
        }
        let imu_dt = 1_000_000_000 / IMU_FREQ;
        let offset = imu_dt / 2;
        let mut time = offset;
        let mut accel_noise = Gaussian::default();
        let mut gyro_noise = Gaussian::default();
        let mut accel_walk = Gaussian::default();
        let mut gyro_walk = Gaussian::default();
        let accel_std = result.calibration.dicrete_time_accel_noise_std()[0];
        let gyro_std = result.calibration.dicrete_time_gyro_noise_std()[0];
        while time < result.frame_timestamps[NUM_FRAMES - 1] {
            let pose = result.spline.pose(time, None);
            let accel = so3_inverse(&pose.rotation)
                * (result.spline.trans_accel_world(time) - Vector3::new(0., 0., -9.81));
            let gyro = result.spline.rot_vel_body(time);
            let velocity = result.spline.trans_vel_world(time);
            result.accel.push(accel);
            result.gyro.push(gyro);
            result.velocity.push(velocity);
            let accel_bias = *result.accel_bias.last().unwrap();
            let gyro_bias = *result.gyro_bias.last().unwrap();
            let mut noisy_accel = accel;
            for value in noisy_accel.iter_mut() {
                *value += accel_noise.sample(&mut rng, accel_std);
            }
            noisy_accel += accel_bias;
            let mut noisy_gyro = gyro;
            for value in noisy_gyro.iter_mut() {
                *value += gyro_noise.sample(&mut rng, gyro_std);
            }
            noisy_gyro += gyro_bias;
            result.noisy_accel.push(noisy_accel);
            result.noisy_gyro.push(noisy_gyro);
            // Preserve the original extra half-interval offset in exported timestamps.
            result.imu_timestamps.push(time + offset);
            let mut diagnostic = [0_f32; 19];
            diagnostic[0] = (time as f64 * 1e-9) as f32;
            for (i, vector) in [
                accel,
                gyro,
                velocity,
                pose.translation.vector,
                gyro_bias,
                accel_bias,
            ]
            .iter()
            .enumerate()
            {
                for axis in 0..3 {
                    diagnostic[1 + i * 3 + axis] = vector[axis] as f32;
                }
            }
            result.imu_diagnostics.push(diagnostic);
            let dt_sqrt = (imu_dt as f64 * 1e-9).sqrt();
            let mut next_accel = accel_bias;
            for value in next_accel.iter_mut() {
                *value +=
                    accel_walk.sample(&mut rng, result.calibration.accel_bias_std[0]) * dt_sqrt;
            }
            result.accel_bias.push(next_accel);
            let mut next_gyro = gyro_bias;
            for value in next_gyro.iter_mut() {
                *value += gyro_walk.sample(&mut rng, result.calibration.gyro_bias_std[0]) * dt_sqrt;
            }
            result.gyro_bias.push(next_gyro);
            time += imu_dt;
        }
        for _ in 0..num_points {
            result
                .points
                .push(uniform.vector().normalize() * point_distance);
        }
        result.compute_projections(&mut rng);
        result
    }
    fn compute_projections(&mut self, rng: &mut Mt19937) {
        let mut distribution = Gaussian::default();
        for (frame, &timestamp) in self.frame_timestamps.iter().enumerate() {
            for camera in 0..self.calibration.t_i_c.len() {
                let mut observations = SimObservations::default();
                let mut noisy = SimObservations::default();
                let transform = se3_inverse(&se3_product(
                    &self.frame_poses[frame],
                    &self.calibration.t_i_c[camera],
                ));
                for (id, point) in self.points.iter().enumerate() {
                    let point = transform.rotation * point + transform.translation.vector;
                    if point.z <= 0.1 {
                        continue;
                    }
                    let mut pixel = Vector2::zeros();
                    let _ = self.calibration.intrinsics[camera].project(
                        &Vector4::new(point.x, point.y, point.z, 0.),
                        &mut pixel,
                        None,
                    );
                    let resolution = self.calibration.resolution[camera];
                    if pixel.x >= 5.
                        && pixel.y >= 5.
                        && pixel.x < f64::from(resolution.x - 6)
                        && pixel.y < f64::from(resolution.y - 6)
                    {
                        observations.pos.push(pixel);
                        observations.id.push(id as i32);
                        pixel.x += distribution.sample(rng, OBS_STD_DEV);
                        pixel.y += distribution.sample(rng, OBS_STD_DEV);
                        noisy.pos.push(pixel);
                        noisy.id.push(id as i32);
                    }
                }
                let id = TimeCamId::new(timestamp, camera);
                self.observations.insert(id, observations);
                self.noisy_observations.insert(id, noisy);
            }
        }
    }
    /// # Panics
    /// Requires a valid frame index.
    #[must_use]
    pub fn vision_input(&self, frame: usize) -> OpticalFlowResult {
        let time = self.frame_timestamps[frame];
        let observations = (0..self.calibration.t_i_c.len())
            .map(|camera| {
                let obs = &self.noisy_observations[&TimeCamId::new(time, camera)];
                obs.id
                    .iter()
                    .zip(&obs.pos)
                    .map(|(&id, pos)| {
                        (
                            id as usize,
                            AffineCompact2 {
                                translation: pos.cast(),
                                ..AffineCompact2::default()
                            },
                        )
                    })
                    .collect()
            })
            .collect();
        OpticalFlowResult {
            t_ns: time,
            observations,
            ..OpticalFlowResult::default()
        }
    }
    /// # Panics
    /// Requires a valid IMU index.
    #[must_use]
    pub fn imu_input(&self, index: usize) -> ImuData<f64> {
        ImuData {
            t_ns: self.imu_timestamps[index],
            accel: self.noisy_accel[index],
            gyro: self.noisy_gyro[index],
        }
    }
    /// Initialize the original VO/VIO worker at the exact simulated state.
    /// # Errors
    /// Returns estimator construction or worker startup errors.
    /// # Panics
    /// Requires a generated, nonempty simulation.
    pub fn setup_vio(
        &self,
        config: &VioConfig,
        use_imu: bool,
        use_double: bool,
    ) -> firefly_error::Result<VioEstimatorBase> {
        self.setup_vio_with_outputs(config, use_imu, use_double, VioWorkerOutputs::default())
    }
    /// Attach output queues before initializing and starting the worker.
    /// # Errors
    /// Returns estimator construction or worker startup errors.
    /// # Panics
    /// Requires a generated, nonempty simulation.
    pub fn setup_vio_with_outputs(
        &self,
        config: &VioConfig,
        use_imu: bool,
        use_double: bool,
        outputs: VioWorkerOutputs,
    ) -> firefly_error::Result<VioEstimatorBase> {
        let mut estimator = VioEstimatorFactory::get_vio_estimator(
            config,
            &self.calibration,
            &Vector3::new(0., 0., -9.81),
            use_imu,
            use_double,
        )?;
        estimator.outputs = outputs;
        estimator.initialize_state(
            self.frame_timestamps[0],
            self.frame_poses[0],
            self.spline.trans_vel_world(self.frame_timestamps[0]),
            self.gyro_bias[0],
            self.accel_bias[0],
        )?;
        Ok(estimator)
    }
    /// Original typed input payloads, suitable for the two independent feeder queues.
    pub fn imu_inputs(&self) -> impl Iterator<Item = Arc<ImuData<f64>>> + '_ {
        (0..self.imu_timestamps.len()).map(|i| Arc::new(self.imu_input(i)))
    }
    pub fn vision_inputs(&self) -> impl Iterator<Item = Arc<OpticalFlowResult>> + '_ {
        (0..self.frame_timestamps.len()).map(|i| Arc::new(self.vision_input(i)))
    }
    /// Write the original `gt_spline.cereal` JSON schema, including its duplicate
    /// `noisy_accel` and `gt_points` keys.
    /// # Errors
    /// Returns archive or output errors.
    pub fn write_archive(&self, output: impl Write) -> io::Result<()> {
        serde_json::to_writer_pretty(output, &SimulationArchive(self)).map_err(io::Error::other)
    }
}

struct SimulationArchive<'a>(&'a Simulation);
impl Serialize for SimulationArchive<'_> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        #[derive(Serialize)]
        struct Pose {
            px: f64,
            py: f64,
            pz: f64,
            qx: f64,
            qy: f64,
            qz: f64,
            qw: f64,
        }
        #[derive(Serialize)]
        struct Observation<'a> {
            key: &'a TimeCamId,
            value: &'a SimObservations,
        }
        let data = self.0;
        let vectors =
            |values: &[Vector3<f64>]| values.iter().map(|v| [v.x, v.y, v.z]).collect::<Vec<_>>();
        let knots = (0..data.spline.num_knots())
            .map(|i| {
                let p = data.spline.knot(i);
                let t = p.translation.vector;
                let q = p.rotation.quaternion();
                Pose {
                    px: t.x,
                    py: t.y,
                    pz: t.z,
                    qx: q.i,
                    qy: q.j,
                    qz: q.k,
                    qw: q.w,
                }
            })
            .collect::<Vec<_>>();
        let mut archive = serializer.serialize_map(Some(14))?;
        archive.serialize_entry("t_ns", &(KNOT_TIME * 1_000_000_000))?;
        archive.serialize_entry("knots", &knots)?;
        archive.serialize_entry("noisy_accel", &vectors(&data.noisy_accel))?;
        archive.serialize_entry("noisy_gyro", &vectors(&data.noisy_gyro))?;
        archive.serialize_entry("noisy_accel", &vectors(&data.accel))?;
        archive.serialize_entry("gt_gyro", &vectors(&data.gyro))?;
        archive.serialize_entry("gt_points", &vectors(&data.points))?;
        archive.serialize_entry("gt_accel_bias", &vectors(&data.accel_bias))?;
        archive.serialize_entry("gt_gyro_bias", &vectors(&data.gyro_bias))?;
        archive.serialize_entry(
            "gt_observations",
            &data
                .observations
                .iter()
                .map(|(key, value)| Observation { key, value })
                .collect::<Vec<_>>(),
        )?;
        archive.serialize_entry(
            "noisy_observations",
            &data
                .noisy_observations
                .iter()
                .map(|(key, value)| Observation { key, value })
                .collect::<Vec<_>>(),
        )?;
        archive.serialize_entry("gt_points", &vectors(&data.points))?;
        archive.serialize_entry("gt_frame_t_ns", &data.frame_timestamps)?;
        archive.serialize_entry("gt_imu_t_ns", &data.imu_timestamps)?;
        archive.end()
    }
}
