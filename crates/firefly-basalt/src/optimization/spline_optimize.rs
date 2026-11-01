// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `optimization/spline_optimize.h` and its sparse LM workflow.
use super::{
    accumulator::SparseHashAccumulator,
    linearize::{CalibCommonData, ReprojectionStats},
    poses_optimize::{load_calibration, save_calibration},
    spline_linearize::{ComputeErrorSplineOpt, LinearizeSplineOpt},
};
use crate::io::dataset_io::{AccelData, AprilgridCornersData, GyroData, MocapPoseData, PoseData};
use firefly_basalt_headers::{
    calibration::calib_bias::{CalibAccelBias, CalibGyroBias},
    calibration::{Calibration, MocapCalibration},
    camera::generic_camera::GenericCamera,
    spline::se3_spline::Se3Spline,
    utils::sophus_utils::{se3_expd, se3_inverse, se3_product, so3_exp, so3_product},
};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{DVector, Isometry3, Vector2, Vector3, Vector4};
use serde::Serialize;
use std::{
    collections::{BTreeMap, VecDeque},
    fs::File,
    io::{BufWriter, Write},
    path::Path,
};

#[derive(Clone, Copy, Debug)]
#[expect(
    clippy::struct_excessive_bools,
    reason = "Independent switches in the upstream optimizer interface"
)]
pub struct Options {
    pub use_intrinsics: bool,
    pub use_poses: bool,
    pub use_april_corners: bool,
    pub opt_cam_time_offset: bool,
    pub opt_imu_scale: bool,
    pub use_mocap: bool,
    pub huber_thresh: f64,
    pub stop_thresh: f64,
    pub print_info: bool,
}
#[derive(Serialize)]
struct MocapArchive<'a> {
    value0: &'a MocapCalibration<f64>,
}

pub struct SplineOptimization<const N: usize> {
    pose_var: f64,
    pub pose_var_inv: f64,
    pub calib: Option<Calibration<f64>>,
    pub mocap_calib: Option<MocapCalibration<f64>>,
    pub mocap_initialized: bool,
    lambda: f64,
    min_lambda: f64,
    max_lambda: f64,
    lambda_vee: f64,
    min_time_us: i64,
    max_time_us: i64,
    pose_measurements: Vec<PoseData>,
    gyro_measurements: Vec<GyroData>,
    accel_measurements: Vec<AccelData>,
    aprilgrid_corners_measurements: Vec<AprilgridCornersData>,
    mocap_measurements: VecDeque<MocapPoseData>,
    offset_cam_intrinsics: Vec<usize>,
    offset_t_i_c: Vec<usize>,
    offset_poses: BTreeMap<i64, usize>,
    mocap_block_offset: usize,
    bias_block_offset: usize,
    opt_size: usize,
    spline: Se3Spline<f64, N>,
    g: Vector3<f64>,
    aprilgrid_corner_pos_3d: Vec<Vector4<f64>>,
    dt_ns: i64,
    gyro_var_inv: Vector3<f64>,
    accel_var_inv: Vector3<f64>,
    mocap_var_inv: f64,
    initialized_pose_var_inv: f64,
}
impl<const N: usize> Default for SplineOptimization<N> {
    fn default() -> Self {
        Self::new(10_000_000, 1e-12)
    }
}
impl<const N: usize> SplineOptimization<N> {
    #[must_use]
    pub fn new(dt_ns: i64, initial_lambda: f64) -> Self {
        Self {
            pose_var: 1e-4,
            pose_var_inv: 1e4,
            calib: None,
            mocap_calib: None,
            mocap_initialized: false,
            lambda: initial_lambda,
            min_lambda: 1e-18,
            max_lambda: 100.,
            lambda_vee: 2.,
            min_time_us: i64::MAX,
            max_time_us: i64::MIN,
            pose_measurements: Vec::new(),
            gyro_measurements: Vec::new(),
            accel_measurements: Vec::new(),
            aprilgrid_corners_measurements: Vec::new(),
            mocap_measurements: VecDeque::new(),
            offset_cam_intrinsics: Vec::new(),
            offset_t_i_c: Vec::new(),
            offset_poses: BTreeMap::new(),
            mocap_block_offset: 0,
            bias_block_offset: 0,
            opt_size: 0,
            spline: Se3Spline::new(dt_ns, 0),
            g: Vector3::zeros(),
            aprilgrid_corner_pos_3d: Vec::new(),
            dt_ns,
            gyro_var_inv: Vector3::zeros(),
            accel_var_inv: Vector3::zeros(),
            mocap_var_inv: 0.,
            initialized_pose_var_inv: 0.,
        }
    }
    #[must_use]
    pub const fn pose_var(&self) -> f64 {
        self.pose_var
    }
    #[must_use]
    /// # Panics
    /// Requires initialized calibration.
    pub fn cam_time_offset_ns(&self) -> i64 {
        self.calib
            .as_ref()
            .expect("calibration initialized")
            .cam_time_offset_ns
    }
    #[must_use]
    /// # Panics
    /// Requires initialized motion-capture calibration.
    pub fn mocap_time_offset_ns(&self) -> i64 {
        self.mocap_calib
            .as_ref()
            .expect("mocap calibration initialized")
            .mocap_time_offset_ns
    }
    #[must_use]
    /// # Panics
    /// Requires initialized calibration and a valid camera index.
    pub fn cam_t_i_c(&self, index: usize) -> &Isometry3<f64> {
        &self.calib.as_ref().expect("calibration initialized").t_i_c[index]
    }
    /// # Panics
    /// Requires initialized calibration and a valid camera index.
    pub fn cam_t_i_c_mut(&mut self, index: usize) -> &mut Isometry3<f64> {
        &mut self.calib.as_mut().expect("calibration initialized").t_i_c[index]
    }
    #[must_use]
    /// # Panics
    /// Requires initialized calibration and a valid camera index.
    pub fn intrinsics(&self, index: usize) -> DVector<f64> {
        self.calib
            .as_ref()
            .expect("calibration initialized")
            .intrinsics[index]
            .param()
    }
    #[must_use]
    /// # Panics
    /// Requires initialized calibration.
    pub fn accel_bias(&self) -> &CalibAccelBias<f64> {
        &self
            .calib
            .as_ref()
            .expect("calibration initialized")
            .calib_accel_bias
    }
    #[must_use]
    /// # Panics
    /// Requires initialized calibration.
    pub fn gyro_bias(&self) -> &CalibGyroBias<f64> {
        &self
            .calib
            .as_ref()
            .expect("calibration initialized")
            .calib_gyro_bias
    }
    /// The original overload validates the names' count but default-constructs all models.
    /// # Panics
    /// Requires one camera name per camera.
    pub fn reset_calib(&mut self, num_cams: usize, camera_types: &[String]) {
        assert_eq!(num_cams, camera_types.len());
        self.calib = Some(Calibration {
            intrinsics: vec![GenericCamera::default(); num_cams],
            t_i_c: vec![Isometry3::identity(); num_cams],
            ..Calibration::default()
        });
        self.reset_mocap_calib();
    }
    pub fn reset_mocap_calib(&mut self) {
        self.mocap_calib = Some(MocapCalibration::default());
    }
    /// # Errors
    /// Propagates invalid calibration archives and file I/O errors except missing files.
    pub fn load_calib(&mut self, directory: &Path) -> Result<()> {
        if let Some(calibration) = load_calibration(&directory.join("calibration.json"))? {
            self.calib = Some(calibration);
        } else {
            log::warn!("No calibration found; run camera calibration first");
        }
        Ok(())
    }
    /// # Errors
    /// Propagates calibration serialization and file I/O errors.
    pub fn save_calib(&self, directory: &Path) -> Result<()> {
        if let Some(calibration) = &self.calib {
            save_calibration(&directory.join("calibration.json"), calibration)?;
        }
        Ok(())
    }
    /// # Panics
    /// Requires motion-capture calibration when camera calibration exists.
    /// # Errors
    /// Propagates motion-capture serialization and file I/O errors.
    pub fn save_mocap_calib(
        &mut self,
        directory: &Path,
        mocap_to_imu_offset_ns: i64,
    ) -> Result<()> {
        if self.calib.is_none() {
            return Ok(());
        }
        let calibration = self
            .mocap_calib
            .as_mut()
            .expect("mocap calibration initialized");
        calibration.mocap_to_imu_offset_ns = mocap_to_imu_offset_ns;
        let error = |source| {
            Error::new(
                ErrorKind::InvalidArgument,
                "Cannot save motion-capture calibration",
            )
            .with_operation("basalt.spline_optimize.save_mocap")
            .with_source(source)
        };
        let mut output =
            BufWriter::new(File::create(directory.join("mocap_calibration.json")).map_err(error)?);
        serde_json::to_writer_pretty(
            &mut output,
            &MocapArchive {
                value0: calibration,
            },
        )
        .map_err(|source| {
            Error::new(
                ErrorKind::InvalidArgument,
                "Cannot serialize motion-capture calibration",
            )
            .with_operation("basalt.spline_optimize.save_mocap")
            .with_source(source)
        })?;
        output.flush().map_err(error)
    }
    #[must_use]
    pub fn calib_initialized(&self) -> bool {
        self.calib.is_some()
    }
    #[must_use]
    pub fn initialized(&self) -> bool {
        self.spline.num_knots() > 0
    }
    pub fn init_spline(&mut self, pose: &Isometry3<f64>, count: usize) {
        self.spline.set_knots(pose, count);
    }
    pub fn init_spline_from(&mut self, other: &Se3Spline<f64, N>, mut sample: impl FnMut() -> f64) {
        self.spline.copy_knots_from(other);
        for i in 0..self.spline.num_knots() {
            let rotation = Vector3::from_fn(|_, _| sample()) / 20.;
            let translation = Vector3::from_fn(|_, _| sample());
            *self.spline.knot_so3_mut(i) =
                so3_product(self.spline.knot_so3(i), &so3_exp(&rotation));
            *self.spline.knot_pos_mut(i) += translation;
        }
    }
    #[must_use]
    pub const fn spline(&self) -> &Se3Spline<f64, N> {
        &self.spline
    }
    #[must_use]
    pub const fn g(&self) -> Vector3<f64> {
        self.g
    }
    pub fn set_g(&mut self, gravity: Vector3<f64>) {
        self.g = gravity;
    }
    #[must_use]
    /// # Panics
    /// Requires initialized motion-capture calibration.
    pub fn t_w_moc(&self) -> Isometry3<f64> {
        se3_inverse(
            &self
                .mocap_calib
                .as_ref()
                .expect("mocap calibration initialized")
                .t_moc_w,
        )
    }
    /// # Panics
    /// Requires initialized motion-capture calibration.
    pub fn set_t_w_moc(&mut self, pose: &Isometry3<f64>) {
        self.mocap_calib
            .as_mut()
            .expect("mocap calibration initialized")
            .t_moc_w = se3_inverse(pose);
    }
    #[must_use]
    /// # Panics
    /// Requires initialized motion-capture calibration.
    pub fn t_mark_i(&self) -> Isometry3<f64> {
        se3_inverse(
            &self
                .mocap_calib
                .as_ref()
                .expect("mocap calibration initialized")
                .t_i_mark,
        )
    }
    /// # Panics
    /// Requires initialized motion-capture calibration.
    pub fn set_t_mark_i(&mut self, pose: &Isometry3<f64>) {
        self.mocap_calib
            .as_mut()
            .expect("mocap calibration initialized")
            .t_i_mark = se3_inverse(pose);
    }
    #[must_use]
    pub fn trans_accel_world(&self, time: i64) -> Vector3<f64> {
        self.spline.trans_accel_world(time)
    }
    #[must_use]
    pub fn rot_vel_body(&self, time: i64) -> Vector3<f64> {
        self.spline.rot_vel_body(time)
    }
    #[must_use]
    pub fn t_w_i(&self, time: i64) -> Isometry3<f64> {
        self.spline.pose(time, None)
    }
    pub fn set_aprilgrid_corners_3d(&mut self, corners: &[Vector4<f64>]) {
        self.aprilgrid_corner_pos_3d = corners.to_vec();
    }
    fn extend_time(&mut self, time: i64) {
        self.min_time_us = self.min_time_us.min(time);
        self.max_time_us = self.max_time_us.max(time);
    }
    pub fn add_pose_measurement(&mut self, time: i64, pose: Isometry3<f64>) {
        self.extend_time(time);
        self.pose_measurements.push(PoseData {
            timestamp_ns: time,
            data: pose,
        });
    }
    pub fn add_mocap_measurement(&mut self, time: i64, pose: Isometry3<f64>) {
        self.mocap_measurements.push_back(MocapPoseData {
            timestamp_ns: time,
            data: pose,
        });
    }
    pub fn add_accel_measurement(&mut self, time: i64, data: Vector3<f64>) {
        self.extend_time(time);
        self.accel_measurements.push(AccelData {
            timestamp_ns: time,
            data,
        });
    }
    pub fn add_gyro_measurement(&mut self, time: i64, data: Vector3<f64>) {
        self.extend_time(time);
        self.gyro_measurements.push(GyroData {
            timestamp_ns: time,
            data,
        });
    }
    pub fn add_aprilgrid_measurement(
        &mut self,
        time: i64,
        camera: i32,
        corners: &[Vector2<f64>],
        ids: &[i32],
    ) {
        self.extend_time(time);
        self.aprilgrid_corners_measurements
            .push(AprilgridCornersData {
                timestamp_ns: time,
                cam_id: camera,
                corner_pos: corners.to_vec(),
                corner_id: ids.to_vec(),
            });
    }
    #[must_use]
    pub fn min_time(&self) -> f64 {
        self.min_time_us as f64 * 1e-9
    }
    #[must_use]
    pub fn max_time(&self) -> f64 {
        self.max_time_us as f64 * 1e-9
    }
    #[must_use]
    pub const fn min_time_ns(&self) -> i64 {
        self.min_time_us
    }
    #[must_use]
    pub const fn max_time_ns(&self) -> i64 {
        self.max_time_us
    }
    /// # Panics
    /// Requires initialized calibration and at least one pose if the spline is empty.
    pub fn init(&mut self) {
        let interval = self.max_time_us - self.min_time_us;
        if self.spline.num_knots() == 0 {
            self.spline.set_start_time_ns(self.min_time_us);
            self.spline.set_knots(
                &self
                    .pose_measurements
                    .first()
                    .expect("initial pose measurement")
                    .data,
                usize::try_from(interval / self.dt_ns).unwrap() + N + 1,
            );
        }
        self.recompute_size();
        while self.mocap_measurements.front().is_some_and(|value| {
            value.timestamp_ns <= self.spline.min_time_ns() + self.spline.time_interval_ns()
        }) {
            self.mocap_measurements.pop_front();
        }
        while self.mocap_measurements.back().is_some_and(|value| {
            value.timestamp_ns >= self.spline.max_time_ns() - self.spline.time_interval_ns()
        }) {
            self.mocap_measurements.pop_back();
        }
        let calibration = self.calib.as_ref().expect("calibration initialized");
        self.gyro_var_inv = calibration
            .dicrete_time_gyro_noise_std()
            .map(|value| 1. / (value * value));
        self.accel_var_inv = calibration
            .dicrete_time_accel_noise_std()
            .map(|value| 1. / (value * value));
        self.mocap_var_inv = self.pose_var_inv;
        self.initialized_pose_var_inv = self.pose_var_inv;
    }
    /// # Panics
    /// Requires initialized calibration.
    pub fn recompute_size(&mut self) {
        self.offset_cam_intrinsics.clear();
        self.bias_block_offset = 6 * self.spline.num_knots();
        let extrinsic_offset = self.bias_block_offset + 24;
        self.offset_t_i_c.push(extrinsic_offset);
        let calibration = self.calib.as_ref().expect("calibration initialized");
        for _ in &calibration.t_i_c {
            self.offset_t_i_c
                .push(self.offset_t_i_c.last().unwrap() + 6);
        }
        self.offset_cam_intrinsics
            .push(*self.offset_t_i_c.last().unwrap());
        for camera in &calibration.intrinsics {
            self.offset_cam_intrinsics
                .push(self.offset_cam_intrinsics.last().unwrap() + camera.num_parameters());
        }
        self.mocap_block_offset = *self.offset_cam_intrinsics.last().unwrap();
        self.opt_size = self.mocap_block_offset + 14;
    }
    fn common_data(&self, options: Options) -> CalibCommonData<'_> {
        CalibCommonData {
            calibration: self.calib.as_ref().expect("calibration initialized"),
            mocap_calibration: self.mocap_calib.as_ref(),
            aprilgrid_corner_pos_3d: &self.aprilgrid_corner_pos_3d,
            offset_t_i_c: &self.offset_t_i_c,
            offset_intrinsics: &self.offset_cam_intrinsics,
            mocap_block_offset: self.mocap_block_offset,
            bias_block_offset: self.bias_block_offset,
            offset_poses: &self.offset_poses,
            g: Some(&self.g),
            opt_intrinsics: options.use_intrinsics,
            opt_cam_time_offset: options.opt_cam_time_offset,
            opt_g: true,
            opt_imu_scale: options.opt_imu_scale,
            pose_var_inv: self.initialized_pose_var_inv,
            gyro_var_inv: self.gyro_var_inv,
            accel_var_inv: self.accel_var_inv,
            mocap_var_inv: self.mocap_var_inv,
            huber_thresh: options.huber_thresh,
        }
    }
    fn linearize(&self, options: Options) -> (SparseHashAccumulator<f64>, ReprojectionStats) {
        let mut linearization =
            LinearizeSplineOpt::<N>::new(self.opt_size, &self.spline, self.common_data(options));
        if options.use_poses {
            linearization.parallel_linearize(&self.pose_measurements);
        }
        if options.use_april_corners {
            linearization.parallel_linearize(&self.aprilgrid_corners_measurements);
        }
        if options.use_mocap && self.mocap_initialized {
            let (first, second) = self.mocap_measurements.as_slices();
            linearization.parallel_linearize(first);
            linearization.parallel_linearize(second);
        } else if options.use_mocap {
            log::info!("Mocap residuals are not used; initialize Mocap first");
        }
        linearization.parallel_linearize(&self.accel_measurements);
        linearization.parallel_linearize(&self.gyro_measurements);
        (linearization.accum, linearization.stats)
    }
    fn compute_error(&self, options: Options) -> ReprojectionStats {
        let mut evaluator =
            ComputeErrorSplineOpt::<N>::new(self.opt_size, &self.spline, self.common_data(options));
        if options.use_poses {
            evaluator.parallel_error(&self.pose_measurements);
        }
        if options.use_april_corners {
            evaluator.parallel_error(&self.aprilgrid_corners_measurements);
        }
        if options.use_mocap && self.mocap_initialized {
            let (first, second) = self.mocap_measurements.as_slices();
            evaluator.parallel_error(first);
            evaluator.parallel_error(second);
        } else if options.use_mocap {
            log::info!("Mocap residuals are not used; initialize Mocap first");
        }
        evaluator.parallel_error(&self.accel_measurements);
        evaluator.parallel_error(&self.gyro_measurements);
        evaluator.stats
    }
    pub fn optimize(&mut self, options: Options, stats: &mut ReprojectionStats) -> bool {
        let (mut accumulator, initial) = self.linearize(options);
        *stats = initial;
        if options.print_info {
            log::info!(
                "[LINEARIZE] Error: {} num points {}",
                initial.error,
                initial.num_points
            );
        }
        accumulator.setup_solver();
        let diagonal = accumulator.h_diagonal();
        let mut converged = false;
        let mut accepted = false;
        let mut attempts = 10;
        while !accepted && attempts > 0 && !converged {
            let damping = diagonal.map(|value| (value * self.lambda).max(self.min_lambda));
            let increment = -accumulator.solve(Some(&damping));
            let maximum = increment.amax();
            if maximum < options.stop_thresh {
                converged = true;
            }
            let calibration = self.calib.clone();
            let mocap = self.mocap_calib.clone();
            let spline = self.spline.clone();
            let gravity = self.g;
            self.apply_inc(&increment);
            let candidate = self.compute_error(options);
            let actual = initial.error - candidate.error;
            let predicted = 0.5 * increment.dot(&(&increment * self.lambda - accumulator.b()));
            let quality = actual / predicted;
            if quality < 0. {
                if options.print_info {
                    log::info!(
                        "[REJECTED] lambda:{} step_quality:{quality} max_inc:{maximum} Error:{} num points:{}",
                        self.lambda,
                        candidate.error,
                        candidate.num_points
                    );
                }
                self.lambda = (self.lambda_vee * self.lambda).min(self.max_lambda);
                self.lambda_vee *= 2.;
                self.spline = spline;
                self.calib = calibration;
                self.mocap_calib = mocap;
                self.g = gravity;
            } else {
                if options.print_info {
                    log::info!(
                        "[ACCEPTED] lambda:{} step_quality:{quality} max_inc:{maximum} Error:{} num points:{}",
                        self.lambda,
                        candidate.error,
                        candidate.num_points
                    );
                }
                self.lambda = (self.lambda * (1. / 3_f64).max(1. - (2. * quality - 1.).powf(3.)))
                    .max(self.min_lambda);
                self.lambda_vee = 2.;
                *stats = candidate;
                accepted = true;
            }
            attempts -= 1;
        }
        if converged && options.print_info {
            log::info!("[CONVERGED]");
        }
        converged
    }
    fn apply_inc(&mut self, increment: &DVector<f64>) {
        let count = self.spline.num_knots();
        for i in 0..count {
            self.spline
                .apply_inc(i, &increment.fixed_rows::<6>(6 * i).into_owned());
        }
        let bias_offset = 6 * count;
        let calibration = self.calib.as_mut().expect("calibration initialized");
        calibration
            .calib_accel_bias
            .apply_increment(&increment.fixed_rows::<9>(bias_offset).into_owned());
        calibration
            .calib_gyro_bias
            .apply_increment(&increment.fixed_rows::<12>(bias_offset + 9).into_owned());
        self.g += increment.fixed_rows::<3>(bias_offset + 21);
        let extrinsic_offset = bias_offset + 24;
        for (i, pose) in calibration.t_i_c.iter_mut().enumerate() {
            *pose = se3_product(
                pose,
                &se3_expd(
                    &increment
                        .fixed_rows::<6>(extrinsic_offset + i * 6)
                        .into_owned(),
                ),
            );
        }
        for (i, camera) in calibration.intrinsics.iter_mut().enumerate() {
            camera.apply_increment(
                &increment
                    .rows(self.offset_cam_intrinsics[i], camera.num_parameters())
                    .into_owned(),
            );
        }
        let mocap_offset = *self.offset_cam_intrinsics.last().unwrap();
        let mocap = self
            .mocap_calib
            .as_mut()
            .expect("mocap calibration initialized");
        mocap.t_moc_w = se3_product(
            &mocap.t_moc_w,
            &se3_expd(&increment.fixed_rows::<6>(mocap_offset).into_owned()),
        );
        mocap.t_i_mark = se3_product(
            &mocap.t_i_mark,
            &se3_expd(&increment.fixed_rows::<6>(mocap_offset + 6).into_owned()),
        );
        mocap.mocap_time_offset_ns =
            (mocap.mocap_time_offset_ns as f64 + 1e9 * increment[mocap_offset + 12]) as i64;
        calibration.cam_time_offset_ns =
            (calibration.cam_time_offset_ns as f64 + 1e9 * increment[mocap_offset + 13]) as i64;
    }
}
