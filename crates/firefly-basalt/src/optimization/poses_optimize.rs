// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/optimization/poses_optimize.h`.
use super::{
    linearize::{CalibCommonData, ReprojectionStats},
    poses_linearize::{ComputeErrorPosesOpt, LinearizePosesOpt},
};
use crate::io::dataset_io::AprilgridCornersData;
use firefly_basalt_headers::{
    calibration::Calibration,
    camera::generic_camera::GenericCamera,
    spline::rd_spline::RdSpline,
    utils::sophus_utils::{se3_expd, se3_product},
};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Isometry3, Vector2, Vector3, Vector4};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    fs::File,
    io::{BufReader, BufWriter, Write},
    path::Path,
};

#[derive(Serialize, Deserialize)]
struct CalibrationArchive {
    value0: Calibration<f64>,
}
fn archive_error(source: impl std::error::Error + Send + Sync + 'static) -> Error {
    Error::new(
        ErrorKind::InvalidArgument,
        "Cannot process calibration archive",
    )
    .with_operation("basalt.poses_optimize.calibration")
    .with_source(source)
}
pub(crate) fn load_calibration(path: &Path) -> Result<Option<Calibration<f64>>> {
    let file = match File::open(path) {
        Ok(file) => file,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(error) => return Err(archive_error(error)),
    };
    Ok(Some(
        serde_json::from_reader::<_, CalibrationArchive>(BufReader::new(file))
            .map_err(archive_error)?
            .value0,
    ))
}
pub(crate) fn save_calibration(path: &Path, calibration: &Calibration<f64>) -> Result<()> {
    let mut output = BufWriter::new(File::create(path).map_err(archive_error)?);
    serde_json::to_writer_pretty(
        &mut output,
        &CalibrationArchive {
            value0: calibration.clone(),
        },
    )
    .map_err(archive_error)?;
    output.flush().map_err(archive_error)
}

pub struct PosesOptimization {
    pub calib: Option<Calibration<f64>>,
    lambda: f64,
    min_lambda: f64,
    max_lambda: f64,
    lambda_vee: f64,
    problem_size: usize,
    offset_poses: BTreeMap<i64, usize>,
    offset_cam_intrinsics: Vec<usize>,
    offset_t_i_c: Vec<usize>,
    timestamp_to_pose: BTreeMap<i64, Isometry3<f64>>,
    aprilgrid_corners_measurements: Vec<AprilgridCornersData>,
    aprilgrid_corner_pos_3d: Vec<Vector4<f64>>,
}
impl Default for PosesOptimization {
    fn default() -> Self {
        Self {
            calib: None,
            lambda: 1e-6,
            min_lambda: 1e-12,
            max_lambda: 100.0,
            lambda_vee: 2.0,
            problem_size: 0,
            offset_poses: BTreeMap::new(),
            offset_cam_intrinsics: Vec::new(),
            offset_t_i_c: Vec::new(),
            timestamp_to_pose: BTreeMap::new(),
            aprilgrid_corners_measurements: Vec::new(),
            aprilgrid_corner_pos_3d: Vec::new(),
        }
    }
}
impl PosesOptimization {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
    /// # Panics
    /// Requires initialized calibration and a valid camera index.
    #[must_use]
    pub fn get_optical_center(&self, camera: usize) -> Vector2<f64> {
        self.calib
            .as_ref()
            .expect("calibration initialized")
            .intrinsics[camera]
            .param()
            .fixed_rows::<2>(2)
            .into_owned()
    }
    /// # Panics
    /// Requires one camera type per camera.
    pub fn reset_calib(&mut self, num_cams: usize, camera_types: &[String]) {
        assert_eq!(num_cams, camera_types.len());
        let mut calibration = Calibration {
            intrinsics: camera_types
                .iter()
                .map(|name| GenericCamera::from_name(name))
                .collect(),
            t_i_c: vec![Isometry3::identity(); num_cams],
            ..Default::default()
        };
        for (camera, name) in calibration.intrinsics.iter_mut().zip(camera_types) {
            if camera.name() != name {
                log::warn!("Unknown camera type {name}; using {}", camera.name());
            }
        }
        self.calib = Some(calibration);
    }
    /// # Errors
    /// Propagates malformed archives and I/O errors other than a missing file.
    pub fn load_calib(&mut self, directory: &Path) -> Result<()> {
        if let Some(calibration) = load_calibration(&directory.join("calibration.json"))? {
            self.calib = Some(calibration);
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
    #[must_use]
    pub fn calib_initialized(&self) -> bool {
        self.calib.is_some()
    }
    #[must_use]
    pub fn initialized(&self) -> bool {
        true
    }
    /// # Panics
    /// Requires calibration to be initialized.
    pub fn recompute_size(&mut self) {
        self.offset_poses.clear();
        self.offset_t_i_c.clear();
        self.offset_cam_intrinsics.clear();
        let mut offset = 0;
        for &time in self.timestamp_to_pose.keys() {
            self.offset_poses.insert(time, offset);
            offset += 6;
        }
        let calibration = self.calib.as_ref().expect("calibration initialized");
        self.offset_t_i_c.push(offset);
        for _ in &calibration.t_i_c {
            offset += 6;
            self.offset_t_i_c.push(offset);
        }
        self.offset_cam_intrinsics.push(offset);
        for camera in &calibration.intrinsics {
            offset += camera.num_parameters();
            self.offset_cam_intrinsics.push(offset);
        }
        self.problem_size = offset;
    }
    pub fn init(&mut self) {
        self.recompute_size();
    }
    #[must_use]
    pub fn get_t_w_i(&self, time: i64) -> Isometry3<f64> {
        self.timestamp_to_pose
            .get(&time)
            .copied()
            .unwrap_or_default()
    }
    pub fn set_aprilgrid_corners_3d(&mut self, corners: &[Vector4<f64>]) {
        self.aprilgrid_corner_pos_3d = corners.to_vec();
    }
    pub fn add_aprilgrid_measurement(
        &mut self,
        time: i64,
        camera: i32,
        corners: &[Vector2<f64>],
        ids: &[i32],
    ) {
        self.aprilgrid_corners_measurements
            .push(AprilgridCornersData {
                timestamp_ns: time,
                cam_id: camera,
                corner_pos: corners.to_vec(),
                corner_id: ids.to_vec(),
            });
    }
    pub fn add_pose_measurement(&mut self, time: i64, pose: Isometry3<f64>) {
        self.timestamp_to_pose.insert(time, pose);
    }
    /// # Panics
    /// Requires initialized calibration.
    pub fn set_vignette(&mut self, vignette: &[RdSpline<f64, 1, 4>]) {
        self.calib
            .as_mut()
            .expect("calibration initialized")
            .vignette = vignette.to_vec();
    }
    /// # Panics
    /// Requires initialized calibration.
    pub fn set_resolution(&mut self, resolution: &[Vector2<i32>]) {
        self.calib
            .as_mut()
            .expect("calibration initialized")
            .resolution = resolution.to_vec();
    }
    fn common_data(&self, opt_intrinsics: bool, huber_thresh: f64) -> CalibCommonData<'_> {
        CalibCommonData {
            calibration: self.calib.as_ref().expect("calibration initialized"),
            mocap_calibration: None,
            aprilgrid_corner_pos_3d: &self.aprilgrid_corner_pos_3d,
            offset_t_i_c: &self.offset_t_i_c,
            offset_intrinsics: &self.offset_cam_intrinsics,
            mocap_block_offset: 0,
            bias_block_offset: 0,
            offset_poses: &self.offset_poses,
            g: None,
            opt_intrinsics,
            opt_cam_time_offset: false,
            opt_g: false,
            opt_imu_scale: false,
            pose_var_inv: 0.0,
            gyro_var_inv: Vector3::zeros(),
            accel_var_inv: Vector3::zeros(),
            mocap_var_inv: 0.0,
            huber_thresh,
        }
    }
    /// Performs one linearization and up to ten LM damping trials.
    /// # Panics
    /// Requires `init`, valid observations and initialized calibration.
    pub fn optimize(
        &mut self,
        opt_intrinsics: bool,
        huber_thresh: f64,
        stop_thresh: f64,
        stats: &mut ReprojectionStats,
    ) -> bool {
        let mut linearization = LinearizePosesOpt::new(
            self.problem_size,
            &self.timestamp_to_pose,
            self.common_data(opt_intrinsics, huber_thresh),
        );
        linearization.parallel_linearize(&self.aprilgrid_corners_measurements);
        *stats = linearization.stats;
        let initial_error = stats.error;
        let mut accum = linearization.accum;
        accum.setup_solver();
        let diagonal = accum.h_diagonal();
        let mut converged = false;
        for _ in 0..10 {
            let poses_backup = self.timestamp_to_pose.clone();
            let calibration_backup = self.calib.clone();
            let damping = diagonal.map(|d| (d * self.lambda).max(self.min_lambda));
            let inc = -accum.solve(Some(&damping));
            let maximum = inc.amax();
            if maximum < stop_thresh {
                converged = true;
            }
            for (time, pose) in &mut self.timestamp_to_pose {
                *pose = se3_product(
                    pose,
                    &se3_expd(&inc.fixed_rows::<6>(self.offset_poses[time]).into_owned()),
                );
            }
            let calibration = self.calib.as_mut().expect("calibration initialized");
            for (i, pose) in calibration.t_i_c.iter_mut().enumerate() {
                *pose = se3_product(
                    pose,
                    &se3_expd(&inc.fixed_rows::<6>(self.offset_t_i_c[i]).into_owned()),
                );
            }
            for (i, camera) in calibration.intrinsics.iter_mut().enumerate() {
                camera.apply_increment(
                    &inc.rows(self.offset_cam_intrinsics[i], camera.num_parameters())
                        .into_owned(),
                );
            }
            let mut error = ComputeErrorPosesOpt::new(
                self.problem_size,
                &self.timestamp_to_pose,
                self.common_data(opt_intrinsics, huber_thresh),
            );
            error.compute(&self.aprilgrid_corners_measurements);
            let trial = error.stats;
            let actual = initial_error - trial.error;
            let predicted = 0.5 * inc.dot(&(&inc * self.lambda - accum.b()));
            let quality = actual / predicted;
            if quality < 0.0 {
                self.lambda = (self.lambda_vee * self.lambda).min(self.max_lambda);
                self.lambda_vee *= 2.0;
                self.timestamp_to_pose = poses_backup;
                self.calib = calibration_backup;
                log::info!(
                    "Calibration step rejected: quality={quality}, max_inc={maximum}, error={}",
                    trial.error
                );
            } else {
                self.lambda = (self.lambda
                    * (1.0 / 3.0_f64).max(1.0 - (2.0 * quality - 1.0).powi(3)))
                .max(self.min_lambda);
                self.lambda_vee = 2.0;
                *stats = trial;
                log::info!(
                    "Calibration step accepted: quality={quality}, max_inc={maximum}, error={}",
                    trial.error
                );
                break;
            }
            if converged {
                break;
            }
        }
        converged
    }
}
