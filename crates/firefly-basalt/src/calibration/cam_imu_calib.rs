// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// Copyright (C) 2008-2010 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2009 Mathieu Gautier <mathieu.gautier@cea.fr>
// SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
// rotation_from_moments includes Eigen Geometry/Quaternion.h's matrix conversion.
//! Headless control flow from original `calibration/cam_imu_calib.cpp`.
use super::calibration_archive::{read_map, write_map};
use super::{
    aprilgrid::AprilGrid,
    calibration_helper::{CalibCornerMap, CalibHelper, CalibInitPoseMap, ProjectedCornerData},
};
use crate::io::dataset_io::DatasetIoFactory;
use crate::utils::sophus_utils::se3_inverse;
use crate::utils::sophus_utils::se3_matrix;
use crate::utils::sophus_utils::se3_product;
use crate::utils::sophus_utils::so3_inverse;
use crate::utils::sophus_utils::so3_log;
use crate::utils::sophus_utils::so3_product;
use crate::{
    io::dataset_io::{GyroData, VioDatasetPtr},
    optimization::{
        linearize::ReprojectionStats,
        spline_optimize::{Options, SplineOptimization},
    },
    utils::common_types::TimeCamId,
};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Isometry3, Matrix3, Quaternion, Translation3, UnitQuaternion, Vector3};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::{
    fs::File,
    io::{BufReader, BufWriter, Write},
    path::Path,
};

pub struct CamImuCalib {
    pub dataset: VioDatasetPtr,
    pub april_grid: AprilGrid,
    pub optimizer: SplineOptimization<5>,
    pub corners: CalibCornerMap,
    pub rejected_corners: CalibCornerMap,
    pub initial_poses: CalibInitPoseMap,
    pub reprojected_corners: HashMap<TimeCamId, ProjectedCornerData>,
    pub options: Options,
}

/// Original Pangolin `DataLog` columns, available to recording/plotting consumers.
#[derive(Default)]
pub struct CalibrationDiagnostics {
    pub imu: Vec<[f32; 13]>,
    pub poses: Vec<[f32; 8]>,
    pub mocap: Vec<[f32; 11]>,
}

fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("basalt.cam_imu_calib")
}

/// Original least-squares rotation initialization, including Eigen's
/// matrix-to-quaternion/AngleAxis projection rather than an SVD projection.
/// # Errors
/// Rejects missing gyro samples or a singular angular-velocity Gram matrix.
pub fn initialize_rotation(
    poses: &[(i64, Isometry3<f64>)],
    gyro: &[GyroData],
    initial_rotation: &UnitQuaternion<f64>,
) -> Result<UnitQuaternion<f64>> {
    if gyro.len() < 2 {
        return Err(invalid("At least two gyro samples are required"));
    }
    let mut gram = Matrix3::zeros();
    let mut cross = Matrix3::zeros();
    for pair in poses.windows(2) {
        let dt = (pair[1].0 - pair[0].0) as f64 * 1e-9;
        if dt <= 0. {
            return Err(invalid("Pose timestamps must increase"));
        }
        let velocity = initial_rotation
            * so3_log(&so3_product(
                &so3_inverse(&pair[0].1.rotation),
                &pair[1].1.rotation,
            ))
            / dt;
        // The original search deliberately starts at sample one, with first-win ties.
        let nearest = gyro[1..]
            .iter()
            .min_by_key(|sample| sample.timestamp_ns.abs_diff(pair[0].0))
            .ok_or_else(|| invalid("No gyro sample"))?;
        gram += velocity * velocity.transpose();
        cross += nearest.data * velocity.transpose();
    }
    rotation_from_moments(gram, cross)
}

fn rotation_from_moments(gram: Matrix3<f64>, cross: Matrix3<f64>) -> Result<UnitQuaternion<f64>> {
    let raw = cross
        * gram
            .try_inverse()
            .ok_or_else(|| invalid("Angular velocities do not span three dimensions"))?;
    let trace = raw.trace();
    let quaternion = if trace > 0. {
        let t = (trace + 1.).sqrt();
        let scale = 0.5 / t;
        Quaternion::new(
            0.5 * t,
            (raw[(2, 1)] - raw[(1, 2)]) * scale,
            (raw[(0, 2)] - raw[(2, 0)]) * scale,
            (raw[(1, 0)] - raw[(0, 1)]) * scale,
        )
    } else {
        let mut i = 0;
        if raw[(1, 1)] > raw[(0, 0)] {
            i = 1;
        }
        if raw[(2, 2)] > raw[(i, i)] {
            i = 2;
        }
        let j = (i + 1) % 3;
        let k = (j + 1) % 3;
        let t = (raw[(i, i)] - raw[(j, j)] - raw[(k, k)] + 1.).sqrt();
        let scale = 0.5 / t;
        let mut xyz = Vector3::zeros();
        xyz[i] = 0.5 * t;
        xyz[j] = (raw[(j, i)] + raw[(i, j)]) * scale;
        xyz[k] = (raw[(k, i)] + raw[(i, k)]) * scale;
        Quaternion::new((raw[(k, j)] - raw[(j, k)]) * scale, xyz.x, xyz.y, xyz.z)
    };
    if !quaternion.coords.iter().all(|value| value.is_finite()) || quaternion.norm_squared() == 0. {
        return Err(invalid("Invalid rotation initialization"));
    }
    Ok(UnitQuaternion::new_normalize(quaternion))
}

impl CamImuCalib {
    #[must_use]
    pub fn has_corners(&self) -> bool {
        !self.corners.is_empty()
    }

    /// Performs the original single-step interactive optimization callback.
    pub fn optimize(&mut self) {
        self.optimize_with_param(true, &mut BTreeMap::new());
    }

    /// Evaluates the original raw/spline IMU, pose and mocap plotting samples.
    /// # Errors
    /// Rejects mismatched IMU arrays or a poisoned dataset lock.
    pub fn recompute_data_log(&self) -> Result<CalibrationDiagnostics> {
        let mut output = CalibrationDiagnostics::default();
        let mut data = self
            .dataset
            .lock()
            .map_err(|_| invalid("Dataset lock poisoned"))?;
        let times = data.image_timestamps().clone();
        let Some(first) = data.accel_data().first() else {
            return Ok(output);
        };
        let start = first.timestamp_ns as f64 * 1e-9;
        if data.accel_data().len() != data.gyro_data().len() {
            return Err(invalid("IMU arrays must be paired"));
        }
        let initialized = self.optimizer.calib_initialized() && self.optimizer.initialized();
        for (accel, gyro) in data.accel_data().iter().zip(data.gyro_data()) {
            let mut a = Vector3::zeros();
            let mut g = Vector3::zeros();
            if initialized {
                let pose = self.optimizer.t_w_i(accel.timestamp_ns);
                a = self.optimizer.accel_bias().invert_calibration(
                    &(so3_inverse(&pose.rotation)
                        * (self.optimizer.trans_accel_world(accel.timestamp_ns)
                            + self.optimizer.g())),
                );
                g = self
                    .optimizer
                    .gyro_bias()
                    .invert_calibration(&self.optimizer.rot_vel_body(accel.timestamp_ns));
            }
            output.imu.push(
                [
                    accel.timestamp_ns as f64 * 1e-9 - start,
                    accel.data.x,
                    accel.data.y,
                    accel.data.z,
                    a.x,
                    a.y,
                    a.z,
                    gyro.data.x,
                    gyro.data.y,
                    gyro.data.z,
                    g.x,
                    g.y,
                    g.z,
                ]
                .map(|value| value as f32),
            );
        }
        for time in times {
            let spline = if self.optimizer.initialized() {
                self.optimizer.t_w_i(time)
            } else {
                Isometry3::identity()
            };
            let measured = self
                .initial_poses
                .get(&TimeCamId {
                    frame_id: time,
                    cam_id: 0,
                })
                .filter(|_| self.optimizer.calib_initialized())
                .map_or_else(Isometry3::identity, |pose| {
                    se3_product(&pose.t_a_c, &se3_inverse(self.optimizer.cam_t_i_c(0)))
                });
            let p = measured.translation.vector;
            let s = spline.translation.vector;
            output.poses.push(
                [
                    time as f64 * 1e-9 - start,
                    p.x,
                    p.y,
                    p.z,
                    s.x,
                    s.y,
                    s.z,
                    spline.rotation.angle_to(&measured.rotation).to_degrees(),
                ]
                .map(|value| value as f32),
            );
        }
        output.mocap = self.mocap_data_log(data.gt_timestamps(), data.gt_pose_data(), start);
        Ok(output)
    }

    fn mocap_data_log(
        &self,
        times: &[i64],
        poses: &[Isometry3<f64>],
        start: f64,
    ) -> Vec<[f32; 11]> {
        let mut output = Vec::new();
        for (index, (&time, pose)) in times.iter().zip(poses).enumerate() {
            let corrected = time
                + if self.optimizer.calib_initialized() {
                    self.optimizer.mocap_time_offset_ns()
                } else {
                    0
                };
            let (spline, measured) = if self.optimizer.calib_initialized() {
                if corrected < self.optimizer.min_time_ns()
                    || corrected > self.optimizer.max_time_ns()
                {
                    continue;
                }
                (
                    self.optimizer.t_w_i(corrected),
                    se3_product(
                        &se3_product(&self.optimizer.t_w_moc(), pose),
                        &self.optimizer.t_mark_i(),
                    ),
                )
            } else {
                (Isometry3::identity(), Isometry3::identity())
            };
            let velocity = if index > 0 {
                so3_log(&so3_product(
                    &so3_inverse(&poses[index - 1].rotation),
                    &pose.rotation,
                )) / ((time - times[index - 1]) as f64 * 1e-9)
            } else {
                Vector3::zeros()
            };
            let p = measured.translation.vector;
            let s = spline.translation.vector;
            output.push(
                [
                    corrected as f64 * 1e-9 - start,
                    p.x,
                    p.y,
                    p.z,
                    s.x,
                    s.y,
                    s.z,
                    spline.rotation.angle_to(&measured.rotation).to_degrees(),
                    velocity.x,
                    velocity.y,
                    velocity.z,
                ]
                .map(|value| value as f32),
            );
        }
        output
    }

    /// Loads the original dataset, Cereal preprocessing caches and camera calibration.
    /// # Errors
    /// Propagates dataset, archive and calibration errors.
    pub fn load_dataset(
        dataset_path: &Path,
        dataset_type: &str,
        grid_path: &Path,
        cache_path: &Path,
        cache_name: &str,
        skip_images: usize,
        imu_noise: [f64; 4],
    ) -> Result<Self> {
        let mut reader = DatasetIoFactory::get_dataset_io(dataset_type, false)?;
        reader.read(dataset_path)?;
        let dataset = reader
            .data()
            .ok_or_else(|| invalid("Dataset reader returned no data"))?;
        if skip_images > 1 {
            let mut data = dataset
                .lock()
                .map_err(|_| invalid("Dataset lock poisoned"))?;
            let times = data.image_timestamps();
            *times = times.iter().copied().step_by(skip_images).collect();
        }
        let mut optimizer = SplineOptimization::default();
        optimizer.load_calib(cache_path)?;
        let calibration = optimizer
            .calib
            .as_mut()
            .ok_or_else(|| invalid("Camera calibration is required"))?;
        calibration.accel_noise_std.fill(imu_noise[0]);
        calibration.gyro_noise_std.fill(imu_noise[1]);
        calibration.accel_bias_std.fill(imu_noise[2]);
        calibration.gyro_bias_std.fill(imu_noise[3]);
        optimizer.reset_mocap_calib();
        let mut result = Self::new(dataset, AprilGrid::new(grid_path)?, optimizer);
        let corner_path = cache_path.join(format!("{cache_name}_detected_corners.cereal"));
        match File::open(corner_path) {
            Ok(file) => {
                let mut input = BufReader::new(file);
                result.corners = read_map(&mut input)?;
                result.rejected_corners = read_map(&mut input)?;
            }
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => {
                return Err(Error::from(error).with_operation("basalt.cam_imu_calib.cache"));
            }
        }
        let pose_path = cache_path.join(format!("{cache_name}_init_poses.cereal"));
        match File::open(pose_path) {
            Ok(file) => result.initial_poses = read_map(&mut BufReader::new(file))?,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => {
                return Err(Error::from(error).with_operation("basalt.cam_imu_calib.cache"));
            }
        }
        Ok(result)
    }

    /// # Errors
    /// Propagates native Cereal archive and filesystem errors.
    pub fn save_preprocessing(&self, cache_path: &Path, cache_name: &str) -> Result<()> {
        let io_error = |error| Error::from(error).with_operation("basalt.cam_imu_calib.cache");
        let mut corners = BufWriter::new(
            File::create(cache_path.join(format!("{cache_name}_detected_corners.cereal")))
                .map_err(io_error)?,
        );
        write_map(&mut corners, &self.corners)?;
        write_map(&mut corners, &self.rejected_corners)?;
        corners.flush().map_err(io_error)?;
        let mut poses = BufWriter::new(
            File::create(cache_path.join(format!("{cache_name}_init_poses.cereal")))
                .map_err(io_error)?,
        );
        write_map(&mut poses, &self.initial_poses)?;
        poses.flush().map_err(io_error)
    }

    /// # Errors
    /// Propagates calibration output errors.
    pub fn save_calib(&self, directory: &Path) -> Result<()> {
        self.optimizer.save_calib(directory)
    }

    /// # Errors
    /// Propagates calibration output or dataset lock errors.
    pub fn save_mocap_calib(&mut self, directory: &Path) -> Result<()> {
        let offset = self
            .dataset
            .lock()
            .map_err(|_| invalid("Dataset lock poisoned"))?
            .mocap_to_imu_offset_ns();
        self.optimizer.save_mocap_calib(directory, offset)
    }

    #[must_use]
    pub fn new(
        dataset: VioDatasetPtr,
        april_grid: AprilGrid,
        optimizer: SplineOptimization<5>,
    ) -> Self {
        Self {
            dataset,
            april_grid,
            optimizer,
            corners: HashMap::new(),
            rejected_corners: HashMap::new(),
            initial_poses: HashMap::new(),
            reprojected_corners: HashMap::new(),
            options: Options {
                use_intrinsics: false,
                use_poses: false,
                use_april_corners: true,
                opt_cam_time_offset: false,
                opt_imu_scale: false,
                use_mocap: false,
                huber_thresh: 4.,
                stop_thresh: 1e-8,
                print_info: false,
            },
        }
    }

    /// # Errors
    /// Propagates dataset and target detection errors.
    pub fn detect_corners(&mut self) -> Result<()> {
        CalibHelper::detect_corners(
            &self.dataset,
            &self.april_grid,
            &mut self.corners,
            &mut self.rejected_corners,
        )
    }

    /// # Errors
    /// Requires detected corners and initialized camera intrinsics.
    pub fn init_cam_poses(&mut self) -> Result<()> {
        if self.corners.is_empty() {
            return Err(invalid("Detect corners before initializing camera poses"));
        }
        let calibration = self
            .optimizer
            .calib
            .as_ref()
            .ok_or_else(|| invalid("Calibrate camera intrinsics first"))?;
        CalibHelper::init_cam_poses(
            calibration,
            &self.april_grid.aprilgrid_corner_pos_3d,
            &self.corners,
            &mut self.initial_poses,
        );
        Ok(())
    }

    /// # Errors
    /// Requires calibrated cameras and observable paired camera/gyro rotations.
    pub fn init_cam_imu_transform(&mut self) -> Result<()> {
        let calibration = self
            .optimizer
            .calib
            .as_mut()
            .ok_or_else(|| invalid("Calibrate camera intrinsics first"))?;
        let first = calibration
            .t_i_c
            .first()
            .ok_or_else(|| invalid("No calibrated cameras"))?;
        let mut dataset = self
            .dataset
            .lock()
            .map_err(|_| invalid("Dataset lock poisoned"))?;
        let timestamps = dataset.image_timestamps().clone();
        // Only adjacent input frames with both camera poses contribute.
        let mut gram = Matrix3::zeros();
        let mut cross = Matrix3::zeros();
        let gyro = dataset.gyro_data();
        if gyro.len() < 2 {
            return Err(invalid("At least two gyro samples are required"));
        }
        for pair in timestamps.windows(2) {
            let a = self.initial_poses.get(&TimeCamId {
                frame_id: pair[0],
                cam_id: 0,
            });
            let b = self.initial_poses.get(&TimeCamId {
                frame_id: pair[1],
                cam_id: 0,
            });
            if let (Some(a), Some(b)) = (a, b) {
                let dt = (pair[1] - pair[0]) as f64 * 1e-9;
                let velocity = first.rotation
                    * so3_log(&so3_product(
                        &so3_inverse(&a.t_a_c.rotation),
                        &b.t_a_c.rotation,
                    ))
                    / dt;
                let nearest = gyro[1..]
                    .iter()
                    .min_by_key(|sample| sample.timestamp_ns.abs_diff(pair[0]))
                    .ok_or_else(|| invalid("No gyro sample"))?;
                gram += velocity * velocity.transpose();
                cross += nearest.data * velocity.transpose();
            }
        }
        let rotation = rotation_from_moments(gram, cross)?;
        let correction = Isometry3::from_parts(Translation3::identity(), rotation);
        for pose in &mut calibration.t_i_c {
            *pose = se3_product(&correction, pose);
        }
        Ok(())
    }

    /// # Errors
    /// Requires paired IMU arrays, 101 gyro samples and a gravity initialization sample.
    pub fn init_optimization(&mut self) -> Result<()> {
        let calibration = self
            .optimizer
            .calib
            .as_ref()
            .ok_or_else(|| invalid("Calibrate camera intrinsics first"))?;
        let camera = *calibration
            .t_i_c
            .first()
            .ok_or_else(|| invalid("No calibrated cameras"))?;
        let mut dataset = self
            .dataset
            .lock()
            .map_err(|_| invalid("Dataset lock poisoned"))?;
        let timestamps = dataset.image_timestamps().clone();
        let accel = dataset.accel_data();
        let gyro = dataset.gyro_data();
        if accel.len() != gyro.len() || gyro.len() < 101 {
            return Err(invalid(
                "Initialization requires paired IMU arrays and 101 gyro samples",
            ));
        }
        let poses: Vec<_> = timestamps
            .iter()
            .filter_map(|&time| {
                self.initial_poses
                    .get(&TimeCamId {
                        frame_id: time,
                        cam_id: 0,
                    })
                    .map(|pose| (time, se3_product(&pose.t_a_c, &se3_inverse(&camera))))
            })
            .collect();
        let gravity = poses
            .iter()
            .find_map(|(time, pose)| {
                accel
                    .iter()
                    .find(|sample| sample.timestamp_ns.abs_diff(*time) < 3_000_000)
                    .map(|sample| pose.rotation * sample.data)
            })
            .ok_or_else(|| invalid("No accelerometer sample within 3 ms of an initial pose"))?;
        self.optimizer
            .set_aprilgrid_corners_3d(&self.april_grid.aprilgrid_corner_pos_3d);
        for (ad, gd) in accel.iter().zip(gyro) {
            self.optimizer
                .add_accel_measurement(ad.timestamp_ns, ad.data);
            self.optimizer
                .add_gyro_measurement(gd.timestamp_ns, gd.data);
        }
        let invalid_times: HashSet<_> = self
            .corners
            .iter()
            .filter(|(_, corners)| corners.corner_ids.len() < 15)
            .map(|(id, _)| id.frame_id)
            .collect();
        for (id, corners) in &self.corners {
            if !invalid_times.contains(&id.frame_id) {
                self.optimizer.add_aprilgrid_measurement(
                    id.frame_id,
                    i32::try_from(id.cam_id).map_err(|_| invalid("Camera index exceeds i32"))?,
                    &corners.corners,
                    &corners.corner_ids,
                );
            }
        }
        for (&time, pose) in dataset.gt_timestamps().iter().zip(dataset.gt_pose_data()) {
            self.optimizer.add_mocap_measurement(time, *pose);
        }
        for (time, pose) in poses {
            self.optimizer.add_pose_measurement(time, pose);
        }
        let dt = gyro[..101]
            .windows(2)
            .map(|pair| (pair[1].timestamp_ns - pair[0].timestamp_ns) as f64 * 1e-9)
            .sum::<f64>()
            / 100.;
        self.optimizer
            .calib
            .as_mut()
            .ok_or_else(|| invalid("Camera calibration is required"))?
            .imu_update_rate = 1. / dt;
        self.optimizer.set_g(gravity);
        self.optimizer.init();
        drop(dataset);
        self.compute_projections()
    }

    /// # Errors
    /// Requires initialized spline, mocap calibration and observable rotations.
    pub fn init_mocap(&mut self) -> Result<()> {
        if !self.optimizer.initialized() {
            return Err(invalid("Initialize optimization first"));
        }
        let dataset = self
            .dataset
            .lock()
            .map_err(|_| invalid("Dataset lock poisoned"))?;
        let poses: Vec<_> = dataset
            .gt_timestamps()
            .iter()
            .copied()
            .zip(dataset.gt_pose_data().iter().copied())
            .collect();
        if poses.is_empty() {
            return Err(invalid("Dataset contains no mocap data"));
        }
        let mocap = self
            .optimizer
            .mocap_calib
            .as_mut()
            .ok_or_else(|| invalid("Initialize mocap calibration first"))?;
        let rotation = initialize_rotation(&poses, dataset.gyro_data(), &mocap.t_i_mark.rotation)?;
        mocap.t_i_mark = se3_product(
            &Isometry3::from_parts(Translation3::identity(), rotation),
            &mocap.t_i_mark,
        );
        let marker = mocap.t_i_mark;
        let (time, pose) = poses[poses.len() / 2];
        let world = se3_product(
            &se3_product(&self.optimizer.t_w_i(time), &marker),
            &se3_inverse(&pose),
        );
        self.optimizer.set_t_w_moc(&world);
        self.optimizer.mocap_initialized = true;
        Ok(())
    }

    /// # Errors
    /// Requires initialized calibration and an unpoisoned dataset.
    pub fn compute_projections(&mut self) -> Result<()> {
        self.reprojected_corners.clear();
        let calibration = self
            .optimizer
            .calib
            .as_ref()
            .ok_or_else(|| invalid("Calibrate camera intrinsics first"))?;
        let timestamps = self
            .dataset
            .lock()
            .map_err(|_| invalid("Dataset lock poisoned"))?
            .image_timestamps()
            .clone();
        for time in timestamps {
            let corrected = time + self.optimizer.cam_time_offset_ns();
            if corrected < self.optimizer.min_time_ns() || corrected >= self.optimizer.max_time_ns()
            {
                continue;
            }
            for (camera, intrinsics) in calibration.intrinsics.iter().enumerate() {
                let transform = se3_matrix(&se3_inverse(&se3_product(
                    &self.optimizer.t_w_i(corrected),
                    &calibration.t_i_c[camera],
                )));
                let mut projection = ProjectedCornerData::default();
                intrinsics.project_transformed(
                    &self.april_grid.aprilgrid_corner_pos_3d,
                    &transform,
                    &mut projection.corners_proj,
                    &mut projection.corners_proj_success,
                );
                self.reprojected_corners.insert(
                    TimeCamId {
                        frame_id: time,
                        cam_id: camera,
                    },
                    projection,
                );
            }
        }
        Ok(())
    }

    /// Performs one original LM controller step and returns its convergence flag.
    pub fn optimize_with_param(
        &mut self,
        print_info: bool,
        stats: &mut BTreeMap<String, f64>,
    ) -> bool {
        if !self.optimizer.calib_initialized() {
            return true;
        }
        let mut error = ReprojectionStats::default();
        let mut options = self.options;
        options.print_info = print_info;
        let converged = self.optimizer.optimize(options, &mut error);
        stats.clear();
        stats.insert("energy_error".into(), error.error);
        stats.insert("num_points".into(), error.num_points as f64);
        stats.insert(
            "mean_energy_error".into(),
            error.error / error.num_points as f64,
        );
        stats.insert("reprojection_error".into(), error.reprojection_error);
        stats.insert(
            "mean_reprojection_error".into(),
            error.reprojection_error / error.num_points as f64,
        );
        converged
    }
}
