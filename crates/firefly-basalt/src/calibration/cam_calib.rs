// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are indexed in docs/licenses/README.md.
//! Non-GUI control flow from original `calibration/cam_calib.h/.cpp`.
//! The original `show_gui=false` path joins corner detection before returning;
//! this implementation performs that work synchronously. Pangolin rendering,
//! widgets and overlay rasterization are not implemented by this controller.
use super::{
    calibration_archive::{read_map, write_map},
    calibration_helper::CalibHelper,
    vignette::VignetteEstimator,
};
use crate::{
    io::dataset_io::{DatasetIoFactory, VioDatasetPtr},
    optimization::{linearize::ReprojectionStats, poses_optimize::PosesOptimization},
};
use firefly_calibration::aprilgrid::AprilGrid;
use firefly_calibration::observation::{
    CalibCornerMap, CalibInitPoseMap, ProjectedCornerData, TimeCamId,
};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Isometry3, Vector2, Vector3, Vector4};
use std::{
    collections::{BTreeMap, HashSet},
    fs::File,
    io::{BufReader, BufWriter, Write},
    path::{Path, PathBuf},
};

#[derive(Clone, Debug, Default)]
pub struct AngleStatistics {
    pub polar_sum: Vec<Vec<f64>>,
    pub polar_count: Vec<Vec<usize>>,
    pub azimuth_sum: Vec<Vec<f64>>,
    pub azimuth_count: Vec<Vec<usize>>,
    pub polar_log: Vec<Vec<[f64; 2]>>,
    pub azimuth_log: Vec<Vec<[f64; 2]>>,
}
pub struct CamCalib {
    pub vio_dataset: Option<VioDatasetPtr>,
    pub calib_corners: CalibCornerMap,
    pub calib_corners_rejected: CalibCornerMap,
    pub calib_init_poses: CalibInitPoseMap,
    pub calib_opt: Option<PosesOptimization>,
    pub reprojected_corners: BTreeMap<TimeCamId, ProjectedCornerData>,
    pub reprojected_vignette: BTreeMap<TimeCamId, ProjectedCornerData>,
    pub reprojected_vignette_error: BTreeMap<TimeCamId, Vec<f64>>,
    pub angle_statistics: AngleStatistics,
    pub vign_data_log: Vec<Vec<f32>>,
    pub april_grid: AprilGrid,
    pub huber_thresh: f64,
    pub opt_intr: bool,
    pub stop_thresh: f64,
    dataset_path: PathBuf,
    dataset_type: String,
    cache_path: PathBuf,
    cache_dataset_name: String,
    skip_images: i32,
    cam_types: Vec<String>,
}
fn failure(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("basalt.cam_calib")
}
fn io_error(error: std::io::Error) -> Error {
    Error::from(error).with_operation("basalt.cam_calib.cache")
}
impl CamCalib {
    pub const MIN_CORNERS: usize = 15;
    /// # Errors
    /// Reports grid parsing and cache directory creation errors.
    pub fn new(
        dataset_path: &Path,
        dataset_type: &str,
        aprilgrid_path: &Path,
        cache_path: &Path,
        cache_dataset_name: &str,
        skip_images: i32,
        cam_types: Vec<String>,
    ) -> Result<Self> {
        let april_grid = AprilGrid::new(aprilgrid_path)?;
        if !cache_path.exists() {
            std::fs::create_dir(cache_path).map_err(io_error)?;
        }
        Ok(Self {
            vio_dataset: None,
            calib_corners: CalibCornerMap::new(),
            calib_corners_rejected: CalibCornerMap::new(),
            calib_init_poses: CalibInitPoseMap::new(),
            calib_opt: None,
            reprojected_corners: BTreeMap::new(),
            reprojected_vignette: BTreeMap::new(),
            reprojected_vignette_error: BTreeMap::new(),
            angle_statistics: AngleStatistics::default(),
            vign_data_log: Vec::new(),
            april_grid,
            huber_thresh: 4.,
            opt_intr: true,
            stop_thresh: 1e-8,
            dataset_path: dataset_path.to_owned(),
            dataset_type: dataset_type.to_owned(),
            cache_path: cache_path.to_owned(),
            cache_dataset_name: cache_dataset_name.to_owned(),
            skip_images,
            cam_types,
        })
    }
    fn dataset(&self) -> Result<VioDatasetPtr> {
        self.vio_dataset
            .clone()
            .ok_or_else(|| failure("Dataset is not loaded"))
    }
    fn cache_file(&self, suffix: &str) -> PathBuf {
        self.cache_path
            .join(format!("{}{suffix}", self.cache_dataset_name))
    }
    #[must_use]
    pub fn has_corners(&self) -> bool {
        !self.calib_corners.is_empty()
    }
    pub fn set_opt_intrinsics(&mut self, opt: bool) {
        self.opt_intr = opt;
    }
    /// # Errors
    /// Reports dataset and native cache archive errors.
    pub fn load_dataset(&mut self) -> Result<()> {
        let mut reader = DatasetIoFactory::get_dataset_io(&self.dataset_type, false)?;
        reader.read(&self.dataset_path)?;
        self.vio_dataset = reader.data();
        if self.skip_images > 1 {
            let dataset = self.dataset()?;
            let mut data = dataset.lock().map_err(|_| failure("Dataset poisoned"))?;
            let times = data.image_timestamps();
            *times = times
                .iter()
                .step_by(self.skip_images as usize)
                .copied()
                .collect();
        }
        let path = self.cache_file("_detected_corners.cereal");
        if path.exists() {
            let mut input = BufReader::new(File::open(path).map_err(io_error)?);
            self.calib_corners = read_map(&mut input)?;
            self.calib_corners_rejected = read_map(&mut input)?;
        }
        let path = self.cache_file("_init_poses.cereal");
        if path.exists() {
            self.calib_init_poses =
                read_map(&mut BufReader::new(File::open(path).map_err(io_error)?))?;
        }
        self.calib_opt
            .get_or_insert_with(PosesOptimization::new)
            .load_calib(&self.cache_path)?;
        self.reprojected_corners.clear();
        self.reprojected_vignette.clear();
        Ok(())
    }
    /// # Errors
    /// Reports detector, dataset and cache write errors.
    pub fn detect_corners(&mut self) -> Result<()> {
        CalibHelper::detect_corners(
            &self.dataset()?,
            &self.april_grid,
            &mut self.calib_corners,
            &mut self.calib_corners_rejected,
        )?;
        let mut output = BufWriter::new(
            File::create(self.cache_file("_detected_corners.cereal")).map_err(io_error)?,
        );
        write_map(&mut output, &self.calib_corners)?;
        write_map(&mut output, &self.calib_corners_rejected)?;
        output.flush().map_err(io_error)
    }
    /// # Errors
    /// Reports missing images, invalid camera counts and initialization backend errors.
    pub fn init_cam_intrinsics(&mut self) -> Result<()> {
        if !self.has_corners() {
            return Ok(());
        }
        let dataset = self.dataset()?;
        let mut data = dataset.lock().map_err(|_| failure("Dataset poisoned"))?;
        let count = data.num_cams();
        let timestamps = data.image_timestamps().clone();
        if count != self.cam_types.len() {
            return Err(failure("Expected one camera type per camera"));
        }
        let opt = self.calib_opt.get_or_insert_with(PosesOptimization::new);
        opt.reset_calib(count, &self.cam_types);
        let increment = if timestamps.len() > 100 { 3 } else { 1 };
        let mut initialized = vec![false; count];
        for camera in 0..count {
            for &timestamp in timestamps.iter().step_by(increment) {
                let images = data.image_data(timestamp)?;
                if let Some(corners) = self.calib_corners.get(&TimeCamId::new(timestamp, camera))
                    && let Some(image) = &images[camera].img
                {
                    let mut intrinsics = Vector4::zeros();
                    if CalibHelper::initialize_intrinsics(
                        &corners.corners,
                        &corners.corner_ids,
                        &self.april_grid,
                        image.w as i32,
                        image.h as i32,
                        &mut intrinsics,
                    ) {
                        initialized[camera] = true;
                        opt.calib
                            .as_mut()
                            .ok_or_else(|| failure("Calibration missing"))?
                            .intrinsics[camera]
                            .set_from_init(intrinsics);
                        break;
                    }
                }
            }
        }
        for camera in 0..count {
            if initialized[camera] {
                continue;
            }
            let mut views = Vec::new();
            let (mut width, mut height) = (0, 0);
            for &timestamp in timestamps.iter().step_by(increment) {
                let images = data.image_data(timestamp)?;
                if let Some(corners) = self.calib_corners.get(&TimeCamId::new(timestamp, camera))
                    && corners.corners.len() > 8
                {
                    views.push(corners.clone());
                }
                if let Some(image) = &images[camera].img {
                    width = image.w as i32;
                    height = image.h as i32;
                }
            }
            if width <= 0 || height <= 0 {
                return Err(failure("No image for pinhole initialization"));
            }
            let mut intrinsics = Vector4::zeros();
            if CalibHelper::initialize_intrinsics_pinhole(
                &views,
                &self.april_grid,
                width,
                height,
                &mut intrinsics,
            )? {
                initialized[camera] = true;
                opt.calib
                    .as_mut()
                    .ok_or_else(|| failure("Calibration missing"))?
                    .intrinsics[camera]
                    .set_from_init(intrinsics);
            }
        }
        // Original starts at index one. Exhaustion is reported instead of its
        // unchecked out-of-range dereference when no synchronized frame exists.
        let mut resolution = None;
        for &timestamp in timestamps.iter().skip(1) {
            let images = data.image_data(timestamp)?;
            if images.iter().all(|i| i.img.is_some()) {
                resolution = Some(
                    images
                        .iter()
                        .filter_map(|i| {
                            i.img
                                .as_ref()
                                .map(|image| Vector2::new(image.w as i32, image.h as i32))
                        })
                        .collect::<Vec<_>>(),
                );
                break;
            }
        }
        opt.set_resolution(
            &resolution.ok_or_else(|| failure("No complete image frame after index zero"))?,
        );
        Ok(())
    }
    /// # Errors
    /// Reports cache write failures.
    pub fn init_cam_poses(&mut self) -> Result<()> {
        if !self.has_corners() {
            return Ok(());
        }
        let Some(calibration) = self.calib_opt.as_ref().and_then(|o| o.calib.as_ref()) else {
            return Ok(());
        };
        CalibHelper::init_cam_poses(
            calibration,
            &self.april_grid.aprilgrid_corner_pos_3d,
            &self.calib_corners,
            &mut self.calib_init_poses,
        );
        let mut output =
            BufWriter::new(File::create(self.cache_file("_init_poses.cereal")).map_err(io_error)?);
        write_map(&mut output, &self.calib_init_poses)?;
        output.flush().map_err(io_error)
    }
    /// Original greedy path through the camera overlap graph (not a spanning tree).
    /// # Errors
    /// Reports missing or poisoned dataset state.
    pub fn init_cam_extrinsics(&mut self) -> Result<()> {
        if self.calib_init_poses.is_empty() {
            return Ok(());
        }
        let dataset = self.dataset()?;
        let mut data = dataset.lock().map_err(|_| failure("Dataset poisoned"))?;
        let count = data.num_cams();
        let times = data.image_timestamps().clone();
        drop(data);
        let Some(calibration) = self.calib_opt.as_mut().and_then(|o| o.calib.as_mut()) else {
            return Ok(());
        };
        if count == 0 {
            return Err(failure("No cameras"));
        }
        let mut graph: BTreeMap<(usize, usize), (usize, i64)> = BTreeMap::new();
        for timestamp in times {
            for left in 0..count {
                let Some(a) = self
                    .calib_init_poses
                    .get(&TimeCamId::new(timestamp, left))
                    .filter(|p| p.num_inliers >= Self::MIN_CORNERS)
                else {
                    continue;
                };
                for right in left + 1..count {
                    let Some(b) = self
                        .calib_init_poses
                        .get(&TimeCamId::new(timestamp, right))
                        .filter(|p| p.num_inliers >= Self::MIN_CORNERS)
                    else {
                        continue;
                    };
                    let entry = graph.entry((left, right)).or_default();
                    let weight = a.num_inliers.min(b.num_inliers);
                    if entry.0 < weight {
                        *entry = (weight, timestamp);
                    }
                }
            }
        }
        let mut initialized = vec![false; count];
        initialized[0] = true;
        let mut last = 0;
        calibration.t_i_c[0] = Isometry3::identity();
        for _ in 0..count - 1 {
            let mut best = None;
            for (camera, &done) in initialized.iter().enumerate() {
                if done {
                    continue;
                }
                if let Some(&(weight, time)) = graph.get(&(camera.min(last), camera.max(last)))
                    && best.is_none_or(|(_, _, score)| score < weight)
                {
                    best = Some((camera, time, weight));
                }
            }
            if let Some((camera, time, _)) = best {
                calibration.t_i_c[camera] = calibration.t_i_c[last]
                    * self.calib_init_poses[&TimeCamId::new(time, last)]
                        .t_a_c
                        .inverse()
                    * self.calib_init_poses[&TimeCamId::new(time, camera)].t_a_c;
                last = camera;
                initialized[last] = true;
            }
        }
        Ok(())
    }
    /// # Errors
    /// Reports missing or poisoned dataset state.
    pub fn init_optimization(&mut self) -> Result<()> {
        if self.calib_opt.is_none() || self.calib_init_poses.is_empty() {
            return Ok(());
        }
        let dataset = self.dataset()?;
        let times = dataset
            .lock()
            .map_err(|_| failure("Dataset poisoned"))?
            .image_timestamps()
            .clone();
        let opt = self
            .calib_opt
            .as_mut()
            .ok_or_else(|| failure("Optimizer missing"))?;
        let extrinsics = opt
            .calib
            .as_ref()
            .ok_or_else(|| failure("Calibration missing"))?
            .t_i_c
            .clone();
        opt.set_aprilgrid_corners_3d(&self.april_grid.aprilgrid_corner_pos_3d);
        let mut invalid: HashSet<_> = self
            .calib_corners
            .iter()
            .filter(|(_, v)| v.corner_ids.len() < Self::MIN_CORNERS)
            .map(|(&id, _)| id)
            .collect();
        for time in times {
            let mut best = None;
            for camera in 0..extrinsics.len() {
                if let Some(pose) = self.calib_init_poses.get(&TimeCamId::new(time, camera))
                    && best.is_none_or(|(_, score)| score < pose.num_inliers)
                {
                    best = Some((camera, pose.num_inliers));
                }
            }
            if let Some((camera, score)) = best
                && score >= Self::MIN_CORNERS
            {
                opt.add_pose_measurement(
                    time,
                    self.calib_init_poses[&TimeCamId::new(time, camera)].t_a_c
                        * extrinsics[camera].inverse(),
                );
            } else {
                for camera in 0..extrinsics.len() {
                    invalid.insert(TimeCamId::new(time, camera));
                }
            }
        }
        for (&id, corners) in &self.calib_corners {
            if !invalid.contains(&id) {
                opt.add_aprilgrid_measurement(
                    id.frame_id,
                    id.cam_id as i32,
                    &corners.corners,
                    &corners.corner_ids,
                );
            }
        }
        opt.init();
        self.compute_projections()
    }
    /// # Errors
    /// Reports dataset poisoning or invalid corner IDs/angle bins.
    pub fn compute_projections(&mut self) -> Result<()> {
        self.reprojected_corners.clear();
        self.reprojected_vignette.clear();
        let (Some(opt), Some(dataset)) = (&self.calib_opt, &self.vio_dataset) else {
            return Ok(());
        };
        let calibration = opt
            .calib
            .as_ref()
            .ok_or_else(|| failure("Calibration missing"))?;
        let times = dataset
            .lock()
            .map_err(|_| failure("Dataset poisoned"))?
            .image_timestamps()
            .clone();
        let count = calibration.intrinsics.len();
        let mut stats = AngleStatistics {
            polar_sum: vec![vec![0.; 90]; count],
            polar_count: vec![vec![0; 90]; count],
            azimuth_sum: vec![vec![0.; 180]; count],
            azimuth_count: vec![vec![0; 180]; count],
            polar_log: vec![Vec::new(); count],
            azimuth_log: vec![Vec::new(); count],
        };
        for time in times {
            for camera in 0..count {
                let id = TimeCamId::new(time, camera);
                let transform = (opt.get_t_w_i(time) * calibration.t_i_c[camera])
                    .inverse()
                    .to_homogeneous();
                let mut corners = ProjectedCornerData::default();
                let mut vignette = ProjectedCornerData::default();
                let mut angles = Vec::new();
                calibration.intrinsics[camera].project_with_angles(
                    &self.april_grid.aprilgrid_corner_pos_3d,
                    &transform,
                    &mut corners.corners_proj,
                    &mut corners.corners_proj_success,
                    &mut angles,
                );
                calibration.intrinsics[camera].project_transformed(
                    &self.april_grid.aprilgrid_vignette_pos_3d,
                    &transform,
                    &mut vignette.corners_proj,
                    &mut vignette.corners_proj_success,
                );
                if let Some(measurement) = self.calib_corners.get(&id) {
                    for (&index, point) in measurement.corner_ids.iter().zip(&measurement.corners) {
                        let index =
                            usize::try_from(index).map_err(|_| failure("Negative corner ID"))?;
                        if *corners
                            .corners_proj_success
                            .get(index)
                            .ok_or_else(|| failure("Corner ID exceeds target"))?
                        {
                            let error = (point - corners.corners_proj[index]).norm();
                            let polar =
                                (180. * angles[index].x / (std::f64::consts::PI * 2.)) as usize;
                            let azimuth = (90.
                                + 180. * angles[index].y / (std::f64::consts::PI * 2.))
                                as usize;
                            if polar >= 90 || azimuth >= 180 {
                                return Err(failure("Projection angle outside histogram domain"));
                            }
                            stats.polar_sum[camera][polar] += error;
                            stats.polar_count[camera][polar] += 1;
                            stats.azimuth_sum[camera][azimuth] += error;
                            stats.azimuth_count[camera][azimuth] += 1;
                        }
                    }
                }
                self.reprojected_corners.insert(id, corners);
                self.reprojected_vignette.insert(id, vignette);
            }
        }
        for camera in 0..count {
            for bin in 0..90 {
                let n = stats.polar_count[camera][bin];
                if n > 3 {
                    stats.polar_log[camera].push([
                        2. * bin as f64 + 1.,
                        stats.polar_sum[camera][bin] / n as f64,
                    ]);
                }
            }
            for bin in 0..180 {
                let n = stats.azimuth_count[camera][bin];
                if n > 3 {
                    stats.azimuth_log[camera].push([
                        2. * bin as f64 + 1. - 180.,
                        stats.azimuth_sum[camera][bin] / n as f64,
                    ]);
                }
            }
        }
        self.angle_statistics = stats;
        Ok(())
    }
    /// One original optimization step; caller drives any convergence loop.
    /// # Errors
    /// Reports no additional errors; kept consistent with the controller API.
    pub fn optimize_with_param(
        &mut self,
        print_info: bool,
        stats: Option<&mut BTreeMap<String, f64>>,
    ) -> Result<bool> {
        if self.calib_init_poses.is_empty() {
            return Ok(true);
        }
        let Some(opt) = self.calib_opt.as_mut().filter(|o| o.calib_initialized()) else {
            return Ok(true);
        };
        let mut result = ReprojectionStats::default();
        let converged = opt.optimize(
            self.opt_intr,
            self.huber_thresh,
            self.stop_thresh,
            &mut result,
        );
        if let Some(stats) = stats {
            stats.clear();
            stats.insert("energy_error".into(), result.error);
            stats.insert("num_points".into(), result.num_points as f64);
            stats.insert(
                "mean_energy_error".into(),
                result.error / result.num_points as f64,
            );
            stats.insert("reprojection_error".into(), result.reprojection_error);
            stats.insert(
                "mean_reprojection_error".into(),
                result.reprojection_error / result.num_points as f64,
            );
        }
        if print_info {
            log::info!(
                "Calibration error {} over {} points; converged={converged}",
                result.error,
                result.num_points
            );
        }
        Ok(converged)
    }
    /// # Errors
    /// Propagates optimizer errors.
    pub fn optimize(&mut self) -> Result<bool> {
        self.optimize_with_param(true, None)
    }
    /// # Errors
    /// Reports calibration archive I/O errors.
    pub fn save_calib(&self) -> Result<()> {
        if let Some(opt) = &self.calib_opt {
            opt.save_calib(&self.cache_path)?;
        }
        Ok(())
    }
    /// # Errors
    /// Reports dataset sampling and vignette PNG output failures.
    pub fn compute_vign(&mut self) -> Result<()> {
        let dataset = self.dataset()?;
        let mut data = dataset.lock().map_err(|_| failure("Dataset poisoned"))?;
        let calibration = self
            .calib_opt
            .as_ref()
            .and_then(|o| o.calib.as_ref())
            .ok_or_else(|| failure("Calibration missing"))?;
        let centers: Vec<_> = (0..calibration.intrinsics.len())
            .map(|i| {
                calibration.intrinsics[i]
                    .param()
                    .fixed_rows::<2>(2)
                    .into_owned()
            })
            .collect();
        let mut samples = BTreeMap::new();
        for time in data.image_timestamps().clone() {
            let images = data.image_data(time)?;
            for (camera, image) in images.iter().enumerate().take(calibration.intrinsics.len()) {
                let id = TimeCamId::new(time, camera);
                if let Some(projection) = self.reprojected_vignette.get(&id)
                    && let Some(pixels) = &image.img
                {
                    let view = pixels.view();
                    let values = projection
                        .corners_proj
                        .iter()
                        .zip(&projection.corners_proj_success)
                        .map(|(point, &valid)| {
                            let value = if view.in_bounds(point, 1.) && valid {
                                let mut value = view.interp::<f64>(point) / f64::from(u16::MAX);
                                if image.exposure > 0. {
                                    value *= 0.001 / image.exposure;
                                }
                                value
                            } else {
                                -1.
                            };
                            Vector3::new(point.x, point.y, value)
                        })
                        .collect();
                    samples.insert(id, values);
                }
            }
        }
        drop(data);
        let mut estimator = VignetteEstimator::new(
            &dataset,
            &centers,
            &calibration.resolution,
            &samples,
            &self.april_grid,
        );
        estimator.optimize();
        estimator.compute_error(Some(&mut self.reprojected_vignette_error));
        self.vign_data_log = estimator.compute_data_log();
        estimator.save_vign_png(&self.cache_path)?;
        self.calib_opt
            .as_mut()
            .ok_or_else(|| failure("Optimizer missing"))?
            .set_vignette(estimator.vign_param());
        Ok(())
    }
}
