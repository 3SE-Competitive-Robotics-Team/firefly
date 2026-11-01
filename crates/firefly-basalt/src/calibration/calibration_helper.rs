// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are in ../../LICENSE.
//! Original `calibration/calibration_helper.h` and `src/calibration/calibraiton_helper.cpp`.
use crate::{io::dataset_io::VioDatasetPtr, utils::ldlt::Ldlt};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::camera::unified_camera::UnifiedCamera;
use firefly_calibration::image::image::ManagedImage;
use firefly_calibration::observation::{CalibCornerData, CalibCornerMap, CalibInitPoseMap};
use firefly_calibration::{
    aprilgrid::AprilGrid,
    initialization,
    observation::{self, AprilTagDetector, CalibrationImageSource},
};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{DMatrix, DVector, SVector, Vector2, Vector4};
use std::collections::HashMap;

pub struct CalibHelper;

struct BasaltImageSource<'a>(&'a VioDatasetPtr);

impl CalibrationImageSource for BasaltImageSource<'_> {
    fn image_timestamps(&self) -> Result<Vec<i64>> {
        Ok(self
            .0
            .lock()
            .expect("dataset poisoned")
            .image_timestamps()
            .clone())
    }

    fn images_at(&self, timestamp: i64) -> Result<Vec<Option<std::sync::Arc<ManagedImage<u16>>>>> {
        Ok(self
            .0
            .lock()
            .expect("dataset poisoned")
            .image_data(timestamp)?
            .into_iter()
            .map(|data| data.img)
            .collect())
    }
}

impl CalibHelper {
    /// Detects the original tag36h11 family with a two-cell black border.
    /// # Errors
    /// Returns dataset, detector or subpixel refinement errors.
    pub fn detect_corners(
        dataset: &VioDatasetPtr,
        grid: &AprilGrid,
        accepted: &mut CalibCornerMap,
        rejected: &mut CalibCornerMap,
    ) -> Result<()> {
        Self::detect_corners_with_factory(
            dataset,
            grid,
            |count| Box::new(crate::utils::apriltag::ApriltagDetector::new(count)),
            accepted,
            rejected,
        )
    }
    /// Runs detection with a caller-supplied real detector for the target tag count.
    /// # Errors
    /// Returns dataset or detector errors.
    /// # Panics
    /// Requires an unpoisoned dataset.
    pub fn detect_corners_with_factory<F>(
        dataset: &VioDatasetPtr,
        grid: &AprilGrid,
        factory: F,
        accepted: &mut CalibCornerMap,
        rejected: &mut CalibCornerMap,
    ) -> Result<()>
    where
        F: Fn(usize) -> Box<dyn AprilTagDetector> + Sync,
    {
        let (detected, rejected_corners) = observation::detect_corners(
            &BasaltImageSource(dataset),
            grid.tag_cols() * grid.tag_rows(),
            factory,
        )?;
        *accepted = detected;
        *rejected = rejected_corners;
        Ok(())
    }
    /// # Panics
    /// Requires valid camera and corner indices.
    pub fn init_cam_poses(
        calibration: &Calibration<f64>,
        target: &[Vector4<f64>],
        corners: &CalibCornerMap,
        poses: &mut CalibInitPoseMap,
    ) {
        *poses = initialization::init_cam_poses(calibration, target, corners);
    }
    /// # Panics
    /// Requires matching corner/ID arrays and valid target indices.
    pub fn initialize_intrinsics(
        corners: &[Vector2<f64>],
        ids: &[i32],
        grid: &AprilGrid,
        columns: i32,
        rows: i32,
        output: &mut Vector4<f64>,
    ) -> bool {
        let by_id: HashMap<_, _> = ids.iter().copied().zip(corners.iter().copied()).collect();
        let center = Vector2::new(f64::from(columns) / 2. - 0.5, f64::from(rows) / 2. - 0.5);
        let mut best_error = f64::MAX;
        let mut best_gamma = 0.;
        let mut success = false;
        for offset in 0..2 {
            for row in 0..grid.tag_rows() {
                let mut points = Vec::new();
                for column in 0..grid.tag_cols() {
                    for i in 0..2 {
                        let id = ((row * grid.tag_cols() + column) * 4 + i + offset * 2) as i32;
                        if let Some(point) = by_id.get(&id) {
                            let centered = point - center;
                            points.push(Vector4::new(
                                centered.x,
                                centered.y,
                                0.5,
                                -0.5 * centered.norm_squared(),
                            ));
                        }
                    }
                }
                if points.len() <= 8 {
                    continue;
                }
                let matrix = DMatrix::from_fn(points.len(), 4, |r, c| points[r][c]);
                let svd = matrix.svd(false, true);
                let v = svd.v_t.expect("V requested");
                let coefficient = v.row(3);
                let t = coefficient[0].powi(2)
                    + coefficient[1].powi(2)
                    + coefficient[2] * coefficient[3];
                if t < 0. {
                    continue;
                }
                let d = (1. / t).sqrt();
                let nx = coefficient[0] * d;
                let ny = coefficient[1] * d;
                if (nx * nx + ny * ny).sqrt() > 0.95 {
                    continue;
                }
                let nz = (1. - nx * nx - ny * ny).sqrt();
                let gamma = (coefficient[2] * d / nz).abs();
                let camera = GenericCamera::Unified(UnifiedCamera::new(SVector::<f64, 5>::new(
                    0.5 * gamma,
                    0.5 * gamma,
                    center.x,
                    center.y,
                    0.5,
                )));
                let Some((pose, count)) = initialization::estimate_transformation(
                    &camera,
                    corners,
                    ids,
                    &grid.aprilgrid_corner_pos_3d,
                ) else {
                    continue;
                };
                if count <= 8 {
                    continue;
                }
                let (count, error) = initialization::reprojection_error(
                    &camera,
                    corners,
                    ids,
                    &grid.aprilgrid_corner_pos_3d,
                    &pose,
                );
                if count > 8 && error / (count as f64) < best_error {
                    best_error = error / count as f64;
                    best_gamma = gamma;
                    success = true;
                }
            }
        }
        if success {
            *output = Vector4::new(0.5 * best_gamma, 0.5 * best_gamma, center.x, center.y);
        }
        success
    }
    /// # Errors
    /// Returns invalid point data or homography estimation errors.
    /// # Panics
    /// Requires valid corner IDs and matching corner/ID arrays.
    pub fn initialize_intrinsics_pinhole(
        views: &[CalibCornerData],
        grid: &AprilGrid,
        columns: i32,
        rows: i32,
        output: &mut Vector4<f64>,
    ) -> Result<bool> {
        use purecv::{
            calib3d::homography::{HomographyMethod, find_homography},
            core::types::Point2f,
        };
        let center = Vector2::new(f64::from(columns) / 2. - 0.5, f64::from(rows) / 2. - 0.5);
        let mut matrix = DMatrix::zeros(views.len() * 2, 2);
        let mut rhs = DVector::zeros(views.len() * 2);
        for (i, view) in views.iter().enumerate() {
            let source: Vec<_> = view
                .corner_ids
                .iter()
                .map(|&id| {
                    let point = grid.aprilgrid_corner_pos_3d[id as usize];
                    Point2f::new(point.x as f32, point.y as f32)
                })
                .collect();
            let destination: Vec<_> = view
                .corners
                .iter()
                .map(|point| Point2f::new(point.x as f32, point.y as f32))
                .collect();
            let homography =
                find_homography(&source, &destination, HomographyMethod::None, 3., None).map_err(
                    |e| {
                        Error::new(ErrorKind::InvalidArgument, e.to_string())
                            .with_operation("basalt.calibration.homography")
                    },
                )?;
            let mut h = nalgebra::Matrix3::from_fn(|r, c| {
                *homography.get(r, c, 0).expect("3x3 homography")
            });
            for c in 0..3 {
                h[(0, c)] -= h[(2, c)] * center.x;
                h[(1, c)] -= h[(2, c)] * center.y;
            }
            let a = h.column(0).into_owned();
            let b = h.column(1).into_owned();
            let diagonal1 = ((a + b) * 0.5).normalize();
            let diagonal2 = ((a - b) * 0.5).normalize();
            let a = a.normalize();
            let b = b.normalize();
            matrix[(2 * i, 0)] = a.x * b.x;
            matrix[(2 * i, 1)] = a.y * b.y;
            matrix[(2 * i + 1, 0)] = diagonal1.x * diagonal2.x;
            matrix[(2 * i + 1, 1)] = diagonal1.y * diagonal2.y;
            rhs[2 * i] = -a.z * b.z;
            rhs[2 * i + 1] = -diagonal1.z * diagonal2.z;
        }
        let focal =
            Ldlt::new(matrix.transpose() * &matrix).solve_vector(&(matrix.transpose() * rhs));
        *output = Vector4::new(
            (1. / focal[0]).abs().sqrt(),
            (1. / focal[1]).abs().sqrt(),
            center.x,
            center.y,
        );
        Ok(true)
    }
}
