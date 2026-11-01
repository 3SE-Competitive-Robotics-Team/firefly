// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are in ../../LICENSE.
//! Original `calibration/vignette.h` and `src/calibration/vignette.cpp`.
use super::aprilgrid::AprilGrid;
use crate::{io::dataset_io::VioDatasetPtr, utils::common_types::TimeCamId};
use firefly_basalt_headers::spline::{rd_spline::RdSpline, spline_common::Jacobian};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{SVector, Vector2, Vector3};
use std::{collections::BTreeMap, path::Path};

pub struct VignetteEstimator {
    optical_centers: Vec<Vector2<f64>>,
    resolutions: Vec<Vector2<i32>>,
    reprojected_vignette: BTreeMap<TimeCamId, Vec<Vector3<f64>>>,
    vign_size: usize,
    irradiance: Vec<f64>,
    vign_param: Vec<RdSpline<f64, 1, 4>>,
}
impl VignetteEstimator {
    pub const KNOT_SPACING: i64 = 10_000_000_000;
    pub const BORDER_SIZE: usize = 2;
    /// # Panics
    /// Requires one optical center and resolution per camera and an unpoisoned dataset.
    #[must_use]
    pub fn new(
        dataset: &VioDatasetPtr,
        optical_centers: &[Vector2<f64>],
        resolutions: &[Vector2<i32>],
        reprojected_vignette: &BTreeMap<TimeCamId, Vec<Vector3<f64>>>,
        grid: &AprilGrid,
    ) -> Self {
        let cameras = dataset.lock().expect("dataset poisoned").num_cams();
        assert_eq!(optical_centers.len(), cameras);
        assert_eq!(resolutions.len(), cameras);
        let vign_size = optical_centers
            .iter()
            .map(|center| (center.norm() * 1.1) as usize)
            .max()
            .unwrap_or(0);
        let mut vign_param = vec![RdSpline::new(Self::KNOT_SPACING, 0); cameras];
        for spline in &mut vign_param {
            while spline.max_time_ns() < vign_size as i64 * 700_000_000 {
                spline.knots_push_back(SVector::repeat(1.));
            }
            while spline.max_time_ns() < vign_size as i64 * 1_000_000_000 {
                spline.knots_push_back(SVector::repeat(0.01));
            }
        }
        Self {
            optical_centers: optical_centers.to_vec(),
            resolutions: resolutions.to_vec(),
            reprojected_vignette: reprojected_vignette.clone(),
            vign_size,
            irradiance: vec![1.; grid.aprilgrid_vignette_pos_3d.len()],
            vign_param,
        }
    }
    /// # Panics
    /// Requires every point array to match the target and radii within spline support.
    pub fn compute_error(&self, mut output: Option<&mut BTreeMap<TimeCamId, Vec<f64>>>) {
        if let Some(values) = &mut output {
            values.clear();
        }
        for (&id, points) in &self.reprojected_vignette {
            assert_eq!(points.len(), self.irradiance.len());
            let mut errors = vec![0.; points.len()];
            for (i, point) in points.iter().enumerate() {
                if point.z >= 0. {
                    let location = ((point.fixed_rows::<2>(0) - self.optical_centers[id.cam_id])
                        .norm()
                        * 1e9) as i64;
                    errors[i] = self.irradiance[i]
                        * self.vign_param[id.cam_id].evaluate::<0>(location, None)[0]
                        - point.z;
                }
            }
            if let Some(values) = &mut output {
                values.insert(id, errors);
            }
        }
    }
    /// # Panics
    /// Requires target-sized arrays and radii within spline support.
    pub fn opt_irradience(&mut self) {
        let mut values = vec![0.; self.irradiance.len()];
        let mut counts = vec![0usize; self.irradiance.len()];
        for (&id, points) in &self.reprojected_vignette {
            assert_eq!(points.len(), self.irradiance.len());
            for (i, point) in points.iter().enumerate() {
                if point.z >= 0. {
                    let location = ((point.fixed_rows::<2>(0) - self.optical_centers[id.cam_id])
                        .norm()
                        * 1e9) as i64;
                    values[i] +=
                        point.z / self.vign_param[id.cam_id].evaluate::<0>(location, None)[0];
                    counts[i] += 1;
                }
            }
        }
        for i in 0..self.irradiance.len() {
            if counts[i] > 0 {
                self.irradiance[i] = values[i] / counts[i] as f64;
            }
        }
    }
    /// # Panics
    /// Requires a camera, equal knot counts, target-sized arrays and supported radii.
    pub fn opt_vign(&mut self) {
        let knots = self.vign_param[0].knots().len();
        let mut values = vec![vec![0.; knots]; self.vign_param.len()];
        let mut counts = values.clone();
        for (&id, points) in &self.reprojected_vignette {
            assert_eq!(points.len(), self.irradiance.len());
            for (i, point) in points.iter().enumerate() {
                if point.z >= 0. {
                    let location = ((point.fixed_rows::<2>(0) - self.optical_centers[id.cam_id])
                        .norm()
                        * 1e9) as i64;
                    let mut jacobian = Jacobian::default();
                    let _ = self.vign_param[id.cam_id].evaluate::<0>(location, Some(&mut jacobian));
                    for (k, &weight) in jacobian.d_val_d_knot.iter().enumerate() {
                        values[id.cam_id][jacobian.start_idx + k] +=
                            weight * point.z / self.irradiance[i];
                        counts[id.cam_id][jacobian.start_idx + k] += weight;
                    }
                }
            }
        }
        let mut maximum = 0.0_f64;
        for camera in 0..self.vign_param.len() {
            for k in 0..knots {
                if counts[camera][k] > 0. {
                    let value = values[camera][k] / counts[camera][k];
                    maximum = maximum.max(value);
                    self.vign_param[camera].knot_mut(k)[0] = value;
                }
            }
        }
        let inverse = 1. / maximum;
        for (camera, spline) in self.vign_param.iter_mut().enumerate() {
            for (k, &count) in counts[camera].iter().enumerate() {
                if count > 0. {
                    spline.knot_mut(k)[0] *= inverse;
                }
            }
        }
    }
    pub fn optimize(&mut self) {
        self.compute_error(None);
        for _ in 0..10 {
            self.opt_irradience();
            self.compute_error(None);
            self.opt_vign();
            self.compute_error(None);
        }
    }
    /// # Panics
    /// Requires target-sized arrays and radii within spline support.
    #[must_use]
    pub fn compute_data_log(&self) -> Vec<Vec<f32>> {
        let mut counts = vec![vec![0.; self.vign_size]; self.vign_param.len()];
        for (&id, points) in &self.reprojected_vignette {
            assert_eq!(points.len(), self.irradiance.len());
            for point in points {
                if point.z >= 0. {
                    let location = (point.fixed_rows::<2>(0) - self.optical_centers[id.cam_id])
                        .norm() as usize;
                    if location < self.vign_size {
                        counts[id.cam_id][location] += 1.;
                    }
                }
            }
        }
        (0..self.vign_size)
            .map(|i| {
                self.vign_param
                    .iter()
                    .enumerate()
                    .flat_map(|(camera, spline)| {
                        [
                            spline.evaluate::<0>(i as i64 * 1_000_000_000, None)[0] as f32,
                            counts[camera][i] as f32,
                        ]
                    })
                    .collect()
            })
            .collect()
    }
    /// Writes the upstream 16-bit grayscale `vingette_<camera>.png` filenames.
    /// # Errors
    /// Returns PNG encoding or filesystem errors.
    pub fn save_vign_png(&self, path: &Path) -> Result<()> {
        for (camera, spline) in self.vign_param.iter().enumerate() {
            let resolution = self.resolutions[camera];
            let mut output = image::ImageBuffer::<image::Luma<u16>, Vec<u16>>::new(
                resolution.x as u32,
                resolution.y as u32,
            );
            for (x, y, pixel) in output.enumerate_pixels_mut() {
                let location = ((Vector2::new(f64::from(x), f64::from(y))
                    - self.optical_centers[camera])
                    .norm()
                    * 1e9) as i64;
                let value = spline.evaluate::<0>(location, None)[0];
                if value >= 0.5 {
                    pixel.0[0] = if value >= 1. {
                        u16::MAX
                    } else {
                        (value * f64::from(u16::MAX)) as u16
                    };
                }
            }
            output
                .save(path.join(format!("vingette_{camera}.png")))
                .map_err(|e| {
                    Error::new(ErrorKind::Internal, "Cannot save vignette PNG")
                        .with_operation("basalt.vignette")
                        .with_source(e)
                })?;
        }
        Ok(())
    }
    #[must_use]
    pub fn vign_param(&self) -> &[RdSpline<f64, 1, 4>] {
        &self.vign_param
    }
}
