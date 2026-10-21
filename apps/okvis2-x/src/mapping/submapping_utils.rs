/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENSE file for details
 */
//! Occupancy queries, point selection, VTK bounds and batched `LiDAR` propagation.
use crate::{
    ceres::imu_error::{IntegrationError, NoiseMode, Preintegration, seconds},
    frames::frame_typedefs::SpeedAndBias,
    kinematics::transformation::{Transformation, delta_q},
    sensors::measurements::ImuMeasurementDeque,
};
use jiff::Timestamp;
use nalgebra::{SMatrix, Vector3};
use rand::seq::IteratorRandom;
use std::{
    io::{self, Write},
    path::Path,
};

/// Safe/unchecked interpolation policy corresponding to `se::Safe`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Safe {
    On,
    Off,
}
/// Backend operations must select `field.occupancy` of valid data, rather than
/// the map's alternative field accessor. The returned scale belongs to that query.
pub trait MeanOccupancyMap {
    fn interp_mean_occupancy(
        &self,
        point: &Vector3<f32>,
        desired_scale: i32,
        safe: Safe,
    ) -> Option<(f32, i32)>;
    fn grad_mean_occupancy(
        &self,
        point: &Vector3<f32>,
        desired_scale: i32,
        safe: Safe,
    ) -> Option<(Vector3<f32>, i32)>;
}
pub fn interp_field_mean_occup(
    map: &impl MeanOccupancyMap,
    point: &Vector3<f32>,
    desired_scale: i32,
    safe: Safe,
    returned_scale: Option<&mut i32>,
) -> Option<f32> {
    let (value, scale) = map.interp_mean_occupancy(point, desired_scale, safe)?;
    if let Some(out) = returned_scale {
        *out = scale;
    }
    Some(value)
}
pub fn grad_field_mean_occup(
    map: &impl MeanOccupancyMap,
    point: &Vector3<f32>,
    desired_scale: i32,
    safe: Safe,
    returned_scale: Option<&mut i32>,
) -> Option<Vector3<f32>> {
    let (value, scale) = map.grad_mean_occupancy(point, desired_scale, safe)?;
    if let Some(out) = returned_scale {
        *out = scale;
    }
    Some(value)
}
pub trait SubmapBounds {
    fn map_aabb(&self) -> (Vector3<f32>, Vector3<f32>);
    fn t_wk(&self) -> SMatrix<f32, 3, 4>;
}
/// # Errors
/// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
/// or a failed storage, integration, alignment, callback, worker or file operation.
pub fn save_bounding_box_vtk(submap: &impl SubmapBounds, filename: &Path) -> io::Result<()> {
    let (lo, hi) = submap.map_aabb();
    let transform = submap.t_wk();
    let corners = [
        Vector3::new(lo.x, lo.y, lo.z),
        Vector3::new(hi.x, lo.y, lo.z),
        Vector3::new(lo.x, hi.y, lo.z),
        Vector3::new(lo.x, lo.y, hi.z),
        Vector3::new(hi.x, hi.y, lo.z),
        Vector3::new(hi.x, lo.y, hi.z),
        Vector3::new(lo.x, hi.y, hi.z),
        Vector3::new(hi.x, hi.y, hi.z),
    ]
    .map(|p| transform * p.push(1.0));
    // Strict comparisons retain the first operand for unordered values, as std::min/max.
    let mut min = corners[0];
    let mut max = corners[0];
    for corner in corners.iter().skip(1) {
        for axis in 0..3 {
            if corner[axis] < min[axis] {
                min[axis] = corner[axis];
            }
            if max[axis] < corner[axis] {
                max[axis] = corner[axis];
            }
        }
    }
    let mut file = std::fs::File::create(filename)?;
    write!(
        file,
        "# vtk DataFile Version 1.0\nBounding Box of Submap\nASCII\nDATASET POLYDATA\nPOINTS 8 float\n"
    )?;
    for point in [
        Vector3::new(min.x, min.y, min.z),
        Vector3::new(max.x, min.y, min.z),
        Vector3::new(max.x, max.y, min.z),
        Vector3::new(min.x, max.y, min.z),
        Vector3::new(min.x, min.y, max.z),
        Vector3::new(max.x, min.y, max.z),
        Vector3::new(max.x, max.y, max.z),
        Vector3::new(min.x, max.y, max.z),
    ] {
        writeln!(file, "{} {} {}", point.x, point.y, point.z)?;
    }
    write!(
        file,
        "LINES 12 36\n2 0 1\n2 1 2\n2 2 3\n2 3 0\n2 4 5\n2 5 6\n2 6 7\n2 7 4\n2 0 4\n2 1 5\n2 2 6\n2 3 7\n"
    )
}
/// Appends a uniform sample, retaining source ordering for selected points.
pub fn downsample_point_cloud<T: Clone>(original: &[T], output: &mut Vec<T>, num_of_points: usize) {
    if num_of_points > original.len() {
        output.extend_from_slice(original);
        return;
    }
    let mut selected = (0..original.len()).sample(&mut rand::rng(), num_of_points);
    selected.sort_unstable();
    output.extend(selected.into_iter().map(|i| original[i].clone()));
}
/// Clears outputs and chooses the lowest uncertainties using a stable ordering.
/// Returns false only if a selected source index has no point or uncertainty.
/// # Panics
/// Panics if a selected voxel has no source point after nonempty-voxel validation.
pub fn downsample_points_uncertainty(
    points: &[Vector3<f32>],
    sigmas: &[f32],
    down_points: &mut Vec<Vector3<f32>>,
    down_sigma: &mut Vec<f32>,
    max_num: usize,
) -> bool {
    down_points.clear();
    down_sigma.clear();
    let mut indices: Vec<_> = (0..sigmas.len()).collect();
    // NaNs violate the source stable_sort comparator's strict weak ordering.
    if sigmas.iter().any(|sigma| sigma.is_nan()) {
        return false;
    }
    indices.sort_by(|&left, &right| sigmas[left].partial_cmp(&sigmas[right]).unwrap());
    let count = points.len().min(max_num);
    if count > indices.len() || indices.iter().take(count).any(|&i| i >= points.len()) {
        return false;
    }
    for i in indices.into_iter().take(count) {
        down_points.push(points[i]);
        down_sigma.push(sigmas[i]);
    }
    true
}

#[derive(Clone, Debug)]
pub struct LidarMotionCompensation {
    t_start: Timestamp,
    t_end: Timestamp,
    delta_t: f64,
    imu_index: usize,
    base: Preintegration,
    interpolated: Preintegration,
    reference: SpeedAndBias,
    omega: Vector3<f64>,
    omega_interpolated: Vector3<f64>,
}
/// The snapshot declares the `LidarMotionCompensation` constructor without a
/// definition. A caller supplying that capability owns its initialization.
pub trait LidarMotionCompensationConstructor {
    fn construct(start: Timestamp, measurements: &ImuMeasurementDeque) -> LidarMotionCompensation;
}
impl LidarMotionCompensation {
    #[must_use]
    pub fn new<C: LidarMotionCompensationConstructor>(
        start: Timestamp,
        measurements: &ImuMeasurementDeque,
    ) -> Self {
        C::construct(start, measurements)
    }
    /// Defined initialization of `BatchedLidarPropagator` in `ViInterface.cpp`.
    #[must_use]
    pub fn batched(start: Timestamp) -> Self {
        Self {
            t_start: start,
            t_end: start,
            delta_t: 0.0,
            imu_index: 0,
            base: Preintegration::default(),
            interpolated: Preintegration::default(),
            reference: SpeedAndBias::zeros(),
            omega: Vector3::zeros(),
            omega_interpolated: Vector3::zeros(),
        }
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn append_to(
        &mut self,
        measurements: &ImuMeasurementDeque,
        _pose: &Transformation,
        sb: &SpeedAndBias,
        end: Timestamp,
    ) -> Result<bool, IntegrationError> {
        let first = measurements
            .front()
            .ok_or(IntegrationError::MissingMeasurements)?;
        let last = measurements
            .back()
            .ok_or(IntegrationError::MissingMeasurements)?;
        if first.time_stamp > self.t_end {
            return Err(IntegrationError::StartNotCovered);
        }
        if last.time_stamp < end {
            return Ok(false);
        }
        let mut time = self.t_end;
        self.reference = *sb;
        let mut started = false;
        while self.imu_index + 1 < measurements.len() {
            let m0 = &measurements[self.imu_index];
            let m1 = &measurements[self.imu_index + 1];
            let (mut w0, mut w1) = (m0.measurement.gyroscopes, m1.measurement.gyroscopes);
            let (mut a0, mut a1) = (m0.measurement.accelerometers, m1.measurement.accelerometers);
            let mut next = m1.time_stamp;
            let mut dt = seconds(next, time);
            if end < next {
                let interval = seconds(next, m0.time_stamp);
                next = end;
                dt = seconds(next, time);
                let r = dt / interval;
                w1 = (1.0 - r) * w0 + r * w1;
                a1 = (1.0 - r) * a0 + r * a1;
            }
            if dt <= 0.0 {
                self.imu_index += 1;
                continue;
            }
            if !started {
                started = true;
                let r = dt / seconds(next, m0.time_stamp);
                w0 = r * w0 + (1.0 - r) * w1;
                a0 = r * a0 + (1.0 - r) * a1;
            }
            if next == end {
                // The last partial interval is queried without committing it.
                self.interpolated = self.base.clone();
                self.omega_interpolated = self.interpolated.step(
                    (w0, w1),
                    (a0, a1),
                    dt,
                    sb,
                    None,
                    (false, NoiseMode::None),
                );
                break;
            }
            self.omega = self
                .base
                .step((w0, w1), (a0, a1), dt, sb, None, (false, NoiseMode::None));
            time = next;
            self.t_end = next;
            self.imu_index += 1;
        }
        self.delta_t = seconds(end, self.t_start);
        Ok(true)
    }
    #[must_use]
    pub fn get_state(
        &self,
        pose: &Transformation,
        sb: &SpeedAndBias,
    ) -> (Transformation, SpeedAndBias, Vector3<f64>) {
        let dt = self.delta_t;
        let preintegration = &self.interpolated;
        let rotation = pose.rotation_matrix();
        let gravity = Vector3::new(0.0, 0.0, 9.81);
        let db = sb.fixed_rows::<6>(3) - self.reference.fixed_rows::<6>(3);
        let position = pose.translation() + sb.fixed_rows::<3>(0) * dt - 0.5 * gravity * dt * dt
            + rotation
                * (preintegration.acc_doubleintegral
                    + preintegration.dp_db_g * db.fixed_rows::<3>(0)
                    - preintegration.c_doubleintegral * db.fixed_rows::<3>(3));
        let dq = delta_q(&(-preintegration.dalpha_db_g * db.fixed_rows::<3>(0)))
            * preintegration.delta_q;
        let quaternion = pose.quaternion() * dq;
        let velocity = sb.fixed_rows::<3>(0) - gravity * dt
            + rotation
                * (preintegration.acc_integral + preintegration.dv_db_g * db.fixed_rows::<3>(0)
                    - preintegration.c_integral * db.fixed_rows::<3>(3));
        let mut output = *sb;
        output.fixed_rows_mut::<3>(0).copy_from(&velocity);
        (
            Transformation::new(position, quaternion),
            output,
            self.omega_interpolated,
        )
    }
}

impl MeanOccupancyMap for crate::mapping::map_typedefs::OccupancyMap {
    fn interp_mean_occupancy(
        &self,
        point: &Vector3<f32>,
        desired_scale: i32,
        safe: Safe,
    ) -> Option<(f32, i32)> {
        let result = self.read().and_then(|map| {
            map.interp(
                point,
                desired_scale,
                safe == Safe::On,
                |data| data.field.valid(),
                |data| data.field.occupancy,
            )
        });
        match result {
            Ok(value) => value,
            Err(error) => {
                log::error!("mean occupancy interpolation failed: {error}");
                None
            }
        }
    }
    fn grad_mean_occupancy(
        &self,
        point: &Vector3<f32>,
        desired_scale: i32,
        safe: Safe,
    ) -> Option<(Vector3<f32>, i32)> {
        let result = self.read().and_then(|map| {
            map.grad(
                point,
                desired_scale,
                safe == Safe::On,
                |data| data.field.valid(),
                |data| data.field.occupancy,
            )
        });
        match result {
            Ok(value) => value,
            Err(error) => {
                log::error!("mean occupancy gradient failed: {error}");
                None
            }
        }
    }
}
impl SubmapBounds for firefly_supereight2::submap::Submap {
    fn map_aabb(&self) -> (Vector3<f32>, Vector3<f32>) {
        if let Some(map) = &self.map {
            match map.read() {
                Ok(map) => {
                    let aabb = map.aabb();
                    return (aabb.min, aabb.max);
                }
                Err(error) => log::error!("submap bounds lock poisoned: {error}"),
            }
        }
        (Vector3::repeat(f32::NAN), Vector3::repeat(f32::NAN))
    }
    fn t_wk(&self) -> SMatrix<f32, 3, 4> {
        self.t_wk
            .to_homogeneous()
            .fixed_view::<3, 4>(0, 0)
            .into_owned()
    }
}
