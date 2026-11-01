// Copyright (c) 2019 Vladyslav Usenko, Michael Loipführer and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
//! Original `src/time_alignment.cpp`: gyro/mocap alignment, grid search,
//! Jacobi-SVD quadratic refinement, and output files. Rendering is caller-owned.
use crate::utils::sophus_utils::se3_inverse;
use crate::utils::sophus_utils::se3_product;
use crate::utils::sophus_utils::so3_inverse;
use crate::utils::sophus_utils::so3_log;
use crate::utils::sophus_utils::so3_product;
use crate::{
    io::dataset_io::VioDataset,
    utils::complete_orthogonal_decomposition::CompleteOrthogonalDecomposition,
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::calibration::MocapCalibration;
use nalgebra::{DMatrix, DVector, Isometry3, Vector3};
use serde::Serialize;
use std::{
    fs::{self, File},
    io::{self, Write},
    path::Path,
};

pub const DEFAULT_MAX_OFFSET_SECONDS: f64 = 10.;

/// Load the two original Cereal JSON calibration documents.
/// # Errors
/// Returns file or archive errors. Both documents must contain `value0`.
pub fn load_calibrations(
    camera_path: &Path,
    mocap_path: &Path,
) -> io::Result<(Calibration<f64>, MocapCalibration<f64>)> {
    let camera: serde_json::Value =
        serde_json::from_reader(File::open(camera_path)?).map_err(io::Error::other)?;
    let mocap: serde_json::Value =
        serde_json::from_reader(File::open(mocap_path)?).map_err(io::Error::other)?;
    Ok((
        serde_json::from_value(camera["value0"].clone()).map_err(io::Error::other)?,
        serde_json::from_value(mocap["value0"].clone()).map_err(io::Error::other)?,
    ))
}

#[derive(Clone, Debug, Default)]
pub struct RotationSamples {
    pub timestamps: Vec<i64>,
    pub values: Vec<Vector3<f64>>,
}
#[derive(Clone, Debug)]
pub struct Alignment {
    pub gyro: RotationSamples,
    pub mocap: RotationSamples,
    pub saturated_gyro: usize,
    pub saturated_mocap: usize,
    pub offsets_ms: Vec<f64>,
    pub errors: Vec<f64>,
    pub best_offset_ns: i32,
    pub best_offset_refined_ns: i32,
    pub best_error_index: i32,
    pub best_error: f64,
    pub coefficients: Vector3<f64>,
}

/// Nearest-sample matching, with the original 120 Hz association bound.
/// # Panics
/// Requires equal timestamp/value lengths and sorted timestamps.
#[must_use]
pub fn compute_error(offset: i64, gyro: &RotationSamples, mocap: &RotationSamples) -> f64 {
    if gyro.timestamps.len() < 2
        || gyro.values.len() < 2
        || mocap.timestamps.is_empty()
        || mocap.values.is_empty()
    {
        return f64::INFINITY;
    }
    assert_eq!(gyro.timestamps.len(), gyro.values.len());
    assert_eq!(mocap.timestamps.len(), mocap.values.len());
    let mut error = 0.;
    let mut count = 0;
    let mut j = 1;
    for (&time, value) in mocap.timestamps.iter().zip(&mocap.values) {
        let corrected = time + offset;
        if corrected <= gyro.timestamps[0] {
            continue;
        }
        while j < gyro.timestamps.len() && gyro.timestamps[j] < corrected {
            j += 1;
        }
        if j >= gyro.timestamps.len() {
            break;
        }
        let after = gyro.timestamps[j] - corrected;
        let before = corrected - gyro.timestamps[j - 1];
        assert!(after >= 0 && before >= 0);
        let index = if after < before { j } else { j - 1 };
        if after.min(before) as f64 > 1e9 / 120. {
            continue;
        }
        error += (gyro.values[index] - value).norm();
        count += 1;
    }
    if count == 0 {
        f64::INFINITY
    } else {
        error / f64::from(count)
    }
}

/// Prepare calibrated streams and perform the original 100 microsecond search.
/// Both calibrations must be supplied together, as with the original CLI.
/// # Panics
/// Requires matching ground-truth timestamp/pose lengths and a finite nonnegative
/// search bound representable in nanoseconds. Sample timestamps must be sorted.
#[must_use]
pub fn align(
    dataset: &dyn VioDataset,
    calibration: Option<(&Calibration<f64>, &MocapCalibration<f64>)>,
    max_offset_s: f64,
) -> Alignment {
    assert_eq!(dataset.gt_timestamps().len(), dataset.gt_pose_data().len());
    let mut gyro = RotationSamples::default();
    let mut mocap = RotationSamples::default();
    let mut saturated_gyro = 0;
    let mut saturated_mocap = 0;
    for sample in dataset.gyro_data() {
        if sample.data.abs().max() > 499. * std::f64::consts::PI / 180. {
            saturated_gyro += 1;
            continue;
        }
        gyro.timestamps.push(sample.timestamp_ns);
        gyro.values
            .push(calibration.map_or(sample.data, |(calib, _)| {
                calib.calib_gyro_bias.calibrated(&sample.data)
            }));
    }
    let marker_to_imu = calibration.map_or_else(Isometry3::identity, |(_, calib)| {
        se3_inverse(&calib.t_i_mark)
    });
    for i in 1..dataset.gt_timestamps().len().saturating_sub(1) {
        let previous = se3_product(&dataset.gt_pose_data()[i - 1], &marker_to_imu);
        let next = se3_product(&dataset.gt_pose_data()[i + 1], &marker_to_imu);
        let dt = (dataset.gt_timestamps()[i + 1] - dataset.gt_timestamps()[i - 1]) as f64 * 1e-9;
        if dt > 2.5 / 120. {
            continue;
        }
        let velocity = so3_log(&so3_product(
            &so3_inverse(&previous.rotation),
            &next.rotation,
        )) / dt;
        if velocity.abs().max() > 500. * std::f64::consts::PI / 180. {
            saturated_mocap += 1;
            continue;
        }
        mocap.timestamps.push(dataset.gt_timestamps()[i]);
        mocap.values.push(velocity);
    }
    let mut result = search(gyro, mocap, max_offset_s);
    result.saturated_gyro = saturated_gyro;
    result.saturated_mocap = saturated_mocap;
    result
}

/// Search including both bounds, then fit the 21 samples around the best index.
/// # Panics
/// Requires a finite nonnegative bound representable as nanoseconds, and valid samples.
#[must_use]
pub fn search(gyro: RotationSamples, mocap: RotationSamples, max_offset_s: f64) -> Alignment {
    assert!(max_offset_s.is_finite() && max_offset_s >= 0. && max_offset_s * 1e9 < i64::MAX as f64);
    let max_offset = (max_offset_s * 1e9) as i64;
    let mut result = Alignment {
        gyro,
        mocap,
        saturated_gyro: 0,
        saturated_mocap: 0,
        offsets_ms: Vec::new(),
        errors: Vec::new(),
        best_offset_ns: 0,
        best_offset_refined_ns: 0,
        best_error_index: -1,
        best_error: f64::MAX,
        coefficients: Vector3::zeros(),
    };
    for offset in (-max_offset..=max_offset).step_by(100_000) {
        let error = compute_error(offset, &result.gyro, &result.mocap);
        result.offsets_ms.push(offset as f64 * 1e-6);
        result.errors.push(error);
        if error < result.best_error {
            result.best_error = error;
            result.best_offset_ns = offset as i32;
            result.best_error_index = (result.errors.len() - 1) as i32;
        }
    }
    result.best_offset_refined_ns = result.best_offset_ns;
    if result.best_error_index >= 10 && result.best_error_index + 10 < (result.errors.len() as i32)
    {
        let index = result.best_error_index as usize;
        result.coefficients = fit_quadratic(&result.errors[index - 10..=index + 10]);
        if result.coefficients[0] > 1e-9 {
            result.best_offset_refined_ns = (f64::from(result.best_offset_refined_ns)
                - 100_000. * result.coefficients[1] / (2. * result.coefficients[0]))
                as i32;
        }
    }
    result
}

// Eigen JacobiSVD real 3-column specialization, MPL-2.0.
// Copyright (C) 2009-2010 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2013-2016 Gael Guennebaud <gael.guennebaud@inria.fr>
// See LICENSE-MPL-2.0. The polynomial matrix is always full column rank.
#[expect(
    clippy::many_single_char_names,
    clippy::too_many_lines,
    reason = "Preserves Eigen's complete two-sided Jacobi SVD notation and update order"
)]
fn fit_quadratic(errors: &[f64]) -> Vector3<f64> {
    let matrix = DMatrix::from_fn(21, 3, |row, col| {
        let x = row as f64 - 10.;
        match col {
            0 => x * x,
            1 => x,
            _ => 1.,
        }
    });
    let scale = 100.;
    let decomposition = CompleteOrthogonalDecomposition::new(matrix / scale);
    let (mut u, mut work, permutation) = decomposition.full_rank_qr_factors();
    let mut v = DMatrix::<f64>::zeros(3, 3);
    for (column, &row) in permutation.iter().enumerate() {
        v[(row, column)] = 1.;
    }
    let mut maximum = (0..3).map(|i| work[(i, i)].abs()).fold(0., f64::max);
    loop {
        let mut finished = true;
        for p in 1..3 {
            for q in 0..p {
                let threshold = f64::MIN_POSITIVE.max(2. * f64::EPSILON * maximum);
                if work[(p, q)].abs() <= threshold && work[(q, p)].abs() <= threshold {
                    continue;
                }
                finished = false;
                let t = work[(p, p)] + work[(q, q)];
                let d = work[(q, p)] - work[(p, q)];
                let (c, s) = if d.abs() < f64::MIN_POSITIVE {
                    (1., 0.)
                } else {
                    let ratio = t / d;
                    let norm = (1. + ratio * ratio).sqrt();
                    (ratio / norm, 1. / norm)
                };
                let x = c * work[(p, p)] + s * work[(q, p)];
                let y = c * work[(p, q)] + s * work[(q, q)];
                let z = -s * work[(p, q)] + c * work[(q, q)];
                let denominator = 2. * y.abs();
                let (cr, sr) = if denominator < f64::MIN_POSITIVE {
                    (1., 0.)
                } else {
                    let tau = (x - z) / denominator;
                    let w = (tau * tau + 1.).sqrt();
                    let tangent = if tau > 0. {
                        1. / (tau + w)
                    } else {
                        1. / (tau - w)
                    };
                    let norm = 1. / (tangent * tangent + 1.).sqrt();
                    (norm, -tangent.signum() * y.signum() * tangent.abs() * norm)
                };
                let cl = c * cr + s * sr;
                let sl = -c * sr + s * cr;
                for col in 0..3 {
                    let x = work[(p, col)];
                    let y = work[(q, col)];
                    work[(p, col)] = cl * x + sl * y;
                    work[(q, col)] = -sl * x + cl * y;
                }
                rotate_right(&mut u, p, q, cl, -sl);
                rotate_right(&mut work, p, q, cr, sr);
                rotate_right(&mut v, p, q, cr, sr);
                maximum = maximum.max(work[(p, p)].abs()).max(work[(q, q)].abs());
            }
        }
        if finished {
            break;
        }
    }
    let mut singular = Vector3::zeros();
    for i in 0..3 {
        singular[i] = work[(i, i)].abs() * scale;
        if work[(i, i)] < 0. {
            u.column_mut(i).neg_mut();
        }
    }
    for i in 0..3 {
        let mut largest = i;
        for j in i + 1..3 {
            if singular[j] > singular[largest] {
                largest = j;
            }
        }
        if singular[largest] == 0. {
            break;
        }
        singular.swap_rows(i, largest);
        u.swap_columns(i, largest);
        v.swap_columns(i, largest);
    }
    let mut projected = u.transpose() * DVector::from_column_slice(errors);
    let threshold = (3. * f64::EPSILON * singular[0]).max(f64::MIN_POSITIVE);
    for i in 0..3 {
        projected[i] = if singular[i] > threshold {
            projected[i] / singular[i]
        } else {
            0.
        };
    }
    let solution = v * projected;
    Vector3::new(solution[0], solution[1], solution[2])
}
#[expect(
    clippy::many_single_char_names,
    reason = "Original Jacobi rotation notation"
)]
fn rotate_right(matrix: &mut DMatrix<f64>, p: usize, q: usize, c: f64, s: f64) {
    for row in 0..matrix.nrows() {
        let x = matrix[(row, p)];
        let y = matrix[(row, q)];
        matrix[(row, p)] = c * x - s * y;
        matrix[(row, q)] = s * x + c * y;
    }
}

#[derive(Serialize)]
#[expect(
    clippy::struct_field_names,
    reason = "Original public Cereal archive keys"
)]
struct OffsetArchive {
    mocap_to_imu_initial_offset_ns: i64,
    mocap_to_imu_additional_offset_refined_ns: i32,
    mocap_to_imu_total_offset_ns: i64,
}
impl Alignment {
    /// # Errors
    /// Returns an output or serialization error.
    pub fn write_offsets(&self, output: impl Write, initial_offset_ns: i64) -> io::Result<()> {
        serde_json::to_writer_pretty(
            output,
            &OffsetArchive {
                mocap_to_imu_initial_offset_ns: initial_offset_ns,
                mocap_to_imu_additional_offset_refined_ns: self.best_offset_refined_ns,
                mocap_to_imu_total_offset_ns: initial_offset_ns
                    + i64::from(self.best_offset_refined_ns),
            },
        )
        .map_err(io::Error::other)
    }
    /// # Errors
    /// Returns an output error.
    pub fn write_error_series(&self, mut output: impl Write) -> io::Result<()> {
        writeln!(output, "#TIME_MS,ERROR,ERROR_FITTED")?;
        writeln!(
            output,
            "# best_offset_ms: {}, best_offset_refined_ms: {}",
            f64::from(self.best_offset_ns) * 1e-6,
            f64::from(self.best_offset_refined_ns) * 1e-6
        )?;
        for (i, (&offset, &error)) in self.offsets_ms.iter().zip(&self.errors).enumerate() {
            let index = f64::from(i as i32 - self.best_error_index);
            let fitted = Vector3::new(index * index, index, 1.).dot(&self.coefficients);
            writeln!(output, "{offset},{error},{fitted}")?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an output error.
    pub fn write_gyro(&self, output: impl Write, min_time: i64) -> io::Result<()> {
        write_samples(output, &self.gyro, min_time, 0, self.gyro.timestamps.len())
    }
    /// The source indexes mocap using the gyro count. A short mocap stream is
    /// reported as an error rather than allowing the original out-of-bounds read.
    /// # Errors
    /// Returns an output error or a mismatched-length error.
    pub fn write_mocap(&self, output: impl Write, min_time: i64) -> io::Result<()> {
        write_samples(
            output,
            &self.mocap,
            min_time,
            i64::from(self.best_offset_ns),
            self.gyro.timestamps.len(),
        )
    }
}
fn write_samples(
    mut output: impl Write,
    samples: &RotationSamples,
    min_time: i64,
    offset: i64,
    count: usize,
) -> io::Result<()> {
    if samples.timestamps.len() < count || samples.values.len() < count {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Original mocap output requires at least as many mocap samples as gyro samples",
        ));
    }
    writeln!(output, "#TIME_M, GX, GY, GZ")?;
    for i in 0..count {
        let t = (samples.timestamps[i] + offset - min_time) as f64 * 1e-9;
        let v = samples.values[i];
        writeln!(output, "{t} {} {} {}", v.x, v.y, v.z)?;
    }
    Ok(())
}

/// # Errors
/// Returns directory creation or output errors.
pub fn write_aligned_dataset(
    path: &Path,
    dataset: &dyn VioDataset,
    offset: i64,
    calibration: Option<&MocapCalibration<f64>>,
) -> io::Result<()> {
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        fs::create_dir_all(parent)?;
    }
    write_aligned_poses(File::create(path)?, dataset, offset, calibration)
}
/// # Errors
/// Returns output errors or a timestamp/pose length mismatch.
pub fn write_aligned_poses(
    mut output: impl Write,
    dataset: &dyn VioDataset,
    offset: i64,
    calibration: Option<&MocapCalibration<f64>>,
) -> io::Result<()> {
    if dataset.gt_timestamps().len() != dataset.gt_pose_data().len() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Ground-truth timestamps and poses differ in length",
        ));
    }
    let marker_to_imu =
        calibration.map_or_else(Isometry3::identity, |calib| se3_inverse(&calib.t_i_mark));
    writeln!(
        output,
        "#timestamp [ns], p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []"
    )?;
    for (&time, pose) in dataset.gt_timestamps().iter().zip(dataset.gt_pose_data()) {
        let pose = se3_product(pose, &marker_to_imu);
        let p = pose.translation.vector;
        let q = pose.rotation.quaternion();
        writeln!(
            output,
            "{},{},{},{},{},{},{},{}",
            time + offset,
            p.x,
            p.y,
            p.z,
            q.w,
            q.i,
            q.j,
            q.k
        )?;
    }
    Ok(())
}
