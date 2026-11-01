// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
//! Original `src/kitti_eval.cpp`: KITTI segment evaluation and trajectory I/O.
use firefly_basalt_headers::utils::sophus_utils::{se3_inverse, se3_product, so3_from_quaternion};
use nalgebra::{Isometry3, Matrix3, Quaternion, Translation3};
use serde::Serialize;
use std::{
    collections::BTreeMap,
    fs::File,
    io::{self, BufRead, BufReader, Write},
    path::Path,
};

pub const DEFAULT_LENGTHS: [f64; 8] = [100., 200., 300., 400., 500., 600., 700., 800.];

/// Read row-major 3x4 KITTI poses; malformed lines return errors instead of
/// constructing a pose from the source's uninitialized matrix entries.
/// # Errors
/// Returns file, parse, or invalid-quaternion errors.
pub fn load_poses(path: &Path) -> io::Result<Vec<Isometry3<f64>>> {
    read_poses(BufReader::new(File::open(path)?))
}

/// # Errors
/// Returns an error for malformed pose rows or failed input reads.
pub fn read_poses(input: impl BufRead) -> io::Result<Vec<Isometry3<f64>>> {
    let mut poses = Vec::new();
    for line in input.lines() {
        let line = line?;
        if line.starts_with('#') {
            continue;
        }
        let values = line
            .split_whitespace()
            .take(12)
            .map(str::parse::<f64>)
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        if values.len() != 12 || values.iter().any(|x| !x.is_finite()) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Expected twelve finite KITTI pose coefficients",
            ));
        }
        let rotation = Matrix3::from_fn(|r, c| values[r * 4 + c]);
        let quaternion = matrix_quaternion(&rotation);
        if !quaternion.norm().is_finite() || quaternion.norm() < 1e-10 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Invalid KITTI rotation",
            ));
        }
        poses.push(Isometry3::from_parts(
            Translation3::new(values[3], values[7], values[11]),
            so3_from_quaternion(quaternion),
        ));
    }
    Ok(poses)
}

// Eigen Quaternion matrix construction, MPL-2.0; see LICENSE-MPL-2.0.
// Copyright (C) 2008-2010 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2009 Mathieu Gautier <mathieu.gautier@cea.fr>
fn matrix_quaternion(matrix: &Matrix3<f64>) -> Quaternion<f64> {
    let mut coefficients = [0.; 4];
    let mut t = matrix.trace();
    if t > 0. {
        t = (t + 1.).sqrt();
        coefficients[3] = 0.5 * t;
        t = 0.5 / t;
        coefficients[0] = (matrix[(2, 1)] - matrix[(1, 2)]) * t;
        coefficients[1] = (matrix[(0, 2)] - matrix[(2, 0)]) * t;
        coefficients[2] = (matrix[(1, 0)] - matrix[(0, 1)]) * t;
    } else {
        let mut i = 0;
        if matrix[(1, 1)] > matrix[(0, 0)] {
            i = 1;
        }
        if matrix[(2, 2)] > matrix[(i, i)] {
            i = 2;
        }
        let j = (i + 1) % 3;
        let k = (j + 1) % 3;
        t = (matrix[(i, i)] - matrix[(j, j)] - matrix[(k, k)] + 1.).sqrt();
        coefficients[i] = 0.5 * t;
        t = 0.5 / t;
        coefficients[3] = (matrix[(k, j)] - matrix[(j, k)]) * t;
        coefficients[j] = (matrix[(j, i)] + matrix[(i, j)]) * t;
        coefficients[k] = (matrix[(k, i)] + matrix[(i, k)]) * t;
    }
    Quaternion::new(
        coefficients[3],
        coefficients[0],
        coefficients[1],
        coefficients[2],
    )
}

#[derive(Clone, Debug, Serialize)]
pub struct SegmentError {
    pub trans_error: f64,
    pub rot_error: f64,
    pub num_meas: usize,
}
pub type Results = BTreeMap<String, SegmentError>;

/// Original float distance accumulation, ten-frame stride, strict segment end,
/// and degree-per-meter rotation metric. Empty segments retain NaN errors.
/// # Panics
/// Requires matching, nonempty ground-truth and estimate trajectories.
#[must_use]
pub fn eval_kitti(
    lengths: &[f64],
    ground_truth: &[Isometry3<f64>],
    estimate: &[Isometry3<f64>],
) -> Results {
    assert_ne!(ground_truth, []);
    assert_eq!(ground_truth.len(), estimate.len());
    let mut distances = vec![0_f32];
    for pair in ground_truth.windows(2) {
        distances.push(
            (f64::from(*distances.last().unwrap())
                + (pair[1].translation.vector - pair[0].translation.vector).norm())
                as f32,
        );
    }
    let mut results = BTreeMap::new();
    for &length in lengths {
        let length = length as f32;
        let mut translation_error = 0.;
        let mut rotation_error = 0.;
        let mut count = 0;
        for first in (0..ground_truth.len()).step_by(10) {
            let Some(last) =
                (first..distances.len()).find(|&i| distances[i] > distances[first] + length)
            else {
                continue;
            };
            let gt = se3_product(&se3_inverse(&ground_truth[first]), &ground_truth[last]);
            let result = se3_product(&se3_inverse(&estimate[first]), &estimate[last]);
            let difference = result.rotation.quaternion() * gt.rotation.quaternion().conjugate();
            let rotation = 2. * difference.vector().norm().atan2(difference.w.abs()) * 180.
                / std::f64::consts::PI;
            translation_error +=
                (result.translation.vector - gt.translation.vector).norm() / f64::from(length);
            rotation_error += rotation / f64::from(length);
            count += 1;
        }
        results.insert(
            (length as i32).to_string(),
            SegmentError {
                trans_error: 100. * translation_error / count as f64,
                rot_error: rotation_error / count as f64,
                num_meas: count,
            },
        );
    }
    results
}

/// # Errors
/// Returns serialization or output errors.
pub fn write_results(mut output: impl Write, results: &Results) -> io::Result<()> {
    // Cereal explicitly enables RapidJSON's NaN/Infinity extension. Serde's
    // default null conversion would lose the no-segment result semantics.
    write!(output, "{{\"results\":{{")?;
    for (index, (length, result)) in results.iter().enumerate() {
        if index != 0 {
            write!(output, ",")?;
        }
        write!(
            output,
            "{}:{{\"num_meas\":{},\"rot_error\":",
            serde_json::to_string(length).map_err(io::Error::other)?,
            result.num_meas
        )?;
        write_number(&mut output, result.rot_error)?;
        write!(output, ",\"trans_error\":")?;
        write_number(&mut output, result.trans_error)?;
        write!(output, "}}")?;
    }
    write!(output, "}}}}")
}
fn write_number(output: &mut impl Write, value: f64) -> io::Result<()> {
    if value.is_nan() {
        write!(output, "NaN")
    } else if value == f64::INFINITY {
        write!(output, "Infinity")
    } else if value == f64::NEG_INFINITY {
        write!(output, "-Infinity")
    } else {
        write!(output, "{value}")
    }
}
