/*
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//! Translation of official Basalt `include/basalt/utils/keypoints.h` and
//! `src/utils/keypoints.cpp`, revision `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

const PATTERN_31_X_A: [i8; 256] = [
    8, 4, -11, 7, 2, 1, -2, -13, -13, 10, -13, -11, 7, -4, -13, -9, 12, -3, -6, 11, 4, 5, 3, -8,
    -2, -13, -7, -4, -10, 5, 5, 1, 9, 4, 2, -4, -8, 4, 0, -13, -3, -6, 8, 0, 7, -13, 10, -6, 10,
    -13, -13, 3, 5, -1, 3, 2, -13, -13, -13, -7, 6, -9, -2, -12, 3, -7, -3, 2, -11, -1, 5, -4, -9,
    -12, 10, 7, -7, -4, 7, -7, -13, -3, 7, -13, 1, 2, -4, -1, 7, 1, 9, -1, -13, 7, 12, 6, 5, 2, 3,
    2, 9, -8, -11, 1, 6, 2, 6, 3, 7, -11, -10, -5, -10, 8, 4, -10, 4, -2, -5, 7, -9, -5, 8, -9, 1,
    7, -2, 11, -12, 3, 5, 0, -9, 0, -1, 5, 3, -13, -5, -4, 6, -7, -13, 1, 4, -2, 2, -2, 4, -6, -3,
    7, 4, -13, 7, 7, -7, -8, -13, 2, 10, -6, 8, 2, -11, -12, -11, 5, -2, -1, -13, -10, -3, 2, -9,
    -4, -4, -6, 6, -13, 11, 7, -1, -4, -7, -13, -7, -8, -5, -13, 1, 1, 9, 5, -1, -9, -1, -13, 8, 2,
    7, -10, -10, 4, 3, -4, 5, 4, -9, 0, -12, 3, -10, 8, -8, 2, 10, 6, -7, -3, -1, -3, -8, 4, 2, 6,
    3, 11, -3, 4, 2, -10, -13, -13, 6, 0, -13, -9, -13, 5, 2, -1, 9, 11, 3, -1, 3, -13, 5, 8, 7,
    -10, 7, 9, 7, -1,
];
const PATTERN_31_Y_A: [i8; 256] = [
    -3, 2, 9, -12, -13, -7, -10, -13, -3, 4, -8, 7, 7, -5, 2, 0, -6, 6, -13, -13, 7, -3, -7, -7,
    11, 12, 3, 2, -12, -12, -6, 0, 11, 7, -1, -12, -5, 11, -8, -2, -2, 9, 12, 9, -5, -6, 7, -3, -9,
    8, 0, 3, 7, 7, -10, -4, 0, -7, 3, 12, -10, -1, -5, 5, -10, -7, -2, 9, -13, 6, -3, -13, -6, -10,
    2, 12, -13, 9, -1, 6, 11, 7, -8, -7, -3, -6, 3, -13, 1, -1, 1, -9, -13, 7, -5, 3, -13, -12, 8,
    6, -12, 4, 12, 12, -9, 3, 3, -3, 8, -5, 11, -8, 5, -1, -6, 12, -2, 0, -8, -6, -13, -13, -8,
    -11, -8, -4, 1, -6, -9, 7, 5, -4, 12, 7, 2, 11, 5, -4, 9, -7, 5, 6, 6, -10, 1, -2, -12, -13, 1,
    -10, -13, 5, -2, 9, 1, -8, -4, 11, 6, 4, -5, -5, -3, -12, -2, -13, 0, -3, -13, -8, -11, -2, 9,
    -3, -13, 6, 12, -11, -3, 11, 11, -5, 12, -8, 1, -12, -2, 5, -1, 7, 5, 0, 12, -8, 11, -3, -10,
    1, -11, -13, -13, -10, -8, -6, 12, 2, -13, -13, 9, 3, 1, 2, -10, -13, -12, 2, 6, 8, 10, -9,
    -13, -7, -2, 2, -5, -9, -1, -1, 0, -11, -4, -6, 7, 12, 0, -1, 3, 8, -6, -9, 7, -6, 5, -3, 0, 4,
    -6, 0, 8, 9, -4, 4, 3, -7, 0, -6,
];
const PATTERN_31_X_B: [i8; 256] = [
    9, 7, -8, 12, 2, 1, -2, -11, -12, 11, -8, -9, 12, -3, -12, -7, 12, -2, -4, 12, 5, 10, 6, -6,
    -1, -8, -5, -3, -6, 6, 7, 4, 11, 4, 4, -2, -7, 9, 1, -8, -2, -4, 10, 1, 11, -11, 12, -6, 12,
    -8, -8, 7, 10, 1, 5, 3, -13, -12, -11, -4, 12, -7, 0, -7, 8, -4, -1, 5, -5, 0, 5, -4, -9, -8,
    12, 12, -6, -3, 12, -5, -12, -2, 12, -11, 12, 3, -2, 1, 8, 3, 12, -1, -10, 10, 12, 7, 6, 2, 4,
    12, 10, -7, -4, 2, 7, 3, 11, 8, 9, -6, -5, -3, -9, 12, 6, -8, 6, -2, -5, 10, -8, -5, 9, -9, 1,
    9, -1, 12, -6, 7, 10, 2, -5, 2, 1, 7, 6, -8, -3, -3, 8, -6, -5, 3, 8, 2, 12, 0, 9, -3, -1, 12,
    5, -9, 8, 7, -7, -7, -12, 3, 12, -6, 9, 2, -10, -7, -10, 11, -1, 0, -12, -10, -2, 3, -4, -3,
    -2, -4, 6, -5, 12, 12, 0, -3, -6, -8, -6, -6, -4, -8, 5, 10, 10, 10, 1, -6, 1, -8, 10, 3, 12,
    -5, -8, 8, 8, -3, 10, 5, -4, 3, -6, 4, -10, 12, -6, 3, 11, 8, -6, -3, -1, -3, -8, 12, 3, 11, 7,
    12, -3, 4, 2, -8, -11, -11, 11, 1, -9, -6, -8, 8, 3, -1, 11, 12, 3, 0, 4, -10, 12, 9, 8, -10,
    12, 10, 12, 0,
];
const PATTERN_31_Y_B: [i8; 256] = [
    5, -12, 2, -13, 12, 6, -4, -8, -9, 9, -9, 12, 6, 0, -3, 5, -1, 12, -8, -8, 1, -3, 12, -2, -10,
    10, -3, 7, 11, -7, -1, -5, -13, 12, 4, 7, -10, 12, -13, 2, 3, -9, 7, 3, -10, 0, 1, 12, -4, -12,
    -4, 8, -7, -12, 6, -10, 5, 12, 8, 7, 8, -6, 12, 5, -13, 5, -7, -11, -13, -1, 2, 12, 6, -4, -3,
    12, 5, 4, 2, 1, 5, -6, -7, -12, 12, 0, -13, 9, -6, 12, 6, 3, 5, 12, 9, 11, 10, 3, -6, -13, 3,
    9, -6, -8, -4, -2, 0, -8, 3, -4, 10, 12, 0, -6, -11, 7, 7, 12, 2, 12, -8, -2, -13, 0, -2, 1,
    -4, -11, 4, 12, 8, 8, -13, 12, 7, -9, -8, 9, -3, -12, 0, 12, -2, 10, -4, -13, 12, -6, 3, -5, 1,
    -11, -7, -5, 6, 6, 1, -8, -8, 9, 3, 7, -8, 8, 3, -9, -5, 8, 12, 9, -5, 11, -13, 2, 0, -10, -7,
    9, 11, 5, 6, -2, 7, -2, 7, -13, -8, -9, 5, 10, -13, -13, -1, -9, -13, 2, 12, -10, -6, -6, -9,
    -7, -13, 5, -13, -3, -12, -1, 3, -9, 1, -8, 9, 12, -5, 7, -8, -12, 5, 9, 5, 4, 3, 12, 11, -13,
    12, 4, 6, 12, 1, 1, 1, -13, -13, 4, -2, -3, -2, 10, -9, -1, -2, -8, 5, 10, 5, 5, 11, -6, -12,
    9, 4, -2, -2, -11,
];

use super::common_types::{Descriptor, KeypointsData, MatchData};

#[cfg(test)]
#[path = "keypoints_tests.rs"]
mod tests;
use firefly_calibration::image::image::Image;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Isometry3, Matrix3, Matrix4, Rotation3, Translation3, UnitQuaternion, Vector2};
use purecv::{
    core::Matrix,
    features2d::{FastFeatureDetector, FastType},
    imgproc::good_features_to_track,
};

const HALF_PATCH_SIZE: i32 = 15;
const EDGE_THRESHOLD: f64 = 19.0;

fn vision_error(error: impl std::fmt::Display) -> Error {
    Error::new(ErrorKind::InvalidArgument, error.to_string())
}

fn clear_features(kd: &mut KeypointsData) {
    kd.corners.clear();
    kd.corner_angles.clear();
    kd.corner_descriptors.clear();
}

/// Detect Shi–Tomasi corners using the upstream `OpenCV` default parameters.
/// # Errors
/// Returns an error if the image or detector arguments are invalid.
pub fn detect_keypoints_mapping(
    image: &Image<'_, u16>,
    kd: &mut KeypointsData,
    num_features: i32,
) -> Result<()> {
    // The C++ overload reads w*h consecutive pixels even for pitched inputs.
    let pixels = image
        .storage()
        .iter()
        .take(image.area())
        .map(|&v| v.to_be_bytes()[0])
        .collect();
    let image8 = Matrix::from_vec(image.h, image.w, 1, pixels);
    let points = good_features_to_track(&image8, num_features, 0.01, 8.0, 3, false, 0.04)
        .map_err(vision_error)?;
    clear_features(kd);
    for p in points {
        let p = Vector2::new(f64::from(p.x), f64::from(p.y));
        if image.in_bounds(&p, EDGE_THRESHOLD) {
            kd.corners.push(p);
        }
    }
    Ok(())
}

/// Detect FAST corners in unoccupied cells, using thresholds 40, 20, 10, 5.
/// # Errors
/// Rejects nonpositive cell sizes, images smaller than a cell, or detector errors.
/// # Panics
/// Panics if image dimensions cannot be represented as an i32.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
pub fn detect_keypoints(
    image: &Image<'_, u16>,
    kd: &mut KeypointsData,
    patch_size: usize,
    num_points_cell: usize,
    current_points: &[Vector2<f64>],
) -> Result<()> {
    clear_features(kd);
    if patch_size == 0 || image.w < patch_size || image.h < patch_size {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "image must contain at least one positive-sized detection cell",
        ));
    }
    let xs = (image.w % patch_size) / 2;
    let ys = (image.h % patch_size) / 2;
    let xe = xs + patch_size * (image.w / patch_size - 1);
    let ye = ys + patch_size * (image.h / patch_size - 1);
    let width = image.w / patch_size + 1;
    let mut cells = vec![0_usize; (image.h / patch_size + 1) * width];
    for p in current_points {
        if p.x >= xs as f64
            && p.y >= ys as f64
            && p.x < (xe + patch_size) as f64
            && p.y < (ye + patch_size) as f64
        {
            let x = ((p.x - xs as f64) / patch_size as f64) as usize;
            let y = ((p.y - ys as f64) / patch_size as f64) as usize;
            cells[y * width + x] += 1;
        }
    }
    for x in (xs..=xe).step_by(patch_size) {
        for y in (ys..=ye).step_by(patch_size) {
            if cells[(y - ys) / patch_size * width + (x - xs) / patch_size] > 0 {
                continue;
            }
            let mut pixels = Vec::with_capacity(patch_size * patch_size);
            for sy in y..y + patch_size {
                for sx in x..x + patch_size {
                    pixels.push((image.get(sx, sy) >> 8) as u8);
                }
            }
            let image8 = Matrix::from_vec(patch_size, patch_size, 1, pixels);
            let mut added = 0;
            let mut threshold = 40;
            while added < num_points_cell && threshold >= 5 {
                let mut points = FastFeatureDetector::new(threshold, true, FastType::Type9_16)
                    .detect(&image8)
                    .map_err(vision_error)?;
                points.sort_unstable_by(|a, b| b.response.total_cmp(&a.response));
                for p in points {
                    if added >= num_points_cell {
                        break;
                    }
                    let position =
                        Vector2::new(x as f64 + f64::from(p.pt.x), y as f64 + f64::from(p.pt.y));
                    if image.in_bounds(&position, EDGE_THRESHOLD) {
                        kd.corners.push(position);
                        added += 1;
                    }
                }
                threshold /= 2;
            }
        }
    }
    Ok(())
}

fn pixel_offset(image: &Image<'_, u16>, center: (i32, i32), offset: (i32, i32)) -> u16 {
    *image.get(
        usize::try_from(center.0 + offset.0).expect("negative feature sample x"),
        usize::try_from(center.1 + offset.1).expect("negative feature sample y"),
    )
}

/// Intensity-centroid orientation in the upstream circular radius-15 patch.
/// # Panics
/// Requires corners with the complete radius-15 neighborhood inside the image.
#[allow(clippy::cast_possible_truncation)]
pub fn compute_angles(image: &Image<'_, u16>, kd: &mut KeypointsData, rotate_features: bool) {
    kd.corner_angles.resize(kd.corners.len(), 0.0);
    for (index, p) in kd.corners.iter().enumerate() {
        let center = (p.x as i32, p.y as i32);
        let mut angle = 0.0;
        if rotate_features {
            let mut m01 = 0.0;
            let mut m10 = 0.0;
            for x in -HALF_PATCH_SIZE..=HALF_PATCH_SIZE {
                for y in -HALF_PATCH_SIZE..=HALF_PATCH_SIZE {
                    if x * x + y * y <= HALF_PATCH_SIZE * HALF_PATCH_SIZE {
                        let val = f64::from(pixel_offset(image, center, (x, y)));
                        m01 += f64::from(y) * val;
                        m10 += f64::from(x) * val;
                    }
                }
            }
            angle = m01.atan2(m10);
        }
        kd.corner_angles[index] = angle;
    }
}

/// Rotated 256-bit ORB comparisons using the upstream four sampling tables.
/// # Panics
/// Requires one angle per corner and every rotated sample within the image.
#[allow(clippy::cast_possible_truncation)]
pub fn compute_descriptors(image: &Image<'_, u16>, kd: &mut KeypointsData) {
    kd.corner_descriptors.resize(kd.corners.len(), [0; 4]);
    for (index, p) in kd.corners.iter().enumerate() {
        let mut descriptor = [0_u64; 4];
        let center = (p.x as i32, p.y as i32);
        let (sin, cos) = kd.corner_angles[index].sin_cos();
        let rotate = |x: i8, y: i8| {
            let x = f64::from(x);
            let y = f64::from(y);
            (
                (cos * x - sin * y).round() as i32,
                (sin * x + cos * y).round() as i32,
            )
        };
        for bit in 0..256 {
            let a = rotate(PATTERN_31_X_A[bit], PATTERN_31_Y_A[bit]);
            let b = rotate(PATTERN_31_X_B[bit], PATTERN_31_Y_B[bit]);
            if pixel_offset(image, center, a) < pixel_offset(image, center, b) {
                descriptor[bit / 64] |= 1_u64 << (bit % 64);
            }
        }
        kd.corner_descriptors[index] = descriptor;
    }
}

fn match_fast_helper(
    first: &[Descriptor],
    second: &[Descriptor],
    threshold: i32,
    test_dist: f64,
) -> std::collections::BTreeMap<i32, i32> {
    let mut matches = std::collections::BTreeMap::new();
    for (i, a) in first.iter().enumerate() {
        let mut best_index = -1;
        let mut best_dist = 500;
        let mut second_dist = 500;
        for (j, b) in second.iter().enumerate() {
            let distance = i32::try_from(
                a.iter()
                    .zip(b)
                    .map(|(a, b)| (a ^ b).count_ones())
                    .sum::<u32>(),
            )
            .unwrap();
            if distance <= best_dist {
                second_dist = best_dist;
                best_dist = distance;
                best_index = i32::try_from(j).unwrap();
            } else if distance < second_dist {
                second_dist = distance;
            }
        }
        if best_dist < threshold && f64::from(best_dist) * test_dist <= f64::from(second_dist) {
            matches.insert(i32::try_from(i).unwrap(), best_index);
        }
    }
    matches
}

/// Mutual Hamming matching with the original strict threshold and ratio test.
/// Output is ordered by source index instead of C++ unordered-map iteration.
/// # Panics
/// Panics if a feature index exceeds i32.
pub fn match_descriptors(
    first: &[Descriptor],
    second: &[Descriptor],
    matches: &mut Vec<(i32, i32)>,
    threshold: i32,
    dist_2_best: f64,
) {
    matches.clear();
    let forward = match_fast_helper(first, second, threshold, dist_2_best);
    let mut reverse = match_fast_helper(second, first, threshold, dist_2_best);
    for (i, j) in forward {
        // operator[] in the upstream inserts zero for a missing reverse match.
        if *reverse.entry(j).or_default() == i {
            matches.push((i, j));
        }
    }
}

/// Homogeneous bearing-vector essential matrix [t/|t|]x R.
#[must_use]
pub fn compute_essential(transform: &Isometry3<f64>) -> Matrix4<f64> {
    let t = transform.translation.vector.normalize();
    let skew = Matrix3::new(0.0, -t.z, t.y, t.z, 0.0, -t.x, -t.y, t.x, 0.0);
    let mut essential = Matrix4::zeros();
    essential
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(skew * transform.rotation.to_rotation_matrix().matrix()));
    essential
}

/// Keep matches whose absolute homogeneous epipolar error is strictly below threshold.
/// # Panics
/// Requires match indices to address the corresponding bearing vectors.
pub fn find_inliers_essential(
    first: &KeypointsData,
    second: &KeypointsData,
    essential: &Matrix4<f64>,
    threshold: f64,
    matches: &mut MatchData,
) {
    matches.inliers.clear();
    for &(i, j) in &matches.matches {
        let a = first.corners_3d[usize::try_from(i).unwrap()];
        let b = second.corners_3d[usize::try_from(j).unwrap()];
        if a.dot(&(essential * b)).abs() < threshold {
            matches.inliers.push((i, j));
        }
    }
}

/// Central relative-pose RANSAC with the upstream Stewenius model and nonlinear refinement.
/// # Errors
/// Returns an error when indices are invalid, RANSAC finds no model, or refinement fails.
pub fn find_inliers_ransac(
    first: &KeypointsData,
    second: &KeypointsData,
    threshold: f64,
    min_inliers: usize,
    matches: &mut MatchData,
) -> Result<()> {
    use firefly_opengv::{
        optimization_functor::RustNumericalBackend,
        relative_pose::{central_relative_adapter::CentralRelativeAdapter, methods},
        sac::{ransac::Ransac, sample_consensus_problem::SampleConsensusProblem},
        sac_problems::relative_pose::central_relative_pose_sac_problem::{
            Algorithm, CentralRelativePoseSacProblem,
        },
    };
    matches.inliers.clear();
    let mut bearings1 = Vec::with_capacity(matches.matches.len());
    let mut bearings2 = Vec::with_capacity(matches.matches.len());
    for &(i, j) in &matches.matches {
        let i = usize::try_from(i).map_err(vision_error)?;
        let j = usize::try_from(j).map_err(vision_error)?;
        bearings1.push(
            first
                .corners_3d
                .get(i)
                .ok_or_else(|| {
                    Error::new(ErrorKind::OutOfRange, "first bearing index out of range")
                })?
                .fixed_rows::<3>(0)
                .into_owned(),
        );
        bearings2.push(
            second
                .corners_3d
                .get(j)
                .ok_or_else(|| {
                    Error::new(ErrorKind::OutOfRange, "second bearing index out of range")
                })?
                .fixed_rows::<3>(0)
                .into_owned(),
        );
    }
    let mut adapter = CentralRelativeAdapter::new(&bearings1, &bearings2).map_err(vision_error)?;
    let backend = RustNumericalBackend;
    let problem =
        CentralRelativePoseSacProblem::new(&mut adapter, Algorithm::Stewenius, &backend, true);
    let mut ransac = Ransac::new(problem, 100, threshold, 0.99);
    if !ransac.compute_model(0) {
        return Err(Error::new(
            ErrorKind::Convergence,
            "relative pose RANSAC found no model",
        ));
    }
    let model = ransac.model_coefficients.ok_or_else(|| {
        Error::new(
            ErrorKind::Convergence,
            "relative pose RANSAC produced no coefficients",
        )
    })?;
    let ransac_count = ransac.inliers.len();
    ransac.sac_model.adapter.t12 = model.column(3).into_owned();
    ransac.sac_model.adapter.r12 = model.fixed_view::<3, 3>(0, 0).into_owned();
    let indices = ransac.inliers.clone();
    let refined = methods::optimize_nonlinear(ransac.sac_model.adapter, &indices, &backend)
        .ok_or_else(|| {
            Error::new(
                ErrorKind::Convergence,
                "relative pose nonlinear refinement failed",
            )
        })?;
    let inliers = ransac.sac_model.select_within_distance(&refined, threshold);
    if inliers.len() + 2 < ransac_count {
        log::warn!(
            "nonlinear refinement reduced relative-pose inliers from {ransac_count} to {}",
            inliers.len()
        );
    }
    matches.t_i_j = Isometry3::from_parts(
        Translation3::from(refined.column(3).normalize()),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
            refined.fixed_view::<3, 3>(0, 0).into_owned(),
        )),
    );
    if inliers.len() >= min_inliers {
        matches
            .inliers
            .extend(inliers.into_iter().map(|i| matches.matches[i]));
    }
    Ok(())
}
