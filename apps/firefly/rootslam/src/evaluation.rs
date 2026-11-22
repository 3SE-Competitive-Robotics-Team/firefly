//! Final-trajectory metrics: rigid SE(3) alignment and six-camera-frame RTE.
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Isometry3, Matrix3, Translation3, Vector3};
use serde::Serialize;
use std::collections::BTreeMap;

#[derive(Clone, Debug, Serialize)]
pub struct Metrics {
    pub camera_frames: usize,
    pub estimated_frames: usize,
    pub associated_frames: usize,
    pub relative_pairs: usize,
    pub success_fraction: f64,
    pub ate_rmse_m: f64,
    pub rte_rmse_m: Option<f64>,
}

fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("rootslam.trajectory.evaluate")
}
fn interpolate(
    times: &[i64],
    poses: &[Isometry3<f64>],
    time: i64,
    max_gap_ns: i64,
) -> Option<Isometry3<f64>> {
    match times.binary_search(&time) {
        Ok(index) => Some(poses[index]),
        Err(index) if index > 0 && index < times.len() => {
            let gap = times[index] - times[index - 1];
            if gap > max_gap_ns {
                return None;
            }
            #[expect(
                clippy::cast_precision_loss,
                reason = "Interpolation of nanosecond intervals in double precision"
            )]
            let alpha = (time - times[index - 1]) as f64 / gap as f64;
            Some(Isometry3::from_parts(
                Translation3::from(
                    poses[index - 1].translation.vector * (1.0 - alpha)
                        + poses[index].translation.vector * alpha,
                ),
                poses[index - 1]
                    .rotation
                    .slerp(&poses[index].rotation, alpha),
            ))
        }
        _ => None,
    }
}

/// Evaluates the final non-causal trajectory at original camera timestamps.
/// RTE uses a separation of six camera frames, preserving gaps in estimated
/// frames rather than interpreting six surviving estimates as six inputs.
/// `max_ground_truth_gap_ns` controls interpolation across unavailable ground
/// truth and is an evaluation choice, not a threshold specified by the paper.
/// # Errors
/// Rejects empty/unsorted inputs, invalid interpolation bounds, non-finite poses,
/// and fewer than three associated samples for rigid alignment.
pub fn evaluate(
    camera_times: &[i64],
    estimates: &BTreeMap<i64, Isometry3<f64>>,
    ground_truth_times: &[i64],
    ground_truth: &[Isometry3<f64>],
    max_ground_truth_gap_ns: i64,
) -> Result<Metrics> {
    if camera_times.is_empty()
        || ground_truth_times.len() != ground_truth.len()
        || max_ground_truth_gap_ns <= 0
        || camera_times.windows(2).any(|pair| pair[0] >= pair[1])
        || ground_truth_times.windows(2).any(|pair| pair[0] >= pair[1])
    {
        return Err(invalid(
            "Invalid trajectory timestamps or interpolation limit",
        ));
    }
    if estimates
        .values()
        .chain(ground_truth.iter())
        .any(|pose| pose.to_homogeneous().iter().any(|v| !v.is_finite()))
    {
        return Err(invalid("Trajectory contains non-finite poses"));
    }
    let paired: Vec<_> = camera_times
        .iter()
        .map(|&time| {
            Some((
                *estimates.get(&time)?,
                interpolate(
                    ground_truth_times,
                    ground_truth,
                    time,
                    max_ground_truth_gap_ns,
                )?,
            ))
        })
        .collect();
    let associations: Vec<_> = paired.iter().flatten().collect();
    if associations.len() < 3 {
        return Err(invalid("Fewer than three ground-truth associations"));
    }
    let ate_rmse_m = aligned_absolute_error(&associations)?;
    let mut relative_sum = 0.0;
    let mut relative_pairs = 0_usize;
    for index in 0..paired.len().saturating_sub(6) {
        if let (Some((first, truth_first)), Some((second, truth_second))) =
            (&paired[index], &paired[index + 6])
        {
            let estimated_relative = first.inverse() * second;
            let truth_relative = truth_first.inverse() * truth_second;
            let error = truth_relative.inverse() * estimated_relative;
            relative_sum += error.translation.vector.norm_squared();
            relative_pairs += 1;
        }
    }
    let estimated_frames = camera_times
        .iter()
        .filter(|time| estimates.contains_key(time))
        .count();
    #[expect(
        clippy::cast_precision_loss,
        reason = "Trajectory counts for success fraction and RMS"
    )]
    let (success_fraction, rte_rmse_m) = (
        estimated_frames as f64 / camera_times.len() as f64,
        (relative_pairs > 0).then(|| (relative_sum / relative_pairs as f64).sqrt()),
    );
    Ok(Metrics {
        camera_frames: camera_times.len(),
        estimated_frames,
        associated_frames: associations.len(),
        relative_pairs,
        success_fraction,
        ate_rmse_m,
        rte_rmse_m,
    })
}

fn aligned_absolute_error(associations: &[&(Isometry3<f64>, Isometry3<f64>)]) -> Result<f64> {
    #[expect(
        clippy::cast_precision_loss,
        reason = "Trajectory sample count for statistical means"
    )]
    let count = associations.len() as f64;
    let estimate_mean = associations
        .iter()
        .map(|(estimate, _)| estimate.translation.vector)
        .sum::<Vector3<f64>>()
        / count;
    let truth_mean = associations
        .iter()
        .map(|(_, truth)| truth.translation.vector)
        .sum::<Vector3<f64>>()
        / count;
    let cross = associations
        .iter()
        .fold(Matrix3::zeros(), |sum, (estimate, truth)| {
            sum + (truth.translation.vector - truth_mean)
                * (estimate.translation.vector - estimate_mean).transpose()
        });
    let decomposition = cross.svd(true, true);
    let (Some(left), Some(right)) = (decomposition.u, decomposition.v_t) else {
        return Err(invalid("Rigid alignment decomposition failed"));
    };
    let mut sign = Matrix3::identity();
    sign[(2, 2)] = (left * right).determinant().signum();
    let rotation = left * sign * right;
    let translation = truth_mean - rotation * estimate_mean;
    let absolute_sum = associations
        .iter()
        .map(|(estimate, truth)| {
            (rotation * estimate.translation.vector + translation - truth.translation.vector)
                .norm_squared()
        })
        .sum::<f64>();
    Ok((absolute_sum / count).sqrt())
}
