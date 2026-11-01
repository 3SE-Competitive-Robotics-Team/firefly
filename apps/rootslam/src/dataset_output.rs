//! Headless output formats and local-estimator statistics from original `vio.cpp`.
use firefly_basalt::vi_estimator::vio_estimator::align_svd;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Isometry3, Vector3};
use serde::{Deserialize, Serialize};
use std::io::Write;

#[derive(Clone, Copy, Debug, Default, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum TrajectoryFormat {
    #[default]
    Tum,
    Euroc,
    Kitti,
}

/// Original result fields; frame count includes the entire input dataset.
#[derive(Debug, Serialize)]
pub struct OriginalResult {
    pub rms_ate: f64,
    pub num_frames: usize,
    pub exec_time_ns: u128,
}

/// Build the original `stats_vio` samples using the existing archive schema.
#[must_use]
#[expect(
    clippy::cast_precision_loss,
    reason = "Original ExecutionStats stores all scalar samples as doubles"
)]
pub fn original_statistics(
    duration: std::time::Duration,
    rms_ate: f64,
    num_states: usize,
    num_frames: usize,
    resident_memory_peak: Option<u64>,
) -> firefly_basalt::utils::time_utils::ExecutionStats {
    let mut stats = firefly_basalt::utils::time_utils::ExecutionStats::default();
    stats.add("exec_time_s", duration.as_secs_f64());
    stats.add("ate_rmse", rms_ate);
    stats.add("ate_num_kfs", num_states as f64);
    stats.add("num_frames", num_frames as f64);
    if let Some(peak) = resident_memory_peak {
        stats.add("resident_memory_peak", peak as f64);
    }
    stats
}

/// Align the complete GT translation stream to the local estimator stream.
/// # Errors
/// Returns an error for malformed or non-overlapping trajectories.
pub fn align_ground_truth(
    estimates: &[(i64, Isometry3<f64>)],
    gt_times: &[i64],
    gt_positions: &mut [Vector3<f64>],
) -> Result<f64> {
    if gt_times.len() != gt_positions.len()
        || gt_times.windows(2).any(|w| w[0] >= w[1])
        || !estimates.iter().any(|(time, _)| {
            let upper = gt_times.partition_point(|gt| gt <= time);
            upper > 0
                && upper < gt_times.len()
                && gt_times[upper] - gt_times[upper - 1] <= 110_000_000
        })
    {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "Original Basalt alignment requires ordered GT and timestamp associations",
        ));
    }
    let times: Vec<_> = estimates.iter().map(|entry| entry.0).collect();
    let positions: Vec<_> = estimates
        .iter()
        .map(|entry| entry.1.translation.vector)
        .collect();
    Ok(align_svd(&times, &positions, gt_times, gt_positions))
}

/// Original TUM, `EuRoC` and KITTI numeric conventions.
/// # Errors
/// Returns output I/O errors.
#[expect(
    clippy::cast_precision_loss,
    reason = "Original TUM timestamps convert nanoseconds to double seconds"
)]
pub fn write_trajectory(
    output: &mut impl Write,
    estimates: &[(i64, Isometry3<f64>)],
    format: TrajectoryFormat,
) -> std::io::Result<()> {
    match format {
        TrajectoryFormat::Tum => writeln!(output, "# timestamp tx ty tz qx qy qz qw")?,
        TrajectoryFormat::Euroc => writeln!(
            output,
            "#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z []"
        )?,
        TrajectoryFormat::Kitti => {}
    }
    for (time, pose) in estimates {
        let t = pose.translation.vector;
        let q = pose.rotation.quaternion();
        match format {
            TrajectoryFormat::Tum => writeln!(
                output,
                "{:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e}",
                *time as f64 * 1e-9,
                t.x,
                t.y,
                t.z,
                q.i,
                q.j,
                q.k,
                q.w
            )?,
            TrajectoryFormat::Euroc => writeln!(
                output,
                "{time},{:.18e},{:.18e},{:.18e},{:.18e},{:.18e},{:.18e},{:.18e}",
                t.x, t.y, t.z, q.w, q.i, q.j, q.k
            )?,
            TrajectoryFormat::Kitti => {
                let matrix = firefly_basalt::utils::sophus_utils::se3_matrix(pose);
                for row in 0..3 {
                    for col in 0..4 {
                        write!(output, "{:.12e} ", matrix[(row, col)])?;
                    }
                }
                writeln!(output)?;
            }
        }
    }
    Ok(())
}
