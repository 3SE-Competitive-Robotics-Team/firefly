//! Scalar field order is part of the diagnostic wire contract.
use firefly_error::Result;
use firefly_pubsub::viz::kind;

/// Recorder-derived display documents; no additional IPC topics or raw-series filtering.
pub(crate) const STATUS_ENTITIES: [&str; 6] = [
    "mimir/status/fc/tick_rate",
    "mimir/status/vio/processing",
    "mimir/status/vio/posterior_age",
    "mimir/status/camera/publish_rate",
    "mimir/status/camera/publish_latency",
    "mimir/status/vio/landmarks",
];

const SPATIAL: &[(&str, &[u32])] = &[
    ("vio/odom", &[kind::POSE]),
    ("vio/map/pose", &[kind::POSE]),
    ("gt/pose", &[kind::POSE]),
    ("corr/odom", &[kind::POSE]),
    ("plan/drone", &[kind::POSE]),
    ("vio/posterior/pose_imu_world", &[kind::POSE]),
    ("vio/prediction/pose_body_odom", &[kind::POSE]),
    ("acceptance/odom", &[kind::POSE]),
    ("acceptance/gt", &[kind::POSE]),
    ("acceptance/corrected", &[kind::POSE]),
    ("acceptance/reference", &[kind::POSE]),
    ("vio/traj", &[kind::LINE_STRIP]),
    ("vio/map/traj", &[kind::LINE_STRIP]),
    ("gt/traj", &[kind::LINE_STRIP]),
    ("corr/traj", &[kind::LINE_STRIP]),
    ("plan/global_path", &[kind::LINE_STRIP]),
    ("plan/local_traj", &[kind::LINE_STRIP]),
    (
        "plan/collision",
        &[kind::VOXELS, kind::VOXELS_ADD, kind::VOXELS_REMOVE],
    ),
    ("plan/local_traj/velocity", &[kind::ARROWS]),
    ("plan/planes", &[kind::ARROWS]),
    ("vio/diagnostics/keypoints/left", &[kind::KEYPOINTS_2D]),
    ("vio/diagnostics/keypoints/right", &[kind::KEYPOINTS_2D]),
    (
        "vio/diagnostics/keypoints/active_left",
        &[kind::KEYPOINTS_2D],
    ),
    ("cameras/left", &[kind::CAMERA]),
    ("cameras/right", &[kind::CAMERA]),
];

/// Parent frame for registered 3D observations; pixel overlays keep camera coordinates.
pub(crate) fn frame(name: &str) -> Option<&'static str> {
    match name {
        "gt/pose" | "gt/traj" | "acceptance/gt" => Some("world"),
        "vio/odom"
        | "vio/traj"
        | "vio/posterior/pose_imu_world"
        | "vio/prediction/pose_body_odom"
        | "acceptance/odom"
        | "plan/drone"
        | "plan/global_path"
        | "plan/local_traj"
        | "plan/collision"
        | "plan/local_traj/velocity"
        | "plan/planes"
        | "acceptance/reference" => Some("odom"),
        "vio/map/pose" | "vio/map/traj" | "corr/odom" | "corr/traj" | "acceptance/corrected" => {
            Some("map")
        }
        "cameras/left" | "cameras/right" => Some("tf#/vio/prediction/pose_body_odom"),
        _ => None,
    }
}

/// Validate the registered entity and payload before serialization.
/// # Errors
/// Unknown path or unsupported payload kind.
pub fn spatial(name: &str, payload: u32) -> Result<()> {
    let accepted = if payload == kind::SCALARS {
        fields(name).is_ok()
    } else if payload == kind::CLEAR {
        matches!(name, "gt" | "corr" | "plan" | "vio" | "acceptance")
            || SPATIAL.iter().any(|(path, _)| *path == name)
            || fields(name).is_ok()
    } else {
        SPATIAL
            .iter()
            .any(|(path, kinds)| *path == name && kinds.contains(&payload))
    };
    if accepted {
        Ok(())
    } else {
        Err(super::invalid(format!(
            "unregistered entity or payload: {name} ({payload})"
        )))
    }
}

/// Decode a bounded UTF-8 entity path.
/// # Errors
/// Invalid length, UTF-8, or path segments.
pub fn entity(bytes: &[u8], length: u32) -> Result<&str> {
    let value = bytes
        .get(..length as usize)
        .ok_or_else(|| super::invalid("entity length"))?;
    let name = std::str::from_utf8(value).map_err(|e| super::invalid(e.to_string()))?;
    if name.is_empty()
        || name.contains('\0')
        || name
            .split('/')
            .any(|part| matches!(part, "" | "." | "..") || part.starts_with("__"))
    {
        return Err(super::invalid("invalid entity path"));
    }
    Ok(name)
}

/// Numeric field names and units in wire order.
/// # Errors
/// Unregistered entity name.
#[allow(clippy::too_many_lines)] // Keep the field registry in one table.
pub fn fields(name: &str) -> Result<&'static [&'static str]> {
    let fields: &[&str] = match name {
        "vio/diagnostics/tracks" => &[
            "left_detected",
            "continued_ids",
            "stereo_ids",
            "landmark_ids",
        ],
        "vio/diagnostics/graph" => &["landmarks", "observations", "states", "keyframe_poses"],
        "vio/diagnostics/keyframes" => &[
            "is_keyframe",
            "continued_landmarks",
            "new_landmarks",
            "window_span_s",
        ],
        "vio/diagnostics/residual_before_px" | "vio/diagnostics/residual_after_px" => {
            &["count", "rms_px", "mean_x_px", "mean_y_px"]
        }
        "vio/diagnostics/residual_rms_px" => &["before_px", "after_px"],
        "vio/diagnostics/stereo_ratio" => &["shared_ids_per_left"],
        "vio/diagnostics/position_delta_m"
        | "fc/debug/position_target_odom"
        | "fc/debug/position_error_odom"
        | "corr/debug/drift" => &["x_m", "y_m", "z_m"],
        "vio/diagnostics/velocity_delta_m_s"
        | "vio/posterior/velocity_world"
        | "vio/prediction/velocity_body_odom"
        | "acceptance/odom_velocity"
        | "acceptance/gt_velocity"
        | "acceptance/corrected_velocity"
        | "acceptance/reference_velocity" => &["x_m_s", "y_m_s", "z_m_s"],
        "vio/diagnostics/rotation_delta_rad" => &["x_rad", "y_rad", "z_rad"],
        "vio/diagnostics/costs_before" | "vio/diagnostics/costs_after" => {
            &["vision", "inertial", "bias_random_walk", "marginal_prior"]
        }
        "vio/diagnostics/solver_termination" => {
            &["attempts", "rejected", "reason", "last_step_inf_norm"]
        }
        "vio/diagnostics/yaw_updates_rad" => &[
            "latest_world_z_rad",
            "oldest_world_z_rad",
            "window_mean_world_z_rad",
            "window_stddev_world_z_rad",
        ],
        "vio/input/gyro_rad_s"
        | "vio/posterior/gyro_bias_imu"
        | "vio/prediction/angular_velocity_body"
        | "vio/gyro_bias"
        | "fc/standby_eskf/gyro_bias"
        | "fc/debug/rate_flu" => &["x_rad_s", "y_rad_s", "z_rad_s"],
        "vio/input/accel_m_s2" | "vio/posterior/accel_bias_imu" => &["x_m_s2", "y_m_s2", "z_m_s2"],
        "vio/prediction/posterior_age_s"
        | "fc/imu/posterior_age_s"
        | "fc/standby_eskf/imu_age_s" => &["age_s"],
        "fc/loop/timing" => &[
            "tick",
            "feedback_age_ms",
            "receive_age_ms",
            "tick_interval_ms",
        ],
        "fc/loop/roll" | "fc/loop/pitch" => &[
            "target_rad_s",
            "feedback_rad_s",
            "raw_gyro_rad_s",
            "torque_normalized",
        ],
        "fc/loop/control" => &[
            "thrust_N",
            "saturated",
            "allocated_roll_normalized",
            "attitude_source",
        ],
        "fc/loop/motors" | "fc/debug/motors" | "acceptance/motors" => {
            &["motor_1_N", "motor_2_N", "motor_3_N", "motor_4_N"]
        }
        "fc/imu/quaternion_xyzw"
        | "fc/standby_eskf/quaternion_xyzw"
        | "acceptance/imu_attitude" => &["x", "y", "z", "w"],
        "fc/imu/health" | "acceptance/imu_health" => {
            &["attitude_available", "vio_fresh", "posterior_age_s"]
        }
        "fc/standby_eskf/attitude_variance" => &["x_rad2", "y_rad2", "z_rad2"],
        "fc/standby_eskf/bias_variance" => &["x_rad2_s2", "y_rad2_s2", "z_rad2_s2"],
        "fc/debug/rate_sp_frd" => &["roll_rad_s", "pitch_rad_s", "yaw_rad_s"],
        "fc/debug/torque_sp_frd" => &["roll_normalized", "pitch_normalized", "yaw_normalized"],
        "fc/debug/allocated_control" => &[
            "roll_normalized",
            "pitch_normalized",
            "yaw_normalized",
            "thrust_z_normalized",
        ],
        "render/capture/stages_s" => &["submit_s", "readback_s", "assemble_s", "queue_s"],
        "render/capture/publish_s" => &["process_s", "publish_s", "total_s"],
        "corr/debug/gate" | "acceptance/visual_update" => {
            &["metric", "limit", "applied_translation_m", "accepted"]
        }
        "corr/debug/map_observation" => &["visual_age_s", "position_disagreement_m", "consistent"],
        "acceptance/physics" => &["physics_steps", "contact_steps", "collisions", "resets"],
        "acceptance/hover_failure/state" => &["flight_mode", "has_odometry", "has_control_origin"],
        "acceptance/hover_failure/freshness" => {
            &["measurement_age_s", "receive_age_s", "initialized"]
        }
        "acceptance/hover_failure/error" => &["horizontal_m", "vertical_m", "speed_m_s"],
        "rootslam/frame_processing_ms" | "rootslam/ipc/imu_publish_ms" | "fc/debug/late_max_ms" => {
            &["duration_ms"]
        }
        "fc/debug/altitude" | "acceptance/penetration" => &["distance_m"],
        "fc/debug/tilt_deg" | "fc/debug/tilt_err_deg" => &["angle_deg"],
        "fc/debug/thrust_total" => &["thrust_N"],
        "fc/debug/hover_thrust" => &["normalized"],
        "fc/debug/tick_rate" => &["rate_Hz"],
        "vision/debug/map_query_wall_s" => &["query_s"],
        "fc/standby_eskf/accel_nis" => &["nis"],
        "fc/debug/saturated_ticks" | "acceptance/loop_backend" => &["count"],
        "fc/arming/failed" | "fc/failsafe/flags" => &["mask"],
        "fc/debug/state"
        | "fc/debug/saturated"
        | "fc/failsafe/action"
        | "fc/imu/source"
        | "fc/standby_eskf/accel_gate"
        | "fc/standby_eskf/health"
        | "acceptance/fc_state"
        | "acceptance/attitude_source" => &["state"],
        _ if name
            .strip_prefix("acceptance/disturbance_force/")
            .is_some_and(|suffix| {
                (2..=3).contains(&suffix.len()) && suffix.bytes().all(|v| v.is_ascii_digit())
            }) =>
        {
            &["world_x_N", "world_y_N", "world_z_N", "duration_s"]
        }
        _ => {
            return Err(super::invalid(format!(
                "unregistered scalar entity: {name}"
            )));
        }
    };
    Ok(fields)
}

/// Convert seconds to the RRD nanosecond index.
/// # Errors
/// Negative, nonfinite or out-of-range timestamps.
pub fn timestamp(stamp: f64) -> Result<i64> {
    if !stamp.is_finite() || stamp < 0. || stamp >= i64::MAX as f64 / 1e9 {
        return Err(super::invalid("invalid measurement timestamp"));
    }
    Ok((stamp * 1e9).round() as i64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_registered_3d_payloads_have_explicit_frames() {
        for (name, kinds) in SPATIAL {
            if kinds.iter().any(|kind| {
                matches!(
                    *kind,
                    kind::POSE | kind::LINE_STRIP | kind::ARROWS | kind::VOXELS | kind::CAMERA
                )
            }) {
                assert!(frame(name).is_some(), "{name}");
            }
        }
        assert_ne!(frame("acceptance/gt"), frame("acceptance/odom"));
        assert_eq!(frame("plan/local_traj/velocity"), Some("odom"));
    }

    #[test]
    fn unknown_entities_and_wrong_payloads_fail_closed() {
        assert!(fields("unregistered/value").is_err());
        assert!(spatial("vio/prediction/pose_body_odom", kind::POSE).is_ok());
        assert!(spatial("vio/prediction/pose_body_odom", kind::ARROWS).is_err());
        assert!(spatial("unknown", kind::CLEAR).is_err());
        assert!(spatial("plan", kind::CLEAR).is_ok());
        for payload in [
            kind::VOXELS,
            kind::VOXELS_ADD,
            kind::VOXELS_REMOVE,
            kind::CLEAR,
        ] {
            assert!(spatial("plan/collision", payload).is_ok());
        }
        assert!(spatial("plan/collision", kind::POSE).is_err());
        assert!(fields("acceptance/disturbance_force/01").is_ok());
        assert!(fields("acceptance/disturbance_force/1234").is_err());
        assert!(entity(b"../invalid", 10).is_err());
        assert!(entity(b"valid", 9).is_err());
    }

    #[test]
    fn invalid_measurement_time_is_rejected() {
        for value in [-1., f64::NAN, f64::INFINITY, i64::MAX as f64] {
            assert!(timestamp(value).is_err());
        }
        assert_eq!(timestamp(0.).unwrap(), 0);
        assert_eq!(timestamp(0.000_000_001_5).unwrap(), 2);
    }
}
