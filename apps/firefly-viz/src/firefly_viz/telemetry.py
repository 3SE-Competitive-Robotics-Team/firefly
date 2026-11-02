"""诊断曲线的分量名称；数值数组顺序与发布端及验收契约一致。"""

import rerun as rr


QUATERNION = ("x", "y", "z", "w")
MOTORS = ("motor_1_N", "motor_2_N", "motor_3_N", "motor_4_N")
IMU_HEALTH = ("attitude_available", "vio_fresh", "posterior_age_s")
SCALAR_FIELDS = {
    "okvis/gyro_bias": ("x_rad_s", "y_rad_s", "z_rad_s"),
    "fc/standby_eskf/gyro_bias": ("x_rad_s", "y_rad_s", "z_rad_s"),
    "fc/standby_eskf/quaternion_xyzw": QUATERNION,
    "fc/standby_eskf/attitude_variance": ("x_rad2", "y_rad2", "z_rad2"),
    "fc/standby_eskf/bias_variance": ("x_rad2_s2", "y_rad2_s2", "z_rad2_s2"),
    "fc/debug/position_target_odom": ("x_m", "y_m", "z_m"),
    "fc/debug/position_error_odom": ("x_m", "y_m", "z_m"),
    "fc/debug/motors": MOTORS,
    "fc/debug/vertical_ned": ("vel_sp_z", "vel_z", "acc_sp_z", "thr_sp_z"),
    "fc/debug/rate_sp_frd": ("roll_rad_s", "pitch_rad_s", "yaw_rad_s"),
    "fc/debug/rate_flu": ("x_rad_s", "y_rad_s", "z_rad_s"),
    "fc/debug/torque_sp_frd": ("roll_normalized", "pitch_normalized", "yaw_normalized"),
    "fc/debug/allocated_control": ("roll_normalized", "pitch_normalized", "yaw_normalized", "thrust_z_normalized"),
    "render/capture/stages_s": ("submit_s", "readback_s", "assemble_s", "queue_s"),
    "render/capture/publish_s": ("process_s", "publish_s", "total_s"),
    "corr/debug/drift": ("x_m", "y_m", "z_m"),
    "acceptance/imu_attitude": QUATERNION,
    "acceptance/imu_health": IMU_HEALTH,
    "acceptance/motors": MOTORS,
    "acceptance/odom_velocity": ("x_m_s", "y_m_s", "z_m_s"),
    "acceptance/gt_velocity": ("x_m_s", "y_m_s", "z_m_s"),
    "acceptance/corrected_velocity": ("x_m_s", "y_m_s", "z_m_s"),
    "acceptance/reference_velocity": ("x_m_s", "y_m_s", "z_m_s"),
    "acceptance/physics": ("physics_steps", "contact_steps", "collisions", "resets"),
    "acceptance/visual_update": ("gate_metric", "gate_limit", "applied_translation_m", "accepted"),
    "acceptance/hover_failure/state": ("flight_mode", "has_odometry", "has_control_origin"),
    "acceptance/hover_failure/freshness": ("measurement_age_s", "receive_age_s", "initialized"),
    "acceptance/hover_failure/error": ("horizontal_m", "vertical_m", "speed_m_s"),
    "hil/link": ("control", "odometry", "telemetry", "dropped_images"),
    "hil/camera/left": ("timestamp_s", "pixel_sum", "width", "height"),
    "hil/camera/right": ("timestamp_s", "pixel_sum", "width", "height"),
    "hil/board": ("imu_gap_s",),
    "viz/queues": ("telemetry_pending", "media_pending", "telemetry_high_water", "dropped_frames"),
    "viz/telemetry/overflow": ("overflow", "high_water"),
}


def scalar_labels(name, size):
    fields = SCALAR_FIELDS.get(name)
    if name.startswith("acceptance/disturbance_force/"):
        fields = ("world_x_N", "world_y_N", "world_z_N", "duration_s")
    if name.startswith("viz/video/"):
        if name.endswith("/encoded"):
            fields = ("bytes", "latency_s", "frame_index")
        elif name.endswith("/frames"):
            fields = ("accepted", "encoded", "encoder_dropped", "capture_skipped")
    if fields is None:
        if size != 1:
            raise ValueError(f"scalar channels need semantic names: {name} ({size})")
        return [name]
    if len(fields) != size:
        raise ValueError(f"invalid scalar channel count for {name}: {size}")
    return [f"{name}/{field}" for field in fields]


def log_scalars(recording, name, values):
    """显式命名曲线，保留原始数组供验收读取。"""
    recording.log(name, rr.Scalars(values), rr.SeriesLines(names=scalar_labels(name, len(values))), strict=True)
