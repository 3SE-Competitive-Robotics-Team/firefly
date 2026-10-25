//! 解锁前检查与失效标志（对照 PX4 `ArmingCheck` / `FailsafeFlags` 的语义）。
//!
//! 本产品没有 RC / GCS / 电池 / 地理围栏 / GNSS / mission 输入，因此枚举只覆盖
//! 这台飞机实际能评估的检查与失效条件；PX4 中依赖这些输入的位不会伪造为“永远通过”。
//!
//! 记录形式：`ArmingCheck` 与 `FailsafeFlags` 都是稳定的位掩码（位号 = 枚举判别值），
//! `apps/fc` 通过 IPC 发布 u32 位掩码，viz 将其转换为具名状态时间线。

use crate::fsm::Health;

/// 解锁前逐项检查；顺序即首次失败的上报顺序（对照 PX4 `ArmingCheck`）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ArmingCheck {
    /// 必须处于 `DISARMED`。
    Disarmed = 0,
    /// 状态估计已初始化且主姿态链路新鲜。
    EstimatorReady = 1,
    /// 姿态会话未改变（VIO 重启后不能沿用旧会话）。
    AttitudeSession = 2,
    /// 已收到机体/执行器描述。
    AirframeReady = 3,
    /// 里程计新鲜。
    OdometryAlive = 4,
    /// IMU 新鲜。
    ImuAlive = 5,
    /// 停在起飞点地面。
    OnGround = 6,
}

impl ArmingCheck {
    /// 全部检查，按上报顺序排列。
    pub const ALL: [Self; 7] = [
        Self::Disarmed,
        Self::EstimatorReady,
        Self::AttitudeSession,
        Self::AirframeReady,
        Self::OdometryAlive,
        Self::ImuAlive,
        Self::OnGround,
    ];

    /// 位号即枚举判别值，供 RRD 位掩码记录。
    #[must_use]
    pub const fn bit(self) -> u32 {
        1 << (self as u32)
    }

    /// 稳定整数编码（顺序即编码，不得重排）。
    #[must_use]
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// 日志/诊断用简称。
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Disarmed => "DISARMED",
            Self::EstimatorReady => "ESTIMATOR_READY",
            Self::AttitudeSession => "ATTITUDE_SESSION",
            Self::AirframeReady => "AIRFRAME_READY",
            Self::OdometryAlive => "ODOMETRY_ALIVE",
            Self::ImuAlive => "IMU_ALIVE",
            Self::OnGround => "ON_GROUND",
        }
    }

    /// 一行中文原因（进日志与 RRD）。
    #[must_use]
    pub const fn reason(self) -> &'static str {
        match self {
            Self::Disarmed => "当前不在停机状态",
            Self::EstimatorReady => "状态估计未就绪",
            Self::AttitudeSession => "VIO 会话已改变",
            Self::AirframeReady => "未收到机体描述",
            Self::OdometryAlive => "里程计陈旧",
            Self::ImuAlive => "IMU 陈旧",
            Self::OnGround => "不在起飞点地面",
        }
    }
}

/// 失效条件位掩码（对照 PX4 `FailsafeFlags.msg` 的产品子集）。
///
/// 只保留本产品能评估的项；PX4 的 RC/GCS/电池/地理围栏/GNSS/mission/失控检测
/// 没有对应输入，故不设位，避免“永远为假”的假通过。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct FailsafeFlags(u32);

impl FailsafeFlags {
    /// 姿态不可用（估计器未就绪或会话失效）。
    pub const ATTITUDE_INVALID: Self = Self(1 << 0);
    /// 局部位置不可用（里程计陈旧）。
    pub const LOCAL_POSITION_INVALID: Self = Self(1 << 1);
    /// 局部速度不可用（与位置同源）。
    pub const LOCAL_VELOCITY_INVALID: Self = Self(1 << 2);
    /// IMU 陈旧。
    pub const IMU_STALE: Self = Self(1 << 3);
    /// 未收到机体描述。
    pub const AIRFRAME_MISSING: Self = Self(1 << 4);
    /// VIO 会话改变（不可恢复，需重启 FC）。
    pub const ATTITUDE_SESSION_CHANGED: Self = Self(1 << 5);
    /// `Track` 下参考流暂时缺席（尚未到失联超时）。
    pub const REFERENCE_STALE: Self = Self(1 << 6);
    /// `Track` 下参考流超过失联超时。
    pub const REFERENCE_LOST: Self = Self(1 << 7);

    /// 已定义的位数。
    pub const COUNT: u32 = 8;

    /// 由健康电平与参考流状态构造。
    #[must_use]
    pub fn from_health(health: Health, reference_stale: bool, reference_lost: bool) -> Self {
        let mut flags = Self::default();
        if !health.estimator_ready || !health.attitude_session_ok {
            flags.insert(Self::ATTITUDE_INVALID);
        }
        if !health.odometry_alive {
            flags.insert(Self::LOCAL_POSITION_INVALID);
            flags.insert(Self::LOCAL_VELOCITY_INVALID);
        }
        if !health.imu_alive {
            flags.insert(Self::IMU_STALE);
        }
        if !health.airframe_ready {
            flags.insert(Self::AIRFRAME_MISSING);
        }
        if !health.attitude_session_ok {
            flags.insert(Self::ATTITUDE_SESSION_CHANGED);
        }
        if reference_stale {
            flags.insert(Self::REFERENCE_STALE);
        }
        if reference_lost {
            flags.insert(Self::REFERENCE_LOST);
        }
        flags
    }

    /// 原始位掩码（RRD 记录用）。
    #[must_use]
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// 是否没有任何失效位。
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// 是否包含某一位。
    #[must_use]
    pub const fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 != 0
    }

    /// 置位。
    pub fn insert(&mut self, flag: Self) {
        self.0 |= flag.0;
    }

    /// 已置位的条件名（日志用）。
    pub fn names(self) -> impl Iterator<Item = &'static str> {
        [
            (Self::ATTITUDE_INVALID, "ATTITUDE_INVALID"),
            (Self::LOCAL_POSITION_INVALID, "LOCAL_POSITION_INVALID"),
            (Self::LOCAL_VELOCITY_INVALID, "LOCAL_VELOCITY_INVALID"),
            (Self::IMU_STALE, "IMU_STALE"),
            (Self::AIRFRAME_MISSING, "AIRFRAME_MISSING"),
            (Self::ATTITUDE_SESSION_CHANGED, "ATTITUDE_SESSION_CHANGED"),
            (Self::REFERENCE_STALE, "REFERENCE_STALE"),
            (Self::REFERENCE_LOST, "REFERENCE_LOST"),
        ]
        .into_iter()
        .filter(move |(flag, _)| self.contains(*flag))
        .map(|(_, name)| name)
    }
}

/// 失效动作（对照 PX4 `FailsafeState` 的产品子集）。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FailsafeAction {
    /// 无失效。
    #[default]
    None,
    /// 参考丢失，回落位置保持。
    Hold,
    /// 估计失效，锁存姿态支持。
    AttitudeFallback,
    /// 地面估计失效，上锁。
    Disarm,
}

impl FailsafeAction {
    /// 稳定整数编码（IPC 诊断）。
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            Self::None => 0,
            Self::Hold => 1,
            Self::AttitudeFallback => 2,
            Self::Disarm => 3,
        }
    }

    /// 日志用简称。
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::None => "NONE",
            Self::Hold => "HOLD",
            Self::AttitudeFallback => "ATTITUDE_FALLBACK",
            Self::Disarm => "DISARM",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy() -> Health {
        Health {
            estimator_ready: true,
            airframe_ready: true,
            odometry_alive: true,
            imu_alive: true,
            attitude_session_ok: true,
        }
    }

    #[test]
    fn arming_check_bits_are_unique_and_ordered() {
        let mut seen = 0u32;
        for (index, check) in ArmingCheck::ALL.iter().enumerate() {
            assert_eq!(check.code() as usize, index);
            assert_eq!(check.bit(), 1 << index);
            assert_eq!(seen & check.bit(), 0);
            seen |= check.bit();
            assert_ne!(check.name(), "");
            assert_ne!(check.reason(), "");
        }
        assert_eq!(seen.count_ones(), ArmingCheck::ALL.len() as u32);
    }

    #[test]
    fn healthy_inputs_set_no_flags() {
        let flags = FailsafeFlags::from_health(healthy(), false, false);
        assert!(flags.is_empty(), "{:?}", flags.names().collect::<Vec<_>>());
    }

    #[test]
    fn each_failure_sets_its_own_flag() {
        let mut health = healthy();
        health.estimator_ready = false;
        let flags = FailsafeFlags::from_health(health, false, false);
        assert!(flags.contains(FailsafeFlags::ATTITUDE_INVALID));

        let mut health = healthy();
        health.attitude_session_ok = false;
        let flags = FailsafeFlags::from_health(health, false, false);
        assert!(flags.contains(FailsafeFlags::ATTITUDE_INVALID));
        assert!(flags.contains(FailsafeFlags::ATTITUDE_SESSION_CHANGED));

        let mut health = healthy();
        health.odometry_alive = false;
        let flags = FailsafeFlags::from_health(health, false, false);
        assert!(flags.contains(FailsafeFlags::LOCAL_POSITION_INVALID));
        assert!(flags.contains(FailsafeFlags::LOCAL_VELOCITY_INVALID));

        let mut health = healthy();
        health.imu_alive = false;
        let flags = FailsafeFlags::from_health(health, false, false);
        assert!(flags.contains(FailsafeFlags::IMU_STALE));

        let mut health = healthy();
        health.airframe_ready = false;
        let flags = FailsafeFlags::from_health(health, false, false);
        assert!(flags.contains(FailsafeFlags::AIRFRAME_MISSING));

        let flags = FailsafeFlags::from_health(healthy(), true, false);
        assert!(flags.contains(FailsafeFlags::REFERENCE_STALE));
        assert!(!flags.contains(FailsafeFlags::REFERENCE_LOST));

        let flags = FailsafeFlags::from_health(healthy(), false, true);
        assert!(flags.contains(FailsafeFlags::REFERENCE_LOST));
    }

    #[test]
    fn flag_bits_fit_the_declared_count() {
        let all = [
            FailsafeFlags::ATTITUDE_INVALID,
            FailsafeFlags::LOCAL_POSITION_INVALID,
            FailsafeFlags::LOCAL_VELOCITY_INVALID,
            FailsafeFlags::IMU_STALE,
            FailsafeFlags::AIRFRAME_MISSING,
            FailsafeFlags::ATTITUDE_SESSION_CHANGED,
            FailsafeFlags::REFERENCE_STALE,
            FailsafeFlags::REFERENCE_LOST,
        ];
        let mut seen = 0u32;
        for flag in all {
            assert_eq!(seen & flag.bits(), 0);
            seen |= flag.bits();
        }
        assert_eq!(seen.count_ones(), FailsafeFlags::COUNT);
        assert_eq!(FailsafeFlags::default().bits(), 0);
    }
}
