//! 飞行状态机；高度相对起飞点，参考流只在 Track 模式消费。

use nalgebra::Vector3;
use serde::Deserialize;

use crate::control::PositionSetpoint;
use crate::precision::{PrecisionLanding, PrecisionParams};
use crate::px4::velocity_smoothing::VelocitySmoothing;
use crate::safety::{ArmingCheck, FailsafeAction, FailsafeFlags};
use crate::state::QuadState;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    Arm,
    Disarm,
    /// 自动起飞到相对起飞点的给定高度（m；`<= 0` 或非有限值 = 用 [`FsmParams::takeoff_altitude`]）。
    Takeoff {
        /// 相对起飞点的高度（m）。
        altitude: f32,
    },
    /// 位置保持：中止当前段（起飞/降落）并原地悬停，地面与空中均可用。
    Hold,
    Track,
    Land,
    PrecisionLand,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reject {
    Arming(ArmingCheck),
    NotArmed,
    WrongState,
    ReferenceNotReady,
    AltitudeOutOfRange,
    LandingTargetNotReady,
}

impl Reject {
    #[must_use]
    pub fn reason(self) -> &'static str {
        match self {
            Self::Arming(check) => check.reason(),
            Self::NotArmed => "未解锁",
            Self::WrongState => "当前状态不允许",
            Self::ReferenceNotReady => "参考流未就绪",
            Self::AltitudeOutOfRange => "起飞高度超范围",
            Self::LandingTargetNotReady => "降落目标未就绪",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlightState {
    Disarmed,
    /// 已解锁、仍在地面（等待起飞或降落收尾）。
    ArmedGrounded,
    Takeoff,
    /// 位置保持（起飞完成后的默认态；也是参考失联的回落态）。
    Hold,
    Track,
    /// 自动降落：按下降率降到起飞点地面，落地后自动上锁。
    Land,
    /// 位置估计失效：仅用 IMU 稳定姿态与标称悬停推力；不保证保高或定点。
    AttitudeFallback,
}

impl FlightState {
    /// Decode the mode reported by the flight controller on its status topic.
    #[must_use]
    pub fn from_code(code: u8) -> Option<Self> {
        [
            Self::Disarmed,
            Self::ArmedGrounded,
            Self::Takeoff,
            Self::Hold,
            Self::Track,
            Self::Land,
            Self::AttitudeFallback,
        ]
        .into_iter()
        .find(|state| state.code() == code)
    }

    #[must_use]
    pub fn motors_enabled(self) -> bool {
        !matches!(self, Self::Disarmed)
    }

    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Disarmed => "DISARMED",
            Self::ArmedGrounded => "ARMED_GROUNDED",
            Self::Takeoff => "TAKEOFF",
            Self::Hold => "HOLD",
            Self::Track => "TRACK",
            Self::Land => "LAND",
            Self::AttitudeFallback => "ATTITUDE_FALLBACK",
        }
    }

    /// 整数编码（IPC 诊断及验收证据；顺序即编码，不得重排）。
    #[must_use]
    pub fn code(self) -> u8 {
        match self {
            Self::Disarmed => 0,
            Self::ArmedGrounded => 1,
            Self::Takeoff => 2,
            Self::Hold => 3,
            Self::Track => 4,
            Self::Land => 5,
            Self::AttitudeFallback => 6,
        }
    }
}

#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Health {
    /// 状态估计就绪（`Firefly/Odometry` 的 `is_initialized`）。
    pub estimator_ready: bool,
    pub airframe_ready: bool,
    pub odometry_alive: bool,
    pub imu_alive: bool,
    /// VIO 姿态会话未改变（会话改变后不可恢复）。
    pub attitude_session_ok: bool,
    /// 最近控制输出低于最小推力到悬停推力区间的触地门限。
    pub landing_thrust_low: bool,
    /// Fresh IMU indicates gravity support (specific force near g) and low angular rate.
    pub landing_motion_low: bool,
}

#[derive(Deserialize, Clone, Copy, Debug)]
#[serde(deny_unknown_fields)]
pub struct FsmParams {
    #[serde(default)]
    pub precision: PrecisionParams,
    /// 缺省起飞高度（相对起飞点，m）：`Takeoff` 不带高度时用它（对照 `MAVLink`
    /// `NAV_TAKEOFF` 的 param7 缺省语义——两家飞控都有对应参数）。
    #[serde(default = "d_takeoff_altitude")]
    pub takeoff_altitude: f32,
    /// 起飞爬升率（m/s）：起飞段的速度上限。
    #[serde(default = "d_takeoff_climb_rate")]
    pub takeoff_climb_rate: f32,
    /// 起飞段最大垂直加速度（m/s²）。
    #[serde(default = "d_takeoff_accel")]
    pub takeoff_accel: f32,
    /// 起飞段最大垂直 jerk（m/s³）。
    #[serde(default = "d_takeoff_jerk")]
    pub takeoff_jerk: f32,
    /// 起飞完成判据①：高度进入目标 ±此比例。（对照 `ArduPilot`：10%）
    #[serde(default = "d_takeoff_complete_frac")]
    pub takeoff_complete_frac: f32,
    /// 起飞完成判据②：垂向速度低于此比例的爬升率。（对照 `ArduPilot`：50%）
    #[serde(default = "d_takeoff_complete_climb_frac")]
    pub takeoff_complete_climb_frac: f32,
    /// 起飞高度下限（m）。
    #[serde(default = "d_min_takeoff_altitude")]
    pub min_takeoff_altitude: f32,
    /// 起飞高度上限（m）。
    #[serde(default = "d_max_takeoff_altitude")]
    pub max_takeoff_altitude: f32,
    /// 降落下降率（m/s）：降落段的速度上限。
    #[serde(default = "d_land_descent_rate")]
    pub land_descent_rate: f32,
    /// 降落段最大垂直加速度（m/s²）。
    #[serde(default = "d_land_accel")]
    pub land_accel: f32,
    /// 降落段最大垂直 jerk（m/s³）。
    #[serde(default = "d_land_jerk")]
    pub land_jerk: f32,
    /// 地面判定：相对起飞点高度阈值（m）。
    #[serde(default = "d_on_ground_altitude")]
    pub on_ground_altitude: f32,
    /// 地面判定：垂向速度阈值（m/s）。
    #[serde(default = "d_on_ground_velocity")]
    pub on_ground_velocity: f32,
    /// 落地持续判定时长（s）：满足地面判据连续这么久才算落地，随后自动上锁。
    #[serde(default = "d_land_complete_hold")]
    pub land_complete_hold: f32,
    /// 参考流失联超时（s）：超过即从 `Track` 回落 `Hold`。
    /// 对照 `PX4` `COM_OF_LOSS_T`（默认 1.0 s）→ 回落 Position 模式。
    #[serde(default = "d_reference_timeout")]
    pub reference_timeout: f32,
}

fn d_takeoff_altitude() -> f32 {
    1.0
}
fn d_takeoff_climb_rate() -> f32 {
    0.6
}
fn d_takeoff_accel() -> f32 {
    1.0
}
fn d_takeoff_jerk() -> f32 {
    4.0
}
fn d_takeoff_complete_frac() -> f32 {
    0.10
}
fn d_takeoff_complete_climb_frac() -> f32 {
    0.50
}
fn d_min_takeoff_altitude() -> f32 {
    0.3
}
fn d_max_takeoff_altitude() -> f32 {
    10.0
}
fn d_land_descent_rate() -> f32 {
    0.4
}
fn d_land_accel() -> f32 {
    0.8
}
fn d_land_jerk() -> f32 {
    3.0
}
fn d_on_ground_altitude() -> f32 {
    0.06
}
fn d_on_ground_velocity() -> f32 {
    0.15
}
fn d_land_complete_hold() -> f32 {
    1.0
}
fn d_reference_timeout() -> f32 {
    1.0
}

impl Default for FsmParams {
    fn default() -> Self {
        Self {
            precision: PrecisionParams::default(),
            takeoff_altitude: d_takeoff_altitude(),
            takeoff_climb_rate: d_takeoff_climb_rate(),
            takeoff_accel: d_takeoff_accel(),
            takeoff_jerk: d_takeoff_jerk(),
            takeoff_complete_frac: d_takeoff_complete_frac(),
            takeoff_complete_climb_frac: d_takeoff_complete_climb_frac(),
            min_takeoff_altitude: d_min_takeoff_altitude(),
            max_takeoff_altitude: d_max_takeoff_altitude(),
            land_descent_rate: d_land_descent_rate(),
            land_accel: d_land_accel(),
            land_jerk: d_land_jerk(),
            on_ground_altitude: d_on_ground_altitude(),
            on_ground_velocity: d_on_ground_velocity(),
            land_complete_hold: d_land_complete_hold(),
            reference_timeout: d_reference_timeout(),
        }
    }
}

impl FsmParams {
    /// # Errors
    /// Rejects landing limits that cannot produce a descending trajectory and timed contact confirmation.
    pub fn validate(&self) -> firefly_error::Result<()> {
        self.precision.validate()?;
        for (name, value) in [
            ("land_descent_rate", self.land_descent_rate),
            ("land_accel", self.land_accel),
            ("land_jerk", self.land_jerk),
            ("on_ground_velocity", self.on_ground_velocity),
            ("land_complete_hold", self.land_complete_hold),
        ] {
            if !value.is_finite() || value <= 0. {
                return Err(firefly_error::Error::new(
                    firefly_error::ErrorKind::InvalidArgument,
                    format!("fsm.{name} must be finite and positive"),
                ));
            }
        }
        if !self.on_ground_altitude.is_finite() || self.on_ground_altitude < 0. {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "fsm.on_ground_altitude must be finite and non-negative",
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Output {
    /// 当前模式下飞控该跟踪的参考（`Track` = 外部参考；其余 = 状态机自己生成）。
    pub setpoint: PositionSetpoint,
    /// 电机是否允许出力（`Disarmed` 为 `false` → 指令全零）。
    pub motors_enabled: bool,
    /// 本 tick 自动产生的决策（指令引起的转移由 [`FlightFsm::command`] 返回）。
    pub event: Event,
    pub flags: FailsafeFlags,
    pub failsafe: FailsafeAction,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    None,
    Transition {
        from: FlightState,
        to: FlightState,
    },
    /// 估计失效：空中进入姿态降级，地面上锁。
    FailsafeEstimatorLost,
    /// 参考流失联，回落位置保持。
    FailsafeReferenceLost,
}

#[derive(Clone, Copy, Debug)]
pub struct FlightFsm {
    params: FsmParams,
    state: FlightState,
    /// 起飞点高度（世界 z，第一次 update 时锁存；`Arm` 时刷新）。
    origin_z: Option<f32>,
    target: Vector3<f32>,
    /// 目标航向（进入各模式时锁存；`Track` 由参考给）。
    target_yaw: f32,
    ramp_z: f32,
    vertical: VelocitySmoothing,
    /// 起飞目标高度（相对起飞点，m）。
    takeoff_target: f32,
    /// 接近、缓降与持续触地确认；确认态拥有连续满足判据的时长（s）。
    landing: LandingProgress,
    /// 最近一次 `update` 是否拿到参考流（`Track` 的准入条件）。
    reference_fresh: bool,
    /// `Track` 下参考流连续缺席的时长（s）。
    reference_lost: Option<f32>,
    precision: Option<PrecisionLanding>,
    landing_target: Option<Vector3<f32>>,
}

#[derive(Clone, Copy, Debug)]
enum LandingProgress {
    Approach,
    Crawl,
    Confirming(f32),
}

impl FlightFsm {
    /// 以通过 [`FsmParams::validate`] 的参数建状态机（初始 [`FlightState::Disarmed`]）。
    #[must_use]
    pub fn new(params: FsmParams) -> Self {
        Self {
            params,
            state: FlightState::Disarmed,
            origin_z: None,
            target: Vector3::zeros(),
            target_yaw: 0.0,
            ramp_z: 0.0,
            vertical: VelocitySmoothing::default(),
            takeoff_target: params.takeoff_altitude,
            landing: LandingProgress::Approach,
            reference_fresh: false,
            reference_lost: None,
            precision: None,
            landing_target: None,
        }
    }

    #[must_use]
    pub fn state(&self) -> FlightState {
        self.state
    }

    /// 调用方按采集时间、接收时间、会话和队伍校验后提交目标。
    pub fn observe_landing_target(&mut self, target: Option<Vector3<f32>>) {
        self.landing_target = target.filter(|p| p.iter().all(|v| v.is_finite()));
        if let (Some(precision), Some(target)) = (&mut self.precision, self.landing_target) {
            precision.observe(target);
        }
    }

    #[must_use]
    pub fn precision_stage(&self) -> Option<crate::precision::PrecisionStage> {
        self.precision.as_ref().map(PrecisionLanding::stage)
    }

    /// 控制器的触地候选：精确降落保留已确认的近地低运动阶段。
    #[must_use]
    pub fn ground_contact_candidate(&self, state: &QuadState) -> bool {
        self.precision
            .map_or_else(|| self.on_ground(state), |p| p.contact_candidate())
    }

    /// 起飞点高度（世界 z；尚未锁存时为 `None`）。
    #[must_use]
    pub fn origin_z(&self) -> Option<f32> {
        self.origin_z
    }

    /// 相对起飞点高度（m；未锁存起飞点时为 0）。
    #[must_use]
    pub fn altitude_above_origin(&self, state: &QuadState) -> f32 {
        self.origin_z.map_or(0.0, |z| state.position.z - z)
    }

    /// 地面判定：贴近起飞点高度且垂向速度小。
    #[must_use]
    pub fn on_ground(&self, state: &QuadState) -> bool {
        (state.position.z
            - self
                .precision
                .map_or(self.origin_z.unwrap_or(state.position.z), |p| p.target().z))
            <= self.params.on_ground_altitude
            && state.velocity.z.abs() <= self.params.on_ground_velocity
    }

    /// 成功时返回该指令引起的转移（状态未变时为 [`Event::None`]）。
    ///
    /// # Errors
    /// 拒绝时返回具体原因（不改变状态）——理由逐项上报，不静默丢弃。
    pub fn command(
        &mut self,
        cmd: Command,
        health: Health,
        state: &QuadState,
    ) -> Result<Event, Reject> {
        let from = self.state;
        if !matches!(cmd, Command::Arm) && self.state.motors_enabled() {
            if !health.estimator_ready {
                return Err(Reject::Arming(ArmingCheck::EstimatorReady));
            }
            if !health.attitude_session_ok {
                return Err(Reject::Arming(ArmingCheck::AttitudeSession));
            }
            if !health.odometry_alive {
                return Err(Reject::Arming(ArmingCheck::OdometryAlive));
            }
            if !health.imu_alive {
                return Err(Reject::Arming(ArmingCheck::ImuAlive));
            }
        }
        if self.state == FlightState::AttitudeFallback
            && !matches!(cmd, Command::Hold | Command::Land | Command::Disarm)
        {
            return Err(Reject::WrongState);
        }
        match cmd {
            Command::Arm => self.arm(health, state),
            Command::Disarm => self.disarm(state),
            Command::Takeoff { altitude } => self.takeoff(altitude, state),
            Command::Hold => self.hold(state),
            Command::Track => self.track(),
            Command::Land => self.land(state),
            Command::PrecisionLand => {
                if !matches!(self.state, FlightState::Hold | FlightState::Track) {
                    return Err(Reject::WrongState);
                }
                let target = self.landing_target.ok_or(Reject::LandingTargetNotReady)?;
                if !target.iter().all(|v| v.is_finite())
                    || (target - state.position).xy().norm() > 0.8
                    || state.position.z - target.z < 0.05
                {
                    return Err(Reject::LandingTargetNotReady);
                }
                self.enter_land(state);
                self.precision = Some(PrecisionLanding::new(self.params.precision, target, state));
                Ok(())
            }
        }?;
        Ok(event_of(from, self.state))
    }

    /// 每 tick 推进：锁存起飞点、跑段内斜坡、检查失效保护与自动转移。
    pub fn update(
        &mut self,
        dt: f32,
        state: &QuadState,
        reference: Option<&PositionSetpoint>,
        health: Health,
    ) -> Output {
        if self.origin_z.is_none() {
            self.origin_z = Some(state.position.z);
            self.target = state.position;
            self.ramp_z = state.position.z;
            self.target_yaw = crate::state::yaw_of(state.attitude);
        }
        self.reference_fresh = reference.is_some();
        let event = self.step(dt, state, reference, health);
        let flags = FailsafeFlags::from_health(
            health,
            matches!(event, Event::FailsafeReferenceLost)
                || (self.state == FlightState::Track && reference.is_none()),
            matches!(event, Event::FailsafeReferenceLost)
                || (self.state == FlightState::Track
                    && self
                        .reference_lost
                        .is_some_and(|elapsed| elapsed >= self.params.reference_timeout)),
        );
        let failsafe = if matches!(event, Event::FailsafeEstimatorLost)
            && self.state == FlightState::Disarmed
        {
            FailsafeAction::Disarm
        } else if matches!(event, Event::FailsafeEstimatorLost)
            || self.state == FlightState::AttitudeFallback
        {
            FailsafeAction::AttitudeFallback
        } else if matches!(event, Event::FailsafeReferenceLost) {
            FailsafeAction::Hold
        } else {
            FailsafeAction::None
        };
        Output {
            setpoint: self.output_setpoint(dt, state, reference),
            motors_enabled: self.state.motors_enabled(),
            event,
            flags,
            failsafe,
        }
    }

    /// 状态转移与失效保护（`update` 的决策部分；返回本次事件）。
    fn step(
        &mut self,
        dt: f32,
        state: &QuadState,
        reference: Option<&PositionSetpoint>,
        health: Health,
    ) -> Event {
        let prev = self.state;
        // 先处理失效状态，避免位置依赖的自动转移使用陈旧高度。
        if self.state.motors_enabled() && (!health.estimator_ready || !health.odometry_alive) {
            if self.state == FlightState::ArmedGrounded {
                self.state = FlightState::Disarmed;
                return Event::FailsafeEstimatorLost;
            }
            if self.state != FlightState::AttitudeFallback {
                self.target_yaw = crate::state::yaw_of(state.attitude);
                self.state = FlightState::AttitudeFallback;
                return Event::FailsafeEstimatorLost;
            }
        }
        match self.state {
            FlightState::Disarmed
            | FlightState::ArmedGrounded
            | FlightState::Hold
            | FlightState::AttitudeFallback => {}
            FlightState::Takeoff => {
                let target_z = self.origin_z.unwrap_or(0.0) + self.takeoff_target;
                let reached = (state.position.z - target_z).abs()
                    <= self.params.takeoff_complete_frac * self.takeoff_target;
                let slow = state.velocity.z.abs()
                    <= self.params.takeoff_complete_climb_frac * self.params.takeoff_climb_rate;
                if reached && slow {
                    // 自动完成起飞须保留整个位姿目标；容差只决定模式切换。
                    self.state = FlightState::Hold;
                    self.target.z = target_z;
                    self.ramp_z = target_z;
                }
            }
            FlightState::Track => {
                if reference.is_none() {
                    if self.reference_lost.is_none() {
                        self.target = state.position;
                        self.ramp_z = state.position.z;
                        self.target_yaw = crate::state::yaw_of(state.attitude);
                    }
                    let elapsed = self.reference_lost.get_or_insert(0.);
                    *elapsed += dt;
                    if *elapsed >= self.params.reference_timeout {
                        self.state = FlightState::Hold;
                        return Event::FailsafeReferenceLost;
                    }
                } else {
                    self.reference_lost = None;
                }
            }
            FlightState::Land => {
                if let Some(precision) = self.precision.as_mut()
                    && !precision.advance(
                        dt,
                        state,
                        state.position.z - precision.target().z <= self.params.on_ground_altitude
                            && state.velocity.norm() <= self.params.on_ground_velocity,
                    )
                {
                    self.enter_hold(state);
                    return Event::FailsafeReferenceLost;
                }
                let altitude = state.position.z
                    - self
                        .precision
                        .map_or(self.origin_z.unwrap_or(0.), |p| p.target().z);
                if matches!(self.landing, LandingProgress::Approach)
                    && altitude <= self.params.on_ground_altitude
                    && self.precision.is_none_or(|p| p.may_descend(state))
                {
                    self.landing = LandingProgress::Crawl;
                }
                if !matches!(self.landing, LandingProgress::Approach)
                    && self.precision.map_or_else(
                        || {
                            altitude <= self.params.on_ground_altitude
                                && state.velocity.norm() <= self.params.on_ground_velocity
                        },
                        |p| p.contact_candidate() && health.landing_motion_low,
                    )
                    && health.landing_thrust_low
                {
                    let elapsed = match self.landing {
                        LandingProgress::Confirming(elapsed) => elapsed + dt,
                        _ => dt,
                    };
                    self.landing = LandingProgress::Confirming(elapsed);
                    if elapsed >= self.params.land_complete_hold {
                        if self.precision.is_some() {
                            self.origin_z = Some(state.position.z);
                        }
                        self.state = FlightState::Disarmed;
                        self.precision = None;
                    }
                } else if matches!(self.landing, LandingProgress::Confirming(_)) {
                    self.landing = LandingProgress::Crawl;
                }
            }
        }
        event_of(prev, self.state)
    }

    /// 生成本 tick 的参考（段内按速率斜坡推进；`Track` 直接透传外部参考）。
    fn output_setpoint(
        &mut self,
        dt: f32,
        state: &QuadState,
        reference: Option<&PositionSetpoint>,
    ) -> PositionSetpoint {
        match self.state {
            FlightState::Track => reference.copied().unwrap_or(PositionSetpoint {
                position: self.target,
                yaw: self.target_yaw,
                ..PositionSetpoint::default()
            }),
            FlightState::Takeoff => {
                let goal = self.origin_z.unwrap_or(0.0) + self.takeoff_target;
                let target = self.vertical_target(goal, self.params.takeoff_climb_rate, true);
                self.vertical.update(dt, target);
                self.vertical_setpoint()
            }
            FlightState::Land => {
                let goal = self
                    .precision
                    .map_or(self.origin_z.unwrap_or(0.), |p| p.target().z);
                let target = if self.precision.is_some_and(|p| !p.may_descend(state)) {
                    0.
                } else if matches!(self.landing, LandingProgress::Approach) {
                    self.vertical_target(goal, self.params.land_descent_rate, false)
                } else {
                    // Keep descending until low thrust and arrested motion confirm contact.
                    // PX4 uses crawl speed above the vertical-motion threshold (factor 1.2).
                    -self
                        .params
                        .land_descent_rate
                        .min(1.2 * self.params.on_ground_velocity)
                };
                self.vertical.update(dt, target);
                let mut setpoint = self.vertical_setpoint();
                if let Some(precision) = self.precision {
                    let (position, velocity, acceleration) = precision.horizontal_reference();
                    for i in 0..2 {
                        setpoint.position[i] = position[i];
                        setpoint.velocity[i] = velocity[i];
                        setpoint.acceleration[i] = acceleration[i];
                    }
                }
                setpoint
            }
            FlightState::Hold => self.ramp_setpoint(0.0),
            FlightState::AttitudeFallback => PositionSetpoint {
                yaw: self.target_yaw,
                ..Default::default()
            },
            FlightState::ArmedGrounded => {
                self.ramp_z = state.position.z;
                self.target.z = state.position.z;
                self.ramp_setpoint(0.0)
            }
            FlightState::Disarmed => PositionSetpoint {
                position: state.position,
                yaw: crate::state::yaw_of(state.attitude),
                ..PositionSetpoint::default()
            },
        }
    }

    /// 垂直速度目标：位置误差线性映射到速度，并按爬升/下降方向限幅。
    ///
    /// 水平位置锁 `target`；垂直通道由 [`VelocitySmoothing`] 限制加速度与 jerk。
    fn vertical_target(&self, goal: f32, max_velocity: f32, ascend: bool) -> f32 {
        // 误差 0.3 m（爬升）/0.2 m（下降）内速度开始收敛，避免终点硬切。
        let gain = max_velocity / if ascend { 0.3 } else { 0.2 };
        let target = (goal - self.vertical.current_position()) * gain;
        if ascend {
            target.clamp(0.0, max_velocity)
        } else {
            target.clamp(-max_velocity, 0.0)
        }
    }

    /// 起飞/降落段的参考：水平锁 `target`，垂直取平滑轨迹的位置/速度/加速度。
    fn vertical_setpoint(&self) -> PositionSetpoint {
        PositionSetpoint {
            position: Vector3::new(
                self.target.x,
                self.target.y,
                self.vertical.current_position(),
            ),
            velocity: Vector3::new(0.0, 0.0, self.vertical.current_velocity()),
            acceleration: Vector3::new(0.0, 0.0, self.vertical.current_acceleration()),
            yaw: self.target_yaw,
            yaw_rate: 0.0,
        }
    }

    /// 保持段的参考：水平位置锁 `target`，高度锁 `ramp_z`，零垂速。
    fn ramp_setpoint(&self, velocity_z: f32) -> PositionSetpoint {
        PositionSetpoint {
            position: Vector3::new(self.target.x, self.target.y, self.ramp_z),
            velocity: Vector3::new(0.0, 0.0, velocity_z),
            acceleration: Vector3::zeros(),
            yaw: self.target_yaw,
            yaw_rate: 0.0,
        }
    }

    fn arm(&mut self, health: Health, state: &QuadState) -> Result<(), Reject> {
        self.arming_check(health, state).map_err(Reject::Arming)?;
        self.origin_z = Some(state.position.z);
        self.target = state.position;
        self.ramp_z = state.position.z;
        self.target_yaw = crate::state::yaw_of(state.attitude);
        self.precision = None;
        self.state = FlightState::ArmedGrounded;
        Ok(())
    }

    /// 逐项执行解锁前检查，返回首个失败项（顺序即 [`ArmingCheck::ALL`]）。
    ///
    /// 状态机与 `apps/firefly/fc` 的诊断共用这一份判定，避免“日志说能解锁、实际拒绝”。
    ///
    /// # Errors
    /// 返回首个不满足的 [`ArmingCheck`]；全部通过时返回 `Ok(())`。
    pub fn arming_check(&self, health: Health, state: &QuadState) -> Result<(), ArmingCheck> {
        if self.state != FlightState::Disarmed {
            return Err(ArmingCheck::Disarmed);
        }
        if !health.estimator_ready {
            return Err(ArmingCheck::EstimatorReady);
        }
        if !health.attitude_session_ok {
            return Err(ArmingCheck::AttitudeSession);
        }
        if !health.airframe_ready {
            return Err(ArmingCheck::AirframeReady);
        }
        if !health.odometry_alive {
            return Err(ArmingCheck::OdometryAlive);
        }
        if !health.imu_alive {
            return Err(ArmingCheck::ImuAlive);
        }
        if !self.on_ground(state) {
            return Err(ArmingCheck::OnGround);
        }
        Ok(())
    }

    fn disarm(&mut self, state: &QuadState) -> Result<(), Reject> {
        if !self.state.motors_enabled() {
            return Err(Reject::WrongState);
        }
        // 空中禁止上锁，避免飞行中关闭电机。
        if !self.on_ground(state) {
            return Err(Reject::Arming(ArmingCheck::OnGround));
        }
        if self.precision.is_some() {
            self.origin_z = Some(state.position.z);
        }
        self.precision = None;
        self.state = FlightState::Disarmed;
        self.landing = LandingProgress::Approach;
        Ok(())
    }

    fn takeoff(&mut self, altitude: f32, state: &QuadState) -> Result<(), Reject> {
        if self.state != FlightState::ArmedGrounded {
            return Err(if self.state.motors_enabled() {
                Reject::WrongState
            } else {
                Reject::NotArmed
            });
        }
        let altitude = if altitude.is_finite() && altitude > 0.0 {
            altitude
        } else {
            self.params.takeoff_altitude
        };
        if !(self.params.min_takeoff_altitude..=self.params.max_takeoff_altitude)
            .contains(&altitude)
        {
            return Err(Reject::AltitudeOutOfRange);
        }
        self.takeoff_target = altitude;
        self.target = state.position;
        self.ramp_z = state.position.z;
        self.target_yaw = crate::state::yaw_of(state.attitude);
        self.vertical.reset(0.0, state.velocity.z, state.position.z);
        self.vertical.set_max_jerk(self.params.takeoff_jerk);
        self.vertical.set_max_accel(self.params.takeoff_accel);
        self.vertical.set_max_vel(self.params.takeoff_climb_rate);
        self.state = FlightState::Takeoff;
        Ok(())
    }

    /// 对所有已解锁状态开放：中止当前段 → 原地保持。
    ///
    /// 失效降级只能在有效里程计恢复后由显式指令退出。
    fn hold(&mut self, state: &QuadState) -> Result<(), Reject> {
        if !self.state.motors_enabled() {
            return Err(Reject::NotArmed);
        }
        self.enter_hold(state);
        Ok(())
    }

    fn track(&mut self) -> Result<(), Reject> {
        match self.state {
            FlightState::Hold | FlightState::Track => {}
            FlightState::Disarmed | FlightState::ArmedGrounded => return Err(Reject::NotArmed),
            FlightState::Takeoff | FlightState::Land | FlightState::AttitudeFallback => {
                return Err(Reject::WrongState);
            }
        }
        if !self.reference_fresh {
            return Err(Reject::ReferenceNotReady);
        }
        self.state = FlightState::Track;
        self.reference_lost = None;
        Ok(())
    }

    fn land(&mut self, state: &QuadState) -> Result<(), Reject> {
        if !self.state.motors_enabled() {
            return Err(Reject::NotArmed);
        }
        self.enter_land(state);
        Ok(())
    }

    fn enter_hold(&mut self, state: &QuadState) {
        self.precision = None;
        self.state = FlightState::Hold;
        self.target = state.position;
        self.ramp_z = state.position.z;
        self.target_yaw = crate::state::yaw_of(state.attitude);
    }

    fn enter_land(&mut self, state: &QuadState) {
        self.precision = None;
        self.state = FlightState::Land;
        self.target = state.position;
        self.ramp_z = state.position.z;
        self.vertical.reset(0.0, state.velocity.z, state.position.z);
        self.vertical.set_max_jerk(self.params.land_jerk);
        self.vertical.set_max_accel(self.params.land_accel);
        self.vertical.set_max_vel(self.params.land_descent_rate);
        self.landing = LandingProgress::Approach;
    }
}

/// 转移事件：状态未变时为 [`Event::None`]。
fn event_of(from: FlightState, to: FlightState) -> Event {
    if from == to {
        Event::None
    } else {
        Event::Transition { from, to }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.005;

    #[test]
    fn hold_restores_height_after_vertical_displacement() {
        let mut fsm = FlightFsm::new(FsmParams::default());
        let mut state = QuadState::default();
        fsm.update(0.001, &state, None, ok_health());
        fsm.command(Command::Arm, ok_health(), &state).unwrap();
        fsm.command(Command::Takeoff { altitude: 1.0 }, ok_health(), &state)
            .unwrap();
        state.position.z = 1.0;
        fsm.update(0.001, &state, None, ok_health());
        assert_eq!(fsm.state(), FlightState::Hold);
        for height in [0.7, 1.3, 0.9] {
            state.position.z = height;
            let out = fsm.update(0.001, &state, None, ok_health());
            assert!((out.setpoint.position.z - 1.0).abs() < 1e-6);
        }
    }

    fn ok_health() -> Health {
        Health {
            estimator_ready: true,
            airframe_ready: true,
            odometry_alive: true,
            imu_alive: true,
            attitude_session_ok: true,
            landing_thrust_low: true,
            landing_motion_low: true,
        }
    }

    fn hovering(z: f32) -> QuadState {
        QuadState {
            position: Vector3::new(1.0, 2.0, z),
            ..QuadState::default()
        }
    }

    struct Rig {
        fsm: FlightFsm,
        state: QuadState,
    }

    impl Rig {
        fn new() -> Self {
            let mut rig = Self {
                fsm: FlightFsm::new(FsmParams::default()),
                state: hovering(0.02),
            };
            let _ = rig.fsm.update(DT, &rig.state, None, ok_health());
            rig
        }

        fn cmd(&mut self, cmd: Command) -> Result<(), Reject> {
            self.fsm.command(cmd, ok_health(), &self.state).map(|_| ())
        }

        fn tick(&mut self) {
            let out = self.fsm.update(DT, &self.state, None, ok_health());
            self.state.position = out.setpoint.position;
            self.state.velocity = Vector3::zeros();
        }

        fn ticks(&mut self, n: usize) {
            for _ in 0..n {
                self.tick();
            }
        }
    }

    #[test]
    fn takeoff_before_arm_is_rejected() {
        let mut rig = Rig::new();
        assert_eq!(
            rig.cmd(Command::Takeoff { altitude: 1.0 }),
            Err(Reject::NotArmed)
        );
        assert_eq!(rig.fsm.state(), FlightState::Disarmed);
    }

    #[test]
    fn arm_requires_every_preflight_item() {
        let mut rig = Rig::new();
        for (health, expect) in [
            (
                Health {
                    estimator_ready: false,
                    ..ok_health()
                },
                Reject::Arming(ArmingCheck::EstimatorReady),
            ),
            (
                Health {
                    attitude_session_ok: false,
                    ..ok_health()
                },
                Reject::Arming(ArmingCheck::AttitudeSession),
            ),
            (
                Health {
                    airframe_ready: false,
                    ..ok_health()
                },
                Reject::Arming(ArmingCheck::AirframeReady),
            ),
            (
                Health {
                    odometry_alive: false,
                    ..ok_health()
                },
                Reject::Arming(ArmingCheck::OdometryAlive),
            ),
            (
                Health {
                    imu_alive: false,
                    ..ok_health()
                },
                Reject::Arming(ArmingCheck::ImuAlive),
            ),
        ] {
            let _ = rig.fsm.update(DT, &rig.state, None, ok_health());
            assert_eq!(
                rig.fsm.command(Command::Arm, health, &rig.state),
                Err(expect),
                "{expect:?}"
            );
            assert_eq!(rig.fsm.state(), FlightState::Disarmed);
        }
    }

    #[test]
    fn arm_in_air_is_rejected() {
        let mut fsm = FlightFsm::new(FsmParams::default());
        let ground = hovering(0.02);
        let _ = fsm.update(DT, &ground, None, ok_health());
        let air = hovering(1.0);
        assert_eq!(
            fsm.command(Command::Arm, ok_health(), &air),
            Err(Reject::Arming(ArmingCheck::OnGround))
        );
    }

    #[test]
    fn takeoff_tolerance_preserves_commanded_hold_pose() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        let origin = rig.state.position;
        let yaw = crate::state::yaw_of(rig.state.attitude);
        rig.cmd(Command::Takeoff { altitude: 1.795 }).unwrap();
        rig.state.position = origin + Vector3::new(0.2, -0.3, 1.795 * 0.91);
        rig.state.attitude =
            nalgebra::UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw + 0.1);
        rig.state.velocity = Vector3::zeros();
        let out = rig.fsm.update(DT, &rig.state, None, ok_health());
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        assert!((out.setpoint.position - origin - Vector3::z() * 1.795).norm() < 1e-6);
        assert!((out.setpoint.yaw - yaw).abs() < 1e-6);
    }

    #[test]
    fn arm_then_takeoff_reaches_altitude_and_holds() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        assert_eq!(rig.fsm.state(), FlightState::ArmedGrounded);
        assert!(
            rig.fsm
                .update(DT, &rig.state, None, ok_health())
                .motors_enabled
        );

        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        assert_eq!(rig.fsm.state(), FlightState::Takeoff);

        let before = rig.state.position.z;
        rig.tick();
        let step = rig.state.position.z - before;
        assert!(step <= 0.6 * DT + 1e-6, "斜坡过陡：{step}");

        rig.ticks(1000);
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        assert!(
            (rig.fsm.altitude_above_origin(&rig.state) - 1.0).abs() < 0.11,
            "起飞后高度 {:.3}",
            rig.fsm.altitude_above_origin(&rig.state)
        );
    }

    #[test]
    fn takeoff_incomplete_stays_in_takeoff() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(20); // 只爬了一小段
        assert_eq!(rig.fsm.state(), FlightState::Takeoff);
        assert_eq!(rig.cmd(Command::Track), Err(Reject::WrongState));
    }

    #[test]
    fn takeoff_without_altitude_uses_default() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 0.0 }).unwrap();
        rig.ticks(1000);
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        let default = FsmParams::default().takeoff_altitude;
        assert!(
            (rig.fsm.altitude_above_origin(&rig.state) - default).abs() < 0.11,
            "缺省起飞高度 {:.3}",
            rig.fsm.altitude_above_origin(&rig.state)
        );
    }

    #[test]
    fn takeoff_altitude_range_is_enforced() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        assert_eq!(
            rig.cmd(Command::Takeoff { altitude: 0.1 }),
            Err(Reject::AltitudeOutOfRange)
        );
        assert_eq!(
            rig.cmd(Command::Takeoff { altitude: 50.0 }),
            Err(Reject::AltitudeOutOfRange)
        );
    }

    #[test]
    fn disarm_in_air_is_rejected_and_on_ground_is_ok() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(1000);
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        assert_eq!(
            rig.cmd(Command::Disarm),
            Err(Reject::Arming(ArmingCheck::OnGround))
        );

        rig.cmd(Command::Land).unwrap();
        assert_eq!(rig.fsm.state(), FlightState::Land);
        rig.ticks(2000);
        assert_eq!(rig.fsm.state(), FlightState::Disarmed, "落地后应自动上锁");
    }

    #[test]
    fn landing_requires_continuous_low_thrust_and_low_motion() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.state.position.z += 0.04;
        rig.cmd(Command::Land).unwrap();
        let hovering = Health {
            landing_thrust_low: false,
            ..ok_health()
        };
        let mut output = rig.fsm.update(DT, &rig.state, None, hovering);
        for _ in 0..1000 {
            output = rig.fsm.update(DT, &rig.state, None, hovering);
        }
        assert_eq!(rig.fsm.state(), FlightState::Land);
        assert!(output.setpoint.position.z < rig.fsm.origin_z.expect("initialized origin"));
        assert!(output.setpoint.velocity.z < 0.0);

        for _ in 0..100 {
            rig.fsm.update(DT, &rig.state, None, ok_health());
        }
        rig.state.velocity.x = 0.3;
        rig.fsm.update(DT, &rig.state, None, ok_health());
        rig.state.velocity.x = 0.0;
        for _ in 0..100 {
            rig.fsm.update(DT, &rig.state, None, ok_health());
        }
        assert_eq!(
            rig.fsm.state(),
            FlightState::Land,
            "motion resets contact confirmation"
        );
        rig.fsm.update(DT, &rig.state, None, hovering);
        for _ in 0..100 {
            rig.fsm.update(DT, &rig.state, None, ok_health());
        }
        assert_eq!(
            rig.fsm.state(),
            FlightState::Land,
            "thrust resets contact confirmation"
        );
        for _ in 0..102 {
            rig.fsm.update(DT, &rig.state, None, ok_health());
        }
        assert_eq!(rig.fsm.state(), FlightState::Disarmed);
    }

    #[test]
    fn track_needs_fresh_reference() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(1000);
        rig.fsm.reference_fresh = false;
        assert_eq!(rig.cmd(Command::Track), Err(Reject::ReferenceNotReady));
        rig.fsm.reference_fresh = true;
        rig.cmd(Command::Track).unwrap();
        assert_eq!(rig.fsm.state(), FlightState::Track);
    }

    #[test]
    fn reference_loss_in_track_falls_back_to_hold_at_current_position() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(1000);
        rig.fsm.reference_fresh = true;
        rig.cmd(Command::Track).unwrap();

        let setpoint = PositionSetpoint {
            position: Vector3::new(3.0, 4.0, 1.02),
            velocity: Vector3::new(0.5, 0.0, 0.0),
            acceleration: Vector3::new(0.2, 0.0, 0.0),
            yaw: 0.3,
            yaw_rate: 0.0,
        };
        let mut out = rig.fsm.update(DT, &rig.state, Some(&setpoint), ok_health());
        assert_eq!(rig.fsm.state(), FlightState::Track);
        assert!((out.setpoint.position - setpoint.position).norm() < 1e-6);
        out = rig.fsm.update(DT, &rig.state, Some(&setpoint), ok_health());
        assert!(
            (out.setpoint.yaw - 0.3).abs() < 1e-6,
            "{}",
            out.setpoint.yaw
        );

        rig.state.position = Vector3::new(10., 6., 1.8);
        let lost_at = rig.state.position;
        let first = rig.fsm.update(DT, &rig.state, None, ok_health());
        assert_eq!(first.setpoint.position, lost_at);
        assert_eq!(first.setpoint.velocity, Vector3::zeros());
        rig.state.position += Vector3::new(0.1, 0.2, -0.1);
        let mut out = rig.fsm.update(DT, &rig.state, None, ok_health());
        let mut event = out.event;
        let mut ticks = 1;
        while ticks < 1000 {
            ticks += 1;
            out = rig.fsm.update(DT, &rig.state, None, ok_health());
            event = out.event;
            if event != Event::None {
                break;
            }
        }
        assert_eq!(event, Event::FailsafeReferenceLost);
        assert!(out.flags.contains(FailsafeFlags::REFERENCE_LOST));
        assert_eq!(out.failsafe, FailsafeAction::Hold);
        let timeout = FsmParams::default().reference_timeout;
        assert!(
            (ticks as f32 * DT - timeout).abs() <= DT,
            "失联判定用了 {:.3} s，期望 {timeout} s",
            ticks as f32 * DT
        );
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        let out = rig.fsm.update(DT, &rig.state, None, ok_health());
        assert!(
            (out.setpoint.position - lost_at).norm() < 1e-3,
            "{:?}",
            out.setpoint.position
        );
        assert!(out.setpoint.velocity.norm() < 1e-6);
    }

    #[test]
    fn zero_duration_loss_latches_once_and_recovery_starts_a_new_episode() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(1000);
        rig.fsm.reference_fresh = true;
        rig.cmd(Command::Track).unwrap();
        let held = rig.state.position;
        assert_eq!(
            rig.fsm
                .update(0., &rig.state, None, ok_health())
                .setpoint
                .position,
            held
        );
        rig.state.position.x += 1.;
        assert_eq!(
            rig.fsm
                .update(0., &rig.state, None, ok_health())
                .setpoint
                .position,
            held
        );
        let reference = PositionSetpoint {
            position: rig.state.position,
            ..Default::default()
        };
        rig.fsm
            .update(DT, &rig.state, Some(&reference), ok_health());
        assert!(rig.fsm.reference_lost.is_none());
        assert_eq!(
            rig.fsm
                .update(0., &rig.state, None, ok_health())
                .setpoint
                .position,
            rig.state.position
        );
    }

    #[test]
    fn estimator_loss_latches_attitude_fallback_until_explicit_recovery() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(1000);
        let bad = Health {
            odometry_alive: false,
            ..ok_health()
        };
        let out = rig.fsm.update(DT, &rig.state, None, bad);
        assert!(matches!(out.event, Event::FailsafeEstimatorLost));
        assert!(out.flags.contains(FailsafeFlags::LOCAL_POSITION_INVALID));
        assert!(out.flags.contains(FailsafeFlags::LOCAL_VELOCITY_INVALID));
        assert_eq!(out.failsafe, FailsafeAction::AttitudeFallback);
        assert_eq!(rig.fsm.state(), FlightState::AttitudeFallback);
        rig.state.position = Vector3::zeros();
        for _ in 0..1000 {
            assert!(rig.fsm.update(DT, &rig.state, None, bad).motors_enabled);
        }
        assert_eq!(
            rig.fsm.command(Command::Land, bad, &rig.state),
            Err(Reject::Arming(ArmingCheck::OdometryAlive))
        );
        rig.fsm.update(DT, &rig.state, None, ok_health());
        assert_eq!(rig.fsm.state(), FlightState::AttitudeFallback);
        rig.fsm
            .command(Command::Hold, ok_health(), &rig.state)
            .unwrap();
        assert_eq!(rig.fsm.state(), FlightState::Hold);
    }

    #[test]
    fn hold_does_not_auto_take_over_reference() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(1000);
        let setpoint = PositionSetpoint {
            position: Vector3::new(9.0, 9.0, 2.0),
            ..PositionSetpoint::default()
        };
        rig.fsm.reference_fresh = true;
        let out = rig.fsm.update(DT, &rig.state, Some(&setpoint), ok_health());
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        assert!((out.setpoint.position - rig.state.position).norm() < 1e-3);
    }

    #[test]
    fn transitions_are_reported_as_events() {
        let mut rig = Rig::new();
        assert_eq!(
            rig.fsm.command(Command::Arm, ok_health(), &rig.state),
            Ok(Event::Transition {
                from: FlightState::Disarmed,
                to: FlightState::ArmedGrounded,
            })
        );
        assert_eq!(
            rig.fsm
                .command(Command::Takeoff { altitude: 1.0 }, ok_health(), &rig.state),
            Ok(Event::Transition {
                from: FlightState::ArmedGrounded,
                to: FlightState::Takeoff,
            })
        );

        let mut event = Event::None;
        for _ in 0..1000 {
            let out = rig.fsm.update(DT, &rig.state, None, ok_health());
            rig.state.position = out.setpoint.position;
            rig.state.velocity = Vector3::zeros();
            if out.event != Event::None {
                event = out.event;
                break;
            }
        }
        assert_eq!(
            event,
            Event::Transition {
                from: FlightState::Takeoff,
                to: FlightState::Hold,
            }
        );
        assert_eq!(
            rig.fsm.update(DT, &rig.state, None, ok_health()).event,
            Event::None
        );
        assert_eq!(
            rig.fsm.command(Command::Hold, ok_health(), &rig.state),
            Ok(Event::None)
        );
    }

    #[test]
    fn hold_aborts_any_armed_segment() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        assert_eq!(rig.cmd(Command::Hold), Ok(()));
        assert_eq!(rig.fsm.state(), FlightState::Hold);

        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(60);
        assert_eq!(rig.fsm.state(), FlightState::Takeoff);
        assert_eq!(rig.cmd(Command::Hold), Ok(()));
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        let held = rig.state.position;
        rig.ticks(200);
        assert!((rig.state.position.z - held.z).abs() < 1e-6);

        rig.cmd(Command::Land).unwrap();
        assert_eq!(rig.fsm.state(), FlightState::Land);
        let bad = Health {
            estimator_ready: false,
            ..ok_health()
        };
        rig.fsm.update(DT, &rig.state, None, bad);
        assert_eq!(rig.fsm.state(), FlightState::AttitudeFallback);
        rig.cmd(Command::Hold).unwrap();
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        rig.fsm.update(DT, &rig.state, None, bad);
        assert_eq!(rig.fsm.state(), FlightState::AttitudeFallback);
    }

    #[test]
    fn motors_follow_state() {
        let mut rig = Rig::new();
        assert!(
            !rig.fsm
                .update(DT, &rig.state, None, ok_health())
                .motors_enabled
        );
        rig.cmd(Command::Arm).unwrap();
        assert!(
            rig.fsm
                .update(DT, &rig.state, None, ok_health())
                .motors_enabled
        );
    }

    #[test]
    fn precision_landing_requires_target_and_missing_target_returns_hold() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1. }).unwrap();
        rig.ticks(1000);
        assert_eq!(
            rig.cmd(Command::PrecisionLand),
            Err(Reject::LandingTargetNotReady)
        );
        let target = Vector3::new(rig.state.position.x, rig.state.position.y, 0.3);
        rig.fsm.observe_landing_target(Some(target));
        rig.cmd(Command::PrecisionLand).unwrap();
        rig.ticks(
            ((rig.fsm.params.precision.target_timeout
                + rig.fsm.params.precision.reacquire_timeout
                + 0.1)
                / DT)
                .ceil() as usize,
        );
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        assert!(rig.state.position.z > 0.5);
    }

    #[test]
    fn precision_contact_requires_imu_support_despite_odometry_drift() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1. }).unwrap();
        rig.ticks(1000);
        let target = Vector3::new(rig.state.position.x, rig.state.position.y, 0.3);
        rig.fsm.observe_landing_target(Some(target));
        rig.cmd(Command::PrecisionLand).unwrap();
        let mut health = ok_health();
        health.landing_thrust_low = false;
        for _ in 0..1500 {
            rig.fsm.observe_landing_target(Some(target));
            let out = rig.fsm.update(DT, &rig.state, None, health);
            rig.state.position = out.setpoint.position;
            rig.state.position.z = rig.state.position.z.max(target.z);
            if rig.fsm.ground_contact_candidate(&rig.state) {
                break;
            }
        }
        assert!(rig.fsm.ground_contact_candidate(&rig.state));
        rig.state.position += Vector3::new(0.2, 0., 0.1);
        rig.state.velocity = Vector3::new(0.3, 0., 0.02);
        health.landing_thrust_low = true;
        health.landing_motion_low = false;
        for _ in 0..150 {
            rig.fsm.update(DT, &rig.state, None, health);
        }
        assert_eq!(rig.fsm.state(), FlightState::Land);
        health.landing_motion_low = true;
        for _ in 0..250 {
            rig.fsm.update(DT, &rig.state, None, health);
        }
        assert_eq!(rig.fsm.state(), FlightState::Disarmed);
    }

    #[test]
    fn precision_contact_uses_observed_platform_height() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1. }).unwrap();
        rig.ticks(1000);
        let target = Vector3::new(rig.state.position.x, rig.state.position.y, 0.3);
        rig.fsm.observe_landing_target(Some(target));
        rig.cmd(Command::PrecisionLand).unwrap();
        for _ in 0..1500 {
            rig.fsm.observe_landing_target(Some(target));
            let mut health = ok_health();
            health.landing_thrust_low = rig.state.position.z <= target.z;
            let out = rig.fsm.update(DT, &rig.state, None, health);
            rig.state.position = out.setpoint.position;
            rig.state.position.z = rig.state.position.z.max(target.z);
            rig.state.velocity = Vector3::zeros();
            if rig.fsm.state() == FlightState::Disarmed {
                break;
            }
        }
        assert_eq!(rig.fsm.state(), FlightState::Disarmed);
        assert!((rig.state.position.z - target.z).abs() < 0.001);
        assert!(rig.fsm.precision_stage().is_none());
        assert_eq!(rig.cmd(Command::Arm), Ok(()));
    }

    #[test]
    fn hold_setpoint_is_stationary() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(1000);
        let out = rig.fsm.update(DT, &rig.state, None, ok_health());
        assert_eq!(out.setpoint.velocity, Vector3::zeros());
        assert_eq!(out.setpoint.acceleration, Vector3::zeros());
    }
    #[test]
    fn takeoff_vertical_reference_is_jerk_limited() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        let (mut previous_accel, mut previous_velocity) = (0.0f32, 0.0f32);
        let (mut max_accel, mut max_velocity) = (0.0f32, 0.0f32);
        for _ in 0..800 {
            let out = rig.fsm.update(DT, &rig.state, None, ok_health());
            if out.event != Event::None {
                break;
            }
            let accel = out.setpoint.acceleration.z;
            let velocity = out.setpoint.velocity.z;
            assert!((accel - previous_accel).abs() <= rig.fsm.params.takeoff_jerk * DT + 1e-3);
            assert!(
                (velocity - previous_velocity).abs() <= rig.fsm.params.takeoff_accel * DT + 1e-3
            );
            previous_accel = accel;
            previous_velocity = velocity;
            max_accel = max_accel.max(accel.abs());
            max_velocity = max_velocity.max(velocity.abs());
            rig.state.position = out.setpoint.position;
            rig.state.velocity = out.setpoint.velocity;
        }
        assert!(max_velocity > 0.2, "应出现爬升速度，实测 {max_velocity:.3}");
        assert!(max_accel <= rig.fsm.params.takeoff_accel + 1e-3);
    }
}
