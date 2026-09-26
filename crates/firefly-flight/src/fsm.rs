//! 飞行状态机：解锁、模式、自动起降与失效保护（对照 `PX4` `nav_state` 与
//! `ArduPilot` 模式机 + auto-takeoff 判据）。
//!
//! 纯逻辑：不做 IO、不读时钟（`dt` 由调用方给），全部转移条件显式、可单测。
//! 上层（`apps/fc`）负责把健康电平与指令喂进来，并把 [`FlightFsm::update`] 的输出
//!（当前模式该飞的参考 + 电机是否使能）映射成 `position_mode` 的参考与 4 电机推力。
//!
//! 指令语义 = **"谁在控制"**（对照 `MAVLink` `COMMAND_LONG`）：目标点（goal）属规划侧
//! 输入，设定值流（`Firefly/Reference`）只在 [`FlightState::Track`] 下被消费，且其
//! 存活性是失效保护条件（对照 `PX4` `COM_OF_LOSS_T` → 回落 Position 模式）。
//!
//! 高度一律相对**起飞点**（`origin`，第一次 update 时锁存；`Arm` 时刷新）——与
//! `ArduPilot` 的 "alt-above-home" 语义一致，免去对世界原点的假设。

use glam::Vec3;
use serde::Deserialize;

use crate::control::PositionSetpoint;
use crate::state::QuadState;

/// 地面站指令（对照 `MAVLink` `COMMAND_LONG` 的解锁/模式类子集）。
///
/// 不含目标点：目标点是**规划侧**输入（`Firefly/Goal` → planner → `Reference`），
/// 飞控只在 [`FlightState::Track`] 下消费参考流。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    /// 解锁（需通过全部解锁前检查）。
    Arm,
    /// 上锁（仅允许在地面）。
    Disarm,
    /// 自动起飞到相对起飞点的给定高度（m；`<= 0` 或非有限值 = 用 [`FsmParams::takeoff_altitude`]）。
    Takeoff {
        /// 相对起飞点的高度（m）。
        altitude: f32,
    },
    /// 位置保持：中止当前段（起飞/降落）并原地悬停，地面与空中均可用。
    Hold,
    /// 跟踪外部参考流（planner；需参考新鲜）。
    Track,
    /// 自动降落（下降 → 落地 → 自动上锁）。
    Land,
}

/// 指令被拒绝的原因（逐项上报，对照 `PX4` `HealthAndArmingChecks` 的"为什么不能解锁"）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reject {
    /// 需要先解锁。
    NotArmed,
    /// 当前状态不允许该指令。
    WrongState,
    /// 状态估计未就绪。
    EstimatorNotReady,
    /// 未收到机体/执行器描述。
    NoAirframe,
    /// 被控对象状态陈旧。
    PlantStale,
    /// IMU 陈旧。
    ImuStale,
    /// 不在起飞点地面（解锁要求停在地面）。
    NotOnGround,
    /// 参考流不存在或陈旧。
    ReferenceNotReady,
    /// 起飞高度超范围。
    AltitudeOutOfRange,
}

impl Reject {
    /// 一行中文原因（进日志与 rrd）。
    #[must_use]
    pub fn reason(self) -> &'static str {
        match self {
            Self::NotArmed => "未解锁",
            Self::WrongState => "当前状态不允许",
            Self::EstimatorNotReady => "状态估计未就绪",
            Self::NoAirframe => "未收到机体描述",
            Self::PlantStale => "被控对象状态陈旧",
            Self::ImuStale => "IMU 陈旧",
            Self::NotOnGround => "不在起飞点地面",
            Self::ReferenceNotReady => "参考流未就绪",
            Self::AltitudeOutOfRange => "起飞高度超范围",
        }
    }
}

/// 飞行状态（可观测、可记录；对照 `PX4` `nav_state` 的四旋翼子集）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlightState {
    /// 停机坪：电机停转。
    Disarmed,
    /// 已解锁、仍在地面（等待起飞或降落收尾）。
    ArmedGrounded,
    /// 自动起飞：按爬升率爬到目标高度。
    Takeoff,
    /// 位置保持（起飞完成后的默认态；也是参考失联的回落态）。
    Hold,
    /// 跟踪外部参考流（planner）。
    Track,
    /// 自动降落：按下降率降到起飞点地面，落地后自动上锁。
    Land,
}

impl FlightState {
    /// 电机是否允许出力。
    #[must_use]
    pub fn motors_enabled(self) -> bool {
        !matches!(self, Self::Disarmed)
    }

    /// 简称（日志/rrd 实体名）。
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Disarmed => "DISARMED",
            Self::ArmedGrounded => "ARMED_GROUNDED",
            Self::Takeoff => "TAKEOFF",
            Self::Hold => "HOLD",
            Self::Track => "TRACK",
            Self::Land => "LAND",
        }
    }

    /// 整数编码（rrd 里当标量记录，见 `docs/how_to_run.md`；顺序即编码，不得重排）。
    #[must_use]
    pub fn code(self) -> u8 {
        match self {
            Self::Disarmed => 0,
            Self::ArmedGrounded => 1,
            Self::Takeoff => 2,
            Self::Hold => 3,
            Self::Track => 4,
            Self::Land => 5,
        }
    }
}

/// 上层每 tick 汇报的健康电平（飞控自己算，不由被控对象喂）。
// 四个布尔是四条独立电平，不是状态编码，故不打包成位域
#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Health {
    /// 状态估计就绪（`Firefly/Odometry` 的 `is_initialized`）。
    pub estimator_ready: bool,
    /// 已收到机体/执行器描述（`Firefly/Airframe`）。
    pub airframe_ready: bool,
    /// 被控对象状态新鲜（`Firefly/PlantState`）。
    pub plant_alive: bool,
    /// IMU 新鲜。
    pub imu_alive: bool,
}

/// 状态机参数（缺键回落默认值；单位与来源见各字段）。
#[derive(Deserialize, Clone, Copy, Debug)]
pub struct FsmParams {
    /// 缺省起飞高度（相对起飞点，m）：`Takeoff` 不带高度时用它（对照 `MAVLink`
    /// `NAV_TAKEOFF` 的 param7 缺省语义——两家飞控都有对应参数）。
    #[serde(default = "d_takeoff_altitude")]
    pub takeoff_altitude: f32,
    /// 起飞爬升率（m/s）：起飞段目标高度按此速率斜坡上升。
    #[serde(default = "d_takeoff_climb_rate")]
    pub takeoff_climb_rate: f32,
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
    /// 降落下降率（m/s）：降落段目标高度按此速率斜坡下降。
    #[serde(default = "d_land_descent_rate")]
    pub land_descent_rate: f32,
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
            takeoff_altitude: d_takeoff_altitude(),
            takeoff_climb_rate: d_takeoff_climb_rate(),
            takeoff_complete_frac: d_takeoff_complete_frac(),
            takeoff_complete_climb_frac: d_takeoff_complete_climb_frac(),
            min_takeoff_altitude: d_min_takeoff_altitude(),
            max_takeoff_altitude: d_max_takeoff_altitude(),
            land_descent_rate: d_land_descent_rate(),
            on_ground_altitude: d_on_ground_altitude(),
            on_ground_velocity: d_on_ground_velocity(),
            land_complete_hold: d_land_complete_hold(),
            reference_timeout: d_reference_timeout(),
        }
    }
}

/// 每 tick 的输出：当前模式该飞的参考 + 电机使能。
#[derive(Clone, Copy, Debug)]
pub struct Output {
    /// 当前模式下飞控该跟踪的参考（`Track` = 外部参考；其余 = 状态机自己生成）。
    pub setpoint: PositionSetpoint,
    /// 电机是否允许出力（`Disarmed` 为 `false` → 指令全零）。
    pub motors_enabled: bool,
    /// 本 tick 自动产生的决策（指令引起的转移由 [`FlightFsm::command`] 返回）。
    pub event: Event,
}

/// 状态机的内部决策（`update` 的返回值，供上层记录/上报）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// 无变化。
    None,
    /// 发生了状态转移（`from` → `to`）。
    Transition {
        /// 转移前状态。
        from: FlightState,
        /// 转移后状态。
        to: FlightState,
    },
    /// 失效保护触发了降落（估计失效）。
    FailsafeEstimatorLost,
    /// 参考流失联，回落位置保持。
    FailsafeReferenceLost,
}

/// 飞行状态机。
#[derive(Clone, Copy, Debug)]
pub struct FlightFsm {
    params: FsmParams,
    state: FlightState,
    /// 起飞点高度（世界 z，第一次 update 时锁存；`Arm` 时刷新）。
    origin_z: Option<f32>,
    /// 当前模式下发给位置环的目标位置（斜坡/保持点都由它承载）。
    target: Vec3,
    /// 目标航向（进入各模式时锁存；`Track` 由参考给）。
    target_yaw: f32,
    /// 目标高度（世界 z）：起飞/降落段按速率斜坡逼近它。
    ramp_z: f32,
    /// 起飞目标高度（相对起飞点，m）。
    takeoff_target: f32,
    /// 连续满足落地判据的时长（s）。
    ground_time: f32,
    /// 最近一次 `update` 是否拿到参考流（`Track` 的准入条件）。
    reference_fresh: bool,
    /// `Track` 下参考流连续缺席的时长（s）。
    reference_lost: f32,
}

impl FlightFsm {
    /// 以参数建状态机（初始 [`FlightState::Disarmed`]）。
    #[must_use]
    pub fn new(params: FsmParams) -> Self {
        Self {
            params,
            state: FlightState::Disarmed,
            origin_z: None,
            target: Vec3::ZERO,
            target_yaw: 0.0,
            ramp_z: 0.0,
            takeoff_target: params.takeoff_altitude,
            ground_time: 0.0,
            reference_fresh: false,
            reference_lost: 0.0,
        }
    }

    /// 当前状态。
    #[must_use]
    pub fn state(&self) -> FlightState {
        self.state
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
        self.altitude_above_origin(state) <= self.params.on_ground_altitude
            && state.velocity.z.abs() <= self.params.on_ground_velocity
    }

    /// 处理一条指令。
    ///
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
        match cmd {
            Command::Arm => self.arm(health, state),
            Command::Disarm => self.disarm(state),
            Command::Takeoff { altitude } => self.takeoff(altitude, state),
            Command::Hold => self.hold(state),
            Command::Track => self.track(),
            Command::Land => self.land(state),
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
        // 起飞点锁存：首次 update 时以当前位置为准（上电停在停机坪），Arm 时刷新
        if self.origin_z.is_none() {
            self.origin_z = Some(state.position.z);
            self.target = state.position;
            self.ramp_z = state.position.z;
            self.target_yaw = crate::state::yaw_of(state.attitude);
        }
        self.reference_fresh = reference.is_some();
        let event = self.step(dt, state, reference, health);
        Output {
            setpoint: self.output_setpoint(dt, state, reference),
            motors_enabled: self.state.motors_enabled(),
            event,
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
        match self.state {
            // 无定时转移：只等指令
            FlightState::Disarmed | FlightState::ArmedGrounded | FlightState::Hold => {}
            FlightState::Takeoff => {
                let target_z = self.origin_z.unwrap_or(0.0) + self.takeoff_target;
                let reached = (state.position.z - target_z).abs()
                    <= self.params.takeoff_complete_frac * self.takeoff_target;
                let slow = state.velocity.z.abs()
                    <= self.params.takeoff_complete_climb_frac * self.params.takeoff_climb_rate;
                if reached && slow {
                    self.enter_hold(state);
                }
            }
            FlightState::Track => {
                if reference.is_none() {
                    self.reference_lost += dt;
                    if self.reference_lost >= self.params.reference_timeout {
                        self.enter_hold(state);
                        return Event::FailsafeReferenceLost;
                    }
                } else {
                    self.reference_lost = 0.0;
                }
            }
            FlightState::Land => {
                if self.on_ground(state) {
                    self.ground_time += dt;
                    if self.ground_time >= self.params.land_complete_hold {
                        self.state = FlightState::Disarmed;
                    }
                } else {
                    self.ground_time = 0.0;
                }
            }
        }
        // 估计失效 → 降落（唯一终端安全动作；对照 `PX4` 的 Hold→RTL→Land 升级链，
        // 我们暂无返航/地理围栏，直接落 Land）
        if self.state.motors_enabled() && !health.estimator_ready {
            self.state = FlightState::Land;
            self.enter_land(state);
            return Event::FailsafeEstimatorLost;
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
                self.ramp_toward(goal, self.params.takeoff_climb_rate * dt, true);
                self.ramp_setpoint()
            }
            FlightState::Land => {
                let goal = self.origin_z.unwrap_or(0.0);
                self.ramp_toward(goal, self.params.land_descent_rate * dt, false);
                self.ramp_setpoint()
            }
            FlightState::Hold | FlightState::ArmedGrounded => {
                self.ramp_z = state.position.z;
                self.target.z = state.position.z;
                self.ramp_setpoint()
            }
            FlightState::Disarmed => PositionSetpoint {
                position: state.position,
                yaw: crate::state::yaw_of(state.attitude),
                ..PositionSetpoint::default()
            },
        }
    }

    /// 目标高度按速率斜坡向 `goal` 推进（`ascend` 决定方向；不越过 `goal`）。
    fn ramp_toward(&mut self, goal: f32, step: f32, ascend: bool) {
        let delta = goal - self.ramp_z;
        let limited = if ascend {
            delta.min(step).max(0.0)
        } else {
            delta.max(-step).min(0.0)
        };
        self.ramp_z += limited;
    }

    /// 斜坡/保持段的参考：水平位置锁 `target`，高度取斜坡值，垂速由斜坡隐含。
    fn ramp_setpoint(&self) -> PositionSetpoint {
        PositionSetpoint {
            position: Vec3::new(self.target.x, self.target.y, self.ramp_z),
            velocity: Vec3::ZERO,
            yaw: self.target_yaw,
            yaw_rate: 0.0,
        }
    }

    fn arm(&mut self, health: Health, state: &QuadState) -> Result<(), Reject> {
        if self.state != FlightState::Disarmed {
            return Err(Reject::WrongState);
        }
        if !health.estimator_ready {
            return Err(Reject::EstimatorNotReady);
        }
        if !health.airframe_ready {
            return Err(Reject::NoAirframe);
        }
        if !health.plant_alive {
            return Err(Reject::PlantStale);
        }
        if !health.imu_alive {
            return Err(Reject::ImuStale);
        }
        if !self.on_ground(state) {
            return Err(Reject::NotOnGround);
        }
        // 起飞点以解锁时刻为准（`ArduPilot` 的 home 语义）
        self.origin_z = Some(state.position.z);
        self.target = state.position;
        self.ramp_z = state.position.z;
        self.target_yaw = crate::state::yaw_of(state.attitude);
        self.state = FlightState::ArmedGrounded;
        Ok(())
    }

    fn disarm(&mut self, state: &QuadState) -> Result<(), Reject> {
        if !self.state.motors_enabled() {
            return Err(Reject::WrongState);
        }
        // 仅允许在地面上锁（空中上锁 = 摔机，留作显式的强制通道另议）
        if !self.on_ground(state) {
            return Err(Reject::NotOnGround);
        }
        self.state = FlightState::Disarmed;
        self.ground_time = 0.0;
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
        // 不带高度（<= 0 / 非有限）时用缺省高度
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
        self.state = FlightState::Takeoff;
        Ok(())
    }

    /// 对所有已解锁状态开放：中止当前段 → 原地保持。
    ///
    /// 失效保护强制进入的 [`FlightState::Land`] 在条件未消失时会被下一 tick 重新拉回，
    /// 所以 `Hold` 只能中止被指令触发的降落。
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
            FlightState::Takeoff | FlightState::Land => return Err(Reject::WrongState),
        }
        // 参考流必须在（否则一进 Track 就立刻失联回落，等于没接管）
        if !self.reference_fresh {
            return Err(Reject::ReferenceNotReady);
        }
        self.state = FlightState::Track;
        self.reference_lost = 0.0;
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
        self.state = FlightState::Hold;
        self.target = state.position;
        self.ramp_z = state.position.z;
        self.target_yaw = crate::state::yaw_of(state.attitude);
    }

    fn enter_land(&mut self, state: &QuadState) {
        self.state = FlightState::Land;
        // 降在起飞点正上方（x/y 取起飞点；我们的 origin 只存了高度，故取当前水平位置）
        self.target = state.position;
        self.ramp_z = self.ramp_z.max(state.position.z);
        self.ground_time = 0.0;
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
    use crate::G;

    const DT: f32 = 0.005;

    fn ok_health() -> Health {
        Health {
            estimator_ready: true,
            airframe_ready: true,
            plant_alive: true,
            imu_alive: true,
        }
    }

    fn hovering(z: f32) -> QuadState {
        QuadState {
            position: Vec3::new(1.0, 2.0, z),
            ..QuadState::default()
        }
    }

    /// 起解锁/起飞/推进的脚手架：把状态机输出当作"完美位置环"（直接吸附参考）。
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
            // 完美位置环：位置/速度直接吃参考（起飞/降落的斜坡因此成为真实轨迹）
            self.state.position = out.setpoint.position;
            self.state.velocity = Vec3::ZERO;
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
                Reject::EstimatorNotReady,
            ),
            (
                Health {
                    airframe_ready: false,
                    ..ok_health()
                },
                Reject::NoAirframe,
            ),
            (
                Health {
                    plant_alive: false,
                    ..ok_health()
                },
                Reject::PlantStale,
            ),
            (
                Health {
                    imu_alive: false,
                    ..ok_health()
                },
                Reject::ImuStale,
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

    /// 空中不许解锁（对照 `ArduPilot` 的 inflight-arming 禁止）。
    #[test]
    fn arm_in_air_is_rejected() {
        let mut fsm = FlightFsm::new(FsmParams::default());
        let ground = hovering(0.02);
        let _ = fsm.update(DT, &ground, None, ok_health());
        let air = hovering(1.0);
        assert_eq!(
            fsm.command(Command::Arm, ok_health(), &air),
            Err(Reject::NotOnGround)
        );
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

        // 斜坡限速：单 tick 的高度增量不超过爬升率×dt
        let before = rig.state.position.z;
        rig.tick();
        let step = rig.state.position.z - before;
        assert!(step <= 0.6 * DT + 1e-6, "斜坡过陡：{step}");

        // 爬到目标附近 + 垂速足够小 → 自动转 Hold（`ArduPilot` 判据）
        rig.ticks(1000);
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        assert!(
            (rig.fsm.altitude_above_origin(&rig.state) - 1.0).abs() < 0.11,
            "起飞后高度 {:.3}",
            rig.fsm.altitude_above_origin(&rig.state)
        );
    }

    /// 起飞未完成（高度停在半路）时不得转 Hold，也不得被 Track 抢走。
    #[test]
    fn takeoff_incomplete_stays_in_takeoff() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(20); // 只爬了一小段
        assert_eq!(rig.fsm.state(), FlightState::Takeoff);
        assert_eq!(rig.cmd(Command::Track), Err(Reject::WrongState));
    }

    /// 起飞高度范围与默认高度（`Takeoff` 指令不带高度时用默认值）。
    #[test]
    fn takeoff_without_altitude_uses_default() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        // 高度 <= 0（或非有限）= 用缺省高度（对照 `MAVLink` `NAV_TAKEOFF` param7 缺省语义）
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
        assert_eq!(rig.cmd(Command::Disarm), Err(Reject::NotOnGround));

        rig.cmd(Command::Land).unwrap();
        assert_eq!(rig.fsm.state(), FlightState::Land);
        rig.ticks(2000);
        assert_eq!(rig.fsm.state(), FlightState::Disarmed, "落地后应自动上锁");
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

    /// 参考失联超过超时（`PX4` `COM_OF_LOSS_T` 类比）→ 回落 Hold，且保持点 = 失联瞬间位置。
    #[test]
    fn reference_loss_in_track_falls_back_to_hold_at_current_position() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(1000);
        rig.fsm.reference_fresh = true;
        rig.cmd(Command::Track).unwrap();

        let setpoint = PositionSetpoint {
            position: Vec3::new(3.0, 4.0, 1.02),
            velocity: Vec3::new(0.5, 0.0, 0.0),
            yaw: 0.3,
            yaw_rate: 0.0,
        };
        // 短暂提供参考：状态仍是 Track、参考被透传
        let mut out = rig.fsm.update(DT, &rig.state, Some(&setpoint), ok_health());
        assert_eq!(rig.fsm.state(), FlightState::Track);
        assert!((out.setpoint.position - setpoint.position).length() < 1e-6);
        out = rig.fsm.update(DT, &rig.state, Some(&setpoint), ok_health());
        assert!(
            (out.setpoint.yaw - 0.3).abs() < 1e-6,
            "{}",
            out.setpoint.yaw
        );

        // 参考消失：超时前仍在 Track，超时后回落 Hold 且保持点 = 当前位置
        let lost_at = rig.state.position;
        let mut event = Event::None;
        let mut ticks = 0;
        while ticks < 1000 {
            ticks += 1;
            event = rig.fsm.step(DT, &rig.state, None, ok_health());
            if event != Event::None {
                break;
            }
        }
        assert_eq!(event, Event::FailsafeReferenceLost);
        let timeout = FsmParams::default().reference_timeout;
        assert!(
            (ticks as f32 * DT - timeout).abs() <= DT,
            "失联判定用了 {:.3} s，期望 {timeout} s",
            ticks as f32 * DT
        );
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        let out = rig.fsm.update(DT, &rig.state, None, ok_health());
        assert!(
            (out.setpoint.position - lost_at).length() < 1e-3,
            "{:?}",
            out.setpoint.position
        );
        assert!(out.setpoint.velocity.length() < 1e-6);
    }

    /// 估计失效 → 强制降落（唯一终端安全动作），落地后自动上锁。
    #[test]
    fn estimator_loss_forces_land() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(1000);
        assert_eq!(rig.fsm.state(), FlightState::Hold);

        let bad = Health {
            estimator_ready: false,
            ..ok_health()
        };
        rig.fsm.update(DT, &rig.state, None, bad);
        assert_eq!(rig.fsm.state(), FlightState::Land);
        // 下降段：高度单调降低，且在落地判据满足前不降落完成
        let mut last = rig.state.position.z;
        for _ in 0..100 {
            let out = rig.fsm.update(DT, &rig.state, None, bad);
            rig.state.position = out.setpoint.position;
            assert!(rig.state.position.z <= last + 1e-9);
            last = rig.state.position.z;
        }
        assert!(rig.state.position.z > 0.02, "不应一步落地");
    }

    /// Hold 期间参考出现也不改变状态（外部控制须显式 Track 接管）。
    #[test]
    fn hold_does_not_auto_take_over_reference() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(1000);
        let setpoint = PositionSetpoint {
            position: Vec3::new(9.0, 9.0, 2.0),
            ..PositionSetpoint::default()
        };
        rig.fsm.reference_fresh = true;
        let out = rig.fsm.update(DT, &rig.state, Some(&setpoint), ok_health());
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        assert!((out.setpoint.position - rig.state.position).length() < 1e-3);
    }

    /// 状态变化必须上报事件：指令引起的转移由 `command` 返回，自动转移由 `update` 返回。
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

        // 起飞完成是自动转移，由 update 上报
        let mut event = Event::None;
        for _ in 0..1000 {
            let out = rig.fsm.update(DT, &rig.state, None, ok_health());
            rig.state.position = out.setpoint.position;
            rig.state.velocity = Vec3::ZERO;
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
        // 无决策的 tick 不报事件；已在该状态时重复指令也算无决策
        assert_eq!(
            rig.fsm.update(DT, &rig.state, None, ok_health()).event,
            Event::None
        );
        assert_eq!(
            rig.fsm.command(Command::Hold, ok_health(), &rig.state),
            Ok(Event::None)
        );
    }

    /// `Hold` 对所有已解锁状态开放：中止起飞/降落段并原地保持。
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
        // 中止后不再爬升（保持点 = 中止瞬间的位置）
        let held = rig.state.position;
        rig.ticks(200);
        assert!((rig.state.position.z - held.z).abs() < 1e-6);

        // 降落同样可中止，但失效保护强制进入的降落不可被绕过：
        // 条件未消失时下一 tick 就会被拉回 Land
        rig.cmd(Command::Land).unwrap();
        assert_eq!(rig.fsm.state(), FlightState::Land);
        let bad = Health {
            estimator_ready: false,
            ..ok_health()
        };
        rig.fsm.update(DT, &rig.state, None, bad);
        assert_eq!(rig.fsm.state(), FlightState::Land);
        rig.cmd(Command::Hold).unwrap();
        assert_eq!(rig.fsm.state(), FlightState::Hold);
        rig.fsm.update(DT, &rig.state, None, bad);
        assert_eq!(rig.fsm.state(), FlightState::Land);
    }

    /// 上锁状态电机不出力；解锁后出力（输出契约）。
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

    /// 悬停平衡点自检：状态机在 Hold 下给出"停在原地"的参考，位置模式不产生水平力。
    #[test]
    fn hold_setpoint_is_stationary() {
        let mut rig = Rig::new();
        rig.cmd(Command::Arm).unwrap();
        rig.cmd(Command::Takeoff { altitude: 1.0 }).unwrap();
        rig.ticks(1000);
        let out = rig.fsm.update(DT, &rig.state, None, ok_health());
        assert_eq!(out.setpoint.velocity, Vec3::ZERO);
        let wrench = crate::position_mode(
            &rig.state,
            &out.setpoint,
            &crate::QuadParams::default(),
            &crate::ControlParams::default(),
        );
        assert!(
            wrench.force.truncate().length() < 1e-3,
            "{:?}",
            wrench.force
        );
        assert!(
            (wrench.force.z - 0.219 * G).abs() < 1e-3,
            "{:?}",
            wrench.force
        );
    }
}
