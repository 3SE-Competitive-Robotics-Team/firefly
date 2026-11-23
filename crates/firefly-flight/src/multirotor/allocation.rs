//! 固定 X 四旋翼分配；轴顺序与归一化约定见 [`super::types`]。
//! 分配矩阵为定长 `4×4`、`f32`，分配路径不做堆分配。

use core::ops::{Deref, DerefMut};

#[allow(clippy::wildcard_imports)]
use super::types::*;

/// 控制轴保留优先级依次为垂直推力、滚转、俯仰和偏航。
const AXIS_PRIORITY: [usize; NUM_AXES] = [AXIS_THRUST_Z, AXIS_ROLL, AXIS_PITCH, AXIS_YAW];

/// 轴保留所需的最小独立度：单位轴行去掉高优先级轴投影后的范数平方（无量纲，约 5.7°）。
/// 该阈值把伪逆沿该轴的增益限制在约 10 倍以内。
const MIN_AXIS_INDEPENDENCE: f32 = 1e-2;

/// 由固定 X 四旋翼的几何生成效能矩阵（旋翼轴恒为 FRD `(0, 0, -1)`，推力系数按单旋翼最大推力归一为 1）。
///
/// - `rotor_positions`：旋翼位置，单位 m，FRD（x 前、y 右、z 下）；z 分量不影响力臂。
/// - `rotor_spins`：旋翼转向，`+1` 表示该旋翼产生 FRD `+z` 反扭矩，`-1` 表示反向，无量纲。
/// - `torque_coefficient`：反扭矩系数，单位 m（每单位归一化推力的力矩）。
///
/// 行顺序为 `[roll, pitch, yaw, thrust_z]`：滚转/俯仰元为力臂 `-y_i` / `x_i`（单位 m），
/// 偏航元为 `torque_coefficient * spin_i`（单位 m），推力元为 `-1`（推力沿 FRD `-z`）。
/// 不足 4 个旋翼时剩余列为 0，多于 4 个时忽略。
#[must_use]
pub fn effectiveness_from_rotors(
    rotor_positions: &[[f32; 3]],
    rotor_spins: &[f32],
    torque_coefficient: f32,
) -> EffectivenessMatrix {
    let mut effectiveness = EffectivenessMatrix::zeros();
    let count = rotor_positions
        .len()
        .min(rotor_spins.len())
        .min(NUM_ACTUATORS);

    for i in 0..count {
        let position = rotor_positions[i];
        effectiveness[(AXIS_ROLL, i)] = -position[1];
        effectiveness[(AXIS_PITCH, i)] = position[0];
        effectiveness[(AXIS_YAW, i)] = torque_coefficient * rotor_spins[i];
        effectiveness[(AXIS_THRUST_Z, i)] = -1.0;
    }

    effectiveness
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlAxis {
    Roll = 0,
    Pitch = 1,
    Yaw = 2,
    ThrustZ = 3,
}

#[allow(clippy::struct_field_names)]
#[derive(Clone, Copy, Debug)]
pub struct ControlAllocation {
    effectiveness: EffectivenessMatrix,
    control_allocation_scale: ControlVector,
    actuator_trim: ActuatorVector,
    actuator_min: ActuatorVector,
    actuator_max: ActuatorVector,
    /// 速率限制：走完 `[min, max]` 允许的最短时间，单位 s；`0` 表示不限速。
    actuator_slew_rate_limit: ActuatorVector,
    prev_actuator_sp: ActuatorVector,
    /// 本次分配的执行器指令，`[0, 1]`；NaN 表示停桨。
    actuator_sp: ActuatorVector,
    control_sp: ControlVector,
    /// 配平/线性化点处的控制量 `effectiveness * linearization_point`。
    control_trim: ControlVector,
    normalize_rpy: bool,
    /// 因不可独立实现而被清零的轴位掩码，位上移量等于轴索引。
    dropped_axes: u8,
    /// 是否发生过执行器失效；失效后冻结归一化比例。
    had_actuator_failure: bool,
}

impl Default for ControlAllocation {
    fn default() -> Self {
        Self::new()
    }
}

impl ControlAllocation {
    #[must_use]
    pub fn new() -> Self {
        Self {
            effectiveness: EffectivenessMatrix::zeros(),
            control_allocation_scale: ControlVector::repeat(1.0),
            actuator_trim: ActuatorVector::zeros(),
            actuator_min: ActuatorVector::zeros(),
            actuator_max: ActuatorVector::repeat(1.0),
            actuator_slew_rate_limit: ActuatorVector::zeros(),
            prev_actuator_sp: ActuatorVector::zeros(),
            actuator_sp: ActuatorVector::zeros(),
            control_sp: ControlVector::zeros(),
            control_trim: ControlVector::zeros(),
            normalize_rpy: false,
            dropped_axes: 0,
            had_actuator_failure: false,
        }
    }

    /// 设置效能矩阵、配平与线性化点。
    ///
    /// 线性化点先限幅再加到配平上，配平本身再限幅一次；`control_trim` 取未清零轴之前的
    /// 矩阵与限幅线性化点之积。`_update_normalization_scale` 在基类中不参与计算，仅由
    /// [`ControlAllocationPseudoInverse::set_effectiveness_matrix`] 使用。
    pub fn set_effectiveness_matrix(
        &mut self,
        effectiveness: EffectivenessMatrix,
        actuator_trim: ActuatorVector,
        linearization_point: ActuatorVector,
        _update_normalization_scale: bool,
    ) {
        self.effectiveness = effectiveness;
        let linearization_point = self.clip_actuator_setpoint(linearization_point);
        self.actuator_trim = actuator_trim + linearization_point;
        self.actuator_trim = self.clip_actuator_setpoint(self.actuator_trim);
        self.control_trim = self.effectiveness * linearization_point;
    }

    #[must_use]
    pub fn actuator_setpoint(&self) -> ActuatorVector {
        self.actuator_sp
    }

    pub fn set_control_setpoint(&mut self, control: ControlVector) {
        self.control_sp = control;
    }

    #[must_use]
    pub fn control_setpoint(&self) -> ControlVector {
        self.control_sp
    }

    /// 已分配的控制量：`(B * (actuator_sp - actuator_trim)) ⊙ control_allocation_scale`。
    ///
    /// `actuator_sp` 中的 NaN（停桨）按 0 推力参与计算。
    #[must_use]
    pub fn allocated_control(&self) -> ControlVector {
        let actuator_sp = self
            .actuator_sp
            .map(|value| if value.is_finite() { value } else { 0.0 });
        (self.effectiveness * (actuator_sp - self.actuator_trim))
            .component_mul(&self.control_allocation_scale)
    }

    #[must_use]
    pub fn effectiveness_matrix(&self) -> EffectivenessMatrix {
        self.effectiveness
    }

    pub fn set_actuator_min(&mut self, actuator_min: ActuatorVector) {
        self.actuator_min = actuator_min;
    }

    #[must_use]
    pub fn actuator_min(&self) -> ActuatorVector {
        self.actuator_min
    }

    pub fn set_actuator_max(&mut self, actuator_max: ActuatorVector) {
        self.actuator_max = actuator_max;
    }

    #[must_use]
    pub fn actuator_max(&self) -> ActuatorVector {
        self.actuator_max
    }

    /// 设置当前执行器指令，`[0, 1]`，立即限幅。
    pub fn set_actuator_setpoint(&mut self, actuator_sp: ActuatorVector) {
        self.actuator_sp = actuator_sp;
        self.actuator_sp = self.clip_actuator_setpoint(self.actuator_sp);
    }

    /// 设置速率限制（走完 `[min, max]` 的最短时间，单位 s；`0` 表示不限速）。
    pub fn set_slew_rate_limit(&mut self, slew_rate_limit: ActuatorVector) {
        self.actuator_slew_rate_limit = slew_rate_limit;
    }

    /// 按 `dt`（单位 s）对执行器指令做速率限制。
    ///
    /// 每步最大变化量为 `dt * (max - min) / slew_rate_limit`。NaN 表示停桨：输入为 NaN
    /// 时按 0 推力限速，结果落到 0 后重新置为 NaN；上一次为 NaN 时按 0 起步。
    pub fn apply_slew_rate_limit(&mut self, dt: f32) {
        for i in 0..NUM_ACTUATORS {
            if self.actuator_slew_rate_limit[i] > f32::EPSILON {
                let mut input = self.actuator_sp[i];
                let mut previous = self.prev_actuator_sp[i];
                let input_is_nan = !is_finite(input);

                if input_is_nan {
                    input = 0.0;
                }

                if !is_finite(previous) {
                    previous = 0.0;
                }

                let delta_sp_max = dt * (self.actuator_max[i] - self.actuator_min[i])
                    / self.actuator_slew_rate_limit[i];
                let delta_sp = input - previous;
                let mut output = input;

                if delta_sp > delta_sp_max {
                    output = previous + delta_sp_max;
                } else if delta_sp < -delta_sp_max {
                    output = previous - delta_sp_max;
                }

                if input_is_nan && output.abs() < f32::EPSILON {
                    output = f32::NAN;
                }

                self.actuator_sp[i] = output;
            }
        }
    }

    /// 把执行器指令限幅到 `[min, max]`。
    ///
    /// `max < min` 的执行器取配平值；NaN 因比较为假而原样保留（表示停桨）。
    #[must_use]
    pub fn clip_actuator_setpoint(&self, mut actuator: ActuatorVector) -> ActuatorVector {
        for i in 0..NUM_ACTUATORS {
            if self.actuator_max[i] < self.actuator_min[i] {
                actuator[i] = self.actuator_trim[i];
            } else if actuator[i] < self.actuator_min[i] {
                actuator[i] = self.actuator_min[i];
            } else if actuator[i] > self.actuator_max[i] {
                actuator[i] = self.actuator_max[i];
            }
        }

        actuator
    }

    /// 把执行器指令归一化到 `[0, 1]`：`(a - min) / (max - min)`。
    ///
    /// `min >= max` 的执行器改用配平值代入同一式子（分母为 0 时结果为 NaN）。
    #[must_use]
    pub fn normalize_actuator_setpoint(&self, actuator: ActuatorVector) -> ActuatorVector {
        let mut actuator_normalized = ActuatorVector::zeros();

        for i in 0..NUM_ACTUATORS {
            if self.actuator_min[i] < self.actuator_max[i] {
                actuator_normalized[i] = (actuator[i] - self.actuator_min[i])
                    / (self.actuator_max[i] - self.actuator_min[i]);
            } else {
                actuator_normalized[i] = (self.actuator_trim[i] - self.actuator_min[i])
                    / (self.actuator_max[i] - self.actuator_min[i]);
            }
        }

        actuator_normalized
    }

    pub fn set_normalize_rpy(&mut self, normalize_rpy: bool) {
        self.normalize_rpy = normalize_rpy;
    }

    /// 被清零的轴位掩码，位上移量等于轴索引。
    #[must_use]
    pub fn dropped_axes(&self) -> u8 {
        self.dropped_axes
    }

    /// 设置执行器失效标志。
    ///
    /// 失效后不再更新归一化比例，避免故障前后指令量纲跳变。
    pub fn set_had_actuator_failure(&mut self, failure: bool) {
        self.had_actuator_failure = failure;
    }

    #[must_use]
    pub fn num_configured_actuators(&self) -> usize {
        NUM_ACTUATORS
    }

    pub fn update_parameters(&mut self) {}

    /// 把掩码对应的执行器指令置为 NaN。
    ///
    /// NaN 表示该电机停桨，由分配器与下游按无效值处理。
    pub fn apply_nan_to_actuators(&mut self, nan_actuators_mask: u32) {
        for i in 0..NUM_ACTUATORS {
            if nan_actuators_mask & (1_u32 << i) != 0 {
                self.actuator_sp[i] = f32::NAN;
            }
        }
    }
}

/// 伪逆分配 + 列归一化。
#[derive(Clone, Copy, Debug)]
pub struct ControlAllocationPseudoInverse {
    base: ControlAllocation,
    /// 混控矩阵（执行器 × 控制轴），`effectiveness` 的伪逆经归一化后的结果。
    mix: MixMatrix,
    mix_update_needed: bool,
    /// 比例分配（不做列归一化）。
    metric_allocation: bool,
    normalization_needs_update: bool,
}

impl Default for ControlAllocationPseudoInverse {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for ControlAllocationPseudoInverse {
    type Target = ControlAllocation;

    fn deref(&self) -> &Self::Target {
        &self.base
    }
}

impl DerefMut for ControlAllocationPseudoInverse {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.base
    }
}

impl ControlAllocationPseudoInverse {
    #[must_use]
    pub fn new() -> Self {
        Self {
            base: ControlAllocation::new(),
            mix: MixMatrix::zeros(),
            mix_update_needed: false,
            metric_allocation: false,
            normalization_needs_update: false,
        }
    }

    /// 设置效能矩阵并标记混控矩阵重算；同时按轴优先级清零不可独立实现的轴。
    pub fn set_effectiveness_matrix(
        &mut self,
        effectiveness: EffectivenessMatrix,
        actuator_trim: ActuatorVector,
        linearization_point: ActuatorVector,
        update_normalization_scale: bool,
    ) {
        self.base.set_effectiveness_matrix(
            effectiveness,
            actuator_trim,
            linearization_point,
            update_normalization_scale,
        );

        let dropped_axes = Self::drop_dependent_axes(&mut self.base.effectiveness);
        self.base.dropped_axes = dropped_axes;
        self.mix_update_needed = true;
        self.normalization_needs_update = update_normalization_scale;

        if self.metric_allocation && update_normalization_scale {
            self.normalization_needs_update = false;
        }
    }

    /// 比例分配开关：开启后跳过列归一化，比例保持不变。
    pub fn set_metric_allocation(&mut self, metric_allocation: bool) {
        self.metric_allocation = metric_allocation;
    }

    /// 分配控制指令到执行器。
    ///
    /// 结果未限幅，调用方按需调用 [`ControlAllocation::clip_actuator_setpoint`]。
    pub fn allocate(&mut self) {
        self.update_pseudo_inverse();

        self.base.prev_actuator_sp = self.base.actuator_sp;
        self.base.actuator_sp =
            self.base.actuator_trim + self.mix * (self.base.control_sp - self.base.control_trim);
    }

    /// 按优先级 `[thrust_z, roll, pitch, yaw]` 清零（近）线性相关的轴行。
    ///
    /// 用已接受单位轴行 Gram 矩阵的 Cholesky 因子做 Gram-Schmidt：轴行去掉高优先级轴投影后
    /// 的范数平方小于 `MIN_AXIS_INDEPENDENCE` 时整行清零，避免求近奇异矩阵的伪逆。
    ///
    /// 返回被清零轴的位掩码，位上移量等于轴索引。
    #[must_use]
    pub fn drop_dependent_axes(effectiveness: &mut EffectivenessMatrix) -> u8 {
        let mut chol = [[0.0_f32; NUM_AXES]; NUM_AXES];
        let mut inv_norm = [0.0_f32; NUM_AXES];
        let mut accepted = [0_usize; NUM_AXES];
        let mut num_accepted = 0_usize;
        let mut dropped = 0_u8;

        for axis in AXIS_PRIORITY {
            let norm_squared = effectiveness.row(axis).norm_squared();

            if norm_squared < f32::EPSILON {
                continue;
            }

            let inv = 1.0 / norm_squared.sqrt();
            let mut projection = [0.0_f32; NUM_AXES];
            let mut projected_norm_squared = 0.0_f32;

            for i in 0..num_accepted {
                let mut sum = effectiveness.row(axis).dot(&effectiveness.row(accepted[i]))
                    * inv
                    * inv_norm[i];

                for j in 0..i {
                    sum -= chol[i][j] * projection[j];
                }

                projection[i] = sum / chol[i][i];
                projected_norm_squared += projection[i] * projection[i];
            }

            let independence = 1.0 - projected_norm_squared;

            if independence < MIN_AXIS_INDEPENDENCE {
                effectiveness.row_mut(axis).fill(0.0);
                dropped |= 1_u8 << axis;
            } else {
                chol[num_accepted][..num_accepted].copy_from_slice(&projection[..num_accepted]);

                chol[num_accepted][num_accepted] = independence.sqrt();
                inv_norm[num_accepted] = inv;
                accepted[num_accepted] = axis;
                num_accepted += 1;
            }
        }

        dropped
    }

    /// 需要时重算混控矩阵与归一化比例。
    fn update_pseudo_inverse(&mut self) {
        if !self.mix_update_needed {
            return;
        }

        self.mix = geninv(&self.base.effectiveness);

        if !self.metric_allocation {
            if self.normalization_needs_update && !self.base.had_actuator_failure {
                self.update_control_allocation_matrix_scale();
                self.normalization_needs_update = false;
            }

            self.normalize_control_allocation_matrix();
        }

        self.mix_update_needed = false;
    }

    /// 计算每个控制轴的归一化比例。
    fn update_control_allocation_matrix_scale(&mut self) {
        let scale = &mut self.base.control_allocation_scale;

        if self.base.normalize_rpy {
            let mut num_non_zero_roll_torque = 0_i32;
            let mut num_non_zero_pitch_torque = 0_i32;

            for i in 0..NUM_ACTUATORS {
                if self.mix[(i, AXIS_ROLL)].abs() > 1e-3 {
                    num_non_zero_roll_torque += 1;
                }

                if self.mix[(i, AXIS_PITCH)].abs() > 1e-3 {
                    num_non_zero_pitch_torque += 1;
                }
            }

            let mut roll_norm_scale = 1.0_f32;

            if num_non_zero_roll_torque > 0 {
                roll_norm_scale = (self.mix.column(AXIS_ROLL).norm_squared()
                    / (num_non_zero_roll_torque as f32 / 2.0))
                    .sqrt();
            }

            let mut pitch_norm_scale = 1.0_f32;

            if num_non_zero_pitch_torque > 0 {
                pitch_norm_scale = (self.mix.column(AXIS_PITCH).norm_squared()
                    / (num_non_zero_pitch_torque as f32 / 2.0))
                    .sqrt();
            }

            scale[AXIS_ROLL] = roll_norm_scale.max(pitch_norm_scale);
            scale[AXIS_PITCH] = scale[AXIS_ROLL];
            scale[AXIS_YAW] = self.mix.column(AXIS_YAW).max();
        } else {
            scale[AXIS_ROLL] = 1.0;
            scale[AXIS_PITCH] = 1.0;
            scale[AXIS_YAW] = 1.0;
        }

        // 推力按各执行器推力轴分量的平均绝对值缩放；没有推力分量时保持 1。
        scale[AXIS_THRUST_Z] = 1.0;

        let mut num_non_zero_thrust = 0_i32;
        let mut norm_sum = 0.0_f32;

        for i in 0..NUM_ACTUATORS {
            let norm = self.mix[(i, AXIS_THRUST_Z)].abs();
            norm_sum += norm;

            if norm > f32::EPSILON {
                num_non_zero_thrust += 1;
            }
        }

        if num_non_zero_thrust > 0 {
            scale[AXIS_THRUST_Z] = norm_sum / num_non_zero_thrust as f32;
        }
    }

    /// 按归一化比例缩放混控矩阵列，并清零小元素。
    fn normalize_control_allocation_matrix(&mut self) {
        let scale = self.base.control_allocation_scale;

        if scale[AXIS_ROLL] > f32::EPSILON {
            self.mix
                .column_mut(AXIS_ROLL)
                .scale_mut(1.0 / scale[AXIS_ROLL]);
            self.mix
                .column_mut(AXIS_PITCH)
                .scale_mut(1.0 / scale[AXIS_PITCH]);
        }

        if scale[AXIS_YAW] > f32::EPSILON {
            self.mix
                .column_mut(AXIS_YAW)
                .scale_mut(1.0 / scale[AXIS_YAW]);
        }

        if scale[AXIS_THRUST_Z] > f32::EPSILON {
            self.mix
                .column_mut(AXIS_THRUST_Z)
                .scale_mut(1.0 / scale[AXIS_THRUST_Z]);
        }

        for i in 0..NUM_ACTUATORS {
            for j in 0..NUM_AXES {
                if self.mix[(i, j)].abs() < 1e-3 {
                    self.mix[(i, j)] = 0.0;
                }
            }
        }
    }
}

/// 效能矩阵的 Moore-Penrose 伪逆。
///
/// 4x4 非奇异时等于解析逆；秩亏时按 SVD 截断（阈值为 `max(m, n) * eps * σ_max`）。
/// 输入含非有限值或 SVD 未收敛时返回零矩阵，分配路径因此不会 panic。
fn geninv(effectiveness: &EffectivenessMatrix) -> MixMatrix {
    if !effectiveness.iter().all(|value| value.is_finite()) {
        return MixMatrix::zeros();
    }

    let svd = effectiveness.svd(true, true);
    let singular_values = svd.singular_values;
    let tolerance = singular_values[0].abs() * NUM_ACTUATORS.max(NUM_AXES) as f32 * f32::EPSILON;

    svd.pseudo_inverse(tolerance)
        .unwrap_or_else(|_| MixMatrix::zeros())
}

/// 混控模式：`0` 关闭、`1` 滚转/俯仰 airmode、`2` 全轴 airmode。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Airmode {
    /// 不用增推解饱和：滚转/俯仰按需削减，偏航只在必要时减推力。
    #[default]
    Disabled,
    /// 滚转/俯仰 airmode：允许改变总推力以满足滚转/俯仰，偏航不增推。
    RollPitch,
    /// 全轴 airmode：偏航也可改变总推力，但优先级低于滚转/俯仰。
    RollPitchYaw,
}

/// 沿轴依次平移执行器指令以消除饱和。
#[derive(Clone, Copy, Debug)]
pub struct ControlAllocationSequentialDesaturation {
    pseudo_inverse: ControlAllocationPseudoInverse,
    airmode: Airmode,
}

impl Default for ControlAllocationSequentialDesaturation {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for ControlAllocationSequentialDesaturation {
    type Target = ControlAllocationPseudoInverse;

    fn deref(&self) -> &Self::Target {
        &self.pseudo_inverse
    }
}

impl DerefMut for ControlAllocationSequentialDesaturation {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.pseudo_inverse
    }
}

impl ControlAllocationSequentialDesaturation {
    /// 偏航在饱和时仍保留的最小权限比例，无量纲。
    pub const MINIMUM_YAW_MARGIN: f32 = 0.15;

    #[must_use]
    pub fn new() -> Self {
        Self {
            pseudo_inverse: ControlAllocationPseudoInverse::new(),
            airmode: Airmode::Disabled,
        }
    }

    pub fn set_airmode(&mut self, airmode: Airmode) {
        self.airmode = airmode;
    }

    /// 分配控制指令到执行器。
    ///
    /// 结果未限幅；饱和已由解饱和过程处理。
    pub fn allocate(&mut self) {
        self.update_pseudo_inverse();

        self.base.prev_actuator_sp = self.base.actuator_sp;

        match self.airmode {
            Airmode::RollPitch => self.mix_airmode_rp(),
            Airmode::RollPitchYaw => self.mix_airmode_rpy(),
            Airmode::Disabled => self.mix_airmode_disabled(),
        }
    }

    /// 沿 `desaturation_vector` 平移执行器指令，把饱和量降到最低。
    ///
    /// 平移分两步：先消除最严重的饱和，再用半步增益平衡上下界。`increase_only` 为真时
    /// 只允许增推方向。
    fn desaturate_actuators(
        &self,
        actuator_sp: &mut ActuatorVector,
        desaturation_vector: &ActuatorVector,
        increase_only: bool,
    ) {
        let mut gain = self.compute_desaturation_gain(desaturation_vector, actuator_sp);

        if increase_only && gain < 0.0 {
            return;
        }

        for i in 0..NUM_ACTUATORS {
            actuator_sp[i] += gain * desaturation_vector[i];
        }

        gain = 0.5 * self.compute_desaturation_gain(desaturation_vector, actuator_sp);

        for i in 0..NUM_ACTUATORS {
            actuator_sp[i] += gain * desaturation_vector[i];
        }
    }

    /// 使最严重饱和回到边界所需的增益。
    ///
    /// 忽略 `|desaturation_vector| < 0.2` 的执行器，避免用弱效能执行器产生过大的增益。
    fn compute_desaturation_gain(
        &self,
        desaturation_vector: &ActuatorVector,
        actuator_sp: &ActuatorVector,
    ) -> f32 {
        let mut k_min = 0.0_f32;
        let mut k_max = 0.0_f32;

        for i in 0..NUM_ACTUATORS {
            if desaturation_vector[i].abs() < 0.2 {
                continue;
            }

            if actuator_sp[i] < self.base.actuator_min[i] {
                let k = (self.base.actuator_min[i] - actuator_sp[i]) / desaturation_vector[i];

                if k < k_min {
                    k_min = k;
                }

                if k > k_max {
                    k_max = k;
                }
            }

            if actuator_sp[i] > self.base.actuator_max[i] {
                let k = (self.base.actuator_max[i] - actuator_sp[i]) / desaturation_vector[i];

                if k < k_min {
                    k_min = k;
                }

                if k > k_max {
                    k_max = k;
                }
            }
        }

        k_min + k_max
    }

    /// 滚转/俯仰 airmode 混控：偏航单独叠加。
    fn mix_airmode_rp(&mut self) {
        let mut actuator_sp = ActuatorVector::zeros();
        let mut thrust_z = ActuatorVector::zeros();

        for i in 0..NUM_ACTUATORS {
            actuator_sp[i] = self.base.actuator_trim[i]
                + self.mix[(i, AXIS_ROLL)]
                    * (self.base.control_sp[AXIS_ROLL] - self.base.control_trim[AXIS_ROLL])
                + self.mix[(i, AXIS_PITCH)]
                    * (self.base.control_sp[AXIS_PITCH] - self.base.control_trim[AXIS_PITCH])
                + self.mix[(i, AXIS_THRUST_Z)]
                    * (self.base.control_sp[AXIS_THRUST_Z] - self.base.control_trim[AXIS_THRUST_Z]);
            thrust_z[i] = self.mix[(i, AXIS_THRUST_Z)];
        }

        self.desaturate_actuators(&mut actuator_sp, &thrust_z, false);
        self.base.actuator_sp = actuator_sp;

        self.mix_yaw();
    }

    /// 全轴 airmode 混控：先整体混控，再按推力、偏航顺序解饱和。
    fn mix_airmode_rpy(&mut self) {
        let mut actuator_sp = ActuatorVector::zeros();
        let mut thrust_z = ActuatorVector::zeros();
        let mut yaw = ActuatorVector::zeros();

        for i in 0..NUM_ACTUATORS {
            actuator_sp[i] = self.base.actuator_trim[i]
                + self.mix[(i, AXIS_ROLL)]
                    * (self.base.control_sp[AXIS_ROLL] - self.base.control_trim[AXIS_ROLL])
                + self.mix[(i, AXIS_PITCH)]
                    * (self.base.control_sp[AXIS_PITCH] - self.base.control_trim[AXIS_PITCH])
                + self.mix[(i, AXIS_YAW)]
                    * (self.base.control_sp[AXIS_YAW] - self.base.control_trim[AXIS_YAW])
                + self.mix[(i, AXIS_THRUST_Z)]
                    * (self.base.control_sp[AXIS_THRUST_Z] - self.base.control_trim[AXIS_THRUST_Z]);
            thrust_z[i] = self.mix[(i, AXIS_THRUST_Z)];
            yaw[i] = self.mix[(i, AXIS_YAW)];
        }

        self.desaturate_actuators(&mut actuator_sp, &thrust_z, false);

        // 偏航可能突破上下界，再沿偏航轴解一次，保证滚转/俯仰优先。
        self.desaturate_actuators(&mut actuator_sp, &yaw, false);
        self.base.actuator_sp = actuator_sp;
    }

    /// 关闭 airmode 的混控：绝不增推解饱和，按推力、滚转、俯仰顺序削减。
    fn mix_airmode_disabled(&mut self) {
        let mut actuator_sp = ActuatorVector::zeros();
        let mut thrust_z = ActuatorVector::zeros();
        let mut roll = ActuatorVector::zeros();
        let mut pitch = ActuatorVector::zeros();

        for i in 0..NUM_ACTUATORS {
            actuator_sp[i] = self.base.actuator_trim[i]
                + self.mix[(i, AXIS_ROLL)]
                    * (self.base.control_sp[AXIS_ROLL] - self.base.control_trim[AXIS_ROLL])
                + self.mix[(i, AXIS_PITCH)]
                    * (self.base.control_sp[AXIS_PITCH] - self.base.control_trim[AXIS_PITCH])
                + self.mix[(i, AXIS_THRUST_Z)]
                    * (self.base.control_sp[AXIS_THRUST_Z] - self.base.control_trim[AXIS_THRUST_Z]);
            thrust_z[i] = self.mix[(i, AXIS_THRUST_Z)];
            roll[i] = self.mix[(i, AXIS_ROLL)];
            pitch[i] = self.mix[(i, AXIS_PITCH)];
        }

        self.desaturate_actuators(&mut actuator_sp, &thrust_z, true);

        self.desaturate_actuators(&mut actuator_sp, &roll, false);
        self.desaturate_actuators(&mut actuator_sp, &pitch, false);
        self.base.actuator_sp = actuator_sp;

        self.mix_yaw();
    }

    /// 在已含滚转/俯仰/推力的执行器指令上叠加偏航并解饱和。
    ///
    /// 上限临时放宽 [`Self::MINIMUM_YAW_MARGIN`]，让满推力时仍保留部分偏航权限；
    /// 之后只允许减推力，不允许增推。
    fn mix_yaw(&mut self) {
        let mut yaw = ActuatorVector::zeros();
        let mut thrust_z = ActuatorVector::zeros();
        let mut actuator_sp = self.base.actuator_sp;

        for i in 0..NUM_ACTUATORS {
            actuator_sp[i] += self.mix[(i, AXIS_YAW)]
                * (self.base.control_sp[AXIS_YAW] - self.base.control_trim[AXIS_YAW]);
            yaw[i] = self.mix[(i, AXIS_YAW)];
            thrust_z[i] = self.mix[(i, AXIS_THRUST_Z)];
        }

        let max_prev = self.base.actuator_max;
        let yaw_margin =
            (self.base.actuator_max - self.base.actuator_min) * Self::MINIMUM_YAW_MARGIN;
        self.base.actuator_max += yaw_margin;
        self.desaturate_actuators(&mut actuator_sp, &yaw, false);
        self.base.actuator_max = max_prev;

        self.desaturate_actuators(&mut actuator_sp, &thrust_z, true);
        self.base.actuator_sp = actuator_sp;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// X 型四旋翼的测试几何：
    /// 旋翼位置 `(±1, ±1, 0)` m，反扭矩系数 `±1` m（顺时针编号）。
    fn quad_x_effectiveness() -> EffectivenessMatrix {
        effectiveness_from_rotors(
            &[
                [1.0, 1.0, 0.0],
                [-1.0, 1.0, 0.0],
                [-1.0, -1.0, 0.0],
                [1.0, -1.0, 0.0],
            ],
            &[1.0, -1.0, 1.0, -1.0],
            1.0,
        )
    }

    fn quad_x_allocation() -> ControlAllocationSequentialDesaturation {
        let mut allocation = ControlAllocationSequentialDesaturation::new();
        allocation.set_effectiveness_matrix(
            quad_x_effectiveness(),
            ActuatorVector::zeros(),
            ActuatorVector::zeros(),
            true,
        );
        allocation
    }

    fn allocate_quad(
        allocation: &mut ControlAllocationSequentialDesaturation,
        roll: f32,
        pitch: f32,
        yaw: f32,
        thrust_z: f32,
    ) -> ActuatorVector {
        let mut control = ControlVector::zeros();
        control[AXIS_ROLL] = roll;
        control[AXIS_PITCH] = pitch;
        control[AXIS_YAW] = yaw;
        control[AXIS_THRUST_Z] = thrust_z;
        allocation.set_control_setpoint(control);
        allocation.allocate();
        allocation.actuator_setpoint()
    }

    fn assert_actuator_equals(actual: ActuatorVector, expected: [f32; 4], context: &str) {
        for i in 0..NUM_ACTUATORS {
            assert!(
                (actual[i] - expected[i]).abs() <= 1e-5,
                "{context} 执行器 {i}: 实际 {} 期望 {}",
                actual[i],
                expected[i]
            );
        }
    }

    #[test]
    fn effectiveness_matrix_of_x_quad() {
        let effectiveness = quad_x_effectiveness();

        // 滚转 = -y，俯仰 = x，偏航 = 反扭矩系数 * 转向，推力 = -1（沿 FRD -z）。
        let expected = [
            [-1.0, -1.0, 1.0, 1.0],
            [1.0, -1.0, -1.0, 1.0],
            [1.0, -1.0, 1.0, -1.0],
            [-1.0, -1.0, -1.0, -1.0],
        ];

        for axis in 0..NUM_AXES {
            for i in 0..NUM_ACTUATORS {
                let actual = effectiveness[(axis, i)];
                assert!(
                    (actual - expected[axis][i]).abs() <= 1e-6,
                    "轴 {axis} 执行器 {i}: {actual} != {}",
                    expected[axis][i]
                );
            }
        }
    }

    #[test]
    fn hover_command_gives_equal_motor_outputs() {
        let mut allocation = quad_x_allocation();

        for step in 0..=10 {
            let thrust = step as f32 * 0.1;
            let outputs = allocate_quad(&mut allocation, 0.0, 0.0, 0.0, -thrust);

            for i in 0..NUM_ACTUATORS {
                assert!(
                    (outputs[i] - thrust).abs() <= 1e-5,
                    "推力 {thrust} 的执行器 {i}: {}",
                    outputs[i]
                );
            }
        }
    }

    #[test]
    fn collective_thrust_scales_linearly() {
        let mut allocation = quad_x_allocation();
        let mut thrust = 0.0_f32;

        while thrust <= 1.0 {
            let outputs = allocate_quad(&mut allocation, 0.0, 0.0, 0.0, -thrust);
            assert_actuator_equals(outputs, [thrust; 4], "总推力");
            thrust += 0.1;
        }
    }

    #[test]
    fn roll_pitch_yaw_allocation() {
        let mut allocation = quad_x_allocation();

        assert_actuator_equals(
            allocate_quad(&mut allocation, 1.0, 0.0, 0.0, -0.5),
            [0.25, 0.25, 0.75, 0.75],
            "滚转 +1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, -1.0, 0.0, 0.0, -0.5),
            [0.75, 0.75, 0.25, 0.25],
            "滚转 -1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, 1.0, 0.0, -0.5),
            [0.75, 0.25, 0.25, 0.75],
            "俯仰 +1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, -1.0, 0.0, -0.5),
            [0.25, 0.75, 0.75, 0.25],
            "俯仰 -1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, 0.0, 1.0, -0.5),
            [0.75, 0.25, 0.75, 0.25],
            "偏航 +1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, 0.0, -1.0, -0.5),
            [0.25, 0.75, 0.25, 0.75],
            "偏航 -1",
        );
    }

    #[test]
    fn roll_pitch_yaw_at_full_thrust() {
        let mut allocation = quad_x_allocation();

        assert_actuator_equals(
            allocate_quad(&mut allocation, 1.0, 0.0, 0.0, -1.0),
            [0.5, 0.5, 1.0, 1.0],
            "满推力滚转 +1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, -1.0, 0.0, 0.0, -1.0),
            [1.0, 1.0, 0.5, 0.5],
            "满推力滚转 -1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, 1.0, 0.0, -1.0),
            [1.0, 0.5, 0.5, 1.0],
            "满推力俯仰 +1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, -1.0, 0.0, -1.0),
            [0.5, 1.0, 1.0, 0.5],
            "满推力俯仰 -1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, 0.0, 1.0, -1.0),
            [1.0, 0.7, 1.0, 0.7],
            "满推力偏航 +1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, 0.0, -1.0, -1.0),
            [0.7, 1.0, 0.7, 1.0],
            "满推力偏航 -1",
        );
    }

    #[test]
    fn zero_thrust_airmode_disabled_allocates_nothing() {
        let mut allocation = quad_x_allocation();

        for (roll, pitch, yaw) in [
            (1.0, 0.0, 0.0),
            (-1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, 0.0, -1.0),
        ] {
            assert_actuator_equals(
                allocate_quad(&mut allocation, roll, pitch, yaw, 0.0),
                [0.0; 4],
                "零推力",
            );
        }
    }

    #[test]
    fn zero_thrust_airmode_roll_pitch() {
        let mut allocation = quad_x_allocation();
        allocation.set_airmode(Airmode::RollPitch);

        assert_actuator_equals(
            allocate_quad(&mut allocation, 1.0, 0.0, 0.0, 0.0),
            [0.0, 0.0, 0.5, 0.5],
            "滚转 +1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, -1.0, 0.0, 0.0, 0.0),
            [0.5, 0.5, 0.0, 0.0],
            "滚转 -1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, 1.0, 0.0, 0.0),
            [0.5, 0.0, 0.0, 0.5],
            "俯仰 +1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, -1.0, 0.0, 0.0),
            [0.0, 0.5, 0.5, 0.0],
            "俯仰 -1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, 0.0, 1.0, 0.0),
            [0.0; 4],
            "偏航 +1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, 0.0, -1.0, 0.0),
            [0.0; 4],
            "偏航 -1",
        );
    }

    #[test]
    fn zero_thrust_airmode_roll_pitch_yaw() {
        let mut allocation = quad_x_allocation();
        allocation.set_airmode(Airmode::RollPitchYaw);

        assert_actuator_equals(
            allocate_quad(&mut allocation, 1.0, 0.0, 0.0, 0.0),
            [0.0, 0.0, 0.5, 0.5],
            "滚转 +1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, -1.0, 0.0, 0.0, 0.0),
            [0.5, 0.5, 0.0, 0.0],
            "滚转 -1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, 1.0, 0.0, 0.0),
            [0.5, 0.0, 0.0, 0.5],
            "俯仰 +1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, -1.0, 0.0, 0.0),
            [0.0, 0.5, 0.5, 0.0],
            "俯仰 -1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, 0.0, 1.0, 0.0),
            [0.5, 0.0, 0.5, 0.0],
            "偏航 +1",
        );
        assert_actuator_equals(
            allocate_quad(&mut allocation, 0.0, 0.0, -1.0, 0.0),
            [0.0, 0.5, 0.0, 0.5],
            "偏航 -1",
        );
    }

    #[test]
    fn thrust_and_yaw_partial() {
        let mut allocation = quad_x_allocation();
        let thrust = 0.75_f32;
        let yaw_torque = 0.02_f32;
        let yaw = yaw_torque / NUM_ACTUATORS as f32;
        let outputs = allocate_quad(&mut allocation, 0.0, 0.0, yaw_torque, -thrust);
        assert_actuator_equals(
            outputs,
            [thrust + yaw, thrust - yaw, thrust + yaw, thrust - yaw],
            "推力 + 偏航",
        );
    }

    #[test]
    fn saturated_yaw_keeps_total_thrust() {
        let mut allocation = quad_x_allocation();
        let thrust = 0.75_f32;
        let outputs = allocate_quad(&mut allocation, 0.0, 0.0, 1.0, -thrust);

        assert_actuator_equals(outputs, [1.0, 0.5, 1.0, 0.5], "饱和偏航");

        let total: f32 = outputs.iter().sum();
        assert!(
            (total - 4.0 * thrust).abs() <= 1e-5,
            "总推力 {total} != {}",
            4.0 * thrust
        );
    }

    #[test]
    fn thrust_and_pitch_partial() {
        let mut allocation = quad_x_allocation();
        let thrust = 0.75_f32;
        let pitch = 0.025_f32; // 每个旋翼 0.1 / 4 的俯仰分量
        let outputs = allocate_quad(&mut allocation, 0.0, 0.1, 0.0, -thrust);
        assert_actuator_equals(
            outputs,
            [
                thrust + pitch,
                thrust - pitch,
                thrust - pitch,
                thrust + pitch,
            ],
            "推力 + 俯仰",
        );
    }

    #[test]
    fn reduced_thrust_for_yaw() {
        let mut allocation = quad_x_allocation();
        let margin = ControlAllocationSequentialDesaturation::MINIMUM_YAW_MARGIN;
        let outputs = allocate_quad(&mut allocation, 0.0, 0.0, 1.0, -3.2);
        assert_actuator_equals(
            outputs,
            [1.0, 1.0 - 2.0 * margin, 1.0, 1.0 - 2.0 * margin],
            "大推力偏航",
        );
    }

    #[test]
    fn reduced_thrust_for_pitch() {
        let mut allocation = quad_x_allocation();
        let outputs = allocate_quad(&mut allocation, 0.0, 2.0, 0.0, -3.0);
        assert_actuator_equals(outputs, [1.0, 0.0, 0.0, 1.0], "大推力俯仰");
    }

    /// 关闭 airmode 时的混控输入/输出表：
    /// `(roll, pitch, yaw, thrust_z)` → 4 个执行器指令。
    const PREVIOUS_MIXING_AIRMODE_DISABLED: [(f32, f32, f32, f32, [f32; 4]); 65] = [
        (0.000, 0.000, 0.000, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (0.000, 0.000, 0.000, -0.100, [0.1, 0.1, 0.1, 0.1]),
        (0.000, 0.000, 0.000, -0.450, [0.45, 0.45, 0.45, 0.45]),
        (0.000, 0.000, 0.000, -0.900, [0.9, 0.9, 0.9, 0.9]),
        (0.000, 0.000, 0.000, -1.000, [1.0, 1.0, 1.0, 1.0]),
        (-0.050, 0.000, 0.000, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (
            -0.050,
            0.000,
            0.000,
            -0.100,
            [0.1125, 0.1125, 0.0875, 0.0875],
        ),
        (
            -0.050,
            0.000,
            0.000,
            -0.450,
            [0.4625, 0.4625, 0.4375, 0.4375],
        ),
        (
            -0.050,
            0.000,
            0.000,
            -0.900,
            [0.9125, 0.9125, 0.8875, 0.8875],
        ),
        (-0.050, 0.000, 0.000, -1.000, [1.0, 1.0, 0.975, 0.975]),
        (0.050, -0.050, 0.000, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (0.050, -0.050, 0.000, -0.100, [0.075, 0.1, 0.125, 0.1]),
        (0.050, -0.050, 0.000, -0.450, [0.425, 0.45, 0.475, 0.45]),
        (0.050, -0.050, 0.000, -0.900, [0.875, 0.9, 0.925, 0.9]),
        (0.050, -0.050, 0.000, -1.000, [0.95, 0.975, 1.0, 0.975]),
        (0.050, 0.050, -0.025, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (
            0.050,
            0.050,
            -0.025,
            -0.100,
            [0.09375, 0.08125, 0.09375, 0.13125],
        ),
        (
            0.050,
            0.050,
            -0.025,
            -0.450,
            [0.44375, 0.43125, 0.44375, 0.48125],
        ),
        (
            0.050,
            0.050,
            -0.025,
            -0.900,
            [0.89375, 0.88125, 0.89375, 0.93125],
        ),
        (0.050, 0.050, -0.025, -1.000, [0.9625, 0.95, 0.9625, 1.0]),
        (0.000, 0.200, -0.025, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (
            0.000,
            0.200,
            -0.025,
            -0.100,
            [0.14375, 0.05625, 0.04375, 0.15625],
        ),
        (
            0.000,
            0.200,
            -0.025,
            -0.450,
            [0.49375, 0.40625, 0.39375, 0.50625],
        ),
        (
            0.000,
            0.200,
            -0.025,
            -0.900,
            [0.94375, 0.85625, 0.84375, 0.95625],
        ),
        (0.000, 0.200, -0.025, -1.000, [0.9875, 0.9, 0.8875, 1.0]),
        (0.200, 0.050, 0.090, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (0.200, 0.050, 0.090, -0.100, [0.085, 0.015, 0.16, 0.14]),
        (0.200, 0.050, 0.090, -0.450, [0.435, 0.365, 0.51, 0.49]),
        (0.200, 0.050, 0.090, -0.900, [0.885, 0.815, 0.96, 0.94]),
        (
            0.200,
            0.050,
            0.090,
            -1.000,
            [0.9225, 0.8525, 0.9975, 0.9775],
        ),
        (-0.125, 0.020, 0.040, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (
            -0.125,
            0.020,
            0.040,
            -0.100,
            [0.14625, 0.11625, 0.07375, 0.06375],
        ),
        (
            -0.125,
            0.020,
            0.040,
            -0.450,
            [0.49625, 0.46625, 0.42375, 0.41375],
        ),
        (
            -0.125,
            0.020,
            0.040,
            -0.900,
            [0.94625, 0.91625, 0.87375, 0.86375],
        ),
        (-0.125, 0.020, 0.040, -1.000, [1.0, 0.97, 0.9275, 0.9175]),
        (1.000, 0.000, 0.000, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (1.000, 0.000, 0.000, -0.100, [0.0, 0.0, 0.2, 0.2]),
        (1.000, 0.000, 0.000, -0.450, [0.2, 0.2, 0.7, 0.7]),
        (1.000, 0.000, 0.000, -0.900, [0.5, 0.5, 1.0, 1.0]),
        (1.000, 0.000, 0.000, -1.000, [0.5, 0.5, 1.0, 1.0]),
        (0.000, -1.000, 0.000, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (0.000, -1.000, 0.000, -0.100, [0.0, 0.2, 0.2, 0.0]),
        (0.000, -1.000, 0.000, -0.450, [0.2, 0.7, 0.7, 0.2]),
        (0.000, -1.000, 0.000, -0.900, [0.5, 1.0, 1.0, 0.5]),
        (0.000, -1.000, 0.000, -1.000, [0.5, 1.0, 1.0, 0.5]),
        (0.000, 0.000, 1.000, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (0.000, 0.000, 1.000, -0.100, [0.2, 0.0, 0.2, 0.0]),
        (0.000, 0.000, 1.000, -0.450, [0.7, 0.2, 0.7, 0.2]),
        (0.000, 0.000, 1.000, -0.900, [1.0, 0.5, 1.0, 0.5]),
        (0.000, 0.000, 1.000, -1.000, [1.0, 0.7, 1.0, 0.7]),
        (1.000, 1.000, -1.000, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (1.000, 1.000, -1.000, -0.100, [0.2, 0.0, 0.0, 0.2]),
        (1.000, 1.000, -1.000, -0.450, [0.1, 0.1, 0.0, 1.0]),
        (1.000, 1.000, -1.000, -0.900, [0.2, 0.0, 0.2, 1.0]),
        (1.000, 1.000, -1.000, -1.000, [0.2, 0.0, 0.2, 1.0]),
        (-1.000, 0.900, -0.900, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (-1.000, 0.900, -0.900, -0.100, [0.2, 0.0, 0.0, 0.2]),
        (-1.000, 0.900, -0.900, -0.450, [0.9, 0.45, 0.0, 0.45]),
        (-1.000, 0.900, -0.900, -0.900, [0.95, 0.6, 0.0, 0.55]),
        (-1.000, 0.900, -0.900, -1.000, [0.95, 0.6, 0.0, 0.55]),
        (-1.000, 0.900, 0.000, -0.000, [0.0, 0.0, 0.0, 0.0]),
        (-1.000, 0.900, 0.000, -0.100, [0.2, 0.0, 0.0, 0.2]),
        (-1.000, 0.900, 0.000, -0.450, [0.9, 0.45, 0.0, 0.45]),
        (-1.000, 0.900, 0.000, -0.900, [1.0, 0.55, 0.05, 0.5]),
        (-1.000, 0.900, 0.000, -1.000, [1.0, 0.55, 0.05, 0.5]),
    ];

    #[test]
    fn previous_mixing_table_airmode_disabled() {
        let mut allocation = quad_x_allocation();

        for (roll, pitch, yaw, thrust_z, expected) in PREVIOUS_MIXING_AIRMODE_DISABLED {
            let outputs = allocate_quad(&mut allocation, roll, pitch, yaw, thrust_z);
            assert_actuator_equals(outputs, expected, "混控表");
        }
    }

    #[test]
    fn pseudo_inverse_round_trip() {
        let mut allocation = ControlAllocationPseudoInverse::new();
        allocation.set_effectiveness_matrix(
            quad_x_effectiveness(),
            ActuatorVector::zeros(),
            ActuatorVector::zeros(),
            true,
        );

        let control = ControlVector::new(0.1, -0.2, 0.05, -0.3);
        allocation.set_control_setpoint(control);
        allocation.allocate();

        let allocated = allocation.allocated_control();

        for axis in 0..NUM_AXES {
            assert!(
                (allocated[axis] - control[axis]).abs() <= 1e-5,
                "轴 {axis}: 分配 {} 期望 {}",
                allocated[axis],
                control[axis]
            );
        }
    }

    #[test]
    fn pseudo_inverse_matches_analytic_inverse() {
        let effectiveness = quad_x_effectiveness();
        let mix = geninv(&effectiveness);
        let analytic = effectiveness.try_inverse().expect("四旋翼效能矩阵应可逆");

        for i in 0..NUM_ACTUATORS {
            for j in 0..NUM_AXES {
                assert!(
                    (mix[(i, j)] - analytic[(i, j)]).abs() <= 1e-5,
                    "mix[{i}][{j}]: 伪逆 {} 解析逆 {}",
                    mix[(i, j)],
                    analytic[(i, j)]
                );
            }
        }

        let identity = effectiveness * mix;

        for i in 0..NUM_AXES {
            for j in 0..NUM_AXES {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (identity[(i, j)] - expected).abs() <= 1e-5,
                    "B * pinv[{i}][{j}] = {}",
                    identity[(i, j)]
                );
            }
        }
    }

    #[test]
    fn pseudo_inverse_all_zero_case() {
        let mut allocation = ControlAllocationPseudoInverse::new();
        allocation.set_effectiveness_matrix(
            EffectivenessMatrix::zeros(),
            ActuatorVector::zeros(),
            ActuatorVector::zeros(),
            false,
        );
        allocation.set_control_setpoint(ControlVector::zeros());
        allocation.allocate();
        let clipped = allocation.clip_actuator_setpoint(allocation.actuator_setpoint());
        allocation.set_actuator_setpoint(clipped);

        for i in 0..NUM_ACTUATORS {
            assert!(
                (allocation.actuator_setpoint()[i]).abs() <= 1e-6,
                "执行器 {i}"
            );
        }

        for axis in 0..NUM_AXES {
            assert!(
                allocation.allocated_control()[axis].abs() <= 1e-6,
                "轴 {axis}"
            );
        }
    }

    #[test]
    fn drop_dependent_axes_keeps_full_rank_x_quad() {
        let mut effectiveness = quad_x_effectiveness();
        let original = effectiveness;

        assert_eq!(
            ControlAllocationPseudoInverse::drop_dependent_axes(&mut effectiveness),
            0
        );
        assert_eq!(effectiveness, original);
    }

    #[test]
    fn drop_dependent_axes_follows_priority() {
        let mut effectiveness = quad_x_effectiveness();

        for axis in 0..NUM_AXES {
            effectiveness[(axis, 0)] = 0.0;
            effectiveness[(axis, 1)] = 0.0;
        }

        let dropped = ControlAllocationPseudoInverse::drop_dependent_axes(&mut effectiveness);

        assert_eq!(dropped & (1_u8 << AXIS_ROLL), 1_u8 << AXIS_ROLL);
        assert_eq!(dropped & (1_u8 << AXIS_YAW), 1_u8 << AXIS_YAW);
        assert_eq!(dropped & (1_u8 << AXIS_PITCH), 0);
        assert_eq!(dropped & (1_u8 << AXIS_THRUST_Z), 0);

        for i in 0..NUM_ACTUATORS {
            assert!(effectiveness[(AXIS_ROLL, i)].abs() <= 1e-6);
            assert!(effectiveness[(AXIS_YAW, i)].abs() <= 1e-6);
        }
    }

    #[test]
    fn clip_actuator_setpoint_respects_min_max() {
        let mut allocation = ControlAllocation::new();
        allocation.set_actuator_min(ActuatorVector::repeat(0.1));
        allocation.set_actuator_max(ActuatorVector::repeat(0.9));

        let clipped = allocation.clip_actuator_setpoint(ActuatorVector::new(-1.0, 0.05, 0.95, 0.5));
        assert_eq!(clipped, ActuatorVector::new(0.1, 0.1, 0.9, 0.5));

        let mut allocation = ControlAllocation::new();
        allocation.set_effectiveness_matrix(
            EffectivenessMatrix::zeros(),
            ActuatorVector::repeat(0.3),
            ActuatorVector::zeros(),
            false,
        );
        allocation.set_actuator_max(ActuatorVector::repeat(0.9));
        allocation.set_actuator_min(ActuatorVector::new(0.1, 0.95, 0.1, 0.1));

        let clipped = allocation.clip_actuator_setpoint(ActuatorVector::new(0.2, 0.5, 0.05, 0.95));
        assert_eq!(clipped, ActuatorVector::new(0.2, 0.3, 0.1, 0.9));
    }

    #[test]
    fn normalize_actuator_setpoint_range() {
        let mut allocation = ControlAllocation::new();
        allocation.set_actuator_min(ActuatorVector::repeat(0.2));
        allocation.set_actuator_max(ActuatorVector::repeat(0.8));

        let normalized =
            allocation.normalize_actuator_setpoint(ActuatorVector::new(0.2, 0.5, 0.8, 0.0));

        assert!((normalized[0] - 0.0).abs() <= 1e-6);
        assert!((normalized[1] - 0.5).abs() <= 1e-6);
        assert!((normalized[2] - 1.0).abs() <= 1e-6);
        assert!((normalized[3] - (-1.0 / 3.0)).abs() <= 1e-6);
    }

    #[test]
    fn trim_offset_returns_trim_at_zero_control() {
        let mut allocation = ControlAllocationPseudoInverse::new();
        allocation.set_effectiveness_matrix(
            quad_x_effectiveness(),
            ActuatorVector::repeat(0.5),
            ActuatorVector::zeros(),
            true,
        );
        allocation.set_control_setpoint(ControlVector::zeros());
        allocation.allocate();

        for i in 0..NUM_ACTUATORS {
            assert!(
                (allocation.actuator_setpoint()[i] - 0.5).abs() <= 1e-6,
                "执行器 {i} 应等于配平值"
            );
        }
    }

    #[test]
    fn linearization_point_round_trip() {
        let mut allocation = ControlAllocationPseudoInverse::new();
        let linearization_point = ActuatorVector::repeat(0.5);
        allocation.set_effectiveness_matrix(
            quad_x_effectiveness(),
            ActuatorVector::zeros(),
            linearization_point,
            true,
        );

        let control_trim = ControlVector::new(0.0, 0.0, 0.0, -2.0);
        allocation.set_control_setpoint(control_trim);
        allocation.allocate();

        for i in 0..NUM_ACTUATORS {
            assert!(
                (allocation.actuator_setpoint()[i] - 0.5).abs() <= 1e-5,
                "执行器 {i} 应回到线性化点"
            );
        }
    }

    #[test]
    fn apply_slew_rate_limit_steps_toward_target() {
        let mut allocation = ControlAllocation::new();
        allocation.set_slew_rate_limit(ActuatorVector::repeat(1.0));
        allocation.set_actuator_setpoint(ActuatorVector::zeros());

        allocation.set_actuator_setpoint(ActuatorVector::repeat(1.0));
        allocation.apply_slew_rate_limit(0.1);

        for i in 0..NUM_ACTUATORS {
            assert!(
                (allocation.actuator_setpoint()[i] - 0.1).abs() <= 1e-6,
                "执行器 {i} 单步应走 0.1"
            );
        }

        allocation.set_actuator_setpoint(ActuatorVector::repeat(f32::NAN));
        allocation.apply_slew_rate_limit(0.1);

        for i in 0..NUM_ACTUATORS {
            assert!(
                allocation.actuator_setpoint()[i].is_nan(),
                "执行器 {i} 应保持停桨"
            );
        }
    }
}
