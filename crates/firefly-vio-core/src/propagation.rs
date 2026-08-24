//! IMU 状态传播（对照 `OpenVINS` `ov_msckf/state/Propagator.cpp` / `Propagator.h`）。
//!
//! 传播工作分三层：
//! - **测量选择**：`select_imu_readings` / `interpolate_data` 从缓存中裁剪/插值出积分区间。
//! - **均值传播**：`predict_mean_discrete` / `predict_mean_rk4` / `predict_mean_analytic`。
//! - **协方差传播**：`compute_f_and_g_analytic` / `compute_f_and_g_discrete` 产出状态转移阵
//!   `F` 与噪声雅可比 `G`，再由 `predict_and_compute` 合成离散噪声协方差 `Qd = G·Qc·Gᵀ`。
//!
//! 本模块不依赖 `State` 类型：一律以值/引用参数传入当前 `q/p/v`，输出传播后的
//! 新状态与 `F/Qd`；与 `State` 的对接（对应 C++ `propagate_and_clone` /
//! `fast_state_propagate` 的编排）由 `firefly-vio` 负责。`Propagator` 只持有
//! 掩码数据缓冲与 feed/clean/select 逻辑。
//!
//! 传播在 IMU 系进行：加速度校正后需左乘 `R_GtoIᵀ` 旋转到全局，重力 `gravity`（全局）单独扣除，
//! 与 JPL 四元数惯例（`firefly-vio-types::quat_ops`）一致。

use std::sync::Mutex;

use nalgebra::{DMatrix, Matrix3, Matrix4, SMatrix, Vector3, Vector4};

use crate::imu_model::{CorrectedImu, ImuCalibration, ImuModel};
use crate::noise::ImuNoise;
use crate::sensor::ImuData;
use firefly_vio_types::quat_ops::{
    exp_so3, jr_so3, log_so3, omega, quat_2_rot, quat_multiply, quatnorm, rot_2_quat, skew_x,
};

/// 均值传播的积分方法（对照 `StateOptions::IntegrationMethod`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntegrationMethod {
    /// 零阶四元数 + 常加速度离散。
    Discrete,
    /// 四阶 Runge-Kutta。
    Rk4,
    /// 基于 `Xi_sum` 的闭式积分（ACI²）。
    Analytical,
}

/// 传播运行期选项（对照 `StateOptions` 中与传播相关的字段）。
#[derive(Debug, Clone, Copy)]
pub struct PropagationOptions {
    /// 均值/协方差的积分方法。
    pub integration_method: IntegrationMethod,
    /// IMU 内参布局模型（`Kalibr` → 校正陀螺旋转，`Rpng` → 校正加速度计旋转）。
    pub imu_model: ImuModel,
    /// 是否估计 IMU 内参（`Dw`/`Da`/旋转，可能还有 `Tg`）。
    pub do_calib_imu_intrinsics: bool,
    /// 是否估计重力敏感阵 `Tg`。
    pub do_calib_imu_g_sensitivity: bool,
    /// 是否使用首估计（FEJ）做线性化。本模块仅记录标志；线性化点由调用方
    /// 经 [`LinearizationPoint`] 传入。
    pub do_fej: bool,
}

impl Default for PropagationOptions {
    fn default() -> Self {
        Self {
            integration_method: IntegrationMethod::Rk4, // 对照 StateOptions 默认 RK4
            imu_model: ImuModel::Kalibr,
            do_calib_imu_intrinsics: true,
            do_calib_imu_g_sensitivity: true,
            do_fej: true,
        }
    }
}

impl PropagationOptions {
    /// IMU 内参的误差态维度（对照 `State::imu_intrinsic_size`）。
    ///
    /// 内参含 `Dw`(6) + `Da`(6) + 旋转(3)，开启重力敏感再 `+9`(Tg)。
    #[must_use]
    pub fn imu_intrinsic_size(&self) -> usize {
        if self.do_calib_imu_intrinsics {
            let mut sz = 15; // 6(Dw) + 6(Da) + 3(旋转)
            if self.do_calib_imu_g_sensitivity {
                sz += 9; // Tg
            }
            sz
        } else {
            0
        }
    }
}

/// 当前 IMU 均值状态（`q_GtoI, p_IinG, v_IinG`），传播的输入。
#[derive(Debug, Clone, Copy)]
pub struct MeanState {
    /// JPL 四元数 `q_GtoI`（标量在最后）。
    pub q: Vector4<f64>,
    /// 位置 `p_IinG`（全局）。
    pub p: Vector3<f64>,
    /// 速度 `v_IinG`（全局）。
    pub v: Vector3<f64>,
}

impl MeanState {
    /// 由四元数/位置/速度构造。
    #[must_use]
    pub fn new(q: Vector4<f64>, p: Vector3<f64>, v: Vector3<f64>) -> Self {
        Self { q, p, v }
    }

    /// 当前姿态旋转矩阵 `R_GtoI`。
    #[must_use]
    pub fn rotation(&self) -> Matrix3<f64> {
        quat_2_rot(&self.q)
    }
}

/// `F/G` 的线性化点（`R_GtoI`、`v_IinG`、`p_IinG`）。
///
/// 对应 C++ `compute_F_and_G_*` 中 `R_k/v_k/p_k`：FEJ 关闭时为当前均值；
/// FEJ 开启时由 `firefly-vio` 传入首估计值（first estimate）。
#[derive(Debug, Clone, Copy)]
pub struct LinearizationPoint {
    /// 姿态旋转 `R_k`。
    pub r: Matrix3<f64>,
    /// 速度 `v_k`。
    pub v: Vector3<f64>,
    /// 位置 `p_k`。
    pub p: Vector3<f64>,
}

impl LinearizationPoint {
    /// 以当前均值为线性化点：`R = quat_2_rot(state.q)`，`v/p` 取当前值。
    #[must_use]
    pub fn from_state(state: &MeanState) -> Self {
        Self {
            r: state.rotation(),
            v: state.v,
            p: state.p,
        }
    }

    /// 由旋转/速度/位置直接构造（FEJ 时传首估计值）。
    #[must_use]
    pub fn new(r: Matrix3<f64>, v: Vector3<f64>, p: Vector3<f64>) -> Self {
        Self { r, v, p }
    }
}

/// `predict_and_compute` 的输出：传播后的均值与状态转移阵/离散噪声协方差。
#[derive(Debug, Clone)]
pub struct Propagated {
    /// 新四元数 `q_GtoI`。
    pub q: Vector4<f64>,
    /// 新速度 `v_IinG`。
    pub v: Vector3<f64>,
    /// 新位置 `p_IinG`。
    pub p: Vector3<f64>,
    /// 状态转移阵 `F`（维度 `15+intrinsic`，含标定块）。
    pub f: DMatrix<f64>,
    /// 离散噪声协方差 `Qd`（对称半正定）。
    pub qd: DMatrix<f64>,
}

/// fast 传播的初始缓存输入（对照 `fast_state_propagate` 首次调用的缓存段）。
#[derive(Debug, Clone)]
pub struct FastInit {
    /// 缓存起始时间（IMU 时钟系）。
    pub time: f64,
    /// 缓存时刻的 IMU-相机时间偏移。
    pub t_off: f64,
    /// IMU 值 `[q4, p3, v3, bg3, ba3]`（16 维，对照 `IMU::value()`）。
    pub est: [f64; 16],
    /// IMU 协方差（15×15，对照 `StateHelper::get_marginal_covariance({imu})`）。
    pub covariance: DMatrix<f64>,
}

/// fast 传播输出（对照 `fast_state_propagate` 的 `state_plus` + `covariance`）。
///
/// 注意速度/协方差均为 **IMU 局部系**（供控制环高频使用）。
#[derive(Debug, Clone)]
pub struct FastState {
    /// 姿态 `q_GtoI`。
    pub q: Vector4<f64>,
    /// 位置 `p_IinG`。
    pub p: Vector3<f64>,
    /// 局部系速度 `v_IinI`。
    pub v: Vector3<f64>,
    /// 最后校正角速度（IMU 系）。
    pub w: Vector3<f64>,
    /// 12×12 协方差（θ/p/v 局部系块 + bg 块取角速度噪声）。
    pub covariance: DMatrix<f64>,
}

/// IMU 传播器（对照 `ov_msckf::state::Propagator`）。
///
/// 持有连续噪声、重力向量、IMU 掩码缓冲与 fast-prop 缓存字段。`feed_imu` /
/// `clean_old_imu_measurements` / `invalidate_cache` 处理缓冲；纯传播数学由
/// `predict_and_compute` 及其下层静态/实例函数承担。
#[derive(Debug)]
pub struct Propagator {
    /// 连续时间噪声参数。
    noise: ImuNoise,
    /// 全局重力向量（默认 `(0, 0, 9.81)`）。
    gravity: Vector3<f64>,
    /// 历史 IMU 消息（时间戳递增）。
    imu_data: Mutex<Vec<ImuData>>,
    /// 上次传播的 IMU-相机时间偏移。
    #[allow(dead_code)]
    last_prop_time_offset: f64,
    /// 是否已设置 `last_prop_time_offset`。
    #[allow(dead_code)]
    have_last_prop_time_offset: bool,
    /// fast-prop 缓存是否有效（对应 `cache_imu_valid`）。
    cache_imu_valid: bool,
    /// fast-prop 缓存的起始时间。
    cache_state_time: f64,
    /// fast-prop 缓存的 IMU 值 `[q4, p3, v3, bg3, ba3]`。
    cache_state_est: [f64; 16],
    /// fast-prop 缓存的 IMU 协方差。
    #[allow(dead_code)]
    cache_state_covariance: DMatrix<f64>,
    /// fast-prop 缓存的时间偏移。
    cache_t_off: f64,
}

impl Default for Propagator {
    fn default() -> Self {
        Self::new(ImuNoise::default())
    }
}

impl Propagator {
    /// 用默认引力幅值 `9.81` 构造。
    #[must_use]
    pub fn new(noise: ImuNoise) -> Self {
        Self::new_with_gravity(noise, 9.81)
    }

    /// 指定重力幅值构造（对照 `Propagator(NoiseManager, double)`，`_gravity = (0,0,mag)`）。
    #[must_use]
    pub fn new_with_gravity(noise: ImuNoise, gravity_mag: f64) -> Self {
        Self {
            noise,
            gravity: Vector3::new(0.0, 0.0, gravity_mag),
            imu_data: Mutex::new(Vec::new()),
            last_prop_time_offset: 0.0,
            have_last_prop_time_offset: false,
            cache_imu_valid: false,
            cache_state_time: 0.0,
            cache_state_est: [0.0; 16],
            cache_state_covariance: DMatrix::zeros(15, 15),
            cache_t_off: 0.0,
        }
    }

    /// 存储接收到的 IMU 测量，并清理早于 `oldest_time - 0.10` 的旧数据
    /// （对应 `feed_imu`，`oldest_time` 默认 `-1` 即默认不清理）。
    ///
    /// # Panics
    ///
    /// 若掩码缓冲锁被毒化则 panic。
    pub fn feed_imu(&self, message: ImuData, oldest_time: f64) {
        self.imu_data
            .lock()
            .expect("imu_data mutex poisoned")
            .push(message);
        self.clean_old_imu_measurements(oldest_time - 0.10);
    }

    /// 移除时间戳早于 `oldest_time` 的测量（对应 `clean_old_imu_measurements`）。
    ///
    /// # Panics
    ///
    /// 若掩码缓冲锁被毒化则 panic。
    pub fn clean_old_imu_measurements(&self, oldest_time: f64) {
        if oldest_time < 0.0 {
            return;
        }
        self.imu_data
            .lock()
            .expect("imu_data mutex poisoned")
            .retain(|d| d.timestamp >= oldest_time);
    }

    /// 使 fast-prop 缓存失效（对应 `invalidate_cache`）。
    pub fn invalidate_cache(&mut self) {
        self.cache_imu_valid = false;
    }

    /// 快照当前缓存中的 IMU 测量（供传播编排取用）。
    ///
    /// # Panics
    ///
    /// 若掩码缓冲锁被毒化则 panic。
    #[must_use]
    pub fn imu_data_snapshot(&self) -> Vec<ImuData> {
        self.imu_data
            .lock()
            .expect("imu_data mutex poisoned")
            .clone()
    }

    /// 当前缓存 IMU 测量数量。
    ///
    /// # Panics
    ///
    /// 若掩码缓冲锁被毒化则 panic。
    #[must_use]
    pub fn imu_data_len(&self) -> usize {
        self.imu_data.lock().expect("imu_data mutex poisoned").len()
    }

    /// 重力向量（只读）。
    #[must_use]
    pub fn gravity(&self) -> Vector3<f64> {
        self.gravity
    }

    /// 在 `[time0, time1]` 内由 `data_minus → data_plus` 做一次传播，得到新均值与 `F/Qd`。
    ///
    /// 对应 `Propagator::predict_and_compute`，完整移植自 `Propagator.cpp`：
    /// 1. 用 `CorrectedImu::correct` 校正两端加速度/角速度；
    /// 2. 按 `integration_method` 计算均值（RK4/ANALYTICAL 先算 `Xi_sum`）；
    /// 3. 计算 `F`/`G`（解析或离散）；
    /// 4. 连续噪声 `Qc`（`sigma_*_2 / dt`）经 `Qd = G·Qc·Gᵀ` 离散化。
    ///
    /// `input` 为传播起点（均值传播与姿态积分用其当前 `q/p/v`）；
    /// `linearization` 为 `F/G` 的线性化点（`R_k/v_k/p_k`）。FEJ 关闭时二者取同一值
    /// （`LinearizationPoint::from_state(input)`）；FEJ 开启时由 `firefly-vio`
    /// 传入首估计值（first estimate）。
    ///
    /// # Panics
    ///
    /// 均值为 0 的 `dt` 会使离散噪声协方差为无穷（同 C++，不做防御，调用方应先经
    /// `select_imu_readings` 剔除零 `dt` 相邻测量）。
    #[must_use]
    pub fn predict_and_compute(
        &self,
        opts: &PropagationOptions,
        calib: &ImuCalibration,
        data_minus: &ImuData,
        data_plus: &ImuData,
        input: &MeanState,
        linearization: &LinearizationPoint,
    ) -> Propagated {
        let dt = data_plus.timestamp - data_minus.timestamp;

        // 校正两端加速度/角速度，并分离出 H 矩阵需要的"未校正"均值。
        let minus = CorrectedImu::correct(data_minus, calib);
        let plus = CorrectedImu::correct(data_plus, calib);
        let a_uncorrected = 0.5 * ((data_minus.am - calib.bias_a) + (data_plus.am - calib.bias_a));
        let w_unc1 = data_minus.wm - calib.bias_g - calib.tg * minus.am;
        let w_unc2 = data_plus.wm - calib.bias_g - calib.tg * plus.am;
        let w_uncorrected = 0.5 * (w_unc1 + w_unc2);
        let a_hat_avg = 0.5 * (minus.am + plus.am);
        let w_hat_avg = 0.5 * (minus.wm + plus.wm);

        // RK4 / ANALYTICAL 用到 Xi_sum；DISCRETE 分支忽略其值（照 C++ 仅在 RK4/ANALYTICAL 计算，
        // 为避免 Option+unwrap 的 panic 风险，这里始终计算，离散模式下的值为无用中间量）。
        let xi_sum = Propagator::compute_xi_sum(dt, &w_hat_avg, &a_hat_avg);

        // 均值传播。
        let (new_q, new_v, new_p) = match opts.integration_method {
            IntegrationMethod::Analytical => {
                self.predict_mean_analytic(dt, &xi_sum, &a_hat_avg, input)
            }
            IntegrationMethod::Rk4 => {
                self.predict_mean_rk4(dt, &minus.wm, &minus.am, &plus.wm, &plus.am, input)
            }
            IntegrationMethod::Discrete => {
                self.predict_mean_discrete(dt, &w_hat_avg, &a_hat_avg, input)
            }
        };

        // F / G，维度 (15 + intrinsic)。线性化点取 `linearization`（FEJ 或当前均值）。
        let dim = 15 + opts.imu_intrinsic_size();
        let mut f = DMatrix::<f64>::zeros(dim, dim);
        let mut g = DMatrix::<f64>::zeros(dim, 12);
        if matches!(
            opts.integration_method,
            IntegrationMethod::Rk4 | IntegrationMethod::Analytical
        ) {
            self.compute_f_and_g_analytic(
                *opts,
                calib,
                dt,
                &w_uncorrected,
                &a_uncorrected,
                &new_q,
                &new_v,
                &new_p,
                &xi_sum,
                &linearization.r,
                &linearization.v,
                &linearization.p,
                &mut f,
                &mut g,
            );
        } else {
            self.compute_f_and_g_discrete(
                *opts,
                calib,
                dt,
                &w_uncorrected,
                &a_uncorrected,
                &new_q,
                &new_v,
                &new_p,
                &linearization.r,
                &linearization.v,
                &linearization.p,
                &mut f,
                &mut g,
            );
        }

        // 离散噪声协方差：Qd = G * Qc * G^T。
        // 注：predict_and_compute 对四项噪声都用 sigma_*_2 / dt（含随机游走），
        // 与 fast_state_propagate 中 wb/ab 用 * dt 不同，这里忠实跟随前者。
        let mut qc = DMatrix::<f64>::zeros(12, 12);
        qc.view_mut((0, 0), (3, 3))
            .copy_from(&(self.noise.sigma_w_2 / dt * Matrix3::identity()));
        qc.view_mut((3, 3), (3, 3))
            .copy_from(&(self.noise.sigma_a_2 / dt * Matrix3::identity()));
        qc.view_mut((6, 6), (3, 3))
            .copy_from(&(self.noise.sigma_wb_2 / dt * Matrix3::identity()));
        qc.view_mut((9, 9), (3, 3))
            .copy_from(&(self.noise.sigma_ab_2 / dt * Matrix3::identity()));
        let mut qd = &g * &qc * g.transpose();
        qd = 0.5 * (&qd + qd.transpose());

        Propagated {
            q: new_q,
            v: new_v,
            p: new_p,
            f,
            qd,
        }
    }

    /// 线性插值两个 IMU 测量（对照 `Propagator::interpolate_data`）。
    ///
    /// # Panics
    ///
    /// 若两测量时间戳相同（分母为零）则产生 `NaN`（同 C++，不做防御）。
    #[must_use]
    pub fn interpolate_data(imu_1: &ImuData, imu_2: &ImuData, timestamp: f64) -> ImuData {
        let lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
        ImuData {
            timestamp,
            am: (1.0 - lambda) * imu_1.am + lambda * imu_2.am,
            wm: (1.0 - lambda) * imu_1.wm + lambda * imu_2.wm,
        }
    }

    /// 从 `[time0, time1]` 间挑选 IMU 测量（对应 `Propagator::select_imu_readings`）。
    ///
    /// 首尾测量在边界处被插值"切断"。忠实复刻 C++ 的 CASE1/2/3 裁剪逻辑，
    /// 含「末测量不足则外推到 `time1`」与零 `dt` 相邻测量移除；测量不足返回 `None`
    /// （对应 C++ 返回空向量）。
    #[must_use]
    pub fn select_imu_readings(
        imu_data: &[ImuData],
        time0: f64,
        time1: f64,
        _warn: bool,
    ) -> Option<Vec<ImuData>> {
        // 警告日志：firefly-vio-core 当前无 log 依赖，`warn` 仅保留 API 语义
        //（波形 2 接观测层后接入）。逻辑对照 C++ 的 PRINT_WARNING 分叉（不改变返回值）。
        if imu_data.is_empty() {
            return None;
        }

        let mut prop_data: Vec<ImuData> = Vec::new();
        let mut i = 0usize;
        while i + 1 < imu_data.len() {
            // CASE 1: 在 time0 处切断当前测量。
            if imu_data[i + 1].timestamp > time0 && imu_data[i].timestamp < time0 {
                prop_data.push(Self::interpolate_data(
                    &imu_data[i],
                    &imu_data[i + 1],
                    time0,
                ));
                i += 1;
                continue;
            }
            // CASE 2: 整段测量落在区间内。
            if imu_data[i].timestamp >= time0 && imu_data[i + 1].timestamp <= time1 {
                prop_data.push(imu_data[i]);
                i += 1;
                continue;
            }
            // CASE 3: 区间末端。
            if imu_data[i + 1].timestamp > time1 {
                if imu_data[i].timestamp > time1 && i == 0 {
                    // 最早数据已越过 time1，无法前向传播。
                    break;
                } else if imu_data[i].timestamp > time1 {
                    prop_data.push(Self::interpolate_data(
                        &imu_data[i - 1],
                        &imu_data[i],
                        time1,
                    ));
                } else {
                    prop_data.push(imu_data[i]);
                }
                // 若最后一个测量不恰好落在 time1，则补插值端点。
                if prop_data
                    .last()
                    .is_none_or(|d| (d.timestamp - time1).abs() > 1e-12)
                {
                    prop_data.push(Self::interpolate_data(
                        &imu_data[i],
                        &imu_data[i + 1],
                        time1,
                    ));
                }
                break;
            }
            i += 1;
        }

        if prop_data.is_empty() {
            return None;
        }

        // 末测量不足则外推到 time1。
        if prop_data
            .last()
            .is_none_or(|d| (d.timestamp - time1).abs() > 1e-12)
        {
            prop_data.push(Self::interpolate_data(
                &imu_data[imu_data.len() - 2],
                &imu_data[imu_data.len() - 1],
                time1,
            ));
        }

        // 移除零 dt 相邻测量（否则噪声协方差为无穷）。
        let mut i = 0usize;
        while i + 1 < prop_data.len() {
            if (prop_data[i + 1].timestamp - prop_data[i].timestamp).abs() < 1e-12 {
                prop_data.remove(i);
            } else {
                i += 1;
            }
        }

        if prop_data.len() < 2 {
            return None;
        }

        Some(prop_data)
    }

    /// fast-prop 缓存是否有效（对照 `cache_imu_valid`）。
    #[must_use]
    pub fn cache_valid(&self) -> bool {
        self.cache_imu_valid
    }

    /// 高频 fast 传播（对照 `Propagator::fast_state_propagate`）。
    ///
    /// 用离散零阶 F/G 与缓存的状态/协方差逐段传播（**不更新 `State`**），
    /// 输出局部系速度与角速度（控制环高频位姿）。首次调用（或缓存失效）
    /// 时用 `initial` 重建缓存；`initial` 缺失且缓存无效返回 `None`。
    ///
    /// 注意与 `predict_and_compute` 的噪声离散化差异：此处随机游走
    /// `sigma_wb_2`/`sigma_ab_2` 乘 `dt`（Trawny 报告式 (129)/(130)），
    /// 而 `predict_and_compute` 全部除 `dt`——两者分别对照 C++ 的
    /// `fast_state_propagate` 与 `predict_and_compute`。
    ///
    /// # Panics
    /// `imu_data` 互斥锁中毒时 panic（本模块内不跨线程持有锁）。
    // 与 C++ 1:1 移植的长流程函数，拆分会破坏对照可审计性。
    #[allow(clippy::too_many_lines)]
    #[must_use]
    pub fn fast_state_propagate(
        &mut self,
        initial: Option<&FastInit>,
        timestamp: f64,
        calib: &ImuCalibration,
    ) -> Option<FastState> {
        // 缓存无效时用初始值重建（对照 C++ 的缓存段）
        if !self.cache_imu_valid {
            let init = initial?;
            self.cache_state_time = init.time;
            self.cache_state_est = init.est;
            self.cache_state_covariance = init.covariance.clone();
            self.cache_t_off = init.t_off;
            self.cache_imu_valid = true;
        }

        // 选取测量（warn=false，对照 C++）
        let time0 = self.cache_state_time + self.cache_t_off;
        let time1 = timestamp + self.cache_t_off;
        let prop_data = {
            let imu = self.imu_data.lock().expect("imu_data 锁");
            Self::select_imu_readings(&imu, time0, time1, false)?
        };
        if prop_data.len() < 2 {
            return None;
        }

        let bias_g = Vector3::new(
            self.cache_state_est[10],
            self.cache_state_est[11],
            self.cache_state_est[12],
        );
        let bias_a = Vector3::new(
            self.cache_state_est[13],
            self.cache_state_est[14],
            self.cache_state_est[15],
        );

        // 逐段离散传播（对照 C++ 的循环：零阶四元数 + 常加速度）
        for i in 0..prop_data.len() - 1 {
            let data_minus = &prop_data[i];
            let data_plus = &prop_data[i + 1];
            let dt = data_plus.timestamp - data_minus.timestamp;

            // 校正测量（对照 C++ 的 a_hat1/2、w_hat1/2）
            let a_hat1 = calib.r_acc_to_imu * calib.da * (data_minus.am - bias_a);
            let a_hat2 = calib.r_acc_to_imu * calib.da * (data_plus.am - bias_a);
            let a_hat = 0.5 * (a_hat1 + a_hat2);
            let w_hat1 =
                calib.r_gyro_to_imu * calib.dw * (data_minus.wm - bias_g - calib.tg * a_hat1);
            let w_hat2 =
                calib.r_gyro_to_imu * calib.dw * (data_plus.wm - bias_g - calib.tg * a_hat2);
            let w_hat = 0.5 * (w_hat1 + w_hat2);

            // 当前缓存状态
            let r_gtoi = quat_2_rot(&Vector4::new(
                self.cache_state_est[0],
                self.cache_state_est[1],
                self.cache_state_est[2],
                self.cache_state_est[3],
            ));
            let p_iin_g = Vector3::new(
                self.cache_state_est[4],
                self.cache_state_est[5],
                self.cache_state_est[6],
            );
            let v_iin_g = Vector3::new(
                self.cache_state_est[7],
                self.cache_state_est[8],
                self.cache_state_est[9],
            );

            // 离散 F（15×15，对照 C++ 的块赋值）
            let exp_w = exp_so3(&(-w_hat * dt));
            let jr_w = jr_so3(&(-w_hat * dt));
            let mut f = DMatrix::<f64>::zeros(15, 15);
            f.view_mut((0, 0), (3, 3)).copy_from(&exp_w);
            f.view_mut((0, 9), (3, 3)).copy_from(&(-exp_w * jr_w * dt));
            f.view_mut((9, 9), (3, 3)).fill_diagonal(1.0);
            f.view_mut((6, 0), (3, 3))
                .copy_from(&(-r_gtoi.transpose() * skew_x(&(a_hat * dt))));
            f.view_mut((6, 6), (3, 3)).fill_diagonal(1.0);
            f.view_mut((6, 12), (3, 3))
                .copy_from(&(-r_gtoi.transpose() * dt));
            f.view_mut((12, 12), (3, 3)).fill_diagonal(1.0);
            f.view_mut((3, 0), (3, 3))
                .copy_from(&(-0.5 * r_gtoi.transpose() * skew_x(&(a_hat * dt * dt))));
            f.view_mut((3, 6), (3, 3))
                .copy_from(&(Matrix3::identity() * dt));
            f.view_mut((3, 12), (3, 3))
                .copy_from(&(-0.5 * r_gtoi.transpose() * dt * dt));
            f.view_mut((3, 3), (3, 3)).fill_diagonal(1.0);

            // 噪声雅可比 G（15×12）
            let mut g = DMatrix::<f64>::zeros(15, 12);
            g.view_mut((0, 0), (3, 3)).copy_from(&(-exp_w * jr_w * dt));
            g.view_mut((6, 3), (3, 3))
                .copy_from(&(-r_gtoi.transpose() * dt));
            g.view_mut((3, 3), (3, 3))
                .copy_from(&(-0.5 * r_gtoi.transpose() * dt * dt));
            g.view_mut((9, 6), (3, 3)).fill_diagonal(1.0);
            g.view_mut((12, 9), (3, 3)).fill_diagonal(1.0);

            // Qd = G·Qc·Gᵀ（注意：随机游走用 *dt，对照 C++）
            let mut qc = DMatrix::<f64>::zeros(12, 12);
            qc.view_mut((0, 0), (3, 3))
                .copy_from(&(self.noise.sigma_w_2 / dt * Matrix3::identity()));
            qc.view_mut((3, 3), (3, 3))
                .copy_from(&(self.noise.sigma_a_2 / dt * Matrix3::identity()));
            qc.view_mut((6, 6), (3, 3))
                .copy_from(&(self.noise.sigma_wb_2 * dt * Matrix3::identity()));
            qc.view_mut((9, 9), (3, 3))
                .copy_from(&(self.noise.sigma_ab_2 * dt * Matrix3::identity()));
            let qd = &g * qc * g.transpose();
            let qd = 0.5 * (&qd + qd.transpose());

            // 协方差与均值传播
            self.cache_state_covariance = &f * &self.cache_state_covariance * f.transpose() + qd;
            let q_new = rot_2_quat(&(exp_w * r_gtoi));
            let p_new = p_iin_g + v_iin_g * dt + 0.5 * r_gtoi.transpose() * a_hat * dt * dt
                - 0.5 * self.gravity * dt * dt;
            let v_new = v_iin_g + r_gtoi.transpose() * a_hat * dt - self.gravity * dt;
            self.cache_state_est[0..4].copy_from_slice(q_new.as_slice());
            self.cache_state_est[4..7].copy_from_slice(p_new.as_slice());
            self.cache_state_est[7..10].copy_from_slice(v_new.as_slice());
        }

        // 推进缓存时间（IMU 时钟系，t_off 清零，对照 C++）
        self.cache_state_time = time1;
        self.cache_t_off = 0.0;

        // 输出（对照 C++：v 转局部系、最后角速度、12×12 协方差）
        let q = Vector4::new(
            self.cache_state_est[0],
            self.cache_state_est[1],
            self.cache_state_est[2],
            self.cache_state_est[3],
        );
        let p = Vector3::new(
            self.cache_state_est[4],
            self.cache_state_est[5],
            self.cache_state_est[6],
        );
        let v_g = Vector3::new(
            self.cache_state_est[7],
            self.cache_state_est[8],
            self.cache_state_est[9],
        );
        let last = prop_data.last().expect("prop_data 非空");
        let last_a = calib.r_acc_to_imu * calib.da * (last.am - bias_a);
        let last_w = calib.r_gyro_to_imu * calib.dw * (last.wm - bias_g - calib.tg * last_a);

        let r = quat_2_rot(&q);
        let mut phi = DMatrix::<f64>::identity(15, 15);
        phi.view_mut((6, 6), (3, 3)).copy_from(&r);
        let cov_tmp = &phi * &self.cache_state_covariance * phi.transpose();
        let mut covariance = DMatrix::<f64>::zeros(12, 12);
        covariance
            .view_mut((0, 0), (9, 9))
            .copy_from(&cov_tmp.view((0, 0), (9, 9)));
        let dt_last = last.timestamp - prop_data[prop_data.len() - 2].timestamp;
        covariance
            .view_mut((9, 9), (3, 3))
            .copy_from(&(self.noise.sigma_w_2 / dt_last * Matrix3::identity()));

        Some(FastState {
            q,
            p,
            v: r * v_g,
            w: last_w,
            covariance,
        })
    }

    /// 离散均值传播（零阶四元数 + 常加速度，对应 `predict_mean_discrete`）。
    ///
    /// 奇点：`w_norm ≤ 1e-12` 时用一阶近似 `I4 + 0.5·dt·Ω(ω)` 替代闭式 `cos/sin`
    /// 展开，避免除零（照抄 C++ 分支）。
    #[must_use]
    fn predict_mean_discrete(
        &self,
        dt: f64,
        w_hat: &Vector3<f64>,
        a_hat: &Vector3<f64>,
        input: &MeanState,
    ) -> (Vector4<f64>, Vector3<f64>, Vector3<f64>) {
        let w_norm = w_hat.norm();
        let i4 = Matrix4::<f64>::identity();
        let big_o = if w_norm > 1e-12 {
            (0.5 * w_norm * dt).cos() * i4 + 1.0 / w_norm * (0.5 * w_norm * dt).sin() * omega(w_hat)
        } else {
            i4 + 0.5 * dt * omega(w_hat)
        };
        let new_q = quatnorm(big_o * input.q);
        let r_gtoi = input.rotation();
        let new_v = input.v + r_gtoi.transpose() * a_hat * dt - self.gravity * dt;
        let new_p = input.p + input.v * dt + 0.5 * r_gtoi.transpose() * a_hat * dt * dt
            - 0.5 * self.gravity * dt * dt;
        (new_q, new_v, new_p)
    }

    /// RK4 均值传播（对应 `predict_mean_rk4`）。
    #[must_use]
    fn predict_mean_rk4(
        &self,
        dt: f64,
        w_hat1: &Vector3<f64>,
        a_hat1: &Vector3<f64>,
        w_hat2: &Vector3<f64>,
        a_hat2: &Vector3<f64>,
        input: &MeanState,
    ) -> (Vector4<f64>, Vector3<f64>, Vector3<f64>) {
        let mut w_hat = *w_hat1;
        let mut a_hat = *a_hat1;
        let w_alpha = (w_hat2 - w_hat1) / dt;
        let a_jerk = (a_hat2 - a_hat1) / dt;

        let q_0 = input.q;
        let p_0 = input.p;
        let v_0 = input.v;

        // k1
        let dq_0 = Vector4::new(0.0, 0.0, 0.0, 1.0);
        let q0_dot = 0.5 * omega(&w_hat) * dq_0;
        let p0_dot = v_0;
        let r_gto0 = quat_2_rot(&quat_multiply(&dq_0, &q_0));
        let v0_dot = r_gto0.transpose() * a_hat - self.gravity;
        let k1_q = q0_dot * dt;
        let k1_p = p0_dot * dt;
        let k1_v = v0_dot * dt;

        // k2
        w_hat += 0.5 * w_alpha * dt;
        a_hat += 0.5 * a_jerk * dt;
        let dq_1 = quatnorm(dq_0 + 0.5 * k1_q);
        let v_1 = v_0 + 0.5 * k1_v;
        let q1_dot = 0.5 * omega(&w_hat) * dq_1;
        let p1_dot = v_1;
        let r_gto1 = quat_2_rot(&quat_multiply(&dq_1, &q_0));
        let v1_dot = r_gto1.transpose() * a_hat - self.gravity;
        let k2_q = q1_dot * dt;
        let k2_p = p1_dot * dt;
        let k2_v = v1_dot * dt;

        // k3
        let dq_2 = quatnorm(dq_0 + 0.5 * k2_q);
        let v_2 = v_0 + 0.5 * k2_v;
        let q2_dot = 0.5 * omega(&w_hat) * dq_2;
        let p2_dot = v_2;
        let r_gto2 = quat_2_rot(&quat_multiply(&dq_2, &q_0));
        let v2_dot = r_gto2.transpose() * a_hat - self.gravity;
        let k3_q = q2_dot * dt;
        let k3_p = p2_dot * dt;
        let k3_v = v2_dot * dt;

        // k4
        w_hat += 0.5 * w_alpha * dt;
        a_hat += 0.5 * a_jerk * dt;
        let dq_3 = quatnorm(dq_0 + k3_q);
        let v_3 = v_0 + k3_v;
        let q3_dot = 0.5 * omega(&w_hat) * dq_3;
        let p3_dot = v_3;
        let r_gto3 = quat_2_rot(&quat_multiply(&dq_3, &q_0));
        let v3_dot = r_gto3.transpose() * a_hat - self.gravity;
        let k4_q = q3_dot * dt;
        let k4_p = p3_dot * dt;
        let k4_v = v3_dot * dt;

        // y+dt
        let dq = quatnorm(
            dq_0 + (1.0 / 6.0) * k1_q
                + (1.0 / 3.0) * k2_q
                + (1.0 / 3.0) * k3_q
                + (1.0 / 6.0) * k4_q,
        );
        let new_q = quat_multiply(&dq, &q_0);
        let new_p =
            p_0 + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p + (1.0 / 3.0) * k3_p + (1.0 / 6.0) * k4_p;
        let new_v =
            v_0 + (1.0 / 6.0) * k1_v + (1.0 / 3.0) * k2_v + (1.0 / 3.0) * k3_v + (1.0 / 6.0) * k4_v;
        (new_q, new_v, new_p)
    }

    /// 解析积分分量 `[R_k, Xi_1, Xi_2, Jr, Xi_3, Xi_4]`（对应 `compute_Xi_sum`）。
    ///
    /// 所有积分量都在 IMU 系内完成。`small_w`（`w_norm < π/360`）时退化为截断级数，
    /// 避免 `1/w_norm` 在接近零角速度处的数值爆炸（照抄 C++ `small_w` 分支）。
    #[must_use]
    fn compute_xi_sum(dt: f64, w_hat: &Vector3<f64>, a_hat: &Vector3<f64>) -> SMatrix<f64, 3, 18> {
        let w_norm = w_hat.norm();
        let d_theta = w_norm * dt;
        let k_hat = if w_norm > 1e-12 {
            w_hat / w_norm
        } else {
            Vector3::zeros()
        };

        let i3 = Matrix3::<f64>::identity();
        let dt_sq = dt * dt;
        let dt_cu = dt * dt * dt;
        let w_norm_sq = w_norm * w_norm;
        let w_norm_cu = w_norm * w_norm * w_norm;
        let cos_th = d_theta.cos();
        let sin_th = d_theta.sin();
        let d_theta_sq = d_theta * d_theta;
        let d_theta_cu = d_theta * d_theta * d_theta;
        let sk = skew_x(&k_hat);
        let sk2 = sk * sk;
        let sa = skew_x(a_hat);

        let r_ktok1 = exp_so3(&(-w_hat * dt));
        let jr_ktok1 = jr_so3(&(-w_hat * dt));

        let (xi_1, xi_2, xi_3, xi_4);
        if w_norm < std::f64::consts::PI / 360.0 {
            // small_w：截断级数近似。
            xi_1 = dt * (i3 + sin_th * sk + (1.0 - cos_th) * sk2);
            xi_2 = 0.5 * dt * xi_1;
            let inner = sin_th * (-sa * sk + sk * sa + k_hat.dot(a_hat) * sk2)
                + (1.0 - cos_th) * (sa * sk2 + sk2 * sa + k_hat.dot(a_hat) * sk);
            xi_3 = 0.5 * dt_sq * (sa + inner);
            xi_4 = 1.0 / 3.0 * dt * xi_3;
        } else {
            xi_1 = i3 * dt + (1.0 - cos_th) / w_norm * sk + (dt - sin_th / w_norm) * sk2;

            xi_2 = 0.5 * dt_sq * i3
                + (d_theta - sin_th) / w_norm_sq * sk
                + (0.5 * dt_sq - (1.0 - cos_th) / w_norm_sq) * sk2;

            xi_3 = 0.5 * dt_sq * sa
                + (sin_th - d_theta) / w_norm_sq * (sa * sk)
                + (sin_th - d_theta * cos_th) / w_norm_sq * (sk * sa)
                + (0.5 * dt_sq - (1.0 - cos_th) / w_norm_sq) * (sa * sk2)
                + (0.5 * dt_sq + (1.0 - cos_th - d_theta * sin_th) / w_norm_sq)
                    * (sk2 * sa + k_hat.dot(a_hat) * sk)
                - (3.0 * sin_th - 2.0 * d_theta - d_theta * cos_th) / w_norm_sq
                    * k_hat.dot(a_hat)
                    * sk2;

            xi_4 = 1.0 / 6.0 * dt_cu * sa
                + (2.0 * (1.0 - cos_th) - d_theta_sq) / (2.0 * w_norm_cu) * (sa * sk)
                + ((2.0 * (1.0 - cos_th) - d_theta * sin_th) / w_norm_cu) * (sk * sa)
                + ((sin_th - d_theta) / w_norm_cu + dt_cu / 6.0) * (sa * sk2)
                + ((d_theta - 2.0 * sin_th + 1.0 / 6.0 * d_theta_cu + d_theta * cos_th)
                    / w_norm_cu)
                    * (sk2 * sa + k_hat.dot(a_hat) * sk)
                + (4.0 * cos_th - 4.0 + d_theta_sq + d_theta * sin_th) / w_norm_cu
                    * k_hat.dot(a_hat)
                    * sk2;
        }

        let mut xi_sum = SMatrix::<f64, 3, 18>::zeros();
        xi_sum.view_mut((0, 0), (3, 3)).copy_from(&r_ktok1);
        xi_sum.view_mut((0, 3), (3, 3)).copy_from(&xi_1);
        xi_sum.view_mut((0, 6), (3, 3)).copy_from(&xi_2);
        xi_sum.view_mut((0, 9), (3, 3)).copy_from(&jr_ktok1);
        xi_sum.view_mut((0, 12), (3, 3)).copy_from(&xi_3);
        xi_sum.view_mut((0, 15), (3, 3)).copy_from(&xi_4);
        xi_sum
    }

    /// 解析均值传播（对应 `predict_mean_analytic`）。
    #[must_use]
    fn predict_mean_analytic(
        &self,
        dt: f64,
        xi_sum: &SMatrix<f64, 3, 18>,
        a_hat: &Vector3<f64>,
        input: &MeanState,
    ) -> (Vector4<f64>, Vector3<f64>, Vector3<f64>) {
        let r_gtok = input.rotation();
        let q_ktok1 = rot_2_quat(&xi_sum.fixed_view::<3, 3>(0, 0).into_owned());
        let xi_1 = xi_sum.fixed_view::<3, 3>(0, 3).into_owned();
        let xi_2 = xi_sum.fixed_view::<3, 3>(0, 6).into_owned();

        let new_q = quat_multiply(&q_ktok1, &input.q);
        let new_v = input.v + r_gtok.transpose() * (xi_1 * a_hat) - self.gravity * dt;
        let new_p = input.p + input.v * dt + r_gtok.transpose() * (xi_2 * a_hat)
            - 0.5 * self.gravity * dt * dt;
        (new_q, new_v, new_p)
    }

    /// 解析状态转移阵 `F` 与噪声雅可比 `G`（对应 `compute_F_and_G_analytic`）。
    ///
    /// 含完整 IMU 块（`th/p/v/bg/ba`）与可选标定块（`Dw/Da/Tg/旋转`）。`r_k/v_k/p_k`
    /// 为线性化点：FEJ 开启时传入首估计值，否则为当前均值。
    #[allow(
        clippy::too_many_arguments,
        clippy::too_many_lines,
        clippy::similar_names
    )]
    fn compute_f_and_g_analytic(
        &self,
        opts: PropagationOptions,
        calib: &ImuCalibration,
        dt: f64,
        w_uncorrected: &Vector3<f64>,
        a_uncorrected: &Vector3<f64>,
        new_q: &Vector4<f64>,
        new_v: &Vector3<f64>,
        new_p: &Vector3<f64>,
        xi_sum: &SMatrix<f64, 3, 18>,
        r_k: &Matrix3<f64>,
        v_k: &Vector3<f64>,
        p_k: &Vector3<f64>,
        f: &mut DMatrix<f64>,
        g: &mut DMatrix<f64>,
    ) {
        let layout = CalibLayout::new(opts);
        let (th_i, p_i, v_i, bg_i, ba_i) = (0usize, 3usize, 6usize, 9usize, 12usize);

        let d_r_ktok1 = quat_2_rot(new_q) * r_k.transpose();

        // nx 3x3 标定阵均为 Copy，取自有符号复制以便后续全自有运算。
        let dw = calib.dw;
        let da = calib.da;
        let tg = calib.tg;
        let r_acc = calib.r_acc_to_imu;
        let r_gyro = calib.r_gyro_to_imu;
        let a_k = r_acc * da * a_uncorrected;
        let w_k = r_gyro * dw * w_uncorrected;

        let xi_1 = xi_sum.fixed_view::<3, 3>(0, 3).into_owned();
        let xi_2 = xi_sum.fixed_view::<3, 3>(0, 6).into_owned();
        let jr = xi_sum.fixed_view::<3, 3>(0, 9).into_owned();
        let xi_3 = xi_sum.fixed_view::<3, 3>(0, 12).into_owned();
        let xi_4 = xi_sum.fixed_view::<3, 3>(0, 15).into_owned();
        let r_k_t = r_k.transpose();

        // theta 行块
        f.view_mut((th_i, th_i), (3, 3)).copy_from(&d_r_ktok1);
        f.view_mut((p_i, th_i), (3, 3)).copy_from(
            &(-skew_x(&(new_p - p_k - v_k * dt + 0.5 * self.gravity * dt * dt)) * r_k_t),
        );
        f.view_mut((v_i, th_i), (3, 3))
            .copy_from(&(-skew_x(&(new_v - v_k + self.gravity * dt)) * r_k_t));

        // p / v 对角与相互项
        f.view_mut((p_i, p_i), (3, 3))
            .copy_from(&Matrix3::identity());
        f.view_mut((p_i, v_i), (3, 3))
            .copy_from(&(Matrix3::identity() * dt));
        f.view_mut((v_i, v_i), (3, 3))
            .copy_from(&Matrix3::identity());

        // bg 行块
        f.view_mut((th_i, bg_i), (3, 3))
            .copy_from(&(-d_r_ktok1 * jr * dt * r_gyro * dw));
        f.view_mut((p_i, bg_i), (3, 3))
            .copy_from(&(r_k_t * xi_4 * r_gyro * dw));
        f.view_mut((v_i, bg_i), (3, 3))
            .copy_from(&(r_k_t * xi_3 * r_gyro * dw));
        f.view_mut((bg_i, bg_i), (3, 3))
            .copy_from(&Matrix3::identity());

        // ba 行块
        f.view_mut((th_i, ba_i), (3, 3))
            .copy_from(&(d_r_ktok1 * jr * dt * r_gyro * dw * tg * r_acc * da));
        f.view_mut((p_i, ba_i), (3, 3))
            .copy_from(&(-r_k_t * (xi_2 + xi_4 * r_gyro * dw * tg) * r_acc * da));
        f.view_mut((v_i, ba_i), (3, 3))
            .copy_from(&(-r_k_t * (xi_1 + xi_3 * r_gyro * dw * tg) * r_acc * da));
        f.view_mut((ba_i, ba_i), (3, 3))
            .copy_from(&Matrix3::identity());

        // Dw 标定块
        if let Some(dw_id) = layout.dw {
            let h_dw = compute_h_dw(opts.imu_model, w_uncorrected);
            f.view_mut((th_i, dw_id), (3, 6))
                .copy_from(&(d_r_ktok1 * jr * dt * r_gyro * h_dw));
            f.view_mut((p_i, dw_id), (3, 6))
                .copy_from(&(-r_k_t * xi_4 * r_gyro * h_dw));
            f.view_mut((v_i, dw_id), (3, 6))
                .copy_from(&(-r_k_t * xi_3 * r_gyro * h_dw));
            identity_blocks(f, dw_id, 6);
        }

        // Da 标定块
        if let Some(da_id) = layout.da {
            let h_da = compute_h_da(opts.imu_model, a_uncorrected);
            f.view_mut((th_i, da_id), (3, 6))
                .copy_from(&(-d_r_ktok1 * jr * dt * r_gyro * dw * tg * r_acc * h_da));
            f.view_mut((p_i, da_id), (3, 6))
                .copy_from(&(r_k_t * (xi_2 + xi_4 * r_gyro * dw * tg) * r_acc * h_da));
            f.view_mut((v_i, da_id), (3, 6))
                .copy_from(&(r_k_t * (xi_1 + xi_3 * r_gyro * dw * tg) * r_acc * h_da));
            identity_blocks(f, da_id, 6);
        }

        // Tg 标定块
        if let Some(tg_id) = layout.tg {
            let h_tg = compute_h_tg(&a_k);
            f.view_mut((th_i, tg_id), (3, 9))
                .copy_from(&(-d_r_ktok1 * jr * dt * r_gyro * dw * h_tg));
            f.view_mut((p_i, tg_id), (3, 9))
                .copy_from(&(r_k_t * xi_4 * r_gyro * dw * h_tg));
            f.view_mut((v_i, tg_id), (3, 9))
                .copy_from(&(r_k_t * xi_3 * r_gyro * dw * h_tg));
            identity_blocks(f, tg_id, 9);
        }

        // 旋转标定块（Kalibr → 陀螺旋转，Rpng → 加速度计旋转）
        if let Some(rot_id) = layout.rot {
            match opts.imu_model {
                ImuModel::Kalibr => {
                    f.view_mut((th_i, rot_id), (3, 3))
                        .copy_from(&(d_r_ktok1 * jr * dt * skew_x(&w_k)));
                    f.view_mut((p_i, rot_id), (3, 3))
                        .copy_from(&(-r_k_t * xi_4 * skew_x(&w_k)));
                    f.view_mut((v_i, rot_id), (3, 3))
                        .copy_from(&(-r_k_t * xi_3 * skew_x(&w_k)));
                }
                ImuModel::Rpng => {
                    f.view_mut((th_i, rot_id), (3, 3))
                        .copy_from(&(-d_r_ktok1 * jr * dt * r_gyro * dw * tg * skew_x(&a_k)));
                    f.view_mut((p_i, rot_id), (3, 3))
                        .copy_from(&(r_k_t * (xi_2 + xi_4 * r_gyro * dw * tg) * skew_x(&a_k)));
                    f.view_mut((v_i, rot_id), (3, 3))
                        .copy_from(&(r_k_t * (xi_1 + xi_3 * r_gyro * dw * tg) * skew_x(&a_k)));
                }
            }
            f.view_mut((rot_id, rot_id), (3, 3))
                .copy_from(&Matrix3::identity());
        }

        // G 噪声雅可比
        g.view_mut((th_i, 0), (3, 3))
            .copy_from(&(-d_r_ktok1 * jr * dt * r_gyro * dw));
        g.view_mut((p_i, 0), (3, 3))
            .copy_from(&(r_k_t * xi_4 * r_gyro * dw));
        g.view_mut((v_i, 0), (3, 3))
            .copy_from(&(r_k_t * xi_3 * r_gyro * dw));
        g.view_mut((th_i, 3), (3, 3))
            .copy_from(&(d_r_ktok1 * jr * dt * r_gyro * dw * tg * r_acc * da));
        g.view_mut((p_i, 3), (3, 3))
            .copy_from(&(-r_k_t * (xi_2 + xi_4 * r_gyro * dw * tg) * r_acc * da));
        g.view_mut((v_i, 3), (3, 3))
            .copy_from(&(-r_k_t * (xi_1 + xi_3 * r_gyro * dw * tg) * r_acc * da));
        g.view_mut((bg_i, 6), (3, 3))
            .copy_from(&(dt * Matrix3::identity()));
        g.view_mut((ba_i, 9), (3, 3))
            .copy_from(&(dt * Matrix3::identity()));
    }

    /// 离散状态转移阵 `F` 与噪声雅可比 `G`（对应 `compute_F_and_G_discrete`）。
    #[allow(
        clippy::too_many_arguments,
        clippy::too_many_lines,
        clippy::similar_names
    )]
    fn compute_f_and_g_discrete(
        &self,
        opts: PropagationOptions,
        calib: &ImuCalibration,
        dt: f64,
        w_uncorrected: &Vector3<f64>,
        a_uncorrected: &Vector3<f64>,
        new_q: &Vector4<f64>,
        new_v: &Vector3<f64>,
        new_p: &Vector3<f64>,
        r_k: &Matrix3<f64>,
        v_k: &Vector3<f64>,
        p_k: &Vector3<f64>,
        f: &mut DMatrix<f64>,
        g: &mut DMatrix<f64>,
    ) {
        let layout = CalibLayout::new(opts);
        let (th_i, p_i, v_i, bg_i, ba_i) = (0usize, 3usize, 6usize, 9usize, 12usize);

        let d_r_ktok1 = quat_2_rot(new_q) * r_k.transpose();

        // 3x3 标定阵为 Copy，取自有符号复制以便全自有运算。
        let dw = calib.dw;
        let da = calib.da;
        let tg = calib.tg;
        let r_acc = calib.r_acc_to_imu;
        let r_gyro = calib.r_gyro_to_imu;
        let a_k = r_acc * da * a_uncorrected;
        let w_k = r_gyro * dw * w_uncorrected;
        let jr = jr_so3(&log_so3(&d_r_ktok1));
        let r_k_t = r_k.transpose();

        // theta 行块
        f.view_mut((th_i, th_i), (3, 3)).copy_from(&d_r_ktok1);
        f.view_mut((th_i, bg_i), (3, 3))
            .copy_from(&(-d_r_ktok1 * jr * dt * r_gyro * dw));
        f.view_mut((th_i, ba_i), (3, 3))
            .copy_from(&(d_r_ktok1 * jr * dt * r_gyro * dw * tg * r_acc * da));

        // 位置行块
        f.view_mut((p_i, th_i), (3, 3)).copy_from(
            &(-skew_x(&(new_p - p_k - v_k * dt + 0.5 * self.gravity * dt * dt)) * r_k_t),
        );
        f.view_mut((p_i, p_i), (3, 3))
            .copy_from(&Matrix3::identity());
        f.view_mut((p_i, v_i), (3, 3))
            .copy_from(&(Matrix3::identity() * dt));
        f.view_mut((p_i, ba_i), (3, 3))
            .copy_from(&(-0.5 * r_k_t * dt * dt * r_acc * da));

        // 速度行块
        f.view_mut((v_i, th_i), (3, 3))
            .copy_from(&(-skew_x(&(new_v - v_k + self.gravity * dt)) * r_k_t));
        f.view_mut((v_i, v_i), (3, 3))
            .copy_from(&Matrix3::identity());
        f.view_mut((v_i, ba_i), (3, 3))
            .copy_from(&(-r_k_t * dt * r_acc * da));

        f.view_mut((bg_i, bg_i), (3, 3))
            .copy_from(&Matrix3::identity());
        f.view_mut((ba_i, ba_i), (3, 3))
            .copy_from(&Matrix3::identity());

        // Dw 标定块
        if let Some(dw_id) = layout.dw {
            let h_dw = compute_h_dw(opts.imu_model, w_uncorrected);
            f.view_mut((th_i, dw_id), (3, 6))
                .copy_from(&(d_r_ktok1 * jr * dt * r_gyro * h_dw));
            identity_blocks(f, dw_id, 6);
        }

        // Da 标定块
        if let Some(da_id) = layout.da {
            let h_da = compute_h_da(opts.imu_model, a_uncorrected);
            f.view_mut((th_i, da_id), (3, 6))
                .copy_from(&(-d_r_ktok1 * jr * dt * r_gyro * tg * r_acc * h_da));
            f.view_mut((p_i, da_id), (3, 6))
                .copy_from(&(0.5 * r_k_t * dt * dt * r_acc * h_da));
            f.view_mut((v_i, da_id), (3, 6))
                .copy_from(&(r_k_t * dt * r_acc * h_da));
            identity_blocks(f, da_id, 6);
        }

        // Tg 标定块
        if let Some(tg_id) = layout.tg {
            let h_tg = compute_h_tg(&a_k);
            f.view_mut((th_i, tg_id), (3, 9))
                .copy_from(&(-d_r_ktok1 * jr * dt * r_gyro * dw * h_tg));
            identity_blocks(f, tg_id, 9);
        }

        // 旋转标定块
        if let Some(rot_id) = layout.rot {
            match opts.imu_model {
                ImuModel::Kalibr => {
                    f.view_mut((th_i, rot_id), (3, 3))
                        .copy_from(&(d_r_ktok1 * jr * dt * skew_x(&w_k)));
                }
                ImuModel::Rpng => {
                    f.view_mut((th_i, rot_id), (3, 3))
                        .copy_from(&(-d_r_ktok1 * jr * dt * r_gyro * dw * tg * skew_x(&a_k)));
                    f.view_mut((p_i, rot_id), (3, 3))
                        .copy_from(&(0.5 * r_k_t * dt * dt * skew_x(&a_k)));
                    f.view_mut((v_i, rot_id), (3, 3))
                        .copy_from(&(r_k_t * dt * skew_x(&a_k)));
                }
            }
            f.view_mut((rot_id, rot_id), (3, 3))
                .copy_from(&Matrix3::identity());
        }

        // G 噪声雅可比
        g.view_mut((th_i, 0), (3, 3))
            .copy_from(&(-d_r_ktok1 * jr * dt * r_gyro * dw));
        g.view_mut((th_i, 3), (3, 3))
            .copy_from(&(d_r_ktok1 * jr * dt * r_gyro * dw * tg * r_acc * da));
        g.view_mut((v_i, 3), (3, 3))
            .copy_from(&(-r_k_t * dt * r_acc * da));
        g.view_mut((p_i, 3), (3, 3))
            .copy_from(&(-0.5 * r_k_t * dt * dt * r_acc * da));
        g.view_mut((bg_i, 6), (3, 3))
            .copy_from(&(dt * Matrix3::identity()));
        g.view_mut((ba_i, 9), (3, 3))
            .copy_from(&(dt * Matrix3::identity()));
    }
}

/// 在 `F` 的对角 `(id, id)` 处置 `n×n` 单位块（归零后写对角线）。
fn identity_blocks(f: &mut DMatrix<f64>, id: usize, n: usize) {
    f.view_mut((id, id), (n, n)).fill(0.0);
    f.view_mut((id, id), (n, n)).fill_diagonal(1.0);
}

/// 标定块的误差态偏移布局（对照 `compute_F_and_G_*` 中的 `local_id` 计算）。
struct CalibLayout {
    dw: Option<usize>,
    da: Option<usize>,
    tg: Option<usize>,
    rot: Option<usize>,
}

impl CalibLayout {
    fn new(opts: PropagationOptions) -> Self {
        if !opts.do_calib_imu_intrinsics {
            return Self {
                dw: None,
                da: None,
                tg: None,
                rot: None,
            };
        }
        let mut local = 15usize;
        let dw = Some(local);
        local += 6;
        let da = Some(local);
        local += 6;
        let tg = if opts.do_calib_imu_g_sensitivity {
            let tg = local;
            local += 9;
            Some(tg)
        } else {
            None
        };
        let rot = Some(local);
        Self { dw, da, tg, rot }
    }
}

/// 陀螺 IMU 内参 `Dw` 的雅可比 `H_Dw`（3×6，对照 `compute_H_Dw`）。
///
/// KALIBR 布局：`[w₁I₃ | w₂e₂ | w₂e₃ | w₃e₃]`；RPNG 布局：`[w₁e₁ | w₂e₁ | w₂e₂ | w₃I₃]`。
#[must_use]
pub fn compute_h_dw(model: ImuModel, w: &Vector3<f64>) -> SMatrix<f64, 3, 6> {
    let e1 = Vector3::new(1.0, 0.0, 0.0);
    let e2 = Vector3::new(0.0, 1.0, 0.0);
    let e3 = Vector3::new(0.0, 0.0, 1.0);
    let col = |m: &mut SMatrix<f64, 3, 6>, c: usize, vec: &Vector3<f64>| {
        for r in 0..3 {
            m[(r, c)] = vec[r];
        }
    };
    let mut h = SMatrix::<f64, 3, 6>::zeros();
    match model {
        ImuModel::Kalibr => {
            col(&mut h, 0, &(w[0] * e1));
            col(&mut h, 1, &(w[0] * e2));
            col(&mut h, 2, &(w[0] * e3));
            col(&mut h, 3, &(w[1] * e2));
            col(&mut h, 4, &(w[1] * e3));
            col(&mut h, 5, &(w[2] * e3));
        }
        ImuModel::Rpng => {
            col(&mut h, 0, &(w[0] * e1));
            col(&mut h, 1, &(w[1] * e1));
            col(&mut h, 2, &(w[1] * e2));
            col(&mut h, 3, &(w[2] * e1));
            col(&mut h, 4, &(w[2] * e2));
            col(&mut h, 5, &(w[2] * e3));
        }
    }
    h
}

/// 加速度计 IMU 内参 `Da` 的雅可比 `H_Da`（3×6，对照 `compute_H_Da`）。
#[must_use]
pub fn compute_h_da(model: ImuModel, a: &Vector3<f64>) -> SMatrix<f64, 3, 6> {
    let e1 = Vector3::new(1.0, 0.0, 0.0);
    let e2 = Vector3::new(0.0, 1.0, 0.0);
    let e3 = Vector3::new(0.0, 0.0, 1.0);
    let col = |m: &mut SMatrix<f64, 3, 6>, c: usize, vec: &Vector3<f64>| {
        for r in 0..3 {
            m[(r, c)] = vec[r];
        }
    };
    let mut h = SMatrix::<f64, 3, 6>::zeros();
    match model {
        ImuModel::Kalibr => {
            col(&mut h, 0, &(a[0] * e1));
            col(&mut h, 1, &(a[0] * e2));
            col(&mut h, 2, &(a[0] * e3));
            col(&mut h, 3, &(a[1] * e2));
            col(&mut h, 4, &(a[1] * e3));
            col(&mut h, 5, &(a[2] * e3));
        }
        ImuModel::Rpng => {
            col(&mut h, 0, &(a[0] * e1));
            col(&mut h, 1, &(a[1] * e1));
            col(&mut h, 2, &(a[1] * e2));
            col(&mut h, 3, &(a[2] * e1));
            col(&mut h, 4, &(a[2] * e2));
            col(&mut h, 5, &(a[2] * e3));
        }
    }
    h
}

/// 重力敏感阵 `Tg` 的雅可比 `H_Tg`（3×9，对照 `compute_H_Tg`）。
///
/// `H_Tg = [a₁I₃ | a₂I₃ | a₃I₃]`，其中 `a` 为去偏置后的加速度（IMU 系、含 IMU 内参校正）。
#[must_use]
pub fn compute_h_tg(a: &Vector3<f64>) -> SMatrix<f64, 3, 9> {
    let e1 = Vector3::new(1.0, 0.0, 0.0);
    let e2 = Vector3::new(0.0, 1.0, 0.0);
    let e3 = Vector3::new(0.0, 0.0, 1.0);
    let mut h = SMatrix::<f64, 3, 9>::zeros();
    for r in 0..3 {
        h[(r, 0)] = a[0] * e1[r];
        h[(r, 1)] = a[0] * e2[r];
        h[(r, 2)] = a[0] * e3[r];
        h[(r, 3)] = a[1] * e1[r];
        h[(r, 4)] = a[1] * e2[r];
        h[(r, 5)] = a[1] * e3[r];
        h[(r, 6)] = a[2] * e1[r];
        h[(r, 7)] = a[2] * e2[r];
        h[(r, 8)] = a[2] * e3[r];
    }
    h
}

#[cfg(test)]
mod tests {
    // 数值测试断言多采用精确值 assert_eq!；此处显式豁免 float_cmp（对照 doc 测试惯例）。
    #![allow(clippy::float_cmp)]

    use super::*;
    use firefly_vio_types::quat_ops::{rot_y, rot_z};

    fn identity_calib() -> ImuCalibration {
        ImuCalibration {
            bias_a: Vector3::new(0.01, -0.02, 0.03),
            bias_g: Vector3::new(0.001, -0.002, 0.003),
            r_acc_to_imu: Matrix3::identity(),
            r_gyro_to_imu: Matrix3::identity(),
            da: Matrix3::identity(),
            dw: Matrix3::identity(),
            tg: Matrix3::zeros(),
        }
    }

    #[test]
    fn fast_state_propagate_zero_imu_matches_gravity() {
        // 零 IMU + 重力：p_z = −½·g·t²，v_z = −g·t（与 predict_and_compute 一致）
        let mut prop = Propagator::new(ImuNoise::default());
        for i in 0..21 {
            prop.feed_imu(
                ImuData {
                    timestamp: 0.05 * f64::from(i),
                    wm: Vector3::zeros(),
                    am: Vector3::zeros(),
                },
                -1.0,
            );
        }
        let calib = identity_calib();
        let est = [
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ];
        let init = FastInit {
            time: 0.0,
            t_off: 0.0,
            est,
            covariance: DMatrix::<f64>::identity(15, 15) * 1e-6,
        };
        let out = prop
            .fast_state_propagate(Some(&init), 1.0, &calib)
            .expect("应有测量可传播");
        let dt = 1.0;
        assert!(
            (out.p.z - (-0.5 * 9.81 * dt * dt)).abs() < 1e-6,
            "p.z = {}",
            out.p.z
        );
        assert!((out.v.z - (-9.81 * dt)).abs() < 1e-6, "v.z = {}", out.v.z);
        // 姿态不变（零角速度）
        assert_eq!(out.q, Vector4::new(0.0, 0.0, 0.0, 1.0));
        // 协方差 12×12
        assert_eq!(out.covariance.nrows(), 12);
    }

    #[test]
    fn fast_state_propagate_uses_cache() {
        // 缓存语义：第二次调用（无 initial）从上次推进处继续，
        // 分段传播与一次传播到同一时刻结果一致
        let mut prop = Propagator::new(ImuNoise::default());
        for i in 0..41 {
            prop.feed_imu(
                ImuData {
                    timestamp: 0.05 * f64::from(i),
                    wm: Vector3::zeros(),
                    am: Vector3::zeros(),
                },
                -1.0,
            );
        }
        let calib = identity_calib();
        let est = [
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ];
        let init = FastInit {
            time: 0.0,
            t_off: 0.0,
            est,
            covariance: DMatrix::<f64>::identity(15, 15) * 1e-6,
        };
        let _out1 = prop
            .fast_state_propagate(Some(&init), 0.5, &calib)
            .expect("第一次应成功");
        let out2 = prop
            .fast_state_propagate(None, 1.0, &calib)
            .expect("缓存应使第二次成功");
        // 分段传播与一次传播一致：p_z(1.0) = −½·g·1²
        assert!(
            (out2.p.z - (-0.5 * 9.81)).abs() < 1e-6,
            "p.z = {}",
            out2.p.z
        );
        assert!((out2.v.z - (-9.81)).abs() < 1e-6, "v.z = {}", out2.v.z);
    }

    fn default_imu_data() -> ImuData {
        ImuData {
            timestamp: 0.0,
            wm: Vector3::new(0.1, -0.2, 0.3),
            am: Vector3::new(1.0, -2.0, 3.0),
        }
    }

    fn approx(a: f64, b: f64, eps: f64) {
        assert!((a - b).abs() < eps, "{a} !~ {b} (eps={eps})");
    }

    fn sample_mean() -> MeanState {
        // 非平凡姿态与位移，覆盖各旋转分块。
        MeanState::new(
            rot_2_quat(&(rot_z(0.4) * rot_y(-0.3))),
            Vector3::new(1.0, -2.0, 3.0),
            Vector3::new(0.2, -0.1, 0.5),
        )
    }

    fn opts_analytical() -> PropagationOptions {
        PropagationOptions {
            integration_method: IntegrationMethod::Analytical,
            imu_model: ImuModel::Kalibr,
            do_calib_imu_intrinsics: true,
            do_calib_imu_g_sensitivity: true,
            do_fej: false,
        }
    }

    fn two_readings(dt: f64) -> (ImuData, ImuData) {
        let mut d1 = default_imu_data();
        d1.timestamp = 0.0;
        let mut d2 = default_imu_data();
        d2.timestamp = dt;
        d2.wm = Vector3::new(0.15, -0.3, 0.4);
        d2.am = Vector3::new(1.2, -2.1, 3.4);
        (d1, d2)
    }

    // ---- 1. RK4 与解析均值一致性 ----
    #[test]
    fn rk4_matches_analytic_mean() {
        let prop = Propagator::default();
        let input = sample_mean();
        let dts = [1e-4, 1e-3, 5e-3, 0.01];
        let w = Vector3::new(0.1, -0.25, 0.6);
        let a = Vector3::new(1.5, -2.0, 3.5);

        for &dt in &dts {
            let xi = Propagator::compute_xi_sum(dt, &w, &a);
            let (q_an, v_an, p_an) = prop.predict_mean_analytic(dt, &xi, &a, &input);
            let (q_rk, v_rk, p_rk) = prop.predict_mean_rk4(dt, &w, &a, &w, &a, &input);
            // RK4 与解析均值在 dt≲10ms 下均为四阶一致，差异 <1e-6。
            for i in 0..4 {
                approx(q_an[i], q_rk[i], 1e-6);
            }
            for i in 0..3 {
                approx(v_an[i], v_rk[i], 1e-6);
                approx(p_an[i], p_rk[i], 1e-6);
            }
        }
    }

    // ---- 2. F 的有限差分验证 ----
    /// 对误差态逐维（th/p/v）施加扰动，数值差商对照 `F·δx`。
    ///
    /// 四元数用左扰动 `q' = dq ⊗ q`、`dq=[0.5·δθ;1]`（与 `IMU::update` boxplus 一致），
    /// 输出 `δθ` 取 `log_so3(R(q')R(q)ᵀ)`；p/v 用直接加性扰动。
    ///
    /// p/v 用 `ε = 1e-7`、容差 1e-4。θ 块用 `ε_th = 1e-4`（信号 ~1e-4 显著大于线性化点随
    /// 扰动的 O(ε²)=1e-8 漂移，能被 1e-5 容差捕获）；并对 θ 取**实际实现**的输入扰动
    /// `δθ_in = log_so3(R(q')R(q)ᵀ)` 作为标量（本库 JPL 约定下 `dq=[0.5ε;1]` 得到
    /// `δθ_in ≈ −ε·e_c`），对照 `F[:,c]·δθ_in[c]`——从而同时校验符号与量级，且对
    /// quat/rot 的符号约定不敏感（早先直接用名义 `ε` 时，θ 信号 < 容差，存在"符号错也能过"
    /// 空档）。
    #[test]
    fn finite_difference_jacobian_f() {
        let prop = Propagator::default();
        let opts = opts_analytical();
        let calib = identity_calib();
        let input = sample_mean();
        let (d_minus, d_plus) = two_readings(0.01);
        let lin = LinearizationPoint::from_state(&input);
        let out = prop.predict_and_compute(&opts, &calib, &d_minus, &d_plus, &input, &lin);
        let eps = 1e-7;
        let eps_th = 1e-4;

        // 计算扰动后的输出差向量 [δθ(3); δp(3); δv(3)]。
        let perturbed = |out_p: &Propagated, out0: &Propagated| {
            let dtheta = log_so3(&(quat_2_rot(&out_p.q) * quat_2_rot(&out0.q).transpose()));
            let dp = out_p.p - out0.p;
            let dv = out_p.v - out0.v;
            let mut res = [0.0f64; 9];
            for r in 0..3 {
                res[r] = dtheta[r];
                res[3 + r] = dp[r];
                res[6 + r] = dv[r];
            }
            res
        };

        // th 扰动（误差列 0..3）。
        // 残差公式为 `dθ_out = log_so3(R(q'_out)·R(q_out)ᵀ)`，与审查方/管理者确认的
        // 原始公式一致（**未交换**，A⁻¹ 侧是 `R(q_out)ᵀ`）。注意该库 `quat_2_rot` 的
        // JPL 约定使 `dq = [0.5·ε·e_c; 1]` 对应的旋转 log 为 `−ε·e_c`
        // （实测 `log_so3(R(dq)·I) ≈ −ε e_c`），故实际实现的输入扰动
        // `δθ_in = log_so3(R(q'_in)·R(q_in)ᵀ) ≈ −ε·e_c`。残差侧推导不变：
        // `R(q'_out)R(q_out)ᵀ = R(q_ktok1)·R(dq)·R(q_ktok1)ᵀ = exp([R(q_ktok1)·δθ_in]×)`
        // ⇒ `dθ_out = R(q_ktok1)·δθ_in = F[:,c]·δθ_in[c]`。因此对照标量取*实际* `δθ_in[c]`
        //（而非名义 `ε`，避免因库的符号约定误判），从而符号与量级都真正得到校验。
        for c in 0..3 {
            let mut e = Vector3::zeros();
            e[c] = 1.0;
            let dq = quatnorm(Vector4::new(
                0.5 * eps_th * e[0],
                0.5 * eps_th * e[1],
                0.5 * eps_th * e[2],
                1.0,
            ));
            let ip = MeanState::new(quat_multiply(&dq, &input.q), input.p, input.v);
            let op = prop.predict_and_compute(&opts, &calib, &d_minus, &d_plus, &ip, &lin);
            let diff = perturbed(&op, &out);
            let delta_in = log_so3(&(quat_2_rot(&ip.q) * quat_2_rot(&input.q).transpose()));
            // 扰动仅作用于单轴，二阶项可忽略 → 输出差 = F[:, c]·δθ_in[c]。
            let eta = delta_in[c];
            assert!(
                (eta.abs() - eps_th).abs() < 1e-8,
                "unexpected perturb scale {eta}"
            );
            for (r, &dv) in diff.iter().enumerate() {
                approx(dv, out.f[(r, c)] * eta, 1e-5);
            }
        }
        // p 扰动（误差列 3..6）
        for c in 0..3 {
            let mut e = Vector3::zeros();
            e[c] = 1.0;
            let ip = MeanState::new(input.q, input.p + eps * e, input.v);
            let op = prop.predict_and_compute(&opts, &calib, &d_minus, &d_plus, &ip, &lin);
            let diff = perturbed(&op, &out);
            for (r, &dv) in diff.iter().enumerate() {
                approx(dv, out.f[(r, 3 + c)] * eps, 1e-4);
            }
        }
        // v 扰动（误差列 6..9）
        for c in 0..3 {
            let mut e = Vector3::zeros();
            e[c] = 1.0;
            let ip = MeanState::new(input.q, input.p, input.v + eps * e);
            let op = prop.predict_and_compute(&opts, &calib, &d_minus, &d_plus, &ip, &lin);
            let diff = perturbed(&op, &out);
            for (r, &dv) in diff.iter().enumerate() {
                approx(dv, out.f[(r, 6 + c)] * eps, 1e-4);
            }
        }
    }

    /// 对误差态 bg/ba 块（误差列 9..15）扰动，数值差商对照 `F·δx` 的对应列。
    ///
    /// bg/ba 位于标定 `calib.bias_g / calib.bias_a`，扰动后重新校正加速度/角速度，
    /// 输出差仅体现在 F 的 bg/ba 列（以及二者对 th/p/v 的耦合列）。容差 1e-4。
    #[test]
    fn finite_difference_bias_columns() {
        let prop = Propagator::default();
        let opts = opts_analytical();
        let calib = identity_calib();
        let input = sample_mean();
        let (d_minus, d_plus) = two_readings(0.01);
        let lin = LinearizationPoint::from_state(&input);
        let out = prop.predict_and_compute(&opts, &calib, &d_minus, &d_plus, &input, &lin);
        let eps = 1e-7;

        let perturbed = |out_p: &Propagated, out0: &Propagated| {
            let dtheta = log_so3(&(quat_2_rot(&out_p.q) * quat_2_rot(&out0.q).transpose()));
            let dp = out_p.p - out0.p;
            let dv = out_p.v - out0.v;
            let mut res = [0.0f64; 9];
            for r in 0..3 {
                res[r] = dtheta[r];
                res[3 + r] = dp[r];
                res[6 + r] = dv[r];
            }
            res
        };

        for (block, col0) in [("bg", 9usize), ("ba", 12usize)] {
            for c in 0..3 {
                let mut e = Vector3::zeros();
                e[c] = 1.0;
                let mut calib2 = identity_calib();
                if block == "bg" {
                    calib2.bias_g = calib.bias_g + eps * e;
                } else {
                    calib2.bias_a = calib.bias_a + eps * e;
                }
                let op = prop.predict_and_compute(&opts, &calib2, &d_minus, &d_plus, &input, &lin);
                let diff = perturbed(&op, &out);
                for (r, &dv) in diff.iter().enumerate() {
                    approx(dv, out.f[(r, col0 + c)] * eps, 1e-4);
                }
            }
        }
    }

    /// 标定开启时 F 的有限差分：扰动 `dw/da/tg` 元，对照 F 的标定列块。
    ///
    /// `dw`→6 列、`da`→6 列、`tg`→9 列；列偏移见 `CalibLayout`（15 基 + 顺序累加）。
    /// Kalibr `Dm` 布局是三角阵，`v_k` 唯一对应 `(i,j)` 元，故直接扰动相应矩阵元即可
    /// 等价于扰动误差态中第 `k` 个 dw/da 参数。
    #[test]
    fn finite_difference_calibration_columns() {
        let prop = Propagator::default();
        let opts = opts_analytical();
        let calib = identity_calib();
        let input = sample_mean();
        let (d_minus, d_plus) = two_readings(0.01);
        let lin = LinearizationPoint::from_state(&input);
        let out = prop.predict_and_compute(&opts, &calib, &d_minus, &d_plus, &input, &lin);
        let eps = 1e-7;

        let perturbed = |out_p: &Propagated, out0: &Propagated| {
            let dtheta = log_so3(&(quat_2_rot(&out_p.q) * quat_2_rot(&out0.q).transpose()));
            let dp = out_p.p - out0.p;
            let dv = out_p.v - out0.v;
            let mut res = [0.0f64; 9];
            for r in 0..3 {
                res[r] = dtheta[r];
                res[3 + r] = dp[r];
                res[6 + r] = dv[r];
            }
            res
        };

        // Kalibr Dm 布局下 6 参数对应的 (i, j) 元：v_k → dw[(i,j)]。
        let dm_entries = [(0, 0), (1, 0), (2, 0), (1, 1), (2, 1), (2, 2)];
        let check_column = |col0: usize, apply: &dyn Fn(&mut ImuCalibration, usize)| {
            for (k, &(i, j)) in dm_entries.iter().enumerate() {
                let mut calib2 = identity_calib();
                // nalgebra 列主序：元 (row=i, col=j) 的扁平索引 = i + 3*j。
                apply(&mut calib2, i + 3 * j);
                let op = prop.predict_and_compute(&opts, &calib2, &d_minus, &d_plus, &input, &lin);
                let diff = perturbed(&op, &out);
                for (r, &dv) in diff.iter().enumerate() {
                    approx(dv, out.f[(r, col0 + k)] * eps, 1e-3);
                }
            }
        };

        // dw 列（col0=15）：扰动 dw 矩阵元。
        check_column(15, &|c, flat| {
            c.dw.as_mut_slice()[flat] += eps;
        });
        // da 列（col0=21）：扰动 da 矩阵元。
        check_column(21, &|c, flat| {
            c.da.as_mut_slice()[flat] += eps;
        });
        // tg 列（col0=27）：3×3 列主序 9 参数，tg[(i,j)]。
        for p in 0..9 {
            let (i, j) = (p % 3, p / 3);
            let mut calib2 = identity_calib();
            calib2.tg[(i, j)] += eps;
            let op = prop.predict_and_compute(&opts, &calib2, &d_minus, &d_plus, &input, &lin);
            let diff = perturbed(&op, &out);
            for (r, &dv) in diff.iter().enumerate() {
                approx(dv, out.f[(r, 27 + p)] * eps, 1e-3);
            }
        }
    }

    /// discrete vs analytic 的 F 在 `dt → 0` 时应收敛一致。
    ///
    /// 容差取 `1e-6`：`dt = 1e-4` 时两种积分法的误差级为 `O(dt)`，远小于该容差。
    #[test]
    fn discrete_and_analytic_f_agree_as_dt_tends_zero() {
        let prop = Propagator::default();
        let calib = identity_calib();
        let input = sample_mean();
        let dt = 1e-4;
        let mut d_minus = default_imu_data();
        d_minus.timestamp = 0.0;
        let mut d_plus = default_imu_data();
        d_plus.timestamp = dt;
        let lin = LinearizationPoint::from_state(&input);

        let opts_d = PropagationOptions {
            integration_method: IntegrationMethod::Discrete,
            ..opts_analytical()
        };
        let opts_a = PropagationOptions {
            integration_method: IntegrationMethod::Analytical,
            ..opts_analytical()
        };

        let fd = prop.predict_and_compute(&opts_d, &calib, &d_minus, &d_plus, &input, &lin);
        let fa = prop.predict_and_compute(&opts_a, &calib, &d_minus, &d_plus, &input, &lin);
        for r in 0..15 {
            for c in 0..15 {
                approx(fd.f[(r, c)], fa.f[(r, c)], 1e-6);
            }
        }
    }

    // ---- 3. Qd 对称半正定；dt→0 行为 ----
    #[test]
    fn qd_is_symmetric_psd() {
        let prop = Propagator::default();
        let opts = opts_analytical();
        let calib = identity_calib();
        let input = sample_mean();
        let (d_minus, d_plus) = two_readings(0.01);
        let out = prop.predict_and_compute(
            &opts,
            &calib,
            &d_minus,
            &d_plus,
            &input,
            &LinearizationPoint::from_state(&input),
        );
        let diff = &out.qd - out.qd.transpose();
        let max_err = diff.iter().fold(0.0f64, |m, &x| m.max(x.abs()));
        assert!(max_err < 1e-12, "Qd not symmetric: {max_err}");
        let sym = 0.5 * (out.qd.clone() + out.qd.transpose());
        for &e in sym.symmetric_eigenvalues().iter() {
            assert!(e >= -1e-6, "Qd has negative eigenvalue {e}");
        }
    }

    #[test]
    fn qd_finite_and_psd_for_tiny_dt() {
        let prop = Propagator::default();
        let opts = opts_analytical();
        let calib = identity_calib();
        let input = sample_mean();
        for dt in [1e-6, 1e-4] {
            let (d_minus, d_plus) = two_readings(dt);
            let out = prop.predict_and_compute(
                &opts,
                &calib,
                &d_minus,
                &d_plus,
                &input,
                &LinearizationPoint::from_state(&input),
            );
            assert!(out.qd.iter().all(|x| x.is_finite()));
            let sym = 0.5 * (out.qd.clone() + out.qd.transpose());
            for &e in sym.symmetric_eigenvalues().iter() {
                assert!(e >= -1e-6);
            }
        }
    }

    // ---- 4. 零/极小角速度奇点 ----
    #[test]
    fn tiny_angular_velocity_no_singularity() {
        let prop = Propagator::default();
        let w_zero = Vector3::new(0.0, 0.0, 0.0);
        let w_small = Vector3::new(1e-13, 0.0, 0.0);
        let a = Vector3::new(1.0, -1.0, 2.0);
        let input = sample_mean();
        let dt = 0.01;

        let xi0 = Propagator::compute_xi_sum(dt, &w_zero, &a);
        let xi1 = Propagator::compute_xi_sum(dt, &w_small, &a);
        assert!(xi0.iter().all(|x| x.is_finite()));
        assert!(xi1.iter().all(|x| x.is_finite()));

        let (q0, v0, p0) = prop.predict_mean_analytic(dt, &xi0, &a, &input);
        let (q1, v1, p1) = prop.predict_mean_analytic(dt, &xi1, &a, &input);
        assert!(q0.iter().all(|x| x.is_finite()));
        assert!(v0.iter().all(|x| x.is_finite()));
        assert!(p0.iter().all(|x| x.is_finite()));
        for i in 0..3 {
            approx(q0[i], q1[i], 1e-4);
            approx(v0[i], v1[i], 1e-4);
            approx(p0[i], p1[i], 1e-4);
        }
    }

    #[test]
    fn discrete_matches_analytic_when_zero_omega() {
        let prop = Propagator::default();
        let w = Vector3::new(0.0, 0.0, 0.0);
        let a = Vector3::new(2.0, -1.0, 0.5);
        let input = sample_mean();
        let dt = 0.005;
        let (q_d, v_d, p_d) = prop.predict_mean_discrete(dt, &w, &a, &input);
        let (q_an, v_an, p_an) =
            prop.predict_mean_analytic(dt, &Propagator::compute_xi_sum(dt, &w, &a), &a, &input);
        for i in 0..4 {
            approx(q_d[i], q_an[i], 1e-6);
        }
        for i in 0..3 {
            approx(v_d[i], v_an[i], 1e-6);
            approx(p_d[i], p_an[i], 1e-6);
        }
    }

    // ---- 5. select_imu_readings 边界 ----
    fn readings(times: &[f64]) -> Vec<ImuData> {
        times
            .iter()
            .map(|&t| ImuData {
                timestamp: t,
                wm: Vector3::zeros(),
                am: Vector3::zeros(),
            })
            .collect()
    }

    #[test]
    fn select_empty_returns_none() {
        assert!(Propagator::select_imu_readings(&[], 0.0, 1.0, false).is_none());
    }

    #[test]
    fn select_cuts_start_and_end() {
        let data = readings(&[-0.02, 0.0, 0.05, 0.10, 0.13]);
        let got = Propagator::select_imu_readings(&data, 0.01, 0.12, false).unwrap();
        assert_eq!(got.first().unwrap().timestamp, 0.01);
        assert_eq!(got.last().unwrap().timestamp, 0.12);
        // 内部整段测量 `0.05` 被完整保留。
        assert!(got.iter().any(|d| d.timestamp == 0.05));
        // 恰好落在区间外的端点不进入结果。
        assert!(!got.iter().any(|d| d.timestamp == 0.13));
    }

    #[test]
    fn select_exact_boundary() {
        let data = readings(&[-0.02, 0.0, 0.05, 0.10, 0.13]);
        let got = Propagator::select_imu_readings(&data, 0.0, 0.10, false).unwrap();
        assert_eq!(got.first().unwrap().timestamp, 0.0);
        assert_eq!(got.last().unwrap().timestamp, 0.10);
    }

    #[test]
    fn select_insufficient_returns_none() {
        // 所有 IMU 数据时间戳都早于整个积分区间 → 无法前向传播（CASE3 i==0 break）。
        let data = readings(&[0.05, 0.06, 0.08]);
        assert!(Propagator::select_imu_readings(&data, 0.0, 0.01, false).is_none());
    }

    #[test]
    fn select_stretch_when_missing_tail() {
        let data = readings(&[-0.02, 0.0, 0.05, 0.10]);
        // time1 超出末测量，应外推到 time1。
        let got = Propagator::select_imu_readings(&data, 0.01, 0.15, false).unwrap();
        assert_eq!(got.last().unwrap().timestamp, 0.15);
    }

    // ---- 6. H_Dw / H_Da / H_Tg 对照头文件公式 ----
    #[test]
    fn h_dw_kalibr_layout() {
        let h = compute_h_dw(ImuModel::Kalibr, &Vector3::new(2.0, 3.0, 4.0));
        assert_eq!(h[(0, 0)], 2.0);
        assert_eq!(h[(1, 1)], 2.0);
        assert_eq!(h[(2, 2)], 2.0);
        assert_eq!(h[(1, 3)], 3.0); // w2 e2
        assert_eq!(h[(2, 4)], 3.0); // w2 e3
        assert_eq!(h[(2, 5)], 4.0); // w3 e3
    }

    #[test]
    fn h_dw_rpng_layout() {
        let h = compute_h_dw(ImuModel::Rpng, &Vector3::new(2.0, 3.0, 4.0));
        assert_eq!(h[(0, 0)], 2.0); // w1 e1
        assert_eq!(h[(0, 1)], 3.0); // w2 e1
        assert_eq!(h[(1, 2)], 3.0); // w2 e2
        assert_eq!(h[(0, 3)], 4.0); // w3 e1
        assert_eq!(h[(1, 4)], 4.0); // w3 e2
        assert_eq!(h[(2, 5)], 4.0); // w3 e3
    }

    #[test]
    fn h_da_kalibr_and_rpng_layout() {
        let h_k = compute_h_da(ImuModel::Kalibr, &Vector3::new(1.0, 2.0, 3.0));
        // [a1 I | a2 e2 | a2 e3 | a3 e3]
        assert_eq!(h_k[(0, 0)], 1.0);
        assert_eq!(h_k[(1, 1)], 1.0);
        assert_eq!(h_k[(2, 2)], 1.0);
        assert_eq!(h_k[(1, 3)], 2.0);
        assert_eq!(h_k[(2, 4)], 2.0);
        assert_eq!(h_k[(2, 5)], 3.0);
        let h_r = compute_h_da(ImuModel::Rpng, &Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(h_r[(0, 0)], 1.0); // a1 e1
        assert_eq!(h_r[(0, 1)], 2.0); // a2 e1
        assert_eq!(h_r[(1, 2)], 2.0); // a2 e2
        assert_eq!(h_r[(0, 3)], 3.0); // a3 e1
        assert_eq!(h_r[(1, 4)], 3.0); // a3 e2
        assert_eq!(h_r[(2, 5)], 3.0); // a3 e3
    }

    #[test]
    fn h_tg_layout() {
        let h = compute_h_tg(&Vector3::new(1.0, 2.0, 3.0));
        // [a1 I | a2 I | a3 I]
        assert_eq!(h[(0, 0)], 1.0);
        assert_eq!(h[(1, 1)], 1.0);
        assert_eq!(h[(2, 2)], 1.0);
        assert_eq!(h[(0, 3)], 2.0);
        assert_eq!(h[(1, 4)], 2.0);
        assert_eq!(h[(2, 5)], 2.0);
        assert_eq!(h[(0, 6)], 3.0);
        assert_eq!(h[(1, 7)], 3.0);
        assert_eq!(h[(2, 8)], 3.0);
    }

    // ---- 7. 重力方向/符号与 JPL 惯例 ----
    #[test]
    fn gravity_direction_default_down() {
        let prop = Propagator::default();
        let input = MeanState::new(
            rot_2_quat(&Matrix3::identity()),
            Vector3::zeros(),
            Vector3::zeros(),
        );
        let dt = 0.1;
        let (_, v, p) =
            prop.predict_mean_discrete(dt, &Vector3::zeros(), &Vector3::zeros(), &input);
        // 静止且零加速度：速度被重力拉向下（g 取 +z，减 g·dt → v_z 为负）。
        approx(v[2], -9.81 * dt, 1e-12);
        approx(p[2], -0.5 * 9.81 * dt * dt, 1e-12);
    }

    #[test]
    fn imu_acceleration_rotated_to_global() {
        let prop = Propagator::default();
        // R_GtoI = Ry(90°)：把全局 +z 映射到 IMU +x，故 IMU +x 在全局为 +z。
        let q = rot_2_quat(&rot_y(std::f64::consts::FRAC_PI_2));
        let input = MeanState::new(q, Vector3::zeros(), Vector3::zeros());
        let a = Vector3::new(1.0, 0.0, 0.0);
        let dt = 0.1;
        let (_, v, _) = prop.predict_mean_discrete(dt, &Vector3::zeros(), &a, &input);
        // v = R_GtoI^T·a·dt - g·dt；R_GtoI^T·(1,0,0) = (0,0,1)。
        approx(v[0], 0.0, 1e-15);
        approx(v[1], 0.0, 1e-15);
        approx(v[2], (1.0 - 9.81) * dt, 1e-12);
    }

    #[test]
    fn exp_so3_sign_consistent_with_quat_ops() {
        // 校验我们使用的 exp_so3(±w) 与 quat_ops 正方向一致。
        let w = Vector3::new(0.0, 0.0, 0.5);
        approx(log_so3(&exp_so3(&(w * 0.1)))[2], 0.05, 1e-12);
        approx(log_so3(&exp_so3(&(-w * 0.1)))[2], -0.05, 1e-12);
    }

    #[test]
    fn interpolate_data_matches_linear() {
        let d1 = ImuData {
            timestamp: 0.0,
            wm: Vector3::new(1.0, 2.0, 3.0),
            am: Vector3::new(4.0, 5.0, 6.0),
        };
        let d2 = ImuData {
            timestamp: 1.0,
            wm: Vector3::new(3.0, 4.0, 7.0),
            am: Vector3::new(6.0, 9.0, 8.0),
        };
        let mid = Propagator::interpolate_data(&d1, &d2, 0.5);
        assert_eq!(mid.timestamp, 0.5);
        assert_eq!(mid.wm, Vector3::new(2.0, 3.0, 5.0));
        assert_eq!(mid.am, Vector3::new(5.0, 7.0, 7.0));
    }

    #[test]
    fn feed_and_clean_imu() {
        let prop = Propagator::new_with_gravity(ImuNoise::default(), 9.81);
        prop.feed_imu(default_imu_data(), -1.0);
        let mut d = default_imu_data();
        d.timestamp = 0.05;
        prop.feed_imu(d, -1.0);
        assert_eq!(prop.imu_data_len(), 2);
        prop.clean_old_imu_measurements(0.03);
        assert_eq!(prop.imu_data_len(), 1);
        assert_eq!(prop.imu_data_snapshot()[0].timestamp, 0.05);
    }

    // ---- 8. 无标定块时 F 维度 ----
    #[test]
    fn f_dimension_without_calibration() {
        let prop = Propagator::default();
        let opts = PropagationOptions {
            do_calib_imu_intrinsics: false,
            do_calib_imu_g_sensitivity: false,
            ..opts_analytical()
        };
        let calib = identity_calib();
        let input = sample_mean();
        let (d_minus, d_plus) = two_readings(0.01);
        let out = prop.predict_and_compute(
            &opts,
            &calib,
            &d_minus,
            &d_plus,
            &input,
            &LinearizationPoint::from_state(&input),
        );
        assert_eq!(out.f.nrows(), 15);
        assert_eq!(out.f.ncols(), 15);
        assert_eq!(out.qd.nrows(), 15);
        assert_eq!(out.qd.ncols(), 15);
    }
}
