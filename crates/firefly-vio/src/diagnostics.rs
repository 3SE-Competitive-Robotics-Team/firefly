//! 相机帧内滤波更新诊断；仅保存数值快照，不参与估计。
use crate::state::State;
use firefly_vio_types::var::Variable;

/// IMU 名义状态及误差协方差，排列与 `OpenVINS` `ImuType` 一致。
#[derive(Debug, Clone)]
pub struct ImuSnapshot {
    /// 名义状态 `[q_GtoI(xyzw), p, v, bg, ba]`，除四元数外采用 SI 单位。
    pub value: [f64; 16],
    /// 误差状态 [theta, p, v, bg, ba] 的对角方差。
    pub variance: [f64; 15],
    /// 姿态、陀螺零偏、加计零偏的 9×9 联合边缘协方差，按行展开。
    pub attitude_bias_covariance: [f64; 81],
}
impl ImuSnapshot {
    /// 从当前滤波状态提取联合边缘分布。
    #[must_use]
    pub fn capture(state: &State) -> Self {
        let mut value = [0.; 16];
        value.copy_from_slice(state.imu.value().as_slice());
        let offset = state.imu.id() as usize;
        let indices = [0, 1, 2, 9, 10, 11, 12, 13, 14];
        Self {
            value,
            variance: std::array::from_fn(|i| state.cov[(offset + i, offset + i)]),
            attitude_bias_covariance: std::array::from_fn(|i| {
                state.cov[(offset + indices[i / 9], offset + indices[i % 9])]
            }),
        }
    }
}

/// 同一相机帧、同一更新来源的输入输出及创新汇总。
#[derive(Debug, Clone)]
pub struct UpdateDiagnostic {
    /// 更新来源；同一帧内相同来源合并。
    pub source: &'static str,
    /// 该来源第一次更新前的状态。
    pub before: ImuSnapshot,
    /// 该来源最后一次更新后的状态。
    pub after: ImuSnapshot,
    /// 残差维数之和、残差 RMS、各次 NIS 之和、更新次数。
    pub innovation: [f64; 4],
    /// 各次 IMU 15 维误差修正之和（各次局部切空间；姿态 rad，其余为 SI）。
    pub correction: [f64; 15],
}
