//! DIVO 测量模型（P3，论文 VI/VII 节）。
//!
//! 技术蓝本为 FAST-LIVO2（`~/Projects/fast_livo2/`）：
//! - 深度点-平面残差（VI 节）：[`plane_update::DepthMeasurement`]，
//!   含深度相机版不确定度模型 [`noise::DepthNoise`]（仿真视差域
//!   `σ∝z²`，`packages/firefly-mujoco/.../env.py:160-171`）；
//! - 稀疏直接视觉残差（VII 节）：[`visual_update::VisualMeasurement`]，
//!   三层金字塔对齐 + 逆曝光联合估计（对照 `vio.cpp:1520` `updateState`）；
//! - 外点剔除与鲁棒核（VII-A 末段）：[`outlier`]；
//! - 多假设重定位初值（P4 启动/退化恢复）：[`relocalize`]。
//!
//! 测量模型实现 [`firefly_void_esikf::update::MeasurementModel`]，注入
//! P1 的顺序更新框架（`EskfUpdater`）。

pub mod noise;
pub mod options;
pub mod outlier;
pub mod planar;
pub mod plane_update;
pub mod prior_update;
pub mod relocalize;
pub mod visual_update;

pub use noise::DepthNoise;
pub use options::{DepthOptions, PriorOptions, RelocalizeOptions, VisualOptions};
pub use plane_update::{DepthMeasurement, point_plane_residual};
pub use prior_update::{PriorDiag, PriorPlaneMeasurement};
pub use relocalize::relocalize_guess;
pub use visual_update::VisualMeasurement;
