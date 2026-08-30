//! DIVO 测量模型（P3 实现）。
//!
//! 技术蓝本为 FAST-LIVO2 论文第 VI 节（深度点-平面残差）与第 VII 节
//! （稀疏直接视觉残差、仿射扭曲、光度/曝光、外点剔除），官方实现见
//! `src/voxel_map.cpp`（`BuildResidualListOMP`）与 `src/vio.cpp`。
//!
//! 本 crate 在 P3 实现 [`firefly_void_esikf::update::MeasurementModel`] 的
//! 具体测量模型并注入顺序更新框架；本阶段只保留空壳保证编译。

/// 深度点-平面测量模型占位（P3 实现）。
pub struct DepthMeasurement;
