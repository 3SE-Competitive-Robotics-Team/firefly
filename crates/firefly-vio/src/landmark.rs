//! SLAM 特征类型（对照 `OpenVINS` `ov_core/src/types/Landmark.h`）。
//!
//! 持久的 SLAM 特征：`featid` 与跟踪器一致，值为表示相关的参数向量
//! （`GLOBAL_3D` → 全局 3D；`ANCHORED_MSCKF_INVERSE_DEPTH` → 锚点系
//! `α, β, ρ`），`get_xyz`/`set_from_xyz` 负责坐标转换。
//!
//! TODO：锚定/逆深度单深度等其余表示的完整转换（当前支持
//! `GLOBAL_3D` 与 `ANCHORED_MSCKF_INVERSE_DEPTH`）。

use nalgebra::{DMatrix, DVector, Vector3};

use crate::options::FeatRepresentation;

/// SLAM 特征（对照 `Landmark`，继承 `Type` 的 id/size 语义）。
#[derive(Debug, Clone)]
pub struct Landmark {
    /// 特征 id（与跟踪器数据库一致）。
    pub featid: usize,
    /// 在状态协方差中的位置（对照 `Type::_id`；未初始化时为 -1）。
    pub id: i32,
    /// 观测该特征的首要相机流 id。
    pub unique_camera_id: i32,
    /// 锚点相机 id（锚定表示）。
    pub anchor_cam_id: i32,
    /// 锚点克隆时刻（锚定表示）。
    pub anchor_clone_timestamp: f64,
    /// 是否发生过锚点切换。
    pub has_had_anchor_change: bool,
    /// 是否应被边缘化。
    pub should_marg: bool,
    /// 连续更新失败次数。
    pub update_fail_count: usize,
    /// 当前估计（表示相关维度：3 或 1）。
    pub value: DVector<f64>,
    /// 首估计（FEJ）。
    pub fej: DVector<f64>,
    /// 初始方位（`ANCHORED_INVERSE_DEPTH_SINGLE` 用；单位向量，
    /// 对照 C++ `uv_norm_zero`）。
    pub uv_norm_zero: Vector3<f64>,
    /// 初始方位 FEJ（对照 C++ `uv_norm_zero_fej`）。
    pub uv_norm_zero_fej: Vector3<f64>,
    /// 特征表示。
    pub representation: FeatRepresentation,
}

impl Landmark {
    /// 构造（对照 `Landmark(int dim)`：维度由表示决定）。
    #[must_use]
    pub fn new(representation: FeatRepresentation, featid: usize) -> Self {
        let dim = if representation == FeatRepresentation::AnchoredInverseDepthSingle {
            1
        } else {
            3
        };
        Self {
            featid,
            id: -1,
            unique_camera_id: -1,
            anchor_cam_id: -1,
            anchor_clone_timestamp: -1.0,
            has_had_anchor_change: false,
            should_marg: false,
            update_fail_count: 0,
            value: DVector::zeros(dim),
            fej: DVector::zeros(dim),
            uv_norm_zero: Vector3::zeros(),
            uv_norm_zero_fej: Vector3::zeros(),
            representation,
        }
    }

    /// 状态变量 id（对照 `Type::id`）。
    #[must_use]
    pub fn id(&self) -> i32 {
        self.id
    }

    /// 设置状态变量 id（对照 `Type::set_local_id`）。
    pub fn set_local_id(&mut self, id: i32) {
        self.id = id;
    }

    /// 状态变量维度（对照 `Type::size`）。
    #[must_use]
    pub fn size(&self) -> usize {
        self.value.len()
    }

    /// 由特征位置设置值/FEJ（对照 `Landmark::set_from_xyz`）。
    ///
    /// `p_Fin` 为锚点系（锚定表示）或全局系（`GLOBAL_3D`）坐标。
    pub fn set_from_xyz(&mut self, p_fin: &Vector3<f64>, is_fej: bool) {
        // 单逆深度：同时记录初始方位（对照 C++ 的 uv_norm_zero 段）
        if self.representation == FeatRepresentation::AnchoredInverseDepthSingle {
            let bearing = (1.0 / p_fin.z) * p_fin;
            if is_fej {
                self.uv_norm_zero_fej = bearing;
            } else {
                self.uv_norm_zero = bearing;
            }
        }
        let v = match self.representation {
            FeatRepresentation::Global3D | FeatRepresentation::Anchored3D => {
                DVector::from_column_slice(p_fin.as_slice())
            }
            // 锚定/全局全逆深度（θ,φ,ρ；对照 C++ 的 g_rho/g_phi/g_theta）
            FeatRepresentation::AnchoredFullInverseDepth
            | FeatRepresentation::GlobalFullInverseDepth => {
                let rho = 1.0 / p_fin.norm();
                let phi = (rho * p_fin.z).acos();
                let theta = p_fin.y.atan2(p_fin.x);
                DVector::from_column_slice(&[theta, phi, rho])
            }
            FeatRepresentation::AnchoredMsckfInverseDepth => {
                // α = px/pz, β = py/pz, ρ = 1/pz
                DVector::from_column_slice(&[p_fin.x / p_fin.z, p_fin.y / p_fin.z, 1.0 / p_fin.z])
            }
            // 单逆深度：只存深度 ρ
            FeatRepresentation::AnchoredInverseDepthSingle => {
                DVector::from_column_slice(&[1.0 / p_fin.z])
            }
        };
        if is_fej {
            self.fej = v;
        } else {
            self.value = v;
        }
    }

    /// 返回特征位置（锚定系/全局系，对照 `Landmark::get_xyz`）。
    #[must_use]
    pub fn get_xyz(&self, fej: bool) -> Vector3<f64> {
        let v = if fej { &self.fej } else { &self.value };
        match self.representation {
            FeatRepresentation::Global3D | FeatRepresentation::Anchored3D => {
                Vector3::new(v[0], v[1], v[2])
            }
            // 全逆深度：p = ρ⁻¹·[cosθ·sinφ, sinθ·sinφ, cosφ]
            FeatRepresentation::AnchoredFullInverseDepth
            | FeatRepresentation::GlobalFullInverseDepth => {
                let (sin_th, cos_th) = v[0].sin_cos();
                let (sin_phi, cos_phi) = v[1].sin_cos();
                Vector3::new(
                    (1.0 / v[2]) * cos_th * sin_phi,
                    (1.0 / v[2]) * sin_th * sin_phi,
                    (1.0 / v[2]) * cos_phi,
                )
            }
            FeatRepresentation::AnchoredMsckfInverseDepth => {
                // p = ρ⁻¹·[α, β, 1]
                Vector3::new(v[0] / v[2], v[1] / v[2], 1.0 / v[2])
            }
            // 单逆深度：p = ρ⁻¹·方位（初始方位固定）
            FeatRepresentation::AnchoredInverseDepthSingle => {
                let bearing = if fej {
                    &self.uv_norm_zero_fej
                } else {
                    &self.uv_norm_zero
                };
                (1.0 / v[0]) * bearing
            }
        }
    }

    /// 用误差增量更新（对照 `Landmark::update`：加性）。
    pub fn update(&mut self, dx: &DVector<f64>) {
        debug_assert_eq!(dx.len(), self.value.len());
        self.value += dx;
    }
}

/// 二维投影辅助（未使用，占位保持与 C++ Landmark 的几何语义对应）。
#[allow(dead_code)]
type JacobianPlaceholder = DMatrix<f64>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn global_3d_roundtrip() {
        let mut lm = Landmark::new(FeatRepresentation::Global3D, 1);
        let p = Vector3::new(1.0, 2.0, 3.0);
        lm.set_from_xyz(&p, false);
        assert_eq!(lm.get_xyz(false), p);
    }

    #[test]
    fn anchored_inverse_depth_roundtrip() {
        let mut lm = Landmark::new(FeatRepresentation::AnchoredMsckfInverseDepth, 2);
        let p = Vector3::new(0.5, 0.2, 3.0);
        lm.set_from_xyz(&p, false);
        let back = lm.get_xyz(false);
        assert!((back - p).norm() < 1e-12);
        assert_eq!(lm.value.len(), 3);
    }

    #[test]
    fn anchored_3d_roundtrip() {
        let mut lm = Landmark::new(FeatRepresentation::Anchored3D, 4);
        let p = Vector3::new(0.3, -1.2, 4.5);
        lm.set_from_xyz(&p, false);
        assert!((lm.get_xyz(false) - p).norm() < 1e-12);
    }

    #[test]
    fn full_inverse_depth_roundtrip() {
        let mut lm = Landmark::new(FeatRepresentation::AnchoredFullInverseDepth, 5);
        let p = Vector3::new(1.0, 2.0, 3.0);
        lm.set_from_xyz(&p, false);
        let back = lm.get_xyz(false);
        assert!((back - p).norm() < 1e-9, "roundtrip {back} vs {p}");
        // θ = atan2(y,x)，φ = acos(ρ·z)，ρ = 1/‖p‖
        let v = &lm.value;
        assert!((v[2] - 1.0 / p.norm()).abs() < 1e-12);
        assert!((v[0] - p.y.atan2(p.x)).abs() < 1e-12);
    }

    #[test]
    fn single_inverse_depth_uses_locked_bearing() {
        let mut lm = Landmark::new(FeatRepresentation::AnchoredInverseDepthSingle, 6);
        let p = Vector3::new(0.5, 0.2, 3.0);
        lm.set_from_xyz(&p, false);
        assert_eq!(lm.size(), 1);
        // 方位 = ρ·p = p/z（单位化方向）
        let bearing = lm.uv_norm_zero;
        let expected = p / p.z;
        assert!((bearing - expected).norm() < 1e-12);
        // get_xyz = (1/ρ)·bearing = p
        let back = lm.get_xyz(false);
        assert!((back - p).norm() < 1e-9);
    }

    #[test]
    fn fej_tracks_separately() {
        let mut lm = Landmark::new(FeatRepresentation::Global3D, 3);
        lm.set_from_xyz(&Vector3::new(1.0, 0.0, 0.0), true);
        lm.set_from_xyz(&Vector3::new(2.0, 0.0, 0.0), false);
        assert!((lm.get_xyz(true).x - 1.0).abs() < 1e-12);
        assert!((lm.get_xyz(false).x - 2.0).abs() < 1e-12);
    }
}
