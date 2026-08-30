//! 深度点不确定度模型（论文 VI-B 的深度相机版）。
//!
//! 官方 `LiDAR` 点不确定度分解为 TOF 距离不确定度 + 编码器方向不确定度 +
//! 束发散角（论文 (20) 式，`voxel_map.cpp:15` `calcBodyCov`）。深度相机
//! 无束发散角，对应替代为：
//! - 距离不确定度 `δd`：仿真视差域高斯 `σ_disp = 4·depth_noise` px，
//!   `σ_z ≈ z²·σ_disp/(f·B)`（`env.py:160-171`），即论文 (20) 式的
//!   深度版（束发散角项 → 空洞邻域不确定度，见 [`DepthNoise`]）；
//! - 方向不确定度 `δω`：像素量化角（1/2 像素 → 角分辨率 `arctan(1/(2f))`）。
//!
//! 协方差结构对照 `calcBodyCov`：`Σ = d̂·σ_d²·d̂ᵀ + A·Σ_ω·Aᵀ`，
//! `A = r·⌊d̂×⌋·N`（`N` 为切平面基）。

use nalgebra::{Matrix2, Matrix3, Vector3};

use crate::options::DepthOptions;

/// 由深度噪声参数构造的点噪声模型。
#[derive(Debug, Clone, Copy)]
pub struct DepthNoise {
    /// `σ_z = z²·σ_disp/(f·B)` 的系数（`DepthOptions::depth_sigma_coeff`）。
    depth_sigma_coeff: f64,
    /// 空洞邻域不确定度：空洞率 5~15% 造成邻域深度跳变，等效
    /// 论文 (20) 式束发散角的深度版（取 `σ = 0.02·z`，`env.py:178`
    /// 边缘阈值 `0.04·z` 的一半）。
    hole_coeff: f64,
    /// 方向不确定度角（rad）。
    angle_sigma: f64,
}

impl DepthNoise {
    /// 构造。
    ///
    /// `angle_sigma` 为像素量化角不确定度（rad）；缺省由
    /// [`DepthNoise::from_intrinsics`] 从内参推出。
    #[must_use]
    pub fn new(opts: &DepthOptions, angle_sigma: f64) -> Self {
        Self {
            depth_sigma_coeff: opts.depth_sigma_coeff,
            hole_coeff: 0.02,
            angle_sigma,
        }
    }

    /// 由相机内参构造：方向不确定度 = 1/2 像素的角分辨率
    /// `arctan(1/(2f))`（x/y 取平均）。
    #[must_use]
    pub fn from_intrinsics(opts: &DepthOptions, fx: f64, fy: f64) -> Self {
        let ang = (0.5 / fx).atan() + (0.5 / fy).atan();
        Self::new(opts, 0.5 * ang)
    }

    /// 距离标准差 `σ_z(z)`（m）。
    ///
    /// `σ_z = z²·σ_disp/(f·B)` 加空洞邻域项 `σ_hole = hole_coeff·z`，
    /// 两独立噪声源平方和。
    #[must_use]
    pub fn range_sigma(&self, z: f64) -> f64 {
        let disp_term = self.depth_sigma_coeff * z * z;
        let hole_term = self.hole_coeff * z;
        (disp_term * disp_term + hole_term * hole_term).sqrt()
    }

    /// 深度点在深度相机系下的 3×3 协方差（论文 (19) 式 `Σ_pj`）。
    ///
    /// 结构对照 `calcBodyCov`（`voxel_map.cpp:15-34`）：
    /// `Σ = d̂·σ_d²·d̂ᵀ + A·Σ_ω·Aᵀ`，`A = r·⌊d̂×⌋·N`；
    /// `Σ_ω = diag(σ_ω², σ_ω²)` 为切平面两个方向的方向不确定度。
    #[must_use]
    pub fn point_covariance(&self, p_cam: &Vector3<f64>) -> Matrix3<f64> {
        let r = p_cam.norm();
        let dir = p_cam / r;
        let range_var = self.range_sigma(r);
        let range_var = range_var * range_var;

        // 切平面正交基（对照 calcBodyCov 的 base_vector1/base_vector2）
        let base1 = if dir[2].abs() > 1e-6 {
            Vector3::new(1.0, 1.0, -(dir[0] + dir[1]) / dir[2]).normalize()
        } else {
            Vector3::new(1.0, 0.0, 0.0)
        };
        let base2 = base1.cross(&dir).normalize();
        let n = Matrix3::from_columns(&[base1, base2, Vector3::zeros()]);
        let n = n.fixed_view::<3, 2>(0, 0).into_owned();

        let dir_hat = firefly_void_types::so3::skew(&dir);
        let a = r * dir_hat * n;

        let ang_var = self.angle_sigma * self.angle_sigma;
        let dir_var = Matrix2::new(ang_var, 0.0, 0.0, ang_var);

        dir * range_var * dir.transpose() + a * dir_var * a.transpose()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::{DepthOptions, sim_depth_noise};

    #[test]
    fn range_sigma_grows_with_z_squared() {
        // σ_z ∝ z²（视差域主导；近距空洞项小，z 大时二次项占优）
        let noise = DepthNoise::new(&DepthOptions::default(), 0.001);
        let s1 = noise.range_sigma(1.0);
        let s2 = noise.range_sigma(2.0);
        let s4 = noise.range_sigma(4.0);
        // 纯 z² 时 s4/s2 = 4，空洞项使比值略低；此处断言增长趋势
        assert!(s2 > s1 * 2.0, "σ(z=2)={s2} 应显著大于 σ(z=1)={s1}");
        assert!(s4 > s2 * 2.5, "σ(z=4)={s4} 应显著大于 σ(z=2)={s2}");
        // 数值核对：z=1 时视差项 σ_disp/(f·B) = 0.08/8.43 ≈ 0.0095 m，
        // 空洞项 0.02 m 叠加后 σ 应略大于视差项且 < 0.03
        let expect = sim_depth_noise::SIGMA_DISP_COEFF * sim_depth_noise::DEPTH_NOISE
            / (sim_depth_noise::DEPTH_FOCAL * sim_depth_noise::BASELINE);
        assert!(
            s1 > expect && s1 < 0.03,
            "z=1 处 σ={s1} 应介于视差项 {expect} 与 0.03 之间"
        );
    }

    #[test]
    fn point_covariance_grows_with_depth() {
        // Σ_pj 随 z 增长：远距点协方差特征值更大
        let noise = DepthNoise::new(&DepthOptions::default(), 0.001);
        let c1 = noise.point_covariance(&Vector3::new(0.1, 0.1, 1.0));
        let c3 = noise.point_covariance(&Vector3::new(0.1, 0.1, 3.0));
        let trace1 = c1.trace();
        let trace3 = c3.trace();
        assert!(
            trace3 > trace1 * 5.0,
            "tr(Σ_3m)={trace3} 应大于 tr(Σ_1m)={trace1}"
        );
        // 对称正定
        assert!((c1 - c1.transpose()).norm() < 1e-12);
        assert!((c3 - c3.transpose()).norm() < 1e-12);
        for (i, j) in [(0, 0), (1, 1), (2, 2)] {
            let _ = (i, j);
        }
        assert!(c1.determinant() > 0.0);
        assert!(c3.determinant() > 0.0);
    }

    #[test]
    fn from_intrinsics_pixel_angle() {
        // f=300：1/2 像素角 ≈ arctan(1/600) ≈ 0.00167 rad
        let noise = DepthNoise::from_intrinsics(&DepthOptions::default(), 300.0, 300.0);
        let expect: f64 = 0.5 * ((0.5_f64 / 300.0).atan() + (0.5_f64 / 300.0).atan());
        let p = Vector3::new(0.0, 0.0, 1.0);
        let c = noise.point_covariance(&p);
        // 正前方点的切向方差 ≈ r²·σ_ω²
        let tangential = 0.5 * (c[(0, 0)] + c[(1, 1)]);
        assert!(
            (tangential.sqrt() - expect).abs() < 1e-3,
            "切向 σ 应≈像素角"
        );
    }
}
