//! 估计变量类型（对照 `OpenVINS` `ov_core/src/types`）。
//!
//! 每个变量有当前估计 `value` 与首估计 `fej`（FEJ），
//! `update(dx)` 实现误差状态 boxplus：
//! - 四元数：`q ← normalize([½dx; 1]) ⊗ q`；
//! - 向量：`v ← v + dx`。

use nalgebra::{DVector, Vector3, Vector4};

use crate::quat_ops::{quat_multiply, quatnorm};

/// 变量公共接口（对照 `Type` 基类）。
pub trait Variable {
    /// 误差状态维度（协方差中的大小）。
    fn size(&self) -> usize;
    /// 在协方差中的位置（-1 = 不在协方差中）。
    fn id(&self) -> i32;
    /// 设置协方差位置（级联设置子变量）。
    fn set_local_id(&mut self, id: i32);
    /// 误差状态 boxplus 更新。
    fn update(&mut self, dx: &DVector<f64>);
    /// 当前估计。
    fn value(&self) -> DVector<f64>;
    /// 首估计（FEJ）。
    fn fej(&self) -> DVector<f64>;
}

/// 四元数变量（JPL 惯例，误差 3 DOF）。
#[derive(Debug, Clone)]
pub struct JplQuat {
    value: Vector4<f64>,
    fej: Vector4<f64>,
    id: i32,
}

impl Default for JplQuat {
    fn default() -> Self {
        Self {
            value: Vector4::new(0.0, 0.0, 0.0, 1.0),
            fej: Vector4::new(0.0, 0.0, 0.0, 1.0),
            id: -1,
        }
    }
}

impl JplQuat {
    #[must_use]
    pub fn quat(&self) -> Vector4<f64> {
        self.value
    }

    #[must_use]
    pub fn quat_fej(&self) -> Vector4<f64> {
        self.fej
    }

    #[must_use]
    pub fn rot(&self) -> nalgebra::Matrix3<f64> {
        crate::quat_ops::quat_2_rot(&self.value)
    }

    #[must_use]
    pub fn rot_fej(&self) -> nalgebra::Matrix3<f64> {
        crate::quat_ops::quat_2_rot(&self.fej)
    }

    pub fn set_value(&mut self, q: Vector4<f64>) {
        self.value = q;
    }

    pub fn set_fej(&mut self, q: Vector4<f64>) {
        self.fej = q;
    }
}

impl Variable for JplQuat {
    fn size(&self) -> usize {
        3
    }

    fn id(&self) -> i32 {
        self.id
    }

    fn set_local_id(&mut self, id: i32) {
        self.id = id;
    }

    fn update(&mut self, dx: &DVector<f64>) {
        debug_assert_eq!(dx.len(), 3);
        let dq = quatnorm(Vector4::new(0.5 * dx[0], 0.5 * dx[1], 0.5 * dx[2], 1.0));
        self.value = quat_multiply(&dq, &self.value);
    }

    fn value(&self) -> DVector<f64> {
        DVector::from_column_slice(self.value.as_slice())
    }

    fn fej(&self) -> DVector<f64> {
        DVector::from_column_slice(self.fej.as_slice())
    }
}

/// 向量变量（误差 = 维度，加法更新）。
#[derive(Debug, Clone)]
pub struct VecVar {
    value: DVector<f64>,
    fej: DVector<f64>,
    id: i32,
    dim: usize,
}

impl VecVar {
    #[must_use]
    pub fn new(dim: usize) -> Self {
        Self {
            value: DVector::zeros(dim),
            fej: DVector::zeros(dim),
            id: -1,
            dim,
        }
    }

    #[must_use]
    pub fn vec(&self) -> DVector<f64> {
        self.value.clone()
    }

    pub fn set_value(&mut self, v: DVector<f64>) {
        self.value = v;
    }

    pub fn set_fej(&mut self, v: DVector<f64>) {
        self.fej = v;
    }
}

impl Variable for VecVar {
    fn size(&self) -> usize {
        self.dim
    }

    fn id(&self) -> i32 {
        self.id
    }

    fn set_local_id(&mut self, id: i32) {
        self.id = id;
    }

    fn update(&mut self, dx: &DVector<f64>) {
        debug_assert_eq!(dx.len(), self.dim);
        self.value += dx;
    }

    fn value(&self) -> DVector<f64> {
        self.value.clone()
    }

    fn fej(&self) -> DVector<f64> {
        self.fej.clone()
    }
}

/// 位姿变量（四元数 + 平移，误差 6 DOF）。
#[derive(Debug, Clone)]
pub struct PoseJpl {
    q: JplQuat,
    p: VecVar,
    id: i32,
}

impl Default for PoseJpl {
    fn default() -> Self {
        Self {
            q: JplQuat::default(),
            p: VecVar::new(3),
            id: -1,
        }
    }
}

impl PoseJpl {
    #[must_use]
    pub fn q(&self) -> &JplQuat {
        &self.q
    }

    #[must_use]
    pub fn p(&self) -> &VecVar {
        &self.p
    }

    #[must_use]
    pub fn quat(&self) -> Vector4<f64> {
        self.q.quat()
    }

    #[must_use]
    pub fn quat_fej(&self) -> Vector4<f64> {
        self.q.quat_fej()
    }

    #[must_use]
    pub fn pos(&self) -> Vector3<f64> {
        Vector3::new(self.p.value[0], self.p.value[1], self.p.value[2])
    }

    #[must_use]
    pub fn pos_fej(&self) -> Vector3<f64> {
        Vector3::new(self.p.fej[0], self.p.fej[1], self.p.fej[2])
    }

    #[must_use]
    pub fn rot(&self) -> nalgebra::Matrix3<f64> {
        self.q.rot()
    }

    #[must_use]
    pub fn rot_fej(&self) -> nalgebra::Matrix3<f64> {
        self.q.rot_fej()
    }
}

impl Variable for PoseJpl {
    fn size(&self) -> usize {
        6
    }

    fn id(&self) -> i32 {
        self.id
    }

    fn set_local_id(&mut self, id: i32) {
        self.id = id;
        self.q.set_local_id(id);
        self.p.set_local_id(id + if id >= 0 { 3 } else { 0 });
    }

    fn update(&mut self, dx: &DVector<f64>) {
        debug_assert_eq!(dx.len(), 6);
        self.q.update(&dx.rows_range(0..3).into_owned());
        self.p.update(&dx.rows_range(3..6).into_owned());
    }

    fn value(&self) -> DVector<f64> {
        let mut v = DVector::zeros(7);
        v.rows_range_mut(0..4).copy_from(&self.q.quat());
        v.rows_range_mut(4..7).copy_from(&self.p.value);
        v
    }

    fn fej(&self) -> DVector<f64> {
        let mut v = DVector::zeros(7);
        v.rows_range_mut(0..4).copy_from(&self.q.quat_fej());
        v.rows_range_mut(4..7).copy_from(&self.p.fej);
        v
    }
}

/// IMU 复合状态（位姿 + 速度 + 陀螺偏置 + 加速度计偏置，误差 15 DOF）。
#[derive(Debug, Clone)]
pub struct ImuState {
    pose: PoseJpl,
    v: VecVar,
    bg: VecVar,
    ba: VecVar,
    id: i32,
}

impl Default for ImuState {
    fn default() -> Self {
        Self {
            pose: PoseJpl::default(),
            v: VecVar::new(3),
            bg: VecVar::new(3),
            ba: VecVar::new(3),
            id: -1,
        }
    }
}

impl ImuState {
    #[must_use]
    pub fn pose(&self) -> &PoseJpl {
        &self.pose
    }

    #[must_use]
    pub fn v(&self) -> &VecVar {
        &self.v
    }

    #[must_use]
    pub fn bg(&self) -> &VecVar {
        &self.bg
    }

    #[must_use]
    pub fn ba(&self) -> &VecVar {
        &self.ba
    }

    #[must_use]
    pub fn quat(&self) -> Vector4<f64> {
        self.pose.quat()
    }

    #[must_use]
    pub fn pos(&self) -> Vector3<f64> {
        self.pose.pos()
    }

    #[must_use]
    pub fn vel(&self) -> Vector3<f64> {
        Vector3::new(self.v.value[0], self.v.value[1], self.v.value[2])
    }

    #[must_use]
    pub fn bias_g(&self) -> Vector3<f64> {
        Vector3::new(self.bg.value[0], self.bg.value[1], self.bg.value[2])
    }

    #[must_use]
    pub fn bias_a(&self) -> Vector3<f64> {
        Vector3::new(self.ba.value[0], self.ba.value[1], self.ba.value[2])
    }
}

impl Variable for ImuState {
    fn size(&self) -> usize {
        15
    }

    fn id(&self) -> i32 {
        self.id
    }

    fn set_local_id(&mut self, id: i32) {
        self.id = id;
        self.pose.set_local_id(id);
        self.v
            .set_local_id(self.pose.id() + if id >= 0 { 6 } else { 0 });
        self.bg
            .set_local_id(self.v.id() + if id >= 0 { 3 } else { 0 });
        self.ba
            .set_local_id(self.bg.id() + if id >= 0 { 3 } else { 0 });
    }

    fn update(&mut self, dx: &DVector<f64>) {
        debug_assert_eq!(dx.len(), 15);
        self.pose.update(&dx.rows_range(0..6).into_owned());
        self.v.update(&dx.rows_range(6..9).into_owned());
        self.bg.update(&dx.rows_range(9..12).into_owned());
        self.ba.update(&dx.rows_range(12..15).into_owned());
    }

    fn value(&self) -> DVector<f64> {
        let mut v = DVector::zeros(16);
        v.rows_range_mut(0..4).copy_from(&self.pose.quat());
        v.rows_range_mut(4..7).copy_from(&self.pose.p().value);
        v.rows_range_mut(7..10).copy_from(&self.v.value);
        v.rows_range_mut(10..13).copy_from(&self.bg.value);
        v.rows_range_mut(13..16).copy_from(&self.ba.value);
        v
    }

    fn fej(&self) -> DVector<f64> {
        let mut v = DVector::zeros(16);
        v.rows_range_mut(0..4).copy_from(&self.pose.quat_fej());
        v.rows_range_mut(4..7).copy_from(&self.pose.p().fej);
        v.rows_range_mut(7..10).copy_from(&self.v.fej);
        v.rows_range_mut(10..13).copy_from(&self.bg.fej);
        v.rows_range_mut(13..16).copy_from(&self.ba.fej);
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quat_update_rotates() {
        let mut q = JplQuat::default();
        // dx = [0, 0, 0.5] → dq = quatnorm([0, 0, 0.25, 1])
        // JPL 惯例正角度旋转方向与 Hamilton 相反 → 等价 rot_z(-θ)，θ = 2·atan(0.25)
        q.update(&DVector::from_vec(vec![0.0, 0.0, 0.5]));
        let theta = 2.0 * 0.25_f64.atan();
        let expected = crate::quat_ops::rot_z(-theta);
        let rot = q.rot();
        for i in 0..3 {
            for j in 0..3 {
                assert!((rot[(i, j)] - expected[(i, j)]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn quat_update_is_unit() {
        let mut q = JplQuat::default();
        for _ in 0..10 {
            q.update(&DVector::from_vec(vec![0.1, -0.2, 0.05]));
        }
        assert!((q.quat().norm() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn vec_update_adds() {
        let mut v = VecVar::new(3);
        v.update(&DVector::from_vec(vec![1.0, 2.0, 3.0]));
        v.update(&DVector::from_vec(vec![0.5, -1.0, 0.0]));
        assert_eq!(v.vec(), DVector::from_vec(vec![1.5, 1.0, 3.0]));
    }

    #[test]
    fn pose_update() {
        let mut p = PoseJpl::default();
        p.update(&DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 2.0, 3.0]));
        assert_eq!(p.pos(), Vector3::new(1.0, 2.0, 3.0));
        assert!((p.quat().norm() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn imu_id_layout() {
        let mut imu = ImuState::default();
        imu.set_local_id(10);
        assert_eq!(imu.id(), 10);
        assert_eq!(imu.pose().id(), 10);
        assert_eq!(imu.v().id(), 16);
        assert_eq!(imu.bg().id(), 19);
        assert_eq!(imu.ba().id(), 22);
        // 总误差大小 = pose 6 + v 3 + bg 3 + ba 3
        assert_eq!(imu.size(), 15);
    }

    #[test]
    fn imu_update_layout() {
        let mut imu = ImuState::default();
        let dx = DVector::from_vec(vec![
            0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.01, 0.02, 0.03, 0.001, 0.002, 0.003,
        ]);
        imu.update(&dx);
        assert_eq!(imu.pos(), Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(imu.vel(), Vector3::new(0.1, 0.2, 0.3));
        assert_eq!(imu.bias_g(), Vector3::new(0.01, 0.02, 0.03));
        assert_eq!(imu.bias_a(), Vector3::new(0.001, 0.002, 0.003));
    }
}
