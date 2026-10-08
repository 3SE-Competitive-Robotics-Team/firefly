use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{
    Isometry3, Matrix3, Matrix4, Matrix6, Point3, Quaternion, Translation3, UnitQuaternion, Vector3,
};
use std::collections::BTreeMap;

/// 坐标系标识。0..1024 保留，自定义传感器使用 1024 及以上编号。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct FrameId(pub u32);
impl FrameId {
    pub const MAP: Self = Self(0);
    pub const ODOM: Self = Self(1);
    pub const BODY: Self = Self(2);
    pub const LEFT_CAMERA: Self = Self(3);
    pub const RIGHT_CAMERA: Self = Self(4);
    pub const DEPTH_CAMERA: Self = Self(5);
}

/// `T_target_source`；列向量约定，`p_target = R p_source + t`。
/// 四元数使用 Hamilton，xyzw 描述 source→target 的主动旋转。
#[derive(Debug, Clone, Copy)]
pub struct RigidTransform {
    target: FrameId,
    source: FrameId,
    pose: Isometry3<f64>,
}
impl RigidTransform {
    /// 创建刚体变换。
    /// # Errors
    /// 非有限值、非单位四元数，或同坐标系非恒等变换。
    pub fn from_parts(
        target: FrameId,
        source: FrameId,
        translation: [f64; 3],
        xyzw: [f64; 4],
    ) -> Result<Self> {
        if !translation.iter().chain(xyzw.iter()).all(|x| x.is_finite()) {
            return Err(invalid("non-finite transform"));
        }
        let q = Quaternion::new(xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
        if (q.norm() - 1.).abs() > 1e-6 {
            return Err(invalid("transform requires unit quaternion"));
        }
        let pose = Isometry3::from_parts(
            Translation3::from(Vector3::from(translation)),
            UnitQuaternion::new_normalize(q),
        );
        if target == source
            && (pose.translation.vector.norm() > 1e-12 || pose.rotation.angle() > 1e-12)
        {
            return Err(invalid("same-frame transform must be identity"));
        }
        Ok(Self {
            target,
            source,
            pose,
        })
    }
    /// 验证并转换 SE(3) 矩阵。
    /// # Errors
    /// 缩放、反射、非有限值、非法齐次行或同坐标系非恒等变换。
    pub fn from_matrix(target: FrameId, source: FrameId, matrix: &Matrix4<f64>) -> Result<Self> {
        let r = matrix.fixed_view::<3, 3>(0, 0).into_owned();
        if !matrix.iter().all(|x| x.is_finite())
            || (r.transpose() * r - Matrix3::identity()).norm() > 1e-6
            || (r.determinant() - 1.).abs() > 1e-6
            || matrix[(3, 0)].abs()
                + matrix[(3, 1)].abs()
                + matrix[(3, 2)].abs()
                + (matrix[(3, 3)] - 1.).abs()
                > 1e-12
        {
            return Err(invalid("matrix is not SE(3)"));
        }
        let rotation = UnitQuaternion::from_matrix(&r);
        let q = rotation.quaternion();
        Self::from_parts(
            target,
            source,
            [matrix[(0, 3)], matrix[(1, 3)], matrix[(2, 3)]],
            [q.i, q.j, q.k, q.w],
        )
    }
    #[must_use]
    pub fn identity(frame: FrameId) -> Self {
        Self {
            target: frame,
            source: frame,
            pose: Isometry3::identity(),
        }
    }
    #[must_use]
    pub const fn target(&self) -> FrameId {
        self.target
    }
    #[must_use]
    pub const fn source(&self) -> FrameId {
        self.source
    }
    /// 由已合成的 `Isometry` 构造，并使用与 [`RigidTransform::from_parts`] 相同的
    /// 校验（有限值、单位四元数、同系恒等）。
    fn from_isometry(target: FrameId, source: FrameId, pose: Isometry3<f64>) -> Result<Self> {
        let q = pose.rotation.quaternion();
        Self::from_parts(
            target,
            source,
            pose.translation.vector.into(),
            [q.i, q.j, q.k, q.w],
        )
    }
    #[must_use]
    pub fn matrix(&self) -> Matrix4<f64> {
        self.pose.to_homogeneous()
    }
    #[must_use]
    pub const fn isometry(&self) -> &Isometry3<f64> {
        &self.pose
    }
    #[must_use]
    pub fn inverse(&self) -> Self {
        Self {
            target: self.source,
            source: self.target,
            pose: self.pose.inverse(),
        }
    }
    /// `T_a_b.compose(T_b_c) = T_a_c`。
    /// # Errors
    /// 中间坐标系不匹配、结果非有限或闭合链不为恒等变换。
    pub fn compose(&self, rhs: &Self) -> Result<Self> {
        if self.source != rhs.target {
            return Err(invalid("transform composition frame mismatch"));
        }
        Self::from_isometry(self.target, rhs.source, self.pose * rhs.pose)
    }
    #[must_use]
    pub fn point(&self, p: Point3<f64>) -> Point3<f64> {
        self.pose.transform_point(&p)
    }
    /// 同一物理向量换基；不包含时变参考系的输运速度。
    #[must_use]
    pub fn vector(&self, v: Vector3<f64>) -> Vector3<f64> {
        self.pose.transform_vector(&v)
    }
    /// SE(3) 伴随矩阵，扭量顺序 `[rotation, translation]`。
    #[must_use]
    pub fn adjoint(&self) -> Matrix6<f64> {
        let r = self.pose.rotation.to_rotation_matrix();
        let mut a = Matrix6::zeros();
        a.fixed_view_mut::<3, 3>(0, 0).copy_from(r.matrix());
        a.fixed_view_mut::<3, 3>(3, 3).copy_from(r.matrix());
        a.fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(crate::se3::skew(&self.pose.translation.vector) * r.matrix()));
        a
    }
    /// 同一 SE(3) 扰动协方差换基：`Ad(T) P Ad(T)ᵀ`。
    #[must_use]
    pub fn tangent_covariance(&self, p: &Matrix6<f64>) -> Matrix6<f64> {
        let a = self.adjoint();
        a * p * a.transpose()
    }
}

/// 同一时刻的几何坐标树；每个 child 唯一 parent，允许不连通子树。
/// 调用方负责时间同步、插值和新鲜度；本类型不缓存时间历史。
#[derive(Debug, Default)]
pub struct FrameTree {
    parents: BTreeMap<FrameId, RigidTransform>,
}
impl FrameTree {
    /// 插入或更新 parent←child。
    /// # Errors
    /// 自环、闭环或修改已注册 child 的 parent。失败不修改树。
    #[fastrace::trace]
    pub fn set(&mut self, t: RigidTransform) -> Result<()> {
        if t.target == t.source {
            return Err(invalid("self edge in frame tree"));
        }
        if self
            .parents
            .get(&t.source)
            .is_some_and(|old| old.target != t.target)
        {
            return Err(invalid("child already has another parent"));
        }
        let mut ancestor = t.target;
        loop {
            if ancestor == t.source {
                return Err(invalid("cycle in frame tree"));
            }
            let Some(edge) = self.parents.get(&ancestor) else {
                break;
            };
            ancestor = edge.target;
        }
        self.parents.insert(t.source, t);
        Ok(())
    }
    /// 累乘到根。边均由 [`FrameTree::set`] 校验过（合法刚体变换），故乘积仍合法，
    /// 直接构造不再重复校验。
    fn to_root(&self, frame: FrameId) -> RigidTransform {
        let mut pose = Isometry3::identity();
        let mut target = frame;
        while let Some(edge) = self.parents.get(&target) {
            pose = edge.pose * pose;
            target = edge.target;
        }
        RigidTransform {
            target,
            source: frame,
            pose,
        }
    }
    /// 经公共祖先查询 target←source，支持反向查询。
    /// # Errors
    /// 坐标系未注册或不连通。
    #[fastrace::trace]
    pub fn lookup(&self, target: FrameId, source: FrameId) -> Result<RigidTransform> {
        for frame in [target, source] {
            if !self.parents.contains_key(&frame)
                && !self.parents.values().any(|edge| edge.target == frame)
            {
                return Err(Error::new(ErrorKind::NotFound, "unknown coordinate frame"));
            }
        }
        let a = self.to_root(target);
        let b = self.to_root(source);
        if a.target != b.target {
            return Err(Error::new(
                ErrorKind::NotFound,
                "disconnected coordinate frames",
            ));
        }
        // T_target_source = T_root_target⁻¹ · T_root_source（`inv_mul` 即 `self⁻¹·rhs`）。
        RigidTransform::from_isometry(target, source, a.pose.inv_mul(&b.pose))
    }
}
fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn composition_reverse_queries_and_tree_invariants() {
        let q = std::f64::consts::FRAC_PI_4;
        let map_odom = RigidTransform::from_parts(
            FrameId::MAP,
            FrameId::ODOM,
            [10., 0., 0.],
            [0., 0., q.sin(), q.cos()],
        )
        .unwrap();
        let odom_body = RigidTransform::from_parts(
            FrameId::ODOM,
            FrameId::BODY,
            [2., 0., 0.],
            [0., 0., 0., 1.],
        )
        .unwrap();
        let mut tree = FrameTree::default();
        tree.set(map_odom).unwrap();
        tree.set(odom_body).unwrap();
        let map_body = tree.lookup(FrameId::MAP, FrameId::BODY).unwrap();
        assert!((map_body.point(Point3::origin()) - Point3::new(10., 2., 0.)).norm() < 1e-12);
        assert!((map_body.vector(Vector3::x()) - Vector3::y()).norm() < 1e-12);
        let inverse = tree.lookup(FrameId::BODY, FrameId::MAP).unwrap();
        assert!(
            (inverse.compose(&map_body).unwrap().matrix() - Matrix4::identity()).norm() < 1e-12
        );
        assert!(odom_body.compose(&map_odom).is_err());
        assert!(tree.set(map_body).is_err());
        assert!(tree.set(inverse).is_err());
        assert!(tree.lookup(FrameId(1024), FrameId::BODY).is_err());
        tree.set(
            RigidTransform::from_parts(FrameId(1024), FrameId(1025), [0.; 3], [0., 0., 0., 1.])
                .unwrap(),
        )
        .unwrap();
        assert!(tree.lookup(FrameId(1024), FrameId::BODY).is_err());
        assert!(
            (tree.lookup(FrameId::MAP, FrameId::BODY).unwrap().matrix() - map_body.matrix()).norm()
                < 1e-12
        );
    }
    #[test]
    fn adjoint_matches_conjugation_and_covariance_roundtrip() {
        let mut xi = nalgebra::Vector6::new(0.2, -0.1, 0.3, 1., 2., -1.);
        let t = RigidTransform::from_matrix(FrameId::MAP, FrameId::ODOM, &crate::se3::se3_exp(&xi))
            .unwrap();
        xi *= 0.03;
        let lhs = t.matrix() * crate::se3::se3_exp(&xi) * t.inverse().matrix();
        assert!((lhs - crate::se3::se3_exp(&(t.adjoint() * xi))).norm() < 1e-10);
        let p = Matrix6::identity();
        assert!((t.inverse().tangent_covariance(&t.tangent_covariance(&p)) - p).norm() < 1e-10);
    }
    #[test]
    fn invalid_geometry_is_rejected() {
        let mut bad = Matrix4::identity();
        bad[(0, 0)] = 2.;
        assert!(RigidTransform::from_matrix(FrameId::MAP, FrameId::BODY, &bad).is_err());
        assert!(RigidTransform::from_parts(FrameId::MAP, FrameId::BODY, [0.; 3], [0.; 4]).is_err());
        bad = Matrix4::identity();
        bad[(0, 0)] = -1.;
        assert!(RigidTransform::from_matrix(FrameId::MAP, FrameId::BODY, &bad).is_err());
        bad[(0, 0)] = f64::NAN;
        assert!(RigidTransform::from_matrix(FrameId::MAP, FrameId::BODY, &bad).is_err());
    }
    #[test]
    fn inconsistent_closed_chain_is_rejected() {
        let a =
            RigidTransform::from_parts(FrameId::MAP, FrameId::ODOM, [1., 0., 0.], [0., 0., 0., 1.])
                .unwrap();
        let b = RigidTransform::from_parts(FrameId::ODOM, FrameId::MAP, [0.; 3], [0., 0., 0., 1.])
            .unwrap();
        assert!(a.compose(&b).is_err());
        assert!(a.compose(&a.inverse()).is_ok());
    }
    #[test]
    fn right_jacobian_matches_reset_finite_difference() {
        let x = nalgebra::Vector6::new(0.2, -0.1, 0.3, 1., 2., -1.);
        let inverse = crate::se3::se3_exp(&(-x));
        let j = crate::se3::right_jacobian(&x);
        for i in 0..6 {
            let mut delta = nalgebra::Vector6::zeros();
            delta[i] = 1e-6;
            let numerical = (crate::se3::se3_log(&(inverse * crate::se3::se3_exp(&(x + delta))))
                - crate::se3::se3_log(&(inverse * crate::se3::se3_exp(&(x - delta)))))
                / (2e-6);
            assert!((numerical - j.column(i)).norm() < 1e-7);
        }
    }
}
