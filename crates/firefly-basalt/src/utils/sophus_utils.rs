// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// Copyright 2011-2017 Hauke Strasdat, 2012-2017 Steven Lovegrove.
// SPDX-License-Identifier: BSD-3-Clause AND MIT AND MPL-2.0
// License notices and conditions are indexed in docs/licenses/README.md.
//! Basalt `utils/sophus_utils.hpp` at aa441ba3e51050c47ba1902537792a2e4db7e43d.
//! SO(3) exp/log follow Sophus d0b7315a0d90fc6143defa54596a3a95d9fa10ec.
//! Tangent vectors place translation before rotation, as in the reference.

use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{
    Isometry3, Matrix3, Quaternion, SMatrix, SVector, Translation3, UnitQuaternion, Vector3,
};

// Eigen QuaternionBase::toRotationMatrix, MPL-2.0; see the license index.
// Copyright (C) 2008-2010 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2009 Mathieu Gautier <mathieu.gautier@cea.fr>
/// Rotation matrix using the pinned Eigen/Sophus quaternion evaluation order.
#[must_use]
pub fn so3_matrix<S: BasaltScalar>(rotation: &UnitQuaternion<S>) -> Matrix3<S> {
    let q = rotation.quaternion();
    let two = S::from_f64_lossy(2.);
    let tx = two * q.i;
    let ty = two * q.j;
    let tz = two * q.k;
    let twx = tx * q.w;
    let twy = ty * q.w;
    let twz = tz * q.w;
    let txx = tx * q.i;
    let txy = ty * q.i;
    let txz = tz * q.i;
    let tyy = ty * q.j;
    let tyz = tz * q.j;
    let tzz = tz * q.k;
    Matrix3::new(
        S::one() - (tyy + tzz),
        txy - twz,
        txz + twy,
        txy + twz,
        S::one() - (txx + tzz),
        tyz - twx,
        txz - twy,
        tyz + twx,
        S::one() - (txx + tyy),
    )
}

/// Homogeneous matrix using the pinned Sophus rotation conversion.
#[must_use]
pub fn se3_matrix<S: BasaltScalar>(pose: &Isometry3<S>) -> SMatrix<S, 4, 4> {
    let mut result = SMatrix::<S, 4, 4>::identity();
    result
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&so3_matrix(&pose.rotation));
    result
        .fixed_view_mut::<3, 1>(0, 3)
        .copy_from(&pose.translation.vector);
    result
}

/// Quaternion constructor used by Sophus SO(3), including normalization.
/// # Panics
/// Panics if the quaternion norm is below the scalar's Sophus epsilon.
#[must_use]
pub fn so3_from_quaternion<S: BasaltScalar>(quaternion: Quaternion<S>) -> UnitQuaternion<S> {
    // Eigen's four-coefficient packet reduction in [x,y,z,w] order.
    let norm = ((quaternion.i * quaternion.i + quaternion.k * quaternion.k)
        + (quaternion.j * quaternion.j + quaternion.w * quaternion.w))
        .sqrt();
    assert!(norm >= S::epsilon());
    UnitQuaternion::new_unchecked(quaternion / norm)
}

/// Sophus quaternion product, normalized by its SO(3) constructor.
/// # Panics
/// Panics if the product violates the quaternion constructor's norm constraint.
#[must_use]
pub fn so3_product<S: BasaltScalar>(
    left: &UnitQuaternion<S>,
    right: &UnitQuaternion<S>,
) -> UnitQuaternion<S> {
    let a = left.quaternion();
    let b = right.quaternion();
    so3_from_quaternion(Quaternion::new(
        a.w * b.w - a.i * b.i - a.j * b.j - a.k * b.k,
        a.w * b.i + a.i * b.w + a.j * b.k - a.k * b.j,
        a.w * b.j + a.j * b.w + a.k * b.i - a.i * b.k,
        a.w * b.k + a.k * b.w + a.i * b.j - a.j * b.i,
    ))
}

/// Sophus inverse: construct and normalize the conjugate quaternion.
/// # Panics
/// Panics if the quaternion violates the constructor's norm constraint.
#[must_use]
pub fn so3_inverse<S: BasaltScalar>(rotation: &UnitQuaternion<S>) -> UnitQuaternion<S> {
    so3_from_quaternion(rotation.quaternion().conjugate())
}

/// Sophus SE(3) composition, including the SO(3) product normalization.
/// # Panics
/// Requires quaternions satisfying the Sophus constructor's norm constraint.
#[must_use]
pub fn se3_product<S: BasaltScalar>(left: &Isometry3<S>, right: &Isometry3<S>) -> Isometry3<S> {
    Isometry3::from_parts(
        Translation3::from(left.translation.vector + left.rotation * right.translation.vector),
        so3_product(&left.rotation, &right.rotation),
    )
}

/// Sophus SE(3) inverse, using the normalized inverse rotation for translation.
/// # Panics
/// Requires a quaternion satisfying the Sophus constructor's norm constraint.
#[must_use]
pub fn se3_inverse<S: BasaltScalar>(pose: &Isometry3<S>) -> Isometry3<S> {
    let rotation = so3_inverse(&pose.rotation);
    Isometry3::from_parts(
        Translation3::from(rotation * -pose.translation.vector),
        rotation,
    )
}

/// Same-scalar copies retain coefficients; cross-scalar casts renormalize.
/// # Panics
/// Panics if conversion yields a quaternion below the target Sophus epsilon.
#[must_use]
pub fn so3_cast<S: BasaltScalar, T: BasaltScalar>(
    rotation: &UnitQuaternion<S>,
) -> UnitQuaternion<T> {
    let quaternion = Quaternion::from_vector(
        rotation
            .quaternion()
            .coords
            .map(|v| T::from_f64_lossy(v.as_f64())),
    );
    if size_of::<S>() == size_of::<T>() {
        UnitQuaternion::new_unchecked(quaternion)
    } else {
        so3_from_quaternion(quaternion)
    }
}

#[must_use]
pub fn hat<S: BasaltScalar>(vector: &Vector3<S>) -> Matrix3<S> {
    Matrix3::new(
        S::zero(),
        -vector.z,
        vector.y,
        vector.z,
        S::zero(),
        -vector.x,
        -vector.y,
        vector.x,
        S::zero(),
    )
}

/// Exponential using the reference small-angle series and scalar thresholds.
/// # Panics
/// Panics if the result violates the upstream unit-quaternion invariant.
#[must_use]
pub fn so3_exp<S: BasaltScalar>(omega: &Vector3<S>) -> UnitQuaternion<S> {
    let squared_angle = omega.norm_squared();
    let (imaginary, real) = if squared_angle < S::epsilon() * S::epsilon() {
        let fourth_power = squared_angle * squared_angle;
        (
            S::from_f64_lossy(0.5) - S::from_f64_lossy(1.0 / 48.0) * squared_angle
                + S::from_f64_lossy(1.0 / 3840.0) * fourth_power,
            S::one() - S::from_f64_lossy(1.0 / 8.0) * squared_angle
                + S::from_f64_lossy(1.0 / 384.0) * fourth_power,
        )
    } else {
        let angle = squared_angle.sqrt();
        let half_angle = S::from_f64_lossy(0.5) * angle;
        let (sin_half, cos_half) = if std::mem::size_of::<S>() == std::mem::size_of::<f32>() {
            // Platform f32 libm differences are amplified by high spline derivatives.
            let half_angle_f64 = half_angle.as_f64();
            (
                S::from_f64_lossy(half_angle_f64.sin()),
                S::from_f64_lossy(half_angle_f64.cos()),
            )
        } else {
            (half_angle.sin(), half_angle.cos())
        };
        (sin_half / angle, cos_half)
    };
    let quaternion = Quaternion::new(
        real,
        imaginary * omega.x,
        imaginary * omega.y,
        imaginary * omega.z,
    );
    assert!((quaternion.norm_squared() - S::one()).abs() < S::epsilon());
    UnitQuaternion::new_unchecked(quaternion)
}

/// Principal quaternion logarithm, including the reference negative-w branch.
/// # Panics
/// Panics if a near-zero imaginary component has an invalid real component.
#[must_use]
pub fn so3_log<S: BasaltScalar>(rotation: &UnitQuaternion<S>) -> Vector3<S> {
    let quaternion = rotation.quaternion();
    let squared_norm = quaternion.vector().norm_squared();
    let real = quaternion.w;
    let factor = if squared_norm < S::epsilon() * S::epsilon() {
        assert!(real.abs() >= S::epsilon());
        let squared_real = real * real;
        S::from_f64_lossy(2.0) / real
            - S::from_f64_lossy(2.0 / 3.0) * squared_norm / (real * squared_real)
    } else {
        let norm = squared_norm.sqrt();
        let angle = if std::mem::size_of::<S>() == std::mem::size_of::<f32>() {
            // Evaluate the branch in f64 and round once back to the requested scalar.
            let y = norm.as_f64();
            let x = real.as_f64();
            S::from_f64_lossy(if real < S::zero() {
                (-y).atan2(-x)
            } else {
                y.atan2(x)
            })
        } else if real < S::zero() {
            (-norm).atan2(-real)
        } else {
            norm.atan2(real)
        };
        S::from_f64_lossy(2.0) * angle / norm
    };
    quaternion.vector() * factor
}

#[must_use]
pub fn se3_logd<S: BasaltScalar>(pose: &Isometry3<S>) -> SVector<S, 6> {
    let mut result = SVector::<S, 6>::zeros();
    result
        .fixed_rows_mut::<3>(0)
        .copy_from(&pose.translation.vector);
    result
        .fixed_rows_mut::<3>(3)
        .copy_from(&so3_log(&pose.rotation));
    result
}

#[must_use]
pub fn se3_expd<S: BasaltScalar>(tangent: &SVector<S, 6>) -> Isometry3<S> {
    Isometry3::from_parts(
        Translation3::from(tangent.fixed_rows::<3>(0).into_owned()),
        so3_exp(&tangent.fixed_rows::<3>(3).into_owned()),
    )
}

#[must_use]
pub fn right_jacobian_so3<S: BasaltScalar>(phi: &Vector3<S>) -> Matrix3<S> {
    let squared_angle = phi.norm_squared();
    let skew = hat(phi);
    let skew_squared = skew * skew;
    let mut jacobian = Matrix3::identity();
    if squared_angle > S::epsilon() {
        let angle = squared_angle.sqrt();
        jacobian -= skew * (S::one() - angle.cos()) / squared_angle;
        jacobian += skew_squared * (angle - angle.sin()) / (squared_angle * angle);
    } else {
        jacobian -= skew / S::from_f64_lossy(2.0);
        jacobian += skew_squared / S::from_f64_lossy(6.0);
    }
    jacobian
}

#[must_use]
pub fn left_jacobian_so3<S: BasaltScalar>(phi: &Vector3<S>) -> Matrix3<S> {
    right_jacobian_so3(&(-phi))
}

/// Inverse right Jacobian with the upstream expansion around pi.
/// # Panics
/// Requires an angle at most pi plus the scalar's Sophus epsilon.
#[must_use]
pub fn right_jacobian_inv_so3<S: BasaltScalar>(phi: &Vector3<S>) -> Matrix3<S> {
    let squared_angle = phi.norm_squared();
    let skew = hat(phi);
    let skew_squared = skew * skew;
    let mut jacobian = Matrix3::identity() + skew / S::from_f64_lossy(2.0);
    if squared_angle > S::epsilon() {
        let angle = squared_angle.sqrt();
        // M_PI promotes the C++ comparisons to double.
        assert!(angle.as_f64() <= std::f64::consts::PI + S::epsilon().as_f64());
        if angle.as_f64() < std::f64::consts::PI - S::epsilon_sqrt().as_f64() {
            jacobian += skew_squared
                * (S::one() / squared_angle
                    - (S::one() + angle.cos()) / (S::from_f64_lossy(2.0) * angle * angle.sin()));
        } else {
            jacobian +=
                skew_squared / S::from_f64_lossy(std::f64::consts::PI * std::f64::consts::PI);
        }
    } else {
        jacobian += skew_squared / S::from_f64_lossy(12.0);
    }
    jacobian
}

/// # Panics
/// Requires the same angle range as [`right_jacobian_inv_so3`].
#[must_use]
pub fn left_jacobian_inv_so3<S: BasaltScalar>(phi: &Vector3<S>) -> Matrix3<S> {
    right_jacobian_inv_so3(&(-phi))
}

#[must_use]
pub fn right_jacobian_se3_decoupled<S: BasaltScalar>(phi: &SVector<S, 6>) -> SMatrix<S, 6, 6> {
    let omega = phi.fixed_rows::<3>(3).into_owned();
    let mut jacobian = SMatrix::<S, 6, 6>::zeros();
    jacobian
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&so3_matrix(&so3_inverse(&so3_exp(&omega))));
    jacobian
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&right_jacobian_so3(&omega));
    jacobian
}

/// # Panics
/// Requires the same angle range as [`right_jacobian_inv_so3`].
#[must_use]
pub fn right_jacobian_inv_se3_decoupled<S: BasaltScalar>(phi: &SVector<S, 6>) -> SMatrix<S, 6, 6> {
    let omega = phi.fixed_rows::<3>(3).into_owned();
    let mut jacobian = SMatrix::<S, 6, 6>::zeros();
    jacobian
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&so3_matrix(&so3_exp(&omega)));
    jacobian
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&right_jacobian_inv_so3(&omega));
    jacobian
}

/// Similarity represented by a scaled quaternion, as in Sophus RxSO3/Sim3.
/// The quaternion's squared norm is the positive similarity scale.
#[derive(Clone, Debug)]
pub struct Sim3<S: BasaltScalar> {
    scaled_rotation: Quaternion<S>,
    pub translation: Vector3<S>,
}

fn quaternion_squared_norm<S: BasaltScalar>(q: &Quaternion<S>) -> S {
    (q.i * q.i + q.k * q.k) + (q.j * q.j + q.w * q.w)
}

fn scaled_rotation_matrix<S: BasaltScalar>(q: &Quaternion<S>) -> Matrix3<S> {
    let x2 = q.i * q.i;
    let y2 = q.j * q.j;
    let z2 = q.k * q.k;
    let w2 = q.w * q.w;
    let two = S::from_f64_lossy(2.0);
    let xy = two * q.i * q.j;
    let xz = two * q.i * q.k;
    let xw = two * q.i * q.w;
    let yz = two * q.j * q.k;
    let yw = two * q.j * q.w;
    let zw = two * q.k * q.w;
    Matrix3::new(
        x2 - y2 - z2 + w2,
        xy - zw,
        xz + yw,
        xy + zw,
        -x2 + y2 - z2 + w2,
        -xw + yz,
        xz - yw,
        xw + yz,
        -x2 - y2 + z2 + w2,
    )
}

impl<S: BasaltScalar> Sim3<S> {
    /// Cereal loads coefficients directly without validating the similarity scale.
    pub(crate) fn from_scaled_quaternion_unchecked(
        rotation: Quaternion<S>,
        translation: Vector3<S>,
    ) -> Self {
        Self {
            scaled_rotation: rotation,
            translation,
        }
    }
    /// # Panics
    /// Requires the Sophus scale interval `[epsilon, 1/epsilon)`.
    #[must_use]
    pub fn from_scaled_quaternion(rotation: Quaternion<S>, translation: Vector3<S>) -> Self {
        let scale = quaternion_squared_norm(&rotation);
        assert!(scale >= S::epsilon() && scale < S::one() / S::epsilon());
        Self {
            scaled_rotation: rotation,
            translation,
        }
    }

    #[must_use]
    pub fn scaled_rotation(&self) -> &Quaternion<S> {
        &self.scaled_rotation
    }

    #[must_use]
    pub fn matrix(&self) -> SMatrix<S, 4, 4> {
        let mut matrix = SMatrix::<S, 4, 4>::identity();
        matrix
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&scaled_rotation_matrix(&self.scaled_rotation));
        matrix
            .fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&self.translation);
        matrix
    }
}

/// Decoupled similarity exponential, including Sophus scale saturation.
#[must_use]
pub fn sim3_expd<S: BasaltScalar>(tangent: &SVector<S, 7>) -> Sim3<S> {
    let epsilon_plus = S::epsilon() * (S::one() + S::epsilon());
    let scale = tangent[6]
        .exp()
        .max(epsilon_plus)
        .min(S::one() / epsilon_plus);
    let rotation = so3_exp(&tangent.fixed_rows::<3>(3).into_owned());
    Sim3::from_scaled_quaternion(
        *rotation.quaternion() * scale.sqrt(),
        tangent.fixed_rows::<3>(0).into_owned(),
    )
}

#[must_use]
pub fn sim3_logd<S: BasaltScalar>(pose: &Sim3<S>) -> SVector<S, 7> {
    let mut tangent = SVector::<S, 7>::zeros();
    tangent.fixed_rows_mut::<3>(0).copy_from(&pose.translation);
    tangent
        .fixed_rows_mut::<3>(3)
        .copy_from(&so3_log(&so3_from_quaternion(pose.scaled_rotation)));
    tangent[6] = quaternion_squared_norm(&pose.scaled_rotation).ln();
    tangent
}

#[must_use]
pub fn right_jacobian_sim3_decoupled<S: BasaltScalar>(phi: &SVector<S, 7>) -> SMatrix<S, 7, 7> {
    let pose = sim3_expd(phi);
    let inverse = pose.scaled_rotation.conjugate() / quaternion_squared_norm(&pose.scaled_rotation);
    let mut jacobian = SMatrix::<S, 7, 7>::zeros();
    jacobian
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&scaled_rotation_matrix(&inverse));
    jacobian
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&right_jacobian_so3(&phi.fixed_rows::<3>(3).into_owned()));
    jacobian[(6, 6)] = S::one();
    jacobian
}

/// # Panics
/// Requires the same rotation range as [`right_jacobian_inv_so3`].
#[must_use]
pub fn right_jacobian_inv_sim3_decoupled<S: BasaltScalar>(phi: &SVector<S, 7>) -> SMatrix<S, 7, 7> {
    let pose = sim3_expd(phi);
    let mut jacobian = SMatrix::<S, 7, 7>::zeros();
    jacobian
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&scaled_rotation_matrix(&pose.scaled_rotation));
    jacobian
        .fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&right_jacobian_inv_so3(
            &phi.fixed_rows::<3>(3).into_owned(),
        ));
    jacobian[(6, 6)] = S::one();
    jacobian
}
