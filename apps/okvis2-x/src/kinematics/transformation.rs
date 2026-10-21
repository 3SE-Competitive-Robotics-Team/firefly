/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 *
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 *
 * SPDX-License-Identifier: BSD-3-Clause, see LICENSE file for details
 */

//! Pose composition and perturbations corresponding to `implementation/Transformation.hpp`.

use crate::kinematics::operators::{cross_mx, oplus};
use nalgebra::{Isometry3, Matrix3, Matrix4, Quaternion, SMatrix, SVector, Vector3, Vector4};
use rand::RngExt;

pub type Transformation = TransformationT<true>;
pub type TransformationCacheless = TransformationT<false>;

#[must_use]
pub fn sinc(x: f64) -> f64 {
    if x.abs() > 1.0e-6 {
        x.sin() / x
    } else {
        let c2 = 1.0 / 6.0;
        let c4 = 1.0 / 120.0;
        let c6 = 1.0 / 5040.0;
        let x2 = x * x;
        let x4 = x2 * x2;
        let x6 = x2 * x2 * x2;
        1.0 - c2 * x2 + c4 * x4 - c6 * x6
    }
}
#[must_use]
pub fn delta_q(delta_alpha: &Vector3<f64>) -> Quaternion<f64> {
    let half_norm = 0.5 * delta_alpha.norm();
    let vector = sinc(half_norm) * 0.5 * delta_alpha;
    Quaternion::new(half_norm.cos(), vector[0], vector[1], vector[2])
}
/// Right SO(3) Jacobian, Forster et al. RSS 2015, equation (8).
#[must_use]
pub fn right_jacobian(phi_vector: &Vector3<f64>) -> Matrix3<f64> {
    let phi = phi_vector.norm();
    let phi_x = cross_mx(phi_vector);
    let phi_x2 = phi_x * phi_x;
    let mut result = Matrix3::identity();
    if phi < 1.0e-4 {
        result += -0.5 * phi_x + 1.0 / 6.0 * phi_x2;
    } else {
        let phi2 = phi * phi;
        let phi3 = phi2 * phi;
        result += -(1.0 - phi.cos()) / phi2 * phi_x + (phi - phi.sin()) / phi3 * phi_x2;
    }
    result
}

#[derive(Clone, Debug)]
pub struct TransformationT<const CACHE_ROTATION: bool> {
    parameters: SVector<f64, 7>,
    rotation_cache: Matrix3<f64>,
}
impl<const CACHE_ROTATION: bool> Default for TransformationT<CACHE_ROTATION> {
    fn default() -> Self {
        let mut parameters = SVector::<f64, 7>::zeros();
        parameters[6] = 1.0;
        Self {
            parameters,
            rotation_cache: Matrix3::identity(),
        }
    }
}
impl<const CACHE_ROTATION: bool> TransformationT<CACHE_ROTATION> {
    #[must_use]
    pub fn new(translation: Vector3<f64>, rotation: Quaternion<f64>) -> Self {
        let mut pose = Self::default();
        pose.set(translation, rotation);
        pose
    }
    #[must_use]
    pub fn from_matrix(matrix: &Matrix4<f64>) -> Self {
        debug_assert!(matrix[(3, 0)].abs() < 1.0e-12);
        debug_assert!(matrix[(3, 1)].abs() < 1.0e-12);
        debug_assert!(matrix[(3, 2)].abs() < 1.0e-12);
        debug_assert!((matrix[(3, 3)] - 1.0).abs() < 1.0e-12);
        let rotation_cache = matrix.fixed_view::<3, 3>(0, 0).into_owned();
        let rotation = quaternion_from_rotation_matrix(&rotation_cache);
        let mut parameters = SVector::<f64, 7>::zeros();
        parameters
            .fixed_rows_mut::<3>(0)
            .copy_from(&matrix.fixed_view::<3, 1>(0, 3));
        parameters
            .fixed_rows_mut::<4>(3)
            .copy_from(&rotation.coords);
        Self {
            parameters,
            rotation_cache,
        }
    }
    #[must_use]
    pub fn from_other<const OTHER_CACHE: bool>(other: &TransformationT<OTHER_CACHE>) -> Self {
        let mut pose = Self {
            parameters: other.parameters,
            rotation_cache: other.rotation_cache,
        };
        if CACHE_ROTATION && !OTHER_CACHE {
            pose.update_rotation_cache();
        }
        pose
    }
    pub fn set_coeffs(&mut self, coefficients: &SVector<f64, 7>) -> bool {
        self.parameters = *coefficients;
        if CACHE_ROTATION {
            self.update_rotation_cache();
        }
        true
    }
    pub fn set_parameters(&mut self, parameters: &SVector<f64, 7>) -> bool {
        self.set_coeffs(parameters)
    }
    #[must_use]
    pub fn coeffs(&self) -> &SVector<f64, 7> {
        &self.parameters
    }
    #[must_use]
    pub fn parameters(&self) -> &SVector<f64, 7> {
        &self.parameters
    }
    /// # Panics
    /// Panics if the required assertion fails: `!CACHE_ROTATION, "can't access writable parameters
    /// in cached mode."`.
    pub fn coeffs_mut(&mut self) -> &mut SVector<f64, 7> {
        assert!(
            !CACHE_ROTATION,
            "can't access writable parameters in cached mode."
        );
        &mut self.parameters
    }
    pub fn parameters_mut(&mut self) -> &mut SVector<f64, 7> {
        self.coeffs_mut()
    }
    #[must_use]
    pub fn translation(&self) -> Vector3<f64> {
        self.parameters.fixed_rows::<3>(0).into_owned()
    }
    #[must_use]
    pub fn quaternion(&self) -> Quaternion<f64> {
        Quaternion::new(
            self.parameters[6],
            self.parameters[3],
            self.parameters[4],
            self.parameters[5],
        )
    }
    #[must_use]
    pub fn rotation_matrix(&self) -> Matrix3<f64> {
        if CACHE_ROTATION {
            self.rotation_cache
        } else {
            raw_rotation_matrix(&self.quaternion())
        }
    }
    #[must_use]
    pub fn matrix(&self) -> Matrix4<f64> {
        let mut matrix = Matrix4::identity();
        matrix
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&self.rotation_matrix());
        matrix
            .fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&self.translation());
        matrix
    }
    #[must_use]
    pub fn matrix_3x4(&self) -> SMatrix<f64, 3, 4> {
        self.matrix().fixed_rows::<3>(0).into_owned()
    }
    #[must_use]
    pub fn inverse(&self) -> Self {
        Self::new(
            -(self.rotation_matrix().transpose() * self.translation()),
            quaternion_inverse(&self.quaternion()),
        )
    }
    pub fn set_random(&mut self) {
        self.set_random_with_bounds(1.0, std::f64::consts::PI);
    }
    pub fn set_random_with_bounds(
        &mut self,
        translation_max_meters: f64,
        rotation_max_radians: f64,
    ) {
        let mut rng = rand::rng();
        let axis = rotation_max_radians
            * Vector3::new(
                rng.random_range(-1.0..=1.0),
                rng.random_range(-1.0..=1.0),
                rng.random_range(-1.0..=1.0),
            );
        let translation = translation_max_meters
            * Vector3::new(
                rng.random_range(-1.0..=1.0),
                rng.random_range(-1.0..=1.0),
                rng.random_range(-1.0..=1.0),
            );
        let angle = axis.norm();
        let norm_squared = axis.norm_squared();
        let normalized = if norm_squared > 0.0 {
            axis / norm_squared.sqrt()
        } else {
            axis
        };
        let vector = (0.5 * angle).sin() * normalized;
        let q = Quaternion::new((0.5 * angle).cos(), vector[0], vector[1], vector[2]);
        self.parameters
            .fixed_rows_mut::<3>(0)
            .copy_from(&translation);
        self.parameters.fixed_rows_mut::<4>(3).copy_from(&q.coords);
        if CACHE_ROTATION {
            self.update_rotation_cache();
        }
    }
    pub fn set_matrix(&mut self, matrix: &Matrix4<f64>) {
        let rotation =
            quaternion_from_rotation_matrix(&matrix.fixed_view::<3, 3>(0, 0).into_owned());
        self.parameters
            .fixed_rows_mut::<3>(0)
            .copy_from(&matrix.fixed_view::<3, 1>(0, 3));
        self.parameters
            .fixed_rows_mut::<4>(3)
            .copy_from(&rotation.coords);
        if CACHE_ROTATION {
            self.update_rotation_cache();
        }
    }
    pub fn set(&mut self, translation: Vector3<f64>, rotation: Quaternion<f64>) {
        self.parameters
            .fixed_rows_mut::<3>(0)
            .copy_from(&translation);
        self.parameters
            .fixed_rows_mut::<4>(3)
            .copy_from(&normalized_quaternion(rotation).coords);
        if CACHE_ROTATION {
            self.update_rotation_cache();
        }
    }
    #[must_use]
    pub fn identity() -> Self {
        Self::default()
    }
    pub fn set_identity(&mut self) {
        self.parameters.fill(0.0);
        self.parameters[6] = 1.0;
        if CACHE_ROTATION {
            self.rotation_cache = Matrix3::identity();
        }
    }
    #[must_use]
    pub fn compose(&self, rhs: &Self) -> Self {
        Self::new(
            self.rotation_matrix() * rhs.translation() + self.translation(),
            self.quaternion() * rhs.quaternion(),
        )
    }
    /// Applies rotation only, matching the source three-vector overload.
    #[must_use]
    pub fn rotate_vector(&self, vector: &Vector3<f64>) -> Vector3<f64> {
        self.rotation_matrix() * vector
    }
    #[must_use]
    pub fn transform_homogeneous(&self, point: &Vector4<f64>) -> Vector4<f64> {
        let xyz = self.rotation_matrix() * point.fixed_rows::<3>(0) + self.translation() * point[3];
        Vector4::new(xyz[0], xyz[1], xyz[2], point[3])
    }
    pub fn oplus(&mut self, delta: &SVector<f64, 6>) -> bool {
        let translation = self.translation() + delta.fixed_rows::<3>(0);
        let rotation = normalized_quaternion(
            delta_q(&delta.fixed_rows::<3>(3).into_owned()) * self.quaternion(),
        );
        self.parameters
            .fixed_rows_mut::<3>(0)
            .copy_from(&translation);
        self.parameters
            .fixed_rows_mut::<4>(3)
            .copy_from(&rotation.coords);
        if CACHE_ROTATION {
            self.update_rotation_cache();
        }
        true
    }
    pub fn oplus_with_jacobian(
        &mut self,
        delta: &SVector<f64, 6>,
        jacobian: &mut SMatrix<f64, 7, 6>,
    ) -> bool {
        if !self.oplus(delta) {
            return false;
        }
        self.oplus_jacobian(jacobian)
    }
    pub fn oplus_jacobian(&self, jacobian: &mut SMatrix<f64, 7, 6>) -> bool {
        jacobian.fill(0.0);
        jacobian
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&Matrix3::identity());
        let mut s = SMatrix::<f64, 4, 3>::zeros();
        s[(0, 0)] = 0.5;
        s[(1, 1)] = 0.5;
        s[(2, 2)] = 0.5;
        jacobian
            .fixed_view_mut::<4, 3>(3, 3)
            .copy_from(&(oplus(&self.quaternion()) * s));
        true
    }
    pub fn lift_jacobian(&self, jacobian: &mut SMatrix<f64, 6, 7>) -> bool {
        jacobian.fill(0.0);
        jacobian
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&Matrix3::identity());
        jacobian.fixed_view_mut::<3, 4>(3, 3).copy_from(
            &(2.0 * oplus(&quaternion_inverse(&self.quaternion())).fixed_view::<3, 4>(0, 0)),
        );
        true
    }
    fn update_rotation_cache(&mut self) {
        assert!(CACHE_ROTATION, "rotation cache is disabled");
        self.rotation_cache = raw_rotation_matrix(&self.quaternion());
    }
}

pub(crate) fn normalized_quaternion(mut q: Quaternion<f64>) -> Quaternion<f64> {
    let squared_norm = q.coords.norm_squared();
    if squared_norm > 0.0 {
        q.coords /= squared_norm.sqrt();
    }
    q
}
/// Eigen's raw coefficient formula, including zero and non-unit quaternions.
pub(crate) fn raw_rotation_matrix(q: &Quaternion<f64>) -> Matrix3<f64> {
    let tx = 2.0 * q.i;
    let ty = 2.0 * q.j;
    let tz = 2.0 * q.k;
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
        1.0 - (tyy + tzz),
        txy - twz,
        txz + twy,
        txy + twz,
        1.0 - (txx + tzz),
        tyz - twx,
        txz - twy,
        tyz + twx,
        1.0 - (txx + tyy),
    )
}

/// Shoemake conversion with Eigen's strict comparisons for tied diagonal entries.
pub(crate) fn quaternion_from_rotation_matrix(matrix: &Matrix3<f64>) -> Quaternion<f64> {
    let trace = matrix.trace();
    if trace > 0.0 {
        let t = (trace + 1.0).sqrt();
        let scale = 0.5 / t;
        Quaternion::new(
            0.5 * t,
            (matrix[(2, 1)] - matrix[(1, 2)]) * scale,
            (matrix[(0, 2)] - matrix[(2, 0)]) * scale,
            (matrix[(1, 0)] - matrix[(0, 1)]) * scale,
        )
    } else {
        let mut i = 0;
        if matrix[(1, 1)] > matrix[(0, 0)] {
            i = 1;
        }
        if matrix[(2, 2)] > matrix[(i, i)] {
            i = 2;
        }
        let j = (i + 1) % 3;
        let k = (j + 1) % 3;
        let t = (matrix[(i, i)] - matrix[(j, j)] - matrix[(k, k)] + 1.0).sqrt();
        let mut coefficients = Vector4::zeros();
        coefficients[i] = 0.5 * t;
        let scale = 0.5 / t;
        coefficients[3] = (matrix[(k, j)] - matrix[(j, k)]) * scale;
        coefficients[j] = (matrix[(j, i)] + matrix[(i, j)]) * scale;
        coefficients[k] = (matrix[(k, i)] + matrix[(i, k)]) * scale;
        Quaternion::from_vector(coefficients)
    }
}

/// Copies coefficients without imposing nalgebra's unit-quaternion operations.
pub(crate) fn from_isometry_coefficients(isometry: &Isometry3<f64>) -> Transformation {
    let mut parameters = SVector::<f64, 7>::zeros();
    parameters
        .fixed_rows_mut::<3>(0)
        .copy_from(&isometry.translation.vector);
    parameters
        .fixed_rows_mut::<4>(3)
        .copy_from(&isometry.rotation.quaternion().coords);
    let mut result = Transformation::default();
    result.set_coeffs(&parameters);
    result
}

pub(crate) fn quaternion_inverse(q: &Quaternion<f64>) -> Quaternion<f64> {
    let squared_norm = q.coords.norm_squared();
    if squared_norm > 0.0 {
        q.conjugate() / squared_norm
    } else {
        Quaternion::from_vector(Vector4::zeros())
    }
}
