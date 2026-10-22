/*
 * SPDX-FileCopyrightText: 2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Axis-aligned bounds and coefficient-wise clamps.
use nalgebra::{IsometryMatrix3, Matrix3, Rotation3, SVector, Translation3, Vector3};
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AlignedBox<const N: usize> {
    pub min: SVector<f32, N>,
    pub max: SVector<f32, N>,
}
impl<const N: usize> AlignedBox<N> {
    #[must_use]
    pub fn is_empty(&self) -> bool {
        (0..N).any(|i| self.min[i] > self.max[i])
    }
}
#[must_use]
pub fn dilate_erode<const N: usize>(bounds: &AlignedBox<N>, distance: f32) -> AlignedBox<N> {
    if bounds.is_empty() {
        return *bounds;
    }
    AlignedBox {
        min: bounds.min.add_scalar(-distance),
        max: bounds.max.add_scalar(distance),
    }
}
/// Match Eigen 3.4's `Isometry3f * Vector3f`: left-associated rotation, then translation.
#[must_use]
pub fn transform_point(t: &IsometryMatrix3<f32>, point: &Vector3<f32>) -> Vector3<f32> {
    let r = t.rotation.matrix();
    Vector3::from_fn(|row, _| {
        ((r[(row, 0)] * point.x + r[(row, 1)] * point.y) + r[(row, 2)] * point.z)
            + t.translation.vector[row]
    })
}
/// Match Eigen 3.4's rigid inverse: transpose the rotation and evaluate `(-R^T) * t`.
#[must_use]
pub fn inverse_transform(t: &IsometryMatrix3<f32>) -> IsometryMatrix3<f32> {
    let rotation = t.rotation.matrix().transpose();
    let translation = matrix_vector_product(&-rotation, &t.translation.vector);
    IsometryMatrix3::from_parts(
        Translation3::from(translation),
        Rotation3::from_matrix_unchecked(rotation),
    )
}

// Eigen's Matrix3f * Vector3f and Transform * Matrix<float, 3, 8> use this reduction.
pub(crate) fn matrix_vector_product(matrix: &Matrix3<f32>, point: &Vector3<f32>) -> Vector3<f32> {
    Vector3::from_fn(|row, _| {
        matrix[(row, 0)] * point.x + (matrix[(row, 1)] * point.y + matrix[(row, 2)] * point.z)
    })
}

/// Match Eigen 3.4's `Transform * Matrix<float, 3, 8>` reduction for eight columns.
#[must_use]
pub fn transform_points(t: &IsometryMatrix3<f32>, points: &[Vector3<f32>; 8]) -> [Vector3<f32>; 8] {
    points.map(|point| matrix_vector_product(t.rotation.matrix(), &point) + t.translation.vector)
}

#[must_use]
pub fn transform(t_ba: &IsometryMatrix3<f32>, bounds: &AlignedBox<3>) -> AlignedBox<3> {
    transform_bounds(t_ba.rotation.matrix(), &t_ba.translation.vector, bounds)
}

/// Match the reference's eight-corner matrix expression, including scaled linear parts.
pub(crate) fn transform_bounds(
    linear: &Matrix3<f32>,
    translation: &Vector3<f32>,
    bounds: &AlignedBox<3>,
) -> AlignedBox<3> {
    if bounds.is_empty() {
        return *bounds;
    }
    let corner = |i: usize| {
        Vector3::new(
            if i & 1 == 0 {
                bounds.min.x
            } else {
                bounds.max.x
            },
            if i & 2 == 0 {
                bounds.min.y
            } else {
                bounds.max.y
            },
            if i & 4 == 0 {
                bounds.min.z
            } else {
                bounds.max.z
            },
        )
    };
    let first = matrix_vector_product(linear, &corner(0)) + translation;
    let mut result = AlignedBox {
        min: first,
        max: first,
    };
    for index in 1..8 {
        let p = matrix_vector_product(linear, &corner(index)) + translation;
        for axis in 0..3 {
            if p[axis] < result.min[axis] {
                result.min[axis] = p[axis];
            }
            if result.max[axis] < p[axis] {
                result.max[axis] = p[axis];
            }
        }
    }
    result
}
pub fn clamp<const N: usize>(
    v: &mut SVector<f32, N>,
    low: &SVector<f32, N>,
    high: &SVector<f32, N>,
) {
    for i in 0..N {
        let value = if v[i] < low[i] { low[i] } else { v[i] };
        v[i] = if high[i] < value { high[i] } else { value };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{map::MapConfig, sensor::SensorConfig};
    use nalgebra::{Matrix3, Rotation3, Translation3};

    #[test]
    fn point_transform_adds_translation_after_the_row_dot_product() {
        let rotation = Rotation3::from_matrix_unchecked(Matrix3::new(
            0.992_808_6,
            0.,
            0.119_712_204,
            0.,
            1.,
            0.,
            -0.119_712_204,
            0.,
            0.992_808_6,
        ));
        let pose = inverse_transform(&IsometryMatrix3::from_parts(
            Translation3::new(0.03, 0., 0.),
            rotation,
        ));
        let point = Vector3::new(-1.875, -1.875, 0.125);
        let actual = transform_point(&pose, &point);
        // Eigen 3.4 Isometry3f * Vector3f, default floating-point evaluation.
        assert_eq!(actual.x.to_bits(), 0xbff4_0078);
    }

    #[test]
    fn configuration_preserves_rotation_matrix_coefficients() {
        let document: toml::Value = toml::from_str(
            r"
[sensor]
R_BS = [0.99280864000320435,0,0.119_712_204, 0,1,0, -0.119_712_204,0,0.99280864000320435]
[map]
R_MW = [0.99280864000320435,0,0.119_712_204, 0,1,0, -0.119_712_204,0,0.99280864000320435]
",
        )
        .unwrap();
        let mut sensor = SensorConfig::default();
        let mut map = MapConfig::default();
        sensor.read_toml(&document).unwrap();
        map.read_toml(&document).unwrap();
        for pose in [sensor.t_bs, map.t_mw] {
            assert_eq!(pose.rotation[(0, 0)].to_bits(), 0.992_808_64_f32.to_bits());
            assert_eq!(pose.rotation[(0, 2)].to_bits(), 0.119_712_204_f32.to_bits());
        }
    }
}
