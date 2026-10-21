// SPDX-License-Identifier: BSD-3-Clause
//! Bit-exact Rust regression baselines for 1,000 general poses and four signed-zero cases.
use firefly_supereight2::{
    allocator,
    data::DataConfig,
    eigen_utils::{AlignedBox, inverse_transform, transform, transform_point, transform_points},
    map::{Map, MapConfig},
};
use nalgebra::{IsometryMatrix3, Matrix3, Rotation3, Translation3, Vector3};

struct Case {
    pose: IsometryMatrix3<f32>,
    values: Vec<f32>,
}

fn cases() -> Vec<Case> {
    let cases: Vec<_> = include_str!("reference/general_transforms.txt")
        .lines()
        .filter(|line| !line.starts_with('#'))
        .map(|line| {
            let values: Vec<_> = line
                .split_whitespace()
                .map(|word| f32::from_bits(u32::from_str_radix(word, 16).unwrap()))
                .collect();
            assert_eq!(values.len(), 93);
            Case {
                pose: IsometryMatrix3::from_parts(
                    Translation3::from(Vector3::from_row_slice(&values[9..12])),
                    Rotation3::from_matrix_unchecked(Matrix3::from_row_slice(&values[..9])),
                ),
                values,
            }
        })
        .collect();
    assert_eq!(cases.len(), 1004);
    cases
}

fn assert_bits(actual: &[f32], expected: &[f32], case: usize) {
    assert_eq!(actual.len(), expected.len());
    for (axis, (a, b)) in actual.iter().zip(expected).enumerate() {
        assert_eq!(
            a.to_bits(),
            b.to_bits(),
            "case {case}, component {axis}: {a} != {b}"
        );
    }
}

#[test]
fn general_point_transforms_match_baseline_bits() {
    for (index, case) in cases().iter().enumerate() {
        let point = Vector3::from_row_slice(&case.values[12..15]);
        assert_bits(
            transform_point(&case.pose, &point).as_slice(),
            &case.values[15..18],
            index,
        );
    }
}

#[test]
fn general_inverse_transforms_match_baseline_bits() {
    for (index, case) in cases().iter().enumerate() {
        let inverse = inverse_transform(&case.pose);
        assert_bits(
            inverse.translation.vector.as_slice(),
            &case.values[18..21],
            index,
        );
        assert_eq!(
            inverse.rotation.matrix(),
            &case.pose.rotation.matrix().transpose()
        );
    }
}

#[test]
fn general_box_transforms_match_baseline_bits() {
    let bounds = AlignedBox {
        min: Vector3::new(-1.7, -0.9, -2.3),
        max: Vector3::new(2.1, 0.4, 1.8),
    };
    for (index, case) in cases().iter().enumerate() {
        let actual = transform(&case.pose, &bounds);
        assert_bits(actual.min.as_slice(), &case.values[21..24], index);
        assert_bits(actual.max.as_slice(), &case.values[24..27], index);
    }
}

#[test]
fn general_map_bounds_match_baseline_bits() -> Result<(), Box<dyn std::error::Error>> {
    for (index, case) in cases().iter().enumerate() {
        let mut map = Map::from_config(
            MapConfig {
                dim: Vector3::repeat(8.0),
                res: 0.1,
                t_mw: case.pose,
            },
            DataConfig::default(),
            8,
        )?;
        for stage in 0..3 {
            if stage > 0 {
                let coord = if stage == 1 {
                    Vector3::new(10, 4, 7)
                } else {
                    Vector3::new(25, 20, 18)
                };
                let root = map.octree.root;
                allocator::block(&coord, &mut map.octree, root)?;
            }
            let actual = map.aabb();
            let start = 27 + stage * 6;
            assert_bits(actual.min.as_slice(), &case.values[start..start + 3], index);
            assert_bits(
                actual.max.as_slice(),
                &case.values[start + 3..start + 6],
                index,
            );
        }
    }
    Ok(())
}

#[test]
fn general_map_and_sensor_corners_match_baseline_bits() -> Result<(), Box<dyn std::error::Error>> {
    for (index, case) in cases().iter().enumerate() {
        let map = Map::from_config(
            MapConfig {
                dim: Vector3::repeat(8.0),
                res: 0.1,
                t_mw: case.pose,
            },
            DataConfig::default(),
            8,
        )?;
        let world = map.voxel_to_corner_points(&Vector3::new(10, 4, 7), 8);
        let sensor = transform_points(&inverse_transform(&case.pose), &world);
        for (points, start) in [(world, 45), (sensor, 69)] {
            for axis in 0..3 {
                let actual = points.map(|point| point[axis]);
                assert_bits(
                    &actual,
                    &case.values[start + axis * 8..start + (axis + 1) * 8],
                    index,
                );
            }
        }
    }
    Ok(())
}
