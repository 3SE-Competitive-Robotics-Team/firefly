// SPDX-License-Identifier: BSD-3-Clause
use super::Result;
use firefly_supereight2::{
    allocator,
    data::DataConfig,
    eigen_utils::{AlignedBox, inverse_transform, transform, transform_point, transform_points},
    map::{Map, MapConfig},
};
use nalgebra::{IsometryMatrix3, Matrix3, Rotation3, Translation3, Vector3};
use std::fmt::Write;
pub fn generate(input: &str) -> Result<String> {
    let mut output = String::from(
        "# Rust baseline: R[9] t[3] point[3] transformed_point[3] inverse_translation[3] box[6] map_bounds[18] world_corners[24] sensor_corners[24]\n",
    );
    for line in input
        .lines()
        .filter(|line| !line.starts_with('#') && !line.trim().is_empty())
    {
        // Only the first fifteen input coefficients are consumed; expected outputs are discarded.
        let values: Vec<_> = line
            .split_whitespace()
            .take(15)
            .map(|word| u32::from_str_radix(word, 16).map(f32::from_bits))
            .collect::<std::result::Result<_, _>>()?;
        if values.len() != 15 {
            return Err("transform input needs fifteen coefficients".into());
        }
        let pose = IsometryMatrix3::from_parts(
            Translation3::from(Vector3::from_row_slice(&values[9..12])),
            Rotation3::from_matrix_unchecked(Matrix3::from_row_slice(&values[..9])),
        );
        let inverse = inverse_transform(&pose);
        let mut result = values.clone();
        result.extend_from_slice(
            transform_point(&pose, &Vector3::from_row_slice(&values[12..15])).as_slice(),
        );
        result.extend_from_slice(inverse.translation.vector.as_slice());
        let bounds = transform(
            &pose,
            &AlignedBox {
                min: Vector3::new(-1.7, -0.9, -2.3),
                max: Vector3::new(2.1, 0.4, 1.8),
            },
        );
        result.extend_from_slice(bounds.min.as_slice());
        result.extend_from_slice(bounds.max.as_slice());
        let mut map = Map::from_config(
            MapConfig {
                dim: Vector3::repeat(8.0),
                res: 0.1,
                t_mw: pose,
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
            let bounds = map.aabb();
            result.extend_from_slice(bounds.min.as_slice());
            result.extend_from_slice(bounds.max.as_slice());
        }
        let world = map.voxel_to_corner_points(&Vector3::new(10, 4, 7), 8);
        for points in [world, transform_points(&inverse, &world)] {
            for axis in 0..3 {
                result.extend(points.iter().map(|p| p[axis]));
            }
        }
        for value in result {
            write!(output, " {:08x}", value.to_bits())?;
        }
        output.push('\n');
    }
    Ok(output)
}
#[cfg(test)]
mod tests {
    #[test]
    fn expected_output_columns_are_not_inputs_to_generation() {
        let input = "3f800000 0 0 0 3f800000 0 0 0 3f800000 0 0 0 3f800000 40000000 40400000";
        let first = super::generate(&format!("{input} deadbeef")).unwrap();
        assert_eq!(
            first,
            super::generate(&format!("{input} completely_invalid_expected_output")).unwrap()
        );
        assert_eq!(first.lines().nth(1).unwrap().split_whitespace().count(), 93);
        assert!(super::generate("0 0").is_err());
    }
}
