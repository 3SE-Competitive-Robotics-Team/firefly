// SPDX-License-Identifier: BSD-3-Clause

//! Occupancy laws, analytic transforms and depth fusion contracts; no external reference files.
use firefly_supereight2::{
    data::{DataConfig, OccupancyData},
    data_field::{OccupancyField, UncertaintyModel},
    eigen_utils::{AlignedBox, inverse_transform, transform, transform_point},
    image::Image,
    key_util::{compact, decode_code, encode_code, expand},
    map::Map,
    map_integrator::MapIntegrator,
    measurement::{Measurement, Measurements},
    multires_ofusion_core::{free_voxel, update_voxel},
    pinhole_camera::{PinholeCamera, PinholeConfig},
    sensor::SensorConfig,
    uncertainty::depth_sigma,
};
use nalgebra::{IsometryMatrix3, Rotation3, Translation3, Vector3};
use proptest::prelude::*;

proptest! {
    #![proptest_config(ProptestConfig::with_failure_persistence(
        proptest::test_runner::FileFailurePersistence::WithSource("proptest-regressions")
    ))]
    #[test]
    fn morton_encoding_preserves_all_nineteen_coordinate_bits(x in 0i32..1<<19, y in 0i32..1<<19, z in 0i32..1<<19) {
        let p = Vector3::new(x,y,z);
        // Each coordinate bit occupies its own interleaved lane; this is an independent oracle.
        let code = encode_code(&p).unwrap();
        for bit in 0..19 {
            for axis in 0..3 {prop_assert_eq!((code>>(3*bit+axis))&1,((p[axis]>>bit)&1) as u64);}
        }
    }

    #[test]
    fn morton_components_roundtrip_all_nineteen_bits(value in 0u64..1<<19) {
        prop_assert_eq!(compact(expand(value).unwrap()).unwrap(), value);
    }

    #[test]
    fn morton_codes_roundtrip_within_documented_decoder_range(x in 0i32..1<<18, y in 0i32..1<<18, z in 0i32..1<<18) {
        let p = Vector3::new(x,y,z);
        prop_assert_eq!(decode_code(encode_code(&p).unwrap()).unwrap(),p);
    }

    #[test]
    fn occupancy_mean_equals_sample_mean_before_weight_saturation(samples in prop::collection::vec(-5i16..6,1..20)) {
        let mut field = OccupancyField::default();
        for &value in &samples {field.update(f32::from(value),32.0);}
        let expected = samples.iter().map(|&v|f32::from(v)).sum::<f32>()/samples.len() as f32;
        prop_assert!((field.occupancy-expected).abs()<2e-6);
        prop_assert_eq!(field.weight,samples.len() as f32);
        prop_assert!(field.valid() && field.observed);
    }

    #[test]
    fn rigid_transform_preserves_distances_and_inverts(
        angles in prop::array::uniform3(-2.0f32..2.0),
        position in prop::array::uniform3(-4.0f32..4.0),
        point in prop::array::uniform3(-4.0f32..4.0)
    ) {
        let t = IsometryMatrix3::from_parts(Translation3::from(Vector3::from(position)),Rotation3::from_euler_angles(angles[0],angles[1],angles[2]));
        let p = Vector3::from(point);
        let result = transform_point(&t,&p);
        // Norm and inverse laws tolerate accumulated f32 rounding, while detecting wrong frames/signs.
        prop_assert!(((result-t.translation.vector).norm()-p.norm()).abs()<4e-6);
        prop_assert!((transform_point(&inverse_transform(&t),&result)-p).amax()<4e-6);
    }
}

#[test]
fn transformed_bounds_match_center_and_absolute_rotation_extents() {
    let bounds = AlignedBox {
        min: Vector3::new(-1.0, -2.0, -0.5),
        max: Vector3::new(3.0, 1.0, 2.5),
    };
    let pose = IsometryMatrix3::from_parts(
        Translation3::new(0.7, -0.3, 1.2),
        Rotation3::from_euler_angles(0.2, -0.6, 0.4),
    );
    let actual = transform(&pose, &bounds);
    // AABB support function: transformed centre +/- |R| times half-extents; no corner loop.
    let centre = (bounds.min + bounds.max) * 0.5;
    let half = (bounds.max - bounds.min) * 0.5;
    let expected_centre = pose.rotation.matrix().cast::<f64>() * centre.cast::<f64>()
        + pose.translation.vector.cast::<f64>();
    let expected_half = pose.rotation.matrix().map(|v| f64::from(v).abs()) * half.cast::<f64>();
    assert!((actual.min.cast::<f64>() - (expected_centre - expected_half)).amax() < 5e-7);
    assert!((actual.max.cast::<f64>() - (expected_centre + expected_half)).amax() < 5e-7);
    let empty = AlignedBox {
        min: Vector3::repeat(1.0),
        max: Vector3::repeat(-1.0),
    };
    assert_eq!(transform(&pose, &empty), empty);
}

#[test]
fn weight_saturation_retains_bounded_memory_and_repeated_observation_flag() {
    let config = DataConfig::default();
    let mut value = OccupancyData::default();
    assert!(free_voxel(&mut value, &config));
    assert!(!free_voxel(&mut value, &config));
    assert!(value.field.occupancy < 0.0 && value.valid());
    let mut field = OccupancyField::default();
    for _ in 0..10 {
        field.update(-2.0, 4.0);
    }
    assert_eq!(field.weight, 4.0);
    assert_eq!(field.occupancy, -2.0);
    field.update(2.0, 4.0);
    assert_eq!(field.weight, 4.0);
    assert_eq!(field.occupancy, -1.0);
}

#[test]
fn inverse_sensor_model_has_free_surface_occupied_and_unobserved_regions() {
    let config = DataConfig::default();
    let (tau, sigma) = (0.4, 0.2);
    for (range, sign) in [
        (-1.0, -1.0),
        (-sigma, -1.0),
        (0.0, 0.0),
        (tau * 0.25, 1.0),
        (tau * 0.75, 1.0),
    ] {
        let mut voxel = OccupancyData::default();
        assert!(update_voxel(&mut voxel, range, tau, sigma, &config));
        assert!(voxel.valid());
        if sign == 0.0 {
            assert!(voxel.field.occupancy.abs() < 1e-6);
        } else {
            assert!(voxel.field.occupancy * sign > 0.0);
        }
    }
    for range in [tau, tau.next_up(), f32::INFINITY, f32::NAN] {
        let mut voxel = OccupancyData::default();
        assert!(!update_voxel(&mut voxel, range, tau, sigma, &config));
        assert_eq!(voxel, OccupancyData::default());
    }
}

#[test]
fn sigma_model_handles_missing_depth_and_clamps_linear_and_quadratic_uncertainty() {
    let mut image = Image::<f32>::new(1, 6, 1);
    image.data = vec![0.0, -1.0, f32::NAN, 0.5, 2.0, 100.0];
    for model in [UncertaintyModel::Linear, UncertaintyModel::Quadratic] {
        let mut config = DataConfig::default().field;
        config.uncertainty_model = model;
        let sigma = depth_sigma(&image, 0.01, &config).unwrap();
        assert_eq!(&sigma.data[..3], &[0.0; 3]);
        for (&z, &actual) in image.data[3..].iter().zip(&sigma.data[3..]) {
            let raw = if model == UncertaintyModel::Linear {
                config.k_sigma * z
            } else {
                config.k_sigma * z * z
            };
            let expected = raw.clamp(
                0.01 * config.sigma_min_factor,
                0.01 * config.sigma_max_factor,
            );
            assert!((actual - expected).abs() < 1e-7);
        }
    }
    image.data.pop();
    assert!(depth_sigma(&image, 0.01, &DataConfig::default().field).is_err());
}

fn sensor() -> PinholeCamera {
    PinholeCamera::new(&PinholeConfig {
        sensor: SensorConfig {
            width: 64,
            height: 48,
            near_plane: 0.1,
            far_plane: 6.0,
            ..Default::default()
        },
        fx: 64.0,
        fy: 63.0,
        cx: 31.5,
        cy: 23.5,
    })
    .unwrap()
}

#[test]
fn depth_fusion_observes_free_space_and_surface_but_leaves_far_space_unknown() {
    let sensor = sensor();
    let mut map = Map::new(Vector3::repeat(8.0), 0.25, DataConfig::default()).unwrap();
    let mut depth = Image::<f32>::new(48, 64, 1);
    depth.data.fill(2.0);
    let measurements = Measurements::new(Measurement {
        image: &depth,
        sensor: &sensor,
        t_wc: IsometryMatrix3::identity(),
    });
    MapIntegrator::new(&mut map)
        .integrate_depth(0, &measurements, None)
        .unwrap();
    let free = map
        .get_data(&Vector3::new(0.125, 0.125, 1.125), true)
        .unwrap();
    let surface = map
        .get_data(&Vector3::new(0.125, 0.125, 2.125), true)
        .unwrap();
    let far = map
        .get_data(&Vector3::new(0.125, 0.125, 3.875), true)
        .unwrap();
    assert!(free.valid() && free.get_field() < 0.0, "free={free:?}");
    assert!(
        surface.valid() && surface.get_field() > 0.0,
        "surface={surface:?}"
    );
    // Default tau_min = 6 voxels = 1.5 m: the occupied support extends to 3.5 m.
    assert!(3.875 > 2.0 + map.resolution * map.data_config.field.tau_min_factor);
    assert!(!far.valid(), "far={far:?}");
    assert!(!map.aabb().is_empty());
}

#[test]
fn invalid_map_geometry_and_depth_shape_are_rejected() {
    for resolution in [0.0, -0.1, f32::NAN, f32::INFINITY] {
        assert!(Map::new(Vector3::repeat(8.0), resolution, DataConfig::default()).is_err());
    }
    assert!(
        Map::new(
            Vector3::new(8.0, f32::NAN, 8.0),
            0.25,
            DataConfig::default()
        )
        .is_err()
    );
    assert!(encode_code(&Vector3::new(-1, 0, 0)).is_err());
    let sensor = sensor();
    let mut map = Map::new(Vector3::repeat(8.0), 0.25, DataConfig::default()).unwrap();
    let before = map.octree.octants.len();
    let depth = Image::<f32>::new(47, 64, 1);
    let measurements = Measurements::new(Measurement {
        image: &depth,
        sensor: &sensor,
        t_wc: IsometryMatrix3::identity(),
    });
    assert!(
        MapIntegrator::new(&mut map)
            .integrate_depth(0, &measurements, None)
            .is_err()
    );
    assert_eq!(map.octree.octants.len(), before);
}

#[test]
fn morton_decoder_retains_explicit_upper_component_error_boundary() {
    // SE08: the source compact() accepts a component-sized integer, but decode_code()
    // passes all interleaved lanes. Preserve the documented rejection, without hiding it
    // by restricting the encoder's independently checked 19-bit input domain.
    let point = Vector3::new(1 << 18, 1 << 17, 0);
    let code = encode_code(&point).unwrap();
    assert_eq!(code, (1u64 << 54) | (1u64 << 52));
    assert!(decode_code(code).is_err());
}
