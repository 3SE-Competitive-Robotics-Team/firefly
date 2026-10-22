// SPDX-License-Identifier: BSD-3-Clause
//! Three-frame depth-map baselines covering axis-aligned and general rotations.
use firefly_supereight2::{
    data::DataConfig,
    image::Image,
    map::Map,
    map_integrator::MapIntegrator,
    measurement::{Measurement, Measurements},
    octant::Octant,
    pinhole_camera::{PinholeCamera, PinholeConfig},
    sensor::SensorConfig,
};
use nalgebra::{IsometryMatrix3, Matrix3, Rotation3, Vector3};
fn field(v: Option<(f32, i32)>, out: &mut Vec<f64>) {
    match v {
        Some((v, s)) => out.extend([1.0, f64::from(v), f64::from(s)]),
        None => out.extend([0.0, 0.0, -99.0]),
    }
}
fn gradient(v: Option<(Vector3<f32>, i32)>, out: &mut Vec<f64>) {
    match v {
        Some((v, s)) => out.extend([
            1.0,
            f64::from(v.x),
            f64::from(v.y),
            f64::from(v.z),
            f64::from(s),
        ]),
        None => out.extend([0.0, 0.0, 0.0, 0.0, -99.0]),
    }
}
fn integrate_frame(
    map: &mut Map,
    sensor: &PinholeCamera,
    rotation: Rotation3<f32>,
    frame: i32,
) -> Result<Vec<usize>, Box<dyn std::error::Error>> {
    let mut image = Image::<f32>::new(48, 64, 1);
    for y in 0..48 {
        for x in 0..64 {
            image.data[x + y * 64] = if (x + 3 * y) % 47 == 0 {
                0.0
            } else {
                2.0f32 + ((x / 16 + y / 12) % 3) as f32 * 0.1f32
            };
        }
    }
    let mut pose = IsometryMatrix3::translation(0.03f32 * frame as f32, 0.0, 0.0);
    pose.rotation = rotation;
    let measurements = Measurements::new(Measurement {
        image: &image,
        sensor,
        t_wc: pose,
    });
    MapIntegrator::new(map).integrate_depth(frame, &measurements, None)?;
    let mut blocks: Vec<usize> = map
        .octree
        .octants
        .iter()
        .enumerate()
        .filter_map(|(i, o)| {
            if matches!(o, Some(Octant::Block(_))) {
                Some(i)
            } else {
                None
            }
        })
        .collect();
    blocks.sort_by_key(|i| {
        let c = map.octree.get(*i).unwrap().base().coord;
        (c.z, c.y, c.x)
    });
    Ok(blocks)
}

fn reference_camera() -> Result<PinholeCamera, Box<dyn std::error::Error>> {
    let config = PinholeConfig {
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
    };
    Ok(PinholeCamera::new(&config)?)
}

fn compare_map_reference(
    text: &str,
    rotation: Rotation3<f32>,
    expected_count: usize,
) -> Result<(), Box<dyn std::error::Error>> {
    let sensor = reference_camera()?;
    let mut map = Map::new(Vector3::repeat(8.0), 0.25, DataConfig::default())?;
    let mut blocks = Vec::new();
    let mut block_index = 0;
    let mut active = 0;
    let mut count = 0;
    let mut diffs = std::collections::BTreeMap::<String, (usize, f64)>::new();
    let mut failures = Vec::new();
    for line in text.lines() {
        let mut words = line.split_whitespace();
        let tag = words.next().unwrap();
        let v: Vec<f64> = words.map(|x| x.parse().unwrap()).collect();
        let mut expected = v.as_slice();
        let mut out = Vec::new();
        match tag {
            "frame" => {
                let frame = v[0] as i32;
                blocks = integrate_frame(&mut map, &sensor, rotation, frame)?;
                out.extend([f64::from(frame), blocks.len() as f64]);
                block_index = 0;
            }
            "block" => {
                active = blocks[block_index];
                block_index += 1;
                let b = map.octree.block(active)?;
                out.extend([
                    f64::from(b.base.coord.x),
                    f64::from(b.base.coord.y),
                    f64::from(b.base.coord.z),
                    f64::from(b.current_scale),
                    f64::from(b.min_scale),
                    f64::from(b.buffer_scale()),
                ]);
            }
            "values" => {
                let b = map.octree.block(active)?;
                let data = b.data_at_scale(b.current_scale)?;
                out.push(data.len() as f64);
                for d in data {
                    out.extend([
                        f64::from(d.field.occupancy),
                        f64::from(d.field.weight),
                        f64::from(d.field.observed),
                    ]);
                }
            }
            "query" => {
                let p = Vector3::new(v[0] as f32, v[1] as f32, v[2] as f32);
                let scale = v[3] as i32;
                field(map.interp_field(&p, scale, true)?, &mut out);
                gradient(map.grad_field(&p, scale, true)?, &mut out);
                field(
                    map.interp(
                        &p,
                        scale,
                        true,
                        firefly_supereight2::data::OccupancyData::valid,
                        |d| d.field.occupancy,
                    )?,
                    &mut out,
                );
                gradient(
                    map.grad(
                        &p,
                        scale,
                        true,
                        firefly_supereight2::data::OccupancyData::valid,
                        |d| d.field.occupancy,
                    )?,
                    &mut out,
                );
                expected = &v[4..];
            }
            _ => panic!("unknown {tag}"),
        }
        if out.len() != expected.len() {
            failures.push(format!("{tag}: length {} vs {}", out.len(), expected.len()));
        }
        for (i, (a, b)) in out.iter().zip(expected).enumerate() {
            count += 1;
            if a == b || a.is_nan() && b.is_nan() {
                continue;
            }
            let err = (a - b).abs() / (1.0 + b.abs());
            let entry = diffs.entry(tag.into()).or_default();
            entry.0 += 1;
            entry.1 = entry.1.max(err);
            if failures.len() < 50 {
                failures.push(format!(
                    "{tag} item{i} block{block_index} {a} expected{b}; rel{err}"
                ));
            }
        }
    }
    assert_eq!(count, expected_count);
    assert!(
        failures.is_empty() && diffs.is_empty(),
        "nonzero reference differences: {diffs:?}; {failures:?}"
    );
    Ok(())
}

#[test]
fn rotated_map_matches_baseline() -> Result<(), Box<dyn std::error::Error>> {
    compare_map_reference(
        include_str!("reference/rotated_map.txt"),
        Rotation3::from_axis_angle(&Vector3::y_axis(), 0.12f32),
        58074,
    )
}

#[test]
fn general_axis_map_matches_baseline() -> Result<(), Box<dyn std::error::Error>> {
    let (header, frames) = include_str!("reference/general_axis_map.txt")
        .split_once('\n')
        .unwrap();
    let mut words = header.split_whitespace();
    assert_eq!(words.next(), Some("rotation"));
    let coefficients: Vec<f32> = words.map(|value| value.parse().unwrap()).collect();
    assert_eq!(coefficients.len(), 9);
    compare_map_reference(
        frames,
        Rotation3::from_matrix_unchecked(Matrix3::from_row_slice(&coefficients)),
        62913,
    )
}
