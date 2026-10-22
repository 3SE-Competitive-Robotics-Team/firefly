// SPDX-License-Identifier: BSD-3-Clause
use super::Result;
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
use std::fmt::Write;
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
) -> Result<Vec<usize>> {
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

fn reference_camera() -> Result<PinholeCamera> {
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

fn row(output: &mut String, tag: &str, values: impl IntoIterator<Item = f64>) -> Result<()> {
    output.push_str(tag);
    for value in values {
        write!(output, " {value}")?;
    }
    output.push('\n');
    Ok(())
}
pub fn generate(input: &str, general_axis: bool) -> Result<String> {
    let mut output = String::new();
    let (rotation, frames) = if general_axis {
        let (header, frames) = input.split_once('\n').ok_or("missing rotation header")?;
        let mut words = header.split_whitespace();
        if words.next() != Some("rotation") {
            return Err("invalid rotation header".into());
        }
        let coefficients = words
            .map(str::parse::<f32>)
            .collect::<std::result::Result<Vec<_>, _>>()?;
        if coefficients.len() != 9 {
            return Err("rotation requires nine coefficients".into());
        }
        writeln!(output, "{header}")?;
        (
            Rotation3::from_matrix_unchecked(Matrix3::from_row_slice(&coefficients)),
            frames,
        )
    } else {
        (Rotation3::from_axis_angle(&Vector3::y_axis(), 0.12), input)
    };
    let sensor = reference_camera()?;
    let mut map = Map::new(Vector3::repeat(8.0), 0.25, DataConfig::default())?;
    for line in frames.lines() {
        let mut words = line.split_whitespace();
        match words.next() {
            Some("frame") => {
                let frame = words.next().ok_or("missing frame number")?.parse::<i32>()?;
                let blocks = integrate_frame(&mut map, &sensor, rotation, frame)?;
                row(
                    &mut output,
                    "frame",
                    [f64::from(frame), blocks.len() as f64],
                )?;
                for id in blocks {
                    let block = map.octree.block(id)?;
                    row(
                        &mut output,
                        "block",
                        [
                            f64::from(block.base.coord.x),
                            f64::from(block.base.coord.y),
                            f64::from(block.base.coord.z),
                            f64::from(block.current_scale),
                            f64::from(block.min_scale),
                            f64::from(block.buffer_scale()),
                        ],
                    )?;
                    let data = block.data_at_scale(block.current_scale)?;
                    let mut values = vec![data.len() as f64];
                    for voxel in data {
                        values.extend([
                            f64::from(voxel.field.occupancy),
                            f64::from(voxel.field.weight),
                            f64::from(voxel.field.observed),
                        ]);
                    }
                    row(&mut output, "values", values)?;
                }
            }
            Some("query") => {
                let inputs = words
                    .take(4)
                    .map(str::parse::<f64>)
                    .collect::<std::result::Result<Vec<_>, _>>()?;
                if inputs.len() != 4 {
                    return Err("query requires point and scale".into());
                }
                let point = Vector3::new(inputs[0] as f32, inputs[1] as f32, inputs[2] as f32);
                let scale = inputs[3] as i32;
                let mut values = inputs;
                field(map.interp_field(&point, scale, true)?, &mut values);
                gradient(map.grad_field(&point, scale, true)?, &mut values);
                field(
                    map.interp(
                        &point,
                        scale,
                        true,
                        firefly_supereight2::data::OccupancyData::valid,
                        |d| d.field.occupancy,
                    )?,
                    &mut values,
                );
                gradient(
                    map.grad(
                        &point,
                        scale,
                        true,
                        firefly_supereight2::data::OccupancyData::valid,
                        |d| d.field.occupancy,
                    )?,
                    &mut values,
                );
                row(&mut output, "query", values)?;
            }
            // Baseline values and block metadata are outputs, never scene inputs.
            Some("block" | "values") | None => {}
            Some(tag) => return Err(format!("invalid input tag: {tag}").into()),
        }
    }
    Ok(output)
}
#[cfg(test)]
mod tests {
    #[test]
    fn missing_rotation_and_query_inputs_are_rejected() {
        assert!(super::generate("rotation 1 0", true).is_err());
        assert!(super::generate("query 0 0", false).is_err());
        assert!(super::generate("invalid 0", false).is_err());
    }
}
