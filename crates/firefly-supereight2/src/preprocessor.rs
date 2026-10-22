/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-License-Identifier: MIT
 */
//! Depth median reduction and pinhole point-cloud conversion.
use crate::{
    image::{Image, invalid, validate_image},
    pinhole_camera::PinholeCamera,
};
use firefly_error::Result;
use nalgebra::{Matrix4, Vector2, Vector3};
/// # Errors
/// Returns an error for invalid image storage or dimensions that do not admit the same integer
/// downsampling factor on both axes.
pub fn downsample_depth(input: &Image<f32>, output: &mut Image<f32>) -> Result<Image<usize>> {
    validate_image(input)?;
    validate_image(output)?;
    if output.cols == 0
        || output.rows == 0
        || input.cols < output.cols
        || input.rows < output.rows
        || !input.cols.is_multiple_of(output.cols)
        || !input.rows.is_multiple_of(output.rows)
        || input.cols / output.cols != input.rows / output.rows
    {
        return Err(invalid(
            "depth dimensions require an equal integer downsampling factor",
        ));
    }
    let ratio = input.cols / output.cols;
    let mut indices = Vec::with_capacity(output.data.len());
    let mut region = Vec::with_capacity(ratio * ratio);
    for y in 0..output.rows {
        for x in 0..output.cols {
            let (x0, y0) = (x * ratio, y * ratio);
            for dy in 0..ratio {
                for dx in 0..ratio {
                    let i = x0 + dx + (y0 + dy) * input.cols;
                    let depth = input.data[i];
                    if depth >= 1e-5 {
                        region.push((i, depth));
                    }
                }
            }
            let (index, depth) = if region.is_empty() {
                (x0 + y0 * input.cols, 0.0)
            } else {
                region.sort_unstable_by(|a, b| {
                    a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal)
                });
                region[region.len() / 2]
            };
            output.data[x + y * output.cols] = depth;
            indices.push(index);
            region.clear();
        }
    }
    Ok(Image::from_vec(output.rows, output.cols, 1, indices))
}
/// # Errors
/// Returns an error for non-contiguous/non-single-channel image storage or mismatched image
/// dimensions.
pub fn depth_to_point_cloud(
    output: &mut Image<Vector3<f32>>,
    depth: &Image<f32>,
    sensor: &PinholeCamera,
) -> Result<()> {
    validate_image(output)?;
    validate_image(depth)?;
    if output.rows != depth.rows || output.cols != depth.cols {
        return Err(invalid("point-cloud/depth dimensions differ"));
    }
    for y in 0..depth.rows {
        for x in 0..depth.cols {
            let i = x + y * depth.cols;
            output.data[i] = if depth.data[i] > 0.0 {
                sensor.model.back_project(&Vector2::new(x as f32, y as f32)) * depth.data[i]
            } else {
                Vector3::zeros()
            };
        }
    }
    Ok(())
}
/// # Errors
/// Returns an error for non-contiguous/non-single-channel image storage or mismatched image
/// dimensions.
pub fn point_cloud_to_depth(
    output: &mut Image<f32>,
    cloud: &Image<Vector3<f32>>,
    t_cx: &Matrix4<f32>,
) -> Result<()> {
    validate_image(output)?;
    validate_image(cloud)?;
    if output.rows != cloud.rows || output.cols != cloud.cols {
        return Err(invalid("point-cloud/depth dimensions differ"));
    }
    for (d, p) in output.data.iter_mut().zip(&cloud.data) {
        *d = (t_cx * p.push(1.0)).z;
    }
    Ok(())
}
