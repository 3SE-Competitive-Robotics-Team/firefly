// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
// 3. The name of the copyright holders may not be used to endorse or promote
// products derived from this software without specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//! `OpenCV` 4.10.0 `cornersubpix.cpp` and `samplers.cpp`, u8 grayscale specialization.
//! Pinned source 71d3237a093b60a27601c20e9ee6c3e52154e8b1; Basalt's fixed
//! zero zone (-1,-1), maximum 100 iterations and epsilon 0.01.
use firefly_error::{Error, ErrorKind, Result};
use purecv::core::types::Point2f;

fn sample(
    image: &[u8],
    width: usize,
    height: usize,
    center: Point2f,
    side: usize,
    output: &mut [f32],
) {
    let x = center.x - (side - 1) as f32 * 0.5;
    let y = center.y - (side - 1) as f32 * 0.5;
    let ix = x.floor() as isize;
    let iy = y.floor() as isize;
    let mut a = x - ix as f32;
    let b = y - iy as f32;
    let pixel = |x: isize, y: isize| {
        f32::from(
            image[y.clamp(0, height as isize - 1) as usize * width
                + x.clamp(0, width as isize - 1) as usize],
        )
    };
    if ix >= 0
        && iy >= 0
        && ix + (side as isize) < width as isize
        && iy + (side as isize) < height as isize
    {
        a = a.max(0.0001);
        let a12 = a * (1. - b);
        let a22 = a * b;
        let scale = (1. - f64::from(a)) / f64::from(a);
        for row in 0..side {
            let yy = iy + row as isize;
            let mut previous = (1. - a) * ((1. - b) * pixel(ix, yy) + b * pixel(ix, yy + 1));
            for col in 0..side {
                let xx = ix + col as isize + 1;
                let next = a12 * pixel(xx, yy) + a22 * pixel(xx, yy + 1);
                output[row * side + col] = previous + next;
                previous = (f64::from(next) * scale) as f32;
            }
        }
    } else {
        let weights = [(1. - a) * (1. - b), a * (1. - b), (1. - a) * b, a * b];
        for row in 0..side {
            for col in 0..side {
                let xx = ix + col as isize;
                let yy = iy + row as isize;
                output[row * side + col] = if xx < 0 || xx >= width as isize - 1 {
                    pixel(xx, yy) * (1. - b) + pixel(xx, yy + 1) * b
                } else {
                    pixel(xx, yy) * weights[0]
                        + pixel(xx + 1, yy) * weights[1]
                        + pixel(xx, yy + 1) * weights[2]
                        + pixel(xx + 1, yy + 1) * weights[3]
                };
            }
        }
    }
}

pub(super) fn refine(
    image: &[u8],
    width: usize,
    height: usize,
    corners: &mut [Point2f],
    window: usize,
) -> Result<()> {
    if window == 0
        || width < 2 * window + 5
        || height < 2 * window + 5
        || image.len() != width * height
    {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "Invalid subpixel image or window",
        ));
    }
    let side = 2 * window + 1;
    let stride = side + 2;
    let mut mask = vec![0_f32; side * side];
    for row in 0..side {
        let y = (row as f32 - window as f32) / window as f32;
        let vertical = (-y * y).exp();
        for col in 0..side {
            let x = (col as f32 - window as f32) / window as f32;
            mask[row * side + col] = vertical * (-x * x).exp();
        }
    }
    let inside = |p: Point2f| p.x >= 0. && p.y >= 0. && p.x < width as f32 && p.y < height as f32;
    let mut patch = vec![0_f32; stride * stride];
    for corner in corners {
        if !inside(*corner) {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "Subpixel corner outside image",
            ));
        }
        let initial = *corner;
        let mut current = initial;
        for _ in 0..100 {
            sample(image, width, height, current, stride, &mut patch);
            let (mut a, mut b, mut c, mut rhs_x, mut rhs_y) = (0., 0., 0., 0., 0.);
            for row in 0..side {
                for col in 0..side {
                    let index = (row + 1) * stride + col + 1;
                    let dx = f64::from(patch[index + 1] - patch[index - 1]);
                    let dy = f64::from(patch[index + stride] - patch[index - stride]);
                    let weight = f64::from(mask[row * side + col]);
                    let xx = dx * dx * weight;
                    let xy = dx * dy * weight;
                    let yy = dy * dy * weight;
                    let px = col as f64 - window as f64;
                    let py = row as f64 - window as f64;
                    a += xx;
                    b += xy;
                    c += yy;
                    rhs_x += xx * px + xy * py;
                    rhs_y += xy * px + yy * py;
                }
            }
            let determinant = a * c - b * b;
            if determinant.abs() <= f64::EPSILON * f64::EPSILON {
                break;
            }
            let scale = 1. / determinant;
            let next = Point2f::new(
                (f64::from(current.x) + c * scale * rhs_x - b * scale * rhs_y) as f32,
                (f64::from(current.y) - b * scale * rhs_x + a * scale * rhs_y) as f32,
            );
            let dx = next.x - current.x;
            let dy = next.y - current.y;
            let error = dx * dx + dy * dy;
            if !inside(next) {
                break;
            }
            current = next;
            if f64::from(error) <= 0.01 * 0.01 {
                break;
            }
        }
        if (current.x - initial.x).abs() > window as f32
            || (current.y - initial.y).abs() > window as f32
        {
            current = initial;
        }
        *corner = current;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture_image(fixture: &serde_json::Value) -> (Vec<u8>, usize, usize) {
        let width = fixture["width"].as_u64().unwrap() as usize;
        let height = fixture["height"].as_u64().unwrap() as usize;
        let origin: [usize; 2] = serde_json::from_value(fixture["patch_origin"].clone()).unwrap();
        let size: [usize; 2] = serde_json::from_value(fixture["patch_size"].clone()).unwrap();
        let patch: Vec<u8> = serde_json::from_value(fixture["pixels"].clone()).unwrap();
        let mut image = vec![0; width * height];
        for row in 0..size[1] {
            let begin = (origin[1] + row) * width + origin[0];
            image[begin..begin + size[0]]
                .copy_from_slice(&patch[row * size[0]..(row + 1) * size[0]]);
        }
        (image, width, height)
    }
    #[test]
    fn original_cpp_real_cam3_corner_with_identical_initial_point() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/reference/apriltag_image_subpixel.json"
        ))
        .unwrap();
        let (image, width, height) = fixture_image(&fixture);
        let initial: [f32; 2] = serde_json::from_value(fixture["initial"].clone()).unwrap();
        let expected: [f32; 2] = serde_json::from_value(fixture["refined"].clone()).unwrap();
        let mut point = [Point2f::new(initial[0], initial[1])];
        refine(
            &image,
            width,
            height,
            &mut point,
            fixture["window"].as_u64().unwrap() as usize,
        )
        .unwrap();
        assert_eq!(point[0].x.to_bits(), expected[0].to_bits());
        assert_eq!(point[0].y.to_bits(), expected[1].to_bits());
    }
    #[test]
    fn original_cpp_quality_boundary_preserves_both_input_profiles() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/reference/apriltag_image_quality.json"
        ))
        .unwrap();
        let (image, width, height) = fixture_image(&fixture);
        for case in fixture["cases"].as_array().unwrap() {
            let initial: [f32; 2] = serde_json::from_value(case["initial"].clone()).unwrap();
            let expected: [f32; 2] = serde_json::from_value(case["refined"].clone()).unwrap();
            let mut point = [Point2f::new(initial[0], initial[1])];
            refine(&image, width, height, &mut point, 3).unwrap();
            assert_eq!(point[0].x.to_bits(), expected[0].to_bits());
            assert_eq!(point[0].y.to_bits(), expected[1].to_bits());
            let displacement = (f64::from(point[0].x) - f64::from(initial[0])).powi(2)
                + (f64::from(point[0].y) - f64::from(initial[1])).powi(2);
            assert_eq!(
                displacement <= fixture["radius"].as_f64().unwrap().powi(2),
                case["accepted"].as_bool().unwrap()
            );
        }
    }
    #[test]
    fn original_cpp_border_and_fractional_sampling() {
        let fixture: serde_json::Value =
            serde_json::from_str(include_str!("../../tests/reference/apriltag.json")).unwrap();
        let fixture = &fixture[0]["subpixel"];
        let pixels: Vec<u8> = serde_json::from_value(fixture["pixels"].clone()).unwrap();
        let initial: Vec<[f32; 2]> = serde_json::from_value(fixture["initial"].clone()).unwrap();
        let expected: Vec<[f32; 2]> = serde_json::from_value(fixture["refined"].clone()).unwrap();
        let mut points: Vec<_> = initial.iter().map(|p| Point2f::new(p[0], p[1])).collect();
        refine(&pixels, 24, 24, &mut points, 3).unwrap();
        for (actual, expected) in points.iter().zip(expected) {
            assert!(
                (actual.x - expected[0]).abs() < 2e-5,
                "x: {actual:?} vs {expected:?}"
            );
            assert!(
                (actual.y - expected[1]).abs() < 2e-5,
                "y: {actual:?} vs {expected:?}"
            );
        }
    }
}
