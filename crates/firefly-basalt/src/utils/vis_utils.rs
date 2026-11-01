// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original `utils/vis_utils.h` geometry and colors, without an OpenGL context.
//! The caller submits the returned primitives to its renderer.
use nalgebra::{Matrix4, Vector3};

pub const CAM_COLOR: [u8; 3] = [250, 0, 26];
pub const STATE_COLOR: [u8; 3] = [250, 0, 26];
pub const POSE_COLOR: [u8; 3] = [0, 50, 255];
pub const GT_COLOR: [u8; 3] = [0, 171, 47];

#[derive(Clone, Debug)]
pub struct CameraDrawing {
    pub transform: Matrix4<f64>,
    pub line_width: f32,
    pub color: [u8; 3],
    pub lines: [Vector3<f32>; 16],
}

/// Return the exact local camera vertices and render state used by the source.
#[must_use]
pub fn render_camera(
    transform: Matrix4<f64>,
    line_width: f32,
    color: [u8; 3],
    size_factor: f32,
) -> CameraDrawing {
    let vertex = |x: f32, y: f32| {
        Vector3::new(
            size_factor * (x - 320.) / 500.,
            size_factor * (y - 240.) / 500.,
            size_factor,
        )
    };
    let a = vertex(0., 0.);
    let b = vertex(0., 479.);
    let c = vertex(639., 479.);
    let d = vertex(639., 0.);
    let zero = Vector3::zeros();
    CameraDrawing {
        transform,
        line_width,
        color,
        lines: [zero, a, zero, b, zero, c, zero, d, d, c, c, b, b, a, a, d],
    }
}

/// Source piecewise color ramp, retaining out-of-range and IEEE NaN behavior.
#[must_use]
pub fn getcolor(position: f32, num_positions: f32) -> [f32; 3] {
    let increment = (4.0_f64 / f64::from(num_positions)) as f32;
    let x = position * increment;
    let mut red = 0.;
    let mut green = 0.;
    let mut blue = 0.;
    if (0. ..=1.).contains(&x) || (5. ..=6.).contains(&x) {
        red = 1.;
    } else if (4. ..=5.).contains(&x) {
        red = x - 4.;
    } else if (1. ..=2.).contains(&x) {
        red = 1. - (x - 1.);
    }
    if (1. ..=3.).contains(&x) {
        green = 1.;
    } else if (0. ..=1.).contains(&x) {
        green = x;
    } else if (3. ..=4.).contains(&x) {
        green = 1. - (x - 3.);
    }
    if (3. ..=5.).contains(&x) {
        blue = 1.;
    } else if (2. ..=3.).contains(&x) {
        blue = x - 2.;
    } else if (5. ..=6.).contains(&x) {
        blue = 1. - (x - 5.);
    }
    [red, green, blue]
}
