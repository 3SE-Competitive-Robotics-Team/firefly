// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See LICENSE for the complete notice and advertising acknowledgement.
//! Original MathUtil.h/MathUtil.cc.
#[must_use]
pub fn distance_2d(a: (f32, f32), b: (f32, f32)) -> f32 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}
#[must_use]
pub fn mod_2pi(value: f32) -> f32 {
    let period = 2. * std::f32::consts::PI;
    let quotient = (value.abs() * (1. / period) + 0.5) as i32;
    let remainder = value.abs() - quotient as f32 * period;
    if value < 0. { -remainder } else { remainder }
}
#[must_use]
pub fn mod_2pi_near(reference: f32, value: f32) -> f32 {
    reference + mod_2pi(value - reference)
}
#[must_use]
pub fn fast_atan2(y: f64, x: f64) -> f64 {
    let first = std::f64::consts::FRAC_PI_4;
    let absolute_y = y.abs() + 1e-10;
    let angle = if x >= 0. {
        first - first * (x - absolute_y) / (x + absolute_y)
    } else {
        3. * first - first * (x + absolute_y) / (absolute_y - x)
    };
    if y < 0. { -angle } else { angle }
}
