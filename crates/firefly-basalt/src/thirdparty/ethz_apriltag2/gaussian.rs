// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See docs/licenses/README.md for the complete notice and advertising acknowledgement.
//! Original Gaussian.h/Gaussian.cc.
#[must_use]
pub fn make_gaussian_filter(sigma: f32, count: usize) -> Vec<f32> {
    let mut filter = vec![0.; count];
    if sigma == 0. {
        filter[count / 2] = 1.;
        return filter;
    }
    let inverse = 1. / f64::from(2. * sigma * sigma);
    let mut sum = 0.0_f32;
    for (i, value) in filter.iter_mut().enumerate() {
        let offset = i as i32 - count as i32 / 2;
        *value = (-f64::from(offset * offset) * inverse).exp() as f32;
        sum += *value;
    }
    for value in &mut filter {
        *value /= sum;
    }
    filter
}
pub fn convolve_symmetric_centered(
    input: &[f32],
    offset: usize,
    length: usize,
    filter: &[f32],
    output: &mut [f32],
    output_offset: usize,
) {
    let half = filter.len() / 2;
    for i in half..filter.len() {
        let mut accumulator = 0.0_f64;
        for (j, &weight) in filter.iter().enumerate() {
            let value = if offset + i < j || offset + i >= length + j {
                input[offset]
            } else {
                input[offset + i - j]
            };
            accumulator += f64::from(value * weight);
        }
        output[output_offset + i - half] = accumulator as f32;
    }
    for i in filter.len()..length {
        let mut accumulator = 0.0_f64;
        for (j, &weight) in filter.iter().enumerate() {
            accumulator += f64::from(input[offset + i - j] * weight);
        }
        output[output_offset + i - half] = accumulator as f32;
    }
    for i in length..length + half {
        let mut accumulator = 0.0_f64;
        for (j, &weight) in filter.iter().enumerate() {
            let value = if offset + i >= length + j || offset + i < j {
                input[offset + length - 1]
            } else {
                input[offset + i - j]
            };
            accumulator += f64::from(value * weight);
        }
        output[output_offset + i - half] = accumulator as f32;
    }
}
