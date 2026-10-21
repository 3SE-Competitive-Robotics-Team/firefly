/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Pinhole depth observations and optional per-pixel uncertainty.
use crate::{image::Image, pinhole_camera::PinholeCamera};
use nalgebra::IsometryMatrix3;
#[derive(Clone, Copy)]
pub struct Measurement<'a> {
    pub image: &'a Image<f32>,
    pub sensor: &'a PinholeCamera,
    pub t_wc: IsometryMatrix3<f32>,
}
#[derive(Clone, Copy)]
pub struct Measurements<'a> {
    pub depth: Measurement<'a>,
    pub depth_sigma: Option<&'a Image<f32>>,
}
impl<'a> Measurements<'a> {
    #[must_use]
    pub fn new(depth: Measurement<'a>) -> Self {
        Self {
            depth,
            depth_sigma: None,
        }
    }
}
