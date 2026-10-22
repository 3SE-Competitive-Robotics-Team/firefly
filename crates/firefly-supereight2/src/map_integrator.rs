/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Pinhole multi-resolution occupancy depth integration.
use crate::{
    image::invalid, map::Map, measurement::Measurements, multires_ofusion_updater::Updater,
    octant::OctantId, uncertainty::depth_sigma, volume_carver::VolumeCarver,
};
use firefly_error::Result;
use std::collections::HashSet;
pub struct MapIntegrator<'a> {
    pub map: &'a mut Map,
}
impl<'a> MapIntegrator<'a> {
    pub fn new(map: &'a mut Map) -> Self {
        Self { map }
    }
    /// # Errors
    /// Returns an error for invalid depth/sigma images, unsupported pooling windows, invalid
    /// projections, or inconsistent tree/buffer state.
    pub fn integrate_depth(
        &mut self,
        timestamp: i32,
        measurements: &Measurements<'_>,
        updated: Option<&mut HashSet<OctantId>>,
    ) -> Result<()> {
        let generated = if measurements.depth_sigma.is_none() {
            Some(depth_sigma(
                measurements.depth.image,
                self.map.resolution,
                &self.map.data_config.field,
            )?)
        } else {
            None
        };
        let sigma = measurements
            .depth_sigma
            .or(generated.as_ref())
            .ok_or_else(|| invalid("sigma image unavailable"))?;
        let depth = measurements.depth.image;
        let sensor = measurements.depth.sensor;
        if depth.channels != 1
            || sigma.channels != 1
            || depth.cols != sensor.model.width
            || depth.rows != sensor.model.height
            || depth.cols != sigma.cols
            || depth.rows != sigma.rows
        {
            return Err(invalid("depth/sigma/sensor dimensions differ"));
        }
        let local = Measurements {
            depth: measurements.depth,
            depth_sigma: Some(sigma),
        };
        let allocation =
            VolumeCarver::new(self.map, sensor, depth, sigma, &local.depth.t_wc, timestamp)?
                .allocate()?;
        Updater::new(self.map, timestamp, &local)?.update(&allocation, updated)
    }
}
