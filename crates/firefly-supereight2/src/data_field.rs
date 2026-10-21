/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Occupancy values and their measurement weight, in log-odds space.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OccupancyField {
    pub occupancy: f32,
    pub weight: f32,
    pub observed: bool,
}
impl OccupancyField {
    pub const NORMALS_ALONG_GRADIENT: bool = false;
    pub const SURFACE_BOUNDARY: f32 = 0.0;
    pub const MIN_OCCUPANCY: f32 = -100.0;
    pub const MAX_OCCUPANCY: f32 = 100.0;
    #[must_use]
    pub fn valid(&self) -> bool {
        self.weight != 0.0
    }
    pub fn update(&mut self, occupancy: f32, max_weight: f32) -> bool {
        if self.weight < max_weight {
            self.weight += 1.0;
        }
        self.occupancy = (self.occupancy * (self.weight - 1.0) + occupancy) / self.weight;
        self.observed = true;
        true
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UncertaintyModel {
    Linear,
    Quadratic,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OccupancyConfig {
    pub k_sigma: f32,
    pub sigma_min_factor: f32,
    pub sigma_max_factor: f32,
    pub k_tau: f32,
    pub tau_min_factor: f32,
    pub tau_max_factor: f32,
    pub log_odd_min: f32,
    pub log_odd_max: f32,
    pub max_weight: f32,
    pub fs_integr_scale: i32,
    pub uncertainty_model: UncertaintyModel,
}
impl Default for OccupancyConfig {
    fn default() -> Self {
        let log_odd_min = -5.015f32;
        Self {
            k_sigma: 0.052,
            sigma_min_factor: 1.5,
            sigma_max_factor: 6.0,
            k_tau: 0.026,
            tau_min_factor: 6.0,
            tau_max_factor: 16.0,
            log_odd_min,
            log_odd_max: 5.015,
            max_weight: (OccupancyField::MIN_OCCUPANCY / (0.97f32 * log_odd_min))
                .abs()
                .floor(),
            fs_integr_scale: 1,
            uncertainty_model: UncertaintyModel::Linear,
        }
    }
}
impl OccupancyConfig {
    pub fn recompute_max_weight(&mut self) {
        self.max_weight = (OccupancyField::MIN_OCCUPANCY / (0.97f32 * self.log_odd_min))
            .abs()
            .floor();
    }
}
