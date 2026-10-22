/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Occupancy-only data and ordered mean/min/max propagation.
use crate::data_field::{OccupancyConfig, OccupancyField};
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OccupancyData {
    pub field: OccupancyField,
}
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DataConfig {
    pub field: OccupancyConfig,
}
impl OccupancyData {
    #[must_use]
    pub fn valid(&self) -> bool {
        self.field.valid()
    }
    #[must_use]
    pub fn get_field(&self) -> f32 {
        self.field.occupancy * self.field.weight
    }
    #[must_use]
    pub fn is_inside(&self) -> bool {
        self.get_field() > OccupancyField::SURFACE_BOUNDARY
    }
    pub fn set_invalid(&mut self) {
        *self = Self::default();
    }
}
#[must_use]
pub fn is_valid(data: &OccupancyData) -> bool {
    data.valid()
}
#[must_use]
pub fn get_field(data: &OccupancyData) -> f32 {
    data.get_field()
}
#[must_use]
pub fn is_inside(data: &OccupancyData) -> bool {
    data.is_inside()
}
pub fn set_invalid(data: &mut OccupancyData) {
    data.set_invalid();
}
pub fn up_prop_mean(parent: &mut OccupancyData, children: &[OccupancyData; 8]) -> usize {
    let (mut occupancy, mut weight, mut count) = (0.0f32, 0.0f32, 0usize);
    for child in children {
        if child.field.valid() {
            count += 1;
            occupancy += child.field.occupancy;
            weight += child.field.weight;
        }
    }
    if count != 0 {
        parent.field.occupancy = occupancy / count as f32;
        parent.field.weight = (weight / count as f32).ceil();
        parent.field.observed = false;
    }
    count
}
fn up_prop_best(
    parent: &mut OccupancyData,
    children: &[OccupancyData; 8],
    worst: f32,
    better: impl Fn(f32, f32) -> bool,
) -> usize {
    let (mut best, mut best_index, mut count, mut observed) = (worst, 0usize, 0usize, 0usize);
    for (index, child) in children.iter().enumerate() {
        if child.field.valid() {
            count += 1;
            observed += usize::from(child.field.observed);
            let occupancy = child.get_field();
            if better(occupancy, best) {
                best = occupancy;
                best_index = index;
            }
        }
    }
    if count != 0 {
        *parent = children[best_index];
        parent.field.observed = observed == 8;
    }
    count
}
pub fn up_prop_min(parent: &mut OccupancyData, children: &[OccupancyData; 8]) -> usize {
    up_prop_best(parent, children, f32::MAX, |a, b| a < b)
}
pub fn up_prop_max(parent: &mut OccupancyData, children: &[OccupancyData; 8]) -> usize {
    up_prop_best(parent, children, f32::MIN, |a, b| a > b)
}

impl DataConfig {
    /// # Errors
    /// Returns an error if an occupancy parameter has an incompatible TOML type.
    pub fn read_toml(&mut self, document: &toml::Value) -> firefly_error::Result<()> {
        let Some(t) = crate::sensor::config_section(document, "data")? else {
            return Ok(());
        };
        let c = &mut self.field;
        for (name, value) in [
            ("k_sigma", &mut c.k_sigma),
            ("sigma_min_factor", &mut c.sigma_min_factor),
            ("sigma_max_factor", &mut c.sigma_max_factor),
            ("k_tau", &mut c.k_tau),
            ("tau_min_factor", &mut c.tau_min_factor),
            ("tau_max_factor", &mut c.tau_max_factor),
            ("log_odd_min", &mut c.log_odd_min),
            ("log_odd_max", &mut c.log_odd_max),
        ] {
            crate::sensor::config_float(t, name, value)?;
        }
        crate::sensor::config_integer(t, "fs_integr_scale", &mut c.fs_integr_scale)?;
        if let Some(model) = t.get("uncertainty_model") {
            match model
                .as_str()
                .ok_or_else(|| crate::image::invalid("uncertainty_model must be a string"))?
            {
                "linear" => c.uncertainty_model = crate::data_field::UncertaintyModel::Linear,
                "quadratic" => c.uncertainty_model = crate::data_field::UncertaintyModel::Quadratic,
                _ => {}
            }
        }
        c.recompute_max_weight();
        Ok(())
    }
}
impl std::fmt::Display for DataConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self.field)
    }
}
