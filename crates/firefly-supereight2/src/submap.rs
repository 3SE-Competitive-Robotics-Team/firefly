/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! A shared map and the pose of its frame in world coordinates.
use crate::map::Map;
use nalgebra::IsometryMatrix3;
use std::sync::{Arc, RwLock};
pub struct Submap {
    pub map: Option<Arc<RwLock<Map>>>,
    pub t_wk: IsometryMatrix3<f32>,
}
impl Default for Submap {
    fn default() -> Self {
        Self {
            map: None,
            t_wk: IsometryMatrix3::identity(),
        }
    }
}
