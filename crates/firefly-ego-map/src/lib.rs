// SPDX-License-Identifier: GPL-3.0-only
//! Rust translation of EGO-Planner-v2's local depth/occupancy mapping.
//!
//! Source: ZJU-FAST-Lab/EGO-Planner-v2, commit
//! `9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010`, `swarm-playground/main_ws/src/planner/plan_env`.
//! Callbacks accept already paired depth/pose data. The caller drives the occupancy
//! timer at 32 ms and the enabled fading timer at 500 ms, in the same clock domain.
//! ROS transport and visualization are outside this numerical module.

mod config;
mod grid;
mod raycast;

pub use config::{DepthImage, MappingParameters};
pub use grid::GridMap;

/// Fixed C++ source revision used by the translation and differential tests.
pub const UPSTREAM_REVISION: &str = "9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010";
