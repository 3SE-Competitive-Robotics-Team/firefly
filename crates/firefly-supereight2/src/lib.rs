// SPDX-License-Identifier: BSD-3-Clause

//! Supereight2 multiresolution occupancy mapping for pinhole depth images.
//!
//! The C++ reference snapshot retains its original hierarchy; Rust modules live in `src/`.
//! Camera projection geometry is provided by `firefly_srl_projection`.

pub mod allocator;
pub mod block;
pub mod data;
pub mod data_field;
pub mod dense_pooling_image;
pub mod edge_tables;
pub mod eigen_utils;
pub mod fetcher;
pub mod image;
pub mod iterator;
pub mod key_util;
pub mod map;
pub mod map_integrator;
pub mod marching_cube;
pub mod math_util;
pub mod measurement;
pub mod mesh;
pub mod multires_ofusion_core;
pub mod multires_ofusion_updater;
pub mod node;
pub mod octant;
pub mod octant_util;
pub mod octree;
pub mod pinhole_camera;
pub mod preprocessor;
pub mod scale;
pub mod sensor;
pub mod setup_util;
pub mod submap;
pub mod type_util;
pub mod uncertainty;
pub mod visitor;
pub mod volume_carver;
