/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Primitive representations used by occupancy maps.
pub type Key = u64;
pub type Code = u64;
pub type KeyScale = u64;
pub type Index = u32;
pub type Field = f32;
pub type Weight = f32;
pub type Timestamp = i32;
pub type FieldVector = nalgebra::Vector3<f32>;
