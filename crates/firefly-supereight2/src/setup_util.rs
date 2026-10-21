/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Compile-time map representation choices and voxel-centre sampling offset.
use nalgebra::Vector3;
pub const SAMPLE_OFFSET_FRAC: Vector3<f32> = Vector3::new(0.5, 0.5, 0.5);
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Field {
    Tsdf,
    Occupancy,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Colour {
    Off,
    On,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Id {
    Off,
    On,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Res {
    Single,
    Multi,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Integ {
    Simple,
    Lidar,
    PinholeCamera,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Safe {
    On,
    Off,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AllocationMethod {
    Raycasting,
    VoxelCarving,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Representation {
    Surface,
    Freespace,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Sort {
    SmallToLarge,
    LargeToSmall,
}
