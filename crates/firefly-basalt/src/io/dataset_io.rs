/*
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
//! Translation of `include/basalt/io/dataset_io.h` at official Basalt
//! revision `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
use firefly_calibration::image::image::ManagedImage;
use firefly_error::Result;
use nalgebra::{Isometry3, Vector2, Vector3};
use serde::{Deserialize, Serialize};
use std::{path::Path, sync::Arc};

#[derive(Clone, Debug, Default)]
pub struct ImageData {
    pub img: Option<Arc<ManagedImage<u16>>>,
    pub exposure: f64,
}
#[derive(Clone, Debug, Default)]
pub struct Observations {
    pub pos: Vec<Vector2<f64>>,
    pub id: Vec<i32>,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct GyroData {
    #[serde(rename = "value0")]
    pub timestamp_ns: i64,
    #[serde(rename = "value1", with = "vector3_archive")]
    pub data: Vector3<f64>,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct AccelData {
    #[serde(rename = "value0")]
    pub timestamp_ns: i64,
    #[serde(rename = "value1", with = "vector3_archive")]
    pub data: Vector3<f64>,
}
#[derive(Clone, Debug, Default)]
pub struct PoseData {
    pub timestamp_ns: i64,
    pub data: Isometry3<f64>,
}
#[derive(Clone, Debug, Default)]
pub struct MocapPoseData {
    pub timestamp_ns: i64,
    pub data: Isometry3<f64>,
}
#[derive(Clone, Debug, Default)]
pub struct AprilgridCornersData {
    pub timestamp_ns: i64,
    pub cam_id: i32,
    pub corner_pos: Vec<Vector2<f64>>,
    pub corner_id: Vec<i32>,
}

/// The upstream dataset abstraction, retaining mutable timestamp access.
pub trait VioDataset: Send {
    fn num_cams(&self) -> usize;
    fn image_timestamps(&mut self) -> &mut Vec<i64>;
    fn accel_data(&self) -> &[AccelData];
    fn gyro_data(&self) -> &[GyroData];
    fn gt_timestamps(&self) -> &[i64];
    fn gt_pose_data(&self) -> &[Isometry3<f64>];
    fn mocap_to_imu_offset_ns(&self) -> i64;
    /// # Errors
    /// Returns an error when requested image data cannot be loaded.
    fn image_data(&mut self, timestamp_ns: i64) -> Result<Vec<ImageData>>;
}
pub type VioDatasetPtr = Arc<std::sync::Mutex<dyn VioDataset>>;

/// Reader lifecycle from the original dataset interface.
pub trait DatasetIoInterface: Send {
    /// # Errors
    /// Returns an error when the dataset cannot be read.
    fn read(&mut self, path: &Path) -> Result<()>;
    fn reset(&mut self);
    fn data(&self) -> Option<VioDatasetPtr>;
}
pub type DatasetIoInterfacePtr = Box<dyn DatasetIoInterface>;

/// Original `src/io/dataset_io.cpp` factory.
pub struct DatasetIoFactory;
impl DatasetIoFactory {
    /// # Errors
    /// Returns an error for an unsupported dataset type.
    pub fn get_dataset_io(
        dataset_type: &str,
        load_mocap_as_gt: bool,
    ) -> Result<DatasetIoInterfacePtr> {
        match dataset_type {
            "euroc" => Ok(Box::new(super::dataset_io_euroc::EurocIo::new(
                load_mocap_as_gt,
            ))),
            "bag" => Ok(Box::new(super::dataset_io_rosbag::RosbagIo::default())),
            "uzh" => Ok(Box::new(super::dataset_io_uzh::UzhIo::default())),
            "kitti" => Ok(Box::new(super::dataset_io_kitti::KittiIo::default())),
            _ => Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "Unsupported dataset type",
            )
            .with_operation("basalt.dataset.factory")
            .with_context("dataset_type", dataset_type)),
        }
    }
}

mod vector3_archive {
    use super::Vector3;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    pub fn serialize<S: Serializer>(
        value: &Vector3<f64>,
        serializer: S,
    ) -> std::result::Result<S::Ok, S::Error> {
        [value.x, value.y, value.z].serialize(serializer)
    }
    pub fn deserialize<'de, D: Deserializer<'de>>(
        deserializer: D,
    ) -> std::result::Result<Vector3<f64>, D::Error> {
        <[f64; 3]>::deserialize(deserializer).map(Vector3::from)
    }
}
