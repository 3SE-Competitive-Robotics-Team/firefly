// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/io/dataset_io_uzh.h` at
//! `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
use super::{
    dataset_common::{self as common, Records},
    dataset_io::{AccelData, DatasetIoInterface, GyroData, ImageData, VioDataset, VioDatasetPtr},
};
use firefly_error::Result;
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
};

use nalgebra::{Isometry3, Vector3};
pub struct UzhVioDataset {
    path: PathBuf,
    records: Records,
    image_path: [HashMap<i64, String>; 2],
}
impl VioDataset for UzhVioDataset {
    fn num_cams(&self) -> usize {
        2
    }
    fn image_timestamps(&mut self) -> &mut Vec<i64> {
        &mut self.records.image_timestamps
    }
    fn accel_data(&self) -> &[AccelData] {
        &self.records.accel
    }
    fn gyro_data(&self) -> &[GyroData] {
        &self.records.gyro
    }
    fn gt_timestamps(&self) -> &[i64] {
        &self.records.gt_timestamps
    }
    fn gt_pose_data(&self) -> &[Isometry3<f64>] {
        &self.records.gt_poses
    }
    fn mocap_to_imu_offset_ns(&self) -> i64 {
        0
    }
    fn image_data(&mut self, timestamp_ns: i64) -> Result<Vec<ImageData>> {
        (0..2)
            .map(|camera| {
                let filename = self.image_path[camera].get(&timestamp_ns).ok_or_else(|| {
                    common::failure(&self.path, "Missing synchronized image timestamp")
                        .with_context("camera", camera)
                        .with_context("timestamp_ns", timestamp_ns)
                })?;
                common::image(&self.path.join(filename), false)
            })
            .collect()
    }
}
#[derive(Default)]
pub struct UzhIo {
    data: Option<VioDatasetPtr>,
}
impl DatasetIoInterface for UzhIo {
    fn read(&mut self, path: &Path) -> Result<()> {
        let mut data = UzhVioDataset {
            path: path.to_owned(),
            records: Records::default(),
            image_path: std::array::from_fn(|_| HashMap::new()),
        };
        for (camera, name) in ["left_images.txt", "right_images.txt"]
            .into_iter()
            .enumerate()
        {
            let file = path.join(name);
            for (number, line) in common::lines(&file)? {
                let values = common::fields(&line, &file, number, 3, false)?;
                let timestamp =
                    common::seconds_ns(common::parse(values[1], &file, number)?, &file, number)?;
                if camera == 0 {
                    data.records.image_timestamps.push(timestamp);
                }
                data.image_path[camera].insert(timestamp, values[2].to_owned());
            }
        }
        let file = path.join("imu.txt");
        for (number, line) in common::lines(&file)? {
            let values = common::fields(&line, &file, number, 8, false)?;
            let timestamp =
                common::seconds_ns(common::parse(values[1], &file, number)?, &file, number)?;
            data.records.gyro.push(GyroData {
                timestamp_ns: timestamp,
                data: Vector3::from(common::numbers::<3>(&values[2..], &file, number)?),
            });
            data.records.accel.push(AccelData {
                timestamp_ns: timestamp,
                data: Vector3::from(common::numbers::<3>(&values[5..], &file, number)?),
            });
        }
        let file = path.join("groundtruth.txt");
        if file.exists() {
            for (number, line) in common::lines(&file)? {
                let values = common::fields(&line, &file, number, 9, false)?;
                let timestamp =
                    common::seconds_ns(common::parse(values[1], &file, number)?, &file, number)?;
                let position = common::numbers::<3>(&values[2..], &file, number)?;
                let xyzw = common::numbers::<4>(&values[5..], &file, number)?;
                data.records.gt_timestamps.push(timestamp);
                data.records.gt_poses.push(common::pose(
                    position,
                    [xyzw[3], xyzw[0], xyzw[1], xyzw[2]],
                    &file,
                    number,
                )?);
            }
        }
        self.data = Some(Arc::new(Mutex::new(data)));
        Ok(())
    }
    fn reset(&mut self) {
        self.data = None;
    }
    fn data(&self) -> Option<VioDatasetPtr> {
        self.data.clone()
    }
}
