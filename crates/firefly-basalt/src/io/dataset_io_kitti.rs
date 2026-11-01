// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/io/dataset_io_kitti.h` at
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

use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion};
pub struct KittiVioDataset {
    path: PathBuf,
    records: Records,
    image_path: HashMap<i64, String>,
}
impl VioDataset for KittiVioDataset {
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
        let filename = self.image_path.get(&timestamp_ns).ok_or_else(|| {
            common::failure(&self.path, "Unknown image timestamp")
                .with_context("timestamp_ns", timestamp_ns)
        })?;
        (0..2)
            .map(|camera| {
                common::image(
                    &self.path.join(format!("image_{camera}")).join(filename),
                    true,
                )
            })
            .collect()
    }
}
#[derive(Default)]
pub struct KittiIo {
    data: Option<VioDatasetPtr>,
}
impl DatasetIoInterface for KittiIo {
    fn read(&mut self, path: &Path) -> Result<()> {
        let mut data = KittiVioDataset {
            path: path.to_owned(),
            records: Records::default(),
            image_path: HashMap::new(),
        };
        let file = path.join("times.txt");
        for (number, line) in common::lines(&file)? {
            let values = common::fields(&line, &file, number, 1, false)?;
            let timestamp =
                common::seconds_ns(common::parse(values[0], &file, number)?, &file, number)?;
            let filename = format!("{:06}.png", data.records.image_timestamps.len());
            data.records.image_timestamps.push(timestamp);
            data.image_path.insert(timestamp, filename);
        }
        let file = path.join("poses.txt");
        if file.exists() {
            for (index, (number, line)) in common::lines(&file)?.into_iter().enumerate() {
                let values = common::fields(&line, &file, number, 12, false)?;
                let values = common::numbers::<12>(&values, &file, number)?;
                let timestamp = *data.records.image_timestamps.get(index).ok_or_else(|| {
                    common::failure(&file, "More poses than image timestamps")
                        .with_context("line", number)
                })?;
                let rot = Matrix3::from_row_slice(&[
                    values[0], values[1], values[2], values[4], values[5], values[6], values[8],
                    values[9], values[10],
                ]);
                let q =
                    UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rot));
                let pose = Isometry3::from_parts(
                    Translation3::new(values[3], values[7], values[11]),
                    UnitQuaternion::new_normalize(q.into_inner()),
                );
                data.records.gt_timestamps.push(timestamp);
                data.records.gt_poses.push(pose);
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
