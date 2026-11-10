// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of `include/basalt/io/dataset_io_euroc.h` at
//! `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.
use super::{
    dataset_common::{self as common, Records},
    dataset_io::{AccelData, DatasetIoInterface, GyroData, ImageData, VioDataset, VioDatasetPtr},
};
use firefly_error::Result;
use nalgebra::{Isometry3, Vector3};
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
};

pub struct EurocVioDataset {
    path: PathBuf,
    records: Records,
    image_path: [HashMap<i64, String>; 2],
    exposure_times: [HashMap<i64, f64>; 2],
}
impl VioDataset for EurocVioDataset {
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
        let mut result = Vec::with_capacity(2);
        for camera in 0..2 {
            let filename = self.image_path[camera].get(&timestamp_ns).ok_or_else(|| {
                common::failure(&self.path, "Unknown image timestamp")
                    .with_context("timestamp_ns", timestamp_ns)
            })?;
            let path = self.path.join(format!("cam{camera}/data")).join(filename);
            let mut image = common::image(&path, false)?;
            if image.img.is_some() {
                image.exposure = *self.exposure_times[camera]
                    .get(&timestamp_ns)
                    .unwrap_or(&0.);
            }
            result.push(image);
        }
        Ok(result)
    }
}

pub struct EurocIo {
    data: Option<VioDatasetPtr>,
    load_mocap_as_gt: bool,
}
impl EurocIo {
    #[must_use]
    pub const fn new(load_mocap_as_gt: bool) -> Self {
        Self {
            data: None,
            load_mocap_as_gt,
        }
    }
}
impl EurocIo {
    /// Reads an ASL sensor directory directly, retaining native camera dimensions.
    /// # Errors
    /// Rejects malformed metadata and unmatched stereo indices.
    pub fn read_asl(&mut self, path: &Path) -> Result<()> {
        self.data = None;
        let mut data = self.read_root(path)?;
        let file = path.join("cam1/data.csv");
        data.image_path[1].clear();
        let mut right_times = Vec::new();
        for (number, line) in common::lines(&file)? {
            let fields = common::fields(&line, &file, number, 2, true)?;
            let timestamp = common::parse(fields[0], &file, number)?;
            right_times.push(timestamp);
            data.image_path[1].insert(timestamp, fields[1].to_owned());
        }
        if right_times != data.records.image_timestamps
            || right_times.windows(2).any(|pair| pair[0] >= pair[1])
        {
            return Err(common::failure(
                path,
                "ASL stereo timestamps must match and increase strictly",
            ));
        }
        self.data = Some(Arc::new(Mutex::new(data)));
        Ok(())
    }

    fn read_root(&self, path: &Path) -> Result<EurocVioDataset> {
        let mut data = EurocVioDataset {
            path: path.to_owned(),
            records: Records::default(),
            image_path: std::array::from_fn(|_| HashMap::new()),
            exposure_times: std::array::from_fn(|_| HashMap::new()),
        };
        let file = path.join("cam0/data.csv");
        for (number, line) in common::lines(&file)? {
            let values = common::fields(&line, &file, number, 2, true)?;
            let timestamp = common::parse(values[0], &file, number)?;
            data.records.image_timestamps.push(timestamp);
            data.image_path[0].insert(
                timestamp,
                values[1]
                    .split_whitespace()
                    .next()
                    .unwrap_or_default()
                    .to_owned(),
            );
        }
        let [left_paths, right_paths] = &mut data.image_path;
        right_paths.clone_from(left_paths);
        let file = path.join("imu0/data.csv");
        for (number, line) in common::lines(&file)? {
            let values = common::fields(&line, &file, number, 7, true)?;
            let timestamp = common::parse::<u64>(values[0], &file, number)? as i64;
            let gyro = Vector3::from(common::numbers::<3>(&values[1..], &file, number)?);
            let accel = Vector3::from(common::numbers::<3>(&values[4..], &file, number)?);
            data.records.gyro.push(GyroData {
                timestamp_ns: timestamp,
                data: gyro,
            });
            data.records.accel.push(AccelData {
                timestamp_ns: timestamp,
                data: accel,
            });
        }
        let state = path.join("state_groundtruth_estimate0/data.csv");
        let gt = path.join("gt/data.csv");
        let mocap = path.join("mocap0/data.csv");
        let groundtruth = if !self.load_mocap_as_gt && state.exists() {
            Some(state)
        } else if !self.load_mocap_as_gt && gt.exists() {
            Some(gt)
        } else if mocap.exists() {
            Some(mocap)
        } else {
            None
        };
        if let Some(file) = groundtruth {
            for (number, line) in common::lines(&file)? {
                let values = common::fields(&line, &file, number, 8, true)?;
                let timestamp = common::parse::<u64>(values[0], &file, number)? as i64;
                let position = common::numbers::<3>(&values[1..], &file, number)?;
                let quaternion = common::numbers::<4>(&values[4..], &file, number)?;
                data.records.gt_timestamps.push(timestamp);
                data.records
                    .gt_poses
                    .push(common::pose(position, quaternion, &file, number)?);
            }
        }
        for camera in 0..2 {
            let file = path.join(format!("cam{camera}/exposure.csv"));
            if file.exists() {
                for (number, line) in common::lines(&file)? {
                    let values = common::fields(&line, &file, number, 2, true)?;
                    let timestamp = common::parse(values[0], &file, number)?;
                    let exposure = common::parse::<i64>(values[1], &file, number)? as f64 * 1e-9;
                    data.exposure_times[camera].insert(timestamp, exposure);
                }
            }
        }
        Ok(data)
    }
}
impl DatasetIoInterface for EurocIo {
    fn read(&mut self, path: &Path) -> Result<()> {
        self.data = Some(Arc::new(Mutex::new(self.read_root(&path.join("mav0"))?)));
        Ok(())
    }
    fn reset(&mut self) {
        self.data = None;
    }
    fn data(&self) -> Option<VioDatasetPtr> {
        self.data.clone()
    }
}
