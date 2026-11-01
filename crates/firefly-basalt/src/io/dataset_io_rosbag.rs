// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Translation of official `include/basalt/io/dataset_io_rosbag.h`.
//! ROS1 storage/decompression is delegated to `rosbag`; payloads retain the
//! original topic selection, timestamps, pixel conversion and GT ordering.
use super::{
    dataset_common::{self as common, Records},
    dataset_io::{AccelData, DatasetIoInterface, GyroData, ImageData, VioDataset, VioDatasetPtr},
};
use firefly_basalt_headers::image::image::ManagedImage;
use firefly_error::Result;
use nalgebra::{Isometry3, Vector3};
use rosbag::{ChunkRecord, IndexRecord, MessageRecord, RosBag};
use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
};

struct Payload<'a> {
    data: &'a [u8],
    path: &'a Path,
}
impl<'a> Payload<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8]> {
        if n > self.data.len() {
            return Err(common::failure(self.path, "Truncated ROS message"));
        }
        let (value, rest) = self.data.split_at(n);
        self.data = rest;
        Ok(value)
    }
    fn u32(&mut self) -> Result<u32> {
        let mut b = [0; 4];
        b.copy_from_slice(self.take(4)?);
        Ok(u32::from_le_bytes(b))
    }
    fn f64(&mut self) -> Result<f64> {
        let mut b = [0; 8];
        b.copy_from_slice(self.take(8)?);
        Ok(f64::from_le_bytes(b))
    }
    fn string(&mut self) -> Result<&'a str> {
        let n = self.u32()? as usize;
        std::str::from_utf8(self.take(n)?)
            .map_err(|e| common::failure(self.path, "Invalid ROS string").with_source(e))
    }
    fn header(&mut self) -> Result<(i64, &'a str)> {
        let _seq = self.u32()?;
        let sec = self.u32()?;
        let ns = self.u32()?;
        Ok((
            i64::from(sec) * 1_000_000_000 + i64::from(ns),
            self.string()?,
        ))
    }
    fn vector(&mut self) -> Result<[f64; 3]> {
        Ok([self.f64()?, self.f64()?, self.f64()?])
    }
    fn quaternion(&mut self) -> Result<[f64; 4]> {
        let x = self.f64()?;
        let y = self.f64()?;
        let z = self.f64()?;
        let w = self.f64()?;
        Ok([w, x, y, z])
    }
}
#[derive(Clone, Copy)]
struct ImageIndex {
    chunk: u64,
    ordinal: usize,
}
pub struct RosbagVioDataset {
    path: PathBuf,
    bag: RosBag,
    records: Records,
    images: BTreeMap<i64, Vec<Option<ImageIndex>>>,
    cameras: usize,
    offset: i64,
}
impl VioDataset for RosbagVioDataset {
    fn num_cams(&self) -> usize {
        self.cameras
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
        self.offset
    }
    fn image_data(&mut self, timestamp: i64) -> Result<Vec<ImageData>> {
        let mut output = vec![ImageData::default(); self.cameras];
        let Some(indices) = self.images.get(&timestamp) else {
            return Ok(output);
        };
        for (camera, index) in indices.iter().enumerate() {
            let Some(index) = index else { continue };
            let mut chunks = self.bag.chunk_records();
            chunks
                .seek(index.chunk)
                .map_err(|e| common::failure(&self.path, e.to_string()))?;
            let Some(ChunkRecord::Chunk(chunk)) = chunks
                .next()
                .transpose()
                .map_err(|e| common::failure(&self.path, e.to_string()))?
            else {
                return Err(common::failure(&self.path, "Image chunk is missing"));
            };
            let Some(MessageRecord::MessageData(message)) = chunk
                .messages()
                .nth(index.ordinal)
                .transpose()
                .map_err(|e| common::failure(&self.path, e.to_string()))?
            else {
                return Err(common::failure(
                    &self.path,
                    "Indexed image message is missing",
                ));
            };
            output[camera] = decode_image(message.data, &self.path)?;
        }
        Ok(output)
    }
}
fn decode_image(data: &[u8], path: &Path) -> Result<ImageData> {
    let mut p = Payload { data, path };
    let (_, frame) = p.header()?;
    let exposure = if frame.as_bytes().first().is_some_and(u8::is_ascii_digit) {
        let digits = frame.bytes().take_while(u8::is_ascii_digit).count();
        common::parse::<i64>(&frame[..digits], path, 0)? as f64 * 1e-9
    } else {
        -1.
    };
    let height = p.u32()? as usize;
    let width = p.u32()? as usize;
    let encoding = p.string()?;
    let _bigendian = p.take(1)?[0];
    let step = p.u32()? as usize;
    let count = p.u32()? as usize;
    let bytes = p.take(count)?;
    let pixel_bytes = match encoding {
        "mono8" => 1,
        "mono16" => 2,
        _ => {
            return Err(common::failure(path, "Unsupported ROS image encoding")
                .with_context("encoding", encoding));
        }
    };
    let expected = width
        .checked_mul(height)
        .and_then(|n| n.checked_mul(pixel_bytes))
        .ok_or_else(|| common::failure(path, "ROS image dimensions overflow"))?;
    if step != width * pixel_bytes || bytes.len() != expected {
        return Err(common::failure(
            path,
            "ROS image requires tightly packed pixels",
        ));
    }
    let mut image = ManagedImage::new(width, height);
    if pixel_bytes == 1 {
        for (out, value) in image.data_mut().iter_mut().zip(bytes) {
            *out = u16::from(*value) << 8;
        }
    } else {
        for (out, value) in image.data_mut().iter_mut().zip(bytes.as_chunks::<2>().0) {
            *out = u16::from_ne_bytes(*value);
        }
    }
    Ok(ImageData {
        img: Some(Arc::new(image)),
        exposure,
    })
}
enum Sample {
    Image(i64, usize, ImageIndex),
    Imu(i64, [f64; 3], [f64; 3]),
    Pose(i64, Isometry3<f64>),
    Point(i64, [f64; 3]),
}
#[derive(Default)]
pub struct RosbagIo {
    data: Option<VioDatasetPtr>,
}
impl DatasetIoInterface for RosbagIo {
    #[expect(
        clippy::too_many_lines,
        reason = "Preserves the original reader's discovery, scan, and clock-alignment phases"
    )]
    fn read(&mut self, path: &Path) -> Result<()> {
        let bag = RosBag::new(path)
            .map_err(|e| common::failure(path, "Cannot open ROS bag").with_source(e))?;
        let mut connections = BTreeMap::new();
        let mut chunks = Vec::new();
        for entry in bag.index_records() {
            match entry.map_err(|e| common::failure(path, e.to_string()))? {
                IndexRecord::Connection(c) => {
                    connections.insert(c.id, (c.storage_topic.to_owned(), c.tp.to_owned()));
                }
                IndexRecord::ChunkInfo(c) => chunks.push(c.chunk_pos),
                IndexRecord::IndexData(_) => {}
            }
        }
        let mut camera_topics = BTreeSet::new();
        let mut imu = None;
        let mut mocap = None;
        let mut point = None;
        for (topic, kind) in connections.values() {
            match kind.as_str() {
                "sensor_msgs/Image" => {
                    camera_topics.insert(topic.clone());
                }
                "sensor_msgs/Imu" if !topic.starts_with("/fcu") => imu = Some(topic.clone()),
                "geometry_msgs/TransformStamped" | "geometry_msgs/PoseStamped" => {
                    mocap = Some(topic.clone());
                }
                "geometry_msgs/PointStamped" => point = Some(topic.clone()),
                _ => {}
            }
        }
        let cameras: BTreeMap<_, _> = camera_topics
            .into_iter()
            .enumerate()
            .map(|(i, t)| (t, i))
            .collect();
        let mut samples = Vec::new();
        for position in chunks {
            let mut iter = bag.chunk_records();
            iter.seek(position)
                .map_err(|e| common::failure(path, e.to_string()))?;
            let Some(ChunkRecord::Chunk(chunk)) = iter
                .next()
                .transpose()
                .map_err(|e| common::failure(path, e.to_string()))?
            else {
                return Err(common::failure(path, "Invalid chunk index"));
            };
            for (ordinal, record) in chunk.messages().enumerate() {
                let MessageRecord::MessageData(message) =
                    record.map_err(|e| common::failure(path, e.to_string()))?
                else {
                    continue;
                };
                let Some((topic, kind)) = connections.get(&message.conn_id) else {
                    return Err(common::failure(path, "Unknown ROS connection"));
                };
                let mut payload = Payload {
                    data: message.data,
                    path,
                };
                let sample = if let Some(camera) = cameras.get(topic) {
                    Some(Sample::Image(
                        payload.header()?.0,
                        *camera,
                        ImageIndex {
                            chunk: position,
                            ordinal,
                        },
                    ))
                } else if imu.as_ref() == Some(topic) {
                    let stamp = payload.header()?.0;
                    payload.take(4 * 8 + 9 * 8)?;
                    let angular = payload.vector()?;
                    payload.take(9 * 8)?;
                    let linear = payload.vector()?;
                    Some(Sample::Imu(stamp, angular, linear))
                } else if mocap.as_ref() == Some(topic) {
                    let stamp = payload.header()?.0;
                    if kind == "geometry_msgs/TransformStamped" {
                        payload.string()?;
                    }
                    let translation = payload.vector()?;
                    let rotation = payload.quaternion()?;
                    Some(Sample::Pose(
                        stamp,
                        common::pose(translation, rotation, path, 0)?,
                    ))
                } else if point.as_ref() == Some(topic) {
                    Some(Sample::Point(payload.header()?.0, payload.vector()?))
                } else {
                    None
                };
                if let Some(sample) = sample {
                    samples.push((message.time, sample));
                }
            }
        }
        samples.sort_by_key(|s| s.0);
        let mut data = RosbagVioDataset {
            path: path.to_owned(),
            bag,
            records: Records::default(),
            images: BTreeMap::new(),
            cameras: cameras.len(),
            offset: 0,
        };
        let mut imu_offsets = Vec::new();
        let mut mocap_offsets = Vec::new();
        let mut poses = Vec::new();
        let mut points = Vec::new();
        for (arrival, sample) in samples {
            match sample {
                Sample::Image(stamp, camera, index) => {
                    data.images
                        .entry(stamp)
                        .or_insert_with(|| vec![None; data.cameras])[camera] = Some(index);
                }
                Sample::Imu(stamp, angular, linear) => {
                    data.records.gyro.push(GyroData {
                        timestamp_ns: stamp,
                        data: Vector3::from(angular),
                    });
                    data.records.accel.push(AccelData {
                        timestamp_ns: stamp,
                        data: Vector3::from(linear),
                    });
                    imu_offsets.push(stamp - arrival as i64);
                }
                Sample::Pose(stamp, pose) => {
                    poses.push((stamp, pose));
                    mocap_offsets.push(stamp - arrival as i64);
                }
                Sample::Point(stamp, point) => {
                    points.push((stamp, point));
                    mocap_offsets.push(stamp - arrival as i64);
                }
            }
        }
        if !mocap_offsets.is_empty() {
            if imu_offsets.is_empty() {
                return Err(common::failure(
                    path,
                    "Mocap clock alignment requires IMU samples",
                ));
            }
            data.offset =
                imu_offsets[imu_offsets.len() / 2] - mocap_offsets[mocap_offsets.len() / 2];
        }
        // Original loops deliberately stop before the final pose/point.
        for (stamp, pose) in poses.iter().take(poses.len().saturating_sub(1)) {
            data.records.gt_timestamps.push(*stamp + data.offset);
            data.records.gt_poses.push(*pose);
        }
        for (stamp, point) in points.iter().take(points.len().saturating_sub(1)) {
            data.records.gt_timestamps.push(*stamp);
            data.records
                .gt_poses
                .push(Isometry3::translation(point[0], point[1], point[2]));
        }
        data.records.image_timestamps = data.images.keys().copied().collect();
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
