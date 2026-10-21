/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Asynchronous `EuRoC` image/IMU recording with bounded drop-oldest queues.
use crate::{
    cameras::n_camera_system::CameraType,
    config::parameters::ViParameters,
    datasets::dataset_reader::ImageCodec,
    runtime::vi_interface::{
        ImageMap, OptimisedGraphCallback, ViInterface, exp_image_transform, exp_image_transform_f64,
    },
    sensors::{
        measurements::{CameraData, CameraMeasurement, ImuMeasurement, ImuSensorReadings},
        vi_sensor_base::BoundedQueue,
    },
};
use jiff::{Timestamp, Zoned};
use nalgebra::Vector3;
use purecv::core::{DynamicData, DynamicMatrix};
use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{self, Write},
    path::{Path, PathBuf},
    sync::Arc,
    thread::{self, JoinHandle},
};
pub struct DatasetWriter {
    pub dataset_directory: PathBuf,
    camera_types: Vec<CameraType>,
    imu: Arc<BoundedQueue<ImuMeasurement>>,
    images: Arc<BoundedQueue<Vec<CameraMeasurement>>>,
    visualisations: Arc<BoundedQueue<Vec<CameraMeasurement>>>,
    imu_thread: Option<JoinHandle<io::Result<()>>>,
    images_thread: Option<JoinHandle<io::Result<()>>>,
    pub optimised_graph_callback: Option<OptimisedGraphCallback>,
}
impl DatasetWriter {
    /// # Errors
    /// Returns an error for malformed dataset records or timestamps, missing/unreadable files,
    /// invalid stream calibration, or a failed image codec, callback or worker operation.
    pub fn new(
        parameters: &ViParameters,
        path: &Path,
        codec: Arc<dyn ImageCodec>,
    ) -> io::Result<Self> {
        Self::with_directory_name(
            parameters,
            path,
            &Zoned::now().strftime("%d-%m-%Y-%H-%M-%S").to_string(),
            codec,
        )
    }
    /// # Errors
    /// Returns an error for malformed dataset records or timestamps, missing/unreadable files,
    /// invalid stream calibration, or a failed image codec, callback or worker operation.
    pub fn with_directory_name(
        parameters: &ViParameters,
        path: &Path,
        name: &str,
        codec: Arc<dyn ImageCodec>,
    ) -> io::Result<Self> {
        if !path.is_dir() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "provided path is not a valid directory",
            ));
        }
        let directory = path.join(name);
        fs::create_dir(&directory)?;
        let imu_directory = directory.join("imu0");
        fs::create_dir(&imu_directory)?;
        let mut imu_csv = File::create(imu_directory.join("data.csv"))?;
        writeln!(
            imu_csv,
            "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"
        )?;
        imu_csv.flush()?;
        let camera_types = (0..parameters.n_camera_system.num_cameras())
            .map(|i| parameters.n_camera_system.camera_type(i).clone())
            .collect::<Vec<_>>();
        let CameraOutputFiles {
            camera_directories,
            camera_csvs,
            depth_directories,
            depth_csvs,
        } = Self::create_camera_outputs(&directory, &camera_types)?;
        let imu = Arc::new(BoundedQueue::<ImuMeasurement>::new(100));
        let images = Arc::new(BoundedQueue::<Vec<CameraMeasurement>>::new(100));
        let visualisations = Arc::new(BoundedQueue::<Vec<CameraMeasurement>>::new(1));
        let input = imu.clone();
        let imu_thread = thread::spawn(move || {
            while let Some(m) = input.pop() {
                let w = m.measurement.gyroscopes;
                let a = m.measurement.accelerometers;
                writeln!(
                    imu_csv,
                    "{},{},{},{},{},{},{}",
                    m.time_stamp.as_nanosecond(),
                    crate::output::trajectory_output::cpp_default(w.x, 6),
                    crate::output::trajectory_output::cpp_default(w.y, 6),
                    crate::output::trajectory_output::cpp_default(w.z, 6),
                    crate::output::trajectory_output::cpp_default(a.x, 6),
                    crate::output::trajectory_output::cpp_default(a.y, 6),
                    crate::output::trajectory_output::cpp_default(a.z, 6)
                )?;
                imu_csv.flush()?;
            }
            Ok(())
        });
        let images_thread = Self::start_image_writer(
            &camera_types,
            (images.clone(), visualisations.clone()),
            codec,
            CameraOutputFiles {
                camera_directories,
                camera_csvs,
                depth_directories,
                depth_csvs,
            },
        );
        Ok(Self {
            dataset_directory: directory,
            camera_types,
            imu,
            images,
            visualisations,
            imu_thread: Some(imu_thread),
            images_thread: Some(images_thread),
            optimised_graph_callback: None,
        })
    }
    fn start_image_writer(
        camera_types: &[CameraType],
        (input, output): (CameraFrameQueue, CameraFrameQueue),
        codec: Arc<dyn ImageCodec>,
        CameraOutputFiles {
            camera_directories,
            mut camera_csvs,
            depth_directories,
            mut depth_csvs,
        }: CameraOutputFiles,
    ) -> JoinHandle<io::Result<()>> {
        let types = camera_types.to_vec();

        thread::spawn(move || {
            while let Some(mut frames) = input.pop() {
                for (i, frame) in frames.iter_mut().enumerate() {
                    let stamp = frame.time_stamp.as_nanosecond();
                    if let Some(image) = &frame.measurement.image {
                        let suffix = if types[i].is_colour { ".jpg" } else { ".png" };
                        writeln!(camera_csvs[i], "{stamp},{stamp}{suffix}")?;
                        camera_csvs[i].flush()?;
                        codec.write(
                            &camera_directories[i]
                                .join("data")
                                .join(format!("{stamp}{suffix}")),
                            image,
                        )?;
                    }
                    if types[i].depth_type.is_depth_camera
                        && let Some(depth) = &mut frame.measurement.depth_image
                    {
                        let csv = depth_csvs[i]
                            .as_mut()
                            .ok_or_else(|| io::Error::other("depth CSV missing"))?;
                        writeln!(csv, "{stamp},{stamp}.tif")?;
                        csv.flush()?;
                        let image = Arc::make_mut(depth);
                        match &mut image.data {
                            DynamicData::F32(m) => exp_image_transform(&mut m.data),
                            DynamicData::F64(m) => exp_image_transform_f64(&mut m.data),
                            _ => {
                                return Err(io::Error::new(
                                    io::ErrorKind::InvalidInput,
                                    "depth recording requires floating-point data",
                                ));
                            }
                        }
                        codec.write(
                            &depth_directories[i]
                                .as_ref()
                                .ok_or_else(|| io::Error::other("depth directory missing"))?
                                .join("data")
                                .join(format!("{stamp}.tif")),
                            image,
                        )?;
                    }
                }
                output.push_dropping(frames);
            }
            Ok(())
        })
    }

    fn create_camera_outputs(
        directory: &Path,
        camera_types: &[CameraType],
    ) -> io::Result<CameraOutputFiles> {
        let (mut camera_directories, mut camera_csvs, mut depth_directories, mut depth_csvs) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        for (i, kind) in camera_types.iter().enumerate() {
            let camera =
                directory.join(format!("{}{i}", if kind.is_colour { "rgb" } else { "cam" }));
            fs::create_dir(&camera)?;
            fs::create_dir(camera.join("data"))?;
            let mut csv = File::create(camera.join("data.csv"))?;
            writeln!(csv, "#timestamp [ns],filename")?;
            csv.flush()?;
            camera_directories.push(camera);
            camera_csvs.push(csv);
            if kind.depth_type.is_depth_camera {
                let depth = directory.join(format!("depth{i}"));
                fs::create_dir(&depth)?;
                fs::create_dir(depth.join("data"))?;
                let mut csv = File::create(depth.join("data.csv"))?;
                writeln!(csv, "#timestamp [ns],filename")?;
                csv.flush()?;
                depth_directories.push(Some(depth));
                depth_csvs.push(Some(csv));
            } else {
                depth_directories.push(None);
                depth_csvs.push(None);
            }
        }
        Ok(CameraOutputFiles {
            camera_directories,
            camera_csvs,
            depth_directories,
            depth_csvs,
        })
    }

    /// # Errors
    /// Returns an error for malformed dataset records or timestamps, missing/unreadable files,
    /// invalid stream calibration, or a failed image codec, callback or worker operation.
    pub fn close(&mut self) -> io::Result<()> {
        self.imu.shutdown();
        self.images.shutdown();
        self.visualisations.shutdown();
        let mut failure = None;
        for worker in [self.imu_thread.take(), self.images_thread.take()]
            .into_iter()
            .flatten()
        {
            let result = worker
                .join()
                .map_err(|_| io::Error::other("dataset writer worker panicked"))
                .and_then(|r| r);
            if let Err(error) = result
                && failure.is_none()
            {
                failure = Some(error);
            }
        }
        if let Some(error) = failure {
            return Err(error);
        }
        Ok(())
    }
}
impl ViInterface for DatasetWriter {
    fn add_images(&mut self, stamp: Timestamp, images: &ImageMap, depth: &ImageMap) -> bool {
        if self.images_thread.is_none() {
            return false;
        }
        let mut frames = (0..self.camera_types.len())
            .map(|_| CameraMeasurement::new(Timestamp::UNIX_EPOCH, CameraData::default(), -1))
            .collect::<Vec<_>>();
        for (&index, image) in images {
            let Some(frame) = frames.get_mut(index) else {
                log::error!("camera index out of range");
                return false;
            };
            frame.measurement.image = Some(image.clone());
            frame.time_stamp = stamp;
            frame.sensor_id = index as i32;
            frame.measurement.delivers_keypoints = Some(false);
        }
        for (&index, image) in depth {
            let Some(frame) = frames.get_mut(index) else {
                log::error!("depth camera index out of range");
                return false;
            };
            frame.measurement.depth_image = Some(image.clone());
            frame.time_stamp = stamp;
            frame.sensor_id = index as i32;
            frame.measurement.delivers_keypoints = Some(false);
        }
        if self.images.push_dropping(frames) {
            log::warn!("frame drop");
        }
        true
    }
    fn add_imu_measurement(
        &mut self,
        stamp: Timestamp,
        alpha: &Vector3<f64>,
        omega: &Vector3<f64>,
    ) -> bool {
        if self.imu_thread.is_none() {
            return false;
        }
        if self.imu.push_dropping(ImuMeasurement::new(
            stamp,
            ImuSensorReadings::new(*omega, *alpha),
            -1,
        )) {
            log::warn!("imu measurement drop");
            false
        } else {
            true
        }
    }
    fn add_lidar_measurement(&mut self, _stamp: Timestamp, _ray: &Vector3<f64>) -> bool {
        log::error!("No addLidar Callback implemented for DatasetWriter");
        false
    }
    fn add_depth_measurement(
        &mut self,
        _stamp: Timestamp,
        _depth: &DynamicMatrix,
        _sigma: Option<&DynamicMatrix>,
    ) -> bool {
        log::error!("No addDepth Callback implemented for DatasetWriter");
        false
    }
    fn set_blocking(&mut self, blocking: bool) {
        if blocking {
            log::warn!("DatasetWriter cannot run in blocking mode");
        }
    }
    fn set_optimised_graph_callback(&mut self, callback: OptimisedGraphCallback) {
        self.optimised_graph_callback = Some(callback);
    }
    fn display(&mut self, images: &mut BTreeMap<String, Arc<DynamicMatrix>>) {
        if let Some(frames) = self.visualisations.try_pop() {
            for (i, frame) in frames.into_iter().enumerate() {
                if let Some(image) = frame.measurement.image {
                    images.insert(
                        format!(
                            "{}{i}",
                            if self.camera_types[i].is_colour {
                                "rgb"
                            } else {
                                "cam"
                            }
                        ),
                        image,
                    );
                }
                if self.camera_types[i].depth_type.is_depth_camera
                    && let Some(depth) = frame.measurement.depth_image
                {
                    images.insert(format!("depth{i}"), depth);
                }
            }
        }
    }
}
impl Drop for DatasetWriter {
    fn drop(&mut self) {
        if let Err(e) = self.close() {
            log::error!("dataset writer shutdown: {e}");
        }
    }
}

struct CameraOutputFiles {
    camera_directories: Vec<PathBuf>,
    camera_csvs: Vec<File>,
    depth_directories: Vec<Option<PathBuf>>,
    depth_csvs: Vec<Option<File>>,
}

type CameraFrameQueue = Arc<BoundedQueue<Vec<CameraMeasurement>>>;
