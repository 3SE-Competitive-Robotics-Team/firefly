/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! SLAM component persistence using the tagged OKVIS graph and BRISK2 text format.

use crate::{
    cameras::n_camera_system::{DistortionType, NCameraSystem},
    ceres::{
        imu_error::ImuError,
        parameter_block::ParameterBlock,
        pose_parameter_block::PoseParameterBlock,
        reprojection_error_base::ReprojectionError2dBase,
        speed_and_bias_parameter_block::SpeedAndBiasParameterBlock,
        two_pose_graph_error::{SharedPoint, SharedPose},
    },
    config::parameters::ImuParameters,
    frames::{
        frame::KeyPoint,
        frame_typedefs::{KeypointIdentifier, LandmarkId, SpeedAndBias, StateId},
        multi_frame::MultiFrame,
    },
    kinematics::transformation::Transformation,
    sensors::measurements::{ImuMeasurement, ImuMeasurementDeque, ImuSensorReadings},
};
use firefly_error::{Error, ErrorKind, Result};
use jiff::Timestamp;
use nalgebra::{Matrix2, Quaternion, Vector2, Vector3, Vector4};
use purecv::core::Matrix;
use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::{BufRead, BufReader, BufWriter, Write},
    path::Path,
    rc::Rc,
    sync::Arc,
};

pub type MultiFramePtr = Rc<RefCell<MultiFrame>>;
#[derive(Clone)]
pub struct ComponentObservation {
    pub landmark_id: LandmarkId,
    pub error_term: Rc<RefCell<Box<dyn ReprojectionError2dBase>>>,
}
#[derive(Default)]
pub struct ComponentState {
    pub timestamp: Option<Timestamp>,
    pub pose: Option<SharedPose>,
    pub speed_and_bias: Option<Rc<RefCell<SpeedAndBiasParameterBlock>>>,
    pub extrinsics: Vec<Option<SharedPose>>,
    pub observations: BTreeMap<KeypointIdentifier, ComponentObservation>,
    pub previous_imu_link: Option<Rc<ImuError>>,
    pub next_imu_link: Option<Rc<ImuError>>,
}
pub struct ComponentLandmark {
    pub h_point: SharedPoint,
    pub quality: f64,
}
/// `ViGraphEstimator` storage and solver operations used by component persistence.
/// Registration must retain the shared blocks and apply `PoseManifold` to poses
/// and extrinsics. Observation insertion constructs the selected camera factor
/// from the multiframe keypoint and enables Cauchy robustification.
pub trait ComponentGraph: Default {
    fn states(&self) -> &BTreeMap<StateId, ComponentState>;
    fn states_mut(&mut self) -> &mut BTreeMap<StateId, ComponentState>;
    fn landmarks(&self) -> &BTreeMap<LandmarkId, ComponentLandmark>;
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn register_pose(&mut self, pose: SharedPose) -> Result<()>;
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn register_speed_and_bias(
        &mut self,
        speed_and_bias: Rc<RefCell<SpeedAndBiasParameterBlock>>,
    ) -> Result<()>;
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn register_extrinsics(&mut self, extrinsics: SharedPose) -> Result<()>;
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn add_landmark(
        &mut self,
        id: LandmarkId,
        point: Vector4<f64>,
        initialized: bool,
    ) -> Result<()>;
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn set_landmark_quality(&mut self, id: LandmarkId, quality: f64) -> Result<()>;
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn add_observation(
        &mut self,
        multi_frame: &MultiFrame,
        landmark_id: LandmarkId,
        keypoint_id: KeypointIdentifier,
        distortion_type: DistortionType,
        use_cauchy: bool,
    ) -> Result<()>;
    /// Registers the four blocks `(pose0, speedBias0, pose1, speedBias1)` and
    /// associates the returned solver residual with both endpoint IMU links.
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    fn register_imu_error(
        &mut self,
        previous: StateId,
        current: StateId,
        error: Rc<ImuError>,
    ) -> Result<()>;
}
pub enum ComponentGraphStorage<'a, G> {
    Borrowed(&'a mut G),
    Owned(Box<G>),
}
impl<G> ComponentGraphStorage<'_, G> {
    fn get(&self) -> &G {
        match self {
            Self::Borrowed(g) => g,
            Self::Owned(g) => g,
        }
    }
    fn get_mut(&mut self) -> &mut G {
        match self {
            Self::Borrowed(g) => g,
            Self::Owned(g) => g,
        }
    }
}
pub struct Component<'a, G: ComponentGraph> {
    pub imu_parameters: ImuParameters,
    pub n_camera_system: NCameraSystem,
    pub full_graph: Option<ComponentGraphStorage<'a, G>>,
    pub multi_frames: BTreeMap<StateId, MultiFramePtr>,
}
impl<'a, G: ComponentGraph> Component<'a, G> {
    #[must_use]
    pub fn new(imu_parameters: ImuParameters, n_camera_system: NCameraSystem) -> Self {
        Self {
            imu_parameters,
            n_camera_system,
            full_graph: None,
            multi_frames: BTreeMap::new(),
        }
    }
    pub fn with_graph(
        imu_parameters: ImuParameters,
        n_camera_system: NCameraSystem,
        full_graph: &'a mut G,
        multi_frames: BTreeMap<StateId, MultiFramePtr>,
    ) -> Self {
        Self {
            imu_parameters,
            n_camera_system,
            full_graph: Some(ComponentGraphStorage::Borrowed(full_graph)),
            multi_frames,
        }
    }
    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    ///
    /// # Panics
    /// Panics if an internal graph invariant is violated after its validation/insertion checks;
    /// referenced states, blocks and factor caches must remain consistent.
    pub fn load(&mut self, path: impl AsRef<Path>) -> Result<bool> {
        let file = File::open(path.as_ref()).map_err(|error| io_error(&error))?;
        if self.full_graph.is_some() {
            return Err(component_error("graph not empty"));
        }
        self.full_graph = Some(ComponentGraphStorage::Owned(Box::<G>::default()));
        let graph = self
            .full_graph
            .as_mut()
            .expect("graph was assigned")
            .get_mut();
        let num_cameras = self.n_camera_system.num_cameras();
        if num_cameras == 0 {
            return Err(component_error("component has no camera"));
        }
        let distortion = self.n_camera_system.distortion_type(0);
        if (1..num_cameras).any(|i| self.n_camera_system.distortion_type(i) != distortion) {
            return Err(component_error("mixed frame types are not supported"));
        }
        let mut extrinsics_blocks: Vec<BTreeMap<u64, SharedPose>> =
            vec![BTreeMap::new(); num_cameras];
        let lines: Vec<String> = BufReader::new(file)
            .lines()
            .collect::<std::io::Result<_>>()
            .map_err(|error| io_error(&error))?;
        let mut index = 0;
        while index < lines.len() {
            let mut row = Tokens::new(&lines[index]);
            index += 1;
            let tag = row.string()?;
            match tag {
                "VERTEX_SE3:QUAT_TIME"
                | "VERTEX_R3:VEL"
                | "VERTEX_R3:ACCBIAS"
                | "VERTEX_R3:GYRBIAS" => Self::load_state_record(graph, tag, &mut row)?,
                "VERTEX_TRACKXYZ" => {
                    let id = LandmarkId::new(row.value()?);
                    let point = row.vector3()?;
                    let quality: f64 = row.value()?;
                    graph.add_landmark(
                        id,
                        Vector4::new(point[0], point[1], point[2], 1.0),
                        quality > 0.001,
                    )?;
                    graph.set_landmark_quality(id, quality)?;
                }
                "FRAME" => Self::load_frame_record(
                    graph,
                    &self.n_camera_system,
                    &mut self.multi_frames,
                    &mut extrinsics_blocks,
                    &mut row,
                    (&lines, &mut index),
                )?,
                "EDGE_IMU" => Self::load_imu_record(
                    graph,
                    &self.imu_parameters,
                    &mut row,
                    (&lines, &mut index),
                )?,
                "EDGE_OBS" => {
                    Self::load_observation_record(graph, &self.multi_frames, distortion, &mut row)?;
                }
                _ => {
                    return Err(component_error(&format!(
                        "unknown component type tag {tag}"
                    )));
                }
            }
        }
        log::info!("--- loaded component: ---");
        log::info!("no. states: {}", graph.states().len());
        log::info!("no. landmarks: {}", graph.landmarks().len());
        Ok(true)
    }
    fn load_state_record(graph: &mut G, tag: &str, row: &mut Tokens<'_>) -> Result<()> {
        let id64 = row.value()?;
        let id = StateId::new(id64);
        graph.states_mut().entry(id).or_default();
        if tag == "VERTEX_SE3:QUAT_TIME" {
            let pose = row.pose()?;
            let timestamp = timestamp_from_nsec(row.value()?)?;
            let block = Rc::new(RefCell::new(PoseParameterBlock::new(
                &pose, id64, timestamp,
            )));
            let state = graph.states_mut().get_mut(&id).expect("state was inserted");
            state.timestamp = Some(timestamp);
            state.pose = Some(block.clone());
            graph.register_pose(block)?;
        } else {
            let reading = row.vector3()?;
            if graph.states()[&id].speed_and_bias.is_none() {
                let timestamp = graph.states()[&id]
                    .pose
                    .as_ref()
                    .ok_or_else(|| component_error("speed/bias record precedes pose"))?
                    .borrow()
                    .timestamp();
                let block = Rc::new(RefCell::new(SpeedAndBiasParameterBlock::new(
                    SpeedAndBias::zeros(),
                    id64,
                    timestamp,
                )));
                graph
                    .states_mut()
                    .get_mut(&id)
                    .expect("state exists")
                    .speed_and_bias = Some(block.clone());
                graph.register_speed_and_bias(block)?;
            }
            let offset = match tag {
                "VERTEX_R3:VEL" => 0,
                "VERTEX_R3:ACCBIAS" => 6,
                _ => 3,
            };
            let mut block = graph.states()[&id]
                .speed_and_bias
                .as_ref()
                .expect("speed/bias was initialized")
                .borrow_mut();
            block
                .parameters_mut()
                .ok_or_else(|| component_error("speed/bias estimate is absent"))?
                [offset..offset + 3]
                .copy_from_slice(reading.as_slice());
        }

        Ok(())
    }
    fn load_frame_record(
        graph: &mut G,
        camera_system: &NCameraSystem,
        multi_frames: &mut BTreeMap<StateId, MultiFramePtr>,
        extrinsics_blocks: &mut [BTreeMap<u64, SharedPose>],
        row: &mut Tokens<'_>,
        (lines, index): (&[String], &mut usize),
    ) -> Result<()> {
        let num_cameras = camera_system.num_cameras();
        let state_id64 = row.value()?;
        let id = StateId::new(state_id64);
        let camera: usize = row.value()?;
        let extrinsics_id = row.value()?;
        let pose = row.pose()?;
        let timestamp = timestamp_from_nsec(row.value()?)?;
        if camera >= num_cameras {
            return Err(component_error("camera *index outside rig"));
        }
        let first = !multi_frames.contains_key(&id);
        let frame = multi_frames
            .entry(id)
            .or_insert_with(|| {
                Rc::new(RefCell::new(MultiFrame::new(
                    camera_system,
                    timestamp,
                    state_id64,
                )))
            })
            .clone();
        let state = graph
            .states_mut()
            .get_mut(&id)
            .ok_or_else(|| component_error("frame refers to absent state"))?;
        if first {
            state.extrinsics.resize(frame.borrow().num_frames(), None);
        }
        let extrinsics = if let Some(block) = extrinsics_blocks[camera].get(&extrinsics_id) {
            block.clone()
        } else {
            let block = Rc::new(RefCell::new(PoseParameterBlock::new(
                &pose,
                extrinsics_id,
                timestamp,
            )));
            extrinsics_blocks[camera].insert(extrinsics_id, block.clone());
            graph.register_extrinsics(block.clone())?;
            block
        };
        *graph
            .states_mut()
            .get_mut(&id)
            .expect("state exists")
            .extrinsics
            .get_mut(camera)
            .ok_or_else(|| component_error("state extrinsics size mismatch"))? = Some(extrinsics);
        let mut keypoints = Vec::with_capacity(1000);
        let mut descriptors = Vec::with_capacity(48 * 1000);
        while *index < lines.len() {
            let mut item = Tokens::new(&lines[*index]);
            let tag = item.string()?;
            if tag != "FRAME:KEYPOINT" {
                let count = keypoints.len();
                let mut frame = frame.borrow_mut();
                frame.reset_keypoints(camera, &keypoints);
                frame.reset_descriptors(
                    camera,
                    Arc::new(Matrix::from_vec(count, 48, 1, descriptors)),
                );
                frame.compute_back_projections(camera);
                break;
            }
            *index += 1;
            let state_id2: u64 = item.value()?;
            let camera2: usize = item.value()?;
            let mut point = KeyPoint::default();
            point.pt.x = item.value()?;
            point.pt.y = item.value()?;
            point.size = item.value()?;
            if item.string()? != "BRISK2" {
                return Err(component_error("only BRISK2 descriptors are supported"));
            }
            if state_id2 != state_id64 || camera2 != camera {
                return Err(component_error("mismatching keypoint frame or camera"));
            }
            keypoints.push(point);
            let descriptor = item.string()?.as_bytes();
            if descriptor.len() < 96 {
                return Err(component_error(
                    "BRISK2 descriptor is shorter than 48 bytes",
                ));
            }
            for col in 0..48 {
                let byte = std::str::from_utf8(&descriptor[2 * col..2 * col + 2])
                    .map_err(|_| component_error("invalid descriptor text"))?;
                descriptors.push(
                    u8::from_str_radix(byte, 16)
                        .map_err(|_| component_error("invalid descriptor byte"))?,
                );
            }
        }

        Ok(())
    }
    fn load_imu_record(
        graph: &mut G,
        imu_parameters: &ImuParameters,
        row: &mut Tokens<'_>,
        (lines, index): (&[String], &mut usize),
    ) -> Result<()> {
        let previous = StateId::new(row.value()?);
        let current = StateId::new(row.value()?);
        let mut measurements = ImuMeasurementDeque::new();
        while *index < lines.len() {
            let mut item = Tokens::new(&lines[*index]);
            if item.string()? != "EDGE_IMU:MEASUREMENTS" {
                let state0 = graph
                    .states()
                    .get(&previous)
                    .ok_or_else(|| component_error("previous IMU state does not exist"))?;
                let state1 = graph
                    .states()
                    .get(&current)
                    .ok_or_else(|| component_error("current IMU state does not exist"))?;
                if state0.pose.is_none()
                    || state0.speed_and_bias.is_none()
                    || state1.pose.is_none()
                    || state1.speed_and_bias.is_none()
                {
                    return Err(component_error("IMU endpoint parameter block is absent"));
                }
                let error = Rc::new(
                    ImuError::new(
                        measurements,
                        imu_parameters.clone(),
                        state0
                            .timestamp
                            .ok_or_else(|| component_error("previous timestamp absent"))?,
                        state1
                            .timestamp
                            .ok_or_else(|| component_error("current timestamp absent"))?,
                    )
                    .map_err(|e| component_error(&format!("IMU edge: {e:?}")))?,
                );
                graph
                    .states_mut()
                    .get_mut(&previous)
                    .expect("previous state exists")
                    .next_imu_link = Some(error.clone());
                graph
                    .states_mut()
                    .get_mut(&current)
                    .expect("current state exists")
                    .previous_imu_link = Some(error.clone());
                graph.register_imu_error(previous, current, error)?;
                break;
            }
            *index += 1;
            let acc = item.vector3()?;
            let gyro = item.vector3()?;
            let timestamp = timestamp_from_nsec(item.value()?)?;
            measurements.push_back(ImuMeasurement::new(
                timestamp,
                ImuSensorReadings::new(gyro, acc),
                -1,
            ));
        }

        Ok(())
    }
    fn load_observation_record(
        graph: &mut G,
        multi_frames: &BTreeMap<StateId, MultiFramePtr>,
        distortion: DistortionType,
        row: &mut Tokens<'_>,
    ) -> Result<()> {
        let state_id64 = row.value()?;
        let id = StateId::new(state_id64);
        let camera: usize = row.value()?;
        let keypoint: usize = row.value()?;
        let landmark = LandmarkId::new(row.value()?);
        let _measurement = Vector2::new(row.value::<f64>()?, row.value::<f64>()?);
        let _information = Matrix2::new(
            row.value::<f64>()?,
            row.value::<f64>()?,
            row.value::<f64>()?,
            row.value::<f64>()?,
        );
        let frame = multi_frames
            .get(&id)
            .ok_or_else(|| component_error("observation refers to absent multiframe"))?;
        if camera >= frame.borrow().num_frames() || keypoint >= frame.borrow().num_keypoints(camera)
        {
            return Err(component_error("observation keypoint outside frame"));
        }
        frame
            .borrow_mut()
            .set_landmark_id(camera, keypoint, landmark.value());
        match distortion {
            DistortionType::RadialTangential
            | DistortionType::Equidistant
            | DistortionType::RadialTangential8 => graph.add_observation(
                &frame.borrow(),
                landmark,
                KeypointIdentifier::new(state_id64, camera, keypoint),
                distortion,
                true,
            )?,
            DistortionType::NoDistortion => {
                return Err(component_error("unsupported distortion type"));
            }
        }
        let landmark_data = graph
            .landmarks()
            .get(&landmark)
            .ok_or_else(|| component_error("observation landmark is absent"))?;
        let point = *landmark_data
            .h_point
            .borrow()
            .estimate()
            .ok_or_else(|| component_error("landmark estimate is absent"))?;
        let state = graph
            .states()
            .get(&id)
            .ok_or_else(|| component_error("observation state is absent"))?;
        let pose = Transformation::from_other(
            state
                .pose
                .as_ref()
                .ok_or_else(|| component_error("state pose absent"))?
                .borrow()
                .estimate()
                .ok_or_else(|| component_error("state pose estimate absent"))?,
        );
        frame.borrow_mut().set_landmark(
            camera,
            keypoint,
            pose.inverse().transform_homogeneous(&point),
            landmark_data.quality > 0.05,
        );

        Ok(())
    }

    /// # Errors
    /// Returns an error if required graph states, parameter blocks or measurements are absent,
    /// input dimensions/values are invalid, or evaluation, optimization or graph I/O fails.
    pub fn save(&self, path: impl AsRef<Path>) -> Result<bool> {
        let mut file =
            BufWriter::new(File::create(path.as_ref()).map_err(|error| io_error(&error))?);
        let graph = self
            .full_graph
            .as_ref()
            .ok_or_else(|| component_error("component graph is absent"))?
            .get();
        let mut written = BTreeSet::new();
        let mut previous_id = None;
        for (&id, state) in graph.states() {
            let pose = state
                .pose
                .as_ref()
                .ok_or_else(|| component_error("state pose is absent"))?
                .borrow();
            let pose = pose
                .estimate()
                .ok_or_else(|| component_error("state pose estimate is absent"))?;
            let t = pose.translation();
            let q = pose.quaternion();
            let timestamp = nsec(
                state
                    .timestamp
                    .ok_or_else(|| component_error("state timestamp is absent"))?,
            )?;
            writeln!(file,"VERTEX_SE3:QUAT_TIME {} {:.16e} {:.16e} {:.16e} {:.16e} {:.16e} {:.16e} {:.16e} {}",id.value(),t[0],t[1],t[2],q.i,q.j,q.k,q.w,timestamp).map_err(|error| io_error(&error))?;
            let sb = state
                .speed_and_bias
                .as_ref()
                .ok_or_else(|| component_error("speed/bias is absent"))?
                .borrow();
            let sb = sb
                .estimate()
                .ok_or_else(|| component_error("speed/bias estimate is absent"))?;
            for (tag, offset) in [
                ("VERTEX_R3:VEL", 0),
                ("VERTEX_R3:ACCBIAS", 6),
                ("VERTEX_R3:GYRBIAS", 3),
            ] {
                writeln!(
                    file,
                    "{tag} {} {:.16e} {:.16e} {:.16e}",
                    id.value(),
                    sb[offset],
                    sb[offset + 1],
                    sb[offset + 2]
                )
                .map_err(|error| io_error(&error))?;
            }
            let frame = self
                .multi_frames
                .get(&id)
                .ok_or_else(|| component_error("state multiframe is absent"))?
                .borrow();
            Self::save_frame_record(&mut file, id, state, &frame)?;
            if let Some(imu) = &state.previous_imu_link {
                let previous = previous_id
                    .ok_or_else(|| component_error("first state has a previous IMU link"))?;
                writeln!(file, "EDGE_IMU {} {}", previous, id.value())
                    .map_err(|error| io_error(&error))?;
                for sample in &imu.imu_measurements {
                    let acc = sample.measurement.accelerometers;
                    let gyr = sample.measurement.gyroscopes;
                    writeln!(
                        file,
                        "EDGE_IMU:MEASUREMENTS {:.16e} {:.16e} {:.16e} {:.16e} {:.16e} {:.16e} {}",
                        acc[0],
                        acc[1],
                        acc[2],
                        gyr[0],
                        gyr[1],
                        gyr[2],
                        nsec(sample.time_stamp)?
                    )
                    .map_err(|error| io_error(&error))?;
                }
            }
            Self::save_observation_records(graph, &mut file, id, state, &mut written)?;
            previous_id = Some(id.value());
        }
        file.flush().map_err(|error| io_error(&error))?;
        Ok(true)
    }
    fn save_observation_records(
        graph: &G,
        file: &mut impl Write,
        id: StateId,
        state: &ComponentState,
        written: &mut BTreeSet<LandmarkId>,
    ) -> Result<()> {
        for (keypoint, observation) in &state.observations {
            let landmark_id = observation.landmark_id;
            if !written.contains(&landmark_id) {
                let landmark = graph
                    .landmarks()
                    .get(&landmark_id)
                    .ok_or_else(|| component_error("observation landmark is absent"))?;
                let point = landmark.h_point.borrow();
                let point = point
                    .estimate()
                    .ok_or_else(|| component_error("landmark estimate is absent"))?;
                writeln!(
                    file,
                    "VERTEX_TRACKXYZ {} {:.16e} {:.16e} {:.16e} {:.16e}",
                    landmark_id.value(),
                    point[0] / point[3],
                    point[1] / point[3],
                    point[2] / point[3],
                    landmark.quality
                )
                .map_err(|error| io_error(&error))?;
                written.insert(landmark_id);
            }
            let factor = observation.error_term.borrow();
            let info = factor
                .information()
                .ok_or_else(|| component_error("observation information is absent"))?;
            let coord = factor
                .measurement()
                .ok_or_else(|| component_error("observation measurement is absent"))?;
            writeln!(
                file,
                "EDGE_OBS {} {} {} {} {:.16e} {:.16e} {:.16e} {:.16e} {:.16e} {:.16e}",
                id.value(),
                keypoint.camera_index,
                keypoint.keypoint_index,
                landmark_id.value(),
                coord[0],
                coord[1],
                info[(0, 0)],
                info[(0, 1)],
                info[(1, 0)],
                info[(1, 1)]
            )
            .map_err(|error| io_error(&error))?;
        }
        Ok(())
    }

    fn save_frame_record(
        file: &mut impl Write,
        id: StateId,
        state: &ComponentState,
        frame: &MultiFrame,
    ) -> Result<()> {
        for camera in 0..frame.num_frames() {
            let extrinsics = state
                .extrinsics
                .get(camera)
                .and_then(Option::as_ref)
                .ok_or_else(|| component_error("state extrinsics is absent"))?
                .borrow();
            let pose = extrinsics
                .estimate()
                .ok_or_else(|| component_error("extrinsics estimate is absent"))?;
            let t = pose.translation();
            let q = pose.quaternion();
            writeln!(
                file,
                "FRAME {} {} {} {:.16e} {:.16e} {:.16e} {:.16e} {:.16e} {:.16e} {:.16e} {}",
                id.value(),
                camera,
                extrinsics.id(),
                t[0],
                t[1],
                t[2],
                q.i,
                q.j,
                q.k,
                q.w,
                nsec(frame.timestamp())?
            )
            .map_err(|error| io_error(&error))?;
            for keypoint in 0..frame.num_keypoints(camera) {
                let mut point = KeyPoint::default();
                if !frame.get_cv_keypoint(camera, keypoint, &mut point) {
                    return Err(component_error("keypoint is absent"));
                }
                write!(
                    file,
                    "FRAME:KEYPOINT {} {} {:.16e} {:.16e} {:.16e} BRISK2 ",
                    id.value(),
                    camera,
                    f64::from(point.pt.x),
                    f64::from(point.pt.y),
                    f64::from(point.size)
                )
                .map_err(|error| io_error(&error))?;
                let descriptor = frame.keypoint_descriptor(camera, keypoint);
                if descriptor.len() < 48 {
                    return Err(component_error("descriptor is shorter than 48 bytes"));
                }
                for byte in &descriptor[..48] {
                    write!(file, "{byte:02x}").map_err(|error| io_error(&error))?;
                }
                writeln!(file).map_err(|error| io_error(&error))?;
            }
        }
        Ok(())
    }
}
struct Tokens<'a>(std::str::SplitWhitespace<'a>);
impl<'a> Tokens<'a> {
    fn new(line: &'a str) -> Self {
        Self(line.split_whitespace())
    }
    fn string(&mut self) -> Result<&'a str> {
        self.0
            .next()
            .ok_or_else(|| component_error("component record is incomplete"))
    }
    fn value<T: std::str::FromStr>(&mut self) -> Result<T> {
        self.string()?
            .parse()
            .map_err(|_| component_error("invalid component record value"))
    }
    fn vector3(&mut self) -> Result<Vector3<f64>> {
        Ok(Vector3::new(self.value()?, self.value()?, self.value()?))
    }
    fn pose(&mut self) -> Result<Transformation> {
        let t = self.vector3()?;
        let qx = self.value()?;
        let qy = self.value()?;
        let qz = self.value()?;
        let qw = self.value()?;
        Ok(Transformation::new(t, Quaternion::new(qw, qx, qy, qz)))
    }
}
fn timestamp_from_nsec(value: u64) -> Result<Timestamp> {
    Timestamp::from_nanosecond(i128::from(value))
        .map_err(|_| component_error("timestamp outside supported range"))
}
fn nsec(timestamp: Timestamp) -> Result<u64> {
    u64::try_from(timestamp.as_nanosecond())
        .map_err(|_| component_error("timestamp is outside unsigned nanoseconds"))
}
fn io_error(error: &std::io::Error) -> Error {
    Error::new(ErrorKind::Internal, error.to_string()).with_operation("Component::io")
}
fn component_error(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("Component")
}
