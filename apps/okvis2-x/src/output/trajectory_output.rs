/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Trajectory CSV output and top-view scene construction.
use crate::{
    ceres::imu_error::IntegrationError,
    frames::frame_typedefs::{MapPointVector, StateId},
    kinematics::transformation::Transformation,
    runtime::{
        queued_trajectory::QueuedTrajectory,
        vi_interface::{State, TrackingQuality, TrackingState, Trajectory},
    },
    sensors::{
        measurements::{ImuMeasurement, ImuSensorReadings},
        vi_sensor_base::BoundedQueue,
    },
};
use jiff::Timestamp;
use nalgebra::{Vector2, Vector3};
use purecv::core::{DynamicMatrix, Matrix};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::{self, Write},
    path::Path,
    sync::Arc,
};
/// OpenCV-compatible drawing primitives: all lines and circles use `LINE_AA`,
/// circles are filled, text is `FONT_HERSHEY_COMPLEX`, and markers are diamonds.
pub trait TopViewPainter {
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    fn line(
        &self,
        image: &mut Matrix<u8>,
        from: Vector2<f64>,
        to: Vector2<f64>,
        bgr: [f64; 3],
        thickness: i32,
    ) -> io::Result<()>;
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    fn circle(
        &self,
        image: &mut Matrix<u8>,
        centre: Vector2<f64>,
        radius: f64,
        bgr: [f64; 3],
    ) -> io::Result<()>;
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    fn text(
        &self,
        image: &mut Matrix<u8>,
        text: &str,
        at: Vector2<f64>,
        scale: f64,
        bgr: [f64; 3],
        thickness: i32,
    ) -> io::Result<()>;
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    fn diamond(
        &self,
        image: &mut Matrix<u8>,
        at: Vector2<f64>,
        bgr: [f64; 3],
        size: i32,
    ) -> io::Result<()>;
}
type GraphStates = (
    State,
    TrackingState,
    Arc<BTreeMap<StateId, State>>,
    Arc<MapPointVector>,
);
pub struct TrajectoryOutput {
    csv: Option<File>,
    rgb_csv: Option<File>,
    rpg: bool,
    draw: bool,
    states: BoundedQueue<GraphStates>,
    trajectory: Trajectory,
    imu: BoundedQueue<ImuMeasurement>,
    image: Matrix<u8>,
    path: Vec<Vector2<f64>>,
    gps: Vec<Vector2<f64>>,
    heights: Vec<f64>,
    scale: f64,
    min: Vector3<f64>,
    max: Vector3<f64>,
    rgb_trajectory: QueuedTrajectory<Arc<DynamicMatrix>>,
    t_aw: Transformation,
}
impl TrajectoryOutput {
    #[must_use]
    pub fn new(draw: bool) -> Self {
        Self {
            csv: None,
            rgb_csv: None,
            rpg: false,
            draw,
            states: BoundedQueue::new(1000),
            trajectory: Trajectory::default(),
            imu: BoundedQueue::new(1000),
            image: Matrix::new(if draw { 500 } else { 0 }, if draw { 500 } else { 0 }, 3),
            path: Vec::new(),
            gps: Vec::new(),
            heights: Vec::new(),
            scale: 1.0,
            min: Vector3::repeat(-0.5),
            max: Vector3::repeat(0.5),
            rgb_trajectory: QueuedTrajectory::default(),
            t_aw: Transformation::default(),
        }
    }
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    pub fn with_csv(path: &Path, rpg: bool, draw: bool) -> io::Result<Self> {
        let mut output = Self::new(draw);
        output.set_csv_file(path, rpg)?;
        Ok(output)
    }
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    pub fn set_csv_file(&mut self, path: &Path, rpg: bool) -> io::Result<()> {
        self.rpg = rpg;
        self.csv = Some(Self::create_csv_file(path, rpg)?);
        Ok(())
    }
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    pub fn set_rgb_csv_file(&mut self, path: &Path) -> io::Result<()> {
        self.rgb_csv = Some(Self::create_csv_file(path, self.rpg)?);
        Ok(())
    }
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    pub fn process_state(
        &mut self,
        state: &State,
        tracking: &TrackingState,
        updated: Arc<BTreeMap<StateId, State>>,
        landmarks: Arc<MapPointVector>,
    ) -> io::Result<()> {
        if let Some(csv) = &mut self.csv {
            Self::write_state_to_csv(csv, state, self.rpg)?;
        }
        if self.draw && !updated.is_empty() {
            self.states
                .push_blocking((state.clone(), tracking.clone(), updated, landmarks));
        }
        Ok(())
    }
    pub fn add_imu_measurement(
        &self,
        stamp: Timestamp,
        alpha: Vector3<f64>,
        omega: Vector3<f64>,
    ) -> bool {
        self.draw
            && !self.imu.push_dropping(ImuMeasurement::new(
                stamp,
                ImuSensorReadings::new(omega, alpha),
                -1,
            ))
    }
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    pub fn process_rgb_image(
        &mut self,
        time: Timestamp,
        image: Arc<DynamicMatrix>,
    ) -> io::Result<bool> {
        let Some(csv) = &mut self.rgb_csv else {
            log::warn!("Cannot write RGB trajectory, since no CSV file is set");
            return Ok(false);
        };
        self.rgb_trajectory.enqueue_data(image, time);
        for (_, state) in self
            .rgb_trajectory
            .get_states(&mut self.trajectory)
            .map_err(integration_error)?
        {
            Self::write_state_to_csv(csv, &state, self.rpg)?;
        }
        Ok(true)
    }
    pub fn convert_to_image_coordinates(&self, p: Vector2<f64>) -> Vector2<f64> {
        let p = (p - Vector2::new(self.min.x, self.min.y)) * self.scale;
        Vector2::new(p.x, 500.0 - p.y)
    }
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    ///
    /// # Panics
    /// Panics if a buffered trajectory endpoint is absent after the nonempty check.
    pub fn draw_top_view(
        &mut self,
        painter: &dyn TopViewPainter,
    ) -> io::Result<Option<Matrix<u8>>> {
        let mut changed = false;
        while let Some((mut state, tracking, updated, landmarks)) = self.states.try_pop() {
            let mut affected = BTreeSet::new();
            self.trajectory
                .update(&tracking, &updated, &mut affected)
                .map_err(integration_error)?;
            if self.states.is_empty() {
                while let Some(imu) = self.imu.try_pop() {
                    if let Some(propagated) = self
                        .trajectory
                        .add_imu_measurement(
                            imu.time_stamp,
                            imu.measurement.accelerometers,
                            imu.measurement.gyroscopes,
                        )
                        .map_err(integration_error)?
                    {
                        state = propagated;
                    }
                }
            }
            let r = state.t_ws.translation();
            let c = state.t_ws.rotation_matrix();
            self.path.push(Vector2::new(r.x, r.y));
            self.heights.push(r.z);
            if r.x - 0.2 < self.min.x {
                self.min.x = r.x - 0.2;
            }
            if r.y - 0.2 < self.min.y {
                self.min.y = r.y - 0.2;
            }
            if r.z < self.min.z {
                self.min.z = r.z;
            }
            if r.x + 0.2 > self.max.x {
                self.max.x = r.x + 0.2;
            }
            if r.y + 0.2 > self.max.y {
                self.max.y = r.y + 0.2;
            }
            if r.z > self.max.z {
                self.max.z = r.z;
            }
            self.scale = (500.0 / (self.max.x - self.min.x)).min(500.0 / (self.max.y - self.min.y));
            self.image.data.fill(10);
            for lm in landmarks.iter() {
                if lm.point.w.abs() < 1e-12 || lm.quality < 0.0001 {
                    continue;
                }
                let p = lm.point.fixed_rows::<3>(0) / lm.point.w;
                let pixel = self.convert_to_image_coordinates(Vector2::new(p.x, p.y));
                if !inside(pixel) {
                    continue;
                }
                painter.circle(
                    &mut self.image,
                    pixel,
                    1.0,
                    [0.0, 255.0 * (lm.quality / 0.1).min(1.0), 0.0],
                )?;
            }
            for updated_state in updated.values() {
                let t_wg = updated_state.t_gw.inverse();
                for point in &updated_state.gps_points {
                    let p = t_wg.transform_homogeneous(&point.push(1.0));
                    self.gps.push(Vector2::new(p.x, p.y));
                }
            }
            self.draw_path(painter)?;
            self.draw_gps(painter)?;
            self.draw_path_noncausal(painter)?;
            self.draw_state_overlays(painter, &state, &tracking, (r, c))?;
            changed = true;
        }
        Ok(changed.then(|| self.image.clone()))
    }
    fn draw_state_overlays(
        &mut self,
        painter: &dyn TopViewPainter,
        state: &State,
        tracking: &TrackingState,
        (r, c): (Vector3<f64>, nalgebra::Matrix3<f64>),
    ) -> io::Result<()> {
        let origin = *self.path.last().unwrap();
        for axis in 0..3 {
            let endpoint = origin + Vector2::new(c[(0, axis)], c[(1, axis)]) * 0.2;
            let from = self.convert_to_image_coordinates(origin);
            let to = self.convert_to_image_coordinates(endpoint);
            painter.line(&mut self.image, from, to, axis_colour(axis), 1)?;
        }
        for (label, value, y) in [
            ("position", r, 15.0),
            ("velocity", state.v_w, 35.0),
            ("gyro bias", state.b_g, 55.0),
            ("acc bias", state.b_a, 75.0),
        ] {
            let text = format!(
                "{label} = [{}, {}, {}]",
                cpp_default(value.x, 6),
                cpp_default(value.y, 6),
                cpp_default(value.z, 6)
            );
            painter.text(
                &mut self.image,
                &text,
                Vector2::new(15.0, y),
                0.5,
                [255.0; 3],
                1,
            )?;
        }
        let (label, colour) = match tracking.tracking_quality {
            TrackingQuality::Lost => ("TRACKING LOST", [0.0, 0.0, 255.0]),
            TrackingQuality::Marginal => ("Tracking marginal", [0.0, 255.0, 255.0]),
            TrackingQuality::Good => ("Tracking good", [0.0, 255.0, 0.0]),
        };
        painter.text(
            &mut self.image,
            label,
            Vector2::new(5.0, 490.0),
            0.3,
            colour,
            1,
        )?;
        if tracking.backend.recognised_place {
            painter.text(
                &mut self.image,
                "Recognised place",
                Vector2::new(160.0, 490.0),
                0.3,
                [255.0, 0.0, 0.0],
                1,
            )?;
        }
        for pose in state.t_ai_w.values() {
            self.t_aw = pose.clone();
        }
        let world = self.t_aw.inverse();
        let c = world.rotation_matrix();
        let r = world.translation();
        let origin = Vector2::new(r.x, r.y);
        for axis in 0..3 {
            let endpoint = origin + Vector2::new(c[(0, axis)], c[(1, axis)]) * 0.4;
            let from = self.convert_to_image_coordinates(origin);
            let to = self.convert_to_image_coordinates(endpoint);
            painter.line(&mut self.image, from, to, axis_colour(axis), 1)?;
        }
        Ok(())
    }

    fn draw_path(&mut self, painter: &dyn TopViewPainter) -> io::Result<()> {
        let mut i = 0;
        while i + 1 < self.path.len() {
            let p0 = self.convert_to_image_coordinates(self.path[i]);
            let p1 = self.convert_to_image_coordinates(self.path[i + 1]);
            if (p1 - p0).norm_squared() < 2.0 {
                self.path.remove(i + 1);
                self.heights.remove(i + 1);
                continue;
            }
            painter.line(&mut self.image, p0, p1, [80.0; 3], 1)?;
            i += 1;
        }
        Ok(())
    }
    fn draw_gps(&mut self, painter: &dyn TopViewPainter) -> io::Result<()> {
        for i in 0..self.gps.len() {
            let p = self.convert_to_image_coordinates(self.gps[i]);
            if inside(p) {
                painter.diamond(&mut self.image, p, [255.0, 0.0, 255.0], 12)?;
            }
        }
        self.gps.clear();
        Ok(())
    }
    fn draw_path_noncausal(&mut self, painter: &dyn TopViewPainter) -> io::Result<()> {
        let mut path = Vec::new();
        let mut heights = Vec::new();
        let mut keyframes = Vec::new();
        for &id in self.trajectory.state_ids() {
            let state = self
                .trajectory
                .state_by_id(id)
                .ok_or_else(|| io::Error::other("trajectory state missing"))?;
            let r = state.t_ws.translation();
            path.push(Vector2::new(r.x, r.y));
            heights.push(r.z);
            keyframes.push(state.is_keyframe);
            if state.is_keyframe {
                let p0 = self.convert_to_image_coordinates(Vector2::new(r.x, r.y));
                for &other in &state.covisible_frame_ids {
                    let other = self
                        .trajectory
                        .state_by_id(other)
                        .ok_or_else(|| io::Error::other("covisible state missing"))?;
                    let r = other.t_ws.translation();
                    let p1 = self.convert_to_image_coordinates(Vector2::new(r.x, r.y));
                    painter.line(&mut self.image, p0, p1, [0.0, 127.0, 127.0], 1)?;
                }
            }
        }
        let mut i = 0;
        let mut j = 1;
        while j < path.len() {
            let p0 = self.convert_to_image_coordinates(path[i]);
            let p1 = self.convert_to_image_coordinates(path[j]);
            if keyframes[j] {
                painter.circle(&mut self.image, p1, 1.0, [255.0, 255.0, 0.0])?;
            }
            if i == 0 {
                painter.circle(&mut self.image, p0, 1.0, [255.0, 255.0, 0.0])?;
            }
            if (p1 - p0).norm_squared() < 2.0 {
                j += 1;
                continue;
            }
            let height = (heights[i] - self.min.z + heights[j] - self.min.z) * 0.5
                / (self.max.z - self.min.z);
            painter.line(
                &mut self.image,
                p0,
                p1,
                [height * 255.0, 0.0, (1.0 - height) * 255.0],
                1,
            )?;
            i = j;
            j += 1;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    pub fn create_csv_file(path: &Path, rpg: bool) -> io::Result<File> {
        let mut file = File::create(path)?;
        if rpg {
            writeln!(file, "# timestamp tx ty tz qx qy qz qw")?;
        } else {
            writeln!(
                file,
                "timestamp, p_WS_W_x, p_WS_W_y, p_WS_W_z, q_WS_x, q_WS_y, q_WS_z, q_WS_w, v_WS_W_x, v_WS_W_y, v_WS_W_z, b_g_x, b_g_y, b_g_z, b_a_x, b_a_y, b_a_z"
            )?;
        }
        file.flush()?;
        Ok(file)
    }
    /// # Errors
    /// Returns an error if output resources/data are missing or invalid, or if the writer,
    /// image codec or visualization backend fails.
    pub fn write_state_to_csv(file: &mut impl Write, state: &State, rpg: bool) -> io::Result<bool> {
        let r = state.t_ws.translation();
        let q = state.t_ws.quaternion();
        let pose = [r.x, r.y, r.z, q.i, q.j, q.k, q.w];
        if rpg {
            write!(
                file,
                "{}",
                cpp_default(
                    state.timestamp.as_second() as f64
                        + f64::from(state.timestamp.subsec_nanosecond()) * 1e-9,
                    19
                )
            )?;
            for value in pose {
                write!(file, " {}", cpp_default(value, 19))?;
            }
            writeln!(file)?;
        } else {
            write!(
                file,
                "{}{:09}, ",
                state.timestamp.as_second(),
                state.timestamp.subsec_nanosecond()
            )?;
            for value in pose
                .into_iter()
                .chain(state.v_w.iter().copied())
                .chain(state.b_g.iter().copied())
                .chain(state.b_a.iter().copied())
            {
                write!(file, "{}, ", cpp_scientific(value, 18))?;
            }
            writeln!(file)?;
        }
        file.flush()?;
        Ok(true)
    }
}
fn integration_error(error: IntegrationError) -> io::Error {
    io::Error::other(format!("trajectory propagation: {error:?}"))
}
fn inside(p: Vector2<f64>) -> bool {
    !(p.x < 0.0 || p.y < 0.0 || p.x > 499.0 || p.y > 499.0)
}
fn axis_colour(axis: usize) -> [f64; 3] {
    match axis {
        0 => [0.0, 0.0, 255.0],
        1 => [0.0, 255.0, 0.0],
        _ => [255.0, 0.0, 0.0],
    }
}
pub(crate) fn cpp_scientific(value: f64, precision: usize) -> String {
    let s = format!("{value:.precision$e}");
    if let Some((mantissa, exp)) = s.split_once('e')
        && let Ok(exponent) = exp.parse::<i32>()
    {
        return format!("{mantissa}e{exponent:+03}");
    }
    s
}
pub(crate) fn cpp_default(value: f64, precision: usize) -> String {
    if !value.is_finite() {
        return value.to_string().to_lowercase();
    }
    let scientific = cpp_scientific(value, precision.saturating_sub(1));
    let exponent = scientific
        .split_once('e')
        .and_then(|(_, e)| e.parse::<i32>().ok())
        .unwrap_or(0);
    if exponent < -4 || exponent >= precision as i32 {
        let (mantissa, exp) = scientific.split_once('e').unwrap();
        let mantissa = if mantissa.contains('.') {
            mantissa.trim_end_matches('0').trim_end_matches('.')
        } else {
            mantissa
        };
        return format!("{mantissa}e{exp}");
    }
    let decimals = (precision as i32 - 1 - exponent).max(0) as usize;
    let fixed = format!("{value:.decimals$}");
    if fixed.contains('.') {
        fixed.trim_end_matches('0').trim_end_matches('.').to_owned()
    } else {
        fixed
    }
}
