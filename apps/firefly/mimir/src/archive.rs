//! Synchronous RRD encoding with direct disk backpressure.
use std::collections::{BTreeMap, BTreeSet};

use firefly_archive::Run;
use firefly_error::Result;
use firefly_pubsub::viz::{ScalarMessage, VOXELS_MAX, kind};
use firefly_pubsub::{navigation::NavigationMessage, odom::OdomMessage};
use re_byte_size::SizeBytes as _;
use re_chunk::{Chunk, PendingRow, TimeColumn, TimePoint, Timeline};
use re_sdk_types::{AsComponents, archetypes as rr, datatypes::Quaternion};

use crate::{
    config::Options,
    failure,
    frames::{Anchor, Frames},
    invalid,
    observation::{Geometry, GeometryData, VoxelChunk},
    schema,
    segments::Segments,
    status::Status,
};

#[derive(Default)]
struct Scalars {
    times: Vec<i64>,
    values: Vec<f64>,
    width: usize,
}

#[derive(Default)]
struct Rows {
    values: Vec<PendingRow>,
    bytes: u64,
}

struct Voxels {
    cells: BTreeSet<[i32; 3]>,
    size: [f32; 3],
    origin: [f32; 3],
    stamp: f64,
}

struct PendingVoxels {
    name: String,
    stamp: f64,
    kind: u32,
    chunks: u32,
    next: u32,
    size: [f32; 3],
    origin: [f32; 3],
    cells: BTreeSet<[i32; 3]>,
}

impl PendingVoxels {
    fn append(&mut self, incoming: &[[i32; 3]], limit: usize) -> Result<()> {
        // Validate the next state before advancing chunk ownership.
        let additions: BTreeSet<_> = incoming
            .iter()
            .copied()
            .filter(|cell| !self.cells.contains(cell))
            .collect();
        if additions.len() > limit.saturating_sub(self.cells.len()) {
            return Err(invalid("voxel update budget exceeded"));
        }
        self.cells.extend(additions);
        self.next += 1;
        Ok(())
    }
}

pub struct Archive {
    segments: Segments,
    options: Options,
    scalars: BTreeMap<String, Scalars>,
    rows: BTreeMap<String, Rows>,
    row_bytes: u64,
    entities: BTreeSet<String>,
    voxels: BTreeMap<String, Voxels>,
    pending_voxels: Option<PendingVoxels>,
    previous_positions: BTreeMap<String, [f32; 3]>,
    point_times: BTreeMap<String, f64>,
    frames: Frames,
    camera_rig: Option<firefly_pubsub::camera::CameraRig>,
    status: Status,
    pub samples: u64,
    pub chunks: u64,
}

impl Archive {
    /// Open the recording writer in a leased run archive.
    /// # Errors
    /// Invalid budgets or directory/encoder/file initialization failure.
    pub fn in_run(options: Options, run: Run) -> Result<Self> {
        options.validate()?;
        let mut archive = Self {
            segments: Segments::new(run)?,
            options,
            scalars: BTreeMap::new(),
            rows: BTreeMap::new(),
            row_bytes: 0,
            entities: BTreeSet::new(),
            voxels: BTreeMap::new(),
            pending_voxels: None,
            previous_positions: BTreeMap::new(),
            point_times: BTreeMap::new(),
            frames: Frames::default(),
            camera_rig: None,
            status: Status::default(),
            samples: 0,
            chunks: 0,
        };
        archive.write("/", None, &rr::ViewCoordinates::RIGHT_HAND_Z_UP())?;
        archive.write(
            "frames/map",
            None,
            &rr::Transform3D::IDENTITY
                .with_parent_frame("tf#/")
                .with_child_frame("map"),
        )?;
        archive.write(
            "mimir/config",
            None,
            &rr::TextDocument::new(format!("{:?}", archive.options)),
        )?;
        println!("recording: {}", archive.segments.run().path().display());
        Ok(archive)
    }

    fn register(&mut self, name: &str) -> Result<()> {
        if !self.entities.contains(name) {
            if self.entities.len() >= self.options.max_entities {
                return Err(invalid("recording entity budget exceeded"));
            }
            self.entities.insert(name.to_owned());
        }
        Ok(())
    }

    fn append(&mut self, chunk: &Chunk) -> Result<()> {
        self.segments.append(chunk, None)?;
        self.chunks += 1;
        Ok(())
    }

    /// Write a typed archetype at its measurement time; `None` denotes static metadata.
    /// # Errors
    /// Invalid entity budget, timestamp, component encoding, or disk writes.
    pub fn write(
        &mut self,
        name: &str,
        stamp: Option<f64>,
        value: &dyn AsComponents,
    ) -> Result<()> {
        self.register(name)?;
        let time = match stamp {
            Some(stamp) => TimePoint::STATIC.with(
                Timeline::new_duration("sim_time"),
                schema::timestamp(stamp)?,
            ),
            None => TimePoint::STATIC,
        };
        let row = PendingRow::from_iter(time, value.as_serialized_batches());
        let bytes = row.total_size_bytes();
        let baseline = stamp.is_none() || name == "frames/odom" || self.voxels.contains_key(name);
        if baseline || bytes >= self.options.batch_bytes as u64 {
            if stamp.is_none() || !baseline {
                self.flush_rows()?;
            }
            let chunk = row
                .into_chunk(name.into())
                .map_err(|e| failure("build chunk", e))?;
            self.segments.append(&chunk, baseline.then_some(name))?;
            self.chunks += 1;
            return Ok(());
        }
        if self.row_bytes + bytes > self.options.batch_bytes as u64 {
            self.flush_rows()?;
        }
        let batch = if let Some(batch) = self.rows.get_mut(name) {
            batch
        } else {
            self.rows.entry(name.to_owned()).or_default()
        };
        batch.values.push(row);
        batch.bytes += bytes;
        self.row_bytes += bytes;
        if batch.values.len() >= self.options.batch_rows {
            self.flush_entity_rows(name)?;
        }
        Ok(())
    }

    fn flush_entity_rows(&mut self, name: &str) -> Result<()> {
        let Some(batch) = self.rows.remove(name) else {
            return Ok(());
        };
        self.row_bytes -= batch.bytes;
        for chunk in
            PendingRow::many_into_chunks(name.into(), self.options.batch_rows as u64, batch.values)
        {
            self.append(&chunk.map_err(|e| failure("build typed columns", e))?)?;
        }
        Ok(())
    }

    fn flush_rows(&mut self) -> Result<()> {
        while let Some(name) = self.rows.keys().next().cloned() {
            self.flush_entity_rows(&name)?;
        }
        Ok(())
    }

    fn spatial(&mut self, name: &str, stamp: f64, value: &dyn AsComponents) -> Result<()> {
        let frame = schema::frame(name).ok_or_else(|| invalid(format!("missing frame: {name}")))?;
        // Attach the frame in the same row, including after recursive clears.
        self.write(
            name,
            Some(stamp),
            &[value, &rr::CoordinateFrame::new(frame)] as &[&dyn AsComponents; 2],
        )
    }

    /// Record the atomic map←odom relation only when it changes.
    /// # Errors
    /// Invalid or reset navigation state, or serialization/I/O failure.
    pub fn navigation(&mut self, msg: NavigationMessage) -> Result<()> {
        if !msg.odom.is_initialized {
            return Ok(());
        }
        let (changed, anchor) = self.frames.navigation(msg)?;
        if changed {
            self.write(
                "frames/odom",
                Some(msg.odom.timestamp),
                &transform(msg.translation, msg.rotation)?
                    .with_parent_frame("map")
                    .with_child_frame("odom"),
            )?;
        }
        self.anchor(anchor)
    }

    /// Match truth to an initialization sample without changing either recorded pose.
    /// # Errors
    /// Invalid pose/time or serialization/I/O failure.
    pub fn truth(&mut self, msg: OdomMessage) -> Result<()> {
        let anchor = self.frames.truth(msg)?;
        self.anchor(anchor)
    }

    fn anchor(&mut self, anchor: Option<Anchor>) -> Result<()> {
        if let Some(anchor) = anchor {
            self.write(
                "frames/world",
                None,
                &transform(anchor.translation, anchor.rotation)?
                    .with_parent_frame("map")
                    .with_child_frame("world"),
            )?;
            self.log("mimir", anchor.stamp, wall_time_ns(),
                "display world aligned to map using one exact-time stationary startup pair (yaw and translation only)", "INFO")?;
        }
        Ok(())
    }

    /// Decode a `VizScalars` row from its registered diagnostic producer.
    /// # Errors
    /// Invalid entity, payload width, source or archive write.
    pub fn viz_scalar(&mut self, msg: &ScalarMessage) -> Result<()> {
        let name = schema::entity(&msg.entity, msg.entity_len)?;
        if !schema::viz_scalar_allowed(name) {
            return Err(invalid(format!("entity owned by business topic: {name}")));
        }
        let values = msg
            .scalars
            .get(..msg.scalar_count as usize)
            .ok_or_else(|| invalid("scalar count"))?;
        self.scalar(name, msg.timestamp, values)
    }

    /// Preserve every scalar row, including repeated measurement timestamps.
    /// # Errors
    /// Unregistered entity, invalid width/time, exhausted entity budget, or encoding/I/O failure.
    pub fn scalar(&mut self, name: &str, stamp: f64, values: &[f64]) -> Result<()> {
        let fields = schema::fields(name)?;
        if values.len() != fields.len() {
            return Err(invalid(format!("wrong scalar width: {name}")));
        }
        let time = schema::timestamp(stamp)?;
        if let Some(texts) = self.status.observe(name, stamp, values) {
            for (entity, text) in schema::STATUS_ENTITIES.into_iter().zip(texts) {
                self.write(entity, Some(stamp), &rr::TextDocument::from_markdown(text))?;
            }
        }
        if !self.scalars.contains_key(name) {
            let names = fields.iter().map(|field| format!("{name}/{field}"));
            self.write(name, None, &rr::SeriesLines::new().with_names(names))?;
            self.scalars.insert(
                name.to_owned(),
                Scalars {
                    times: Vec::with_capacity(self.options.batch_rows),
                    values: Vec::with_capacity(self.options.batch_rows * values.len()),
                    width: values.len(),
                },
            );
        }
        let batch = self
            .scalars
            .get_mut(name)
            .ok_or_else(|| invalid("scalar registration absent"))?;
        batch.times.push(time);
        batch.values.extend_from_slice(values);
        self.samples += 1;
        if batch.times.len() >= self.options.batch_rows {
            self.flush_scalar(name)?;
        }
        Ok(())
    }

    fn flush_scalar(&mut self, name: &str) -> Result<()> {
        let batch = self
            .scalars
            .get_mut(name)
            .ok_or_else(|| invalid("scalar registration absent"))?;
        if batch.times.is_empty() {
            return Ok(());
        }
        let columns = rr::Scalars::new(batch.values.iter().copied())
            .columns(std::iter::repeat_n(batch.width, batch.times.len()))
            .map_err(|e| failure("serialize scalar columns", e))?;
        let time = TimeColumn::new_duration_nanos("sim_time", batch.times.iter().copied());
        let chunk =
            Chunk::from_columns(name, [time], columns.map(|c| (c.descriptor, c.list_array)))
                .map_err(|e| failure("build scalar columns", e))?;
        batch.times.clear();
        batch.values.clear();
        self.append(&chunk)
    }

    /// Flush all pending rows without resampling or discarding data.
    /// # Errors
    /// Serialization or file write/flush failure.
    pub fn flush(&mut self) -> Result<()> {
        let names: Vec<_> = self.scalars.keys().cloned().collect();
        for name in names {
            self.flush_scalar(&name)?;
        }
        self.flush_rows()?;
        self.segments.flush()
    }

    /// Store an error or lifecycle event, including before the simulation clock starts.
    /// # Errors
    /// Timestamp, serialization, or I/O failure.
    pub fn log(
        &mut self,
        tag: &str,
        stamp: f64,
        wall_ns: i64,
        text: &str,
        level: &str,
    ) -> Result<()> {
        let name = format!("logs/{tag}");
        schema::entity(name.as_bytes(), name.len() as u32)?;
        self.register(&name)?;
        let time = if stamp >= 0. {
            TimePoint::STATIC.with(
                Timeline::new_duration("sim_time"),
                schema::timestamp(stamp)?,
            )
        } else {
            TimePoint::STATIC.with(Timeline::new_timestamp("wall_time"), wall_ns)
        };
        let chunk = Chunk::builder(name)
            .with_archetype_auto_row(time, &rr::TextLog::new(text).with_level(level))
            .build()
            .map_err(|e| failure("build log", e))?;
        self.append(&chunk)
    }

    /// Decode registered spatial payload kinds without retaining IPC loans.
    /// # Errors
    /// Invalid counts, geometry, timestamp, unsupported kind, or I/O failure.
    pub fn geometry(&mut self, msg: &Geometry) -> Result<()> {
        let name = &*msg.name;
        let stamp = Some(msg.timestamp);
        match &msg.data {
            GeometryData::Pose { position, rotation } => {
                self.pose(name, msg.timestamp, *position, *rotation)
            }
            GeometryData::Scalars(values) => self.scalar(name, msg.timestamp, values),
            GeometryData::Line(points) => {
                let points = points
                    .iter()
                    .map(|p| p.map(|v| v as f32))
                    .collect::<Vec<_>>();
                self.spatial(
                    name,
                    msg.timestamp,
                    &rr::LineStrips3D::new([points]).with_colors([msg.color]),
                )
            }
            GeometryData::Keypoints { points, detected } => {
                let pixels = points
                    .iter()
                    .map(|p| [p[0] as f32 + 0.5, p[1] as f32 + 0.5]);
                let output = match name {
                    "vio/diagnostics/keypoints/left" => "cameras/left/points",
                    "vio/diagnostics/keypoints/right" => "cameras/right/points",
                    _ => name,
                };
                self.point_times.insert(output.to_owned(), msg.timestamp);
                self.write(
                    output,
                    stamp,
                    &rr::Points2D::new(pixels).with_colors([msg.color]),
                )?;
                self.write(
                    output,
                    stamp,
                    &re_sdk_types::AnyValues::default()
                        .with_component::<re_sdk_types::components::Scalar>(
                            "track_id",
                            points.iter().map(|p| p[2]),
                        )
                        .with_component::<re_sdk_types::components::Scalar>(
                            "detected_count",
                            [*detected],
                        ),
                )
            }
            GeometryData::Arrows { origins, vectors } => self.spatial(
                name,
                msg.timestamp,
                &rr::Arrows3D::from_vectors(vectors.iter().map(|p| p.map(|v| v as f32)))
                    .with_origins(origins.iter().map(|p| p.map(|v| v as f32)))
                    .with_colors([msg.color]),
            ),
            GeometryData::Bars(bins) => self.write(name, stamp, &rr::BarChart::new(&**bins)),
            GeometryData::Clear => {
                self.flush()?;
                self.segments.clear(name);
                self.voxels
                    .retain(|key, _| key != name && !key.starts_with(&format!("{name}/")));
                self.previous_positions
                    .retain(|key, _| key != name && !key.starts_with(&format!("{name}/")));
                self.write(name, stamp, &rr::Clear::recursive())
            }
        }
    }

    pub(crate) fn camera_rig(&mut self, rig: &firefly_pubsub::camera::CameraRig) -> Result<()> {
        if let Some(existing) = &self.camera_rig {
            return if existing == rig {
                Ok(())
            } else {
                Err(invalid("camera rig changed during recording"))
            };
        }
        let mut chunks = Vec::with_capacity(3);
        for (name, cam) in [
            ("cameras/left", rig.left),
            ("cameras/right", rig.right),
            ("cameras/down", rig.down),
        ] {
            let [fx, fy, cx, cy] = cam.intrinsics.map(|v| v as f32);
            if cam.resolution.contains(&0)
                || ![fx, fy, cx, cy].iter().all(|v| v.is_finite())
                || fx <= 0.
                || fy <= 0.
            {
                return Err(invalid("camera calibration"));
            }
            let parent = schema::frame(name).ok_or_else(|| invalid("camera frame"))?;
            let extrinsic =
                transform(cam.translation, cam.rotation_xyzw)?.with_parent_frame(parent);
            let pinhole = rr::Pinhole::new([[fx, 0., 0.], [0., fy, 0.], [cx + 0.5, cy + 0.5, 1.]])
                .with_resolution(cam.resolution.map(|v| v as f32))
                .with_parent_frame(parent);
            let components: [&dyn AsComponents; 2] = [&pinhole, &extrinsic];
            self.register(name)?;
            let chunk =
                PendingRow::from_iter(TimePoint::STATIC, components.as_serialized_batches())
                    .into_chunk(name.into())
                    .map_err(|e| failure("build camera calibration chunk", e))?;
            chunks.push((name, chunk));
        }
        self.flush_rows()?;
        self.segments.append_snapshot(&chunks)?;
        self.chunks += chunks.len() as u64;
        self.camera_rig = Some(*rig);
        Ok(())
    }

    pub(crate) fn prediction(&mut self, msg: &firefly_pubsub::odom::VioPrediction) -> Result<()> {
        if msg.has_attitude > 1 {
            return Err(invalid("invalid prediction attitude flag"));
        }
        self.pose(
            "vio/prediction/pose_body_odom",
            msg.timestamp,
            msg.position,
            msg.quat_xyzw,
        )?;
        self.scalar(
            "vio/prediction/velocity_body_odom",
            msg.timestamp,
            &msg.velocity,
        )?;
        if msg.has_attitude == 1 {
            self.scalar(
                "vio/prediction/angular_velocity_body",
                msg.timestamp,
                &msg.angular_velocity,
            )?;
            self.scalar(
                "vio/prediction/posterior_age_s",
                msg.timestamp,
                &[msg.posterior_age_s],
            )?;
        }
        Ok(())
    }

    /// Write a pose in its original coordinate frame.
    /// # Errors
    /// Nonfinite translation, invalid quaternion/time, or file failure.
    pub fn pose(&mut self, name: &str, stamp: f64, xyz: [f64; 3], quat: [f64; 4]) -> Result<()> {
        let frame = schema::frame(name).ok_or_else(|| invalid(format!("pose frame: {name}")))?;
        self.write(
            name,
            Some(stamp),
            &transform(xyz, quat)?.with_parent_frame(frame),
        )?;
        let trajectory = match name {
            "gt/pose" => "gt/traj",
            "vio/odom" => "vio/traj",
            "vio/map/pose" => "vio/map/traj",
            _ => return Ok(()),
        };
        let point = xyz.map(|v| v as f32);
        if let Some(previous) = self.previous_positions.insert(name.to_owned(), point) {
            self.spatial(
                trajectory,
                stamp,
                &rr::LineStrips3D::new([[previous, point]]),
            )?;
        }
        Ok(())
    }

    /// Clear stale overlays without clearing points already received for this image.
    /// # Errors
    /// Invalid timestamp or a serialization/disk failure.
    pub fn image(&mut self, camera: usize, stamp: f64, value: &rr::Image) -> Result<()> {
        let side = match camera {
            0 => "left",
            1 => "right",
            3 => "down",
            _ => return Err(invalid("camera id")),
        };
        let points = format!("cameras/{side}/points");
        if self.point_times.get(&points) != Some(&stamp) {
            self.write(&points, Some(stamp), &rr::Clear::flat())?;
        }
        self.write(&format!("cameras/{side}/image"), Some(stamp), value)
    }

    /// Assemble every voxel chunk before publishing a complete map snapshot.
    /// # Errors
    /// Missing/reordered chunks, invalid geometry, missing baseline, or capacity/I/O failure.
    pub fn voxel(&mut self, msg: &VoxelChunk) -> Result<()> {
        let name = &*msg.name;
        self.register(name)?;
        if !matches!(
            msg.kind,
            kind::VOXELS | kind::VOXELS_ADD | kind::VOXELS_REMOVE
        ) || msg.chunk_count == 0
            || msg.chunk_index >= msg.chunk_count
            || msg.chunk_count as usize > self.options.max_voxels.div_ceil(VOXELS_MAX)
            || msg.voxel_size.iter().any(|v| !v.is_finite() || *v <= 0.)
            || msg.voxel_origin.iter().any(|v| !v.is_finite())
        {
            return Err(invalid("invalid voxel update metadata"));
        }
        if msg.chunk_index == 0 {
            if self.pending_voxels.is_some() {
                return Err(invalid("unfinished voxel update"));
            }
            self.pending_voxels = Some(PendingVoxels {
                name: name.to_owned(),
                stamp: msg.timestamp,
                kind: msg.kind,
                chunks: msg.chunk_count,
                next: 0,
                size: msg.voxel_size,
                origin: msg.voxel_origin,
                cells: BTreeSet::new(),
            });
        }
        let pending = self
            .pending_voxels
            .as_mut()
            .ok_or_else(|| invalid("missing voxel first chunk"))?;
        if pending.name != name
            || pending.stamp != msg.timestamp
            || pending.kind != msg.kind
            || pending.chunks != msg.chunk_count
            || pending.next != msg.chunk_index
            || pending.size != msg.voxel_size
            || pending.origin != msg.voxel_origin
        {
            return Err(invalid("mismatched voxel chunk"));
        }
        pending.append(&msg.voxels, self.options.max_voxels)?;
        if pending.next < pending.chunks {
            return Ok(());
        }
        let pending = self
            .pending_voxels
            .take()
            .ok_or_else(|| invalid("missing voxel update"))?;
        if msg.kind == kind::VOXELS {
            self.voxels.insert(
                name.to_owned(),
                Voxels {
                    cells: pending.cells,
                    size: msg.voxel_size,
                    origin: msg.voxel_origin,
                    stamp: msg.timestamp,
                },
            );
        } else {
            let state = self
                .voxels
                .get_mut(name)
                .ok_or_else(|| invalid("voxel delta without baseline"))?;
            if state.size != msg.voxel_size
                || state.origin != msg.voxel_origin
                || msg.timestamp < state.stamp
            {
                return Err(invalid("voxel geometry or time changed"));
            }
            for cell in pending.cells {
                if msg.kind == kind::VOXELS_ADD {
                    state.cells.insert(cell);
                } else {
                    state.cells.remove(&cell);
                }
            }
            state.stamp = msg.timestamp;
        }
        let state = &self.voxels[name];
        if state.cells.len() > self.options.max_voxels {
            return Err(invalid("voxel map budget exceeded"));
        }
        let map = rr::VoxelGridMap::new(state.cells.iter().copied(), state.size)
            .with_translation(state.origin);
        self.spatial(name, msg.timestamp, &map)
    }

    /// Finish pending data and write the complete RRD footer before syncing the file.
    /// # Errors
    /// Incomplete voxel update or any serialization/flush/sync failure.
    pub fn finish(mut self, error: Option<&str>) -> Result<()> {
        self.flush()?;
        if self.frames.truth_unaligned() {
            self.log("mimir", -1., wall_time_ns(),
                "world display frame is disconnected: no exact-time stationary session-start pair; inspect truth in a world-frame view", "WARN")?;
        }
        let incomplete = error
            .or_else(|| {
                self.pending_voxels
                    .is_some()
                    .then_some("incomplete voxel update")
            })
            .or_else(|| {
                self.segments
                    .lost_data()
                    .then_some("disk pressure interrupted recording")
            });
        if let Some(detail) = incomplete {
            self.segments.gap(detail);
        }
        self.log(
            "mimir",
            -1.,
            wall_time_ns(),
            incomplete.unwrap_or("recording complete"),
            if incomplete.is_some() {
                "ERROR"
            } else {
                "INFO"
            },
        )?;
        self.segments.finish()?;
        if self.pending_voxels.is_some() {
            return Err(invalid("incomplete voxel update"));
        }
        Ok(())
    }
}

fn transform(xyz: [f64; 3], quat: [f64; 4]) -> Result<rr::Transform3D> {
    if xyz.iter().chain(&quat).any(|v| !v.is_finite())
        || (quat.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-5
    {
        return Err(invalid("invalid pose"));
    }
    Ok(rr::Transform3D::from_translation_rotation(
        xyz.map(|v| v as f32),
        Quaternion::from_xyzw(quat.map(|v| v as f32)),
    ))
}

pub(crate) fn wall_time_ns() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_nanos().min(i64::MAX as u128) as i64)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        fs,
        io::BufReader,
        path::{Path, PathBuf},
    };

    use re_log_encoding::{DecoderApp, read_rrd_footer};
    use re_log_msg::LogMsg;

    fn test_run(config: &str) -> (PathBuf, Run) {
        let root = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../target/mimir-tests")
            .join(format!("{}-{}", std::process::id(), wall_time_ns()));
        fs::create_dir_all(root.join("configs")).unwrap();
        fs::write(root.join("configs/archive.toml"), config).unwrap();
        let run = Run::create_at(&root, "test").unwrap();
        (root, run)
    }

    #[test]
    fn disk_pressure_resumes_with_a_new_indexed_segment_and_fails_completeness() {
        let (root, run) =
            test_run("keep_free_bytes = 0\nmax_bytes = 16777216\ncleanup_bytes = 12582912\n");
        let mut competing = run.rrd_segment().unwrap();
        competing.reserve(16 * 1024 * 1024 - 384 * 1024).unwrap();
        let path = run.path().to_owned();
        let mut archive = Archive::in_run(Options::default(), run).unwrap();
        assert!(archive.segments.lost_data());
        competing.discard().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(1050));
        archive
            .log("test", 7., wall_time_ns(), "recovered", "INFO")
            .unwrap();
        assert!(archive.finish(None).is_err());
        let files = fs::read_dir(path.join("rrd"))
            .unwrap()
            .map(|e| e.unwrap().path())
            .collect::<Vec<_>>();
        assert_eq!(files.len(), 1);
        let file = fs::File::open(&files[0]).unwrap();
        assert!(
            futures_lite::future::block_on(read_rrd_footer(&file))
                .unwrap()
                .is_some()
        );
        let entities: BTreeSet<_> = DecoderApp::decode_lazy(BufReader::new(file))
            .filter_map(|msg| match msg.unwrap() {
                LogMsg::ArrowMsg(store, msg) if store.is_recording() => {
                    Some(msg.batch.entity_path().to_string())
                }
                _ => None,
            })
            .collect();
        assert!(entities.contains("/mimir/config") && entities.contains("/logs/test"));
        assert!(
            fs::read_to_string(path.join("manifest.json"))
                .unwrap()
                .contains("RRD chunks lost")
        );
        fs::remove_dir_all(root).unwrap();
    }

    fn register_test_rig(archive: &mut Archive) {
        let camera = firefly_pubsub::camera::CameraCalibration {
            resolution: [320, 240],
            intrinsics: [200., 200., 159.5, 119.5],
            translation: [0.; 3],
            rotation_xyzw: [0., 0., 0., 1.],
        };
        let rig = firefly_pubsub::camera::CameraRig {
            left: camera,
            right: camera,
            down: camera,
        };
        archive.camera_rig(&rig).unwrap();
        archive.camera_rig(&rig).unwrap();
        let mut changed = rig;
        changed.left.intrinsics[0] += 1.;
        assert!(archive.camera_rig(&changed).is_err());
    }

    #[test]
    fn segment_rotation_keeps_the_three_camera_calibrations_together() {
        let (root, run) = test_run("keep_free_bytes = 0\nrrd_segment_bytes = 1\n");
        let directory = run.path().join("rrd");
        let mut archive = Archive::in_run(Options::default(), run).unwrap();
        register_test_rig(&mut archive);
        archive.finish(None).unwrap();
        let mut camera_segments = 0;
        for entry in fs::read_dir(directory).unwrap() {
            let file = fs::File::open(entry.unwrap().path()).unwrap();
            let cameras: BTreeSet<_> = DecoderApp::decode_lazy(BufReader::new(file))
                .filter_map(|msg| match msg.unwrap() {
                    LogMsg::ArrowMsg(store, msg) if store.is_recording() => {
                        Some(msg.batch.entity_path().to_string())
                    }
                    _ => None,
                })
                .filter(|path| path.starts_with("/cameras/"))
                .collect();
            if !cameras.is_empty() {
                assert_eq!(cameras.len(), 3);
                camera_segments += 1;
            }
        }
        assert!(camera_segments > 0);
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn rotation_keeps_standalone_baselines_and_exact_measurements() {
        let (root, run) = test_run("keep_free_bytes = 0\nrrd_segment_bytes = 720000\n");
        let directory = run.path().join("rrd");
        let mut archive = Archive::in_run(
            Options {
                batch_rows: 1,
                ..Options::default()
            },
            run.clone(),
        )
        .unwrap();
        assert!(Archive::in_run(Options::default(), run).is_err());
        archive
            .navigation(NavigationMessage {
                odom: OdomMessage {
                    timestamp: 1.,
                    is_initialized: true,
                    ..Default::default()
                },
                rotation: [0., 0., 0., 1.],
                ..Default::default()
            })
            .unwrap();
        let mut map = firefly_pubsub::viz::VoxelMessage::base(kind::VOXELS, 1., "plan/collision");
        map.voxel_size = [0.1; 3];
        map.voxel_count = 1;
        map.voxels[0] = [2, 3, 5];
        archive
            .voxel(&VoxelChunk::from_message(&map).unwrap())
            .unwrap();
        register_test_rig(&mut archive);
        let mut noise = 1_u32;
        for stamp in 0..20 {
            let pixels: Vec<u8> = (0..256 * 256)
                .map(|_| {
                    noise ^= noise << 13;
                    noise ^= noise >> 17;
                    noise ^= noise << 5;
                    noise as u8
                })
                .collect();
            archive
                .image(3, f64::from(stamp), &rr::Image::from_l8(pixels, [256, 256]))
                .unwrap();
        }
        archive.finish(None).unwrap();
        let mut files: Vec<_> = fs::read_dir(directory)
            .unwrap()
            .map(|e| e.unwrap().path())
            .collect();
        files.retain(|p| p.extension().is_some_and(|s| s == "rrd"));
        files.sort();
        assert!(files.len() > 1);
        let mut images = BTreeSet::new();
        let mut baseline_ids = BTreeMap::new();
        let mut recording_ids = BTreeSet::new();
        for path in files {
            let file = fs::File::open(path).unwrap();
            let footer = futures_lite::future::block_on(read_rrd_footer(&file))
                .unwrap()
                .unwrap();
            assert!(
                footer
                    .manifests
                    .keys()
                    .any(re_log_types::StoreId::is_blueprint)
            );
            let mut baselines = BTreeMap::new();
            for msg in DecoderApp::decode_lazy(BufReader::new(file)) {
                if let LogMsg::ArrowMsg(store, msg) = msg.unwrap()
                    && store.is_recording()
                {
                    recording_ids.insert(store);
                    let name = msg.batch.entity_path().to_string();
                    let id = msg.batch.chunk_id();
                    match name.as_str() {
                        "/frames/odom" | "/plan/collision" | "/mimir/config" | "/cameras/left"
                        | "/cameras/right" | "/cameras/down" => {
                            assert!(baselines.insert(name, id).is_none());
                        }
                        "/cameras/down/image" => assert!(images.insert(id)),
                        _ => (),
                    }
                }
            }
            assert_eq!(baselines.len(), 6);
            if !baseline_ids.is_empty() {
                assert_eq!(baseline_ids, baselines);
            }
            baseline_ids = baselines;
        }
        assert_eq!(images.len(), 20);
        assert_eq!(recording_ids.len(), 1);
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn rejected_chunk_does_not_advance_or_mutate_the_pending_update() {
        let mut pending = PendingVoxels {
            name: "test".into(),
            stamp: 1.,
            kind: kind::VOXELS,
            chunks: 3,
            next: 0,
            size: [1.; 3],
            origin: [0.; 3],
            cells: BTreeSet::new(),
        };
        pending.append(&[[1, 0, 0], [1, 0, 0]], 2).unwrap();
        assert_eq!(pending.next, 1);
        assert_eq!(pending.cells.len(), 1);
        assert!(pending.append(&[[2, 0, 0], [3, 0, 0]], 2).is_err());
        assert_eq!(pending.next, 1);
        assert_eq!(pending.cells, BTreeSet::from([[1, 0, 0]]));
        pending.append(&[[1, 0, 0], [2, 0, 0]], 2).unwrap();
        assert_eq!(pending.next, 2);
        assert_eq!(pending.cells.len(), 2);
    }
}
