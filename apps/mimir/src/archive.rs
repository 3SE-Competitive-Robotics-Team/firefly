//! Synchronous RRD encoding: disk backpressure cannot accumulate in an SDK queue.
use std::collections::{BTreeMap, BTreeSet};
use std::fs::{File, OpenOptions};
use std::io::BufWriter;
use std::path::Path;

use firefly_error::Result;
use firefly_pubsub::viz::{VizMessage, VoxelMessage, kind};
use re_chunk::{Chunk, RowId, TimeColumn, TimePoint, Timeline};
use re_log_encoding::{Encoder, EncodingOptions};
use re_log_types::{LogMsg, SetStoreInfo, StoreId, StoreInfo, StoreKind, StoreSource};
use re_sdk_types::{AsComponents, archetypes as rr, datatypes::Quaternion};

use crate::{config::Options, failure, invalid, schema};

#[derive(Default)]
struct Scalars {
    times: Vec<i64>,
    values: Vec<f64>,
    width: usize,
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

pub struct Archive {
    encoder: Encoder<BufWriter<File>>,
    store: StoreId,
    options: Options,
    scalars: BTreeMap<String, Scalars>,
    entities: BTreeSet<String>,
    voxels: BTreeMap<String, Voxels>,
    pending_voxels: Option<PendingVoxels>,
    previous_positions: BTreeMap<String, [f32; 3]>,
    point_times: BTreeMap<String, f64>,
    pub samples: u64,
    pub chunks: u64,
}

impl Archive {
    /// Create a new file without overwriting existing evidence.
    /// # Errors
    /// Invalid budgets, an existing path, or encoder/file initialization failure.
    pub fn create(path: &Path, options: Options) -> Result<Self> {
        options.validate()?;
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            std::fs::create_dir_all(parent)
                .map_err(|e| failure("create recording directory", e))?;
        }
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|e| failure("create recording", e))?;
        let mut encoder = Encoder::new_eager(
            re_build_info::CrateVersion::LOCAL,
            EncodingOptions::PROTOBUF_COMPRESSED,
            BufWriter::new(file),
        )
        .map_err(|e| failure("initialize RRD", e))?;
        let store = StoreId::random(StoreKind::Recording, "firefly");
        encoder
            .append(&LogMsg::SetStoreInfo(SetStoreInfo {
                row_id: *RowId::new(),
                info: StoreInfo::new(store.clone(), StoreSource::Unknown),
            }))
            .map_err(|e| failure("write store info", e))?;
        let mut archive = Self {
            encoder,
            store,
            options,
            scalars: BTreeMap::new(),
            entities: BTreeSet::new(),
            voxels: BTreeMap::new(),
            pending_voxels: None,
            previous_positions: BTreeMap::new(),
            point_times: BTreeMap::new(),
            samples: 0,
            chunks: 0,
        };
        archive.write("/", None, &rr::ViewCoordinates::RIGHT_HAND_Z_UP())?;
        archive.write(
            "mimir/config",
            None,
            &rr::TextDocument::new(format!("{:?}", archive.options)),
        )?;
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
        let msg = chunk
            .to_arrow_msg()
            .map_err(|e| failure("serialize chunk", e))?;
        self.encoder
            .append(&LogMsg::ArrowMsg(self.store.clone(), msg))
            .map_err(|e| failure("write RRD", e))?;
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
            Some(stamp) => TimePoint::default().with(
                Timeline::new_duration("sim_time"),
                schema::timestamp(stamp)?,
            ),
            None => TimePoint::default(),
        };
        let chunk = Chunk::builder(name)
            .with_archetype(RowId::new(), time, value)
            .build()
            .map_err(|e| failure("build chunk", e))?;
        self.append(&chunk)
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
        self.encoder
            .flush_blocking()
            .map_err(|e| failure("flush RRD", e))
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
            TimePoint::default().with(
                Timeline::new_duration("sim_time"),
                schema::timestamp(stamp)?,
            )
        } else {
            TimePoint::default().with(Timeline::new_timestamp("wall_time"), wall_ns)
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
    #[allow(clippy::too_many_lines)] // One exhaustive wire-kind dispatcher.
    pub fn geometry(&mut self, msg: &VizMessage) -> Result<()> {
        let name = schema::entity(&msg.entity, msg.entity_len)?;
        schema::spatial(name, msg.kind)?;
        schema::timestamp(msg.timestamp)?;
        let stamp = Some(msg.timestamp);
        match msg.kind {
            kind::POSE => self.pose(name, msg.timestamp, msg.xyz, msg.quat_xyzw),
            kind::SCALARS => self.scalar(
                name,
                msg.timestamp,
                msg.scalars
                    .get(..msg.scalar_count as usize)
                    .ok_or_else(|| invalid("scalar count"))?,
            ),
            kind::LINE_STRIP => {
                let points = msg
                    .points
                    .get(..msg.point_count as usize)
                    .ok_or_else(|| invalid("line count"))?;
                let points = points
                    .iter()
                    .map(|p| p.map(|v| v as f32))
                    .collect::<Vec<_>>();
                self.write(
                    name,
                    stamp,
                    &rr::LineStrips3D::new([points]).with_colors([msg.color]),
                )
            }
            kind::KEYPOINTS_2D => {
                let points = msg
                    .points
                    .get(..msg.point_count as usize)
                    .ok_or_else(|| invalid("keypoint count"))?;
                if msg.scalar_count != 3
                    || msg.scalars[2] < f64::from(msg.point_count)
                    || points.iter().flatten().any(|v| !v.is_finite())
                {
                    return Err(invalid("keypoint metadata"));
                }
                let pixels = points
                    .iter()
                    .map(|p| [p[0] as f32 + 0.5, p[1] as f32 + 0.5]);
                let output = match name {
                    "okvis/debug/keypoints/left" | "vio/diagnostics/keypoints/left" => {
                        "cameras/left/points"
                    }
                    "okvis/debug/keypoints/right" | "vio/diagnostics/keypoints/right" => {
                        "cameras/right/points"
                    }
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
                            [msg.scalars[2]],
                        ),
                )
            }
            kind::ARROWS => {
                let n = msg.arrow_count as usize;
                let origins = msg
                    .arrow_origins
                    .get(..n)
                    .ok_or_else(|| invalid("arrow count"))?;
                let vectors = msg
                    .arrow_vectors
                    .get(..n)
                    .ok_or_else(|| invalid("arrow count"))?;
                self.write(
                    name,
                    stamp,
                    &rr::Arrows3D::from_vectors(vectors.iter().map(|p| p.map(|v| v as f32)))
                        .with_origins(origins.iter().map(|p| p.map(|v| v as f32)))
                        .with_colors([msg.color]),
                )
            }
            kind::BAR_CHART => self.write(
                name,
                stamp,
                &rr::BarChart::new(
                    msg.bins
                        .get(..msg.bin_count as usize)
                        .ok_or_else(|| invalid("histogram count"))?,
                ),
            ),
            kind::CLEAR => {
                self.flush()?;
                self.voxels
                    .retain(|key, _| key != name && !key.starts_with(&format!("{name}/")));
                self.previous_positions
                    .retain(|key, _| key != name && !key.starts_with(&format!("{name}/")));
                self.write(name, stamp, &rr::Clear::recursive())
            }
            kind::CAMERA => {
                if msg.scalar_count != 4
                    || msg.point_count != 1
                    || msg.scalars[..2].iter().any(|v| !v.is_finite() || *v <= 0.)
                {
                    return Err(invalid("camera calibration"));
                }
                self.write(
                    name,
                    None,
                    &rr::Pinhole::new([
                        [msg.scalars[0] as f32, 0., 0.],
                        [0., msg.scalars[1] as f32, 0.],
                        [msg.scalars[2] as f32 + 0.5, msg.scalars[3] as f32 + 0.5, 1.],
                    ])
                    .with_resolution([msg.points[0][0] as f32, msg.points[0][1] as f32]),
                )?;
                self.write(name, None, &transform(msg.xyz, msg.quat_xyzw)?)
            }
            _ => Err(invalid(format!("unsupported geometry kind {}", msg.kind))),
        }
    }

    /// Write a pose in its original coordinate frame.
    /// # Errors
    /// Nonfinite translation, invalid quaternion/time, or file failure.
    pub fn pose(&mut self, name: &str, stamp: f64, xyz: [f64; 3], quat: [f64; 4]) -> Result<()> {
        self.write(name, Some(stamp), &transform(xyz, quat)?)?;
        let trajectory = match name {
            "gt/pose" => "gt/traj",
            "okvis/odom" => "okvis/traj",
            "okvis/map/pose" => "okvis/map/traj",
            _ => return Ok(()),
        };
        let point = xyz.map(|v| v as f32);
        if let Some(previous) = self.previous_positions.insert(name.to_owned(), point) {
            self.write(
                trajectory,
                Some(stamp),
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
    pub fn voxel(&mut self, msg: &VoxelMessage) -> Result<()> {
        let name = schema::entity(&msg.entity, msg.entity_len)?;
        schema::spatial(name, msg.kind)?;
        schema::timestamp(msg.timestamp)?;
        self.register(name)?;
        if !matches!(
            msg.kind,
            kind::VOXELS | kind::VOXELS_ADD | kind::VOXELS_REMOVE
        ) || msg.chunk_count == 0
            || msg.chunk_index >= msg.chunk_count
            || msg.chunk_count as usize > self.options.max_voxels.div_ceil(msg.voxels.len())
            || msg.voxel_size.iter().any(|v| !v.is_finite() || *v <= 0.)
            || msg.voxel_origin.iter().any(|v| !v.is_finite())
        {
            return Err(invalid("invalid voxel update metadata"));
        }
        let incoming = msg
            .voxels
            .get(..msg.voxel_count as usize)
            .ok_or_else(|| invalid("voxel count"))?;
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
        pending.cells.extend(incoming);
        if pending.cells.len() > self.options.max_voxels {
            return Err(invalid("voxel update budget exceeded"));
        }
        pending.next += 1;
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
        self.write(name, Some(msg.timestamp), &map)
    }

    /// Finish pending data and write the complete RRD footer before syncing the file.
    /// # Errors
    /// Incomplete voxel update or any serialization/flush/sync failure.
    pub fn finish(mut self, error: Option<&str>) -> Result<()> {
        self.flush()?;
        let incomplete = error.or_else(|| {
            self.pending_voxels
                .is_some()
                .then_some("incomplete voxel update")
        });
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
        self.encoder
            .finish()
            .map_err(|e| failure("finish RRD footer", e))?;
        self.encoder
            .flush_blocking()
            .map_err(|e| failure("flush RRD footer", e))?;
        let file = self
            .encoder
            .into_inner()
            .map_err(|e| failure("close RRD", e))?;
        file.get_ref()
            .sync_all()
            .map_err(|e| failure("sync RRD", e))?;
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
