//! Versioned numerical inputs for replaying global BA. This is deliberately not
//! a persistent-map checkpoint: images, descriptors, features, aliases and map
//! lifecycle state are absent. Restored snapshots are for optimization only.
use crate::{
    global_optimization::{
        KrylovPrecision, OptimizationOptions, OptimizationResult, Precision, optimize_global,
    },
    map::{MapKeyframe, MapLandmark, MapObservation, MapSnapshot, ObservationKey},
    pose_graph::{EdgeKind, PoseGraphEdge},
};
use firefly_calibration::calibration::Calibration;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Isometry3, Quaternion, SMatrix, Translation3, UnitQuaternion, Vector2, Vector3};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    fs::{File, OpenOptions},
    io::{BufReader, BufWriter, Write},
    path::Path,
};

pub const FORMAT_VERSION: u32 = 1;
const FORMAT_NAME: &str = "rootslam-global-ba-numerical-problem";
fn invalid(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message.into()).with_operation("rootslam.global_problem")
}
fn io_error(error: std::io::Error) -> Error {
    Error::from(error).with_operation("rootslam.global_problem.io")
}
fn has_null(value: &serde_json::Value) -> bool {
    match value {
        serde_json::Value::Null => true,
        serde_json::Value::Array(a) => a.iter().any(has_null),
        serde_json::Value::Object(m) => m.values().any(has_null),
        _ => false,
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Pose {
    pub translation: [f64; 3],
    /// Raw quaternion coefficients, without normalization on restoration.
    pub rotation_xyzw: [f64; 4],
}
impl Pose {
    fn capture(pose: &Isometry3<f64>) -> Self {
        let q = pose.rotation.quaternion();
        Self {
            translation: [pose.translation.x, pose.translation.y, pose.translation.z],
            rotation_xyzw: [q.i, q.j, q.k, q.w],
        }
    }
    fn restore(&self) -> Isometry3<f64> {
        let [x, y, z, w] = self.rotation_xyzw;
        Isometry3::from_parts(
            Translation3::from(Vector3::from(self.translation)),
            UnitQuaternion::new_unchecked(Quaternion::new(w, x, y, z)),
        )
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Keyframe {
    pub map_key: i64,
    pub id: i64,
    pub pose: Pose,
    pub active: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Observation {
    pub keyframe: i64,
    pub camera: usize,
    pub feature: usize,
    pub pixel: [f64; 2],
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Landmark {
    pub map_key: u64,
    pub id: u64,
    pub host: i64,
    pub position: [f64; 3],
    pub observations: Vec<Observation>,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EdgeType {
    Temporal,
    Covisibility,
    Loop,
    SpanningTree,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Edge {
    pub from: i64,
    pub to: i64,
    pub measurement: Pose,
    pub kind: EdgeType,
    /// Explicit row-major storage, independent of nalgebra's storage layout.
    pub sqrt_information: [[f64; 6]; 6],
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GlobalProblem {
    pub format: String,
    pub version: u32,
    pub revision: u64,
    pub calibration: Calibration<f64>,
    pub options: OptimizationOptions,
    pub keyframes: Vec<Keyframe>,
    pub landmarks: Vec<Landmark>,
    pub edges: Vec<Edge>,
}
impl GlobalProblem {
    /// Copies every input consumed by global optimization, preserving map keys
    /// separately from object IDs and preserving edge/observation iteration order.
    /// # Errors
    /// Rejects nonfinite numbers, which JSON cannot represent losslessly.
    pub fn capture(
        snapshot: &MapSnapshot,
        calibration: &Calibration<f64>,
        options: &OptimizationOptions,
    ) -> Result<Self> {
        let result = Self {
            format: FORMAT_NAME.into(),
            version: FORMAT_VERSION,
            revision: snapshot.revision,
            calibration: calibration.clone(),
            options: options.clone(),
            keyframes: snapshot
                .keyframes
                .iter()
                .map(|(&map_key, k)| Keyframe {
                    map_key,
                    id: k.id,
                    pose: Pose::capture(&k.t_w_b),
                    active: k.active,
                })
                .collect(),
            landmarks: snapshot
                .landmarks
                .iter()
                .map(|(&map_key, p)| Landmark {
                    map_key,
                    id: p.id,
                    host: p.host,
                    position: [p.position.x, p.position.y, p.position.z],
                    observations: p
                        .observations
                        .iter()
                        .map(|(key, o)| Observation {
                            keyframe: key.keyframe,
                            camera: key.camera,
                            feature: key.feature,
                            pixel: [o.pixel.x, o.pixel.y],
                        })
                        .collect(),
                })
                .collect(),
            edges: snapshot
                .edges
                .iter()
                .map(|e| Edge {
                    from: e.from,
                    to: e.to,
                    measurement: Pose::capture(&e.measurement),
                    kind: match e.kind {
                        EdgeKind::Temporal => EdgeType::Temporal,
                        EdgeKind::Covisibility => EdgeType::Covisibility,
                        EdgeKind::Loop => EdgeType::Loop,
                        EdgeKind::SpanningTree => EdgeType::SpanningTree,
                    },
                    sqrt_information: std::array::from_fn(|r| {
                        std::array::from_fn(|c| e.sqrt_information[(r, c)])
                    }),
                })
                .collect(),
        };
        result.validate()?;
        Ok(result)
    }
    fn validate(&self) -> Result<()> {
        if self.format != FORMAT_NAME || self.version != FORMAT_VERSION {
            return Err(invalid("Unsupported global problem format/version"));
        }
        let pose_finite = |p: &Pose| {
            p.translation
                .iter()
                .chain(p.rotation_xyzw.iter())
                .all(|v| v.is_finite())
        };
        if self.keyframes.iter().any(|k| !pose_finite(&k.pose))
            || self.landmarks.iter().any(|p| {
                p.position.iter().any(|v| !v.is_finite())
                    || p.observations
                        .iter()
                        .any(|o| o.pixel.iter().any(|v| !v.is_finite()))
            })
            || self.edges.iter().any(|e| {
                !pose_finite(&e.measurement)
                    || e.sqrt_information.iter().flatten().any(|v| !v.is_finite())
            })
        {
            return Err(invalid("Nonfinite numerical problem input"));
        }
        // Calibration's existing archive contains no optional values; null means
        // a nonfinite float was encountered during its JSON serialization.
        let calibration =
            serde_json::to_value(&self.calibration).map_err(|e| invalid(e.to_string()))?;
        if has_null(&calibration) {
            return Err(invalid("Nonfinite calibration in numerical archive"));
        }
        self.options.validate()
    }
    /// Creates a new file and refuses to overwrite any existing artifact.
    /// # Errors
    /// Reports invalid inputs, existing destinations, or serialization/I/O errors.
    pub fn write_new(&self, path: &Path) -> Result<()> {
        self.validate()?;
        write_new(path, self)
    }
    /// # Errors
    /// Reports corrupt archives, unknown versions or nonfinite values.
    pub fn load(path: &Path) -> Result<Self> {
        let archive: Self =
            serde_json::from_reader(BufReader::new(File::open(path).map_err(io_error)?))
                .map_err(|e| invalid(e.to_string()))?;
        archive.validate()?;
        Ok(archive)
    }
    /// Restores a snapshot for global optimization. Feature and image fields are
    /// empty; persistent mapping and loop detection use live map snapshots.
    /// # Errors
    /// Rejects unsupported versions, invalid numeric inputs and duplicate keys.
    pub fn restore(&self) -> Result<(MapSnapshot, Calibration<f64>, OptimizationOptions)> {
        self.validate()?;
        let mut snapshot = MapSnapshot {
            revision: self.revision,
            ..Default::default()
        };
        for k in &self.keyframes {
            if snapshot
                .keyframes
                .insert(
                    k.map_key,
                    MapKeyframe {
                        id: k.id,
                        t_w_b: k.pose.restore(),
                        active: k.active,
                        features: Vec::new(),
                        images: None,
                    },
                )
                .is_some()
            {
                return Err(invalid("Duplicate archived keyframe map key"));
            }
        }
        for p in &self.landmarks {
            let mut observations = BTreeMap::new();
            for o in &p.observations {
                if observations
                    .insert(
                        ObservationKey {
                            keyframe: o.keyframe,
                            camera: o.camera,
                            feature: o.feature,
                        },
                        MapObservation {
                            pixel: Vector2::from(o.pixel),
                        },
                    )
                    .is_some()
                {
                    return Err(invalid("Duplicate archived landmark observation"));
                }
            }
            if snapshot
                .landmarks
                .insert(
                    p.map_key,
                    MapLandmark {
                        id: p.id,
                        host: p.host,
                        position: Vector3::from(p.position),
                        observations,
                    },
                )
                .is_some()
            {
                return Err(invalid("Duplicate archived landmark map key"));
            }
        }
        snapshot.edges = self
            .edges
            .iter()
            .map(|e| PoseGraphEdge {
                from: e.from,
                to: e.to,
                measurement: e.measurement.restore(),
                kind: match e.kind {
                    EdgeType::Temporal => EdgeKind::Temporal,
                    EdgeType::Covisibility => EdgeKind::Covisibility,
                    EdgeType::Loop => EdgeKind::Loop,
                    EdgeType::SpanningTree => EdgeKind::SpanningTree,
                },
                sqrt_information: SMatrix::from_fn(|r, c| e.sqrt_information[r][c]),
            })
            .collect();
        Ok((snapshot, self.calibration.clone(), self.options.clone()))
    }
    /// Replays the same inputs and numerical thresholds with only scalar/Krylov
    /// precision changed. Diagnostic archive paths are disabled for replay.
    /// # Errors
    /// Propagates archive validation and solver failures without relaxing gates.
    pub fn replay(
        &self,
        precision: Precision,
        krylov_precision: KrylovPrecision,
    ) -> Result<ReplayResult> {
        let (snapshot, calibration, mut options) = self.restore()?;
        options.precision = precision;
        options.krylov_precision = krylov_precision;
        options.linear_system_archive = None;
        options.problem_archive = None;
        let result = optimize_global(&snapshot, &calibration, &options)?;
        Ok(ReplayResult::from_result(&result, options))
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReplayResult {
    pub revision: u64,
    pub options: OptimizationOptions,
    pub poses: Vec<(i64, Pose)>,
    pub points: Vec<(u64, [f64; 3])>,
    pub initial_cost: f64,
    pub final_cost: f64,
    pub iterations: usize,
    pub accepted_steps: usize,
    pub converged: bool,
    pub linear_iterations: usize,
    pub peak_linear_storage_bytes: usize,
}
impl ReplayResult {
    /// Records a solve and the actual options used, including explicit replay
    /// iteration budgets or diagnostic paths supplied by the caller.
    #[must_use]
    pub fn from_result(result: &OptimizationResult, options: OptimizationOptions) -> Self {
        Self {
            revision: result.revision,
            options,
            poses: result
                .poses
                .iter()
                .map(|(&id, p)| (id, Pose::capture(p)))
                .collect(),
            points: result
                .points
                .iter()
                .map(|(&id, p)| (id, [p.x, p.y, p.z]))
                .collect(),
            initial_cost: result.initial_cost,
            final_cost: result.final_cost,
            iterations: result.iterations,
            accepted_steps: result.accepted_steps,
            converged: result.converged,
            linear_iterations: result.linear_iterations,
            peak_linear_storage_bytes: result.peak_linear_storage_bytes,
        }
    }
    /// # Errors
    /// Refuses overwrites and reports serialization or I/O failures.
    pub fn write_new(&self, path: &Path) -> Result<()> {
        write_new(path, self)
    }
}
fn write_new(path: &Path, value: &impl Serialize) -> Result<()> {
    let file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(io_error)?;
    let mut output = BufWriter::new(file);
    serde_json::to_writer(&mut output, value).map_err(|e| invalid(e.to_string()))?;
    output.flush().map_err(io_error)
}
