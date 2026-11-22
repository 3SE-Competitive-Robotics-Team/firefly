//! Independent acceptance of closed Mimir recordings.
mod flight;
mod recording;
mod resources;
mod run;
mod timeline;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde::Deserialize;
use serde_json::{Value, json};
use std::{collections::BTreeMap, path::Path};

type Vec3 = Vector3<f64>;
type Rotation = UnitQuaternion<f64>;
type Event = (f64, Value);
type Pose = (f64, Vec3, [f64; 4]);
type Scalar = (f64, Vec<f64>);
#[derive(Default, Deserialize)]
pub struct Evidence {
    events: Vec<Event>,
    poses: Vec<Pose>,
    scalars: BTreeMap<String, Vec<Scalar>>,
    down_frames: Vec<(f64, u32, u32, usize)>,
    recording_complete: bool,
    capture: Value,
}
impl Evidence {
    fn scalar(&self, name: &str, a: f64, b: f64) -> &[Scalar] {
        window(
            self.scalars.get(name).map_or(&[], Vec::as_slice),
            a,
            b,
            |s| s.0,
        )
    }
}
/// Borrow one sortie's rows, including the settling interval and the final lease acknowledgement.
struct FlightEvidence<'a> {
    source: &'a Evidence,
    events: &'a [Event],
    poses: &'a [Pose],
    down_frames: &'a [(f64, u32, u32, usize)],
    start: f64,
    release_end: f64,
}
impl<'a> FlightEvidence<'a> {
    fn new(
        source: &'a Evidence,
        events: &'a [Event],
        start: f64,
        end: f64,
        release_end: f64,
    ) -> Self {
        Self {
            source,
            events,
            poses: window(&source.poses, start - 0.05, end + 0.05, |s| s.0),
            down_frames: window(&source.down_frames, start, end, |s| s.0),
            start,
            release_end,
        }
    }
    fn scalar(&self, name: &str, start: f64, end: f64) -> &[Scalar] {
        self.source
            .scalar(name, self.start.max(start), self.release_end.min(end))
    }
}
fn window<T>(rows: &[T], a: f64, b: f64, time: impl Fn(&T) -> f64) -> &[T] {
    let start = rows.partition_point(|v| time(v) < a);
    let end = rows.partition_point(|v| time(v) <= b);
    &rows[start.min(end)..end]
}
fn rotation(q: [f64; 4]) -> Rotation {
    Rotation::new_normalize(Quaternion::new(q[3], q[0], q[1], q[2]))
}
#[derive(Default)]
struct Report {
    failures: Vec<String>,
    metrics: serde_json::Map<String, Value>,
}
impl Report {
    fn check(&mut self, valid: bool, message: impl Into<String>) {
        if !valid {
            self.failures.push(message.into());
        }
    }
    fn value(self) -> Value {
        json!({"passed":self.failures.is_empty(),"failures":self.failures,"metrics":self.metrics})
    }
}
/// Evaluate recorded measurements in their original time order.
#[must_use]
pub fn evaluate(data: &Evidence, transport: Option<&str>) -> Value {
    run::evaluate(data, transport).unwrap_or_else(|e| invalid(&e.to_string()))
}
fn invalid(message: &str) -> Value {
    json!({"passed":false,"failures":[format!("invalid recording: {message}")],"metrics":{}})
}
/// Decode one closed recording and grade its observed flights.
#[must_use]
pub fn verify(path: &Path, transport: Option<&str>) -> Value {
    match recording::read(path) {
        Ok(d) => evaluate(&d, transport),
        Err(e) => invalid(&e.with_context("input", path.display()).to_string()),
    }
}
