// Copyright (c) 2020-2021 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Timers and execution statistics from original Basalt `utils/time_utils.hpp`.
use std::{collections::HashMap, time::Instant};

/// Process-local monotonic clock for joining frame stages across worker threads.
#[must_use]
pub fn profile_clock_ns() -> u64 {
    static ORIGIN: std::sync::LazyLock<Instant> = std::sync::LazyLock::new(Instant::now);
    ORIGIN.elapsed().as_nanos().try_into().unwrap_or(u64::MAX)
}

/// Emit a frame interval through the existing RRD `TextLog` transport.
/// Intervals sharing a frame timestamp use the same monotonic clock; nested
/// intervals describe subdivisions and must not be added to their parent.
#[expect(
    clippy::must_use_candidate,
    reason = "Logging is the primary effect; the next interval boundary is optional"
)]
pub fn profile_stage(frame_ns: i64, stage: &str, begin_ns: u64) -> u64 {
    let end_ns = profile_clock_ns();
    log::debug!(target: "basalt-profile",
        "frame_ns={frame_ns} stage={stage} begin_ns={begin_ns} end_ns={end_ns}");
    end_ns
}

#[derive(Debug)]
pub struct Timer(Instant);
impl Default for Timer {
    fn default() -> Self {
        Self(Instant::now())
    }
}
impl Timer {
    #[must_use]
    pub fn elapsed(&self) -> f64 {
        self.0.elapsed().as_secs_f64()
    }
    pub fn reset(&mut self) -> f64 {
        let now = Instant::now();
        let elapsed = now.duration_since(self.0).as_secs_f64();
        self.0 = now;
        elapsed
    }
}

pub struct ScopedTimer<'a> {
    destination: &'a mut f64,
    timer: Timer,
    add: bool,
}
impl<'a> ScopedTimer<'a> {
    pub fn new(destination: &'a mut f64) -> Self {
        Self {
            destination,
            timer: Timer::default(),
            add: false,
        }
    }
    pub fn adding(destination: &'a mut f64) -> Self {
        Self {
            destination,
            timer: Timer::default(),
            add: true,
        }
    }
}
impl Drop for ScopedTimer<'_> {
    fn drop(&mut self) {
        if self.add {
            *self.destination += self.timer.elapsed();
        } else {
            *self.destination = self.timer.elapsed();
        }
    }
}

pub fn log_timing<T>(time: &mut f64, function: impl FnOnce() -> T) -> T {
    let _timer = ScopedTimer::new(time);
    function()
}
pub fn log_timing_add<T>(time: &mut f64, function: impl FnOnce() -> T) -> T {
    let _timer = ScopedTimer::adding(time);
    function()
}

#[derive(Clone, Debug)]
pub enum Samples {
    Scalars(Vec<f64>),
    Vectors(Vec<Vec<f64>>),
}
#[derive(Clone, Debug)]
pub struct Meta {
    pub data: Samples,
    pub format: String,
}
impl Meta {
    pub fn format(&mut self, format: &str) -> &mut Self {
        format.clone_into(&mut self.format);
        self
    }
    pub fn set_meta(&mut self, other: &Self) {
        self.format.clone_from(&other.format);
    }
}

#[derive(Clone, Debug, Default)]
pub struct ExecutionStats {
    stats: HashMap<String, Meta>,
    order: Vec<String>,
}
impl ExecutionStats {
    fn entry(&mut self, name: &str, vectors: bool) -> &mut Meta {
        self.stats.entry(name.to_owned()).or_insert_with(|| {
            self.order.push(name.to_owned());
            Meta {
                data: if vectors {
                    Samples::Vectors(Vec::new())
                } else {
                    Samples::Scalars(Vec::new())
                },
                format: String::new(),
            }
        })
    }
    /// # Panics
    /// A statistic name must retain its scalar or vector type.
    pub fn add(&mut self, name: &str, value: f64) -> &mut Meta {
        let meta = self.entry(name, false);
        let Samples::Scalars(values) = &mut meta.data else {
            panic!("statistic type changed")
        };
        values.push(value);
        meta
    }
    /// # Panics
    /// A statistic name must retain its scalar or vector type.
    pub fn add_vector(&mut self, name: &str, value: &[f64]) -> &mut Meta {
        let meta = self.entry(name, true);
        let Samples::Vectors(values) = &mut meta.data else {
            panic!("statistic type changed")
        };
        values.push(value.to_vec());
        meta
    }
    pub fn merge_all(&mut self, other: &Self) {
        for name in &other.order {
            let meta = &other.stats[name];
            match &meta.data {
                Samples::Scalars(values) => {
                    for value in values {
                        self.add(name, *value);
                    }
                }
                Samples::Vectors(values) => {
                    for value in values {
                        self.add_vector(name, value);
                    }
                }
            }
            self.entry(name, matches!(meta.data, Samples::Vectors(_)))
                .set_meta(meta);
        }
    }
    /// Vector statistics are not summed, matching the upstream implementation.
    pub fn merge_sums(&mut self, other: &Self) {
        for name in &other.order {
            let meta = &other.stats[name];
            if let Samples::Scalars(values) = &meta.data {
                self.add(name, values.iter().sum()).set_meta(meta);
            } else if let Some(destination) = self.stats.get_mut(name) {
                destination.set_meta(meta);
            }
        }
    }
    /// Original scalar arrays and flattened vector/index archive schema.
    #[must_use]
    pub fn to_json(&self) -> serde_json::Value {
        let mut output = serde_json::Map::new();
        for name in &self.order {
            match &self.stats[name].data {
                Samples::Scalars(values) => {
                    output.insert(name.clone(), serde_json::json!(values));
                }
                Samples::Vectors(vectors) => {
                    let mut indices = Vec::new();
                    let mut values = Vec::new();
                    for vector in vectors {
                        indices.push(values.len());
                        values.extend_from_slice(vector);
                    }
                    output.insert(format!("{name}__index"), serde_json::json!(indices));
                    output.insert(format!("{name}__values"), serde_json::json!(values));
                }
            }
        }
        output.into()
    }
    #[must_use]
    pub fn get(&self, name: &str) -> Option<&Meta> {
        self.stats.get(name)
    }

    pub fn print(&self) {
        for name in &self.order {
            let meta = &self.stats[name];
            match &meta.data {
                Samples::Scalars(values) if !values.is_empty() && meta.format != "none" => {
                    let scale = if meta.format == "ms" { 1000.0 } else { 1.0 };
                    #[expect(
                        clippy::cast_precision_loss,
                        reason = "Statistical sample counts converted to f64 as upstream"
                    )]
                    let count = values.len() as f64;
                    let mean = values.iter().sum::<f64>() / count;
                    let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / count;
                    let min = values.iter().copied().fold(f64::INFINITY, f64::min);
                    let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                    log::info!(
                        "{name:20} ({}): {:.2}+-{:.2} [{:.2}, {:.2}]",
                        values.len(),
                        mean * scale,
                        variance.sqrt() * scale,
                        min * scale,
                        max * scale
                    );
                }
                Samples::Vectors(values) => log::info!("{name:20} ({})", values.len()),
                Samples::Scalars(_) => {}
            }
        }
    }

    /// Writes UBJSON with the upstream `.ubjson` extension and array schema.
    /// # Errors
    /// Returns file creation or write errors.
    pub fn save_json(&self, path: impl AsRef<std::path::Path>) -> firefly_error::Result<()> {
        use std::io::Write;
        let path = path.as_ref().with_extension("ubjson");
        let mut bytes = Vec::new();
        encode_ubjson(&self.to_json(), &mut bytes);
        std::fs::File::create(&path)
            .and_then(|mut file| file.write_all(&bytes))
            .map_err(|source| {
                firefly_error::Error::new(
                    firefly_error::ErrorKind::Internal,
                    "Cannot save execution statistics",
                )
                .with_operation("basalt.stats.save")
                .with_context("path", path.display().to_string())
                .with_source(source)
            })
    }
}

fn encode_length(length: usize, bytes: &mut Vec<u8>) {
    bytes.push(b'L');
    bytes.extend_from_slice(
        &i64::try_from(length)
            .expect("archive length fits i64")
            .to_be_bytes(),
    );
}
fn encode_ubjson(value: &serde_json::Value, bytes: &mut Vec<u8>) {
    use serde_json::Value;
    match value {
        Value::Null => bytes.push(b'Z'),
        Value::Bool(value) => bytes.push(if *value { b'T' } else { b'F' }),
        Value::Number(number) => {
            if let Some(integer) = number.as_i64() {
                bytes.push(b'L');
                bytes.extend_from_slice(&integer.to_be_bytes());
            } else {
                bytes.push(b'D');
                bytes.extend_from_slice(&number.as_f64().expect("JSON number").to_be_bytes());
            }
        }
        Value::String(value) => {
            bytes.push(b'S');
            encode_length(value.len(), bytes);
            bytes.extend_from_slice(value.as_bytes());
        }
        Value::Array(values) => {
            bytes.push(b'[');
            for value in values {
                encode_ubjson(value, bytes);
            }
            bytes.push(b']');
        }
        Value::Object(values) => {
            bytes.push(b'{');
            for (key, value) in values {
                encode_length(key.len(), bytes);
                bytes.extend_from_slice(key.as_bytes());
                encode_ubjson(value, bytes);
            }
            bytes.push(b'}');
        }
    }
}
