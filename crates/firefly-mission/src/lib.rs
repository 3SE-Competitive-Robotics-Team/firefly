//! Operator and onboard client contract for one powered run.
use std::io::{self, BufRead, BufReader, Read, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::time::Duration;

use serde::{Deserialize, Serialize, de::DeserializeOwned};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Arrival {
    Stop,
    Pass,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Waypoint {
    pub offset: [f64; 3],
    pub arrival: Arrival,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Plan {
    #[serde(default)]
    pub return_offset: [f64; 2],
    pub altitude: f64,
    pub hover_seconds: f64,
    pub precision_land: bool,
    #[serde(default)]
    pub waypoints: Vec<Waypoint>,
}

impl Plan {
    /// The final user waypoint may pass through to the return point.
    #[must_use]
    pub fn return_point(&self) -> [f64; 3] {
        [self.return_offset[0], self.return_offset[1], self.altitude]
    }

    /// # Errors
    /// Rejects invalid timing, altitude and degenerate pass segments.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.waypoints.len() > 16 {
            return Err("a mission plan supports up to 16 waypoints");
        }
        if !self.return_offset.iter().all(|v| v.is_finite()) {
            return Err("invalid return offset");
        }
        if !(self.altitude.is_finite()
            && self.altitude > 0.0
            && self.hover_seconds.is_finite()
            && (0.0..=3600.).contains(&self.hover_seconds))
        {
            return Err("invalid takeoff height or dwell time");
        }
        for (i, point) in self.waypoints.iter().enumerate() {
            if !point.offset.iter().all(|v| v.is_finite()) || point.offset[2] <= 0. {
                return Err("waypoints require finite coordinates and positive height");
            }
            let next = self
                .waypoints
                .get(i + 1)
                .map_or_else(|| self.return_point(), |w| w.offset);
            if point.arrival == Arrival::Pass
                && point
                    .offset
                    .iter()
                    .zip(next)
                    .all(|(a, b)| (a - b).abs() < 1e-6)
            {
                return Err("pass waypoint requires a distinct successor");
            }
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
pub enum Request {
    Status { id: Option<String> },
    Submit { id: String, plan: Plan },
    Cancel { id: String },
}

#[must_use]
pub fn valid_id(id: &str) -> bool {
    (1..=64).contains(&id.len())
        && id
            .bytes()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, b'-' | b'_'))
}

#[must_use]
pub fn runtime_dir() -> PathBuf {
    if cfg!(target_os = "linux") {
        PathBuf::from("/run/firefly")
    } else {
        firefly_config::paths::root().join("state")
    }
}

#[must_use]
pub fn socket_path() -> PathBuf {
    runtime_dir().join("mission.sock")
}

/// Read a single bounded newline-delimited JSON frame.
/// # Errors
/// Returns framing, decoding or transport errors.
pub fn read_frame<T: DeserializeOwned>(stream: &UnixStream) -> io::Result<T> {
    let mut buf = Vec::new();
    BufReader::new(stream.take(65537)).read_until(b'\n', &mut buf)?;
    if buf.len() > 65536 || buf.last() != Some(&b'\n') {
        return Err(io::Error::other("invalid mission frame"));
    }
    Ok(serde_json::from_slice(&buf)?)
}

/// # Errors
/// Returns serialization or socket errors.
pub fn write_frame(stream: &UnixStream, value: &impl Serialize) -> io::Result<()> {
    let mut frame = serde_json::to_vec(value)?;
    frame.push(b'\n');
    if frame.len() > 65536 {
        return Err(io::Error::other("mission frame too large"));
    }
    let mut writer = stream;
    writer.write_all(&frame)
}

/// # Errors
/// Returns daemon rejection, timeout, or transport errors.
pub fn call(request: &Request) -> io::Result<serde_json::Value> {
    let stream = UnixStream::connect(socket_path())?;
    let timeout = Some(Duration::from_secs(5));
    stream.set_read_timeout(timeout)?;
    stream.set_write_timeout(timeout)?;
    write_frame(&stream, request)?;
    let reply: Result<serde_json::Value, String> = read_frame(&stream)?;
    reply.map_err(io::Error::other)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn final_pass_needs_a_distinct_return_goal() {
        let mut plan = Plan {
            altitude: 1.7,
            return_offset: [0., 0.],
            hover_seconds: 1.,
            precision_land: true,
            waypoints: vec![Waypoint {
                offset: [0., 0., 1.7],
                arrival: Arrival::Pass,
            }],
        };
        assert!(plan.validate().is_err());
        plan.return_offset = [0.12, -0.08];
        assert!(plan.validate().is_ok());
        plan.waypoints[0].offset[1] = f64::NAN;
        assert!(plan.validate().is_err());
    }

    #[test]
    fn mission_id_is_one_filename_component() {
        assert!(valid_id("inspection_01-A"));
        assert!(!valid_id("../previous-run"));
        assert!(!valid_id(""));
        assert!(!valid_id(&"a".repeat(65)));
    }

    #[test]
    fn an_incomplete_socket_request_is_rejected() {
        let (mut client, server) = UnixStream::pair().unwrap();
        client.write_all(br#"{"op":"status"}"#).unwrap();
        client.shutdown(std::net::Shutdown::Write).unwrap();
        assert!(read_frame::<Request>(&server).is_err());
    }
}
