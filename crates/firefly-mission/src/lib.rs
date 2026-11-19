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
        if !(0.0..=0.3).contains(&self.return_offset[0].hypot(self.return_offset[1])) {
            return Err("return offset must be within 0.3 m of takeoff");
        }
        if !((0.5..=2.0).contains(&self.altitude)
            && self.hover_seconds > 0.0
            && self.hover_seconds <= 600.0)
        {
            return Err("invalid takeoff height or dwell time");
        }
        let mut previous = [0.0, 0.0, self.altitude];
        for (index, waypoint) in self.waypoints.iter().enumerate() {
            let [x, y, z] = waypoint.offset;
            if !(0.5..=3.3).contains(&z) || !(0.0..=10.0).contains(&x.hypot(y)) {
                return Err(
                    "waypoints require height in [0.5,3.3] m and horizontal radius <= 10 m",
                );
            }
            if index == 0 && x.hypot(y) < 1.0 {
                return Err("first waypoint must be at least 1 m horizontally from takeoff");
            }
            if !valid_leg(previous, waypoint.offset) {
                return Err("consecutive waypoint spacing must be in [0.5,10] m");
            }
            previous = waypoint.offset;
        }
        if !self.waypoints.is_empty() && !valid_leg(previous, self.return_point()) {
            return Err("return leg must be in [0.5,10] m");
        }
        Ok(())
    }
}

fn valid_leg(a: [f64; 3], b: [f64; 3]) -> bool {
    (0.5..=10.0).contains(&(a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2]))
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
    runtime_dir_for(&firefly_config::paths::root())
}

/// Resolve sockets for the board installation or an isolated SIL prefix.
#[must_use]
pub fn runtime_dir_for(prefix: &std::path::Path) -> PathBuf {
    if cfg!(target_os = "linux") && prefix == std::path::Path::new("/opt/firefly") {
        PathBuf::from("/run/firefly")
    } else {
        prefix.join("state")
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
    call_at(&socket_path(), request)
}

/// Submit a request through a local daemon or USB HIL bridge socket.
/// # Errors
/// Returns connection, framing, timeout or mission errors.
pub fn call_at(socket: &std::path::Path, request: &Request) -> io::Result<serde_json::Value> {
    let stream = UnixStream::connect(socket)?;
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
        plan.waypoints[0].offset = [1., 0., 1.7];
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
    fn rejects_out_of_envelope_inputs_before_flight() {
        let mut plan = Plan {
            altitude: 2.,
            hover_seconds: 600.,
            return_offset: [0., 0.],
            precision_land: false,
            waypoints: vec![Waypoint {
                arrival: Arrival::Stop,
                offset: [1., 0., 2.],
            }],
        };
        assert!(plan.validate().is_ok());
        for bad in [0.49, 2.01, f64::NAN] {
            plan.altitude = bad;
            assert!(plan.validate().is_err());
        }
        plan.altitude = 2.;
        for bad in [0., 600.01, f64::INFINITY] {
            plan.hover_seconds = bad;
            assert!(plan.validate().is_err());
        }
        plan.hover_seconds = 1.;
        for offset in [[100., 0., 2.], [0.9, 0., 2.], [1., 0., 4.], [1., 0., 0.4]] {
            plan.waypoints[0].offset = offset;
            assert!(plan.validate().is_err());
        }
        plan.waypoints[0].offset = [6., 0., 3.3];
        plan.waypoints.push(Waypoint {
            arrival: Arrival::Stop,
            offset: [6., 0., 1.3],
        });
        assert!(plan.validate().is_ok());
        for offset in [[6., 0., 3.2], [-6., 0., 2.]] {
            plan.waypoints[1].offset = offset;
            assert!(plan.validate().is_err());
        }
    }

    #[test]
    fn an_incomplete_socket_request_is_rejected() {
        let (mut client, server) = UnixStream::pair().unwrap();
        client.write_all(br#"{"op":"status"}"#).unwrap();
        client.shutdown(std::net::Shutdown::Write).unwrap();
        assert!(read_frame::<Request>(&server).is_err());
    }
}
