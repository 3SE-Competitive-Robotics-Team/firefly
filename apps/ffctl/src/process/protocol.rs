//! supervisor 与 CLI 之间的控制协议（Unix socket 上的单行 JSON）。

use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
#[serde(tag = "cmd", rename_all = "lowercase")]
pub(crate) enum Request {
    Start { names: Vec<String> },
    Stop { names: Vec<String> },
    Restart { names: Vec<String> },
    Status,
}

#[derive(Debug, Deserialize, Serialize)]
pub(crate) struct Response {
    pub(crate) ok: bool,
    pub(crate) message: String,
    pub(crate) processes: Vec<ProcessStatus>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub(crate) struct ProcessStatus {
    pub(crate) name: String,
    pub(crate) running: bool,
    pub(crate) pid: Option<u32>,
    pub(crate) restarts: u32,
    /// Older supervisors report only `running`; preserve that information when decoding.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub(crate) state: Option<ProcessState>,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub(crate) enum ProcessState {
    Stopped,
    Running,
    Backoff,
    Stopping,
}

impl ProcessStatus {
    pub(crate) fn state_name(&self) -> &'static str {
        match self.state {
            Some(ProcessState::Stopped) => "stopped",
            Some(ProcessState::Running) => "running",
            Some(ProcessState::Backoff) => "backoff",
            Some(ProcessState::Stopping) => "stopping",
            None if self.running => "running",
            None => "unknown",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn requests_and_status_roundtrip() {
        let start = Request::Start {
            names: vec!["firefly-rootslam".to_owned()],
        };
        let text = serde_json::to_string(&start).unwrap();
        assert_eq!(text, r#"{"cmd":"start","names":["firefly-rootslam"]}"#);
        let decoded: Request = serde_json::from_str(&text).unwrap();
        assert!(matches!(decoded, Request::Start { names } if names == ["firefly-rootslam"]));

        let status = Response {
            ok: true,
            message: "ok".to_owned(),
            processes: vec![ProcessStatus {
                name: "firefly-rootslam".to_owned(),
                running: true,
                pid: Some(42),
                restarts: 1,
                state: Some(ProcessState::Running),
            }],
        };
        let text = serde_json::to_string(&status).unwrap();
        let decoded: Response = serde_json::from_str(&text).unwrap();
        assert!(decoded.ok);
        assert_eq!(decoded.processes[0].pid, Some(42));
        assert_eq!(decoded.processes[0].state, Some(ProcessState::Running));
    }

    #[test]
    fn legacy_status_preserves_uncertainty_about_stopped_processes() {
        let mut status: ProcessStatus =
            serde_json::from_str(r#"{"name":"fc","running":false,"pid":null,"restarts":2}"#)
                .unwrap();
        assert_eq!(status.state, None);
        assert_eq!(status.state_name(), "unknown");
        status.running = true;
        assert_eq!(status.state_name(), "running");
    }

    #[test]
    fn state_is_an_additive_wire_field() {
        #[derive(Deserialize)]
        struct LegacyStatus {
            running: bool,
            pid: Option<u32>,
        }
        for (state, running, pid) in [
            (ProcessState::Stopped, false, None),
            (ProcessState::Backoff, false, None),
            (ProcessState::Running, true, Some(42)),
            (ProcessState::Stopping, true, Some(42)),
        ] {
            let status = ProcessStatus {
                name: "fc".to_owned(),
                running,
                pid,
                restarts: 2,
                state: Some(state),
            };
            let text = serde_json::to_string(&status).unwrap();
            let old: LegacyStatus = serde_json::from_str(&text).unwrap();
            assert_eq!((old.running, old.pid), (running, pid));
            let decoded: ProcessStatus = serde_json::from_str(&text).unwrap();
            assert_eq!(decoded.state, Some(state));
            assert!(text.contains(&format!("\"state\":\"{}\"", status.state_name())));
        }
    }
}
