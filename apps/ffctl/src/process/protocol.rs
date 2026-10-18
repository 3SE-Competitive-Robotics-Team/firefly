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
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn requests_and_status_roundtrip() {
        let start = Request::Start {
            names: vec!["vio".to_owned()],
        };
        let text = serde_json::to_string(&start).unwrap();
        assert_eq!(text, r#"{"cmd":"start","names":["vio"]}"#);
        let decoded: Request = serde_json::from_str(&text).unwrap();
        assert!(matches!(decoded, Request::Start { names } if names == ["vio"]));

        let status = Response {
            ok: true,
            message: "ok".to_owned(),
            processes: vec![ProcessStatus {
                name: "vio".to_owned(),
                running: true,
                pid: Some(42),
                restarts: 1,
            }],
        };
        let text = serde_json::to_string(&status).unwrap();
        let decoded: Response = serde_json::from_str(&text).unwrap();
        assert!(decoded.ok);
        assert_eq!(decoded.processes[0].pid, Some(42));
    }
}
