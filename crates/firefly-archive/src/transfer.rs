//! Read-only HIL access to bounded text and journal ranges of the current boot.
use crate::Run;
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{self, Read, Seek},
    os::unix::fs::OpenOptionsExt,
};

/// Worst-case decimal JSON expansion still fits the 8192-byte HIL RPC frame.
pub const BLOCK: usize = 1900;

#[derive(Deserialize, Serialize)]
#[serde(tag = "archive", rename_all = "snake_case", deny_unknown_fields)]
pub enum Query {
    Snapshot,
    Read {
        boot: String,
        stream: String,
        segment: u64,
        offset: u64,
        limit: usize,
    },
}

#[derive(Clone, Copy, Deserialize, Serialize)]
pub struct Cursor {
    pub first: u64,
    pub segment: u64,
    pub offset: u64,
}

#[derive(Deserialize, Serialize)]
pub struct Snapshot {
    pub boot: String,
    pub streams: BTreeMap<String, Cursor>,
    pub gaps: BTreeMap<String, u64>,
}

/// Parse a stream identity from a board-generated cursor or an incoming read request.
/// # Errors
/// Only text and system logs with a single source component are accessible.
pub fn source(stream: &str) -> io::Result<(&str, &str)> {
    if let Some((kind, name)) = stream.split_once('/')
        && matches!(kind, "text" | "system")
        && (1..=80).contains(&name.len())
        && !name.starts_with('.')
        && name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"._-".contains(&b))
    {
        return Ok((kind, name));
    }
    Err(io::Error::other("invalid board log stream"))
}

/// Serve the two archive operations on the board's existing non-real-time RPC worker.
/// # Errors
/// Returns a scoped read, identity, manifest, or transfer-budget error.
pub fn reply(run: &Run, query: Query) -> io::Result<serde_json::Value> {
    let manifest: serde_json::Value =
        serde_json::from_reader(File::open(crate::manifest_path(run.path()))?)?;
    let boot = manifest["boot_id"]
        .as_str()
        .ok_or_else(|| io::Error::other("missing board boot ID"))?;
    match query {
        Query::Snapshot => {
            let mut streams = BTreeMap::new();
            for kind in ["text", "system"] {
                for entry in fs::read_dir(run.path().join(kind))? {
                    let entry = entry?;
                    if !entry.file_type()?.is_file() {
                        continue;
                    }
                    let name = entry.file_name();
                    let Some((name, seq)) = name
                        .to_str()
                        .and_then(|s| s.strip_suffix(".log"))
                        .and_then(|s| s.rsplit_once('.'))
                    else {
                        continue;
                    };
                    let segment = seq.parse::<u64>().map_err(io::Error::other)?;
                    let key = format!("{kind}/{name}");
                    source(&key)?;
                    let cursor = streams.entry(key).or_insert(Cursor {
                        first: segment,
                        segment: 0,
                        offset: 0,
                    });
                    cursor.first = cursor.first.min(segment);
                    if segment > cursor.segment {
                        cursor.segment = segment;
                        cursor.offset = entry.metadata()?.len();
                    }
                }
            }
            let gaps = manifest["gaps"]
                .as_object()
                .ok_or_else(|| io::Error::other("invalid board archive"))?
                .iter()
                .map(|(source, event)| {
                    (source.clone(), event["count"].as_u64().unwrap_or_default())
                })
                .collect();
            Ok(serde_json::to_value(Snapshot {
                boot: boot.into(),
                streams,
                gaps,
            })?)
        }
        Query::Read {
            boot: expected,
            stream,
            segment,
            offset,
            limit,
        } => {
            if boot != expected || !(1..=BLOCK).contains(&limit) || segment == 0 {
                return Err(io::Error::other("invalid boot or board log range"));
            }
            let (kind, name) = source(&stream)?;
            let path = run
                .path()
                .join(kind)
                .join(format!("{name}.{segment:06}.log"));
            let mut file = File::options()
                .read(true)
                .custom_flags(nix::libc::O_NOFOLLOW | nix::libc::O_NONBLOCK)
                .open(path)?;
            let meta = file.metadata()?;
            if !meta.is_file() || offset > meta.len() {
                return Err(io::Error::other("invalid board log offset"));
            }
            file.seek(io::SeekFrom::Start(offset))?;
            let mut bytes = Vec::with_capacity(limit);
            file.take(limit as u64).read_to_end(&mut bytes)?;
            Ok(serde_json::to_value(bytes)?)
        }
    }
}
