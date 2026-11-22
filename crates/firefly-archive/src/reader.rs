//! Locked snapshots of recording segments in archive order.
use serde::Deserialize;
use std::{
    collections::BTreeMap,
    fs::{self, File},
    io,
    os::unix::fs::MetadataExt,
    path::{Path, PathBuf},
};

#[derive(Deserialize)]
struct Manifest {
    format: u32,
    rrd_last: u64,
    gaps: BTreeMap<String, serde_json::Value>,
    evicted: BTreeMap<String, serde_json::Value>,
}

pub struct RecordingFiles {
    paths: Vec<PathBuf>,
    _guard: Option<File>,
}

fn read_lock(run: &Path) -> io::Result<File> {
    let path = run.join(".read.lock");
    let lock = File::open(&path)?;
    lock.lock_shared()?;
    let held = lock.metadata()?;
    let current = fs::metadata(path)?;
    if (held.dev(), held.ino()) != (current.dev(), current.ino()) {
        return Err(io::Error::new(
            io::ErrorKind::NotFound,
            "archive number was reused",
        ));
    }
    Ok(lock)
}

impl RecordingFiles {
    /// Open a legacy recording or lock all segments of a managed run for reading.
    ///
    /// # Errors
    /// The run is incomplete, inconsistent, inaccessible, or has no sealed recording.
    pub fn open(path: &Path) -> io::Result<Self> {
        Self::read(path, true)
    }

    /// Open the retained, sealed ranges of a possibly incomplete run for inspection.
    ///
    /// # Errors
    /// Returns I/O errors or an invalid archive identity or segment name.
    pub fn retained(path: &Path) -> io::Result<Self> {
        Self::read(path, false)
    }

    fn read(path: &Path, complete: bool) -> io::Result<Self> {
        if !path.is_dir() {
            if path.extension().is_some_and(|ext| ext == "partial") {
                return Err(io::Error::other("recording segment is unfinished"));
            }
            let mut guard = None;
            if let Some(run) = path
                .parent()
                .filter(|p| p.ends_with("rrd"))
                .and_then(Path::parent)
                && crate::manifest_path(run).is_file()
            {
                guard = Some(read_lock(run)?);
            }
            return Ok(Self {
                paths: vec![path.to_owned()],
                _guard: guard,
            });
        }
        let guard = read_lock(path)?;
        let writer_lock = path.join(".rrd.lock");
        let _snapshot = if complete && writer_lock.try_exists()? {
            let writer = File::open(writer_lock)?;
            writer
                .try_lock_shared()
                .map_err(|e| io::Error::other(format!("recording still active: {e}")))?;
            Some(writer)
        } else {
            None
        };
        let manifest: Manifest = serde_json::from_reader(File::open(crate::manifest_path(path))?)?;
        if manifest.format != 1 {
            return Err(io::Error::other("unknown run archive format"));
        }
        if complete && (manifest.gaps.contains_key("rrd") || manifest.evicted.contains_key("rrd")) {
            return Err(io::Error::other("run has missing recording evidence"));
        }
        let mut segments = BTreeMap::new();
        for entry in fs::read_dir(path.join("rrd"))? {
            let entry = entry?;
            let path = entry.path();
            if path
                .extension()
                .is_some_and(|ext| ext == "credit" || (!complete && ext == "partial"))
            {
                continue;
            }
            let number = path
                .file_stem()
                .and_then(|s| s.to_str())
                .and_then(|s| s.parse::<u64>().ok())
                .ok_or_else(|| io::Error::other("invalid recording segment name"))?;
            if !entry.file_type()?.is_file()
                || entry.file_name().to_str() != Some(&format!("{number:06}.rrd"))
                || segments.insert(number, path).is_some()
            {
                return Err(io::Error::other("unfinished or invalid recording segment"));
            }
        }
        if segments.is_empty() || (complete && segments.keys().copied().ne(1..=manifest.rrd_last)) {
            return Err(io::Error::other("missing recording segment"));
        }
        Ok(Self {
            paths: segments.into_values().collect(),
            _guard: Some(guard),
        })
    }

    #[must_use]
    pub fn paths(&self) -> &[PathBuf] {
        &self.paths
    }
}
