//! Per-boot and per-command run archives. Filesystem locks coordinate independent producers.

pub mod host;
pub mod output;
pub mod reader;
mod storage;
pub mod transfer;

use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{self, Write},
    path::{Path, PathBuf},
    sync::Arc,
    time::{SystemTime, UNIX_EPOCH},
};

use serde::{Deserialize, Serialize};
pub use storage::{Segment, Stream};

const MANIFEST: &str = "manifest.json";
const SLOTS: u16 = 9999;
const VERSION: u8 = 1;
// Includes the atomic manifest replacement, journal checkpoints and coordination files.
const METADATA_BUDGET: u64 = 256 * 1024;

#[derive(Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct Limits {
    max_bytes: u64,
    cleanup_bytes: u64,
    keep_free_bytes: u64,
    rrd_segment_bytes: u64,
    stream_segment_bytes: u64,
    stream_segments: usize,
}

impl Default for Limits {
    fn default() -> Self {
        Self {
            max_bytes: 8 << 30,
            cleanup_bytes: 7 << 30,
            keep_free_bytes: 2 << 30,
            rrd_segment_bytes: 256 << 20,
            stream_segment_bytes: 16 << 20,
            stream_segments: 4,
        }
    }
}

impl Limits {
    fn load(root: &Path) -> io::Result<Self> {
        let limits = firefly_config::load::<Self>(root.join("configs/archive.toml"))
            .map_err(io::Error::other)?;
        if limits.cleanup_bytes < METADATA_BUDGET
            || limits.cleanup_bytes >= limits.max_bytes
            || limits.stream_segments == 0
            || limits.stream_segment_bytes == 0
            || limits.rrd_segment_bytes == 0
        {
            return Err(invalid("invalid archive byte or segment limits"));
        }
        Ok(limits)
    }
}

fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

fn millis(time: SystemTime) -> u64 {
    time.duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

fn now() -> u64 {
    millis(SystemTime::now())
}

#[derive(Serialize, Deserialize)]
struct Event {
    count: u64,
    first_ms: u64,
    last_ms: u64,
    detail: String,
}

#[derive(Serialize, Deserialize)]
struct Eviction {
    first: u64,
    last: u64,
    first_ms: u64,
    last_ms: u64,
}

#[derive(Serialize, Deserialize)]
struct Manifest {
    format: u8,
    sequence: u64,
    source: String,
    boot_id: Option<String>,
    started_ms: u64,
    ended_ms: Option<u64>,
    state: String,
    rrd_last: u64,
    gaps: BTreeMap<String, Event>,
    evicted: BTreeMap<String, Eviction>,
}

/// Path to the run manifest, including timestamped archives from earlier installations.
#[must_use]
pub fn manifest_path(dir: &Path) -> PathBuf {
    let path = dir.join(MANIFEST);
    let legacy = dir.join("run.json");
    if !path.exists() && legacy.is_file() {
        legacy
    } else {
        path
    }
}

impl Manifest {
    fn read(dir: &Path) -> io::Result<Self> {
        let value: Self = serde_json::from_reader(File::open(manifest_path(dir))?)?;
        if value.format != VERSION || value.sequence == 0 {
            return Err(invalid("unknown run manifest"));
        }
        Ok(value)
    }

    fn save(&self, dir: &Path) -> io::Result<()> {
        let tmp = dir.join(".manifest.json.new");
        let data = serde_json::to_vec(self)?;
        if data.len() > 96 * 1024 {
            return Err(invalid("run manifest exceeds metadata budget"));
        }
        let mut file = File::create(&tmp)?;
        file.write_all(&data)?;
        file.sync_all()?;
        fs::rename(tmp, manifest_path(dir))?;
        sync_dir(dir)
    }
}

fn sync_dir(dir: &Path) -> io::Result<()> {
    File::open(dir)?.sync_all()
}

fn validate_source(name: &str) -> io::Result<()> {
    if name.is_empty()
        || name.len() > 80
        || !name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"_-.".contains(&b))
        || name.starts_with('.')
    {
        return Err(invalid("invalid archive source name"));
    }
    Ok(())
}

fn lock_file(path: &Path) -> io::Result<File> {
    File::options()
        .create(true)
        .truncate(false)
        .read(true)
        .write(true)
        .open(path)
}

fn global(runs: &Path) -> io::Result<File> {
    let guard = lock_file(&runs.join(".archive.lock"))?;
    guard.lock()?;
    Ok(guard)
}

struct Inner {
    runs: PathBuf,
    path: PathBuf,
    limits: Limits,
    active: File,
}

impl Drop for Inner {
    fn drop(&mut self) {
        if let Err(error) = self.close() {
            eprintln!("close archive {}: {error}", self.path.display());
        }
    }
}

impl Inner {
    fn close(&self) -> io::Result<()> {
        let _guard = global(&self.runs)?;
        self.active.unlock()?;
        if storage::exclusive(&self.active)? {
            let mut manifest = Manifest::read(&self.path)?;
            manifest.state = if manifest.boot_id.is_some() {
                "idle"
            } else {
                "closed"
            }
            .into();
            manifest.ended_ms = Some(now());
            manifest.save(&self.path)?;
        }
        Ok(())
    }
}

/// A lease protecting the complete run directory while a producer is alive.
#[derive(Clone)]
pub struct Run(Arc<Inner>);

impl Run {
    /// Start a new command archive under the installation root.
    /// # Errors
    /// Returns filesystem, configuration and quota errors.
    pub fn create(source: &str) -> io::Result<Self> {
        Self::create_at(&firefly_config::paths::root(), source)
    }

    /// Start a distinct command archive under an explicit installation root.
    /// # Errors
    /// Returns filesystem, configuration and quota errors.
    pub fn create_at(root: &Path, source: &str) -> io::Result<Self> {
        Self::open(root, source, None, None)
    }

    /// Join the current Linux system boot.
    /// # Errors
    /// Returns boot identity, configuration or filesystem errors.
    pub fn board() -> io::Result<Self> {
        let id = fs::read_to_string("/proc/sys/kernel/random/boot_id")?;
        let id = id.trim();
        validate_source(id)?;
        Self::open(&firefly_config::paths::root(), "board", Some(id), None)
    }

    /// Join an orchestrated run, a board boot or a standalone recorder session.
    /// # Errors
    /// Returns invalid session identity, configuration or filesystem errors.
    pub fn recorder() -> io::Result<Self> {
        if let Some(path) = std::env::var_os("FIREFLY_RUN_DIR") {
            Self::join(Path::new(&path))
        } else if cfg!(all(target_os = "linux", target_arch = "aarch64")) {
            Self::board()
        } else {
            Self::create("mimir")
        }
    }

    /// Acquire another producer lease on a run created by the launch command.
    /// # Errors
    /// Rejects paths outside the managed root and invalid manifests.
    pub fn join(path: &Path) -> io::Result<Self> {
        Self::open(&firefly_config::paths::root(), "command", None, Some(path))
    }

    fn open(
        root: &Path,
        source: &str,
        boot: Option<&str>,
        join: Option<&Path>,
    ) -> io::Result<Self> {
        validate_source(source)?;
        let limits = Limits::load(root)?;
        let runs = root.join("logs/runs");
        fs::create_dir_all(&runs)?;
        let runs = runs.canonicalize()?;
        let _guard = global(&runs)?;
        let candidates = storage::managed(&runs)?;
        for (manifest, path) in &candidates {
            if manifest.state == "idle" && boot.is_some() && manifest.boot_id.as_deref() != boot {
                let mut previous = Manifest::read(path)?;
                "closed".clone_into(&mut previous.state);
                previous.save(path)?;
            }
            if manifest.state == "active" {
                let lease = File::open(path.join(".active.lock"))?;
                if storage::exclusive(&lease)? {
                    let mut interrupted = Manifest::read(path)?;
                    "interrupted".clone_into(&mut interrupted.state);
                    interrupted.ended_ms = None;
                    interrupted.save(path)?;
                }
            }
        }
        let existing = if let Some(path) = join {
            let path = path.canonicalize()?;
            if path.parent() != Some(runs.as_path()) || !candidates.iter().any(|(_, p)| *p == path)
            {
                return Err(invalid("run directory is outside managed archives"));
            }
            Some(path)
        } else {
            candidates
                .iter()
                .find(|(m, _)| boot.is_some() && m.boot_id.as_deref() == boot)
                .map(|(_, p)| p.clone())
        };
        let path = if let Some(path) = existing {
            if let Err(error) = storage::make_room(&runs, Some(&path), &limits, 0)
                && error.kind() != io::ErrorKind::StorageFull
            {
                return Err(error);
            }
            let mut manifest = Manifest::read(&path)?;
            if manifest.state != "active" {
                manifest.state = "active".into();
                manifest.ended_ms = None;
                manifest.save(&path)?;
            }
            path
        } else {
            let sequence = candidates.last().map_or(Ok(1), |(m, _)| {
                m.sequence
                    .checked_add(1)
                    .ok_or_else(|| invalid("archive sequence exhausted"))
            })?;
            storage::make_room(&runs, None, &limits, METADATA_BUDGET)?;
            let path = storage::next_directory(&runs, &candidates, SLOTS)?;
            fs::create_dir(&path)?;
            for kind in ["text", "system", "rrd"] {
                fs::create_dir(path.join(kind))?;
            }
            lock_file(&path.join(".read.lock"))?;
            lock_file(&path.join(".active.lock"))?;
            Manifest {
                format: VERSION,
                sequence,
                source: source.to_owned(),
                boot_id: boot.map(str::to_owned),
                started_ms: now(),
                ended_ms: None,
                state: "active".into(),
                rrd_last: 0,
                gaps: BTreeMap::new(),
                evicted: BTreeMap::new(),
            }
            .save(&path)?;
            sync_dir(&runs)?;
            path
        };
        let active = File::open(path.join(".active.lock"))?;
        active.lock_shared()?;
        Ok(Self(Arc::new(Inner {
            runs,
            path,
            limits,
            active,
        })))
    }

    /// Path printed by Mimir and passed to archive readers.
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.0.path
    }

    /// Target encoded data size at which Mimir closes a complete segment.
    #[must_use]
    pub fn rrd_target(&self) -> u64 {
        self.0.limits.rrd_segment_bytes
    }

    /// Register recorder startup after acquiring the exclusive recording lease.
    /// A new encoder cannot observe the interval since an earlier encoder exited.
    /// # Errors
    /// Returns manifest read or update errors.
    pub fn recorder_started(&self) -> io::Result<()> {
        if Manifest::read(self.path())?.rrd_last != 0 {
            self.gap(
                "rrd",
                "recorder restarted; coverage between recorder sessions is unknown",
            )?;
        }
        Ok(())
    }

    /// Open an RRD partial. Sealing publishes it to readers.
    /// # Errors
    /// Returns metadata, storage or reservation failures.
    pub fn rrd_segment(&self) -> io::Result<Segment> {
        storage::Segment::create(self, "rrd", "", 0)
    }

    /// Open a rolling text or system log. Each write represents one record.
    /// # Errors
    /// Returns invalid source or filesystem errors.
    pub fn stream(&self, kind: &str, source: &str) -> io::Result<Stream> {
        Stream::new(self, kind, source)
    }

    /// Record cumulative loss and its wall-clock interval for a bounded source.
    /// # Errors
    /// Returns manifest or filesystem errors.
    pub fn gap(&self, source: &str, detail: &str) -> io::Result<()> {
        validate_source(source)?;
        let _guard = global(&self.0.runs)?;
        let mut m = Manifest::read(self.path())?;
        let stamp = now();
        let event = m.gaps.entry(source.into()).or_insert_with(|| Event {
            count: 0,
            first_ms: stamp,
            last_ms: stamp,
            detail: String::new(),
        });
        event.count = event.count.saturating_add(1);
        event.last_ms = stamp;
        event.detail = detail.chars().take(240).collect();
        m.save(self.path())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{
        Barrier,
        atomic::{AtomicUsize, Ordering},
    };

    struct Workspace(PathBuf);

    impl Workspace {
        fn new(extra: &str) -> Self {
            static NEXT: AtomicUsize = AtomicUsize::new(0);
            let root = Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../target/firefly-archive-tests")
                .join(format!(
                    "archive-{}-{}",
                    std::process::id(),
                    NEXT.fetch_add(1, Ordering::Relaxed)
                ));
            fs::create_dir_all(root.join("configs")).unwrap();
            fs::write(
                root.join("configs/archive.toml"),
                format!("keep_free_bytes=0\n{extra}"),
            )
            .unwrap();
            Self(root)
        }

        fn run(&self) -> Run {
            Run::create_at(&self.0, "test").unwrap()
        }
    }

    impl Drop for Workspace {
        fn drop(&mut self) {
            fs::remove_dir_all(&self.0).unwrap();
        }
    }

    #[test]
    fn boot_identity_and_last_producer_lease_define_lifecycle() {
        let root = Workspace::new("");
        let a = Run::open(&root.0, "board", Some("boot-a"), None).unwrap();
        let b = Run::open(&root.0, "board", Some("boot-a"), None).unwrap();
        let path = a.path().to_owned();
        assert!(path.ends_with("log_0001"));
        assert!(path.join("manifest.json").is_file());
        assert_eq!(path, b.path());
        drop(a);
        assert_eq!(Manifest::read(&path).unwrap().state, "active");
        drop(b);
        assert!(Manifest::read(&path).unwrap().ended_ms.is_some());
        let c = Run::open(&root.0, "board", Some("boot-b"), None).unwrap();
        assert_ne!(c.path(), path);
        assert!(c.path().ends_with("log_0002"));
        assert_eq!(Manifest::read(c.path()).unwrap().sequence, 2);
    }

    #[test]
    fn numbered_runs_follow_legacy_runs_even_if_the_clock_is_at_epoch() {
        let root = Workspace::new("");
        let first = root.run();
        let numbered = first.path().to_owned();
        drop(first);
        let legacy = numbered.with_file_name("19700101_000000");
        fs::rename(&numbered, &legacy).unwrap();
        fs::rename(legacy.join(MANIFEST), legacy.join("run.json")).unwrap();
        let mut old = Manifest::read(&legacy).unwrap();
        old.started_ms = 0;
        old.save(&legacy).unwrap();
        let next = root.run();
        assert!(next.path().ends_with("log_0001"));
        assert_eq!(Manifest::read(next.path()).unwrap().sequence, 2);
        assert_eq!(Manifest::read(&legacy).unwrap().started_ms, 0);
    }

    #[test]
    fn numbering_wraps_and_reuses_only_an_unlocked_managed_directory() {
        let root = Workspace::new("");
        let first = root.run();
        let first_path = first.path().to_owned();
        let runs = first.0.runs.clone();
        drop(first);
        let last = runs.join("log_9999");
        fs::rename(&first_path, &last).unwrap();
        let wrapped = root.run();
        assert_eq!(wrapped.path(), first_path);

        // Exercise a full number range with two slots, one protected by a reader.
        let reader = File::open(first_path.join(".read.lock")).unwrap();
        reader.lock_shared().unwrap();
        drop(wrapped);
        let second = root.run();
        let second_path = second.path().to_owned();
        let candidates = storage::managed(&runs).unwrap();
        assert_eq!(
            storage::next_directory(&runs, &candidates, 2)
                .unwrap_err()
                .kind(),
            io::ErrorKind::StorageFull
        );
        drop(second);
        assert_eq!(
            storage::next_directory(&runs, &candidates, 2).unwrap(),
            second_path
        );
        assert!(first_path.is_dir());
        assert!(last.is_dir());
    }

    #[test]
    fn pruning_uses_creation_sequence_and_preserves_readers_and_foreign_files() {
        let root = Workspace::new("max_bytes=900000\ncleanup_bytes=600000");
        let first = root.run();
        let first_path = first.path().to_owned();
        let reader = File::open(first_path.join(".read.lock")).unwrap();
        reader.lock_shared().unwrap();
        drop(first);
        let second = root.run();
        let second_path = second.path().to_owned();
        drop(second);
        let third = root.run();
        let foreign = third.0.runs.join("20200101_000000");
        fs::create_dir(&foreign).unwrap();
        fs::write(foreign.join("notes"), "operator evidence").unwrap();
        let fourth = root.run();
        assert!(fourth.path().exists());
        assert!(third.path().exists());
        assert!(first_path.exists());
        assert!(!second_path.exists());
        assert!(foreign.join("notes").exists());
    }

    #[test]
    fn active_run_retains_recent_complete_segments_and_records_evictions() {
        let root = Workspace::new(
            "max_bytes=350000\ncleanup_bytes=290000\nstream_segments=2\nstream_segment_bytes=8",
        );
        let run = root.run();
        let mut text = run.stream("text", "camera").unwrap();
        for _ in 0..4 {
            text.write_all(b"one row\n").unwrap();
        }
        drop(text);
        assert_eq!(
            Manifest::read(run.path()).unwrap().evicted["text.camera"].last,
            2
        );
        let payload = vec![1; 25_000];
        for _ in 0..6 {
            let mut seg = run.rrd_segment().unwrap();
            seg.write_all(&payload).unwrap();
            seg.seal().unwrap();
        }
        let m = Manifest::read(run.path()).unwrap();
        assert_eq!(m.rrd_last, 6);
        assert!(m.evicted["rrd"].last >= 2);
        assert!(run.path().join("rrd/000006.rrd").exists());
        assert!(run.path().join("text/camera.000004.log").exists());
        assert!(!run.path().join("text/camera.000001.log").exists());
    }

    #[test]
    fn concurrent_writers_cannot_double_spend_unused_reservations() {
        let root = Workspace::new("max_bytes=400000\ncleanup_bytes=350000");
        let run = root.run();
        let gate = Barrier::new(2);
        let wins = std::thread::scope(|scope| {
            let handles: Vec<_> = (0..2)
                .map(|_| {
                    scope.spawn(|| {
                        let mut segment = run.rrd_segment().unwrap();
                        gate.wait();
                        let result = segment.reserve(100_000);
                        gate.wait();
                        result
                    })
                })
                .collect();
            handles
                .into_iter()
                .map(|handle| handle.join().unwrap())
                .collect::<Vec<_>>()
        });
        assert_eq!(wins.iter().filter(|r| r.is_ok()).count(), 1);
        assert_eq!(
            wins.into_iter().find_map(Result::err).unwrap().kind(),
            io::ErrorKind::StorageFull
        );
        let mut next = run.rrd_segment().unwrap();
        next.reserve(100_000).unwrap();
        next.discard().unwrap();
        assert!(run.path().join("rrd/000001.rrd.partial").is_file());
    }

    #[test]
    fn interrupted_retirement_resumes_and_unfinished_recordings_survive_reopen() {
        let root = Workspace::new("");
        let first = root.run();
        let expired = first.path().to_owned();
        let runs = first.0.runs.clone();
        drop(first);
        let board = Run::open(&root.0, "board", Some("boot"), None).unwrap();
        let path = board.path().to_owned();
        let mut partial = board.rrd_segment().unwrap();
        partial.write_all(b"unfinished payload").unwrap();
        drop(partial);
        drop(board);
        File::create(runs.join(".prune-in-progress")).unwrap();
        fs::rename(expired, runs.join(".deleting")).unwrap();
        let resumed = Run::open(&root.0, "board", Some("boot"), None).unwrap();
        assert_eq!(resumed.path(), path);
        assert!(!runs.join(".deleting").exists());
        assert_eq!(
            fs::read(path.join("rrd/000001.rrd.partial")).unwrap(),
            b"unfinished payload"
        );
        let mut next = resumed.rrd_segment().unwrap();
        next.write_all(b"latest evidence").unwrap();
        next.seal().unwrap();
        assert!(path.join("rrd/000002.rrd").is_file());
    }
}
