use std::{
    fs::{self, File},
    io::{self, Read, Seek, Write},
    os::unix::fs::FileExt,
    path::{Path, PathBuf},
    time::{Duration, Instant},
};

use crate::{
    Eviction, Limits, METADATA_BUDGET, Manifest, Run, global, invalid, lock_file, millis, sync_dir,
    validate_source,
};

pub(super) fn exclusive(file: &File) -> io::Result<bool> {
    match file.try_lock() {
        Ok(()) => Ok(true),
        Err(std::fs::TryLockError::WouldBlock) => Ok(false),
        Err(std::fs::TryLockError::Error(error)) => Err(error),
    }
}

fn slot(path: &Path) -> Option<u16> {
    let digits = path.file_name()?.to_str()?.strip_prefix("log_")?;
    if digits.len() != 4 || !digits.bytes().all(|c| c.is_ascii_digit()) {
        return None;
    }
    digits.parse::<u16>().ok().filter(|&n| n != 0)
}

// Called under the root lock. Candidates are ordered by creation sequence, including
// the previous newest run if the byte-budget cleanup has just retired it.
pub(super) fn next_directory(
    root: &Path,
    runs: &[(Manifest, PathBuf)],
    slots: u16,
) -> io::Result<PathBuf> {
    let previous = runs
        .iter()
        .rev()
        .find_map(|(_, path)| slot(path))
        .unwrap_or(0);
    for offset in 0..slots {
        let number = (previous + offset) % slots + 1;
        let path = root.join(format!("log_{number:04}"));
        match fs::symlink_metadata(&path) {
            Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(path),
            Err(e) => return Err(e),
            Ok(_) => {}
        }
    }
    let boot_id = current_boot();
    for (manifest, path) in runs {
        if slot(path).is_some_and(|n| n <= slots)
            && !this_boot(manifest, boot_id.as_deref())
            && retire(root, path)?
        {
            return Ok(path.clone());
        }
    }
    Err(io::Error::new(
        io::ErrorKind::StorageFull,
        "all run directory numbers are occupied",
    ))
}

pub(super) fn managed(root: &Path) -> io::Result<Vec<(Manifest, PathBuf)>> {
    let intent = root.join(".prune-in-progress");
    if intent.try_exists()? {
        finish_retirement(root, &intent)?;
    }
    let mut runs = Vec::new();
    for entry in fs::read_dir(root)? {
        let entry = entry?;
        if !entry.file_type()?.is_dir()
            || (slot(&entry.path()).is_none()
                && entry
                    .file_name()
                    .as_encoded_bytes()
                    .first()
                    .is_none_or(|b| !b.is_ascii_digit()))
        {
            continue;
        }
        let path = entry.path();
        match Manifest::read(&path) {
            Ok(manifest) => runs.push((manifest, path)),
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::NotFound | io::ErrorKind::InvalidData
                ) => {}
            Err(error) => return Err(error),
        }
    }
    runs.sort_by_key(|(m, _)| m.sequence);
    Ok(runs)
}

fn claim_path(data: &Path) -> PathBuf {
    data.with_added_extension("credit")
}

#[derive(Default)]
struct Usage {
    charged: u64,
    outstanding: u64,
}

// A live claim contains the funded end offset. A dead writer's file length is
// the only remaining charge. Claim changes and directory mutations hold the global lock.
fn usage(path: &Path) -> io::Result<Usage> {
    let mut total = Usage {
        charged: METADATA_BUDGET,
        outstanding: 0,
    };
    for kind in ["rrd", "system", "text"] {
        for entry in fs::read_dir(path.join(kind))? {
            let entry = entry?;
            let data = entry.path();
            if data.extension().is_some_and(|ext| ext == "credit") {
                // A crash can occur after publishing the RRD and before unlinking its lease.
                if !data.with_extension("").try_exists()? && data.try_exists()? {
                    fs::remove_file(data)?;
                }
                continue;
            }
            if !entry.file_type()?.is_file() {
                continue;
            }
            let length = entry.metadata()?.len();
            let mut end = length;
            let claim = claim_path(&data);
            match File::open(&claim) {
                Ok(mut file) if !exclusive(&file)? => {
                    let mut buf = [0; 8];
                    file.read_exact(&mut buf)?;
                    end = length.max(u64::from_le_bytes(buf));
                }
                Ok(_) => fs::remove_file(claim)?,
                Err(error) if error.kind() == io::ErrorKind::NotFound => (),
                Err(error) => return Err(error),
            }
            total.charged = total.charged.saturating_add(end);
            total.outstanding = total.outstanding.saturating_add(end - length);
        }
    }
    Ok(total)
}

fn available(root: &Path) -> io::Result<u64> {
    let fs = nix::sys::statvfs::statvfs(root)?;
    let bytes = u128::from(fs.blocks_available()) * u128::from(fs.fragment_size());
    Ok(u64::try_from(bytes).unwrap_or(u64::MAX))
}

// Acquire both locks nonblocking while holding the root lock. Readers and
// writers can finish without ever waiting on a pruner that is waiting for them.
fn retire(root: &Path, path: &Path) -> io::Result<bool> {
    let active = File::open(path.join(".active.lock"))?;
    let readers = File::open(path.join(".read.lock"))?;
    if !exclusive(&active)? || !exclusive(&readers)? {
        return Ok(false);
    }
    let trash = root.join(".deleting");
    if trash.try_exists()? {
        return Err(invalid("archive retirement path is occupied"));
    }
    let intent = root.join(".prune-in-progress");
    File::create_new(&intent)?.sync_all()?;
    sync_dir(root)?;
    fs::rename(path, &trash)?;
    sync_dir(root)?;
    finish_retirement(root, &intent)?;
    Ok(true)
}

fn finish_retirement(root: &Path, intent: &Path) -> io::Result<()> {
    let trash = root.join(".deleting");
    if trash.try_exists()? {
        fs::remove_dir_all(trash)?;
        sync_dir(root)?;
    }
    fs::remove_file(intent)?;
    sync_dir(root)
}

fn sequence(path: &Path, prefix: &str, suffix: &str) -> Option<u64> {
    path.file_name()?
        .to_str()?
        .strip_prefix(prefix)?
        .strip_suffix(suffix)?
        .parse()
        .ok()
}

fn segments(dir: &Path, prefix: &str, suffix: &str) -> io::Result<Vec<(u64, PathBuf)>> {
    let mut files = Vec::new();
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if let Some(n) = sequence(&path, prefix, suffix)
            && entry.file_type()?.is_file()
        {
            files.push((n, path));
        }
    }
    files.sort_by_key(|(n, _)| *n);
    Ok(files)
}

fn evict(path: &Path, source: &str, n: u64, file: &Path) -> io::Result<bool> {
    if let Ok(claim) = File::open(claim_path(file))
        && !exclusive(&claim)?
    {
        return Ok(false);
    }
    let mut m = Manifest::read(path)?;
    let meta = fs::metadata(file)?;
    let end = millis(meta.modified()?);
    let begin = meta.created().map_or(end, millis);
    let span = m.evicted.entry(source.to_owned()).or_insert(Eviction {
        first: n,
        last: n,
        first_ms: begin,
        last_ms: end,
    });
    span.first = span.first.min(n);
    span.last = span.last.max(n);
    span.first_ms = span.first_ms.min(begin);
    span.last_ms = span.last_ms.max(end);
    m.save(path)?;
    fs::remove_file(file)?;
    Ok(true)
}

fn prune_rrd(path: &Path) -> io::Result<bool> {
    let readers = File::open(path.join(".read.lock"))?;
    if !exclusive(&readers)? {
        return Ok(false);
    }
    let mut files = segments(&path.join("rrd"), "", ".rrd")?;
    // The latest sealed segment remains independently replayable during recovery.
    files.pop();
    files.extend(segments(&path.join("rrd"), "", ".rrd.partial")?);
    files.sort_by_key(|(n, _)| *n);
    for (n, file) in files {
        if evict(path, "rrd", n, &file)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn prune_old_stream(path: &Path) -> io::Result<bool> {
    let mut oldest = None;
    for kind in ["text", "system"] {
        let mut groups = std::collections::BTreeMap::<String, Vec<(u64, PathBuf)>>::new();
        for entry in fs::read_dir(path.join(kind))? {
            let entry = entry?;
            if !entry.file_type()?.is_file() {
                continue;
            }
            let name = entry.file_name();
            if let Some((source, n)) = name
                .to_str()
                .and_then(|name| name.strip_suffix(".log"))
                .and_then(|stem| stem.rsplit_once('.'))
                && let Ok(n) = n.parse::<u64>()
            {
                groups
                    .entry(source.to_owned())
                    .or_default()
                    .push((n, entry.path()));
            }
        }
        for (source, mut files) in groups {
            if files.len() < 2 {
                continue;
            }
            files.sort_by_key(|(n, _)| *n);
            let (n, file) = files.remove(0);
            let stamp = fs::metadata(&file)?.modified()?;
            if oldest.as_ref().is_none_or(|(t, _, _, _)| stamp < *t) {
                oldest = Some((stamp, format!("{kind}.{source}"), n, file));
            }
        }
    }
    if let Some((_, source, n, file)) = oldest {
        evict(path, &source, n, &file)
    } else {
        Ok(false)
    }
}

fn current_boot() -> Option<String> {
    if cfg!(target_os = "linux") {
        fs::read_to_string("/proc/sys/kernel/random/boot_id").ok()
    } else {
        None
    }
}

fn this_boot(manifest: &Manifest, current: Option<&str>) -> bool {
    manifest
        .boot_id
        .as_deref()
        .is_some_and(|id| current.is_some_and(|boot| boot.trim() == id))
}

pub(super) fn make_room(
    root: &Path,
    current: Option<&Path>,
    limits: &Limits,
    extra: u64,
) -> io::Result<()> {
    if extra > limits.max_bytes {
        return Err(io::Error::new(
            io::ErrorKind::StorageFull,
            "write is larger than the archive budget",
        ));
    }
    let mut cleaning = false;
    loop {
        let runs = managed(root)?;
        let mut total = extra;
        let mut promised = extra;
        for (_, path) in &runs {
            let usage = usage(path)?;
            total = total.saturating_add(usage.charged);
            promised = promised.saturating_add(usage.outstanding);
        }
        let enough_disk = available(root)? >= promised.saturating_add(limits.keep_free_bytes);
        let fits = total <= limits.max_bytes && enough_disk;
        if fits && (!cleaning || total <= limits.cleanup_bytes) {
            return Ok(());
        }
        cleaning = true;
        let mut retired = false;
        let boot_id = current_boot();
        for (m, path) in runs {
            if this_boot(&m, boot_id.as_deref()) {
                continue;
            }
            if Some(path.as_path()) != current && retire(root, &path)? {
                retired = true;
                break;
            }
        }
        if retired {
            continue;
        }
        if let Some(path) = current
            && (prune_rrd(path)? || prune_old_stream(path)?)
        {
            continue;
        }
        return if fits {
            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::StorageFull,
                "Firefly archive budget exhausted",
            ))
        };
    }
}

/// A quota-backed file. Unsealed RRD files retain their partial suffix.
pub struct Segment {
    run: Run,
    data: File,
    path: PathBuf,
    claim: File,
    end: u64,
    bytes: u64,
    checked: Instant,
}

impl Segment {
    pub(super) fn create(
        run: &Run,
        kind: &str,
        source: &str,
        first_write: u64,
    ) -> io::Result<Self> {
        let _guard = global(&run.0.runs)?;
        if first_write > 0 {
            make_room(&run.0.runs, Some(run.path()), &run.0.limits, first_write)?;
        }
        let dir = run.path().join(kind);
        let path = if kind == "rrd" {
            let mut m = Manifest::read(run.path())?;
            m.rrd_last = m
                .rrd_last
                .checked_add(1)
                .ok_or_else(|| invalid("RRD sequence exhausted"))?;
            m.save(run.path())?;
            dir.join(format!("{:06}.rrd.partial", m.rrd_last))
        } else {
            let prefix = format!("{source}.");
            let files = segments(&dir, &prefix, ".log")?;
            if let Some((_, path)) = files.last()
                && let Some(segment) = Self::resume(run, path, first_write)?
            {
                return Ok(segment);
            }
            let n = files.last().map_or(1, |(n, _)| n.saturating_add(1));
            let count = files
                .len()
                .saturating_add(1)
                .saturating_sub(run.0.limits.stream_segments);
            for (old, path) in files.iter().take(count) {
                if !evict(run.path(), &format!("{kind}.{source}"), *old, path)? {
                    return Err(io::Error::new(
                        io::ErrorKind::WouldBlock,
                        "source segment in use",
                    ));
                }
            }
            dir.join(format!("{prefix}{n:06}.log"))
        };
        let claim_path = claim_path(&path);
        let mut claim = lock_file(&claim_path)?;
        claim.lock()?;
        claim.write_all(&first_write.to_le_bytes())?;
        let data = File::create_new(&path)?;
        sync_dir(&dir)?;
        Ok(Self {
            run: run.clone(),
            data,
            path,
            claim,
            end: first_write,
            bytes: 0,
            checked: Instant::now(),
        })
    }

    // Preserve short segments across a service or journal collector restart.
    // A torn final line remains intact for inspection in the previous file.
    fn resume(run: &Run, path: &Path, first_write: u64) -> io::Result<Option<Self>> {
        let data = File::options().read(true).append(true).open(path)?;
        let bytes = data.metadata()?.len();
        let end = bytes.saturating_add(first_write);
        if end > run.0.limits.stream_segment_bytes {
            return Ok(None);
        }
        if bytes != 0 {
            let mut last = [0];
            data.read_exact_at(&mut last, bytes - 1)?;
            if last[0] != b'\n' {
                return Ok(None);
            }
        }
        let mut claim = lock_file(&claim_path(path))?;
        if !exclusive(&claim)? {
            return Ok(None);
        }
        claim.write_all(&end.to_le_bytes())?;
        Ok(Some(Self {
            run: run.clone(),
            data,
            path: path.to_owned(),
            claim,
            end,
            bytes,
            checked: Instant::now(),
        }))
    }

    /// Number of bytes written into the active segment.
    #[must_use]
    pub fn bytes(&self) -> u64 {
        self.bytes
    }

    /// Guarantee at least this much additional capacity for data and footer.
    /// # Errors
    /// Returns storage-full when no eligible logs can be reclaimed.
    pub fn reserve(&mut self, additional: u64) -> io::Result<()> {
        let need = self
            .bytes
            .checked_add(additional)
            .ok_or_else(|| invalid("segment size overflow"))?;
        if need <= self.end {
            if self.checked.elapsed() >= Duration::from_secs(10) {
                let _guard = global(&self.run.0.runs)?;
                // Already-funded bytes include the footer of the open RRD.
                if let Err(error) = make_room(
                    &self.run.0.runs,
                    Some(self.run.path()),
                    &self.run.0.limits,
                    0,
                ) && error.kind() != io::ErrorKind::StorageFull
                {
                    return Err(error);
                }
                self.checked = Instant::now();
            }
            return Ok(());
        }
        let _guard = global(&self.run.0.runs)?;
        let limits = &self.run.0.limits;
        // Small test budgets and short text segments should not reserve a large window.
        let window = (limits.max_bytes / 128).clamp(1, 1 << 20);
        let preferred = need.saturating_add(window - 1) / window * window;
        let fund = |end| {
            make_room(
                &self.run.0.runs,
                Some(self.run.path()),
                limits,
                end - self.end,
            )
        };
        let end = match fund(preferred) {
            Ok(()) => preferred,
            Err(e) if preferred != need && e.kind() == io::ErrorKind::StorageFull => {
                fund(need)?;
                need
            }
            Err(e) => return Err(e),
        };
        self.claim.rewind()?;
        self.claim.write_all(&end.to_le_bytes())?;
        self.end = end;
        self.checked = Instant::now();
        Ok(())
    }

    /// Publish a complete RRD or finish writing a log segment.
    /// # Errors
    /// Returns file synchronization or rename errors.
    pub fn seal(self) -> io::Result<()> {
        self.data.sync_all()?;
        let _guard = global(&self.run.0.runs)?;
        if self.path.extension().is_some_and(|ext| ext == "partial") {
            fs::rename(&self.path, self.path.with_extension(""))?;
            sync_dir(&self.run.path().join("rrd"))?;
        }
        fs::remove_file(claim_path(&self.path))
    }

    /// Remove a segment whose initialization failed before accepting observations.
    /// # Errors
    /// Returns filesystem errors.
    pub fn discard(self) -> io::Result<()> {
        let _guard = global(&self.run.0.runs)?;
        fs::remove_file(&self.path)?;
        fs::remove_file(claim_path(&self.path))
    }
}

impl Write for Segment {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.reserve(buf.len() as u64)?;
        let n = self.data.write(buf)?;
        self.bytes += n as u64;
        Ok(n)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.data.flush()
    }
}

/// Record-aligned rolling output for one named process or system source.
pub struct Stream {
    run: Run,
    kind: String,
    source: String,
    segment: Option<Segment>,
}

impl Stream {
    pub(super) fn new(run: &Run, kind: &str, source: &str) -> io::Result<Self> {
        validate_source(source)?;
        if !matches!(kind, "text" | "system") {
            return Err(invalid("unknown archive stream kind"));
        }
        Ok(Self {
            run: run.clone(),
            kind: kind.into(),
            source: source.into(),
            segment: None,
        })
    }
}

impl Write for Stream {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        if self.segment.as_ref().is_some_and(|s| {
            s.bytes > 0
                && s.bytes.saturating_add(buf.len() as u64) > self.run.0.limits.stream_segment_bytes
        }) && let Some(old) = self.segment.take()
        {
            old.seal()?;
        }
        if self.segment.is_none() {
            self.segment = Some(Segment::create(
                &self.run,
                &self.kind,
                &self.source,
                buf.len() as u64,
            )?);
        }
        let segment = self.segment.as_mut().unwrap();
        segment.reserve(buf.len() as u64)?;
        segment.write_all(buf)?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        if let Some(segment) = &self.segment {
            segment.data.sync_data()?;
        }
        Ok(())
    }
}

impl Drop for Stream {
    fn drop(&mut self) {
        if let Some(segment) = self.segment.take()
            && let Err(error) = segment.seal()
        {
            eprintln!("close {} stream {}: {error}", self.kind, self.source);
        }
    }
}
