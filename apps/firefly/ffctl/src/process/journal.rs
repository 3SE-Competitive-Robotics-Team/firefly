//! Archive the kernel and Firefly service journal for the board boot.

use std::fs::{self, File};
use std::io::{self, BufRead, BufReader, Read, Seek, SeekFrom, Write};
use std::path::Path;
use std::process::{Command, Stdio};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use firefly_archive::Run;
use serde::Deserialize;

const RECORD_LIMIT: usize = 128 * 1024;
const POLL: Duration = Duration::from_millis(100);
const SOURCE: &str = "board";
const UNITS: [&str; 3] = [
    "firefly.service",
    "firefly-daemon.service",
    "firefly-mimir.service",
];

pub(super) struct Journal {
    stop: Arc<AtomicBool>,
    worker: JoinHandle<()>,
}

impl Journal {
    pub(super) fn start(mut run: Option<Run>) -> io::Result<Self> {
        let stop = Arc::new(AtomicBool::new(false));
        let shutdown = Arc::clone(&stop);
        let worker = thread::Builder::new()
            .name("board-journal".into())
            .spawn(move || {
                let mut last_error = String::new();
                while !shutdown.load(Ordering::Relaxed) {
                    let result = (|| {
                        let run = match &run {
                            Some(run) => run,
                            None => run.insert(Run::board()?),
                        };
                        follow(run, &shutdown)
                    })();
                    if let Err(error) = result {
                        let detail = error.to_string();
                        if detail != last_error {
                            eprintln!("ffctl: system journal: {error}");
                            if let Some(run) = &run
                                && let Err(error) = run.gap("system-board", &detail)
                            {
                                eprintln!("ffctl: journal gap: {error}");
                            }
                            last_error = detail;
                        }
                        if error.kind() == io::ErrorKind::NotFound && !cfg!(target_os = "linux") {
                            break;
                        }
                        let deadline = Instant::now() + Duration::from_secs(2);
                        while !shutdown.load(Ordering::Relaxed) && Instant::now() < deadline {
                            thread::sleep(POLL);
                        }
                    }
                }
            })?;
        Ok(Self { stop, worker })
    }

    pub(super) fn finish(self) {
        self.stop.store(true, Ordering::Relaxed);
        if self.worker.join().is_err() {
            eprintln!("ffctl: system journal worker panicked");
        }
    }
}

fn follow(run: &Run, stop: &AtomicBool) -> io::Result<()> {
    let mut command = Command::new("journalctl");
    command.args(["--boot", "--follow", "--no-tail", "--no-pager", "--output=json", "--all",
        "--output-fields=MESSAGE,PRIORITY,SYSLOG_IDENTIFIER,_PID,_TRANSPORT,_SYSTEMD_UNIT,UNIT,COREDUMP_UNIT"]);
    let cursor = last_cursor(run.path())?;
    if let Some(cursor) = &cursor {
        command.arg(format!("--cursor={cursor}"));
    }
    command.arg("_TRANSPORT=kernel");
    command.args(["+", "SYSLOG_IDENTIFIER=firefly-hil-gadget"]);
    for field in ["_SYSTEMD_UNIT", "UNIT", "COREDUMP_UNIT"] {
        command.arg("+");
        for unit in UNITS {
            command.arg(format!("{field}={unit}"));
        }
    }
    let mut child = command
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .spawn()?;
    let pipe = child.stdout.take().expect("piped journal stdout");
    let (status, copied) = thread::scope(|scope| -> io::Result<_> {
        let copier =
            scope.spawn(move || copy_records(run, BufReader::new(pipe), cursor.as_deref()));
        let status = loop {
            if let Some(status) = child.try_wait()? {
                break status;
            }
            if stop.load(Ordering::Relaxed) || copier.is_finished() {
                let _ = child.kill();
                break child.wait()?;
            }
            thread::sleep(POLL);
        };
        Ok((
            status,
            copier
                .join()
                .map_err(|_| io::Error::other("journal reader panicked"))?,
        ))
    })?;
    copied?;
    if stop.load(Ordering::Relaxed) {
        return Ok(());
    }
    Err(io::Error::other(format!(
        "journalctl exited ({status}); resuming from archived cursor"
    )))
}

#[derive(Deserialize)]
struct Position<'a> {
    #[serde(rename = "__CURSOR")]
    cursor: &'a str,
}

fn copy_records(
    run: &Run,
    mut input: impl BufRead,
    mut checkpoint: Option<&str>,
) -> io::Result<()> {
    let mut stream = run.stream("system", SOURCE)?;
    let mut row = Vec::new();
    let mut last_sync = Instant::now();
    loop {
        row.clear();
        if (&mut input)
            .take(RECORD_LIMIT as u64)
            .read_until(b'\n', &mut row)?
            == 0
        {
            break;
        }
        if !row.ends_with(b"\n") {
            input.skip_until(b'\n')?;
            run.gap("system-board", "oversized or interrupted journal record")?;
            continue;
        }
        let position: Position = serde_json::from_slice(&row).map_err(io::Error::other)?;
        if let Some(expected) = checkpoint.take() {
            if position.cursor == expected {
                continue;
            }
            run.gap(
                "system-board",
                "journal checkpoint expired before capture resumed",
            )?;
        }
        stream.write_all(&row)?;
        if last_sync.elapsed() >= Duration::from_secs(1) {
            stream.flush()?;
            last_sync = Instant::now();
        }
    }
    stream.flush()
}

fn last_cursor(run_path: &Path) -> io::Result<Option<String>> {
    let entries = match fs::read_dir(run_path.join("system")) {
        Ok(entries) => entries,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(None),
        Err(error) => return Err(error),
    };
    let mut segments = Vec::new();
    for entry in entries {
        let entry = entry?;
        if !entry.file_type()?.is_file() {
            continue;
        }
        let name = entry.file_name();
        if let Some(number) = name
            .to_str()
            .and_then(|name| name.strip_prefix("board."))
            .and_then(|name| name.strip_suffix(".log"))
            .and_then(|n| n.parse::<u64>().ok())
        {
            segments.push((number, entry.path()));
        }
    }
    segments.sort_unstable_by_key(|(number, _)| *number);
    for (_, segment) in segments.into_iter().rev() {
        let mut file = File::open(segment)?;
        // A torn final record and the preceding complete record both fit in this window.
        let offset = file
            .metadata()?
            .len()
            .saturating_sub(2 * RECORD_LIMIT as u64 + 1);
        file.seek(SeekFrom::Start(offset))?;
        let mut tail = Vec::new();
        file.take(2 * RECORD_LIMIT as u64 + 1)
            .read_to_end(&mut tail)?;
        if let Some(cursor) = cursor_from_tail(&tail) {
            return Ok(Some(cursor));
        }
    }
    Ok(None)
}

fn cursor_from_tail(tail: &[u8]) -> Option<String> {
    let end = tail.iter().rposition(|byte| *byte == b'\n')?;
    tail[..end].rsplit(|byte| *byte == b'\n').find_map(|row| {
        let position: Position = serde_json::from_slice(row).ok()?;
        Some(position.cursor.to_owned())
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resumes_after_last_complete_record_including_after_torn_writes() {
        let rows = b"{\"__CURSOR\":\"first\"}\n{\"__CURSOR\":\"second\"}\n{\"__CUR";
        assert_eq!(cursor_from_tail(rows).as_deref(), Some("second"));
        assert_eq!(cursor_from_tail(b"{\"__CURSOR\":\"torn\"}"), None);
    }

    #[test]
    fn journal_restarts_deduplicate_or_report_expired_checkpoint() {
        let root = firefly_config::paths::root()
            .join("target/ffctl-tests")
            .join(format!("cursor-{}", std::process::id()));
        let run = Run::create_at(&root, "journal-test").unwrap();
        copy_records(&run, &b"{\"__CURSOR\":\"a\"}\n"[..], None).unwrap();
        let checkpoint = last_cursor(run.path()).unwrap();
        copy_records(
            &run,
            &b"{\"__CURSOR\":\"a\"}\n{\"__CURSOR\":\"b\"}\n"[..],
            checkpoint.as_deref(),
        )
        .unwrap();
        let rows: String = fs::read_dir(run.path().join("system"))
            .unwrap()
            .map(|e| e.unwrap().path())
            .filter(|path| path.extension().is_some_and(|ext| ext == "log"))
            .map(|path| fs::read_to_string(path).unwrap())
            .collect();
        assert_eq!(rows.matches("{\"__CURSOR\":\"a\"}\n").count(), 1);
        assert_eq!(rows.matches("{\"__CURSOR\":\"b\"}\n").count(), 1);
        let checkpoint = last_cursor(run.path()).unwrap();
        copy_records(&run, &b"{\"__CURSOR\":\"d\"}\n"[..], checkpoint.as_deref()).unwrap();
        assert_eq!(last_cursor(run.path()).unwrap().as_deref(), Some("d"));
        let metadata = fs::read_to_string(run.path().join("manifest.json")).unwrap();
        assert!(metadata.contains("journal checkpoint expired"));
        drop(run);
        fs::remove_dir_all(root).unwrap();
    }
}
