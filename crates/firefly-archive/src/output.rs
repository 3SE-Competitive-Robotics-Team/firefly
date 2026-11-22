//! Bounded, asynchronous capture of supervised process output.

use std::io::{self, BufRead, BufReader, Read, Write};
use std::process::Child;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, SyncSender};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::Run;

const PAYLOAD: usize = 4096;
const QUEUE: usize = 128;

/// A producer handle for one named process. The original handle owns the writer thread.
pub struct Output {
    sender: SyncSender<Vec<u8>>,
    lost: Arc<AtomicU64>,
    worker: Option<JoinHandle<io::Result<()>>>,
}

impl Clone for Output {
    fn clone(&self) -> Self {
        Self {
            sender: self.sender.clone(),
            lost: Arc::clone(&self.lost),
            worker: None,
        }
    }
}

impl Output {
    /// Create a bounded queue for both output streams of a source.
    ///
    /// # Errors
    /// Returns a thread creation error if the archive writer cannot start.
    pub fn new(run: Run, source: &str) -> io::Result<Self> {
        Self::start(Some(run), source.to_owned())
    }

    /// Start collecting even while the board's boot archive is temporarily unavailable.
    ///
    /// # Errors
    /// Returns a thread creation error if the archive writer cannot start.
    pub fn board(source: &str) -> io::Result<Self> {
        Self::start(None, source.to_owned())
    }

    fn start(run: Option<Run>, source: String) -> io::Result<Self> {
        let (sender, receiver) = mpsc::sync_channel(QUEUE);
        let lost = Arc::new(AtomicU64::new(0));
        let dropped = Arc::clone(&lost);
        let worker = thread::Builder::new()
            .name(format!("text-{source}"))
            .spawn(move || write_stream(run, &source, &receiver, &dropped))?;
        Ok(Self {
            sender,
            lost,
            worker: Some(worker),
        })
    }

    /// Queue an output record with its receipt time, OS process ID and original stream.
    pub fn line(&self, pid: u32, stream: &'static str, bytes: &[u8]) {
        let bytes = bytes.strip_suffix(b"\n").unwrap_or(bytes);
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis();
        let prefix = format!("{now} {pid} {stream} ");
        let mut chunks = bytes.chunks(PAYLOAD);
        let mut chunk = chunks.next().unwrap_or_default();
        loop {
            let mut row = Vec::with_capacity(prefix.len() + chunk.len() + 1);
            row.extend_from_slice(prefix.as_bytes());
            row.extend_from_slice(chunk);
            row.push(b'\n');
            if self.sender.try_send(row).is_err() {
                self.lost.fetch_add(1, Ordering::Relaxed);
            }
            let Some(next) = chunks.next() else { break };
            chunk = next;
        }
    }

    /// Start draining each piped descriptor the caller left on the child.
    pub fn attach(&self, child: &mut Child) {
        let pid = child.id();
        if let Some(pipe) = child.stdout.take() {
            self.drain(pid, "stdout", pipe);
        }
        if let Some(pipe) = child.stderr.take() {
            self.drain(pid, "stderr", pipe);
        }
    }

    fn drain(&self, pid: u32, stream: &'static str, pipe: impl Read + Send + 'static) {
        let output = self.clone();
        thread::spawn(move || {
            let mut pipe = BufReader::new(pipe);
            let mut row = Vec::with_capacity(PAYLOAD);
            let mut continued = false;
            loop {
                row.clear();
                match (&mut pipe).take(PAYLOAD as u64).read_until(b'\n', &mut row) {
                    Ok(0) => break,
                    Ok(_) => {
                        if !continued || row != b"\n" {
                            output.line(pid, stream, &row);
                        }
                        continued = row.len() == PAYLOAD && !row.ends_with(b"\n");
                    }
                    Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
                    Err(error) => {
                        eprintln!("text capture {pid} {stream}: {error}");
                        output.lost.fetch_add(1, Ordering::Relaxed);
                        break;
                    }
                }
            }
        });
    }

    /// Drain and sync accepted records. Call on the original handle after child exit
    /// and after releasing any extra producer handles.
    ///
    /// # Errors
    /// Returns an error if the writer panicked, lost records, or could not sync the log.
    pub fn finish(self) -> io::Result<()> {
        drop(self.sender);
        if let Some(worker) = self.worker {
            worker
                .join()
                .map_err(|_| io::Error::other("text archive writer panicked"))??;
        }
        Ok(())
    }
}

fn write_stream(
    mut run: Option<Run>,
    source: &str,
    receiver: &Receiver<Vec<u8>>,
    dropped: &AtomicU64,
) -> io::Result<()> {
    let mut stream = None;
    let mut retry_at = None;
    let mut total_lost = 0_u64;
    let mut reported = 0_u64;
    let mut next_report = Instant::now();

    for row in receiver {
        total_lost += dropped.swap(0, Ordering::Relaxed);
        if retry_at.is_some_and(|at| Instant::now() < at) {
            total_lost += 1;
            continue;
        }
        let written = (|| -> io::Result<()> {
            let run = match &run {
                Some(run) => run,
                None => run.insert(Run::board()?),
            };
            let writer = match &mut stream {
                Some(writer) => writer,
                None => stream.insert(run.stream("text", source)?),
            };
            writer.write_all(&row)
        })();
        match written {
            Ok(()) if reported < total_lost && Instant::now() >= next_report => {
                retry_at = None;
                let detail = format!("text archive dropped {total_lost} records");
                eprintln!("{source}: {detail}");
                if let Some(run) = &run {
                    match run.gap(source, &detail) {
                        Ok(()) => reported = total_lost,
                        Err(error) => eprintln!("text archive {source}: {error}"),
                    }
                }
                next_report = Instant::now() + Duration::from_secs(1);
            }
            Ok(()) => retry_at = None,
            Err(error) => {
                if retry_at.is_none() {
                    eprintln!("text archive {source}: {error}");
                }
                if error.kind() != io::ErrorKind::StorageFull {
                    stream = None;
                }
                retry_at = Some(Instant::now() + Duration::from_secs(1));
                total_lost += 1;
            }
        }
    }
    if let Some(mut writer) = stream {
        writer.flush()?;
    }
    total_lost += dropped.swap(0, Ordering::Relaxed);
    if total_lost > 0 {
        let detail = format!("text archive dropped {total_lost} records");
        if reported < total_lost
            && let Some(run) = &run
        {
            run.gap(source, &detail)?;
        }
        return Err(io::Error::other(detail));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::io::Cursor;
    use std::path::PathBuf;

    use super::*;

    fn fixture(name: &str) -> (PathBuf, Run) {
        let root = firefly_config::paths::root()
            .join("target/firefly-archive-tests")
            .join(format!("{name}-{}", std::process::id()));
        let run = Run::create_at(&root, name).unwrap();
        (root, run)
    }

    #[test]
    fn exact_payload_boundary_preserves_blank_lines_and_final_unterminated_line() {
        let (root, run) = fixture("capture-boundaries");
        let output = Output::new(run.clone(), "fixture").unwrap();
        let mut data = vec![b'x'; PAYLOAD];
        data.extend_from_slice(b"\n\nend");
        output.drain(123, "stdout", Cursor::new(data));
        output.finish().unwrap();
        let text = fs::read_to_string(run.path().join("text/fixture.000001.log")).unwrap();
        let rows: Vec<_> = text.lines().collect();
        assert_eq!(rows.len(), 3);
        assert!(rows[0].ends_with(&"x".repeat(PAYLOAD)));
        assert!(rows[1].ends_with("123 stdout "));
        assert!(rows[2].ends_with("123 stdout end"));
        drop(run);
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn saturated_reader_reports_lost_records_and_keeps_accepted_records() {
        let (root, run) = fixture("capture-overload");
        let (sender, receiver) = mpsc::sync_channel(1);
        let dropped = Arc::new(AtomicU64::new(0));
        let output = Output {
            sender,
            lost: Arc::clone(&dropped),
            worker: None,
        };
        output.line(2, "stderr", b"accepted");
        output.line(2, "stderr", b"dropped");
        drop(output);
        assert!(write_stream(Some(run.clone()), "fixture", &receiver, &dropped).is_err());
        let text = fs::read_to_string(run.path().join("text/fixture.000001.log")).unwrap();
        assert!(text.ends_with("2 stderr accepted\n"));
        let metadata = fs::read_to_string(run.path().join("manifest.json")).unwrap();
        assert!(metadata.contains("text archive dropped 1 records"));
        drop(run);
        fs::remove_dir_all(root).unwrap();
    }
}
