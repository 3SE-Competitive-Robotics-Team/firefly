//! System events for the host processes participating in a command archive.
use crate::Run;
use std::{
    collections::BTreeSet,
    io::{self, BufRead, BufReader, Read, Write},
    process::{Command, Stdio},
    time::{SystemTime, UNIX_EPOCH},
};

/// Collect the host journal for the exact process IDs and duration of the command.
///
/// # Errors
/// Returns source, subprocess, or archive write errors.
pub fn system(run: &Run, start: SystemTime, pids: &BTreeSet<u32>) -> io::Result<()> {
    if pids.is_empty() {
        return Ok(());
    }
    let stamp = |time: SystemTime| -> io::Result<String> {
        Ok(format!(
            "@{}",
            time.duration_since(UNIX_EPOCH)
                .map_err(io::Error::other)?
                .as_secs()
        ))
    };
    let start = stamp(start)?;
    let end = stamp(SystemTime::now() + std::time::Duration::from_secs(1))?;
    let mut cmd = if cfg!(target_os = "macos") {
        let predicate = pids
            .iter()
            .map(|pid| format!("processIdentifier == {pid}"))
            .collect::<Vec<_>>()
            .join(" OR ");
        let mut cmd = Command::new("/usr/bin/log");
        cmd.args([
            "show",
            "--info",
            "--style",
            "ndjson",
            "--start",
            &start,
            "--end",
            &end,
            "--predicate",
            &predicate,
        ]);
        cmd
    } else {
        let mut cmd = Command::new("journalctl");
        cmd.args([
            "--boot",
            "--no-pager",
            "--output=json",
            "--since",
            &start,
            "--until",
            &end,
        ])
        .args(pids.iter().map(|pid| format!("_PID={pid}")));
        cmd
    };
    let mut output = run.stream("system", "host")?;
    let mut child = cmd.stdin(Stdio::null()).stdout(Stdio::piped()).spawn()?;
    let stdout = child
        .stdout
        .take()
        .ok_or_else(|| io::Error::other("missing system journal pipe"))?;
    let mut pipe = BufReader::new(stdout);
    let mut row = Vec::new();
    let copied = (|| -> io::Result<()> {
        loop {
            row.clear();
            if (&mut pipe).take(128 << 10).read_until(b'\n', &mut row)? == 0 {
                break;
            }
            output.write_all(&row)?;
        }
        output.flush()
    })();
    if copied.is_err() {
        let _ = child.kill();
    }
    let status = child.wait()?;
    copied?;
    if !status.success() {
        return Err(io::Error::other(format!(
            "host system journal exited {status}"
        )));
    }
    Ok(())
}
