use firefly_archive::{Run, output::Output, reader::RecordingFiles};
use serde::Deserialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    io::{self, BufRead, Write},
    path::Path,
    process::Command,
    time::SystemTime,
};

pub fn view(path: &Path) -> io::Result<()> {
    let files = RecordingFiles::retained(path)?;
    let viewer = super::target::workspace_root().join(".venv/bin/rerun");
    let status = Command::new(viewer).args(files.paths()).status()?;
    if !status.success() {
        return Err(io::Error::other(format!("viewer exited {status}")));
    }
    Ok(())
}

/// The Python launcher shares Rust's locks, disk budget and process-output writer.
pub fn serve(root: &Path) -> io::Result<()> {
    #[derive(Deserialize)]
    struct ProcessRecord {
        pid: u32,
        source: String,
        stream: String,
        line: String,
    }
    let start = SystemTime::now();
    let run = Run::create_at(root, "python")?;
    println!("{}", run.path().display());
    io::stdout().flush()?;
    let mut outputs = BTreeMap::new();
    let mut pids = BTreeSet::new();
    for input in io::stdin().lock().lines() {
        let row: ProcessRecord = serde_json::from_str(&input?)?;
        pids.insert(row.pid);
        let stream = match row.stream.as_str() {
            "stdout" => "stdout",
            "stderr" => "stderr",
            "started" => continue,
            "gap" => {
                run.gap(&row.source, &row.line)?;
                continue;
            }
            _ => return Err(io::Error::other("invalid process output stream")),
        };
        let output = match outputs.entry(row.source) {
            std::collections::btree_map::Entry::Occupied(entry) => entry.into_mut(),
            std::collections::btree_map::Entry::Vacant(entry) => {
                let writer = Output::new(run.clone(), entry.key())?;
                entry.insert(writer)
            }
        };
        output.line(row.pid, stream, row.line.as_bytes());
    }
    let mut result = Ok(());
    for output in outputs.into_values() {
        result = result.and(output.finish());
    }
    if let Err(error) = firefly_archive::host::system(&run, start, &pids) {
        eprintln!("host system log: {error}");
        run.gap("system-host", &error.to_string())?;
    }
    result
}
