use std::collections::BTreeSet;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Read, Write};
use std::os::fd::AsFd as _;
use std::thread::{self, JoinHandle};
use std::time::SystemTime;

use firefly_archive::{Run, output::Output};
use firefly_error::Result;

#[derive(Clone, Copy)]
enum Descriptor {
    Stdout,
    Stderr,
}

impl Descriptor {
    fn label(self) -> &'static str {
        match self {
            Self::Stdout => "stdout",
            Self::Stderr => "stderr",
        }
    }

    fn save(self) -> io::Result<File> {
        match self {
            Self::Stdout => io::stdout().as_fd().try_clone_to_owned(),
            Self::Stderr => io::stderr().as_fd().try_clone_to_owned(),
        }
        .map(Into::into)
    }

    fn redirect(self, destination: impl std::os::fd::AsFd) -> io::Result<()> {
        match self {
            Self::Stdout => nix::unistd::dup2_stdout(destination),
            Self::Stderr => nix::unistd::dup2_stderr(destination),
        }
        .map_err(Into::into)
    }
}

/// A standalone Mimir has no launcher's pipe. Tee its descriptor back to the
/// original terminal and through the same bounded output writer as its peers.
struct Tee {
    descriptor: Descriptor,
    saved: File,
    worker: Option<JoinHandle<io::Result<()>>>,
}

impl Tee {
    fn start(descriptor: Descriptor, output: Output) -> io::Result<Self> {
        let saved = descriptor.save()?;
        let mut terminal = saved.try_clone()?;
        let (reader, writer) = io::pipe()?;
        let worker = thread::Builder::new()
            .name(format!("mimir-{}", descriptor.label()))
            .spawn(move || {
                let mut reader = BufReader::new(reader);
                let mut row = Vec::new();
                loop {
                    row.clear();
                    if (&mut reader).take(4096).read_until(b'\n', &mut row)? == 0 {
                        return Ok(());
                    }
                    output.line(std::process::id(), descriptor.label(), &row);
                    terminal.write_all(&row)?;
                }
            })?;
        descriptor.redirect(writer)?;
        Ok(Self {
            descriptor,
            saved,
            worker: Some(worker),
        })
    }

    fn finish(&mut self) -> io::Result<()> {
        if let Some(worker) = self.worker.take() {
            self.descriptor.redirect(&self.saved)?;
            worker
                .join()
                .map_err(|_| io::Error::other("console tee panicked"))??;
        }
        Ok(())
    }
}

impl Drop for Tee {
    fn drop(&mut self) {
        let _ = self.finish();
    }
}

pub fn capture(run: &Run, record: impl FnOnce() -> Result<()>) -> Result<()> {
    let start = SystemTime::now();
    let output = Output::new(run.clone(), "mimir")?;
    let mut stdout = Tee::start(Descriptor::Stdout, output.clone())?;
    let mut stderr = Tee::start(Descriptor::Stderr, output.clone())?;
    let result = record();
    if let Err(error) = &result {
        output.line(std::process::id(), "stderr", error.to_string().as_bytes());
    }
    io::stdout().flush()?;
    stdout.finish()?;
    stderr.finish()?;
    let text = output.finish().map_err(Into::into);
    if !cfg!(all(target_os = "linux", target_arch = "aarch64"))
        && let Err(error) =
            firefly_archive::host::system(run, start, &BTreeSet::from([std::process::id()]))
    {
        eprintln!("system journal: {error}");
        run.gap("system.host", &error.to_string())?;
    }
    result.and(text)
}
