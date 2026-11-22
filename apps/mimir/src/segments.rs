use std::collections::BTreeMap;
use std::fs::File;
use std::io::{self, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use firefly_archive::{Run, Segment};
use firefly_error::Result;
use re_chunk::{Chunk, ComponentIdentifier, RowId};
use re_log_encoding::{DecoderApp, EncodeError, Encoder, EncodingOptions};
use re_log_msg::{LogMsg, SetStoreInfo, StoreInfo, StoreSource};
use re_log_types::{StoreId, StoreKind};

use crate::failure;

const MAX_CHUNKS: usize = 4096;
const INITIAL_FOOTER_BYTES: u64 = 256 * 1024;
const FOOTER_BYTES_PER_MESSAGE: u64 = 1024;
const RETRY_PERIOD: Duration = Duration::from_secs(1);

/// The encoder submits each data message in a single write. Keep its projected
/// footer credit available until `finish()` starts submitting the footer itself.
struct Writer {
    segment: Segment,
    footer_bytes: u64,
    closing: Arc<AtomicBool>,
    damaged: bool,
}

impl Write for Writer {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if !self.closing.load(Ordering::Relaxed) {
            let footer = self.footer_bytes + FOOTER_BYTES_PER_MESSAGE;
            self.segment.reserve(footer + buf.len() as u64)?;
            self.footer_bytes = footer;
        }
        if let Err(error) = self.segment.write_all(buf) {
            self.damaged = true;
            return Err(error);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        self.segment.flush()
    }
}

type Baselines = BTreeMap<(String, bool, Vec<ComponentIdentifier>), LogMsg>;

struct Active {
    encoder: Encoder<Writer>,
    closing: Arc<AtomicBool>,
    bytes: u64,
    payload_chunks: usize,
}

impl Active {
    fn open(run: &Run, store: &StoreId, baselines: &Baselines) -> io::Result<Self> {
        let mut segment = run.rrd_segment()?;
        if let Err(error) = segment.reserve(INITIAL_FOOTER_BYTES + 4096) {
            segment.discard()?;
            return Err(error);
        }
        let closing = Arc::new(AtomicBool::new(false));
        let writer = Writer {
            segment,
            footer_bytes: INITIAL_FOOTER_BYTES,
            closing: Arc::clone(&closing),
            damaged: false,
        };
        let encoder = Encoder::new_eager(
            re_build_info::CrateVersion::LOCAL,
            EncodingOptions::PROTOBUF_COMPRESSED,
            writer,
        )
        .map_err(encoding_error)?;
        let mut active = Self {
            encoder,
            closing,
            bytes: 0,
            payload_chunks: 0,
        };
        if let Err(error) = active.header(store, baselines) {
            active
                .encoder
                .into_inner()
                .map_err(encoding_error)?
                .segment
                .discard()?;
            return Err(error);
        }
        Ok(active)
    }

    fn header(&mut self, store: &StoreId, baselines: &Baselines) -> io::Result<()> {
        for msg in
            DecoderApp::decode_lazy(include_bytes!("../../../configs/firefly.rbl").as_slice())
        {
            let mut msg = msg.map_err(io::Error::other)?;
            if let LogMsg::BlueprintActivationCommand(command) = &mut msg {
                command.make_active = false;
                command.make_default = true;
            }
            self.bytes += self.encoder.append(&msg).map_err(encoding_error)?;
        }
        self.bytes += self
            .encoder
            .append(&LogMsg::SetStoreInfo(SetStoreInfo {
                row_id: *RowId::new(),
                info: StoreInfo::new(store.clone(), StoreSource::Unknown),
            }))
            .map_err(encoding_error)?;
        for msg in baselines.values() {
            self.bytes += self.encoder.append(msg).map_err(encoding_error)?;
        }
        Ok(())
    }

    fn seal(mut self) -> io::Result<()> {
        self.closing.store(true, Ordering::Relaxed);
        self.encoder.finish().map_err(encoding_error)?;
        self.encoder.flush_blocking().map_err(encoding_error)?;
        let writer = self.encoder.into_inner().map_err(encoding_error)?;
        if writer.damaged {
            return Err(io::Error::other("partial RRD message write"));
        }
        writer.segment.seal()
    }
}

fn encoding_error(error: EncodeError) -> io::Error {
    match error {
        EncodeError::Write(error) => error,
        other => io::Error::other(other),
    }
}

pub(crate) struct Segments {
    run: Run,
    _recorder: File,
    store: StoreId,
    active: Option<Active>,
    baselines: Baselines,
    retry_at: Option<Instant>,
    lost_chunks: u64,
}

impl Segments {
    pub fn new(run: Run) -> Result<Self> {
        let recorder = File::options()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(run.path().join(".rrd.lock"))
            .map_err(|e| failure("open recorder lease", e))?;
        recorder
            .try_lock()
            .map_err(|e| failure("acquire exclusive recorder lease", e))?;
        run.recorder_started()
            .map_err(|e| failure("register recorder session", e))?;
        Ok(Self {
            run,
            _recorder: recorder,
            store: StoreId::random(StoreKind::Recording, "firefly"),
            active: None,
            baselines: BTreeMap::new(),
            retry_at: None,
            lost_chunks: 0,
        })
    }

    pub fn run(&self) -> &Run {
        &self.run
    }

    pub fn lost_data(&self) -> bool {
        self.lost_chunks != 0
    }

    pub fn clear(&mut self, name: &str) {
        let prefix = format!("{name}/");
        self.baselines.retain(|(entity, is_static, _), _| {
            *is_static || !(entity == name || entity.starts_with(&prefix))
        });
    }

    pub fn append(&mut self, chunk: &Chunk, baseline: Option<&str>) -> Result<()> {
        self.append_chunk(chunk, baseline, true)
    }

    /// Keep the baseline chunks from one static snapshot in the same segment.
    pub fn append_snapshot(&mut self, chunks: &[(&str, Chunk)]) -> Result<()> {
        for (index, (name, chunk)) in chunks.iter().enumerate() {
            self.append_chunk(chunk, Some(name), index == 0)?;
        }
        Ok(())
    }

    fn append_chunk(
        &mut self,
        chunk: &Chunk,
        baseline: Option<&str>,
        allow_rotation: bool,
    ) -> Result<()> {
        let msg = LogMsg::ArrowMsg(
            self.store.clone(),
            chunk
                .to_arrow_msg()
                .map_err(|e| failure("serialize chunk", e))?,
        );
        if self.retry_at.is_none_or(|retry| Instant::now() >= retry) {
            match self.write(&msg, allow_rotation) {
                Ok(()) => self.retry_at = None,
                Err(error) if error.kind() == io::ErrorKind::StorageFull => {
                    if self.retry_at.is_none() {
                        self.gap("RRD writes suspended: disk capacity exhausted");
                    }
                    self.retry_at = Some(Instant::now() + RETRY_PERIOD);
                    if let Some(active) = self.active.take()
                        && let Err(error) = active.seal()
                    {
                        self.gap(&format!("RRD segment interrupted: {error}"));
                    }
                }
                Err(error) => {
                    self.gap(&format!("RRD write failed: {error}"));
                    return Err(failure("write RRD", error));
                }
            }
        }
        if self.retry_at.is_some() {
            self.lost_chunks += 1;
        }
        if let Some(name) = baseline {
            let mut components = chunk.components_identifiers().collect::<Vec<_>>();
            components.sort_unstable();
            self.baselines
                .insert((name.to_owned(), chunk.is_static(), components), msg);
        }
        Ok(())
    }

    fn write(&mut self, msg: &LogMsg, allow_rotation: bool) -> io::Result<()> {
        if allow_rotation
            && self.active.as_ref().is_some_and(|active| {
                active.payload_chunks > 0
                    && (active.bytes >= self.run.rrd_target()
                        || active.payload_chunks >= MAX_CHUNKS)
            })
            && let Some(active) = self.active.take()
        {
            active.seal()?;
        }
        let active = match &mut self.active {
            Some(active) => active,
            slot @ None => slot.insert(Active::open(&self.run, &self.store, &self.baselines)?),
        };
        active.bytes += active.encoder.append(msg).map_err(encoding_error)?;
        active.payload_chunks += 1;
        if self.retry_at.is_some() {
            self.gap(&format!(
                "RRD writes resumed; {} chunks missed in this recording",
                self.lost_chunks
            ));
        }
        Ok(())
    }

    pub fn gap(&self, detail: &str) {
        eprintln!("{detail}");
        if let Err(error) = self.run.gap("rrd", detail) {
            eprintln!("could not persist RRD gap: {error}");
        }
    }

    pub fn flush(&mut self) -> Result<()> {
        if let Some(active) = &mut self.active {
            active
                .encoder
                .flush_blocking()
                .map_err(|e| failure("flush RRD", e))?;
        }
        Ok(())
    }

    pub fn finish(mut self) -> Result<()> {
        if let Some(active) = self.active.take() {
            active
                .seal()
                .inspect_err(|error| self.gap(&format!("RRD seal failed: {error}")))
                .map_err(|e| failure("close RRD", e))?;
        }
        if self.lost_chunks > 0 {
            let message = format!("{} RRD chunks lost to disk pressure", self.lost_chunks);
            self.gap(&message);
            return Err(failure("finalize recording", message));
        }
        Ok(())
    }
}
