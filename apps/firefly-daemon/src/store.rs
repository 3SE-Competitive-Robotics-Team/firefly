use crate::flight::Receipt;
use std::{
    collections::{HashMap, VecDeque},
    fs::{self, File},
    io::{self, Write},
    path::{Path, PathBuf},
    sync::mpsc::{self, Receiver, SyncSender, TryRecvError},
    time::{Duration, Instant},
};

pub struct Store {
    dir: PathBuf,
    pub run: String,
}
impl Store {
    pub fn open() -> io::Result<Self> {
        #[cfg(target_os = "linux")]
        let run = fs::read_to_string("/proc/sys/kernel/random/boot_id")?;
        #[cfg(not(target_os = "linux"))]
        let run = String::from_utf8(
            std::process::Command::new("sysctl")
                .args(["-n", "kern.bootsessionuuid"])
                .output()?
                .stdout,
        )
        .map_err(io::Error::other)?;
        let run = run.trim().to_owned();
        if !firefly_mission::valid_id(&run) {
            return Err(io::Error::other("missing OS boot identity"));
        }
        let dir = firefly_config::paths::root()
            .join("state/missions")
            .join(&run);
        fs::create_dir_all(&dir)?;
        Ok(Self { dir, run })
    }
    pub fn save(&self, r: &Receipt) -> io::Result<()> {
        let tmp = self.dir.join(".pending");
        let mut file = File::create(&tmp)?;
        serde_json::to_writer(&mut file, r)?;
        file.flush()?;
        file.sync_all()?;
        fs::rename(tmp, self.dir.join(format!("{}.json", r.id)))?;
        File::open(&self.dir)?.sync_all()
    }
    pub fn recover(&self) -> io::Result<Recovered> {
        let mut receipts = HashMap::new();
        let mut session = None;
        let mut last_flight = 0;
        let mut active = None;
        for entry in fs::read_dir(&self.dir)? {
            let path = entry?.path();
            if path.extension() != Some(std::ffi::OsStr::new("json")) {
                continue;
            }
            let r: Receipt = serde_json::from_reader(File::open(&path)?)?;
            if !firefly_mission::valid_id(&r.id)
                || path.file_stem() != Some(Path::new(&r.id).as_os_str())
                || r.run != self.run
                || session.is_some_and(|s| s != r.session)
            {
                return Err(io::Error::other("inconsistent mission receipt"));
            }
            session = Some(r.session);
            last_flight = last_flight.max(r.flight_number.unwrap_or(0));
            if !r.phase.terminal() && active.replace(r.clone()).is_some() {
                return Err(io::Error::other("multiple active missions"));
            }
            receipts.insert(r.id.clone(), r);
        }
        Ok(Recovered {
            session,
            last_flight,
            active,
            receipts,
        })
    }
}

pub struct Recovered {
    pub session: Option<u64>,
    pub last_flight: u64,
    pub active: Option<Receipt>,
    pub receipts: HashMap<String, Receipt>,
}

/// One FIFO writer preserves transition order. The oldest outstanding commit bounds I/O latency.
pub struct Journal {
    tx: SyncSender<Receipt>,
    rx: Receiver<io::Result<Receipt>>,
    pending: VecDeque<Instant>,
}

impl Journal {
    pub fn start(store: Store) -> io::Result<Self> {
        let (tx, jobs) = mpsc::sync_channel(8);
        let (done, rx) = mpsc::sync_channel(8);
        std::thread::Builder::new()
            .name("mission-journal".into())
            .spawn(move || {
                for receipt in jobs {
                    let result = store.save(&receipt).map(|()| receipt);
                    let failed = result.is_err();
                    if done.send(result).is_err() || failed {
                        break;
                    }
                }
            })?;
        Ok(Self {
            tx,
            rx,
            pending: VecDeque::new(),
        })
    }
    pub fn submit(&mut self, receipt: &Receipt) -> io::Result<()> {
        if self.pending.len() >= 8 {
            return Err(io::Error::other("mission journal queue is full"));
        }
        self.tx
            .try_send(receipt.clone())
            .map_err(io::Error::other)?;
        self.pending.push_back(Instant::now());
        Ok(())
    }
    pub fn pending(&self) -> bool {
        !self.pending.is_empty()
    }
    pub fn poll(&mut self) -> io::Result<Option<Receipt>> {
        let Some(started) = self.pending.front() else {
            return Ok(None);
        };
        match self.rx.try_recv() {
            Ok(result) => {
                self.pending.pop_front();
                result.map(Some)
            }
            Err(TryRecvError::Empty) if started.elapsed() < Duration::from_secs(3) => Ok(None),
            Err(TryRecvError::Empty) => Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "mission journal failed to commit within 3 s",
            )),
            Err(error) => Err(io::Error::other(error)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flight::Phase;
    use firefly_mission::Plan;

    #[test]
    fn queued_receipts_commit_in_order_and_recover_the_last_transition() {
        let dir = tempfile::tempdir_in(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target"))
            .unwrap();
        let store = || Store {
            dir: dir.path().to_owned(),
            run: "boot".into(),
        };
        let mut journal = Journal::start(store()).unwrap();
        let mut receipt = Receipt {
            id: "survey".into(),
            run: "boot".into(),
            session: 1,
            flight_number: Some(1),
            plan: Plan {
                altitude: 1.,
                hover_seconds: 2.,
                return_offset: [0.; 2],
                precision_land: false,
                waypoints: vec![],
            },
            phase: Phase::Navigate,
            waypoint: 0,
            home: [0.; 3],
            home_quaternion: [0., 0., 0., 1.],
            reason: None,
            attention: None,
        };
        for index in 0..8 {
            receipt.waypoint = index;
            journal.submit(&receipt).unwrap();
        }
        assert!(journal.submit(&receipt).is_err());
        let mut observed = Vec::new();
        while journal.pending() {
            if let Some(committed) = journal.poll().unwrap() {
                observed.push(committed.waypoint);
            } else {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        assert_eq!(observed, (0..8).collect::<Vec<_>>());
        assert_eq!(store().recover().unwrap().active.unwrap().waypoint, 7);
    }
}
