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
    pub fn open(missions: &Path, run: String) -> io::Result<Self> {
        if !firefly_mission::valid_id(&run) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "invalid system run ID",
            ));
        }
        let dir = missions.join(&run);
        fs::create_dir_all(&dir)?;
        Ok(Self { dir, run })
    }
    pub fn save(&self, r: &Receipt) -> io::Result<()> {
        let tmp = self.dir.join(".pending");
        let mut file = File::create(&tmp)?;
        serde_json::to_writer(&mut file, r)?;
        file.flush()?;
        file.sync_all()?;
        fs::rename(tmp, self.dir.join(format!("{}.json", r.mission.id)))?;
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
            if !firefly_mission::valid_id(&r.mission.id)
                || path.file_stem() != Some(Path::new(&r.mission.id).as_os_str())
                || r.mission.run != self.run
                || session.is_some_and(|s| s != r.mission.session)
            {
                return Err(io::Error::other("inconsistent mission receipt"));
            }
            session = Some(r.mission.session);
            last_flight = last_flight.max(r.mission.flight_number.unwrap_or(0));
            if !r.phase.terminal() && active.replace(r.clone()).is_some() {
                return Err(io::Error::other("multiple active missions"));
            }
            receipts.insert(r.mission.id.clone(), r);
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

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Commit {
    Accepted(f64),
    Airborne(f64),
    PhaseChanged(f64),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommitState {
    Idle,
    WritingPhase,
    WritingIdentity,
}

/// One FIFO writer preserves transition order. The oldest outstanding commit bounds I/O latency.
pub struct Journal {
    tx: SyncSender<Receipt>,
    rx: Receiver<io::Result<Receipt>>,
    pending: VecDeque<(Instant, Commit)>,
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
    pub fn submit(&mut self, receipt: Receipt, commit: Commit) -> io::Result<()> {
        if self.pending.len() >= 8 {
            return Err(io::Error::other("mission journal queue is full"));
        }
        self.tx.try_send(receipt).map_err(io::Error::other)?;
        self.pending.push_back((Instant::now(), commit));
        Ok(())
    }
    pub fn state(&self) -> CommitState {
        if self.pending.is_empty() {
            CommitState::Idle
        } else if self
            .pending
            .iter()
            .any(|(_, commit)| matches!(commit, Commit::Accepted(_) | Commit::Airborne(_)))
        {
            CommitState::WritingIdentity
        } else {
            CommitState::WritingPhase
        }
    }
    pub fn poll(&mut self) -> io::Result<Option<(Receipt, Commit)>> {
        let Some(&(started, commit)) = self.pending.front() else {
            return Ok(None);
        };
        match self.rx.try_recv() {
            Ok(result) => {
                self.pending.pop_front();
                result.map(|receipt| Some((receipt, commit)))
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
    use crate::flight::Mission;
    use firefly_mission::Phase;
    use firefly_mission::Plan;

    #[test]
    fn queued_receipts_commit_in_order_and_recover_the_last_transition() {
        let dir =
            tempfile::tempdir_in(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../target"))
                .unwrap();
        let store = || Store::open(dir.path(), "boot".into()).unwrap();
        assert!(Store::open(dir.path(), "../escape".into()).is_err());
        let mut journal = Journal::start(store()).unwrap();
        let mut receipt = Receipt {
            phase: Phase::Navigate,
            mission: Mission {
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
                waypoint: 0,
                home: [0.; 3],
                home_quaternion: [0., 0., 0., 1.],
                reason: None,
                attention: None,
            },
        };
        for index in 0..8 {
            receipt.mission.waypoint = index;
            let commit = if index == 3 {
                Commit::Airborne(1.5)
            } else {
                Commit::PhaseChanged(index as f64)
            };
            journal.submit(receipt.clone(), commit).unwrap();
        }
        assert!(journal.submit(receipt, Commit::PhaseChanged(9.)).is_err());
        let mut observed = Vec::new();
        while journal.state() != CommitState::Idle {
            if let Some((committed, reason)) = journal.poll().unwrap() {
                let index = committed.mission.waypoint;
                assert_eq!(
                    reason,
                    if index == 3 {
                        Commit::Airborne(1.5)
                    } else {
                        Commit::PhaseChanged(index as f64)
                    }
                );
                assert_eq!(journal.state() == CommitState::WritingIdentity, index < 3);
                observed.push(index);
            } else {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        assert_eq!(observed, (0..8).collect::<Vec<_>>());
        let next_boot = Store::open(dir.path(), "next-boot".into()).unwrap();
        assert!(next_boot.recover().unwrap().receipts.is_empty());
        assert_eq!(
            store().recover().unwrap().active.unwrap().mission.waypoint,
            7
        );
    }
}
