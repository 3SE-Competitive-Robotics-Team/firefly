mod boot;
mod bus;
mod events;
mod flight;
mod store;
#[cfg(target_os = "linux")]
mod watchdog;

use bus::{Bus, speed};
use firefly_flight::FlightState as Mode;
use firefly_mission::{Phase, Request, read_frame, valid_id, write_frame};
use flight::{Flight, Mission, Receipt};
use serde_json::{Value, json};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::Instant;
use std::{
    fs::{self, File},
    io,
    os::unix::{
        fs::PermissionsExt,
        net::{UnixListener, UnixStream},
    },
    sync::mpsc::{self, Receiver, SyncSender},
    thread,
    time::Duration,
};
use store::{Commit, CommitState, Store};

type Reply = Result<Value, String>;
type Query = (Request, SyncSender<Reply>);

fn overview(r: &Mission, phase: Phase) -> Value {
    json!({"run":r.run,"id":r.id,"phase":phase,"session":r.session,"waypoint":r.waypoint,
        "home":r.home,"flight_number":r.flight_number,"reason":r.reason,"attention":r.attention})
}

struct RunAnnouncement {
    recovered: Option<Receipt>,
}

struct Daemon {
    run: String,
    journal: store::Journal,
    receipts: std::collections::HashMap<String, Receipt>,
    session: Option<u64>,
    flight: Option<Flight>,
    bus: Bus,
    team: u32,
    last_flight: u64,
}
impl Daemon {
    fn new(store: Store) -> Result<Self, Box<dyn std::error::Error>> {
        let store::Recovered {
            session,
            last_flight,
            active,
            mut receipts,
        } = store.recover()?;
        let flight = active.map(Flight::resume);
        if let Some(f) = &flight {
            let receipt = f.receipt();
            store.save(&receipt)?;
            receipts.insert(f.mission.id.clone(), receipt);
        }
        Ok(Self {
            run: store.run.clone(),
            journal: store::Journal::start(store)?,
            receipts,
            session,
            last_flight,
            flight,
            bus: Bus::open()?,
            team: firefly_config::game::load()?.team.code(),
        })
    }
    fn ground(&self) -> Result<(firefly_pubsub::odom::OdomMessage, u64), String> {
        if self.bus.airframe.within(Duration::from_secs(3)).is_none() {
            return Err("waiting for actuator airframe data".into());
        }
        let (pose, session, mode) = self
            .bus
            .navigation()
            .ok_or("waiting for fresh FC, attitude and initialized odometry")?;
        if mode != Mode::Disarmed {
            return Err(format!(
                "waiting for FC DISARMED, currently {}",
                mode.name()
            ));
        }
        let velocity = speed(pose);
        if self.bus.fc.fresh().is_none_or(|fc| fc.can_arm != 1) {
            return Err("waiting for FC arming checks".into());
        }
        if velocity > 0.12 {
            return Err(format!(
                "waiting for stationary odometry, speed {velocity:.3} m/s exceeds 0.12 m/s"
            ));
        }
        if self.session.is_some_and(|s| s != session) {
            return Err("VIO session changed during system run".into());
        }
        Ok((pose, session))
    }
    fn request(&mut self, request: Request) -> io::Result<Reply> {
        let id = match &request {
            Request::Status { id: None } => {
                let ground = self.ground();
                let mode = self
                    .bus
                    .fc
                    .fresh()
                    .and_then(|fc| Mode::from_code(fc.mode))
                    .map(Mode::name);
                return Ok(Ok(
                    json!({"run":self.run,"ready":ground.is_ok() && self.flight.is_none(),"waiting":ground.err(),
                "session":self.session,"fc_mode":mode,"planner_ready":self.bus.heartbeat.fresh().is_some(),
                "active":self.flight.as_ref().map(|f|overview(&f.mission, f.phase()))}),
                ));
            }
            Request::Submit { id, .. }
            | Request::Cancel { id }
            | Request::Status { id: Some(id) } => id,
        };
        if !valid_id(id) {
            return Ok(Err("invalid mission id".into()));
        }
        let existing = self.receipts.get(id);
        match request {
            Request::Status { .. } => Ok(existing
                .map(|r| overview(&r.mission, r.phase))
                .ok_or_else(|| "unknown mission id".into())),
            Request::Cancel { id } => {
                if let Some(f) = self.flight.as_mut().filter(|f| f.mission.id == id) {
                    if !matches!(f.phase(), Phase::Recover | Phase::NeedsOperator) {
                        f.interrupt("operator cancelled");
                        self.journal.submit(
                            f.receipt(),
                            Commit::PhaseChanged(self.bus.odom.last_stamp()),
                        )?;
                    }
                    Ok(Ok(overview(&f.mission, f.phase())))
                } else {
                    Ok(existing
                        .map(|r| overview(&r.mission, r.phase))
                        .ok_or_else(|| "unknown mission id".into()))
                }
            }
            Request::Submit { id, plan } => {
                if let Some(r) = existing {
                    return Ok(if r.mission.plan == plan {
                        Ok(overview(&r.mission, r.phase))
                    } else {
                        Err("mission id already has a different plan".into())
                    });
                }
                if self.flight.is_some() {
                    return Ok(Err("another mission is active".into()));
                }
                if let Err(e) = plan.validate() {
                    return Ok(Err(e.into()));
                }
                if !plan.waypoints.is_empty() && self.bus.heartbeat.fresh().is_none() {
                    return Ok(Err("waiting for planner heartbeat".into()));
                }
                let (p, session) = match self.ground() {
                    Ok(v) => v,
                    Err(e) => return Ok(Err(e)),
                };
                let f = Flight::new(Mission {
                    id,
                    plan,
                    session,
                    run: self.run.clone(),
                    home: [p.position_x, p.position_y, p.position_z],
                    home_quaternion: [p.quat_x, p.quat_y, p.quat_z, p.quat_w],
                    waypoint: 0,
                    flight_number: None,
                    reason: None,
                    attention: None,
                });
                self.journal
                    .submit(f.receipt(), Commit::Accepted(p.timestamp))?;
                self.session = Some(session);
                let reply = overview(&f.mission, f.phase());
                self.flight = Some(f);
                Ok(Ok(reply))
            }
        }
    }
    fn tick(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let Some(f) = &mut self.flight else {
            return Ok(());
        };
        if f.mission.flight_number.is_none()
            && let Some((pose, session, mode)) = self.bus.navigation()
            && session == f.mission.session
            && matches!(mode, Mode::Takeoff | Mode::Hold | Mode::Track | Mode::Land)
            && pose.position_z - f.mission.home[2] > 0.15
        {
            self.last_flight += 1;
            f.mission.flight_number = Some(self.last_flight);
            self.journal
                .submit(f.receipt(), Commit::Airborne(pose.timestamp))?;
            return Ok(());
        }
        let before = (f.phase(), f.mission.waypoint);
        let lease = self.bus.landing_active();
        f.tick(&mut self.bus, self.team)?;
        if lease != self.bus.landing_active() {
            events::record(
                &self.bus,
                &f.mission,
                json!({"kind":"activity","enabled":!lease}),
            );
        }
        if before != (f.phase(), f.mission.waypoint) {
            self.journal.submit(
                f.receipt(),
                Commit::PhaseChanged(self.bus.odom.last_stamp()),
            )?;
        }
        if f.phase().terminal() {
            self.bus.lease(false)?;
            self.flight = None;
        }
        Ok(())
    }

    fn serve(
        &mut self,
        requests: &Receiver<Query>,
        stopping: &AtomicBool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        #[cfg(target_os = "linux")]
        let mut watchdog = watchdog::Watchdog::ready()?;
        let mut pending_reply: Option<(SyncSender<Reply>, Reply)> = None;
        let mut announcement = Some(RunAnnouncement {
            recovered: self.flight.as_ref().map(Flight::receipt),
        });
        while !stopping.load(Ordering::Relaxed) {
            self.bus.pump()?;
            if self.bus.odom.last_stamp() >= 0.0
                && let Some(start) = announcement.take()
            {
                let event = json!({"kind":"system_run","run":self.run});
                self.bus.log.event(
                    "flight",
                    self.bus.odom.last_stamp(),
                    &format!("mission {event}"),
                );
                if let Some(receipt) = start.recovered {
                    events::phase(&self.bus, &receipt, self.bus.odom.last_stamp());
                }
            }
            let mut commits = self.collect_commits()?;
            if commits == CommitState::Idle
                && let Some((reply, value)) = pending_reply.take()
            {
                let _ = reply.send(value);
            }
            if commits == CommitState::Idle
                && let Ok((request, reply)) = requests.try_recv()
            {
                let value = self.request(request)?;
                commits = self.journal.state();
                if commits == CommitState::Idle {
                    let _ = reply.send(value);
                } else {
                    pending_reply = Some((reply, value));
                }
            }
            if pending_reply.is_none() && commits != CommitState::WritingIdentity {
                self.tick()?;
            }
            #[cfg(target_os = "linux")]
            if let Some(watchdog) = &mut watchdog {
                watchdog.completed_iteration()?;
            }
            thread::sleep(Duration::from_millis(20));
        }
        self.stop()
    }

    fn collect_commits(&mut self) -> io::Result<CommitState> {
        while let Some((receipt, commit)) = self.journal.poll()? {
            match commit {
                Commit::Accepted(stamp) => events::start(&self.bus, &receipt, self.team, stamp),
                Commit::Airborne(stamp) => events::record_at(
                    &self.bus,
                    &receipt.mission,
                    stamp,
                    json!({"kind":"airborne"}),
                ),
                Commit::PhaseChanged(stamp) => events::phase(&self.bus, &receipt, stamp),
            }
            self.receipts.insert(receipt.mission.id.clone(), receipt);
        }
        Ok(self.journal.state())
    }

    fn stop(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(f) = &mut self.flight {
            if !matches!(f.phase(), Phase::Recover | Phase::NeedsOperator) {
                f.interrupt("service stopping");
                self.journal.submit(
                    f.receipt(),
                    Commit::PhaseChanged(self.bus.odom.last_stamp()),
                )?;
            }
            let deadline = Instant::now() + Duration::from_secs(2);
            while Instant::now() < deadline && self.flight.is_some() {
                self.bus.pump()?;
                self.collect_commits()?;
                self.tick()?;
                if self
                    .bus
                    .fc
                    .fresh()
                    .is_some_and(|fc| fc.mode == Mode::Land.code())
                {
                    break;
                }
                thread::sleep(Duration::from_millis(20));
            }
        }
        while self.collect_commits()? != CommitState::Idle {
            thread::sleep(Duration::from_millis(5));
        }
        Ok(())
    }
}

fn client(stream: &UnixStream, tx: &SyncSender<Query>) -> io::Result<()> {
    let timeout = Some(Duration::from_secs(2));
    stream.set_read_timeout(timeout)?;
    stream.set_write_timeout(timeout)?;
    let reply = (|| {
        let request = read_frame(stream).map_err(|e| e.to_string())?;
        let (send, recv) = mpsc::sync_channel(1);
        tx.try_send((request, send)).map_err(|e| e.to_string())?;
        recv.recv_timeout(Duration::from_secs(4))
            .map_err(|e| e.to_string())?
    })();
    write_frame(stream, &reply)
}

/// 管理机载任务服务
#[derive(usage_rs::Cli)]
#[usage(bin = "firefly-daemon")]
struct Arguments {}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    Arguments::parse();
    iceoryx2::prelude::set_log_level(iceoryx2::prelude::LogLevel::Error);
    let dir = firefly_mission::runtime_dir();
    fs::create_dir_all(&dir)?;
    let lock = File::options()
        .create(true)
        .truncate(false)
        .write(true)
        .open(dir.join("mission.lock"))?;
    lock.try_lock()?;
    let missions = firefly_config::paths::root().join("state/missions");
    let mut daemon = Daemon::new(Store::open(&missions, boot::run_id()?)?)?;
    let path = firefly_mission::socket_path();
    match fs::remove_file(&path) {
        Ok(()) => (),
        Err(e) if e.kind() == io::ErrorKind::NotFound => (),
        Err(e) => return Err(e.into()),
    }
    let mask = nix::sys::stat::umask(nix::sys::stat::Mode::from_bits_truncate(0o077));
    let listener = UnixListener::bind(&path);
    nix::sys::stat::umask(mask);
    let listener = listener?;
    fs::set_permissions(&path, fs::Permissions::from_mode(0o600))?;
    let stopping = Arc::new(AtomicBool::new(false));
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        signal_hook::flag::register(signal, Arc::clone(&stopping))?;
    }
    let (tx, rx) = mpsc::sync_channel(8);
    // Each worker serves at most one local client. A stalled socket cannot block flight control.
    for _ in 0..4 {
        let listener = listener.try_clone()?;
        let tx = tx.clone();
        thread::spawn(move || {
            for stream in listener.incoming().flatten() {
                let _ = client(&stream, &tx);
            }
        });
    }
    let result = daemon.serve(&rx, &stopping);
    fs::remove_file(path)?;
    result
}
