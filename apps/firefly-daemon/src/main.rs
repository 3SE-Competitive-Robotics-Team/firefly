mod bus;
mod events;
mod flight;
mod store;

use bus::{Bus, speed};
use firefly_flight::FlightState as Mode;
use firefly_mission::{Request, read_frame, valid_id, write_frame};
use flight::{Flight, Phase, Receipt};
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
use store::Store;

type Reply = Result<Value, String>;
type Query = (Request, SyncSender<Reply>);

fn overview(r: &Receipt) -> Value {
    json!({"run":r.run,"id":r.id,"phase":r.phase,"session":r.session,"waypoint":r.waypoint,
        "home":r.home,"flight_number":r.flight_number,"reason":r.reason,"attention":r.attention})
}

struct Daemon {
    store: Store,
    session: Option<u64>,
    flight: Option<Flight>,
    bus: Bus,
    team: u32,
    last_flight: u64,
}
impl Daemon {
    fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let store = Store::open()?;
        let (session, last_flight, active) = store.recover()?;
        let mut flight = active.map(Flight::new);
        if let Some(f) = &mut flight {
            f.interrupt("daemon restarted during mission");
            store.save(&f.record)?;
        }
        Ok(Self {
            store,
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
                    json!({"run":self.store.run,"ready":ground.is_ok() && self.flight.is_none(),"waiting":ground.err(),
                "session":self.session,"fc_mode":mode,"active":self.flight.as_ref().map(|f|overview(&f.record))}),
                ));
            }
            Request::Submit { id, .. }
            | Request::Cancel { id }
            | Request::Status { id: Some(id) } => id,
        };
        if !valid_id(id) {
            return Ok(Err("invalid mission id".into()));
        }
        let existing = self.store.load(id)?;
        match request {
            Request::Status { .. } => Ok(existing
                .as_ref()
                .map(overview)
                .ok_or_else(|| "unknown mission id".into())),
            Request::Cancel { id } => {
                if let Some(f) = self.flight.as_mut().filter(|f| f.record.id == id) {
                    if f.record.phase != Phase::Recover {
                        f.interrupt("operator cancelled");
                        self.store.save(&f.record)?;
                        events::phase(&self.bus, f);
                    }
                    Ok(Ok(overview(&f.record)))
                } else {
                    Ok(existing
                        .as_ref()
                        .map(overview)
                        .ok_or_else(|| "unknown mission id".into()))
                }
            }
            Request::Submit { id, plan } => {
                if let Some(r) = existing {
                    return Ok(if r.plan == plan {
                        Ok(overview(&r))
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
                let f = Flight::new(Receipt {
                    id,
                    plan,
                    session,
                    run: self.store.run.clone(),
                    home: [p.position_x, p.position_y, p.position_z],
                    home_quaternion: [p.quat_x, p.quat_y, p.quat_z, p.quat_w],
                    phase: Phase::Arm,
                    waypoint: 0,
                    flight_number: None,
                    reason: None,
                    attention: None,
                });
                self.store.save(&f.record)?;
                self.session = Some(session);
                events::start(&self.bus, &f, self.team);
                let reply = overview(&f.record);
                self.flight = Some(f);
                Ok(Ok(reply))
            }
        }
    }
    fn serve(
        &mut self,
        requests: &Receiver<Query>,
        stopping: &AtomicBool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        while !stopping.load(Ordering::Relaxed) {
            self.bus.pump()?;
            if let Ok((request, reply)) = requests.try_recv() {
                let _ = reply.send(self.request(request)?);
            }
            if let Some(f) = &mut self.flight {
                let before = (f.record.phase, f.record.waypoint, f.record.flight_number);
                if f.record.flight_number.is_none()
                    && self.bus.navigation().is_some_and(|(p, s, m)| {
                        s == f.record.session
                            && matches!(m, Mode::Takeoff | Mode::Hold | Mode::Track | Mode::Land)
                            && p.position_z - f.record.home[2] > 0.15
                    })
                {
                    self.last_flight += 1;
                    f.record.flight_number = Some(self.last_flight);
                }
                let lease = self.bus.landing_active();
                f.tick(&mut self.bus, self.team)?;
                if lease != self.bus.landing_active() {
                    events::record(
                        &self.bus,
                        &f.record.id,
                        json!({"kind":"activity","enabled":!lease}),
                    );
                }
                if before != (f.record.phase, f.record.waypoint, f.record.flight_number) {
                    self.store.save(&f.record)?;
                    if (before.0, before.1) != (f.record.phase, f.record.waypoint) {
                        events::phase(&self.bus, f);
                    }
                }
                if f.record.phase.terminal() {
                    self.bus.lease(false)?;
                    self.flight = None;
                }
            }
            thread::sleep(Duration::from_millis(20));
        }
        self.stop()
    }

    fn stop(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(f) = &mut self.flight {
            f.interrupt("service stopping");
            self.store.save(&f.record)?;
            events::phase(&self.bus, f);
            let deadline = Instant::now() + Duration::from_secs(2);
            while Instant::now() < deadline {
                self.bus.pump()?;
                f.tick(&mut self.bus, self.team)?;
                if f.record.phase.terminal() {
                    self.store.save(&f.record)?;
                    break;
                }
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    iceoryx2::prelude::set_log_level(iceoryx2::prelude::LogLevel::Error);
    let dir = firefly_mission::runtime_dir();
    fs::create_dir_all(&dir)?;
    let lock = File::options()
        .create(true)
        .truncate(false)
        .write(true)
        .open(dir.join("mission.lock"))?;
    lock.try_lock()?;
    let mut daemon = Daemon::new()?;
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
