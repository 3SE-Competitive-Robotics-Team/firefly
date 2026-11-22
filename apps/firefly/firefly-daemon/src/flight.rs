use std::time::{Duration, Instant};

use crate::bus::{Bus, nanos, speed};
use firefly_flight::FlightState as Mode;
use firefly_mission::{Arrival, Phase, Plan};
use firefly_pubsub::{
    command::kind,
    goal::{GoalMessage, PASS, STOP},
    odom::OdomMessage,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy)]
enum Step {
    SendArm,
    AwaitArmed,
    Takeoff,
    Hover,
    Navigate,
    Hold,
    Dwell,
    Acquire(TargetWindow),
    RequestLanding,
    AwaitDisarm,
    Ground,
    Recover { command: Option<u8> },
    NeedsOperator,
    Complete,
    Interrupted,
}
impl Step {
    fn phase(self) -> Phase {
        match self {
            Self::SendArm | Self::AwaitArmed => Phase::Arm,
            Self::Takeoff => Phase::Takeoff,
            Self::Hover => Phase::Hover,
            Self::Navigate => Phase::Navigate,
            Self::Hold => Phase::Hold,
            Self::Dwell => Phase::Dwell,
            Self::Acquire(_) => Phase::Acquire,
            Self::RequestLanding | Self::AwaitDisarm => Phase::Land,
            Self::Ground => Phase::Ground,
            Self::Recover { .. } => Phase::Recover,
            Self::NeedsOperator => Phase::NeedsOperator,
            Self::Complete => Phase::Complete,
            Self::Interrupted => Phase::Interrupted,
        }
    }
    fn timeout(self, dwell: f64) -> Duration {
        Duration::from_secs_f64(match self {
            Self::Navigate => 100.,
            Self::RequestLanding | Self::AwaitDisarm => 45.,
            Self::Dwell | Self::Hover => dwell * 2. + 30.,
            _ => 30.,
        })
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Receipt {
    #[serde(flatten)]
    pub mission: Mission,
    pub phase: Phase,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Mission {
    pub id: String,
    pub run: String,
    pub plan: Plan,
    pub home: [f64; 3],
    pub home_quaternion: [f64; 4],
    pub session: u64,
    pub waypoint: usize,
    pub flight_number: Option<u64>,
    pub reason: Option<String>,
    pub attention: Option<String>,
}

impl Mission {
    fn offset(&self, i: usize) -> [f64; 3] {
        self.plan
            .waypoints
            .get(i)
            .map_or_else(|| self.plan.return_point(), |w| w.offset)
    }
    pub fn point(&self, i: usize) -> [f64; 3] {
        let offset = self.offset(i);
        std::array::from_fn(|axis| self.home[axis] + offset[axis])
    }
}

#[derive(Clone, Copy)]
struct TargetWindow {
    latest: f64,
    streak: Option<(f64, u32)>,
}

impl TargetWindow {
    fn new(start: f64) -> Self {
        Self {
            latest: start,
            streak: None,
        }
    }
    fn observe(
        &mut self,
        target: Option<firefly_pubsub::landing::LandingTargetMessage>,
        now: f64,
        session: u64,
        team: u32,
    ) -> bool {
        let Some(t) = target.filter(|t| {
            t.valid == 1
                && t.session == session
                && t.team == team
                && (0.0..=0.25).contains(&(now - t.timestamp))
                && (0.5..=1.).contains(&t.confidence)
                && (0.0..=2.).contains(&t.reprojection_px)
                && t.position.iter().all(|v| v.is_finite())
        }) else {
            self.streak = None;
            return false;
        };
        if t.timestamp <= self.latest {
            return false;
        }
        self.latest = t.timestamp;
        let (first, count) = self.streak.get_or_insert((t.timestamp, 0));
        *count += 1;
        *count >= 3 && t.timestamp - *first >= 0.15
    }
}

pub struct Flight {
    pub mission: Mission,
    step: Step,
    entered: Instant,
    sequence: u64,
    since: Option<f64>,
    goal: f64,
}

impl Flight {
    pub fn new(mission: Mission) -> Self {
        Self {
            mission,
            step: Step::SendArm,
            entered: Instant::now(),
            sequence: nanos(),
            since: None,
            goal: 0.,
        }
    }
    pub fn resume(receipt: Receipt) -> Self {
        let mut flight = Self::new(receipt.mission);
        if receipt.phase == Phase::NeedsOperator {
            flight.enter(Step::NeedsOperator);
        } else {
            flight.interrupt("daemon restarted during mission");
        }
        flight
    }
    pub fn phase(&self) -> Phase {
        self.step.phase()
    }
    pub fn receipt(&self) -> Receipt {
        Receipt {
            mission: self.mission.clone(),
            phase: self.phase(),
        }
    }
    fn enter(&mut self, step: Step) {
        self.step = step;
        self.entered = Instant::now();
        self.sequence = nanos().max(self.sequence + 1);
        self.since = None;
    }
    pub fn interrupt(&mut self, reason: &str) {
        if matches!(self.step, Step::NeedsOperator) {
            return;
        }
        self.mission.reason.get_or_insert_with(|| reason.to_owned());
        self.enter(Step::Recover { command: None });
    }
    fn sustained(&mut self, condition: bool, stamp: f64, duration: f64) -> bool {
        if condition {
            stamp - *self.since.get_or_insert(stamp) >= duration
        } else {
            self.since = None;
            false
        }
    }
    fn passing(&self) -> bool {
        self.mission
            .plan
            .waypoints
            .get(self.mission.waypoint)
            .is_some_and(|w| w.arrival == Arrival::Pass)
    }
    fn navigate(&mut self, reuse: bool) {
        self.enter(Step::Navigate);
        if !reuse {
            self.goal = (self.sequence as f64 / 1e9).max(self.goal.next_up());
        }
    }
    fn finish_route(&mut self, stamp: f64) {
        self.enter(if self.mission.plan.precision_land {
            Step::Acquire(TargetWindow::new(stamp))
        } else {
            Step::RequestLanding
        });
    }
    fn route(&mut self, bus: &Bus, pose: OdomMessage, mode: Mode) -> firefly_error::Result<()> {
        let i = self.mission.waypoint;
        let [x, y, z] = self.mission.point(i);
        let passing = self.passing();
        let planner = bus.planner.fresh().filter(|p| {
            p.goal_timestamp == self.goal && (pose.timestamp - p.timestamp).abs() < 0.5
        });
        let reference = bus.reference.fresh().is_some_and(|r| {
            r.goal_timestamp == self.goal && (pose.timestamp - r.timestamp).abs() < 0.5
        });
        if planner.is_none() {
            bus.goal.publish(GoalMessage {
                timestamp: self.goal,
                position_x: x,
                position_y: y,
                position_z: z,
                arrival: if passing { PASS } else { STOP },
                next_position: self.mission.point(i + 1),
                ..Default::default()
            })?;
        }
        if mode == Mode::Hold && reference && planner.is_some() {
            bus.command(kind::TRACK, 0., self.sequence)?;
        }
        if mode == Mode::Track && passing && planner.is_some_and(|p| p.passed == 1) {
            self.mission.waypoint += 1;
            self.navigate(!self.passing());
        } else if self.sustained(
            mode == Mode::Track
                && !passing
                && planner.is_some_and(|p| p.finished == 1)
                && (pose.position_x - x)
                    .hypot(pose.position_y - y)
                    .hypot(pose.position_z - z)
                    <= 0.15
                && speed(pose) <= 0.12,
            pose.timestamp,
            0.5,
        ) {
            self.enter(Step::Hold);
        }
        Ok(())
    }
    fn recovery(&mut self, bus: &Bus) -> firefly_error::Result<()> {
        let mode = bus.fc.fresh().and_then(|fc| Mode::from_code(fc.mode));
        if matches!(self.step, Step::Recover { .. })
            && (mode == Some(Mode::AttitudeFallback)
                || self.entered.elapsed() >= Duration::from_secs(45))
        {
            self.mission.attention = Some(format!(
                "disarm unconfirmed, FC {}",
                mode.map_or("unavailable", Mode::name)
            ));
            self.enter(Step::NeedsOperator);
        }
        if let Some(fc) = bus.fc.fresh() {
            let command = match mode {
                _ if matches!(self.step, Step::NeedsOperator) => None,
                Some(Mode::ArmedGrounded) => Some(kind::DISARM),
                Some(Mode::Takeoff | Mode::Hold | Mode::Track) => Some(kind::LAND),
                _ => None,
            };
            if let Some(kind) = command {
                if matches!(self.step, Step::Recover { command: last } if last != command) {
                    self.sequence = nanos().max(self.sequence + 1);
                }
                bus.command(kind, 0., self.sequence)?;
            }
            if let Step::Recover { command: last } = &mut self.step {
                *last = command;
            }
            if self.sustained(fc.mode == Mode::Disarmed.code(), fc.timestamp, 0.75) {
                self.enter(Step::Interrupted);
            }
        } else {
            self.since = None;
        }
        Ok(())
    }
    fn handshake(
        &mut self,
        mode: Mode,
        send: impl FnOnce(u8, u64) -> firefly_error::Result<()>,
    ) -> firefly_error::Result<()> {
        use Step::{AwaitArmed, AwaitDisarm, RequestLanding, SendArm};
        match (self.step, mode) {
            (SendArm | AwaitArmed, Mode::ArmedGrounded) => self.enter(Step::Takeoff),
            (SendArm, Mode::Disarmed) => {
                send(kind::ARM, self.sequence)?;
                self.step = AwaitArmed;
            }
            (RequestLanding | AwaitDisarm, Mode::Hold) => {
                if matches!(self.step, AwaitDisarm) {
                    self.sequence = nanos().max(self.sequence + 1);
                }
                send(
                    if self.mission.plan.precision_land {
                        kind::PRECISION_LAND
                    } else {
                        kind::LAND
                    },
                    self.sequence,
                )?;
                self.step = RequestLanding;
            }
            (RequestLanding, Mode::Land) => self.step = AwaitDisarm,
            (AwaitDisarm, Mode::Disarmed) => self.enter(Step::Ground),
            (RequestLanding, Mode::Disarmed) => {
                self.interrupt("disarmed before landing acknowledgement");
            }
            _ => (),
        }
        Ok(())
    }

    pub fn tick(&mut self, bus: &mut Bus, team: u32) -> firefly_error::Result<()> {
        use Phase::{
            Acquire, Arm, Complete, Dwell, Ground, Hold, Hover, Interrupted, Land, Navigate,
            Recover, Takeoff,
        };
        let phase = self.phase();
        let landing_in_progress = matches!(phase, Recover | Phase::NeedsOperator)
            && bus
                .fc
                .fresh()
                .is_some_and(|fc| fc.mode == Mode::Land.code());
        bus.lease(
            self.mission.plan.precision_land
                && (matches!(phase, Takeoff | Acquire | Land) || landing_in_progress),
        )?;
        if matches!(phase, Recover | Phase::NeedsOperator) {
            return self.recovery(bus);
        }
        let Some((pose, session, mode)) = bus.navigation() else {
            self.interrupt("navigation or flight-controller telemetry stale");
            return Ok(());
        };
        let expected = match phase {
            Arm => matches!(mode, Mode::Disarmed | Mode::ArmedGrounded),
            Takeoff => matches!(mode, Mode::ArmedGrounded | Mode::Takeoff | Mode::Hold),
            Hover | Acquire | Dwell => mode == Mode::Hold,
            Navigate | Hold => matches!(mode, Mode::Hold | Mode::Track),
            Land => matches!(mode, Mode::Disarmed | Mode::Hold | Mode::Land),
            Ground => mode == Mode::Disarmed,
            _ => false,
        };
        let dwell = self.mission.plan.hover_seconds;
        if session != self.mission.session {
            self.interrupt("VIO session changed");
            return Ok(());
        }
        if !expected {
            self.interrupt(&format!("unexpected FC {} during {phase:?}", mode.name()));
            return Ok(());
        }
        if self.entered.elapsed() > self.step.timeout(dwell) {
            self.interrupt(&format!("{phase:?} timed out, FC {}", mode.name()));
            return Ok(());
        }
        match phase {
            Arm | Land => {
                self.handshake(mode, |command, sequence| bus.command(command, 0., sequence))?;
            }
            Takeoff if mode == Mode::Hold => {
                self.enter(Step::Hover);
            }
            Takeoff if mode == Mode::ArmedGrounded => {
                bus.command(kind::TAKEOFF, self.mission.plan.altitude, self.sequence)?;
            }
            Hover if self.sustained(true, pose.timestamp, dwell) => {
                if self.mission.plan.waypoints.is_empty() {
                    self.finish_route(pose.timestamp);
                } else {
                    self.navigate(false);
                }
            }
            Navigate => self.route(bus, pose, mode)?,
            Hold if mode == Mode::Track => bus.command(kind::HOLD, 0., self.sequence)?,
            Hold => self.enter(Step::Dwell),
            Dwell if self.sustained(true, pose.timestamp, dwell) => {
                if self.mission.waypoint == self.mission.plan.waypoints.len() {
                    self.finish_route(pose.timestamp);
                } else {
                    self.mission.waypoint += 1;
                    self.navigate(false);
                }
            }
            Acquire => {
                if let Step::Acquire(window) = &mut self.step
                    && window.observe(
                        bus.target.fresh(),
                        pose.timestamp,
                        self.mission.session,
                        team,
                    )
                {
                    self.enter(Step::RequestLanding);
                }
            }
            Ground if self.sustained(speed(pose) <= 0.12, pose.timestamp, 0.75) => {
                self.enter(Step::Complete);
            }
            Takeoff
            | Hover
            | Dwell
            | Ground
            | Recover
            | Complete
            | Interrupted
            | Phase::NeedsOperator => (),
        }
        Ok(())
    }
}

#[cfg(test)]
mod contract {
    use super::{Flight, Mode, Phase, Receipt, Step, TargetWindow, kind};
    use firefly_mission::Phase::*;
    use firefly_pubsub::landing::LandingTargetMessage;

    fn receipt(phase: Phase) -> Receipt {
        let old = serde_json::json!({
            "id":"survey","run":"boot","phase":phase,"session":7,"flight_number":null,
            "home":[0.,0.,0.],"home_quaternion":[0.,0.,0.,1.],"waypoint":0,
            "plan":{"altitude":1.,"hover_seconds":2.,"return_offset":[0.,0.],"precision_land":true,"waypoints":[]},
            "reason":null,"attention":null
        });
        let receipt: Receipt = serde_json::from_value(old.clone()).unwrap();
        assert_eq!(serde_json::to_value(&receipt).unwrap(), old);
        receipt
    }

    #[test]
    fn arm_is_published_once_and_requires_fc_acknowledgement() {
        let mut f = Flight::new(receipt(Arm).mission);
        let mut sent = Vec::new();
        for mode in [Mode::Disarmed, Mode::Disarmed, Mode::ArmedGrounded] {
            f.handshake(mode, |cmd, _| {
                sent.push(cmd);
                Ok(())
            })
            .unwrap();
        }
        assert_eq!(sent, [kind::ARM]);
        assert_eq!(f.phase(), Takeoff);
    }

    #[test]
    fn landing_requires_a_current_acknowledgement() {
        let mut flight = Flight::new(receipt(Land).mission);
        flight.enter(Step::RequestLanding);
        let mut commands = Vec::new();
        for mode in [Mode::Hold, Mode::Land, Mode::Hold, Mode::Disarmed] {
            flight
                .handshake(mode, |cmd, seq| {
                    commands.push((cmd, seq));
                    Ok(())
                })
                .unwrap();
        }
        assert_eq!(commands.len(), 2);
        assert!(commands.iter().all(|(cmd, _)| *cmd == kind::PRECISION_LAND));
        assert!(commands[1].1 > commands[0].1);
        assert_eq!(flight.phase(), Recover);
        flight.enter(Step::RequestLanding);
        for mode in [Mode::Land, Mode::Disarmed] {
            flight
                .handshake(mode, |_, _| panic!("unexpected command"))
                .unwrap();
        }
        assert_eq!(flight.phase(), Ground);
    }

    #[test]
    fn restart_recovers_in_flight_and_preserves_operator_control() {
        assert_eq!(Flight::resume(receipt(Land)).phase(), Recover);
        assert_eq!(
            Flight::resume(receipt(NeedsOperator)).phase(),
            NeedsOperator
        );
    }

    #[test]
    fn acquisition_requires_distinct_frames_after_the_latest_interruption() {
        let mut window = TargetWindow::new(5.);
        let mut sample = |timestamp, valid| {
            window.observe(
                Some(LandingTargetMessage {
                    timestamp,
                    session: 7,
                    confidence: 0.9,
                    valid,
                    ..Default::default()
                }),
                timestamp,
                7,
                0,
            )
        };
        assert!(!sample(5., 1));
        assert!(!sample(5.1, 1));
        assert!(!sample(5.2, 1));
        assert!(!sample(5.25, 0));
        assert!(!sample(5.2, 1));
        assert!(!sample(5.3, 1));
        assert!(!sample(5.5, 1));
        assert!(!sample(5.5, 1));
        assert!(sample(5.6, 1));
    }

    #[test]
    fn durable_receipts_and_recording_names_are_stable() {
        let phases = [
            Arm,
            Takeoff,
            Hover,
            Navigate,
            Hold,
            Dwell,
            Acquire,
            Land,
            Ground,
            Recover,
            NeedsOperator,
            Complete,
            Interrupted,
        ];
        assert_eq!(
            serde_json::to_value(phases).unwrap(),
            serde_json::json!([
                "arm",
                "takeoff",
                "hover",
                "navigate",
                "hold",
                "dwell",
                "acquire",
                "land",
                "ground",
                "recover",
                "needs_operator",
                "complete",
                "interrupted"
            ])
        );
    }
}
