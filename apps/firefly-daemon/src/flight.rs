use std::time::{Duration, Instant};

use crate::bus::{Bus, nanos, speed};
use firefly_flight::FlightState as Mode;
use firefly_mission::{Arrival, Plan};
use firefly_pubsub::{
    command::kind,
    goal::{GoalMessage, PASS, STOP},
    odom::OdomMessage,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Phase {
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
}
impl Phase {
    fn timeout(self, dwell: f64) -> Duration {
        Duration::from_secs_f64(match self {
            Self::Navigate => 100.,
            Self::Land => 45.,
            Self::Dwell | Self::Hover => dwell * 2. + 30.,
            _ => 30.,
        })
    }
    pub fn terminal(self) -> bool {
        matches!(self, Self::Complete | Self::Interrupted)
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Receipt {
    pub id: String,
    pub run: String,
    pub plan: Plan,
    pub phase: Phase,
    pub home: [f64; 3],
    pub home_quaternion: [f64; 4],
    pub session: u64,
    pub waypoint: usize,
    pub flight_number: Option<u64>,
    pub reason: Option<String>,
    pub attention: Option<String>,
}

pub struct Flight {
    pub record: Receipt,
    entered: Instant,
    sequence: u64,
    arm_sent: bool,
    since: Option<f64>,
    goal: f64,
    acquire_after: f64,
    last_target: f64,
    targets: u32,
    landed_mode: bool,
    recovery_command: Option<u8>,
}

impl Flight {
    pub fn new(record: Receipt) -> Self {
        Self {
            record,
            entered: Instant::now(),
            sequence: nanos(),
            arm_sent: false,
            since: None,
            goal: 0.,
            acquire_after: 0.,
            last_target: 0.,
            targets: 0,
            landed_mode: false,
            recovery_command: None,
        }
    }
    fn enter(&mut self, phase: Phase) {
        self.record.phase = phase;
        self.entered = Instant::now();
        self.sequence = nanos().max(self.sequence + 1);
        self.since = None;
    }
    pub fn interrupt(&mut self, reason: &str) {
        if self.record.phase == Phase::NeedsOperator {
            return;
        }
        self.record.reason.get_or_insert_with(|| reason.to_owned());
        self.enter(Phase::Recover);
    }
    fn sustained(&mut self, condition: bool, stamp: f64, duration: f64) -> bool {
        if condition {
            stamp - *self.since.get_or_insert(stamp) >= duration
        } else {
            self.since = None;
            false
        }
    }
    fn offset(&self, i: usize) -> [f64; 3] {
        self.record
            .plan
            .waypoints
            .get(i)
            .map_or_else(|| self.record.plan.return_point(), |w| w.offset)
    }
    pub fn point(&self, i: usize) -> [f64; 3] {
        let offset = self.offset(i);
        std::array::from_fn(|axis| self.record.home[axis] + offset[axis])
    }
    fn passing(&self) -> bool {
        self.record
            .plan
            .waypoints
            .get(self.record.waypoint)
            .is_some_and(|w| w.arrival == Arrival::Pass)
    }
    fn navigate(&mut self, reuse: bool) {
        self.enter(Phase::Navigate);
        if !reuse {
            self.goal = (self.sequence as f64 / 1e9).max(self.goal.next_up());
        }
    }
    fn finish_route(&mut self, stamp: f64) {
        self.acquire_after = stamp;
        self.enter(if self.record.plan.precision_land {
            Phase::Acquire
        } else {
            Phase::Land
        });
    }
    fn route(&mut self, bus: &Bus, pose: OdomMessage, mode: Mode) -> firefly_error::Result<()> {
        let i = self.record.waypoint;
        let [x, y, z] = self.point(i);
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
                next_position: self.point(i + 1),
                ..Default::default()
            })?;
        }
        if mode == Mode::Hold && reference && planner.is_some() {
            bus.command(kind::TRACK, 0., self.sequence)?;
        }
        if mode == Mode::Track && passing && planner.is_some_and(|p| p.passed == 1) {
            self.record.waypoint += 1;
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
            self.enter(Phase::Hold);
        }
        Ok(())
    }
    fn acquired(&mut self, bus: &Bus, stamp: f64, team: u32) -> bool {
        let target = bus.target.fresh().filter(|t| {
            t.valid == 1
                && t.session == self.record.session
                && t.team == team
                && t.timestamp > self.acquire_after
                && (0.0..=0.25).contains(&(stamp - t.timestamp))
                && (0.5..=1.).contains(&t.confidence)
                && (0.0..=2.).contains(&t.reprojection_px)
                && t.position.iter().all(|v| v.is_finite())
        });
        let Some(t) = target else {
            self.targets = 0;
            self.since = None;
            return false;
        };
        if t.timestamp > self.last_target {
            self.targets += 1;
            self.last_target = t.timestamp;
            self.since.get_or_insert(t.timestamp);
        }
        self.targets >= 3 && self.since.is_some_and(|s| t.timestamp - s >= 0.15)
    }
    fn recovery(&mut self, bus: &Bus) -> firefly_error::Result<()> {
        let mode = bus.fc.fresh().and_then(|fc| Mode::from_code(fc.mode));
        if self.record.phase == Phase::Recover
            && (mode == Some(Mode::AttitudeFallback)
                || self.entered.elapsed() >= Duration::from_secs(45))
        {
            self.record.attention = Some(format!(
                "disarm unconfirmed, FC {}",
                mode.map_or("unavailable", Mode::name)
            ));
            self.enter(Phase::NeedsOperator);
        }
        if let Some(fc) = bus.fc.fresh() {
            let command = match mode {
                _ if self.record.phase == Phase::NeedsOperator => None,
                Some(Mode::ArmedGrounded) => Some(kind::DISARM),
                Some(Mode::Takeoff | Mode::Hold | Mode::Track) => Some(kind::LAND),
                _ => None,
            };
            if let Some(kind) = command {
                if self.recovery_command != command {
                    self.sequence = nanos().max(self.sequence + 1);
                }
                bus.command(kind, 0., self.sequence)?;
            }
            self.recovery_command = command;
            if self.sustained(fc.mode == Mode::Disarmed.code(), fc.timestamp, 0.75) {
                self.enter(Phase::Interrupted);
            }
        } else {
            self.since = None;
        }
        Ok(())
    }
    fn arm(&mut self, bus: &Bus) -> firefly_error::Result<()> {
        if !self.arm_sent {
            bus.command(kind::ARM, 0., self.sequence)?;
            self.arm_sent = true;
        }
        Ok(())
    }

    pub fn tick(&mut self, bus: &mut Bus, team: u32) -> firefly_error::Result<()> {
        use Phase::{
            Acquire, Arm, Complete, Dwell, Ground, Hold, Hover, Interrupted, Land, Navigate,
            Recover, Takeoff,
        };
        let phase = self.record.phase;
        let landing_in_progress = matches!(phase, Recover | Phase::NeedsOperator)
            && bus
                .fc
                .fresh()
                .is_some_and(|fc| fc.mode == Mode::Land.code());
        bus.lease(
            self.record.plan.precision_land
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
        let dwell = self.record.plan.hover_seconds;
        if session != self.record.session {
            self.interrupt("VIO session changed");
            return Ok(());
        }
        if !expected {
            self.interrupt(&format!("unexpected FC {} during {phase:?}", mode.name()));
            return Ok(());
        }
        if self.entered.elapsed() > phase.timeout(dwell) {
            self.interrupt(&format!("{phase:?} timed out, FC {}", mode.name()));
            return Ok(());
        }
        match phase {
            Arm if mode == Mode::ArmedGrounded => self.enter(Takeoff),
            Arm => self.arm(bus)?,
            Takeoff if mode == Mode::Hold => {
                self.enter(Hover);
            }
            Takeoff if mode == Mode::ArmedGrounded => {
                bus.command(kind::TAKEOFF, self.record.plan.altitude, self.sequence)?;
            }
            Hover if self.sustained(true, pose.timestamp, dwell) => {
                if self.record.plan.waypoints.is_empty() {
                    self.finish_route(pose.timestamp);
                } else {
                    self.navigate(false);
                }
            }
            Navigate => self.route(bus, pose, mode)?,
            Hold if mode == Mode::Track => bus.command(kind::HOLD, 0., self.sequence)?,
            Hold => self.enter(Dwell),
            Dwell if self.sustained(true, pose.timestamp, dwell) => {
                if self.record.waypoint == self.record.plan.waypoints.len() {
                    self.finish_route(pose.timestamp);
                } else {
                    self.record.waypoint += 1;
                    self.navigate(false);
                }
            }
            Acquire if self.acquired(bus, pose.timestamp, team) => self.enter(Land),
            Land if mode == Mode::Hold => bus.command(
                if self.record.plan.precision_land {
                    kind::PRECISION_LAND
                } else {
                    kind::LAND
                },
                0.,
                self.sequence,
            )?,
            Land if mode == Mode::Land => self.landed_mode = true,
            Land if mode == Mode::Disarmed && self.landed_mode => self.enter(Ground),
            Land if mode == Mode::Disarmed => {
                self.interrupt("disarmed before landing acknowledgement");
            }
            Ground if self.sustained(speed(pose) <= 0.12, pose.timestamp, 0.75) => {
                self.enter(Complete);
            }
            Takeoff
            | Hover
            | Dwell
            | Acquire
            | Ground
            | Land
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
    use super::Phase::*;

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
