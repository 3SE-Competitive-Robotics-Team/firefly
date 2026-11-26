//! H 起飞观测回放。每份输入是一架次；只输出观测和接管候选。
use crate::{Result, external, invalid, tracking::Settings};
use firefly_landing::{Detection, geometry::outline_candidates};
use firefly_mission::Phase;
use firefly_pubsub::camera::{DOWN_HEIGHT, DOWN_WIDTH};
use nalgebra::{Isometry3, Quaternion, Rotation2, Translation3, UnitQuaternion, Vector2, Vector3};
use serde::Deserialize;
use std::{
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Frame {
    time: f64,
    image: PathBuf,
    launch_active: Option<bool>,
    takeoff_report: Option<f64>,
    /// 回放器提供最近一次已就绪的独立 IMU 姿态，采用 xyzw。
    imu: Option<[f64; 4]>,
    vio: Option<Vio>,
}

#[derive(Deserialize)]
#[serde(untagged)]
enum Replay {
    Mission {
        frames: Vec<Frame>,
        events: Vec<TimedEvent>,
        end: Option<f64>,
    },
    Single(Vec<Frame>),
}

#[derive(Deserialize)]
struct TimedEvent {
    time: f64,
    run: String,
    mission_id: String,
    #[serde(flatten)]
    event: MissionEvent,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum MissionEvent {
    Home { session: u64 },
    Phase { name: Phase },
}

#[derive(Default)]
enum Attempt {
    #[default]
    Idle,
    Waiting,
    TakingOff(Observer),
    Released,
    Closed,
}

#[derive(Default)]
struct MissionReplay {
    identity: Option<(String, String, u64)>,
    seen: std::collections::HashSet<(String, String)>,
    attempt: Attempt,
}

impl MissionReplay {
    fn clock(&mut self, until: f64) {
        if let Attempt::TakingOff(observer) = &mut self.attempt
            && let Some(time) = observer.expire(until)
        {
            println!(
                "{}",
                serde_json::json!({"kind":"timeout","time":time,"state":observer.state.name(),"mission":self.identity})
            );
        }
    }

    fn advance(
        &mut self,
        events: &mut std::iter::Peekable<std::vec::IntoIter<TimedEvent>>,
        until: f64,
    ) {
        while events.peek().is_some_and(|e| e.time <= until) {
            if let Some(event) = events.next() {
                self.clock(event.time);
                self.event(event);
            }
        }
        self.clock(until);
    }
    fn event(&mut self, message: TimedEvent) {
        let same = self
            .identity
            .as_ref()
            .is_some_and(|(run, id, _)| run == &message.run && id == &message.mission_id);
        if same && matches!(self.attempt, Attempt::Closed) {
            return;
        }
        match message.event {
            MissionEvent::Home { session } => {
                if same {
                    if self.identity.as_ref().is_some_and(|key| key.2 != session) {
                        self.attempt = Attempt::Released;
                    }
                } else if matches!(self.attempt, Attempt::Idle | Attempt::Closed)
                    && self
                        .seen
                        .insert((message.run.clone(), message.mission_id.clone()))
                {
                    self.identity = Some((message.run, message.mission_id, session));
                    self.attempt = Attempt::Waiting;
                }
            }
            MissionEvent::Phase { name } if same => match name {
                Phase::Takeoff if matches!(self.attempt, Attempt::Waiting) => {
                    self.attempt = Attempt::TakingOff(Observer {
                        launch: Some(message.time),
                        ..Default::default()
                    });
                }
                Phase::Arm | Phase::Takeoff | Phase::Hover => {}
                Phase::Navigate
                | Phase::Hold
                | Phase::Dwell
                | Phase::Acquire
                | Phase::Land
                | Phase::Ground
                | Phase::Recover
                | Phase::NeedsOperator => self.attempt = Attempt::Released,
                Phase::Complete | Phase::Interrupted => self.attempt = Attempt::Closed,
            },
            MissionEvent::Phase { .. } => {}
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Vio {
    position: [f64; 3],
    velocity: [f64; 3],
    orientation: [f64; 4],
    posterior_age_s: f64,
    stereo_ids: u32,
    landmark_ids: u32,
}

impl Vio {
    fn overlap(
        &self,
        imu: Option<UnitQuaternion<f64>>,
        level_body_from_h: Option<Isometry3<f64>>,
    ) -> Option<OverlapSample> {
        if !(0.0..=0.15).contains(&self.posterior_age_s)
            || self.stereo_ids < 8
            || self.landmark_ids < 6
            || Vector3::from(self.velocity).norm() > 0.25
        {
            return None;
        }
        let rotation = quaternion(self.orientation)?;
        let yaw = imu?.euler_angles().2 - rotation.euler_angles().2;
        let position = Vector3::from(self.position);
        Some(OverlapSample {
            position,
            yaw,
            vio_from_h: Isometry3::from_parts(position.into(), rotation) * level_body_from_h?,
        })
    }
}

fn quaternion(xyzw: [f64; 4]) -> Option<UnitQuaternion<f64>> {
    let rotation = Quaternion::new(xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
    ((rotation.norm_squared() - 1.).abs() < 0.01).then(|| UnitQuaternion::new_normalize(rotation))
}

struct Measurement {
    pad: Option<Vector3<f64>>,
    body_from_h: Option<Isometry3<f64>>,
    level_body_from_h: Option<Isometry3<f64>>,
}

fn measure(
    settings: &Settings,
    pixels: &[u8],
    rotation: Option<UnitQuaternion<f64>>,
) -> Result<Measurement> {
    let camera = rotation
        .map(|q| Isometry3::from_parts(Translation3::identity(), q) * settings.camera_in_body);
    let full = Detection {
        bounds: [0., 0., DOWN_WIDTH as f32, DOWN_HEIGHT as f32],
        confidence: 1.,
    };
    let outlines = outline_candidates(pixels, DOWN_WIDTH, DOWN_HEIGHT, full)?;
    let target = camera.as_ref().and_then(|camera| {
        outlines
            .iter()
            .filter_map(|o| {
                settings
                    .geometry
                    .horizontal_target(o.rectangle, camera)
                    .map(|t| (o, t))
            })
            .min_by(|a, b| a.1.reprojection_px.total_cmp(&b.1.reprojection_px))
    });
    let visual = if let Some((outline, _)) = target {
        settings.geometry.marker_pose(outline)
    } else {
        outlines
            .iter()
            .filter_map(|o| settings.geometry.marker_pose(o))
            .min_by(|a, b| a.reprojection_px.total_cmp(&b.reprojection_px))
    };
    let body_from_h = visual.map(|v| settings.camera_in_body * v.camera_from_h);
    let level_body_from_h =
        rotation
            .zip(body_from_h)
            .zip(target)
            .and_then(|((attitude, visual), (_, plane))| {
                let axes = attitude * visual.rotation;
                if (axes * Vector3::z()).angle(&Vector3::z()) > 25_f64.to_radians() {
                    return None;
                }
                let across = axes * Vector3::x();
                let level =
                    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), across.y.atan2(across.x));
                Some(Isometry3::from_parts(
                    (attitude.inverse() * plane.center).into(),
                    attitude.inverse() * level,
                ))
            });
    Ok(Measurement {
        pad: target.map(|(_, t)| -t.center - Vector3::z() * settings.contact_offset_m),
        body_from_h,
        level_body_from_h,
    })
}

/// `p_pad = R_z(yaw) * p_vio + translation`。在重叠窗口内固定这组参数。
#[derive(Clone, Copy)]
struct Alignment {
    yaw: f64,
    translation: Vector3<f64>,
}

#[derive(Clone, Copy)]
struct OverlapSample {
    position: Vector3<f64>,
    yaw: f64,
    vio_from_h: Isometry3<f64>,
}

fn same_marker_pose(reference: Isometry3<f64>, current: Isometry3<f64>) -> bool {
    let delta = reference.inverse() * current;
    let half_turn = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::PI);
    delta.translation.vector.norm() <= 0.06
        && delta
            .rotation
            .angle()
            .min((delta.rotation * half_turn).angle())
            <= 5_f64.to_radians()
}

fn rotate(yaw: f64, p: Vector3<f64>) -> Vector3<f64> {
    let xy = Rotation2::new(yaw) * Vector2::new(p.x, p.y);
    Vector3::new(xy.x, xy.y, p.z)
}

#[derive(Default)]
enum State {
    #[default]
    Seeking,
    Acquiring(u8),
    Pad,
    Lost,
    Overlap {
        start: f64,
        alignment: Alignment,
        vio_from_h: Isometry3<f64>,
    },
    Eligible(Alignment),
    Abort(&'static str),
}

impl State {
    fn name(&self) -> &'static str {
        match self {
            Self::Seeking => "seeking_h",
            Self::Acquiring(_) => "confirming_h",
            Self::Pad => "tracking_h",
            Self::Lost => "waiting_for_h",
            Self::Overlap { .. } => "checking_vio",
            Self::Eligible(_) => "handover_candidate",
            Self::Abort(reason) => reason,
        }
    }
}

#[derive(Default)]
struct Observer {
    state: State,
    launch: Option<f64>,
    last: Option<(f64, Vector3<f64>)>,
    saw_fc_takeoff: bool,
}

impl Observer {
    fn expire(&mut self, now: f64) -> Option<f64> {
        let start = self.launch?;
        let (due, reason) = match self.state {
            State::Eligible(_) | State::Abort(_) => return None,
            State::Seeking | State::Acquiring(_) => (
                start + 1.5,
                if self.saw_fc_takeoff {
                    "abort_h_acquisition_timeout"
                } else {
                    "abort_fc_takeoff_timeout"
                },
            ),
            State::Pad | State::Overlap { .. } | State::Lost => {
                let lost = self.last?.0 + 0.25;
                if lost < start + 6. {
                    (lost, "abort_h_lost")
                } else {
                    (start + 6., "abort_handover_timeout")
                }
            }
        };
        if now <= due {
            return None;
        }
        self.state = State::Abort(reason);
        Some(due)
    }
    fn sample(
        &mut self,
        active: Option<bool>,
        takeoff_report: Option<f64>,
        time: f64,
        pad: Option<Vector3<f64>>,
        vio: Option<OverlapSample>,
        scoped: bool,
    ) -> &'static str {
        if matches!(self.state, State::Abort(_)) {
            return self.state.name();
        }
        if scoped
            && !self.saw_fc_takeoff
            && active == Some(true)
            && !takeoff_report.is_some_and(|t| {
                self.launch.is_some_and(|start| t >= start) && (0.0..=0.25).contains(&(time - t))
            })
        {
            return "waiting_for_fc";
        }
        match active {
            Some(true) => {
                self.saw_fc_takeoff = true;
                self.observe(time, pad, vio);
            }
            Some(false) if scoped && self.saw_fc_takeoff => {
                self.state = State::Abort("abort_fc_left_takeoff");
            }
            Some(false) if scoped && self.launch.is_some_and(|start| time - start > 1.5) => {
                self.state = State::Abort("abort_fc_takeoff_timeout");
            }
            Some(false) => {
                if !scoped {
                    *self = Self::default();
                }
                return "waiting_for_fc";
            }
            None if self.launch.is_some() => self.state = State::Abort("abort_fc_stale"),
            None => return "waiting_for_fc",
        }
        self.state.name()
    }
    fn observe(&mut self, time: f64, pad: Option<Vector3<f64>>, vio: Option<OverlapSample>) {
        // 一次回放只评分首次接管机会。实际切换后应由飞行中的健康监控接手。
        if matches!(self.state, State::Eligible(_) | State::Abort(_)) {
            return;
        }
        self.saw_fc_takeoff = true;
        self.launch.get_or_insert(time);
        if self.expire(time).is_some() {
            return;
        }
        let tracking = matches!(self.state, State::Pad | State::Overlap { .. } | State::Lost);
        let pad = pad.filter(|p| (0.0..=1.35).contains(&p.z) && p.xy().norm() < 0.35);
        if tracking && let Some((stamp, prev)) = self.last {
            let dt = time - stamp;
            if pad.is_none_or(|p| (p - prev).norm() > 0.04 + dt) {
                self.state = State::Lost;
                return;
            }
        }
        let Some(p) = pad else {
            self.state = State::Seeking;
            self.last = None;
            return;
        };
        let consistent = self.last.is_some_and(|(t, prev)| {
            let dt = time - t;
            dt > 0. && dt <= 0.25 && (p - prev).norm() <= 0.04 + dt
        });
        self.last = Some((time, p));
        if !consistent {
            self.state = State::Acquiring(1);
            return;
        }
        if let State::Acquiring(count) = self.state {
            self.state = if count >= 2 {
                State::Pad
            } else {
                State::Acquiring(count + 1)
            };
            return;
        }
        let Some(OverlapSample {
            position: v,
            yaw,
            vio_from_h,
        }) = vio.filter(|_| (0.9..=1.1).contains(&p.z))
        else {
            self.state = State::Pad;
            return;
        };
        if let State::Overlap {
            start,
            alignment,
            vio_from_h: reference,
        } = self.state
        {
            let angle = (yaw - alignment.yaw)
                .sin()
                .atan2((yaw - alignment.yaw).cos());
            let delta = rotate(alignment.yaw, v) + alignment.translation - p;
            if angle.abs() > 5_f64.to_radians()
                || delta.norm() > 0.06
                || !same_marker_pose(reference, vio_from_h)
            {
                self.state = State::Pad;
            } else if time - start >= 0.5 {
                self.state = State::Eligible(alignment);
            }
        } else {
            self.state = State::Overlap {
                start: time,
                vio_from_h,
                alignment: Alignment {
                    yaw,
                    translation: p - rotate(yaw, v),
                },
            };
        }
    }
}

pub(crate) fn run(settings: &Settings, manifest: &Path) -> Result<()> {
    let file = File::open(manifest).map_err(|e| external("takeoff.open", e))?;
    let input: Replay = serde_json::from_reader(BufReader::new(file))
        .map_err(|e| external("takeoff.manifest", e))?;
    let (frames, events, scoped, end) = match input {
        Replay::Mission {
            frames,
            events,
            end,
        } => (frames, events, true, end),
        Replay::Single(frames) => (frames, Vec::new(), false, None),
    };
    if events.iter().any(|e| !e.time.is_finite())
        || events.windows(2).any(|w| w[0].time > w[1].time)
    {
        return Err(invalid(
            "mission events must be ordered by measurement time",
        ));
    }
    let mut events = events.into_iter().peekable();
    let mut mission = MissionReplay::default();
    let mut legacy = Observer::default();
    let mut previous = None;
    if end.is_some_and(|t| !t.is_finite() || frames.last().is_some_and(|f| f.time > t)) {
        return Err(invalid("invalid replay end time"));
    }
    if frames.is_empty() && end.is_none() {
        return Err(invalid("empty takeoff replay"));
    }
    for frame in frames {
        if !frame.time.is_finite() || previous.is_some_and(|last| frame.time <= last) {
            return Err(invalid("takeoff replay requires increasing capture times"));
        }
        previous = Some(frame.time);
        let image_path = manifest.with_file_name(&frame.image);
        let img = image::open(&image_path)
            .map_err(|e| external("takeoff.image", e).with_context("path", image_path.display()))?
            .to_luma8();
        if img.dimensions() != (DOWN_WIDTH as u32, DOWN_HEIGHT as u32) {
            return Err(invalid("takeoff replay requires 352x288 down frames"));
        }
        let imu = frame.imu.and_then(quaternion);
        let Measurement {
            pad,
            body_from_h,
            level_body_from_h,
        } = measure(settings, img.as_raw(), imu)?;
        let vio = frame.vio.and_then(|v| v.overlap(imu, level_body_from_h));
        mission.advance(&mut events, frame.time);
        let observer = if scoped {
            match &mut mission.attempt {
                Attempt::TakingOff(observer) => Some(observer),
                _ => None,
            }
        } else {
            Some(&mut legacy)
        };
        let (state, alignment) = if let Some(observer) = observer {
            let state = observer.sample(
                frame.launch_active,
                frame.takeoff_report,
                frame.time,
                pad,
                vio,
                scoped,
            );
            let alignment = match observer.state {
                State::Eligible(a) => Some(serde_json::json!({"yaw_rad": a.yaw,
                "translation_m": a.translation.as_slice()})),
                _ => None,
            };
            (state, alignment)
        } else {
            ("outside_mission_takeoff", None)
        };
        println!(
            "{}",
            serde_json::json!({"time": frame.time, "pad_m": pad.map(|v| [v.x,v.y,v.z]),
            "body_from_h_mod_pi":body_from_h.map(|v| serde_json::json!({"translation_m":v.translation.vector.as_slice(),"xyzw":v.rotation.coords.as_slice()})),
            "gravity_aided_body_from_h_mod_pi":level_body_from_h.map(|v| serde_json::json!({"translation_m":v.translation.vector.as_slice(),"xyzw":v.rotation.coords.as_slice()})),
            "mission":mission.identity, "state": state, "alignment": alignment})
        );
    }
    if let Some(end) = end {
        mission.advance(&mut events, end);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn stationary_marker(position: Vector3<f64>) -> OverlapSample {
        OverlapSample {
            position,
            yaw: 0.,
            vio_from_h: Isometry3::identity(),
        }
    }

    #[test]
    fn mission_deadline_fires_once_without_another_image() {
        let mut obs = Observer {
            launch: Some(4.),
            ..Default::default()
        };
        assert_eq!(obs.expire(5.5), None);
        assert_eq!(obs.expire(7.), Some(5.5));
        assert!(matches!(
            obs.state,
            State::Abort("abort_fc_takeoff_timeout")
        ));
        assert_eq!(obs.expire(8.), None);
        let mut obs = Observer::default();
        for time in [4., 4.1, 4.2] {
            obs.observe(time, Some(Vector3::z() * 0.4), None);
        }
        assert_eq!(obs.expire(6.), Some(4.45));
        assert!(matches!(obs.state, State::Abort("abort_h_lost")));
    }

    #[test]
    fn hold_and_prior_takeoff_ack_cannot_open_a_new_mission() {
        let mut obs = Observer {
            launch: Some(10.),
            ..Default::default()
        };
        for stamp in [None, Some(9.99), Some(10.2)] {
            assert_eq!(
                obs.sample(Some(true), stamp, 10.1, None, None, true),
                "waiting_for_fc"
            );
            assert!(!obs.saw_fc_takeoff);
        }
        assert_eq!(
            obs.sample(Some(true), Some(10.05), 10.1, None, None, true),
            "seeking_h"
        );
        assert!(obs.saw_fc_takeoff);
    }

    #[test]
    fn handover_requires_continuous_overlap_at_height_with_fixed_alignment() {
        let mut observer = Observer::default();
        let p = Vector3::z();
        let origin = Vector3::new(-2., 3., 12.);
        for i in 0..=18 {
            let pad = (i >= 2 && i != 6).then_some(p);
            let vio = (i != 11).then_some(stationary_marker(p + origin));
            observer.observe(f64::from(i) * 0.125, pad, vio);
            if i < 16 {
                assert!(!matches!(observer.state, State::Eligible(_)));
            }
        }
        let State::Eligible(a) = observer.state else {
            panic!("expected handover")
        };
        assert_eq!(a.translation, -origin);
    }

    #[test]
    fn drift_blindness_and_sparse_frames_cannot_qualify() {
        for (height, interval, drift) in [(0.5, 0.1, 0.), (1., 0.3, 0.), (1., 0.1, 0.5)] {
            let mut observer = Observer::default();
            for i in 0..30 {
                let t = f64::from(i) * interval;
                observer.observe(
                    t,
                    Some(Vector3::z() * height),
                    Some(stationary_marker(Vector3::x() * drift * t)),
                );
                assert!(!matches!(observer.state, State::Eligible(_)));
            }
            observer.observe(10., None, None);
            assert!(matches!(observer.state, State::Abort(_)));
        }
    }

    #[test]
    fn lost_pad_expires_before_reacquisition_and_aborts_the_attempt() {
        let mut observer = Observer::default();
        let pad = Some(Vector3::z() * 0.4);
        for t in [0., 0.1, 0.2] {
            observer.observe(t, pad, None);
        }
        observer.observe(0.3, None, None);
        assert!(matches!(observer.state, State::Lost));
        observer.observe(0.46, pad, None);
        observer.observe(0.52, pad, None);
        assert!(matches!(observer.state, State::Abort("abort_h_lost")));
    }

    #[test]
    fn blind_launch_expires_even_if_vio_reports_a_position() {
        let mut observer = Observer::default();
        for t in [0., 0.7, 1.4, 1.51] {
            observer.observe(t, None, Some(stationary_marker(Vector3::z())));
        }
        assert!(matches!(
            observer.state,
            State::Abort("abort_h_acquisition_timeout")
        ));
    }
    fn event(id: &str, time: f64, event: MissionEvent) -> TimedEvent {
        TimedEvent {
            time,
            run: "boot".into(),
            mission_id: id.into(),
            event,
        }
    }

    #[test]
    fn retry_cancel_recover_and_next_mission_have_distinct_takeoff_windows() {
        let mut replay = MissionReplay::default();
        let go = |id, t| {
            event(
                id,
                t,
                MissionEvent::Phase {
                    name: Phase::Takeoff,
                },
            )
        };
        let home = |id, t| event(id, t, MissionEvent::Home { session: 17 });
        replay.event(home("hover", 1.));
        replay.event(go("hover", 2.));
        replay.event(home("hover", 2.1));
        replay.event(go("hover", 2.2));
        replay.event(home("square", 2.3));
        assert!(matches!(&replay.attempt,Attempt::TakingOff(o) if o.launch==Some(2.)));
        for name in [Phase::Recover, Phase::NeedsOperator] {
            replay.event(event("hover", 3., MissionEvent::Phase { name }));
            replay.event(go("hover", 3.1));
            replay.event(home("square", 3.2));
            assert!(matches!(replay.attempt, Attempt::Released));
        }
        replay.event(event(
            "hover",
            4.,
            MissionEvent::Phase {
                name: Phase::Interrupted,
            },
        ));
        replay.event(home("square", 5.));
        replay.event(go("square", 6.));
        replay.event(event(
            "hover",
            6.2,
            MissionEvent::Phase { name: Phase::Land },
        ));
        assert!(
            matches!(&replay.attempt,Attempt::TakingOff(o) if o.launch==Some(6.) && o.last.is_none())
        );
        replay.event(event(
            "square",
            7.,
            MissionEvent::Phase {
                name: Phase::Navigate,
            },
        ));
        replay.event(event(
            "square",
            8.,
            MissionEvent::Phase { name: Phase::Hold },
        ));
        replay.event(go("square", 9.));
        assert!(matches!(replay.attempt, Attempt::Released));
        replay.event(event(
            "square",
            10.,
            MissionEvent::Phase {
                name: Phase::Complete,
            },
        ));
        replay.event(home("hover", 11.));
        assert!(matches!(replay.attempt, Attempt::Closed));
    }

    #[test]
    fn attitude_gate_accepts_half_turn_symmetry_but_rejects_roll_pitch_and_yaw_drift() {
        let reference = Isometry3::from_parts(
            Translation3::new(0.2, 0.1, -0.7),
            UnitQuaternion::from_euler_angles(0.4, 0.2, -0.2),
        );
        for (roll, pitch, yaw, accept) in [
            (0., 0., std::f64::consts::PI, true),
            (0.2, 0., 0., false),
            (0., 0.2, 0., false),
            (0., 0., 0.2, false),
        ] {
            let change = Isometry3::from_parts(
                Translation3::identity(),
                UnitQuaternion::from_euler_angles(roll, pitch, yaw),
            );
            assert_eq!(same_marker_pose(reference, reference * change), accept);
        }
    }
}
