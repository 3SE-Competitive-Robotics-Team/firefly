use super::{
    FlightEvidence, Pose, Report, Rotation, Scalar, Value, Vec3, json, resources, rotation,
    timeline::{Timeline, number, vec3},
    window,
};
use firefly_error::{Error, ErrorKind, Result};
use std::collections::BTreeSet;

pub(super) type Frame = (Rotation, Vec3);
fn bracket(poses: &[Pose], stamp: f64) -> (usize, usize, f64) {
    let right = poses.partition_point(|s| s.0 < stamp).min(poses.len() - 1);
    let left = right.saturating_sub(1);
    let dt = poses[right].0 - poses[left].0;
    let u = if dt > 0. {
        ((stamp - poses[left].0) / dt).clamp(0., 1.)
    } else {
        0.
    };
    (left, right, u)
}

fn position(poses: &[Pose], stamp: f64) -> Vec3 {
    let (i, j, u) = bracket(poses, stamp);
    poses[i].1.lerp(&poses[j].1, u)
}

fn attitude(poses: &[Pose], stamp: f64) -> Rotation {
    let (i, j, u) = bracket(poses, stamp);
    rotation(poses[i].2).slerp(&rotation(poses[j].2), u)
}

pub(super) fn frame(poses: &[Pose], stamp: f64, home: &Value) -> Option<Frame> {
    let poses = window(poses, stamp - 0.1, stamp + 0.1, |s| s.0);
    if poses.len() < 2
        || poses[0].0 > stamp
        || poses.last()?.0 < stamp
        || poses.windows(2).any(|p| p[1].0 <= p[0].0)
    {
        return None;
    }
    align(poses, stamp, home)
}

fn align(poses: &[Pose], stamp: f64, home: &Value) -> Option<Frame> {
    let q: [f64; 4] = serde_json::from_value(home["quaternion"].clone()).ok()?;
    if q.iter().all(|x| *x == 0.) {
        return None;
    }
    let rot = attitude(poses, stamp) * rotation(q).inverse();
    Some((rot, position(poses, stamp) - rot * vec3(&home["position"])?))
}

fn velocities(poses: &[Pose]) -> Vec<Vec3> {
    (0..poses.len())
        .map(|i| {
            let (stamp, p, _) = poses[i];
            if i == 0 {
                return (poses[1].1 - p) / (poses[1].0 - stamp);
            }
            let (ta, a, _) = poses[i - 1];
            let before = (p - a) / (stamp - ta);
            let Some((tb, b, _)) = poses.get(i + 1) else {
                return before;
            };
            // Nonuniform three-point derivative, with one-sided endpoints.
            let after = (b - p) / (tb - stamp);
            (before * (tb - stamp) + after * (stamp - ta)) / (tb - ta)
        })
        .collect()
}

fn yaw_diff(a: Rotation, b: Rotation) -> f64 {
    let diff = a.euler_angles().2 - b.euler_angles().2;
    diff.sin().atan2(diff.cos()).abs().to_degrees()
}

fn takeoff(
    data: &FlightEvidence<'_>,
    poses: &[Pose],
    modes: &[Scalar],
    timeline: &Timeline,
    frame: &Frame,
    start: Vec3,
    report: &mut Report,
) {
    let Some(begin) = modes.iter().find(|s| s.1[0] == 2.).map(|s| s.0) else {
        return;
    };
    report.metrics.insert("takeoff_s".into(), json!(begin));
    let Some(end) = modes
        .iter()
        .find(|s| s.0 > begin && s.1[0] == 3.)
        .map(|s| s.0)
    else {
        return;
    };
    let selected = window(poses, begin, end, |s| s.0);
    if !selected.is_empty() {
        let rot = attitude(poses, timeline.start);
        report.metrics.insert(
            "takeoff_actual_yaw_change_deg".into(),
            json!(
                selected
                    .iter()
                    .map(|s| yaw_diff(rotation(s.2), rot))
                    .fold(0., f64::max)
            ),
        );
        report.metrics.insert(
            "takeoff_xy_displacement_m".into(),
            json!(
                selected
                    .iter()
                    .map(|s| (s.1 - start).xy().norm())
                    .fold(0., f64::max)
            ),
        );
    }
    let sources: BTreeSet<_> = data
        .scalar("/fc/imu/source", begin, end)
        .iter()
        .map(|s| s.1[0] as i64)
        .collect();
    if !sources.is_empty() {
        report.check(
            sources.iter().all(|v| *v == 1),
            "takeoff did not continuously use VIO attitude",
        );
        report
            .metrics
            .insert("takeoff_attitude_sources".into(), json!(sources));
    }
    let health = data.scalar("/fc/imu/health", begin, end);
    if !health.is_empty() {
        report.metrics.insert(
            "takeoff_posterior_max_age_s".into(),
            json!(
                health
                    .iter()
                    .map(|s| s.1[2])
                    .fold(f64::NEG_INFINITY, f64::max)
            ),
        );
        report.check(
            health.iter().all(|s| s.1[0] == 1. && s.1[1] == 1.),
            "takeoff VIO attitude or posterior became unavailable",
        );
    }
    let quats = data.scalar("/fc/imu/quaternion_xyzw", begin, end);
    if !quats.is_empty() {
        let error = quats
            .iter()
            .map(|(stamp, q)| {
                yaw_diff(
                    frame.0 * rotation([q[0], q[1], q[2], q[3]]),
                    attitude(poses, *stamp),
                )
            })
            .fold(0., f64::max);
        report
            .metrics
            .insert("takeoff_heading_error_deg".into(), json!(error));
    }
}
struct Track<'a> {
    poses: &'a [Pose],
    speeds: Vec<Vec3>,
    modes: &'a [Scalar],
    frame: Frame,
    home: Vec3,
    pad: Vec3,
}

impl Track<'_> {
    fn goal(&self, point: Vec3) -> Vec3 {
        self.frame.1 + self.frame.0 * point
    }
    fn indices(&self, a: f64, b: f64) -> std::ops::Range<usize> {
        self.poses.partition_point(|s| s.0 < a)..self.poses.partition_point(|s| s.0 <= b)
    }
    fn route(&self, timeline: &Timeline, approach: f64, report: &mut Report) {
        let ret = &timeline.phases["return"];
        report.check(
            window(self.modes, ret.timestamp, approach, |s| s.0)
                .iter()
                .any(|s| s.1[0] == 4.),
            "return leg requires native TRACK state",
        );
        self.waypoints(timeline, report);
        let pos = position(self.poses, approach);
        let error = pos - self.goal(ret.goal.expect("validated return"));
        report
            .metrics
            .insert("return_xy_error_m".into(), json!(error.xy().norm()));
        report
            .metrics
            .insert("return_height_error_m".into(), json!(error.z.abs()));
        report.check(
            error.xy().norm() <= 0.3,
            "physical return horizontal error exceeds 0.3m",
        );
        report.check(
            error.z.abs() <= 0.3,
            "physical return height error exceeds 0.3m",
        );
        report.check(
            (pos - self.pad).xy().norm() <= 0.45,
            "return did not reach the pad neighbourhood",
        );
        let mut prev = number(&timeline.settings["altitude"]);
        for (travel, end) in timeline
            .legs
            .iter()
            .map(|l| (&l.travel, l.following.timestamp))
            .chain(std::iter::once((ret, approach)))
        {
            let target =
                travel.goal.expect("validated goal").z - number(&timeline.home["position"][2]);
            let cruise = window(self.poses, travel.timestamp, end, |s| s.0);
            report.check(
                !cruise.is_empty()
                    && cruise
                        .iter()
                        .all(|s| s.1.z - self.home.z >= prev.min(target) - 0.4),
                "premature descent during waypoint flight",
            );
            prev = target;
        }
    }
    fn waypoints(&self, timeline: &Timeline, report: &mut Report) {
        let mut waypoints = Vec::new();
        let mut max_error = 0_f64;
        for (i, leg) in timeline.legs.iter().enumerate() {
            let travel = leg.travel.timestamp;
            let end = leg.following.timestamp;
            let goal = self.goal(leg.travel.goal.expect("validated waypoint"));
            if let Some(hold) = &leg.hold {
                report.check(
                    self.modes
                        .iter()
                        .any(|s| travel <= s.0 && s.0 < hold.timestamp && s.1[0] == 4.),
                    format!("waypoint {i}: native TRACK state missing"),
                );
                report.check(
                    hold.timestamp > travel
                        && end - hold.timestamp
                            >= number(&timeline.settings["hover_seconds"]) - 0.05,
                    format!("waypoint {i}: hover duration too short"),
                );
                let mut modes = self
                    .modes
                    .iter()
                    .filter(|s| hold.timestamp <= s.0 && s.0 < end)
                    .peekable();
                report.check(
                    modes.peek().is_some() && modes.all(|s| s.1[0] == 3.),
                    format!("waypoint {i}: native HOLD state missing during hover"),
                );
                let samples = self.indices(hold.timestamp, end);
                if samples.is_empty() {
                    report.check(
                        false,
                        format!("waypoint {i}: hover ground truth is missing"),
                    );
                    continue;
                }
                let error = samples
                    .clone()
                    .map(|k| (self.poses[k].1 - goal).norm())
                    .fold(0., f64::max);
                let speed = samples.map(|k| self.speeds[k].norm()).fold(0., f64::max);
                max_error = max_error.max(error);
                waypoints.push(json!({"index":i,"arrival":"stop","max_error_m":error,"max_speed_mps":speed,"hover_seconds":end-hold.timestamp}));
                report.check(
                    error <= 0.3,
                    format!("waypoint {i}: physical waypoint hover error exceeds 0.3m"),
                );
                report.check(
                    speed <= 0.25,
                    format!("waypoint {i}: physical waypoint hover did not settle"),
                );
            } else {
                let indices = self.indices(travel.max(end - 0.5), end + 0.5);
                let dist = |k: usize| (self.poses[k].1 - goal).norm();
                let Some(nearest) = indices.clone().min_by(|i, j| dist(*i).total_cmp(&dist(*j)))
                else {
                    report.check(
                        false,
                        format!("waypoint {i}: missing ground truth during passage"),
                    );
                    continue;
                };
                let modes = window(
                    self.modes,
                    self.poses[indices.start].0,
                    self.poses[indices.end - 1].0,
                    |s| s.0,
                );
                report.check(
                    !modes.is_empty() && modes.iter().all(|s| s.1[0] == 4.),
                    format!("waypoint {i}: continuous TRACK required through the waypoint"),
                );
                let error = dist(nearest);
                let speed = self.speeds[nearest].xy().norm();
                report.check(
                    error <= 0.35 && speed > 0.25,
                    format!("waypoint {i}: physical fly-through missed or stopped"),
                );
                max_error = max_error.max(error);
                waypoints.push(json!({"index":i,"arrival":"pass","closest_error_m":error,"pass_speed_mps":speed}));
            }
        }
        report.metrics.insert("waypoints".into(), json!(waypoints));
        report
            .metrics
            .insert("waypoint_max_error_m".into(), json!(max_error));
    }

    fn land(&self, timeline: &Timeline, precision: bool, report: &mut Report) {
        let modes = window(
            self.modes,
            timeline.phases["land"].timestamp,
            timeline.end(),
            |s| s.0,
        );
        let land = modes.iter().position(|s| s.1[0] == 5.);
        report.check(
            land.is_some() && modes.last().is_some_and(|s| s.1[0] == 0.),
            "LAND must finish in DISARMED",
        );
        if let Some(i) = land {
            report.check(
                modes[i..].iter().all(|s| matches!(s.1[0], 0. | 5.)),
                "FC left LAND before completion",
            );
        }
        if let Some(i) = modes.iter().position(|s| s.1[0] == 0.) {
            let stamp = modes[i].0;
            report.metrics.insert("disarmed_s".into(), json!(stamp));
            report.check(
                modes[i..].iter().all(|s| s.1[0] == 0.),
                "FC rearmed after landing",
            );
            report.check(
                (position(self.poses, stamp).z - self.home.z).abs() <= 0.04,
                "disarmed before physical ground contact",
            );
            let (left, right, weight) = bracket(self.poses, stamp);
            report.check(
                self.speeds[left].lerp(&self.speeds[right], weight).norm() <= 0.15,
                "disarmed while physically moving",
            );
            report.check(
                timeline.end() - stamp >= 0.5,
                "post-disarm settling evidence too short",
            );
        }
        let tail = self.indices(timeline.end() - 0.5, f64::INFINITY);
        let xy = tail
            .clone()
            .map(|i| (self.poses[i].1 - self.pad).xy().norm())
            .fold(0., f64::max);
        let height = tail
            .clone()
            .map(|i| (self.poses[i].1.z - self.home.z).abs())
            .fold(0., f64::max);
        let speed = tail.map(|i| self.speeds[i].norm()).fold(0., f64::max);
        report
            .metrics
            .insert("landing_xy_error_m".into(), json!(xy));
        report
            .metrics
            .insert("landing_height_error_m".into(), json!(height));
        report
            .metrics
            .insert("landing_max_speed_mps".into(), json!(speed));
        if precision {
            report.check(xy <= 0.12, "physical landing error exceeds 0.12m");
        }
        report.check(
            height <= 0.02,
            "physical landing is not on the original pad surface",
        );
        report.check(speed <= 0.05, "vehicle is not stationary on the pad");
    }
}

fn height_envelope(
    poses: &[Pose],
    home: Vec3,
    timeline: &Timeline,
    flight_start: f64,
    report: &mut Report,
) {
    let altitude = number(&timeline.settings["altitude"]);
    let max_height = poses
        .iter()
        .map(|s| s.1.z - home.z)
        .fold(f64::NEG_INFINITY, f64::max);
    let takeoff_peak = poses
        .iter()
        .take_while(|s| s.0 <= flight_start)
        .map(|s| s.1.z - home.z)
        .fold(f64::NEG_INFINITY, f64::max);
    let ceiling = timeline
        .legs
        .iter()
        .filter_map(|l| l.travel.goal)
        .map(|g| g.z - number(&timeline.home["position"][2]))
        .fold(altitude, f64::max);
    report.check(
        takeoff_peak >= altitude - 0.2,
        "takeoff height was not reached",
    );
    report.check(
        max_height <= ceiling + 0.4,
        "flight exceeded the altitude envelope",
    );
    if ceiling > altitude + 1e-6 {
        report.check(
            takeoff_peak <= altitude + 0.4,
            "takeoff exceeded the altitude envelope",
        );
    }
    report
        .metrics
        .insert("max_height_m".into(), json!(max_height));
}

fn check_coverage(poses: &[Pose], modes: &[Scalar], start: f64, end: f64, report: &mut Report) {
    report.check(
        poses.len() > 10 && modes.len() > 5,
        "ground truth or FC state evidence is missing",
    );
    if poses.len() <= 10 || modes.len() <= 5 {
        return;
    }
    report.check(
        poses
            .iter()
            .all(|s| s.0.is_finite() && s.1.iter().all(|x| x.is_finite())),
        "nonfinite ground truth",
    );
    report.check(
        poses
            .windows(2)
            .all(|w| w[1].0 > w[0].0 && w[1].0 - w[0].0 <= 0.1),
        "ground truth has duplicate times or coverage gaps",
    );
    report.check(
        poses[0].0 <= start && poses[poses.len() - 1].0 >= end - 0.05,
        "ground truth must cover the whole mission",
    );
    report.check(
        modes.windows(2).all(|w| w[1].0 - w[0].0 <= 0.35),
        "FC state evidence has coverage gaps",
    );
    report.check(
        modes.iter().all(|s| s.1[0] != 6.),
        "FC entered attitude fallback",
    );
}

pub(super) fn grade(
    data: &FlightEvidence<'_>,
    frame: Option<Frame>,
    first: bool,
) -> Result<Report> {
    let mut report = Report::default();
    let Some(timeline) = Timeline::read(data, &mut report) else {
        return Ok(report);
    };
    let precision = timeline.settings["precision_land"] == true;
    if precision {
        resources::images(data, &timeline, &mut report);
    }
    if timeline.settings.get("mission_id").is_some() {
        report.check(
            data.source.capture["scene"]
                .as_str()
                .is_some_and(|s| !s.is_empty()),
            "missing recorded simulation scene",
        );
    }
    let (start, end) = (timeline.start, timeline.end());
    report.metrics.insert("home_s".into(), json!(start));
    report.metrics.insert("complete_s".into(), json!(end));
    let poses = window(data.poses, start - 0.05, end + 0.05, |s| s.0);
    let modes = data.scalar("/fc/debug/state", start, end);
    check_coverage(poses, modes, start, end, &mut report);
    if !report.failures.is_empty() {
        return Ok(report);
    }
    let frame = frame
        .or_else(|| align(poses, start, &timeline.home))
        .ok_or_else(|| Error::new(ErrorKind::InvalidData, "invalid home anchor"))?;
    let home = position(poses, start);
    let approach = timeline.phases[if precision { "acquire" } else { "land" }].timestamp;
    let flight_start = timeline
        .legs
        .first()
        .map_or(approach, |leg| leg.travel.timestamp);
    report.check(
        window(
            modes,
            timeline.phases["takeoff"].timestamp,
            flight_start,
            |s| s.0,
        )
        .iter()
        .any(|s| s.1[0] == 2.),
        "TAKEOFF state not observed",
    );
    let pads: Value = serde_json::from_str(include_str!(
        "../../../../packages/firefly-mujoco/src/firefly_mujoco/hangars.json"
    ))
    .map_err(|e| Error::caused_by(ErrorKind::InvalidData, "decode landing pad coordinates", e))?;
    let scene = timeline.settings["scene"]
        .as_str()
        .unwrap_or("rmuc2027@1.0.0");
    let team = timeline.settings["team"]
        .as_str()
        .ok_or_else(|| Error::new(ErrorKind::InvalidData, "missing team"))?;
    let pad = vec3(&pads[scene][team]).ok_or_else(|| {
        Error::new(
            ErrorKind::InvalidData,
            format!("unknown scene or team: {scene}/{team}"),
        )
    })?;
    report.check(
        (home - pad).xy().norm() <= if first { 0.03 } else { 0.12 }
            && (0.015..=0.03).contains(&(home.z - pad.z)),
        "mission did not start on the configured team pad",
    );
    height_envelope(poses, home, &timeline, flight_start, &mut report);
    takeoff(data, poses, modes, &timeline, &frame, home, &mut report);
    let track = Track {
        poses,
        modes,
        home,
        pad,
        frame,
        speeds: velocities(poses),
    };
    if !timeline.legs.is_empty() {
        track.route(&timeline, approach, &mut report);
    }
    if precision {
        resources::precision(data, &timeline, &mut report);
    }
    track.land(&timeline, precision, &mut report);
    Ok(report)
}
