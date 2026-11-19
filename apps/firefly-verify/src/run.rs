use super::{
    Event, Evidence, FlightEvidence, Report, Value, flight, json,
    timeline::{Timeline, rows},
};
use firefly_mission::Phase;
use serde::Deserialize;
use std::num::NonZeroU64;

enum Attempt {
    Ground,
    Airborne(NonZeroU64),
}

impl Attempt {
    fn observe(&mut self, row: &Value, report: &mut Report) {
        let id = row["flight_id"].as_u64().and_then(NonZeroU64::new);
        if row["kind"] == "airborne" {
            if let (Self::Ground, Some(id)) = (&*self, id) {
                *self = Self::Airborne(id);
            } else {
                report.check(false, "invalid or repeated airborne assignment");
            }
        }
        let consistent = match self {
            Self::Ground => row["flight_id"].is_null(),
            Self::Airborne(assigned) => Some(*assigned) == id,
        };
        report.check(
            consistent,
            "flight identity changed without airborne assignment",
        );
    }
}

type Groups = Vec<(u64, Vec<Event>)>;

fn interrupted_ground_attempt(events: &[Event]) -> bool {
    use Phase::{Arm, Interrupted, NeedsOperator, Recover, Takeoff};
    let mut phases = rows(events, "phase")
        .map(|e| Phase::deserialize(&e.1["name"]).ok())
        .peekable();
    if phases.next() != Some(Some(Arm)) {
        return false;
    }
    phases.next_if_eq(&Some(Takeoff));
    if phases.next() != Some(Some(Recover)) {
        return false;
    }
    // Each daemon restart can announce the same durable recovery phase again.
    while phases.next_if_eq(&Some(Recover)).is_some() {}
    while phases.next_if_eq(&Some(NeedsOperator)).is_some() {}
    phases.next() == Some(Some(Interrupted)) && phases.next().is_none()
}

fn native_groups(data: &Evidence, report: &mut Report) -> Option<Groups> {
    report.check(data.recording_complete, "incomplete recording");
    report.check(
        rows(&data.events, "system_run").next().is_some(),
        "missing system run declaration",
    );
    let mut run = None;
    let mut missions: Vec<(&str, Vec<Event>)> = Vec::new();
    let mut previous = None;
    for event @ (_, row) in &data.events {
        let id = row["run"].as_str().filter(|s| !s.is_empty());
        if row["kind"] == "system_run" {
            run = run.or(id);
            report.check(
                run.is_some() && id == run,
                "conflicting system run identities",
            );
            continue;
        }
        let Some(mission) = row["mission_id"]
            .as_str()
            .filter(|s| !s.is_empty() && id.is_some() && row.get("flight_id").is_some())
        else {
            report.check(false, "missing native run, mission or airborne identity");
            return None;
        };
        run = run.or(id);
        report.check(id == run, "conflicting system run identities");
        let index = missions.iter().position(|(name, _)| *name == mission);
        if previous != Some(mission) {
            report.check(index.is_none(), "mission events overlap");
            previous = Some(mission);
        }
        if let Some(i) = index {
            missions[i].1.push(event.clone());
        } else {
            missions.push((mission, vec![event.clone()]));
        }
    }
    let mut attempts = Vec::new();
    let mut groups = Groups::new();
    for (mission, events) in missions {
        let mut attempt = Attempt::Ground;
        for (_, row) in &events {
            attempt.observe(row, report);
        }
        let Attempt::Airborne(assigned) = attempt else {
            attempts.push(mission);
            report.check(
                interrupted_ground_attempt(&events),
                format!("{mission}: missing airborne or interruption evidence"),
            );
            continue;
        };
        let id = assigned.get();
        report.check(
            groups.last().is_none_or(|g| g.0.checked_add(1) == Some(id)),
            "flight sequence has gaps or duplicates",
        );
        if let Some(g) = groups.iter_mut().find(|g| g.0 == id) {
            g.1 = events;
        } else {
            groups.push((id, events));
        }
    }
    report.metrics.insert("run".into(), json!(run));
    report
        .metrics
        .insert("ground_attempts".into(), json!(attempts));
    Some(groups)
}

fn legacy_groups(data: &Evidence, count: u64, report: &mut Report) -> Groups {
    let mut groups = Groups::new();
    let mut last = 0;
    for e in &data.events {
        if matches!(
            e.1["kind"].as_str(),
            Some("system_run" | "source" | "provenance")
        ) {
            continue;
        }
        let Some(id) = e.1["flight_id"]
            .as_u64()
            .filter(|n| (1..=count).contains(n))
        else {
            report.check(false, "missing or invalid flight identity");
            continue;
        };
        report.check(id >= last, "flight events overlap or run out of order");
        last = id;
        if let Some(g) = groups.iter_mut().find(|g| g.0 == id) {
            g.1.push(e.clone());
        } else {
            groups.push((id, vec![e.clone()]));
        }
    }
    report.check(
        groups.len() as u64 == count && groups.iter().enumerate().all(|(i, g)| g.0 == i as u64 + 1),
        "run did not execute all declared flights in order",
    );
    groups
}

fn lifecycle(data: &FlightEvidence<'_>, session: &Value) -> Report {
    let mut report = Report::default();
    if let Some(timeline) = Timeline::read(data, &mut report) {
        report.check(
            !session.is_null() && timeline.home["session"] == *session,
            "shared VIO session changed",
        );
        let modes = data.scalar("/fc/debug/state", timeline.start, timeline.end());
        let mut last = timeline.start;
        let coverage = modes.iter().all(|s| {
            let dt = s.0 - last;
            last = s.0;
            dt <= 0.35 && s.1[0] != 6.
        });
        report.check(
            coverage && timeline.end() - last <= 0.35,
            "FC status gap or attitude fallback during flight",
        );
        let takeoff = modes.iter().position(|s| s.1[0] == 2.);
        let land = modes.iter().position(|s| s.1[0] == 5.);
        report.check(
            takeoff.zip(land).is_some_and(|(a, b)| a < b)
                && modes.last().is_some_and(|s| s.1[0] == 0.),
            "FC takeoff, landing or final disarm evidence missing",
        );
    }
    report
}

fn grade_groups(
    data: &Evidence,
    groups: &Groups,
    count: Option<u64>,
    board: bool,
    report: &mut Report,
) -> Result<(), String> {
    let mut session = Value::Null;
    let mut frame = None;
    let mut previous = None;
    let mut flights = Vec::new();
    for (i, (id, events)) in groups.iter().enumerate() {
        let settings = rows(events, "settings")
            .next()
            .map_or(&Value::Null, |s| &s.1);
        let home = rows(events, "home").next();
        let start = home.unwrap_or(&events[0]).0;
        let end = events
            .iter()
            .find(|e| e.1["name"] == "complete")
            .unwrap_or(&events[events.len() - 1])
            .0;
        let release_end = groups
            .iter()
            .find(|g| id.checked_add(1) == Some(g.0))
            .map_or(end + 0.5, |g| (end + 0.5).min(g.1[0].0));
        let part = FlightEvidence::new(data, events, start, end, release_end);
        let mut boundary = Report::default();
        if let Some((stamp, h)) = home {
            if i == 0 {
                session = h["session"].clone();
                if !board {
                    frame = flight::frame(part.poses, *stamp, h);
                }
            }
            if !board {
                boundary.check(frame.is_some(), "initial odometry alignment is missing");
            }
            boundary.check(
                !session.is_null() && h["session"] == session,
                "shared VIO session changed",
            );
        }
        if let Some(prev) = previous {
            let idle = data.scalar("/fc/debug/state", prev - 0.1, start);
            let mut last = prev - 0.1;
            let settled = idle.iter().all(|s| {
                let dt = s.0 - last;
                last = s.0;
                dt <= 0.35 && s.1[0] == 0.
            });
            boundary.check(
                start > prev && !idle.is_empty() && settled && start - last <= 0.35,
                "next flight requires a disarmed ground interval",
            );
        }
        let mut flight_report = if board {
            Report::default()
        } else {
            flight::grade(&part, frame, i == 0)?
        };
        let native = settings["mission_id"]
            .as_str()
            .is_some_and(|s| !s.is_empty());
        if board {
            boundary.check(native, "missing native mission settings");
        }
        let mut extra =
            json!({"flight_id":id,"mission_id":settings["mission_id"],"shape":settings["shape"]});
        if native {
            let life = lifecycle(&part, &session);
            for msg in &life.failures {
                if !flight_report.failures.contains(msg) {
                    flight_report.failures.push(msg.clone());
                }
            }
            extra["lifecycle"] = life.value();
            extra["phase"] = rows(events, "phase")
                .last()
                .map_or(Value::Null, |e| e.1["name"].clone());
            extra["ground_truth_available"] = json!(!part.poses.is_empty());
        }
        flight_report.failures.extend(boundary.failures);
        report.failures.extend(
            flight_report
                .failures
                .iter()
                .map(|m| format!("flight {id}: {m}")),
        );
        if groups.len() == 1 {
            report.metrics.extend(flight_report.metrics.clone());
        }
        let mut result = flight_report.value();
        result
            .as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        flights.push(result);
        previous = Some(end);
    }
    if let Some(n) = count {
        report.metrics.insert("planned_flights".into(), json!(n));
    }
    report.metrics.insert("vio_session".into(), session);
    report.metrics.insert("flights".into(), json!(flights));
    Ok(())
}

pub(super) fn evaluate(data: &Evidence, mode: Option<&str>) -> Result<Value, String> {
    for (name, series) in &data.scalars {
        let min = match name.as_str() {
            "/landing/target"
            | "/landing/runtime"
            | "/landing/quality"
            | "/fc/imu/quaternion_xyzw" => 4,
            "/fc/imu/health" => 3,
            _ => 1,
        };
        if series.iter().any(|sample| sample.1.len() < min) {
            return Err(format!("incomplete scalar row at {name}"));
        }
    }
    let board = mode == Some("board");
    let mut report = Report::default();
    let headers: Vec<_> = rows(&data.events, "system_run").collect();
    if board
        || data.events.iter().any(|s| s.1.get("mission_id").is_some())
        || headers.iter().any(|s| s.1.get("run").is_some())
    {
        if let Some(groups) = native_groups(data, &mut report) {
            grade_groups(data, &groups, None, board, &mut report)?;
        }
    } else if headers.is_empty() && !data.events.iter().any(|s| s.1.get("flight_id").is_some()) {
        report = flight::grade(
            &FlightEvidence::new(
                data,
                &data.events,
                f64::NEG_INFINITY,
                f64::INFINITY,
                f64::INFINITY,
            ),
            None,
            true,
        )?;
    } else if headers.len() == 1
        && let Some(count) = headers[0].1["planned_flights"].as_u64().filter(|c| *c > 0)
    {
        let groups = legacy_groups(data, count, &mut report);
        grade_groups(data, &groups, Some(count), false, &mut report)?;
    } else {
        report.check(false, "invalid system run declaration");
    }
    if board {
        report
            .metrics
            .insert("verification_scope".into(), json!("onboard_lifecycle"));
    } else if let Some(mode) = mode
        && data.events.iter().any(|e| e.1["precision_land"] == true)
    {
        let matched = data
            .scalars
            .get("/landing/runtime")
            .is_some_and(|v| !v.is_empty() && v.iter().all(|s| s.1[3] == f64::from(mode == "hil")));
        report.check(
            matched,
            format!("recorded inference backend differs from {mode} acceptance target"),
        );
    }
    Ok(report.value())
}
