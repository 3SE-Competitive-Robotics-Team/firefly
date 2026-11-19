use super::{Event, FlightEvidence, Report, Value, Vec3};
use std::collections::BTreeMap;
#[derive(Clone)]
pub(super) struct Phase {
    pub timestamp: f64,
    pub goal: Option<Vec3>,
    pub index: Option<u64>,
}
pub(super) struct Leg {
    pub travel: Phase,
    pub hold: Option<Phase>,
    pub following: Phase,
}
pub(super) struct Timeline {
    pub settings: Value,
    pub home: Value,
    pub start: f64,
    pub phases: BTreeMap<&'static str, Phase>,
    pub legs: Vec<Leg>,
}

pub(super) fn vec3(v: &Value) -> Option<Vec3> {
    let [x, y, z] = v.as_array()?.as_slice() else {
        return None;
    };
    Some(Vec3::new(x.as_f64()?, y.as_f64()?, z.as_f64()?))
}

pub(super) fn number(v: &Value) -> f64 {
    v.as_f64().unwrap_or(f64::NAN)
}

pub(super) fn rows<'a>(events: &'a [Event], kind: &'a str) -> impl Iterator<Item = &'a Event> {
    events.iter().filter(move |(_, row)| row["kind"] == kind)
}

type PhaseLayout = Vec<(&'static str, Option<&'static str>)>;
fn layout(native: bool, round: bool, precision: bool, arrivals: &[Option<&str>]) -> PhaseLayout {
    let mut layout = Vec::new();
    if native {
        layout.push(("arm", None));
    }
    layout.push(("takeoff", Some("takeoff")));
    if native {
        layout.push(("hover", None));
    }
    if round {
        for arrival in arrivals {
            layout.push((
                if native { "navigate" } else { "waypoint" },
                Some("waypoint"),
            ));
            if *arrival == Some("stop") {
                if native {
                    layout.push(("hold", None));
                }
                layout.push((
                    if native { "dwell" } else { "waypoint_hover" },
                    Some("waypoint_hover"),
                ));
            }
        }
        layout.push((if native { "navigate" } else { "return" }, Some("return")));
        if native {
            layout.extend([("hold", None), ("dwell", None)]);
        }
    }
    if precision {
        layout.push(("acquire", Some("acquire")));
    }
    layout.push(("land", Some("land")));
    if native {
        layout.push(("ground", None));
    }
    layout.push(("complete", Some("complete")));
    layout
}

fn observed_windows(
    layout: PhaseLayout,
    observed: &[&Event],
    arrivals: &[Option<&str>],
    native: bool,
    report: &mut Report,
) -> Option<Vec<(&'static str, Phase)>> {
    if !layout
        .iter()
        .map(|(n, _)| Some(*n))
        .eq(observed.iter().map(|e| e.1["name"].as_str()))
    {
        report.check(
            false,
            if native {
                "native phase sequence is incomplete or out of order"
            } else {
                "mission phase sequence is incomplete or out of order"
            },
        );
        return None;
    }
    let mut windows = Vec::new();
    let mut cursor = 0;
    for ((name, key), (stamp, row)) in layout.into_iter().zip(observed) {
        let p = Phase {
            timestamp: *stamp,
            index: row["index"].as_u64(),
            goal: vec3(&row["goal"]),
        };
        if native && matches!(name, "navigate" | "hold" | "dwell") {
            report.check(
                p.index == Some(cursor as u64),
                "phase refers to the wrong waypoint",
            );
            if name == "dwell"
                || (name == "navigate" && arrivals.get(cursor) == Some(&Some("pass")))
            {
                cursor += 1;
            }
        }
        if let Some(key) = key {
            windows.push((key, p));
        }
    }
    Some(windows)
}

fn waypoint_legs(
    timeline: &Timeline,
    planned: &[&Event],
    arrivals: &[Option<&str>],
    windows: &[Phase],
    report: &mut Report,
) -> Option<Vec<Leg>> {
    let settings = &timeline.settings;
    let home = &timeline.home;
    let count = settings["waypoint_count"].as_u64();
    let start = timeline.start;
    let phases = &timeline.phases;
    let native = settings.get("mission_id").is_some() && settings.get("flight_id").is_some();
    let mut legs = Vec::new();
    report.check(
        planned.len() as u64 == count?
            && planned
                .iter()
                .enumerate()
                .all(|(i, e)| e.1["index"].as_u64() == Some(i as u64)),
        "ordered waypoint declarations are missing or duplicated",
    );
    if planned.len() as u64 != count? {
        return None;
    }
    let origin = vec3(&home["position"]);
    let altitude = number(&settings["altitude"]);
    let mut cursor = 1;
    for ((i, (stamp, w)), arrival) in planned.iter().enumerate().zip(arrivals) {
        let travel = windows[cursor].clone();
        cursor += 1;
        let hold = if *arrival == Some("stop") {
            cursor += 1;
            Some(windows[cursor - 1].clone())
        } else {
            None
        };
        let goal = match w["offset"].as_array().map(Vec::as_slice) {
            Some([x, y]) => Some(Vec3::new(number(x), number(y), altitude)),
            _ => vec3(&w["offset"]),
        };
        let ceiling = if native || settings["shape"] == "cuboid" {
            3.3
        } else {
            altitude
        };
        report.check(
            goal.is_some_and(|v| v.iter().all(|x| x.is_finite()) && (0.5..=ceiling).contains(&v.z)),
            "invalid declared waypoint offset",
        );
        let target = origin.zip(goal).map(|(p, v)| p + v);
        let matches = |p: &Phase| p.index == Some(i as u64) && target.is_some() && p.goal == target;
        report.check(
            *stamp <= start && matches(&travel) && hold.as_ref().is_none_or(matches),
            format!("waypoint {i}: phase identity/goal differs from the declared route"),
        );
        legs.push(Leg {
            travel,
            hold,
            following: windows[cursor].clone(),
        });
    }
    let offset = &settings["return_offset"];
    let dest = origin.map(|o| o + Vec3::new(number(&offset[0]), number(&offset[1]), altitude));
    report.check(
        dest.is_some() && dest == phases["return"].goal,
        "return goal differs from the takeoff-frame approach point",
    );
    Some(legs)
}

impl Timeline {
    pub fn end(&self) -> f64 {
        self.phases["complete"].timestamp
    }
    pub fn read(data: &FlightEvidence<'_>, report: &mut Report) -> Option<Self> {
        let settings: Vec<_> = rows(data.events, "settings").collect();
        let homes: Vec<_> = rows(data.events, "home").collect();
        report.check(
            data.source.recording_complete,
            "Mimir did not confirm complete recording integrity",
        );
        report.check(
            settings.len() == 1 && homes.len() == 1,
            "one settings event and one takeoff-frame anchor required",
        );
        report.check(
            rows(data.events, "failure").next().is_none(),
            "mission reported a failure",
        );
        let mut settings = settings.first()?.1.clone();
        let (start, home) = (*homes.first()?).clone();
        let round = settings["mission"] == "roundtrip";
        report.check(
            round || settings["mission"] == "hover",
            "unknown mission kind",
        );
        if !report.failures.is_empty() {
            return None;
        }
        let count = settings["waypoint_count"].as_u64();
        report.check(
            if round {
                count.is_some_and(|n| (1..=16).contains(&n))
            } else {
                count == Some(0)
            },
            "invalid waypoint count",
        );
        if !report.failures.is_empty() {
            return None;
        }
        let planned: Vec<_> = rows(data.events, "waypoint").collect();
        let arrivals: Vec<_> = planned
            .iter()
            .map(|e| e.1.get("arrival").map_or(Some("stop"), Value::as_str))
            .collect();
        report.check(
            arrivals.iter().all(|a| matches!(a, Some("pass" | "stop"))),
            "invalid waypoint arrival mode",
        );
        let native = settings.get("mission_id").is_some() && settings.get("flight_id").is_some();
        let precision = settings["precision_land"] == true;
        let observed: Vec<_> = rows(data.events, "phase").collect();
        if native {
            if let Some((_, last)) = observed.last()
                && let Some(name @ ("recover" | "needs_operator" | "interrupted")) =
                    last["name"].as_str()
            {
                report.check(
                    false,
                    format!(
                        "mission ended in {name}: {}",
                        last["reason"].as_str().unwrap_or("reason missing")
                    ),
                );
                return None;
            }
            if let Some(cfg) = data.source.capture.as_object() {
                settings.as_object_mut()?.extend(cfg.clone());
            }
        }
        let decoded = observed_windows(
            layout(native, round, precision, &arrivals),
            &observed,
            &arrivals,
            native,
            report,
        )?;
        let phases = decoded.iter().cloned().collect();
        let windows: Vec<_> = decoded.into_iter().map(|(_, phase)| phase).collect();
        let mut timeline = Self {
            settings,
            home,
            start,
            phases,
            legs: Vec::new(),
        };
        if round {
            timeline.legs = waypoint_legs(&timeline, &planned, &arrivals, &windows, report)?;
        }
        report.check(
            windows.windows(2).all(|w| w[1].timestamp > w[0].timestamp),
            "phase timestamps must increase",
        );
        report.failures.is_empty().then_some(timeline)
    }
}
