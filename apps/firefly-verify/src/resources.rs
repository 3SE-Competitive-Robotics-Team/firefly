use super::{
    FlightEvidence, Report, json,
    timeline::{Timeline, number, rows},
    window,
};

pub(super) fn images(data: &FlightEvidence<'_>, timeline: &Timeline, report: &mut Report) {
    let (begin_time, end_time) = (timeline.start, timeline.end());
    let frames = window(data.down_frames, begin_time, end_time, |s| s.0);
    report.check(
        frames.len() >= 3,
        "down-camera images were not recorded; enable mimir.images",
    );
    report.check(
        frames
            .iter()
            .all(|&(_, w, h, n)| (w, h, n) == (352, 288, 352 * 288)),
        "recorded down-camera images must be lossless CIF grayscale",
    );
    let (Some(first), Some(last)) = (frames.first(), frames.last()) else {
        return;
    };
    let span = last.0 - first.0;
    let n = frames.len() - 1;
    report
        .metrics
        .insert("down_frames".into(), json!(frames.len()));
    let mut min_gap = f64::INFINITY;
    let mut max_gap = 0_f64;
    for pair in frames.windows(2) {
        let gap = pair[1].0 - pair[0].0;
        if !gap.is_finite() || gap <= 0. {
            report.check(false, "down-camera timestamps must increase");
            return;
        }
        min_gap = min_gap.min(gap);
        max_gap = max_gap.max(gap);
    }
    report
        .metrics
        .insert("max_down_gap_s".into(), json!(max_gap));
    report.metrics.insert(
        "down_fps".into(),
        json!(if span > 0. { n as f64 / span } else { 0. }),
    );
    report.check(
        first.0 <= begin_time + 0.5 && last.0 >= end_time - 0.5 && max_gap <= 0.5,
        "continuous down-camera recording does not cover the mission",
    );
    let rate = number(&timeline.settings["down_rate_hz"]);
    if rate != 15. && rate != 30. {
        report.check(false, "invalid requested down-camera rate");
        return;
    }
    report.check(
        min_gap >= 1. / rate - 0.001_000_001,
        "down-camera exceeds the requested capture rate",
    );
    let long_term_rate_ok = n as f64 <= (span + 0.002) * rate;
    report.metrics.insert(
        "down_rate_limit_hz".into(),
        timeline.settings["down_rate_hz"].clone(),
    );
    landing_worker(data, timeline, report, long_term_rate_ok);
}

fn landing_worker(
    data: &FlightEvidence<'_>,
    timeline: &Timeline,
    report: &mut Report,
    long_term_rate_ok: bool,
) {
    let (begin_time, end_time) = (timeline.start, timeline.end());
    let requests: Vec<_> = rows(data.events, "activity")
        .filter(|e| begin_time <= e.0 && e.0 <= end_time)
        .collect();
    report.check(
        !requests.is_empty(),
        "resource lifecycle requests are missing",
    );
    let mut active = false;
    let mut since = begin_time;
    let mut intervals = Vec::new();
    for (stamp, row) in requests {
        let enabled = row["enabled"] == true;
        if enabled != active {
            if *stamp > since {
                intervals.push((since, *stamp, active));
            }
            active = enabled;
            since = *stamp;
        }
    }
    if since < end_time {
        intervals.push((since, end_time, active));
    }
    report.check(
        intervals.iter().filter(|i| i.2).count() >= 2,
        "takeoff and landing require separate activations",
    );
    report.check(
        long_term_rate_ok,
        "down-camera long-term rate exceeds the configured rate",
    );
    let runtime = data.scalar("/landing/runtime", f64::NEG_INFINITY, f64::INFINITY);
    report.check(!runtime.is_empty(), "landing lifecycle evidence is missing");
    let backend = timeline.settings["transport"].as_str().map_or_else(
        || runtime.first().map(|s| s.1[3]),
        |s| Some(f64::from(s == "hil")),
    );
    report.check(
        backend.is_some_and(|v| (v == 0. || v == 1.) && runtime.iter().all(|s| s.1[3] == v)),
        "landing backend does not match SIL/HIL execution",
    );
    for &(start, end, _) in intervals.iter().filter(|i| i.2) {
        report.check(
            window(runtime, start, end, |s| s.0)
                .iter()
                .any(|s| s.1[0] == 1.),
            "landing pipeline was not active during a requested window",
        );
    }
    let idle = |s: &super::Scalar| s.1[0] == 0. && s.1[2] == 0.;
    let released = window(runtime, since, since + 0.5, |s| s.0)
        .iter()
        .find(|s| idle(s));
    report.check(
        !active
            && since >= timeline.phases["land"].timestamp
            && released.is_some_and(|s| {
                window(runtime, s.0, end_time + 0.5, |s| s.0)
                    .iter()
                    .all(idle)
            }),
        "landing resources were not released at mission completion",
    );
    for (begin, end, _) in intervals.into_iter().filter(|i| !i.2 && i.1 - i.0 > 0.5) {
        let begin = begin + 0.5;
        let idle_rows: Vec<_> = runtime
            .iter()
            .filter(|s| begin <= s.0 && s.0 < end)
            .collect();
        let min = if end - begin >= 3. {
            3
        } else {
            usize::from(end - begin >= 1.)
        };
        report.check(
            idle_rows.len() >= min && idle_rows.iter().all(|s| idle(s)),
            "landing pipeline did not suspend during idle time",
        );
        report.check(
            idle_rows.windows(2).all(|w| w[0].1[1] == w[1].1[1]),
            "landing inference continued during idle time",
        );
    }
    let counts = window(runtime, begin_time, end_time + 0.5, |s| s.0);
    let range = counts.first().map_or(0., |first| {
        let (lo, hi) = counts.iter().fold((first.1[1], first.1[1]), |(lo, hi), s| {
            (lo.min(s.1[1]), hi.max(s.1[1]))
        });
        hi - lo
    });
    report
        .metrics
        .insert("landing_inferences".into(), json!(range));
}

pub(super) fn precision(data: &FlightEvidence<'_>, timeline: &Timeline, report: &mut Report) {
    let begin_time = timeline.phases["acquire"].timestamp;
    let end_time = timeline.phases["land"].timestamp;
    let quality = data.scalar("/landing/quality", begin_time, end_time);
    let valid: Vec<_> = data
        .scalar("/landing/target", begin_time, end_time)
        .iter()
        .filter_map(|(stamp, v)| {
            if *stamp <= begin_time || v[3] != 1. || !v[..3].iter().all(|x| x.is_finite()) {
                return None;
            }
            let (_, q) = window(quality, *stamp, *stamp, |s| s.0).last()?;
            (q[0] == number(&timeline.home["session"])
                && q[1] == f64::from(timeline.settings["team"] == "blue")
                && (0.0..=2.).contains(&q[2])
                && (0.5..=1.).contains(&q[3]))
            .then_some(*stamp)
        })
        .collect();
    let distinct =
        usize::from(!valid.is_empty()) + valid.windows(2).filter(|w| w[0] != w[1]).count();
    report.check(
        distinct >= 3
            && ((valid[valid.len() - 1] - valid[0]) * 1e9).round_ties_even() >= 150_000_000.,
        "three fresh, same-session/team visual observations over 0.15s required after return",
    );
    report.check(
        valid
            .last()
            .is_some_and(|last| (0.0..=0.25).contains(&(end_time - last))),
        "landing began with a stale visual target",
    );
    let states = data.scalar("/fc/debug/precision", end_time, timeline.end());
    let steps = [1., 2., 3.].map(|code| states.iter().position(|s| s.1[0] == code));
    report.check(
        matches!(steps,[Some(i),Some(j),Some(k)] if i<j && j<k),
        "ordered Align, Descend and Final precision stages required",
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Evidence, timeline::Phase};

    #[test]
    fn nonfinite_coordinates_do_not_confirm_acquisition() {
        let phases = [("acquire", 0.), ("land", 0.4), ("complete", 1.)].map(|(name, timestamp)| {
            (
                name,
                Phase {
                    timestamp,
                    goal: None,
                    index: None,
                },
            )
        });
        let timeline = Timeline {
            settings: json!({"team":"red"}),
            home: json!({"session":7}),
            start: 0.,
            phases: phases.into(),
            legs: vec![],
        };
        let mut data = Evidence::default();
        let stamps = [0.1, 0.2, 0.3];
        data.scalars.insert(
            "/landing/quality".into(),
            stamps.map(|t| (t, vec![7., 0., 0.2, 0.9])).into(),
        );
        data.scalars.insert(
            "/fc/debug/precision".into(),
            vec![(0.5, vec![1.]), (0.6, vec![2.]), (0.7, vec![3.])],
        );
        for (axis, value) in [
            (0, 1.),
            (0, f64::NAN),
            (1, f64::INFINITY),
            (2, f64::NEG_INFINITY),
        ] {
            let mut targets = stamps.map(|t| (t, vec![0., 0., 0., 1.]));
            targets[1].1[axis] = value;
            data.scalars
                .insert("/landing/target".into(), targets.into());
            let mut report = Report::default();
            precision(
                &FlightEvidence::new(&data, &[], 0., 1., 1.),
                &timeline,
                &mut report,
            );
            if value.is_finite() {
                assert!(report.failures.is_empty(), "{:?}", report.failures);
            } else {
                assert_eq!(report.failures.len(), 1);
                assert!(report.failures[0].contains("three fresh"));
            }
        }
    }
}
