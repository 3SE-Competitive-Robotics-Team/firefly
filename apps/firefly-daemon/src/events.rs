use crate::{
    bus::Bus,
    flight::{Mission, Receipt},
};
use firefly_mission::Phase;
use serde_json::{Value, json};

pub fn record(bus: &Bus, mission: &Mission, event: Value) {
    record_at(bus, mission, bus.odom.last_stamp(), event);
}

pub fn record_at(bus: &Bus, mission: &Mission, stamp: f64, mut event: Value) {
    event["run"] = json!(mission.run);
    event["mission_id"] = json!(mission.id);
    event["flight_id"] = json!(mission.flight_number);
    // Recovery and shutdown can follow a loss of telemetry. Use the last measured
    // timestamp for the observation, including when it is too old to navigate.
    bus.log.event("flight", stamp, &format!("mission {event}"));
}

pub fn start(bus: &Bus, r: &Receipt, team: u32, stamp: f64) {
    let emit = |event| record_at(bus, &r.mission, stamp, event);
    emit(
        json!({"kind":"settings","mission":if r.mission.plan.waypoints.is_empty(){"hover"}else{"roundtrip"},
        "team":if team==0{"red"}else{"blue"},"waypoint_count":r.mission.plan.waypoints.len(),"precision_land":r.mission.plan.precision_land,
        "altitude":r.mission.plan.altitude,"hover_seconds":r.mission.plan.hover_seconds,"return_offset":r.mission.plan.return_offset}),
    );
    for (index, w) in r.mission.plan.waypoints.iter().enumerate() {
        emit(json!({"kind":"waypoint","index":index,"offset":w.offset,"arrival":w.arrival}));
    }
    emit(
        json!({"kind":"home","position":r.mission.home,"quaternion":r.mission.home_quaternion,"session":r.mission.session}),
    );
    emit(json!({"kind":"phase","name":r.phase}));
}

pub fn phase(bus: &Bus, r: &Receipt, stamp: f64) {
    let mut event = json!({"kind":"phase","name":r.phase});
    if matches!(r.phase, Phase::Navigate | Phase::Hold | Phase::Dwell) {
        event["index"] = json!(r.mission.waypoint);
        event["goal"] = json!(r.mission.point(r.mission.waypoint));
    }
    if let Some(reason) = r.mission.attention.as_ref().or(r.mission.reason.as_ref()) {
        event["reason"] = json!(reason);
    }
    record_at(bus, &r.mission, stamp, event);
}
