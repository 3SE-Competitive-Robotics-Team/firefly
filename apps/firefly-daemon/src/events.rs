use crate::{
    bus::Bus,
    flight::{Flight, Phase, Receipt},
};
use serde_json::{Value, json};

pub fn record(bus: &Bus, receipt: &Receipt, event: Value) {
    record_at(bus, receipt, bus.odom.last_stamp(), event);
}

pub fn record_at(bus: &Bus, receipt: &Receipt, stamp: f64, mut event: Value) {
    event["run"] = json!(receipt.run);
    event["mission_id"] = json!(receipt.id);
    event["flight_id"] = json!(receipt.flight_number);
    // Recovery and shutdown can follow a loss of telemetry. Use the last measured
    // timestamp for the observation, including when it is too old to navigate.
    bus.log.event("flight", stamp, &format!("mission {event}"));
}

pub fn start(bus: &Bus, r: &Receipt, team: u32, stamp: f64) {
    let emit = |event| record_at(bus, r, stamp, event);
    emit(
        json!({"kind":"settings","mission":if r.plan.waypoints.is_empty(){"hover"}else{"roundtrip"},
        "team":if team==0{"red"}else{"blue"},"waypoint_count":r.plan.waypoints.len(),"precision_land":r.plan.precision_land,
        "altitude":r.plan.altitude,"hover_seconds":r.plan.hover_seconds,"return_offset":r.plan.return_offset}),
    );
    for (index, w) in r.plan.waypoints.iter().enumerate() {
        emit(json!({"kind":"waypoint","index":index,"offset":w.offset,"arrival":w.arrival}));
    }
    emit(
        json!({"kind":"home","position":r.home,"quaternion":r.home_quaternion,"session":r.session}),
    );
    emit(json!({"kind":"phase","name":r.phase}));
}

pub fn phase(bus: &Bus, f: &Flight) {
    let r = &f.record;
    let mut event = json!({"kind":"phase","name":r.phase});
    if matches!(r.phase, Phase::Navigate | Phase::Hold | Phase::Dwell) {
        event["index"] = json!(r.waypoint);
        event["goal"] = json!(f.point(r.waypoint));
    }
    if let Some(reason) = r.attention.as_ref().or(r.reason.as_ref()) {
        event["reason"] = json!(reason);
    }
    record(bus, r, event);
}
