use crate::{
    bus::Bus,
    flight::{Flight, Phase},
};
use serde_json::{Value, json};

pub fn record(bus: &Bus, id: &str, mut event: Value) {
    event["mission_id"] = json!(id);
    let stamp = bus.odom.fresh().map_or(-1., |p| p.timestamp);
    bus.log.event("flight", stamp, &format!("mission {event}"));
}
pub fn start(bus: &Bus, f: &Flight, team: u32) {
    let r = &f.record;
    record(
        bus,
        &r.id,
        json!({"kind":"settings","run":r.run,"mission":if r.plan.waypoints.is_empty(){"hover"}else{"roundtrip"},
        "team":if team==0{"red"}else{"blue"},"waypoint_count":r.plan.waypoints.len(),"precision_land":r.plan.precision_land,
        "altitude":r.plan.altitude,"hover_seconds":r.plan.hover_seconds,"return_offset":&r.plan.return_point()[..2]}),
    );
    for (index, w) in r.plan.waypoints.iter().enumerate() {
        record(
            bus,
            &r.id,
            json!({"kind":"waypoint","index":index,"offset":w.offset,"arrival":w.arrival}),
        );
    }
    record(
        bus,
        &r.id,
        json!({"kind":"home","position":r.home,"quaternion":r.home_quaternion,"session":r.session}),
    );
    phase(bus, f);
}
pub fn phase(bus: &Bus, f: &Flight) {
    let r = &f.record;
    let index = r.waypoint;
    let event = match r.phase {
        Phase::Arm => json!({"name":"takeoff"}),
        Phase::Navigate => {
            json!({"name":if index<r.plan.waypoints.len(){"waypoint"}else{"return"},"index":index,"goal":f.point(index)})
        }
        Phase::Dwell if index < r.plan.waypoints.len() => {
            json!({"name":"waypoint_hover","index":index,"goal":f.point(index)})
        }
        Phase::Acquire => json!({"name":"acquire"}),
        Phase::Land => json!({"name":"land"}),
        Phase::Complete => json!({"name":"complete"}),
        Phase::Recover => {
            record(bus, &r.id, json!({"kind":"failure","reason":r.reason}));
            return;
        }
        Phase::NeedsOperator => {
            record(
                bus,
                &r.id,
                json!({"kind":"operator_required","reason":r.attention}),
            );
            return;
        }
        _ => return,
    };
    let mut event = event;
    event["kind"] = json!("phase");
    record(bus, &r.id, event);
}
