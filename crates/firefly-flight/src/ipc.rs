use crate::{Airframe, QuadParams, Rotor};
use firefly_pubsub::plant::AirframeMessage;

/// Interpret the plant's rotor geometry and physical parameters in body FLU.
#[must_use]
pub fn airframe_to_params(msg: &AirframeMessage) -> (QuadParams, Airframe) {
    let quad = QuadParams {
        mass: msg.mass as f32,
        inertia: [
            msg.inertia_x as f32,
            msg.inertia_y as f32,
            msg.inertia_z as f32,
        ],
        linear_drag: msg.linear_drag as f32,
        angular_drag: msg.angular_drag as f32,
    };
    let airframe = Airframe {
        rotors: std::array::from_fn(|i| Rotor {
            position: msg.rotor_positions[i].map(|v| v as f32),
            spin: msg.rotor_spins[i] as f32,
        }),
        max_thrust_per_motor: msg.max_thrust_per_motor as f32,
        torque_coefficient: msg.torque_coefficient as f32,
    };
    (quad, airframe)
}
