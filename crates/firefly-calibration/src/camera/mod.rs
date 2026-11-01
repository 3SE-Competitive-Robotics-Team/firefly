//! Camera models from the pinned original Basalt headers.
pub mod bal_camera;
pub mod double_sphere_camera;
pub mod extended_camera;
pub mod fov_camera;
pub mod generic_camera;
pub mod kannala_brandt_camera4;
pub mod pinhole_camera;
pub mod pinhole_radtan8_camera;
pub mod stereographic_param;
pub mod unified_camera;

fn clamp<S: crate::scalar::BasaltScalar>(value: S, low: S, high: S) -> S {
    if value < low {
        low
    } else if high < value {
        high
    } else {
        value
    }
}
