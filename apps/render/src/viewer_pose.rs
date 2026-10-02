//! 主视图专用延迟插值；传感器位姿和测量时间戳不得使用此平滑结果。
use bevy::prelude::*;

#[derive(Resource, Default)]
pub struct ViewerPose {
    previous: Option<Transform>,
    current: Option<Transform>,
    stamp: f64,
    received: f64,
    interval: f64,
}

impl ViewerPose {
    pub fn observe(&mut self, pose: Transform, stamp: f64, wall: f64) {
        if self.current.is_some() && stamp <= self.stamp {
            return;
        }
        self.previous = self.current.or(Some(pose));
        self.current = Some(pose);
        self.interval = (wall - self.received).clamp(0.001, 0.2);
        self.received = wall;
        self.stamp = stamp;
    }

    pub fn sample(&self, wall: f64) -> Option<Transform> {
        let (previous, current) = (self.previous?, self.current?);
        let alpha = ((wall - self.received) / self.interval).clamp(0.0, 1.0) as f32;
        Some(Transform {
            translation: previous.translation.lerp(current.translation, alpha),
            rotation: previous.rotation.slerp(current.rotation, alpha),
            ..default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn interpolate_display_only_without_extrapolation_or_reordered_samples() {
        let mut state = ViewerPose::default();
        state.observe(Transform::IDENTITY, 1.0, 1.0);
        state.observe(Transform::from_translation(Vec3::X), 1.1, 1.1);
        assert!((state.sample(1.15).unwrap().translation.x - 0.5).abs() < 1e-5);
        assert_eq!(state.sample(2.0).unwrap().translation, Vec3::X);
        state.observe(Transform::from_translation(Vec3::Y), 1.0, 2.0);
        assert_eq!(state.sample(2.0).unwrap().translation, Vec3::X);
    }
}
