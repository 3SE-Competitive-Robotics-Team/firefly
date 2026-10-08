//! 飞控反馈固定在 odom 系；同时间戳的地图位姿仅用于建立 map←odom 边。
use firefly_base::{FrameId, FrameTree, RigidTransform};
use firefly_flight::PositionSetpoint;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::reference::ReferenceMessage;
use nalgebra::{Point3, Vector3};
use std::collections::VecDeque;
use std::time::{Duration, Instant};

#[derive(Default)]
pub struct ControlFrames {
    tree: FrameTree,
    history: VecDeque<(f64, RigidTransform, Instant)>,
    corrected: Option<(OdomMessage, Instant)>,
    alignment_at: Option<Instant>,
    alignment_time: Option<f64>,
}
impl ControlFrames {
    pub fn observe_odom(&mut self, msg: &OdomMessage, now: Instant) -> bool {
        if !valid(msg)
            || self
                .history
                .back()
                .is_some_and(|(time, _, _)| msg.timestamp <= *time)
        {
            return false;
        }
        let Ok(pose) = msg.body_pose(FrameId::ODOM) else {
            return false;
        };
        if self.tree.set(pose).is_err() {
            return false;
        }
        self.history.push_back((msg.timestamp, pose, now));
        while self.history.len() > 256
            || self
                .history
                .front()
                .is_some_and(|(t, _, _)| msg.timestamp - *t > 2.0)
        {
            self.history.pop_front();
        }
        self.update_alignment();
        true
    }
    pub fn observe_corrected(&mut self, msg: OdomMessage, now: Instant) {
        if !valid(&msg) || !msg.is_initialized || msg.body_pose(FrameId::MAP).is_err() {
            self.alignment_at = None;
            return;
        }
        if self
            .corrected
            .as_ref()
            .is_none_or(|(old, _)| msg.timestamp > old.timestamp)
        {
            self.corrected = Some((msg, now));
            self.update_alignment();
        }
    }
    fn update_alignment(&mut self) {
        let Some((msg, received)) = self.corrected else {
            return;
        };
        if self
            .alignment_time
            .is_some_and(|time| msg.timestamp <= time)
        {
            return;
        }
        let Some((_, raw, raw_received)) = self
            .history
            .iter()
            .find(|(t, _, _)| (*t - msg.timestamp).abs() < 1e-9)
        else {
            return;
        };
        let Ok(map_body) = msg.body_pose(FrameId::MAP) else {
            return;
        };
        let Ok(map_odom) = map_body.compose(&raw.inverse()) else {
            return;
        };
        if self.tree.set(map_odom).is_ok() {
            self.alignment_at = Some(received.min(*raw_received));
            self.alignment_time = Some(msg.timestamp);
        }
    }
    /// `dt`：参考时刻到当前时刻的**仿真秒**差（调用方由 IMU 时刻给出），用于把
    /// 10Hz 的阶跃参考外推到当前时刻；限幅由调用方负责。
    pub fn reference(
        &self,
        msg: &ReferenceMessage,
        now: Instant,
        max_age: Duration,
        dt: f64,
    ) -> Option<PositionSetpoint> {
        if now.duration_since(self.alignment_at?) > max_age {
            return None;
        }
        let transform = self.tree.lookup(FrameId::ODOM, FrameId::MAP).ok()?;
        let position = transform.point(Point3::new(msg.position_x, msg.position_y, msg.position_z));
        let velocity =
            transform.vector(Vector3::new(msg.velocity_x, msg.velocity_y, msg.velocity_z));
        let acceleration = transform.vector(Vector3::new(
            msg.acceleration_x,
            msg.acceleration_y,
            msg.acceleration_z,
        ));
        // 设定点外推到当前时刻：p += v·dt + ½a·dt²，v += a·dt（刚体变换下与先
        // 变换后外推等价）。
        let position = position + velocity * dt + 0.5 * acceleration * dt * dt;
        let velocity = velocity + acceleration * dt;
        let direction = transform.vector(Vector3::new(msg.yaw.cos(), msg.yaw.sin(), 0.));
        let derivative =
            transform.vector(Vector3::new(-msg.yaw.sin(), msg.yaw.cos(), 0.)) * msg.yaw_dot;
        let horizontal = direction.x * direction.x + direction.y * direction.y;
        if horizontal < 1e-12 {
            return None;
        }
        let result = PositionSetpoint {
            position: glam::Vec3::new(position.x as f32, position.y as f32, position.z as f32),
            velocity: glam::Vec3::new(velocity.x as f32, velocity.y as f32, velocity.z as f32),
            acceleration: glam::Vec3::new(
                acceleration.x as f32,
                acceleration.y as f32,
                acceleration.z as f32,
            ),
            yaw: direction.y.atan2(direction.x) as f32,
            yaw_rate: ((direction.x * derivative.y - direction.y * derivative.x) / horizontal)
                as f32,
        };
        (result.position.is_finite()
            && result.velocity.is_finite()
            && result.yaw.is_finite()
            && result.yaw_rate.is_finite())
        .then_some(result)
    }
}
fn valid(msg: &OdomMessage) -> bool {
    msg.timestamp >= 0.
        && [
            msg.timestamp,
            msg.velocity_x,
            msg.velocity_y,
            msg.velocity_z,
        ]
        .iter()
        .all(|v| v.is_finite())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn raw(t: f64) -> OdomMessage {
        OdomMessage {
            timestamp: t,
            is_initialized: true,
            ..Default::default()
        }
    }
    #[test]
    fn map_reference_is_converted_without_switching_feedback() {
        let now = Instant::now();
        let mut frames = ControlFrames::default();
        let raw = raw(1.);
        let q = std::f64::consts::FRAC_PI_4;
        frames.observe_odom(&raw, now);
        frames.observe_corrected(
            OdomMessage {
                position_x: 10.,
                quat_z: q.sin(),
                quat_w: q.cos(),
                ..raw
            },
            now,
        );
        let reference = ReferenceMessage {
            position_x: 10.,
            position_y: 2.,
            velocity_y: 1.,
            yaw: std::f64::consts::FRAC_PI_2,
            yaw_dot: 0.3,
            ..Default::default()
        };
        let r = frames
            .reference(&reference, now, Duration::from_millis(500), 0.0)
            .unwrap();
        assert!((r.position - glam::Vec3::new(2., 0., 0.)).length() < 1e-6);
        assert!((r.velocity - glam::Vec3::X).length() < 1e-6);
        assert!(r.yaw.abs() < 1e-6 && (r.yaw_rate - 0.3).abs() < 1e-6);
        assert!(
            frames
                .reference(
                    &reference,
                    now + Duration::from_secs(1),
                    Duration::from_millis(500),
                    0.0
                )
                .is_none()
        );
    }
    /// dt 外推：`p += v·dt + ½a·dt²`、`v += a·dt`。
    #[test]
    fn reference_setpoint_is_extrapolated_by_dt() {
        let now = Instant::now();
        let mut frames = ControlFrames::default();
        let raw = raw(1.);
        frames.observe_odom(&raw, now);
        frames.observe_corrected(raw, now);
        let reference = ReferenceMessage {
            position_x: 1.,
            velocity_x: 2.,
            acceleration_x: 3.,
            ..Default::default()
        };
        let dt = 0.5;
        let r = frames
            .reference(&reference, now, Duration::from_millis(500), dt)
            .unwrap();
        // 1 + 2·0.5 + ½·3·0.25 = 2.375；2 + 3·0.5 = 3.5
        assert!((r.position.x - 2.375).abs() < 1e-6, "{:?}", r.position);
        assert!((r.velocity.x - 3.5).abs() < 1e-6, "{:?}", r.velocity);
        assert!(
            (r.acceleration.x - 3.0).abs() < 1e-6,
            "{:?}",
            r.acceleration
        );
        let r0 = frames
            .reference(&reference, now, Duration::from_millis(500), 0.0)
            .unwrap();
        assert!((r0.position.x - 1.0).abs() < 1e-6 && (r0.velocity.x - 2.0).abs() < 1e-6);
    }

    #[test]
    fn unsynchronized_and_repeated_samples_cannot_refresh_alignment() {
        let now = Instant::now();
        let mut frames = ControlFrames::default();
        frames.observe_corrected(raw(2.), now);
        frames.observe_odom(&raw(1.), now);
        assert!(frames.alignment_at.is_none());
        frames.observe_odom(&raw(2.), now);
        assert_eq!(frames.alignment_at, Some(now));
        frames.observe_corrected(raw(2.), now + Duration::from_secs(1));
        assert_eq!(frames.alignment_at, Some(now));
    }
}
