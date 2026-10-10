//! 地图系飞行对照：真值轨迹与融合定位影子只进入调试视图。

use std::collections::VecDeque;

use bevy::camera::visibility::RenderLayers;
use bevy::prelude::*;
use bevy::text::FontSize;
use firefly_base::FrameId;
use firefly_pubsub::odom::OdomMessage;

use crate::{link::IpcPorts, rig::PATH_LAYER, viewer_pose::ViewerPose};

/// 每条轨迹最多保留 8192 个点；连续流按最高 20Hz 采样，断流恢复立即追加。
const CAPACITY: usize = 8192;
const TRAIL_PERIOD: f64 = 0.05;
/// 墙钟失联或测量时间间断超过 500ms 时隐藏影子、断开轨迹。
const STALE_SECS: f64 = 0.5;
const ACTUAL_COLOR: Color = Color::srgb(0.0, 0.9, 1.0);
const ESTIMATE_COLOR: Color = Color::srgb(1.0, 0.45, 0.05);

#[derive(Default, Reflect, GizmoConfigGroup)]
pub struct FlightGizmos;

#[derive(Resource, Default)]
pub struct FlightOverlay {
    pub actual: Track,
    estimate: Track,
}

struct TrailPoint {
    stamp: f64,
    position: Vec3,
    connected: bool,
}

/// 单会话单时钟轨迹；新会话需要重启 render，乱序样本不能触发时钟重置。
#[derive(Default)]
pub struct Track {
    points: VecDeque<TrailPoint>,
    stamp: Option<f64>,
    received: f64,
    valid: bool,
    viewer: ViewerPose,
}

impl Track {
    pub fn observe(&mut self, msg: &OdomMessage, wall: f64) {
        if !msg.timestamp.is_finite()
            || msg.timestamp < 0.0
            || self.stamp.is_some_and(|stamp| msg.timestamp <= stamp)
        {
            return;
        }
        let connected = self.valid
            && wall - self.received <= STALE_SECS
            && self
                .stamp
                .is_some_and(|stamp| msg.timestamp - stamp <= STALE_SECS);
        self.stamp = Some(msg.timestamp);
        self.received = wall;
        self.valid = false;
        let Ok(pose) = msg.body_pose(FrameId::MAP) else {
            return;
        };
        if !msg.is_initialized {
            return;
        }
        let iso = pose.isometry();
        let p = &iso.translation.vector;
        let q = iso.rotation.quaternion();
        let display = Transform {
            translation: Vec3::new(p.x as f32, p.y as f32, p.z as f32),
            rotation: Quat::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32),
            ..default()
        };
        if !display.translation.is_finite() || !display.rotation.is_finite() {
            return;
        }
        if !connected {
            self.viewer = ViewerPose::default();
        }
        self.viewer.observe(display, msg.timestamp, wall);
        self.valid = true;
        if !connected
            || self
                .points
                .back()
                .is_none_or(|p| msg.timestamp - p.stamp >= TRAIL_PERIOD)
        {
            if self.points.len() == CAPACITY {
                self.points.pop_front();
            }
            self.points.push_back(TrailPoint {
                stamp: msg.timestamp,
                position: display.translation,
                connected,
            });
        }
    }

    fn sample(&self, wall: f64) -> Option<Transform> {
        if !self.valid || wall - self.received > STALE_SECS {
            return None;
        }
        self.viewer.sample(wall)
    }
}

#[derive(Component)]
pub struct EstimateLabel;

#[allow(clippy::needless_pass_by_value)]
pub fn setup(mut commands: Commands, mut store: ResMut<GizmoConfigStore>) {
    let (config, _) = store.config_mut::<FlightGizmos>();
    config.render_layers = RenderLayers::layer(PATH_LAYER);
    config.line.width = 2.0;
    config.depth_bias = -0.01;
    commands.spawn((
        EstimateLabel,
        Text::new("CYAN actual | ORANGE map estimate: waiting"),
        TextFont {
            font_size: FontSize::Px(14.0),
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            left: px(12),
            top: px(34),
            ..default()
        },
    ));
}

#[fastrace::trace]
#[allow(clippy::needless_pass_by_value)]
pub fn poll_estimate(
    ports: NonSend<IpcPorts>,
    mut overlay: ResMut<FlightOverlay>,
    time: Res<Time<Real>>,
) {
    let Some(subscriber) = &ports.corrected_sub else {
        return;
    };
    loop {
        match subscriber.receive() {
            Ok(Some(sample)) => overlay.estimate.observe(&sample, time.elapsed_secs_f64()),
            Ok(None) => break,
            Err(error) => {
                log::warn!("地图定位影子接收失败: {error:?}");
                break;
            }
        }
    }
}

#[fastrace::trace]
#[allow(clippy::needless_pass_by_value)]
pub fn draw(
    overlay: Res<FlightOverlay>,
    time: Res<Time<Real>>,
    mut gizmos: Gizmos<FlightGizmos>,
    mut labels: Query<&mut Text, With<EstimateLabel>>,
) {
    for (track, color) in [
        (&overlay.actual, ACTUAL_COLOR),
        (&overlay.estimate, ESTIMATE_COLOR),
    ] {
        for (a, b) in track.points.iter().zip(track.points.iter().skip(1)) {
            if b.connected {
                gizmos.line(a.position, b.position, color);
            }
        }
    }
    let pose = overlay.estimate.sample(time.elapsed_secs_f64());
    let status = if pose.is_some() {
        "live"
    } else if overlay.estimate.stamp.is_some() {
        "stale / invalid"
    } else {
        "waiting"
    };
    for mut label in &mut labels {
        let text = format!("CYAN actual | ORANGE map estimate: {status}");
        if label.0 != text {
            label.0 = text;
        }
    }
    let Some(pose) = pose else { return };
    // 线框四旋翼使用 body→map 姿态；只进 PATH_LAYER，不污染相机测量。
    for x in [-0.065, 0.065] {
        for y in [-0.065, 0.065] {
            let rotor = pose.transform_point(Vec3::new(x, y, 0.0));
            gizmos.line(pose.translation, rotor, ESTIMATE_COLOR);
            gizmos.circle(Isometry3d::new(rotor, pose.rotation), 0.04, ESTIMATE_COLOR);
        }
    }
    gizmos.arrow(
        pose.translation,
        pose.transform_point(Vec3::X * 0.25),
        ESTIMATE_COLOR,
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn message(stamp: f64, x: f64) -> OdomMessage {
        OdomMessage {
            timestamp: stamp,
            position_x: x,
            is_initialized: true,
            ..default()
        }
    }

    #[test]
    fn estimate_uses_map_pose_without_truth_and_expires_without_new_measurements() {
        let mut overlay = FlightOverlay::default();
        overlay.estimate.observe(&message(1.0, -13.0), 10.0);
        assert!(overlay.actual.points.is_empty());
        assert_eq!(
            overlay.estimate.sample(10.0).unwrap().translation,
            Vec3::new(-13.0, 0.0, 0.0)
        );
        overlay.estimate.observe(&message(1.0, 5.0), 10.4);
        overlay.estimate.observe(&message(0.9, 6.0), 10.5);
        assert!(overlay.estimate.sample(10.51).is_none());
        assert_eq!(overlay.estimate.points.len(), 1);
    }

    #[test]
    fn invalid_and_missing_samples_break_trail_and_display_interpolation() {
        let mut track = Track::default();
        track.observe(&message(1.0, 0.0), 1.0);
        track.observe(&message(1.1, 1.0), 1.1);
        assert!(track.points.back().unwrap().connected);
        let mut invalid = message(1.2, 2.0);
        invalid.is_initialized = false;
        track.observe(&invalid, 1.2);
        assert!(track.sample(1.2).is_none());
        track.observe(&message(1.3, 3.0), 1.3);
        assert!(!track.points.back().unwrap().connected);
        assert_eq!(track.sample(1.3).unwrap().translation.x, 3.0);
        track.observe(&message(2.0, 4.0), 1.4);
        assert!(!track.points.back().unwrap().connected);
        track.observe(&message(2.1, 5.0), 2.0);
        assert!(!track.points.back().unwrap().connected);
        invalid = message(2.2, f64::NAN);
        track.observe(&invalid, 2.1);
        assert!(track.sample(2.1).is_none());
    }

    #[test]
    fn trajectories_are_bounded_and_independent() {
        let mut overlay = FlightOverlay::default();
        for i in 0..(CAPACITY + 20) {
            let stamp = i as f64 * 0.1;
            overlay.actual.observe(&message(stamp, i as f64), stamp);
        }
        assert_eq!(overlay.actual.points.len(), CAPACITY);
        assert_eq!(overlay.actual.points.front().unwrap().position.x, 20.0);
        assert!(overlay.estimate.points.is_empty());
    }

    #[test]
    fn overlay_layer_excludes_sensor_cameras() {
        let mut app = App::new();
        app.insert_resource(GizmoConfigStore::default());
        app.world_mut()
            .resource_mut::<GizmoConfigStore>()
            .insert(GizmoConfig::default(), FlightGizmos);
        app.add_systems(Startup, setup);
        app.update();
        let store = app.world().resource::<GizmoConfigStore>();
        let (config, _) = store.config::<FlightGizmos>();
        assert!(
            !config
                .render_layers
                .intersects(&RenderLayers::layer(crate::rig::WORLD_LAYER))
        );
        assert!(
            config
                .render_layers
                .intersects(&RenderLayers::layer(PATH_LAYER))
        );
    }
}
