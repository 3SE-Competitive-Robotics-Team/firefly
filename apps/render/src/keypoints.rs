//! VIO 活动点显示：按相机时间戳配对原图，显示副本与传感器发布数据隔离。

use std::collections::VecDeque;

use bevy::prelude::*;
use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH};
use firefly_pubsub::viz::{VIO_KEYPOINT_ENTITIES, VizMessage, kind};

use crate::ui::DebugViews;

/// 只保留八拍灰度图（320×240 双目共 1.17 MiB），限制调试缓存内存。
const HISTORY_SIZE: usize = 8;
/// 超过一秒墙钟无新结果时显示原图，不保留陈旧标记。
const STALE_SECONDS: f64 = 1.0;

struct Frame {
    stamp: f64,
    gray: [Vec<u8>; 2],
}

#[derive(Default)]
struct Observations {
    stamp: Option<f64>,
    received_at: f64,
    total: usize,
    color: [u8; 3],
    points: Vec<[f64; 2]>,
}

#[derive(Resource)]
pub struct KeypointOverlay {
    frames: VecDeque<Frame>,
    observations: [Observations; 2],
    displayed: [Option<(f64, bool)>; 2],
}

impl Default for KeypointOverlay {
    fn default() -> Self {
        Self {
            frames: VecDeque::with_capacity(HISTORY_SIZE),
            observations: Default::default(),
            displayed: [None; 2],
        }
    }
}

impl KeypointOverlay {
    pub fn push_frame(&mut self, stamp: f64, gray: [Vec<u8>; 2]) {
        if !stamp.is_finite()
            || gray.iter().any(|pixels| pixels.len() != IMAGE_SIZE)
            || self.frames.back().is_some_and(|frame| stamp <= frame.stamp)
        {
            return;
        }
        if self.frames.len() == HISTORY_SIZE {
            self.frames.pop_front();
        }
        self.frames.push_back(Frame { stamp, gray });
    }

    #[allow(clippy::float_cmp)] // 图像尺寸是整数元数据，必须精确匹配。
    pub fn observe(&mut self, msg: &VizMessage, wall_time: f64) {
        if msg.kind != kind::KEYPOINTS_2D
            || !msg.timestamp.is_finite()
            || msg.point_count as usize > msg.points.len()
            || msg.scalar_count != 3
            || msg.scalars[0] != IMAGE_WIDTH as f64
            || msg.scalars[1] != IMAGE_HEIGHT as f64
            || !msg.scalars[2].is_finite()
            || msg.scalars[2] < f64::from(msg.point_count)
        {
            return;
        }
        let Some(eye) = VIO_KEYPOINT_ENTITIES
            .iter()
            .position(|entity| *entity == msg.entity())
        else {
            return;
        };
        let observation = &mut self.observations[eye];
        if observation
            .stamp
            .is_some_and(|stamp| msg.timestamp <= stamp)
        {
            return;
        }
        observation.stamp = Some(msg.timestamp);
        observation.received_at = wall_time;
        observation.total = msg.scalars[2] as usize;
        observation.color = msg.color;
        observation.points.clear();
        observation.points.extend(
            msg.points[..msg.point_count as usize]
                .iter()
                .filter_map(|p| {
                    (p[0].is_finite()
                        && p[1].is_finite()
                        && p[0] >= 0.0
                        && p[0] < IMAGE_WIDTH as f64
                        && p[1] >= 0.0
                        && p[1] < IMAGE_HEIGHT as f64)
                        .then_some([p[0], p[1]])
                }),
        );
    }

    fn selected(&self, eye: usize, wall_time: f64) -> Option<(&Frame, bool)> {
        let observation = &self.observations[eye];
        if wall_time - observation.received_at < STALE_SECONDS
            && let Some(frame) = self
                .frames
                .iter()
                .find(|frame| Some(frame.stamp) == observation.stamp)
        {
            return Some((frame, true));
        }
        self.frames.back().map(|frame| (frame, false))
    }
}

#[derive(Component)]
pub struct KeypointLabel(pub usize);

/// 灰度图只在显示帧或标记状态变化时转换；缩略图等比例显示原图坐标。
#[allow(clippy::needless_pass_by_value)]
pub fn update_panels(
    mut overlay: ResMut<KeypointOverlay>,
    views: Res<DebugViews>,
    mut images: ResMut<Assets<Image>>,
    wall: Res<Time<Real>>,
    mut labels: Query<(&KeypointLabel, &mut Text)>,
) {
    let now = wall.elapsed_secs_f64();
    for (eye, handle) in [&views.left_gray, &views.right_gray]
        .into_iter()
        .enumerate()
    {
        let Some((frame, annotated)) = overlay.selected(eye, now) else {
            continue;
        };
        let key = (frame.stamp, annotated);
        if overlay.displayed[eye] != Some(key) {
            let Some(mut image) = images.get_mut(handle) else {
                continue;
            };
            let Some(pixels) = image.data.as_mut() else {
                continue;
            };
            crate::sensors::gray_to_display(&frame.gray[eye], pixels);
            if annotated {
                let observation = &overlay.observations[eye];
                draw_points(pixels, &observation.points, observation.color);
            }
            overlay.displayed[eye] = Some(key);
        }
    }
    for (label, mut text) in &mut labels {
        let side = if label.0 == 0 { "L" } else { "R" };
        let caption = match overlay.selected(label.0, now) {
            Some((frame, true)) => {
                let observation = &overlay.observations[label.0];
                format!(
                    "{side} VIO {}/{} | t={:.2}",
                    observation.points.len(),
                    observation.total,
                    frame.stamp
                )
            }
            _ => format!("{side} gray | waiting for VIO"),
        };
        if text.0 != caption {
            text.0 = caption;
        }
    }
}

fn draw_points(pixels: &mut [u8], points: &[[f64; 2]], color: [u8; 3]) {
    for &[u, v] in points {
        let (cx, cy) = (u.round() as i32, v.round() as i32);
        for dy in -3i32..=3 {
            for dx in -3i32..=3 {
                let radius2 = dx * dx + dy * dy;
                let (x, y) = (cx + dx, cy + dy);
                if (4..=10).contains(&radius2)
                    && (0..IMAGE_WIDTH as i32).contains(&x)
                    && (0..IMAGE_HEIGHT as i32).contains(&y)
                {
                    let offset = 4 * (y as usize * IMAGE_WIDTH + x as usize);
                    pixels[offset..offset + 3].copy_from_slice(&color);
                    pixels[offset + 3] = 255;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn message(eye: usize, stamp: f64) -> VizMessage {
        let mut msg = VizMessage::base(kind::KEYPOINTS_2D, stamp, VIO_KEYPOINT_ENTITIES[eye]);
        msg.scalar_count = 3;
        msg.scalars[..3].copy_from_slice(&[IMAGE_WIDTH as f64, IMAGE_HEIGHT as f64, 1.0]);
        msg.point_count = 1;
        msg.points[0] = [10.0, 20.0, 0.0];
        msg.color = [64, 255, 128];
        msg
    }

    fn push(overlay: &mut KeypointOverlay, stamp: f64) {
        overlay.push_frame(stamp, [vec![17; IMAGE_SIZE], vec![23; IMAGE_SIZE]]);
    }

    #[test]
    fn delayed_points_match_exact_frame_and_expire_without_timestamp_refresh() {
        let mut overlay = KeypointOverlay::default();
        push(&mut overlay, 1.0);
        push(&mut overlay, 2.0);
        overlay.observe(&message(0, 1.0), 10.0);
        let (selected, annotated) = overlay.selected(0, 10.5).unwrap();
        assert_eq!(selected.stamp, 1.0);
        assert!(annotated);
        assert_eq!(overlay.selected(1, 10.5).unwrap().0.stamp, 2.0);
        assert!(!overlay.selected(1, 10.5).unwrap().1);
        overlay.observe(&message(0, 1.0), 10.9);
        overlay.observe(&message(0, 0.5), 10.9);
        assert!(!overlay.selected(0, 11.0).unwrap().1);
        overlay.observe(&message(0, 3.0), 11.0);
        assert!(!overlay.selected(0, 11.0).unwrap().1);
        push(&mut overlay, 3.0);
        assert!(overlay.selected(0, 11.0).unwrap().1);
    }

    #[test]
    fn bounded_history_empty_observations_and_bad_payloads() {
        let mut overlay = KeypointOverlay::default();
        for i in 0..=HISTORY_SIZE {
            push(&mut overlay, i as f64);
        }
        assert_eq!(overlay.frames.len(), HISTORY_SIZE);
        overlay.observe(&message(0, 0.0), 1.0);
        assert!(!overlay.selected(0, 1.0).unwrap().1);
        let mut msg = message(0, HISTORY_SIZE as f64);
        msg.point_count = 0;
        msg.scalars[2] = 0.0;
        overlay.observe(&msg, 1.0);
        assert!(overlay.selected(0, 1.0).unwrap().1);
        assert_eq!(overlay.observations[0].points, [] as [[f64; 2]; 0]);
        msg.timestamp += 1.0;
        msg.point_count = u32::MAX;
        overlay.observe(&msg, 2.0);
        assert_eq!(overlay.observations[0].stamp, Some(HISTORY_SIZE as f64));
    }

    #[test]
    fn markers_clip_at_image_edges_and_do_not_modify_source_pixels() {
        let gray = vec![17; IMAGE_SIZE];
        let mut rgba = vec![0; IMAGE_SIZE * 4];
        crate::sensors::gray_to_display(&gray, &mut rgba);
        draw_points(&mut rgba, &[[0.0, 0.0], [319.0, 239.0]], [64, 255, 128]);
        assert_eq!(&rgba[2 * 4..2 * 4 + 4], &[64, 255, 128, 255]);
        assert_eq!(&rgba[100 * 4..100 * 4 + 4], &[17, 17, 17, 255]);
        assert!(gray.iter().all(|&pixel| pixel == 17));
    }
}
