//! 独立接收线程保存传感器时间窗口；匹配推理不得阻塞 IMU 预测里程计与深度接收。
use firefly_pubsub::{
    camera::{DEPTH_TOPIC, DepthImageMessage},
    odom::OdomMessage,
    subscriber::{OdomSubscriber, Subscriber},
};
use firefly_vision_match::{
    calibration::{left_from_depth, pinhole},
    depth::{RegisteredDepth, register_depth},
};
use nalgebra::{Quaternion, UnitQuaternion};
use std::{
    collections::VecDeque,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::JoinHandle,
    time::Duration,
};

#[derive(Default)]
struct History {
    odom: VecDeque<OdomMessage>,
    depth: VecDeque<(f64, Vec<f32>)>,
    reset: bool,
}
impl History {
    fn push_odom(&mut self, m: OdomMessage) {
        if !m.is_initialized {
            if !self.odom.is_empty() {
                self.reset = true;
            }
            return;
        }
        if !m.timestamp.is_finite()
            || m.timestamp < 0.
            || m.body_pose(firefly_base::FrameId::ODOM).is_err()
            || self
                .odom
                .back()
                .is_some_and(|last| m.timestamp <= last.timestamp)
        {
            return;
        }
        self.odom.push_back(m);
        while self.odom.len() > 1500 {
            self.odom.pop_front();
        }
    }
    fn odom_at(&self, t: f64) -> Option<OdomMessage> {
        if self.reset || !t.is_finite() {
            return None;
        }
        let mut previous = self.odom.front()?;
        for next in &self.odom {
            if next.timestamp >= t && previous.timestamp <= t {
                let dt = next.timestamp - previous.timestamp;
                if dt > 0.1 {
                    return None;
                }
                let alpha = if dt > 0. {
                    (t - previous.timestamp) / dt
                } else {
                    0.
                };
                let q = |m: &OdomMessage| {
                    UnitQuaternion::from_quaternion(Quaternion::new(
                        m.quat_w, m.quat_x, m.quat_y, m.quat_z,
                    ))
                };
                let rotation = q(previous).slerp(&q(next), alpha);
                let q = rotation.quaternion();
                let lerp = |a: f64, b: f64| a + alpha * (b - a);
                let m = OdomMessage {
                    timestamp: t,
                    position_x: lerp(previous.position_x, next.position_x),
                    position_y: lerp(previous.position_y, next.position_y),
                    position_z: lerp(previous.position_z, next.position_z),
                    quat_x: q.i,
                    quat_y: q.j,
                    quat_z: q.k,
                    quat_w: q.w,
                    ..*previous
                };
                return Some(m);
            }
            previous = next;
        }
        None
    }
    fn sample(&self, t: f64) -> Option<(OdomMessage, RegisteredDepth)> {
        let odom = self.odom_at(t)?;
        let (_, depth) = self.depth.iter().find(|(s, _)| (s - t).abs() < 1e-6)?;
        Some((
            odom,
            register_depth(depth, pinhole(), pinhole(), &left_from_depth()).ok()?,
        ))
    }
    /// T_map_body(t) = T_map_body(s) T_odom_body(s)^-1 T_odom_body(t)。
    fn map_prior(&self, t: f64, corrected: OdomMessage) -> Option<OdomMessage> {
        use firefly_base::FrameId;
        if !corrected.is_initialized {
            return None;
        }
        let current = self
            .odom_at(corrected.timestamp)?
            .body_pose(FrameId::ODOM)
            .ok()?;
        let frame = self.odom_at(t)?.body_pose(FrameId::ODOM).ok()?;
        let alignment = corrected
            .body_pose(FrameId::MAP)
            .ok()?
            .compose(&current.inverse())
            .ok()?;
        let pose = alignment.compose(&frame).ok()?;
        let p = pose.isometry().translation.vector;
        let q = pose.isometry().rotation;
        Some(OdomMessage {
            timestamp: t,
            position_x: p.x,
            position_y: p.y,
            position_z: p.z,
            quat_x: q.i,
            quat_y: q.j,
            quat_z: q.k,
            quat_w: q.w,
            ..corrected
        })
    }
}
pub struct Sensors {
    history: Arc<Mutex<History>>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}
impl Sensors {
    pub fn start() -> firefly_error::Result<Self> {
        let history = Arc::new(Mutex::new(History::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let (h, s) = (history.clone(), stop.clone());
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let thread = std::thread::Builder::new()
            .name("loop-sensors".into())
            .spawn(move || {
                let result = (|| {
                    let node = firefly_pubsub::node::create_node()?;
                    let odom = OdomSubscriber::new(&node)?;
                    let depth = Subscriber::<DepthImageMessage>::with_topic(&node, DEPTH_TOPIC)?;
                    let _ = tx.send(Ok(()));
                    while !s.load(Ordering::Relaxed) && node.wait(Duration::from_millis(2)).is_ok()
                    {
                        let mut history = h.lock().map_err(|_| {
                            firefly_error::Error::new(
                                firefly_error::ErrorKind::Internal,
                                "sensor history poisoned",
                            )
                        })?;
                        while let Some(m) = odom.receive()? {
                            history.push_odom(*m);
                        }
                        while let Some(d) = depth.receive()? {
                            if d.width != 320
                                || d.height != 240
                                || !d.timestamp.is_finite()
                                || d.timestamp < 0.
                                || history.depth.back().is_some_and(|(t, _)| d.timestamp <= *t)
                            {
                                continue;
                            }
                            history.depth.push_back((d.timestamp, d.data.to_vec()));
                            while history.depth.len() > 120 {
                                history.depth.pop_front();
                            }
                        }
                    }
                    Ok::<(), firefly_error::Error>(())
                })();
                s.store(true, Ordering::Relaxed);
                if let Err(e) = result {
                    let _ = tx.send(Err(e.to_string()));
                    log::error!("online sensor collector: {e}");
                }
            })
            .map_err(|e| {
                firefly_error::Error::new(firefly_error::ErrorKind::Internal, e.to_string())
            })?;
        rx.recv()
            .map_err(|e| {
                firefly_error::Error::new(firefly_error::ErrorKind::Internal, e.to_string())
            })?
            .map_err(|e| firefly_error::Error::new(firefly_error::ErrorKind::Internal, e))?;
        Ok(Self {
            history,
            stop,
            thread: Some(thread),
        })
    }
    pub fn map_prior(&self, t: f64, corrected: OdomMessage) -> Option<OdomMessage> {
        self.history.lock().ok()?.map_prior(t, corrected)
    }
    pub fn is_stopped(&self) -> bool {
        self.stop.load(Ordering::Relaxed)
    }
    pub fn sample(&self, t: f64) -> Option<(OdomMessage, RegisteredDepth)> {
        self.history.lock().ok()?.sample(t)
    }
}
impl Drop for Sensors {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn image_prior_uses_historical_motion_including_turns() {
        let mut h = History::default();
        for i in 0..=20 {
            let t = 1. + i as f64 * 0.05;
            let half_yaw = (t - 1.) * std::f64::consts::FRAC_PI_4;
            h.push_odom(OdomMessage {
                timestamp: t,
                position_x: t - 1.,
                quat_z: half_yaw.sin(),
                quat_w: half_yaw.cos(),
                is_initialized: true,
                ..Default::default()
            });
        }
        let corrected = OdomMessage {
            timestamp: 2.,
            position_x: 10.,
            position_y: 21.,
            quat_z: 1.,
            quat_w: 0.,
            is_initialized: true,
            ..Default::default()
        };
        let frame = h.map_prior(1., corrected).unwrap();
        assert!((frame.position_x - 10.).abs() < 1e-12);
        assert!((frame.position_y - 20.).abs() < 1e-12);
        assert!((frame.quat_z.abs() - 0.5_f64.sqrt()).abs() < 1e-12);
        assert!((frame.quat_w.abs() - 0.5_f64.sqrt()).abs() < 1e-12);
        assert_eq!(frame.timestamp, 1.);
        assert!(h.map_prior(0.99, corrected).is_none());
        assert!(h.map_prior(2.01, corrected).is_none());
    }
    #[test]
    fn interpolation_requires_coverage_and_rejects_reset() {
        let mut h = History::default();
        h.depth.push_back((1.025, vec![1.; 320 * 240]));
        h.push_odom(OdomMessage {
            timestamp: 1.,
            is_initialized: true,
            ..Default::default()
        });
        assert!(h.sample(1.025).is_none());
        h.push_odom(OdomMessage {
            timestamp: 1.05,
            position_x: 0.1,
            is_initialized: true,
            ..Default::default()
        });
        let (m, _) = h.sample(1.025).unwrap();
        assert!((m.position_x - 0.05).abs() < 1e-12);
        assert!(h.sample(1.03).is_none());
        h.push_odom(OdomMessage {
            timestamp: 1.02,
            position_x: 100.,
            is_initialized: true,
            ..Default::default()
        });
        assert!((h.sample(1.025).unwrap().0.position_x - 0.05).abs() < 1e-12);
        h.push_odom(OdomMessage::default());
        assert!(h.sample(1.025).is_none());
    }
}
