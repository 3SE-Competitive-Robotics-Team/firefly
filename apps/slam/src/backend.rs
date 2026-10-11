//! 回环确认与后端确认事务；约束拒绝或未确认时不融合路标身份。
use firefly_base::FrameId;
use firefly_error::{Error, ErrorKind, Result};
use firefly_landmarks::{Frame, Map, Options, map::LoopMatch};
use firefly_pubsub::{landmarks::Correction, odom::OdomMessage, vision::LoopConstraint};
use std::time::Instant;

pub struct Backend {
    pub map: Map,
    raw: Vec<OdomMessage>,
    confirmation: Option<(f64, nalgebra::Isometry3<f64>, u32, usize)>,
    pending: Option<(LoopMatch, Instant)>,
    last_emitted: f64,
}
pub struct Output {
    pub edge: Option<LoopConstraint>,
    pub diagnostics: [f64; 4],
}
impl Backend {
    pub fn new(options: Options) -> Result<Self> {
        Ok(Self {
            map: Map::new(options)?,
            raw: Vec::new(),
            confirmation: None,
            pending: None,
            last_emitted: f64::NEG_INFINITY,
        })
    }
    /// 普通建图 1Hz；连续确认按 5Hz 快照处理，不把入库节拍当作验证节拍。
    pub fn wants(&self, timestamp: f64) -> bool {
        let period = if self.confirmation.is_some() { 0.2 } else { 1. };
        self.map
            .frames
            .last()
            .is_none_or(|f| timestamp - f.timestamp >= period - 1e-9)
    }
    pub fn waiting(&self) -> Result<bool> {
        if self
            .pending
            .as_ref()
            .is_some_and(|(_, start)| start.elapsed().as_secs_f64() > 5.)
        {
            return Err(Error::new(
                ErrorKind::Internal,
                "loop backend acknowledgement timeout",
            ));
        }
        Ok(self.pending.is_some())
    }
    pub fn correct(&mut self, correction: &Correction) -> Result<()> {
        if correction.loop_event
            && self.pending.as_ref().is_none_or(|(found, _)| {
                !correction
                    .from
                    .total_cmp(&self.map.frames[found.historic].timestamp)
                    .is_eq()
                    || !correction
                        .to
                        .total_cmp(&self.map.frames[found.current].timestamp)
                        .is_eq()
            })
        {
            return Ok(());
        }
        if correction.accepted {
            if correction.count as usize > correction.poses.len() {
                return Err(Error::new(
                    ErrorKind::InvalidArgument,
                    "invalid correction size",
                ));
            }
            let poses: Vec<_> = self
                .map
                .frames
                .iter()
                .map(|frame| {
                    correction.poses[..correction.count as usize]
                        .iter()
                        .find(|p| p.timestamp.total_cmp(&frame.timestamp).is_eq())
                        .map(|p| p.body_pose(FrameId::MAP).map(|pose| (p.timestamp, pose)))
                        .transpose()
                })
                .collect::<Result<Option<Vec<_>>>>()?
                .unwrap_or_default();
            if !correction.loop_event && poses.len() != self.map.frames.len() {
                log::warn!("地图锚点快照缺少正在接收的关键帧，等待完整快照");
                return Ok(());
            }
            self.map.correct(&poses)?;
            if correction.loop_event
                && let Some((found, _)) = &self.pending
            {
                self.map.merge(&found.pairs);
            }
            log::info!(
                "地图修正与路标融合完成 t={:.3} pairs={}",
                correction.to,
                self.pending
                    .as_ref()
                    .map_or(0, |(found, _)| found.pairs.len())
            );
        }
        if correction.loop_event {
            self.pending = None;
        }
        self.confirmation = None;
        Ok(())
    }
    #[fastrace::trace]
    pub fn process(&mut self, session: u64, frame: Frame, odom: OdomMessage) -> Result<Output> {
        let current = self.map.insert(session, frame)?;
        self.raw.push(odom);
        let mut candidates = self.map.candidates(current);
        if let Some((_, _, _, historic)) = self.confirmation {
            candidates.retain(|&k| k != historic);
            candidates.insert(0, historic);
        }
        let mut best: Option<LoopMatch> = None;
        let mut conflict = false;
        for &historic in &candidates {
            let prediction = self
                .confirmation
                .as_ref()
                .filter(|(_, _, _, k)| *k == historic)
                .map(|(_, correction, _, _)| {
                    let transform = self.map.frames[historic].pose.isometry().inverse()
                        * correction
                        * self.map.frames[current].pose.isometry();
                    firefly_base::RigidTransform::from_matrix(
                        firefly_landmarks::geometry::HISTORIC_BODY,
                        FrameId::BODY,
                        &transform.to_homogeneous(),
                    )
                })
                .transpose()?;
            if let Some(found) = self.map.verify_predicted(current, historic, prediction) {
                if let Some(previous) = &best {
                    let a = self.map.frames[previous.historic].pose.isometry()
                        * previous.transform.isometry();
                    let b = self.map.frames[historic].pose.isometry() * found.transform.isometry();
                    if (a.translation.vector - b.translation.vector).norm() > 0.3
                        || a.rotation.angle_to(&b.rotation) > 5_f64.to_radians()
                    {
                        conflict = true;
                        break;
                    }
                }
                if best.as_ref().is_none_or(|p| p.inliers < found.inliers) {
                    best = Some(found);
                }
            }
        }
        let mut confirmations = 0;
        let mut inliers = 0;
        let mut edge = None;
        if let Some(found) = best.filter(|_| !conflict) {
            let t = self.map.frames[current].timestamp;
            let correction = self.map.frames[found.historic].pose.isometry()
                * found.transform.isometry()
                * self.map.frames[current].pose.isometry().inverse();
            confirmations = self
                .confirmation
                .as_ref()
                .filter(|(last, p, _, _)| {
                    t > *last
                        && t - *last <= 3.
                        && (p.translation.vector - correction.translation.vector).norm() < 0.3
                        && p.rotation.angle_to(&correction.rotation) < 5_f64.to_radians()
                })
                .map_or(1, |(_, _, n, _)| n + 1);
            self.confirmation = Some((t, correction, confirmations, found.historic));
            inliers = found.inliers;
            log::info!(
                "loop {:.3}->{t:.3} inliers={inliers} confirmations={confirmations}",
                self.map.frames[found.historic].timestamp
            );
            if confirmations >= 3 && t - self.last_emitted >= 5. {
                let q = found.transform.isometry().rotation;
                edge = Some(LoopConstraint {
                    from: self.raw[found.historic],
                    to: self.raw[current],
                    translation: found.transform.isometry().translation.vector.into(),
                    quaternion: [q.i, q.j, q.k, q.w],
                    inliers: inliers as u32,
                    confirmations,
                    reprojection_error: found.error,
                });
                self.pending = Some((found, Instant::now()));
                self.last_emitted = t;
                self.confirmation = None;
            }
        } else {
            self.confirmation = None;
        }
        log::info!(
            "map t={:.3} keyframes={} landmarks={} candidates={} geometry={}",
            odom.timestamp,
            self.map.frames.len(),
            self.map.landmark_count(),
            candidates.len(),
            inliers
        );
        Ok(Output {
            edge,
            diagnostics: [
                self.map.frames.len() as f64,
                candidates.len() as f64,
                inliers as f64,
                f64::from(confirmations),
            ],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_base::RigidTransform;
    use firefly_landmarks::geometry::{CAMERA, Camera};
    fn frame(t: f64) -> Frame {
        Frame {
            timestamp: t,
            raw: RigidTransform::from_parts(
                FrameId::ODOM,
                FrameId::BODY,
                [0.; 3],
                [0., 0., 0., 1.],
            )
            .unwrap(),
            pose: RigidTransform::from_parts(
                FrameId::MAP,
                FrameId::BODY,
                [0.; 3],
                [0., 0., 0., 1.],
            )
            .unwrap(),
            camera: Camera {
                camera_from_body: RigidTransform::from_parts(
                    CAMERA,
                    FrameId::BODY,
                    [0.; 3],
                    [0., 0., 0., 1.],
                )
                .unwrap(),
                intrinsics: [100., 100., 160., 120.],
            },
            right_camera: None,
            features: Vec::new(),
        }
    }
    #[test]
    fn confirmation_cadence_is_independent_of_normal_keyframe_cadence() {
        let mut backend = Backend::new(Options::default()).unwrap();
        backend.map.insert(1, frame(1.)).unwrap();
        assert!(!backend.wants(1.2));
        assert!(backend.wants(2.));
        backend.confirmation = Some((1., nalgebra::Isometry3::identity(), 1, 0));
        assert!(backend.wants(1.2));
        assert!(!backend.wants(1.));
    }
    #[test]
    #[allow(clippy::large_stack_arrays)]
    fn backend_acknowledgement_is_atomic_and_matches_endpoints() {
        std::thread::Builder::new()
            .stack_size(8 * 1024 * 1024)
            .spawn(|| {
                let mut backend = Backend::new(Options::default()).unwrap();
                backend.map.insert(1, frame(1.)).unwrap();
                backend.map.insert(1, frame(2.)).unwrap();
                backend.pending = Some((
                    LoopMatch {
                        historic: 0,
                        current: 1,
                        transform: RigidTransform::from_parts(
                            firefly_landmarks::geometry::HISTORIC_BODY,
                            FrameId::BODY,
                            [0.; 3],
                            [0., 0., 0., 1.],
                        )
                        .unwrap(),
                        inliers: 30,
                        error: 0.,
                        pairs: Vec::new(),
                    },
                    Instant::now(),
                ));
                let mut correction = Correction {
                    from: 1.,
                    to: 3.,
                    accepted: true,
                    loop_event: true,
                    count: 1,
                    poses: [OdomMessage::default(); 4096],
                };
                backend.correct(&correction).unwrap();
                assert!(backend.waiting().unwrap());
                correction.to = 2.;
                correction.poses[0] = OdomMessage {
                    timestamp: 1.,
                    position_x: 1.,
                    is_initialized: true,
                    ..Default::default()
                };
                assert!(backend.correct(&correction).is_err());
                assert_eq!(backend.map.frames[0].pose.isometry().translation.x, 0.);
                correction.count = 2;
                correction.poses[1] = OdomMessage {
                    timestamp: 2.,
                    position_x: 1.,
                    is_initialized: true,
                    ..Default::default()
                };
                backend.correct(&correction).unwrap();
                assert!(!backend.waiting().unwrap());
                assert_eq!(backend.map.frames[0].pose.isometry().translation.x, 1.);
                assert_eq!(backend.map.frames[0].raw.isometry().translation.x, 0.);
                correction.loop_event = false;
                correction.poses[0].position_x = 2.;
                correction.poses[1].position_x = 2.;
                backend.correct(&correction).unwrap();
                assert_eq!(backend.map.frames[0].pose.isometry().translation.x, 2.);
            })
            .unwrap()
            .join()
            .unwrap();
    }
}
