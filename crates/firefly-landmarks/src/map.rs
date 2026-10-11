//! 路标身份、共视图与区域检索；容量耗尽显式拒绝，原始 VIO 位姿不可覆盖。
use crate::{
    descriptor::{self, Descriptor},
    geometry::{self, Camera},
};
use firefly_base::{FrameId, RigidTransform};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector3;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

#[derive(Clone, Debug, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Options {
    pub max_keyframes: usize,
    pub max_landmarks: usize,
    pub max_observations: usize,
    pub local_window: usize,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            max_keyframes: 512,
            max_landmarks: 50_000,
            max_observations: 32,
            local_window: 10,
        }
    }
}
#[derive(Clone, Debug)]
pub struct Feature {
    pub track: u64,
    pub pixel: [f32; 2],
    pub normalized: [f64; 2],
    pub right_normalized: Option<[f64; 2]>,
    pub descriptor: Descriptor,
    pub seed_body: Option<[f64; 3]>,
}
#[derive(Clone, Debug)]
pub struct Frame {
    pub timestamp: f64,
    pub raw: RigidTransform,
    pub pose: RigidTransform,
    pub camera: Camera,
    pub right_camera: Option<Camera>,
    pub features: Vec<Feature>,
}
#[derive(Clone, Debug)]
struct Landmark {
    owner: usize,
    point_body: Vector3<f64>,
    observations: BTreeMap<usize, usize>,
}
#[derive(Clone, Debug)]
pub struct LoopMatch {
    pub historic: usize,
    pub current: usize,
    pub transform: RigidTransform,
    pub inliers: usize,
    pub error: f64,
    pub pairs: Vec<(u64, u64)>,
}

pub struct Map {
    options: Options,
    next_track: u64,
    session: Option<u64>,
    pub frames: Vec<Frame>,
    points: HashMap<u64, Landmark>,
    aliases: HashMap<u64, u64>,
    index: HashMap<(usize, u16), BTreeSet<usize>>,
}
impl Map {
    /// # Errors
    /// 容量或局部窗口小于算法下限。
    pub fn new(options: Options) -> Result<Self> {
        if !(2..=4096).contains(&options.max_keyframes)
            || options.max_landmarks == 0
            || options.max_observations < 2
            || options.local_window < 2
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "invalid landmark map capacity",
            ));
        }
        Ok(Self {
            options,
            next_track: 1 << 63,
            session: None,
            frames: Vec::new(),
            points: HashMap::new(),
            aliases: HashMap::new(),
            index: HashMap::new(),
        })
    }
    fn canonical(&self, id: u64) -> u64 {
        self.aliases.get(&id).copied().unwrap_or(id)
    }
    #[must_use]
    pub fn landmark_count(&self) -> usize {
        self.points.len()
    }
    #[must_use]
    pub fn point(&self, id: u64) -> Option<Vector3<f64>> {
        let p = self.points.get(&self.canonical(id))?;
        Some(self.frames[p.owner].pose.point(p.point_body.into()).coords)
    }
    fn observed(&self, frame: usize, feature: usize) -> Option<(u64, Vector3<f64>)> {
        let id = self.canonical(self.frames[frame].features[feature].track);
        let landmark = self.points.get(&id)?;
        if landmark.observations.get(&frame) != Some(&feature) {
            return None;
        }
        self.point(id)
            .filter(|p| p.iter().all(|v| v.is_finite()))
            .map(|p| (id, p))
    }
    #[fastrace::trace]
    /// # Errors
    /// 会话、时间、观测无效或容量不足；拒绝不修改地图。
    /// # Panics
    /// 内部观测容量不变量遭到破坏。
    pub fn insert(&mut self, session: u64, mut frame: Frame) -> Result<usize> {
        if session == 0
            || self.session.is_some_and(|s| s != session)
            || !frame.timestamp.is_finite()
            || self
                .frames
                .last()
                .is_some_and(|f| frame.timestamp <= f.timestamp)
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "landmark session or timestamp mismatch",
            ));
        }
        let mut next_track = self.next_track;
        for feature in &mut frame.features {
            if feature.track == u64::MAX {
                feature.track = next_track;
                next_track = next_track
                    .checked_add(1)
                    .ok_or_else(|| Error::new(ErrorKind::Internal, "landmark id exhausted"))?;
            }
        }
        let mut ids = HashSet::new();
        if frame.features.iter().any(|f| {
            !ids.insert(f.track)
                || !f.normalized.iter().all(|v| v.is_finite())
                || !f.pixel.iter().all(|v| v.is_finite())
                || f.right_normalized.is_some_and(|p| {
                    frame.right_camera.is_none() || !p.iter().all(|v| v.is_finite())
                })
                || f.seed_body
                    .is_some_and(|p| !p.iter().all(|v| v.is_finite()))
        }) {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "invalid landmark observation",
            ));
        }
        let new = frame
            .features
            .iter()
            .filter(|f| !self.points.contains_key(&self.canonical(f.track)))
            .count();
        if self.frames.len() >= self.options.max_keyframes
            || self.points.len() + new > self.options.max_landmarks
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "landmark map capacity exhausted",
            ));
        }
        if let Some(previous) = self.frames.last() {
            frame.pose = previous
                .pose
                .compose(&previous.raw.inverse())?
                .compose(&frame.raw)?;
        }
        self.associate(&mut frame);
        self.next_track = next_track;
        self.session = Some(session);
        let k = self.frames.len();
        for (i, f) in frame.features.iter().enumerate() {
            let id = self.canonical(f.track);
            let entry = self.points.entry(id).or_insert_with(|| Landmark {
                owner: k,
                point_body: Vector3::from(f.seed_body.unwrap_or([f64::NAN; 3])),
                observations: BTreeMap::new(),
            });
            if !entry.point_body.iter().all(|v| v.is_finite())
                && let Some(p) = f.seed_body
            {
                entry.owner = k;
                entry.point_body = p.into();
            }
            entry.observations.insert(k, i);
            while entry.observations.len() > self.options.max_observations {
                let oldest = *entry
                    .observations
                    .keys()
                    .find(|&&j| j != entry.owner)
                    .expect("bounded observations");
                entry.observations.remove(&oldest);
            }
            for (slot, bytes) in f.descriptor.as_chunks::<2>().0.iter().enumerate() {
                self.index
                    .entry((slot, u16::from_le_bytes(*bytes)))
                    .or_default()
                    .insert(k);
            }
        }
        self.frames.push(frame);
        self.refine_local(k);
        self.cull(k);
        Ok(k)
    }
    /// 补充外观仅通过真实两视图三角化或已建路标的重投影建立身份。
    fn associate(&self, frame: &mut Frame) {
        let query: Vec<_> = frame.features.iter().map(|f| f.descriptor).collect();
        let mut used: HashSet<_> = frame
            .features
            .iter()
            .filter(|f| f.track < 1 << 63)
            .map(|f| self.canonical(f.track))
            .collect();
        let mut associated = HashSet::new();
        for previous in self.frames.iter().rev().take(3) {
            if frame.timestamp - previous.timestamp > 3. {
                continue;
            }
            let train: Vec<_> = previous.features.iter().map(|f| f.descriptor).collect();
            for (a, b) in descriptor::matches(&query, &train) {
                if frame.features[a].track < 1 << 63 || associated.contains(&a) {
                    continue;
                }
                let id = self.canonical(previous.features[b].track);
                if used.contains(&id) {
                    continue;
                }
                let observations = [
                    (
                        &previous.pose,
                        &previous.camera,
                        previous.features[b].normalized,
                    ),
                    (&frame.pose, &frame.camera, frame.features[a].normalized),
                ];
                let point = self
                    .point(id)
                    .filter(|p| p.iter().all(|v| v.is_finite()))
                    .or_else(|| geometry::triangulate(&observations));
                let Some(point) = point else { continue };
                if observations.iter().any(|(pose, cam, uv)| {
                    cam.residual(pose.inverse().point(point.into()).coords, *uv)
                        .is_none_or(|r| r.norm() > 3.)
                }) {
                    continue;
                }
                frame.features[a].track = id;
                frame.features[a].seed_body =
                    Some(frame.pose.inverse().point(point.into()).coords.into());
                used.insert(id);
                associated.insert(a);
            }
        }
    }
    #[fastrace::trace]
    fn refine_local(&mut self, current: usize) {
        let ids: Vec<_> = self.frames[current]
            .features
            .iter()
            .map(|f| self.canonical(f.track))
            .collect();
        self.refine_points(ids);
    }
    fn refine_points(&mut self, mut ids: Vec<u64>) {
        ids.sort_unstable();
        ids.dedup();
        for id in ids {
            let point = &self.points[&id];
            let observations: Vec<_> = point
                .observations
                .iter()
                .rev()
                .take(self.options.local_window)
                .flat_map(|(&k, &i)| {
                    let f = &self.frames[k];
                    let feature = &f.features[i];
                    let left = Some((&f.pose, &f.camera, feature.normalized));
                    let right = f
                        .right_camera
                        .as_ref()
                        .zip(feature.right_normalized)
                        .map(|(camera, uv)| (&f.pose, camera, uv));
                    [left, right].into_iter().flatten()
                })
                .collect();
            let seed = self
                .point(id)
                .filter(|p| p.iter().all(|v| v.is_finite()))
                .or_else(|| geometry::triangulate(&observations));
            let Some(seed) = seed else { continue };
            if let Some(refined) = geometry::refine_point(seed, &observations) {
                let owner = point.owner;
                self.points.get_mut(&id).unwrap().point_body = self.frames[owner]
                    .pose
                    .inverse()
                    .point(refined.into())
                    .coords;
            }
        }
    }
    /// 保留外观记录；几何离群观测不参与共视、精化和后续回环。
    fn cull(&mut self, current: usize) {
        let frames = &self.frames;
        self.points.retain(|_, point| {
            if !point.point_body.iter().all(|v| v.is_finite()) {
                return point
                    .observations
                    .keys()
                    .any(|&k| current.saturating_sub(k) <= 3);
            }
            let world = frames[point.owner].pose.point(point.point_body.into());
            point.observations.retain(|&k, &mut i| {
                let frame = &frames[k];
                frame
                    .camera
                    .residual(
                        frame.pose.inverse().point(world).coords,
                        frame.features[i].normalized,
                    )
                    .is_some_and(|r| r.norm() <= 3.)
                    && frame
                        .right_camera
                        .as_ref()
                        .zip(frame.features[i].right_normalized)
                        .is_none_or(|(camera, uv)| {
                            camera
                                .residual(frame.pose.inverse().point(world).coords, uv)
                                .is_some_and(|r| r.norm() <= 3.)
                        })
            });
            !point.observations.is_empty()
        });
    }
    #[must_use]
    pub fn connected(&self, k: usize) -> Vec<(usize, usize)> {
        let mut counts = HashMap::<usize, usize>::new();
        let mut seen = HashSet::new();
        for (i, f) in self.frames[k].features.iter().enumerate() {
            let id = self.canonical(f.track);
            if !seen.insert(id) {
                continue;
            }
            let Some(p) = self.points.get(&id) else {
                continue;
            };
            if p.observations.get(&k) != Some(&i) || !p.point_body.iter().all(|v| v.is_finite()) {
                continue;
            }
            for &j in p.observations.keys() {
                if j != k {
                    *counts.entry(j).or_default() += 1;
                }
            }
        }
        let mut result: Vec<_> = counts.into_iter().collect();
        result.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        let strong = result.iter().take_while(|(_, n)| *n >= 15).count();
        result.truncate(strong.max(1));
        result
    }
    /// 多表二进制倒排索引仅作召回，精确 Hamming 比率匹配负责投票。
    #[fastrace::trace]
    pub fn candidates(&self, k: usize) -> Vec<usize> {
        let mut local: BTreeSet<_> = self.connected(k).iter().map(|(i, _)| *i).collect();
        local.insert(k);
        for j in local.clone() {
            local.extend(self.connected(j).into_iter().map(|(i, _)| i));
        }
        let mut votes = HashMap::<usize, usize>::new();
        for f in &self.frames[k].features {
            let mut frames = HashMap::<usize, usize>::new();
            for (slot, bytes) in f.descriptor.as_chunks::<2>().0.iter().enumerate() {
                if let Some(found) = self.index.get(&(slot, u16::from_le_bytes(*bytes))) {
                    // 常见词的信息量更低；同一个查询描述子对一帧只投最稀有词的一票。
                    let weight = 65536 / found.len();
                    for &j in found {
                        frames
                            .entry(j)
                            .and_modify(|v| *v = (*v).max(weight))
                            .or_insert(weight);
                    }
                }
            }
            for (j, weight) in frames {
                if !local.contains(&j) {
                    *votes.entry(j).or_default() += weight;
                }
            }
        }
        let mut shortlist: Vec<_> = votes.into_iter().collect();
        shortlist.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        shortlist.truncate(12);
        let query: Vec<_> = self.frames[k]
            .features
            .iter()
            .map(|f| f.descriptor)
            .collect();
        let mut ranked: Vec<_> = shortlist
            .into_iter()
            .map(|(j, _)| {
                let train: Vec<_> = self.frames[j]
                    .features
                    .iter()
                    .map(|f| f.descriptor)
                    .collect();
                (j, descriptor::matches(&query, &train).len())
            })
            .collect();
        ranked.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        let threshold = ranked.first().map_or(0, |(_, n)| n * 4 / 5);
        let mut result = Vec::new();
        for (j, n) in ranked {
            if n <= threshold {
                continue;
            }
            let connected = self.connected(j);
            if connected
                .iter()
                .any(|(i, _)| local.contains(i) || result.contains(i))
            {
                continue;
            }
            result.push(j);
            if result.len() == 3 {
                break;
            }
        }
        result
    }
    #[fastrace::trace]
    pub fn verify(&self, current: usize, historic: usize) -> Option<LoopMatch> {
        self.verify_predicted(current, historic, None)
    }
    /// 已验证区域可使用相邻 VIO 运动传播的初解；最终双向几何验收完全相同。
    #[fastrace::trace]
    pub fn verify_predicted(
        &self,
        current: usize,
        historic: usize,
        prediction: Option<RigidTransform>,
    ) -> Option<LoopMatch> {
        if prediction
            .as_ref()
            .is_some_and(|p| p.target() != geometry::HISTORIC_BODY || p.source() != FrameId::BODY)
        {
            return None;
        }
        let old = &self.frames[historic];
        let now = &self.frames[current];
        let mut views = vec![historic];
        views.extend(
            self.connected(historic)
                .into_iter()
                .take(10)
                .map(|(j, _)| j),
        );
        let query: Vec<_> = now.features.iter().map(|f| f.descriptor).collect();
        let mut proposals = Vec::new();
        let mut appearance = 0;
        for &j in &views {
            let view = &self.frames[j];
            let train: Vec<_> = view.features.iter().map(|f| f.descriptor).collect();
            for (a, b) in descriptor::matches(&query, &train) {
                appearance += 1;
                let Some((id, p)) = self.observed(j, b) else {
                    continue;
                };
                let Some((query_id, q)) = self.observed(current, a) else {
                    continue;
                };
                if id == query_id {
                    continue;
                }
                proposals.push((
                    descriptor::distance(&query[a], &train[b]),
                    a,
                    id,
                    query_id,
                    p,
                    q,
                    j,
                    b,
                ));
            }
        }
        proposals.sort_by_key(|p| (p.0, p.1, p.6, p.7));
        let mut pairs = BTreeMap::new();
        let mut used = HashSet::new();
        let mut used_query = HashSet::new();
        for (_, a, id, query_id, p, q, j, b) in proposals {
            if pairs.contains_key(&a) || used.contains(&id) || used_query.contains(&query_id) {
                continue;
            }
            used.insert(id);
            used_query.insert(query_id);
            pairs.insert(a, (id, p, q, j, b));
        }
        log::info!(
            "loop candidate {:.3}->{:.3} appearance={} bilateral={} views={}",
            old.timestamp,
            now.timestamp,
            appearance,
            pairs.len(),
            views.len()
        );
        if prediction.is_none() && pairs.len() < 20 {
            return None;
        }
        let correspondences: Vec<_> = pairs
            .iter()
            .map(|(&a, &(_, p, q, j, b))| geometry::Correspondence {
                source: now.pose.inverse().point(q.into()).coords,
                target: old.pose.inverse().point(p.into()).coords,
                source_camera: &now.camera,
                target_camera: &self.frames[j].camera,
                target_from_anchor: self.frames[j].pose.isometry().inverse() * old.pose.isometry(),
                source_uv: now.features[a].normalized,
                target_uv: self.frames[j].features[b].normalized,
            })
            .collect();
        let Some(mut estimate) = prediction.or_else(|| geometry::loop_consensus(&correspondences))
        else {
            log::info!(
                "loop reject bilateral consensus {:.3}->{:.3}",
                old.timestamp,
                now.timestamp
            );
            return None;
        };
        let initial_count = pairs.len();
        let trusted: HashSet<_> = pairs
            .keys()
            .zip(&correspondences)
            .filter(|(_, pair)| pair.agrees(&estimate))
            .map(|(&index, _)| index)
            .collect();
        pairs.retain(|index, _| trusted.contains(index));
        used = pairs.values().map(|(id, _, _, _, _)| *id).collect();
        used_query = pairs
            .keys()
            .map(|&a| self.canonical(now.features[a].track))
            .collect();
        // 对照 ORB-SLAM3：初解引导候选区域路标投影，补充描述子全局比率筛选遗漏的对应。
        for &j in &views {
            for (b, feature) in self.frames[j].features.iter().enumerate() {
                let id = self.canonical(feature.track);
                if used.contains(&id) {
                    continue;
                }
                let Some((_, point)) = self.observed(j, b) else {
                    continue;
                };
                let historic_point = old.pose.inverse().point(point.into()).coords;
                let projected = estimate.inverse().point(historic_point.into()).coords;
                let mut nearby: Vec<_> = now
                    .features
                    .iter()
                    .enumerate()
                    .filter(|(a, f)| {
                        !pairs.contains_key(a)
                            && now
                                .camera
                                .residual(projected, f.normalized)
                                .is_some_and(|r| r.norm() <= 3.)
                    })
                    .map(|(a, f)| (descriptor::distance(&feature.descriptor, &f.descriptor), a))
                    .collect();
                nearby.sort_unstable();
                let Some(&(distance, a)) = nearby.first() else {
                    continue;
                };
                if distance > 50
                    || nearby
                        .get(1)
                        .is_some_and(|(next, _)| distance * 10 >= next * 8)
                {
                    continue;
                }
                let Some((_, current_point)) = self.observed(current, a) else {
                    continue;
                };
                let query_id = self.canonical(now.features[a].track);
                if used_query.contains(&query_id) || id == query_id {
                    continue;
                }
                used.insert(id);
                used_query.insert(query_id);
                pairs.insert(a, (id, point, current_point, j, b));
            }
        }
        let observations: Vec<_> = pairs
            .iter()
            .map(|(&a, (_, p, _, _, _))| {
                (
                    old.pose.inverse().point((*p).into()).coords,
                    now.features[a].normalized,
                )
            })
            .collect();
        if let Some(refined) = geometry::refine_pose(estimate, &now.camera, &observations) {
            estimate = refined;
        }
        let prior = old.raw.isometry().inverse() * now.raw.isometry();
        if (estimate.isometry().translation.vector - prior.translation.vector).norm() > 3.
            || estimate.isometry().rotation.angle_to(&prior.rotation) > 15_f64.to_radians()
        {
            log::info!(
                "loop reject VIO innovation {:.3}->{:.3}",
                old.timestamp,
                now.timestamp
            );
            return None;
        }
        let mut inliers = Vec::new();
        let mut cells = HashSet::new();
        let mut error = 0.;
        for (&a, &(id, point, current_point, j, b)) in &pairs {
            let target = old.pose.inverse().point(point.into()).coords;
            let source = now.pose.inverse().point(current_point.into()).coords;
            let corrected_current = estimate.inverse().point(target.into()).coords;
            let Some(forward) = now
                .camera
                .residual(corrected_current, now.features[a].normalized)
            else {
                continue;
            };
            let old_body = estimate.point(source.into()).coords;
            let old_world = old.pose.point(old_body.into()).coords;
            let view = &self.frames[j];
            let view_body = view.pose.inverse().point(old_world.into()).coords;
            let Some(backward) = view.camera.residual(view_body, view.features[b].normalized)
            else {
                continue;
            };
            if forward.norm() > 3. || backward.norm() > 3. {
                continue;
            }
            inliers.push((self.canonical(now.features[a].track), id));
            let uv = now.features[a].pixel;
            cells.insert(((uv[0] / 80.) as usize, (uv[1] / 80.) as usize));
            error += forward.norm();
        }
        log::info!(
            "loop reprojection {:.3}->{:.3} inliers={} pairs={} initial={} cells={}",
            old.timestamp,
            now.timestamp,
            inliers.len(),
            pairs.len(),
            initial_count,
            cells.len()
        );
        if inliers.len() < 30
            || inliers.len() * 5 < pairs.len().max(initial_count) * 2
            || cells.len() < 4
        {
            return None;
        }
        Some(LoopMatch {
            historic,
            current,
            transform: estimate,
            inliers: inliers.len(),
            error: error / inliers.len() as f64,
            pairs: inliers,
        })
    }
    /// 后端接受并提供全部关键帧修正后才能合并路标，禁止先污染后续确认。
    /// # Panics
    /// 内部路标索引或观测容量不变量遭到破坏。
    pub fn merge(&mut self, pairs: &[(u64, u64)]) {
        let mut merged = Vec::new();
        for &(a, b) in pairs {
            let a = self.canonical(a);
            let b = self.canonical(b);
            if a == b {
                continue;
            }
            let (Some(left), Some(right)) = (self.points.get(&a), self.points.get(&b)) else {
                continue;
            };
            if left
                .observations
                .iter()
                .any(|(k, i)| right.observations.get(k).is_some_and(|j| j != i))
            {
                continue;
            }
            let removed = self.points.remove(&a).unwrap();
            let target = self.points.get_mut(&b).unwrap();
            target.observations.extend(removed.observations);
            while target.observations.len() > self.options.max_observations {
                let oldest = *target
                    .observations
                    .keys()
                    .find(|&&k| k != target.owner)
                    .unwrap();
                target.observations.remove(&oldest);
            }
            for value in self.aliases.values_mut() {
                if *value == a {
                    *value = b;
                }
            }
            self.aliases.insert(a, b);
            merged.push(b);
        }
        let ids = merged.into_iter().map(|id| self.canonical(id)).collect();
        self.refine_points(ids);
    }
    /// # Errors
    /// 快照缺失关键帧或坐标方向不正确。
    pub fn correct(&mut self, poses: &[(f64, RigidTransform)]) -> Result<()> {
        if poses.len() != self.frames.len()
            || poses.iter().zip(&self.frames).any(|((t, p), f)| {
                !t.total_cmp(&f.timestamp).is_eq()
                    || p.target() != FrameId::MAP
                    || p.source() != FrameId::BODY
            })
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "incomplete landmark pose correction",
            ));
        }
        for ((_, pose), frame) in poses.iter().zip(&mut self.frames) {
            frame.pose = *pose;
        }
        if !self.frames.is_empty() {
            self.refine_points(self.points.keys().copied().collect());
            self.cull(self.frames.len() - 1);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn frame(t: f64, offset: u64, seed: bool) -> Frame {
        let raw =
            RigidTransform::from_parts(FrameId::ODOM, FrameId::BODY, [0.; 3], [0., 0., 0., 1.])
                .unwrap();
        let camera = Camera {
            camera_from_body: RigidTransform::from_parts(
                geometry::CAMERA,
                FrameId::BODY,
                [0.; 3],
                [0., 0., 0., 1.],
            )
            .unwrap(),
            intrinsics: [100., 100., 160., 120.],
        };
        let mut random = 42u64;
        let features = (0..48)
            .map(|i| {
                let uv = [
                    -0.9 + f64::from(i % 8) * 0.25,
                    -0.7 + f64::from(i / 8) * 0.28,
                ];
                let z = 4. + f64::from(i % 3) * 0.2;
                let descriptor = std::array::from_fn(|_| {
                    random = random
                        .wrapping_mul(6_364_136_223_846_793_005)
                        .wrapping_add(1);
                    (random >> 32) as u8
                });
                Feature {
                    track: offset + i as u64,
                    pixel: [(160. + 100. * uv[0]) as f32, (120. + 100. * uv[1]) as f32],
                    normalized: uv,
                    right_normalized: None,
                    descriptor,
                    seed_body: seed.then_some([uv[0] * z, uv[1] * z, z]),
                }
            })
            .collect();
        Frame {
            timestamp: t,
            raw,
            pose: RigidTransform::from_matrix(FrameId::MAP, FrameId::BODY, &raw.matrix()).unwrap(),
            camera,
            right_camera: None,
            features,
        }
    }
    #[test]
    fn verified_region_prediction_uses_new_observations_and_keeps_final_gate() {
        let mut map = Map::new(Options::default()).unwrap();
        let mut old = frame(1., 0, true);
        let mut now = frame(21., 1000, true);
        for feature in old.features.iter_mut().chain(&mut now.features) {
            feature.descriptor = [0; 32];
        }
        map.insert(1, old).unwrap();
        map.insert(1, now).unwrap();
        let prediction = RigidTransform::from_parts(
            geometry::HISTORIC_BODY,
            FrameId::BODY,
            [0.; 3],
            [0., 0., 0., 1.],
        )
        .unwrap();
        assert!(map.verify(1, 0).is_none());
        assert_eq!(
            map.verify_predicted(1, 0, Some(prediction))
                .unwrap()
                .inliers,
            48
        );
        let mut incomplete = map.frames[1].clone();
        incomplete.timestamp = 21.2;
        incomplete.features.truncate(29);
        map.insert(1, incomplete).unwrap();
        assert!(map.verify_predicted(2, 0, Some(prediction)).is_none());
    }
    #[test]
    fn retrieval_does_not_saturate_to_the_earliest_keyframes() {
        let mut map = Map::new(Options::default()).unwrap();
        let mut random = 42u64;
        for k in 0..30 {
            let mut view = frame(4. * f64::from(k) + 1., 0, false);
            let template = view.features[0].clone();
            view.features = (0..600)
                .map(|i| {
                    let mut feature = template.clone();
                    feature.track = (k * 600 + i) as u64;
                    feature.descriptor = std::array::from_fn(|_| {
                        random = random
                            .wrapping_mul(6_364_136_223_846_793_005)
                            .wrapping_add(1);
                        (random >> 32) as u8
                    });
                    feature
                })
                .collect();
            map.insert(1, view).unwrap();
        }
        let mut revisit = map.frames[25].clone();
        revisit.timestamp = 125.;
        for feature in &mut revisit.features {
            feature.track += 100_000;
            for byte in feature.descriptor.iter_mut().step_by(3) {
                *byte ^= 1;
            }
        }
        let current = map.insert(1, revisit).unwrap();
        assert_eq!(map.candidates(current).first(), Some(&25));
    }
    #[test]
    fn stereo_observations_keep_metric_depth_during_point_refinement() {
        let mut map = Map::new(Options::default()).unwrap();
        let mut first = frame(1., 0, true);
        let right = Camera {
            camera_from_body: RigidTransform::from_parts(
                geometry::CAMERA,
                FrameId::BODY,
                [-0.1, 0., 0.],
                [0., 0., 0., 1.],
            )
            .unwrap(),
            intrinsics: first.camera.intrinsics,
        };
        for feature in &mut first.features {
            let point = Vector3::from(feature.seed_body.unwrap());
            feature.right_normalized = Some(right.project(point).unwrap().into());
            feature.seed_body = Some((point * 0.8).into());
        }
        first.right_camera = Some(right);
        map.insert(1, first).unwrap();
        // 同一位置的左目观测没有三角化视差，不能替代保留的右目观测。
        map.insert(1, frame(2., 0, false)).unwrap();
        for (id, feature) in frame(1., 0, true).features.iter().enumerate() {
            assert!(
                (map.point(id as u64).unwrap() - Vector3::from(feature.seed_body.unwrap())).norm()
                    < 1e-6
            );
        }
    }
    #[test]
    fn projection_replaces_descriptor_outliers_without_lowering_final_support() {
        let mut map = Map::new(Options::default()).unwrap();
        let mut historic = frame(1., 0, true);
        for i in 20..48 {
            let mut impostor = historic.features[(i * 17) % 48].clone();
            impostor.track = 200 + i as u64;
            impostor.descriptor = historic.features[i].descriptor;
            impostor.descriptor[0] ^= 1;
            historic.features[i].descriptor[0] ^= 255;
            historic.features[i].descriptor[1] ^= 255;
            historic.features.push(impostor);
        }
        map.insert(1, historic).unwrap();
        map.insert(1, frame(2., 1000, true)).unwrap();
        let result = map
            .verify(1, 0)
            .expect("guided matching must reconsider rejected appearance pairs");
        assert_eq!(result.inliers, 48);
        assert!(result.transform.isometry().translation.vector.norm() < 1e-9);
    }

    #[test]
    fn geometric_hypothesis_does_not_bypass_thirty_inlier_acceptance() {
        let mut map = Map::new(Options::default()).unwrap();
        let mut old = frame(1., 0, true);
        old.features.truncate(29);
        let mut now = frame(2., 100, true);
        now.features.truncate(29);
        map.insert(1, old).unwrap();
        map.insert(1, now).unwrap();
        assert!(map.verify(1, 0).is_none());
    }

    #[test]
    fn appearance_only_points_get_geometry_from_real_observations() {
        let mut map = Map::new(Options::default()).unwrap();
        let mut first = frame(1., 0, false);
        for f in &mut first.features {
            f.track = u64::MAX;
        }
        map.insert(1, first).unwrap();
        let mut second = frame(2., 0, false);
        second.raw = RigidTransform::from_parts(
            FrameId::ODOM,
            FrameId::BODY,
            [0.5, 0., 0.],
            [0., 0., 0., 1.],
        )
        .unwrap();
        for (i, f) in second.features.iter_mut().enumerate() {
            let z = 4. + (i % 3) as f64 * 0.2;
            f.normalized[0] -= 0.5 / z;
            f.pixel[0] = (160. + 100. * f.normalized[0]) as f32;
            f.track = u64::MAX;
        }
        map.insert(1, second).unwrap();
        assert_eq!(map.landmark_count(), 48);
        assert_eq!(map.connected(1), [(0, 48)]);
        let id = map.frames[0].features[0].track;
        assert!((map.point(id).unwrap() - Vector3::new(-3.6, -2.8, 4.)).norm() < 1e-8);
        let mut invalid = frame(3., 1000, true);
        invalid.features[0].normalized = [100., 100.];
        map.insert(1, invalid).unwrap();
        assert!(map.observed(2, 0).is_none());
    }

    #[test]
    fn persistent_geometry_survives_missing_current_depth_and_tracks_covisibility() {
        let mut map = Map::new(Options::default()).unwrap();
        map.insert(1, frame(1., 0, true)).unwrap();
        map.insert(1, frame(2., 0, false)).unwrap();
        assert_eq!(map.connected(1), vec![(0, 48)]);
        assert!(map.point(0).unwrap().iter().all(|v| v.is_finite()));
        assert_eq!(map.candidates(1), [] as [usize; 0]);
        map.insert(1, frame(10., 100, true)).unwrap();
        map.insert(1, frame(11., 100, false)).unwrap();
        assert_eq!(map.candidates(3), vec![0]);
        let found = map.verify(3, 0).unwrap();
        assert_eq!(found.inliers, 48);
        assert!(found.transform.isometry().translation.vector.norm() < 1e-10);
        assert!(found.error < 1e-9);
        assert_eq!(map.landmark_count(), 96);
        map.merge(&found.pairs);
        assert_eq!(map.landmark_count(), 48);
        assert_eq!(map.canonical(100), 0);
    }
    #[test]
    fn correction_moves_landmark_owners_and_never_changes_raw_odometry() {
        let mut map = Map::new(Options::default()).unwrap();
        map.insert(7, frame(1., 0, true)).unwrap();
        let before = map.point(0).unwrap();
        let raw = map.frames[0].raw;
        let moved =
            RigidTransform::from_parts(FrameId::MAP, FrameId::BODY, [2., 3., 0.], [0., 0., 0., 1.])
                .unwrap();
        assert!(map.correct(&[]).is_err());
        map.correct(&[(1., moved)]).unwrap();
        assert!((map.point(0).unwrap() - before - Vector3::new(2., 3., 0.)).norm() < 1e-10);
        assert_eq!(map.frames[0].raw.matrix(), raw.matrix());
    }
    #[test]
    fn session_capacity_and_duplicate_observations_fail_atomically() {
        let mut map = Map::new(Options {
            max_keyframes: 2,
            ..Default::default()
        })
        .unwrap();
        map.insert(1, frame(1., 0, true)).unwrap();
        assert!(map.insert(2, frame(2., 100, true)).is_err());
        assert!(map.insert(1, frame(1., 100, true)).is_err());
        let mut duplicate = frame(2., 100, true);
        duplicate.features[1].track = duplicate.features[0].track;
        assert!(map.insert(1, duplicate).is_err());
        assert_eq!(map.frames.len(), 1);
        map.insert(1, frame(2., 0, false)).unwrap();
        assert!(map.insert(1, frame(3., 0, false)).is_err());
    }
}
