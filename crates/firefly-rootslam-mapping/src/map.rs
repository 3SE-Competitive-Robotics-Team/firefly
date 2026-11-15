//! Persistent map based on `RootSLAM` §III-A, with explicit culling and fusion policies.
use crate::pose_graph::{EdgeKind, PoseGraphEdge};
use firefly_basalt::{
    optical_flow::{OpticalFlowInput, OpticalFlowResult},
    utils::common_types::Descriptor,
    vi_estimator::local_map::LocalMapSnapshot,
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::stereographic_param::StereographicParam;
use nalgebra::{Isometry3, SMatrix, Vector2, Vector3};
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
};

pub type KeyframeId = i64;
pub type LandmarkId = u64;
pub type FeatureId = usize;
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct ObservationKey {
    pub keyframe: KeyframeId,
    pub camera: usize,
    pub feature: FeatureId,
}
#[derive(Clone, Debug)]
pub struct MapObservation {
    pub pixel: Vector2<f64>,
}
#[derive(Clone, Debug)]
pub struct MapLandmark {
    pub id: LandmarkId,
    pub host: KeyframeId,
    pub position: Vector3<f64>,
    pub observations: BTreeMap<ObservationKey, MapObservation>,
}
#[derive(Clone, Debug)]
pub struct MapFeature {
    pub pixel: Vector2<f64>,
    pub descriptor: Option<Descriptor>,
    pub landmark: Option<LandmarkId>,
}
#[derive(Clone, Debug)]
pub struct MapKeyframe {
    pub id: KeyframeId,
    pub t_w_b: Isometry3<f64>,
    pub active: bool,
    pub features: Vec<BTreeMap<FeatureId, MapFeature>>,
    /// Shared immutable input images, also used by the asynchronous loop worker.
    pub images: Option<Arc<OpticalFlowInput>>,
}
#[derive(Clone, Debug, Default)]
pub struct MapSnapshot {
    pub revision: u64,
    pub keyframes: BTreeMap<KeyframeId, MapKeyframe>,
    pub landmarks: BTreeMap<LandmarkId, MapLandmark>,
    pub edges: Vec<PoseGraphEdge>,
}
#[derive(Clone, Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MapPolicy {
    /// Paper §III-A: strictly more than 90 percent.
    pub redundancy_fraction: f64,
    /// Paper §III-A: observations shared by at least three other keyframes.
    pub redundant_observers: usize,
    pub max_keyframes: usize,
    pub cull_batch: usize,
    /// Minimum shared observations for strong covisibility. `None` keeps all
    /// positive edges; this threshold is a project policy.
    pub essential_min_shared: Option<usize>,
    /// Scale of the pose-graph residuals, the paper's `S_ab`.
    ///
    /// The reprojection residuals of the bundle adjustment are measured in
    /// pixels and the pose-graph residuals in metres and radians. This scale
    /// brings both to pixel-equivalent units. The default derives the scale
    /// from the camera model.
    #[serde(default)]
    pub edge_information: crate::edge_information::EdgeInformation,
}
impl MapPolicy {
    /// # Errors
    /// Returns an error for invalid map bounds, fractions or edge information.
    pub fn validate(&self) -> firefly_error::Result<()> {
        if !(0. ..=1.).contains(&self.redundancy_fraction)
            || self.redundant_observers == 0
            || self.max_keyframes == 0
            || self.cull_batch == 0
        {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "Invalid persistent map policy",
            )
            .with_operation("rootslam.map.policy"));
        }
        self.edge_information.validate()?;
        Ok(())
    }
    #[must_use]
    pub fn paper_thresholds() -> Self {
        Self {
            redundancy_fraction: 0.9,
            redundant_observers: 3,
            max_keyframes: 1500,
            cull_batch: 30,
            essential_min_shared: None,
            edge_information: crate::edge_information::EdgeInformation::default(),
        }
    }
}
#[derive(Clone)]
pub struct PersistentMap {
    pub snapshot: MapSnapshot,
    pub policy: MapPolicy,
    pub calibration: Calibration<f64>,
    /// Square root information matrix shared by every edge of the graph.
    ///
    /// The map owns a common scale for odometry, covisibility and loop edges.
    edge_sqrt_information: SMatrix<f64, 6, 6>,
    aliases: BTreeMap<LandmarkId, LandmarkId>,
    removed_keyframes: BTreeSet<KeyframeId>,
    culled_poses: BTreeMap<KeyframeId, (KeyframeId, Isometry3<f64>)>,
    last_optimization_revision: u64,
    pose_revisions: BTreeMap<KeyframeId, u64>,
    point_revisions: BTreeMap<LandmarkId, u64>,
}
impl PersistentMap {
    /// # Errors
    /// Returns an error for an invalid policy or a camera model that cannot
    /// be calibrated at the requested reference depth.
    pub fn new(calibration: Calibration<f64>, policy: MapPolicy) -> firefly_error::Result<Self> {
        policy.validate()?;
        let edge_sqrt_information = policy.edge_information.resolve(&calibration)?;
        Ok(Self {
            snapshot: MapSnapshot::default(),
            policy,
            calibration,
            edge_sqrt_information,
            aliases: BTreeMap::new(),
            removed_keyframes: BTreeSet::new(),
            culled_poses: BTreeMap::new(),
            last_optimization_revision: 0,
            pose_revisions: BTreeMap::new(),
            point_revisions: BTreeMap::new(),
        })
    }
    #[must_use]
    pub fn canonical_landmark(&self, mut id: LandmarkId) -> LandmarkId {
        while let Some(&parent) = self.aliases.get(&id) {
            id = parent;
        }
        id
    }
    fn features(flow: &OpticalFlowResult) -> Vec<BTreeMap<FeatureId, MapFeature>> {
        flow.observations
            .iter()
            .map(|observations| {
                observations
                    .iter()
                    .map(|(&id, p)| {
                        (
                            id,
                            MapFeature {
                                pixel: p.translation.map(f64::from),
                                descriptor: None,
                                landmark: None,
                            },
                        )
                    })
                    .collect()
            })
            .collect()
    }
    fn detach_observation(&mut self, id: LandmarkId, key: ObservationKey) {
        if let Some(point) = self.snapshot.landmarks.get_mut(&id) {
            point.observations.remove(&key);
        }
        if let Some(feature) = self
            .snapshot
            .keyframes
            .get_mut(&key.keyframe)
            .and_then(|frame| frame.features.get_mut(key.camera))
            .and_then(|camera| camera.get_mut(&key.feature))
            && feature.landmark == Some(id)
        {
            feature.landmark = None;
        }
    }
    fn attach_observation(&mut self, id: LandmarkId, key: ObservationKey, pixel: Vector2<f64>) {
        if !self.snapshot.landmarks.contains_key(&id) {
            return;
        }
        let Some(camera) = self
            .snapshot
            .keyframes
            .get_mut(&key.keyframe)
            .and_then(|frame| frame.features.get_mut(key.camera))
        else {
            return;
        };
        let feature = camera.entry(key.feature).or_insert_with(|| MapFeature {
            pixel,
            descriptor: None,
            landmark: None,
        });
        let old = feature.landmark.replace(id);
        if let Some(old) = old.filter(|old| *old != id)
            && let Some(point) = self.snapshot.landmarks.get_mut(&old)
        {
            point.observations.remove(&key);
        }
        if let Some(point) = self.snapshot.landmarks.get_mut(&id) {
            point.observations.insert(key, MapObservation { pixel });
        }
    }
    fn apply_local_rejections(&mut self, local: &LocalMapSnapshot) {
        let mut affected = BTreeSet::new();
        for &track in &local.rejected_landmarks {
            let id = self.canonical_landmark(track as u64);
            let keys: Vec<_> = self.snapshot.landmarks.get(&id).map_or_default(|point| {
                point
                    .observations
                    .keys()
                    .filter(|key| key.feature == track)
                    .copied()
                    .collect()
            });
            for key in keys {
                self.detach_observation(id, key);
            }
            affected.insert(id);
        }
        for &(track, target) in &local.rejected_observations {
            let id = self.canonical_landmark(track as u64);
            self.detach_observation(
                id,
                ObservationKey {
                    keyframe: target.frame_id,
                    camera: target.cam_id,
                    feature: track,
                },
            );
            affected.insert(id);
        }
        for id in affected {
            if self
                .snapshot
                .landmarks
                .get(&id)
                .is_some_and(|point| point.observations.is_empty())
            {
                self.remove_landmark(id);
            }
            self.point_revisions.insert(id, self.snapshot.revision + 1);
        }
    }
    /// Copies optimized geometry before local marginalization; no retriangulation.
    /// Non-keyframe poses remain local-only and are not inserted in the map.
    #[expect(
        clippy::too_many_lines,
        reason = "Pose and point updates precede culling and graph refresh in one map transaction"
    )]
    pub fn ingest(&mut self, local: &LocalMapSnapshot) {
        self.apply_local_rejections(local);
        let formerly_active: BTreeSet<_> = self
            .snapshot
            .keyframes
            .values()
            .filter(|k| k.active)
            .map(|k| k.id)
            .collect();
        for keyframe in self.snapshot.keyframes.values_mut() {
            keyframe.active = local.keyframes.contains(&keyframe.id);
        }
        for &id in &local.keyframes {
            if self.removed_keyframes.contains(&id) {
                continue;
            }
            let (Some(pose), Some(flow)) = (local.poses.get(&id), local.frames.get(&id)) else {
                continue;
            };
            self.pose_revisions.insert(id, self.snapshot.revision + 1);
            if let Some(frame) = self.snapshot.keyframes.get_mut(&id) {
                frame.t_w_b = *pose;
                frame.active = true;
            } else {
                self.snapshot.keyframes.insert(
                    id,
                    MapKeyframe {
                        id,
                        t_w_b: *pose,
                        active: true,
                        features: Self::features(flow),
                        images: flow.input_images.clone(),
                    },
                );
                if let Some((&previous, previous_frame)) =
                    self.snapshot.keyframes.range(..id).next_back()
                {
                    self.snapshot.edges.push(PoseGraphEdge {
                        from: previous,
                        to: id,
                        measurement: previous_frame.t_w_b.inverse() * pose,
                        kind: EdgeKind::Temporal,
                        sqrt_information: self.edge_sqrt_information,
                    });
                }
            }
        }
        for (&track, point) in &local.landmarks {
            let host = point.host_kf_id;
            if point.inv_dist <= 0.
                || !point.inv_dist.is_finite()
                || !self.snapshot.keyframes.contains_key(&host.frame_id)
            {
                continue;
            }
            let Some(body_pose) = local.poses.get(&host.frame_id) else {
                continue;
            };
            let Some(extrinsic) = self.calibration.t_i_c.get(host.cam_id) else {
                continue;
            };
            let ray = StereographicParam::unproject(&point.direction, None);
            let camera_point = nalgebra::Point3::from(ray.fixed_rows::<3>(0) / point.inv_dist);
            let position = (body_pose * extrinsic * camera_point).coords;
            if !position.iter().all(|v| v.is_finite()) {
                continue;
            }
            let id = self.canonical_landmark(track as u64);
            let landmark = self
                .snapshot
                .landmarks
                .entry(id)
                .or_insert_with(|| MapLandmark {
                    id,
                    host: host.frame_id,
                    position,
                    observations: BTreeMap::new(),
                });
            // A fused point retains its map geometry; an unfused active point is
            // refreshed from the latest local optimization.
            if id == track as u64 {
                landmark.position = position;
                self.point_revisions.insert(id, self.snapshot.revision + 1);
            }
            for (target, pixel) in &point.obs {
                self.attach_observation(
                    id,
                    ObservationKey {
                        keyframe: target.frame_id,
                        camera: target.cam_id,
                        feature: track,
                    },
                    *pixel,
                );
            }
        }
        self.snapshot.revision += 1;
        let newly_inactive: Vec<_> = formerly_active
            .into_iter()
            .filter(|id| !local.keyframes.contains(id))
            .collect();
        for id in newly_inactive {
            if self.is_redundant(id) {
                self.remove_keyframe(id);
            }
        }
        self.cull_to_budget();
        self.rebuild_essential_graph();
    }
    #[must_use]
    pub fn covisibility(&self) -> BTreeMap<(KeyframeId, KeyframeId), usize> {
        let mut counts = BTreeMap::new();
        for point in self.snapshot.landmarks.values() {
            let frames: BTreeSet<_> = point.observations.keys().map(|o| o.keyframe).collect();
            let frames: Vec<_> = frames.into_iter().collect();
            for (i, &a) in frames.iter().enumerate() {
                for &b in &frames[i + 1..] {
                    *counts.entry((a, b)).or_insert(0) += 1;
                }
            }
        }
        counts
    }
    #[must_use]
    pub fn is_redundant(&self, id: KeyframeId) -> bool {
        let Some(frame) = self.snapshot.keyframes.get(&id) else {
            return false;
        };
        if frame.active {
            return false;
        }
        let mut observations = 0_usize;
        let mut redundant = 0_usize;
        for feature in frame.features.iter().flat_map(BTreeMap::values) {
            let Some(point) = feature
                .landmark
                .and_then(|p| self.snapshot.landmarks.get(&p))
            else {
                continue;
            };
            observations += 1;
            let others: BTreeSet<_> = point
                .observations
                .keys()
                .filter(|o| o.keyframe != id)
                .map(|o| o.keyframe)
                .collect();
            if others.len() >= self.policy.redundant_observers {
                redundant += 1;
            }
        }
        observations > 0 && redundant as f64 / observations as f64 > self.policy.redundancy_fraction
    }
    fn remove_keyframe(&mut self, id: KeyframeId) {
        if self.snapshot.keyframes.get(&id).is_none_or(|k| k.active) {
            return;
        }
        // Trajectory-export implementation choice: anchor a removed keyframe
        // to its nearest retained timestamp. Chained anchors survive later culls.
        let Some((&parent, frame)) = self
            .snapshot
            .keyframes
            .iter()
            .filter(|(key, _)| **key != id)
            .min_by_key(|(key, _)| key.abs_diff(id))
        else {
            return;
        };
        let relative = frame.t_w_b.inverse() * self.snapshot.keyframes[&id].t_w_b;
        // Contract through the corrected parent-to-removed estimate while
        // preserving each temporal/loop measurement:
        // T_parent_neighbor = T_parent_removed * T_removed_neighbor.
        let neighbors: Vec<_> = self
            .snapshot
            .edges
            .iter()
            .filter_map(|edge| {
                if !matches!(edge.kind, EdgeKind::Temporal | EdgeKind::Loop) {
                    return None;
                }
                let neighbor = if edge.from == id {
                    edge.to
                } else if edge.to == id {
                    edge.from
                } else {
                    return None;
                };
                (neighbor != parent).then_some((
                    neighbor,
                    edge.kind,
                    edge.sqrt_information,
                    edge.measurement,
                    edge.from == id,
                ))
            })
            .collect();
        self.culled_poses.insert(id, (parent, relative));
        self.snapshot.keyframes.remove(&id);
        self.removed_keyframes.insert(id);
        let hosted: Vec<_> = self
            .snapshot
            .landmarks
            .values()
            .filter(|l| l.host == id)
            .map(|l| l.id)
            .collect();
        for point in hosted {
            self.remove_landmark(point);
        }
        for point in self.snapshot.landmarks.values_mut() {
            point.observations.retain(|key, _| key.keyframe != id);
        }
        self.snapshot
            .edges
            .retain(|edge| edge.from != id && edge.to != id);
        for (neighbor, kind, sqrt_information, measurement, forward) in neighbors {
            if self.snapshot.edges.iter().any(|edge| {
                (edge.from == parent && edge.to == neighbor)
                    || (edge.from == neighbor && edge.to == parent)
            }) {
                continue;
            }
            if !self.snapshot.keyframes.contains_key(&neighbor) {
                continue;
            }
            // `relative` is the current estimate of the removed keyframe in the
            // parent frame, so folding it into the original measurement keeps
            // the edge constraint instead of restating the current estimate.
            let folded = if forward {
                relative * measurement
            } else {
                relative * measurement.inverse()
            };
            self.snapshot.edges.push(PoseGraphEdge {
                from: parent,
                to: neighbor,
                measurement: folded,
                kind,
                sqrt_information,
            });
        }
    }
    #[must_use]
    pub fn resolved_pose(&self, id: KeyframeId) -> Option<Isometry3<f64>> {
        if let Some(frame) = self.snapshot.keyframes.get(&id) {
            return Some(frame.t_w_b);
        }
        let (parent, relative) = self.culled_poses.get(&id)?;
        self.resolved_pose(*parent).map(|pose| pose * relative)
    }
    pub fn apply_descriptors(
        &mut self,
        keyframe: KeyframeId,
        descriptors: &BTreeMap<(usize, FeatureId), Descriptor>,
    ) {
        if let Some(frame) = self.snapshot.keyframes.get_mut(&keyframe) {
            for (&(camera, feature), descriptor) in descriptors {
                if let Some(feature) = frame
                    .features
                    .get_mut(camera)
                    .and_then(|f| f.get_mut(&feature))
                {
                    feature.descriptor = Some(*descriptor);
                }
            }
        }
    }
    /// Called by the map manager only after any required pose-graph correction.
    pub fn apply_loop_associations(
        &mut self,
        closure: &crate::loop_closure::LoopCandidate,
    ) -> bool {
        let Some(closure) = self.revalidate_loop(closure) else {
            return false;
        };
        if !self.snapshot.keyframes.contains_key(&closure.query)
            || !self.snapshot.keyframes.contains_key(&closure.candidate)
        {
            return false;
        }
        for &(first, second) in &closure.landmark_pairs {
            let _ = self.fuse_landmarks(first, second);
        }
        for &(key, id) in &closure.associations {
            let id = self.canonical_landmark(id);
            let old = self
                .snapshot
                .keyframes
                .get(&key.keyframe)
                .and_then(|k| k.features.get(key.camera))
                .and_then(|features| features.get(&key.feature))
                .and_then(|f| f.landmark);
            if let Some(old) = old.filter(|old| *old != id)
                && let Some(point) = self.snapshot.landmarks.get_mut(&old)
            {
                point.observations.remove(&key);
            }
            let Some(point) = self.snapshot.landmarks.get_mut(&id) else {
                continue;
            };
            let Some(feature) = self
                .snapshot
                .keyframes
                .get_mut(&key.keyframe)
                .and_then(|k| k.features.get_mut(key.camera))
                .and_then(|features| features.get_mut(&key.feature))
            else {
                continue;
            };
            feature.landmark = Some(id);
            point.observations.insert(
                key,
                MapObservation {
                    pixel: feature.pixel,
                },
            );
        }
        // The map owns the pose-graph scale: the loop detector only performs the
        // geometric validation, so the edge it proposes is weighted here.
        let mut loop_edge = closure.edge.clone();
        loop_edge.sqrt_information = self.edge_sqrt_information;
        if let Some(existing) =
            self.snapshot.edges.iter_mut().find(|e| {
                e.kind == EdgeKind::Loop && e.from == loop_edge.from && e.to == loop_edge.to
            })
        {
            *existing = loop_edge;
        } else {
            self.snapshot.edges.push(loop_edge);
        }
        self.snapshot.revision += 1;
        self.rebuild_essential_graph();
        true
    }
    /// Commit corrected poses, hosted geometry and verified associations as one
    /// map transaction. Failed post-correction validation leaves the live map
    /// and its revision unchanged.
    pub fn apply_loop_correction(
        &mut self,
        revision: u64,
        poses: &BTreeMap<KeyframeId, Isometry3<f64>>,
        points: &BTreeMap<LandmarkId, Vector3<f64>>,
        closure: &crate::loop_closure::LoopCandidate,
    ) -> bool {
        let mut updated = self.clone();
        if !updated.apply_updates(revision, poses, points)
            || !updated.apply_loop_associations(closure)
        {
            return false;
        }
        *self = updated;
        true
    }
    /// Rechecks asynchronous geometric evidence against the current map, including
    /// canonical IDs after intervening fusions and corrected candidate poses.
    #[must_use]
    pub fn revalidate_loop(
        &self,
        closure: &crate::loop_closure::LoopCandidate,
    ) -> Option<crate::loop_closure::LoopCandidate> {
        if closure.source_revision > self.snapshot.revision {
            return None;
        }
        let candidate = self.snapshot.keyframes.get(&closure.candidate)?;
        let query = self.snapshot.keyframes.get(&closure.query)?;
        let verified_pose = candidate.t_w_b * closure.edge.measurement;
        let mut associations = Vec::new();
        let mut pairs = BTreeSet::new();
        let mut unique = BTreeSet::new();
        for &(key, id) in &closure.associations {
            if key.keyframe != closure.query {
                continue;
            }
            let id = self.canonical_landmark(id);
            let Some(point) = self.snapshot.landmarks.get(&id) else {
                continue;
            };
            let Some(feature) = query
                .features
                .get(key.camera)
                .and_then(|f| f.get(&key.feature))
            else {
                continue;
            };
            let Some(pixel) = crate::loop_closure::project(
                &self.calibration,
                &verified_pose,
                &point.position,
                key.camera,
            ) else {
                continue;
            };
            if (pixel - feature.pixel).norm() > closure.reprojection_radius
                || !unique.insert((key.camera, id))
            {
                continue;
            }
            if let Some(old) = feature.landmark {
                let old = self.canonical_landmark(old);
                if old != id {
                    pairs.insert((old, id));
                }
            }
            associations.push((key, id));
        }
        if associations.len() < closure.min_inliers {
            return None;
        }
        let mut result = closure.clone();
        result.source_revision = self.snapshot.revision;
        result.verified_t_w_b = verified_pose;
        result.drift_m = (verified_pose.translation.vector - query.t_w_b.translation.vector).norm();
        result.associations = associations;
        result.landmark_pairs = pairs.into_iter().collect();
        Some(result)
    }
    fn remove_landmark(&mut self, id: LandmarkId) {
        self.snapshot.landmarks.remove(&id);
        for feature in self
            .snapshot
            .keyframes
            .values_mut()
            .flat_map(|k| k.features.iter_mut())
            .flat_map(|m| m.values_mut())
        {
            if feature.landmark == Some(id) {
                feature.landmark = None;
            }
        }
    }
    fn cull_to_budget(&mut self) {
        if self.snapshot.keyframes.len() <= self.policy.max_keyframes {
            return;
        }
        // Implementation choice: rank by independently-supported observations,
        // then by total observations, then age. The paper omits its exact score.
        let mut ranking: Vec<_> = self
            .snapshot
            .keyframes
            .values()
            .filter(|k| !k.active)
            .map(|k| {
                let mut unique = 0;
                let mut total = 0;
                for feature in k.features.iter().flat_map(BTreeMap::values) {
                    if let Some(point) = feature
                        .landmark
                        .and_then(|id| self.snapshot.landmarks.get(&id))
                    {
                        total += 1;
                        let others: BTreeSet<_> = point
                            .observations
                            .keys()
                            .filter(|o| o.keyframe != k.id)
                            .map(|o| o.keyframe)
                            .collect();
                        if others.len() < self.policy.redundant_observers {
                            unique += 1;
                        }
                    }
                }
                (unique, total, k.id)
            })
            .collect();
        ranking.sort_unstable();
        for (_, _, id) in ranking.into_iter().take(self.policy.cull_batch) {
            self.remove_keyframe(id);
        }
    }
    /// Implementation choice: maximum-weight covisibility spanning forest plus
    /// temporal, validated loop and configured strong covisibility edges.
    pub fn rebuild_essential_graph(&mut self) {
        self.snapshot
            .edges
            .retain(|e| matches!(e.kind, EdgeKind::Temporal | EdgeKind::Loop));
        let mut existing: BTreeSet<_> = self
            .snapshot
            .edges
            .iter()
            .map(|e| (e.from.min(e.to), e.from.max(e.to)))
            .collect();
        let mut links: Vec<_> = self.covisibility().into_iter().collect();
        links.sort_by_key(|&(pair, weight)| (std::cmp::Reverse(weight), pair));
        let mut components: BTreeMap<_, _> =
            self.snapshot.keyframes.keys().map(|&id| (id, id)).collect();
        for ((a, b), weight) in links {
            let mut ca = a;
            while components[&ca] != ca {
                ca = components[&ca];
            }
            let mut cb = b;
            while components[&cb] != cb {
                cb = components[&cb];
            }
            let spanning = ca != cb;
            if spanning {
                components.insert(cb, ca);
            }
            if (spanning
                || self
                    .policy
                    .essential_min_shared
                    .is_none_or(|threshold| weight >= threshold))
                && existing.insert((a, b))
            {
                self.snapshot.edges.push(PoseGraphEdge {
                    from: a,
                    to: b,
                    measurement: self.snapshot.keyframes[&a].t_w_b.inverse()
                        * self.snapshot.keyframes[&b].t_w_b,
                    kind: if spanning {
                        EdgeKind::SpanningTree
                    } else {
                        EdgeKind::Covisibility
                    },
                    sqrt_information: self.edge_sqrt_information,
                });
            }
        }
    }
    /// Fusion retains the point with most observations, tie-broken by ID.
    /// The ID tie-break gives deterministic fusion across equal observation counts.
    pub fn fuse_landmarks(&mut self, first: LandmarkId, second: LandmarkId) -> Option<LandmarkId> {
        let first = self.canonical_landmark(first);
        let second = self.canonical_landmark(second);
        if first == second {
            return Some(first);
        }
        let a = self.snapshot.landmarks.get(&first)?;
        let b = self.snapshot.landmarks.get(&second)?;
        let (keep, remove) = if (a.observations.len(), std::cmp::Reverse(first))
            >= (b.observations.len(), std::cmp::Reverse(second))
        {
            (first, second)
        } else {
            (second, first)
        };
        let removed = self.snapshot.landmarks.remove(&remove)?;
        self.snapshot
            .landmarks
            .get_mut(&keep)?
            .observations
            .extend(removed.observations);
        for feature in self
            .snapshot
            .keyframes
            .values_mut()
            .flat_map(|k| k.features.iter_mut())
            .flat_map(|m| m.values_mut())
        {
            if feature.landmark == Some(remove) {
                feature.landmark = Some(keep);
            }
        }
        self.aliases.insert(remove, keep);
        self.snapshot.revision += 1;
        self.point_revisions.insert(keep, self.snapshot.revision);
        Some(keep)
    }
    /// Applies an asynchronous optimizer result to still-existing map elements.
    /// Newly arrived active poses remain fixed, as required by §III-B.
    pub fn apply_updates(
        &mut self,
        revision: u64,
        poses: &BTreeMap<KeyframeId, Isometry3<f64>>,
        points: &BTreeMap<LandmarkId, Vector3<f64>>,
    ) -> bool {
        if revision < self.last_optimization_revision || revision > self.snapshot.revision {
            return false;
        }
        let corrections: BTreeMap<_, _> = poses
            .iter()
            .filter_map(|(id, pose)| {
                self.snapshot
                    .keyframes
                    .get(id)
                    .filter(|frame| {
                        !frame.active
                            && self
                                .pose_revisions
                                .get(id)
                                .is_none_or(|version| *version <= revision)
                    })
                    .map(|frame| (*id, *pose * frame.t_w_b.inverse()))
            })
            .collect();
        for point in self.snapshot.landmarks.values_mut() {
            let accepted = points.get(&point.id).filter(|_| {
                !self
                    .snapshot
                    .keyframes
                    .get(&point.host)
                    .is_some_and(|host| host.active)
                    && self
                        .point_revisions
                        .get(&point.id)
                        .is_none_or(|version| *version <= revision)
            });
            if let Some(position) = accepted {
                point.position = *position;
            } else if let Some(correction) = corrections.get(&point.host) {
                point.position = (correction * nalgebra::Point3::from(point.position)).coords;
            }
        }
        for (id, pose) in poses {
            if corrections.contains_key(id)
                && let Some(frame) = self.snapshot.keyframes.get_mut(id)
            {
                frame.t_w_b = *pose;
            }
        }
        self.snapshot.revision += 1;
        self.last_optimization_revision = self.snapshot.revision;
        true
    }
    /// End-of-stream transition before the paper's final whole-map optimization.
    pub fn finish_local(&mut self) {
        for frame in self.snapshot.keyframes.values_mut() {
            frame.active = false;
        }
        self.snapshot.revision += 1;
    }
}
