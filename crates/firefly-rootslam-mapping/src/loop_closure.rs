//! `HashBoW`, ORB matching, map reprojection and generalized P3P loop validation.
//! Only the 10 cm correction branch is specified numerically by `RootSLAM`.
//! Retrieval, matching and RANSAC controls below are explicit implementation
//! choices; they are not claimed to be the unpublished author's defaults.
use crate::{
    descriptor::{compute_keyframe_descriptors, hamming},
    map::{FeatureId, KeyframeId, LandmarkId, MapSnapshot, ObservationKey},
    pose_graph::{EdgeKind, PoseGraphEdge},
};
use firefly_basalt::{
    hash_bow::HashBow,
    utils::common_types::{Descriptor, HashBowVector, TimeCamId},
};
use firefly_calibration::calibration::Calibration;
use firefly_error::{Error, ErrorKind, Result};
use firefly_opengv::{
    absolute_pose::noncentral_absolute_adapter::NoncentralAbsoluteAdapter,
    optimization_functor::RustNumericalBackend,
    sac::{ransac::Ransac, sample_consensus_problem::SampleConsensusProblem},
    sac_problems::absolute_pose::absolute_pose_sac_problem::{AbsolutePoseSacProblem, Algorithm},
};
use nalgebra::{
    Isometry3, Matrix3, SMatrix, Translation3, UnitQuaternion, Vector2, Vector3, Vector4,
};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct LoopOptions {
    pub hash_bits: usize,
    pub max_candidates: usize,
    pub min_separation_ns: i64,
    pub min_bow_score: f64,
    pub max_hamming: u32,
    pub descriptor_ratio: f64,
    pub min_inliers: usize,
    pub max_ransac_iterations: i32,
    /// Outer LM budget for online loop-closing PGO, independent of final global BA.
    pub pose_graph_max_iterations: usize,
    pub ransac_probability: f64,
    /// `OpenGV`'s angular distance: 1 - dot(normalized rays).
    pub angular_threshold: f64,
    pub reprojection_radius: f64,
    pub edge_sqrt_information: SMatrix<f64, 6, 6>,
}
impl Default for LoopOptions {
    fn default() -> Self {
        Self {
            hash_bits: 16,
            max_candidates: 10,
            min_separation_ns: 2_000_000_000,
            min_bow_score: 0.05,
            max_hamming: 80,
            descriptor_ratio: 0.8,
            min_inliers: 12,
            max_ransac_iterations: 200,
            pose_graph_max_iterations: 20,
            ransac_probability: 0.99,
            angular_threshold: 1e-4,
            reprojection_radius: 3.,
            edge_sqrt_information: SMatrix::identity(),
        }
    }
}
impl LoopOptions {
    /// # Errors
    /// Returns an error for invalid descriptor, retrieval or geometric controls.
    pub fn validate(&self) -> Result<()> {
        if !(1..=32).contains(&self.hash_bits)
            || self.max_candidates == 0
            || self.min_separation_ns < 0
            || !(0. ..=1.).contains(&self.min_bow_score)
            || !(1..=256).contains(&self.max_hamming)
            || !(0. ..1.).contains(&self.descriptor_ratio)
            || self.descriptor_ratio == 0.
            || self.min_inliers < 4
            || self.max_ransac_iterations <= 0
            || self.pose_graph_max_iterations == 0
            || !(0. ..1.).contains(&self.ransac_probability)
            || self.ransac_probability == 0.
            || !self.angular_threshold.is_finite()
            || self.angular_threshold <= 0.
            || self.angular_threshold >= 2.
            || !self.reprojection_radius.is_finite()
            || self.reprojection_radius <= 0.
            || !self.edge_sqrt_information.iter().all(|v| v.is_finite())
        {
            return Err(failure(
                "Invalid loop retrieval, matching, RANSAC, pose-graph budget, or edge information controls",
            ));
        }
        Ok(())
    }
}
#[derive(Clone, Debug)]
pub struct LoopCandidate {
    pub source_revision: u64,
    pub min_inliers: usize,
    pub reprojection_radius: f64,
    pub query: KeyframeId,
    pub candidate: KeyframeId,
    pub verified_t_w_b: Isometry3<f64>,
    pub drift_m: f64,
    pub edge: PoseGraphEdge,
    pub landmark_pairs: Vec<(LandmarkId, LandmarkId)>,
    pub associations: Vec<(ObservationKey, LandmarkId)>,
}
#[derive(Clone, Debug)]
pub struct LoopResult {
    pub keyframe: KeyframeId,
    pub descriptors: BTreeMap<(usize, FeatureId), Descriptor>,
    pub candidate: Option<LoopCandidate>,
}
#[derive(Clone)]
struct Correspondence {
    key: ObservationKey,
    point: LandmarkId,
    pixel: Vector2<f64>,
    distance: u32,
}
pub struct LoopDetector {
    calibration: Calibration<f64>,
    options: LoopOptions,
    bow: HashBow<256>,
    descriptors: BTreeMap<KeyframeId, BTreeMap<(usize, FeatureId), Descriptor>>,
    histograms: BTreeMap<KeyframeId, HashBowVector>,
}
impl LoopDetector {
    #[must_use]
    pub fn new(calibration: Calibration<f64>, options: LoopOptions) -> Self {
        let bow = HashBow::new(options.hash_bits);
        Self {
            calibration,
            options,
            bow,
            descriptors: BTreeMap::new(),
            histograms: BTreeMap::new(),
        }
    }
    /// # Errors
    /// Returns an error for absent keyframes or malformed calibration/correspondences.
    pub fn process(&mut self, map: &MapSnapshot, keyframe: KeyframeId) -> Result<LoopResult> {
        self.options.validate()?;
        let previous_count = self.histograms.len();
        self.histograms
            .retain(|id, _| map.keyframes.contains_key(id));
        self.descriptors
            .retain(|id, _| map.keyframes.contains_key(id));
        if previous_count != self.histograms.len() {
            self.bow = HashBow::new(self.options.hash_bits);
            for (&id, histogram) in &self.histograms {
                self.bow.add_to_database(TimeCamId::new(id, 0), histogram);
            }
        }
        let frame = map
            .keyframes
            .get(&keyframe)
            .ok_or_else(|| failure("Loop query keyframe has been culled"))?;
        if frame.features.len() > self.calibration.intrinsics.len()
            || self.calibration.t_i_c.len() != self.calibration.intrinsics.len()
        {
            return Err(failure("Loop frame/calibration camera counts do not match"));
        }
        let descriptors = compute_keyframe_descriptors(frame);
        let descriptor_list: Vec<_> = descriptors.values().copied().collect();
        let mut hashes = Vec::new();
        let mut histogram = Vec::new();
        self.bow
            .compute_bow(&descriptor_list, &mut hashes, &mut histogram);
        let mut retrieved = Vec::new();
        self.bow.query_database(
            &histogram,
            self.histograms.len(),
            &mut retrieved,
            Some(keyframe.saturating_sub(self.options.min_separation_ns)),
        );
        retrieved.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
        let candidates: BTreeMap<_, _> = retrieved
            .into_iter()
            .filter_map(|(id, score)| {
                map.keyframes
                    .get(&id.frame_id)
                    .filter(|k| {
                        !k.active && k.id != keyframe && score >= self.options.min_bow_score
                    })
                    .map(|k| (k.id, score))
            })
            .take(self.options.max_candidates)
            .collect();
        let islands = candidate_islands(map, &candidates);
        let mut result = None;
        for island in islands {
            for candidate in island {
                if let Some(found) = self.verify(map, keyframe, candidate, &descriptors)? {
                    result = Some(found);
                    break;
                }
            }
            if result.is_some() {
                break;
            }
        }
        // Own descriptor cache is essential: the map manager applies results
        // asynchronously, so an input snapshot may predate its descriptor update.
        if !self.descriptors.contains_key(&keyframe) {
            self.bow
                .add_to_database(TimeCamId::new(keyframe, 0), &histogram);
        }
        self.descriptors.insert(keyframe, descriptors.clone());
        self.histograms.insert(keyframe, histogram);
        self.descriptors
            .retain(|id, _| map.keyframes.contains_key(id));
        Ok(LoopResult {
            keyframe,
            descriptors,
            candidate: result,
        })
    }
    #[expect(
        clippy::too_many_lines,
        reason = "Ordered descriptor, reprojection, RANSAC and pixel validation stages"
    )]
    fn verify(
        &self,
        map: &MapSnapshot,
        query: KeyframeId,
        candidate: KeyframeId,
        descriptors: &BTreeMap<(usize, FeatureId), Descriptor>,
    ) -> Result<Option<LoopCandidate>> {
        let target = &map.keyframes[&candidate];
        let current = &map.keyframes[&query];
        let cached = self.descriptors.get(&candidate);
        let mut target_descriptors: BTreeMap<LandmarkId, Vec<Descriptor>> = BTreeMap::new();
        for (camera, features) in target.features.iter().enumerate() {
            for (&id, feature) in features {
                let Some(point) = feature.landmark.filter(|id| map.landmarks.contains_key(id))
                else {
                    continue;
                };
                if let Some(descriptor) = cached
                    .and_then(|d| d.get(&(camera, id)))
                    .copied()
                    .or(feature.descriptor)
                {
                    target_descriptors
                        .entry(point)
                        .or_default()
                        .push(descriptor);
                }
            }
        }
        let mut correspondences = BTreeMap::new();
        for (&(camera, feature), descriptor) in descriptors {
            let mut ranking: Vec<_> = target_descriptors
                .iter()
                .map(|(&id, values)| {
                    (
                        values
                            .iter()
                            .map(|other| hamming(descriptor, other))
                            .min()
                            .unwrap(),
                        id,
                    )
                })
                .collect();
            ranking.sort_unstable();
            let Some(&(best, point)) = ranking.first() else {
                continue;
            };
            let next = ranking.get(1).map_or(257, |v| v.0);
            if best < self.options.max_hamming
                && f64::from(best) < self.options.descriptor_ratio * f64::from(next)
            {
                let key = ObservationKey {
                    keyframe: query,
                    camera,
                    feature,
                };
                let pixel = current.features[camera][&feature].pixel;
                correspondences.insert(
                    (camera, feature),
                    Correspondence {
                        key,
                        point,
                        pixel,
                        distance: best,
                    },
                );
            }
        }
        // Add descriptor-supported nearest-pixel matches by projecting candidate
        // landmarks into the current estimate. These are RANSAC hypotheses, not
        // accepted correspondences; large-drift loops retain brute-force matches.
        for (&point, point_descriptors) in &target_descriptors {
            for camera in 0..current.features.len() {
                let Some(pixel) = project(
                    &self.calibration,
                    &current.t_w_b,
                    &map.landmarks[&point].position,
                    camera,
                ) else {
                    continue;
                };
                let nearest = current.features[camera]
                    .iter()
                    .filter(|(id, _)| !correspondences.contains_key(&(camera, **id)))
                    .filter_map(|(&id, feature)| {
                        let descriptor = descriptors.get(&(camera, id))?;
                        let distance = (feature.pixel - pixel).norm();
                        let hamming = point_descriptors
                            .iter()
                            .map(|d| hamming(descriptor, d))
                            .min()?;
                        (distance <= self.options.reprojection_radius
                            && hamming < self.options.max_hamming)
                            .then_some((distance, id, hamming))
                    })
                    .min_by(|a, b| a.0.total_cmp(&b.0));
                if let Some((_, feature, distance)) = nearest {
                    let key = ObservationKey {
                        keyframe: query,
                        camera,
                        feature,
                    };
                    correspondences.insert(
                        (camera, feature),
                        Correspondence {
                            key,
                            point,
                            pixel: current.features[camera][&feature].pixel,
                            distance,
                        },
                    );
                }
            }
        }
        // A landmark may legitimately appear once per camera, but repeated
        // features in one camera must not inflate the geometric consensus.
        let mut unique: BTreeMap<(usize, LandmarkId), Correspondence> = BTreeMap::new();
        for value in correspondences.into_values() {
            let key = (value.key.camera, value.point);
            if unique
                .get(&key)
                .is_none_or(|old| value.distance < old.distance)
            {
                unique.insert(key, value);
            }
        }
        let mut matches = Vec::new();
        let mut rays = Vec::new();
        let mut cameras = Vec::new();
        let mut points = Vec::new();
        for value in unique.into_values() {
            let mut ray = Vector4::zeros();
            if self.calibration.intrinsics[value.key.camera].unproject(&value.pixel, &mut ray, None)
            {
                rays.push(ray.fixed_rows::<3>(0).into_owned().normalize());
                cameras.push(value.key.camera);
                points.push(map.landmarks[&value.point].position);
                matches.push(value);
            }
        }
        if matches.len() < self.options.min_inliers.max(4) {
            return Ok(None);
        }
        let offsets: Vec<_> = self
            .calibration
            .t_i_c
            .iter()
            .map(|pose| pose.translation.vector)
            .collect();
        let rotations: Vec<_> = self
            .calibration
            .t_i_c
            .iter()
            .map(|pose| pose.rotation.to_rotation_matrix().into_inner())
            .collect();
        let mut adapter =
            NoncentralAbsoluteAdapter::new(&rays, &cameras, &points, &offsets, &rotations)
                .map_err(failure)?;
        let backend = RustNumericalBackend;
        let problem = AbsolutePoseSacProblem::new(&mut adapter, Algorithm::Gp3p, &backend, false);
        let mut consensus = Ransac::new(
            problem,
            self.options.max_ransac_iterations,
            self.options.angular_threshold,
            self.options.ransac_probability,
        );
        if !consensus.compute_model(0) || consensus.inliers.len() < self.options.min_inliers {
            return Ok(None);
        }
        let initial = *consensus.model_coefficients.as_ref().unwrap();
        let inliers = consensus.inliers.clone();
        let model = consensus
            .sac_model
            .optimize_model_coefficients(&inliers, &initial)
            .unwrap_or(initial);
        let rotation = Matrix3::from(model.fixed_view::<3, 3>(0, 0));
        let pose = Isometry3::from_parts(
            Translation3::from(model.column(3).into_owned()),
            UnitQuaternion::from_matrix(&rotation),
        );
        let verified: Vec<_> = matches
            .into_iter()
            .filter(|m| {
                project(
                    &self.calibration,
                    &pose,
                    &map.landmarks[&m.point].position,
                    m.key.camera,
                )
                .is_some_and(|pixel| (pixel - m.pixel).norm() <= self.options.reprojection_radius)
            })
            .collect();
        if verified.len() < self.options.min_inliers {
            return Ok(None);
        }
        let mut pairs = BTreeSet::new();
        let mut associations = Vec::new();
        for m in verified {
            if let Some(id) = current.features[m.key.camera][&m.key.feature].landmark
                && id != m.point
            {
                pairs.insert((id, m.point));
            }
            associations.push((m.key, m.point));
        }
        let edge = PoseGraphEdge {
            from: candidate,
            to: query,
            measurement: target.t_w_b.inverse() * pose,
            kind: EdgeKind::Loop,
            sqrt_information: self.options.edge_sqrt_information,
        };
        Ok(Some(LoopCandidate {
            source_revision: map.revision,
            min_inliers: self.options.min_inliers,
            reprojection_radius: self.options.reprojection_radius,
            query,
            candidate,
            verified_t_w_b: pose,
            drift_m: (pose.translation.vector - current.t_w_b.translation.vector).norm(),
            edge,
            landmark_pairs: pairs.into_iter().collect(),
            associations,
        }))
    }
}
fn failure(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("rootslam.loop_closure")
}
pub(crate) fn project(
    calibration: &Calibration<f64>,
    pose: &Isometry3<f64>,
    point: &Vector3<f64>,
    camera: usize,
) -> Option<Vector2<f64>> {
    let camera_pose = pose * calibration.t_i_c.get(camera)?;
    let point = camera_pose.inverse() * nalgebra::Point3::from(*point);
    let mut pixel = Vector2::zeros();
    calibration
        .intrinsics
        .get(camera)?
        .project(&point.coords, &mut pixel, None)
        .then_some(pixel)
}
/// Retrieval islands are connected components of the candidate-induced graph.
/// This grouping/ranking is an implementation choice absent from the paper.
#[must_use]
pub fn candidate_islands(
    map: &MapSnapshot,
    candidates: &BTreeMap<KeyframeId, f64>,
) -> Vec<Vec<KeyframeId>> {
    let mut pending: BTreeSet<_> = candidates.keys().copied().collect();
    let mut islands = Vec::new();
    while let Some(seed) = pending.pop_first() {
        let mut island = vec![seed];
        let mut stack = vec![seed];
        while let Some(id) = stack.pop() {
            for edge in &map.edges {
                let next = if edge.from == id {
                    edge.to
                } else if edge.to == id {
                    edge.from
                } else {
                    continue;
                };
                if pending.remove(&next) {
                    island.push(next);
                    stack.push(next);
                }
            }
        }
        island.sort_by(|a, b| candidates[b].total_cmp(&candidates[a]).then(a.cmp(b)));
        islands.push(island);
    }
    islands.sort_by(|a, b| {
        b.iter()
            .map(|id| candidates[id])
            .sum::<f64>()
            .total_cmp(&a.iter().map(|id| candidates[id]).sum::<f64>())
    });
    islands
}
