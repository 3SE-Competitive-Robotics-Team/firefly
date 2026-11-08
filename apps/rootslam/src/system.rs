//! Asynchronous persistent-map and loop-closing workers, followed by global BA.
pub mod navigation;
use firefly_basalt::vi_estimator::local_map::LocalMapSnapshot;
use firefly_calibration::calibration::Calibration;
use firefly_error::{Error, ErrorKind, Result};
use firefly_rootslam_mapping::{
    global_optimization::{
        OptimizationOptions, OptimizationResult, optimize_global, optimize_pose_graph,
    },
    loop_closure::{LoopCandidate, LoopDetector, LoopOptions, LoopResult},
    map::{MapKeyframe, MapPolicy, MapSnapshot, PersistentMap},
};
use nalgebra::Isometry3;
use navigation::MappingUpdates;
use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    sync::{
        Arc, RwLock,
        mpsc::{self, Receiver, RecvTimeoutError, Sender},
    },
    thread::JoinHandle,
    time::Duration,
};

pub struct SystemOptions {
    pub map: MapPolicy,
    pub loops: LoopOptions,
    pub optimization: OptimizationOptions,
    pub global_optimization: bool,
    /// 回环检测开关；关闭后只保留局部建图与关键帧共视图。
    pub loop_closures: bool,
}
impl Default for SystemOptions {
    fn default() -> Self {
        Self {
            map: MapPolicy::paper_thresholds(),
            loops: LoopOptions::default(),
            optimization: OptimizationOptions::default(),
            global_optimization: true,
            loop_closures: true,
        }
    }
}
pub struct SystemResult {
    pub map: PersistentMap,
    pub trajectory: BTreeMap<i64, Isometry3<f64>>,
    pub loops_verified: usize,
    pub pose_graph_optimizations: usize,
    pub global: Option<OptimizationResult>,
}
pub struct MappingWorker {
    input: Option<Sender<Option<Arc<LocalMapSnapshot>>>>,
    thread: Option<JoinHandle<Result<SystemResult>>>,
    updates: MappingUpdates,
}
fn error(message: &str) -> Error {
    Error::new(ErrorKind::Internal, message).with_operation("rootslam.mapping.worker")
}

impl MappingWorker {
    /// # Errors
    /// Returns thread creation errors.
    pub fn spawn(calibration: Calibration<f64>, options: SystemOptions) -> Result<Self> {
        let (input, receiver) = mpsc::channel();
        let updates = MappingUpdates::default();
        let output = updates.clone();
        let thread = std::thread::Builder::new()
            .name("rootslam-map".into())
            .spawn(move || run(receiver, calibration, options, output))
            .map_err(|source| error("Cannot start map worker").with_source(source))?;
        Ok(Self {
            input: Some(input),
            thread: Some(thread),
            updates,
        })
    }
    /// # Panics
    /// Requires the worker not to have been joined.
    #[must_use]
    pub fn input(&self) -> Sender<Option<Arc<LocalMapSnapshot>>> {
        self.input.as_ref().expect("map worker joined").clone()
    }
    #[must_use]
    pub fn updates(&self) -> MappingUpdates {
        self.updates.clone()
    }
    /// # Errors
    /// Reports a worker failure before the online session publishes stale state.
    pub fn check_running(&mut self) -> Result<()> {
        if self.thread.as_ref().is_some_and(JoinHandle::is_finished) {
            self.thread
                .take()
                .ok_or_else(|| error("Map worker has already been joined"))?
                .join()
                .map_err(|_| error("Map worker panicked"))??;
            return Err(error("Map worker stopped before end of input"));
        }
        Ok(())
    }
    /// Signals end of input and waits for loop processing and final global BA.
    /// # Errors
    /// Propagates processing and worker panic errors.
    pub fn finish(mut self) -> Result<SystemResult> {
        if let Some(input) = self.input.take() {
            let _ = input.send(None);
        }
        self.thread
            .take()
            .ok_or_else(|| error("Map worker has already been joined"))?
            .join()
            .map_err(|_| error("Map worker panicked"))?
    }
}
impl Drop for MappingWorker {
    fn drop(&mut self) {
        if let Some(input) = self.input.take() {
            let _ = input.send(None);
        }
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

enum LoopJob {
    Detect(Arc<MapKeyframe>),
    Correct(Arc<MapSnapshot>, Box<LoopCandidate>),
}
type LatestMap = Arc<RwLock<Arc<MapSnapshot>>>;

fn detect_latest(
    detector: &mut LoopDetector,
    latest: &LatestMap,
    query: &MapKeyframe,
) -> Result<LoopResult> {
    // The lock protects only an Arc pointer; geometric work never holds it.
    let snapshot = {
        let guard = latest
            .read()
            .map_err(|_| error("Latest map pointer poisoned"))?;
        Arc::clone(&guard)
    };
    if !snapshot.keyframes.contains_key(&query.id) {
        return Ok(LoopResult {
            keyframe: query.id,
            descriptors: BTreeMap::new(),
            candidate: None,
        });
    }
    detector.process(&snapshot, query.id)
}

fn publish_map(latest: &LatestMap, map: &MapSnapshot) -> Result<Arc<MapSnapshot>> {
    // Only the current detector, current published version and at most one PGO
    // retain full snapshots. Queued detections retain their query payload alone.
    let snapshot = Arc::new(map.clone());
    *latest
        .write()
        .map_err(|_| error("Latest map pointer poisoned"))? = Arc::clone(&snapshot);
    Ok(snapshot)
}

fn correction_snapshot(
    map: &MapSnapshot,
    loop_edge: &firefly_rootslam_mapping::pose_graph::PoseGraphEdge,
) -> Arc<MapSnapshot> {
    let mut snapshot = map.clone();
    // Section III-B: existing graph edges are measured from the current
    // estimates for this PGO request; only the new loop uses its gP3P estimate.
    // The solver selects temporal and loop edges. Covisibility and spanning
    // edges remain map topology; distinct additional constraints would change
    // the optimized objective even with current-pose relative measurements.
    snapshot.edges.retain_mut(|edge| {
        if !edge.kind.is_measurement_bearing() {
            return true;
        }
        let (Some(first), Some(second)) = (
            snapshot.keyframes.get(&edge.from),
            snapshot.keyframes.get(&edge.to),
        ) else {
            return false;
        };
        edge.measurement = first.t_w_b.inverse() * second.t_w_b;
        true
    });
    if let Some(existing) = snapshot.edges.iter_mut().find(|edge| {
        edge.kind == loop_edge.kind && edge.from == loop_edge.from && edge.to == loop_edge.to
    }) {
        *existing = loop_edge.clone();
    } else {
        snapshot.edges.push(loop_edge.clone());
    }
    // Do not insert a tentative constraint into the live map before the
    // optimization succeeds. Its result keeps the source geometry revision.
    Arc::new(snapshot)
}
enum LoopEvent {
    Detected(Box<LoopResult>),
    Corrected(OptimizationResult, Box<LoopCandidate>),
}
struct LoopWorker {
    jobs: Sender<LoopJob>,
    events: Receiver<Result<LoopEvent>>,
    thread: JoinHandle<()>,
}
impl LoopWorker {
    fn spawn(
        calibration: Calibration<f64>,
        loops: LoopOptions,
        mut optimization: OptimizationOptions,
        latest: LatestMap,
    ) -> Result<Self> {
        loops.validate()?;
        optimization.max_iterations = loops.pose_graph_max_iterations;
        let (jobs, receiver) = mpsc::channel();
        let (sender, events) = mpsc::channel();
        let thread = std::thread::Builder::new()
            .name("rootslam-loop".into())
            .spawn(move || {
                let mut detector = LoopDetector::new(calibration, loops);
                while let Ok(job) = receiver.recv() {
                    let result = match job {
                        LoopJob::Detect(query) => detect_latest(&mut detector, &latest, &query)
                            .map(|result| LoopEvent::Detected(Box::new(result))),
                        LoopJob::Correct(map, candidate) => {
                            optimize_pose_graph(&map, &optimization)
                                .map(|result| LoopEvent::Corrected(result, candidate))
                        }
                    };
                    if sender.send(result).is_err() {
                        break;
                    }
                }
            })
            .map_err(|source| error("Cannot start loop worker").with_source(source))?;
        Ok(Self {
            jobs,
            events,
            thread,
        })
    }
}

struct MapWorkerState {
    input: Receiver<Option<Arc<LocalMapSnapshot>>>,
    latest: LatestMap,
    map: PersistentMap,
    seen: BTreeSet<i64>,
    anchors: BTreeMap<i64, (i64, Isometry3<f64>)>,
    original_poses: BTreeMap<i64, Isometry3<f64>>,
    pending: usize,
    correction_pending: bool,
    candidates: VecDeque<LoopCandidate>,
    loops_verified: usize,
    pose_graph_optimizations: usize,
    loop_closures: bool,
    updates: MappingUpdates,
    navigation_origin: Option<(i64, Isometry3<f64>)>,
    latest_local_ns: i64,
    corrections_rejected: usize,
    detections_processed: usize,
}
impl MapWorkerState {
    fn handle_event(&mut self, event: LoopEvent) -> Result<()> {
        let mut accepted_revision = None;
        match event {
            LoopEvent::Detected(result) => {
                self.detections_processed += 1;
                self.map
                    .apply_descriptors(result.keyframe, &result.descriptors);
                if let Some(candidate) = result
                    .candidate
                    .and_then(|candidate| self.map.revalidate_loop(&candidate))
                {
                    self.loops_verified += 1;
                    if candidate.drift_m < 0.1 {
                        self.map.apply_loop_associations(&candidate);
                    } else {
                        self.candidates.push_back(candidate);
                    }
                }
            }
            LoopEvent::Corrected(result, candidate) => {
                self.correction_pending = false;
                if usable_correction(&result)
                    && self.map.apply_loop_correction(
                        result.revision,
                        &result.poses,
                        &result.points,
                        &candidate,
                    )
                {
                    self.pose_graph_optimizations += 1;
                    accepted_revision = Some(result.revision);
                } else {
                    self.corrections_rejected += 1;
                }
            }
        }
        self.publish_navigation(accepted_revision)
    }
    fn dispatch_correction(&mut self, jobs: &Sender<LoopJob>) -> Result<()> {
        if self.correction_pending {
            return Ok(());
        }
        while let Some(candidate) = self.candidates.pop_front() {
            // Revalidate again after waiting behind another correction.
            let Some(candidate) = self.map.revalidate_loop(&candidate) else {
                continue;
            };
            if candidate.drift_m < 0.1 {
                self.map.apply_loop_associations(&candidate);
                publish_map(&self.latest, &self.map.snapshot)?;
                continue;
            }
            let snapshot = correction_snapshot(&self.map.snapshot, &candidate.edge);
            jobs.send(LoopJob::Correct(snapshot, Box::new(candidate)))
                .map_err(|_| error("Loop worker disconnected"))?;
            self.pending += 1;
            self.correction_pending = true;
            break;
        }
        Ok(())
    }
    fn ingest(&mut self, local: &LocalMapSnapshot, jobs: &Sender<LoopJob>) -> Result<()> {
        if local.t_ns < self.latest_local_ns {
            return Err(error("local map timestamp regressed"));
        }
        self.latest_local_ns = local.t_ns;
        if self.navigation_origin.is_none()
            && let Some((&id, pose)) = local
                .keyframes
                .iter()
                .find_map(|id| local.poses.get(id).map(|pose| (id, pose)))
        {
            self.navigation_origin = Some((id, *pose));
        }
        for (&time, pose) in &local.poses {
            self.original_poses.insert(time, *pose);
            if let Some(&anchor) = local
                .keyframes
                .range(..=time)
                .next_back()
                .or_else(|| local.keyframes.first())
                && let Some(anchor_pose) = local.poses.get(&anchor)
            {
                self.anchors
                    .insert(time, (anchor, anchor_pose.inverse() * pose));
            }
        }
        self.map.ingest(local);
        publish_map(&self.latest, &self.map.snapshot)?;
        if !self.loop_closures {
            return Ok(());
        }
        for &keyframe in &local.keyframes {
            if self.seen.insert(keyframe) && self.map.snapshot.keyframes.contains_key(&keyframe) {
                jobs.send(LoopJob::Detect(Arc::new(
                    self.map.snapshot.keyframes[&keyframe].clone(),
                )))
                .map_err(|_| error("Loop worker disconnected"))?;
                self.pending += 1;
            }
        }
        Ok(())
    }
    fn process(&mut self, loops: &LoopWorker) -> Result<()> {
        let mut ended = false;
        while !ended || self.pending > 0 || !self.candidates.is_empty() {
            let mut changed = false;
            while let Ok(event) = loops.events.try_recv() {
                self.pending = self
                    .pending
                    .checked_sub(1)
                    .ok_or_else(|| error("Unexpected loop worker event"))?;
                self.handle_event(event?)?;
                changed = true;
            }
            if changed {
                publish_map(&self.latest, &self.map.snapshot)?;
            }
            self.dispatch_correction(&loops.jobs)?;
            if loops.thread.is_finished() && self.pending > 0 {
                return Err(error("Loop worker exited before completing queued work"));
            }
            if ended {
                if self.pending > 0 {
                    std::thread::sleep(Duration::from_millis(1));
                }
                continue;
            }
            match self.input.recv_timeout(Duration::from_millis(5)) {
                Ok(Some(local)) => self.ingest(&local, &loops.jobs)?,
                Ok(None) | Err(RecvTimeoutError::Disconnected) => {
                    ended = true;
                    log::info!(
                        "RootSLAM local input ended: pending_loop_jobs={} queued_candidates={}",
                        self.pending,
                        self.candidates.len()
                    );
                }
                Err(RecvTimeoutError::Timeout) => {}
            }
        }
        log::info!(
            "RootSLAM loop processing complete: keyframes={} landmarks={} verified_loops={} pose_graph_optimizations={}",
            self.map.snapshot.keyframes.len(),
            self.map.snapshot.landmarks.len(),
            self.loops_verified,
            self.pose_graph_optimizations
        );
        Ok(())
    }
    fn finish(
        mut self,
        global_enabled: bool,
        options: &OptimizationOptions,
    ) -> Result<SystemResult> {
        self.map.finish_local();
        let global = if global_enabled && !self.map.snapshot.keyframes.is_empty() {
            log::info!(
                "RootSLAM starting final global BA: keyframes={} landmarks={}",
                self.map.snapshot.keyframes.len(),
                self.map.snapshot.landmarks.len()
            );
            let result = optimize_global(&self.map.snapshot, &self.map.calibration, options)?;
            if !self
                .map
                .apply_updates(result.revision, &result.poses, &result.points)
            {
                return Err(error("Final global optimization revision rejected"));
            }
            Some(result)
        } else {
            None
        };
        let trajectory = self
            .original_poses
            .into_iter()
            .map(|(time, original)| {
                let corrected = self
                    .anchors
                    .get(&time)
                    .and_then(|(anchor, relative)| {
                        self.map.resolved_pose(*anchor).map(|pose| pose * relative)
                    })
                    .unwrap_or(original);
                (time, corrected)
            })
            .collect();
        Ok(SystemResult {
            map: self.map,
            trajectory,
            loops_verified: self.loops_verified,
            pose_graph_optimizations: self.pose_graph_optimizations,
            global,
        })
    }
}

fn run(
    input: Receiver<Option<Arc<LocalMapSnapshot>>>,
    calibration: Calibration<f64>,
    options: SystemOptions,
    updates: MappingUpdates,
) -> Result<SystemResult> {
    let latest = Arc::new(RwLock::new(Arc::new(MapSnapshot::default())));
    let loops = LoopWorker::spawn(
        calibration.clone(),
        options.loops,
        options.optimization.clone(),
        Arc::clone(&latest),
    )?;
    let mut state = MapWorkerState {
        input,
        latest,
        map: PersistentMap::new(calibration, options.map)?,
        seen: BTreeSet::new(),
        anchors: BTreeMap::new(),
        original_poses: BTreeMap::new(),
        pending: 0,
        correction_pending: false,
        candidates: VecDeque::new(),
        loops_verified: 0,
        pose_graph_optimizations: 0,
        loop_closures: options.loop_closures,
        updates,
        navigation_origin: None,
        latest_local_ns: i64::MIN,
        corrections_rejected: 0,
        detections_processed: 0,
    };
    let processing = state.process(&loops);
    drop(loops.jobs);
    let joined = loops
        .thread
        .join()
        .map_err(|_| error("Loop worker panicked"));
    processing?;
    joined?;
    state.finish(options.global_optimization, &options.optimization)
}

fn usable_correction(result: &OptimizationResult) -> bool {
    result.initial_cost.is_finite()
        && result.final_cost.is_finite()
        && result.final_cost >= 0.
        && result.final_cost <= result.initial_cost
        && (result.converged
            || (result.accepted_steps > 0 && result.final_cost < result.initial_cost))
        && result
            .poses
            .values()
            .all(|pose| pose.to_homogeneous().iter().all(|v| v.is_finite()))
        && result
            .points
            .values()
            .all(|point| point.iter().all(|v| v.is_finite()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn online_poll_reports_a_failed_mapping_worker_before_shutdown() {
        let (input, _receiver) = mpsc::channel();
        let thread = std::thread::spawn(|| Err(error("missing navigation anchor")));
        while !thread.is_finished() {
            std::thread::yield_now();
        }
        let mut worker = MappingWorker {
            input: Some(input),
            thread: Some(thread),
            updates: MappingUpdates::default(),
        };
        assert!(
            worker
                .check_running()
                .unwrap_err()
                .to_string()
                .contains("missing navigation anchor")
        );
    }

    /// 建图 worker 的最小夹具；地图里预置一个关键帧，使回环检测有候选。
    fn ingest_fixture(loop_closures: bool) -> (MapWorkerState, Sender<LoopJob>, Receiver<LoopJob>) {
        let (_, input) = mpsc::channel();
        let mut map =
            PersistentMap::new(Calibration::default(), MapPolicy::paper_thresholds()).unwrap();
        map.snapshot.keyframes.insert(
            3,
            MapKeyframe {
                id: 3,
                t_w_b: Isometry3::identity(),
                active: true,
                features: vec![],
                images: None,
            },
        );
        let latest = Arc::new(RwLock::new(Arc::new(map.snapshot.clone())));
        let (jobs, detected) = mpsc::channel();
        let state = MapWorkerState {
            input,
            latest,
            map,
            seen: BTreeSet::new(),
            anchors: BTreeMap::new(),
            original_poses: BTreeMap::new(),
            pending: 0,
            correction_pending: false,
            candidates: VecDeque::new(),
            loops_verified: 0,
            pose_graph_optimizations: 0,
            loop_closures,
            updates: MappingUpdates::default(),
            navigation_origin: None,
            latest_local_ns: i64::MIN,
            corrections_rejected: 0,
            detections_processed: 0,
        };
        (state, jobs, detected)
    }

    fn keyframe_snapshot() -> LocalMapSnapshot {
        LocalMapSnapshot {
            t_ns: 1,
            poses: BTreeMap::from([(3, Isometry3::identity())]),
            keyframes: BTreeSet::from([3]),
            landmarks: BTreeMap::new(),
            frames: BTreeMap::new(),
            rejected_landmarks: BTreeSet::new(),
            rejected_observations: BTreeSet::new(),
        }
    }

    #[test]
    fn disabled_loop_closures_keep_the_local_map_without_detecting_loops() {
        let (mut state, jobs, detected) = ingest_fixture(false);
        state.ingest(&keyframe_snapshot(), &jobs).unwrap();
        assert!(
            detected.try_recv().is_err(),
            "disabled loop closures must not enqueue detection jobs"
        );
        assert_eq!(state.map.snapshot.keyframes.len(), 1);
        assert_eq!(state.pending, 0);
    }

    #[test]
    fn enabled_loop_closures_enqueue_detection_for_new_keyframes() {
        let (mut state, jobs, detected) = ingest_fixture(true);
        state.ingest(&keyframe_snapshot(), &jobs).unwrap();
        assert!(matches!(detected.try_recv(), Ok(LoopJob::Detect(_))));
        assert_eq!(state.pending, 1);
    }

    #[test]
    fn loop_worker_uses_separate_default_and_explicit_pose_graph_budgets() {
        use firefly_rootslam_mapping::pose_graph::{EdgeKind, PoseGraphEdge};
        let mut map = MapSnapshot::default();
        for id in 0..2 {
            map.keyframes.insert(
                id,
                MapKeyframe {
                    id,
                    t_w_b: Isometry3::translation(id as f64 * 2., 0., 0.),
                    active: id == 0,
                    features: vec![],
                    images: None,
                },
            );
        }
        map.edges.push(PoseGraphEdge {
            from: 0,
            to: 1,
            measurement: Isometry3::translation(1., 0., 0.),
            kind: EdgeKind::Loop,
            sqrt_information: nalgebra::SMatrix::identity(),
        });
        let optimization = OptimizationOptions {
            // High damping keeps this well-conditioned problem away from its
            // optimum long enough to observe the actual worker iteration cap.
            initial_damping: 1e10,
            step_tolerance: 0.,
            relative_cost_tolerance: 0.,
            ..OptimizationOptions::default()
        };
        assert_eq!(optimization.max_iterations, 50);
        for (configuration, expected) in [("{}", 20), (r#"{"pose_graph_max_iterations":1}"#, 1)] {
            let loops: LoopOptions = serde_json::from_str(configuration).unwrap();
            let latest = Arc::new(RwLock::new(Arc::new(map.clone())));
            let worker =
                LoopWorker::spawn(Calibration::default(), loops, optimization.clone(), latest)
                    .unwrap();
            let (_, candidate, _) = correction_fixture();
            worker
                .jobs
                .send(LoopJob::Correct(Arc::new(map.clone()), Box::new(candidate)))
                .unwrap();
            let event = worker.events.recv().unwrap().unwrap();
            drop(worker.jobs);
            worker.thread.join().unwrap();
            let LoopEvent::Corrected(result, _) = event else {
                panic!("expected PGO result");
            };
            assert_eq!(result.iterations, expected);
            assert_eq!(result.poses[&0], map.keyframes[&0].t_w_b);
            assert!(result.final_cost < result.initial_cost);
        }
        let invalid: LoopOptions =
            serde_json::from_str(r#"{"pose_graph_max_iterations":0}"#).unwrap();
        assert!(invalid.validate().is_err());
    }
    fn correction_map() -> (PersistentMap, firefly_rootslam_mapping::map::ObservationKey) {
        use firefly_calibration::camera::generic_camera::GenericCamera;
        use firefly_calibration::camera::pinhole_camera::PinholeCamera;
        use firefly_rootslam_mapping::map::{
            MapFeature, MapLandmark, MapObservation, ObservationKey,
        };
        let calibration = Calibration {
            intrinsics: vec![GenericCamera::Pinhole(PinholeCamera::new(
                nalgebra::Vector4::new(100., 100., 0., 0.),
            ))],
            t_i_c: vec![Isometry3::identity()],
            ..Default::default()
        };
        let mut map = PersistentMap::new(calibration, MapPolicy::paper_thresholds()).unwrap();
        let key = ObservationKey {
            keyframe: 1,
            camera: 0,
            feature: 7,
        };
        for id in 0..3 {
            map.snapshot.keyframes.insert(
                id,
                MapKeyframe {
                    id,
                    t_w_b: if id == 1 {
                        Isometry3::translation(0.2, 0., 0.)
                    } else {
                        Isometry3::identity()
                    },
                    active: id == 1,
                    features: vec![if id == 1 {
                        BTreeMap::from([(
                            7,
                            MapFeature {
                                pixel: nalgebra::Vector2::zeros(),
                                descriptor: None,
                                landmark: Some(7),
                            },
                        )])
                    } else {
                        BTreeMap::new()
                    }],
                    images: None,
                },
            );
        }
        map.snapshot.landmarks.insert(
            7,
            MapLandmark {
                id: 7,
                host: 2,
                position: nalgebra::Vector3::new(0., 0., 2.),
                observations: BTreeMap::from([(
                    key,
                    MapObservation {
                        pixel: nalgebra::Vector2::zeros(),
                    },
                )]),
            },
        );
        (map, key)
    }
    fn correction_fixture() -> (MapWorkerState, LoopCandidate, OptimizationResult) {
        use firefly_rootslam_mapping::pose_graph::{EdgeKind, PoseGraphEdge};
        let (map, key) = correction_map();
        let candidate = LoopCandidate {
            source_revision: 0,
            min_inliers: 1,
            reprojection_radius: 3.,
            query: 1,
            candidate: 0,
            verified_t_w_b: Isometry3::identity(),
            drift_m: 0.2,
            edge: PoseGraphEdge {
                from: 0,
                to: 1,
                measurement: Isometry3::identity(),
                kind: EdgeKind::Loop,
                sqrt_information: nalgebra::SMatrix::identity(),
            },
            landmark_pairs: vec![],
            associations: vec![(key, 7)],
        };
        let result = OptimizationResult {
            revision: 0,
            poses: BTreeMap::from([
                (0, Isometry3::translation(0.2, 0., 0.)),
                (1, Isometry3::translation(0.2, 0., 0.)),
                (2, Isometry3::identity()),
            ]),
            points: BTreeMap::new(),
            initial_cost: 1.,
            final_cost: 0.5,
            iterations: 1,
            accepted_steps: 1,
            converged: true,
            linear_iterations: 1,
            peak_linear_storage_bytes: 0,
            precision: firefly_rootslam_mapping::global_optimization::Precision::Float64,
            krylov_precision:
                firefly_rootslam_mapping::global_optimization::KrylovPrecision::Float64,
        };
        let (_, input) = mpsc::channel();
        let latest = Arc::new(RwLock::new(Arc::new(map.snapshot.clone())));
        (
            MapWorkerState {
                input,
                latest,
                map,
                seen: BTreeSet::new(),
                anchors: BTreeMap::new(),
                original_poses: BTreeMap::new(),
                pending: 0,
                correction_pending: true,
                candidates: VecDeque::new(),
                loops_verified: 1,
                pose_graph_optimizations: 0,
                loop_closures: true,
                updates: MappingUpdates::default(),
                navigation_origin: None,
                latest_local_ns: i64::MIN,
                corrections_rejected: 0,
                detections_processed: 0,
            },
            candidate,
            result,
        )
    }
    #[test]
    fn rejected_post_correction_geometry_does_not_partially_commit_pose_graph() {
        let (mut state, candidate, result) = correction_fixture();
        assert!(state.map.revalidate_loop(&candidate).is_some());
        state
            .handle_event(LoopEvent::Corrected(result, Box::new(candidate)))
            .unwrap();
        assert_eq!(
            state.map.snapshot.keyframes[&0].t_w_b,
            Isometry3::identity()
        );
        assert_eq!(state.map.snapshot.revision, 0);
        assert!(state.map.snapshot.edges.is_empty());
        assert_eq!(state.pose_graph_optimizations, 0);
    }
    #[test]
    fn accepted_correction_commits_geometry_and_loop_edge_together() {
        let (mut state, candidate, result) = correction_fixture();
        state.navigation_origin = Some((0, Isometry3::identity()));
        state.latest_local_ns = 2;
        state
            .original_poses
            .insert(1, state.map.snapshot.keyframes[&1].t_w_b);
        state.map.snapshot.landmarks.get_mut(&7).unwrap().host = 0;
        state
            .handle_event(LoopEvent::Corrected(result, Box::new(candidate)))
            .unwrap();
        assert_eq!(
            state.map.snapshot.keyframes[&0].t_w_b.translation.vector.x,
            0.2
        );
        assert_eq!(state.map.snapshot.landmarks[&7].position.x, 0.2);
        assert_eq!(state.map.snapshot.edges.len(), 1);
        assert_eq!(state.pose_graph_optimizations, 1);
        let correction = state.updates.snapshot().unwrap().correction.unwrap();
        assert_eq!(correction.revision, 1);
        assert!((correction.map_from_odom.translation.vector.x + 0.2).abs() < 1e-12);
        assert_eq!(
            state.map.snapshot.keyframes[&1].t_w_b,
            state.original_poses[&1]
        );
        state.map.snapshot.keyframes.get_mut(&0).unwrap().t_w_b =
            Isometry3::translation(0.4, 0., 0.);
        state.publish_navigation(Some(1)).unwrap();
        let next = state.updates.snapshot().unwrap().correction.unwrap();
        assert_eq!(next.revision, 2);
        assert!((next.map_from_odom.translation.vector.x + 0.4).abs() < 1e-12);
        assert_eq!(next.anchor_ns, correction.anchor_ns);
    }
    #[test]
    fn failed_solver_never_commits_a_map_or_navigation_revision() {
        for invalid_cost in [false, true] {
            let (mut state, candidate, mut result) = correction_fixture();
            state.map.snapshot.landmarks.get_mut(&7).unwrap().host = 0;
            result.accepted_steps = 0;
            result.converged = false;
            result.final_cost = if invalid_cost {
                f64::NAN
            } else {
                result.initial_cost
            };
            state
                .handle_event(LoopEvent::Corrected(result, Box::new(candidate)))
                .unwrap();
            assert_eq!(state.map.snapshot.revision, 0);
            let update = state.updates.snapshot().unwrap();
            assert_eq!(update.accepted, 0);
            assert_eq!(update.rejected, 1);
            assert!(update.correction.is_none());
        }
    }
    #[test]
    fn pgo_uses_current_edges_and_only_new_loop_keeps_measurement() {
        use firefly_rootslam_mapping::pose_graph::{EdgeKind, PoseGraphEdge};
        let mut map = MapSnapshot::default();
        for id in 0..3 {
            map.keyframes.insert(
                id,
                MapKeyframe {
                    id,
                    t_w_b: Isometry3::translation(id as f64 * 2.0, 0.0, 0.0),
                    active: false,
                    features: vec![],
                    images: None,
                },
            );
        }
        let edge = |from, to, x, kind| PoseGraphEdge {
            from,
            to,
            measurement: Isometry3::translation(x, 0.0, 0.0),
            kind,
            sqrt_information: nalgebra::SMatrix::identity(),
        };
        map.edges.push(edge(0, 1, 0.3, EdgeKind::Temporal));
        map.edges.push(edge(1, 2, 0.4, EdgeKind::Loop));
        let new_loop = edge(0, 2, 3.5, EdgeKind::Loop);
        let request = correction_snapshot(&map, &new_loop);
        assert_eq!(request.edges[0].measurement.translation.vector.x, 2.0);
        assert_eq!(request.edges[1].measurement.translation.vector.x, 2.0);
        assert_eq!(request.edges[2].measurement, new_loop.measurement);
        assert_eq!(map.edges.len(), 2);
        assert_eq!(map.edges[0].measurement.translation.vector.x, 0.3);
        assert_eq!(request.revision, map.revision);
    }
    #[test]
    fn queued_detection_does_not_retain_full_snapshots_and_culled_query_is_skipped() {
        let snapshot = Arc::new(MapSnapshot::default());
        let latest: LatestMap = Arc::new(RwLock::new(Arc::clone(&snapshot)));
        let query = Arc::new(MapKeyframe {
            id: 10,
            t_w_b: Isometry3::identity(),
            active: false,
            features: vec![],
            images: None,
        });
        let jobs: Vec<_> = (0..100)
            .map(|_| LoopJob::Detect(Arc::clone(&query)))
            .collect();
        assert_eq!(
            Arc::strong_count(&snapshot),
            2,
            "queued jobs must not retain an entire map version"
        );
        let mut detector = LoopDetector::new(Calibration::default(), LoopOptions::default());
        for job in jobs {
            if let LoopJob::Detect(query) = job {
                let result = detect_latest(&mut detector, &latest, &query).unwrap();
                assert_eq!(result.keyframe, 10);
                assert!(result.candidate.is_none());
            }
        }
    }
}
