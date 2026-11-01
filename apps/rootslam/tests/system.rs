use firefly_basalt::{optical_flow::OpticalFlowResult, vi_estimator::local_map::LocalMapSnapshot};
use firefly_calibration::calibration::Calibration;
use firefly_rootslam::system::{MappingWorker, SystemOptions};
use nalgebra::Isometry3;
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::{Arc, mpsc},
    time::Duration,
};

#[test]
fn workers_drain_keyframes_and_global_optimization_before_returning_trajectory() {
    let mut options = SystemOptions::default();
    options.optimization.precision =
        firefly_rootslam_mapping::global_optimization::Precision::Float64;
    let worker = MappingWorker::spawn(Calibration::default(), options).unwrap();
    let input = worker.input();
    let mut poses = BTreeMap::new();
    let mut frames = BTreeMap::new();
    let mut keyframes = BTreeSet::new();
    for time in [10_i64, 20, 30] {
        poses.insert(time, Isometry3::translation(time as f64 * 0.01, 0.0, 0.0));
        frames.insert(
            time,
            Arc::new(OpticalFlowResult {
                t_ns: time,
                ..OpticalFlowResult::default()
            }),
        );
        keyframes.insert(time);
        input
            .send(Some(Arc::new(LocalMapSnapshot {
                t_ns: time,
                poses: poses.clone(),
                keyframes: keyframes.clone(),
                landmarks: BTreeMap::new(),
                frames: frames.clone(),
                rejected_landmarks: BTreeSet::new(),
                rejected_observations: BTreeSet::new(),
            })))
            .unwrap();
    }
    input.send(None).unwrap();
    drop(input);
    let (sender, receiver) = mpsc::channel();
    std::thread::spawn(move || {
        sender.send(worker.finish()).unwrap();
    });
    let result = receiver
        .recv_timeout(Duration::from_secs(10))
        .expect("worker shutdown must finish")
        .unwrap();
    assert_eq!(result.map.snapshot.keyframes.len(), 3);
    assert!(
        result
            .map
            .snapshot
            .keyframes
            .values()
            .all(|frame| !frame.active)
    );
    assert_eq!(result.trajectory.len(), 3);
    for (time, pose) in poses {
        assert!((result.trajectory[&time].to_homogeneous() - pose.to_homogeneous()).norm() < 1e-12);
    }
    assert_eq!(result.loops_verified, 0);
    assert!(result.global.is_some());
}

#[test]
fn queued_keyframes_can_be_culled_while_loop_worker_drains() {
    let mut options = SystemOptions {
        global_optimization: false,
        ..Default::default()
    };
    options.map.max_keyframes = 3;
    options.map.cull_batch = 1;
    let worker = MappingWorker::spawn(Calibration::default(), options).unwrap();
    let input = worker.input();
    for time in 1_i64..=100 {
        let pose = Isometry3::translation(time as f64, 0.0, 0.0);
        input
            .send(Some(Arc::new(LocalMapSnapshot {
                t_ns: time,
                poses: BTreeMap::from([(time, pose)]),
                keyframes: BTreeSet::from([time]),
                landmarks: BTreeMap::new(),
                frames: BTreeMap::from([(
                    time,
                    Arc::new(OpticalFlowResult {
                        t_ns: time,
                        ..Default::default()
                    }),
                )]),
                rejected_landmarks: BTreeSet::new(),
                rejected_observations: BTreeSet::new(),
            })))
            .unwrap();
    }
    drop(input);
    let (sender, receiver) = mpsc::channel();
    std::thread::spawn(move || {
        sender.send(worker.finish()).unwrap();
    });
    let result = receiver
        .recv_timeout(Duration::from_secs(10))
        .expect("culled queued queries must drain safely")
        .unwrap();
    assert!(result.map.snapshot.keyframes.len() <= 3);
    assert_eq!(result.trajectory.len(), 100);
    for time in 1_i64..=100 {
        assert!((result.trajectory[&time].translation.vector.x - time as f64).abs() < 1e-12);
    }
}
