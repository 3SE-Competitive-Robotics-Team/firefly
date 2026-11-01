use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use firefly_calibration::camera::pinhole_camera::PinholeCamera;
use firefly_rootslam_mapping::{
    global_optimization::{KrylovPrecision, OptimizationOptions, Precision, optimize_global},
    global_problem::GlobalProblem,
    map::{MapKeyframe, MapLandmark, MapObservation, MapSnapshot, ObservationKey},
    pose_graph::{EdgeKind, PoseGraphEdge},
};
use nalgebra::{Isometry3, SMatrix, Vector2, Vector3, Vector4};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};
struct Directory(PathBuf);
impl Directory {
    fn new(name: &str) -> Self {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../logs")
            .join(format!("rootslam-problem-{name}-{}", std::process::id()));
        std::fs::create_dir_all(&path).unwrap();
        Self(path)
    }
}
impl Drop for Directory {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.0).unwrap();
    }
}
fn scene() -> (MapSnapshot, Calibration<f64>, OptimizationOptions) {
    let camera = GenericCamera::Pinhole(PinholeCamera::new(Vector4::new(300., 310., 0., 0.)));
    let calibration = Calibration {
        intrinsics: vec![camera.clone(), camera],
        t_i_c: vec![Isometry3::identity(), Isometry3::translation(0.2, 0., 0.)],
        resolution: vec![Vector2::new(640, 480); 2],
        ..Default::default()
    };
    let mut map = MapSnapshot {
        revision: 35,
        ..Default::default()
    };
    for frame in 0..2 {
        map.keyframes.insert(
            frame,
            MapKeyframe {
                id: frame,
                t_w_b: Isometry3::translation(frame as f64 * 0.15, 0., 0.),
                active: frame == 1,
                features: vec![BTreeMap::new(); 2],
                images: None,
            },
        );
    }
    for point in 0..5 {
        let truth = Vector3::new(
            0.1 + 0.07 * point as f64,
            -0.08 + 0.03 * point as f64,
            3. + 0.2 * point as f64,
        );
        let mut observations = BTreeMap::new();
        for frame in 0..2 {
            for camera in 0..2 {
                let x = truth.x - 0.2 * camera as f64 - 0.2 * frame as f64;
                observations.insert(
                    ObservationKey {
                        keyframe: frame,
                        camera,
                        feature: point,
                    },
                    MapObservation {
                        pixel: Vector2::new(300. * x / truth.z, 310. * truth.y / truth.z),
                    },
                );
            }
        }
        map.landmarks.insert(
            point as u64,
            MapLandmark {
                id: point as u64,
                host: 0,
                position: truth + Vector3::new(0.1, -0.03, 0.2),
                observations,
            },
        );
    }
    map.edges.push(PoseGraphEdge {
        from: 0,
        to: 1,
        measurement: Isometry3::translation(0.2, 0., 0.),
        kind: EdgeKind::Temporal,
        sqrt_information: SMatrix::identity(),
    });
    (
        map,
        calibration,
        OptimizationOptions {
            max_iterations: 1,
            ..Default::default()
        },
    )
}
fn assert_numbers_equal_bits(a: &serde_json::Value, b: &serde_json::Value) {
    match (a, b) {
        (serde_json::Value::Number(a), serde_json::Value::Number(b)) => {
            if a.is_f64() {
                assert_eq!(a.as_f64().unwrap().to_bits(), b.as_f64().unwrap().to_bits());
            } else {
                assert_eq!(a, b);
            }
        }
        (serde_json::Value::Array(a), serde_json::Value::Array(b)) => {
            assert_eq!(a.len(), b.len());
            for (a, b) in a.iter().zip(b) {
                assert_numbers_equal_bits(a, b);
            }
        }
        (serde_json::Value::Object(a), serde_json::Value::Object(b)) => {
            assert_eq!(a.len(), b.len());
            for (k, v) in a {
                assert_numbers_equal_bits(v, &b[k]);
            }
        }
        _ => assert_eq!(a, b),
    }
}
#[test]
fn archive_preserves_f64_bits_and_refuses_overwrites() {
    let directory = Directory::new("bits");
    let path = directory.0.join("problem.json");
    let (mut map, mut calibration, options) = scene();
    let values = [
        0.,
        -0.,
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        f64::MAX,
        f64::from_bits(0x3ff0_0000_0000_0001),
    ];
    for (i, point) in map.landmarks.values_mut().enumerate() {
        point.position = Vector3::new(values[i], values[i + 1], -values[i]);
    }
    calibration.gyro_noise_std = Vector3::new(
        -0.,
        f64::from_bits(1),
        f64::from_bits(0x3ff0_0000_0000_0001),
    );
    map.edges[0].sqrt_information[(3, 5)] = -0.;
    map.landmarks.get_mut(&0).unwrap().id = u64::MAX;
    map.keyframes.get_mut(&1).unwrap().t_w_b.rotation = nalgebra::UnitQuaternion::new_unchecked(
        nalgebra::Quaternion::new(values[5], -0., values[2], 0.),
    );
    let original = GlobalProblem::capture(&map, &calibration, &options).unwrap();
    original.write_new(&path).unwrap();
    let before = std::fs::read(&path).unwrap();
    assert!(original.write_new(&path).is_err());
    assert_eq!(std::fs::read(&path).unwrap(), before);
    let loaded = GlobalProblem::load(&path).unwrap();
    let (restored, calibration, options) = loaded.restore().unwrap();
    let again = GlobalProblem::capture(&restored, &calibration, &options).unwrap();
    assert_numbers_equal_bits(
        &serde_json::to_value(original).unwrap(),
        &serde_json::to_value(again).unwrap(),
    );
    assert!(
        restored
            .keyframes
            .values()
            .all(|k| k.features.is_empty() && k.images.is_none())
    );
    assert!(restored.keyframes[&1].active);
    let mut invalid = loaded.clone();
    invalid.version += 1;
    assert!(invalid.restore().is_err());
    invalid = loaded.clone();
    invalid.keyframes.push(invalid.keyframes[0].clone());
    assert!(invalid.restore().is_err());
    invalid = loaded.clone();
    let repeated = invalid.landmarks[0].observations[0].clone();
    invalid.landmarks[0].observations.push(repeated);
    assert!(invalid.restore().is_err());
    invalid = loaded;
    invalid.landmarks[0].position[0] = f64::NAN;
    let bad_path = directory.0.join("bad.json");
    assert!(invalid.write_new(&bad_path).is_err());
    assert!(!bad_path.exists());
}
#[test]
fn restored_problem_replays_all_precision_modes_with_identical_results() {
    let directory = Directory::new("replay");
    let (map, calibration, mut options) = scene();
    options.problem_archive = Some(directory.0.join("do-not-recapture.json"));
    options.linear_system_archive = Some(directory.0.join("do-not-capture-linear.json"));
    let problem = GlobalProblem::capture(&map, &calibration, &options).unwrap();
    let path = directory.0.join("problem.json");
    problem.write_new(&path).unwrap();
    let problem = GlobalProblem::load(&path).unwrap();
    for (precision, krylov) in [
        (Precision::Float64, KrylovPrecision::Native),
        (Precision::Float64, KrylovPrecision::Float64),
        (Precision::Float32, KrylovPrecision::Native),
        (Precision::Float32, KrylovPrecision::Float64),
    ] {
        let mut original_options = options.clone();
        original_options.precision = precision;
        original_options.krylov_precision = krylov;
        original_options.problem_archive = None;
        original_options.linear_system_archive = None;
        let direct = optimize_global(&map, &calibration, &original_options).unwrap();
        let replay = problem.replay(precision, krylov).unwrap();
        assert_eq!(direct.initial_cost.to_bits(), replay.initial_cost.to_bits());
        assert_eq!(direct.final_cost.to_bits(), replay.final_cost.to_bits());
        assert_eq!(direct.linear_iterations, replay.linear_iterations);
        assert_eq!(direct.accepted_steps, replay.accepted_steps);
        assert_eq!(direct.converged, replay.converged);
        for (id, position) in &replay.points {
            for (a, b) in direct.points[id].iter().zip(position) {
                assert_eq!(a.to_bits(), b.to_bits());
            }
        }
        for (id, pose) in &replay.poses {
            let direct = &direct.poses[id];
            for (a, b) in direct.translation.vector.iter().zip(pose.translation) {
                assert_eq!(a.to_bits(), b.to_bits());
            }
            let q = direct.rotation.quaternion();
            for (a, b) in [q.i, q.j, q.k, q.w].iter().zip(pose.rotation_xyzw) {
                assert_eq!(a.to_bits(), b.to_bits());
            }
        }
        assert_eq!(
            replay.options.lsqr_tolerance.to_bits(),
            options.lsqr_tolerance.to_bits()
        );
        assert_eq!(
            replay.options.initial_damping.to_bits(),
            options.initial_damping.to_bits()
        );
    }
    let output = problem
        .replay(Precision::Float64, KrylovPrecision::Native)
        .unwrap();
    output.write_new(&directory.0.join("result.json")).unwrap();
    assert!(output.write_new(&directory.0.join("result.json")).is_err());
    assert!(!directory.0.join("do-not-recapture.json").exists());
    assert!(!directory.0.join("do-not-capture-linear.json").exists());
}

#[test]
fn optimizer_hook_freezes_initial_problem_and_replay_records_explicit_budget() {
    let directory = Directory::new("hook");
    let (map, calibration, mut options) = scene();
    let path = directory.0.join("captured.json");
    options.precision = Precision::Float64;
    options.problem_archive = Some(path.clone());
    let first = optimize_global(&map, &calibration, &options).unwrap();
    let problem = GlobalProblem::load(&path).unwrap();
    let (restored, calibration, mut replay_options) = problem.restore().unwrap();
    for (&id, point) in &map.landmarks {
        assert_eq!(point.position, restored.landmarks[&id].position);
    }
    assert_ne!(first.points[&0], map.landmarks[&0].position);
    assert!(optimize_global(&map, &calibration, &options).is_err());
    replay_options.problem_archive = None;
    replay_options.max_iterations = 2;
    let result = optimize_global(&restored, &calibration, &replay_options).unwrap();
    let recorded = firefly_rootslam_mapping::global_problem::ReplayResult::from_result(
        &result,
        replay_options,
    );
    assert_eq!(recorded.options.max_iterations, 2);
    assert_eq!(problem.options.max_iterations, 1);
    assert_eq!(
        recorded.options.lsqr_tolerance.to_bits(),
        problem.options.lsqr_tolerance.to_bits()
    );
}
