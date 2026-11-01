use firefly_basalt::{mapper_sim::MapperSimulation, mapper_sim_naive::NaiveMapperSimulation};
use nalgebra::Vector3;
use serde_json::Value;
use std::path::PathBuf;
fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/reference/mapper_sim_naive")
}
fn check_vectors(actual: &[Vector3<f64>], expected: &Value) {
    let expected = expected.as_array().unwrap();
    assert_eq!(actual.len(), expected.len());
    for (vector, row) in actual.iter().zip(expected) {
        for index in 0..3 {
            assert!((vector[index] - row[index].as_f64().unwrap()).abs() < 1e-10);
        }
    }
}
#[test]
fn original_gen_data_preserves_duplicate_fields_and_keyframe_selection() {
    let path = fixture();
    let input = NaiveMapperSimulation::load(&path.join("calibration.json"), &path).unwrap();
    let expected: Value =
        serde_json::from_str(include_str!("reference/mapper_sim_naive.json")).unwrap();
    assert_eq!(
        serde_json::to_value(&input.keyframe_timestamps).unwrap(),
        expected["keyframes"]
    );
    assert_eq!(
        serde_json::to_value(&input.frame_timestamps).unwrap(),
        expected["frames"]
    );
    assert_eq!(
        serde_json::to_value(&input.imu_timestamps).unwrap(),
        expected["imu"]
    );
    check_vectors(&input.accel, &expected["accel"]);
    check_vectors(&input.noisy_accel, &expected["noisy_accel"]);
    check_vectors(&input.gt_positions, &expected["positions"]);
    check_vectors(&input.points, &expected["points"]);
    assert_ne!(input.accel.last(), input.noisy_accel.last());
    for &time in &input.keyframe_timestamps {
        let image = input.vision_input(time).unwrap();
        assert_eq!(image.observations.len(), 2);
        assert!(image.observations.iter().all(|camera| camera.len() == 8));
    }
    let mut index = 0;
    assert!(input.next_step(&mut index));
    assert!(input.next_step(&mut index));
    assert!(!input.next_step(&mut index));
    assert_eq!(index, 2);
}
#[test]
fn naive_replay_runs_both_input_streams_and_preserves_full_state_window() {
    let path = fixture();
    let mut input = NaiveMapperSimulation::load(&path.join("calibration.json"), &path).unwrap();
    let output = input.run().unwrap();
    assert_eq!(output.states.len(), 3);
    assert_eq!(output.diagnostics.len(), 3);
    assert_eq!(output.visualizations[&200_000_000].states.len(), 3);
    assert!(input.align(&output).unwrap().is_finite());
}
#[test]
fn mapper_cache_builds_five_view_tracks_and_executes_original_headless_flow() {
    let path = fixture();
    let mut mapper = MapperSimulation::load(&path.join("calibration.json"), &path).unwrap();
    mapper.run().unwrap();
    assert_eq!(mapper.mapper.feature_tracks.len(), 8);
    assert_ne!(mapper.points, [] as [Vector3<f64>; 0]);
    assert!(
        mapper
            .points
            .iter()
            .all(|p| p.iter().all(|value| value.is_finite()))
    );
    assert!(mapper.align().is_finite());
}

#[test]
fn missing_vision_input_finishes_imu_producer_and_reports_error() {
    let path = fixture();
    let mut input = NaiveMapperSimulation::load(&path.join("calibration.json"), &path).unwrap();
    input.noisy_observations.clear();
    assert!(input.run().is_err());
}

#[test]
fn mapper_random_callbacks_consume_the_original_eigen_stream() {
    let path = fixture();
    let mut mapper = MapperSimulation::load(&path.join("calibration.json"), &path).unwrap();
    mapper.random_inc();
    mapper.random_yaw_inc();
    let matrix =
        firefly_basalt::utils::sophus_utils::se3_matrix(mapper.mapper.base.frame_poses[&0].pose());
    let expected: Value = serde_json::from_str(include_str!("reference/mapper_sim.json")).unwrap();
    for row in 0..4 {
        for column in 0..4 {
            assert!(
                (matrix[(row, column)] - expected[0]["random_pose"][row][column].as_f64().unwrap())
                    .abs()
                    < 1e-10
            );
        }
    }
}

#[test]
fn imu_producer_panic_sends_eos_before_join() {
    let path = fixture();
    let mut input = NaiveMapperSimulation::load(&path.join("calibration.json"), &path).unwrap();
    input.noisy_gyro.clear();
    assert!(input.run().is_err());
}
