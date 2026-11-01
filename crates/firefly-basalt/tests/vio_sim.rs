use firefly_basalt::{utils::common_types::TimeCamId, vio_sim::Simulation};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::camera::generic_camera::GenericCamera;
use nalgebra::{Isometry3, Vector2, Vector3, Vector4};
use serde_json::Value;

fn calibration() -> Calibration<f64> {
    let mut camera = GenericCamera::from_name("pinhole");
    camera.set_from_init(Vector4::new(320., 315., 320., 240.));
    Calibration {
        intrinsics: vec![camera.clone(), camera],
        t_i_c: vec![Isometry3::identity(), Isometry3::translation(0.125, 0., 0.)],
        resolution: vec![Vector2::new(640, 480); 2],
        imu_update_rate: 200.,
        gyro_noise_std: Vector3::new(0.001, 0.002, 0.003),
        accel_noise_std: Vector3::new(0.01, 0.02, 0.03),
        gyro_bias_std: Vector3::new(0.0001, 0.0002, 0.0003),
        accel_bias_std: Vector3::new(0.001, 0.002, 0.003),
        ..Calibration::default()
    }
}
fn check(actual: &[f64], expected: &Value, label: &str) {
    assert_eq!(actual.len(), expected.as_array().unwrap().len());
    for (i, &actual) in actual.iter().enumerate() {
        let expected = expected[i].as_f64().unwrap();
        assert!(
            (actual - expected).abs() <= 1e-10 * (1. + expected.abs()),
            "{label}[{i}]: {actual} != {expected}"
        );
    }
}
fn pose(actual: &Isometry3<f64>, expected: &Value) {
    check(
        actual.translation.vector.as_slice(),
        &expected["translation"],
        "position",
    );
    check(
        actual.rotation.quaternion().coords.as_slice(),
        &expected["quaternion"],
        "quaternion",
    );
}
#[test]
fn original_complete_generation_with_seeded_random_streams() {
    let reference: Value = serde_json::from_str(include_str!("reference/vio_sim.json")).unwrap();
    let simulation = Simulation::generate(calibration(), 32, 10.);
    assert_eq!(simulation.frame_timestamps.len(), 1000);
    assert_eq!(
        simulation.imu_timestamps.len() as u64,
        reference["imu_count"].as_u64().unwrap()
    );
    assert_eq!(
        simulation.spline.num_knots(),
        reference["knots"].as_array().unwrap().len()
    );
    for (i, knot) in reference["knots"].as_array().unwrap().iter().enumerate() {
        pose(&simulation.spline.knot(i), knot);
    }
    for frame in reference["frames"].as_array().unwrap() {
        let i = frame["index"].as_u64().unwrap() as usize;
        assert_eq!(
            simulation.frame_timestamps[i],
            frame["time"].as_i64().unwrap()
        );
        pose(&simulation.frame_poses[i], &frame["pose"]);
    }
    for sample in reference["imu"].as_array().unwrap() {
        let i = sample["index"].as_u64().unwrap() as usize;
        assert_eq!(
            simulation.imu_timestamps[i],
            sample["time"].as_i64().unwrap()
        );
        for (values, key) in [
            (&simulation.accel, "accel"),
            (&simulation.gyro, "gyro"),
            (&simulation.velocity, "velocity"),
            (&simulation.noisy_accel, "noisy_accel"),
            (&simulation.noisy_gyro, "noisy_gyro"),
            (&simulation.accel_bias, "accel_bias"),
            (&simulation.gyro_bias, "gyro_bias"),
        ] {
            check(values[i].as_slice(), &sample[key], key);
        }
        let input = simulation.imu_input(i);
        assert_eq!(input.t_ns, simulation.imu_timestamps[i]);
        assert_eq!(input.accel, simulation.noisy_accel[i]);
    }
    check(
        simulation.accel_bias.last().unwrap().as_slice(),
        &reference["final_accel_bias"],
        "last accel bias",
    );
    check(
        simulation.gyro_bias.last().unwrap().as_slice(),
        &reference["final_gyro_bias"],
        "last gyro bias",
    );
    for (i, point) in reference["points"].as_array().unwrap().iter().enumerate() {
        check(simulation.points[i].as_slice(), point, "landmark");
    }
    for observation in reference["observations"].as_array().unwrap() {
        let frame = observation["frame"].as_u64().unwrap() as usize;
        let camera = observation["camera"].as_u64().unwrap() as usize;
        let key = TimeCamId::new(simulation.frame_timestamps[frame], camera);
        let exact = &simulation.observations[&key];
        let noisy = &simulation.noisy_observations[&key];
        assert_eq!(serde_json::to_value(&exact.id).unwrap(), observation["ids"]);
        for i in 0..exact.pos.len() {
            check(exact.pos[i].as_slice(), &observation["exact"][i], "pixel");
            check(
                noisy.pos[i].as_slice(),
                &observation["noisy"][i],
                "noisy pixel",
            );
        }
        let vision = simulation.vision_input(frame);
        assert_eq!(vision.observations[camera].len(), noisy.id.len());
    }
    assert!(
        simulation
            .images
            .iter()
            .all(|image| image.data().iter().all(|&pixel| pixel == 200))
    );
    let mut archive = Vec::new();
    simulation.write_archive(&mut archive).unwrap();
    let archive = String::from_utf8(archive).unwrap();
    assert_eq!(archive.matches("\"noisy_accel\":").count(), 2);
    assert_eq!(archive.matches("\"gt_points\":").count(), 2);
}

#[test]
fn simulation_attaches_real_state_channel_before_worker_start() {
    use firefly_basalt::{
        utils::vio_config::VioConfig, vi_estimator::vio_estimator::VioWorkerOutputs,
    };
    use std::{sync::mpsc, time::Duration};
    let simulation = Simulation::generate(calibration(), 32, 10.);
    let (tx, rx) = mpsc::channel();
    let mut estimator = simulation
        .setup_vio_with_outputs(
            &VioConfig::default(),
            true,
            true,
            VioWorkerOutputs {
                state: Some(tx),
                ..VioWorkerOutputs::default()
            },
        )
        .unwrap();
    for input in simulation.imu_inputs().take(25) {
        estimator.imu_data_queue().send(Some(input)).unwrap();
    }
    estimator.imu_data_queue().send(None).unwrap();
    for input in simulation.vision_inputs().take(3) {
        estimator.vision_data_queue().send(Some(input)).unwrap();
    }
    estimator.vision_data_queue().send(None).unwrap();
    let mut timestamps = Vec::new();
    while let Some(state) = rx.recv_timeout(Duration::from_secs(10)).unwrap() {
        timestamps.push(state.state.pose.t_ns);
    }
    estimator.maybe_join().unwrap();
    assert_eq!(timestamps, simulation.frame_timestamps[..3]);
}
