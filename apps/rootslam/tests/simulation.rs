use firefly_basalt::{utils::vio_config::VioConfig, vio_sim::Simulation};
use firefly_basalt_headers::{calibration::Calibration, camera::generic_camera::GenericCamera};
use nalgebra::{Isometry3, Vector2, Vector3, Vector4};

fn simulation() -> Simulation {
    let mut camera = GenericCamera::from_name("pinhole");
    camera.set_from_init(Vector4::new(320., 315., 320., 240.));
    let calibration = Calibration {
        intrinsics: vec![camera.clone(), camera],
        t_i_c: vec![Isometry3::identity(), Isometry3::translation(0.125, 0., 0.)],
        resolution: vec![Vector2::new(640, 480); 2],
        imu_update_rate: 200.,
        gyro_noise_std: Vector3::repeat(0.001),
        accel_noise_std: Vector3::repeat(0.01),
        gyro_bias_std: Vector3::repeat(0.0001),
        accel_bias_std: Vector3::repeat(0.001),
        ..Calibration::default()
    };
    let mut simulation = Simulation::generate(calibration, 256, 10.);
    simulation.frame_timestamps.truncate(8);
    simulation.imu_timestamps.truncate(100);
    simulation
}

#[test]
fn simulated_vision_and_imu_complete_the_application_queues() {
    let simulation = simulation();
    for use_double in [false, true] {
        let states =
            firefly_rootslam::simulation::run(&simulation, &VioConfig::default(), true, use_double)
                .unwrap();
        assert_eq!(states.len(), simulation.frame_timestamps.len());
        for (state, time) in states.iter().zip(&simulation.frame_timestamps) {
            assert_eq!(state.state.pose.t_ns, *time);
            assert!(
                state
                    .state
                    .pose
                    .t_w_i
                    .translation
                    .vector
                    .iter()
                    .all(|value| value.is_finite())
            );
        }
    }
}

#[test]
fn imu_producer_panic_closes_the_estimator_instead_of_hanging() {
    let mut simulation = simulation();
    simulation.noisy_gyro.truncate(5);
    let (sender, receiver) = std::sync::mpsc::channel();
    let worker = std::thread::spawn(move || {
        let result =
            firefly_rootslam::simulation::run(&simulation, &VioConfig::default(), true, true);
        sender.send(result.is_err()).unwrap();
    });
    assert!(
        receiver
            .recv_timeout(std::time::Duration::from_secs(10))
            .expect("Simulation failed to drain after IMU producer panic")
    );
    worker.join().unwrap();
}
