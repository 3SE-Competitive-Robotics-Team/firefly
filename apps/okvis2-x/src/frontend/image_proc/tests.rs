use super::*;
use crate::frames::frame::KeyPoint;
use crate::{application::live::config::Config, frontend::pipeline::RustBriskFeatureFactory};
use crate::{
    ceres::vi_slam_backend::ViSlamBackend,
    frames::multi_frame::MultiFrame,
    frontend::{frame_absolute_pose_sac_problem::OpenGvSolver, pipeline::Frontend},
    kinematics::transformation::Transformation,
};
use purecv::core::{DynamicMatrix, Matrix, types::Point2f};
use std::{
    cell::Cell,
    sync::{
        Condvar, Mutex,
        atomic::{AtomicUsize, Ordering},
    },
    thread,
    time::Duration,
};

fn cameras() -> Vec<Arc<dyn CameraBase>> {
    let parameters = Config::default().parameters().unwrap();
    (0..2)
        .map(|c| parameters.n_camera_system.camera_geometry(c).clone())
        .collect()
}

fn input(camera: usize, geometry: &Arc<dyn CameraBase>, marker: u8) -> Input {
    let width = geometry.image_width() as usize;
    let height = geometry.image_height() as usize;
    let mut seed = 37u32 + camera as u32;
    let mut pixels: Vec<_> = (0..width * height)
        .map(|_| {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed as u8
        })
        .collect();
    pixels[0] = marker;
    Input {
        camera,
        geometry: geometry.clone(),
        image: Arc::new(DynamicMatrix::new_u8(height, width, 1, pixels).unwrap()),
        direction: Vector3::new(0.2, -0.8, -0.4),
    }
}

fn assert_features_equal(expected: &Frame, actual: &Frame) {
    assert_eq!(expected.num_keypoints(), actual.num_keypoints());
    for k in 0..expected.num_keypoints() {
        let mut a = KeyPoint::default();
        let mut b = KeyPoint::default();
        expected.get_cv_keypoint(k, &mut a);
        actual.get_cv_keypoint(k, &mut b);
        assert_eq!(a, b);
        assert_eq!(
            expected.keypoint_descriptor(k),
            actual.keypoint_descriptor(k)
        );
        let mut a = Vector3::zeros();
        let mut b = Vector3::zeros();
        assert_eq!(
            expected.get_back_projection(k, &mut a),
            actual.get_back_projection(k, &mut b)
        );
        for (a, b) in a.iter().zip(b.iter()) {
            assert_eq!(a.to_bits(), b.to_bits());
        }
        assert_eq!(actual.landmark_id(k), 0);
    }
}

#[test]
fn parallel_brisk_matches_serial_frame_path_across_direction_changes() {
    let parameters = Config::default().parameters().unwrap();
    let cameras: Vec<_> = (0..2)
        .map(|c| parameters.n_camera_system.camera_geometry(c).clone())
        .collect();
    let factory = Arc::new(RustBriskFeatureFactory);
    let settings = BriskSettings::default();
    let mut frontend = Frontend::<ViSlamBackend, OpenGvSolver, _>::with_vocabulary(
        parameters.n_camera_system.num_cameras(),
        &firefly_dbow2::templated_vocabulary::TemplatedVocabulary::default(),
        OpenGvSolver::default(),
        RustBriskFeatureFactory,
    );
    let resources: Vec<_> = cameras
        .iter()
        .map(|c| {
            factory
                .create(c.as_ref(), &settings, Vector3::zeros())
                .unwrap()
        })
        .collect();
    for round in 0..3 {
        let pose = Transformation::new(
            Vector3::zeros(),
            nalgebra::UnitQuaternion::from_euler_angles(0.3 * f64::from(round), 0.2, -0.1)
                .into_inner(),
        );
        let mut frame =
            MultiFrame::new(&parameters.n_camera_system, jiff::Timestamp::UNIX_EPOCH, 0);
        let mut expected = Vec::new();
        for camera in 0..2 {
            let mut input = input(camera, &cameras[camera], round);
            if round == 2 {
                input.image = Arc::new(
                    DynamicMatrix::new_u8(
                        cameras[camera].image_height() as usize,
                        cameras[camera].image_width() as usize,
                        1,
                        vec![
                            0;
                            (cameras[camera].image_height() * cameras[camera].image_width())
                                as usize
                        ],
                    )
                    .unwrap(),
                );
            }
            input.direction = pose
                .compose(frame.t_sc(camera))
                .inverse()
                .rotate_vector(&Vector3::new(0., 0., -1.))
                .cast::<f32>();
            frame.set_image(camera, input.image.clone());
            let (detector, extractor) = &resources[camera];
            extractor.set_extraction_direction(input.direction);
            let mut serial = Frame::new(
                input.image.clone(),
                input.geometry.clone(),
                detector.clone(),
                extractor.clone(),
                None,
            );
            serial.detect().unwrap();
            serial.describe().unwrap();
            serial.compute_back_projections();
            assert_eq!(serial.num_keypoints() == 0, round == 2);
            expected.push(serial);
        }
        let frame = std::rc::Rc::new(std::cell::RefCell::new(frame));
        frontend.detect_frame(&frame, &pose).unwrap();
        for (camera, expected) in expected.iter().enumerate() {
            assert_features_equal(expected, frame.borrow().frame(camera));
        }
    }
    frontend.end_image_workers().unwrap();
}

#[test]
fn failed_batch_leaves_all_graph_frame_features_untouched() {
    let parameters = Config::default().parameters().unwrap();
    let probe = Arc::new(Probe::default());
    let mut frontend = Frontend::<ViSlamBackend, OpenGvSolver, _>::with_vocabulary(
        parameters.n_camera_system.num_cameras(),
        &firefly_dbow2::templated_vocabulary::TemplatedVocabulary::default(),
        OpenGvSolver::default(),
        probe.clone(),
    );
    let mut frame = MultiFrame::new(&parameters.n_camera_system, jiff::Timestamp::UNIX_EPOCH, 0);
    for camera in 0..2 {
        frame.set_image(
            camera,
            input(
                camera,
                parameters.n_camera_system.camera_geometry(camera),
                if camera == 1 { 250 } else { 0 },
            )
            .image,
        );
        frame.reset_keypoints(
            camera,
            &[KeyPoint::new(Point2f::new(80., 90.), 10., -1., 1., 0, -1)],
        );
        frame.reset_descriptors(camera, Arc::new(Matrix::from_vec(1, 48, 1, vec![7; 48])));
        frame.set_landmark_id(camera, 0, 42);
    }
    let frame = std::rc::Rc::new(std::cell::RefCell::new(frame));
    assert!(
        frontend
            .detect_frame(&frame, &Transformation::identity())
            .is_err()
    );
    for camera in 0..2 {
        let frame = frame.borrow();
        assert_eq!(frame.num_keypoints(camera), 1);
        assert_eq!(frame.frame(camera).keypoint_descriptor(0), &[7; 48]);
        assert_eq!(frame.landmark_id(camera, 0), 42);
    }
    assert_eq!(probe.completed.load(Ordering::SeqCst), 1);
    assert_eq!(probe.dropped.load(Ordering::SeqCst), 2);
}

#[derive(Default)]
struct Probe {
    created: AtomicUsize,
    completed: AtomicUsize,
    dropped: AtomicUsize,
    rendezvous: Option<(Mutex<usize>, Condvar)>,
    fail_init: bool,
}

struct FeatureProbe {
    probe: Arc<Probe>,
    owner: thread::ThreadId,
    direction: std::rc::Rc<Cell<Vector3<f32>>>,
}

impl Drop for FeatureProbe {
    fn drop(&mut self) {
        assert_eq!(self.owner, thread::current().id());
        self.probe.dropped.fetch_add(1, Ordering::SeqCst);
    }
}

impl BriskFeatureFactory for Arc<Probe> {
    #[allow(
        clippy::arc_with_non_send_sync,
        reason = "exercise thread-local feature traits whose API returns Arc"
    )]
    fn create(
        &self,
        _: &dyn CameraBase,
        _: &BriskSettings,
        direction: Vector3<f32>,
    ) -> Result<(Arc<dyn FeatureDetector>, Arc<dyn DescriptorExtractor>)> {
        let index = self.created.fetch_add(1, Ordering::SeqCst);
        if self.fail_init && index == 1 {
            return Err(error("injected initialization failure"));
        }
        let resource = Arc::new(FeatureProbe {
            probe: self.clone(),
            owner: thread::current().id(),
            direction: std::rc::Rc::new(Cell::new(direction)),
        });
        Ok((resource.clone(), resource))
    }
}

impl FeatureDetector for FeatureProbe {
    fn detect(&self, image: &DynamicMatrix, points: &mut Vec<KeyPoint>) -> Result<()> {
        assert_eq!(self.owner, thread::current().id());
        if let Some((lock, ready)) = &self.probe.rendezvous {
            let mut count = lock.lock().unwrap();
            *count += 1;
            ready.notify_all();
            let (_count, timeout) = ready
                .wait_timeout_while(count, Duration::from_secs(5), |count| *count < 2)
                .unwrap();
            if timeout.timed_out() {
                return Err(error("cameras did not run concurrently"));
            }
        }
        match image.as_matrix_u8().unwrap().data[0] {
            250 => return Err(error("injected detection failure")),
            251 => panic!("injected worker panic"),
            _ => {}
        }
        points.push(KeyPoint::new(Point2f::new(40., 60.), 12., -1., 1., 0, -1));
        Ok(())
    }
}

impl DescriptorExtractor for FeatureProbe {
    fn set_extraction_direction(&self, direction: Vector3<f32>) {
        self.direction.set(direction);
    }
    fn compute(
        &self,
        _: &DynamicMatrix,
        _: &mut Vec<KeyPoint>,
        descriptors: &mut Matrix<u8>,
    ) -> Result<()> {
        assert_eq!(self.owner, thread::current().id());
        *descriptors = Matrix::from_vec(1, 48, 1, vec![self.direction.get().x.to_bits() as u8; 48]);
        self.probe.completed.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
}

#[test]
fn fixed_workers_overlap_and_drop_resources_on_their_own_threads() {
    let cameras = cameras();
    let probe = Arc::new(Probe {
        rendezvous: Some((Mutex::new(0), Condvar::new())),
        ..Probe::default()
    });
    let factory = Arc::new(probe.clone());
    let mut pipeline = Pipeline::new(3);
    for _ in 0..2 {
        let outputs = pipeline
            .process(
                vec![input(1, &cameras[1], 0), input(0, &cameras[0], 0)],
                &factory,
                &BriskSettings::default(),
            )
            .unwrap();
        assert_eq!(
            outputs.iter().map(|o| o.camera).collect::<Vec<_>>(),
            vec![0, 1]
        );
    }
    assert_eq!(probe.created.load(Ordering::SeqCst), 2);
    assert_eq!(probe.completed.load(Ordering::SeqCst), 4);
    let outputs = pipeline
        .process(
            vec![input(1, &cameras[1], 0)],
            &factory,
            &BriskSettings::default(),
        )
        .unwrap();
    assert_eq!(outputs.len(), 1);
    assert_eq!(outputs[0].camera, 1);
    assert_eq!(probe.created.load(Ordering::SeqCst), 2);
    assert_eq!(probe.completed.load(Ordering::SeqCst), 5);
    drop(pipeline);
    assert_eq!(probe.dropped.load(Ordering::SeqCst), 2);
}

#[test]
fn initialization_panic_is_reported_and_calibration_change_closes_pipeline() {
    assert!(worker::Worker::spawn(0, || panic!("injected initialization panic")).is_err());
    let cameras = cameras();
    let probe = Arc::new(Probe::default());
    let factory = Arc::new(probe.clone());
    let mut pipeline = Pipeline::new(1);
    pipeline
        .process(
            vec![input(0, &cameras[0], 0)],
            &factory,
            &BriskSettings::default(),
        )
        .unwrap();
    assert!(
        pipeline
            .process(
                vec![input(0, &cameras[1], 0)],
                &factory,
                &BriskSettings::default()
            )
            .is_err()
    );
    assert_eq!(probe.completed.load(Ordering::SeqCst), 1);
    assert_eq!(probe.dropped.load(Ordering::SeqCst), 1);
}

#[test]
fn failures_drain_other_camera_and_close_pipeline() {
    let cameras = cameras();
    for marker in [250, 251] {
        let probe = Arc::new(Probe::default());
        let factory = Arc::new(probe.clone());
        let mut pipeline = Pipeline::new(2);
        assert!(
            pipeline
                .process(
                    vec![input(0, &cameras[0], marker), input(1, &cameras[1], 0)],
                    &factory,
                    &BriskSettings::default()
                )
                .is_err()
        );
        assert_eq!(probe.completed.load(Ordering::SeqCst), 1);
        assert_eq!(probe.dropped.load(Ordering::SeqCst), 2);
        assert!(
            pipeline
                .process(vec![], &factory, &BriskSettings::default())
                .is_err()
        );
        pipeline.shutdown().unwrap();
    }
}

#[test]
fn initialization_failure_joins_workers_and_invalid_batches_submit_nothing() {
    let cameras = cameras();
    let probe = Arc::new(Probe {
        fail_init: true,
        ..Probe::default()
    });
    let factory = Arc::new(probe.clone());
    let mut pipeline = Pipeline::new(2);
    for ids in [[0, 0], [0, 2]] {
        assert!(
            pipeline
                .process(
                    ids.into_iter()
                        .map(|id| input(id, &cameras[0], 0))
                        .collect(),
                    &factory,
                    &BriskSettings::default()
                )
                .is_err()
        );
        assert_eq!(probe.created.load(Ordering::SeqCst), 0);
    }
    assert!(
        pipeline
            .process(vec![], &factory, &BriskSettings::default())
            .unwrap()
            .is_empty()
    );
    assert!(
        pipeline
            .process(
                vec![input(0, &cameras[0], 0), input(1, &cameras[1], 0)],
                &factory,
                &BriskSettings::default()
            )
            .is_err()
    );
    assert_eq!(probe.completed.load(Ordering::SeqCst), 1);
    assert_eq!(probe.dropped.load(Ordering::SeqCst), 1);
}
