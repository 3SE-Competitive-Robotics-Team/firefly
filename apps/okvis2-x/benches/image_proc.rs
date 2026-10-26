//! Synthetic stereo feature-stage benchmark; raw samples go to the viz recorder.
use firefly_okvis_x::{
    cameras::{
        camera_base::CameraBase,
        eucm_camera::EucmCamera,
        n_camera_system::{CameraType, DistortionType, NCameraSystem},
    },
    ceres::{component::MultiFramePtr, vi_slam_backend::ViSlamBackend},
    frames::{
        frame::{DescriptorExtractor, FeatureDetector, KeyPoint},
        multi_frame::MultiFrame,
    },
    frontend::{
        frame_absolute_pose_sac_problem::OpenGvSolver,
        pipeline::{BriskFeatureFactory, BriskSettings, Frontend, RustBriskFeatureFactory},
    },
    kinematics::transformation::Transformation,
};
use nalgebra::{UnitQuaternion, Vector3};
use purecv::core::DynamicMatrix;
use std::{
    cell::RefCell,
    hint::black_box,
    rc::Rc,
    sync::Arc,
    time::{Duration, Instant},
};

mod common;
type Pair = (Arc<dyn FeatureDetector>, Arc<dyn DescriptorExtractor>);
type Parallel = Frontend<ViSlamBackend, OpenGvSolver, RustBriskFeatureFactory>;

struct Fixture {
    rig: NCameraSystem,
    images: Vec<Arc<DynamicMatrix>>,
}
impl Fixture {
    fn new(width: usize, height: usize, textured: bool) -> Self {
        let mut rig = NCameraSystem::new();
        let mut images = Vec::new();
        for camera in 0..2 {
            let mut geometry = EucmCamera::new(
                (width as i32, height as i32),
                (
                    width as f64 * 0.7,
                    width as f64 * 0.7,
                    (width as f64 - 1.) / 2.,
                    (height as f64 - 1.) / 2.,
                    0.,
                    0.,
                ),
                camera,
            );
            geometry.initialise_camera_awareness_maps();
            rig.add_camera(
                &Arc::new(Transformation::identity()),
                Arc::new(geometry),
                DistortionType::NoDistortion,
                true,
                CameraType::default(),
            );
            let mut seed = 37u32 + camera as u32;
            let pixels = (0..width * height)
                .map(|_| {
                    seed ^= seed << 13;
                    seed ^= seed >> 17;
                    seed ^= seed << 5;
                    if textured { seed as u8 } else { 0 }
                })
                .collect();
            images.push(Arc::new(
                DynamicMatrix::new_u8(height, width, 1, pixels).unwrap(),
            ));
        }
        Self { rig, images }
    }
    fn frame(&self) -> MultiFramePtr {
        let mut frame = MultiFrame::new(&self.rig, jiff::Timestamp::UNIX_EPOCH, 0);
        for c in 0..2 {
            frame.set_image(c, self.images[c].clone());
        }
        Rc::new(RefCell::new(frame))
    }
}

fn serial(fixture: &Fixture, resources: &mut Vec<Pair>, pose: &Transformation) -> MultiFramePtr {
    let frame = fixture.frame();
    for c in 0..2 {
        let direction = pose
            .compose(frame.borrow().t_sc(c))
            .inverse()
            .rotate_vector(&Vector3::new(0., 0., -1.))
            .cast::<f32>();
        if resources.len() <= c {
            resources.push(
                RustBriskFeatureFactory
                    .create(
                        frame.borrow().geometry(c).as_ref(),
                        &BriskSettings::default(),
                        direction,
                    )
                    .unwrap(),
            );
        }
        let (detector, extractor) = &resources[c];
        extractor.set_extraction_direction(direction);
        let mut frame = frame.borrow_mut();
        frame.set_detector(c, detector.clone());
        frame.set_extractor(c, extractor.clone());
        frame.detect(c).unwrap();
        frame.describe(c).unwrap();
        frame.compute_back_projections(c);
    }
    frame
}

fn parallel(fixture: &Fixture, frontend: &mut Parallel, pose: &Transformation) -> MultiFramePtr {
    let frame = fixture.frame();
    frontend.detect_frame(&frame, pose).unwrap();
    frame
}

fn check(a: &MultiFramePtr, b: &MultiFramePtr) {
    let a = a.borrow();
    let b = b.borrow();
    for c in 0..2 {
        assert_eq!(a.num_keypoints(c), b.num_keypoints(c));
        for k in 0..a.num_keypoints(c) {
            let mut ka = KeyPoint::default();
            let mut kb = KeyPoint::default();
            a.frame(c).get_cv_keypoint(k, &mut ka);
            b.frame(c).get_cv_keypoint(k, &mut kb);
            assert_eq!(ka, kb);
            assert_eq!(
                a.frame(c).keypoint_descriptor(k),
                b.frame(c).keypoint_descriptor(k)
            );
            let mut ra = Vector3::zeros();
            let mut rb = Vector3::zeros();
            assert_eq!(
                a.get_back_projection(c, k, &mut ra),
                b.get_back_projection(c, k, &mut rb)
            );
            for (a, b) in ra.iter().zip(rb.iter()) {
                assert_eq!(a.to_bits(), b.to_bits());
            }
        }
    }
}

fn host_identity() {
    if let Ok(cpuinfo) = std::fs::read_to_string("/proc/cpuinfo")
        && let Some(model) = cpuinfo.lines().find(|line| line.starts_with("model name"))
    {
        log::info!("host {model}; architecture={}", std::env::consts::ARCH);
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    if !std::env::args().any(|arg| arg == "--bench") {
        return Ok(());
    }
    firefly_observability::init();
    let node = firefly_pubsub::node::create_node()?;
    let ipc = firefly_observability::init_ipc(&node, "image-proc-bench");
    host_identity();
    common::host_snapshot("idle_start");
    firefly_observability::pump_log_ipc(&ipc);
    std::thread::sleep(Duration::from_secs(2));
    common::host_snapshot("idle_end");
    log::info!(
        "image_proc benchmark release={} synthetic stereo; 61 alternating paired individual frames; allocation/dispatch/install included; calibration maps and input generation excluded",
        !cfg!(debug_assertions)
    );
    for (width, height, textured) in [(320, 240, true), (320, 240, false), (640, 480, true)] {
        let fixture = Fixture::new(width, height, textured);
        for round in 0..3 {
            let mut resources = Vec::new();
            let mut frontend = Parallel::with_vocabulary(
                2,
                &firefly_dbow2::templated_vocabulary::TemplatedVocabulary::default(),
                OpenGvSolver::default(),
                RustBriskFeatureFactory,
            );
            let pose = Transformation::new(
                Vector3::zeros(),
                UnitQuaternion::from_euler_angles(0.1 * f64::from(round), 0.2, -0.1).into_inner(),
            );
            let mut cold = [None, None];
            for variant in if round % 2 == 0 { [0, 1] } else { [1, 0] } {
                let start = Instant::now();
                let frame = if variant == 0 {
                    serial(&fixture, &mut resources, &pose)
                } else {
                    parallel(&fixture, &mut frontend, &pose)
                };
                let seconds = start.elapsed().as_secs_f64();
                log::info!(
                    "cold width={width} height={height} textured={textured} round={round} variant={variant} seconds={seconds:.9}"
                );
                cold[variant] = Some(frame);
            }
            check(cold[0].as_ref().unwrap(), cold[1].as_ref().unwrap());
            let points = cold[0].as_ref().unwrap().borrow().total_keypoints();
            drop(cold);
            let warmup = Instant::now();
            while warmup.elapsed() < Duration::from_millis(500) {
                black_box(serial(&fixture, &mut resources, &pose));
                black_box(parallel(&fixture, &mut frontend, &pose));
            }
            common::host_snapshot("case_start");
            firefly_observability::pump_log_ipc(&ipc);
            let root = fastrace::Span::root(
                "image_proc_benchmark",
                fastrace::prelude::SpanContext::random(),
            );
            let parent = root.set_local_parent();
            let mut times = [Vec::new(), Vec::new()];
            for sample in 0..61 {
                let mut frames = [None, None];
                for variant in if sample % 2 == 0 { [0, 1] } else { [1, 0] } {
                    let start = Instant::now();
                    let frame = if variant == 0 {
                        serial(&fixture, &mut resources, &pose)
                    } else {
                        parallel(&fixture, &mut frontend, &pose)
                    };
                    let seconds = start.elapsed().as_secs_f64();
                    times[variant].push(seconds);
                    frames[variant] = Some(black_box(frame));
                    log::info!(
                        "sample width={width} height={height} textured={textured} round={round} sample={sample} variant={variant} seconds={seconds:.9}"
                    );
                }
                check(frames[0].as_ref().unwrap(), frames[1].as_ref().unwrap());
                firefly_observability::pump_log_ipc(&ipc);
            }
            for values in &mut times {
                values.sort_by(f64::total_cmp);
            }
            log::info!(
                "summary width={width} height={height} textured={textured} round={round} points={points} serial_ms={:.6} parallel_ms={:.6} speedup={:.4} serial_p95_ms={:.6} parallel_p95_ms={:.6} exact_match=true",
                times[0][30] * 1000.,
                times[1][30] * 1000.,
                times[0][30] / times[1][30],
                times[0][57] * 1000.,
                times[1][57] * 1000.
            );
            common::host_snapshot("case_end");
            firefly_observability::pump_log_ipc(&ipc);
            drop(parent);
            drop(root);
            firefly_observability::flush();
        }
    }
    firefly_observability::flush();
    firefly_observability::pump_log_ipc(&ipc);
    Ok(())
}
