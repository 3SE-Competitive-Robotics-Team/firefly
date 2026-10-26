//! Production graph solves on repeatable stereo geometry; record samples through firefly-viz.
use firefly_okvis_x::{
    cameras::{no_distortion::NoDistortion, pinhole_camera::PinholeCamera},
    ceres::{
        homogeneous_point_parameter_block::HomogeneousPointParameterBlock,
        parameter_block::ParameterBlock,
        pose_error::PoseError,
        pose_parameter_block::PoseParameterBlock,
        reprojection_error::ReprojectionError,
        vi_graph::{
            GraphFactor, GraphProblem, GraphSolver, ManifoldKind, Parameter, RobustLoss,
            RustGraphSolver, SolverOptions,
        },
    },
    kinematics::transformation::Transformation,
};
use jiff::Timestamp;
use nalgebra::{Isometry3, Matrix2, Quaternion, SMatrix, Vector2, Vector3, Vector4};
use std::{
    cell::RefCell,
    rc::Rc,
    sync::Arc,
    time::{Duration, Instant},
};
mod common;

struct PhaseReporter;
impl fastrace::collector::Reporter for PhaseReporter {
    fn report(&mut self, spans: Vec<fastrace::collector::SpanRecord>) {
        for span in spans {
            log::info!(
                "phase trace={:032x} name={} milliseconds={:.6}",
                span.trace_id.0,
                span.name,
                span.duration_ns as f64 / 1e6
            );
        }
    }
}

struct Fixture {
    graph: GraphProblem,
    initial: Vec<(Parameter, Vec<f64>)>,
}
impl Fixture {
    fn new(points: usize) -> Self {
        let mut graph = GraphProblem::default();
        let camera = Arc::new(PinholeCamera::new(
            (640, 480),
            (320., 320., 320., 240.),
            NoDistortion,
            0,
        ));
        let poses: Vec<_> = (0..8)
            .map(|i| {
                let truth = Vector3::new(i as f64 * 0.05, 0., 0.);
                let pose = Parameter::Pose(Rc::new(RefCell::new(PoseParameterBlock::new(
                    &Transformation::new(
                        truth + Vector3::new(0.01, -0.01, 0.005),
                        Quaternion::identity(),
                    ),
                    i,
                    Timestamp::UNIX_EPOCH,
                ))));
                graph.register(pose.clone(), ManifoldKind::Pose);
                graph.add(
                    GraphFactor::PosePrior(Box::new(PoseError::new(
                        Isometry3::translation(truth.x, truth.y, truth.z),
                        SMatrix::identity() * 100.,
                    ))),
                    vec![pose.clone()],
                    None,
                );
                pose
            })
            .collect();
        let extrinsics: Vec<_> = (0..2)
            .map(|c| {
                let p = Parameter::Pose(Rc::new(RefCell::new(PoseParameterBlock::new(
                    &Transformation::new(
                        Vector3::new(c as f64 * 0.1, 0., 0.),
                        Quaternion::identity(),
                    ),
                    8 + c,
                    Timestamp::UNIX_EPOCH,
                ))));
                p.set_fixed(true);
                graph.register(p.clone(), ManifoldKind::Pose);
                p
            })
            .collect();
        for i in 0..points {
            let truth = Vector4::new(
                (i % 31) as f64 * 0.05 - 0.7,
                (i % 17) as f64 * 0.04 - 0.3,
                3. + (i % 7) as f64 * 0.1,
                1.,
            );
            let point =
                Parameter::Point(Rc::new(RefCell::new(HomogeneousPointParameterBlock::new(
                    truth + Vector4::new(0.01, -0.02, 0.03, 0.),
                    10 + i as u64,
                    true,
                ))));
            graph.register(point.clone(), ManifoldKind::HomogeneousPoint);
            for view in 0..3 {
                let frame = (i + view) % poses.len();
                for (c, extrinsic) in extrinsics.iter().enumerate() {
                    let measurement = Vector2::new(
                        320. * (truth.x - frame as f64 * 0.05 - c as f64 * 0.1) / truth.z + 320.,
                        320. * truth.y / truth.z + 240.,
                    );
                    graph.add(
                        GraphFactor::Reprojection(Rc::new(RefCell::new(Box::new(
                            ReprojectionError::new(
                                camera.clone(),
                                c as u64,
                                measurement,
                                Matrix2::identity(),
                            ),
                        )))),
                        vec![poses[frame].clone(), point.clone(), extrinsic.clone()],
                        Some(RobustLoss::Huber(1.0)),
                    );
                }
            }
        }
        let initial = graph
            .parameters
            .values()
            .map(|(p, _)| (p.clone(), p.values().unwrap()))
            .collect();
        Self { graph, initial }
    }
    fn reset(&self) {
        for (p, values) in &self.initial {
            match p {
                Parameter::Pose(p) => p.borrow_mut().set_parameters(values),
                Parameter::Point(p) => p.borrow_mut().set_parameters(values),
                Parameter::SpeedBias(p) => p.borrow_mut().set_parameters(values),
            }
        }
    }
    fn values(&self) -> Vec<f64> {
        self.initial
            .iter()
            .flat_map(|(p, _)| p.values().unwrap())
            .collect()
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let timing = std::env::args().any(|arg| arg == "--bench");
    let ipc = if timing {
        firefly_observability::init();
        fastrace::set_reporter(PhaseReporter, fastrace::collector::Config::default());
        let node = firefly_pubsub::node::create_node()?;
        Some((firefly_observability::init_ipc(&node, "solver-bench"), node))
    } else {
        None
    };
    for points in if timing {
        vec![150, 500, 1500]
    } else {
        vec![5]
    } {
        let mut fixture = Fixture::new(points);
        let mut solvers: Vec<_> = (0..3).map(|_| RustGraphSolver::default()).collect();
        for iterations in [0, 3] {
            let mut reference = None;
            let mut samples = [Vec::new(), Vec::new(), Vec::new()];
            let rounds = if timing { 31 } else { 1 };
            if let Some((ipc, _)) = &ipc {
                common::host_snapshot("case_start");
                firefly_observability::pump_log_ipc(ipc);
            }
            // Pool construction and first buffer allocation are excluded from steady-state samples.
            let warmup = Instant::now();
            loop {
                for (variant, threads) in [1, 2, 4].into_iter().enumerate() {
                    fixture.reset();
                    solvers[variant].solve(
                        &mut fixture.graph,
                        &SolverOptions {
                            num_threads: threads,
                            max_iterations: iterations,
                            linear_solver: "DENSE_SCHUR",
                            function_tolerance: 0.0,
                            ..Default::default()
                        },
                    )?;
                }
                if !timing || warmup.elapsed() >= Duration::from_millis(300) {
                    break;
                }
            }
            for sample in 0..rounds {
                for index in 0..3 {
                    let variant = (sample + index) % 3;
                    let threads = [1, 2, 4][variant];
                    fixture.reset();
                    let start = Instant::now();
                    let summary = solvers[variant].solve(
                        &mut fixture.graph,
                        &SolverOptions {
                            num_threads: threads,
                            max_iterations: iterations,
                            linear_solver: "DENSE_SCHUR",
                            function_tolerance: 0.0,
                            ..Default::default()
                        },
                    )?;
                    let seconds = start.elapsed().as_secs_f64();
                    let values = fixture.values();
                    let result = (values, summary.final_cost, summary.iterations);
                    if let Some(expected) = &reference {
                        assert_eq!(&result, expected);
                    } else {
                        reference = Some(result);
                    }
                    samples[variant].push(seconds);
                    if timing {
                        log::info!(
                            "sample points={points} factors={} iterations={iterations} threads={threads} sample={sample} seconds={seconds:.9} exact_match=true",
                            fixture.graph.residuals.len()
                        );
                    }
                }
                if let Some((ipc, _)) = &ipc {
                    firefly_observability::pump_log_ipc(ipc);
                }
            }
            if let Some((ipc, _)) = &ipc {
                for (variant, mut times) in samples.into_iter().enumerate() {
                    times.sort_by(f64::total_cmp);
                    log::info!(
                        "summary points={points} iterations={iterations} threads={} median_ms={:.6} p95_ms={:.6}",
                        [1, 2, 4][variant],
                        times[15] * 1000.,
                        times[29] * 1000.
                    );
                }
                common::host_snapshot("case_end");
                firefly_observability::pump_log_ipc(ipc);
                // Instrumented phase probes are separate from steady-state timing samples.
                if iterations > 0 {
                    probe_phases(&mut fixture, &mut solvers, points, iterations)?;
                    firefly_observability::pump_log_ipc(ipc);
                }
            }
        }
    }
    Ok(())
}

fn probe_phases(
    fixture: &mut Fixture,
    solvers: &mut [RustGraphSolver],
    points: usize,
    iterations: usize,
) -> Result<(), Box<dyn std::error::Error>> {
    for (variant, threads) in [1, 2, 4].into_iter().enumerate() {
        fixture.reset();
        let context = fastrace::prelude::SpanContext::random();
        log::info!(
            "diagnostic points={points} threads={threads} trace={:032x}",
            context.trace_id.0
        );
        let root = fastrace::Span::root("solver-diagnostic", context);
        let parent = root.set_local_parent();
        solvers[variant].solve(
            &mut fixture.graph,
            &SolverOptions {
                num_threads: threads,
                max_iterations: iterations,
                linear_solver: "DENSE_SCHUR",
                function_tolerance: 0.0,
                ..Default::default()
            },
        )?;
        drop(parent);
        drop(root);
        firefly_observability::flush();
    }
    Ok(())
}
