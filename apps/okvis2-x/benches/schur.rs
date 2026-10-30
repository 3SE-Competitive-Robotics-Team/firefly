//! Run with a firefly-viz recorder and `RUST_LOG=info` to retain samples in RRD.
//! Timings include layout, allocation, assembly, elimination, solve and back substitution.
use std::{
    hint::black_box,
    time::{Duration, Instant},
};

#[path = "../src/ceres/vi_graph/schur.rs"]
mod schur;
use nalgebra::{DMatrix, DVector};
use schur::{Row, SchurSystem};
#[path = "../src/ceres/vi_graph/schur/fixtures.rs"]
mod fixtures;

fn batch(system: &SchurSystem<'_>, diagonal: &DVector<f64>, block: bool, iterations: usize) -> f64 {
    let name = if block { "schur_block" } else { "schur_dense" };
    let _span = fastrace::local::LocalSpan::enter_with_local_parent(name);
    let start = Instant::now();
    for _ in 0..iterations {
        let result = if block {
            black_box(system).block(black_box(diagonal))
        } else {
            black_box(system).dense(black_box(diagonal))
        };
        black_box(result.expect("positive definite benchmark system"));
    }
    start.elapsed().as_secs_f64() / iterations as f64
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Cargo supplies --bench for benchmarks and no mode flag for all-target tests.
    // Keep the default path independent of IPC, timing and host sensors.
    if !std::env::args().any(|arg| arg == "--bench") {
        for (frames, points, views) in [(2, 3, 1), (2, 3, 2), (2, 0, 0)] {
            check_solution(&fixtures::Fixture::window(frames, points, views));
        }
        check_retries(&fixtures::Fixture::window(2, 3, 2), 4);
        return Ok(());
    }
    firefly_observability::init();
    let node = firefly_pubsub::node::create_node()?;
    let ipc = firefly_observability::init_ipc(&node, "schur-bench");
    host_snapshot("idle_start");
    firefly_observability::pump_log_ipc(&ipc);
    std::thread::sleep(Duration::from_secs(1));
    host_snapshot("idle_end");
    firefly_observability::pump_log_ipc(&ipc);
    if std::env::args().any(|arg| arg == "--retries") {
        for points in [150, 500] {
            let fixture = fixtures::Fixture::window(8, points, 3);
            for attempts in [1, 2, 4] {
                retry_case(&fixture, attempts, || {
                    firefly_observability::pump_log_ipc(&ipc);
                });
            }
        }
        firefly_observability::flush();
        firefly_observability::pump_log_ipc(&ipc);
        return Ok(());
    }
    log::info!(
        "Schur benchmark: synthetic inputs; 31 alternating paired batches; release={}; layout and allocation included",
        !cfg!(debug_assertions)
    );
    firefly_observability::pump_log_ipc(&ipc);
    for (frames, points, views) in [
        (4, 40, 2),
        (8, 150, 3),
        (12, 500, 3),
        (8, 150, 8),
        (8, 0, 0),
    ] {
        let fixture = fixtures::Fixture::window(frames, points, views);
        let system = fixture.system();
        let difference = check_solution(&fixture);
        let warmup = Instant::now();
        while warmup.elapsed() < Duration::from_millis(300) {
            batch(&system, &fixture.regularization, false, 1);
            batch(&system, &fixture.regularization, true, 1);
        }
        let calibration = batch(&system, &fixture.regularization, false, 3);
        let iterations = (0.03 / calibration).ceil().clamp(1.0, 10000.0) as usize;
        host_snapshot("case_start");
        firefly_observability::pump_log_ipc(&ipc);
        let root =
            fastrace::Span::root("schur_benchmark", fastrace::prelude::SpanContext::random());
        let parent = root.set_local_parent();
        log::info!(
            "case frames={frames} points={points} views={views} columns={} iterations={iterations} normalized_difference={difference:.3e}",
            fixture.gradient.len()
        );
        let mut samples = [Vec::new(), Vec::new()];
        for sample in 0..31 {
            for variant in if sample % 2 == 0 { [0, 1] } else { [1, 0] } {
                let seconds = batch(&system, &fixture.regularization, variant == 1, iterations);
                samples[variant].push(seconds);
                log::info!(
                    "sample frames={frames} points={points} views={views} batch={sample} variant={} seconds_per_solve={seconds:.9} iterations={iterations}",
                    if variant == 0 { "dense" } else { "block" }
                );
                firefly_observability::pump_log_ipc(&ipc);
            }
        }
        for values in &mut samples {
            values.sort_by(f64::total_cmp);
        }
        log::info!(
            "summary frames={frames} points={points} views={views} dense_median_us={:.3} block_median_us={:.3} speedup={:.3} dense_batch_p95_us={:.3} block_batch_p95_us={:.3}",
            samples[0][15] * 1e6,
            samples[1][15] * 1e6,
            samples[0][15] / samples[1][15],
            samples[0][29] * 1e6,
            samples[1][29] * 1e6
        );
        firefly_observability::pump_log_ipc(&ipc);
        host_snapshot("case_end");
        firefly_observability::pump_log_ipc(&ipc);
        drop(parent);
        drop(root);
        firefly_observability::flush();
    }
    firefly_observability::flush();
    firefly_observability::pump_log_ipc(&ipc);
    Ok(())
}

fn retry_case(fixture: &fixtures::Fixture, attempts: usize, mut pump: impl FnMut()) {
    let points = fixture.points.len();
    check_retries(fixture, attempts);
    let warmup = Instant::now();
    while warmup.elapsed() < Duration::from_millis(300) {
        retry_batch(fixture, attempts, false);
        retry_batch(fixture, attempts, true);
    }
    host_snapshot("case_start");
    pump();
    let mut samples = [Vec::new(), Vec::new()];
    for sample in 0..31 {
        for variant in if sample % 2 == 0 { [0, 1] } else { [1, 0] } {
            let seconds = retry_batch(fixture, attempts, variant == 1);
            samples[variant].push(seconds);
            log::info!(
                "retry_sample points={points} attempts={attempts} sample={sample} cached={} seconds={seconds:.9}",
                variant == 1
            );
        }
        pump();
    }
    for values in &mut samples {
        values.sort_by(f64::total_cmp);
    }
    log::info!(
        "retry_summary points={points} attempts={attempts} repeated_median_us={:.3} cached_median_us={:.3} repeated_p95_us={:.3} cached_p95_us={:.3} speedup={:.3}",
        samples[0][15] * 1e6,
        samples[1][15] * 1e6,
        samples[0][29] * 1e6,
        samples[1][29] * 1e6,
        samples[0][15] / samples[1][15]
    );
    host_snapshot("case_end");
    pump();
}

fn retry_batch(fixture: &fixtures::Fixture, attempts: usize, cached: bool) -> f64 {
    let damping: Vec<_> = (0..attempts)
        .map(|i| &fixture.regularization * 2_f64.powi(i as i32))
        .collect();
    let system = fixture.system();
    let start = Instant::now();
    let normal = cached.then(|| black_box(&system).assemble());
    for damping in &damping {
        let result = match &normal {
            Some(normal) => normal.solve(black_box(damping), false),
            None => black_box(&system).block(black_box(damping)),
        };
        black_box(result.expect("positive definite retry system"));
    }
    drop(normal);
    start.elapsed().as_secs_f64()
}

fn check_retries(fixture: &fixtures::Fixture, attempts: usize) {
    let system = fixture.system();
    let normal = system.assemble();
    for i in 0..attempts {
        let damping = &fixture.regularization * 2_f64.powi(i as i32);
        let expected = system.block(&damping).unwrap();
        assert_eq!(normal.solve(&damping, false).unwrap(), expected);
    }
}

fn check_solution(fixture: &fixtures::Fixture) -> f64 {
    let system = fixture.system();
    let dense = system.dense(&fixture.regularization).unwrap();
    let block = system.block(&fixture.regularization).unwrap();
    let difference = (&dense - &block).amax() / (1.0 + dense.amax());
    assert!(difference < 1e-9, "solution mismatch: {difference}");
    difference
}

mod common;
use common::host_snapshot;
