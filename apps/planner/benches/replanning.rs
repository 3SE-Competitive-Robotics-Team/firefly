//! Real solver versus a bounded worker under 100 Hz execution / 20 Hz safety scheduling.
#[path = "../../okvis2-x/benches/common/mod.rs"]
mod common;
use firefly_map::{GridMapBuilder, VoxelState};
use firefly_planner::worker::{MapVersion, PlanningWorker};
use firefly_planner::{ManagerOptions, Planner, PlannerConfig, PlannerManager, ReplanFsm, State};
use nalgebra::{Point3, Vector3};
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    if !std::env::args().any(|arg| arg == "--bench") {
        return Ok(());
    }
    let node = firefly_pubsub::node::create_node()?;
    let updates = std::env::args().any(|arg| arg == "--updates");
    firefly_observability::init();
    let ipc = firefly_observability::init_ipc(&node, "replanning-bench");
    common::host_snapshot("before");
    firefly_observability::pump_log_ipc(&ipc);
    for round in 0..2 {
        for obstacle in [false, true] {
            for deferred in if round == 0 {
                [false, true]
            } else {
                [true, false]
            } {
                run(deferred, obstacle, round, updates)?;
                firefly_observability::pump_log_ipc(&ipc);
            }
        }
    }
    common::host_snapshot("after");
    firefly_observability::pump_log_ipc(&ipc);
    firefly_observability::flush();
    Ok(())
}

#[allow(clippy::too_many_lines)]
fn run(
    deferred: bool,
    obstacle: bool,
    round: usize,
    updates: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut map = GridMapBuilder::new(0.1, [160, 160, 80]).build()?;
    if obstacle {
        for x in 39..42 {
            for y in 20..60 {
                for z in 0..30 {
                    map.set_state([x, y, z], VoxelState::Occupied);
                }
            }
        }
    }
    let mut measured = State {
        position: Point3::new(1., 4., 1.),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let manager = PlannerManager::with_planner(
        Planner::new(PlannerConfig::default(), map),
        ManagerOptions::default(),
        measured.position.coords,
        Vector3::new(7., 4., 1.),
    )?;
    let mut fsm = ReplanFsm::new(manager);
    fsm.set_goal(0., Some(measured), Vector3::new(7., 4., 1.))?;
    let mut worker = PlanningWorker::new()?;
    let mut execution = Vec::new();
    let mut safety = Vec::new();
    let mut safety_lateness = Vec::new();
    let mut adopted = 0;
    let mut next_exec = Duration::from_millis(10);
    let mut next_safety = Duration::from_millis(50);
    let origin = Instant::now();
    log::info!("scenario round={round} async={deferred} obstacle={obstacle} updates={updates}");
    while origin.elapsed() < Duration::from_secs(3) {
        let next = next_exec.min(next_safety);
        if let Some(wait) = next.checked_sub(origin.elapsed()) {
            std::thread::sleep(wait);
        }
        let elapsed = origin.elapsed();
        let now = elapsed.as_secs_f64();
        if let Some(local) = fsm.manager().local() {
            let sample = local.traj.eval(now - local.start_time);
            measured = State {
                position: Point3::from(sample.position),
                velocity: sample.velocity,
                acceleration: sample.acceleration,
            };
        }
        if elapsed >= next_safety {
            safety_lateness.push(elapsed.saturating_sub(next_safety).as_secs_f64() * 1000.);
            let start = Instant::now();
            if updates {
                // Pin the pre-update snapshot to exercise the copy-on-write path even for short solves.
                let snapshot = fsm.manager().map().clone();
                let map = fsm.manager_mut().map_mut();
                map.move_ring_buffer(measured.position.coords);
                for i in 0..128 {
                    map.set_state(
                        [20 + i % 16, 120 + i / 16, 10],
                        if (elapsed.as_millis() / 50).is_multiple_of(2) {
                            VoxelState::Occupied
                        } else {
                            VoxelState::Free
                        },
                    );
                }
                map.inflate_obstacles();
                std::hint::black_box(snapshot);
            }
            if deferred {
                let _ = fsm.safety_tick_deferred(now);
            } else {
                adopted += u64::from(fsm.safety_tick(now, Some(measured)).replanned);
            }
            safety.push(start.elapsed().as_secs_f64() * 1000.);
            next_safety = Duration::from_millis((elapsed.as_millis() as u64 / 50 + 1) * 50);
        }
        if elapsed >= next_exec {
            let start = Instant::now();
            if deferred {
                let _ = fsm.exec_tick_deferred(now, Some(measured));
                let version = MapVersion {
                    localization: 1,
                    geometry: fsm.manager().map().revision(),
                };
                adopted += u64::from(worker.tick(&mut fsm, now, Some(measured), version));
            } else {
                adopted += u64::from(fsm.exec_tick(now, Some(measured)).replanned);
            }
            execution.push(start.elapsed().as_secs_f64() * 1000.);
            next_exec = Duration::from_millis((elapsed.as_millis() as u64 / 10 + 1) * 10);
        }
    }
    worker.cancel();
    let stats = worker.stats();
    for (metric, values) in [
        ("execution_ms", execution),
        ("safety_ms", safety),
        ("safety_lateness_ms", safety_lateness),
    ] {
        let mut sorted = values.clone();
        sorted.sort_by(f64::total_cmp);
        log::info!(
            "replanning round={round} async={deferred} obstacle={obstacle} metric={metric} count={} p50={:.6} p95={:.6} p99={:.6} max={:.6}",
            sorted.len(),
            sorted[sorted.len() / 2],
            sorted[sorted.len() * 95 / 100],
            sorted[sorted.len() * 99 / 100],
            sorted.last().unwrap()
        );
        for (chunk, samples) in values.chunks(24).enumerate() {
            let samples = samples
                .iter()
                .map(|value| format!("{value:.6}"))
                .collect::<Vec<_>>()
                .join(",");
            log::info!(
                "samples round={round} async={deferred} obstacle={obstacle} metric={metric} chunk={chunk} values={samples}"
            );
        }
    }
    log::info!(
        "replanning round={round} async={deferred} obstacle={obstacle} adopted={adopted} worker={stats:?} position={:?}",
        measured.position
    );
    assert!(
        adopted > 0,
        "callback responsiveness must retain planning progress"
    );
    Ok(())
}
