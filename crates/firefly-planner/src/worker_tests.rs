use super::*;
use crate::{ExecState, ManagerOptions, Planner, PlannerConfig};
use firefly_map::{GridMapBuilder, VoxelState};
use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
use nalgebra::{Point3, Vector3};

fn state() -> State {
    State {
        position: Point3::new(1., 1., 1.),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    }
}
fn fsm() -> ReplanFsm {
    let manager = PlannerManager::with_planner(
        Planner::new(
            PlannerConfig::default(),
            GridMapBuilder::new(0.25, [40, 40, 16]).build().unwrap(),
        ),
        ManagerOptions::default(),
        state().position.coords,
        Vector3::new(4., 1., 1.),
    )
    .unwrap();
    let mut fsm = ReplanFsm::new(manager);
    fsm.set_goal(0., Some(state()), Vector3::new(4., 1., 1.))
        .unwrap();
    let _ = fsm.exec_tick_deferred(0., Some(state()));
    let _ = fsm.exec_tick_deferred(0., Some(state()));
    assert_eq!(fsm.state(), ExecState::SequentialStart);
    fsm
}
fn version(fsm: &ReplanFsm) -> MapVersion {
    MapVersion {
        localization: 1,
        geometry: fsm.manager.map().revision(),
    }
}
fn completion(fsm: &ReplanFsm) -> Completion {
    let endpoint = Endpoint {
        position: state().position.coords,
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let trajectory = MincoBuilder::new(
        SolverOrder::MinimumJerk,
        endpoint,
        Endpoint {
            position: Vector3::new(4., 1., 1.),
            ..endpoint
        },
    )
    .build(&[Point3::new(2.5, 1., 1.)], &[1.5, 1.5])
    .unwrap()
    .solve()
    .unwrap();
    let mut manager = fsm.manager.snapshot();
    assert!(manager.adopt_plan(
        PlanResult {
            trajectory,
            iterations: 1,
            planes: Vec::new()
        },
        0.,
        true
    ));
    Completion {
        ticket: Ticket {
            generation: fsm.generation,
            trajectory: fsm.manager.traj_id(),
            map: version(fsm),
            start: 0.,
            max_age: Duration::from_secs(1),
            cancellation: Cancellation {
                flag: Arc::new(AtomicBool::new(false)),
                deadline: Instant::now() + Duration::from_secs(10),
            },
        },
        candidate: Some((
            manager.local.take().unwrap(),
            manager.last_result.take().unwrap(),
            true,
        )),
        global: manager.global,
        replans: 1,
        failures: 0,
    }
}

#[test]
fn retarget_localization_stop_and_expiry_revoke_results() {
    for change in 0..7 {
        let mut fsm = fsm();
        let result = completion(&fsm);
        let mut map = version(&fsm);
        let mut now = 0.01;
        match change {
            0 => fsm
                .set_goal(now, Some(state()), Vector3::new(4., 2., 1.))
                .unwrap(),
            1 => fsm.invalidate_localization(now, state()).unwrap(),
            2 => fsm.mandatory_stop(now, Some(state())),
            3 => result.ticket.cancellation.cancel(),
            4 => now = result.ticket.max_age.as_secs_f64() + 0.001,
            5 => map.localization += 1,
            6 => now = -0.01,
            _ => unreachable!(),
        }
        assert!(!eligible(&result.ticket, &fsm, now, map), "change {change}");
        assert_eq!(fsm.manager.traj_id(), 0);
    }
}

#[test]
fn new_obstacle_rejects_candidate_without_overwriting_live_map() {
    let mut fsm = fsm();
    let result = completion(&fsm);
    let index = fsm
        .manager
        .map()
        .index_of(Vector3::new(2.5, 1., 1.))
        .unwrap();
    fsm.manager.map_mut().set_state(index, VoxelState::Occupied);
    assert_ne!(version(&fsm).geometry, result.ticket.map.geometry);
    assert!(eligible(&result.ticket, &fsm, 0.01, version(&fsm)));
    assert!(!adopt(result, &mut fsm, 0.01, Some(state())));
    assert!(fsm.manager.local().is_none());
    assert_eq!(fsm.manager.map().state(index), VoxelState::Occupied);
    assert_eq!(fsm.manager.traj_id(), 0);
}

#[test]
fn moving_window_revalidates_coverage_and_preserves_time_origin() {
    for center in [Vector3::new(4., 4., 2.), Vector3::new(20., 4., 2.)] {
        let mut fsm = fsm();
        let result = completion(&fsm);
        fsm.manager.map_mut().move_ring_buffer(center);
        let low = fsm.manager.map().global_index([0; 3]);
        let accepted = adopt(result, &mut fsm, 0.01, Some(state()));
        assert_eq!(accepted, center.x < 10.);
        assert_eq!(fsm.manager.map().global_index([0; 3]), low);
        if accepted {
            assert_eq!(fsm.manager.local().unwrap().start_time, 0.);
        }
    }
}

#[test]
fn changed_peer_rejects_result() {
    let mut fsm = fsm();
    let result = completion(&fsm);
    let peer = result.candidate.as_ref().unwrap().0.traj.clone();
    fsm.manager.register_swarm_traj(1, peer, 0., 0.5);
    assert!(!adopt(result, &mut fsm, 0.01, Some(state())));
}

#[test]
fn delayed_candidate_must_match_current_measured_position() {
    let mut fsm = fsm();
    let result = completion(&fsm);
    assert!(eligible(&result.ticket, &fsm, 0.9, version(&fsm)));
    assert!(
        (result.candidate.as_ref().unwrap().0.traj.eval(0.9).position - state().position.coords)
            .norm()
            > fsm.manager.map().resolution()
    );
    assert!(!adopt(result, &mut fsm, 0.9, Some(state())));
    assert!(fsm.manager.local().is_none());
}

#[test]
fn wall_deadline_expires_even_when_navigation_clock_is_paused() {
    let fsm = fsm();
    let mut result = completion(&fsm);
    result.ticket.cancellation.deadline = Instant::now();
    assert!(!eligible(&result.ticket, &fsm, 0., version(&fsm)));
}

#[test]
fn astar_cancellation_interrupts_search_and_allows_workspace_reuse() {
    let map = GridMapBuilder::new(0.1, [160, 160, 80]).build().unwrap();
    let mut search = firefly_search::Astar::default();
    let cancelled = Arc::new(AtomicBool::new(false));
    let flag = Arc::clone(&cancelled);
    search.set_cancellation(Some(Arc::new(move || flag.swap(true, Ordering::Relaxed))));
    assert!(
        search
            .search(&map, Vector3::new(1., 1., 1.), Vector3::new(14., 14., 1.))
            .is_err()
    );
    assert!(cancelled.load(Ordering::Relaxed));
    search.set_cancellation(None);
    assert!(
        search
            .search(&map, Vector3::new(1., 1., 1.), Vector3::new(14., 14., 1.))
            .is_ok()
    );
}

#[test]
fn stalled_solver_cannot_block_safety_or_queue_snapshots() {
    let mut fsm = fsm();
    assert!(adopt(completion(&fsm), &mut fsm, 0., Some(state())));
    fsm.set_goal(0., Some(state()), Vector3::new(4., 2., 1.))
        .unwrap();
    let (entered, entry) = mpsc::sync_channel(1);
    let (release, gate) = mpsc::sync_channel(1);
    let mut worker = PlanningWorker::with_solver(move |job| {
        entered.send(()).unwrap();
        let _ = gate.recv_timeout(Duration::from_secs(2));
        job.solve(&mut firefly_search::Astar::default())
    })
    .unwrap();
    let map = version(&fsm);
    assert!(!worker.tick(&mut fsm, 0., Some(state()), map));
    entry.recv_timeout(Duration::from_secs(2)).unwrap();
    for _ in 0..20 {
        assert!(!worker.tick(&mut fsm, 0.01, Some(state()), map));
    }
    assert_eq!(worker.stats().submitted, 1);
    let at = fsm.manager.local().unwrap().traj.eval(0.1).position;
    let index = fsm.manager.map().index_of(at).unwrap();
    fsm.manager.map_mut().set_state(index, VoxelState::Occupied);
    let report = fsm.safety_tick_deferred(0.01);
    assert!(report.collision && report.emergency_stop);
    worker.cancel();
    assert!(worker.active.as_ref().unwrap().cancellation.is_cancelled());
    let _ = fsm.exec_tick_deferred(0.01, Some(state()));
    let stopped_id = fsm.manager.traj_id();
    release.send(()).unwrap();
    let deadline = Instant::now() + Duration::from_secs(2);
    while worker.stats().discarded == 0 {
        assert!(Instant::now() < deadline);
        assert!(!worker.tick(&mut fsm, 0.02, Some(state()), map));
        std::thread::yield_now();
    }
    assert_eq!(worker.stats().adopted, 0);
    assert_eq!(fsm.manager.traj_id(), stopped_id);
}

#[test]
fn cancelled_solver_does_not_run_optimization_or_retries() {
    let fsm = fsm();
    let ticket = completion(&fsm).ticket;
    ticket.cancellation.cancel();
    let result = Job {
        ticket,
        manager: fsm.manager.snapshot(),
        kind: SolveKind::Global(state(), false),
    }
    .solve(&mut firefly_search::Astar::default());
    assert!(result.candidate.is_none());
    assert_eq!(result.failures, 0);
}

#[test]
fn expiry_between_eligibility_and_adoption_preserves_execution() {
    let mut fsm = fsm();
    let mut result = completion(&fsm);
    assert!(eligible(&result.ticket, &fsm, 0.01, version(&fsm)));
    result.ticket.cancellation.deadline = Instant::now();
    assert!(!adopt(result, &mut fsm, 0.01, Some(state())));
    assert!(fsm.manager.local().is_none());
    assert_eq!(fsm.manager.traj_id(), 0);
    assert_eq!(fsm.state(), ExecState::SequentialStart);
}

#[test]
fn invalid_solver_budget_stops_without_submitting() {
    for budget in [0., -1., f64::NAN, f64::INFINITY, 1e19] {
        let mut fsm = fsm();
        fsm.manager.options.emergency_time = budget;
        let mut worker = PlanningWorker::new().unwrap();
        let map = version(&fsm);
        assert!(!worker.tick(&mut fsm, 0., Some(state()), map));
        assert!(fsm.is_emergency());
        assert_eq!(worker.stats().submitted, 0);
    }
}
