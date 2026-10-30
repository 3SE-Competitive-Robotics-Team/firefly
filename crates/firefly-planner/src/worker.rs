//! A single bounded solver lane. Execution owns the live map and all adoption decisions.
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
    mpsc,
};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::manager::GlobalTraj;
use crate::{LocalTraj, PlanResult, PlannerManager, ReplanFsm, State};

#[derive(Clone)]
pub(crate) struct Cancellation {
    flag: Arc<AtomicBool>,
    deadline: Instant,
}
impl Cancellation {
    pub(crate) fn is_cancelled(&self) -> bool {
        self.flag.load(Ordering::Relaxed) || Instant::now() >= self.deadline
    }
    fn cancel(&self) {
        self.flag.store(true, Ordering::Relaxed);
    }
}

pub(crate) enum SolveKind {
    Global(State, bool),
    Local(bool),
}

/// Identifies the coordinate frame and the exact derived grid used for a solve.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MapVersion {
    pub localization: u64,
    pub geometry: u64,
}

#[derive(Clone)]
struct Ticket {
    generation: u64,
    trajectory: u32,
    map: MapVersion,
    start: f64,
    max_age: Duration,
    cancellation: Cancellation,
}
struct Job {
    ticket: Ticket,
    manager: PlannerManager,
    kind: SolveKind,
}
struct Completion {
    ticket: Ticket,
    candidate: Option<(LocalTraj, PlanResult, bool)>,
    global: GlobalTraj,
    replans: usize,
    failures: u32,
}
impl Job {
    fn solve(mut self, workspace: &mut firefly_search::Astar) -> Completion {
        self.manager.planner.exchange_workspace(workspace);
        self.manager
            .planner
            .set_cancellation(self.ticket.cancellation.clone());
        let success = match self.kind {
            SolveKind::Global(state, random) => {
                self.manager
                    .plan_from_global_traj(self.ticket.start, state, 10, random)
            }
            SolveKind::Local(new_target) => {
                self.manager
                    .plan_from_local_traj(self.ticket.start, 1, new_target)
            }
        };
        self.manager.planner.exchange_workspace(workspace);
        let candidate = if success {
            self.manager
                .local
                .take()
                .zip(self.manager.last_result.take())
                .map(|(local, result)| (local, result, self.manager.touch_goal))
        } else {
            None
        };
        Completion {
            ticket: self.ticket,
            candidate,
            global: self.manager.global,
            replans: self.manager.replans,
            failures: self.manager.consecutive_failures,
        }
    }
}

/// At most one outstanding snapshot. Cancelling never waits or queues another stale snapshot.
pub struct PlanningWorker {
    sender: Option<mpsc::SyncSender<Job>>,
    receiver: mpsc::Receiver<Completion>,
    thread: Option<JoinHandle<()>>,
    active: Option<Ticket>,
    stats: WorkerStats,
}

/// Counters and snapshot cost measured on the execution thread.
#[derive(Default, Clone, Copy, Debug)]
pub struct WorkerStats {
    pub submitted: u64,
    pub adopted: u64,
    pub discarded: u64,
    pub revalidated: u64,
    pub snapshot_time: Duration,
    pub first_snapshot_time: Duration,
    pub max_snapshot_time: Duration,
}
impl PlanningWorker {
    /// # Errors
    /// The operating system cannot create the solver thread.
    pub fn new() -> std::io::Result<Self> {
        let mut workspace = firefly_search::Astar::default();
        Self::with_solver(move |job| job.solve(&mut workspace))
    }
    fn with_solver(
        mut solve: impl FnMut(Job) -> Completion + Send + 'static,
    ) -> std::io::Result<Self> {
        let (sender, jobs) = mpsc::sync_channel(1);
        let (results, receiver) = mpsc::sync_channel(1);
        let thread = thread::Builder::new()
            .name("planner-solver".into())
            .spawn(move || {
                while let Ok(job) = jobs.recv() {
                    let completion = solve(job);
                    if results.send(completion).is_err() {
                        break;
                    }
                }
            })?;
        Ok(Self {
            sender: Some(sender),
            receiver,
            thread: Some(thread),
            active: None,
            stats: WorkerStats::default(),
        })
    }

    #[must_use]
    pub const fn stats(&self) -> WorkerStats {
        self.stats
    }

    /// Revoke eligibility immediately, including when navigation/map freshness is lost.
    pub fn cancel(&mut self) {
        if let Some(ticket) = &self.active {
            ticket.cancellation.cancel();
        }
    }

    /// Call only after synchronizing and checking live navigation and map coverage/freshness.
    /// Returns true only when a candidate was adopted. No solver work or blocking receive occurs.
    pub fn tick(
        &mut self,
        fsm: &mut ReplanFsm,
        now: f64,
        measured: Option<State>,
        map: MapVersion,
    ) -> bool {
        if self
            .active
            .as_ref()
            .is_some_and(|ticket| !eligible(ticket, fsm, now, map))
        {
            self.cancel();
        }
        let mut adopted = false;
        match self.receiver.try_recv() {
            Ok(completion) => {
                self.active = None;
                if eligible(&completion.ticket, fsm, now, map) {
                    if completion.ticket.map.geometry != map.geometry {
                        self.stats.revalidated += 1;
                    }
                    adopted = adopt(completion, fsm, now, measured);
                }
                if adopted {
                    self.stats.adopted += 1;
                } else {
                    self.stats.discarded += 1;
                }
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.cancel();
                fsm.trigger_emergency_stop_disable_failsafe(now, measured);
                return false;
            }
            Err(mpsc::TryRecvError::Empty) => {}
        }
        if self.active.is_none()
            && let Some(kind) = fsm.solve_kind(measured)
        {
            let max_age =
                Duration::try_from_secs_f64(fsm.manager.options.emergency_time).unwrap_or_default();
            let Some(deadline) = Instant::now()
                .checked_add(max_age)
                .filter(|_| !max_age.is_zero())
            else {
                fsm.trigger_emergency_stop_disable_failsafe(now, measured);
                return false;
            };
            let ticket = Ticket {
                generation: fsm.generation,
                trajectory: fsm.manager.traj_id(),
                map,
                start: now,
                max_age,
                cancellation: Cancellation {
                    flag: Arc::new(AtomicBool::new(false)),
                    deadline,
                },
            };
            let snapshot_start = Instant::now();
            fsm.manager.refresh_inflation();
            let job = Job {
                ticket: ticket.clone(),
                manager: fsm.manager.snapshot(),
                kind,
            };
            let snapshot_time = snapshot_start.elapsed();
            self.stats.snapshot_time += snapshot_time;
            self.stats.max_snapshot_time = self.stats.max_snapshot_time.max(snapshot_time);
            if self.stats.submitted == 0 {
                self.stats.first_snapshot_time = snapshot_time;
            }
            if self
                .sender
                .as_ref()
                .is_some_and(|sender| sender.try_send(job).is_ok())
            {
                self.stats.submitted += 1;
                self.active = Some(ticket);
            } else {
                fsm.trigger_emergency_stop_disable_failsafe(now, measured);
            }
        }
        adopted
    }
}
impl Drop for PlanningWorker {
    fn drop(&mut self) {
        self.cancel();
        self.sender.take();
        // There is at most one result; the worker cannot block behind an unconsumed result.
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn eligible(ticket: &Ticket, fsm: &ReplanFsm, now: f64, map: MapVersion) -> bool {
    !ticket.cancellation.is_cancelled()
        && now.is_finite()
        && now >= ticket.start
        && now - ticket.start <= ticket.max_age.as_secs_f64()
        && ticket.generation == fsm.generation
        && ticket.trajectory == fsm.manager.traj_id()
        && ticket.map.localization == map.localization
        && !fsm.is_emergency()
        && !fsm.is_mandatory_stopped()
}

fn adopt(completion: Completion, fsm: &mut ReplanFsm, now: f64, measured: Option<State>) -> bool {
    fsm.manager.refresh_inflation();
    if completion.ticket.cancellation.is_cancelled() {
        return false;
    }
    let Some((local, result, touch_goal)) = completion.candidate else {
        fsm.manager.consecutive_failures = completion.failures;
        fsm.solve_finished(false);
        return false;
    };
    let elapsed = now - local.start_time;
    // A delayed candidate must still start near the vehicle at its original time origin.
    // Bound the handoff displacement by the existing map discretization, not solver duration.
    if measured.is_none_or(|state| {
        (local.traj.eval(elapsed).position - state.position.coords).norm()
            > fsm.manager.map().resolution()
    }) || elapsed >= local.traj.duration()
        || local
            .points_to_check
            .last()
            .and_then(|bucket| bucket.last())
            .is_none_or(|(t, _)| *t <= elapsed)
        || local.points_to_check.iter().flatten().any(|(t, p)| {
            *t >= elapsed
                && (fsm.manager.map().index_of(*p).is_none()
                    || fsm.manager.map().is_occupied_inflated(*p))
        })
    {
        return false;
    }
    // Revalidate against the current grid (including moved windows) and current peers.
    // Never install the worker's map or rebase a trajectory's time origin.
    let previous = fsm.manager.local.replace(local);
    let previous_touch = std::mem::replace(&mut fsm.manager.touch_goal, touch_goal);
    let previous_id = fsm.manager.traj_id;
    fsm.manager.traj_id = previous_id + 1;
    if fsm.manager.next_swarm_conflict(now).is_some()
        || completion.ticket.cancellation.is_cancelled()
    {
        fsm.manager.local = previous;
        fsm.manager.touch_goal = previous_touch;
        fsm.manager.traj_id = previous_id;
        return false;
    }
    fsm.manager.global = completion.global;
    fsm.manager.last_result = Some(result);
    fsm.manager.replans = completion.replans;
    fsm.manager.consecutive_failures = completion.failures;
    fsm.solve_finished(true);
    true
}

#[cfg(test)]
#[path = "worker_tests.rs"]
mod tests;
