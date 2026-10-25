//! 对照 EGO-Planner-v2 `ego_replan_fsm.cpp`：执行回调与安全回调。
use crate::{PlannerManager, State};
use firefly_error::Result;
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExecState {
    Init,
    WaitTarget,
    SequentialStart,
    GenNewTraj,
    ReplanTraj,
    ExecTraj,
    EmergencyStop,
}

// 各事件标志可同时成立，不承担状态编码。
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Default)]
pub struct TickReport {
    pub replanned: bool,
    pub finished: bool,
    pub collision: bool,
    pub emergency_stop: bool,
    pub ground_height: Option<f64>,
}

// 保留上游彼此独立的输入和 fail-safe 标志，执行状态由 ExecState 唯一编码。
#[allow(clippy::struct_excessive_bools)]
pub struct ReplanFsm {
    pub(crate) manager: PlannerManager,
    state: ExecState,
    state_calls: usize,
    have_target: bool,
    have_trigger: bool,
    have_new_target: bool,
    pub(crate) failsafe_enabled: bool,
    pub(crate) mandatory_stop: bool,
    escape_emergency: bool,
    pub(crate) finished: bool,
}

impl ReplanFsm {
    #[must_use]
    pub fn new(manager: PlannerManager) -> Self {
        Self {
            manager,
            state: ExecState::Init,
            state_calls: 1,
            have_target: false,
            have_trigger: false,
            have_new_target: false,
            failsafe_enabled: true,
            mandatory_stop: false,
            escape_emergency: true,
            finished: false,
        }
    }
    #[must_use]
    pub const fn state(&self) -> ExecState {
        self.state
    }
    #[must_use]
    pub const fn manager(&self) -> &PlannerManager {
        &self.manager
    }
    pub const fn manager_mut(&mut self) -> &mut PlannerManager {
        &mut self.manager
    }
    #[must_use]
    pub const fn has_target(&self) -> bool {
        self.have_target
    }
    #[must_use]
    pub const fn is_finished(&self) -> bool {
        self.finished
    }
    #[must_use]
    pub const fn is_mandatory_stopped(&self) -> bool {
        self.mandatory_stop
    }
    #[must_use]
    pub const fn is_emergency(&self) -> bool {
        matches!(self.state, ExecState::EmergencyStop)
    }
    fn change_state(&mut self, state: ExecState) {
        self.state_calls = if self.state == state {
            self.state_calls + 1
        } else {
            1
        };
        self.state = state;
    }
    /// 接入层将单个交互目标视作已有启动触发；预置航点可独立调用 trigger。
    /// # Errors
    /// 全局引导生成失败时保持当前任务。
    pub fn set_goal(
        &mut self,
        now: f64,
        measured: Option<State>,
        goal: Vector3<f64>,
    ) -> Result<()> {
        if self.mandatory_stop {
            return Ok(());
        }
        let changed = (goal - self.manager.goal().coords).norm() >= 1e-6;
        self.manager.set_goal(now, measured, goal)?;
        self.have_target = true;
        self.have_trigger = true;
        self.have_new_target |= changed;
        self.finished = false;
        if changed && matches!(self.state, ExecState::ExecTraj) {
            self.change_state(ExecState::ReplanTraj);
        }
        Ok(())
    }
    pub fn trigger(&mut self) {
        self.have_trigger = true;
    }
    /// # Errors
    /// 新坐标快照无法生成全局引导。
    pub fn invalidate_localization(&mut self, now: f64, measured: State) -> Result<()> {
        self.manager.invalidate_localization(now, measured)?;
        if !self.mandatory_stop {
            self.change_state(ExecState::GenNewTraj);
        }
        self.finished = false;
        Ok(())
    }
    pub(crate) fn request_emergency(&mut self) {
        self.change_state(ExecState::EmergencyStop);
    }
    pub fn mandatory_stop(&mut self, _now: f64, _measured: Option<State>) {
        self.mandatory_stop = true;
        self.failsafe_enabled = false;
        self.request_emergency();
    }
    pub fn trigger_emergency_stop_disable_failsafe(&mut self, _now: f64, _measured: Option<State>) {
        self.failsafe_enabled = false;
        self.change_state(ExecState::EmergencyStop);
    }
    /// 对照 execFSMCallback：一次回调只执行当前状态的分支。
    #[must_use]
    pub fn exec_tick(&mut self, now: f64, measured: Option<State>) -> TickReport {
        let mut report = TickReport::default();
        match self.state {
            ExecState::Init => {
                if measured.is_some() {
                    self.change_state(ExecState::WaitTarget);
                }
            }
            ExecState::WaitTarget => {
                if self.have_target && self.have_trigger {
                    self.change_state(ExecState::SequentialStart);
                }
            }
            ExecState::SequentialStart | ExecState::GenNewTraj => {
                if (self.state == ExecState::GenNewTraj || self.manager.sequential_start_ready())
                    && let Some(measured) = measured
                {
                    if self
                        .manager
                        .plan_from_global_traj(now, measured, 10, self.state_calls != 1)
                    {
                        self.have_new_target = false;
                        report.replanned = true;
                        self.escape_emergency = true;
                        self.change_state(ExecState::ExecTraj);
                    } else {
                        self.change_state(self.state);
                    }
                }
            }
            ExecState::ReplanTraj => {
                if self.plan_from_local(now) {
                    report.replanned = true;
                    self.change_state(ExecState::ExecTraj);
                } else {
                    self.change_state(ExecState::ReplanTraj);
                }
            }
            ExecState::ExecTraj => {
                self.manager.refresh_inflation();
                if self.manager.modify_in_collision_final_goal(now, measured) {
                    self.have_new_target = true;
                    self.change_state(ExecState::ReplanTraj);
                } else if let Some(local) = self.manager.local() {
                    let elapsed = (now - local.start_time).min(local.traj.duration());
                    if elapsed > local.traj.duration() - 1e-2 && self.manager.touch_goal {
                        self.have_target = false;
                        self.have_trigger = false;
                        self.finished = true;
                        report.finished = true;
                        self.change_state(ExecState::WaitTarget);
                    } else if elapsed > self.manager.options.replan_thresh
                        || (!self.manager.touch_goal && self.manager.close_to_traj_end(elapsed))
                    {
                        self.change_state(ExecState::ReplanTraj);
                    }
                }
            }
            ExecState::EmergencyStop => {
                if self.escape_emergency {
                    if let Some(measured) = measured {
                        self.manager.emergency_stop(now, measured.position.coords);
                        report.replanned = true;
                    }
                } else if self.failsafe_enabled
                    && measured.is_some_and(|s| {
                        s.velocity.norm() < self.manager.options.emergency_recover_speed
                    })
                {
                    self.change_state(ExecState::GenNewTraj);
                }
                self.escape_emergency = false;
            }
        }
        report.emergency_stop = self.is_emergency();
        report
    }
    fn plan_from_local(&mut self, now: f64) -> bool {
        let result = self
            .manager
            .plan_from_local_traj(now, 1, self.have_new_target);
        self.have_new_target = false;
        result
    }
    /// 对照 checkCollisionCallback，安全检查有独立的 20Hz 调用入口。
    #[must_use]
    pub fn safety_tick(&mut self, now: f64, _measured: Option<State>) -> TickReport {
        let mut report = TickReport {
            ground_height: self.manager.measure_ground_height(now),
            ..TickReport::default()
        };
        if self.state == ExecState::WaitTarget || self.manager.traj_id() == 0 {
            return report;
        }
        self.manager.refresh_inflation();
        let hit = match (
            self.manager.next_collision(now),
            self.manager.next_swarm_conflict(now),
        ) {
            (Some(obstacle), Some((_, peer))) => Some(obstacle.min(peer)),
            (Some(hit), None) | (None, Some((_, hit))) => Some(hit),
            (None, None) => None,
        };
        if let Some(hit) = hit {
            report.collision = true;
            let elapsed = self
                .manager
                .local()
                .map_or(0.0, |local| now - local.start_time);
            if self.plan_from_local(now) {
                report.replanned = true;
                self.change_state(ExecState::ExecTraj);
            } else if hit - elapsed < self.manager.options.emergency_time {
                self.change_state(ExecState::EmergencyStop);
            } else {
                self.change_state(ExecState::ReplanTraj);
            }
        }
        report.emergency_stop = self.is_emergency();
        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ManagerOptions, Planner, PlannerConfig};
    use firefly_map::GridMapBuilder;
    use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
    use nalgebra::Point3;

    fn measured() -> State {
        State {
            position: Point3::new(1.0, 1.0, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        }
    }
    fn fsm() -> ReplanFsm {
        let planner = Planner::new(
            PlannerConfig::default(),
            GridMapBuilder::new(0.5, [16, 16, 12]).build().unwrap(),
        );
        ReplanFsm::new(
            PlannerManager::with_planner(
                planner,
                ManagerOptions::default(),
                measured().position.coords,
                Vector3::new(5.0, 1.0, 1.0),
            )
            .unwrap(),
        )
    }
    fn start(fsm: &mut ReplanFsm) {
        fsm.set_goal(0.0, Some(measured()), Vector3::new(5.0, 1.0, 1.0))
            .unwrap();
        assert_eq!(fsm.state(), ExecState::Init);
        assert!(!fsm.exec_tick(0.0, None).replanned);
        assert_eq!(fsm.state(), ExecState::Init);
        let _ = fsm.exec_tick(0.01, Some(measured()));
        assert_eq!(fsm.state(), ExecState::WaitTarget);
        let _ = fsm.exec_tick(0.02, Some(measured()));
        assert_eq!(fsm.state(), ExecState::SequentialStart);
        assert!(fsm.manager.local().is_none());
        assert!(fsm.exec_tick(0.03, Some(measured())).replanned);
        assert_eq!(fsm.state(), ExecState::ExecTraj);
    }
    #[test]
    fn callbacks_keep_upstream_state_boundaries() {
        let mut fsm = fsm();
        start(&mut fsm);
        let id = fsm.manager.traj_id();
        let report = fsm.exec_tick(1.04, Some(measured()));
        assert!(!report.replanned);
        assert_eq!(fsm.state(), ExecState::ReplanTraj);
        assert_eq!(fsm.manager.traj_id(), id);
        assert!(fsm.exec_tick(1.05, Some(measured())).replanned);
        assert_eq!(fsm.state(), ExecState::ExecTraj);
        assert_eq!(fsm.manager.traj_id(), id + 1);
    }
    #[test]
    fn retarget_keeps_previous_trajectory_until_new_result() {
        let mut fsm = fsm();
        start(&mut fsm);
        let id = fsm.manager.traj_id();
        fsm.set_goal(0.2, Some(measured()), Vector3::new(4.0, 2.0, 1.0))
            .unwrap();
        assert_eq!(fsm.state(), ExecState::ReplanTraj);
        assert_eq!(fsm.manager.local().unwrap().traj_id, id);
        assert!(fsm.exec_tick(0.21, Some(measured())).replanned);
        assert_eq!(fsm.manager.traj_id(), id + 1);
    }
    #[test]
    fn first_generation_retries_ten_times_then_uses_random_initialization() {
        let mut fsm = fsm();
        // 固定起点与对端重合，任何候选都无法满足集群后检查。
        let endpoint = Endpoint {
            position: measured().position.coords,
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let peer = MincoBuilder::new(SolverOrder::MinimumJerk, endpoint, endpoint)
            .build(&[], &[30.0])
            .unwrap()
            .solve()
            .unwrap();
        fsm.manager.register_swarm_traj(1, peer, 0.0, 0.5);
        fsm.set_goal(0.0, Some(measured()), Vector3::new(5.0, 1.0, 1.0))
            .unwrap();
        let _ = fsm.exec_tick(0.0, Some(measured()));
        let _ = fsm.exec_tick(0.01, Some(measured()));
        assert!(!fsm.exec_tick(0.02, Some(measured())).replanned);
        assert_eq!(fsm.state(), ExecState::SequentialStart);
        assert_eq!(fsm.manager.consecutive_failures, 10);
        assert!(!fsm.exec_tick(0.03, Some(measured())).replanned);
        assert_eq!(fsm.manager.consecutive_failures, 20);
    }
    #[test]
    fn emergency_transition_and_recovery_take_separate_callbacks() {
        let mut fsm = fsm();
        start(&mut fsm);
        let id = fsm.manager.traj_id();
        fsm.request_emergency();
        assert_eq!(fsm.manager.traj_id(), id);
        assert!(fsm.exec_tick(0.2, Some(measured())).emergency_stop);
        assert_eq!(fsm.manager.traj_id(), id + 1);
        let report = fsm.exec_tick(0.21, Some(measured()));
        assert!(!report.replanned);
        assert_eq!(fsm.state(), ExecState::GenNewTraj);
        assert!(fsm.exec_tick(0.22, Some(measured())).replanned);
        assert_eq!(fsm.state(), ExecState::ExecTraj);
    }
}
