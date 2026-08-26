//! 规划器配置（论文 Table S6 参数）。
//!
//! 默认值取仿真场景 A–D；实飞场景（E–H）安全距离更小、速度更低。

#[derive(Debug, Clone, serde::Deserialize)]
#[serde(default)]
pub struct PlannerConfig {
    /// 每段路径长度（米），段数随引导路径长度自适应（官方 `polyTraj_piece_length`）。
    pub piece_length: f64,
    pub constraint_points_per_piece: usize,
    pub planning_distance: f64,
    pub obstacle_clearance: f64,
    /// 障碍软净距（官方 v2 `obstacle_clearance_soft`，平滑尾）。
    pub obstacle_clearance_soft: f64,
    /// 软层权重（官方 v2 `weight_obstacle_soft`）。
    pub weight_obstacle_soft: f64,
    /// 集群安全净距（米，官方 `optimization/swarm_clearance`；swarm-playground
    /// 各 launch 取 0.5）：FSM 层 swarm 预测检查与规划内层 swarm 惩罚共用。
    pub swarm_clearance: f64,
    /// 本机编号（官方 `manager/drone_id`，launch 默认 0）：≤0 视为单机/0 号机，
    /// 初始规划无需等待；≥1 必须收齐 `0..drone_id` 全部前序机轨迹才允许起飞
    /// （官方 `SEQUENTIAL_START` 态语义）。
    pub drone_id: i32,
    /// 心跳超时（秒，官方 `traj_server::cmdCallback` 硬编码 0.5）：超过该时长
    /// 未收到新执行节拍时，参考输出降级为零速悬停并停用轨迹跟踪。
    pub heartbeat_timeout: f64,
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub max_jerk: f64,
    pub weight_smoothness: f64,
    pub weight_time: f64,
    pub weight_feasibility: f64,
    pub weight_obstacle: f64,
    pub weight_swarm: f64,
    pub weight_formation: f64,
    /// 约束点均匀分布权重(官方 `weight_sqrvariance`)。
    pub weight_sqrvariance: f64,
    /// 多拓扑候选轨迹(官方 `manager/use_multitopology_trajs`,默认关闭;
    /// 关闭时规划路径与单候选完全一致)。
    pub use_multitopology_trajs: bool,
    /// 虚拟地板 z（米，世界坐标；官方 `grid_map/virtual_ground`，None = 不启用）。
    pub virtual_ground: Option<f64>,
    /// 虚拟天花板 z（米，世界坐标；官方 `grid_map/virtual_ceil`，None = 不启用）。
    pub virtual_ceiling: Option<f64>,
}

impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            piece_length: 1.5,
            // 官方 Table S6 / advanced_param.xml:constraint_points_perPiece = 5
            constraint_points_per_piece: 5,
            planning_distance: 7.5,
            obstacle_clearance: 0.1,
            obstacle_clearance_soft: 0.5,
            weight_obstacle_soft: 5000.0,
            swarm_clearance: 0.5,
            drone_id: 0,
            heartbeat_timeout: 0.5,
            max_velocity: 1.5,
            max_acceleration: 6.0,
            max_jerk: 10.0,
            weight_smoothness: 1.0,
            weight_time: 10.0,
            weight_feasibility: 10_000.0,
            weight_obstacle: 10_000.0,
            weight_swarm: 10_000.0,
            weight_formation: 100.0,
            weight_sqrvariance: 10_000.0,
            use_multitopology_trajs: false,
            virtual_ground: None,
            virtual_ceiling: None,
        }
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_paper_table_s6() {
        let c = PlannerConfig::default();
        assert_eq!(c.constraint_points_per_piece, 5);
        assert_eq!(c.obstacle_clearance, 0.1);
        assert_eq!(c.obstacle_clearance_soft, 0.5);
        assert_eq!(c.weight_obstacle_soft, 5000.0);
        assert_eq!(c.swarm_clearance, 0.5);
        // 官方 manager/drone_id launch 默认 0；traj_server 心跳超时硬编码 0.5s
        assert_eq!(c.drone_id, 0);
        assert_eq!(c.heartbeat_timeout, 0.5);
        assert_eq!(c.max_velocity, 1.5);
        assert_eq!(c.max_acceleration, 6.0);
        assert_eq!(c.max_jerk, 10.0);
        assert_eq!(c.weight_smoothness, 1.0);
        assert_eq!(c.weight_time, 10.0);
        assert_eq!(c.weight_feasibility, 10_000.0);
        assert_eq!(c.weight_obstacle, 10_000.0);
        assert_eq!(c.weight_swarm, 10_000.0);
        assert_eq!(c.weight_formation, 100.0);
        assert_eq!(c.weight_sqrvariance, 10_000.0);
        // 官方 planner_manager.cpp:24 默认 false(run_in_sim.xml 同)
        assert!(!c.use_multitopology_trajs);
        // 官方 grid_map.cpp 默认 enable_virtual_wall = false
        assert_eq!(c.virtual_ground, None);
        assert_eq!(c.virtual_ceiling, None);
    }
}
