//! 规划器配置（论文 Table S6 参数）。
//!
//! 默认值取仿真场景 A–D；实飞场景（E–H）安全距离更小、速度更低。

#[derive(Debug, Clone)]
pub struct PlannerConfig {
    pub trajectory_pieces: usize,
    pub constraint_points_per_piece: usize,
    pub planning_distance: f64,
    pub obstacle_clearance: f64,
    pub swarm_clearance: f64,
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub max_jerk: f64,
    pub weight_smoothness: f64,
    pub weight_time: f64,
    pub weight_feasibility: f64,
    pub weight_obstacle: f64,
    pub weight_swarm: f64,
    pub weight_formation: f64,
}

impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            trajectory_pieces: 5,
            constraint_points_per_piece: 5,
            planning_distance: 7.5,
            obstacle_clearance: 0.5,
            swarm_clearance: 0.5,
            max_velocity: 1.5,
            max_acceleration: 6.0,
            max_jerk: 10.0,
            weight_smoothness: 1.0,
            weight_time: 10.0,
            weight_feasibility: 10_000.0,
            weight_obstacle: 10_000.0,
            weight_swarm: 10_000.0,
            weight_formation: 100.0,
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
        assert_eq!(c.trajectory_pieces, 5);
        assert_eq!(c.constraint_points_per_piece, 5);
        assert_eq!(c.obstacle_clearance, 0.5);
        assert_eq!(c.swarm_clearance, 0.5);
        assert_eq!(c.max_velocity, 1.5);
        assert_eq!(c.max_acceleration, 6.0);
        assert_eq!(c.max_jerk, 10.0);
        assert_eq!(c.weight_smoothness, 1.0);
        assert_eq!(c.weight_time, 10.0);
        assert_eq!(c.weight_feasibility, 10_000.0);
        assert_eq!(c.weight_obstacle, 10_000.0);
        assert_eq!(c.weight_swarm, 10_000.0);
        assert_eq!(c.weight_formation, 100.0);
    }
}
