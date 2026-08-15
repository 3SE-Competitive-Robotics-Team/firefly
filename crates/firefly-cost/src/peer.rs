use firefly_trajectory::Trajectory;
/// 其他机轨迹（含绝对开始时间、机号与安全距离）。
///
/// 无人机与动态障碍统一表示：动态障碍的预测轨迹也作为 Peer，
/// 仅 `clearance`（按障碍体积）不同（论文：only different E and Cw values）。
#[derive(Debug, Clone)]
pub struct Peer {
    pub drone_id: usize,
    pub start_time: f64,
    pub traj: Trajectory,
    /// 该 peer 的安全距离（障碍体积决定，无人机为集群安全距离 Cw）。
    pub clearance: f64,
}

impl Peer {
    #[must_use]
    pub fn new(drone_id: usize, start_time: f64, traj: Trajectory, clearance: f64) -> Self {
        Self {
            drone_id,
            start_time,
            traj,
            clearance,
        }
    }
}
