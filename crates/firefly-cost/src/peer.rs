use firefly_trajectory::Trajectory;
/// 其他机轨迹（开始时间与查询时间必须使用同一时钟）。
///
/// 无人机与动态障碍统一表示：动态障碍的预测轨迹也作为 Peer，
/// 仅 `clearance`（按障碍体积）不同（论文：only different E and Cw values）。
#[derive(Debug, Clone)]
pub struct Peer {
    pub drone_id: usize,
    /// 秒；规划器以本次规划起点为零，调用方须将全局时间平移到该时钟。
    pub start_time: f64,
    pub traj: Trajectory,
    /// 该 peer 的安全距离（障碍体积决定，无人机为集群安全距离 Cw）。
    pub clearance: f64,
}

impl Peer {
    /// 在共享时钟上求值；负段内时间按多项式回推，末端之后匀速外推。
    #[must_use]
    pub fn sample_at(&self, time: f64) -> firefly_trajectory::Sample {
        let local = time - self.start_time;
        let duration = self.traj.duration();
        if local < duration {
            return self.traj.eval(local);
        }
        let mut state = self.traj.eval(duration);
        state.position += state.velocity * (local - duration);
        state.acceleration.fill(0.0);
        state.jerk.fill(0.0);
        state.snap.fill(0.0);
        state
    }

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
