//! 前端初始化：A* 引导路径 → MINCO 参数 {q, T} 与边界条件。

use firefly_error::Result;
use firefly_map::GridMap;
use firefly_search::Astar;
use firefly_trajectory::{Endpoint, Minco, MincoBuilder, SolverOrder, Trajectory};
use nalgebra::{Point3, Vector3};

pub struct InitConfig {
    pub pieces: usize,
    pub max_velocity: f64,
    /// 每段路径长度（米，官方 `polyTraj_piece_length`；暖启动段数按它计算）。
    pub piece_length: f64,
}

/// 段数由引导路径拐点数决定（官方 initMJO 以拐点为 waypoint），
/// 长度兜底防止过疏，限制 [5, 24]。
#[must_use]
pub fn pieces_for_guide(guide: &[Vector3<f64>], piece_length: f64) -> usize {
    let len: f64 = guide.windows(2).map(|w| (w[1] - w[0]).norm()).sum();
    let by_len = ((len / piece_length.max(1e-3)).ceil() as usize).min(24);
    let corners = corner_indices(guide).len();
    (corners + 1).clamp(5, 24).max(by_len).min(24)
}

/// 引导路径拐点索引（方向变化 > ~8°）。
fn corner_indices(path: &[Vector3<f64>]) -> Vec<usize> {
    (1..path.len() - 1)
        .filter(|&i| {
            (path[i] - path[i - 1])
                .normalize()
                .dot(&(path[i + 1] - path[i]).normalize())
                < 0.99
        })
        .collect()
}

/// # Errors
///
/// `NotFound`/`OutOfRange`/`Convergence`：A* 搜索失败（目标不可达等）。
pub fn search_guide(
    astar: &mut Astar,
    map: &GridMap,
    start: Vector3<f64>,
    goal: Vector3<f64>,
) -> Result<Vec<Vector3<f64>>> {
    let path = astar.search(map, start, goal)?;
    Ok(firefly_search::simplify_path(map, path.points()))
}

/// 从引导路径生成 MINCO 初始解。
/// # Errors
///
/// `InvalidArgument`：引导路径过短；`Convergence`：MINCO 系统奇异。
pub fn init_from_path(
    config: &InitConfig,
    start: Endpoint,
    goal: Point3<f64>,
    guide: &[Vector3<f64>],
) -> Result<Minco> {
    if guide.len() < 2 {
        // 近终点 / 退化引导（A* 到很近目标路径退化为 ≤1 点）：不报错，退化为
        // start→goal 的单段直飞 MINCO。否则 demo 在终点外一小段反复
        // "guide path too short" → 悬停卡死无法抵达（>ARRIVE_DIST 完成不了）。
        let dist = (goal.coords - start.position).norm();
        let t = (dist / config.max_velocity).max(1e-3);
        let end = Endpoint {
            position: goal.coords,
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        return MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&[], &[t])
            .map_err(|e| e.with_operation("planner::init:degenerate"));
    }
    let pieces = config.pieces;
    let waypoints = sample_waypoints(guide, pieces - 1);
    // 完整段端点：start → waypoints → goal
    let mut segments = Vec::with_capacity(pieces);
    let mut prev = start.position;
    for q in &waypoints {
        segments.push((q.coords - prev).norm());
        prev = q.coords;
    }
    segments.push((goal.coords - prev).norm());
    let durations = allocate_time(&segments, config.max_velocity);
    let end = Endpoint {
        position: goal.coords,
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&waypoints, &durations)
        .map_err(|e| e.with_operation("planner::init"))
}

/// 暖启动初始解（官方 `computeInitState` case 2，planner_manager.cpp:255-320）：
/// 以上一条最优轨迹的剩余段为主干，耗尽后沿全局轨迹段
/// （`last_glb_t_of_lc_tgt → glb_t_of_lc_tgt`，实体为 `guide_tail` 的等时采样）
/// 接到局部目标。
///
/// 组合时间轴：前 `remaining = prev.duration() - elapsed` 秒取自旧轨迹
/// （绝对时刻 `elapsed + t` 采样），其后 `glb_seg` 秒取自全局轨迹段——内点在
/// 组合轴上均匀取（`piece_dur = t_to_lc_tgt / pieces`，`t_to_lc_tgt =
/// remaining + glb_seg`），段时长同此均匀分布（官方 `piece_dur_vec =
/// Constant(piece_nums, t_to_lc_tgt / piece_nums)`）。`guide_tail` 为全局轨迹
/// 段的等时采样（含两端），按归一化时间线性插值取点；缺失（空）时以旧轨迹
/// 末端兜底。
///
/// 文档化偏离：官方 case2 的 MINCO 尾状态为 `[目标, local_target_vel, 0]`
/// （局部目标处延续全局轨迹速度）；firefly 的优化目标端状态固定零速，
/// 此处以零速收尾（对各个局部目标的到达判定不变量一致）。
///
/// # Errors
///
/// 旧轨迹已耗尽（`elapsed ≥ duration`）、`glb_seg < 0` 或组合轴退化时返回
/// `InvalidArgument`——调用方应降级冷启动（官方 case2 → case1 策略链）。
pub fn init_warm_start(
    config: &InitConfig,
    start: Endpoint,
    goal: Point3<f64>,
    prev: &Trajectory,
    elapsed: f64,
    glb_seg: f64,
    guide_tail: &[Vector3<f64>],
) -> Result<Minco> {
    let remaining = prev.duration() - elapsed;
    if remaining <= 0.05 {
        return Err(firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            format!("旧轨迹剩余 {remaining:.3}s，暖启动退化为冷启动"),
        ));
    }
    if glb_seg < 0.0 {
        return Err(firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            format!("全局轨迹段时长非法（glb_seg={glb_seg:.3}）"),
        ));
    }
    let t_to_lc_tgt = remaining + glb_seg;
    // 官方 case2 段数 = ceil(直线距离/piece_length)，下限 2
    let dist = (goal.coords - start.position).norm();
    let pieces = ((dist / config.piece_length.max(1e-3)).ceil() as usize).clamp(2, 24);
    let piece_dur = t_to_lc_tgt / pieces as f64;
    if piece_dur <= 0.0 {
        return Err(firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            "组合时间轴退化（t_to_lc_tgt <= 0）",
        ));
    }
    // 内点：组合时间轴上均匀取段。t < remaining 取旧轨迹；其后取全局轨迹
    // 段（guide_tail 按归一化时间 u ∈ [0,1] 线性插值）。
    let fallback = prev.eval(prev.duration()).position;
    let tail_len = guide_tail.len();
    let mut waypoints = Vec::with_capacity(pieces - 1);
    let mut t = piece_dur;
    for _ in 0..pieces - 1 {
        let pos = if t < remaining {
            prev.eval(elapsed + t).position
        } else if tail_len == 0 {
            fallback
        } else {
            let u = ((t - remaining) / glb_seg).clamp(0.0, 1.0);
            let f = u * (tail_len - 1) as f64;
            let k = (f.floor() as usize).min(tail_len - 2);
            let alpha = f - k as f64;
            guide_tail[k] * (1.0 - alpha) + guide_tail[k + 1] * alpha
        };
        waypoints.push(Point3::from(pos));
        t += piece_dur;
    }
    let durations = vec![piece_dur; pieces];
    let end = Endpoint {
        position: goal.coords,
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&waypoints, &durations)
        .map_err(|e| e.with_operation("planner::init:warm_start"))
}

/// 取 count 个中间 waypoint（不含两端）：拐点优先（官方 initMJO），
/// 拐点不足时按弧长均匀补充。
fn sample_waypoints(path: &[Vector3<f64>], count: usize) -> Vec<Point3<f64>> {
    let corners = corner_indices(path);
    if corners.len() >= count {
        return (0..count)
            .map(|k| Point3::from(path[corners[k * corners.len() / count]]))
            .collect();
    }
    // 拐点全部 + 均匀补充（按弧长插值）
    let mut result: Vec<Point3<f64>> = corners.iter().map(|&i| Point3::from(path[i])).collect();
    let arcs = arc_lengths(path);
    let total = *arcs.last().unwrap_or(&0.0);
    let need = count - result.len();
    let mut seg = 0usize;
    for k in 1..=need {
        let target = total * k as f64 / (need + 1) as f64;
        while seg + 1 < arcs.len() && arcs[seg + 1] < target {
            seg += 1;
        }
        let seg_len = arcs[seg + 1] - arcs[seg];
        let alpha = (target - arcs[seg]) / seg_len;
        result.push(Point3::from(
            path[seg] + alpha * (path[seg + 1] - path[seg]),
        ));
    }
    result
}

/// 路径累计弧长。
fn arc_lengths(path: &[Vector3<f64>]) -> Vec<f64> {
    let mut arcs = Vec::with_capacity(path.len());
    let mut acc = 0.0;
    arcs.push(0.0);
    for w in path.windows(2) {
        acc += (w[1] - w[0]).norm();
        arcs.push(acc);
    }
    arcs
}

/// `按段长分配时间：T_i` ∝ 段长，总时长 = `2×路径长/v_max`。
/// 系数 2.0：最小 jerk 静止到静止轨迹峰值速度 ≈ 1.875×平均速度，
/// 保证初始轨迹可行（否则可行性惩罚从初始就爆炸，优化无法收敛）。
fn allocate_time(segments: &[f64], max_velocity: f64) -> Vec<f64> {
    let total: f64 = segments.iter().sum();
    let budget = 2.0 * total / max_velocity.max(1e-3);
    segments
        .iter()
        .map(|l| l / total.max(1e-9) * budget)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn degenerate_near_goal_builds_trivial_minco() {
        // 近终点：A* 引导路径退化（≤1 点）时产出单段直飞 MINCO，不报错
        let config = InitConfig {
            pieces: 1,
            max_velocity: 2.0,
            piece_length: 1.5,
        };
        let start = Endpoint {
            position: Vector3::new(8.0, 4.0, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let goal = Point3::new(8.6, 4.0, 1.0);
        let guide = vec![Vector3::new(8.0, 4.0, 1.0)];
        let m = init_from_path(&config, start, goal, &guide).expect("近终点退化不应报错");
        assert_eq!(m.pieces(), 1);
        assert!(m.duration() > 0.0);
        let traj = m.solve().expect("退化 MINCO 应可解");
        let sf = traj.eval(traj.duration());
        assert!((sf.position - goal.coords).norm() < 1e-6, "终点应是 goal");
    }

    #[test]
    fn samples_along_path_evenly() {
        let path: Vec<Vector3<f64>> = (0..=10)
            .map(|k| Vector3::new(f64::from(k), 0.0, 0.0))
            .collect();
        let pts = sample_waypoints(&path, 3);
        assert_eq!(pts.len(), 3);
        assert!((pts[0].x - 2.5).abs() < 1e-9);
        assert!((pts[1].x - 5.0).abs() < 1e-9);
        assert!((pts[2].x - 7.5).abs() < 1e-9);
    }

    #[test]
    fn time_allocation_is_positive_and_finite() {
        let t = allocate_time(&[1.0, 2.0, 3.0], 1.0);
        assert_eq!(t.len(), 3);
        assert!(t.iter().all(|ti| *ti > 0.0));
        assert!(t.iter().all(|ti| ti.is_finite()));
    }

    #[test]
    fn warm_start_splices_prev_and_global_tail_on_official_timeline() {
        // 官方 case2 时间换算：组合时间轴（旧轨迹剩余 remaining 秒 + 全局
        // 轨迹段 glb_seg 秒）均匀取段——早段内点来自旧轨迹、晚段来自
        // guide_tail；段时长 = (remaining + glb_seg)/pieces。
        let start = Endpoint {
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(5.0, 0.0, 0.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        // 旧轨迹：0→5 直线单段 10s（elapsed = 4 → remaining = 6）
        let prev = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&[], &[10.0])
            .unwrap()
            .solve()
            .unwrap();
        // 全局轨迹段等时采样：x 从 5 推进到 9（归一化时间线性对应）
        let tail: Vec<Vector3<f64>> = (0..=4)
            .map(|k| Vector3::new(5.0 + f64::from(k), 0.0, 0.0))
            .collect();
        let config = InitConfig {
            pieces: 0, // 暖启动段数由距离决定，不使用
            max_velocity: 1.5,
            piece_length: 1.0,
        };
        let goal = Point3::new(9.0, 0.0, 0.0);
        let m =
            init_warm_start(&config, start, goal, &prev, 4.0, 4.0, &tail).expect("暖启动应成功");
        // 段数 = ceil(9/1) = 9；时长均匀 = (6+4)/9
        assert_eq!(m.pieces(), 9);
        let piece_dur = 10.0 / 9.0;
        for i in 0..m.pieces() {
            assert!(
                (m.piece_duration(i) - piece_dur).abs() < 1e-9,
                "段时长应均匀分布"
            );
        }
        let traj = m.solve().unwrap();
        assert!(
            (traj.eval(traj.duration()).position - goal.coords).norm() < 1e-6,
            "终点应为 goal"
        );
        // 内点拼接：早段（t < remaining）来自旧轨迹（x < 5），晚段来自
        // guide_tail（x > 5）
        let wps: Vec<Point3<f64>> = m.waypoints().collect();
        assert_eq!(wps.len(), 8);
        assert!(wps.first().unwrap().x < 5.0, "首内点来自旧轨迹剩余段");
        assert!(wps.last().unwrap().x > 5.0, "末内点来自全局轨迹段");
        let mid = traj.eval(traj.duration() / 2.0).position;
        assert!(mid.x > 0.0 && mid.x < 9.0, "中程点应落在拼接区间内");
    }
}
