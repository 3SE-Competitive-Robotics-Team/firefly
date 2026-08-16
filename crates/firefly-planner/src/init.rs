//! 前端初始化：A* 引导路径 → MINCO 参数 {q, T} 与边界条件。

use firefly_error::{Error, ErrorKind, Result};
use firefly_map::GridMap;
use firefly_search::Astar;
use firefly_trajectory::{Endpoint, Minco, MincoBuilder, SolverOrder};
use nalgebra::{Point3, Vector3};

pub struct InitConfig {
    pub pieces: usize,
    pub max_velocity: f64,
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
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "guide path too short",
        ));
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
}
