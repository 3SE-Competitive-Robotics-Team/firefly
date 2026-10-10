//! EGO 多项式冷启动、随机种子与连续轨迹暖启动。

use firefly_error::Result;
use firefly_trajectory::{Endpoint, Minco, MincoBuilder, SolverOrder, Trajectory};
use nalgebra::{Point3, Vector3};

/// 官方 `computeInitState` case 1：2s 单段多项式种子，按空间段数等时重采样。
///
/// # Errors
/// 段长或速度无效，或 MINCO 边界系统不可解。
pub fn init_polynomial(config: &InitConfig, start: Endpoint, end: Endpoint) -> Result<Minco> {
    if !config.piece_length.is_finite()
        || config.piece_length <= 0.0
        || !config.max_velocity.is_finite()
        || config.max_velocity <= 0.0
    {
        return Err(firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            "invalid polynomial initialization limits",
        ));
    }
    let seed = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&[], &[2.0])?
        .solve()?;
    let pieces =
        (((end.position - start.position).norm() / config.piece_length).round() as usize).max(2);
    let points: Vec<_> = (1..pieces)
        .map(|i| Point3::from(seed.eval(2.0 * i as f64 / pieces as f64).position))
        .collect();
    MincoBuilder::new(SolverOrder::MinimumJerk, start, end).build(
        &points,
        &vec![config.piece_length / config.max_velocity; pieces],
    )
}

pub struct InitConfig {
    pub max_velocity: f64,
    /// 每段路径长度（米，官方 `polyTraj_piece_length`；暖启动段数按它计算）。
    pub piece_length: f64,
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
/// # Errors
///
/// 旧轨迹已耗尽（`elapsed ≥ duration`）、`glb_seg < 0` 或组合轴退化时返回
/// `InvalidArgument`——调用方应降级冷启动（官方 case2 → case1 策略链）。
pub fn init_warm_start(
    config: &InitConfig,
    start: Endpoint,
    end: Endpoint,
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
    let dist = (end.position - start.position).norm();
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
    MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&waypoints, &durations)
        .map_err(|e| e.with_operation("planner::init:warm_start"))
}

/// 随机多项式初始化（官方 `computeInitState` `flag_randomPolyTraj`）：
/// 起终点中点沿水平/垂直正交方向随机偏移（幅度随连败次数增长），2 段 min-jerk
/// 作种子，再按 case1 后半段同构重采样为正式段数。起点终点过近时退化为直飞。
///
/// # Errors
///
/// MINCO 系统奇异。
pub fn init_random(
    config: &InitConfig,
    start: Endpoint,
    end: Endpoint,
    mid: Vector3<f64>,
) -> Result<Minco> {
    let dist = (end.position - start.position).norm();
    if dist < 1e-6 {
        return MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&[], &[1e-3])
            .map_err(|e| e.with_operation("planner::init:random_degenerate"));
    }
    // 种子：起 → 随机中点 → 终，2 段各 1s（官方 init_of_init_totaldur = 2.0）。
    let seed = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&[Point3::from(mid)], &[1.0, 1.0])
        .map_err(|e| e.with_operation("planner::init:random_seed"))?;
    let seed_traj = seed
        .solve()
        .map_err(|e| e.with_operation("planner::init:random_seed"))?;
    // 重采样：段数 = round(距离/piece_length)，下限 2；段时长均匀 ts
    //（官方 `piece_dur_vec = Constant(piece_nums, ts)`，ts = piece_length/max_vel）。
    let mut pieces = (dist / config.piece_length.max(1e-3)).round() as usize;
    pieces = pieces.max(2);
    // 采样步长 = 2.0/pieces（种子轨迹总时长 2s），段时长均匀 ts
    //（官方 `piece_dur_vec = Constant(piece_nums, ts)`，ts = piece_length/max_vel）。
    let ts = config.piece_length.max(1e-3) / config.max_velocity.max(1e-3);
    let step = 2.0 / pieces as f64;
    let mut waypoints = Vec::with_capacity(pieces.saturating_sub(1));
    let mut t = step;
    while t < 2.0 - step / 2.0 && waypoints.len() + 1 < pieces {
        waypoints.push(Point3::from(seed_traj.eval(t).position));
        t += step;
    }
    while waypoints.len() + 1 < pieces {
        waypoints.push(Point3::from(end.position));
    }
    let durations = vec![ts; pieces];
    MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&waypoints, &durations)
        .map_err(|e| e.with_operation("planner::init:random"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polynomial_seed_matches_analytic_quintic_and_uniform_times() {
        let config = InitConfig {
            max_velocity: 1.5,
            piece_length: 1.5,
        };
        let start = Endpoint {
            position: Vector3::new(0., 0., 1.),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(6., 0., 1.),
            ..start
        };
        let m = init_polynomial(&config, start, end).unwrap();
        assert_eq!(m.pieces(), 4);
        for (i, p) in m.waypoints().enumerate() {
            let u = (i + 1) as f64 / 4.;
            let expected = 6. * (10. * u.powi(3) - 15. * u.powi(4) + 6. * u.powi(5));
            assert!((p.x - expected).abs() < 1e-10);
        }
        for i in 0..4 {
            assert!((m.piece_duration(i) - 1.).abs() < 1e-12);
        }
        let trajectory = m.solve().unwrap();
        for i in 1..100 {
            let t = 4. * f64::from(i) / 100.;
            let u = t / 4.;
            let expected_v = 1.5 * (30. * u.powi(2) - 60. * u.powi(3) + 30. * u.powi(4));
            assert!((trajectory.eval(t).velocity.x - expected_v).abs() < 1e-9);
            let dt = 1e-4;
            let numeric =
                (trajectory.eval(t + dt).position - trajectory.eval(t - dt).position) / (2. * dt);
            assert!((numeric - trajectory.eval(t).velocity).norm() < 1e-7);
        }
        let short = init_polynomial(
            &config,
            start,
            Endpoint {
                position: start.position + Vector3::x() * 0.1,
                ..start
            },
        )
        .unwrap();
        assert_eq!(short.pieces(), 2);
        assert!((short.solve().unwrap().eval(short.duration()).position.x - 0.1).abs() < 1e-10);
    }

    #[test]
    fn random_init_matches_endpoints_and_mid() {
        let config = InitConfig {
            max_velocity: 1.5,
            piece_length: 1.5,
        };
        let start = Endpoint {
            position: Vector3::new(0.0, 0.0, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(6.0, 0.0, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let mid = Vector3::new(3.0, 1.0, 1.0);
        let m = init_random(&config, start, end, mid).expect("随机初值应可建");
        // 段数 = round(6/1.5) = 4
        assert_eq!(m.pieces(), 4);
        let traj = m.solve().expect("随机初值应可解");
        let s0 = traj.eval(0.0);
        let s1 = traj.eval(traj.duration());
        assert!(
            (s0.position - start.position).norm() < 1e-9,
            "起点应是 start"
        );
        assert!((s1.position - end.position).norm() < 1e-6, "终点应是 goal");
        // 内点约束经过随机中点附近（种子轨迹精确过点，重采样近似）
        let mut dmin = f64::INFINITY;
        let mut t = 0.0;
        while t <= traj.duration() {
            dmin = dmin.min((traj.eval(t).position - mid).norm());
            t += 0.05;
        }
        assert!(dmin < 0.5, "应经过随机中点附近，实际 {dmin:.3}");
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
            max_velocity: 1.5,
            piece_length: 1.0,
        };
        let goal = Point3::new(9.0, 0.0, 0.0);
        let m = init_warm_start(
            &config,
            start,
            Endpoint {
                position: goal.coords,
                velocity: Vector3::zeros(),
                acceleration: Vector3::zeros(),
            },
            &prev,
            4.0,
            4.0,
            &tail,
        )
        .expect("暖启动应成功");
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
