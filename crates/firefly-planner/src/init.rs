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

/// 上游 `computeInitState` case 2：旧局部轨迹剩余段与全局轨迹段拼接。
/// `global_start`、`global_end` 为全局多项式上的相对时刻（秒）。
/// 内点沿组合时间轴等时取样，直接计算相应多项式的位置。
///
/// # Errors
/// 旧局部轨迹已过期、全局起始时刻无效，或 MINCO 系统不可解。
#[allow(clippy::too_many_arguments)]
pub fn init_warm_start(
    config: &InitConfig,
    start: Endpoint,
    end: Endpoint,
    prev: &Trajectory,
    elapsed: f64,
    global: &Trajectory,
    global_start: f64,
    global_end: f64,
) -> Result<Minco> {
    if global_start < 0.0 {
        return Err(firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            "invalid previous global target time",
        ));
    }
    let remaining = prev.duration() - elapsed;
    if remaining < 0.0 {
        return Err(firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            "previous local trajectory has expired",
        ));
    }
    let t_to_lc_tgt = remaining + (global_end - global_start);
    let pieces = ((end.position - start.position).norm() / config.piece_length).ceil() as usize;
    let pieces = pieces.max(2);
    let piece_dur = t_to_lc_tgt / pieces as f64;
    let mut waypoints = Vec::with_capacity(pieces - 1);
    let mut t = piece_dur;
    for _ in 0..pieces - 1 {
        let pos = if t < remaining {
            prev.eval(elapsed + t).position
        } else {
            global.eval(t - remaining + global_start).position
        };
        waypoints.push(Point3::from(pos));
        t += piece_dur;
    }
    MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&waypoints, &vec![piece_dur; pieces])
        .map_err(|e| e.with_operation("planner::init:warm_start"))
}

/// 随机多项式初始化（官方 `computeInitState` `flag_randomPolyTraj`）：
/// 起终点中点沿水平/垂直正交方向随机偏移（幅度随连败次数增长），2 段 min-jerk
/// 作种子，再按 case1 后半段同构重采样为正式段数。
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
    // 种子：起 → 随机中点 → 终，2 段各 1s（官方 init_of_init_totaldur = 2.0）。
    let seed = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&[Point3::from(mid)], &[1.0, 1.0])
        .map_err(|e| e.with_operation("planner::init:random_seed"))?;
    let seed_traj = seed
        .solve()
        .map_err(|e| e.with_operation("planner::init:random_seed"))?;
    // 重采样：段数 = round(距离/piece_length)，下限 2；段时长均匀 ts
    //（官方 `piece_dur_vec = Constant(piece_nums, ts)`，ts = piece_length/max_vel）。
    let mut pieces = (dist / config.piece_length).round() as usize;
    pieces = pieces.max(2);
    // 采样步长 = 2.0/pieces（种子轨迹总时长 2s），段时长均匀 ts
    //（官方 `piece_dur_vec = Constant(piece_nums, ts)`，ts = piece_length/max_vel）。
    let ts = config.piece_length / config.max_velocity;
    let step = 2.0 / pieces as f64;
    let mut waypoints = Vec::with_capacity(pieces.saturating_sub(1));
    let mut t = step;
    while t < 2.0 - step / 2.0 {
        waypoints.push(Point3::from(seed_traj.eval(t).position));
        t += step;
    }
    if waypoints.len() != pieces - 1 {
        return Err(firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            "random initialization waypoint count mismatch",
        ));
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
    fn random_init_uses_two_piece_seed_for_coincident_or_nearby_endpoints() {
        let config = InitConfig {
            max_velocity: 1.5,
            piece_length: 1.5,
        };
        let start = Endpoint {
            position: Vector3::new(1., 1., 1.),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        for distance in [0., 1e-8] {
            let end = Endpoint {
                position: start.position + Vector3::x() * distance,
                ..start
            };
            let m = init_random(&config, start, end, (start.position + end.position) / 2.).unwrap();
            assert_eq!(m.pieces(), 2);
            assert_eq!(m.piece_duration(0), 1.);
            assert_eq!(m.piece_duration(1), 1.);
        }
    }

    fn warm_start_fixture() -> (Trajectory, Trajectory) {
        // 旧局部轨迹 p(t)=(t/2,0,0)，全局轨迹 p(t)=(5+t³,t²,0)。
        let local_start = Endpoint {
            position: Vector3::zeros(),
            velocity: Vector3::new(0.5, 0., 0.),
            acceleration: Vector3::zeros(),
        };
        let local_end = Endpoint {
            position: Vector3::new(5., 0., 0.),
            ..local_start
        };
        let prev = MincoBuilder::new(SolverOrder::MinimumJerk, local_start, local_end)
            .build(&[], &[10.])
            .unwrap()
            .solve()
            .unwrap();
        let global =
            MincoBuilder::new(SolverOrder::MinimumJerk, global_state(0.), global_state(4.))
                .build(&[], &[4.])
                .unwrap()
                .solve()
                .unwrap();
        (prev, global)
    }

    fn global_state(t: f64) -> Endpoint {
        Endpoint {
            position: Vector3::new(5. + t.powi(3), t.powi(2), 0.),
            velocity: Vector3::new(3. * t.powi(2), 2. * t, 0.),
            acceleration: Vector3::new(6. * t, 2., 0.),
        }
    }

    #[test]
    fn warm_start_evaluates_global_polynomial_on_upstream_timeline() {
        let (prev, global) = warm_start_fixture();
        let config = InitConfig {
            max_velocity: 1.5,
            piece_length: 1.0,
        };
        let start = Endpoint {
            position: Vector3::new(2., 0., 0.),
            velocity: Vector3::new(0.5, 0., 0.),
            acceleration: Vector3::zeros(),
        };
        let end = global_state(3.2);
        let m = init_warm_start(&config, start, end, &prev, 4., &global, 0.7, 3.2).unwrap();
        let pieces = (end.position - start.position).norm().ceil() as usize;
        assert_eq!(m.pieces(), pieces);
        assert!(pieces > 24);
        let piece_duration = 8.5 / pieces as f64;
        for i in 0..pieces {
            assert!((m.piece_duration(i) - piece_duration).abs() < 1e-12);
        }
        let mut local_points = 0;
        let mut global_points = 0;
        for (i, point) in m.waypoints().enumerate() {
            let t = (i + 1) as f64 * piece_duration;
            let expected = if t < 6. {
                local_points += 1;
                Vector3::new(2. + 0.5 * t, 0., 0.)
            } else {
                global_points += 1;
                global_state(t - 6. + 0.7).position
            };
            assert!((point.coords - expected).norm() < 1e-9, "waypoint {i}");
        }
        assert!(local_points > 0 && global_points > 0);
    }

    #[test]
    fn warm_start_accepts_short_or_zero_local_remainder_and_rejects_expiration() {
        let (prev, global) = warm_start_fixture();
        let config = InitConfig {
            max_velocity: 1.5,
            piece_length: 1.5,
        };
        for elapsed in [9.98, 10.] {
            let sample = prev.eval(elapsed);
            let start = Endpoint {
                position: sample.position,
                velocity: sample.velocity,
                acceleration: sample.acceleration,
            };
            let m = init_warm_start(
                &config,
                start,
                global_state(3.2),
                &prev,
                elapsed,
                &global,
                0.7,
                3.2,
            )
            .unwrap();
            let duration = (10. - elapsed + 2.5) / m.pieces() as f64;
            for (i, point) in m.waypoints().enumerate() {
                let global_t = (i + 1) as f64 * duration - (10. - elapsed) + 0.7;
                assert!((point.coords - global_state(global_t).position).norm() < 1e-9);
            }
        }
        assert!(
            init_warm_start(
                &config,
                global_state(0.),
                global_state(3.2),
                &prev,
                10.01,
                &global,
                0.7,
                3.2
            )
            .is_err()
        );
    }
}
