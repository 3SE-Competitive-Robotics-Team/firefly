//! 独立轨迹验收：递归导数隔离极值，稠密几何采样，不调用规划器后检查。
use firefly_map::GridMap;
use firefly_planner::{PlannerConfig, State};
use firefly_trajectory::{Endpoint, Trajectory};

fn value(coefficients: &[f64], t: f64) -> f64 {
    coefficients.iter().rev().fold(0., |v, c| v * t + c)
}

fn derivative(c: &[f64]) -> Vec<f64> {
    c.iter()
        .enumerate()
        .skip(1)
        .map(|(i, c)| *c * i as f64)
        .collect()
}

/// 在导数根之间函数单调；检查分隔点可保留偶重根。
fn roots(c: &[f64]) -> Vec<f64> {
    let scale = c.iter().fold(0f64, |a, b| a.max(b.abs()));
    if scale == 0. {
        return vec![];
    }
    let mut c: Vec<_> = c.iter().map(|x| x / scale).collect();
    while c.len() > 1 && c.last().unwrap().abs() < 1e-14 {
        c.pop();
    }
    if c.len() <= 1 {
        return vec![];
    }
    let mut boundaries = vec![0.];
    boundaries.extend(roots(&derivative(&c)));
    boundaries.push(1.);
    boundaries.sort_by(f64::total_cmp);
    let mut result = Vec::new();
    for &x in &boundaries {
        if value(&c, x).abs() < 1e-12 {
            result.push(x);
        }
    }
    for w in boundaries.windows(2) {
        let (mut left, mut right) = (w[0], w[1]);
        if value(&c, left) * value(&c, right) >= 0. {
            continue;
        }
        let sign = value(&c, left).is_sign_positive();
        for _ in 0..60 {
            let middle = f64::midpoint(left, right);
            if value(&c, middle).is_sign_positive() == sign {
                left = middle;
            } else {
                right = middle;
            }
        }
        result.push(f64::midpoint(left, right));
    }
    result.sort_by(f64::total_cmp);
    result.dedup_by(|a, b| (*a - *b).abs() < 1e-10);
    result
}

pub fn assert_trajectory_contract(
    t: &Trajectory,
    map: &GridMap,
    config: &PlannerConfig,
    start: State,
    end: Endpoint,
) {
    assert!(t.coefficients().iter().all(|x| x.is_finite()));
    assert!(t.durations().iter().all(|x| x.is_finite() && *x > 0.));
    for (time, p, v, a) in [
        (
            0.,
            start.position.coords,
            start.velocity,
            start.acceleration,
        ),
        (t.duration(), end.position, end.velocity, end.acceleration),
    ] {
        let s = t.eval(time);
        assert!(
            (s.position - p).norm() < 1e-6,
            "position boundary at {time}"
        );
        assert!(
            (s.velocity - v).norm() < 1e-6,
            "velocity boundary at {time}"
        );
        assert!(
            (s.acceleration - a).norm() < 1e-6,
            "acceleration boundary at {time}"
        );
    }
    for i in 0..t.pieces() {
        let duration = t.durations()[i];
        for (order, limit) in [
            (1, config.max_velocity),
            (2, config.max_acceleration),
            (3, config.max_jerk),
        ] {
            let mut squared = vec![0.; 2 * (6 - order) - 1];
            for axis in 0..3 {
                let mut c: Vec<_> = (0..6)
                    .map(|k| t.coefficients()[(6 * i + k, axis)])
                    .collect();
                for _ in 0..order {
                    c = derivative(&c);
                }
                for (k, x) in c.iter_mut().enumerate() {
                    *x *= duration.powi(k as i32);
                }
                for (j, x) in c.iter().enumerate() {
                    for (k, y) in c.iter().enumerate() {
                        squared[j + k] += x * y;
                    }
                }
            }
            let mut candidates = roots(&derivative(&squared));
            candidates.extend([0., 1.]);
            for u in candidates {
                let s = t.eval_piece(i, u * duration);
                let norm = match order {
                    1 => s.velocity.norm(),
                    2 => s.acceleration.norm(),
                    _ => s.jerk.norm(),
                };
                assert!(
                    norm <= limit * (1. + 1e-6),
                    "piece={i} order={order} u={u}: {norm} > {limit}"
                );
            }
        }
        if i + 1 < t.pieces() {
            let a = t.eval_piece(i, duration);
            let b = t.eval_piece(i + 1, 0.);
            assert!((a.position - b.position).norm() < 1e-6);
            assert!((a.velocity - b.velocity).norm() < 1e-6);
            assert!((a.acceleration - b.acceleration).norm() < 1e-6);
        }
    }
    // 每步位移上界为体素边长的 1/32；此项是数值几何回归，非连续无碰撞证明。
    let samples = (t.duration() * config.max_velocity / (map.resolution() / 32.)).ceil() as usize;
    for k in 0..=samples {
        let p = t.eval(t.duration() * k as f64 / samples as f64).position;
        assert!(map.index_of(p).is_some(), "trajectory left map: {p:?}");
        assert!(!map.is_occupied_inflated(p), "collision at {p:?}");
    }
}

#[test]
fn independent_root_oracle_handles_simple_and_repeated_roots() {
    let r = roots(&[-0.02, 0.24, -0.9, 1.]); // (x-0.2)^2 (x-0.5)
    assert_eq!(r.len(), 2);
    assert!((r[0] - 0.2).abs() < 1e-8 && (r[1] - 0.5).abs() < 1e-8);
}
