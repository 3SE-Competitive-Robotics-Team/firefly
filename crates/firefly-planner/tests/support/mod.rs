//! 优化输出的数值契约：有限系数、正时长、端点 PVA 与分段 C2 连续性。
use firefly_planner::State;
use firefly_trajectory::{Endpoint, Trajectory};

pub fn assert_trajectory_contract(t: &Trajectory, start: State, end: Endpoint) {
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
    for i in 0..t.pieces().saturating_sub(1) {
        let a = t.eval_piece(i, t.durations()[i]);
        let b = t.eval_piece(i + 1, 0.);
        assert!((a.position - b.position).norm() < 1e-6);
        assert!((a.velocity - b.velocity).norm() < 1e-6);
        assert!((a.acceleration - b.acceleration).norm() < 1e-6);
    }
}
