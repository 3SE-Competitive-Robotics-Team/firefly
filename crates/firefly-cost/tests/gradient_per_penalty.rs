//! 逐项验证：每个 penalty 的 dF/dc 与系数空间中心差分对比。
//! 用于定位累加器错误，也是长期回归测试。

use firefly_cost::{Accumulator, FeasibilityPenalty, ObstaclePenalty, Penalty, SmoothnessPenalty};
use firefly_map::Plane;
use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder, Trajectory};
use nalgebra::{Point3, Vector3};

fn build_minco() -> firefly_trajectory::Minco {
    let start = Endpoint {
        position: Vector3::new(0.0, 0.0, 0.0),
        velocity: Vector3::new(0.5, 0.0, 0.0),
        acceleration: Vector3::zeros(),
    };
    let end = Endpoint {
        position: Vector3::new(4.0, 2.0, 1.0),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(
            &[Point3::new(1.5, 0.8, 0.3), Point3::new(2.8, 1.5, 0.6)],
            &[2.0, 2.0, 2.0],
        )
        .unwrap()
}

fn traj() -> Trajectory {
    build_minco().solve().unwrap()
}

fn check_gradient_c(name: &str, traj: &Trajectory, penalty: &dyn Penalty) {
    let mut acc = Accumulator::new(traj.pieces());
    penalty.accumulate(traj, 1.0, &mut acc);
    let h = 1e-7;
    for r in 0..acc.d_f_d_c.nrows() {
        for c in 0..acc.d_f_d_c.ncols() {
            let mut t_plus = traj.clone();
            let mut t_minus = traj.clone();
            *t_plus.coefficients_mut().get_mut((r, c)).unwrap() += h;
            *t_minus.coefficients_mut().get_mut((r, c)).unwrap() -= h;
            let numeric = (penalty.evaluate(&t_plus) - penalty.evaluate(&t_minus)) / (2.0 * h);
            let analytic = acc.d_f_d_c[(r, c)];
            assert!(
                (numeric - analytic).abs() <= 1e-4 * (1.0 + analytic.abs()),
                "{name}: dF/dc[{r}][{c}] analytic={analytic} numeric={numeric}"
            );
        }
    }
}

#[test]
fn per_penalty_gradients_match_numerical() {
    let traj = traj();
    check_gradient_c("smoothness", &traj, &SmoothnessPenalty);
    check_gradient_c("feasibility", &traj, &FeasibilityPenalty::new(1.5, 6.0));
    let planes = vec![
        Plane::new(Vector3::new(0.2, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
        Plane::new(Vector3::new(3.8, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0)),
    ];
    let per_point: Vec<Vec<Plane>> = (0..3 * 6).map(|_| planes.clone()).collect();
    check_gradient_c(
        "obstacle",
        &traj,
        &ObstaclePenalty::new(0.1, 0.5, 5000.0, 5, per_point),
    );
}
