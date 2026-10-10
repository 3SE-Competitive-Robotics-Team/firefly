//! 最终静态碰撞验收：Bernstein 凸包覆盖整段曲线，de Casteljau 二分收紧包围盒。
//! 包围盒涉及的全部体素自由才接受；深度/工作量耗尽或数值非法均拒绝。
//! 地图外与虚拟墙均拒绝；浮点包围盒留相对余量，不是区间算术证明。

use firefly_map::GridMap;
use firefly_trajectory::Trajectory;
use nalgebra::Vector3;

#[fastrace::trace]
pub(crate) fn trajectory_is_free(map: &GridMap, trajectory: &Trajectory) -> bool {
    if trajectory.pieces() == 0 {
        return false;
    }
    let width = trajectory.coefficients().nrows() / trajectory.pieces();
    let mut budget = 200_000usize;
    for (piece, &duration) in trajectory.durations().iter().enumerate() {
        if !duration.is_finite() || duration <= 0.0 {
            return false;
        }
        // p(Tu) = Σ a_k u^k；b_i = Σ_{k≤i} a_k C(i,k)/C(n,k)。
        let mut controls = Vec::with_capacity(width);
        for i in 0..width {
            let mut control = Vector3::zeros();
            let mut ratio = 1.0;
            let mut time_power = 1.0;
            for k in 0..=i {
                for axis in 0..3 {
                    control[axis] +=
                        trajectory.coefficients()[(piece * width + k, axis)] * time_power * ratio;
                }
                time_power *= duration;
                if k < i {
                    ratio *= (i - k) as f64 / (width - 1 - k) as f64;
                }
            }
            controls.push(control);
        }
        if !hull_is_free(map, &controls, 0, &mut budget) {
            return false;
        }
    }
    true
}

fn hull_is_free(map: &GridMap, controls: &[Vector3<f64>], depth: u32, budget: &mut usize) -> bool {
    if *budget == 0 || controls.iter().any(|p| p.iter().any(|v| !v.is_finite())) {
        return false;
    }
    *budget -= 1;
    let last = controls.len() - 1;
    if map.index_of(controls[0]).is_none()
        || map.index_of(controls[last]).is_none()
        || map.is_occupied_inflated(controls[0])
        || map.is_occupied_inflated(controls[last])
    {
        return false;
    }
    let mut lo = controls[0];
    let mut hi = lo;
    for point in controls {
        lo = lo.inf(point);
        hi = hi.sup(point);
    }
    let margin = 1e-10 * (1.0 + lo.abs().max().max(hi.abs().max()));
    lo.add_scalar_mut(-margin);
    hi.add_scalar_mut(margin);
    if (hi - lo).max() <= 2.0 * map.resolution() && box_is_free(map, lo, hi) {
        return true;
    }
    if depth >= 24 {
        return false;
    }
    let mut work = controls.to_vec();
    let mut left = vec![controls[0]];
    let mut right = vec![controls[last]; controls.len()];
    for level in 1..controls.len() {
        for j in 0..controls.len() - level {
            work[j] = (work[j] + work[j + 1]) * 0.5;
        }
        left.push(work[0]);
        right[last - level] = work[last - level];
    }
    hull_is_free(map, &left, depth + 1, budget) && hull_is_free(map, &right, depth + 1, budget)
}

fn box_is_free(map: &GridMap, lo: Vector3<f64>, hi: Vector3<f64>) -> bool {
    if map.index_of(lo).is_none() || map.index_of(hi).is_none() {
        return false;
    }
    if map
        .virtual_wall()
        .is_some_and(|w| lo.z <= w.ground || hi.z >= w.ceil)
    {
        return false;
    }
    let low = map.pos_to_global_index(lo);
    let high = map.pos_to_global_index(hi);
    let origin_index = map.global_index([0, 0, 0]);
    let origin = map.origin();
    for x in low[0]..=high[0] {
        for y in low[1]..=high[1] {
            for z in low[2]..=high[2] {
                let index = [x, y, z];
                let point = Vector3::from_fn(|axis, _| {
                    origin[axis]
                        + ((index[axis] - origin_index[axis]) as f64 + 0.5) * map.resolution()
                });
                if map.is_occupied_inflated(point) {
                    return false;
                }
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_map::{GridMapBuilder, VoxelState};
    use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};

    #[test]
    fn rejects_curve_outside_map_even_with_both_endpoints_inside() {
        let map = GridMapBuilder::new(1., [10, 10, 10]).build().unwrap();
        // z(t)=0.5−4t+4t²，端点 0.5m，中点 −0.5m。
        let curve = polynomial(&[1.5, 1.], &[1.5], &[0.5, -4., 4.]);
        assert!(map.index_of(curve.eval(0.).position).is_some());
        assert!(map.index_of(curve.eval(1.).position).is_some());
        assert!(!trajectory_is_free(&map, &curve));
    }

    fn polynomial(x: &[f64], y: &[f64], z: &[f64]) -> Trajectory {
        let endpoint = Endpoint {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let mut trajectory = MincoBuilder::new(SolverOrder::MinimumJerk, endpoint, endpoint)
            .build(&[], &[1.0])
            .unwrap()
            .solve()
            .unwrap();
        trajectory.coefficients_mut().fill(0.0);
        for (axis, coefficients) in [x, y, z].iter().enumerate() {
            for (k, &value) in coefficients.iter().enumerate() {
                trajectory.coefficients_mut()[(k, axis)] = value;
            }
        }
        trajectory
    }

    #[test]
    fn rejects_subsample_corner_crossing_but_accepts_nearby_free_line() {
        let mut map = GridMapBuilder::new(1.0, [10, 10, 10])
            .with_obstacles_inflation(0.0)
            .build()
            .unwrap();
        map.set_state([5, 5, 5], VoxelState::Occupied);
        map.inflate_obstacles();
        // 两条直线距体素角只有毫米量级；碰撞线仅在 t ∈ [0.411,0.412] 入格。
        let hit = polynomial(&[4.589, 1.0], &[5.412, -1.0], &[5.5]);
        let free = polynomial(&[4.589, 1.0], &[5.410, -1.0], &[5.5]);
        assert!(map.is_occupied_inflated(hit.eval(0.4115).position));
        assert!(!trajectory_is_free(&map, &hit));
        assert!(trajectory_is_free(&map, &free));
    }

    #[test]
    fn checks_curvature_endpoints_and_virtual_walls() {
        let mut map = GridMapBuilder::new(1.0, [10, 10, 10])
            .with_obstacles_inflation(0.0)
            .build()
            .unwrap();
        map.set_state([5, 5, 5], VoxelState::Occupied);
        map.inflate_obstacles();
        assert!(!trajectory_is_free(
            &map,
            &polynomial(&[5.5], &[4.5, 4.0, -4.0], &[5.5])
        ));
        assert!(!trajectory_is_free(
            &map,
            &polynomial(&[4.5, 0.5], &[5.5], &[5.5])
        ));
        assert!(trajectory_is_free(
            &map,
            &polynomial(&[2.0], &[2.0], &[2.0])
        ));
        let walls = GridMapBuilder::new(1.0, [10, 10, 10])
            .with_virtual_wall(0.0, 3.0)
            .build()
            .unwrap();
        assert!(!trajectory_is_free(
            &walls,
            &polynomial(&[2.0], &[2.0], &[2.0, 8.0, -8.0])
        ));
    }
}
