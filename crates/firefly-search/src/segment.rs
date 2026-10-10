//! 线段与膨胀体素的闭集相交检查；沿体素面分段，覆盖擦角与沿面运动。

use firefly_map::GridMap;
use nalgebra::Vector3;

/// 两端和整条线段都在地图内、未触及膨胀体素或虚拟墙时返回 true。
/// 同时穿越多个体素面时检查全部相邻格；地图外和非有限输入拒绝。
#[must_use]
pub fn segment_is_clear(map: &GridMap, a: Vector3<f64>, b: Vector3<f64>) -> bool {
    if a.iter().chain(b.iter()).any(|v| !v.is_finite())
        || map.index_of(a).is_none()
        || map.index_of(b).is_none()
        || map.is_occupied_inflated(a)
        || map.is_occupied_inflated(b)
    {
        return false;
    }
    let from = (a - map.origin()) / map.resolution();
    let to = (b - map.origin()) / map.resolution();
    let delta = to - from;
    let mut events = vec![0., 1.];
    for axis in 0..3 {
        if delta[axis] == 0. {
            continue;
        }
        let lo = from[axis].min(to[axis]).ceil() as i64;
        let hi = from[axis].max(to[axis]).floor() as i64;
        for plane in lo..=hi {
            let t = (plane as f64 - from[axis]) / delta[axis];
            if t > 0. && t < 1. {
                events.push(t);
            }
        }
    }
    events.sort_unstable_by(f64::total_cmp);
    events.dedup();
    events
        .iter()
        .all(|&t| touched_cells_free(map, from + delta * t))
        && events
            .windows(2)
            .all(|w| touched_cells_free(map, from + delta * w[0].midpoint(w[1])))
}

fn touched_cells_free(map: &GridMap, p: Vector3<f64>) -> bool {
    let mut lo = [0_i64; 3];
    let mut hi = [0_i64; 3];
    for axis in 0..3 {
        let nearest = p[axis].round();
        let tolerance = 64. * f64::EPSILON * (1. + p[axis].abs());
        if (p[axis] - nearest).abs() <= tolerance {
            lo[axis] = nearest as i64 - 1;
            hi[axis] = nearest as i64;
        } else {
            lo[axis] = p[axis].floor() as i64;
            hi[axis] = lo[axis];
        }
    }
    let dims = map.dims();
    for x in lo[0]..=hi[0] {
        for y in lo[1]..=hi[1] {
            for z in lo[2]..=hi[2] {
                let idx = [x, y, z];
                if (0..3).all(|axis| idx[axis] >= 0 && idx[axis] < dims[axis] as i64)
                    && map.is_inflated([x as usize, y as usize, z as usize])
                {
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

    #[test]
    fn voxel_traversal_matches_independent_segment_box_intersections() {
        let mut map = GridMapBuilder::new(1., [10, 10, 10])
            .with_obstacles_inflation(1.)
            .build()
            .unwrap();
        map.set_state([5, 5, 5], VoxelState::Occupied);
        map.inflate_obstacles();
        let mut seed = 73_u64;
        for _ in 0..100 {
            let mut point = || {
                Vector3::from_fn(|_, _| {
                    seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
                    0.1 + ((seed >> 32) % 9800) as f64 / 1000.
                })
            };
            let a = point();
            let b = point();
            let mut expected = true;
            for x in 4..=6 {
                for y in 4..=6 {
                    for z in 4..=6 {
                        let lo = Vector3::new(f64::from(x), f64::from(y), f64::from(z));
                        let hi = lo.add_scalar(1.);
                        // 独立 slab 公式：求三轴闭区间交集，不做体素步进。
                        let (mut enter, mut leave) = (0_f64, 1_f64);
                        for axis in 0..3 {
                            let d = b[axis] - a[axis];
                            if d == 0. {
                                if a[axis] < lo[axis] || a[axis] > hi[axis] {
                                    leave = -1.;
                                }
                            } else {
                                let t0 = (lo[axis] - a[axis]) / d;
                                let t1 = (hi[axis] - a[axis]) / d;
                                enter = enter.max(t0.min(t1));
                                leave = leave.min(t0.max(t1));
                            }
                        }
                        if enter <= leave {
                            expected = false;
                        }
                    }
                }
            }
            assert_eq!(segment_is_clear(&map, a, b), expected, "{a:?} -> {b:?}");
            assert_eq!(segment_is_clear(&map, b, a), expected);
        }
    }

    #[test]
    fn rejects_subsample_inflated_corner_and_preserves_near_miss() {
        let mut map = GridMapBuilder::new(1., [10, 10, 10])
            .with_obstacles_inflation(1.)
            .build()
            .unwrap();
        map.set_state([6, 6, 5], VoxelState::Occupied);
        map.inflate_obstacles();
        let a = Vector3::new(4.589, 5.412, 5.5);
        let b = Vector3::new(5.589, 4.412, 5.5);
        // 膨胀格 [5,6]² 的解析穿越区间为 t ∈ [0.411, 0.412]。
        assert!(map.is_occupied_inflated(a + (b - a) * 0.4115));
        assert!(!segment_is_clear(&map, a, b));
        assert!(!segment_is_clear(&map, b, a));
        assert!(segment_is_clear(
            &map,
            a - Vector3::y() * 0.002,
            b - Vector3::y() * 0.002
        ));
    }

    #[test]
    fn checks_endpoints_grid_faces_corners_and_bounds() {
        let mut map = GridMapBuilder::new(1., [10, 10, 10])
            .with_obstacles_inflation(0.)
            .build()
            .unwrap();
        map.set_state([5, 5, 5], VoxelState::Occupied);
        map.inflate_obstacles();
        for (a, b) in [
            ([5.5, 5.5, 5.5], [5.5, 5.5, 5.5]),
            ([4.5, 5., 5.5], [6.5, 5., 5.5]),
            ([4.5, 5.5, 5.5], [5.5, 4.5, 5.5]),
            ([-0.1, 0.5, 0.5], [0.5, 0.5, 0.5]),
            ([f64::NAN, 0.5, 0.5], [0.5, 0.5, 0.5]),
        ] {
            assert!(!segment_is_clear(&map, Vector3::from(a), Vector3::from(b)));
        }
        assert!(segment_is_clear(
            &map,
            Vector3::repeat(0.5),
            Vector3::new(9.5, 0.5, 0.5)
        ));
        map.set_virtual_wall(firefly_map::VirtualWall {
            ground: 0.,
            ceil: 3.,
        });
        assert!(!segment_is_clear(
            &map,
            Vector3::repeat(0.5),
            Vector3::new(0.5, 0.5, 3.)
        ));
    }
}
