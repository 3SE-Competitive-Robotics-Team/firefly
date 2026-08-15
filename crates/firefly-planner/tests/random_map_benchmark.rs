//! 随机地图 benchmark（论文 v1 Sec. VI-B 方法）：
//! 随机障碍场景 ×N，统计成功率、规划耗时（min/avg/max）、轨迹能量、峰值速度。

use firefly_map::{GridMapBuilder, VoxelState};
use firefly_planner::{Planner, PlannerConfig, State};
use nalgebra::{Point3, Vector3};
use std::time::Instant;

/// 简单确定性 LCG（避免引入 rand 依赖，种子可复现）。
struct Lcg(u64);

impl Lcg {
    fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }

    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, min: f64, max: f64) -> f64 {
        min + (max - min) * self.next()
    }
}

struct RandomMap {
    map: firefly_map::GridMap,
    start: Point3<f64>,
    goal: Point3<f64>,
}

/// 点周围 radius 内无占据体素。
#[allow(clippy::many_single_char_names)]
fn free_around(map: &firefly_map::GridMap, p: Vector3<f64>, radius: f64) -> bool {
    let step = map.resolution();
    let n = (radius / step).ceil() as i32;
    let Some(center) = map.index_of(p) else {
        return false;
    };
    let [dx, dy, dz] = map.dims();
    for i in -n..=n {
        for j in -n..=n {
            for k in -n..=n {
                let x = center[0] as i32 + i;
                let y = center[1] as i32 + j;
                let z = center[2] as i32 + k;
                if x < 0 || y < 0 || z < 0 || x >= dx as i32 || y >= dy as i32 || z >= dz as i32 {
                    continue;
                }
                if map.state([x as usize, y as usize, z as usize]) == VoxelState::Occupied {
                    return false;
                }
            }
        }
    }
    true
}

/// 随机地图：地图内随机圆柱障碍 + 随机起终点（保证可达）。
fn random_map(rng: &mut Lcg) -> RandomMap {
    const DIM_X: usize = 20;
    const DIM_Y: usize = 20;
    const DIM_Z: usize = 10;
    let mut map = GridMapBuilder::new(0.5, [DIM_X, DIM_Y, DIM_Z])
        .build()
        .unwrap();

    // 5~8 个圆柱障碍（半径 0.5~1.5m，高度 1~2.5m），限制在中部区域
    let n_obstacles = 5 + (rng.next() * 4.0) as usize;
    for _ in 0..n_obstacles {
        let cx = rng.range(3.0, 6.5);
        let cy = rng.range(2.0, (DIM_Y as f64) * 0.5 - 2.0);
        let radius = rng.range(0.5, 1.5);
        let height = rng.range(1.0, 2.5);
        let r_voxels = (radius / 0.5).ceil() as usize;
        for ix in 0..DIM_X {
            for iy in 0..DIM_Y {
                let x = (ix as f64 + 0.5) * 0.5;
                let y = (iy as f64 + 0.5) * 0.5;
                let d2 = (x - cx).powi(2) + (y - cy).powi(2);
                if d2 <= (r_voxels as f64 * 0.5).powi(2) {
                    let nz = (height / 0.5).ceil() as usize;
                    for iz in 0..nz.min(DIM_Z) {
                        map.set_state([ix, iy, iz], VoxelState::Occupied);
                    }
                }
            }
        }
    }

    // 随机起终点（两端区域，周围 0.75m 无障碍）
    let mut start = Point3::new(0.5, 0.5, 0.5);
    let mut goal = Point3::new(9.5, 0.5, 0.5);
    for _ in 0..32 {
        start = Point3::new(
            rng.range(0.5, 2.5),
            rng.range(0.5, 4.0),
            rng.range(0.5, 1.5),
        );
        goal = Point3::new(
            rng.range(7.5, 9.5),
            rng.range(0.5, 4.0),
            rng.range(0.5, 1.5),
        );
        if free_around(&map, start.coords, 0.75) && free_around(&map, goal.coords, 0.75) {
            break;
        }
    }
    RandomMap { map, start, goal }
}

#[test]
#[ignore = "benchmark：手动运行（cargo test -- --ignored），debug 模式约 90s"]
fn random_map_benchmark() {
    const SCENARIOS: usize = 30;
    firefly_observability::init();
    let mut rng = Lcg::new(20_260_815);

    let mut success = 0usize;
    let mut times_ms: Vec<f64> = Vec::new();
    let _ = &times_ms;
    let mut energies: Vec<f64> = Vec::new();
    let mut vmax_vals: Vec<f64> = Vec::new();
    let mut iterations: Vec<usize> = Vec::new();

    for s in 0..SCENARIOS {
        let rm = random_map(&mut rng);
        let config = PlannerConfig {
            planning_distance: 5.0,
            ..PlannerConfig::default()
        };
        let mut planner = Planner::new(config, rm.map);
        let start = State {
            position: rm.start,
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };

        let t0 = Instant::now();
        match planner.plan(start, rm.goal) {
            Ok(result) => {
                success += 1;
                let elapsed = t0.elapsed().as_secs_f64() * 1000.0;
                if elapsed > 20.0 {
                    log::warn!(
                        "scenario {s}: slow plan {elapsed:.0}ms, {} rebound rounds",
                        result.iterations
                    );
                }
                times_ms.push(elapsed);
                iterations.push(result.iterations);
                let traj = &result.trajectory;
                energies.push(
                    firefly_cost::Cost::new()
                        .add(1.0, firefly_cost::SmoothnessPenalty)
                        .evaluate(traj),
                );
                let mut vmax: f64 = 0.0;
                for k in 0..200 {
                    let t = traj.duration() * f64::from(k) / 200.0;
                    vmax = vmax.max(traj.eval(t).velocity.norm());
                }
                vmax_vals.push(vmax);
            }
            Err(e) => {
                log::warn!("scenario {s} failed: {e}");
            }
        }
    }

    let rate = success as f64 / SCENARIOS as f64;
    let avg = |v: &[f64]| v.iter().sum::<f64>() / v.len().max(1) as f64;
    let min = |v: &[f64]| v.iter().copied().fold(f64::MAX, f64::min);
    let max = |v: &[f64]| v.iter().copied().fold(0.0, f64::max);
    let median = |v: &[f64]| {
        let mut s = v.to_vec();
        s.sort_by(f64::total_cmp);
        s[s.len() / 2]
    };
    log::info!("benchmark: success={success}/{SCENARIOS} (rate={rate:.2})");
    log::info!(
        "plan time ms: min={:.1} median={:.1} avg={:.1} max={:.1}",
        min(&times_ms),
        median(&times_ms),
        avg(&times_ms),
        max(&times_ms)
    );
    log::info!(
        "energy: avg={:.2} | vmax: avg={:.2} | rebound iters: avg={:.1}",
        avg(&energies),
        avg(&vmax_vals),
        avg(&iterations.iter().map(|v| *v as f64).collect::<Vec<_>>())
    );

    assert!(rate >= 0.8, "成功率 {rate:.2} 过低（目标 >=0.8）");
    assert!(
        median(&times_ms) < 1000.0,
        "中位规划耗时 {:.0}ms 过高",
        median(&times_ms)
    );
    firefly_observability::flush();
}
