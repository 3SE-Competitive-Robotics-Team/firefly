//! 障碍检测与 {s, v} 平面生成。
//!
//! 论文（补充材料 S6, Eq. S20–S23）：约束点 p̊ᵢ,ⱼ 若发现新障碍，
//! 生成固定安全点 s 与安全向量 `v，d_o` = (p̊ − s)ᵀv。
//! s 取无碰撞引导路径 Γ 上最近点，v 指向 p̊ → Γ 方向（v1 论文 Fig. 3b）。

use firefly_map::{GridMap, Plane};
use firefly_trajectory::{Sample, Trajectory};
use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct Hit {
    pub point_index: usize,
    pub sample: Sample,
    pub guide_point: Vector3<f64>,
}

pub struct ObstacleScanner<'a> {
    map: &'a GridMap,
    samples_per_piece: usize,
}

impl<'a> ObstacleScanner<'a> {
    pub fn new(map: &'a GridMap) -> Self {
        Self {
            map,
            samples_per_piece: 5,
        }
    }

    pub fn with_samples(mut self, samples: usize) -> Self {
        self.samples_per_piece = samples;
        self
    }

    /// 一次遍历同时完成安全检查和障碍扫描（避免重复全量求值）。
    /// 返回 (新障碍点, 是否安全)。
    #[fastrace::trace]
    pub fn scan_all(
        &self,
        traj: &Trajectory,
        guide: &[Vector3<f64>],
        planes_by_point: &[Vec<Plane>],
    ) -> (Vec<Hit>, bool) {
        const DETECT: usize = 40;
        let mut hits = Vec::new();
        let mut safe = true;
        for (i, ti) in traj.durations().iter().enumerate() {
            for k in 0..DETECT {
                let tau = k as f64 / DETECT as f64;
                let t = segment_time(traj, i, *ti, tau);
                let s = traj.eval(t);
                if !self.map.is_occupied(s.position) {
                    continue;
                }
                safe = false;
                let j = (tau * self.samples_per_piece as f64).round() as usize;
                let point_index = i * (self.samples_per_piece + 1) + j;
                if planes_by_point[point_index]
                    .iter()
                    .all(|pl| pl.distance(s.position).value() > 0.0)
                    && let Some(nearest) = nearest_point(guide, s.position)
                {
                    hits.push(Hit {
                        point_index,
                        sample: s,
                        guide_point: nearest,
                    });
                }
            }
        }
        (hits, safe)
    }

    /// 生成 {s, v} 平面：v 指向引导路径（出障碍方向），
    /// s 为该方向上的障碍表面点（v1 论文 Fig. 3a）。
    pub fn build_plane(map: &GridMap, point: Vector3<f64>, guide_point: Vector3<f64>) -> Plane {
        let dir = guide_point - point;
        let norm = dir.norm();
        if norm < 1e-9 {
            return Plane::new(point, Vector3::new(1.0, 0.0, 0.0));
        }
        let v = dir / norm;
        let r = map.resolution();
        let mut t = 0.0;
        let s = loop {
            let p = point + v * t;
            if !map.is_occupied(p) {
                break p;
            }
            t += r * 0.25;
        };
        Plane::new(s, v)
    }

    /// 当前轨迹是否物理安全：高密度采样均不在占据体素内。
    pub fn is_safe(&self, traj: &Trajectory) -> bool {
        const DETECT: usize = 40;
        for (i, ti) in traj.durations().iter().enumerate() {
            for k in 0..DETECT {
                let tau = k as f64 / DETECT as f64;
                let t = segment_time(traj, i, *ti, tau);
                if self.map.is_occupied(traj.eval(t).position) {
                    return false;
                }
            }
        }
        true
    }
}

fn segment_time(traj: &Trajectory, piece: usize, duration: f64, tau: f64) -> f64 {
    let mut t = 0.0;
    for k in 0..piece {
        t += traj.durations()[k];
    }
    t + tau * duration
}

fn nearest_point(points: &[Vector3<f64>], p: Vector3<f64>) -> Option<Vector3<f64>> {
    points
        .iter()
        .min_by(|a, b| (*a - p).norm_squared().total_cmp(&(*b - p).norm_squared()))
        .copied()
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_map::GridMapBuilder;
    use firefly_map::VoxelState;
    use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
    use nalgebra::Point3;

    #[test]
    fn plane_points_toward_guide() {
        let mut map = GridMapBuilder::new(0.5, [12, 12, 12]).build().unwrap();
        // 原点附近放一个占据体素（pkey=(0,0,0) 在障碍内）
        map.set_state([0, 0, 0], VoxelState::Occupied);
        let plane = ObstacleScanner::build_plane(
            &map,
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
        );
        // 表面点 s 在障碍表面（x ≈ 0.25），v 指向 +x（出障碍方向）
        assert!(plane.point().x.abs() <= 0.6, "s={:?}", plane.point());
        assert!(plane.normal().x > 0.9, "v={:?}", plane.normal());
        // pkey 在障碍侧：do < 0
        let d = plane.distance(Vector3::new(0.0, 0.0, 0.0)).value();
        assert!(d < 0.0, "pkey side should be negative, got {d}");
        // 表面外侧：do > 0
        let d_free = plane.distance(Vector3::new(1.0, 0.0, 0.0)).value();
        assert!(d_free > 0.0, "free side should be positive, got {d_free}");
    }

    #[test]
    fn scan_finds_colliding_points() {
        let mut map = GridMapBuilder::new(0.5, [12, 12, 12]).build().unwrap();
        // 在 y=0 平面处放一堵墙（轨迹沿 y=0 直线，必然被挡）
        for y in 0..2 {
            for z in 0..12 {
                map.set_state([5, y, z], VoxelState::Occupied);
            }
        }
        let scanner = ObstacleScanner::new(&map);

        let start = Endpoint {
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(6.0, 0.0, 0.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        // 直线穿越墙的轨迹
        let minco = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(
                &[Point3::new(2.0, 0.0, 0.0), Point3::new(4.0, 0.0, 0.0)],
                &[1.5, 1.5, 1.5],
            )
            .unwrap();
        let traj = minco.solve().unwrap();

        let guide: Vec<Vector3<f64>> = (0..=12)
            .map(|k| Vector3::new(f64::from(k) * 0.5, 0.0, 0.0))
            .collect();
        let planes_by_point: Vec<Vec<Plane>> = vec![Vec::new(); traj.pieces() * 6];
        let (hits, _) = scanner.scan_all(&traj, &guide, &planes_by_point);
        assert!(!hits.is_empty(), "直线穿墙必然有碰撞采样点");
    }
}
