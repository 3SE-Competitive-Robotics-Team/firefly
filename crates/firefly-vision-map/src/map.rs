//! 地图类型与空间查询。

/// 单个 3D 路标：全局坐标 + 库图 2D 观测 + ALIKED 描述子（`PnP` 的 3D 侧）。
#[derive(Debug, Clone)]
pub struct VisionMapPoint {
    /// 全局位置（米）。
    pub position: [f64; 3],
    /// 库图中的像素坐标（`LightGlue` 图-图匹配后转 2D-3D 用）。
    pub uv: [f32; 2],
    /// 描述子（128 维，与 `aliked-n16-k512.onnx` 一致）。
    pub descriptor: [f32; 128],
    /// 检测得分（建图质量，供查询截断）。
    pub score: f32,
}

/// 视觉关键帧：位姿 + 可见路标（对照 `KeyFrame`）。
#[derive(Debug, Clone)]
pub struct VisionKeyFrame {
    /// 帧序号（`0..`）。
    pub id: u64,
    /// 采集时间戳（秒）。
    pub timestamp: f64,
    /// 全局位置（米）。
    pub position: [f64; 3],
    /// 全局姿态四元数 `[x, y, z, w]`。
    pub quat_xyzw: [f64; 4],
    /// 本帧路标。
    pub points: Vec<VisionMapPoint>,
}

/// 视觉先验地图：关键帧集合（在线只读、冻结）。
#[derive(Debug, Clone, Default)]
pub struct VisionMap {
    /// 关键帧（按 `id` 递增）。
    pub frames: Vec<VisionKeyFrame>,
}

impl VisionMap {
    /// 新建空地图。
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// 路标总数（诊断用）。
    #[must_use]
    pub fn num_points(&self) -> usize {
        self.frames.iter().map(|f| f.points.len()).sum()
    }

    /// 半径内帧（下标，按距离平方升序）。
    fn scored_in_radius(&self, position: [f64; 3], radius: f64) -> Vec<(usize, f64)> {
        let mut scored: Vec<(usize, f64)> = self
            .frames
            .iter()
            .enumerate()
            .map(|(i, f)| {
                let dx = f.position[0] - position[0];
                let dy = f.position[1] - position[1];
                let dz = f.position[2] - position[2];
                (i, dx * dx + dy * dy + dz * dz)
            })
            .filter(|&(_, d2)| d2 <= radius * radius)
            .collect();
        scored.sort_by(|a, b| a.1.total_cmp(&b.1));
        scored
    }

    /// 空间邻域查询：返回与 `position` 距离 `≤ radius` 的帧下标（按距离升序，
    /// 至多 `max_frames` 个）。调用方用 VIO（矫正后）位姿作先验。
    #[must_use]
    pub fn query_neighbors(
        &self,
        position: [f64; 3],
        radius: f64,
        max_frames: usize,
    ) -> Vec<usize> {
        let mut scored = self.scored_in_radius(position, radius);
        scored.truncate(max_frames);
        scored.into_iter().map(|(i, _)| i).collect()
    }

    /// 分散采样查询：半径内贪心最远点采样（首帧取最近），保证视角分散，
    /// 破单点共面导致的 `PnP` 翻转二义性。
    #[must_use]
    pub fn query_diverse(&self, position: [f64; 3], radius: f64, max_frames: usize) -> Vec<usize> {
        let scored = self.scored_in_radius(position, radius);
        if scored.is_empty() || max_frames == 0 {
            return Vec::new();
        }
        let mut selected = vec![scored[0].0];
        let mut remaining: Vec<usize> = scored.iter().skip(1).map(|&(i, _)| i).collect();
        while selected.len() < max_frames && !remaining.is_empty() {
            let mut best = 0usize;
            let mut best_d2 = -1.0f64;
            for (k, &cand) in remaining.iter().enumerate() {
                let cp = self.frames[cand].position;
                let mut min_d2 = f64::INFINITY;
                for &sel in &selected {
                    let sp = self.frames[sel].position;
                    let dx = cp[0] - sp[0];
                    let dy = cp[1] - sp[1];
                    let dz = cp[2] - sp[2];
                    min_d2 = min_d2.min(dx * dx + dy * dy + dz * dz);
                }
                if min_d2 > best_d2 {
                    best_d2 = min_d2;
                    best = k;
                }
            }
            selected.push(remaining.remove(best));
        }
        selected
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(id: u64, x: f64) -> VisionKeyFrame {
        VisionKeyFrame {
            id,
            timestamp: f64::from(u32::try_from(id).unwrap_or(u32::MAX)),
            position: [x, 0.0, 0.0],
            quat_xyzw: [0.0, 0.0, 0.0, 1.0],
            points: vec![VisionMapPoint {
                position: [x, 0.0, 0.0],
                uv: [0.0, 0.0],
                descriptor: [0.0; 128],
                score: 1.0,
            }],
        }
    }

    #[test]
    fn neighbors_by_distance() {
        let map = VisionMap {
            frames: vec![frame(0, 0.0), frame(1, 5.0), frame(2, 1.0)],
        };
        assert_eq!(map.query_neighbors([0.2, 0.0, 0.0], 2.0, 10), vec![0, 2]);
        assert_eq!(map.query_neighbors([0.2, 0.0, 0.0], 2.0, 1), vec![0]);
        assert!(map.query_neighbors([100.0, 0.0, 0.0], 2.0, 10).is_empty());
        assert_eq!(map.num_points(), 3);
    }
}
