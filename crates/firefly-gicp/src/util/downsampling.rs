//! 点云降采样（对照 official `util/downsampling.hpp`）。
//!
//! 提供体素栅格平均降采样与随机降采样，并行性通过 rayon 内部实现，对外 API 与
//! 官方串行版一致。

use std::default::Default;

use nalgebra::Vector4;
use rayon::prelude::*;

use crate::points::traits::{PointCloudMut, PointCloudTrait};
use crate::util::fast_floor::fast_floor;

/// 可注入的随机数源（对照 `std::mt19937` 的 `operator()` 接口）。
///
/// 不引入 `rand` 等外部依赖；默认实现 [`SplitMix64`] 即可满足随机降采样的均匀性。
pub trait Rng {
    /// 返回 `[0, u64::MAX]` 上的均匀分布随机字。
    fn next_u64(&mut self) -> u64;
}

/// SplitMix64 伪随机数发生器（对照 `std::mt19937` 的轻量替代）。
#[derive(Clone, Debug)]
pub struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    /// 以种子构造。
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed ^ 0x9E37_9B97_F4A7_C15B,
        }
    }
}

impl Rng for SplitMix64 {
    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_9B97_F4A7_C15B);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
}

/// 体素栅格降采样：每个体素取点的精确平均，单个体素可容纳任意点数。
///
/// 离散化体素坐标须落在 21bit 范围 `[-1048576, 1048575]`；若降采样分辨率 0.01m，
/// 点坐标须落在 `[-10485.76, 10485.75]m`。超出范围的点被忽略（对照官方 `warning`）。
///
/// `Output` 默认等于输入点云类型；输出点云仅写坐标，不继承输入的法向/协方差。
#[fastrace::trace]
pub fn voxelgrid_sampling<P, O>(points: &P, leaf_size: f64) -> O
where
    P: PointCloudTrait + Sync,
    O: PointCloudTrait + PointCloudMut + Default,
{
    if points.num_points() == 0 {
        return O::default();
    }

    let inv_leaf_size = 1.0 / leaf_size;
    const COORD_BIT_SIZE: u64 = 21;
    const COORD_BIT_MASK: u64 = (1u64 << 21) - 1;
    const COORD_OFFSET: i64 = 1 << 20; // 1 << (21 - 1)，使坐标非负
    const INVALID_COORD: u64 = u64::MAX;

    let n = points.num_points();
    // 预先把每个点映射到体素键（21bit × 3 打包进 63bit），并行计算
    let coord_pt: Vec<(u64, usize)> = (0..n)
        .into_par_iter()
        .map(|i| {
            let p = points.point(i) * inv_leaf_size;
            let coord = fast_floor(&p);
            let mut key = 0u64;
            let mut valid = true;
            for c in 0..3 {
                let v = i64::from(coord[c]) + COORD_OFFSET;
                if v < 0 || v > (COORD_BIT_MASK as i64) {
                    valid = false;
                    break;
                }
                key |= ((v as u64) & COORD_BIT_MASK) << (COORD_BIT_SIZE * c as u64);
            }
            if valid { (key, i) } else { (INVALID_COORD, i) }
        })
        .collect();

    let mut coord_pt = coord_pt;
    coord_pt.sort_by_key(|a| a.0);

    let mut out = O::default();
    out.resize(n); // 上界为体素数（≤ 点数）

    let mut num_points = 0usize;
    // 初始化累加器为第一个点的齐次坐标（对照官方：sum_pt = point(front)）
    let mut sum_pt = points.point(coord_pt[0].1);
    for i in 1..n {
        let (key, idx) = coord_pt[i];
        if key == INVALID_COORD {
            continue;
        }
        if coord_pt[i - 1].0 != key {
            out.set_point(num_points, sum_pt / sum_pt.w);
            num_points += 1;
            sum_pt = Vector4::zeros();
        }
        sum_pt += points.point(idx);
    }
    out.set_point(num_points, sum_pt / sum_pt.w);
    num_points += 1;
    out.resize(num_points);

    out
}

/// 随机降采样：从输入中等概率无放回抽取 `num_samples` 个点。
///
/// `num_samples >= 点数` 时退回全部点（对照官方把 `num_samples` 钳到点数）。
#[fastrace::trace]
pub fn random_sampling<P, O>(points: &P, num_samples: usize, rng: &mut impl Rng) -> O
where
    P: PointCloudTrait,
    O: PointCloudTrait + PointCloudMut + Default,
{
    if points.num_points() == 0 {
        return O::default();
    }

    let size = points.num_points();
    let num = if num_samples >= size {
        size
    } else {
        num_samples
    };

    // 部分 Fisher–Yates 洗牌，取前 `num` 个索引（与 std::sample 等价：无放回均匀）
    let mut indices: Vec<usize> = (0..size).collect();
    for i in 0..num {
        let j = i + (rng.next_u64() % (size - i) as u64) as usize;
        indices.swap(i, j);
    }

    let mut out = O::default();
    out.resize(num);
    for i in 0..num {
        out.set_point(i, points.point(indices[i]));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::points::point_cloud::PointCloud;
    use nalgebra::Vector3;
    use std::collections::HashSet;

    fn grid_points() -> Vec<Vector3<f64>> {
        let mut pts = Vec::new();
        for x in 0..20i64 {
            for y in 0..20i64 {
                for z in 0..10i64 {
                    pts.push(Vector3::new(x as f64, y as f64, z as f64));
                }
            }
        }
        pts
    }

    #[test]
    fn empty_input() {
        let empty: Vec<Vector3<f64>> = Vec::new();
        let ds: PointCloud = voxelgrid_sampling(&empty, 0.1);
        assert_eq!(ds.num_points(), 0);
        let ds2: PointCloud = random_sampling(&empty, 1000, &mut SplitMix64::new(0));
        assert_eq!(ds2.num_points(), 0);
    }

    #[test]
    fn voxel_count_le_points() {
        let pts = grid_points();
        for &res in &[0.5f64, 1.0, 2.0] {
            let ds: PointCloud = voxelgrid_sampling(&pts, res);
            assert!(ds.num_points() <= pts.len());
            // 0.5 / 1.0 叶尺寸小于点间距（1），无合并；2.0 合并 2×2×2
            let expected = if res <= 1.0 { pts.len() } else { 500 };
            assert_eq!(ds.num_points(), expected, "resolution={res}");
        }
    }

    #[test]
    fn voxel_centroid_exact() {
        // 构造规则网格，验证每个体素输出点是该体素点的精确平均
        let pts = grid_points();
        let res = 1.0;
        let ds: PointCloud = voxelgrid_sampling(&pts, res);
        // 独立按体素分桶求平均
        let mut buckets: std::collections::HashMap<(i64, i64, i64), (Vector3<f64>, usize)> =
            std::collections::HashMap::new();
        for p in &pts {
            let k = (p.x as i64, p.y as i64, p.z as i64);
            let e = buckets.entry(k).or_insert((Vector3::zeros(), 0));
            e.0 += *p;
            e.1 += 1;
        }
        assert_eq!(ds.num_points(), buckets.len());
        for i in 0..ds.num_points() {
            let p = ds.point(i);
            let k = (p.x as i64, p.y as i64, p.z as i64);
            let (sum, cnt) = buckets[&k];
            let mean = sum / cnt as f64;
            assert!((mean.x - p.x).abs() < 1e-9);
            assert!((mean.y - p.y).abs() < 1e-9);
            assert!((mean.z - p.z).abs() < 1e-9);
            assert!((p.w - 1.0).abs() < 1e-12);
        }
    }

    #[test]
    fn random_sampling_properties() {
        let pts = grid_points();
        let ds: PointCloud = voxelgrid_sampling(&pts, 0.1);
        for &num in &[0usize, 50, 200, 400] {
            let result: PointCloud = random_sampling(&ds, num, &mut SplitMix64::new(12345));
            let expected = num.min(ds.num_points());
            assert_eq!(result.num_points(), expected, "num={num}");
            // 唯一性 + 存在性
            let mut seen = HashSet::new();
            for i in 0..result.num_points() {
                let rp = result.point(i);
                let idx = pts.iter().position(|p| {
                    (p.x - rp.x).abs() < 1e-9
                        && (p.y - rp.y).abs() < 1e-9
                        && (p.z - rp.z).abs() < 1e-9
                });
                assert!(idx.is_some(), "采样点须来自输入");
                assert!(seen.insert(idx.unwrap()), "采样点须唯一");
            }
        }
    }
}
