//! 体素地图（对照 Voxel-SVIO `mapManagement` 与
//! `voxelStereoVio::getRecentVoxel`/`featureUpdate` 选点段）。
//!
//! 约定：路标位置由调用方状态持有，本结构存 `featid → 体素` 索引与每体素
//! 访问节拍；`recent_voxels` 的查询位置取调用方当前帧三角化/估计位置。

use std::collections::HashMap;

use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector3;

use crate::options::VoxelOptions;

/// 体素键：位置除以体素尺寸向零截断取整（对照 `voxel(kx,ky,kz)` 的
/// `static_cast<short>` 语义；索引类型换 `i32` 防大场景溢出）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VoxelKey(pub i32, pub i32, pub i32);

/// 体素内一点（`featid` + 调用方同步来的全局位置）。
#[derive(Debug, Clone, Copy)]
struct VoxelPoint {
    featid: usize,
    position: Vector3<f64>,
}

/// 体素块（对照 `voxelBlock`：点表 + 最近访问时刻）。
#[derive(Debug, Clone, Default)]
struct VoxelBlock {
    points: Vec<VoxelPoint>,
    last_visit: f64,
}

/// 体素地图。
#[derive(Debug, Clone)]
pub struct VoxelMap {
    options: VoxelOptions,
    blocks: HashMap<VoxelKey, VoxelBlock>,
    index: HashMap<usize, VoxelKey>,
}

impl VoxelMap {
    /// 构造。非法参数回落默认值并告警（`voxel_size <= 0` 会除零，
    /// `neighbor_radius < 0` 无意义）。
    #[must_use]
    pub fn new(options: VoxelOptions) -> Self {
        let mut options = options;
        if !options.voxel_size.is_finite() || options.voxel_size <= 0.0 {
            log::warn!(
                "体素尺寸非法 {}，回落默认 {}",
                options.voxel_size,
                VoxelOptions::default().voxel_size
            );
            options.voxel_size = VoxelOptions::default().voxel_size;
        }
        if options.neighbor_radius < 0 {
            log::warn!("邻域半径非法 {}，回落 0", options.neighbor_radius);
            options.neighbor_radius = 0;
        }
        Self {
            options,
            blocks: HashMap::new(),
            index: HashMap::new(),
        }
    }

    /// 当前参数（调用方透传配置用）。
    #[must_use]
    pub fn options(&self) -> &VoxelOptions {
        &self.options
    }

    /// 位置对应的体素键。
    #[must_use]
    pub fn key_of(&self, position: &Vector3<f64>) -> VoxelKey {
        // `as` 向零截断 = C++ `static_cast` 语义（含负坐标）。
        let s = self.options.voxel_size;
        VoxelKey(
            (position.x / s) as i32,
            (position.y / s) as i32,
            (position.z / s) as i32,
        )
    }

    /// 已索引点数。
    #[must_use]
    pub fn num_points(&self) -> usize {
        self.index.len()
    }

    /// 非空体素数。
    #[must_use]
    pub fn num_voxels(&self) -> usize {
        self.blocks
            .values()
            .filter(|b| !b.points.is_empty())
            .count()
    }

    /// 是否已索引该特征。
    #[must_use]
    pub fn contains(&self, featid: usize) -> bool {
        self.index.contains_key(&featid)
    }

    /// 收录一点（对照 `addPointToVoxel`）。
    ///
    /// 体素已满或与块内点过近（`< min_point_distance`）时拒绝并返回
    /// `false`；已收录的 id 转为位置更新并返回 `true`。
    #[fastrace::trace]
    pub fn add_point(&mut self, featid: usize, position: &Vector3<f64>) -> bool {
        if self.index.contains_key(&featid) {
            let _ = self.update_point(featid, position);
            return true;
        }
        let key = self.key_of(position);
        let options = &self.options;
        let block = self.blocks.entry(key).or_insert_with(|| VoxelBlock {
            last_visit: -1.0,
            ..VoxelBlock::default()
        });
        if block.points.len() >= options.max_points_per_voxel {
            return false;
        }
        let min_sq = options.min_point_distance * options.min_point_distance;
        if block
            .points
            .iter()
            .any(|p| (p.position - position).norm_squared() < min_sq)
        {
            return false;
        }
        block.points.push(VoxelPoint {
            featid,
            position: *position,
        });
        self.index.insert(featid, key);
        true
    }

    /// 同步位置（对照 `changeHostVoxel`）。
    ///
    /// 同体素只更新存储位置；跨体素从旧块迁出（旧块变空则删块）并迁入
    /// 新块（迁入不限容量，与原文一致）。
    ///
    /// # Errors
    ///
    /// 未收录的 `featid`（`NotFound`）。
    #[fastrace::trace]
    pub fn update_point(&mut self, featid: usize, position: &Vector3<f64>) -> Result<()> {
        let Some(old_key) = self.index.get(&featid).copied() else {
            return Err(
                Error::new(ErrorKind::NotFound, "voxel index miss").with_context("featid", featid)
            );
        };
        let new_key = self.key_of(position);
        if old_key == new_key {
            if let Some(block) = self.blocks.get_mut(&old_key)
                && let Some(p) = block.points.iter_mut().find(|p| p.featid == featid)
            {
                p.position = *position;
            }
            return Ok(());
        }
        if let Some(old_block) = self.blocks.get_mut(&old_key) {
            old_block.points.retain(|p| p.featid != featid);
            if old_block.points.is_empty() {
                self.blocks.remove(&old_key);
            }
        }
        let new_block = self.blocks.entry(new_key).or_insert_with(|| VoxelBlock {
            last_visit: -1.0,
            ..VoxelBlock::default()
        });
        new_block.points.push(VoxelPoint {
            featid,
            position: *position,
        });
        self.index.insert(featid, new_key);
        Ok(())
    }

    /// 删除一点（对照 `deleteFromVoxel`；块变空则删块）。
    ///
    /// # Errors
    ///
    /// 未收录的 `featid`（`NotFound`）。
    pub fn remove_point(&mut self, featid: usize) -> Result<()> {
        let Some(key) = self.index.remove(&featid) else {
            return Err(
                Error::new(ErrorKind::NotFound, "voxel index miss").with_context("featid", featid)
            );
        };
        if let Some(block) = self.blocks.get_mut(&key) {
            block.points.retain(|p| p.featid != featid);
            if block.points.is_empty() {
                self.blocks.remove(&key);
            }
        }
        Ok(())
    }

    /// 当前帧可见体素（对照 `getRecentVoxel`）。
    ///
    /// 每个查询位置按邻域半径展开，命中非空、且本时刻未访问过
    /// （`last_visit < timestamp` 去重）的体素；命中即盖访问时刻戳。
    #[fastrace::trace]
    pub fn recent_voxels(&mut self, queries: &[Vector3<f64>], timestamp: f64) -> Vec<VoxelKey> {
        let mut recent = Vec::new();
        let r = self.options.neighbor_radius;
        for q in queries {
            let center = self.key_of(q);
            for dx in -r..=r {
                for dy in -r..=r {
                    for dz in -r..=r {
                        let key = VoxelKey(
                            center.0.saturating_add(dx),
                            center.1.saturating_add(dy),
                            center.2.saturating_add(dz),
                        );
                        let Some(block) = self.blocks.get_mut(&key) else {
                            continue;
                        };
                        if block.points.is_empty() || block.last_visit >= timestamp {
                            continue;
                        }
                        block.last_visit = timestamp;
                        recent.push(key);
                    }
                }
            }
        }
        recent
    }

    /// 可见体素内的代表特征（对照 Voxel-SVIO `featureUpdate` 选点段）。
    ///
    /// `use_all_points=false`（默认）时每体素只取**块首**点（官方
    /// `voxel_block.points.front()`）；块首对应的 feature 若已从库中消失，官方
    /// 该体素不更新任何东西，这里同样只返回块首 id，由调用方按库内存在性取舍。
    /// `true` 时返回块内全部点（保序）。缺失/已空体素跳过；`recent` 顺序即输出
    /// 顺序。
    #[must_use]
    pub fn select(&self, recent: &[VoxelKey]) -> Vec<usize> {
        let mut out = Vec::new();
        for key in recent {
            let Some(block) = self.blocks.get(key) else {
                continue;
            };
            if self.options.use_all_points {
                out.extend(block.points.iter().map(|p| p.featid));
            } else if let Some(front) = block.points.first() {
                out.push(front.featid);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_map() -> VoxelMap {
        VoxelMap::new(VoxelOptions {
            enabled: true,
            ..VoxelOptions::default()
        })
    }

    /// 不变量：索引与块内容双向一致（`index.len == 各块点数之和`；每个索引
    /// 项都能在对应块里找到，每个块内点都有正确反向索引）。
    fn assert_consistent(map: &VoxelMap) {
        let total: usize = map.blocks.values().map(|b| b.points.len()).sum();
        assert_eq!(map.index.len(), total, "索引与块点数不一致");
        for (id, key) in &map.index {
            let block = map.blocks.get(key).expect("索引指向缺失块");
            assert!(
                block.points.iter().any(|p| &p.featid == id),
                "索引 {id} 在块 {key:?} 中缺失"
            );
        }
        for (key, block) in &map.blocks {
            assert!(!block.points.is_empty(), "空块 {key:?} 未回收");
            for p in &block.points {
                assert_eq!(
                    map.index.get(&p.featid),
                    Some(key),
                    "块内点 {} 反向索引错误",
                    p.featid
                );
            }
        }
    }

    #[test]
    fn option_defaults_match_documented_values() {
        let o = VoxelOptions::default();
        assert!(!o.enabled);
        assert!((o.voxel_size - 0.1).abs() < 1e-12);
        assert_eq!(o.max_points_per_voxel, 5);
        assert!((o.min_point_distance - 0.03).abs() < 1e-12);
        assert_eq!(o.neighbor_radius, 1);
        assert!(!o.use_all_points);
    }

    #[test]
    fn illegal_options_fall_back_to_defaults() {
        for bad_size in [0.0, -0.1, f64::NAN, f64::INFINITY] {
            let map = VoxelMap::new(VoxelOptions {
                voxel_size: bad_size,
                ..VoxelOptions::default()
            });
            assert!(
                (map.options().voxel_size - VoxelOptions::default().voxel_size).abs() < 1e-12,
                "非法体素尺寸 {bad_size} 未回落"
            );
        }
        let map = VoxelMap::new(VoxelOptions {
            neighbor_radius: -3,
            ..VoxelOptions::default()
        });
        assert_eq!(map.options().neighbor_radius, 0);
    }

    #[test]
    fn key_quantization_truncates_toward_zero() {
        let map = test_map();
        assert_eq!(
            map.key_of(&Vector3::new(0.15, -0.05, 1.0)),
            VoxelKey(1, 0, 10)
        );
        // 向零截断：负半格归 0（含 C++ `static_cast` 语义），与 floor 区分。
        assert_eq!(
            map.key_of(&Vector3::new(-0.05, 0.0, 0.0)),
            VoxelKey(0, 0, 0)
        );
        assert_eq!(
            map.key_of(&Vector3::new(-0.15, 0.0, 0.0)),
            VoxelKey(-1, 0, 0)
        );
        // 精确整格点（0.5 米格）：1.5 → 3，-1.5 → -3。
        let half = VoxelMap::new(VoxelOptions {
            voxel_size: 0.5,
            ..VoxelOptions::default()
        });
        assert_eq!(
            half.key_of(&Vector3::new(1.5, -1.5, 0.0)),
            VoxelKey(3, -3, 0)
        );
    }

    #[test]
    fn add_rejects_full_and_crowded_voxels() {
        let mut map = VoxelMap::new(VoxelOptions {
            max_points_per_voxel: 2,
            min_point_distance: 0.03,
            ..VoxelOptions::default()
        });
        assert!(map.add_point(1, &Vector3::new(0.0, 0.0, 0.0)));
        // 同体素 1cm 处：过近拒绝
        assert!(!map.add_point(2, &Vector3::new(0.01, 0.0, 0.0)));
        assert!(map.add_point(2, &Vector3::new(0.05, 0.0, 0.0)));
        // 已满拒绝
        assert!(!map.add_point(3, &Vector3::new(0.08, 0.0, 0.0)));
        assert_eq!(map.num_points(), 2);
        assert_consistent(&map);
    }

    #[test]
    fn add_accepts_exact_min_distance() {
        // 拒绝条件是严格 `< min`：恰等于最小间距时收录。
        let mut map = VoxelMap::new(VoxelOptions {
            min_point_distance: 0.5,
            ..VoxelOptions::default()
        });
        assert!(map.add_point(1, &Vector3::new(0.0, 0.0, 0.0)));
        assert!(map.add_point(2, &Vector3::new(0.5, 0.0, 0.0)));
        assert_eq!(map.num_points(), 2);
        assert_consistent(&map);
    }

    #[test]
    fn readd_same_id_updates_position() {
        // 已收录 id 再次 add 转为位置更新（返回 true，点数不变），跨体素自动搬家。
        let mut map = VoxelMap::new(VoxelOptions {
            min_point_distance: 0.0,
            ..VoxelOptions::default()
        });
        assert!(map.add_point(1, &Vector3::new(0.05, 0.0, 0.0)));
        assert!(map.add_point(1, &Vector3::new(5.0, 0.0, 0.0)));
        assert_eq!(map.num_points(), 1);
        assert_eq!(map.num_voxels(), 1);
        assert_eq!(map.key_of(&Vector3::new(5.0, 0.0, 0.0)), VoxelKey(50, 0, 0));
        assert!(map.contains(1));
        assert_consistent(&map);
    }

    #[test]
    fn update_moves_across_voxels() {
        let mut map = test_map();
        assert!(map.add_point(1, &Vector3::new(0.05, 0.0, 0.0)));
        let before = map.num_voxels();
        map.update_point(1, &Vector3::new(5.0, 0.0, 0.0))
            .expect("indexed");
        assert_eq!(map.num_voxels(), before);
        assert!(map.remove_point(1).is_ok());
        assert_eq!(map.num_points(), 0);
        assert!(map.update_point(1, &Vector3::zeros()).is_err());
        assert!(map.remove_point(1).is_err());
        assert_consistent(&map);
    }

    #[test]
    fn update_same_voxel_refreshes_stored_position() {
        // 同体素更新必须刷新存储位置（EKF 更新后路标会动）。
        let mut map = test_map();
        map.add_point(1, &Vector3::new(0.01, 0.0, 2.0));
        map.update_point(1, &Vector3::new(0.05, 0.0, 2.0))
            .expect("indexed");
        let key = map.key_of(&Vector3::new(0.05, 0.0, 2.0));
        let stored = map.blocks[&key]
            .points
            .iter()
            .find(|p| p.featid == 1)
            .expect("point present");
        assert!((stored.position - Vector3::new(0.05, 0.0, 2.0)).norm() < 1e-12);
        assert_consistent(&map);
    }

    #[test]
    fn update_into_full_voxel_is_unbounded_like_upstream() {
        // 跨体素迁入不限容量（与官方 `changeHostVoxel` 一致）：目标满仍迁入。
        let mut map = VoxelMap::new(VoxelOptions {
            max_points_per_voxel: 1,
            min_point_distance: 0.0,
            ..VoxelOptions::default()
        });
        map.add_point(1, &Vector3::new(0.05, 0.0, 0.0));
        map.add_point(2, &Vector3::new(5.0, 0.0, 0.0));
        map.update_point(2, &Vector3::new(0.06, 0.0, 0.0))
            .expect("indexed");
        assert_eq!(map.num_points(), 2);
        assert_eq!(map.num_voxels(), 1);
        assert_consistent(&map);
    }

    #[test]
    fn remove_recycles_empty_blocks() {
        let mut map = VoxelMap::new(VoxelOptions {
            min_point_distance: 0.0,
            ..VoxelOptions::default()
        });
        map.add_point(1, &Vector3::new(0.01, 0.0, 0.0));
        map.add_point(2, &Vector3::new(0.02, 0.0, 0.0));
        assert_eq!(map.num_voxels(), 1);
        assert!(map.remove_point(1).is_ok());
        assert_eq!(map.num_voxels(), 1);
        assert!(map.remove_point(2).is_ok());
        assert_eq!(map.num_voxels(), 0);
        assert!(map.remove_point(2).is_err());
        assert_consistent(&map);
    }

    #[test]
    fn recent_dedups_within_timestamp() {
        let mut map = test_map();
        assert!(map.add_point(1, &Vector3::new(0.05, 0.0, 0.0)));
        let queries = [Vector3::new(0.05, 0.0, 0.0)];
        let first = map.recent_voxels(&queries, 10.0);
        assert_ne!(first, [] as [VoxelKey; 0]);
        // 同一时刻重复查询：已盖戳，不重复
        assert_eq!(map.recent_voxels(&queries, 10.0), [] as [VoxelKey; 0]);
        // 新时刻可再次命中
        assert_ne!(map.recent_voxels(&queries, 11.0), [] as [VoxelKey; 0]);
    }

    #[test]
    fn recent_dedups_across_overlapping_queries() {
        // 同一体素被多个查询命中时只出现一次（同时刻）。
        let mut map = test_map();
        map.add_point(1, &Vector3::new(0.05, 0.0, 3.0));
        let queries = [
            Vector3::new(0.05, 0.0, 3.0),
            Vector3::new(0.06, 0.0, 3.0),
            Vector3::new(0.05, 0.01, 3.0),
        ];
        let recent = map.recent_voxels(&queries, 1.0);
        assert_eq!(recent.len(), 1);
    }

    /// 体素键 `k` 的格心（复现 C++ 显示代码的 `kx>=0 ? kx+0.5 : kx-0.5`
    /// 语义，保证键心位置反过来映射回同一键；截断向零使负键的格心在 `k-0.5`）。
    fn center(k: i32, size: f64) -> f64 {
        if k >= 0 {
            (f64::from(k) + 0.5) * size
        } else {
            (f64::from(k) - 0.5) * size
        }
    }

    /// 填入以 `origin` 为中心的 3×3×3 体素邻域，返回生成的 id 数。
    fn fill_27(map: &mut VoxelMap, origin: (i32, i32, i32), size: f64) -> usize {
        let mut id = 0usize;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    id += 1;
                    map.add_point(
                        id,
                        &Vector3::new(
                            center(origin.0 + dx, size),
                            center(origin.1 + dy, size),
                            center(origin.2 + dz, size),
                        ),
                    );
                }
            }
        }
        id
    }

    #[test]
    fn recent_radius_zero_hits_center_only() {
        let size = 0.1;
        let mut map = VoxelMap::new(VoxelOptions {
            voxel_size: size,
            neighbor_radius: 0,
            min_point_distance: 0.0,
            max_points_per_voxel: 100,
            ..VoxelOptions::default()
        });
        // 以 (0,0,31) 为中心填满 27 邻域，半径 0 时只命中中心。
        let n = fill_27(&mut map, (0, 0, 31), size);
        assert_eq!(n, 27);
        assert_eq!(map.num_voxels(), 27);
        let recent = map.recent_voxels(&[Vector3::new(0.05, 0.0, 3.15)], 1.0);
        assert_eq!(recent, vec![VoxelKey(0, 0, 31)]);
    }

    #[test]
    fn recent_radius_one_expands_to_27_in_order() {
        let size = 0.1;
        let mut map = VoxelMap::new(VoxelOptions {
            voxel_size: size,
            min_point_distance: 0.0,
            max_points_per_voxel: 100,
            ..VoxelOptions::default()
        });
        fill_27(&mut map, (0, 0, 31), size);
        let recent = map.recent_voxels(&[Vector3::new(0.05, 0.0, 3.15)], 2.0);
        // dx 外层、dz 内层确定顺序断言（复现性看护）。
        assert_eq!(recent.len(), 27);
        assert_eq!(recent[0], VoxelKey(-1, -1, 30));
        assert_eq!(recent[26], VoxelKey(1, 1, 32));
        assert_eq!(recent[13], VoxelKey(0, 0, 31));
    }

    #[test]
    fn recent_skips_empty_and_unknown() {
        let mut map = test_map();
        map.add_point(1, &Vector3::new(0.05, 0.0, 3.0));
        // 空查询与地图外查询都返回空。
        assert_eq!(map.recent_voxels(&[], 1.0), [] as [VoxelKey; 0]);
        assert_eq!(
            map.recent_voxels(&[Vector3::new(50.0, 50.0, 50.0)], 1.0),
            [] as [VoxelKey; 0]
        );
        assert_consistent(&map);
    }

    /// 每体素只取块首点（对照官方 `points.front()`）。
    #[test]
    fn select_takes_block_front_point_only() {
        let mut map = VoxelMap::new(VoxelOptions {
            min_point_distance: 0.0,
            ..VoxelOptions::default()
        });
        assert!(map.add_point(1, &Vector3::new(0.01, 0.0, 0.0)));
        assert!(map.add_point(2, &Vector3::new(0.02, 0.0, 0.0)));
        let recent = map.recent_voxels(&[Vector3::new(0.01, 0.0, 0.0)], 1.0);
        assert_eq!(map.select(&recent), vec![1]);
    }

    #[test]
    fn select_all_points_when_configured() {
        let mut map = VoxelMap::new(VoxelOptions {
            min_point_distance: 0.0,
            use_all_points: true,
            ..VoxelOptions::default()
        });
        assert!(map.add_point(1, &Vector3::new(0.01, 0.0, 0.0)));
        assert!(map.add_point(2, &Vector3::new(0.02, 0.0, 0.0)));
        let recent = map.recent_voxels(&[Vector3::new(0.01, 0.0, 0.0)], 1.0);
        assert_eq!(map.select(&recent), vec![1, 2]);
    }

    #[test]
    fn select_skips_missing_keys_and_empty_input() {
        let map = test_map();
        assert_eq!(map.select(&[]), [] as [usize; 0]);
        assert_eq!(map.select(&[VoxelKey(999, 999, 999)]), [] as [usize; 0]);
    }

    /// 块首点被移除后，同体素的下一个点顶上（插入序）。
    #[test]
    fn select_promotes_next_point_after_front_removed() {
        let mut map = VoxelMap::new(VoxelOptions {
            min_point_distance: 0.0,
            ..VoxelOptions::default()
        });
        map.add_point(1, &Vector3::new(0.01, 0.0, 0.0));
        map.add_point(2, &Vector3::new(0.02, 0.0, 0.0));
        let recent = map.recent_voxels(&[Vector3::new(0.01, 0.0, 0.0)], 1.0);
        assert_eq!(map.select(&recent), vec![1]);
        map.remove_point(1).expect("indexed");
        let recent = map.recent_voxels(&[Vector3::new(0.01, 0.0, 0.0)], 2.0);
        assert_eq!(map.select(&recent), vec![2]);
    }

    /// 每可见体素恰一个代表点、无重复、顺序随 `recent`，且可复现。
    #[test]
    fn select_is_one_point_per_voxel_in_recent_order() {
        let mut map = VoxelMap::new(VoxelOptions {
            min_point_distance: 0.0,
            max_points_per_voxel: 10,
            ..VoxelOptions::default()
        });
        for id in 1..=20usize {
            let x = f64::from(id as u32 % 5) * 0.2 + 0.05;
            let z = 2.0 + f64::from(id as u32 / 5) * 0.2;
            map.add_point(id, &Vector3::new(x, 0.0, z));
        }
        assert_consistent(&map);
        let queries: Vec<Vector3<f64>> = (1..=20usize)
            .map(|id| {
                let x = f64::from(id as u32 % 5) * 0.2 + 0.05;
                let z = 2.0 + f64::from(id as u32 / 5) * 0.2;
                Vector3::new(x, 0.0, z)
            })
            .collect();
        // 先克隆再查询：`recent_voxels` 会盖 `last_visit`，同刻重放须从头开始。
        let mut map2 = map.clone();
        let recent = map.recent_voxels(&queries, 1.0);
        let selected = map.select(&recent);
        assert_eq!(selected.len(), recent.len(), "每体素恰一个代表点");
        let mut dedup = selected.clone();
        dedup.sort_unstable();
        dedup.dedup();
        assert_eq!(dedup.len(), selected.len(), "选中出现重复");
        let recent2 = map2.recent_voxels(&queries, 1.0);
        assert_eq!(map2.select(&recent2), selected);
    }

    #[test]
    fn multiframe_pipeline_end_to_end() {
        // 整体输入输出：收录 → 可见查询 → 选点 → 边缘化 → 跨体素搬家 → 次帧重查。
        let mut map = VoxelMap::new(VoxelOptions {
            min_point_distance: 0.0,
            ..VoxelOptions::default()
        });
        for (id, x) in [(1usize, 0.01), (2, 0.02), (3, 5.0), (4, 5.02)] {
            assert!(map.add_point(id, &Vector3::new(x, 0.0, 3.0)));
        }
        let queries = [
            Vector3::new(0.01, 0.0, 3.0),
            Vector3::new(5.0, 0.0, 3.0),
            Vector3::new(50.0, 0.0, 3.0),
        ];
        let recent = map.recent_voxels(&queries, 1.0);
        assert_eq!(recent.len(), 2);
        assert_eq!(map.select(&recent), vec![1, 3]);
        // 边缘化块首 1：同体素下一点 2 顶上。
        map.remove_point(1).expect("indexed");
        let recent = map.recent_voxels(&queries, 2.0);
        assert_eq!(map.select(&recent), vec![2, 3]);
        // 路标 4 跨体素搬家：块首仍是 2，全点模式含 4。
        map.update_point(4, &Vector3::new(0.03, 0.0, 3.0))
            .expect("indexed");
        assert_eq!(map.num_voxels(), 2);
        let recent = map.recent_voxels(&queries, 3.0);
        assert_eq!(map.select(&recent), vec![2, 3]);
        assert_consistent(&map);
    }
}
