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

    /// 可见体素内待更新特征（对照 `featureUpdate` 选点段）。
    ///
    /// 默认每体素只取首点（`use_all_points=false`）；缺失/已空的体素跳过。
    #[must_use]
    pub fn select(&self, recent: &[VoxelKey]) -> Vec<usize> {
        let mut out = Vec::new();
        for key in recent {
            let Some(block) = self.blocks.get(key) else {
                continue;
            };
            if self.options.use_all_points {
                out.extend(block.points.iter().map(|p| p.featid));
            } else if let Some(first) = block.points.first() {
                out.push(first.featid);
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

    #[test]
    fn key_quantization_truncates_toward_zero() {
        let map = test_map();
        assert_eq!(
            map.key_of(&Vector3::new(0.15, -0.05, 1.0)),
            VoxelKey(1, 0, 10)
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
    }

    #[test]
    fn recent_dedups_within_timestamp() {
        let mut map = test_map();
        assert!(map.add_point(1, &Vector3::new(0.05, 0.0, 0.0)));
        let queries = [Vector3::new(0.05, 0.0, 0.0)];
        let first = map.recent_voxels(&queries, 10.0);
        assert!(!first.is_empty());
        // 同一时刻重复查询：已盖戳，不重复
        assert!(map.recent_voxels(&queries, 10.0).is_empty());
        // 新时刻可再次命中
        assert!(!map.recent_voxels(&queries, 11.0).is_empty());
    }

    #[test]
    fn select_first_only_by_default() {
        let mut map = VoxelMap::new(VoxelOptions {
            min_point_distance: 0.0,
            ..VoxelOptions::default()
        });
        assert!(map.add_point(1, &Vector3::new(0.01, 0.0, 0.0)));
        assert!(map.add_point(2, &Vector3::new(0.02, 0.0, 0.0)));
        let queries = [Vector3::new(0.01, 0.0, 0.0)];
        let recent = map.recent_voxels(&queries, 1.0);
        assert_eq!(map.select(&recent), vec![1]);
    }

    #[test]
    fn select_all_when_configured() {
        let mut map = VoxelMap::new(VoxelOptions {
            min_point_distance: 0.0,
            use_all_points: true,
            ..VoxelOptions::default()
        });
        assert!(map.add_point(1, &Vector3::new(0.01, 0.0, 0.0)));
        assert!(map.add_point(2, &Vector3::new(0.02, 0.0, 0.0)));
        let queries = [Vector3::new(0.01, 0.0, 0.0)];
        let recent = map.recent_voxels(&queries, 1.0);
        assert_eq!(map.select(&recent), vec![1, 2]);
    }
}
