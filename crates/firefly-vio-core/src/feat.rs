//! 稀疏特征数据库（对照 `OpenVINS` `ov_core/src/feat`：`Feature`/`FeatureDatabase`）。
//!
//! `Feature` 收集单个跟踪点的全部历史测量（按相机 id 分桶），
//! `FeatureDatabase` 维护正在跟踪的特征集合，支持按时间戳/相机查询、
//! 删除与测量清理。
//!
//! ## 并发说明
//! 原版 `FeatureDatabase` 用 `std::mutex` 提供异步多线程接入（每个特征为裸指针、
//! 直接返回共享且不安全，需 `remove=true` 取出以免跟踪器并发改写）。Rust 借用规则
//! 由编译期保证所有权安全，故此处省略互斥锁，采用单线程所有权模型：
//! `remove=true` 时移出数据库并转移所有权，`remove=false` 时返回克隆快照
//! （语义见各方法）。wave 2 若需并发，再在 crate 层包一层 `Mutex`。
//!
//! ## MSCKF updater 消费约定
//! 移植 `MSCKF` 滑动窗口 updater 时，取用本库特征**必须使用 `remove=true` 语义**
//! （含三个查询方法与 [`FeatureDatabase::get_feature`]），与 C++ 中多线程异步跟踪场景
//! 持"取出后跟踪器不得再改写"的约定一致；`remove=false`/克隆仅用于纯只读快照。

use nalgebra::{Vector2, Vector3};
use std::collections::HashMap;
use std::collections::hash_map::Entry;

/// 单个稀疏特征（对照 `OpenVINS` `ov_core/src/feat/Feature.h`）。
///
/// 每个特征有唯一 `featid`，并携带若干时刻的像素/归一化测量与时间戳，
/// 按观察它的相机 id 分桶保存。`FeatureDatabase` 负责写入与删除本条数据。
///
/// `p_FinA`/`p_FinG` 为跨端契约钉死的名字（与 `OpenVINS` C++ 字段一致，
/// 意为 `feature position in Anchor/Global frame`），故豁免 `snake_case` 检查。
#[allow(non_snake_case)]
#[derive(Debug, Clone)]
pub struct Feature {
    /// 此特征的唯一 id。
    pub featid: usize,
    /// 是否应被 `FeatureDatabase` 删除（跟踪失败的标记）。
    pub to_delete: bool,
    /// 各相机观测到的原始像素坐标 `(u, v)`（对照 `Feature::uvs`）。
    pub uvs: HashMap<usize, Vec<Vector2<f32>>>,
    /// 各相机观测到的去畸变/归一化坐标（对照 `Feature::uvs_norm`）。
    pub uvs_norm: HashMap<usize, Vec<Vector2<f32>>>,
    /// 每次测量对应的时间戳，按相机 id 分桶（对照 `Feature::timestamps`）。
    pub timestamps: HashMap<usize, Vec<f64>>,
    /// 姿态锚定的相机 id，首条测量即锚点；无锚时为 `-1`（对照 `Feature::anchor_cam_id`）。
    pub anchor_cam_id: i32,
    /// 锚点克隆的时间戳（对照 `Feature::anchor_clone_timestamp`）。
    pub anchor_clone_timestamp: f64,
    /// 三角化位置，锚点坐标系（对照 `Feature::p_FinA`）。
    pub p_FinA: Vector3<f64>,
    /// 三角化位置，全局坐标系（对照 `Feature::p_FinG`）。
    pub p_FinG: Vector3<f64>,
}

impl Feature {
    /// 移除所有不在 `valid_times` 中发生的测量
    /// （对照 `OpenVINS` `Feature::clean_old_measurements`）。
    ///
    /// 常用于保证剩余测量都恰好在克隆时刻发生。
    pub fn clean_old_measurements(&mut self, valid_times: &[f64]) {
        retain_measurements(
            &mut self.timestamps,
            &mut self.uvs,
            &mut self.uvs_norm,
            |t| !valid_times.contains(&t),
        );
    }

    /// 移除所有在 `invalid_times` 中发生的测量
    /// （对照 `OpenVINS` `Feature::clean_invalid_measurements`）。
    pub fn clean_invalid_measurements(&mut self, invalid_times: &[f64]) {
        retain_measurements(
            &mut self.timestamps,
            &mut self.uvs,
            &mut self.uvs_norm,
            |t| invalid_times.contains(&t),
        );
    }

    /// 移除所有早于（含等于）`timestamp` 的测量
    /// （对照 `OpenVINS` `Feature::clean_older_measurements`）。
    pub fn clean_older_measurements(&mut self, timestamp: f64) {
        retain_measurements(
            &mut self.timestamps,
            &mut self.uvs,
            &mut self.uvs_norm,
            |t| t <= timestamp,
        );
    }
}

impl Default for Feature {
    fn default() -> Self {
        Self {
            featid: 0,
            to_delete: false,
            uvs: HashMap::new(),
            uvs_norm: HashMap::new(),
            timestamps: HashMap::new(),
            anchor_cam_id: -1,
            anchor_clone_timestamp: 0.0,
            // 原版默认未初始化（Eigen 的 `Vector3d` 默认不赋值、`double` 也不赋值，
            // 需三角化后才能用）；Rust 以安全零值占位，使用前须由三角化逻辑写入。
            p_FinA: Vector3::zeros(),
            p_FinG: Vector3::zeros(),
        }
    }
}

/// 按相机分桶的测量清理：对每个相机，删去 `remove_if(time)` 为真的测量，
/// 同时以相同下标保持 `timestamps`/`uvs`/`uvs_norm` 三向一致（原地、保序）。
fn retain_measurements(
    timestamps: &mut HashMap<usize, Vec<f64>>,
    uvs: &mut HashMap<usize, Vec<Vector2<f32>>>,
    uvs_norm: &mut HashMap<usize, Vec<Vector2<f32>>>,
    remove_if: impl Fn(f64) -> bool,
) {
    // 先取出相机 id 集合，避免与后续可变借用冲突。
    let cam_ids: Vec<usize> = timestamps.keys().copied().collect();
    for cam_id in cam_ids {
        let Some(ts) = timestamps.get_mut(&cam_id) else {
            continue;
        };
        let Some(uv) = uvs.get_mut(&cam_id) else {
            continue;
        };
        let Some(uvn) = uvs_norm.get_mut(&cam_id) else {
            continue;
        };
        // 原版以 `assert` 保证三者等长（release 下为 no-op）；Rust 编译期保证不了
        // 跨字段不变量，故用 `debug_assert` 显式检查，release 下紧随其后的按位
        // 压缩需要三者等长，索引均在各自切片内。
        let len = ts.len();
        debug_assert_eq!(len, uv.len(), "uvs 与 timestamps 长度不一致 (cam {cam_id})");
        debug_assert_eq!(
            len,
            uvn.len(),
            "uvs_norm 与 timestamps 长度不一致 (cam {cam_id})"
        );
        // 双指针原地压缩，保序且 O(n)：`w` 为保留位写入下标，`r` 为读取下标。
        let mut w = 0;
        for r in 0..len {
            if !remove_if(ts[r]) {
                ts[w] = ts[r];
                uv[w] = uv[r];
                uvn[w] = uvn[r];
                w += 1;
            }
        }
        ts.truncate(w);
        uv.truncate(w);
        uvn.truncate(w);
    }
}

/// 正在跟踪的特征数据库
/// （对照 `OpenVINS` `ov_core/src/feat/FeatureDatabase.h`）。
///
/// 每个视觉跟踪器持有一个本数据库，跟踪得到的新测量经
/// [`FeatureDatabase::update_feature`] 写入，滚动窗口更新阶段按时间戳查询
/// 可用特征并在处理后取出删除。
#[derive(Debug, Clone, Default)]
pub struct FeatureDatabase {
    /// 按 id 查找特征的主存储（crate 内可见，供测试与内部使用）。
    pub(crate) features: HashMap<usize, Feature>,
}

impl FeatureDatabase {
    /// 构造一个空数据库（对照 `FeatureDatabase::FeatureDatabase`）。
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// 遍历库中全部特征（对照 C++ `FeatureDatabase::get_internal_data`；
    /// 供初始化器等外部模块只读遍历）。
    pub fn iter_features(&self) -> impl Iterator<Item = &Feature> {
        self.features.values()
    }

    /// 取出指定 id 的特征（对照 `OpenVINS` `FeatureDatabase::get_feature`）。
    ///
    /// `remove=true` 时从库中移除并转移所有权；`remove=false` 时返回一份克隆
    /// （原版基于 `shared_ptr` 直接返回共享引用，Rust 以所有权模型取代——
    /// 克隆保证调用方无法与原库内容并发改写，需跟随后续变化的请用 `remove=true`）。
    ///
    /// 不存在则返回 `None`。
    pub fn get_feature(&mut self, id: usize, remove: bool) -> Option<Feature> {
        if remove {
            self.features.remove(&id)
        } else {
            self.features.get(&id).cloned()
        }
    }

    /// 克隆指定 id 的特征（对照 `OpenVINS` `FeatureDatabase::get_feature_clone`）。
    ///
    /// 只读、不改动数据库；不存在返回 `None`。
    #[must_use]
    pub fn get_feature_clone(&self, id: usize) -> Option<Feature> {
        self.features.get(&id).cloned()
    }

    /// 当前库内全部特征的 `(id, 引用)` 迭代（诊断/测试用，无序）。
    pub fn iter_features_with_id(&self) -> impl Iterator<Item = (usize, &Feature)> {
        self.features.iter().map(|(k, v)| (*k, v))
    }

    /// 追加一次测量（对照 `OpenVINS` `FeatureDatabase::update_feature`）。
    ///
    /// 若 id 已存在则向对应相机桶追加速度/归一化坐标与时间戳；
    /// 若是新 id 则自动创建并入库。原版返回 `void`，Rust 端用 `&mut self` 表达就地更新。
    #[allow(clippy::too_many_arguments)] // 形参由跨端 API 契约逐一定死，与 C++ 签名一一对应
    pub fn update_feature(
        &mut self,
        id: usize,
        timestamp: f64,
        cam_id: usize,
        u: f32,
        v: f32,
        u_n: f32,
        v_n: f32,
    ) {
        let feat = self.features.entry(id).or_insert_with(|| Feature {
            featid: id,
            ..Feature::default()
        });
        feat.uvs.entry(cam_id).or_default().push(Vector2::new(u, v));
        feat.uvs_norm
            .entry(cam_id)
            .or_default()
            .push(Vector2::new(u_n, v_n));
        feat.timestamps.entry(cam_id).or_default().push(timestamp);
    }

    /// 返回没有比指定时间更新的测量的所有特征
    /// （对照 `OpenVINS` `FeatureDatabase::features_not_containing_newer`）。
    ///
    /// "没有更新的测量" 指任一相机的桶中最近一次测量都不 `>= timestamp`
    /// （即未成功跟踪到最新帧的特征）。`remove=true` 时从库中移除并转移所有权，
    /// 否则返回克隆。`skip_deleted=true` 时跳过 `to_delete` 标记的特征。
    pub fn features_not_containing_newer(
        &mut self,
        timestamp: f64,
        remove: bool,
        skip_deleted: bool,
    ) -> Vec<Feature> {
        self.collect_matching(remove, skip_deleted, |f| {
            !f.timestamps
                .values()
                .any(|ts| ts.last().is_some_and(|&t| t >= timestamp))
        })
    }

    /// 返回含至少一条早于指定时间的测量的所有特征
    /// （对照 `OpenVINS` `FeatureDatabase::features_containing_older`）。
    ///
    /// 判断依据是任一相机桶的**首元素**（即 `at(0)`，非"全局最早一刻"）`< timestamp`
    /// 即命中。其余语义同
    /// [`FeatureDatabase::features_not_containing_newer`] 的 `remove`/`skip_deleted`。
    pub fn features_containing_older(
        &mut self,
        timestamp: f64,
        remove: bool,
        skip_deleted: bool,
    ) -> Vec<Feature> {
        self.collect_matching(remove, skip_deleted, |f| {
            f.timestamps
                .values()
                .any(|ts| ts.first().is_some_and(|&t| t < timestamp))
        })
    }

    /// 返回含指定时刻测量的所有特征
    /// （对照 `OpenVINS` `FeatureDatabase::features_containing`）。
    ///
    /// 判断依据是任一相机桶的测量中恰好存在等于 `timestamp` 的时刻。其余语义同
    /// [`FeatureDatabase::features_not_containing_newer`] 的 `remove`/`skip_deleted`。
    /// 常用于取某滑动窗口克隆时刻的全部测量。
    pub fn features_containing(
        &mut self,
        timestamp: f64,
        remove: bool,
        skip_deleted: bool,
    ) -> Vec<Feature> {
        self.collect_matching(remove, skip_deleted, |f| {
            f.timestamps.values().any(|ts| ts.contains(&timestamp))
        })
    }

    /// 从库中收集满足 `pred` 的特征。
    ///
    /// `skip_deleted=true` 时跳过 `to_delete` 特征；`remove=true` 时取走所有权，
    /// 否则返回克隆。先确定候选 id 再移除，避免借用冲突（语义逐行对照
    /// `FeatureDatabase.cpp` 三处查询的循环体）。
    fn collect_matching(
        &mut self,
        remove: bool,
        skip_deleted: bool,
        pred: impl Fn(&Feature) -> bool,
    ) -> Vec<Feature> {
        if remove {
            let ids: Vec<usize> = self
                .features
                .iter()
                .filter(|(_, f)| !(skip_deleted && f.to_delete))
                .filter(|(_, f)| pred(f))
                .map(|(id, _)| *id)
                .collect();
            ids.into_iter()
                .filter_map(|id| self.features.remove(&id))
                .collect()
        } else {
            self.features
                .iter()
                .filter(|(_, f)| !(skip_deleted && f.to_delete))
                .filter(|(_, f)| pred(f))
                .map(|(_, f)| f.clone())
                .collect()
        }
    }

    /// 删除所有带 `to_delete` 标记的特征（对照 `OpenVINS` `FeatureDatabase::cleanup`）。
    pub fn cleanup(&mut self) {
        self.features.retain(|_, f| !f.to_delete);
    }

    /// 标记一批特征为待删除（对照 C++ 中 `feat->to_delete = true` 的语义）。
    ///
    /// Rust 端 `get_feature(.., remove=false)` 返回克隆，updater 在克隆上置位
    /// `to_delete` 不会回流进库；此方法把 id 集合的删除标记写回库，供
    /// `VioManager` 在更新结束后调用（`cleanup()` 再做物理删除）。
    pub fn mark_deleted<I: IntoIterator<Item = usize>>(&mut self, ids: I) {
        for id in ids {
            if let Some(f) = self.features.get_mut(&id) {
                f.to_delete = true;
            }
        }
    }

    /// 清理每个特征中早于（含）`timestamp` 的测量；测量清空的特征整体删除
    /// （对照 `OpenVINS` `FeatureDatabase::cleanup_measurements`）。
    pub fn cleanup_measurements(&mut self, timestamp: f64) {
        self.purge_empty_after(|f| f.clean_older_measurements(timestamp));
    }

    /// 删除每个特征中恰好发生在 `timestamp` 时刻的测量；若某特征被清空则整体删除
    /// （对照 `OpenVINS` `FeatureDatabase::cleanup_measurements_exact`）。
    ///
    /// 注意：原版实现是调用 `clean_invalid_measurements({timestamp})`，即**删掉**
    /// 该时刻的测量（常用于剔除刚被边缘化克隆时刻的测量），并非"仅保留该时刻"。
    pub fn cleanup_measurements_exact(&mut self, timestamp: f64) {
        let times = [timestamp];
        self.purge_empty_after(|f| f.clean_invalid_measurements(&times));
    }

    /// 先执行 `trim`（就地清理测量的闭包），再把测量数为零的特征删除。
    fn purge_empty_after(&mut self, trim: impl Fn(&mut Feature)) {
        for f in self.features.values_mut() {
            trim(f);
        }
        self.features
            .retain(|_, f| f.timestamps.values().map(Vec::len).sum::<usize>() > 0);
    }

    /// 数据库内特征数量（对照 `OpenVINS` `FeatureDatabase::size`）。
    #[must_use]
    pub fn size(&self) -> usize {
        self.features.len()
    }

    /// 全库最早测量时间戳；空库或无测量时返回 `-1.0`
    /// （对照 `OpenVINS` `FeatureDatabase::get_oldest_timestamp`，原实现用 `-1` 哨兵）。
    #[must_use]
    pub fn get_oldest_timestamp(&self) -> f64 {
        // 取全库最早测量时刻：先展平出各相机桶的首个时间戳，再取最小。
        // `min_by(f64::total_cmp)` 以确定性全序比较（`f64` 未实现 `Ord`）。
        // 注意 `total_cmp` 的边界语义：`-0.0 < +0.0` 且 NaN 有确定位置，
        // 与 C++ 原实现 `oldest_time > at(0)` 的 IEEE 偏序不同——VIO 测量时间戳
        // 无 NaN，±0.0 亦为无效时刻，此处差异仅理论存在，结果等同。
        self.features
            .values()
            .flat_map(|f| f.timestamps.values())
            .filter_map(|ts| ts.first())
            .copied()
            .min_by(f64::total_cmp)
            .unwrap_or(-1.0)
    }

    /// 把 `other` 中比本地更新的测量追加进来
    /// （对照 `OpenVINS` `FeatureDatabase::append_new_measurements`）。
    ///
    /// - 已存在特征：逐相机合并，仅追加本地尚未出现的时间戳（其余字段不变）。
    /// - 不存在特征：复制 `featid`/`timestamps`/`uvs`/`uvs_norm` 入库
    ///   （原版复制的正是这四项，`to_delete`/锚点/三角化不在其列）。
    pub fn append_new_measurements(&mut self, other: &FeatureDatabase) {
        for (id, ofeat) in &other.features {
            match self.features.entry(*id) {
                Entry::Occupied(mut local) => {
                    for cam_id in ofeat.timestamps.keys() {
                        append_cam_measurements(local.get_mut(), ofeat, *cam_id);
                    }
                }
                Entry::Vacant(vacant) => {
                    let feat = Feature {
                        featid: ofeat.featid,
                        timestamps: ofeat.timestamps.clone(),
                        uvs: ofeat.uvs.clone(),
                        uvs_norm: ofeat.uvs_norm.clone(),
                        ..Feature::default()
                    };
                    vacant.insert(feat);
                }
            }
        }
    }

    /// 把 `id_old` 特征重命名为 `id_new`，并把库的索引 key 一并迁移
    /// （对照 `OpenVINS` `TrackBase::change_feat_id` 中操作数据库那段语义）。
    ///
    /// 语义与 C++ 逐行对齐：    /// - 仅当库中存在 `id_old` 才处理；否则为 no-op（不新建，直接返回）。
    /// - 从 `id_old` 键下取出特征、改其 `featid` 为 `id_new`、再以 `id_new` 键插入。
    /// - **覆盖冲突照 C++ `insert`**：若 `id_new` 键已存在，则**不覆盖**已有特征，
    ///   重映射后的这份被丢弃（`id_old` 已删除，`id_new` 的旧特征保留）。
    /// - 仅作用于本数据库的索引表；`TrackBase` 里各相机的点 id 缓存 `ids_last`
    ///   属 track 模块、不在此方法范围内。
    pub fn remap_feature_id(&mut self, id_old: usize, id_new: usize) {
        let Some(mut feat) = self.features.remove(&id_old) else {
            return; // id_old 不存在：对照 C++ 分支，no-op
        };
        feat.featid = id_new;
        self.features.entry(id_new).or_insert(feat);
    }
}

/// 把 `other` 特征在 `cam_id` 桶下的测量并入 `local`（幂等：跳过已存在的时间戳）。
///
/// 对应 `append_new_measurements` 中"已存在特征逐相机合并"的一段，
/// 且与 C++ 相同——先对现有时间戳取快照再判定，故会忠实保留原版对
/// 同桶内重复时间戳的追加行为。
///
/// 去重判定 `snap.contains(&t)` 用 f64 的 `==`（因此 `+0.0` 与 `-0.0` 视为同一时刻），
/// 与 C++ `std::find` 对时间戳的相等比较一致。
fn append_cam_measurements(local: &mut Feature, other: &Feature, cam_id: usize) {
    let Some(other_ts) = other.timestamps.get(&cam_id) else {
        return;
    };
    let Some(other_uv) = other.uvs.get(&cam_id) else {
        return;
    };
    let Some(other_uvn) = other.uvs_norm.get(&cam_id) else {
        return;
    };
    match local.timestamps.entry(cam_id) {
        Entry::Vacant(vacant) => {
            vacant.insert(other_ts.clone());
            local.uvs.insert(cam_id, other_uv.clone());
            local.uvs_norm.insert(cam_id, other_uvn.clone());
        }
        Entry::Occupied(mut occ) => {
            // 快照当前时间戳集合，避免逐条试探时被自身追加污染。
            // `snap.contains(&t)` 采用 f64 `==`（`+0.0 == -0.0`），忠实 C++ `std::find`。
            let snap = occ.get().clone();
            let local_ts = occ.get_mut();
            let local_uv = local.uvs.entry(cam_id).or_default();
            let local_uvn = local.uvs_norm.entry(cam_id).or_default();
            for i in 0..other_ts.len() {
                let t = other_ts[i];
                if !snap.contains(&t) {
                    local_ts.push(t);
                    local_uv.push(other_uv[i]);
                    local_uvn.push(other_uvn[i]);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    // 测试中大量比较精确可复现的浮点值（整数型常数、逐字转录的字面量），
    // 直接相等即可，豁免 pedantic 的 `float_cmp`。
    #![allow(clippy::float_cmp)]

    use super::*;
    use nalgebra::Vector2;

    /// 以给定各相机时间戳构造一个特征（像素/归一化坐标用占位值，保证三桶等长）。
    fn make_feature(id: usize, cam_times: &[(&usize, &[f64])], to_delete: bool) -> Feature {
        let mut f = Feature {
            featid: id,
            to_delete,
            ..Feature::default()
        };
        for (cam_id, times) in cam_times {
            for t in *times {
                f.timestamps.entry(**cam_id).or_default().push(*t);
                f.uvs
                    .entry(**cam_id)
                    .or_default()
                    .push(Vector2::new(1.0, 2.0));
                f.uvs_norm
                    .entry(**cam_id)
                    .or_default()
                    .push(Vector2::new(0.1, 0.2));
            }
        }
        f
    }

    fn cam0_t(f: &Feature) -> &[f64] {
        f.timestamps.get(&0).map_or(&[], Vec::as_slice)
    }

    fn all_times(f: &Feature) -> Vec<f64> {
        f.timestamps
            .values()
            .flat_map(|v| v.iter().copied())
            .collect()
    }

    // ---- CRUD ----

    #[test]
    fn crud_insert_get_clone_remove() {
        let mut db = FeatureDatabase::new();
        assert_eq!(db.size(), 0);
        db.update_feature(7, 1.0, 0, 10.0, 20.0, 0.1, 0.2);
        assert_eq!(db.size(), 1);

        // 克隆不改变库
        let clone = db.get_feature_clone(7);
        let c = clone.unwrap();
        assert_eq!(c.featid, 7);
        assert_eq!(c.anchor_cam_id, -1);
        assert_eq!(cam0_t(&c), &[1.0]);
        assert_eq!(db.size(), 1);

        // get_feature remove=false 亦为克隆，不删
        let g1 = db.get_feature(7, false).unwrap();
        assert_eq!(cam0_t(&g1), &[1.0]);
        assert_eq!(db.size(), 1);

        // remove=true 转移所有权
        let g2 = db.get_feature(7, true).unwrap();
        assert_eq!(g2.featid, 7);
        assert_eq!(db.size(), 0);
        assert!(db.get_feature(7, true).is_none());
        assert!(db.get_feature_clone(7).is_none());
    }

    // ---- remap_feature_id ----

    #[test]
    fn remap_feature_id_renames_key_and_featid() {
        let mut db = FeatureDatabase::new();
        db.update_feature(7, 1.0, 0, 10.0, 20.0, 0.1, 0.2);

        db.remap_feature_id(7, 11);
        assert_eq!(db.size(), 1, "重命名不改变数量");
        assert!(db.get_feature_clone(7).is_none(), "旧 key 被移除");
        let f = db.get_feature_clone(11).unwrap();
        assert_eq!(f.featid, 11, "特征自身的 featid 改为新 id");
        assert_eq!(cam0_t(&f), &[1.0], "测量数据随特征迁移");
    }

    #[test]
    fn remap_feature_id_no_op_when_old_missing() {
        let mut db = FeatureDatabase::new();
        db.update_feature(5, 1.0, 0, 1.0, 1.0, 0.1, 0.1);

        db.remap_feature_id(999, 5); // id_old 不存在 -> no-op
        assert_eq!(db.size(), 1);
        assert!(db.get_feature_clone(5).is_some(), "不新建、不影响既有特征");
        assert!(db.get_feature_clone(999).is_none());
    }

    #[test]
    fn remap_feature_id_conflict_does_not_overwrite_existing_new() {
        // C++ `insert({id_new, feat})` 在 key 已存在时**不覆盖**：id_old 被删、
        // 重映射后的这份被丢弃，保留原 id_new 特征。
        let mut db = FeatureDatabase::new();
        db.update_feature(7, 1.0, 0, 7.0, 7.0, 0.7, 0.7); // 将被重映射
        db.update_feature(11, 2.0, 0, 11.0, 11.0, 1.1, 1.1); // 已存在的 id_new

        db.remap_feature_id(7, 11);
        assert_eq!(db.size(), 1, "冲突时只留一个特征");
        assert!(db.get_feature_clone(7).is_none(), "id_old 已被移除");
        let kept = db.get_feature_clone(11).unwrap();
        assert_eq!(
            cam0_t(&kept),
            &[2.0],
            "保留原 id_new 特征，重映射的那份被丢弃"
        );
        assert_eq!(kept.uvs[&0][0].x, 11.0);
    }

    #[test]
    fn remap_feature_id_same_id_is_idempotent() {
        let mut db = FeatureDatabase::new();
        db.update_feature(7, 1.0, 0, 7.0, 7.0, 0.7, 0.7);
        db.remap_feature_id(7, 7); // id_old == id_new：等同于删除后插回
        assert_eq!(db.size(), 1);
        assert_eq!(db.get_feature_clone(7).unwrap().featid, 7);
    }

    // ---- update_feature ----

    #[test]
    fn update_append_multi_cam_multi_time_and_autocreate() {
        let mut db = FeatureDatabase::new();
        // 新 id 自动创建
        db.update_feature(1, 1.0, 0, 1.0, 2.0, 0.1, 0.2);
        // 同相机追加不同时刻
        db.update_feature(1, 2.0, 0, 3.0, 4.0, 0.3, 0.4);
        // 另一相机
        db.update_feature(1, 3.0, 1, 5.0, 6.0, 0.5, 0.6);
        db.update_feature(2, 9.0, 0, 7.0, 8.0, 0.7, 0.8);

        assert_eq!(db.size(), 2);
        let f = db.get_feature_clone(1).unwrap();
        assert_eq!(cam0_t(&f), &[1.0, 2.0]);
        assert_eq!(f.uvs.get(&0).unwrap()[1].x, 3.0);
        assert_eq!(f.uvs_norm.get(&0).unwrap()[1].y, 0.4);
        assert_eq!(cam0_t(&f).len(), 2);
        assert_eq!(f.timestamps.get(&1).unwrap(), &[3.0]);
        // 三桶等长不变量
        for (cid, ts) in &f.timestamps {
            assert_eq!(ts.len(), f.uvs[cid].len());
            assert_eq!(ts.len(), f.uvs_norm[cid].len());
        }
        assert_eq!(db.get_feature_clone(2).unwrap().featid, 2);
    }

    // ---- 三个查询 ----

    #[test]
    fn not_containing_newer_semantics() {
        let mut db = FeatureDatabase::new();
        db.features
            .insert(1, make_feature(1, &[(&0, &[1.0, 3.0])], false));
        db.features
            .insert(2, make_feature(2, &[(&0, &[1.0, 2.0, 5.0])], false));
        db.features
            .insert(3, make_feature(3, &[(&0, &[1.0])], false));
        db.features
            .insert(4, make_feature(4, &[(&1, &[1.0])], false)); // 相机1最晚=1.0

        // timestamp=4: 特征1(最晚3<4)、3(1<4)、4(1<4) 归为 old；特征2最晚5>=4 除外
        let olds_no_rm = db.features_not_containing_newer(4.0, false, false);
        let mut ids: Vec<_> = olds_no_rm.iter().map(|f| f.featid).collect();
        ids.sort_unstable();
        assert_eq!(ids, vec![1, 3, 4]);
        // remove=false 不改库
        assert_eq!(db.size(), 4);

        // remove=true 语义
        let olds_rm = db.features_not_containing_newer(4.0, true, false);
        assert_eq!(olds_rm.len(), 3);
        assert_eq!(db.size(), 1); // 只剩 2
    }

    #[test]
    fn not_containing_newer_skip_deleted() {
        let mut db = FeatureDatabase::new();
        db.features
            .insert(1, make_feature(1, &[(&0, &[1.0, 3.0])], false));
        db.features
            .insert(2, make_feature(2, &[(&0, &[1.0])], true)); // 删除标记
        db.features
            .insert(3, make_feature(3, &[(&1, &[1.0])], false));

        let olds = db.features_not_containing_newer(4.0, false, true);
        let ids: Vec<_> = olds.iter().map(|f| f.featid).collect();
        assert!(!ids.contains(&2), "skip_deleted 应跳过已删除特征");
        assert!(ids.contains(&1) && ids.contains(&3));
    }

    #[test]
    fn containing_older_uses_oldest_per_camera() {
        let mut db = FeatureDatabase::new();
        // 特征1：相机0 最早3.0（不<2.0），相机1 最早1.0（<2.0）=> 命中
        db.features.insert(
            1,
            make_feature(1, &[(&0, &[3.0, 5.0]), (&1, &[1.0, 6.0])], false),
        );
        // 特征2：最早4.0，不 <2.0 => 不命中
        db.features
            .insert(2, make_feature(2, &[(&0, &[4.0, 8.0])], false));

        let old = db.features_containing_older(2.0, false, false);
        let mut ids: Vec<_> = old.iter().map(|f| f.featid).collect();
        ids.sort_unstable();
        assert_eq!(ids, vec![1]);
        assert_eq!(db.size(), 2);

        let old_rm = db.features_containing_older(2.0, true, false);
        assert_eq!(old_rm.len(), 1);
        assert_eq!(db.size(), 1);
    }

    #[test]
    fn containing_matches_exact_timestamp_across_cameras() {
        let mut db = FeatureDatabase::new();
        db.features
            .insert(1, make_feature(1, &[(&0, &[1.0, 2.0])], false));
        db.features
            .insert(2, make_feature(2, &[(&1, &[2.0, 5.0])], false));
        db.features
            .insert(3, make_feature(3, &[(&0, &[3.0])], false));

        let hit = db.features_containing(2.0, false, false);
        let mut ids: Vec<_> = hit.iter().map(|f| f.featid).collect();
        ids.sort_unstable();
        assert_eq!(ids, vec![1, 2]); // 特征1、2 都有时刻2.0
        assert_eq!(db.size(), 3);

        let hit_rm = db.features_containing(2.0, true, false);
        assert_eq!(hit_rm.len(), 2);
        assert_eq!(db.size(), 1);
    }

    #[test]
    fn containing_skip_deleted_interaction() {
        let mut db = FeatureDatabase::new();
        db.features
            .insert(1, make_feature(1, &[(&0, &[1.0])], true));
        db.features
            .insert(2, make_feature(2, &[(&0, &[1.0])], false));

        // skip_deleted=true 时仅命中未删除的
        let hit = db.features_containing(1.0, false, true);
        let ids: Vec<_> = hit.iter().map(|f| f.featid).collect();
        assert_eq!(ids, vec![2]);

        // skip_deleted=false 时包含已删除特征
        let hit_all = db.features_containing(1.0, false, false);
        assert_eq!(hit_all.len(), 2);
    }

    #[test]
    fn containing_older_hits_other_camera_when_first_cam_first_ge_t() {
        // 审查场景：某相机桶首元素 >= t，但另一相机桶更早。
        // C++ `contains_older` 逐相机桶判 `first < timestamp`，任一命中即成立
        // （等价于全局最早时刻 < t）。cam0 first=6.0(>=5.0)，cam1 first=3.0(<5.0)。
        let mut db = FeatureDatabase::new();
        db.features.insert(
            1,
            make_feature(1, &[(&0, &[6.0, 7.0]), (&1, &[3.0])], false),
        );
        // 反例：所有相机桶首元素都不 <5.0
        db.features.insert(
            2,
            make_feature(2, &[(&0, &[6.0, 7.0]), (&1, &[5.0, 8.0])], false),
        );

        let old = db.features_containing_older(5.0, false, false);
        let mut ids: Vec<_> = old.iter().map(|f| f.featid).collect();
        ids.sort_unstable();
        assert_eq!(ids, vec![1], "任一相机桶更早即应命中");
    }

    #[test]
    fn query_boundary_equal_to_timestamp() {
        // 审查场景：三个查询在 `timestamp == t` 边界的包含/排除行为（t=5.0）。
        // 特征A 唯一测量恰为 5.0：
        //   not_containing_newer：最晚=5.0 >= 5.0 -> 有更新的测量 -> 排除
        //   containing_older    ：first=5.0，5.0<5.0 为假 -> 不排除的更早测量 -> 排除
        //   containing          ：含 5.0 -> 命中
        // 特征B 全部 >5.0：三者均排除。
        // 特征C 全部 <5.0：not_containing_newer 命中、containing_older 命中、containing 排除。
        let mut db = FeatureDatabase::new();
        db.features
            .insert(1, make_feature(1, &[(&0, &[5.0])], false));
        db.features
            .insert(2, make_feature(2, &[(&1, &[6.0])], false));
        db.features
            .insert(3, make_feature(3, &[(&0, &[4.0])], false));

        // not_containing_newer：仅最近一次测量 <5.0 的特征（特征C）
        let newer = db.features_not_containing_newer(5.0, false, false);
        let mut nids: Vec<_> = newer.iter().map(|f| f.featid).collect();
        nids.sort_unstable();
        assert_eq!(nids, vec![3], "最晚==t 不算更新测量，故特征A 被排除");

        // containing_older：仅最早测量 <5.0 的特征（特征C；特征A first==t 的排除）
        let older = db.features_containing_older(5.0, false, false);
        let mut oids: Vec<_> = older.iter().map(|f| f.featid).collect();
        oids.sort_unstable();
        assert_eq!(oids, vec![3], "首元素==t 不算更早，故特征A 被排除");

        // containing：仅恰含 5.0 的特征（特征A）
        let exact = db.features_containing(5.0, false, false);
        let mut eids: Vec<_> = exact.iter().map(|f| f.featid).collect();
        eids.sort_unstable();
        assert_eq!(eids, vec![1]);
    }

    #[test]
    fn query_tolerates_missing_camera_bucket() {
        // 审查场景：特征只有 cam0 桶、根本没有 cam1 桶，三查询遍历 `timestamps` 时
        // 不应因缺失的相机而 panic 或误判；clean_older 同样只动既有桶。
        let mut db = FeatureDatabase::new();
        db.features
            .insert(1, make_feature(1, &[(&0, &[7.0])], false)); // 无 cam1

        // 缺桶相机不产生时间向量，::values() 只含 cam0
        assert_eq!(db.get_feature_clone(1).unwrap().timestamps.len(), 1);

        let n = db.features_not_containing_newer(9.0, false, false);
        assert_eq!(n.len(), 1, "仅 cam0 的桶参与判断");
        let o = db.features_containing_older(8.0, false, false);
        assert_eq!(o.len(), 1);
        let c = db.features_containing(7.0, false, false);
        assert_eq!(c.iter().map(|f| f.featid).collect::<Vec<_>>(), vec![1]);

        // clean_older 只清理既有桶，不触碰缺失的 cam1
        db.cleanup_measurements(8.0); // <=8 全删 -> 特征1 清空被整体删除
        assert_eq!(db.size(), 0);
    }

    // ---- cleanup ----

    #[test]
    fn cleanup_removes_to_delete_marked() {
        let mut db = FeatureDatabase::new();
        db.features
            .insert(1, make_feature(1, &[(&0, &[1.0])], false));
        db.features
            .insert(2, make_feature(2, &[(&0, &[1.0])], true));
        db.features
            .insert(3, make_feature(3, &[(&0, &[1.0])], true));
        assert_eq!(db.size(), 3);
        db.cleanup();
        assert_eq!(db.size(), 1);
        assert!(db.get_feature_clone(1).is_some());
        assert!(db.get_feature_clone(2).is_none());
        assert!(db.get_feature_clone(3).is_none());
    }

    #[test]
    fn cleanup_measurements_removes_old_and_drops_empty() {
        let mut db = FeatureDatabase::new();
        db.features
            .insert(1, make_feature(1, &[(&0, &[1.0, 2.0, 3.0])], false));
        db.features
            .insert(2, make_feature(2, &[(&0, &[1.0])], false)); // 全被清
        db.features
            .insert(3, make_feature(3, &[(&0, &[0.5]), (&1, &[2.5])], false));

        db.cleanup_measurements(2.0); // 删 <=2.0
        assert_eq!(db.size(), 2); // 特征2 空被删
        let f1 = db.get_feature_clone(1).unwrap();
        assert_eq!(cam0_t(&f1), &[3.0]);
        let f3 = db.get_feature_clone(3).unwrap();
        assert_eq!(cam0_t(&f3), &[]);
        assert_eq!(f3.timestamps.get(&1).unwrap(), &[2.5]);
    }

    #[test]
    fn cleanup_measurements_exact_removes_that_timestamp() {
        let mut db = FeatureDatabase::new();
        db.features
            .insert(1, make_feature(1, &[(&0, &[1.0, 2.0, 3.0])], false));
        db.features
            .insert(2, make_feature(2, &[(&0, &[1.0, 2.0])], false));
        db.features
            .insert(3, make_feature(3, &[(&0, &[2.0])], false)); // 仅有该时刻 -> 清空删
        db.features
            .insert(4, make_feature(4, &[(&0, &[7.0])], false)); // 不含该时刻 -> 保留

        // 语义：删除恰好发生在 2.0 的测量
        db.cleanup_measurements_exact(2.0);
        assert_eq!(db.size(), 3); // 特征3 被清空删除
        assert_eq!(cam0_t(&db.get_feature_clone(1).unwrap()), &[1.0, 3.0]);
        assert_eq!(cam0_t(&db.get_feature_clone(2).unwrap()), &[1.0]);
        assert_eq!(cam0_t(&db.get_feature_clone(4).unwrap()), &[7.0]);
        assert!(db.get_feature_clone(3).is_none());
    }

    // ---- append_new_measurements ----

    #[test]
    fn append_merges_new_measurements_only() {
        let mut db = FeatureDatabase::new();
        db.update_feature(1, 1.0, 0, 1.0, 1.0, 0.1, 0.1); // 本地已有时刻1.0
        db.update_feature(1, 3.0, 0, 3.0, 3.0, 0.3, 0.3);

        let mut other = FeatureDatabase::new();
        other.update_feature(1, 1.0, 0, 1.0, 1.0, 0.1, 0.1); // 重复 -> 跳过
        other.update_feature(1, 2.0, 0, 2.0, 2.0, 0.2, 0.2); // 新增 -> 追加
        other.update_feature(1, 4.0, 1, 4.0, 4.0, 0.4, 0.4); // 新相机 -> 整体拷贝
        other.update_feature(2, 9.0, 0, 9.0, 9.0, 0.9, 0.9); // 新特征 -> 拷贝

        db.append_new_measurements(&other);
        assert_eq!(db.size(), 2);
        let f1 = db.get_feature_clone(1).unwrap();
        assert_eq!(cam0_t(&f1), &[1.0, 3.0, 2.0]);
        assert_eq!(f1.uvs.get(&0).unwrap()[1].x, 3.0); // 本地顺序保留
        assert_eq!(f1.timestamps.get(&1).unwrap(), &[4.0]);
        // 锚点等不变量不被覆盖
        assert_eq!(f1.anchor_cam_id, -1);
        let f2 = db.get_feature_clone(2).unwrap();
        assert_eq!(cam0_t(&f2), &[9.0]);
    }

    #[test]
    fn append_copied_feature_keeps_only_measurement_fields() {
        let mut db = FeatureDatabase::new();

        let mut other = FeatureDatabase::new();
        let mut feat = make_feature(5, &[(&0, &[1.0])], true);
        feat.anchor_cam_id = 2;
        feat.p_FinG = Vector3::new(1.0, 2.0, 3.0);
        other.features.insert(5, feat);

        db.append_new_measurements(&other);
        let f = db.get_feature_clone(5).unwrap();
        // 原版新特征只复制 featid/timestamps/uvs/uvs_norm
        assert_eq!(f.featid, 5);
        assert_eq!(cam0_t(&f), &[1.0]);
        assert!(!f.to_delete);
        assert_eq!(f.anchor_cam_id, -1, "锚点/三角化不随 append 拷贝");
        assert_eq!(f.p_FinG, Vector3::zeros());
    }

    #[test]
    fn append_preserves_three_bucket_length_invariant() {
        // G4：append 后，任一特征的每个相机桶三向（timestamps/uvs/uvs_norm）等长。
        // 覆盖全部三条路径：新相机整体拷贝(Vacant)、已存在相机逐条合并(Occupied)、
        // 整个新特征拷贝(Vacant with new id)。
        let mut db = FeatureDatabase::new();
        db.update_feature(1, 1.0, 0, 1.0, 1.0, 0.1, 0.1);
        db.update_feature(1, 3.0, 0, 3.0, 3.0, 0.3, 0.3); // cam0 本地两时刻

        let mut other = FeatureDatabase::new();
        other.update_feature(1, 2.0, 0, 2.0, 2.0, 0.2, 0.2); // cam0 合并(Occupied)
        other.update_feature(1, 4.0, 1, 4.0, 4.0, 0.4, 0.4); // cam1 新相机(Vacant)
        other.update_feature(2, 9.0, 0, 9.0, 9.0, 0.9, 0.9); // 新特征整体拷贝

        db.append_new_measurements(&other);

        assert_eq!(db.size(), 2);
        for f in db.features.values() {
            for (cid, ts) in &f.timestamps {
                assert_eq!(
                    ts.len(),
                    f.uvs[cid].len(),
                    "特征{} cam{cid} uvs/timestamps 应等长",
                    f.featid
                );
                assert_eq!(
                    ts.len(),
                    f.uvs_norm[cid].len(),
                    "特征{} cam{cid} uvs_norm/timestamps 应等长",
                    f.featid
                );
            }
        }
        let f1 = db.get_feature_clone(1).unwrap();
        assert_eq!(cam0_t(&f1), &[1.0, 3.0, 2.0]);
        assert_eq!(f1.timestamps[&1], &[4.0]);
    }

    #[test]
    fn get_feature_remove_false_returns_independent_clone() {
        // G5：`get_feature(id, false)` 返回克隆，调用方改写不得影响库内原对象；
        // 库内对象只能是 `remove=true` 才被取出并脱离库。
        let mut db = FeatureDatabase::new();
        db.update_feature(1, 1.0, 0, 1.0, 2.0, 0.1, 0.2);

        let mut clone = db.get_feature(1, false).unwrap();
        assert_eq!(db.size(), 1, "remove=false 不应改变库大小");
        clone.to_delete = true;
        clone.featid = 999;
        clone.uvs.get_mut(&0).unwrap().push(Vector2::new(8.0, 8.0));
        clone.timestamps.get_mut(&0).unwrap().push(5.0);

        // 库内原对象不受克隆改写影响
        let stored = db.get_feature_clone(1).unwrap();
        assert_eq!(stored.featid, 1, "克隆改 featid 不得外泄");
        assert!(!stored.to_delete, "克隆改 to_delete 不得外泄");
        assert_eq!(cam0_t(&stored), &[1.0], "克隆追加测量不得外泄");

        // 多个 remove=false 之间也应彼此独立（都是深克隆）
        let mut second = db.get_feature(1, false).unwrap();
        second.timestamps.get_mut(&0).unwrap().push(6.0);
        assert_eq!(cam0_t(&db.get_feature_clone(1).unwrap()), &[1.0]);
    }

    // ---- 空库与边界 ----

    #[test]
    fn empty_db_behaviour() {
        let mut db = FeatureDatabase::new();
        assert_eq!(db.size(), 0);
        assert_eq!(db.get_oldest_timestamp(), -1.0);
        assert!(db.get_feature_clone(0).is_none());
        assert!(db.get_feature(0, true).is_none());
        assert!(
            db.features_not_containing_newer(1.0, false, false)
                .is_empty()
        );
        assert!(db.features_containing_older(1.0, false, false).is_empty());
        assert!(db.features_containing(1.0, false, false).is_empty());
        db.cleanup();
        db.cleanup_measurements(1.0);
        db.cleanup_measurements_exact(1.0);
        assert_eq!(db.size(), 0);
    }

    #[test]
    fn get_oldest_timestamp_across_cameras_and_no_meas() {
        let mut db = FeatureDatabase::new();
        db.features
            .insert(1, make_feature(1, &[(&0, &[5.0, 9.0])], false));
        db.features
            .insert(2, make_feature(2, &[(&1, &[2.0, 3.0])], false));
        assert_eq!(db.get_oldest_timestamp(), 2.0);

        // 有特征但无测量的相机不影响
        db.features.insert(
            3,
            Feature {
                featid: 3,
                timestamps: HashMap::from([(0, Vec::new())]),
                uvs: HashMap::new(),
                uvs_norm: HashMap::new(),
                ..Feature::default()
            },
        );
        assert_eq!(db.get_oldest_timestamp(), 2.0);
    }

    // ---- Feature 清理 ----

    #[test]
    fn feature_clean_old_invalid_older() {
        let mut f = Feature {
            timestamps: HashMap::from([(0, vec![1.0, 2.0, 3.0])]),
            uvs: HashMap::from([(
                0,
                vec![
                    Vector2::new(1.0, 1.0),
                    Vector2::new(2.0, 2.0),
                    Vector2::new(3.0, 3.0),
                ],
            )]),
            uvs_norm: HashMap::from([(
                0,
                vec![
                    Vector2::new(0.1, 0.1),
                    Vector2::new(0.2, 0.2),
                    Vector2::new(0.3, 0.3),
                ],
            )]),
            ..Feature::default()
        };

        // clean_old：保留 valid 中出现的时刻
        f.clean_old_measurements(&[1.0, 3.0]);
        assert_eq!(cam0_t(&f), &[1.0, 3.0]);
        assert_eq!(f.uvs[&0][1].x, 3.0);

        // clean_invalid：移除指定时刻
        f.clean_invalid_measurements(&[1.0]);
        assert_eq!(cam0_t(&f), &[3.0]);

        // clean_older：移除 <=2.0（当前唯一为 3.0，不受影响）
        f.clean_older_measurements(2.0);
        assert_eq!(cam0_t(&f), &[3.0]);
        f.clean_older_measurements(3.0);
        assert!(cam0_t(&f).is_empty());
        // 三桶等长不变量
        assert_eq!(f.uvs[&0].len(), 0);
        assert_eq!(f.uvs_norm[&0].len(), 0);
    }

    #[test]
    fn feature_clean_multi_camera_independent() {
        let mut f = Feature {
            timestamps: HashMap::from([(0, vec![1.0, 2.0]), (1, vec![5.0])]),
            uvs: HashMap::from([
                (0, vec![Vector2::new(1.0, 1.0), Vector2::new(2.0, 2.0)]),
                (1, vec![Vector2::new(5.0, 5.0)]),
            ]),
            uvs_norm: HashMap::from([
                (0, vec![Vector2::new(0.1, 0.1), Vector2::new(0.2, 0.2)]),
                (1, vec![Vector2::new(0.5, 0.5)]),
            ]),
            ..Feature::default()
        };
        f.clean_older_measurements(4.0); // 相机0 的 <=4 全删，相机1 的 5 保留
        assert_eq!(cam0_t(&f), &[]);
        assert_eq!(f.timestamps[&1], &[5.0]);
        assert_eq!(
            all_times(&f).len(),
            f.uvs.values().map(Vec::len).sum::<usize>()
        );
    }
}
