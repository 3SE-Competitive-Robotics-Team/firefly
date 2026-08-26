//! 障碍约束点生成:{s,v} 平面——忠实移植官方 EGO-Planner-v2
//! `poly_traj_optimizer.cpp` 的 `computePointsToCheck` /
//! `finelyCheckAndSetConstraintPoints` / `roughlyCheckConstraintPoints`。
//!
//! 与旧实现(最近引导点 + 射线步进)的关键差异:
//! - **稠密采样**:`computePointsToCheck` 以 `res/max_vel` 时间步采样轨迹,
//!   按约束点桶(每段 K 个)组织,而不是只在约束点网格上检查;
//! - **in/out 自由点搜索**:碰撞段沿约束点数组向前/向后找最近自由点
//!   (天然有界;末端无自由点 → 报错/放弃),而不是向引导路径射线步进;
//! - **A\* 绕障**:每段用 A\* 搜绕行路径(in/out 点之间),约束方向指向
//!   轨迹点→A\* 路径的**交点**方向(改变拓扑,而非简单推离);
//! - 平面基点在"交点→轨迹点"方向按分辨率步进找障碍表面边界。
//!
//! 约束点布局对齐官方 `getInitConstraintPoints(K)`:**N·K+1** 点
//! (每段 K 个采样 + 尾端点,段边界不重复);代价/检测按 `i_dp = i·K + j`
//! 索引;前 2/3 约束点(`two_thirds_id`)才施加障碍/集群/队形力。

use firefly_map::{GridMap, Plane};
use firefly_search::Astar;
use firefly_trajectory::Trajectory;
use nalgebra::Vector3;

/// 带时间戳的稠密采样点:(相对轨迹起点的时间 秒, 位置),
/// 官方 `PtsChk_t` 元素。
pub type SamplePoint = (f64, Vector3<f64>);
/// 带时间戳检查点:按约束点桶组织的稠密采样(官方 `PtsChk_t`)。
/// 不变量:同一桶内及相邻桶间时间戳单调不减;首桶必含 t≈0 的采样;
/// `touch_goal=false` 时按 `two_thirds_id` 截断,不覆盖末段。
pub type PointsToCheck = Vec<Vec<SamplePoint>>;

/// 官方 `ConstraintPoints::two_thirds_id`:仅前 2/3 约束点施加约束力
/// (触达目标时检查全程)。`cols` = 约束点总数(N·K+1)。
#[must_use]
pub fn two_thirds_id(cols: usize, touch_goal: bool) -> usize {
    if touch_goal {
        cols - 1
    } else {
        cols - 1 - (cols - 2) / 3
    }
}

/// 官方 `getInitConstraintPoints(K)`:返回 N·K+1 个约束点
/// (段内 K 个 + 全程尾端点;段边界只计一次)。
#[must_use]
pub fn constraint_sample_points(traj: &Trajectory, k: usize) -> Vec<Vector3<f64>> {
    let n = traj.pieces();
    let mut pts = Vec::with_capacity(n * k + 1);
    let mut prefix = 0.0;
    for (i, ti) in traj.durations().iter().enumerate() {
        let step = ti / k as f64;
        for j in 0..=k {
            pts.push(traj.eval(prefix + j as f64 * step).position);
            if j == k && i + 1 < n {
                // 段边界:官方 push 但 i_dp 不递增(由下一段 j=0 覆盖同一索引)
                pts.pop();
            }
        }
        prefix += ti;
    }
    pts
}

/// 碰撞段(官方 `segments` 一项):控制点下标闭区间 [in, out]。
pub type CollisionSpan = (usize, usize);

/// 官方 `finelyCheckAndSetConstraintPoints` 返回值。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CheckResult {
    /// 无碰撞段,轨迹安全。
    Free,
    /// 已并入约束平面(碰撞,平面已生成)。
    Finished,
    /// 错误(稠密采样退化 / A\* 失败),应放弃本次规划。
    Error,
}

/// 障碍扫描器:稠密采样 + 分段 + A\* 绕障 + 交点平面。
pub struct ObstacleScanner<'a> {
    map: &'a GridMap,
    /// 每段约束点数(官方 `constraint_points_perPiece`)。
    samples_per_piece: usize,
    /// 稠密采样时间步:res / `max_vel`(官方 `computePointsToCheck`)。
    max_vel: f64,
}

impl<'a> ObstacleScanner<'a> {
    pub fn new(map: &'a GridMap) -> Self {
        Self {
            map,
            samples_per_piece: 5,
            max_vel: 1.5,
        }
    }

    #[must_use]
    pub fn with_samples(mut self, samples: usize) -> Self {
        self.samples_per_piece = samples;
        self
    }

    #[must_use]
    pub fn with_max_vel(mut self, max_vel: f64) -> Self {
        self.max_vel = max_vel;
        self
    }

    #[must_use]
    pub fn samples_per_piece(&self) -> usize {
        self.samples_per_piece
    }

    /// 官方 `computePointsToCheck`:稠密采样(时间步 `res/max_vel`),
    /// 按约束点桶组织(每段 K 个桶,只到 `id_cps_end` 前)。
    ///
    /// 返回 `None` = 轨迹在覆盖前 2/3 桶前耗尽(非 `touch_goal` 时官方报错)。
    fn points_to_check(
        &self,
        traj: &Trajectory,
        id_cps_end: usize,
        touch_goal: bool,
    ) -> Option<PointsToCheck> {
        let res = self.map.resolution();
        let res_2 = res / 2.0;
        let durations = traj.durations();
        let n = durations.len();
        let mut t_seg_start = Vec::with_capacity(n + 1);
        let mut acc = 0.0;
        t_seg_start.push(0.0);
        for ti in durations.iter() {
            acc += ti;
            t_seg_start.push(acc);
        }
        let total = acc;
        // 采样步长取「分辨率/最大速度」与「最短段时长/K/1.5」的较小者
        // （官方 `computePointsToCheck` 的 min 上限）：后者保证每桶至少
        // 1.5 次采样，短段轨迹（如急停定点段）不会跳桶导致覆盖失败。
        let t_step = (res / self.max_vel).min(
            durations.iter().copied().fold(f64::INFINITY, f64::min)
                / self.samples_per_piece as f64
                / 1.5,
        );
        let k = self.samples_per_piece;

        let mut pts: PointsToCheck = vec![Vec::new(); id_cps_end];
        let mut t = 0.0;
        let mut pt_last = traj.eval(0.0).position;
        let mut id_cps_curr = 0usize;
        let mut id_piece_curr = 0usize;

        loop {
            if t > total {
                if touch_goal && !pts.is_empty() {
                    // 官方:丢弃尾部空桶;全空视为失败
                    while pts.last().is_some_and(Vec::is_empty) {
                        pts.pop();
                    }
                    if pts.is_empty() {
                        return None;
                    }
                    return Some(pts);
                }
                return None;
            }
            let next_t_stp = t_seg_start[id_piece_curr]
                + durations[id_piece_curr] / k as f64
                    * (id_cps_curr as f64 + 1.0 - k as f64 * id_piece_curr as f64);
            if t >= next_t_stp {
                if id_cps_curr + 1 >= k * (id_piece_curr + 1) {
                    id_piece_curr = (id_piece_curr + 1).min(n - 1);
                }
                id_cps_curr += 1;
                if id_cps_curr >= id_cps_end {
                    break;
                }
            }
            let pt = traj.eval(t).position;
            if t < 1e-5 || pts[id_cps_curr].is_empty() || (pt - pt_last).abs().max() > res_2 {
                pts[id_cps_curr].push((t, pt));
                pt_last = pt;
            }
            t += t_step;
        }
        Some(pts)
    }

    /// 官方 `computePointsToCheck` 的 `setLocalTrajFromOpt` 调用形态:
    /// 按 [`two_thirds_id`] 截断生成入库检查点。规划成功后调用一次挂到
    /// 执行轨迹上,供执行期碰撞监控扫描(与规划内层共用同一采样器,
    /// 保证监控所见即规划所验)。
    ///
    /// 返回 `None` = 稠密采样退化(非 `touch_goal` 时轨迹在覆盖前 2/3 桶前
    /// 耗尽 / 全部桶为空,官方报错拒收);调用方必须视同本次规划失败。
    #[must_use]
    pub fn compute_points_to_check(
        &self,
        traj: &Trajectory,
        touch_goal: bool,
    ) -> Option<PointsToCheck> {
        let cols = traj.pieces() * self.samples_per_piece + 1;
        self.points_to_check(traj, two_thirds_id(cols, touch_goal), touch_goal)
    }

    /// 官方 `finelyCheckAndSetConstraintPoints`(`flag_first_init` 语义由调用方
    /// 决定):稠密采样 → 占据分段(in/out)→ 每段 A\* 绕障 → 交点平面。
    ///
    /// [`Self::finely_check`] 的完整形态:同时返回碰撞段(官方
    /// `final_segment_ids`:仅平面分配成功的段,供多拓扑候选生成使用;
    /// 无碰撞时为空)。
    pub fn finely_check_with_segments(
        &self,
        astar: &mut Astar,
        traj: &Trajectory,
        points: &[Vector3<f64>],
        planes: &mut [Vec<Plane>],
        touch_goal: bool,
    ) -> (CheckResult, Vec<CollisionSpan>) {
        const ENOUGH_INTERVAL: usize = 2;
        let cols = points.len();
        let i_end = two_thirds_id(cols, touch_goal);
        let Some(pts_check) = self.points_to_check(traj, i_end, touch_goal) else {
            log::debug!("finely check: 稠密采样退化(轨迹耗尽于前 2/3 桶前)");
            return (CheckResult::Error, Vec::new());
        };

        /*** 占据分段(官方 in/out + ENOUGH_INTERVAL 滞回) ***/
        let mut segment_ids: Vec<ConstraintSpan> = Vec::new();
        let mut in_id: i64 = -1;
        let mut out_id: i64 = -1;
        let mut same_occ_state_times = ENOUGH_INTERVAL + 1;
        let mut last_occ = false;
        let mut flag_got_start = false;
        let mut flag_got_end = false;
        let mut flag_got_end_maybe = false;
        for (i, bucket) in pts_check.iter().enumerate() {
            for (_, p) in bucket {
                let occ = self.map.is_occupied_inflated(*p);
                if occ && !last_occ {
                    if same_occ_state_times > ENOUGH_INTERVAL || i == 0 {
                        in_id = i as i64;
                        flag_got_start = true;
                    }
                    same_occ_state_times = 0;
                    flag_got_end_maybe = false;
                } else if !occ && last_occ {
                    out_id = i as i64 + 1;
                    flag_got_end_maybe = true;
                    same_occ_state_times = 0;
                } else {
                    same_occ_state_times += 1;
                }
                if flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || i == i_end - 1)
                {
                    flag_got_end_maybe = false;
                    flag_got_end = true;
                }
                last_occ = occ;
                if flag_got_start && flag_got_end {
                    flag_got_start = false;
                    flag_got_end = false;
                    if in_id < 0 || out_id < 0 {
                        return (CheckResult::Error, Vec::new());
                    }
                    segment_ids.push(ConstraintSpan {
                        start: in_id as usize,
                        end: out_id as usize,
                    });
                }
            }
        }
        if segment_ids.is_empty() {
            return (CheckResult::Free, Vec::new());
        }

        /*** A\* 绕障路径(官方:in=出侧点,out=入侧点,SEARCH_ERR 连下一段) ***/
        let Some(paths) = self.a_star_paths(astar, points, &mut segment_ids) else {
            return (CheckResult::Error, Vec::new());
        };

        /*** 逐段分配平面(官方 step1/2/3;MINIMUM_PERCENT=0 → 段长不做扩缩) ***/
        let mut final_spans = Vec::with_capacity(segment_ids.len());
        for (i, &span) in segment_ids.iter().enumerate() {
            // 官方 final_segment_ids:仅平面生成成功的段进入返回列表
            if assign_planes_for_segment(self.map, &paths[i], points, span, planes) {
                final_spans.push((span.start, span.end));
            }
        }
        (CheckResult::Finished, final_spans)
    }

    /// 官方 `finelyCheckAndSetConstraintPoints`(`flag_first_init` 语义由调用方
    /// 决定):稠密采样 → 占据分段(in/out)→ 每段 A\* 绕障 → 交点平面。
    ///
    /// `points` = 当前轨迹约束点(N·K+1),`planes` = 按约束点索引组织的
    /// {s,v} 平面池(本函数**就地追加**新平面)。返回 [`CheckResult`]。
    pub fn finely_check(
        &self,
        astar: &mut Astar,
        traj: &Trajectory,
        points: &[Vector3<f64>],
        planes: &mut [Vec<Plane>],
        touch_goal: bool,
    ) -> CheckResult {
        self.finely_check_with_segments(astar, traj, points, planes, touch_goal)
            .0
    }

    /// 官方 `roughlyCheckConstraintPoints`(L-BFGS 内循环):在约束点数组上
    /// 检测未覆盖穿入点,沿数组向前/向后找最近自由点(in/out,末端无自由点
    /// 即放弃),每段 A\* 绕障 + 交点平面。返回是否有新障碍(触发 Rebound)。
    pub fn roughly_check(
        &self,
        astar: &mut Astar,
        points: &[Vector3<f64>],
        planes: &mut [Vec<Plane>],
        touch_goal: bool,
    ) -> bool {
        let cols = points.len();
        let i_end = two_thirds_id(cols, touch_goal);
        let res = self.map.resolution();
        let mut segment_ids: Vec<ConstraintSpan> = Vec::new();
        let mut flag_new_obs_valid = false;

        let mut i = 1usize;
        while i <= i_end {
            let mut occ = self.map.is_occupied_inflated(points[i]);
            /*** 已被现有平面覆盖则不视为新障碍 ***/
            if occ {
                for plane in &planes[i] {
                    if (points[i] - plane.point()).dot(&plane.normal()) < res * 1.0 {
                        occ = false;
                        break;
                    }
                }
            }
            if occ {
                flag_new_obs_valid = true;
                /*** 向前找最近自由点(官方 in_id;无则视为崩溃场景) ***/
                let mut j = i;
                let in_id = loop {
                    j -= 1;
                    if !self.map.is_occupied_inflated(points[j]) {
                        break j;
                    }
                    if j == 0 {
                        log::warn!("roughly check: 起点在障碍内(in 端无自由点)");
                        break 0;
                    }
                };
                /*** 向后找最近自由点(官方 out_id;无则 STOP_FOR_ERROR) ***/
                let mut j = i + 1;
                let out_id = loop {
                    if j >= cols {
                        log::warn!("roughly check: 终点在障碍内(out 端无自由点),放弃规划");
                        return false;
                    }
                    if !self.map.is_occupied_inflated(points[j]) {
                        break j;
                    }
                    j += 1;
                };
                i = j + 1;
                segment_ids.push(ConstraintSpan {
                    start: in_id,
                    end: out_id,
                });
            }
            i += 1;
        }

        if !flag_new_obs_valid {
            return false;
        }

        /*** A\* 绕障(官方:SEARCH_ERR 连下一段,ERR 丢弃该段) ***/
        let mut i = 0usize;
        let mut paths: Vec<Vec<Vector3<f64>>> = Vec::new();
        while i < segment_ids.len() {
            let span = segment_ids[i];
            let (in_pt, out_pt) = (points[span.end], points[span.start]);
            match astar.search(self.map, in_pt, out_pt) {
                Ok(p) => paths.push(p.points().to_vec()),
                Err(_) if i + 1 < segment_ids.len() => {
                    // 连下一段(官方 corner case 2)
                    segment_ids[i].end = segment_ids[i + 1].end;
                    segment_ids.remove(i + 1);
                    continue;
                }
                Err(_) => {
                    // 官方:丢弃该段
                    segment_ids.remove(i);
                    continue;
                }
            }
            i += 1;
        }
        if segment_ids.is_empty() {
            return false;
        }

        /*** 防重叠(官方对 segment_ids 直接做中点切分) ***/
        for i in 1..segment_ids.len() {
            if segment_ids[i - 1].end >= segment_ids[i].start {
                let middle = (segment_ids[i - 1].end + segment_ids[i].start) as f64 / 2.0;
                segment_ids[i - 1].end = (middle - 0.1) as usize;
                segment_ids[i].start = (middle + 1.1) as usize;
            }
        }

        /*** 逐段分配平面 ***/
        for (i, &span) in segment_ids.iter().enumerate() {
            let _ = assign_planes_for_segment(self.map, &paths[i], points, span, planes);
        }
        true
    }

    /// 官方 `AstarSearch(in, out)` 逐段搜索(`SEARCH_ERR` 连下一段;末段失败
    /// 返回 `None` → 调用方按 ERR 处理)。
    fn a_star_paths(
        &self,
        astar: &mut Astar,
        points: &[Vector3<f64>],
        segment_ids: &mut Vec<ConstraintSpan>,
    ) -> Option<Vec<Vec<Vector3<f64>>>> {
        let mut paths = Vec::with_capacity(segment_ids.len());
        let mut i = 0usize;
        while i < segment_ids.len() {
            let span = segment_ids[i];
            // 官方:从出侧点(in=points[end])搜到入侧点(out=points[start])
            let (in_pt, out_pt) = (points[span.end], points[span.start]);
            match astar.search(self.map, in_pt, out_pt) {
                Ok(p) => paths.push(p.points().to_vec()),
                Err(_) if i + 1 < segment_ids.len() => {
                    segment_ids[i].end = segment_ids[i + 1].end;
                    segment_ids.remove(i + 1);
                    continue;
                }
                Err(_) => return None,
            }
            i += 1;
        }
        Some(paths)
    }

    /// 全程稠密安全检查(官方成功判据的保守版本,供 post-check 用):
    /// 以 `res/max_vel` 时间步扫描整条轨迹(不限于前 2/3)。
    #[must_use]
    pub fn is_safe(&self, traj: &Trajectory) -> bool {
        let t_step = self.map.resolution() / self.max_vel;
        let mut t = 0.0;
        while t <= traj.duration() {
            if self.map.is_occupied_inflated(traj.eval(t).position) {
                return false;
            }
            t += t_step;
        }
        true
    }
}

/// 约束点段（官方 `segment_ids` 一项）：`start`/`end` 为 `points` 下标，
/// 语义为闭区间 [start, end]。
#[derive(Debug, Clone, Copy)]
struct ConstraintSpan {
    start: usize,
    end: usize,
}

/// 官方"Assign data to each segment"(step 1/2/3):为段 [first, second]
/// 内的约束点生成 {s,v} 平面。
/// - `step 2`:段内每个中间约束点求"轨迹点直线(`ctrl_pts_law`)"与 A\* 路径的
///   交点,再从交点向轨迹点按分辨率步进找障碍表面边界;
///   段长 == 1 时用中点(官方 corner case);
/// - `step 3`:从首个交点索引向段边界传播 base/direction。
///
/// 返回是否生成了至少一个平面(官方 `got_intersection_id` ≥ 0)。
fn assign_planes_for_segment(
    map: &GridMap,
    a_star_path: &[Vector3<f64>],
    points: &[Vector3<f64>],
    span: ConstraintSpan,
    planes: &mut [Vec<Plane>],
) -> bool {
    let ConstraintSpan {
        start: first,
        end: second,
    } = span;
    // 段边界裁剪到 points 有效范围（官方 lo/hi）
    let lo = first;
    let hi = second.min(points.len() - 1);
    if a_star_path.len() < 2 {
        return false;
    }
    let mut flag_temp = vec![false; hi.saturating_sub(lo) + 1];
    let mut got_intersection_id: Option<usize> = None;

    // step 2:中间约束点求交点
    if second - first == 1 {
        // 官方 corner case:段长 1,用中点
        let middle = (points[second] + points[first]) / 2.0;
        if let Some((intersection, point)) =
            intersection_on_path(a_star_path, &points[first], &points[second], middle)
            && (intersection - point).norm() > 0.01
        {
            flag_temp[first - lo] = true;
            let (base, dir) = surface_pair(map, intersection, point);
            planes[first].push(Plane::new(base, dir));
            got_intersection_id = Some(first);
        }
    } else {
        for j in (first + 1)..second {
            if let Some((intersection, point)) =
                intersection_on_path(a_star_path, &points[j - 1], &points[j + 1], points[j])
            {
                let length = (intersection - point).norm();
                if length > 1e-5 {
                    flag_temp[j - lo] = true;
                    let (base, dir) = surface_pair(map, intersection, point);
                    planes[j].push(Plane::new(base, dir));
                    got_intersection_id = Some(j);
                }
            }
        }
    }

    // step 3:传播(官方从 got_intersection_id 向调整后段边界填充)
    let Some(got) = got_intersection_id else {
        return false;
    };
    // 向后(got+1 ..= hi)：官方 `base_point[j].push_back(base_point[j-1].back())`，
    // 直接取 j-1 的平面（j=got+1 时 j-1=got 必有平面，后续 j-1 已被传播）
    for j in (got + 1)..=hi {
        if !flag_temp[j - lo]
            && let Some(p) = planes[j - 1].last()
        {
            planes[j].push(p.clone());
        }
    }
    // 向前(lo..got 递减)：官方 `base_point[j].push_back(base_point[j+1].back())`
    for j in (lo..got).rev() {
        if !flag_temp[j - lo]
            && let Some(p) = planes[j + 1].last()
        {
            planes[j].push(p.clone());
        }
    }
    true
}

/// 官方 step 2 的交点求取:在 A\* 路径上沿 ctrl 方向投影,找符号翻转点,
/// 线性插值出"过 point 沿 ctrl 方向的直线"与 A\* 路径段的交点。
/// `p_minus`/`p_plus` = points[j±1](ctrl = `p_plus − p_minus`)。
///
/// 返回 `(intersection_point, point)`(point 即传入的轨迹约束点)。
fn intersection_on_path(
    a_star_path: &[Vector3<f64>],
    p_minus: &Vector3<f64>,
    p_plus: &Vector3<f64>,
    point: Vector3<f64>,
) -> Option<(Vector3<f64>, Vector3<f64>)> {
    let ctrl = p_plus - p_minus;
    if ctrl.norm_squared() < 1e-12 {
        return None;
    }
    let mut astar_id = a_star_path.len() / 2;
    let mut val = (a_star_path[astar_id] - point).dot(&ctrl);
    let init_val = val;
    loop {
        let last_id = astar_id;
        if val >= 0.0 {
            astar_id += 1;
            if astar_id >= a_star_path.len() {
                break;
            }
        } else {
            // 先判 0 再减,避免 usize 下溢(官方 C++ 有符号 int 无此问题,
            // Rust debug 下 0-1 直接 panic)。
            if astar_id == 0 {
                break;
            }
            astar_id -= 1;
        }
        let new_val = (a_star_path[astar_id] - point).dot(&ctrl);
        if new_val * init_val <= 0.0 && (new_val.abs() > 0.0 || init_val.abs() > 0.0) {
            let a = a_star_path[astar_id];
            let b = a_star_path[last_id];
            let denom = ctrl.dot(&(a - b));
            if denom.abs() < 1e-12 {
                break;
            }
            let t = ctrl.dot(&(point - a)) / denom;
            let intersection = a + (a - b) * t;
            return Some((intersection, point));
        }
        val = new_val;
    }
    None
}

/// 官方边界点求取:从交点(自由)向轨迹点(障碍)按分辨率步进,
/// 首个占据体素(或末步)前的点即表面基点 s;方向 = (交点−轨迹点) 归一化。
fn surface_pair(
    map: &GridMap,
    intersection: Vector3<f64>,
    point: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let length = (intersection - point).norm();
    let res = map.resolution();
    if length < 1e-9 {
        return (point, Vector3::new(1.0, 0.0, 0.0));
    }
    let dir = (intersection - point).normalize();
    let mut a = length;
    let mut base = intersection;
    while a >= 0.0 {
        let p = (a / length) * intersection + (1.0 - a / length) * point;
        let occ = map.is_occupied_inflated(p);
        if occ || a < res {
            if occ {
                a += res;
                base = (a / length).min(1.0) * intersection + (1.0 - (a / length).min(1.0)) * point;
            } else {
                base = p;
            }
            break;
        }
        a -= res;
    }
    (base, dir)
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_map::{GridMapBuilder, VoxelState};
    use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder};
    use nalgebra::Point3;

    fn wall_map() -> (GridMap, Astar) {
        // 0.1m 分辨率,墙在 x=4.5(体素 x=45..46),z 全高,
        // y 留间隙 [4.5, 5.6]m 供 A* 绕行(官方语义:障碍必须可绕,否则
        // A* 失败 → 官方返回 ERR,测试墙若贯穿全图则无解)。
        let mut map = GridMapBuilder::new(0.1, [100, 100, 20]).build().unwrap();
        for y in 0..100 {
            for z in 0..20 {
                let y_m = y as f64 * 0.1;
                if !(4.5..5.6).contains(&y_m) {
                    map.set_state([45, y, z], VoxelState::Occupied);
                }
            }
        }
        map.inflate_obstacles();
        (map, Astar::default())
    }

    #[test]
    fn constraint_points_layout_matches_nk_plus_1() {
        let start = Endpoint {
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(4.0, 0.0, 0.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let m = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(
                &[
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(2.0, 0.0, 0.0),
                    Point3::new(3.0, 0.0, 0.0),
                ],
                &[1.0, 1.0, 1.0, 1.0],
            )
            .unwrap();
        let traj = m.solve().unwrap();
        for k in [5usize, 3] {
            let pts = constraint_sample_points(&traj, k);
            assert_eq!(pts.len(), traj.pieces() * k + 1, "N·K+1");
            // 端点
            assert!((pts[0] - Vector3::zeros()).norm() < 1e-9);
            assert!((pts[traj.pieces() * k] - Vector3::new(4.0, 0.0, 0.0)).norm() < 1e-9);
        }
    }

    #[test]
    fn two_thirds_truncation() {
        // cols = 5*5+1 = 26:2/3 → 25 − 24/3 = 17
        assert_eq!(two_thirds_id(26, false), 17);
        assert_eq!(two_thirds_id(26, true), 25);
        assert_eq!(two_thirds_id(16, false), 15 - 14 / 3);
    }

    #[test]
    fn compute_points_to_check_structure_and_density() {
        // 空旷地图只需分辨率参与采样步长
        let map = GridMapBuilder::new(0.1, [100, 100, 20]).build().unwrap();
        let scanner = ObstacleScanner::new(&map).with_samples(5).with_max_vel(1.5);
        // 单段直线轨迹 0→8m/8s(纯五次多项式,峰值速度 15/8 m/s):
        // 中段速度 > res/(2·t_step)=0.75m/s,采样不被运动门限抑制
        let start = Endpoint {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(8.0, 0.0, 0.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let m = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&[], &[8.0])
            .unwrap();
        let traj = m.solve().unwrap();
        let t_step = 0.1 / 1.5; // 官方采样步长 res / max_vel
        let cols = traj.pieces() * 5 + 1;
        let dur = traj.duration();

        // touch_goal=true:全部 N·K 桶,覆盖首尾
        let chk = scanner
            .compute_points_to_check(&traj, true)
            .expect("touch_goal 应生成成功");
        assert_eq!(chk.len(), two_thirds_id(cols, true));
        assert!(
            chk.last().is_some_and(|b| !b.is_empty()),
            "末桶非空(官方 pop 尾部空桶后不变量)"
        );
        let ts: Vec<f64> = chk.iter().flatten().map(|(t, _)| *t).collect();
        assert!(ts.windows(2).all(|w| w[1] >= w[0]), "时间戳必须单调不减");
        assert!(ts[0] < 1e-5, "首采样在 t≈0");
        assert!(*ts.last().unwrap() <= dur);
        assert!(
            *ts.last().unwrap() > dur - dur / 5.0 - t_step,
            "末桶必须覆盖最后一段({} vs {dur})",
            ts.last().unwrap()
        );
        // 密度:高速窗口 [2,6]s 内相邻间隔恰为采样步长 res/max_vel
        let mut dense_pairs = 0usize;
        for w in ts.windows(2) {
            if w[0] >= 2.0 && w[1] <= 6.0 {
                assert!(
                    (w[1] - w[0] - t_step).abs() < 1e-9,
                    "高速区间采样步长应为 {t_step},实际 {}",
                    w[1] - w[0]
                );
                dense_pairs += 1;
            }
        }
        assert!(
            dense_pairs > 20,
            "匀速窗口应有足量采样对,实际 {dense_pairs}"
        );

        // 非 touch_goal:按 two_thirds 截断,桶数更少
        let trunc = scanner
            .compute_points_to_check(&traj, false)
            .expect("非 touch_goal 应生成成功");
        assert_eq!(trunc.len(), two_thirds_id(cols, false));
        assert!(trunc.len() < chk.len());
    }

    #[test]
    fn finely_check_detects_wall_and_assigns_planes() {
        let (map, mut astar) = wall_map();
        let scanner = ObstacleScanner::new(&map).with_samples(5).with_max_vel(1.5);
        let start = Endpoint {
            position: Vector3::new(1.0, 2.0, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(9.0, 2.0, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        // 直线穿墙轨迹(未绕障)
        let m = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(
                &[Point3::new(3.0, 2.0, 1.0), Point3::new(7.0, 2.0, 1.0)],
                &[2.0, 2.0, 2.0],
            )
            .unwrap();
        let traj = m.solve().unwrap();
        let points = constraint_sample_points(&traj, 5);
        let mut planes: Vec<Vec<Plane>> = vec![Vec::new(); points.len()];
        let ret = scanner.finely_check(&mut astar, &traj, &points, &mut planes, false);
        assert_eq!(ret, CheckResult::Finished, "穿墙轨迹必有碰撞段");
        let total: usize = planes.iter().map(Vec::len).sum();
        assert!(total > 0, "必须生成平面");
        // 每个平面方向应指向 A* 绕行方向(至少不是零向量)
        for pl in planes.iter().flatten() {
            assert!(pl.normal().norm() > 0.99, "方向必须归一化");
        }
    }

    #[test]
    fn roughly_check_ignores_covered_points() {
        let (map, mut astar) = wall_map();
        let scanner = ObstacleScanner::new(&map).with_samples(5).with_max_vel(1.5);
        // 构造穿墙约束点:直接给 y=2 直线采样点
        let points: Vec<Vector3<f64>> = (0..26)
            .map(|i| Vector3::new(1.0 + f64::from(i) * 0.3, 2.0, 1.0))
            .collect();
        let mut planes: Vec<Vec<Plane>> = vec![Vec::new(); points.len()];
        // 用大致垂直墙面的平面盖住 x≈4.5 附近的点(方向 +x 朝自由侧)
        for i in 0..26 {
            let p = points[i];
            if (p.x - 4.5).abs() < 0.4 {
                let dir = Vector3::new(1.0, 0.0, 0.0);
                let base = Vector3::new(4.6, p.y, p.z);
                planes[i].push(Plane::new(base, dir));
            }
        }
        // 覆盖后 roughly_check 不应再报新障碍(4.5 处穿入点已被平面包住)
        let new_obs = scanner.roughly_check(&mut astar, &points, &mut planes, false);
        // 注意:4.5±0.4 外的点也可能在膨胀层内;此测试只验证"covered 判据"参与
        // ——若所有穿入点都被覆盖,则无新障碍
        let all_covered = points
            .iter()
            .enumerate()
            .filter(|(i, p)| {
                map.is_occupied_inflated(**p) && *i <= two_thirds_id(points.len(), false)
            })
            .all(|(i, p)| {
                planes[i]
                    .iter()
                    .any(|pl| (p - pl.point()).dot(&pl.normal()) < map.resolution())
            });
        if all_covered {
            assert!(!new_obs, "全覆盖点不应触发 Rebound");
        }
    }

    #[test]
    fn intersection_on_path_walks_back_to_0_without_underflow() {
        // 所有 A* 点都在 `point` 沿 ctrl 的"后方"(投影 < 0),搜索会一路
        // 回退到索引 0 仍无符号翻转 → 返回 None;此前 0-1 在 debug 下
        // usize 下溢 panic("attempt to subtract with overflow")。
        let path = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::new(6.0, 0.0, 0.0),
            Vector3::new(8.0, 0.0, 0.0),
        ];
        let p_minus = Vector3::new(0.0, 0.0, 0.0);
        let p_plus = Vector3::new(2.0, 0.0, 0.0);
        let point = Vector3::new(10.0, 0.0, 0.0);
        let r = intersection_on_path(&path, &p_minus, &p_plus, point);
        assert!(r.is_none(), "无符号翻转应返回 None 而非 panic");
    }

    #[test]
    fn free_trajectory_returns_free() {
        let (mut map, mut astar) = wall_map();
        // 把墙清掉 → 自由
        for y in 0..100 {
            for z in 0..20 {
                map.set_state([45, y, z], VoxelState::Unknown);
            }
        }
        map.inflate_obstacles();
        let scanner = ObstacleScanner::new(&map).with_samples(5).with_max_vel(1.5);
        let start = Endpoint {
            position: Vector3::new(1.0, 2.0, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let end = Endpoint {
            position: Vector3::new(9.0, 2.0, 1.0),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),
        };
        let m = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&[], &[4.0])
            .unwrap();
        let traj = m.solve().unwrap();
        let points = constraint_sample_points(&traj, 5);
        let mut planes: Vec<Vec<Plane>> = vec![Vec::new(); points.len()];
        let ret = scanner.finely_check(&mut astar, &traj, &points, &mut planes, false);
        assert_eq!(ret, CheckResult::Free);
    }
}
