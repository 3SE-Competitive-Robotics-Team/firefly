//! A* 网格搜索。
//!
//! 对齐官方 EGO-Planner `dyn_a_star`：26 邻域（代价 `√(dx²+dy²+dz²)`×resolution）、
//! 判据 = 官方 `checkOccupancy`（`getInflateOccupancy`，含膨胀）；`tie_breaker` ×
//! 对角启发、端点入障沿远离对端方向外推至自由、节点池预分配 +
//! 世代计数复用（`rounds` 区分每次搜索，免重置）。
//!
//! 地图索引固定在占据窗口；26 邻域只检查目标节点的膨胀占据。

use firefly_error::{Error, ErrorKind};
use firefly_map::GridMap;
use nalgebra::Vector3;

const SQRT_3: f64 = 1.732_050_807_568_877_2;

#[derive(Debug, Clone)]
pub struct AstarConfig {
    pub heuristic_weight: f64,
    pub max_expansions: usize,
}

impl Default for AstarConfig {
    fn default() -> Self {
        Self {
            heuristic_weight: 1.0,
            max_expansions: 500_000,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Path {
    points: Vec<Vector3<f64>>,
}

impl Path {
    #[must_use]
    pub fn points(&self) -> &[Vector3<f64>] {
        &self.points
    }
}

/// 线性体素索引（`x * dy * dz + y * dz + z`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Node(u32);

impl Node {
    const NONE: Node = Node(u32::MAX);
}

#[derive(Clone, Copy)]
struct Entry {
    node: Node,
    g: f64,
    f: f64,
}

/// 节点状态（官方 `GridNode::enum_state`）。
#[derive(Clone, Copy, PartialEq)]
enum NodeState {
    Undefined,
    Open,
    Closed,
}

/// A* 搜索器：节点池缓冲按需扩容、跨搜索复用（官方 `rounds_` 世代）。
pub struct Astar {
    config: AstarConfig,
    g_score: Vec<f64>,
    came_from: Vec<Node>,
    rounds: Vec<u32>,
    state: Vec<NodeState>,
    round: u32,
    /// 索引二叉堆（去重 + decrease-key）；`heap_pos[node]` 为堆内下标。
    heap: Vec<Entry>,
    heap_pos: Vec<u32>,
}

const NO_HEAP: u32 = u32::MAX;
const HEAP_ARITY: usize = 4;

impl Default for Astar {
    fn default() -> Self {
        Self::with_config(AstarConfig::default())
    }
}

impl Astar {
    #[must_use]
    pub fn with_config(config: AstarConfig) -> Self {
        Self {
            config,
            g_score: Vec::new(),
            came_from: Vec::new(),
            rounds: Vec::new(),
            state: Vec::new(),
            round: 0,
            heap: Vec::new(),
            heap_pos: Vec::new(),
        }
    }

    /// # Errors
    ///
    /// `OutOfRange`：起点/终点在地图外或无法外推至自由；`NotFound`：无可达路径；
    /// `Convergence`：超出扩展上限。
    #[fastrace::trace]
    pub fn search(
        &mut self,
        map: &GridMap,
        start: Vector3<f64>,
        goal: Vector3<f64>,
    ) -> firefly_error::Result<Path> {
        if start.iter().chain(goal.iter()).any(|v| !v.is_finite()) {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "non-finite search endpoint",
            ));
        }
        let mut start_pt = start;
        let mut goal_pt = goal;
        let mut start_idx = map
            .index_of(start_pt)
            .ok_or_else(|| Error::new(ErrorKind::OutOfRange, "start is outside the map"))?;
        let mut goal_idx = map
            .index_of(goal_pt)
            .ok_or_else(|| Error::new(ErrorKind::OutOfRange, "goal is outside the map"))?;
        // 端点落入膨胀占据时按 resolution 沿远离对端方向外推，直到自由或越出地图
        // （对照官方 `ConvertToIndexAndAdjustStartEndPoints` 的 `checkOccupancy` =
        // `getInflateOccupancy`，即含膨胀；终点分支按语义检查自身索引，不复刻官方
        // 误查 `start_idx` 的笔误）。
        while map.is_occupied_inflated(center_of(map, start_idx)) {
            let dir = (start_pt - goal_pt)
                .try_normalize(0.0)
                .ok_or_else(|| Error::new(ErrorKind::OutOfRange, "start coincides with goal"))?;
            start_pt += dir * map.resolution();
            start_idx = map.index_of(start_pt).ok_or_else(|| {
                Error::new(ErrorKind::OutOfRange, "start extrapolation left the map")
            })?;
        }
        while map.is_occupied_inflated(center_of(map, goal_idx)) {
            let dir = (goal_pt - start_pt)
                .try_normalize(0.0)
                .ok_or_else(|| Error::new(ErrorKind::OutOfRange, "goal coincides with start"))?;
            goal_pt += dir * map.resolution();
            goal_idx = map.index_of(goal_pt).ok_or_else(|| {
                Error::new(ErrorKind::OutOfRange, "goal extrapolation left the map")
            })?;
        }

        let total = map.dims()[0] * map.dims()[1] * map.dims()[2];
        self.ensure_capacity(total);
        self.round = self.round.wrapping_add(1);
        if self.round == 0 {
            // 世代回绕：全部清零重来
            self.g_score.fill(f64::INFINITY);
            self.came_from.fill(Node::NONE);
            self.rounds.fill(0);
            self.state.fill(NodeState::Undefined);
            self.round = 1;
        }
        let round = self.round;

        let start_node = Node(Self::linear(map, start_idx));
        let goal_node = Node(Self::linear(map, goal_idx));
        self.touch(start_node, round);
        self.g_score[start_node.0 as usize] = 0.0;
        self.came_from[start_node.0 as usize] = Node::NONE;
        self.state[start_node.0 as usize] = NodeState::Open;

        self.heap_clear();
        self.heap_push(Entry {
            node: start_node,
            g: 0.0,
            f: self.config.heuristic_weight * Self::heuristic(map, start_idx, goal_idx),
        });

        let mut expansions = 0usize;
        while let Some(Entry { node, g, .. }) = self.heap_pop() {
            if self.rounds[node.0 as usize] != round
                || self.state[node.0 as usize] != NodeState::Open
            {
                continue;
            }
            if node == goal_node {
                return Ok(self.reconstruct(map, goal_node, start_pt, goal_pt));
            }
            if expansions >= self.config.max_expansions {
                return Err(Error::temporary(
                    ErrorKind::Convergence,
                    "a* exceeded expansion limit",
                ));
            }
            expansions += 1;
            self.state[node.0 as usize] = NodeState::Closed;
            let idx = Self::decode(map, node);
            self.expand(map, idx, goal_idx, node, g);
        }
        Err(Error::temporary(ErrorKind::NotFound, "no path found"))
    }

    /// 展开一个节点：缓存 3×3×3 膨胀占用，逐个邻居松弛（不经过中间 Vec）。
    #[inline]
    fn expand(&mut self, map: &GridMap, idx: [usize; 3], goal_idx: [usize; 3], node: Node, g: f64) {
        let blocked = map.inflated_neighborhood_mask(idx);
        Self::for_each_neighbor(map, idx, blocked, |nb, step, nb_idx| {
            self.relax_neighbor(map, goal_idx, node, g, nb, step, nb_idx);
        });
    }

    /// 遍历 26 邻域中避开膨胀层的单位步。
    fn for_each_neighbor(
        map: &GridMap,
        idx: [usize; 3],
        blocked: u32,
        mut visit: impl FnMut(Node, f64, [usize; 3]),
    ) {
        let [dx, dy, dz] = map.dims();
        let resolution = map.resolution();
        let [x, y, z] = idx;
        for i in -1i32..=1 {
            for j in -1i32..=1 {
                for k in -1i32..=1 {
                    if i == 0 && j == 0 && k == 0 {
                        continue;
                    }
                    let (nx, ny, nz) = (x as i32 + i, y as i32 + j, z as i32 + k);
                    if nx < 0
                        || ny < 0
                        || nz < 0
                        || nx >= dx as i32
                        || ny >= dy as i32
                        || nz >= dz as i32
                    {
                        continue;
                    }
                    let direction = ((i + 1) * 3 + (j + 1)) as usize * 3 + (k + 1) as usize;
                    if blocked & (1 << direction) != 0 {
                        continue;
                    }
                    let step = match i * i + j * j + k * k {
                        1 => resolution,
                        2 => std::f64::consts::SQRT_2 * resolution,
                        3 => SQRT_3 * resolution,
                        _ => unreachable!("26 邻域步长平方只能是 1/2/3"),
                    };
                    let nidx = [nx as usize, ny as usize, nz as usize];
                    visit(Node(Self::linear(map, nidx)), step, nidx);
                }
            }
        }
    }

    /// 单邻居松弛：闭集与无改进的节点跳过。
    #[inline]
    #[allow(
        clippy::too_many_arguments,
        reason = "展开热路径，避免为邻居临时打包结构体"
    )]
    fn relax_neighbor(
        &mut self,
        map: &GridMap,
        goal_idx: [usize; 3],
        node: Node,
        g: f64,
        nb: Node,
        step_len: f64,
        nb_idx: [usize; 3],
    ) {
        let tentative = g + step_len;
        let round = self.round;
        let explored = self.rounds[nb.0 as usize] == round;
        if explored
            && (self.state[nb.0 as usize] == NodeState::Closed
                || tentative >= self.g_score[nb.0 as usize])
        {
            return;
        }
        if !explored {
            self.touch(nb, round);
            self.state[nb.0 as usize] = NodeState::Open;
        }
        self.g_score[nb.0 as usize] = tentative;
        self.came_from[nb.0 as usize] = node;
        self.heap_push(Entry {
            node: nb,
            g: tentative,
            f: tentative + self.config.heuristic_weight * Self::heuristic(map, nb_idx, goal_idx),
        });
    }

    /// 测试用：返回邻居集合（生产路径直接 `expand`，不分配）。
    #[cfg(test)]
    fn fill_neighbors(map: &GridMap, idx: [usize; 3], out: &mut Vec<(Node, f64, [usize; 3])>) {
        out.clear();
        let blocked = map.inflated_neighborhood_mask(idx);
        Self::for_each_neighbor(map, idx, blocked, |node, step, nidx| {
            out.push((node, step, nidx));
        });
    }

    fn heap_clear(&mut self) {
        for i in 0..self.heap.len() {
            let node = self.heap[i].node.0 as usize;
            self.heap_pos[node] = NO_HEAP;
        }
        self.heap.clear();
    }

    /// 与旧 `BinaryHeap` 的 `Ord` 平局规则一致：f 相同取 node 较大者先出。
    fn heap_less(a: &Entry, b: &Entry) -> bool {
        match a.f.total_cmp(&b.f) {
            std::cmp::Ordering::Less => true,
            std::cmp::Ordering::Greater => false,
            std::cmp::Ordering::Equal => a.node > b.node,
        }
    }

    fn heap_push(&mut self, entry: Entry) {
        let node = entry.node.0 as usize;
        let pos = self.heap_pos[node];
        if pos == NO_HEAP {
            let index = self.heap.len();
            self.heap_pos[node] = index as u32;
            self.heap.push(entry);
            self.sift_up(index);
        } else {
            let index = pos as usize;
            self.heap[index] = entry;
            self.sift_up(index);
        }
    }

    fn heap_pop(&mut self) -> Option<Entry> {
        let last = self.heap.pop()?;
        if self.heap.is_empty() {
            self.heap_pos[last.node.0 as usize] = NO_HEAP;
            return Some(last);
        }
        let root = std::mem::replace(&mut self.heap[0], last);
        self.heap_pos[root.node.0 as usize] = NO_HEAP;
        self.heap_pos[self.heap[0].node.0 as usize] = 0;
        self.sift_down(0);
        Some(root)
    }

    fn sift_up(&mut self, mut index: usize) {
        while index > 0 {
            let parent = (index - 1) / HEAP_ARITY;
            if !Self::heap_less(&self.heap[index], &self.heap[parent]) {
                break;
            }
            self.heap.swap(index, parent);
            self.heap_pos[self.heap[index].node.0 as usize] = index as u32;
            self.heap_pos[self.heap[parent].node.0 as usize] = parent as u32;
            index = parent;
        }
    }

    fn sift_down(&mut self, mut index: usize) {
        loop {
            let first = HEAP_ARITY * index + 1;
            if first >= self.heap.len() {
                break;
            }
            let end = (first + HEAP_ARITY).min(self.heap.len());
            let mut smallest = first;
            for child in first + 1..end {
                if Self::heap_less(&self.heap[child], &self.heap[smallest]) {
                    smallest = child;
                }
            }
            if !Self::heap_less(&self.heap[smallest], &self.heap[index]) {
                break;
            }
            self.heap.swap(index, smallest);
            self.heap_pos[self.heap[index].node.0 as usize] = index as u32;
            self.heap_pos[self.heap[smallest].node.0 as usize] = smallest as u32;
            index = smallest;
        }
    }

    /// 对角启发（对照官方 `tie_breaker_ * getDiagHeu`）：三轴位移格数排序后按
    /// √3/√2/1 分段计费（体对角/面对角/轴向步），与官方三分支写法数学等价；
    /// 乘 `TIE_BREAKER` 轻微高估打破 f 平局、偏向直趋目标，乘 resolution 与 g
    /// （米）保持同量纲。
    fn heuristic(map: &GridMap, a: [usize; 3], b: [usize; 3]) -> f64 {
        /// 官方 `tie_breaker_`。
        const TIE_BREAKER: f64 = 1.0 + 1.0 / 10000.0;
        let dx = a[0].abs_diff(b[0]) as f64;
        let dy = a[1].abs_diff(b[1]) as f64;
        let dz = a[2].abs_diff(b[2]) as f64;
        let low = dx.min(dy).min(dz);
        let high = dx.max(dy).max(dz);
        let mid = dx + dy + dz - low - high;
        TIE_BREAKER
            * map.resolution()
            * (SQRT_3 * low + std::f64::consts::SQRT_2 * (mid - low) + (high - mid))
    }

    /// 标记节点为本代已访问（首次使用时分配缓冲）。
    fn touch(&mut self, node: Node, round: u32) {
        self.rounds[node.0 as usize] = round;
    }

    fn ensure_capacity(&mut self, total: usize) {
        if self.g_score.len() < total {
            self.g_score.resize(total, f64::INFINITY);
            self.came_from.resize(total, Node::NONE);
            self.rounds.resize(total, 0);
            self.state.resize(total, NodeState::Undefined);
            self.heap_pos.resize(total, NO_HEAP);
        }
    }

    fn linear(map: &GridMap, idx: [usize; 3]) -> u32 {
        let [_, dy, dz] = map.dims();
        ((idx[0] * dy + idx[1]) * dz + idx[2]) as u32
    }

    fn decode(map: &GridMap, node: Node) -> [usize; 3] {
        let [_, dy, dz] = map.dims();
        let l = node.0 as usize;
        let z = l % dz;
        let y = (l / dz) % dy;
        let x = l / (dz * dy);
        [x, y, z]
    }

    fn reconstruct(
        &self,
        map: &GridMap,
        goal: Node,
        start: Vector3<f64>,
        goal_point: Vector3<f64>,
    ) -> Path {
        let mut nodes = vec![goal];
        let mut current = goal;
        while current != Node::NONE {
            let prev = self.came_from[current.0 as usize];
            if prev == Node::NONE {
                break;
            }
            nodes.push(prev);
            current = prev;
        }
        nodes.reverse();
        let r = map.resolution();
        let origin = map.origin();
        let mut points: Vec<Vector3<f64>> = nodes
            .into_iter()
            .map(|n| {
                let idx = Self::decode(map, n);
                Vector3::new(
                    origin.x + (idx[0] as f64 + 0.5) * r,
                    origin.y + (idx[1] as f64 + 0.5) * r,
                    origin.z + (idx[2] as f64 + 0.5) * r,
                )
            })
            .collect();
        // 保留中心连边；精确端点到所在体素中心的线段完全位于该体素内。
        if points.first() != Some(&start) {
            points.insert(0, start);
        }
        if points.last() != Some(&goal_point) {
            points.push(goal_point);
        }
        Path { points }
    }
}

/// 索引 → 体素中心（`world = origin + (idx + 0.5) * resolution`）。
fn center_of(map: &GridMap, idx: [usize; 3]) -> Vector3<f64> {
    let o = map.origin();
    let r = map.resolution();
    Vector3::new(
        o.x + (idx[0] as f64 + 0.5) * r,
        o.y + (idx[1] as f64 + 0.5) * r,
        o.z + (idx[2] as f64 + 0.5) * r,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn empty_map() -> GridMap {
        firefly_map::GridMapBuilder::new(1.0, [10, 10, 10])
            .with_obstacles_inflation(1.0)
            .build()
            .unwrap()
    }

    fn search(
        astar: &mut Astar,
        map: &GridMap,
        start: [f64; 3],
        goal: [f64; 3],
    ) -> firefly_error::Result<Path> {
        astar.search(
            map,
            Vector3::new(start[0], start[1], start[2]),
            Vector3::new(goal[0], goal[1], goal[2]),
        )
    }

    #[test]
    fn straight_line_path() {
        let map = empty_map();
        let mut astar = Astar::default();
        let path = search(&mut astar, &map, [0.5, 0.5, 0.5], [9.5, 0.5, 0.5]).unwrap();
        assert_eq!(path.points().first(), Some(&Vector3::new(0.5, 0.5, 0.5)));
        assert_eq!(path.points().last(), Some(&Vector3::new(9.5, 0.5, 0.5)));
        assert!(path.points().len() <= 10);
    }

    #[test]
    fn no_path_when_blocked() {
        let mut map = empty_map();
        for y in 0..10 {
            for z in 0..10 {
                map.set_state([5, y, z], firefly_map::VoxelState::Occupied);
            }
        }
        map.inflate_obstacles();
        let mut astar = Astar::default();
        let r = search(&mut astar, &map, [0.5, 0.5, 0.5], [9.5, 9.5, 0.5]);
        assert_eq!(r.unwrap_err().kind(), ErrorKind::NotFound);
    }

    #[test]
    fn detours_around_wall_in_3d() {
        let mut map = empty_map();
        for y in 2..8 {
            for z in 0..2 {
                map.set_state([4, y, z], firefly_map::VoxelState::Occupied);
            }
        }
        map.inflate_obstacles();
        let mut astar = Astar::default();
        let path = search(&mut astar, &map, [0.5, 4.5, 0.5], [9.5, 4.5, 0.5]).unwrap();
        // 路径不穿过墙体（x=4, y 2..8, z 0..2）
        assert!(
            path.points()
                .iter()
                .all(|p| !(p.x > 3.5 && p.x < 4.5 && (2.0..=8.0).contains(&p.y) && p.z < 2.0))
        );
        assert!(path.points().len() > 2);
    }

    #[test]
    fn diagonal_step_checks_only_destination_occupancy() {
        for dims in [[2, 2, 1], [2, 2, 2]] {
            let mut map = firefly_map::GridMapBuilder::new(1.0, dims)
                .with_obstacles_inflation(0.)
                .build()
                .unwrap();
            let last = [1, 1, dims[2] - 1];
            for x in 0..2 {
                for y in 0..2 {
                    for z in 0..dims[2] {
                        let idx = [x, y, z];
                        if idx != [0, 0, 0] && idx != last {
                            map.set_state(idx, firefly_map::VoxelState::Occupied);
                        }
                    }
                }
            }
            map.inflate_obstacles();
            let start = [0.5, 0.5, 0.5];
            let goal = [1.5, 1.5, last[2] as f64 + 0.5];
            let path = search(&mut Astar::default(), &map, start, goal).unwrap();
            assert_eq!(path.points(), &[Vector3::from(start), Vector3::from(goal)]);
        }
    }

    #[test]
    fn open_grid_path_length_is_analytic_optimum() {
        // 空图对角线：8 格 dx/dy/dz 全等 → 最优 8×√3（米制 resolution=1）。
        let map = empty_map();
        let mut astar = Astar::default();
        let path = search(&mut astar, &map, [0.5, 0.5, 0.5], [8.5, 8.5, 8.5]).unwrap();
        let len: f64 = path.points().windows(2).map(|w| (w[1] - w[0]).norm()).sum();
        assert!(
            (len - 8.0 * 3f64.sqrt()).abs() < 1e-6,
            "开放网格应对角最优，实际 {len:.6}"
        );
    }

    #[test]
    fn off_map_endpoints_error_without_search() {
        let map = empty_map();
        let mut astar = Astar::default();
        assert_eq!(
            search(&mut astar, &map, [-5.0, 0.5, 0.5], [9.5, 9.5, 9.5])
                .unwrap_err()
                .kind(),
            ErrorKind::OutOfRange
        );
        assert_eq!(
            search(&mut astar, &map, [0.5, 0.5, 0.5], [50.0, 50.0, 50.0])
                .unwrap_err()
                .kind(),
            ErrorKind::OutOfRange
        );
    }

    #[test]
    fn path_nodes_keep_inflated_clearance() {
        // 上游 checkOccupancy 使用膨胀占据，验收路径节点与 26 邻域步长。
        let mut map = empty_map();
        map.set_state([5, 5, 5], firefly_map::VoxelState::Occupied);
        map.inflate_obstacles();
        let mut astar = Astar::default();
        let path = search(&mut astar, &map, [1.5, 1.5, 1.5], [9.5, 9.5, 9.5]).unwrap();
        for p in path.points() {
            assert!(!map.is_occupied_inflated(*p), "路径点 {p:?} 落在膨胀层内");
        }
        let idx: Vec<_> = path
            .points()
            .iter()
            .map(|p| map.index_of(*p).expect("路径点应在地图内"))
            .collect();
        for w in idx.windows(2) {
            for axis in 0..3 {
                assert!(
                    w[0][axis].abs_diff(w[1][axis]) <= 1,
                    "相邻路径格轴差 >1: {:?} -> {:?}",
                    w[0],
                    w[1]
                );
            }
        }
        assert_eq!(path.points().first(), Some(&Vector3::new(1.5, 1.5, 1.5)));
        assert_eq!(path.points().last(), Some(&Vector3::new(9.5, 9.5, 9.5)));
    }

    #[test]
    fn buffer_reused_across_searches() {
        // 第二次搜索复用同一世代缓冲：正确性 + 世代递增
        let map = empty_map();
        let mut astar = Astar::default();
        let p1 = search(&mut astar, &map, [0.5, 0.5, 0.5], [9.5, 9.5, 0.5]).unwrap();
        assert!(p1.points().len() > 2);
        assert_eq!(astar.round, 1);
        let p2 = search(&mut astar, &map, [0.5, 0.5, 0.5], [9.5, 9.5, 9.5]).unwrap();
        assert!(p2.points().len() > 2);
        assert_eq!(astar.round, 2);
    }

    #[test]
    fn corner_touch_allows_diagonal_step_like_upstream() {
        let mut map = firefly_map::GridMapBuilder::new(1., [10, 10, 10])
            .with_obstacles_inflation(0.)
            .build()
            .unwrap();
        map.set_state([5, 5, 5], firefly_map::VoxelState::Occupied);
        map.inflate_obstacles();
        let path = search(
            &mut Astar::default(),
            &map,
            [4.5, 5.5, 5.5],
            [5.5, 4.5, 5.5],
        )
        .unwrap();
        let length: f64 = path.points().windows(2).map(|w| (w[1] - w[0]).norm()).sum();
        assert!((length - std::f64::consts::SQRT_2).abs() < 1e-12);
        assert_eq!(path.points().len(), 2);
    }

    #[test]
    fn distinct_endpoints_in_one_voxel_are_preserved() {
        let path = search(
            &mut Astar::default(),
            &empty_map(),
            [0.1, 0.2, 0.3],
            [0.7, 0.8, 0.9],
        )
        .unwrap();
        assert_eq!(path.points().first(), Some(&Vector3::new(0.1, 0.2, 0.3)));
        assert_eq!(path.points().last(), Some(&Vector3::new(0.7, 0.8, 0.9)));
    }

    #[test]
    fn fine_grid_large_map() {
        // 0.1m 分辨率 280×80×32 网格上绕柱搜索，验证性能与正确性
        let mut map = firefly_map::GridMapBuilder::new(0.1, [280, 80, 32])
            .build()
            .unwrap();
        for x in 100..180 {
            for z in 0..32 {
                for y in 0..40 {
                    map.set_state([x, y, z], firefly_map::VoxelState::Occupied);
                }
            }
        }
        let mut astar = Astar::default();
        let path = search(&mut astar, &map, [1.0, 4.0, 1.0], [27.0, 4.0, 1.0]).unwrap();
        assert!(path.points().len() > 100);
    }

    #[test]
    fn heuristic_matches_official_diag_heu() {
        // 对照官方 getDiagHeu 取值；resolution = 1.0 时米与格数同值
        let map = empty_map();
        let tb = 1.0 + 1.0 / 10000.0;
        for (b, cells) in [
            ([5usize, 0, 0], 5.0),                // 轴向
            ([0, 0, 1], 1.0),                     // 单步
            ([1, 1, 1], 3f64.sqrt()),             // 体对角
            ([3, 2, 0], 2.0 * 2f64.sqrt() + 1.0), // 面对角 + 轴向
        ] {
            let h = Astar::heuristic(&map, [0, 0, 0], b);
            assert!((h - tb * cells).abs() < 1e-12, "b={b:?} h={h}");
        }
        // 分辨率因子：h 按米计，与 g 同量纲
        let fine = firefly_map::GridMapBuilder::new(0.5, [10, 10, 10])
            .build()
            .unwrap();
        let h = Astar::heuristic(&fine, [0, 0, 0], [4, 0, 0]);
        assert!((h - tb * 2.0).abs() < 1e-12);
    }

    #[test]
    fn start_inside_occupied_is_pushed_out_to_free() {
        let mut map = empty_map();
        map.set_state([2, 5, 5], firefly_map::VoxelState::Occupied);
        map.inflate_obstacles();
        let mut astar = Astar::default();
        let path = search(&mut astar, &map, [2.5, 5.5, 5.5], [9.5, 5.5, 5.5]).unwrap();
        // 路径起点为外推后的自由体素中心（沿远离目标方向），而非原始入障坐标
        assert_eq!(path.points().first(), Some(&Vector3::new(0.5, 5.5, 5.5)));
        assert_eq!(path.points().last(), Some(&Vector3::new(9.5, 5.5, 5.5)));
    }

    #[test]
    fn start_inside_inflation_is_pushed_out() {
        // 仅膨胀区（原始自由）：官方 `checkOccupancy` = `getInflateOccupancy` 同样算占用
        // → 起点仍沿远离对端方向外推至膨胀层外。
        let mut map = empty_map();
        map.set_state([3, 5, 5], firefly_map::VoxelState::Occupied);
        map.inflate_obstacles(); // 起点 [2,5,5] 落在膨胀层内，原始自由
        let mut astar = Astar::default();
        let path = search(&mut astar, &map, [2.5, 5.5, 5.5], [9.5, 5.5, 5.5]).unwrap();
        assert_eq!(path.points().first(), Some(&Vector3::new(1.5, 5.5, 5.5)));
        assert_eq!(path.points().last(), Some(&Vector3::new(9.5, 5.5, 5.5)));
    }

    #[test]
    fn goal_inside_occupied_is_pushed_out_to_free() {
        let mut map = empty_map();
        map.set_state([7, 5, 5], firefly_map::VoxelState::Occupied);
        map.inflate_obstacles();
        let mut astar = Astar::default();
        let path = search(&mut astar, &map, [0.5, 5.5, 5.5], [7.5, 5.5, 5.5]).unwrap();
        // 路径终点为外推后的自由体素中心（沿远离起点方向）
        assert_eq!(path.points().first(), Some(&Vector3::new(0.5, 5.5, 5.5)));
        assert_eq!(path.points().last(), Some(&Vector3::new(9.5, 5.5, 5.5)));
    }

    #[test]
    fn extrapolation_out_of_map_errors() {
        let mut map = empty_map();
        map.set_state([0, 0, 0], firefly_map::VoxelState::Occupied);
        map.inflate_obstacles(); // 角落起点入障，远离目标方向直接指向图外
        let mut astar = Astar::default();
        let err = search(&mut astar, &map, [0.5, 0.5, 0.5], [9.5, 9.5, 9.5]).unwrap_err();
        assert_eq!(err.kind(), ErrorKind::OutOfRange);
    }
}

#[cfg(test)]
mod gap_tests {
    use super::*;
    use firefly_map::{GridMapBuilder, VoxelState};

    /// 两堵墙留 1 格门，A* 应穿过门（而非绕墙外圈）。
    #[test]
    fn finds_path_through_gap() {
        let mut map = GridMapBuilder::new(1.0, [12, 12, 12])
            .with_obstacles_inflation(0.0)
            .build()
            .unwrap();
        // x=5 与 x=7 两堵墙（y,z 全占），但 y=6 留出门缝（1 格宽）
        for x in [5usize, 7] {
            for y in 0..12 {
                for z in 0..12 {
                    if y == 6 {
                        continue; // 门
                    }
                    map.set_state([x, y, z], VoxelState::Occupied);
                }
            }
        }
        map.inflate_obstacles();
        let mut astar = Astar::default();
        let path = astar
            .search(
                &map,
                Vector3::new(0.5, 6.5, 0.5),
                Vector3::new(11.5, 6.5, 0.5),
            )
            .expect("应找到穿门路径");
        // 路径经过 x in [5,7] 时 y 应落在门（6）附近
        let through_gap = path
            .points()
            .iter()
            .any(|p| p.x >= 4.5 && p.x <= 7.5 && (p.y - 6.5).abs() < 1.0);
        assert!(through_gap, "A* 应穿过门缝而非绕外圈");
    }
}

#[cfg(test)]
mod neighbor_tests {
    use super::*;
    use firefly_map::{GridMapBuilder, VirtualWall, VoxelState};

    /// 上游邻居判定：只查询目标节点的膨胀占据。
    fn upstream_neighbors(map: &GridMap, idx: [usize; 3]) -> Vec<(u32, f64)> {
        let [dx, dy, dz] = map.dims();
        let [x, y, z] = idx;
        let mut out = Vec::with_capacity(26);
        for i in -1i32..=1 {
            for j in -1i32..=1 {
                for k in -1i32..=1 {
                    if i == 0 && j == 0 && k == 0 {
                        continue;
                    }
                    let (nx, ny, nz) = (x as i32 + i, y as i32 + j, z as i32 + k);
                    if nx < 0
                        || ny < 0
                        || nz < 0
                        || nx >= dx as i32
                        || ny >= dy as i32
                        || nz >= dz as i32
                    {
                        continue;
                    }
                    let nidx = [nx as usize, ny as usize, nz as usize];
                    if map.is_occupied_inflated(center_of(map, nidx)) {
                        continue;
                    }
                    out.push((
                        Astar::linear(map, nidx),
                        f64::from(i * i + j * j + k * k).sqrt() * map.resolution(),
                    ));
                }
            }
        }
        out
    }

    #[test]
    fn fill_neighbors_matches_upstream_occupancy() {
        let mut map = GridMapBuilder::new(0.4, [12, 12, 12])
            .with_origin(Vector3::new(-1.3, 2.1, -0.7))
            .with_obstacles_inflation(0.8)
            .build()
            .unwrap();
        let mut seed = 0x2E8B_A5F3_4C1D_77E9_u64;
        let mut draw = || {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (seed >> 33) as usize % 12
        };
        for _ in 0..40 {
            map.set_state([draw(), draw(), draw()], VoxelState::Occupied);
        }
        map.inflate_obstacles();
        map.set_virtual_wall(VirtualWall {
            ground: 0.0,
            ceil: 3.0,
        });
        let mut actual = Vec::new();
        for x in 0..12 {
            for y in 0..12 {
                for z in 0..12 {
                    let idx = [x, y, z];
                    Astar::fill_neighbors(&map, idx, &mut actual);
                    let got: Vec<(u32, f64)> = actual
                        .iter()
                        .map(|&(node, step, _)| (node.0, step))
                        .collect();
                    assert_eq!(got, upstream_neighbors(&map, idx), "at {idx:?}");
                }
            }
        }
    }
}
