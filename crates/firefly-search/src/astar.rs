//! A* 网格搜索。
//!
//! 对齐官方 EGO-Planner `dyn_a_star`：26 邻域（代价 `√(dx²+dy²+dz²)` 格长）、
//! 节点池预分配 + 世代计数复用（`rounds` 区分每次搜索，免重置）。

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use firefly_error::{Error, ErrorKind};
use firefly_map::GridMap;
use nalgebra::Vector3;

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

impl PartialEq for Entry {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}

impl Eq for Entry {}

impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Entry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f
            .partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| self.node.cmp(&other.node))
    }
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
}

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
        }
    }

    /// # Errors
    ///
    /// `OutOfRange`：起点/终点在地图外；`InvalidArgument`：终点被占据；
    /// `NotFound`：无可达路径；`Convergence`：超出扩展上限。
    #[fastrace::trace]
    pub fn search(
        &mut self,
        map: &GridMap,
        start: Vector3<f64>,
        goal: Vector3<f64>,
    ) -> firefly_error::Result<Path> {
        let start_idx = map
            .index_of(start)
            .ok_or_else(|| Error::new(ErrorKind::OutOfRange, "start is outside the map"))?;
        let goal_idx = map
            .index_of(goal)
            .ok_or_else(|| Error::new(ErrorKind::OutOfRange, "goal is outside the map"))?;
        if map.is_inflated(goal_idx) {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "goal is occupied (inflated)",
            ));
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

        let mut open = BinaryHeap::new();
        open.push(Entry {
            node: start_node,
            g: 0.0,
            f: self.config.heuristic_weight * Self::heuristic(map, start_idx, goal_idx),
        });

        let mut expansions = 0usize;
        while let Some(Entry { node, g, .. }) = open.pop() {
            if self.rounds[node.0 as usize] != round
                || self.state[node.0 as usize] != NodeState::Open
            {
                continue;
            }
            if g > self.g_score[node.0 as usize] {
                continue; // 陈旧条目（节点已通过更优 g 重新入队）
            }
            if node == goal_node {
                return Ok(self.reconstruct(map, goal_node, start, goal));
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
            for (nb, step_len) in Self::neighbors(map, idx) {
                let tentative = g + step_len;
                let explored = self.rounds[nb.0 as usize] == round;
                if !explored {
                    self.touch(nb, round);
                    self.g_score[nb.0 as usize] = tentative;
                    self.came_from[nb.0 as usize] = node;
                    self.state[nb.0 as usize] = NodeState::Open;
                    open.push(Entry {
                        node: nb,
                        g: tentative,
                        f: tentative
                            + self.config.heuristic_weight
                                * Self::heuristic(map, Self::decode(map, nb), goal_idx),
                    });
                } else if tentative < self.g_score[nb.0 as usize] {
                    self.g_score[nb.0 as usize] = tentative;
                    self.came_from[nb.0 as usize] = node;
                    open.push(Entry {
                        node: nb,
                        g: tentative,
                        f: tentative
                            + self.config.heuristic_weight
                                * Self::heuristic(map, Self::decode(map, nb), goal_idx),
                    });
                }
            }
        }
        Err(Error::temporary(ErrorKind::NotFound, "no path found"))
    }

    /// 26 邻域（官方 `dx,dy,dz ∈ {-1,0,1}`），代价 = 格长 `√(dx²+dy²+dz²)`。
    fn neighbors(map: &GridMap, idx: [usize; 3]) -> Vec<(Node, f64)> {
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
                    let idx = [nx as usize, ny as usize, nz as usize];
                    if map.is_inflated(idx) {
                        continue;
                    }
                    let step = f64::from(i * i + j * j + k * k).sqrt() * map.resolution();
                    out.push((Node(Self::linear(map, idx)), step));
                }
            }
        }
        out
    }

    fn heuristic(map: &GridMap, a: [usize; 3], b: [usize; 3]) -> f64 {
        let r = map.resolution();
        let da = (a[0] as f64 - b[0] as f64) * r;
        let db = (a[1] as f64 - b[1] as f64) * r;
        let dc = (a[2] as f64 - b[2] as f64) * r;
        (da * da + db * db + dc * dc).sqrt()
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
        // 端点修正：体素中心替换为精确的 start/goal
        if let Some(first) = points.first_mut() {
            *first = start;
        }
        if let Some(last) = points.last_mut() {
            *last = goal_point;
        }
        Path { points }
    }
}

/// 字符串拉直：删除可直线直达的中间点，使网格路径贴合直线。
#[must_use]
pub fn simplify_path(map: &GridMap, path: &[Vector3<f64>]) -> Vec<Vector3<f64>> {
    if path.len() <= 2 {
        return path.to_vec();
    }
    let mut result = vec![path[0]];
    let mut i = 0usize;
    while i < path.len() - 1 {
        let mut j = path.len() - 1;
        while j > i + 1 && !line_is_clear(map, path[i], path[j]) {
            j -= 1;
        }
        result.push(path[j]);
        i = j;
    }
    result
}

fn line_is_clear(map: &GridMap, a: Vector3<f64>, b: Vector3<f64>) -> bool {
    let dist = (b - a).norm();
    let steps = (dist / map.resolution() * 2.0).ceil() as usize;
    for k in 1..steps {
        let p = a + (b - a) * (k as f64 / steps as f64);
        // 必须用膨胀检查（与 A* 搜索、MINCO 净距一致）：仅检查原始占用会漏掉
        // 直线"擦边穿越"膨胀层——simplify 会把穿过膨胀区的直线判为 clear，
        // 全局路径贴柱、本地规划因净距不足卡死。
        if map.is_occupied_inflated(p) {
            return false;
        }
    }
    true
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
}

#[cfg(test)]
mod line_clear_tests {
    use super::line_is_clear;
    use firefly_map::{GridMapBuilder, VoxelState};

    /// 回归：simplify 的直线检查必须用**膨胀**判断——仅查原始占用会漏掉
    /// 直线擦边穿过膨胀层（全局路径贴柱→MINCO 净距不足→卡死）。
    #[test]
    fn line_clipping_inflation_is_blocked() {
        let mut map = GridMapBuilder::new(1.0, [10, 10, 10])
            .with_obstacles_inflation(1.0)
            .build()
            .unwrap();
        // 一条 y=1、z=1 的细障碍（原始仅 1 格厚）
        map.set_state([5, 1, 1], VoxelState::Occupied);
        map.inflate_obstacles(); // 膨胀到 y∈[0,2]

        // 直线沿 y=1.0 穿过原始占用格 → 应 blocked
        assert!(!line_is_clear(
            &map,
            nalgebra::Vector3::new(0.0, 1.0, 1.0),
            nalgebra::Vector3::new(9.0, 1.0, 1.0)
        ));
        // 直线沿 y=0.5 穿过**仅膨胀区**（原始无占用）→ 现在也须 blocked
        assert!(
            !line_is_clear(
                &map,
                nalgebra::Vector3::new(0.0, 0.5, 1.0),
                nalgebra::Vector3::new(9.0, 0.5, 1.0)
            ),
            "擦边穿越膨胀层必须判定为不清（原用 is_occupied 漏检）"
        );
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
