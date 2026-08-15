//! A* 网格搜索。

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
            max_expansions: 100_000,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct Node {
    idx: [usize; 3],
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
            .then_with(|| self.node.idx.cmp(&other.node.idx))
    }
}

pub struct Astar<'a> {
    map: &'a GridMap,
    config: AstarConfig,
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
        if map.is_occupied(p) {
            return false;
        }
    }
    true
}

impl<'a> Astar<'a> {
    #[must_use]
    pub fn new(map: &'a GridMap) -> Self {
        Self {
            map,
            config: AstarConfig::default(),
        }
    }

    #[must_use]
    pub fn with_config(map: &'a GridMap, config: AstarConfig) -> Self {
        Self { map, config }
    }

    /// # Errors
    ///
    /// `OutOfRange`：起点/终点在地图外；`InvalidArgument`：终点被占据；
    /// `NotFound`：无可达路径；`Convergence`：超出扩展上限。
    #[fastrace::trace]
    #[logcall::logcall("debug", output = "")]
    pub fn search(&self, start: Vector3<f64>, goal: Vector3<f64>) -> firefly_error::Result<Path> {
        let start_idx = self
            .map
            .index_of(start)
            .ok_or_else(|| Error::new(ErrorKind::OutOfRange, "start is outside the map"))?;
        let goal_idx = self
            .map
            .index_of(goal)
            .ok_or_else(|| Error::new(ErrorKind::OutOfRange, "goal is outside the map"))?;
        if self.map.state(goal_idx) == firefly_map::VoxelState::Occupied {
            return Err(Error::new(ErrorKind::InvalidArgument, "goal is occupied"));
        }

        let start_node = Node { idx: start_idx };
        let goal_node = Node { idx: goal_idx };

        let mut open = BinaryHeap::new();
        let mut g_score = std::collections::HashMap::new();
        let mut came_from = std::collections::HashMap::new();

        g_score.insert(start_node, 0.0f64);
        open.push(Entry {
            node: start_node,
            g: 0.0,
            f: self.config.heuristic_weight * self.heuristic(start_idx, goal_idx),
        });

        let mut expansions = 0usize;
        while let Some(Entry { node, g, .. }) = open.pop() {
            if node == goal_node {
                return Ok(self.reconstruct(&came_from, node, start, goal));
            }
            if expansions >= self.config.max_expansions {
                return Err(Error::temporary(
                    ErrorKind::Convergence,
                    "a* exceeded expansion limit",
                ));
            }
            expansions += 1;
            for nb in self.neighbors(node) {
                let tentative = g + self.step_cost(node.idx, nb.idx);
                if tentative < *g_score.get(&nb).unwrap_or(&f64::INFINITY) {
                    g_score.insert(nb, tentative);
                    came_from.insert(nb, node);
                    open.push(Entry {
                        node: nb,
                        g: tentative,
                        f: tentative
                            + self.config.heuristic_weight * self.heuristic(nb.idx, goal_idx),
                    });
                }
            }
        }
        Err(Error::temporary(ErrorKind::NotFound, "no path found"))
    }

    fn heuristic(&self, a: [usize; 3], b: [usize; 3]) -> f64 {
        let r = self.map.resolution();
        let da = (a[0] as f64 - b[0] as f64) * r;
        let db = (a[1] as f64 - b[1] as f64) * r;
        let dc = (a[2] as f64 - b[2] as f64) * r;
        (da * da + db * db + dc * dc).sqrt()
    }

    fn step_cost(&self, a: [usize; 3], b: [usize; 3]) -> f64 {
        self.heuristic(a, b)
    }

    fn neighbors(&self, node: Node) -> impl Iterator<Item = Node> + '_ {
        let [x, y, z] = node.idx;
        let [dx, dy, dz] = self.map.dims();
        [
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
            (0, 0, 1),
            (0, 0, -1),
        ]
        .into_iter()
        .filter_map(move |(i, j, k)| {
            let nx = x as i32 + i;
            let ny = y as i32 + j;
            let nz = z as i32 + k;
            if nx < 0 || ny < 0 || nz < 0 || nx >= dx as i32 || ny >= dy as i32 || nz >= dz as i32 {
                return None;
            }
            let idx = [nx as usize, ny as usize, nz as usize];
            if self.map.state(idx) == firefly_map::VoxelState::Occupied {
                return None;
            }
            Some(Node { idx })
        })
    }

    fn reconstruct(
        &self,
        came_from: &std::collections::HashMap<Node, Node>,
        goal: Node,
        start: Vector3<f64>,
        goal_point: Vector3<f64>,
    ) -> Path {
        let mut nodes = vec![goal];
        let mut current = goal;
        while let Some(prev) = came_from.get(&current) {
            nodes.push(*prev);
            current = *prev;
        }
        nodes.reverse();
        let r = self.map.resolution();
        let origin = self.map.origin();
        let mut points: Vec<Vector3<f64>> = nodes
            .into_iter()
            .map(|n| {
                Vector3::new(
                    origin.x + (n.idx[0] as f64 + 0.5) * r,
                    origin.y + (n.idx[1] as f64 + 0.5) * r,
                    origin.z + (n.idx[2] as f64 + 0.5) * r,
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

#[cfg(test)]
mod tests {
    use super::*;

    fn empty_map() -> GridMap {
        firefly_map::GridMapBuilder::new(1.0, [10, 10, 10])
            .build()
            .unwrap()
    }

    #[test]
    fn straight_line_path() {
        let map = empty_map();
        let astar = Astar::new(&map);
        let path = astar
            .search(Vector3::new(0.5, 0.5, 0.5), Vector3::new(9.5, 0.5, 0.5))
            .unwrap();
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
        let astar = Astar::new(&map);
        let r = astar.search(Vector3::new(0.5, 0.5, 0.5), Vector3::new(9.5, 9.5, 0.5));
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
        let astar = Astar::new(&map);
        let path = astar
            .search(Vector3::new(0.5, 4.5, 0.5), Vector3::new(9.5, 4.5, 0.5))
            .unwrap();
        assert!(path.points().len() > 10);
    }
}
