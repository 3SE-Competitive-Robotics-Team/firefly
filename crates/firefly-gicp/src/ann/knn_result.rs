//! KNN 结果容器（对照 official `ann/knn_result.hpp`）。
//!
//! 维护容量固定（或动态指定）的最近邻缓冲，按距离升序插入；并列距离（相等）
//! 不插入，保留先到者（对照 `push` 的 `distance >= worst_distance()` 早退）。

/// 无效索引哨兵（对照 `KnnResult::INVALID = std::numeric_limits<size_t>::max()`）。
pub const INVALID_INDEX: usize = usize::MAX;

/// KNN 搜索设置（对照 `KnnSetting`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct KnnSetting {
    /// 提前终止阈值：结果中最远距小于该值即视为满足（默认 0，永不提前终止）。
    pub epsilon: f64,
}

impl KnnSetting {
    /// 结果是否达到提前终止条件。
    pub fn fulfilled(&self, result: &KnnResult) -> bool {
        result.worst_distance() < self.epsilon
    }
}

/// KNN 结果容器：按距离升序维护 `(index, sq_dist)` 缓冲。
pub struct KnnResult<'a> {
    capacity: usize,
    num_found: usize,
    indices: &'a mut [usize],
    distances: &'a mut [f64],
}

impl<'a> KnnResult<'a> {
    /// 构造：`capacity` 个槽位初始化为哨兵（索引 `MAX`，距离 `+inf`）。
    pub fn new(indices: &'a mut [usize], distances: &'a mut [f64], capacity: usize) -> Self {
        for x in indices.iter_mut().take(capacity) {
            *x = INVALID_INDEX;
        }
        for x in distances.iter_mut().take(capacity) {
            *x = f64::MAX;
        }
        Self {
            capacity,
            num_found: 0,
            indices,
            distances,
        }
    }

    /// 缓冲区容量（最大邻居数）。
    pub fn buffer_size(&self) -> usize {
        self.capacity
    }

    /// 已找到的邻居数。
    pub fn num_found(&self) -> usize {
        self.num_found
    }

    /// 当前最远距（缓冲区末位槽）。
    pub fn worst_distance(&self) -> f64 {
        self.distances[self.capacity - 1]
    }

    /// 压入一个候选 `(index, distance)`；按距离升序插入，并列不插入。
    pub fn push(&mut self, index: usize, distance: f64) {
        if distance >= self.worst_distance() {
            return;
        }

        let buf = self.capacity;
        if buf == 1 {
            self.indices[0] = index;
            self.distances[0] = distance;
        } else {
            let mut insert_loc = self.num_found.min(buf - 1);
            while insert_loc > 0 && distance < self.distances[insert_loc - 1] {
                self.indices[insert_loc] = self.indices[insert_loc - 1];
                self.distances[insert_loc] = self.distances[insert_loc - 1];
                insert_loc -= 1;
            }
            self.indices[insert_loc] = index;
            self.distances[insert_loc] = distance;
        }

        self.num_found = (self.num_found + 1).min(buf);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sorted_insert_and_ties() {
        let mut idx = [0usize; 3];
        let mut dist = [0.0f64; 3];
        let mut r = KnnResult::new(&mut idx, &mut dist, 3);
        r.push(0, 2.0);
        r.push(1, 0.5);
        r.push(2, 1.5);
        // 并列 1.5 < 当前最远 2.0，插入并挤掉最远（对照 push 的 `>= worst` 早退）
        r.push(3, 1.5);
        assert_eq!(r.num_found(), 3);
        assert_eq!(r.distances, [0.5, 1.5, 1.5]);
        assert_eq!(r.indices, [1, 2, 3]);
    }

    #[test]
    fn capacity_one_replaces() {
        let mut idx = [0usize; 1];
        let mut dist = [0.0f64; 1];
        let mut r = KnnResult::new(&mut idx, &mut dist, 1);
        r.push(5, 3.0);
        assert_eq!(r.indices[0], 5);
        r.push(6, 1.0);
        assert_eq!(r.indices[0], 6);
        r.push(7, 9.0); // 9.0 >= 当前最远 1.0，不插入
        assert_eq!(r.indices[0], 6);
        assert_eq!(r.num_found(), 1);
    }

    #[test]
    fn full_buffer_rejects_farther() {
        let mut idx = [0usize; 2];
        let mut dist = [0.0f64; 2];
        let mut r = KnnResult::new(&mut idx, &mut dist, 2);
        r.push(1, 1.0);
        r.push(2, 2.0);
        r.push(3, 5.0); // 5.0 >= 2.0 不插入
        assert_eq!(r.num_found(), 2);
        assert_eq!(r.indices, [1, 2]);
        r.push(4, 1.5); // 1.5 < 2.0，替换最远
        assert_eq!(r.indices, [1, 4]);
        assert_eq!(r.distances, [1.0, 1.5]);
    }
}
