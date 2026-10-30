//! 栅格配置、参数校验与缓冲初始化。

use super::{GridMap, VirtualWall};
use firefly_error::{Error, ErrorKind};
use nalgebra::Vector3;
use std::collections::HashSet;

#[derive(Debug)]
pub struct GridMapBuilder {
    origin: Vector3<f64>,
    resolution: f64,
    dims: [usize; 3],
    virtual_wall: Option<VirtualWall>,
    /// 障碍膨胀半径（米，官方 `grid_map/obstacles_inflation`）。
    obstacles_inflation: f64,
    /// 局部更新范围（米，官方 `grid_map/local_update_range_x/y/z`，launch
    /// advanced_param.xml:79-81 默认 5.5/5.5/2.0）。Some 时覆盖 `dims`：
    /// buffer 尺寸 = 2 × ceil(range / resolution)（官方 `grid_map.cpp:55`）。
    local_update_range: Option<[f64; 3]>,
}

impl GridMapBuilder {
    #[must_use]
    pub fn new(resolution: f64, dims: [usize; 3]) -> Self {
        Self {
            origin: Vector3::zeros(),
            resolution,
            dims,
            virtual_wall: None,
            obstacles_inflation: 0.2,
            local_update_range: None,
        }
    }

    #[must_use]
    pub fn with_origin(mut self, origin: Vector3<f64>) -> Self {
        self.origin = origin;
        self
    }

    /// 启用虚拟地面/天花板（单位：米，世界坐标 z；单侧不启用传 ±∞ 哨兵）。
    #[must_use]
    pub fn with_virtual_wall(mut self, ground: f64, ceil: f64) -> Self {
        self.virtual_wall = Some(VirtualWall { ground, ceil });
        self
    }

    /// 覆盖障碍膨胀半径（米，默认 0.2）；超过 4 格时构建自动放大分辨率（官方 grid_map.cpp:44-48）。
    #[must_use]
    pub fn with_obstacles_inflation(mut self, radius: f64) -> Self {
        self.obstacles_inflation = radius;
        self
    }

    /// 以局部更新范围（米）确定 buffer 尺寸 = 2 × ceil(range/resolution)
    /// （官方 `ringbuffer_size3i_ = 2 * local_update_range3i_`，`grid_map.cpp:55`），
    /// 覆盖 `new(resolution, dims)` 给出的 dims。官方 launch 默认 5.5/5.5/2.0。
    #[must_use]
    pub fn with_local_update_range(mut self, range: [f64; 3]) -> Self {
        self.local_update_range = Some(range);
        self
    }

    /// # Errors
    ///
    /// `InvalidArgument`：resolution 非正、dims 含零、或 `local_update_range`
    /// 非有限/非正。
    pub fn build(self) -> firefly_error::Result<GridMap> {
        if self.resolution <= 0.0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "resolution must be positive",
            ));
        }
        if let Some(range) = self.local_update_range
            && range.iter().any(|&v| !v.is_finite() || v <= 0.0)
        {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "local_update_range must be finite and positive",
            ));
        }
        // 膨胀步长与分辨率（官方 grid_map.cpp:44-48）：inf_grid > 4 时放大分辨率；
        // 先定分辨率再换算范围/尺寸（与官方顺序一致）
        let mut resolution = self.resolution;
        let mut inf_grid = ((self.obstacles_inflation - 1e-5) / resolution).ceil() as i32;
        if inf_grid > 4 {
            inf_grid = 4;
            resolution = self.obstacles_inflation / f64::from(inf_grid);
        }
        let dims = match self.local_update_range {
            Some(range) => {
                let inv = 1.0 / resolution;
                [
                    2 * (range[0] * inv).ceil().max(1.0) as usize,
                    2 * (range[1] * inv).ceil().max(1.0) as usize,
                    2 * (range[2] * inv).ceil().max(1.0) as usize,
                ]
            }
            None => self.dims,
        };
        if dims.contains(&0) {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "dims must be positive",
            ));
        }
        let capacity = dims[0] * dims[1] * dims[2];
        // 膨胀缓冲每侧多 inf_grid 格（官方 grid_map.cpp:56
        // `ringbuffer_inf_size3i_ = ringbuffer_size3i_ + 2*inf_grid`）
        let g = inf_grid.unsigned_abs() as usize;
        let inf_dims = [dims[0] + 2 * g, dims[1] + 2 * g, dims[2] + 2 * g];
        let inf_capacity = inf_dims[0] * inf_dims[1] * inf_dims[2];

        // 初始窗口覆盖整个 buffer：低维全局索引 [0, dims)，环形原点对齐 0，
        // 与静态地图行为一致（首次 move_ring_buffer 才开始平移）
        let dims_i64 = [dims[0] as i64, dims[1] as i64, dims[2] as i64];
        let ig = i64::from(inf_grid);
        Ok(GridMap {
            world_origin: self.origin,
            resolution,
            dims,
            inf_dims,
            local_update_range3i: [dims_i64[0] / 2, dims_i64[1] / 2, dims_i64[2] / 2],
            low: [0; 3],
            up: [dims_i64[0] - 1, dims_i64[1] - 1, dims_i64[2] - 1],
            inf_low: [-ig; 3],
            inf_up: [
                dims_i64[0] - 1 + ig,
                dims_i64[1] - 1 + ig,
                dims_i64[2] - 1 + ig,
            ],
            ring_origin: [0; 3],
            inf_ring_origin: [0; 3],
            center_last: [dims_i64[0] / 2, dims_i64[1] / 2, dims_i64[2] / 2],
            occupancy: super::Buffer::new(false, capacity),
            prior: std::sync::Arc::new(HashSet::new()),
            inflate: super::Buffer::new(0, inf_capacity),
            virtual_wall: self.virtual_wall,
            inf_grid,
            revision: 0,
            inflated_revision: 0,
            inflation_dirty: Vec::new(),
            inflation_pending: super::Buffer::new(false, capacity),
            inflation_rebuild: false,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{GridMapBuilder, VoxelState};

    #[test]
    fn builder_rejects_bad_resolution() {
        let r = GridMapBuilder::new(0.0, [10, 10, 10]).build();
        assert_eq!(r.unwrap_err().kind(), ErrorKind::InvalidArgument);
    }

    #[test]
    fn builder_rejects_bad_dims_and_range() {
        assert_eq!(
            GridMapBuilder::new(0.5, [0, 10, 10])
                .build()
                .unwrap_err()
                .kind(),
            ErrorKind::InvalidArgument
        );
        assert_eq!(
            GridMapBuilder::new(0.5, [1, 1, 1])
                .with_local_update_range([5.5, 0.0, 2.0])
                .build()
                .unwrap_err()
                .kind(),
            ErrorKind::InvalidArgument
        );
    }

    #[test]
    fn local_update_range_sets_buffer_size() {
        // 官方 launch advanced_param.xml:79-81 默认 5.5/5.5/2.0，res 0.4 →
        // ceil(5.5/0.4)=14、ceil(2/0.4)=5，buffer = 2× → [28,28,10]
        let m = GridMapBuilder::new(0.4, [1, 1, 1])
            .with_local_update_range([5.5, 5.5, 2.0])
            .build()
            .unwrap();
        assert_eq!(m.dims(), [28, 28, 10]);
        let lr = m.local_update_range();
        assert!((lr.x - 5.6).abs() < 1e-12); // 14 格 × 0.4
        assert!((lr.y - 5.6).abs() < 1e-12);
        assert!((lr.z - 2.0).abs() < 1e-12);
        // 初始窗口覆盖整个 buffer（静态语义）
        assert_eq!(m.index_of(Vector3::new(0.1, 0.1, 0.1)), Some([0, 0, 0]));
        assert_eq!(m.index_of(Vector3::new(11.1, 11.1, 3.9)), Some([27, 27, 9]));
        assert_eq!(m.index_of(Vector3::new(11.3, 0.1, 0.1)), None);
    }

    #[test]
    fn fractional_half_range_rounds_before_doubling_like_upstream() {
        let mut map = GridMapBuilder::new(1.0, [1; 3])
            .with_local_update_range([2.1, 3.2, 0.1])
            .build()
            .unwrap();
        assert_eq!(map.dims(), [6, 8, 2]);
        assert_eq!(map.local_update_range(), Vector3::new(3.0, 4.0, 1.0));
        map.move_ring_buffer(Vector3::zeros());
        assert_eq!(map.origin(), Vector3::new(-3.0, -4.0, -1.0));
        assert_eq!(map.index_of(Vector3::new(2.5, 3.5, 0.5)), Some([5, 7, 1]));
        assert_eq!(map.index_of(Vector3::new(3.0, 4.0, 1.0)), None);
    }

    #[test]
    fn builder_enlarges_resolution_when_inflation_exceeds_4() {
        // ceil((0.3-1e-5)/0.05)=6 > 4 → 分辨率放大为 0.3/4（官方 grid_map.cpp:44-48）
        let mut m = GridMapBuilder::new(0.05, [40, 40, 40])
            .with_obstacles_inflation(0.3)
            .build()
            .unwrap();
        assert!((m.resolution() - 0.3 / 4.0).abs() < 1e-12);
        // step=4：本体与 4 格邻域在膨胀层，5 格外不在
        m.set_state([20, 20, 20], VoxelState::Occupied);
        m.inflate_obstacles();
        assert_eq!(m.inflate_at([24, 20, 20]), 1);
        assert_eq!(m.inflate_at([25, 20, 20]), 0);
    }
}
