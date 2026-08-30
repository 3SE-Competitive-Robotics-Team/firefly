//! 投影搜索（对照 `ann/projective_search.hpp`）。
//!
//! 将点云按等距柱状投影到 2D 索引图，查询时在投影邻域窗口内暴力比距离。

use nalgebra::Vector4;

use crate::ann::NearestNeighborSearch;
use crate::ann::knn_result::{INVALID_INDEX, KnnResult, KnnSetting};
use crate::points::traits::PointCloudTrait;

/// 等距柱状投影（对照 `EquirectangularProjection`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct EquirectangularProjection;

impl EquirectangularProjection {
    /// 投影到归一化图像坐标 `[0,1]×[0,1]`（对照 `operator()`）。
    pub fn project(&self, pt: &Vector4<f64>) -> [f64; 2] {
        let xyz = pt.fixed_rows::<3>(0).into_owned();
        if xyz.norm_squared() < 1e-3 {
            return [0.5, 0.5];
        }
        let bearing = xyz.normalize();
        let lat = -bearing[1].asin();
        let lon = bearing[0].atan2(bearing[2]);
        [
            lon / (2.0 * std::f64::consts::PI) + 0.5,
            lat / std::f64::consts::PI + 0.5,
        ]
    }
}

/// 边界钳制（对照 `BorderClamp`）：越界直接丢弃。
#[derive(Clone, Copy, Debug, Default)]
pub struct BorderClamp;

impl BorderClamp {
    #[allow(dead_code)]
    fn clamp(&self, x: i32, _width: i32) -> i32 {
        x
    }
    fn in_bounds(&self, x: i32, width: i32) -> bool {
        x >= 0 && x < width
    }
}

/// 边界环绕（对照 `BorderRepeat`）：水平环绕。
#[derive(Clone, Copy, Debug, Default)]
pub struct BorderRepeat;

impl BorderRepeat {
    fn clamp(&self, x: i32, width: i32) -> i32 {
        if x < 0 {
            x + width
        } else if x >= width {
            x - width
        } else {
            x
        }
    }
    fn in_bounds(&self, _x: i32, _width: i32) -> bool {
        true
    }
}

/// 不持有点云所有权的投影搜索（对照 `UnsafeProjectiveSearch`）。
pub struct UnsafeProjectiveSearch<'a, P: PointCloudTrait, Proj = EquirectangularProjection> {
    points: &'a P,
    width: i32,
    height: i32,
    index_map: Vec<u32>,
    search_window_h: i32,
    search_window_v: i32,
    _proj: std::marker::PhantomData<Proj>,
}

impl<'a, P: PointCloudTrait> UnsafeProjectiveSearch<'a, P, EquirectangularProjection> {
    /// 构造（对照 `UnsafeProjectiveSearch(width, height, points)`）。
    pub fn new(width: i32, height: i32, points: &'a P) -> Self {
        Self::with_projection::<EquirectangularProjection>(width, height, points)
    }
}

impl<'a, P: PointCloudTrait, Proj> UnsafeProjectiveSearch<'a, P, Proj>
where
    Proj: Default + Clone,
    Proj: ProjectiveProjection,
{
    /// 以指定投影构造。
    pub fn with_projection<Pr: ProjectiveProjection>(
        width: i32,
        height: i32,
        points: &'a P,
    ) -> UnsafeProjectiveSearch<'a, P, Pr>
    where
        Pr: Default,
    {
        let mut index_map = vec![u32::MAX; (width * height) as usize];
        let proj = Pr::default();
        for i in 0..points.num_points() {
            let pt = points.point(i);
            let uv = proj.project(&pt);
            let u = (uv[0] * width as f64) as i32;
            let v = (uv[1] * height as f64) as i32;
            if u < 0 || u >= width || v < 0 || v >= height {
                continue;
            }
            let idx = (v * width + u) as usize;
            index_map[idx] = i as u32;
        }
        UnsafeProjectiveSearch {
            points,
            width,
            height,
            index_map,
            search_window_h: 10,
            search_window_v: 5,
            _proj: std::marker::PhantomData,
        }
    }

    /// 设置搜索窗口。
    pub fn set_window(&mut self, h: i32, v: i32) {
        self.search_window_h = h;
        self.search_window_v = v;
    }
}

/// 投影抽象。
pub trait ProjectiveProjection: Default {
    fn project(&self, pt: &Vector4<f64>) -> [f64; 2];
}

impl ProjectiveProjection for EquirectangularProjection {
    fn project(&self, pt: &Vector4<f64>) -> [f64; 2] {
        EquirectangularProjection::project(self, pt)
    }
}

impl<'a, P: PointCloudTrait, Proj> UnsafeProjectiveSearch<'a, P, Proj>
where
    Proj: ProjectiveProjection + Default,
{
    fn knn_search_inner(
        &self,
        query: &Vector4<f64>,
        result: &mut KnnResult,
        _setting: &KnnSetting,
    ) {
        let proj = Proj::default();
        let uv = proj.project(query);
        let u0 = (uv[0] * self.width as f64) as i32;
        let v0 = (uv[1] * self.height as f64) as i32;

        // 水平环绕、垂直钳制（对照 `BorderRepeat` / `BorderClamp`）
        let h_mode = BorderRepeat;
        let v_mode = BorderClamp;

        for du in -self.search_window_h..=self.search_window_h {
            let u = h_mode.clamp(u0 + du, self.width);
            if !h_mode.in_bounds(u0 + du, self.width) && !v_mode.in_bounds(u, self.width) {
                // actually repeat always in bounds
            }
            if u < 0 || u >= self.width {
                continue;
            }
            for dv in -self.search_window_v..=self.search_window_v {
                let v_raw = v0 + dv;
                if !v_mode.in_bounds(v_raw, self.height) {
                    continue;
                }
                let v = v_raw;
                let idx = (v * self.width + u) as usize;
                let pi = self.index_map[idx];
                if pi == u32::MAX {
                    continue;
                }
                let sq = (self.points.point(pi as usize) - query).norm_squared();
                result.push(pi as usize, sq);
            }
        }
    }

    /// k 近邻搜索（动态容量）。
    pub fn knn_search(
        &self,
        query: &Vector4<f64>,
        k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize {
        let mut result = KnnResult::new(k_indices, k_sq_dists, k);
        let setting = KnnSetting::default();
        self.knn_search_inner(query, &mut result, &setting);
        result.num_found()
    }

    /// 最近邻。
    pub fn nearest_neighbor_search(
        &self,
        query: &Vector4<f64>,
        k_index: &mut usize,
        k_sq_dist: &mut f64,
    ) -> usize {
        let mut idx = [INVALID_INDEX];
        let mut dist = [0.0];
        let n = self.knn_search(query, 1, &mut idx, &mut dist);
        *k_index = idx[0];
        *k_sq_dist = dist[0];
        n
    }
}

impl<'a, P: PointCloudTrait, Proj> NearestNeighborSearch for UnsafeProjectiveSearch<'a, P, Proj>
where
    Proj: ProjectiveProjection + Default + Send + Sync,
{
    fn knn_search(
        &self,
        query: &Vector4<f64>,
        k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize {
        self.knn_search(query, k, k_indices, k_sq_dists)
    }
}

/// 持有点云所有权的投影搜索（对照 `ProjectiveSearch`）。
pub struct ProjectiveSearch<P: PointCloudTrait, Proj = EquirectangularProjection> {
    points: P,
    search: UnsafeProjectiveSearchOwned<Proj>,
}

struct UnsafeProjectiveSearchOwned<Proj> {
    width: i32,
    height: i32,
    index_map: Vec<u32>,
    search_window_h: i32,
    search_window_v: i32,
    _proj: std::marker::PhantomData<Proj>,
}

impl<P: PointCloudTrait, Proj> ProjectiveSearch<P, Proj>
where
    Proj: ProjectiveProjection + Default,
{
    /// 构造（消费点云）。
    pub fn new(width: i32, height: i32, points: P) -> Self {
        let mut index_map = vec![u32::MAX; (width * height) as usize];
        let proj = Proj::default();
        for i in 0..points.num_points() {
            let pt = points.point(i);
            let uv = proj.project(&pt);
            let u = (uv[0] * width as f64) as i32;
            let v = (uv[1] * height as f64) as i32;
            if u < 0 || u >= width || v < 0 || v >= height {
                continue;
            }
            let idx = (v * width + u) as usize;
            index_map[idx] = i as u32;
        }
        Self {
            points,
            search: UnsafeProjectiveSearchOwned {
                width,
                height,
                index_map,
                search_window_h: 10,
                search_window_v: 5,
                _proj: std::marker::PhantomData,
            },
        }
    }

    /// 访问点云。
    pub fn points(&self) -> &P {
        &self.points
    }

    fn knn_search_inner(&self, query: &Vector4<f64>, result: &mut KnnResult) {
        let proj = Proj::default();
        let uv = proj.project(query);
        let u0 = (uv[0] * self.search.width as f64) as i32;
        let v0 = (uv[1] * self.search.height as f64) as i32;
        let h_mode = BorderRepeat;
        let v_mode = BorderClamp;
        for du in -self.search.search_window_h..=self.search.search_window_h {
            let u = h_mode.clamp(u0 + du, self.search.width);
            if u < 0 || u >= self.search.width {
                continue;
            }
            for dv in -self.search.search_window_v..=self.search.search_window_v {
                let v_raw = v0 + dv;
                if !v_mode.in_bounds(v_raw, self.search.height) {
                    continue;
                }
                let v = v_raw;
                let idx = (v * self.search.width + u) as usize;
                let pi = self.search.index_map[idx];
                if pi == u32::MAX {
                    continue;
                }
                let sq = (self.points.point(pi as usize) - query).norm_squared();
                result.push(pi as usize, sq);
            }
        }
    }

    /// k 近邻搜索。
    pub fn knn_search(
        &self,
        query: &Vector4<f64>,
        k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize {
        let mut result = KnnResult::new(k_indices, k_sq_dists, k);
        self.knn_search_inner(query, &mut result);
        result.num_found()
    }

    /// 最近邻。
    pub fn nearest_neighbor_search(
        &self,
        query: &Vector4<f64>,
        k_index: &mut usize,
        k_sq_dist: &mut f64,
    ) -> usize {
        let mut idx = [INVALID_INDEX];
        let mut dist = [0.0];
        let n = self.knn_search(query, 1, &mut idx, &mut dist);
        *k_index = idx[0];
        *k_sq_dist = dist[0];
        n
    }
}

impl<P: PointCloudTrait, Proj> NearestNeighborSearch for ProjectiveSearch<P, Proj>
where
    Proj: ProjectiveProjection + Default + Send + Sync,
{
    fn knn_search(
        &self,
        query: &Vector4<f64>,
        k: usize,
        k_indices: &mut [usize],
        k_sq_dists: &mut [f64],
    ) -> usize {
        self.knn_search(query, k, k_indices, k_sq_dists)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::points::point_cloud::PointCloud;
    use crate::points::traits::PointCloudMut;
    use nalgebra::Vector4;

    #[test]
    fn projective_search_finds_near() {
        let mut cloud = PointCloud::new();
        cloud.resize(3);
        cloud.set_point(0, Vector4::new(10.0, 0.0, 0.0, 1.0));
        cloud.set_point(1, Vector4::new(0.0, 10.0, 0.0, 1.0));
        cloud.set_point(2, Vector4::new(0.0, 0.0, 10.0, 1.0));

        let search = UnsafeProjectiveSearch::<PointCloud>::new(64, 32, &cloud);
        let mut idx = 0usize;
        let mut dist = 0.0f64;
        let n =
            search.nearest_neighbor_search(&Vector4::new(10.1, 0.0, 0.0, 1.0), &mut idx, &mut dist);
        assert_eq!(n, 1);
        // 最近应为索引 0（同方向）
        assert_eq!(idx, 0);
    }
}
