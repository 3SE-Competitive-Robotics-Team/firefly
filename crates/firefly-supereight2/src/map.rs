/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2023 Nils Funk
 * SPDX-FileCopyrightText: 2019-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Multi-resolution occupancy map coordinates and query interfaces.
use crate::eigen_utils::{
    AlignedBox, inverse_transform, transform_bounds, transform_point, transform_points,
};
use crate::{
    data::{DataConfig, OccupancyData},
    image::invalid,
    octree::Octree,
    visitor,
};
use firefly_error::Result;
use nalgebra::{IsometryMatrix3, Vector3};
#[derive(Clone, Copy, Debug)]
pub struct MapConfig {
    pub dim: Vector3<f32>,
    pub res: f32,
    pub t_mw: IsometryMatrix3<f32>,
}
impl Default for MapConfig {
    fn default() -> Self {
        Self {
            dim: Vector3::repeat(10.0),
            res: 0.1,
            t_mw: IsometryMatrix3::translation(5.0, 5.0, 5.0),
        }
    }
}
#[derive(Clone, Debug)]
pub struct Map {
    pub octree: Octree,
    pub resolution: f32,
    pub dim: Vector3<f32>,
    pub t_mw: IsometryMatrix3<f32>,
    pub t_wm: IsometryMatrix3<f32>,
    pub data_config: DataConfig,
}
impl Map {
    /// # Errors
    /// Returns an error for nonpositive/non-finite dimensions or resolution, an unrepresentable
    /// voxel count, or invalid octree dimensions.
    pub fn new(dim: Vector3<f32>, res: f32, data_config: DataConfig) -> Result<Self> {
        Self::from_config(
            MapConfig {
                t_mw: IsometryMatrix3::translation(dim.x / 2.0, dim.y / 2.0, dim.z / 2.0),
                dim,
                res,
            },
            data_config,
            8,
        )
    }
    /// # Errors
    /// Returns an error for nonpositive/non-finite dimensions or resolution, an unrepresentable
    /// voxel count, or invalid block/tree sizes.
    pub fn from_config(
        config: MapConfig,
        data_config: DataConfig,
        block_size: i32,
    ) -> Result<Self> {
        if !config.res.is_finite()
            || config.res <= 0.0
            || config.dim.iter().any(|v| !v.is_finite() || *v <= 0.0)
        {
            return Err(invalid(
                "positive finite map dimensions/resolution required",
            ));
        }
        let size = (config.dim.max() / config.res).ceil();
        if size > i32::MAX as f32 {
            return Err(invalid("map voxel count overflow"));
        }
        let octree = Octree::new(size as i32, block_size)?;
        let dim = Vector3::repeat(octree.size as f32 * config.res);
        let t_wm = inverse_transform(&config.t_mw);
        let result = Self {
            octree,
            resolution: config.res,
            dim,
            t_mw: config.t_mw,
            t_wm,
            data_config,
        };
        let origin = result.t_mw.translation.vector;
        if !result.contains_map_point(&origin) {
            log::warn!("World origin is outside the map");
        }
        Ok(result)
    }
    fn contains_map_point(&self, p: &Vector3<f32>) -> bool {
        (0..3).all(|i| p[i] >= 0.0 && p[i] < self.dim[i])
    }
    #[must_use]
    pub fn contains(&self, p: &Vector3<f32>) -> bool {
        self.contains_map_point(&transform_point(&self.t_mw, p))
    }
    #[must_use]
    pub fn point_to_voxel(&self, p: &Vector3<f32>) -> Vector3<f32> {
        transform_point(&self.t_mw, p) / self.resolution
    }
    #[must_use]
    pub fn point_to_voxel_checked(&self, p: &Vector3<f32>) -> Option<Vector3<f32>> {
        self.contains(p).then(|| self.point_to_voxel(p))
    }
    /// # Errors
    /// Returns an error if a transformed coordinate is non-finite or outside the `i32` range.
    pub fn point_to_voxel_index(&self, p: &Vector3<f32>) -> Result<Vector3<i32>> {
        let v = self.point_to_voxel(p);
        if v.iter()
            .any(|x| !x.is_finite() || *x < i32::MIN as f32 || *x >= i32::MAX as f32)
        {
            return Err(invalid("voxel coordinate outside integer range"));
        }
        Ok(v.map(|x| x as i32))
    }
    #[must_use]
    pub fn voxel_to_point(&self, coord: &Vector3<i32>, stride: i32) -> Vector3<f32> {
        transform_point(
            &self.t_wm,
            &((coord.cast::<f32>() + Vector3::repeat(stride as f32 * 0.5)) * self.resolution),
        )
    }
    #[must_use]
    pub fn voxel_to_corner_points(&self, coord: &Vector3<i32>, stride: i32) -> [Vector3<f32>; 8] {
        let corners = std::array::from_fn(|i| {
            let offset = Vector3::new(
                i32::from(i & 1 != 0),
                i32::from(i & 2 != 0),
                i32::from(i & 4 != 0),
            ) * stride;
            (coord + offset).cast::<f32>() * self.resolution
        });
        transform_points(&self.t_wm, &corners)
    }
    /// World-space bounds of allocated octants, with the source half-open maximum.
    pub fn aabb(&self) -> AlignedBox<3> {
        let Some((min, max)) = self.octree.aabb else {
            return AlignedBox {
                min: Vector3::repeat(f32::MAX),
                max: Vector3::repeat(f32::MIN),
            };
        };
        let low = min.cast::<f32>();
        let high = (max + Vector3::repeat(1)).cast::<f32>().map(f32::next_down);
        let linear = self.t_wm.rotation.into_inner() * self.resolution;
        transform_bounds(
            &linear,
            &self.t_wm.translation.vector,
            &AlignedBox {
                min: low,
                max: high,
            },
        )
    }
    /// # Errors
    /// Returns an error if the query produces unrepresentable voxel indices or requires
    /// invalid/unallocated tree data or scales.
    pub fn get_data(&self, p: &Vector3<f32>, safe: bool) -> Result<OccupancyData> {
        if safe && !self.contains(p) {
            return Ok(OccupancyData::default());
        }
        visitor::get_data(&self.octree, &self.point_to_voxel_index(p)?)
    }
    /// # Errors
    /// Returns an error if the query produces unrepresentable voxel indices or requires
    /// invalid/unallocated tree data or scales.
    pub fn get_min_data(&self, p: &Vector3<f32>, scale: i32, safe: bool) -> Result<OccupancyData> {
        if safe && !self.contains(p) {
            return Ok(OccupancyData::default());
        }
        visitor::get_min_data(&self.octree, &self.point_to_voxel_index(p)?, scale)
    }
    /// # Errors
    /// Returns an error if the query produces unrepresentable voxel indices or requires
    /// invalid/unallocated tree data or scales.
    pub fn get_max_data(&self, p: &Vector3<f32>, scale: i32, safe: bool) -> Result<OccupancyData> {
        if safe && !self.contains(p) {
            return Ok(OccupancyData::default());
        }
        visitor::get_max_data(&self.octree, &self.point_to_voxel_index(p)?, scale)
    }
    /// # Errors
    /// Returns an error if the query produces unrepresentable voxel indices or requires
    /// invalid/unallocated tree data or scales.
    pub fn interp(
        &self,
        p: &Vector3<f32>,
        scale: i32,
        safe: bool,
        valid: impl Fn(&OccupancyData) -> bool,
        get: impl Fn(&OccupancyData) -> f32,
    ) -> Result<Option<(f32, i32)>> {
        if safe && !self.contains(p) {
            return Ok(None);
        }
        visitor::interp(&self.octree, &self.point_to_voxel(p), scale, valid, get)
    }
    /// # Errors
    /// Returns an error if the query produces unrepresentable voxel indices or requires
    /// invalid/unallocated tree data or scales.
    pub fn interp_field(
        &self,
        p: &Vector3<f32>,
        scale: i32,
        safe: bool,
    ) -> Result<Option<(f32, i32)>> {
        self.interp(
            p,
            scale,
            safe,
            |d| d.field.valid(),
            OccupancyData::get_field,
        )
    }
    /// # Errors
    /// Returns an error if the query produces unrepresentable voxel indices or requires
    /// invalid/unallocated tree data or scales.
    pub fn grad(
        &self,
        p: &Vector3<f32>,
        scale: i32,
        safe: bool,
        valid: impl Fn(&OccupancyData) -> bool,
        get: impl Fn(&OccupancyData) -> f32,
    ) -> Result<Option<(Vector3<f32>, i32)>> {
        if safe && !self.contains(p) {
            return Ok(None);
        }
        Ok(
            visitor::grad(&self.octree, &self.point_to_voxel(p), scale, valid, get)?
                .map(|(v, s)| (v / self.resolution, s)),
        )
    }
    /// # Errors
    /// Returns an error if the query produces unrepresentable voxel indices or requires
    /// invalid/unallocated tree data or scales.
    pub fn grad_field(
        &self,
        p: &Vector3<f32>,
        scale: i32,
        safe: bool,
    ) -> Result<Option<(Vector3<f32>, i32)>> {
        self.grad(
            p,
            scale,
            safe,
            |d| d.field.valid(),
            OccupancyData::get_field,
        )
    }
}

impl MapConfig {
    /// # Errors
    /// Returns an error for incompatible map option types or a transform that is not finite and
    /// rigid.
    pub fn read_toml(&mut self, document: &toml::Value) -> Result<()> {
        let Some(t) = crate::sensor::config_section(document, "map")? else {
            return Ok(());
        };
        if let Some(a) = crate::sensor::config_array::<3>(t, "dim")? {
            self.dim = Vector3::from(a);
        }
        crate::sensor::config_float(t, "res", &mut self.res)?;
        self.t_mw = crate::sensor::config_transform(
            t,
            "T_MW",
            "t_MW",
            "R_MW",
            IsometryMatrix3::translation(self.dim.x / 2.0, self.dim.y / 2.0, self.dim.z / 2.0)
                .to_homogeneous(),
        )?;
        Ok(())
    }
}
impl std::fmt::Display for MapConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "dim: {}\nres: {}\nT_MW: {}",
            self.dim.transpose(),
            self.res,
            self.t_mw.to_homogeneous()
        )
    }
}
