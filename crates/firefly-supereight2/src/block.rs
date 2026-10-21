/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Multi-resolution occupancy voxel blocks with shared finest-scale extrema.
use crate::{
    data::OccupancyData,
    octant::{OctantBase, OctantId, invalid},
};
use firefly_error::Result;
use nalgebra::Vector3;
#[derive(Clone, Debug)]
pub struct Block {
    pub base: OctantBase,
    pub max_scale: i32,
    pub min_scale: i32,
    pub current_scale: i32,
    pub init_data: OccupancyData,
    data: Vec<Vec<OccupancyData>>,
    min_data: Vec<Option<Vec<OccupancyData>>>,
    max_data: Vec<Option<Vec<OccupancyData>>>,
    pub curr_integr_count: Option<usize>,
    pub curr_observed_count: Option<usize>,
    pub buffer_integr_count: Option<usize>,
    pub buffer_observed_count: Option<usize>,
    buffer_scale: i32,
    buffer: Option<Vec<OccupancyData>>,
}
impl Block {
    /// # Errors
    /// Returns an error if `size` is not a positive power of two.
    pub fn new(
        coord: Vector3<i32>,
        size: i32,
        parent: OctantId,
        init_data: OccupancyData,
    ) -> Result<Self> {
        if size <= 0 || (size & (size - 1)) != 0 {
            return Err(invalid("block size must be a positive power of two"));
        }
        let max_scale = size.ilog2() as i32;
        Ok(Self {
            base: OctantBase::new(coord, size, Some(parent)),
            max_scale,
            min_scale: -1,
            current_scale: max_scale,
            init_data,
            data: vec![vec![init_data]],
            min_data: vec![None],
            max_data: vec![None],
            curr_integr_count: None,
            curr_observed_count: None,
            buffer_integr_count: None,
            buffer_observed_count: None,
            buffer_scale: -1,
            buffer: None,
        })
    }
    fn level(&self, scale: i32) -> Result<usize> {
        if scale < 0 || scale > self.max_scale {
            return Err(invalid("block scale outside allocated range"));
        }
        Ok((self.max_scale - scale) as usize)
    }
    /// # Errors
    /// Returns an error if the scale is outside the block or the required data level has not
    /// been allocated.
    pub fn size_at_scale(&self, scale: i32) -> Result<usize> {
        self.level(scale)?;
        Ok((self.base.size >> scale) as usize)
    }
    /// # Errors
    /// Returns an error if the scale is invalid or the voxel coordinate is outside the block.
    pub fn voxel_index(&self, coord: &Vector3<i32>, scale: i32) -> Result<usize> {
        let n = self.size_at_scale(scale)?;
        let offset = (coord - self.base.coord) / (1 << scale);
        if offset.iter().any(|v| *v < 0 || *v >= n as i32) {
            return Err(invalid("voxel coordinate outside block"));
        }
        Ok(offset.x as usize + offset.y as usize * n + offset.z as usize * n * n)
    }
    #[must_use]
    pub fn data(&self) -> OccupancyData {
        self.data[0][0]
    }
    #[must_use]
    pub fn min_data(&self) -> OccupancyData {
        self.min_data[0].as_ref().map_or(self.data[0][0], |v| v[0])
    }
    #[must_use]
    pub fn max_data(&self) -> OccupancyData {
        self.max_data[0].as_ref().map_or(self.data[0][0], |v| v[0])
    }
    /// # Errors
    /// Returns an error if the scale is outside the block or the required data level has not
    /// been allocated.
    pub fn data_at_scale(&self, scale: i32) -> Result<&[OccupancyData]> {
        let l = self.level(scale)?;
        self.data
            .get(l)
            .map(Vec::as_slice)
            .ok_or_else(|| invalid("mean data scale is not allocated"))
    }
    /// # Errors
    /// Returns an error if the scale is outside the block or the required data level has not
    /// been allocated.
    pub fn data_at_scale_mut(&mut self, scale: i32) -> Result<&mut [OccupancyData]> {
        let l = self.level(scale)?;
        self.data
            .get_mut(l)
            .map(Vec::as_mut_slice)
            .ok_or_else(|| invalid("mean data scale is not allocated"))
    }
    /// # Errors
    /// Returns an error if the scale is outside the block or the required data level has not
    /// been allocated.
    pub fn min_data_at_scale(&self, scale: i32) -> Result<&[OccupancyData]> {
        let l = self.level(scale)?;
        let v = self
            .min_data
            .get(l)
            .ok_or_else(|| invalid("minimum data scale is not allocated"))?;
        Ok(v.as_ref().map_or(self.data[l].as_slice(), Vec::as_slice))
    }
    /// # Errors
    /// Returns an error if the scale is outside the block or the required data level has not
    /// been allocated.
    pub fn min_data_at_scale_mut(&mut self, scale: i32) -> Result<&mut [OccupancyData]> {
        let l = self.level(scale)?;
        let v = self
            .min_data
            .get_mut(l)
            .ok_or_else(|| invalid("minimum data scale is not allocated"))?;
        Ok(if let Some(v) = v {
            v.as_mut_slice()
        } else {
            self.data
                .get_mut(l)
                .ok_or_else(|| invalid("mean data scale is not allocated"))?
                .as_mut_slice()
        })
    }
    /// # Errors
    /// Returns an error if the scale is outside the block or the required data level has not
    /// been allocated.
    pub fn max_data_at_scale(&self, scale: i32) -> Result<&[OccupancyData]> {
        let l = self.level(scale)?;
        let v = self
            .max_data
            .get(l)
            .ok_or_else(|| invalid("maximum data scale is not allocated"))?;
        Ok(v.as_ref().map_or(self.data[l].as_slice(), Vec::as_slice))
    }
    /// # Errors
    /// Returns an error if the scale is outside the block or the required data level has not
    /// been allocated.
    pub fn max_data_at_scale_mut(&mut self, scale: i32) -> Result<&mut [OccupancyData]> {
        let l = self.level(scale)?;
        let v = self
            .max_data
            .get_mut(l)
            .ok_or_else(|| invalid("maximum data scale is not allocated"))?;
        Ok(if let Some(v) = v {
            v.as_mut_slice()
        } else {
            self.data
                .get_mut(l)
                .ok_or_else(|| invalid("mean data scale is not allocated"))?
                .as_mut_slice()
        })
    }
    /// # Errors
    /// Returns an error if the coordinate/scale is invalid or the requested data level is
    /// absent.
    pub fn data_at(&self, coord: &Vector3<i32>, scale: i32) -> Result<OccupancyData> {
        let l = self.level(scale)?;
        if l >= self.data.len() {
            return Ok(self.init_data);
        }
        Ok(self.data_at_scale(scale)?[self.voxel_index(coord, scale)?])
    }
    /// # Errors
    /// Returns an error if the coordinate/scale is invalid or the requested minimum/mean data
    /// level is absent.
    pub fn min_data_at(&self, coord: &Vector3<i32>, scale: i32) -> Result<OccupancyData> {
        let l = self.level(scale)?;
        if l >= self.data.len() {
            return Ok(self.init_data);
        }
        Ok(self.min_data_at_scale(scale)?[self.voxel_index(coord, scale)?])
    }
    /// # Errors
    /// Returns an error if the coordinate/scale is invalid or the requested maximum/mean data
    /// level is absent.
    pub fn max_data_at(&self, coord: &Vector3<i32>, scale: i32) -> Result<OccupancyData> {
        let l = self.level(scale)?;
        if l >= self.data.len() {
            return Ok(self.init_data);
        }
        Ok(self.max_data_at_scale(scale)?[self.voxel_index(coord, scale)?])
    }
    /// # Errors
    /// Returns an error if the scale is outside the block or the required data level has not
    /// been allocated.
    pub fn allocate_down_to(&mut self, scale: i32) -> Result<()> {
        self.level(scale)?;
        if scale >= self.current_scale {
            return Ok(());
        }
        self.min_data.pop();
        self.max_data.pop();
        for s in (scale..=self.current_scale).rev() {
            let n = self.size_at_scale(s)?;
            if s != self.current_scale {
                self.data.push(vec![self.init_data; n * n * n]);
            }
            if s == scale {
                self.min_data.push(None);
                self.max_data.push(None);
            } else {
                let d = self
                    .data
                    .last()
                    .ok_or_else(|| invalid("mean data absent"))?
                    .clone();
                self.min_data.push(Some(d.clone()));
                self.max_data.push(Some(d));
            }
        }
        self.current_scale = scale;
        self.min_scale = scale;
        Ok(())
    }
    /// # Errors
    /// Returns an error if the scale is outside the block or the required data level has not
    /// been allocated.
    pub fn delete_up_to(&mut self, scale: i32) -> Result<()> {
        self.level(scale)?;
        if scale <= self.min_scale || self.min_scale == -1 {
            return Ok(());
        }
        let len = self.level(scale)? + 1;
        if len > self.data.len() {
            return Err(invalid("coarser block level absent"));
        }
        self.data.truncate(len);
        self.min_data.truncate(len);
        self.max_data.truncate(len);
        self.min_data[len - 1] = None;
        self.max_data[len - 1] = None;
        self.current_scale = scale;
        self.min_scale = scale;
        Ok(())
    }
    /// # Errors
    /// Returns an error for an invalid current scale or a negative/non-finite inherited
    /// observation weight.
    pub fn init_curr_count(&mut self) -> Result<()> {
        if self.init_data.field.observed {
            let n = self.size_at_scale(self.current_scale)?;
            if !self.init_data.field.weight.is_finite() || self.init_data.field.weight < 0.0 {
                return Err(invalid("invalid inherited observation weight"));
            }
            self.curr_integr_count = Some(self.init_data.field.weight as usize);
            self.curr_observed_count = Some(n * n * n);
        } else {
            self.reset_curr_count();
        }
        Ok(())
    }
    pub fn reset_curr_count(&mut self) {
        self.curr_integr_count = Some(0);
        self.curr_observed_count = Some(0);
    }
    /// # Errors
    /// Returns an error if the current integration counter has not been initialized.
    pub fn curr_integr_count(&self) -> Result<usize> {
        self.curr_integr_count
            .ok_or_else(|| invalid("current integration count uninitialized"))
    }
    /// # Errors
    /// Returns an error if the current observation counter has not been initialized.
    pub fn curr_observed_count(&self) -> Result<usize> {
        self.curr_observed_count
            .ok_or_else(|| invalid("current observation count uninitialized"))
    }
    /// # Errors
    /// Returns an error if the current integration counter has not been initialized.
    pub fn incr_curr_integr_count(&mut self) -> Result<()> {
        self.curr_integr_count = Some(self.curr_integr_count()?.wrapping_add(1));
        Ok(())
    }
    /// # Errors
    /// Returns an error if an increment is requested before initializing the current
    /// observation counter.
    pub fn incr_curr_observed_count(&mut self, increment: bool) -> Result<()> {
        if increment {
            self.curr_observed_count = Some(self.curr_observed_count()?.wrapping_add(1));
        }
        Ok(())
    }
    #[must_use]
    pub fn buffer_scale(&self) -> i32 {
        self.buffer_scale
    }
    /// # Errors
    /// Returns an error if the buffer integration counter has not been initialized.
    pub fn buffer_integr_count(&self) -> Result<usize> {
        self.buffer_integr_count
            .ok_or_else(|| invalid("buffer integration count uninitialized"))
    }
    /// # Errors
    /// Returns an error if the buffer observation counter has not been initialized.
    pub fn buffer_observed_count(&self) -> Result<usize> {
        self.buffer_observed_count
            .ok_or_else(|| invalid("buffer observation count uninitialized"))
    }
    pub fn reset_buffer_count(&mut self) {
        self.buffer_integr_count = Some(0);
        self.buffer_observed_count = Some(0);
    }
    pub fn reset_buffer(&mut self) {
        self.buffer = None;
        self.buffer_scale = -1;
        self.reset_buffer_count();
    }
    /// # Errors
    /// Returns an error if the scale is outside the block or the required data level has not
    /// been allocated.
    pub fn init_buffer(&mut self, scale: i32) -> Result<()> {
        self.level(scale)?;
        self.reset_buffer();
        self.buffer_scale = scale;
        if scale < self.current_scale {
            let n = self.size_at_scale(scale)?;
            self.buffer = Some(vec![OccupancyData::default(); n * n * n]);
        } else {
            self.data_at_scale(scale)?;
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if the buffer is absent, aliases an unavailable level, or `index` is
    /// out of bounds.
    pub fn buffer_data(&self, index: usize) -> Result<&OccupancyData> {
        let values = if self.buffer_scale < self.current_scale {
            self.buffer
                .as_deref()
                .ok_or_else(|| invalid("buffer absent"))?
        } else {
            self.data_at_scale(self.buffer_scale)?
        };
        values
            .get(index)
            .ok_or_else(|| invalid("buffer index outside scale"))
    }
    /// # Errors
    /// Returns an error if the buffer is absent, aliases an unavailable level, or `index` is
    /// out of bounds.
    pub fn buffer_data_mut(&mut self, index: usize) -> Result<&mut OccupancyData> {
        let values = if self.buffer_scale < self.current_scale {
            self.buffer
                .as_deref_mut()
                .ok_or_else(|| invalid("buffer absent"))?
        } else {
            self.data_at_scale_mut(self.buffer_scale)?
        };
        values
            .get_mut(index)
            .ok_or_else(|| invalid("buffer index outside scale"))
    }
    /// # Errors
    /// Returns an error if a requested increment needs an uninitialized buffer counter or
    /// invalid buffer level.
    pub fn incr_buffer_observed_count(&mut self, increment: bool) -> Result<()> {
        if increment {
            self.buffer_observed_count = Some(self.buffer_observed_count()?.wrapping_add(1));
        }
        Ok(())
    }
    fn normalized_buffer_observations(&self) -> Result<(usize, usize)> {
        self.level(self.buffer_scale)?;
        self.level(self.current_scale)?;
        let a = self
            .buffer_observed_count()?
            .wrapping_mul((1usize << self.buffer_scale).pow(3));
        let b = self
            .curr_observed_count()?
            .wrapping_mul((1usize << self.current_scale).pow(3));
        Ok((a, b))
    }
    /// # Errors
    /// Returns an error if a requested increment needs an uninitialized buffer integration
    /// counter.
    pub fn incr_buffer_integr_count(&mut self, increment: bool) -> Result<()> {
        let enough = if increment {
            true
        } else {
            let (a, b) = self.normalized_buffer_observations()?;
            (a as f64) >= 0.90 * b as f64
        };
        if enough {
            self.buffer_integr_count = Some(self.buffer_integr_count()?.wrapping_add(1));
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error if buffer/current counters or the scale data needed for the switch are
    /// unavailable.
    pub fn switch_data(&mut self) -> Result<bool> {
        if self.buffer_integr_count()? < 20 {
            return Ok(false);
        }
        let (a, b) = self.normalized_buffer_observations()?;
        if (a as f64) < 0.9 * b as f64 {
            return Ok(false);
        }
        let scale = self.buffer_scale;
        if scale < self.current_scale {
            if scale + 1 != self.current_scale {
                return Err(invalid("finer buffer must differ by one scale"));
            }
            let values = self.buffer.take().ok_or_else(|| invalid("buffer absent"))?;
            self.data.push(values);
            self.min_data.push(None);
            self.max_data.push(None);
            let n = self.size_at_scale(scale + 1)?;
            let level = self.level(scale + 1)?;
            self.min_data[level] = Some(vec![OccupancyData::default(); n * n * n]);
            self.max_data[level] = Some(vec![OccupancyData::default(); n * n * n]);
            self.current_scale = scale;
            self.min_scale = scale;
        } else {
            self.delete_up_to(scale)?;
        }
        let level = self.level(scale)?;
        let mut newly = 0usize;
        for d in &mut self.data[level] {
            if d.field.weight > 0.0 && !d.field.observed {
                d.field.observed = true;
                newly += 1;
            }
        }
        self.buffer_observed_count = Some(self.buffer_observed_count()?.wrapping_add(newly));
        self.curr_integr_count = Some(self.buffer_integr_count()?);
        self.curr_observed_count = Some(self.buffer_observed_count()?);
        self.buffer = None;
        self.buffer_scale = -1;
        self.reset_buffer_count();
        Ok(true)
    }
}
