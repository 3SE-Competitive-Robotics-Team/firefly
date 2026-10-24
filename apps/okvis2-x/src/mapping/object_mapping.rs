/* FindAnything / OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2026 Technical University of Munich and ETH Zurich. SPDX-License-Identifier: BSD-3-Clause */
//! SAM/raycast segment association and submap object descriptors.
use crate::{deep_learning::utils::invalid, sensors::measurements::DeepLearningImageData};
use firefly_error::Result;
use purecv::core::{Matrix, types::Size};
use std::collections::{BTreeMap, BTreeSet};
pub type Descriptor = [f32; 768];
pub type MapId = u64;
pub type SegmentId = u16;
pub const NO_ID: SegmentId = 0;
pub const NOT_MAPPED: SegmentId = u16::MAX;
#[derive(Clone)]
pub struct SubmapObject {
    pub id: SegmentId,
    pub map_id: MapId,
    pub vl_feature: Descriptor,
    pub num_pixels: usize,
}
impl SubmapObject {
    #[must_use]
    pub fn new(id: SegmentId, map_id: MapId, vl_feature: Descriptor, num_pixels: usize) -> Self {
        Self {
            id,
            map_id,
            vl_feature,
            num_pixels,
        }
    }
    pub fn update_vision_language_descriptor(&mut self, new: &Descriptor, count: usize) {
        for (old, new) in self.vl_feature.iter_mut().zip(new) {
            *old = (self.num_pixels as f32 * *old + count as f32 * *new)
                / (self.num_pixels + count) as f32;
        }
        self.num_pixels += count;
    }
}
#[derive(Default)]
pub struct GlobalObject;
/// Map availability and support for semantic instance IDs.
pub trait ObjectMapBackend {
    fn contains_map(&self, map: MapId) -> bool;
    fn supports_colour_and_ids(&self) -> bool;
}
pub struct ObjectMap<B> {
    pub backend: B,
    object_counter: SegmentId,
    pub submap_objects: BTreeMap<MapId, BTreeMap<SegmentId, SubmapObject>>,
    pub global_objects: BTreeSet<SegmentId>,
}
impl<B: ObjectMapBackend> ObjectMap<B> {
    pub fn new(backend: B) -> Self {
        Self {
            backend,
            object_counter: 0,
            submap_objects: BTreeMap::new(),
            global_objects: BTreeSet::new(),
        }
    }
    pub fn contains_map(&self, map: MapId) -> bool {
        self.submap_objects.contains_key(&map)
    }
    pub fn obtain_new_id(&mut self) -> SegmentId {
        self.object_counter = self.object_counter.wrapping_add(1);
        self.object_counter
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn process_data(
        &mut self,
        map: MapId,
        invalid_depth: &Matrix<u8>,
        sam: &DeepLearningImageData,
        features: &DeepLearningImageData,
        surface: &Matrix<u16>,
        iou_threshold: f32,
    ) -> Result<Matrix<u16>> {
        let mut matched = BTreeSet::new();
        let result = self.track_segments(
            invalid_depth,
            sam,
            surface,
            &mut matched,
            0.00049,
            iou_threshold,
        )?;
        for id in matched {
            let mask = Matrix::from_vec(
                result.rows,
                result.cols,
                1,
                result
                    .data
                    .iter()
                    .map(|p| if *p == id { 255 } else { 0 })
                    .collect(),
            );
            if self
                .submap_objects
                .get(&map)
                .is_some_and(|objects| objects.contains_key(&id))
            {
                self.update_object(map, id, &mask, features)?;
            } else {
                self.add_new_object(map, &mask, features, id)?;
            }
        }
        Ok(result)
    }
    /// `percentage=0.002` and `iou_threshold=0.25` are the source API defaults.
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn track_segments(
        &mut self,
        invalid_depth: &Matrix<u8>,
        sam: &DeepLearningImageData,
        surface: &Matrix<u16>,
        matched: &mut BTreeSet<SegmentId>,
        percentage: f32,
        iou_threshold: f32,
    ) -> Result<Matrix<u16>> {
        Self::validate_segment_inputs(invalid_depth, surface)?;
        let (n, height, width, data) = sam_data(sam)?;
        let (rows, cols) = (invalid_depth.rows, invalid_depth.cols);
        let mut raycast = split(surface);
        let mut output = Matrix::new(rows, cols, 1);
        let mut assignable = vec![1u8; rows * cols];
        if raycast.is_empty() {
            return Ok(output);
        }
        raycast.remove(&NOT_MAPPED);
        let minimum = (percentage * cols as f32 * rows as f32) as usize;
        let mut sam_sizes = Vec::new();
        for mask_index in 0..n {
            let raw = &data[mask_index * height * width..(mask_index + 1) * height * width];
            let original = raw.iter().filter(|&&v| v != 0.).count();
            let mut mask = Matrix::from_vec(
                height,
                width,
                1,
                raw.iter()
                    .map(|x| x.round_ties_even().clamp(0., 255.) as u8)
                    .collect(),
            );
            if (height, width) != (rows, cols) {
                mask = purecv::imgproc::resize::resize(&mask, Size::new(cols, rows))
                    .map_err(|e| invalid(e.to_string(), "ObjectMap::track_segments"))?;
            }
            for (value, invalid) in mask.data.iter_mut().zip(&invalid_depth.data) {
                if *invalid != 0 {
                    *value = 0;
                }
            }
            let count = mask.data.iter().filter(|&&v| v != 0).count();
            let ratio = count as f32 / original as f32;
            if count < minimum || (f64::from(ratio) < 0.9 && count < 10 * minimum) {
                continue;
            }
            sam_sizes.push((mask, count));
        }
        sam_sizes.sort_unstable_by_key(|(_, size)| *size);
        let mut se_sizes = raycast
            .iter()
            .map(|(&id, mask)| (id, mask.data.iter().filter(|&&v| v != 0).count()))
            .collect::<Vec<_>>();
        se_sizes.sort_unstable_by_key(|(_, size)| *size);
        for (mask, sam_area) in sam_sizes {
            let mut max_iou = (u16::MAX, 0.0f32);
            for &(id, se_area) in &se_sizes {
                let se = &raycast[&id];
                let intersection = se
                    .data
                    .iter()
                    .zip(&mask.data)
                    .filter(|(a, b)| **a & **b != 0)
                    .count();
                if (se_area as f32) < sam_area as f32 * 1.2
                    && intersection as f32 / se_area as f32 > 0.8
                {
                    for (p, available) in assignable.iter_mut().enumerate().take(rows * cols) {
                        if se.data[p] & mask.data[p] & *available != 0 {
                            output.data[p] = id;
                            *available = 0;
                        }
                    }
                    matched.insert(id);
                }
                let iou = intersection as f32 / (sam_area + se_area - intersection) as f32;
                if iou > max_iou.1 {
                    max_iou = (id, iou);
                }
            }
            let remaining = mask
                .data
                .iter()
                .zip(&assignable)
                .map(|(a, b)| a & b)
                .collect::<Vec<_>>();
            for (free, remaining) in assignable.iter_mut().zip(&remaining) {
                if *remaining != 0 {
                    *free = 0;
                }
            }
            if remaining.iter().any(|p| *p != 0) {
                let id = if iou_threshold < max_iou.1 {
                    max_iou.0
                } else {
                    self.obtain_new_id()
                };
                matched.insert(id);
                for (dst, mask) in output.data.iter_mut().zip(remaining) {
                    if mask != 0 {
                        *dst = id;
                    }
                }
            }
        }
        Ok(output)
    }
    fn validate_segment_inputs(invalid_depth: &Matrix<u8>, surface: &Matrix<u16>) -> Result<()> {
        if invalid_depth.channels != 1
            || surface.channels != 1
            || (invalid_depth.rows, invalid_depth.cols) != (surface.rows, surface.cols)
        {
            return Err(invalid(
                "mask and surface dimensions disagree",
                "ObjectMap::track_segments",
            ));
        }
        Ok(())
    }

    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn add_new_object(
        &mut self,
        map: MapId,
        mask: &Matrix<u8>,
        features: &DeepLearningImageData,
        id: SegmentId,
    ) -> Result<()> {
        if !self.backend.contains_map(map) {
            return Err(invalid(
                "object submap does not exist",
                "ObjectMap::add_new_object",
            ));
        }
        let (feature, count) = average_descriptor(features, mask)?;
        self.submap_objects
            .entry(map)
            .or_default()
            .entry(id)
            .or_insert_with(|| SubmapObject::new(id, map, feature, count));
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn update_object(
        &mut self,
        map: MapId,
        id: SegmentId,
        mask: &Matrix<u8>,
        features: &DeepLearningImageData,
    ) -> Result<()> {
        let object = self
            .submap_objects
            .get_mut(&map)
            .and_then(|objects| objects.get_mut(&id))
            .ok_or_else(|| {
                invalid(
                    "object does not exist in submap",
                    "ObjectMap::update_object",
                )
            })?;
        let (feature, count) = average_descriptor(features, mask)?;
        object.update_vision_language_descriptor(&feature, count);
        Ok(())
    }
}
/// # Errors
/// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
/// or a failed storage, integration, alignment, callback, worker or file operation.
pub fn bitwise_and3(a: &Matrix<u8>, b: &Matrix<u8>, c: &Matrix<u8>) -> Result<Matrix<u8>> {
    if a.channels != 1
        || (a.rows, a.cols, a.channels) != (b.rows, b.cols, b.channels)
        || (a.rows, a.cols, a.channels) != (c.rows, c.cols, c.channels)
    {
        return Err(invalid(
            "bitwise masks must be matching single-channel byte images",
            "bitwise_and3",
        ));
    }
    Ok(Matrix::from_vec(
        a.rows,
        a.cols,
        1,
        a.data
            .iter()
            .zip(&b.data)
            .zip(&c.data)
            .map(|((a, b), c)| a & b & c)
            .collect(),
    ))
}
#[must_use]
pub fn split(segments: &Matrix<u16>) -> BTreeMap<SegmentId, Matrix<u8>> {
    let mut masks = BTreeMap::new();
    for (index, &id) in segments.data.iter().enumerate() {
        if id != NO_ID {
            masks
                .entry(id)
                .or_insert_with(|| Matrix::new(segments.rows, segments.cols, 1))
                .data[index] = 1;
        }
    }
    masks
}
fn sam_data(sam: &DeepLearningImageData) -> Result<(usize, usize, usize, &[f32])> {
    if sam.shape.len() < 3 {
        return Err(invalid("SAM shape requires N,H,W", "ObjectMap::sam_data"));
    }
    let n = usize::try_from(sam.shape[0])
        .map_err(|_| invalid("negative mask count", "ObjectMap::sam_data"))?;
    let h = usize::try_from(sam.shape[1])
        .map_err(|_| invalid("negative mask height", "ObjectMap::sam_data"))?;
    let w = usize::try_from(sam.shape[2])
        .map_err(|_| invalid("negative mask width", "ObjectMap::sam_data"))?;
    let data = sam
        .data
        .as_deref()
        .ok_or_else(|| invalid("SAM storage absent", "ObjectMap::sam_data"))?;
    if n.checked_mul(h)
        .and_then(|v| v.checked_mul(w))
        .is_none_or(|length| length > data.len())
    {
        return Err(invalid(
            "SAM shape exceeds allocation",
            "ObjectMap::sam_data",
        ));
    }
    Ok((n, h, w, data))
}
/// # Errors
/// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
/// or a failed storage, integration, alignment, callback, worker or file operation.
pub fn average_descriptor(
    features: &DeepLearningImageData,
    mask: &Matrix<u8>,
) -> Result<(Descriptor, usize)> {
    if features.shape.len() < 2
        || features.shape[0] <= 0
        || features.shape[1] <= 0
        || mask.channels != 1
    {
        return Err(invalid(
            "feature/mask shape invalid",
            "ObjectMap::average_descriptor",
        ));
    }
    let (h, w) = (features.shape[0] as usize, features.shape[1] as usize);
    let data = features
        .data
        .as_deref()
        .ok_or_else(|| invalid("feature storage absent", "ObjectMap::average_descriptor"))?;
    let mut result = [0.; 768];
    let mut count = 0;
    for y in 0..mask.rows {
        for x in 0..mask.cols {
            if mask.data[y * mask.cols + x] == 0 {
                continue;
            }
            let fx = (x as f32 / mask.cols as f32 * w as f32) as usize;
            let fy = (y as f32 / mask.rows as f32 * h as f32) as usize;
            let offset = (fy * w + fx) * 768;
            let pixel = data.get(offset..offset + 768).ok_or_else(|| {
                invalid(
                    "feature projection exceeds allocation",
                    "ObjectMap::average_descriptor",
                )
            })?;
            for (sum, value) in result.iter_mut().zip(pixel) {
                *sum += value;
            }
            count += 1;
        }
    }
    for value in &mut result {
        *value /= count as f32;
    }
    Ok((result, count))
}

/// Multi-resolution occupancy maps without color or instance IDs.
pub struct OccupancyMapBackend {
    pub maps: BTreeMap<MapId, crate::mapping::map_typedefs::OccupancyMap>,
    pub shared_maps: crate::mapping::map_typedefs::SharedOccupancyMaps,
    pub map_config: firefly_supereight2::map::MapConfig,
    pub data_config: firefly_supereight2::data::DataConfig,
}
impl OccupancyMapBackend {
    #[must_use]
    pub fn new(
        map_config: firefly_supereight2::map::MapConfig,
        data_config: firefly_supereight2::data::DataConfig,
    ) -> Self {
        Self {
            maps: BTreeMap::new(),
            shared_maps: std::sync::Arc::default(),
            map_config,
            data_config,
        }
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn map_handle(&self, id: MapId) -> Result<&crate::mapping::map_typedefs::OccupancyMap> {
        self.maps
            .get(&id)
            .ok_or_else(|| invalid("occupancy map absent", "OccupancyMapBackend::map"))
    }
}
impl ObjectMapBackend for OccupancyMapBackend {
    fn contains_map(&self, map: MapId) -> bool {
        self.maps.contains_key(&map)
    }
    fn supports_colour_and_ids(&self) -> bool {
        false
    }
}
