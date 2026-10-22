/* FindAnything / OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2026 Technical University of Munich and ETH Zurich. SPDX-License-Identifier: BSD-3-Clause */
//! SAM/raycast segment association and submap object descriptors, geometry and export.
use crate::{
    deep_learning::utils::{ColourMap, invalid},
    mapping::map_typedefs::CHAIR_EMBEDDING,
    sensors::measurements::DeepLearningImageData,
};
use firefly_error::Result;
use nalgebra::{Matrix4, Vector3};
use purecv::core::{Matrix, types::Size};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::{BufWriter, Write},
    path::Path,
};
pub type Descriptor = [f32; 768];
pub type MapId = u64;
pub type SegmentId = u16;
pub const NO_ID: SegmentId = 0;
pub const NOT_MAPPED: SegmentId = u16::MAX;
#[derive(Clone, Debug)]
pub struct SegmentInfo {
    pub centroid: Vector3<f32>,
    pub aabb_min: Vector3<f32>,
    pub aabb_max: Vector3<f32>,
    pub num_vertices: usize,
}
#[derive(Clone)]
pub struct SubmapObject {
    pub id: SegmentId,
    pub map_id: MapId,
    pub vl_feature: Descriptor,
    pub num_pixels: usize,
    pub segment_info: Option<SegmentInfo>,
}
impl SubmapObject {
    #[must_use]
    pub fn new(id: SegmentId, map_id: MapId, vl_feature: Descriptor, num_pixels: usize) -> Self {
        Self {
            id,
            map_id,
            vl_feature,
            num_pixels,
            segment_info: None,
        }
    }
    pub fn finish_object(&mut self, info: SegmentInfo) {
        self.segment_info = Some(info);
    }
    #[must_use]
    pub fn finished(&self) -> bool {
        self.segment_info.is_some()
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
#[derive(Clone)]
pub struct MeshFace {
    pub id: SegmentId,
    pub scale: i8,
    pub vertices: [Vector3<f32>; 3],
    pub colour: Option<[u8; 3]>,
}
pub struct IdVoxelBlock {
    pub current_scale: u32,
    pub block_size: usize,
    pub ids: Vec<SegmentId>,
}
/// Supereight storage, mesh ID geometry extraction and tinycolormap lookup.
/// Each block exposes only the active scale, in its native voxel iteration order.
pub trait ObjectMapBackend {
    fn contains_map(&self, map: MapId) -> bool;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn mesh(&self, map: MapId) -> Result<&[MeshFace]>;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn mesh_mut(&mut self, map: MapId) -> Result<&mut [MeshFace]>;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn voxel_blocks(&self, map: MapId) -> Result<Vec<IdVoxelBlock>>;
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    fn mesh_id_info(&self, map: MapId) -> Result<BTreeMap<SegmentId, SegmentInfo>>;
    fn colour(&self, value: f64, map: ColourMap) -> [f64; 3];
    fn supports_colour_and_ids(&self) -> bool;
}
pub struct ObjectMap<B> {
    pub backend: B,
    object_counter: SegmentId,
    pub submap_objects: BTreeMap<MapId, BTreeMap<SegmentId, SubmapObject>>,
    pub global_objects: BTreeSet<SegmentId>,
    number_of_objects: usize,
}
impl<B: ObjectMapBackend> ObjectMap<B> {
    pub fn new(backend: B) -> Self {
        Self {
            backend,
            object_counter: 0,
            submap_objects: BTreeMap::new(),
            global_objects: BTreeSet::new(),
            number_of_objects: 0,
        }
    }
    pub fn contains_map(&self, map: MapId) -> bool {
        self.submap_objects.contains_key(&map)
    }
    pub fn number_of_objects(&self) -> usize {
        self.number_of_objects
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
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    ///
    /// # Panics
    /// Panics if a mesh segment has no accumulated segment entry after segment collection.
    pub fn finish_submap_objects(&mut self, map: MapId) -> Result<()> {
        if !self.backend.supports_colour_and_ids() {
            return Ok(());
        }
        if !self.submap_objects.contains_key(&map) {
            return Err(invalid(
                "object submap does not exist",
                "ObjectMap::finish_submap_objects",
            ));
        }
        let histogram = voxels_per_id(&self.backend.voxel_blocks(map)?)?;
        let mut file = BufWriter::new(
            File::create(format!("/tmp/hist-map-{map}.tsv"))
                .map_err(|e| invalid(e.to_string(), "ObjectMap::finish_submap_objects"))?,
        );
        writeln!(file, "Segment ID\tVoxels").map_err(|error| io_error(&error))?;
        for (id, count) in histogram {
            writeln!(file, "{id}\t{count}").map_err(|error| io_error(&error))?;
        }
        file.flush().map_err(|error| io_error(&error))?;
        self.colour_mesh_by_match(
            map,
            &CHAIR_EMBEDDING,
            ColourMap::Jet,
            (0., 0.5),
            (false, Vector3::new(-1., 0., -1.), [0x40; 3]),
        )?;
        let info = self.backend.mesh_id_info(map)?;
        let objects = self.submap_objects.get_mut(&map).unwrap();
        objects.retain(|id, object| {
            if let Some(info) = info.get(id) {
                object.finish_object(info.clone());
                true
            } else {
                false
            }
        });
        self.number_of_objects += objects.len();
        Ok(())
    }

    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn colour_mesh_by_match(
        &mut self,
        map: MapId,
        query: &Descriptor,
        colormap: ColourMap,
        (low, high): (f32, f32),
        (shading, light, ambient): (bool, Vector3<f32>, [u8; 3]),
    ) -> Result<()> {
        let objects = self
            .submap_objects
            .get(&map)
            .ok_or_else(|| invalid("object submap absent", "ObjectMap::colour_mesh_by_match"))?;
        let qnorm_squared = query.iter().map(|v| v * v).sum::<f32>();
        let qnorm = qnorm_squared.sqrt();
        let mesh = self.backend.mesh(map)?;
        let mut colours = Vec::with_capacity(mesh.len());
        for face in mesh {
            let unit = if face.id == NO_ID {
                0.
            } else {
                let object = objects.get(&face.id).ok_or_else(|| {
                    invalid(
                        "mesh ID has no object descriptor",
                        "ObjectMap::colour_mesh_by_match",
                    )
                })?;
                let norm_squared = object.vl_feature.iter().map(|v| v * v).sum::<f32>();
                let norm = norm_squared.sqrt();
                let d = object
                    .vl_feature
                    .iter()
                    .zip(query)
                    .map(|(a, b)| {
                        let a = if norm_squared > 0.0 { a / norm } else { *a };
                        let b = if qnorm_squared > 0.0 { b / qnorm } else { *b };
                        a * b
                    })
                    .sum::<f32>();
                (d.clamp(low, high) - low) / (high - low)
            };
            let rgb = self.backend.colour(f64::from(unit), colormap);
            let mut colour = rgb.map(|v| (255.0_f32 * v as f32) as u8);
            if shading && face.id != NO_ID {
                let mut normal = (face.vertices[1] - face.vertices[0])
                    .cross(&(face.vertices[2] - face.vertices[1]));
                let norm_squared = normal.norm_squared();
                if norm_squared > 0.0 {
                    normal /= norm_squared.sqrt();
                }
                let dot = normal.dot(&light);
                let intensity = if dot < 0.0 { 0.0 } else { dot };
                for c in 0..3 {
                    colour[c] = (intensity * f32::from(colour[c]) + f32::from(ambient[c]))
                        .clamp(0., 255.) as u8;
                }
            }
            colours.push(colour);
        }
        for (face, colour) in self.backend.mesh_mut(map)?.iter_mut().zip(colours) {
            face.colour = Some(colour);
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
    /// or a failed storage, integration, alignment, callback, worker or file operation.
    pub fn save_objects(&self, map: MapId, path: &Path, t_wm: &Matrix4<f32>) -> Result<bool> {
        let Some(objects) = self.submap_objects.get(&map) else {
            return Ok(false);
        };
        let Ok(file) = File::create(path.join(format!("objectMap{map}.csv"))) else {
            return Ok(false);
        };
        let mut writer = BufWriter::new(file);
        write!(writer, "ObjectId, num_vert, c_x, c_y, c_z").map_err(|error| io_error(&error))?;
        for i in 0..8 {
            write!(writer, ", aabb_{i}_x, aabb_{i}_y, aabb_{i}_z")
                .map_err(|error| io_error(&error))?;
        }
        for i in 0..768 {
            write!(writer, ", f{i}").map_err(|error| io_error(&error))?;
        }
        writeln!(writer).map_err(|error| io_error(&error))?;
        for (&id, object) in objects {
            let info = object.segment_info.as_ref().ok_or_else(|| {
                invalid(
                    "object geometry is not initialized",
                    "ObjectMap::save_objects",
                )
            })?;
            write!(writer, "{id}, {}", info.num_vertices).map_err(|error| io_error(&error))?;
            let centroid =
                t_wm.fixed_view::<3, 3>(0, 0) * info.centroid + t_wm.fixed_view::<3, 1>(0, 3);
            for c in 0..3 {
                write!(writer, ", {:.6}", centroid[c]).map_err(|error| io_error(&error))?;
            }
            for bits in 0..8 {
                let corner = t_wm.fixed_view::<3, 3>(0, 0) * aabb_corner(info, bits)
                    + t_wm.fixed_view::<3, 1>(0, 3);
                for c in 0..3 {
                    write!(writer, ", {:.6}", corner[c]).map_err(|error| io_error(&error))?;
                }
            }
            for feature in &object.vl_feature {
                write!(writer, ", {feature:.6}").map_err(|error| io_error(&error))?;
            }
            writeln!(writer).map_err(|error| io_error(&error))?;
        }
        writer.flush().map_err(|error| io_error(&error))?;
        Ok(true)
    }
}
fn io_error(error: &std::io::Error) -> firefly_error::Error {
    invalid(error.to_string(), "ObjectMap::write")
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
/// # Errors
/// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
/// or a failed storage, integration, alignment, callback, worker or file operation.
pub fn voxels_per_id(blocks: &[IdVoxelBlock]) -> Result<BTreeMap<SegmentId, usize>> {
    let mut histogram = BTreeMap::new();
    for block in blocks {
        let side = block
            .block_size
            .checked_shr(block.current_scale)
            .ok_or_else(|| invalid("voxel scale out of range", "voxels_per_id"))?;
        let count = side
            .checked_pow(3)
            .ok_or_else(|| invalid("voxel count overflows", "voxels_per_id"))?;
        let volume = 1usize
            .checked_shl(block.current_scale)
            .and_then(|side| side.checked_pow(3))
            .ok_or_else(|| invalid("voxel volume overflows", "voxels_per_id"))?;
        if block.ids.len() < count {
            return Err(invalid(
                "voxel block does not cover active scale",
                "voxels_per_id",
            ));
        }
        for &id in &block.ids[..count] {
            if id != NO_ID {
                *histogram.entry(id).or_insert(0) += volume;
            }
        }
    }
    Ok(histogram)
}
fn aabb_corner(info: &SegmentInfo, bits: usize) -> Vector3<f32> {
    Vector3::new(
        if bits & 1 == 0 {
            info.aabb_min.x
        } else {
            info.aabb_max.x
        },
        if bits & 2 == 0 {
            info.aabb_min.y
        } else {
            info.aabb_max.y
        },
        if bits & 4 == 0 {
            info.aabb_min.z
        } else {
            info.aabb_max.z
        },
    )
}
/// # Errors
/// Returns an error for invalid configuration or sensor/map data, absent frames/maps,
/// or a failed storage, integration, alignment, callback, worker or file operation.
pub fn save_object_bounding_box_vtk(object: &SubmapObject, path: &Path) -> Result<()> {
    let info = object
        .segment_info
        .as_ref()
        .ok_or_else(|| invalid("object geometry absent", "save_object_bounding_box_vtk"))?;
    let mut writer = BufWriter::new(File::create(path).map_err(|error| io_error(&error))?);
    writeln!(writer,"# vtk DataFile Version 1.0\nBounding Box of Submap\nASCII\nDATASET POLYDATA\nPOINTS 9 float").map_err(|error| io_error(&error))?;
    for bits in [0, 1, 3, 2, 4, 5, 7, 6] {
        let p = aabb_corner(info, bits);
        writeln!(writer, "{} {} {}", p.x, p.y, p.z).map_err(|error| io_error(&error))?;
    }
    writeln!(writer,"{} {} {}\nLINES 12 36\n2 0 1\n2 1 2\n2 2 3\n2 3 0\n2 4 5\n2 5 6\n2 6 7\n2 7 4\n2 0 4\n2 1 5\n2 2 6\n2 3 7",info.centroid.x,info.centroid.y,info.centroid.z).map_err(|error| io_error(&error))?;
    writer.flush().map_err(|error| io_error(&error))
}

/// Multi-resolution occupancy maps with per-submap mesh caches, without color or instance IDs.
pub struct OccupancyMapBackend {
    pub maps: BTreeMap<MapId, crate::mapping::map_typedefs::OccupancyMap>,
    pub meshes: BTreeMap<MapId, Vec<MeshFace>>,
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
            meshes: BTreeMap::new(),
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
    fn mesh(&self, map: MapId) -> Result<&[MeshFace]> {
        self.meshes
            .get(&map)
            .map(Vec::as_slice)
            .ok_or_else(|| invalid("mesh has not been cached", "OccupancyMapBackend::mesh"))
    }
    fn mesh_mut(&mut self, map: MapId) -> Result<&mut [MeshFace]> {
        self.meshes
            .get_mut(&map)
            .map(Vec::as_mut_slice)
            .ok_or_else(|| invalid("mesh has not been cached", "OccupancyMapBackend::mesh_mut"))
    }
    fn voxel_blocks(&self, _map: MapId) -> Result<Vec<IdVoxelBlock>> {
        Err(invalid(
            "occupancy-only maps have no instance ID voxels",
            "OccupancyMapBackend::voxel_blocks",
        ))
    }
    fn mesh_id_info(&self, _map: MapId) -> Result<BTreeMap<SegmentId, SegmentInfo>> {
        Err(invalid(
            "occupancy-only maps have no instance ID geometry",
            "OccupancyMapBackend::mesh_id_info",
        ))
    }
    fn colour(&self, value: f64, map: ColourMap) -> [f64; 3] {
        tiny_colour(
            value,
            match map {
                ColourMap::Inferno => INFERNO,
                ColourMap::Jet => JET,
                ColourMap::Parula => PARULA,
            },
        )
    }
    fn supports_colour_and_ids(&self) -> bool {
        false
    }
}
fn tiny_colour(value: f64, table: &[[f64; 3]]) -> [f64; 3] {
    if value.is_nan() {
        return [f64::NAN; 3];
    }
    let a = value.clamp(0.0, 1.0) * (table.len() - 1) as f64;
    let low = a.floor() as usize;
    let high = a.ceil() as usize;
    let t = a - low as f64;
    std::array::from_fn(|i| (1.0 - t) * table[low][i] + t * table[high][i])
}

/*
MIT License

Copyright (c) 2018-2020 Yuki Koyama

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
// tinycolormap RGB samples.
const PARULA: &[[f64; 3]] = &[
    [0.2081, 0.1663, 0.5292],
    [0.2091, 0.1721, 0.5411],
    [0.2101, 0.1779, 0.553],
    [0.2109, 0.1837, 0.565],
    [0.2116, 0.1895, 0.5771],
    [0.2121, 0.1954, 0.5892],
    [0.2124, 0.2013, 0.6013],
    [0.2125, 0.2072, 0.6135],
    [0.2123, 0.2132, 0.6258],
    [0.2118, 0.2192, 0.6381],
    [0.2111, 0.2253, 0.6505],
    [0.2099, 0.2315, 0.6629],
    [0.2084, 0.2377, 0.6753],
    [0.2063, 0.244, 0.6878],
    [0.2038, 0.2503, 0.7003],
    [0.2006, 0.2568, 0.7129],
    [0.1968, 0.2632, 0.7255],
    [0.1921, 0.2698, 0.7381],
    [0.1867, 0.2764, 0.7507],
    [0.1802, 0.2832, 0.7634],
    [0.1728, 0.2902, 0.7762],
    [0.1641, 0.2975, 0.789],
    [0.1541, 0.3052, 0.8017],
    [0.1427, 0.3132, 0.8145],
    [0.1295, 0.3217, 0.8269],
    [0.1147, 0.3306, 0.8387],
    [0.0986, 0.3397, 0.8495],
    [0.0816, 0.3486, 0.8588],
    [0.0646, 0.3572, 0.8664],
    [0.0482, 0.3651, 0.8722],
    [0.0329, 0.3724, 0.8765],
    [0.0213, 0.3792, 0.8796],
    [0.0136, 0.3853, 0.8815],
    [0.0086, 0.3911, 0.8827],
    [0.006, 0.3965, 0.8833],
    [0.0051, 0.4017, 0.8834],
    [0.0054, 0.4066, 0.8831],
    [0.0067, 0.4113, 0.8825],
    [0.0089, 0.4159, 0.8816],
    [0.0116, 0.4203, 0.8805],
    [0.0148, 0.4246, 0.8793],
    [0.0184, 0.4288, 0.8779],
    [0.0223, 0.4329, 0.8763],
    [0.0264, 0.437, 0.8747],
    [0.0306, 0.441, 0.8729],
    [0.0349, 0.4449, 0.8711],
    [0.0394, 0.4488, 0.8692],
    [0.0437, 0.4526, 0.8672],
    [0.0477, 0.4564, 0.8652],
    [0.0514, 0.4602, 0.8632],
    [0.0549, 0.464, 0.8611],
    [0.0582, 0.4677, 0.8589],
    [0.0612, 0.4714, 0.8568],
    [0.064, 0.4751, 0.8546],
    [0.0666, 0.4788, 0.8525],
    [0.0689, 0.4825, 0.8503],
    [0.071, 0.4862, 0.8481],
    [0.0729, 0.4899, 0.846],
    [0.0746, 0.4937, 0.8439],
    [0.0761, 0.4974, 0.8418],
    [0.0773, 0.5012, 0.8398],
    [0.0782, 0.5051, 0.8378],
    [0.0789, 0.5089, 0.8359],
    [0.0794, 0.5129, 0.8341],
    [0.0795, 0.5169, 0.8324],
    [0.0793, 0.521, 0.8308],
    [0.0788, 0.5251, 0.8293],
    [0.0778, 0.5295, 0.828],
    [0.0764, 0.5339, 0.827],
    [0.0746, 0.5384, 0.8261],
    [0.0724, 0.5431, 0.8253],
    [0.0698, 0.5479, 0.8247],
    [0.0668, 0.5527, 0.8243],
    [0.0636, 0.5577, 0.8239],
    [0.06, 0.5627, 0.8237],
    [0.0562, 0.5677, 0.8234],
    [0.0523, 0.5727, 0.8231],
    [0.0484, 0.5777, 0.8228],
    [0.0445, 0.5826, 0.8223],
    [0.0408, 0.5874, 0.8217],
    [0.0372, 0.5922, 0.8209],
    [0.0342, 0.5968, 0.8198],
    [0.0317, 0.6012, 0.8186],
    [0.0296, 0.6055, 0.8171],
    [0.0279, 0.6097, 0.8154],
    [0.0265, 0.6137, 0.8135],
    [0.0255, 0.6176, 0.8114],
    [0.0248, 0.6214, 0.8091],
    [0.0243, 0.625, 0.8066],
    [0.0239, 0.6285, 0.8039],
    [0.0237, 0.6319, 0.801],
    [0.0235, 0.6352, 0.798],
    [0.0233, 0.6384, 0.7948],
    [0.0231, 0.6415, 0.7916],
    [0.023, 0.6445, 0.7881],
    [0.0229, 0.6474, 0.7846],
    [0.0227, 0.6503, 0.781],
    [0.0227, 0.6531, 0.7773],
    [0.0232, 0.6558, 0.7735],
    [0.0238, 0.6585, 0.7696],
    [0.0246, 0.6611, 0.7656],
    [0.0263, 0.6637, 0.7615],
    [0.0282, 0.6663, 0.7574],
    [0.0306, 0.6688, 0.7532],
    [0.0338, 0.6712, 0.749],
    [0.0373, 0.6737, 0.7446],
    [0.0418, 0.6761, 0.7402],
    [0.0467, 0.6784, 0.7358],
    [0.0516, 0.6808, 0.7313],
    [0.0574, 0.6831, 0.7267],
    [0.0629, 0.6854, 0.7221],
    [0.0692, 0.6877, 0.7173],
    [0.0755, 0.6899, 0.7126],
    [0.082, 0.6921, 0.7078],
    [0.0889, 0.6943, 0.7029],
    [0.0956, 0.6965, 0.6979],
    [0.1031, 0.6986, 0.6929],
    [0.1104, 0.7007, 0.6878],
    [0.118, 0.7028, 0.6827],
    [0.1258, 0.7049, 0.6775],
    [0.1335, 0.7069, 0.6723],
    [0.1418, 0.7089, 0.6669],
    [0.1499, 0.7109, 0.6616],
    [0.1585, 0.7129, 0.6561],
    [0.1671, 0.7148, 0.6507],
    [0.1758, 0.7168, 0.6451],
    [0.1849, 0.7186, 0.6395],
    [0.1938, 0.7205, 0.6338],
    [0.2033, 0.7223, 0.6281],
    [0.2128, 0.7241, 0.6223],
    [0.2224, 0.7259, 0.6165],
    [0.2324, 0.7275, 0.6107],
    [0.2423, 0.7292, 0.6048],
    [0.2527, 0.7308, 0.5988],
    [0.2631, 0.7324, 0.5929],
    [0.2735, 0.7339, 0.5869],
    [0.2845, 0.7354, 0.5809],
    [0.2953, 0.7368, 0.5749],
    [0.3064, 0.7381, 0.5689],
    [0.3177, 0.7394, 0.563],
    [0.3289, 0.7406, 0.557],
    [0.3405, 0.7417, 0.5512],
    [0.352, 0.7428, 0.5453],
    [0.3635, 0.7438, 0.5396],
    [0.3753, 0.7446, 0.5339],
    [0.3869, 0.7454, 0.5283],
    [0.3986, 0.7461, 0.5229],
    [0.4103, 0.7467, 0.5175],
    [0.4218, 0.7473, 0.5123],
    [0.4334, 0.7477, 0.5072],
    [0.4447, 0.7482, 0.5021],
    [0.4561, 0.7485, 0.4972],
    [0.4672, 0.7487, 0.4924],
    [0.4783, 0.7489, 0.4877],
    [0.4892, 0.7491, 0.4831],
    [0.5, 0.7491, 0.4786],
    [0.5106, 0.7492, 0.4741],
    [0.5212, 0.7492, 0.4698],
    [0.5315, 0.7491, 0.4655],
    [0.5418, 0.749, 0.4613],
    [0.5519, 0.7489, 0.4571],
    [0.5619, 0.7487, 0.4531],
    [0.5718, 0.7485, 0.449],
    [0.5816, 0.7482, 0.4451],
    [0.5913, 0.7479, 0.4412],
    [0.6009, 0.7476, 0.4374],
    [0.6103, 0.7473, 0.4335],
    [0.6197, 0.7469, 0.4298],
    [0.629, 0.7465, 0.4261],
    [0.6382, 0.746, 0.4224],
    [0.6473, 0.7456, 0.4188],
    [0.6564, 0.7451, 0.4152],
    [0.6653, 0.7446, 0.4116],
    [0.6742, 0.7441, 0.4081],
    [0.683, 0.7435, 0.4046],
    [0.6918, 0.743, 0.4011],
    [0.7004, 0.7424, 0.3976],
    [0.7091, 0.7418, 0.3942],
    [0.7176, 0.7412, 0.3908],
    [0.7261, 0.7405, 0.3874],
    [0.7346, 0.7399, 0.384],
    [0.743, 0.7392, 0.3806],
    [0.7513, 0.7385, 0.3773],
    [0.7596, 0.7378, 0.3739],
    [0.7679, 0.7372, 0.3706],
    [0.7761, 0.7364, 0.3673],
    [0.7843, 0.7357, 0.3639],
    [0.7924, 0.735, 0.3606],
    [0.8005, 0.7343, 0.3573],
    [0.8085, 0.7336, 0.3539],
    [0.8166, 0.7329, 0.3506],
    [0.8246, 0.7322, 0.3472],
    [0.8325, 0.7315, 0.3438],
    [0.8405, 0.7308, 0.3404],
    [0.8484, 0.7301, 0.337],
    [0.8563, 0.7294, 0.3336],
    [0.8642, 0.7288, 0.33],
    [0.872, 0.7282, 0.3265],
    [0.8798, 0.7276, 0.3229],
    [0.8877, 0.7271, 0.3193],
    [0.8954, 0.7266, 0.3156],
    [0.9032, 0.7262, 0.3117],
    [0.911, 0.7259, 0.3078],
    [0.9187, 0.7256, 0.3038],
    [0.9264, 0.7256, 0.2996],
    [0.9341, 0.7256, 0.2953],
    [0.9417, 0.7259, 0.2907],
    [0.9493, 0.7264, 0.2859],
    [0.9567, 0.7273, 0.2808],
    [0.9639, 0.7285, 0.2754],
    [0.9708, 0.7303, 0.2696],
    [0.9773, 0.7326, 0.2634],
    [0.9831, 0.7355, 0.257],
    [0.9882, 0.739, 0.2504],
    [0.9922, 0.7431, 0.2437],
    [0.9952, 0.7476, 0.2373],
    [0.9973, 0.7524, 0.231],
    [0.9986, 0.7573, 0.2251],
    [0.9991, 0.7624, 0.2195],
    [0.999, 0.7675, 0.2141],
    [0.9985, 0.7726, 0.209],
    [0.9976, 0.7778, 0.2042],
    [0.9964, 0.7829, 0.1995],
    [0.995, 0.788, 0.1949],
    [0.9933, 0.7931, 0.1905],
    [0.9914, 0.7981, 0.1863],
    [0.9894, 0.8032, 0.1821],
    [0.9873, 0.8083, 0.178],
    [0.9851, 0.8133, 0.174],
    [0.9828, 0.8184, 0.17],
    [0.9805, 0.8235, 0.1661],
    [0.9782, 0.8286, 0.1622],
    [0.9759, 0.8337, 0.1583],
    [0.9736, 0.8389, 0.1544],
    [0.9713, 0.8441, 0.1505],
    [0.9692, 0.8494, 0.1465],
    [0.9672, 0.8548, 0.1425],
    [0.9654, 0.8603, 0.1385],
    [0.9638, 0.8659, 0.1343],
    [0.9623, 0.8716, 0.1301],
    [0.9611, 0.8774, 0.1258],
    [0.96, 0.8834, 0.1215],
    [0.9593, 0.8895, 0.1171],
    [0.9588, 0.8958, 0.1126],
    [0.9586, 0.9022, 0.1082],
    [0.9587, 0.9088, 0.1036],
    [0.9591, 0.9155, 0.099],
    [0.9599, 0.9225, 0.0944],
    [0.961, 0.9296, 0.0897],
    [0.9624, 0.9368, 0.085],
    [0.9641, 0.9443, 0.0802],
    [0.9662, 0.9518, 0.0753],
    [0.9685, 0.9595, 0.0703],
    [0.971, 0.9673, 0.0651],
    [0.9736, 0.9752, 0.0597],
    [0.9763, 0.9831, 0.0538],
];

// tinycolormap RGB table (MIT), preserving the source samples.
const JET: &[[f64; 3]] = &[
    [0.0, 0.0, 0.5],
    [0.0, 0.0, 1.0],
    [0.0, 0.5, 1.0],
    [0.0, 1.0, 1.0],
    [0.5, 1.0, 0.5],
    [1.0, 1.0, 0.0],
    [1.0, 0.5, 0.0],
    [1.0, 0.0, 0.0],
    [0.5, 0.0, 0.0],
];

// tinycolormap RGB table (MIT), preserving the source samples.
const INFERNO: &[[f64; 3]] = &[
    [0.001_462, 0.000_466, 0.013_866],
    [0.002_267, 0.001_270, 0.018_570],
    [0.003_299, 0.002_249, 0.024_239],
    [0.004_547, 0.003_392, 0.030_909],
    [0.006_006, 0.004_692, 0.038_558],
    [0.007_676, 0.006_136, 0.046_836],
    [0.009_561, 0.007_713, 0.055_143],
    [0.011_663, 0.009_417, 0.063_460],
    [0.013_995, 0.011_225, 0.071_862],
    [0.016_561, 0.013_136, 0.080_282],
    [0.019_373, 0.015_133, 0.088_767],
    [0.022_447, 0.017_199, 0.097_327],
    [0.025_793, 0.019_331, 0.105_930],
    [0.029_432, 0.021_503, 0.114_621],
    [0.033_385, 0.023_702, 0.123_397],
    [0.037_668, 0.025_921, 0.132_232],
    [0.042_253, 0.028_139, 0.141_141],
    [0.046_915, 0.030_324, 0.150_164],
    [0.051_644, 0.032_474, 0.159_254],
    [0.056_449, 0.034_569, 0.168_414],
    [0.061_340, 0.036_590, 0.177_642],
    [0.066_331, 0.038_504, 0.186_962],
    [0.071_429, 0.040_294, 0.196_354],
    [0.076_637, 0.041_905, 0.205_799],
    [0.081_962, 0.043_328, 0.215_289],
    [0.087_411, 0.044_556, 0.224_813],
    [0.092_990, 0.045_583, 0.234_358],
    [0.098_702, 0.046_402, 0.243_904],
    [0.104_551, 0.047_008, 0.253_430],
    [0.110_536, 0.047_399, 0.262_912],
    [0.116_656, 0.047_574, 0.272_321],
    [0.122_908, 0.047_536, 0.281_624],
    [0.129_285, 0.047_293, 0.290_788],
    [0.135_778, 0.046_856, 0.299_776],
    [0.142_378, 0.046_242, 0.308_553],
    [0.149_073, 0.045_468, 0.317_085],
    [0.155_850, 0.044_559, 0.325_338],
    [0.162_689, 0.043_554, 0.333_277],
    [0.169_575, 0.042_489, 0.340_874],
    [0.176_493, 0.041_402, 0.348_111],
    [0.183_429, 0.040_329, 0.354_971],
    [0.190_367, 0.039_309, 0.361_447],
    [0.197_297, 0.038_400, 0.367_535],
    [0.204_209, 0.037_632, 0.373_238],
    [0.211_095, 0.037_030, 0.378_563],
    [0.217_949, 0.036_615, 0.383_522],
    [0.224_763, 0.036_405, 0.388_129],
    [0.231_538, 0.036_405, 0.392_400],
    [0.238_273, 0.036_621, 0.396_353],
    [0.244_967, 0.037_055, 0.400_007],
    [0.251_620, 0.037_705, 0.403_378],
    [0.258_234, 0.038_571, 0.406_485],
    [0.264_810, 0.039_647, 0.409_345],
    [0.271_347, 0.040_922, 0.411_976],
    [0.277_850, 0.042_353, 0.414_392],
    [0.284_321, 0.043_933, 0.416_608],
    [0.290_763, 0.045_644, 0.418_637],
    [0.297_178, 0.047_470, 0.420_491],
    [0.303_568, 0.049_396, 0.422_182],
    [0.309_935, 0.051_407, 0.423_721],
    [0.316_282, 0.053_490, 0.425_116],
    [0.322_610, 0.055_634, 0.426_377],
    [0.328_921, 0.057_827, 0.427_511],
    [0.335_217, 0.060_060, 0.428_524],
    [0.341_500, 0.062_325, 0.429_425],
    [0.347_771, 0.064_616, 0.430_217],
    [0.354_032, 0.066_925, 0.430_906],
    [0.360_284, 0.069_247, 0.431_497],
    [0.366_529, 0.071_579, 0.431_994],
    [0.372_768, 0.073_915, 0.432_400],
    [0.379_001, 0.076_253, 0.432_719],
    [0.385_228, 0.078_591, 0.432_955],
    [0.391_453, 0.080_927, 0.433_109],
    [0.397_674, 0.083_257, 0.433_183],
    [0.403_894, 0.085_580, 0.433_179],
    [0.410_113, 0.087_896, 0.433_098],
    [0.416_331, 0.090_203, 0.432_943],
    [0.422_549, 0.092_501, 0.432_714],
    [0.428_768, 0.094_790, 0.432_412],
    [0.434_987, 0.097_069, 0.432_039],
    [0.441_207, 0.099_338, 0.431_594],
    [0.447_428, 0.101_597, 0.431_080],
    [0.453_651, 0.103_848, 0.430_498],
    [0.459_875, 0.106_089, 0.429_846],
    [0.466_100, 0.108_322, 0.429_125],
    [0.472_328, 0.110_547, 0.428_334],
    [0.478_558, 0.112_764, 0.427_475],
    [0.484_789, 0.114_974, 0.426_548],
    [0.491_022, 0.117_179, 0.425_552],
    [0.497_257, 0.119_379, 0.424_488],
    [0.503_493, 0.121_575, 0.423_356],
    [0.509_730, 0.123_769, 0.422_156],
    [0.515_967, 0.125_960, 0.420_887],
    [0.522_206, 0.128_150, 0.419_549],
    [0.528_444, 0.130_341, 0.418_142],
    [0.534_683, 0.132_534, 0.416_667],
    [0.540_920, 0.134_729, 0.415_123],
    [0.547_157, 0.136_929, 0.413_511],
    [0.553_392, 0.139_134, 0.411_829],
    [0.559_624, 0.141_346, 0.410_078],
    [0.565_854, 0.143_567, 0.408_258],
    [0.572_081, 0.145_797, 0.406_369],
    [0.578_304, 0.148_039, 0.404_411],
    [0.584_521, 0.150_294, 0.402_385],
    [0.590_734, 0.152_563, 0.400_290],
    [0.596_940, 0.154_848, 0.398_125],
    [0.603_139, 0.157_151, 0.395_891],
    [0.609_330, 0.159_474, 0.393_589],
    [0.615_513, 0.161_817, 0.391_219],
    [0.621_685, 0.164_184, 0.388_781],
    [0.627_847, 0.166_575, 0.386_276],
    [0.633_998, 0.168_992, 0.383_704],
    [0.640_135, 0.171_438, 0.381_065],
    [0.646_260, 0.173_914, 0.378_359],
    [0.652_369, 0.176_421, 0.375_586],
    [0.658_463, 0.178_962, 0.372_748],
    [0.664_540, 0.181_539, 0.369_846],
    [0.670_599, 0.184_153, 0.366_879],
    [0.676_638, 0.186_807, 0.363_849],
    [0.682_656, 0.189_501, 0.360_757],
    [0.688_653, 0.192_239, 0.357_603],
    [0.694_627, 0.195_021, 0.354_388],
    [0.700_576, 0.197_851, 0.351_113],
    [0.706_500, 0.200_728, 0.347_777],
    [0.712_396, 0.203_656, 0.344_383],
    [0.718_264, 0.206_636, 0.340_931],
    [0.724_103, 0.209_670, 0.337_424],
    [0.729_909, 0.212_759, 0.333_861],
    [0.735_683, 0.215_906, 0.330_245],
    [0.741_423, 0.219_112, 0.326_576],
    [0.747_127, 0.222_378, 0.322_856],
    [0.752_794, 0.225_706, 0.319_085],
    [0.758_422, 0.229_097, 0.315_266],
    [0.764_010, 0.232_554, 0.311_399],
    [0.769_556, 0.236_077, 0.307_485],
    [0.775_059, 0.239_667, 0.303_526],
    [0.780_517, 0.243_327, 0.299_523],
    [0.785_929, 0.247_056, 0.295_477],
    [0.791_293, 0.250_856, 0.291_390],
    [0.796_607, 0.254_728, 0.287_264],
    [0.801_871, 0.258_674, 0.283_099],
    [0.807_082, 0.262_692, 0.278_898],
    [0.812_239, 0.266_786, 0.274_661],
    [0.817_341, 0.270_954, 0.270_390],
    [0.822_386, 0.275_197, 0.266_085],
    [0.827_372, 0.279_517, 0.261_750],
    [0.832_299, 0.283_913, 0.257_383],
    [0.837_165, 0.288_385, 0.252_988],
    [0.841_969, 0.292_933, 0.248_564],
    [0.846_709, 0.297_559, 0.244_113],
    [0.851_384, 0.302_260, 0.239_636],
    [0.855_992, 0.307_038, 0.235_133],
    [0.860_533, 0.311_892, 0.230_606],
    [0.865_006, 0.316_822, 0.226_055],
    [0.869_409, 0.321_827, 0.221_482],
    [0.873_741, 0.326_906, 0.216_886],
    [0.878_001, 0.332_060, 0.212_268],
    [0.882_188, 0.337_287, 0.207_628],
    [0.886_302, 0.342_586, 0.202_968],
    [0.890_341, 0.347_957, 0.198_286],
    [0.894_305, 0.353_399, 0.193_584],
    [0.898_192, 0.358_911, 0.188_860],
    [0.902_003, 0.364_492, 0.184_116],
    [0.905_735, 0.370_140, 0.179_350],
    [0.909_390, 0.375_856, 0.174_563],
    [0.912_966, 0.381_636, 0.169_755],
    [0.916_462, 0.387_481, 0.164_924],
    [0.919_879, 0.393_389, 0.160_070],
    [0.923_215, 0.399_359, 0.155_193],
    [0.926_470, 0.405_389, 0.150_292],
    [0.929_644, 0.411_479, 0.145_367],
    [0.932_737, 0.417_627, 0.140_417],
    [0.935_747, 0.423_831, 0.135_440],
    [0.938_675, 0.430_091, 0.130_438],
    [0.941_521, 0.436_405, 0.125_409],
    [0.944_285, 0.442_772, 0.120_354],
    [0.946_965, 0.449_191, 0.115_272],
    [0.949_562, 0.455_660, 0.110_164],
    [0.952_075, 0.462_178, 0.105_031],
    [0.954_506, 0.468_744, 0.099_874],
    [0.956_852, 0.475_356, 0.094_695],
    [0.959_114, 0.482_014, 0.089_499],
    [0.961_293, 0.488_716, 0.084_289],
    [0.963_387, 0.495_462, 0.079_073],
    [0.965_397, 0.502_249, 0.073_859],
    [0.967_322, 0.509_078, 0.068_659],
    [0.969_163, 0.515_946, 0.063_488],
    [0.970_919, 0.522_853, 0.058_367],
    [0.972_590, 0.529_798, 0.053_324],
    [0.974_176, 0.536_780, 0.048_392],
    [0.975_677, 0.543_798, 0.043_618],
    [0.977_092, 0.550_850, 0.039_050],
    [0.978_422, 0.557_937, 0.034_931],
    [0.979_666, 0.565_057, 0.031_409],
    [0.980_824, 0.572_209, 0.028_508],
    [0.981_895, 0.579_392, 0.026_250],
    [0.982_881, 0.586_606, 0.024_661],
    [0.983_779, 0.593_849, 0.023_770],
    [0.984_591, 0.601_122, 0.023_606],
    [0.985_315, 0.608_422, 0.024_202],
    [0.985_952, 0.615_750, 0.025_592],
    [0.986_502, 0.623_105, 0.027_814],
    [0.986_964, 0.630_485, 0.030_908],
    [0.987_337, 0.637_890, 0.034_916],
    [0.987_622, 0.645_320, 0.039_886],
    [0.987_819, 0.652_773, 0.045_581],
    [0.987_926, 0.660_250, 0.051_750],
    [0.987_945, 0.667_748, 0.058_329],
    [0.987_874, 0.675_267, 0.065_257],
    [0.987_714, 0.682_807, 0.072_489],
    [0.987_464, 0.690_366, 0.079_990],
    [0.987_124, 0.697_944, 0.087_731],
    [0.986_694, 0.705_540, 0.095_694],
    [0.986_175, 0.713_153, 0.103_863],
    [0.985_566, 0.720_782, 0.112_229],
    [0.984_865, 0.728_427, 0.120_785],
    [0.984_075, 0.736_087, 0.129_527],
    [0.983_196, 0.743_758, 0.138_453],
    [0.982_228, 0.751_442, 0.147_565],
    [0.981_173, 0.759_135, 0.156_863],
    [0.980_032, 0.766_837, 0.166_353],
    [0.978_806, 0.774_545, 0.176_037],
    [0.977_497, 0.782_258, 0.185_923],
    [0.976_108, 0.789_974, 0.196_018],
    [0.974_638, 0.797_692, 0.206_332],
    [0.973_088, 0.805_409, 0.216_877],
    [0.971_468, 0.813_122, 0.227_658],
    [0.969_783, 0.820_825, 0.238_686],
    [0.968_041, 0.828_515, 0.249_972],
    [0.966_243, 0.836_191, 0.261_534],
    [0.964_394, 0.843_848, 0.273_391],
    [0.962_517, 0.851_476, 0.285_546],
    [0.960_626, 0.859_069, 0.298_010],
    [0.958_720, 0.866_624, 0.310_820],
    [0.956_834, 0.874_129, 0.323_974],
    [0.954_997, 0.881_569, 0.337_475],
    [0.953_215, 0.888_942, 0.351_369],
    [0.951_546, 0.896_226, 0.365_627],
    [0.950_018, 0.903_409, 0.380_271],
    [0.948_683, 0.910_473, 0.395_289],
    [0.947_594, 0.917_399, 0.410_665],
    [0.946_809, 0.924_168, 0.426_373],
    [0.946_392, 0.930_761, 0.442_367],
    [0.946_403, 0.937_159, 0.458_592],
    [0.946_903, 0.943_348, 0.474_970],
    [0.947_937, 0.949_318, 0.491_426],
    [0.949_545, 0.955_063, 0.507_860],
    [0.951_740, 0.960_587, 0.524_203],
    [0.954_529, 0.965_896, 0.540_361],
    [0.957_896, 0.971_003, 0.556_275],
    [0.961_812, 0.975_924, 0.571_925],
    [0.966_249, 0.980_678, 0.587_206],
    [0.971_162, 0.985_282, 0.602_154],
    [0.976_511, 0.989_753, 0.616_760],
    [0.982_257, 0.994_109, 0.631_017],
    [0.988_362, 0.998_364, 0.644_924],
];
