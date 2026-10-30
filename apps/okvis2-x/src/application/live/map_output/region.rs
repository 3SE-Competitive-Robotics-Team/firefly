//! Spatial submap lookup and region-pruned octree traversal.
use super::{Snapshot, invalid};
use crate::{
    kinematics::transformation::Transformation, mapping::object_mapping::OccupancyMapBackend,
};
use firefly_base::{FrameId, RigidTransform};
use firefly_error::Result;
use firefly_pubsub::map::{MAX_MAP_CELLS, MapElement, MapRegion};
use firefly_supereight2::octant::Octant;
use jiff::Timestamp;
use nalgebra::{Matrix4, Vector3};
use std::collections::{BTreeMap, BTreeSet, HashMap};

// Index granularity affects candidate count, never exported geometry.
const BUCKET_SIZE: f64 = 16.;
type Bounds = ([f64; 3], [f64; 3]);

#[derive(Default)]
pub struct RegionIndex {
    bounds: HashMap<u64, Bounds>,
    buckets: HashMap<[i64; 3], BTreeSet<u64>>,
}
impl RegionIndex {
    /// Refresh only created or pose-updated submaps; depth cannot leave their root volumes.
    /// # Errors
    /// Missing submap poses or inaccessible map storage.
    pub fn update(
        &mut self,
        changed: &[u64],
        poses: &BTreeMap<u64, Transformation>,
        backend: &OccupancyMapBackend,
    ) -> Result<()> {
        for id in changed {
            let Some(handle) = backend.maps.get(id) else {
                continue;
            };
            let pose = poses
                .get(id)
                .ok_or_else(|| invalid("submap pose missing in region index"))?;
            let map = handle.read()?;
            let transform = pose.matrix() * map.t_wm.to_homogeneous().cast::<f64>();
            let bounds = transformed(
                &transform,
                Vector3::zeros(),
                Vector3::repeat(f64::from(map.octree.size) * f64::from(map.resolution)),
            );
            if self.bounds.get(id) == Some(&bounds) {
                continue;
            }
            if let Some(old) = self.bounds.insert(*id, bounds) {
                for key in buckets(old)? {
                    if let Some(ids) = self.buckets.get_mut(&key) {
                        ids.remove(id);
                        if ids.is_empty() {
                            self.buckets.remove(&key);
                        }
                    }
                }
            }
            for key in buckets(bounds)? {
                self.buckets.entry(key).or_default().insert(*id);
            }
        }
        Ok(())
    }

    /// Export a complete snapshot of the header's odom-frame coverage.
    /// # Errors
    /// Invalid bounds/alignment, missing map state, or exceeded snapshot capacity.
    #[fastrace::trace]
    pub fn snapshot(
        &self,
        revision: u64,
        time: Timestamp,
        poses: &BTreeMap<u64, Transformation>,
        backend: &OccupancyMapBackend,
        header: MapElement,
    ) -> Result<Snapshot> {
        self.snapshot_with_buffer(revision, time, poses, backend, header, Vec::new())
    }

    /// Export into an owned buffer recycled after IPC publication.
    /// # Errors
    /// Same coverage, storage and capacity checks as [`Self::snapshot`].
    #[allow(clippy::too_many_lines)]
    pub fn snapshot_with_buffer(
        &self,
        revision: u64,
        time: Timestamp,
        poses: &BTreeMap<u64, Transformation>,
        backend: &OccupancyMapBackend,
        mut header: MapElement,
        mut cells: Vec<MapElement>,
    ) -> Result<Snapshot> {
        let region = MapRegion {
            min: header.min,
            max: header.max,
        };
        if !region.valid() {
            return Err(invalid("invalid local map coverage"));
        }
        let alignment = RigidTransform::from_parts(
            FrameId::MAP,
            FrameId::ODOM,
            header.translation,
            header.rotation,
        )?;
        let query = transformed(
            &alignment.matrix(),
            Vector3::from(region.min),
            Vector3::from(region.max),
        );
        let mut candidates = BTreeSet::new();
        for key in buckets(query)? {
            if let Some(ids) = self.buckets.get(&key) {
                candidates.extend(ids.iter().copied());
            }
        }
        header.timestamp = super::super::output::timestamp(time);
        cells.clear();
        cells.push(header);
        let mut visited_submaps = 0;
        let mut visited_octants = 0;
        for id in candidates {
            if !overlaps(self.bounds[&id], query) {
                continue;
            }
            visited_submaps += 1;
            let pose = poses
                .get(&id)
                .ok_or_else(|| invalid("submap pose missing during export"))?;
            let map = backend
                .maps
                .get(&id)
                .ok_or_else(|| invalid("indexed submap missing"))?
                .read()?;
            let transform = pose.matrix() * map.t_wm.to_homogeneous().cast::<f64>();
            let resolution = f64::from(map.resolution);
            let mut stack = vec![map.octree.root];
            while let Some(id) = stack.pop() {
                visited_octants += 1;
                let octant = map.octree.get(id)?;
                // Fusion propagates the maximum field to ancestors before this committed read.
                // Unknown aggregates are not evidence that descendants contain no obstacles.
                let maximum = octant.max_data();
                if maximum.valid() && !maximum.is_inside() {
                    continue;
                }
                let low = octant.base().coord.cast::<f64>() * resolution;
                let high = low + Vector3::repeat(f64::from(octant.base().size) * resolution);
                if !overlaps(transformed(&transform, low, high), query) {
                    continue;
                }
                if let Octant::Node(node) = octant
                    && !node.base.is_leaf()
                {
                    stack.extend(node.children.iter().flatten().copied());
                    continue;
                }
                let singleton = [octant.data()];
                let (scale, data) = match octant {
                    Octant::Block(block) => (
                        block.current_scale,
                        block.data_at_scale(block.current_scale)?,
                    ),
                    Octant::Node(node) => (node.base.scale(), singleton.as_slice()),
                };
                let stride = 1_i32 << scale;
                let n = (octant.base().size / stride) as usize;
                for (index, _) in data
                    .iter()
                    .enumerate()
                    .filter(|(_, value)| value.valid() && value.is_inside())
                {
                    let offset = Vector3::new(
                        (index % n) as i32,
                        (index / n % n) as i32,
                        (index / (n * n)) as i32,
                    );
                    let low = (octant.base().coord + offset * stride).cast::<f64>() * resolution;
                    let bounds = transformed(
                        &transform,
                        low,
                        low + Vector3::repeat(f64::from(stride) * resolution),
                    );
                    if !overlaps(bounds, query) {
                        continue;
                    }
                    if cells.len() > MAX_MAP_CELLS {
                        return Err(invalid("local occupancy snapshot capacity exhausted"));
                    }
                    cells.push(MapElement {
                        kind: 1,
                        min: bounds.0,
                        max: bounds.1,
                        ..Default::default()
                    });
                }
            }
        }
        Ok(Snapshot {
            graph_revision: revision,
            cells,
            visited_submaps,
            visited_octants,
        })
    }
}

fn transformed(transform: &Matrix4<f64>, low: Vector3<f64>, high: Vector3<f64>) -> Bounds {
    let rotation = transform.fixed_view::<3, 3>(0, 0);
    let center = rotation * ((low + high) * 0.5) + transform.fixed_view::<3, 1>(0, 3);
    let extent = rotation.map(f64::abs) * ((high - low) * 0.5);
    ((center - extent).into(), (center + extent).into())
}
fn overlaps(a: Bounds, b: Bounds) -> bool {
    (0..3).all(|i| a.0[i] < b.1[i] && a.1[i] > b.0[i])
}
fn buckets(bounds: Bounds) -> Result<impl Iterator<Item = [i64; 3]>> {
    let low: [i64; 3] = bounds.0.map(|v| (v / BUCKET_SIZE).floor() as i64);
    let high: [i64; 3] = bounds.1.map(|v| (v / BUCKET_SIZE).floor() as i64);
    let count = (0..3).try_fold(1_u64, |count, i| {
        u64::try_from(high[i].checked_sub(low[i])?.checked_add(1)?)
            .ok()?
            .checked_mul(count)
    });
    if !bounds
        .0
        .iter()
        .chain(bounds.1.iter())
        .all(|v| v.is_finite())
        || count.is_none_or(|count| count > MAX_MAP_CELLS as u64)
    {
        return Err(invalid("map region exceeds spatial index capacity"));
    }
    Ok((low[0]..=high[0]).flat_map(move |x| {
        (low[1]..=high[1]).flat_map(move |y| (low[2]..=high[2]).map(move |z| [x, y, z]))
    }))
}
