//! Owned recording events keep only the active part of each shared-memory sample.
use firefly_error::Result;
use firefly_pubsub::viz::{VizMessage, VoxelMessage, kind};

use crate::{invalid, schema};

fn prefix<T>(values: &[T], count: u32) -> Result<&[T]> {
    values
        .get(..count as usize)
        .ok_or_else(|| invalid("observation count exceeds capacity"))
}

pub(crate) struct Geometry {
    pub name: Box<str>,
    pub timestamp: f64,
    pub color: [u8; 3],
    pub data: GeometryData,
}

pub(crate) enum GeometryData {
    Pose {
        position: [f64; 3],
        rotation: [f64; 4],
    },
    Scalars(Box<[f64]>),
    Line(Box<[[f64; 3]]>),
    Keypoints {
        points: Box<[[f64; 3]]>,
        detected: f64,
    },
    Arrows {
        origins: Box<[[f64; 3]]>,
        vectors: Box<[[f64; 3]]>,
    },
    Bars(Box<[u64]>),
    Clear,
}

impl Geometry {
    pub fn from_message(msg: &VizMessage) -> Result<Self> {
        let name = schema::entity(&msg.entity, msg.entity_len)?;
        schema::spatial(name, msg.kind)?;
        schema::timestamp(msg.timestamp)?;
        let data = match msg.kind {
            kind::POSE => GeometryData::Pose {
                position: msg.xyz,
                rotation: msg.quat_xyzw,
            },
            kind::SCALARS => GeometryData::Scalars(prefix(&msg.scalars, msg.scalar_count)?.into()),
            kind::LINE_STRIP => GeometryData::Line(prefix(&msg.points, msg.point_count)?.into()),
            kind::KEYPOINTS_2D => {
                let points = prefix(&msg.points, msg.point_count)?;
                if msg.scalar_count != 3
                    || msg.scalars[2] < f64::from(msg.point_count)
                    || points.iter().flatten().any(|v| !v.is_finite())
                {
                    return Err(invalid("keypoint metadata"));
                }
                GeometryData::Keypoints {
                    points: points.into(),
                    detected: msg.scalars[2],
                }
            }
            kind::ARROWS => GeometryData::Arrows {
                origins: prefix(&msg.arrow_origins, msg.arrow_count)?.into(),
                vectors: prefix(&msg.arrow_vectors, msg.arrow_count)?.into(),
            },
            kind::BAR_CHART => GeometryData::Bars(prefix(&msg.bins, msg.bin_count)?.into()),
            kind::CLEAR => GeometryData::Clear,
            other => return Err(invalid(format!("unsupported geometry kind {other}"))),
        };
        Ok(Self {
            name: name.into(),
            timestamp: msg.timestamp,
            color: msg.color,
            data,
        })
    }

    pub fn heap_bytes(&self) -> usize {
        self.name.len()
            + match &self.data {
                GeometryData::Pose { .. } | GeometryData::Clear => 0,
                GeometryData::Scalars(values) => size_of_val(&**values),
                GeometryData::Line(points) | GeometryData::Keypoints { points, .. } => {
                    size_of_val(&**points)
                }
                GeometryData::Arrows { origins, vectors } => {
                    size_of_val(&**origins) + size_of_val(&**vectors)
                }
                GeometryData::Bars(values) => size_of_val(&**values),
            }
    }
}

pub(crate) struct VoxelChunk {
    pub name: Box<str>,
    pub kind: u32,
    pub timestamp: f64,
    pub voxel_size: [f32; 3],
    pub voxel_origin: [f32; 3],
    pub chunk_count: u32,
    pub chunk_index: u32,
    pub voxels: Box<[[i32; 3]]>,
}

impl VoxelChunk {
    pub fn from_message(msg: &VoxelMessage) -> Result<Self> {
        let name = schema::entity(&msg.entity, msg.entity_len)?;
        schema::spatial(name, msg.kind)?;
        schema::timestamp(msg.timestamp)?;
        Ok(Self {
            name: name.into(),
            kind: msg.kind,
            timestamp: msg.timestamp,
            voxel_size: msg.voxel_size,
            voxel_origin: msg.voxel_origin,
            chunk_count: msg.chunk_count,
            chunk_index: msg.chunk_index,
            voxels: prefix(&msg.voxels, msg.voxel_count)?.into(),
        })
    }

    pub fn heap_bytes(&self) -> usize {
        self.name.len() + size_of_val(&*self.voxels)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sparse_events_charge_only_their_valid_prefix() {
        let mut msg = VoxelMessage::base(kind::VOXELS_ADD, 1., "plan/collision");
        msg.voxel_count = 1;
        msg.voxels[0] = [1, 2, 3];
        let chunk = VoxelChunk::from_message(&msg).unwrap();
        assert_eq!(&*chunk.voxels, &[[1, 2, 3]]);
        assert_eq!(chunk.heap_bytes(), "plan/collision".len() + 12);
        msg.voxel_count = u32::MAX;
        assert!(VoxelChunk::from_message(&msg).is_err());

        let mut msg = VizMessage::base(kind::LINE_STRIP, 1., "plan/global_path");
        msg.point_count = 2;
        msg.arrow_count = u32::MAX;
        let line = Geometry::from_message(&msg).unwrap();
        assert_eq!(
            line.heap_bytes(),
            "plan/global_path".len() + 2 * size_of::<[f64; 3]>()
        );
        msg.point_count = u32::MAX;
        assert!(Geometry::from_message(&msg).is_err());
    }
}
