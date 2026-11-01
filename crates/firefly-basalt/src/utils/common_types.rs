// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2018, Vladyslav Usenko and Nikolaus Demmel.
// Cereal bitset encoding: Copyright (c) 2014, Randolph Voorhies, Shane Grant.
// Eigen/Sophus archive layout: Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
// Source: VladyslavUsenko/basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc,
// include/basalt/utils/common_types.h. See the crate LICENSE-BSD-3-Clause.

use nalgebra::{Isometry3, Vector2, Vector3, Vector4};
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, HashMap},
    sync::{Arc, RwLock},
};

pub type FeatureId = i32;
pub type FrameId = i64;
pub type CamId = usize;

pub use firefly_calibration::observation::TimeCamId;

/// Numeric hash of an image pair, matching upstream's four sequential combines.
#[must_use]
pub fn image_pair_hash(left: TimeCamId, right: TimeCamId) -> usize {
    let mut seed = left.hash_value();
    hash_combine(&mut seed, right.frame_id as usize);
    hash_combine(&mut seed, right.cam_id);
    seed
}

fn hash_combine(seed: &mut usize, value_hash: usize) {
    #[cfg(target_pointer_width = "64")]
    {
        *seed = (*seed as u64
            ^ (value_hash as u64)
                .wrapping_add(0x9e37_79b9_7f4a_7c15)
                .wrapping_add((*seed as u64) << 12)
                .wrapping_add((*seed as u64) >> 4)) as usize;
    }
    #[cfg(target_pointer_width = "32")]
    {
        *seed = (*seed as u32
            ^ (value_hash as u32)
                .wrapping_add(0x9e37_79b9)
                .wrapping_add((*seed as u32) << 6)
                .wrapping_add((*seed as u32) >> 2)) as usize;
    }
}
pub const FEATURE_HASH_MAX_SIZE: usize = 32;
pub type FeatureHash = u32;
pub type HashBowVector = Vec<(FeatureHash, f64)>;
/// Bit i occupies bit i % 64 of word i / 64, matching `std::bitset` indexing.
pub type Descriptor = [u64; 4];

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(from = "archive::Keypoints", into = "archive::Keypoints")]
pub struct KeypointsData {
    pub corners: Vec<Vector2<f64>>,
    pub corner_angles: Vec<f64>,
    pub corner_descriptors: Vec<Descriptor>,
    pub corners_3d: Vec<Vector4<f64>>,
    pub hashes: Vec<FeatureHash>,
    pub bow_vector: HashBowVector,
}
pub type Corners = RwLock<HashMap<TimeCamId, KeypointsData>>;

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(from = "archive::Matches", into = "archive::Matches")]
pub struct MatchData {
    /// Transforms a point from the second image frame into the first.
    pub t_i_j: Isometry3<f64>,
    pub matches: Vec<(FeatureId, FeatureId)>,
    pub inliers: Vec<(FeatureId, FeatureId)>,
}
pub type Matches = RwLock<HashMap<(TimeCamId, TimeCamId), MatchData>>;
pub type ImageFeaturePair = (TimeCamId, FeatureId);
pub type FeatureTrack = BTreeMap<TimeCamId, FeatureId>;
pub type TrackId = i64;
pub type FeatureTracks = HashMap<TrackId, FeatureTrack>;

#[derive(Clone, Debug, Default)]
pub struct Camera {
    /// Camera-to-world transform.
    pub t_w_c: Isometry3<f64>,
}
#[derive(Clone, Debug, Default)]
pub struct Landmark {
    pub p: Vector3<f64>,
    pub obs: FeatureTrack,
    pub outlier_obs: FeatureTrack,
}
pub type Cameras = BTreeMap<TimeCamId, Camera>;
pub type Landmarks = HashMap<TrackId, Landmark>;

#[derive(Clone, Debug, Default)]
pub struct CameraCandidate {
    pub tcid: TimeCamId,
    pub shared_tracks: Vec<TrackId>,
    pub tried: bool,
    pub camera_added: bool,
    pub landmarks_added: bool,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum CandidateStage {
    #[default]
    ComputeCandidates,
    AddCameras,
    AddLandmarks,
    Optimize,
    RemoveOutliers,
    Done,
}
#[derive(Clone, Debug, Default)]
pub struct CameraCandidates {
    pub cameras: Vec<CameraCandidate>,
    pub current_stage: CandidateStage,
    pub min_localization_inliers: i32,
    pub max_cameras_to_add: i32,
}
impl CameraCandidates {
    #[must_use]
    pub fn num_cameras_added(&self) -> i32 {
        self.cameras
            .iter()
            .filter(|c| c.camera_added)
            .fold(0, |count, _| count + 1)
    }
    #[must_use]
    pub fn num_landmarks_added(&self) -> i32 {
        self.cameras
            .iter()
            .filter(|c| c.landmarks_added)
            .fold(0, |count, _| count + 1)
    }
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutlierFlags {
    None = 0,
    ReprojectionErrorHuge = 1,
    ReprojectionErrorNormal = 2,
    CameraDistance = 4,
    ZCoordinate = 8,
}
#[derive(Clone, Debug)]
pub struct ProjectedLandmark {
    pub point_measured: Vector2<f64>,
    pub point_reprojected: Vector2<f64>,
    pub point_3d_c: Vector3<f64>,
    pub track_id: TrackId,
    pub reprojection_error: f64,
    pub outlier_flags: u32,
}
impl Default for ProjectedLandmark {
    fn default() -> Self {
        Self {
            point_measured: Vector2::zeros(),
            point_reprojected: Vector2::zeros(),
            point_3d_c: Vector3::zeros(),
            track_id: -1,
            reprojection_error: 0.,
            outlier_flags: OutlierFlags::None as u32,
        }
    }
}
pub type ProjectedLandmarkPtr = Arc<RwLock<ProjectedLandmark>>;
pub type ProjectedLandmarkConstPtr = Arc<ProjectedLandmark>;
#[derive(Clone, Debug, Default)]
pub struct ImageProjection {
    pub obs: Vec<ProjectedLandmarkConstPtr>,
    pub outlier_obs: Vec<ProjectedLandmarkConstPtr>,
}
pub type ImageProjections = BTreeMap<TimeCamId, ImageProjection>;
pub type TrackProjections = HashMap<TrackId, BTreeMap<TimeCamId, ProjectedLandmarkConstPtr>>;

#[cfg(test)]
mod tests {
    use super::*;
    use std::fmt::Write;

    fn oracle() -> serde_json::Value {
        serde_json::from_str(include_str!("../../tests/reference/common_types.json")).unwrap()
    }
    #[test]
    fn json_matches_cpp_cereal_and_omits_derived_keypoint_fields() {
        let expected = oracle();
        let id: TimeCamId = serde_json::from_value(expected["id"].clone()).unwrap();
        assert_eq!(id, TimeCamId::new(-123, 2));
        assert_eq!(serde_json::to_value(id).unwrap(), expected["id"]);
        let mut points: KeypointsData = serde_json::from_value(expected["points"].clone()).unwrap();
        assert_eq!(
            points.corners,
            [Vector2::new(1.25, -2.5), Vector2::new(4., 8.)]
        );
        assert_eq!(
            points.corner_descriptors,
            [[1, 0, 0, 0], [0, 0, 0, 1_u64 << 63], [u64::MAX; 4]]
        );
        points.corners_3d.push(Vector4::repeat(123.));
        points.hashes.push(789);
        points.bow_vector.push((42, 1.));
        assert_eq!(serde_json::to_value(points).unwrap(), expected["points"]);
        let matches: MatchData = serde_json::from_value(expected["matches"].clone()).unwrap();
        assert_eq!(matches.matches, [(-1, 2), (3, 4)]);
        assert_eq!(matches.t_i_j.translation.vector, Vector3::new(1., 2., 3.));
        assert_eq!(serde_json::to_value(matches).unwrap(), expected["matches"]);
    }

    #[test]
    #[cfg(all(target_endian = "little", target_pointer_width = "64"))]
    fn native_binary_and_hash_match_cpp_exactly() {
        let expected = oracle();
        let id: TimeCamId = serde_json::from_value(expected["id"].clone()).unwrap();
        let points: KeypointsData = serde_json::from_value(expected["points"].clone()).unwrap();
        let matches: MatchData = serde_json::from_value(expected["matches"].clone()).unwrap();
        assert_eq!(id.hash_value() as u64, expected["hash"].as_u64().unwrap());
        assert_eq!(
            image_pair_hash(id, TimeCamId::new(456, 1)) as u64,
            expected["pair_hash"].as_u64().unwrap()
        );
        let mut bytes = Vec::new();
        id.write_cereal_binary(&mut bytes).unwrap();
        points.write_cereal_binary(&mut bytes).unwrap();
        let match_start = bytes.len();
        matches.write_cereal_binary(&mut bytes).unwrap();
        let mut hex = String::new();
        for byte in &bytes {
            write!(&mut hex, "{byte:02x}").unwrap();
        }
        assert_eq!(hex, expected["binary"].as_str().unwrap());
        let mut reader = bytes.as_slice();
        assert_eq!(TimeCamId::read_cereal_binary(&mut reader).unwrap(), id);
        assert_eq!(
            serde_json::to_value(KeypointsData::read_cereal_binary(&mut reader).unwrap()).unwrap(),
            expected["points"]
        );
        assert_eq!(
            serde_json::to_value(MatchData::read_cereal_binary(&mut reader).unwrap()).unwrap(),
            expected["matches"]
        );
        assert_eq!(reader, [0_u8; 0]);
        assert!(MatchData::read_cereal_binary(&mut &bytes[match_start..bytes.len() - 1]).is_err());
    }

    #[test]
    fn bitset_encodings_and_malformed_archives() {
        for kind in [0, 1] {
            let value = serde_json::json!({"value0":[],"value1":[],"value2":[{"type":kind,"data":u64::MAX}]});
            let points: KeypointsData = serde_json::from_value(value).unwrap();
            assert_eq!(points.corner_descriptors, [[u64::MAX, 0, 0, 0]]);
        }
        let value =
            serde_json::json!({"value0":[],"value1":[],"value2":[{"type":2,"data":"00101"}]});
        assert_eq!(
            serde_json::from_value::<KeypointsData>(value)
                .unwrap()
                .corner_descriptors,
            [[5, 0, 0, 0]]
        );
        for descriptor in [
            serde_json::json!({"type":4,"data":0}),
            serde_json::json!({"type":2,"data":"102"}),
        ] {
            let value = serde_json::json!({"value0":[],"value1":[],"value2":[descriptor]});
            assert!(serde_json::from_value::<KeypointsData>(value).is_err());
        }
        for length in 0..8 + std::mem::size_of::<usize>() {
            let bytes = vec![0; length];
            assert!(TimeCamId::read_cereal_binary(&mut bytes.as_slice()).is_err());
        }
    }
}

mod binary {
    use super::{Descriptor, Isometry3, KeypointsData, MatchData, TimeCamId, Vector2};
    use firefly_error::{Error, ErrorKind, Result};
    use nalgebra::{Quaternion, Translation3, UnitQuaternion};
    use std::io::{self, Read, Write};

    /// The native cereal `BinaryArchive` representation, without a portable-archive endian header.
    pub trait CerealBinary: Sized {
        /// # Errors
        /// Returns an error for truncated or malformed input or failed I/O.
        fn read_cereal_binary(reader: &mut impl Read) -> Result<Self>;
        /// # Errors
        /// Returns an error if writing fails.
        fn write_cereal_binary(&self, writer: &mut impl Write) -> Result<()>;
    }
    fn error(source: io::Error) -> Error {
        Error::new(
            ErrorKind::InvalidArgument,
            "Cannot process cereal binary archive",
        )
        .with_operation("basalt.cereal.binary")
        .with_source(source)
    }
    fn bytes<const N: usize>(reader: &mut impl Read) -> io::Result<[u8; N]> {
        let mut value = [0; N];
        reader.read_exact(&mut value)?;
        Ok(value)
    }
    fn count(reader: &mut impl Read) -> io::Result<u64> {
        Ok(u64::from_ne_bytes(bytes(reader)?))
    }
    fn scalar(reader: &mut impl Read) -> io::Result<f64> {
        Ok(f64::from_ne_bytes(bytes(reader)?))
    }
    fn write_count(writer: &mut impl Write, count: usize) -> io::Result<()> {
        writer.write_all(&(count as u64).to_ne_bytes())
    }
    fn invalid(message: &str) -> io::Error {
        io::Error::new(io::ErrorKind::InvalidData, message)
    }
    fn descriptor(reader: &mut impl Read) -> io::Result<Descriptor> {
        let mut words = [0_u64; 4];
        match bytes::<1>(reader)?[0] {
            0 => words[0] = std::os::raw::c_ulong::from_ne_bytes(bytes(reader)?) as u64,
            1 => words[0] = u64::from_ne_bytes(bytes(reader)?),
            2 => {
                let len = count(reader)?;
                let mut text = Vec::new();
                for _ in 0..len {
                    text.push(bytes::<1>(reader)?[0]);
                }
                for (bit, &byte) in text[..text.len().min(256)].iter().rev().enumerate() {
                    match byte {
                        b'0' => {}
                        b'1' => words[bit / 64] |= 1_u64 << (bit % 64),
                        _ => return Err(invalid("invalid bitset character")),
                    }
                }
            }
            3 => {
                for byte in 0..32 {
                    words[byte / 8] |=
                        u64::from(bytes::<1>(reader)?[0].reverse_bits()) << ((byte % 8) * 8);
                }
            }
            _ => return Err(invalid("invalid bitset representation")),
        }
        Ok(words)
    }
    fn read_points(reader: &mut impl Read) -> io::Result<KeypointsData> {
        let mut result = KeypointsData::default();
        for _ in 0..count(reader)? {
            result
                .corners
                .push(Vector2::new(scalar(reader)?, scalar(reader)?));
        }
        for _ in 0..count(reader)? {
            result.corner_angles.push(scalar(reader)?);
        }
        for _ in 0..count(reader)? {
            result.corner_descriptors.push(descriptor(reader)?);
        }
        Ok(result)
    }
    fn write_points(data: &KeypointsData, writer: &mut impl Write) -> io::Result<()> {
        write_count(writer, data.corners.len())?;
        for p in &data.corners {
            writer.write_all(&p.x.to_ne_bytes())?;
            writer.write_all(&p.y.to_ne_bytes())?;
        }
        write_count(writer, data.corner_angles.len())?;
        for angle in &data.corner_angles {
            writer.write_all(&angle.to_ne_bytes())?;
        }
        write_count(writer, data.corner_descriptors.len())?;
        for words in &data.corner_descriptors {
            writer.write_all(&[3])?;
            for byte in 0..32 {
                writer
                    .write_all(&[((words[byte / 8] >> ((byte % 8) * 8)) as u8).reverse_bits()])?;
            }
        }
        Ok(())
    }
    fn pairs(reader: &mut impl Read) -> io::Result<Vec<(i32, i32)>> {
        let mut result = Vec::new();
        for _ in 0..count(reader)? {
            result.push((
                i32::from_ne_bytes(bytes(reader)?),
                i32::from_ne_bytes(bytes(reader)?),
            ));
        }
        Ok(result)
    }
    fn read_match(reader: &mut impl Read) -> io::Result<MatchData> {
        let mut pose = [0.; 7];
        for value in &mut pose {
            *value = scalar(reader)?;
        }
        Ok(MatchData {
            t_i_j: Isometry3::from_parts(
                Translation3::new(pose[0], pose[1], pose[2]),
                UnitQuaternion::new_unchecked(Quaternion::new(pose[6], pose[3], pose[4], pose[5])),
            ),
            matches: pairs(reader)?,
            inliers: pairs(reader)?,
        })
    }
    fn write_match(data: &MatchData, writer: &mut impl Write) -> io::Result<()> {
        let t = data.t_i_j.translation.vector;
        let q = data.t_i_j.rotation.quaternion();
        for value in [t.x, t.y, t.z, q.i, q.j, q.k, q.w] {
            writer.write_all(&value.to_ne_bytes())?;
        }
        for pairs in [&data.matches, &data.inliers] {
            write_count(writer, pairs.len())?;
            for (a, b) in pairs {
                writer.write_all(&a.to_ne_bytes())?;
                writer.write_all(&b.to_ne_bytes())?;
            }
        }
        Ok(())
    }
    impl CerealBinary for TimeCamId {
        fn read_cereal_binary(reader: &mut impl Read) -> Result<Self> {
            let read = |reader: &mut _| -> io::Result<Self> {
                Ok(Self::new(
                    i64::from_ne_bytes(bytes(reader)?),
                    usize::from_ne_bytes(bytes(reader)?),
                ))
            };
            read(reader).map_err(error)
        }
        fn write_cereal_binary(&self, writer: &mut impl Write) -> Result<()> {
            writer
                .write_all(&self.frame_id.to_ne_bytes())
                .and_then(|()| writer.write_all(&self.cam_id.to_ne_bytes()))
                .map_err(error)
        }
    }
    impl CerealBinary for KeypointsData {
        fn read_cereal_binary(reader: &mut impl Read) -> Result<Self> {
            read_points(reader).map_err(error)
        }
        fn write_cereal_binary(&self, writer: &mut impl Write) -> Result<()> {
            write_points(self, writer).map_err(error)
        }
    }
    impl CerealBinary for MatchData {
        fn read_cereal_binary(reader: &mut impl Read) -> Result<Self> {
            read_match(reader).map_err(error)
        }
        fn write_cereal_binary(&self, writer: &mut impl Write) -> Result<()> {
            write_match(self, writer).map_err(error)
        }
    }
}
pub use binary::CerealBinary;

mod archive {
    use super::{Descriptor, Deserialize, Isometry3, KeypointsData, MatchData, Serialize, Vector2};
    use nalgebra::{Quaternion, Translation3, UnitQuaternion};

    #[derive(Serialize, Deserialize)]
    pub(super) struct Keypoints {
        value0: Vec<[f64; 2]>,
        value1: Vec<f64>,
        value2: Vec<Bits>,
    }
    impl From<KeypointsData> for Keypoints {
        fn from(data: KeypointsData) -> Self {
            Self {
                value0: data.corners.into_iter().map(|v| [v.x, v.y]).collect(),
                value1: data.corner_angles,
                value2: data.corner_descriptors.into_iter().map(Bits).collect(),
            }
        }
    }
    impl From<Keypoints> for KeypointsData {
        fn from(data: Keypoints) -> Self {
            Self {
                corners: data.value0.into_iter().map(Vector2::from).collect(),
                corner_angles: data.value1,
                corner_descriptors: data.value2.into_iter().map(|b| b.0).collect(),
                ..Self::default()
            }
        }
    }

    #[derive(Serialize, Deserialize)]
    pub(super) struct Matches {
        value0: Pose,
        value1: Vec<Pair>,
        value2: Vec<Pair>,
    }
    #[derive(Serialize, Deserialize)]
    struct Pair {
        first: i32,
        second: i32,
    }
    #[derive(Serialize, Deserialize)]
    struct Pose {
        px: f64,
        py: f64,
        pz: f64,
        qx: f64,
        qy: f64,
        qz: f64,
        qw: f64,
    }
    impl From<MatchData> for Matches {
        fn from(data: MatchData) -> Self {
            let t = data.t_i_j.translation.vector;
            let q = data.t_i_j.rotation.quaternion();
            Self {
                value0: Pose {
                    px: t.x,
                    py: t.y,
                    pz: t.z,
                    qx: q.i,
                    qy: q.j,
                    qz: q.k,
                    qw: q.w,
                },
                value1: data
                    .matches
                    .into_iter()
                    .map(|(first, second)| Pair { first, second })
                    .collect(),
                value2: data
                    .inliers
                    .into_iter()
                    .map(|(first, second)| Pair { first, second })
                    .collect(),
            }
        }
    }
    impl From<Matches> for MatchData {
        fn from(data: Matches) -> Self {
            let p = data.value0;
            Self {
                t_i_j: Isometry3::from_parts(
                    Translation3::new(p.px, p.py, p.pz),
                    UnitQuaternion::new_unchecked(Quaternion::new(p.qw, p.qx, p.qy, p.qz)),
                ),
                matches: data
                    .value1
                    .into_iter()
                    .map(|p| (p.first, p.second))
                    .collect(),
                inliers: data
                    .value2
                    .into_iter()
                    .map(|p| (p.first, p.second))
                    .collect(),
            }
        }
    }

    struct Bits(Descriptor);
    impl Serialize for Bits {
        fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            use serde::ser::SerializeStruct;
            let mut state = serializer.serialize_struct("bitset", 2)?;
            // On LP64 hosts both unsigned long and unsigned long long are 64 bits.
            if self.0[1..].iter().all(|&word| word == 0) {
                let kind = u8::from(std::os::raw::c_ulong::try_from(self.0[0]).is_err());
                state.serialize_field("type", &kind)?;
                state.serialize_field("data", &self.0[0])?;
            } else {
                state.serialize_field("type", &2_u8)?;
                let text: String = (0..256)
                    .rev()
                    .map(|bit| {
                        if self.0[bit / 64] & (1_u64 << (bit % 64)) == 0 {
                            '0'
                        } else {
                            '1'
                        }
                    })
                    .collect();
                state.serialize_field("data", &text)?;
            }
            state.end()
        }
    }
    impl<'de> Deserialize<'de> for Bits {
        fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            use serde::de::Error;
            let value = serde_json::Value::deserialize(deserializer)?;
            let mut words = [0_u64; 4];
            match value["type"].as_u64() {
                Some(0 | 1) => {
                    words[0] = value["data"]
                        .as_u64()
                        .ok_or_else(|| D::Error::custom("invalid integer bitset"))?;
                }
                Some(2) => {
                    let text = value["data"]
                        .as_str()
                        .ok_or_else(|| D::Error::custom("invalid string bitset"))?;
                    let bytes = &text.as_bytes()[..text.len().min(256)];
                    for (bit, &byte) in bytes.iter().rev().enumerate() {
                        match byte {
                            b'0' => {}
                            b'1' => words[bit / 64] |= 1_u64 << (bit % 64),
                            _ => return Err(D::Error::custom("invalid bitset character")),
                        }
                    }
                }
                Some(3) => {
                    for byte in 0..32 {
                        let key = format!("value{byte}");
                        let v = value[&key]
                            .as_u64()
                            .and_then(|v| u8::try_from(v).ok())
                            .ok_or_else(|| D::Error::custom("invalid binary bitset chunk"))?;
                        words[byte / 8] |= u64::from(v.reverse_bits()) << ((byte % 8) * 8);
                    }
                }
                _ => return Err(D::Error::custom("invalid bitset representation")),
            }
            Ok(Self(words))
        }
    }
}
