// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Non-GUI data flow and callbacks from original `src/mapper_sim.cpp`.
use crate::spline::se3_spline::Se3Spline;
use crate::utils::sophus_utils::se3_expd;
use crate::utils::sophus_utils::se3_inverse;
use crate::utils::sophus_utils::se3_product;
use crate::utils::sophus_utils::so3_inverse;
use crate::utils::sophus_utils::so3_product;
use crate::{
    io::marg_data_io::MargDataLoader,
    utils::{
        common_types::KeypointsData,
        imu_types::{AbsOrderMap, MargData, PoseStateWithLin, RelPoseFactor, RollPitchFactor},
        ldlt::Ldlt,
        nfr::{abs_position_error, rel_pose_error, roll_pitch_error, yaw_error},
        sim_utils::SimObservations,
        vio_config::VioConfig,
    },
    vi_estimator::{marg_helper::MargHelper, nfr_mapper::NfrMapper, vio_estimator::align_svd},
};
use firefly_calibration::calibration::Calibration;
use firefly_calibration::observation::TimeCamId;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{
    DMatrix, Isometry3, Quaternion, SMatrix, SVector, Translation3, UnitQuaternion, Vector3,
};
use serde::{
    Deserialize,
    de::{DeserializeOwned, MapAccess, Visitor},
};
use serde_json::Value;
use std::{
    collections::{BTreeMap, BTreeSet},
    fmt,
    fs::File,
    io::BufReader,
    path::Path,
    sync::mpsc,
};

pub(crate) fn invalid(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message.into()).with_operation("basalt.mapper_sim")
}
pub(crate) fn io_error(error: std::io::Error) -> Error {
    Error::from(error).with_operation("basalt.mapper_sim.io")
}

/// Cereal searches from the object's beginning only when its current name differs.
/// Keeping member order preserves the simulation archive's duplicate names.
pub(crate) struct Archive {
    entries: Vec<(String, Value)>,
    cursor: usize,
}
impl<'de> Deserialize<'de> for Archive {
    fn deserialize<D: serde::Deserializer<'de>>(
        deserializer: D,
    ) -> std::result::Result<Self, D::Error> {
        struct Members;
        impl<'de> Visitor<'de> for Members {
            type Value = Archive;
            fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
                f.write_str("Cereal JSON object")
            }
            fn visit_map<M: MapAccess<'de>>(
                self,
                mut map: M,
            ) -> std::result::Result<Self::Value, M::Error> {
                let mut entries = Vec::new();
                while let Some(entry) = map.next_entry()? {
                    entries.push(entry);
                }
                Ok(Archive { entries, cursor: 0 })
            }
        }
        deserializer.deserialize_map(Members)
    }
}
impl Archive {
    pub(crate) fn open(path: &Path) -> Result<Self> {
        serde_json::from_reader(BufReader::new(File::open(path).map_err(io_error)?))
            .map_err(|e| invalid(e.to_string()))
    }
    pub(crate) fn read<T: DeserializeOwned>(&mut self, name: &str) -> Result<T> {
        let index = if self
            .entries
            .get(self.cursor)
            .is_some_and(|(key, _)| key == name)
        {
            self.cursor
        } else {
            self.entries
                .iter()
                .position(|(key, _)| key == name)
                .ok_or_else(|| invalid(format!("Missing Cereal field {name}")))?
        };
        self.cursor = index + 1;
        serde_json::from_value(self.entries[index].1.clone()).map_err(|e| invalid(e.to_string()))
    }
    pub(crate) fn spline(&mut self) -> Result<Se3Spline<f64, 5>> {
        #[derive(Deserialize)]
        struct Pose {
            px: f64,
            py: f64,
            pz: f64,
            qx: f64,
            qy: f64,
            qz: f64,
            qw: f64,
        }
        let interval = self.read("t_ns")?;
        let knots: Vec<Pose> = self.read("knots")?;
        let mut spline = Se3Spline::new(interval, 0);
        for p in knots {
            spline.knots_push_back(&Isometry3::from_parts(
                Translation3::new(p.px, p.py, p.pz),
                UnitQuaternion::new_unchecked(Quaternion::new(p.qw, p.qx, p.qy, p.qz)),
            ));
        }
        Ok(spline)
    }
    pub(crate) fn observations(
        &mut self,
        name: &str,
    ) -> Result<BTreeMap<TimeCamId, SimObservations>> {
        #[derive(Deserialize)]
        struct Entry {
            key: TimeCamId,
            value: SimObservations,
        }
        Ok(self
            .read::<Vec<Entry>>(name)?
            .into_iter()
            .map(|e| (e.key, e.value))
            .collect())
    }
    pub(crate) fn vectors(&mut self, name: &str) -> Result<Vec<Vector3<f64>>> {
        Ok(self
            .read::<Vec<[f64; 3]>>(name)?
            .into_iter()
            .map(Vector3::from)
            .collect())
    }
}
pub(crate) fn load_calibration(path: &Path) -> Result<Calibration<f64>> {
    Archive::open(path)?.read("value0")
}
pub(crate) fn load_marginalization(path: &Path) -> Result<Vec<MargData>> {
    let (tx, rx) = mpsc::sync_channel(1000);
    let loader = MargDataLoader::start(path, tx)?;
    let mut result = Vec::new();
    while let Ok(Some(data)) = rx.recv() {
        result.push((*data).clone());
    }
    loader.finish()?;
    Ok(result)
}

pub struct MapperSimulation {
    pub calibration: Calibration<f64>,
    pub spline: Se3Spline<f64, 5>,
    pub gt_positions: Vec<Vector3<f64>>,
    pub gt_timestamps: Vec<i64>,
    pub observations: BTreeMap<TimeCamId, SimObservations>,
    pub noisy_observations: BTreeMap<TimeCamId, SimObservations>,
    pub marg_data: BTreeMap<i64, MargData>,
    pub mapper: NfrMapper,
    pub points: Vec<Vector3<f64>>,
    pub point_ids: Vec<i32>,
    pub edges: Vec<Vector3<f64>>,
    pub roll_pitch_edges: Vec<Vector3<f64>>,
    pub relative_edges: Vec<Vector3<f64>>,
    random: crate::vio_sim::EigenRandom,
}
impl MapperSimulation {
    /// # Errors
    /// Returns calibration, simulation archive or marginalization loading errors.
    pub fn load(calibration_path: &Path, cache_path: &Path) -> Result<Self> {
        let calibration = load_calibration(calibration_path)?;
        let mut archive = Archive::open(&cache_path.join("gt_spline.cereal"))?;
        let spline = archive.spline()?;
        let observations = archive.observations("gt_observations")?;
        let noisy_observations = archive.observations("noisy_observations")?;
        let mut gt_timestamps = Vec::new();
        let mut gt_positions = Vec::new();
        let mut time = 0;
        while time < spline.max_time_ns() {
            gt_timestamps.push(time);
            gt_positions.push(spline.pose(time, None).translation.vector);
            time += 1_000_000_000 / 50;
        }
        let mut marg_data = BTreeMap::new();
        for data in load_marginalization(cache_path)? {
            let time = *data
                .kfs_to_marg
                .first()
                .ok_or_else(|| invalid("Marginalization record has no marginalized keyframe"))?;
            marg_data.insert(time, data);
        }
        let mut mapper = NfrMapper::new(&calibration, &VioConfig::default());
        for data in marg_data.values_mut() {
            mapper.add_marg_data(data);
        }
        let mut result = Self {
            calibration,
            spline,
            gt_positions,
            gt_timestamps,
            observations,
            noisy_observations,
            marg_data,
            mapper,
            points: Vec::new(),
            point_ids: Vec::new(),
            edges: Vec::new(),
            roll_pitch_edges: Vec::new(),
            relative_edges: Vec::new(),
            random: crate::vio_sim::EigenRandom::new(),
        };
        result.compute_edge_vis();
        Ok(result)
    }
    /// Original observation IDs supply tracks; tracks need at least five views.
    /// # Errors
    /// Returns missing simulated observations or a poisoned feature store.
    pub fn setup_points(&mut self) -> Result<()> {
        for &time in self.mapper.base.frame_poses.keys() {
            for camera in 0..self.calibration.intrinsics.len() {
                let id = TimeCamId::new(time, camera);
                let obs = self
                    .noisy_observations
                    .get(&id)
                    .ok_or_else(|| invalid("Missing keyframe observations"))?;
                self.mapper
                    .feature_corners
                    .write()
                    .map_err(|_| invalid("Feature store poisoned"))?
                    .insert(
                        id,
                        KeypointsData {
                            corners: obs.pos.clone(),
                            ..KeypointsData::default()
                        },
                    );
                for (index, &track) in obs.id.iter().enumerate() {
                    self.mapper
                        .feature_tracks
                        .entry(i64::from(track))
                        .or_default()
                        .insert(
                            id,
                            i32::try_from(index)
                                .map_err(|_| invalid("Feature index exceeds i32"))?,
                        );
                }
            }
        }
        self.mapper
            .feature_tracks
            .retain(|_, track| track.len() >= 5);
        self.mapper.setup_opt();
        self.mapper
            .base
            .get_current_points(&mut self.points, &mut self.point_ids);
        Ok(())
    }
    pub fn optimize(&mut self) {
        self.mapper.optimize(10);
        self.mapper
            .base
            .get_current_points(&mut self.points, &mut self.point_ids);
        self.compute_edge_vis();
    }
    pub fn compute_edge_vis(&mut self) {
        self.edges.clear();
        for (host, targets) in self.mapper.base.lmdb.observations() {
            for target in targets.keys() {
                self.edges.push(
                    self.mapper.base.frame_poses[&host.frame_id]
                        .pose()
                        .translation
                        .vector,
                );
                self.edges.push(
                    self.mapper.base.frame_poses[&target.frame_id]
                        .pose()
                        .translation
                        .vector,
                );
            }
        }
        self.roll_pitch_edges.clear();
        for factor in &self.mapper.roll_pitch_factors {
            let pose = self.mapper.base.frame_poses[&factor.t_ns].pose();
            let p = pose.translation.vector;
            let d = so3_product(&factor.r_w_i_meas, &so3_inverse(&pose.rotation)) * (-Vector3::z());
            self.roll_pitch_edges.extend([p, p + 0.1 * d]);
        }
        self.relative_edges.clear();
        for factor in &self.mapper.rel_pose_factors {
            self.relative_edges.extend([
                self.mapper.base.frame_poses[&factor.t_i_ns]
                    .pose()
                    .translation
                    .vector,
                self.mapper.base.frame_poses[&factor.t_j_ns]
                    .pose()
                    .translation
                    .vector,
            ]);
        }
    }
    /// Uses the original persistent Eigen/glibc random stream.
    pub fn random_inc(&mut self) {
        let sample = SVector::<f64, 6>::from_fn(|_, _| self.random.sample());
        self.apply_increment(se3_expd(&(sample.map(f64::abs) / 10.)));
    }
    pub fn random_yaw_inc(&mut self) {
        let mut delta = SVector::<f64, 6>::zeros();
        delta[5] = self.random.sample().abs();
        self.apply_increment(se3_expd(&delta));
    }
    fn apply_increment(&mut self, increment: Isometry3<f64>) {
        for (&time, state) in &mut self.mapper.base.frame_poses {
            *state = PoseStateWithLin::new(time, se3_product(&increment, state.pose()), false);
        }
        self.compute_edge_vis();
    }
    pub fn align(&mut self) -> f64 {
        let times: Vec<_> = self.mapper.base.frame_poses.keys().copied().collect();
        let positions: Vec<_> = self
            .mapper
            .base
            .frame_poses
            .values()
            .map(|s| s.pose().translation.vector)
            .collect();
        align_svd(
            &times,
            &positions,
            &self.gt_timestamps,
            &mut self.gt_positions,
        )
    }
    /// Executes the source's headless setup and ten-iteration optimization.
    /// # Errors
    /// Returns missing observation errors.
    pub fn run(&mut self) -> Result<()> {
        self.setup_points()?;
        self.optimize();
        Ok(())
    }
}

/// Standalone source callback; unlike `NfrMapper`, always marginalizes the matrix.
/// # Panics
/// Requires consistent order dimensions, referenced states and 6- or 15-element blocks.
pub fn process_marg_data(data: &mut MargData) {
    assert_eq!(data.aom.total_size, data.abs_h.ncols());
    let mut order = AbsOrderMap::default();
    let (mut keep, mut remove) = (BTreeSet::new(), BTreeSet::new());
    for (&time, &(offset, size)) in &data.aom.abs_order_map {
        let offset = offset as usize;
        match size {
            6 => {
                keep.extend(offset..offset + 6);
                order.abs_order_map.insert(time, (offset as i32, 6));
                order.total_size += 6;
            }
            15 => {
                if data.kfs_all.contains(&time) {
                    keep.extend(offset..offset + 6);
                    remove.extend(offset + 6..offset + 15);
                    order
                        .abs_order_map
                        .insert(time, (order.total_size as i32, 6));
                    order.total_size += 6;
                    data.frame_poses.insert(
                        time,
                        PoseStateWithLin::from_imu_state(&data.frame_states[&time]),
                    );
                } else {
                    remove.extend(offset..offset + 15);
                }
                data.frame_states.remove(&time);
            }
            _ => panic!("Unknown marginalization block size"),
        }
    }
    (data.abs_h, data.abs_b) =
        MargHelper::marginalize_sq_to_sq(&mut data.abs_h, &mut data.abs_b, &keep, &remove);
    data.aom = order;
    assert_eq!(data.aom.total_size, data.abs_h.ncols());
}
/// Standalone source callback using LDLT covariance and absolute/relative pose factors.
/// # Panics
/// Requires a marginalized keyframe and invertible roll-pitch covariance.
#[must_use]
pub fn extract_nonlinear_factors(data: &MargData) -> (Vec<RollPitchFactor>, Vec<RelPoseFactor>) {
    let size = data.aom.total_size;
    let covariance = Ldlt::new(data.abs_h.clone()).solve(&DMatrix::identity(size, size));
    let keyframe = *data.kfs_to_marg.first().expect("marginalized keyframe");
    let start = data.aom.abs_order_map[&keyframe].0 as usize;
    let pose = data.frame_poses[&keyframe].pose();
    let (mut jp, mut jy, mut jr) = (
        SMatrix::<f64, 3, 6>::zeros(),
        SMatrix::<f64, 1, 6>::zeros(),
        SMatrix::<f64, 2, 6>::zeros(),
    );
    let _ = abs_position_error(pose, &pose.translation.vector, Some(&mut jp));
    let _ = yaw_error(
        pose,
        &(so3_inverse(&pose.rotation) * Vector3::x()),
        Some(&mut jy),
    );
    let _ = roll_pitch_error(pose, &pose.rotation, Some(&mut jr));
    let mut j = DMatrix::zeros(6, size);
    j.view_mut((0, start), (3, 6)).copy_from(&jp);
    j.view_mut((3, start), (1, 6)).copy_from(&jy);
    j.view_mut((4, start), (2, 6)).copy_from(&jr);
    let cov = &j * &covariance * j.transpose();
    let rp = RollPitchFactor {
        t_ns: keyframe,
        r_w_i_meas: pose.rotation,
        cov_inv: cov
            .fixed_view::<2, 2>(4, 4)
            .try_inverse()
            .expect("invertible roll-pitch covariance"),
    };
    let mut relative = Vec::new();
    for &other in &data.kfs_all {
        if other == keyframe {
            continue;
        }
        let Some(state) = data.frame_poses.get(&other) else {
            continue;
        };
        let measurement = se3_product(&se3_inverse(pose), state.pose());
        let (mut ji, mut jj) = (SMatrix::<f64, 6, 6>::zeros(), SMatrix::<f64, 6, 6>::zeros());
        let _ = rel_pose_error(
            &measurement,
            pose,
            state.pose(),
            Some(&mut ji),
            Some(&mut jj),
        );
        let mut j = DMatrix::zeros(6, size);
        j.view_mut((0, start), (6, 6)).copy_from(&ji);
        j.view_mut((0, data.aom.abs_order_map[&other].0 as usize), (6, 6))
            .copy_from(&jj);
        let cov = &j * &covariance * j.transpose();
        relative.push(RelPoseFactor {
            t_i_ns: keyframe,
            t_j_ns: other,
            t_i_j: measurement,
            cov_inv: Ldlt::new(cov)
                .solve(&DMatrix::identity(6, 6))
                .fixed_view::<6, 6>(0, 0)
                .into_owned(),
        });
    }
    (vec![rp], relative)
}
