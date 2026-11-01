// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original `src/mapper.cpp` offline controller, cache I/O and trajectory export.
use crate::utils::sophus_utils::so3_inverse;
use crate::utils::sophus_utils::so3_product;
use crate::{
    io::marg_data_io::binary,
    mapper_sim::{load_calibration, load_marginalization},
    utils::{imu_types::MargData, vio_config::VioConfig},
    vi_estimator::{nfr_mapper::NfrMapper, vio_estimator::align_svd},
};
use firefly_error::Result;
use nalgebra::Vector3;
use std::{
    collections::BTreeMap,
    fs::File,
    io::{BufReader, Write},
    path::Path,
};

pub struct Mapper {
    pub mapper: NfrMapper,
    pub marg_data: BTreeMap<i64, MargData>,
    pub gt_timestamps: Vec<i64>,
    pub gt_positions: Vec<Vector3<f64>>,
    pub image_timestamps: Vec<i64>,
    pub points: Vec<Vector3<f64>>,
    pub point_ids: Vec<i32>,
    pub edges: Vec<Vector3<f64>>,
    pub roll_pitch_edges: Vec<Vector3<f64>>,
    pub relative_edges: Vec<Vector3<f64>>,
    pub num_opt_iter: usize,
    pub outlier_threshold: f64,
}
impl Mapper {
    /// Loads the original native `gt.cereal` and marginalization cache, then adds
    /// marginalization factors in timestamp order, as the original main does.
    /// # Errors
    /// Reports missing, truncated or invalid archives.
    pub fn load_data(
        calibration_path: &Path,
        cache_path: &Path,
        config: &VioConfig,
    ) -> Result<Self> {
        let calibration = load_calibration(calibration_path)?;
        let mut input =
            BufReader::new(File::open(cache_path.join("gt.cereal")).map_err(binary::error)?);
        let (gt_timestamps, gt_positions) = read_ground_truth(&mut input)?;
        let mut marg_data = BTreeMap::new();
        for data in load_marginalization(cache_path)? {
            let time = *data
                .kfs_to_marg
                .first()
                .ok_or_else(|| binary::invalid("Empty marginalized keyframe set"))?;
            marg_data.insert(time, data);
        }
        let mut mapper = NfrMapper::new(&calibration, config);
        for data in marg_data.values_mut() {
            mapper.add_marg_data(data);
        }
        let mut image_timestamps: Vec<_> = mapper.img_data.keys().copied().collect();
        image_timestamps.sort_unstable();
        let mut result = Self {
            mapper,
            marg_data,
            gt_timestamps,
            gt_positions,
            image_timestamps,
            points: Vec::new(),
            point_ids: Vec::new(),
            edges: Vec::new(),
            roll_pitch_edges: Vec::new(),
            relative_edges: Vec::new(),
            num_opt_iter: 10,
            outlier_threshold: 3.,
        };
        result.compute_edge_vis();
        Ok(result)
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
            let direction =
                so3_product(&factor.r_w_i_meas, &so3_inverse(&pose.rotation)) * -Vector3::z();
            self.roll_pitch_edges.push(pose.translation.vector);
            self.roll_pitch_edges
                .push(pose.translation.vector + 0.1 * direction);
        }
        self.relative_edges.clear();
        for factor in &self.mapper.rel_pose_factors {
            self.relative_edges.push(
                self.mapper.base.frame_poses[&factor.t_i_ns]
                    .pose()
                    .translation
                    .vector,
            );
            self.relative_edges.push(
                self.mapper.base.frame_poses[&factor.t_j_ns]
                    .pose()
                    .translation
                    .vector,
            );
        }
    }
    pub fn optimize(&mut self) {
        self.mapper.optimize(self.num_opt_iter);
        self.refresh_points();
        self.compute_edge_vis();
    }
    /// # Errors
    /// Propagates image/keypoint processing errors.
    pub fn detect(&mut self) -> Result<()> {
        self.mapper
            .feature_corners
            .get_mut()
            .map_err(|_| binary::invalid("Poisoned feature corners"))?
            .clear();
        self.mapper
            .feature_matches
            .get_mut()
            .map_err(|_| binary::invalid("Poisoned feature matches"))?
            .clear();
        self.mapper.detect_keypoints()
    }
    /// # Errors
    /// Propagates descriptor matching errors.
    pub fn match_features(&mut self) -> Result<()> {
        self.mapper
            .feature_matches
            .get_mut()
            .map_err(|_| binary::invalid("Poisoned feature matches"))?
            .clear();
        self.mapper.match_stereo();
        self.mapper.match_all()
    }
    pub fn tracks(&mut self) {
        self.mapper.build_tracks();
        self.mapper.setup_opt();
        self.refresh_points();
        self.compute_edge_vis();
    }
    pub fn filter(&mut self) {
        self.mapper.base.filter_outliers(self.outlier_threshold, 4);
        self.refresh_points();
    }
    fn refresh_points(&mut self) {
        self.mapper
            .base
            .get_current_points(&mut self.points, &mut self.point_ids);
    }
    pub fn align(&mut self) -> f64 {
        let times: Vec<_> = self.mapper.base.frame_poses.keys().copied().collect();
        let positions: Vec<_> = self
            .mapper
            .base
            .frame_poses
            .values()
            .map(|p| p.pose().translation.vector)
            .collect();
        align_svd(
            &times,
            &positions,
            &self.gt_timestamps,
            &mut self.gt_positions,
        )
    }
    /// Executes the original headless detect/match/tracks/optimize/filter/optimize sequence.
    /// # Errors
    /// Propagates feature extraction and matching errors.
    pub fn run(&mut self) -> Result<()> {
        self.detect()?;
        self.match_features()?;
        self.tracks();
        self.optimize();
        self.filter();
        self.optimize();
        Ok(())
    }
    /// # Errors
    /// Propagates output I/O errors.
    pub fn save_trajectory(&self, mut output: impl Write, tum: bool) -> std::io::Result<()> {
        if tum {
            writeln!(output, "# timestamp tx ty tz qx qy qz qw")?;
        } else {
            writeln!(
                output,
                "#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z []"
            )?;
        }
        for (&time, state) in &self.mapper.base.frame_poses {
            let pose = state.pose();
            let position = pose.translation.vector;
            let q = pose.rotation.quaternion();
            if tum {
                writeln!(
                    output,
                    "{:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e} {:.18e}",
                    time as f64 * 1e-9,
                    position.x,
                    position.y,
                    position.z,
                    q.i,
                    q.j,
                    q.k,
                    q.w
                )?;
            } else {
                writeln!(
                    output,
                    "{time},{:.18e},{:.18e},{:.18e},{:.18e},{:.18e},{:.18e},{:.18e}",
                    position.x, position.y, position.z, q.w, q.i, q.j, q.k
                )?;
            }
        }
        Ok(())
    }
}

/// Native Cereal vector counts followed by timestamp and row-major Vector3 data.
/// # Errors
/// Reports truncated input or impossible vector counts.
pub fn read_ground_truth(input: &mut impl std::io::Read) -> Result<(Vec<i64>, Vec<Vector3<f64>>)> {
    let times = (0..binary::count(input)?)
        .map(|_| binary::i64(input))
        .collect::<Result<Vec<_>>>()?;
    let positions = (0..binary::count(input)?)
        .map(|_| binary::vector::<3>(input))
        .collect::<Result<Vec<_>>>()?;
    Ok((times, positions))
}
