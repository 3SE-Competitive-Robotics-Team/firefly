// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// See the crate LICENSE-BSD-3-Clause.
//! Translation of `include/basalt/utils/imu_types.h`, original Basalt
//! `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

use crate::imu::imu_types::PoseState;
use crate::imu::imu_types::PoseVelBiasState;
use crate::imu::preintegration::IntegratedImuMeasurement;
use crate::optical_flow::OpticalFlowResult;
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector, Isometry3, Matrix2, SMatrix, SVector, UnitQuaternion, Vector3};
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
};

pub const GRAVITY: [f64; 3] = [0.0, 0.0, -9.81];
pub const GRAVITY_DIRECTION: [f64; 3] = [0.0, 0.0, -1.0];
#[path = "imu_types_archive.rs"]
mod archive;

/// IMU state with a fixed linearization point and accumulated local increment.
#[derive(Clone, Debug)]
pub struct PoseVelBiasStateWithLin<S: BasaltScalar> {
    linearized: bool,
    delta: SVector<S, 15>,
    state_linearized: PoseVelBiasState<S>,
    state_current: PoseVelBiasState<S>,
    backup: Option<(SVector<S, 15>, PoseVelBiasState<S>, PoseVelBiasState<S>)>,
}

impl<S: BasaltScalar> Default for PoseVelBiasStateWithLin<S> {
    fn default() -> Self {
        Self::new(
            PoseVelBiasState::new(
                0,
                Isometry3::identity(),
                Vector3::zeros(),
                Vector3::zeros(),
                Vector3::zeros(),
            ),
            false,
        )
    }
}

impl<S: BasaltScalar> PoseVelBiasStateWithLin<S> {
    #[must_use]
    pub fn new(state: PoseVelBiasState<S>, linearized: bool) -> Self {
        Self {
            linearized,
            delta: SVector::zeros(),
            state_current: state.clone(),
            state_linearized: state,
            backup: None,
        }
    }
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> PoseVelBiasStateWithLin<T> {
        PoseVelBiasStateWithLin {
            linearized: self.linearized,
            delta: self.delta.map(|value| T::from_f64_lossy(value.as_f64())),
            state_linearized: self.state_linearized.cast(),
            state_current: self.state_current.cast(),
            backup: None,
        }
    }
    pub fn set_lin_false(&mut self) {
        self.linearized = false;
        self.delta.fill(S::zero());
    }
    /// # Panics
    /// Requires a zero accumulated increment.
    pub fn set_lin_true(&mut self) {
        self.linearized = true;
        assert!(self.delta.iter().all(|value| *value == S::zero()));
        self.state_current = self.state_linearized.clone();
    }
    pub fn apply_inc(&mut self, increment: &SVector<S, 15>) {
        if self.linearized {
            self.delta += increment;
            self.state_current = self.state_linearized.clone();
            self.state_current.apply_inc(&self.delta);
        } else {
            self.state_linearized.apply_inc(increment);
        }
    }
    #[must_use]
    pub fn state(&self) -> &PoseVelBiasState<S> {
        if self.linearized {
            &self.state_current
        } else {
            &self.state_linearized
        }
    }
    #[must_use]
    pub fn state_lin(&self) -> &PoseVelBiasState<S> {
        &self.state_linearized
    }
    #[must_use]
    pub fn is_linearized(&self) -> bool {
        self.linearized
    }
    #[must_use]
    pub fn delta(&self) -> &SVector<S, 15> {
        &self.delta
    }
    #[must_use]
    pub fn t_ns(&self) -> i64 {
        self.state_linearized.state.pose.t_ns
    }
    pub fn backup(&mut self) {
        self.backup = Some((
            self.delta,
            self.state_linearized.clone(),
            self.state_current.clone(),
        ));
    }
    /// Restores numeric state; the linearization flag is not part of the backup.
    /// # Panics
    /// Requires a preceding call to [`Self::backup`].
    pub fn restore(&mut self) {
        let (delta, linearized, current) = self
            .backup
            .as_ref()
            .expect("state must be backed up before restore");
        self.delta = *delta;
        self.state_linearized = linearized.clone();
        self.state_current = current.clone();
    }
}

/// Pose-only linearization state with the same decoupled increment convention.
#[derive(Clone, Debug)]
pub struct PoseStateWithLin<S: BasaltScalar> {
    linearized: bool,
    delta: SVector<S, 6>,
    pose_linearized: PoseState<S>,
    pose_current: Isometry3<S>,
    backup: Option<(SVector<S, 6>, PoseState<S>, Isometry3<S>)>,
}
impl<S: BasaltScalar> Default for PoseStateWithLin<S> {
    fn default() -> Self {
        Self::new(0, Isometry3::identity(), false)
    }
}
impl<S: BasaltScalar> PoseStateWithLin<S> {
    #[must_use]
    pub fn new(t_ns: i64, pose: Isometry3<S>, linearized: bool) -> Self {
        Self {
            linearized,
            delta: SVector::zeros(),
            pose_linearized: PoseState::new(t_ns, pose),
            pose_current: pose,
            backup: None,
        }
    }
    #[must_use]
    pub fn from_imu_state(state: &PoseVelBiasStateWithLin<S>) -> Self {
        let mut result = Self::new(
            state.t_ns(),
            state.state_linearized.state.pose.t_w_i,
            state.linearized,
        );
        result.delta = state.delta.fixed_rows::<6>(0).into_owned();
        PoseState::inc_pose(&result.delta, &mut result.pose_current);
        result
    }
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> PoseStateWithLin<T> {
        PoseStateWithLin {
            linearized: self.linearized,
            delta: self.delta.map(|value| T::from_f64_lossy(value.as_f64())),
            pose_linearized: self.pose_linearized.cast(),
            pose_current: PoseState::new(0, self.pose_current).cast::<T>().t_w_i,
            backup: None,
        }
    }
    /// # Panics
    /// Requires a zero accumulated increment.
    pub fn set_lin_true(&mut self) {
        self.linearized = true;
        assert!(self.delta.iter().all(|value| *value == S::zero()));
        self.pose_current = self.pose_linearized.t_w_i;
    }
    pub fn apply_inc(&mut self, increment: &SVector<S, 6>) {
        if self.linearized {
            self.delta += increment;
            self.pose_current = self.pose_linearized.t_w_i;
            PoseState::inc_pose(&self.delta, &mut self.pose_current);
        } else {
            self.pose_linearized.apply_inc(increment);
        }
    }
    #[must_use]
    pub fn pose(&self) -> &Isometry3<S> {
        if self.linearized {
            &self.pose_current
        } else {
            &self.pose_linearized.t_w_i
        }
    }
    #[must_use]
    pub fn pose_lin(&self) -> &Isometry3<S> {
        &self.pose_linearized.t_w_i
    }
    #[must_use]
    pub fn is_linearized(&self) -> bool {
        self.linearized
    }
    #[must_use]
    pub fn delta(&self) -> &SVector<S, 6> {
        &self.delta
    }
    #[must_use]
    pub fn t_ns(&self) -> i64 {
        self.pose_linearized.t_ns
    }
    pub fn backup(&mut self) {
        self.backup = Some((self.delta, self.pose_linearized.clone(), self.pose_current));
    }
    /// # Panics
    /// Requires a preceding call to [`Self::backup`].
    pub fn restore(&mut self) {
        let (delta, linearized, current) = self
            .backup
            .as_ref()
            .expect("pose must be backed up before restore");
        self.delta = *delta;
        self.pose_linearized = linearized.clone();
        self.pose_current = *current;
    }
}

#[derive(Clone, Debug, Default)]
pub struct AbsOrderMap {
    pub abs_order_map: BTreeMap<i64, (i32, i32)>,
    pub items: usize,
    pub total_size: usize,
}
impl AbsOrderMap {
    pub fn print_order(&self) {
        for (timestamp, (offset, size)) in &self.abs_order_map {
            log::info!("{timestamp} ({offset},{size})");
        }
    }
}

pub struct ImuLinData<'a, S: BasaltScalar> {
    pub g: &'a Vector3<S>,
    pub gyro_bias_weight_sqrt: &'a Vector3<S>,
    pub accel_bias_weight_sqrt: &'a Vector3<S>,
    pub imu_meas: BTreeMap<i64, &'a IntegratedImuMeasurement<S>>,
}

#[derive(Clone, Debug)]
pub struct MargLinData<S: BasaltScalar> {
    pub is_sqrt: bool,
    pub order: AbsOrderMap,
    pub h: DMatrix<S>,
    pub b: DVector<S>,
}
impl<S: BasaltScalar> Default for MargLinData<S> {
    fn default() -> Self {
        Self {
            is_sqrt: true,
            order: AbsOrderMap::default(),
            h: DMatrix::zeros(0, 0),
            b: DVector::zeros(0),
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct MargData {
    pub aom: AbsOrderMap,
    pub abs_h: DMatrix<f64>,
    pub abs_b: DVector<f64>,
    pub frame_states: BTreeMap<i64, PoseVelBiasStateWithLin<f64>>,
    pub frame_poses: BTreeMap<i64, PoseStateWithLin<f64>>,
    pub kfs_all: BTreeSet<i64>,
    pub kfs_to_marg: BTreeSet<i64>,
    pub use_imu: bool,
    pub opt_flow_res: Vec<Arc<OpticalFlowResult>>,
}

#[derive(Clone, Debug)]
pub struct RelPoseFactor {
    pub t_i_ns: i64,
    pub t_j_ns: i64,
    pub t_i_j: Isometry3<f64>,
    pub cov_inv: SMatrix<f64, 6, 6>,
}
#[derive(Clone, Debug)]
pub struct RollPitchFactor {
    pub t_ns: i64,
    pub r_w_i_meas: UnitQuaternion<f64>,
    pub cov_inv: Matrix2<f64>,
}
