// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Common types from original Basalt `linearization/landmark_block.hpp`.
use firefly_basalt_headers::scalar::BasaltScalar;
use nalgebra::{Matrix4, SMatrix};

#[derive(Clone, Debug)]
pub struct RelPoseLin<S: BasaltScalar> {
    pub t_t_h: Matrix4<S>,
    pub d_rel_d_h: SMatrix<S, 6, 6>,
    pub d_rel_d_t: SMatrix<S, 6, 6>,
}
#[derive(Clone, Copy, Debug)]
pub struct Options<S: BasaltScalar> {
    pub use_householder: bool,
    pub use_valid_projections_only: bool,
    pub huber_parameter: S,
    pub obs_std_dev: S,
    pub jacobi_scaling_eps: S,
}
impl<S: BasaltScalar> Default for Options<S> {
    fn default() -> Self {
        Self {
            use_householder: true,
            use_valid_projections_only: true,
            huber_parameter: S::zero(),
            obs_std_dev: S::one(),
            jacobi_scaling_eps: S::from_f64_lossy(1e-6),
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum State {
    Uninitialized,
    Allocated,
    NumericalFailure,
    Linearized,
    Marginalized,
}
