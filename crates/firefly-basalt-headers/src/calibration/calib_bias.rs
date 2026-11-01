/*
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt-headers.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@file
@brief Definition of static IMU biases used for calibration
*/
//! Translation of `include/basalt/calibration/calib_bias.hpp` at headers
//! revision `aa441ba3e51050c47ba1902537792a2e4db7e43d`.
use crate::scalar::BasaltScalar;
use nalgebra::{Matrix3, SVector, Vector3};

/// Static accelerometer bias and lower-triangular scale/misalignment parameters.
#[derive(Clone, Debug)]
pub struct CalibAccelBias<S: BasaltScalar> {
    param: SVector<S, 9>,
}
impl<S: BasaltScalar> Default for CalibAccelBias<S> {
    fn default() -> Self {
        Self {
            param: SVector::zeros(),
        }
    }
}
impl<S: BasaltScalar> CalibAccelBias<S> {
    #[must_use]
    pub fn param(&self) -> &SVector<S, 9> {
        &self.param
    }
    pub fn param_mut(&mut self) -> &mut SVector<S, 9> {
        &mut self.param
    }
    pub fn apply_increment(&mut self, inc: &SVector<S, 9>) {
        self.param += inc;
    }
    /// Upstream test helper, with the Eigen uniform [-1,1] sample stream supplied by the caller.
    pub fn set_random_with(&mut self, mut sample: impl FnMut() -> S) {
        self.param = SVector::from_fn(|_, _| sample());
        for i in 0..3 {
            self.param[i] /= S::from_f64_lossy(10.0);
        }
        for i in 3..9 {
            self.param[i] /= S::from_f64_lossy(100.0);
        }
    }
    #[must_use]
    pub fn bias_and_scale(&self) -> (Vector3<S>, Matrix3<S>) {
        let mut scale = Matrix3::zeros();
        scale.set_column(0, &self.param.fixed_rows::<3>(3));
        scale[(1, 1)] = self.param[6];
        scale[(2, 1)] = self.param[7];
        scale[(2, 2)] = self.param[8];
        (self.param.fixed_rows::<3>(0).into_owned(), scale)
    }
    #[must_use]
    pub fn calibrated(&self, raw: &Vector3<S>) -> Vector3<S> {
        let (bias, scale) = self.bias_and_scale();
        *raw + scale * raw - bias
    }
    #[must_use]
    pub fn invert_calibration(&self, calibrated: &Vector3<S>) -> Vector3<S> {
        let (bias, scale) = self.bias_and_scale();
        inverse3(Matrix3::identity() + scale) * (*calibrated + bias)
    }
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> CalibAccelBias<T> {
        CalibAccelBias {
            param: self.param.map(|v| T::from_f64_lossy(v.as_f64())),
        }
    }
}

/// Static gyroscope bias and full 3x3 scale/misalignment parameters.
#[derive(Clone, Debug)]
pub struct CalibGyroBias<S: BasaltScalar> {
    param: SVector<S, 12>,
}
impl<S: BasaltScalar> Default for CalibGyroBias<S> {
    fn default() -> Self {
        Self {
            param: SVector::zeros(),
        }
    }
}
impl<S: BasaltScalar> CalibGyroBias<S> {
    #[must_use]
    pub fn param(&self) -> &SVector<S, 12> {
        &self.param
    }
    pub fn param_mut(&mut self) -> &mut SVector<S, 12> {
        &mut self.param
    }
    pub fn apply_increment(&mut self, inc: &SVector<S, 12>) {
        self.param += inc;
    }
    /// Upstream test helper, with the Eigen uniform [-1,1] sample stream supplied by the caller.
    pub fn set_random_with(&mut self, mut sample: impl FnMut() -> S) {
        self.param = SVector::from_fn(|_, _| sample());
        for i in 0..3 {
            self.param[i] /= S::from_f64_lossy(10.0);
        }
        for i in 3..12 {
            self.param[i] /= S::from_f64_lossy(100.0);
        }
    }
    #[must_use]
    pub fn bias_and_scale(&self) -> (Vector3<S>, Matrix3<S>) {
        (
            self.param.fixed_rows::<3>(0).into_owned(),
            Matrix3::from_column_slice(&self.param.as_slice()[3..]),
        )
    }
    #[must_use]
    pub fn calibrated(&self, raw: &Vector3<S>) -> Vector3<S> {
        let (bias, scale) = self.bias_and_scale();
        *raw + scale * raw - bias
    }
    #[must_use]
    pub fn invert_calibration(&self, calibrated: &Vector3<S>) -> Vector3<S> {
        let (bias, scale) = self.bias_and_scale();
        inverse3(Matrix3::identity() + scale) * (*calibrated + bias)
    }
    #[must_use]
    pub fn cast<T: BasaltScalar>(&self) -> CalibGyroBias<T> {
        CalibGyroBias {
            param: self.param.map(|v| T::from_f64_lossy(v.as_f64())),
        }
    }
}

// The original fixed-size inverse divides cofactors by the determinant; preserve
// nonfinite results for singular calibration rather than introducing a fallback.
fn inverse3<S: BasaltScalar>(matrix: Matrix3<S>) -> Matrix3<S> {
    let a = matrix.column(0);
    let b = matrix.column(1);
    let c = matrix.column(2);
    let bc = b.cross(&c);
    Matrix3::from_rows(&[
        bc.transpose(),
        c.cross(&a).transpose(),
        a.cross(&b).transpose(),
    ]) / a.dot(&bc)
}
