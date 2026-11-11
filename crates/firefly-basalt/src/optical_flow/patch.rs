/*
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

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
*/

//! Translation of `include/basalt/optical_flow/patch.h` at upstream
//! `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`.

use super::patterns::Pattern;

/// Scalar-specific reduction used by the pinned Eigen implementation.
pub trait PatchScalar: ImageScalar {
    fn product_sum3(a: Self, b: Self, c: Self) -> Self;
}
impl PatchScalar for f32 {
    fn product_sum3(a: Self, b: Self, c: Self) -> Self {
        a + (b + c)
    }
}
impl PatchScalar for f64 {
    fn product_sum3(a: Self, b: Self, c: Self) -> Self {
        (a + b) + c
    }
}
use firefly_calibration::image::image::Image;
use firefly_calibration::image::image::ImageScalar;
use firefly_calibration::image::image::Pixel;
use nalgebra::{Isometry2, Matrix3, Point2, SMatrix, SVector, Vector2, Vector3};
use std::marker::PhantomData;

/// Eigen's coefficient product uses a balanced scalar reduction for the fixed
/// 24/52-sample, strided patch rows. A sequential dot changes tracker updates.
pub(super) fn patch_increment<const N: usize>(
    inverse_jacobian: &SMatrix<f32, 3, N>,
    residual: &SVector<f32, N>,
) -> Vector3<f32> {
    const fn tree<const N: usize>() -> [(usize, usize); N] {
        let mut pairs = [(0, 0); N];
        append(&mut pairs, &mut 0, 0, N);
        pairs
    }
    const fn append<const N: usize>(
        pairs: &mut [(usize, usize); N],
        next: &mut usize,
        start: usize,
        length: usize,
    ) {
        if length > 1 {
            let half = length / 2;
            append(pairs, next, start, half);
            append(pairs, next, start + half, length - half);
            pairs[*next] = (start, start + half);
            *next += 1;
        }
    }
    // Compile the exact Eigen reduction tree once per pattern. Each node adds
    // all three pose coordinates, retaining the original floating-point order.
    assert!(N > 0, "optical-flow patterns contain samples");
    let pairs = &const { tree::<N>() };
    let mut values: [[f32; 3]; N] = std::array::from_fn(|column| {
        std::array::from_fn(|row| -inverse_jacobian[(row, column)] * residual[column])
    });
    for &(left, right) in &pairs[..N - 1] {
        let other = values[right];
        for (value, addend) in values[left].iter_mut().zip(other) {
            *value += addend;
        }
    }
    Vector3::from(values[0])
}

/// Image operations used by the upstream templated patch helpers.
pub trait PatchImage<S: ImageScalar> {
    fn in_bounds(&self, position: &Vector2<S>, border: S) -> bool;
    fn interp(&self, position: &Vector2<S>) -> S;
    fn interp_grad(&self, position: &Vector2<S>) -> Vector3<S>;
}
impl<S: ImageScalar, T: Pixel> PatchImage<S> for Image<'_, T> {
    fn in_bounds(&self, p: &Vector2<S>, border: S) -> bool {
        self.in_bounds(p, border)
    }
    fn interp(&self, p: &Vector2<S>) -> S {
        self.interp(p)
    }
    fn interp_grad(&self, p: &Vector2<S>) -> Vector3<S> {
        self.interp_grad(p)
    }
}

/// Mean-normalized inverse-compositional SE(2) optical-flow patch.
pub struct OpticalFlowPatch<S: PatchScalar, P: Pattern<S, N>, const N: usize> {
    pub pos: Vector2<S>,
    /// Negative entries denote samples outside the source image.
    pub data: SVector<S, N>,
    pub h_se2_inv_j_se2_t: SMatrix<S, 3, N>,
    pub mean: S,
    pub valid: bool,
    pattern: PhantomData<P>,
}

impl<S: PatchScalar, P: Pattern<S, N>, const N: usize> Default for OpticalFlowPatch<S, P, N> {
    fn default() -> Self {
        Self {
            pos: Vector2::zeros(),
            data: SVector::zeros(),
            h_se2_inv_j_se2_t: SMatrix::zeros(),
            mean: S::zero(),
            valid: false,
            pattern: PhantomData,
        }
    }
}

impl<S: PatchScalar, P: Pattern<S, N>, const N: usize> OpticalFlowPatch<S, P, N> {
    #[must_use]
    pub fn new(image: &Image<'_, u16>, pos: &Vector2<S>) -> Self {
        let mut patch = Self::default();
        patch.set_from_image(image, pos);
        patch
    }

    pub fn set_data(
        image: &impl PatchImage<S>,
        pos: &Vector2<S>,
        mean: &mut S,
        data: &mut SVector<S, N>,
        se2: Option<&Isometry2<S>>,
    ) {
        let pattern = P::pattern2();
        let mut count = S::zero();
        let mut sum = S::zero();
        for i in 0..N {
            let offset = pattern.column(i).into_owned();
            let p = *pos + se2.map_or(offset, |t| t.transform_point(&Point2::from(offset)).coords);
            if image.in_bounds(&p, S::one() + S::one()) {
                data[i] = image.interp(&p);
                sum += data[i];
                count += S::one();
            } else {
                data[i] = -S::one();
            }
        }
        *mean = sum / count;
        // Upstream normalizes invalid entries too; preserve NaN/Inf for empty/black patches.
        *data /= *mean;
    }

    pub fn set_data_jac_se2(
        image: &impl PatchImage<S>,
        pos: &Vector2<S>,
        mean: &mut S,
        data: &mut SVector<S, N>,
        jacobian: &mut SMatrix<S, N, 3>,
    ) {
        let pattern = P::pattern2();
        let mut count = S::zero();
        let mut sum = S::zero();
        let mut grad_sum = Vector3::<S>::zeros();
        for i in 0..N {
            let p = *pos + pattern.column(i);
            if image.in_bounds(&p, S::one() + S::one()) {
                let g = image.interp_grad(&p);
                data[i] = g[0];
                sum += g[0];
                jacobian[(i, 0)] = g[1];
                jacobian[(i, 1)] = g[2];
                jacobian[(i, 2)] = -g[1] * pattern[(1, i)] + g[2] * pattern[(0, i)];
                grad_sum += jacobian.row(i).transpose();
                count += S::one();
            } else {
                data[i] = -S::one();
            }
        }
        *mean = sum / count;
        let mean_inv = count / sum;
        for i in 0..N {
            if data[i] >= S::zero() {
                for j in 0..3 {
                    jacobian[(i, j)] -= grad_sum[j] * data[i] / sum;
                }
                data[i] *= mean_inv;
            } else {
                jacobian.row_mut(i).fill(S::zero());
            }
        }
        *jacobian *= mean_inv;
    }

    pub fn set_from_image(&mut self, image: &Image<'_, u16>, pos: &Vector2<S>) {
        self.pos = *pos;
        let mut jacobian = SMatrix::<S, N, 3>::zeros();
        Self::set_data_jac_se2(image, pos, &mut self.mean, &mut self.data, &mut jacobian);
        let h = jacobian.transpose() * jacobian;
        let h_inv = ldlt_inverse(h);
        // Eigen's packetized double and unrolled float products have different
        // reduction orders; nearly singular patches amplify reassociation.
        self.h_se2_inv_j_se2_t = SMatrix::from_fn(|r, c| {
            S::product_sum3(
                h_inv[(r, 0)] * jacobian[(c, 0)],
                h_inv[(r, 1)] * jacobian[(c, 1)],
                h_inv[(r, 2)] * jacobian[(c, 2)],
            )
        });
        self.valid = self.mean > S::default_epsilon()
            && self
                .h_se2_inv_j_se2_t
                .iter()
                .all(nalgebra::ComplexField::is_finite)
            && self.data.iter().all(nalgebra::ComplexField::is_finite);
    }

    #[must_use]
    pub fn residual(
        &self,
        image: &Image<'_, u16>,
        transformed_pattern: &SMatrix<S, 2, N>,
        residual: &mut SVector<S, N>,
    ) -> bool {
        let mut sum = S::zero();
        let mut count = S::zero();
        for i in 0..N {
            let p = transformed_pattern.column(i).into_owned();
            if image.in_bounds(&p, S::one() + S::one()) {
                residual[i] = image.interp(&p);
                sum += residual[i];
                count += S::one();
            } else {
                residual[i] = -S::one();
            }
        }
        if sum < S::default_epsilon() {
            residual.fill(S::zero());
            return false;
        }
        let mut num_residuals = 0;
        for i in 0..N {
            if residual[i] >= S::zero() && self.data[i] >= S::zero() {
                residual[i] = count * residual[i] / sum - self.data[i];
                num_residuals += 1;
            } else {
                residual[i] = S::zero();
            }
        }
        num_residuals > N / 2
    }
}

// Fixed 3x3 real specialization of Eigen's pivoted LDLT and solve.
// Eigen/src/Cholesky/LDLT.h, Eigen 3.4.0:
// Copyright (C) 2008-2011 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2009 Keir Mierle <mierle@gmail.com>
// Copyright (C) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2011 Timothy E. Holy <tim.holy@gmail.com >
// This Source Code Form is subject to the terms of the Mozilla
// Public License, v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
pub(super) fn ldlt_inverse<S: ImageScalar>(mut a: Matrix3<S>) -> Matrix3<S> {
    let mut swaps = [0, 1, 2];
    for k in 0..3 {
        let mut pivot = k;
        for j in k + 1..3 {
            if a[(j, j)].abs() > a[(pivot, pivot)].abs() {
                pivot = j;
            }
        }
        swaps[k] = pivot;
        if pivot != k {
            for j in 0..k {
                a.swap((k, j), (pivot, j));
            }
            for i in pivot + 1..3 {
                a.swap((i, k), (i, pivot));
            }
            a.swap((k, k), (pivot, pivot));
            for i in k + 1..pivot {
                a.swap((i, k), (pivot, i));
            }
        }
        let mut temp = Vector3::<S>::zeros();
        for j in 0..k {
            temp[j] = a[(j, j)] * a[(k, j)];
        }
        let mut diagonal_update = S::zero();
        for j in 0..k {
            diagonal_update += a[(k, j)] * temp[j];
        }
        a[(k, k)] -= diagonal_update;
        for i in k + 1..3 {
            let mut update = S::zero();
            for j in 0..k {
                update += a[(i, j)] * temp[j];
            }
            a[(i, k)] -= update;
        }
        let d = a[(k, k)];
        let valid = d.abs() > S::zero();
        if k == 0 && !valid {
            swaps = [0, 1, 2];
            break;
        }
        if valid {
            for i in k + 1..3 {
                a[(i, k)] /= d;
            }
        }
    }
    let mut result = Matrix3::<S>::identity();
    for (k, &pivot) in swaps.iter().enumerate() {
        result.swap_rows(k, pivot);
    }
    for i in 0..3 {
        for col in 0..3 {
            let mut update = S::zero();
            for j in 0..i {
                update += a[(i, j)] * result[(j, col)];
            }
            result[(i, col)] -= update;
        }
    }
    for i in 0..3 {
        let d = a[(i, i)];
        if d.abs() > S::min_positive_value() {
            result.row_mut(i).apply(|v| *v /= d);
        } else {
            result.row_mut(i).fill(S::zero());
        }
    }
    for i in (0..3).rev() {
        for col in 0..3 {
            let mut update = S::zero();
            for j in i + 1..3 {
                update += a[(j, i)] * result[(j, col)];
            }
            result[(i, col)] -= update;
        }
    }
    for (k, &pivot) in swaps.iter().enumerate().rev() {
        result.swap_rows(k, pivot);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check_increment_order<const N: usize>() {
        fn reference(matrix: &[f32], residual: &[f32], row: usize) -> f32 {
            if residual.len() == 1 {
                -matrix[row] * residual[0]
            } else {
                let half = residual.len() / 2;
                reference(&matrix[..3 * half], &residual[..half], row)
                    + reference(&matrix[3 * half..], &residual[half..], row)
            }
        }
        let mut bits = 17_u32;
        let mut scalar = || {
            bits = bits.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            f32::from_bits((bits & 0x807f_ffff) | ((100 + bits % 50) << 23))
        };
        for _ in 0..1024 {
            let matrix = SMatrix::<f32, 3, N>::from_fn(|_, _| scalar());
            let residual = SVector::<f32, N>::from_fn(|_, _| scalar());
            let actual = patch_increment(&matrix, &residual);
            for row in 0..3 {
                assert_eq!(
                    actual[row].to_bits(),
                    reference(matrix.as_slice(), residual.as_slice(), row).to_bits()
                );
            }
        }
    }

    #[test]
    fn compiled_reduction_tree_preserves_eigen_order_bit_for_bit() {
        check_increment_order::<1>();
        check_increment_order::<3>();
        check_increment_order::<24>();
        check_increment_order::<52>();
    }
    use crate::optical_flow::patterns::Pattern52;

    // Port of test/src/test_patch.cpp: analytic image avoids testing a numerical
    // image gradient against the derivative of a different interpolant.
    struct SmoothFunction;
    impl PatchImage<f64> for SmoothFunction {
        fn in_bounds(&self, _: &Vector2<f64>, _: f64) -> bool {
            true
        }
        fn interp(&self, p: &Vector2<f64>) -> f64 {
            (p.x / 100.0 + p.y / 20.0).sin()
        }
        fn interp_grad(&self, p: &Vector2<f64>) -> Vector3<f64> {
            let a = p.x / 100.0 + p.y / 20.0;
            Vector3::new(a.sin(), a.cos() / 100.0, a.cos() / 20.0)
        }
    }

    #[test]
    fn patch_se2_jacobian_matches_upstream_smooth_function() {
        type Patch = OpticalFlowPatch<f64, Pattern52, 52>;
        let pos = Vector2::new(231.4, 123.34345);
        let mut data = SVector::zeros();
        let mut jacobian = SMatrix::zeros();
        let mut mean = 0.0;
        Patch::set_data_jac_se2(&SmoothFunction, &pos, &mut mean, &mut data, &mut jacobian);
        let mut data_only = SVector::zeros();
        let mut mean_only = 0.0;
        Patch::set_data(&SmoothFunction, &pos, &mut mean_only, &mut data_only, None);
        assert!((mean - mean_only).abs() < 1e-8);
        assert!((data - data_only).norm() < 1e-8);
        let epsilon = 1e-6;
        for axis in 0..3 {
            let mut plus = SVector::zeros();
            let mut minus = SVector::zeros();
            // Single-axis SE(2) exponentials: pure translations or pure rotation.
            let mut translation = Vector2::zeros();
            let angle = if axis == 2 {
                epsilon
            } else {
                translation[axis] = epsilon;
                0.0
            };
            let transform = Isometry2::new(translation, angle);
            Patch::set_data(
                &SmoothFunction,
                &pos,
                &mut mean_only,
                &mut plus,
                Some(&transform),
            );
            Patch::set_data(
                &SmoothFunction,
                &pos,
                &mut mean_only,
                &mut minus,
                Some(&transform.inverse()),
            );
            let numeric = (plus - minus) / (2.0 * epsilon);
            assert!((numeric - jacobian.column(axis)).amax() < 1e-8);
        }
    }

    #[test]
    fn default_black_and_constant_patch_semantics() {
        type Patch = OpticalFlowPatch<f64, Pattern52, 52>;
        let default = Patch::default();
        assert!(!default.valid);
        assert_eq!(default.mean, 0.0);
        let pos = Vector2::new(16.0, 16.0);
        let black = vec![0_u16; 32 * 32];
        let black_image = Image::new(&black, 32, 32, 32);
        let black_patch = Patch::new(&black_image, &pos);
        assert!(!black_patch.valid);
        let mut residual = SVector::repeat(1.0);
        let positions =
            <Pattern52 as Pattern<f64, 52>>::pattern2().map_with_location(|row, _, v| v + pos[row]);
        assert!(!black_patch.residual(&black_image, &positions, &mut residual));
        assert_eq!(residual, SVector::<f64, 52>::zeros());
        let constant = vec![10_u16; 32 * 32];
        let constant_image = Image::new(&constant, 32, 32, 32);
        let constant_patch = Patch::new(&constant_image, &pos);
        // Eigen LDLT's zero-diagonal pseudoinverse accepts the zero Hessian.
        assert!(constant_patch.valid);
        assert_eq!(
            constant_patch.h_se2_inv_j_se2_t,
            SMatrix::<f64, 3, 52>::zeros()
        );
        assert!(constant_patch.residual(&constant_image, &positions, &mut residual));
        assert_eq!(residual, SVector::<f64, 52>::zeros());
    }
}
