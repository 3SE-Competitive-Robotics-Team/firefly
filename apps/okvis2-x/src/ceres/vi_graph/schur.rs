/*
 * OKVIS2-X, Copyright (c) ETH Zurich, Imperial College London and TUM.
 * Ceres Solver, Copyright 2023 Google Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of Google Inc. nor the names of its contributors may be
 * used to endorse or promote products derived from this software without
 * specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
//! Normal equations and Schur elimination for linearized graph factors.
//! Input Jacobians already include manifold, robust-loss and column scaling.
//! Regularization is the additive diagonal of the normal equations.
use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

pub struct Row {
    pub residual: DVector<f64>,
    pub blocks: Vec<(usize, DMatrix<f64>)>,
}

/// Borrowed linear system; offsets refer to tangent-space columns.
/// Eliminated point offsets identify disjoint, contiguous three-column blocks.
pub struct SchurSystem<'a> {
    pub rows: &'a [Row],
    pub gradient: &'a DVector<f64>,
    pub eliminated_points: &'a [usize],
}

impl SchurSystem<'_> {
    /// Assemble only retained normal equations and connected landmark blocks.
    /// Factors connecting distinct landmarks use the complete dense system.
    pub fn block(&self, regularization: &DVector<f64>) -> Option<DVector<f64>> {
        let count = self.gradient.len();
        let mut point_for_column = vec![None; count];
        for (point, &offset) in self.eliminated_points.iter().enumerate() {
            for local in 0..3 {
                point_for_column[offset + local] = Some((point, local));
            }
        }
        let retained: Vec<_> = (0..count)
            .filter(|&column| point_for_column[column].is_none())
            .collect();
        let mut retained_index = vec![0; count];
        for (index, &column) in retained.iter().enumerate() {
            retained_index[column] = index;
        }
        let Some(mut points) = self.point_blocks(&point_for_column, &retained_index) else {
            return self.dense(regularization);
        };
        let mut reduced = DMatrix::<f64>::zeros(retained.len(), retained.len());
        for row in self.rows {
            for (oa, left) in &row.blocks {
                for (ob, right) in &row.blocks {
                    for ca in 0..left.ncols() {
                        for cb in 0..right.ncols() {
                            let (a, b) = (oa + ca, ob + cb);
                            if a < b {
                                continue;
                            }
                            let value = left.column(ca).dot(&right.column(cb));
                            match (point_for_column[a], point_for_column[b]) {
                                (None, None) => {
                                    reduced[(retained_index[a], retained_index[b])] += value;
                                }
                                (Some((point, i)), Some((_, j))) => {
                                    points[point].hessian[(i, j)] += value;
                                }
                                (Some((point, i)), None) | (None, Some((point, i))) => {
                                    let retained_column =
                                        if point_for_column[a].is_none() { a } else { b };
                                    let point = &mut points[point];
                                    let j = point
                                        .neighbors
                                        .binary_search(&retained_index[retained_column])
                                        .unwrap();
                                    point.cross[(i, j)] += value;
                                }
                            }
                        }
                    }
                }
            }
        }
        let mut rhs =
            DVector::from_iterator(retained.len(), retained.iter().map(|&c| -self.gradient[c]));
        for (index, &column) in retained.iter().enumerate() {
            reduced[(index, index)] += regularization[column];
        }
        let mut solved = Vec::with_capacity(points.len());
        for (point, &offset) in points.iter_mut().zip(self.eliminated_points) {
            for i in 0..3 {
                point.hessian[(i, i)] += regularization[offset + i];
            }
            if point.hessian.iter().any(|value| !value.is_finite()) {
                return None;
            }
            let factor = point.hessian.cholesky()?;
            let solved_cross = factor.solve(&point.cross);
            let solved_rhs = factor.solve(&Vector3::from_fn(|i, _| -self.gradient[offset + i]));
            for (i, &row) in point.neighbors.iter().enumerate() {
                rhs[row] -= point.cross.column(i).dot(&solved_rhs);
                for (j, &column) in point.neighbors[..=i].iter().enumerate() {
                    reduced[(row, column)] -= point.cross.column(i).dot(&solved_cross.column(j));
                }
            }
            solved.push((solved_cross, solved_rhs));
        }
        if reduced.diagonal().iter().any(|value| !value.is_finite()) {
            return None;
        }
        let solution = if retained.is_empty() {
            DVector::zeros(0)
        } else {
            reduced.cholesky()?.solve(&rhs)
        };
        let mut output = DVector::zeros(count);
        for (index, &column) in retained.iter().enumerate() {
            output[column] = solution[index];
        }
        for ((point, (cross, mut rhs)), &offset) in
            points.iter().zip(solved).zip(self.eliminated_points)
        {
            for (i, &column) in point.neighbors.iter().enumerate() {
                rhs -= cross.column(i) * solution[column];
            }
            output.rows_mut(offset, 3).copy_from(&rhs);
        }
        output
            .iter()
            .all(|value| value.is_finite())
            .then_some(output)
    }

    fn point_blocks(
        &self,
        point_for_column: &[Option<(usize, usize)>],
        retained_index: &[usize],
    ) -> Option<Vec<PointBlock>> {
        let mut neighbors = vec![Vec::new(); self.eliminated_points.len()];
        for row in self.rows {
            let mut row_point = None;
            for (offset, block) in &row.blocks {
                for &(point, _) in point_for_column[*offset..offset + block.ncols()]
                    .iter()
                    .flatten()
                {
                    if row_point.is_some_and(|previous| previous != point) {
                        return None;
                    }
                    row_point = Some(point);
                }
            }
            if let Some(point) = row_point {
                for (offset, block) in &row.blocks {
                    for column in *offset..offset + block.ncols() {
                        if point_for_column[column].is_none() {
                            neighbors[point].push(retained_index[column]);
                        }
                    }
                }
            }
        }
        let points = neighbors
            .into_iter()
            .map(|mut neighbors| {
                neighbors.sort_unstable();
                neighbors.dedup();
                PointBlock {
                    hessian: Matrix3::zeros(),
                    cross: DMatrix::zeros(3, neighbors.len()),
                    neighbors,
                }
            })
            .collect();
        Some(points)
    }

    pub fn dense(&self, regularization: &DVector<f64>) -> Option<DVector<f64>> {
        self.solve_dense_normal(&self.lower_normal(regularization))
    }
    /// Assemble the lower triangle in dense storage for complete-system solvers.
    pub fn lower_normal(&self, regularization: &DVector<f64>) -> DMatrix<f64> {
        let count = self.gradient.len();
        let mut lower = DMatrix::<f64>::zeros(count, count);
        for row in self.rows {
            for (oa, left_block) in &row.blocks {
                for (ob, right_block) in &row.blocks {
                    for ca in 0..left_block.ncols() {
                        for cb in 0..right_block.ncols() {
                            let (index, column) = (oa + ca, ob + cb);
                            if index >= column {
                                lower[(index, column)] +=
                                    left_block.column(ca).dot(&right_block.column(cb));
                            }
                        }
                    }
                }
            }
        }
        for index in 0..count {
            lower[(index, index)] += regularization[index];
        }
        lower
    }
    /// Eliminate independent 3D landmark blocks, solve the reduced normal
    /// equations by Cholesky, then back-substitute the landmark increments.
    pub fn solve_dense_normal(&self, lower: &DMatrix<f64>) -> Option<DVector<f64>> {
        // Cholesky may turn infinite pivots into a finite zero step. A finite
        // output alone does not establish a valid normal system.
        if lower.diagonal().iter().any(|value| !value.is_finite()) {
            return None;
        }
        let count = self.gradient.len();
        let mut point_for_column = vec![None; count];
        for (point, &offset) in self.eliminated_points.iter().enumerate() {
            point_for_column[offset..offset + 3].fill(Some(point));
        }
        // Coupled landmarks cannot form an independent elimination set. Solve
        // their complete normal equations without dropping cross terms.
        if (0..count).any(|index| {
            (0..=index).any(|column| {
                lower[(index, column)] != 0.0
                    && matches!((point_for_column[index], point_for_column[column]),
                    (Some(a), Some(b)) if a != b)
            })
        }) {
            let matrix = lower.clone();
            return matrix
                .cholesky()
                .map(|factorization| factorization.solve(&(-self.gradient)))
                .filter(|velocity| velocity.iter().all(|value_value| value_value.is_finite()));
        }
        let retained: Vec<_> = (0..count)
            .filter(|index| point_for_column[*index].is_none())
            .collect();
        let get = |index: usize, column: usize| lower[(index.max(column), index.min(column))];
        let mut reduced = DMatrix::from_fn(retained.len(), retained.len(), |index, column| {
            get(retained[index], retained[column])
        });
        let mut rhs = DVector::from_iterator(
            retained.len(),
            retained.iter().map(|&index| -self.gradient[index]),
        );
        let mut points = Vec::new();
        for &offset in self.eliminated_points {
            let point_hessian =
                Matrix3::from_fn(|index, column| get(offset + index, offset + column));
            let factor = point_hessian.cholesky()?;
            let cross = DMatrix::from_fn(3, retained.len(), |index, column| {
                get(offset + index, retained[column])
            });
            let point_rhs = Vector3::from_fn(|index, _| -self.gradient[offset + index]);
            let solved_cross = factor.solve(&cross);
            let solved_rhs = factor.solve(&point_rhs);
            reduced.gemm_tr(-1.0, &cross, &solved_cross, 1.0);
            rhs.gemv_tr(-1.0, &cross, &solved_rhs, 1.0);
            points.push((offset, solved_cross, solved_rhs));
        }
        let solution = if retained.is_empty() {
            DVector::zeros(0)
        } else {
            reduced.cholesky()?.solve(&rhs)
        };
        let mut output = DVector::zeros(count);
        for (index, &original) in retained.iter().enumerate() {
            output[original] = solution[index];
        }
        for (offset, cross, rhs) in points {
            output
                .rows_mut(offset, 3)
                .copy_from(&(rhs - cross * &solution));
        }
        output
            .iter()
            .all(|value_value| value_value.is_finite())
            .then_some(output)
    }
}

struct PointBlock {
    hessian: Matrix3<f64>,
    neighbors: Vec<usize>,
    cross: DMatrix<f64>,
}
