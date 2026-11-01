// Copyright (C) 2008-2012 Gael Guennebaud <gael.guennebaud@inria.fr>.
// Derived from LDL, Copyright (c) 2005 Timothy A. Davis.
// SPDX-License-Identifier: MPL-2.0
//! Eigen 5.0.1 simplicial LDLT numeric elimination with the original AMD order.
//! Sparse columns grow to their symbolic fill pattern; no dense system is formed.
use super::amd::minimum_degree_ordering;
use firefly_basalt_headers::scalar::BasaltScalar;
use nalgebra::DVector;
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
pub struct SparseMatrix<S: BasaltScalar> {
    pub size: usize,
    pub columns: Vec<BTreeMap<usize, S>>,
}
impl<S: BasaltScalar> SparseMatrix<S> {
    #[must_use]
    pub fn zeros(size: usize) -> Self {
        Self {
            size,
            columns: vec![BTreeMap::new(); size],
        }
    }
    /// Add a triplet, retaining explicit structural zeros and merging duplicates.
    /// # Panics
    /// Requires in-bounds matrix coordinates.
    pub fn add(&mut self, row: usize, col: usize, value: S) {
        assert!(row < self.size && col < self.size);
        *self.columns[col].entry(row).or_insert(S::zero()) += value;
    }
    #[must_use]
    pub fn diagonal(&self) -> DVector<S> {
        DVector::from_fn(self.size, |i, _| {
            self.columns[i].get(&i).copied().unwrap_or(S::zero())
        })
    }
    /// # Panics
    /// Requires a full-size input vector.
    #[must_use]
    pub fn multiply(&self, vector: &DVector<S>) -> DVector<S> {
        assert_eq!(vector.len(), self.size);
        let mut result = DVector::zeros(self.size);
        for (col, entries) in self.columns.iter().enumerate() {
            for (&row, &value) in entries {
                result[row] += value * vector[col];
            }
        }
        result
    }
}

pub struct SparseLdlt<S: BasaltScalar> {
    columns: Vec<Vec<(usize, S)>>,
    diagonal: DVector<S>,
    permutation: Vec<usize>,
    valid: bool,
}
impl<S: BasaltScalar> SparseLdlt<S> {
    /// Uses only the input's lower triangle, as Eigen's default specialization.
    /// # Panics
    /// Requires a valid square sparse pattern with representable indices.
    #[must_use]
    pub fn new(matrix: &SparseMatrix<S>) -> Self {
        let size = matrix.size;
        let mut symmetric = vec![BTreeMap::new(); size];
        for (col, entries) in matrix.columns.iter().enumerate() {
            for (&row, &value) in entries.range(col..) {
                symmetric[col].insert(row, value);
                symmetric[row].insert(col, value);
            }
        }
        let mut outer = Vec::with_capacity(size + 1);
        let mut inner = Vec::new();
        outer.push(0);
        for column in &symmetric {
            inner.extend(column.keys().copied());
            outer.push(inner.len());
        }
        let permutation = minimum_degree_ordering(&outer, &inner);
        let mut inverse = vec![0; size];
        for (new, &old) in permutation.iter().enumerate() {
            inverse[old] = new;
        }
        let mut upper = vec![BTreeMap::new(); size];
        for (col, entries) in matrix.columns.iter().enumerate() {
            for (&row, &value) in entries.range(col..) {
                let a = inverse[row];
                let b = inverse[col];
                upper[a.max(b)].insert(a.min(b), value);
            }
        }
        // Same elimination forest as Eigen 5's symbolic disjoint-set pass.
        let mut parent = vec![None; size];
        let mut ancestor: Vec<_> = (0..size).collect();
        for (column, entries) in upper.iter().enumerate().skip(1) {
            for (&row, _) in entries.range(..column) {
                let mut root = row;
                while ancestor[root] != root {
                    root = ancestor[ancestor[root]];
                }
                if root != column {
                    parent[root] = Some(column);
                }
                let mut node = row;
                while ancestor[node] != column {
                    let next = ancestor[node];
                    ancestor[node] = column;
                    node = next;
                }
            }
        }
        let mut columns: Vec<Vec<(usize, S)>> = vec![Vec::new(); size];
        let mut diagonal = DVector::<S>::zeros(size);
        let mut workspace = DVector::<S>::zeros(size);
        let mut tags = vec![usize::MAX; size];
        let mut valid = true;
        for column in 0..size {
            workspace[column] = S::zero();
            tags[column] = column;
            let mut pattern = Vec::new();
            for (&row, &value) in upper[column].range(..=column) {
                workspace[row] += value;
                let mut node = row;
                let mut path = Vec::new();
                while tags[node] != column {
                    path.push(node);
                    tags[node] = column;
                    node =
                        parent[node].expect("symbolic elimination path must reach current column");
                }
                pattern.extend(path.into_iter().rev());
            }
            let mut pivot = workspace[column];
            workspace[column] = S::zero();
            for &previous in pattern.iter().rev() {
                let value = workspace[previous];
                workspace[previous] = S::zero();
                let coefficient = value / diagonal[previous];
                for &(row, entry) in &columns[previous] {
                    workspace[row] -= entry * value;
                }
                pivot -= coefficient * value;
                columns[previous].push((column, coefficient));
            }
            diagonal[column] = pivot;
            if pivot == S::zero() {
                valid = false;
                break;
            }
        }
        Self {
            columns,
            diagonal,
            permutation,
            valid,
        }
    }
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.valid
    }
    #[must_use]
    pub fn permutation(&self) -> &[usize] {
        &self.permutation
    }
    #[must_use]
    pub fn diagonal(&self) -> &DVector<S> {
        &self.diagonal
    }
    /// # Panics
    /// Requires the original system size and a successful factorization.
    #[must_use]
    pub fn solve(&self, rhs: &DVector<S>) -> DVector<S> {
        assert_eq!(rhs.len(), self.permutation.len());
        assert!(self.valid, "sparse LDLT has a zero pivot");
        let mut result = DVector::from_fn(rhs.len(), |i, _| rhs[self.permutation[i]]);
        for (column, entries) in self.columns.iter().enumerate() {
            let value = result[column];
            if value != S::zero() {
                for &(row, coefficient) in entries {
                    result[row] -= value * coefficient;
                }
            }
        }
        for i in 0..result.len() {
            result[i] *= S::one() / self.diagonal[i];
        }
        for column in (0..self.columns.len()).rev() {
            let mut value = result[column];
            for &(row, coefficient) in &self.columns[column] {
                value -= coefficient * result[row];
            }
            result[column] = value;
        }
        let mut solution = DVector::zeros(rhs.len());
        for (new, &old) in self.permutation.iter().enumerate() {
            solution[old] = result[new];
        }
        solution
    }
}

/// Eigen's diagonally preconditioned conjugate gradient with default 2*n iterations.
/// # Panics
/// Requires a right-hand side matching the sparse matrix dimension.
#[must_use]
pub fn conjugate_gradient<S: BasaltScalar>(
    matrix: &SparseMatrix<S>,
    rhs: &DVector<S>,
    tolerance: S,
) -> DVector<S> {
    assert_eq!(matrix.size, rhs.len());
    let mut solution = DVector::zeros(rhs.len());
    let mut residual = rhs.clone();
    let rhs_norm = rhs.norm_squared();
    if rhs_norm == S::zero() {
        return solution;
    }
    let threshold = (tolerance * tolerance * rhs_norm).max(S::min_positive_value());
    let mut residual_norm = residual.norm_squared();
    if residual_norm < threshold {
        return solution;
    }
    let diagonal = matrix.diagonal().map(|v| {
        if v == S::zero() {
            S::one()
        } else {
            S::one() / v
        }
    });
    let mut direction = diagonal.component_mul(&residual);
    let mut magnitude = residual.dot(&direction);
    for _ in 0..2 * matrix.size {
        let product = matrix.multiply(&direction);
        let alpha = magnitude / direction.dot(&product);
        solution += &direction * alpha;
        residual -= product * alpha;
        residual_norm = residual.norm_squared();
        if residual_norm < threshold {
            break;
        }
        let preconditioned = diagonal.component_mul(&residual);
        let old = magnitude;
        magnitude = residual.dot(&preconditioned);
        direction = preconditioned + direction * (magnitude / old);
    }
    solution
}
