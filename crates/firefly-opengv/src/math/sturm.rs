/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

use nalgebra::DMatrix;
use std::collections::VecDeque;

#[derive(Clone, Debug)]
pub struct Bracket {
    lower_bound: f64,
    upper_bound: f64,
    lower_changes: Option<usize>,
    upper_changes: Option<usize>,
}
impl Bracket {
    #[must_use]
    pub fn new(lower_bound: f64, upper_bound: f64) -> Self {
        Self {
            lower_bound,
            upper_bound,
            lower_changes: None,
            upper_changes: None,
        }
    }
    #[must_use]
    pub fn with_changes(lower: f64, upper: f64, changes: usize, set_upper: bool) -> Self {
        let mut b = Self::new(lower, upper);
        if set_upper {
            b.set_upper_bound_changes(changes);
        } else {
            b.set_lower_bound_changes(changes);
        }
        b
    }
    #[must_use]
    pub fn lower_bound(&self) -> f64 {
        self.lower_bound
    }
    #[must_use]
    pub fn upper_bound(&self) -> f64 {
        self.upper_bound
    }
    #[must_use]
    pub fn lower_bound_changes_computed(&self) -> bool {
        self.lower_changes.is_some()
    }
    #[must_use]
    pub fn upper_bound_changes_computed(&self) -> bool {
        self.upper_changes.is_some()
    }
    pub fn set_lower_bound_changes(&mut self, changes: usize) {
        self.lower_changes = Some(changes);
    }
    pub fn set_upper_bound_changes(&mut self, changes: usize) {
        self.upper_changes = Some(changes);
    }
    #[must_use]
    pub fn number_roots(&self) -> usize {
        if let (Some(l), Some(u)) = (self.lower_changes, self.upper_changes) {
            l.wrapping_sub(u)
        } else {
            log::error!("cannot evaluate number of roots");
            0
        }
    }
    #[must_use]
    pub fn dividable(&self, eps: f64) -> bool {
        if self.number_roots() == 1 && self.upper_bound - self.lower_bound < eps {
            return false;
        }
        if self.number_roots() == 0 {
            return false;
        }
        let bound_sum = self.upper_bound + self.lower_bound;
        let center = bound_sum / 2.0;
        center != self.upper_bound && center != self.lower_bound
    }
    /// Uncomputed counters retain the constructor's zero storage value when split.
    pub fn divide(&self, brackets: &mut VecDeque<Self>) {
        let bound_sum = self.upper_bound + self.lower_bound;
        let center = bound_sum / 2.0;
        brackets.push_back(Self::with_changes(
            self.lower_bound,
            center,
            self.lower_changes.unwrap_or(0),
            false,
        ));
        brackets.push_back(Self::with_changes(
            center,
            self.upper_bound,
            self.upper_changes.unwrap_or(0),
            true,
        ));
    }
}
/// Sturm chain coefficients, in descending power order.
#[derive(Clone, Debug)]
pub struct Sturm {
    c: DMatrix<f64>,
    dimension: usize,
}
impl Sturm {
    #[must_use]
    pub fn new(p: &[f64]) -> Option<Self> {
        let dimension = p.len();
        if dimension < 2 {
            return None;
        }
        let mut c = DMatrix::zeros(dimension, dimension);
        for i in 0..dimension {
            c[(0, i)] = p[i];
        }
        for i in 1..dimension {
            c[(1, i)] = c[(0, i - 1)] * (dimension - i) as f64;
        }
        for i in 2..dimension {
            let p1: Vec<_> = (i - 2..dimension).map(|j| c[(i - 2, j)]).collect();
            let p2: Vec<_> = (i - 1..dimension).map(|j| c[(i - 1, j)]).collect();
            let r = Self::compute_negated_remainder(&p1, &p2)?;
            for j in i..dimension {
                c[(i, j)] = r[j - i + 2];
            }
        }
        Some(Self { c, dimension })
    }
    #[must_use]
    pub fn from_matrix(p: &DMatrix<f64>) -> Option<Self> {
        if p.nrows() != 1 {
            return None;
        }
        Self::new(&p.row(0).iter().copied().collect::<Vec<_>>())
    }
    #[must_use]
    pub fn compute_negated_remainder(p1: &[f64], p2: &[f64]) -> Option<Vec<f64>> {
        if p1.len() != p2.len() + 1 || p2.len() < 2 {
            return None;
        }
        let mut poly1 = vec![0.0; p1.len()];
        let mut poly2 = poly1.clone();
        let mut poly3 = poly1.clone();
        for j in 0..p2.len() {
            poly1[j] = (p1[0] / p2[0]) * p2[j];
            poly2[j + 1] = (p1[1] / p2[0]) * p2[j];
            poly3[j + 1] = (-p2[1] * p1[0] / p2[0].powf(2.0)) * p2[j];
        }
        Some(
            (0..p1.len())
                .map(|j| -p1[j] + poly1[j] + poly2[j] + poly3[j])
                .collect(),
        )
    }
    fn monomials(&self, bound: f64) -> Vec<f64> {
        let mut m = vec![0.0; self.dimension];
        m[self.dimension - 1] = 1.0;
        for i in 2..=self.dimension {
            m[self.dimension - i] = m[self.dimension - i + 1] * bound;
        }
        m
    }
    #[must_use]
    pub fn evaluate_chain(&self, bound: f64) -> usize {
        let m = self.monomials(bound);
        let mut signs = vec![0.0; self.dimension];
        for (i, sign) in signs.iter_mut().enumerate() {
            for (j, monomial) in m.iter().enumerate().skip(i) {
                *sign += self.c[(i, j)] * monomial;
            }
        }
        Self::changes(&signs)
    }
    #[must_use]
    pub fn evaluate_chain2(&self, bound: f64) -> usize {
        let m = self.monomials(bound);
        let mut signs = vec![0.0; self.dimension];
        for (i, sign) in signs.iter_mut().enumerate() {
            for (j, monomial) in m.iter().enumerate().skip(i) {
                *sign += self.c[(i, j)] * monomial;
            }
        }
        Self::changes(&signs)
    }
    fn changes(signs: &[f64]) -> usize {
        let mut positive = signs[0] > 0.0;
        let mut changes = 0;
        for s in &signs[1..] {
            if positive {
                if *s < 0.0 {
                    changes += 1;
                    positive = false;
                }
            } else if *s > 0.0 {
                changes += 1;
                positive = true;
            }
        }
        changes
    }
    #[must_use]
    pub fn compute_lagrangian_bound(&self) -> f64 {
        let mut coefficients: Vec<_> = (0..self.dimension - 1)
            .map(|i| {
                (self.c[(0, i + 1)] / self.c[(0, 0)])
                    .abs()
                    .powf(1.0 / (i + 1) as f64)
            })
            .collect();
        let mut j = 0;
        let mut max1 = -1.0;
        for (i, v) in coefficients.iter().enumerate() {
            if *v > max1 {
                j = i;
                max1 = *v;
            }
        }
        coefficients[j] = -1.0;
        let mut max2 = -1.0;
        for v in coefficients {
            if v > max2 {
                max2 = v;
            }
        }
        max1 + max2
    }
    fn initial_bracket(&self) -> Bracket {
        let bound = self.compute_lagrangian_bound();
        let mut b = Bracket::new(-bound, bound);
        b.set_lower_bound_changes(self.evaluate_chain2(b.lower_bound()));
        b.set_upper_bound_changes(self.evaluate_chain2(b.upper_bound()));
        b
    }
    fn split(&self, b: &Bracket, stack: &mut VecDeque<Bracket>) {
        b.divide(stack);
        let n = stack.len();
        let changes = self.evaluate_chain2(stack[n - 1].lower_bound());
        stack[n - 1].set_lower_bound_changes(changes);
        stack[n - 2].set_upper_bound_changes(changes);
    }
    pub fn bracket_roots(&self, roots: &mut Vec<f64>, eps: f64) {
        let b = self.initial_bracket();
        let local_eps = if eps < 0.0 {
            self.compute_lagrangian_bound() / (10.0 * b.number_roots() as f64)
        } else {
            eps
        };
        let mut stack = VecDeque::from([b]);
        while let Some(b) = stack.pop_front() {
            if b.dividable(local_eps) {
                self.split(&b, &mut stack);
            } else if b.number_roots() > 0 {
                let bound_sum = b.lower_bound() + b.upper_bound();
                roots.push(0.5 * bound_sum);
            }
        }
    }
    #[must_use]
    pub fn find_roots(&self) -> Vec<f64> {
        let mut roots = vec![];
        self.bracket_roots(&mut roots, -1.0);
        for r in &mut roots {
            for _ in 0..5 {
                let m = self.monomials(*r);
                let value = self.value(0, &m);
                let derivative = self.value(1, &m);
                let reduction_term = value / derivative;
                *r -= reduction_term;
            }
        }
        roots
    }
    fn value(&self, row: usize, m: &[f64]) -> f64 {
        (0..self.dimension).map(|j| self.c[(row, j)] * m[j]).sum()
    }
    pub fn find_roots2(&self, roots: &mut Vec<f64>, eps_x: f64, eps_val: f64) {
        let mut stack = VecDeque::from([self.initial_bracket()]);
        while let Some(b) = stack.pop_front() {
            if b.dividable(eps_x) {
                let mut divide = true;
                if b.number_roots() == 1 {
                    let mut converged = false;
                    let bound_sum = b.lower_bound() + b.upper_bound();
                    let mut root = 0.5 * bound_sum;
                    let mut m = self.monomials(root);
                    let mut value = self.value(0, &m);
                    while !converged {
                        let derivative = self.value(1, &m);
                        let new_root = root - value / derivative;
                        if new_root < b.lower_bound() || new_root > b.upper_bound() {
                            break;
                        }
                        m = self.monomials(new_root);
                        let new_value = self.value(0, &m);
                        if new_value.abs() > value.abs() {
                            break;
                        }
                        value = new_value;
                        root = new_root;
                        if value.abs() < eps_val {
                            converged = true;
                        }
                    }
                    if converged {
                        divide = false;
                        roots.push(root);
                    }
                }
                if divide {
                    self.split(&b, &mut stack);
                }
            } else if b.number_roots() > 0 {
                let bound_sum = b.lower_bound() + b.upper_bound();
                roots.push(0.5 * bound_sum);
            }
        }
    }
}
