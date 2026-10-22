/******************************************************************************
 * Authors:  Laurent Kneip & Paul Furgale                                     *
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

use rand::{RngExt, SeedableRng, rngs::StdRng};

/// Sampling storage. Indices retain caller order; partial shuffles persist between draws.
pub struct SamplingState {
    pub indices: Vec<usize>,
    pub shuffled_indices: Vec<usize>,
    pub max_sample_checks: i32,
    rng: StdRng,
}
impl SamplingState {
    #[must_use]
    pub fn new(random_seed: bool) -> Self {
        Self {
            indices: Vec::new(),
            shuffled_indices: Vec::new(),
            max_sample_checks: 10,
            rng: if random_seed {
                StdRng::from_rng(&mut rand::rng())
            } else {
                StdRng::seed_from_u64(12345)
            },
        }
    }
    pub fn set_indices(&mut self, indices: Vec<usize>) {
        self.shuffled_indices.clone_from(&indices);
        self.indices = indices;
    }
    pub fn set_uniform_indices(&mut self, n: usize) {
        self.set_indices((0..n).collect());
    }
    pub fn rnd(&mut self) -> usize {
        self.rng.random_range(0..=i32::MAX) as usize
    }
    pub fn draw_index_sample(&mut self, sample: &mut [usize]) {
        let n = self.shuffled_indices.len();
        for i in 0..sample.len() {
            let j = i + self.rnd() % (n - i);
            self.shuffled_indices.swap(i, j);
        }
        sample.copy_from_slice(&self.shuffled_indices[..sample.len()]);
    }
}
/// Concrete problem functions plus the source's implemented sample/score operations.
/// Selected distance output is in precisely the supplied index order.
pub trait SampleConsensusProblem {
    type Model: Clone;
    fn state(&self) -> &SamplingState;
    fn state_mut(&mut self) -> &mut SamplingState;
    fn sample_size(&self) -> usize;
    fn compute_model_coefficients(&mut self, indices: &[usize]) -> Option<Self::Model>;
    fn optimize_model_coefficients(
        &mut self,
        inliers: &[usize],
        model: &Self::Model,
    ) -> Option<Self::Model>;
    fn selected_distances_to_model(&mut self, model: &Self::Model, indices: &[usize]) -> Vec<f64>;
    fn is_sample_good(&self, _: &[usize]) -> bool {
        true
    }
    fn indices(&self) -> &[usize] {
        &self.state().indices
    }
    fn set_indices(&mut self, indices: Vec<usize>) {
        self.state_mut().set_indices(indices);
    }
    fn set_uniform_indices(&mut self, n: usize) {
        self.state_mut().set_uniform_indices(n);
    }
    fn rnd(&mut self) -> usize {
        self.state_mut().rnd()
    }
    fn draw_index_sample(&mut self, sample: &mut [usize]) {
        self.state_mut().draw_index_sample(sample);
    }
    fn get_samples(&mut self, iterations: &mut i32, samples: &mut Vec<usize>) {
        let n = self.sample_size();
        if self.state().indices.len() < n {
            log::error!(
                "Cannot select {n} unique points out of {}",
                self.state().indices.len()
            );
            samples.clear();
            *iterations = i32::MAX;
            return;
        }
        samples.resize(n, 0);
        for _ in 0..self.state().max_sample_checks {
            self.draw_index_sample(samples);
            if self.is_sample_good(samples) {
                return;
            }
        }
        log::warn!(
            "Could not select {n} sample points in {} iterations",
            self.state().max_sample_checks
        );
        samples.clear();
    }
    fn distances_to_model(&mut self, model: &Self::Model) -> Vec<f64> {
        let indices = self.state().indices.clone();
        self.selected_distances_to_model(model, &indices)
    }
    fn select_within_distance(&mut self, model: &Self::Model, threshold: f64) -> Vec<usize> {
        let dist = self.distances_to_model(model);
        dist.iter()
            .enumerate()
            .filter(|&(_, &d)| d < threshold)
            .map(|(i, _)| self.indices()[i])
            .collect()
    }
    fn count_within_distance(&mut self, model: &Self::Model, threshold: f64) -> usize {
        self.distances_to_model(model)
            .iter()
            .filter(|&&d| d < threshold)
            .count()
    }
}
