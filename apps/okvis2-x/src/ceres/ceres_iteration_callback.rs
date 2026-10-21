/*
 * OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich.
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Predictive optimization time budget corresponding to `CeresIterationCallback`.

#[derive(Clone, Copy, Debug)]
pub struct IterationSummary {
    pub iteration: i32,
    pub cumulative_time_in_seconds: f64,
    pub iteration_time_in_seconds: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CallbackReturnType {
    SolverContinue,
    SolverTerminateSuccessfully,
}
/// The next iteration is assumed to take as long as the current one.
#[derive(Clone, Copy, Debug)]
pub struct CeresIterationCallback {
    time_limit: f64,
    iteration_minimum: i32,
}
impl CeresIterationCallback {
    #[must_use]
    pub fn new(time_limit: f64, iteration_minimum: i32) -> Self {
        Self {
            time_limit,
            iteration_minimum,
        }
    }
    #[must_use]
    pub fn call(&self, summary: &IterationSummary) -> CallbackReturnType {
        if summary.iteration >= self.iteration_minimum
            && summary.cumulative_time_in_seconds + summary.iteration_time_in_seconds
                > self.time_limit
        {
            CallbackReturnType::SolverTerminateSuccessfully
        } else {
            CallbackReturnType::SolverContinue
        }
    }
    pub fn set_time_limit(&mut self, time_limit: f64) {
        self.time_limit = time_limit;
    }
    pub fn set_minimum_iterations(&mut self, iteration_minimum: i32) {
        self.iteration_minimum = iteration_minimum;
    }
}
