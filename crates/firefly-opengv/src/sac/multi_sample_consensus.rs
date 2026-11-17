use super::multi_sample_consensus_problem::MultiSampleConsensusProblem;

/// Shared algorithm state. A coefficient exists only after a successful fit.
pub struct MultiSampleConsensus<P: MultiSampleConsensusProblem> {
    pub sac_model: P,
    pub max_iterations: i32,
    pub threshold: f64,
    pub probability: f64,
    pub iterations: i32,
    pub model: Vec<Vec<usize>>,
    pub model_coefficients: Option<P::Model>,
    pub inliers: Vec<Vec<usize>>,
}
impl<P: MultiSampleConsensusProblem> MultiSampleConsensus<P> {
    pub fn new(sac_model: P, max_iterations: i32, threshold: f64, probability: f64) -> Self {
        Self {
            sac_model,
            max_iterations,
            threshold,
            probability,
            iterations: 0,
            model: Vec::new(),
            model_coefficients: None,
            inliers: Vec::new(),
        }
    }
}
