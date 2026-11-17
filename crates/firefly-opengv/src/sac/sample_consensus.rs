use super::sample_consensus_problem::SampleConsensusProblem;

/// Shared algorithm state. A coefficient exists only after a successful fit.
pub struct SampleConsensus<P: SampleConsensusProblem> {
    pub sac_model: P,
    pub max_iterations: i32,
    pub threshold: f64,
    pub probability: f64,
    pub iterations: i32,
    pub model: Vec<usize>,
    pub model_coefficients: Option<P::Model>,
    pub inliers: Vec<usize>,
}
impl<P: SampleConsensusProblem> SampleConsensus<P> {
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
