use crate::{
    relative_pose::{methods, relative_adapter_base::RelativeAdapterBase},
    sac::sample_consensus_problem::{SampleConsensusProblem, SamplingState},
};
use nalgebra::Matrix3;
pub struct RotationOnlySacProblem<'a, A: RelativeAdapterBase> {
    pub adapter: &'a mut A,
    pub sampling: SamplingState,
}
impl<'a, A: RelativeAdapterBase> RotationOnlySacProblem<'a, A> {
    pub fn new(adapter: &'a mut A, random_seed: bool) -> Self {
        let mut sampling = SamplingState::new(random_seed);
        sampling.set_uniform_indices(adapter.number_correspondences());
        Self { adapter, sampling }
    }
}
impl<A: RelativeAdapterBase> SampleConsensusProblem for RotationOnlySacProblem<'_, A> {
    type Model = Matrix3<f64>;
    fn state(&self) -> &SamplingState {
        &self.sampling
    }
    fn state_mut(&mut self) -> &mut SamplingState {
        &mut self.sampling
    }
    fn sample_size(&self) -> usize {
        2
    }
    fn compute_model_coefficients(&mut self, indices: &[usize]) -> Option<Self::Model> {
        Some(methods::twopt_rotation_only(self.adapter, indices))
    }
    fn selected_distances_to_model(&mut self, model: &Self::Model, indices: &[usize]) -> Vec<f64> {
        indices
            .iter()
            .map(|&i| {
                1.0 - self
                    .adapter
                    .bearing_vector1(i)
                    .dot(&(model * self.adapter.bearing_vector2(i)))
            })
            .collect()
    }
    fn optimize_model_coefficients(
        &mut self,
        inliers: &[usize],
        _: &Self::Model,
    ) -> Option<Self::Model> {
        Some(methods::rotation_only(self.adapter, inliers))
    }
}
