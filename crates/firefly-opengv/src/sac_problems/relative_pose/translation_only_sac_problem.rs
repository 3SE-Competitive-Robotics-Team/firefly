use super::central_relative_pose_sac_problem::{central_scores, transformation};
use crate::{
    optimization_functor::NumericalBackend,
    relative_pose::{methods, relative_adapter_base::RelativeAdapterBase},
    sac::sample_consensus_problem::{SampleConsensusProblem, SamplingState},
    types::Transformation,
};
pub struct TranslationOnlySacProblem<'a, A: RelativeAdapterBase> {
    pub adapter: &'a mut A,
    pub backend: &'a dyn NumericalBackend,
    pub sampling: SamplingState,
}
impl<'a, A: RelativeAdapterBase> TranslationOnlySacProblem<'a, A> {
    pub fn new(adapter: &'a mut A, backend: &'a dyn NumericalBackend, random_seed: bool) -> Self {
        let mut sampling = SamplingState::new(random_seed);
        sampling.set_uniform_indices(adapter.number_correspondences());
        Self {
            adapter,
            backend,
            sampling,
        }
    }
}
impl<A: RelativeAdapterBase> SampleConsensusProblem for TranslationOnlySacProblem<'_, A> {
    type Model = Transformation;
    fn state(&self) -> &SamplingState {
        &self.sampling
    }
    fn state_mut(&mut self) -> &mut SamplingState {
        &mut self.sampling
    }
    fn sample_size(&self) -> usize {
        2
    }
    fn compute_model_coefficients(&mut self, indices: &[usize]) -> Option<Transformation> {
        Some(transformation(
            self.adapter.r12(),
            methods::twopt(self.adapter, true, indices),
        ))
    }
    fn selected_distances_to_model(
        &mut self,
        model: &Transformation,
        indices: &[usize],
    ) -> Vec<f64> {
        central_scores(self.adapter, model, indices)
    }
    fn optimize_model_coefficients(
        &mut self,
        inliers: &[usize],
        model: &Transformation,
    ) -> Option<Transformation> {
        self.adapter.set_t12(model.column(3).into_owned());
        self.adapter
            .set_r12(model.fixed_view::<3, 3>(0, 0).into_owned());
        methods::optimize_nonlinear(self.adapter, inliers, self.backend)
    }
}
