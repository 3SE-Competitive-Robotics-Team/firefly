use super::central_relative_pose_sac_problem::current_score;
use crate::{
    math::cayley::{cayley2rot, rot2cayley},
    optimization_functor::NumericalBackend,
    relative_pose::{methods, relative_adapter_base::RelativeAdapterBase},
    sac::sample_consensus_problem::{SampleConsensusProblem, SamplingState},
    types::EigensolverOutput,
};
use rand::RngExt;
pub struct EigensolverSacProblem<'a, A: RelativeAdapterBase> {
    pub adapter: &'a mut A,
    pub sample_size: usize,
    pub backend: &'a dyn NumericalBackend,
    pub sampling: SamplingState,
}
impl<'a, A: RelativeAdapterBase> EigensolverSacProblem<'a, A> {
    pub fn new(
        adapter: &'a mut A,
        sample_size: usize,
        backend: &'a dyn NumericalBackend,
        random_seed: bool,
    ) -> Self {
        let mut sampling = SamplingState::new(random_seed);
        sampling.set_uniform_indices(adapter.number_correspondences());
        Self {
            adapter,
            sample_size,
            backend,
            sampling,
        }
    }
}
impl<A: RelativeAdapterBase> SampleConsensusProblem for EigensolverSacProblem<'_, A> {
    type Model = EigensolverOutput;
    fn state(&self) -> &SamplingState {
        &self.sampling
    }
    fn state_mut(&mut self) -> &mut SamplingState {
        &mut self.sampling
    }
    fn sample_size(&self) -> usize {
        self.sample_size
    }
    fn compute_model_coefficients(&mut self, indices: &[usize]) -> Option<Self::Model> {
        let original = self.adapter.r12();
        let mut c = rot2cayley(&original);
        for i in 0..3 {
            c[i] += (f64::from(rand::rng().random_range(0..=i32::MAX)) / f64::from(i32::MAX) - 0.5)
                * 2.0
                * 0.1;
        }
        methods::eigensolver_with_rotation(
            self.adapter,
            indices,
            &cayley2rot(&c),
            false,
            self.backend,
        )
    }
    fn selected_distances_to_model(&mut self, model: &Self::Model, indices: &[usize]) -> Vec<f64> {
        let t = self.adapter.t12();
        let r = self.adapter.r12();
        self.adapter.set_t12(model.translation);
        self.adapter.set_r12(model.rotation);
        let scores = indices
            .iter()
            .map(|&i| current_score(self.adapter, i))
            .collect();
        self.adapter.set_t12(t);
        self.adapter.set_r12(r);
        scores
    }
    fn optimize_model_coefficients(
        &mut self,
        inliers: &[usize],
        model: &Self::Model,
    ) -> Option<Self::Model> {
        let mut c = rot2cayley(&model.rotation);
        for i in 0..3 {
            c[i] += (f64::from(rand::rng().random_range(0..=i32::MAX)) / f64::from(i32::MAX) - 0.5)
                * 2.0
                * 0.0;
        }
        methods::eigensolver_with_rotation(
            self.adapter,
            inliers,
            &cayley2rot(&c),
            true,
            self.backend,
        )
    }
}
