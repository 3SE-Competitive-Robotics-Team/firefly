use crate::{
    point_cloud::{methods, point_cloud_adapter_base::PointCloudAdapterBase},
    sac::sample_consensus_problem::{SampleConsensusProblem, SamplingState},
    types::Transformation,
};
pub struct PointCloudSacProblem<'a, A: PointCloudAdapterBase> {
    pub adapter: &'a mut A,
    pub sampling: SamplingState,
}
impl<'a, A: PointCloudAdapterBase> PointCloudSacProblem<'a, A> {
    pub fn new(adapter: &'a mut A, random_seed: bool) -> Self {
        let mut sampling = SamplingState::new(random_seed);
        sampling.set_uniform_indices(adapter.number_correspondences());
        Self { adapter, sampling }
    }
}
impl<A: PointCloudAdapterBase> SampleConsensusProblem for PointCloudSacProblem<'_, A> {
    type Model = Transformation;
    fn state(&self) -> &SamplingState {
        &self.sampling
    }
    fn state_mut(&mut self) -> &mut SamplingState {
        &mut self.sampling
    }
    fn sample_size(&self) -> usize {
        3
    }
    fn compute_model_coefficients(&mut self, indices: &[usize]) -> Option<Transformation> {
        Some(methods::threept_arun(self.adapter, indices))
    }
    fn selected_distances_to_model(
        &mut self,
        model: &Transformation,
        indices: &[usize],
    ) -> Vec<f64> {
        indices
            .iter()
            .map(|&i| {
                let point = self.adapter.point2(i);
                let transformed = model * nalgebra::Vector4::new(point[0], point[1], point[2], 1.0);
                (self.adapter.point1(i) - transformed).norm()
            })
            .collect()
    }
    fn optimize_model_coefficients(
        &mut self,
        inliers: &[usize],
        _: &Transformation,
    ) -> Option<Transformation> {
        Some(methods::threept_arun(self.adapter, inliers))
    }
}
