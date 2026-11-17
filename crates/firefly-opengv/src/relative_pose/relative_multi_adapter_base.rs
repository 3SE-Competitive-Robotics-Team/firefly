use super::relative_adapter_base::RelativeAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Grouped correspondences with stable serialization in group-major order.
pub trait RelativeMultiAdapterBase: RelativeAdapterBase {
    fn bearing_vector1_at(&self, group: usize, index: usize) -> Vector3<f64>;
    fn bearing_vector2_at(&self, group: usize, index: usize) -> Vector3<f64>;
    fn weight_at(&self, group: usize, index: usize) -> f64;
    fn cam_offset(&self, group: usize) -> Vector3<f64>;
    fn cam_rotation(&self, group: usize) -> Matrix3<f64>;
    fn number_correspondences_at(&self, group: usize) -> usize;
    fn number_pairs(&self) -> usize;
    fn convert_multiindex(&self, group: usize, index: usize) -> usize;
    fn multi_pairindex(&self, index: usize) -> usize;
    fn multicorrespondence_index(&self, index: usize) -> usize;
    fn convert_multi_indices(&self, indices: &[Vec<usize>]) -> Vec<usize> {
        indices
            .iter()
            .enumerate()
            .flat_map(|(g, v)| v.iter().map(move |&i| self.convert_multiindex(g, i)))
            .collect()
    }
}
