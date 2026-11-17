use super::absolute_adapter_base::AbsoluteAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Grouped correspondences with stable serialization in group-major order.
pub trait AbsoluteMultiAdapterBase: AbsoluteAdapterBase {
    fn bearing_vector_at(&self, group: usize, index: usize) -> Vector3<f64>;
    fn point_at(&self, group: usize, index: usize) -> Vector3<f64>;
    fn weight_at(&self, group: usize, index: usize) -> f64;
    fn multi_cam_offset(&self, group: usize) -> Vector3<f64>;
    fn multi_cam_rotation(&self, group: usize) -> Matrix3<f64>;
    fn number_correspondences_at(&self, group: usize) -> usize;
    fn number_frames(&self) -> usize;
    fn convert_multiindex(&self, group: usize, index: usize) -> usize;
    fn multi_frameindex(&self, index: usize) -> usize;
    fn multicorrespondence_index(&self, index: usize) -> usize;
    fn convert_multi_indices(&self, indices: &[Vec<usize>]) -> Vec<usize> {
        indices
            .iter()
            .enumerate()
            .flat_map(|(g, v)| v.iter().map(move |&i| self.convert_multiindex(g, i)))
            .collect()
    }
}
