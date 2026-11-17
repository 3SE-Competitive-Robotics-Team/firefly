use nalgebra::{Matrix3, Vector3};

/// Correspondence access; indices must satisfy the source adapter bounds.
pub trait RelativeAdapterBase {
    fn bearing_vector1(&self, index: usize) -> Vector3<f64>;
    fn cam_offset1(&self, index: usize) -> Vector3<f64>;
    fn cam_rotation1(&self, index: usize) -> Matrix3<f64>;
    fn bearing_vector2(&self, index: usize) -> Vector3<f64>;
    fn cam_offset2(&self, index: usize) -> Vector3<f64>;
    fn cam_rotation2(&self, index: usize) -> Matrix3<f64>;
    fn weight(&self, index: usize) -> f64;
    fn number_correspondences(&self) -> usize;
    fn t12(&self) -> Vector3<f64>;
    fn set_t12(&mut self, value: Vector3<f64>);
    fn r12(&self) -> Matrix3<f64>;
    fn set_r12(&mut self, value: Matrix3<f64>);
}
