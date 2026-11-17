use nalgebra::{Matrix3, Vector3};

/// Correspondence access; indices must satisfy the source adapter bounds.
pub trait AbsoluteAdapterBase {
    fn bearing_vector(&self, index: usize) -> Vector3<f64>;
    fn point(&self, index: usize) -> Vector3<f64>;
    fn cam_offset(&self, index: usize) -> Vector3<f64>;
    fn cam_rotation(&self, index: usize) -> Matrix3<f64>;
    fn weight(&self, index: usize) -> f64;
    fn number_correspondences(&self) -> usize;
    fn t(&self) -> Vector3<f64>;
    fn set_t(&mut self, value: Vector3<f64>);
    fn r(&self) -> Matrix3<f64>;
    fn set_r(&mut self, value: Matrix3<f64>);
}
