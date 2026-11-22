use crate::absolute_pose::absolute_adapter_base::AbsoluteAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Borrowed bearing/point pairs with a pose prior.
pub struct CentralAbsoluteAdapter<'a> {
    pub points: &'a [Vector3<f64>],
    pub bearing_vectors: &'a [Vector3<f64>],
    pub t: Vector3<f64>,
    pub r: Matrix3<f64>,
}
impl<'a> CentralAbsoluteAdapter<'a> {
    /// # Errors
    /// Bearing and point counts differ.
    pub fn new(
        bearing_vectors: &'a [Vector3<f64>],
        points: &'a [Vector3<f64>],
    ) -> firefly_error::Result<Self> {
        if points.len() != bearing_vectors.len() {
            return Err(crate::invalid_correspondences());
        }
        Ok(Self {
            points,
            bearing_vectors,
            t: Vector3::zeros(),
            r: Matrix3::identity(),
        })
    }
    #[must_use]
    pub fn with_rotation(mut self, rotation: Matrix3<f64>) -> Self {
        self.r = rotation;
        self
    }
    #[must_use]
    pub fn with_pose(mut self, translation: Vector3<f64>, rotation: Matrix3<f64>) -> Self {
        self.t = translation;
        self.r = rotation;
        self
    }
}
impl AbsoluteAdapterBase for CentralAbsoluteAdapter<'_> {
    fn bearing_vector(&self, index: usize) -> Vector3<f64> {
        self.bearing_vectors[index]
    }
    fn point(&self, index: usize) -> Vector3<f64> {
        assert!(index < self.bearing_vectors.len());
        self.points[index]
    }
    fn cam_offset(&self, _: usize) -> Vector3<f64> {
        Vector3::zeros()
    }
    fn cam_rotation(&self, _: usize) -> Matrix3<f64> {
        Matrix3::identity()
    }
    fn weight(&self, _: usize) -> f64 {
        1.0
    }
    fn number_correspondences(&self) -> usize {
        self.bearing_vectors.len()
    }
    fn t(&self) -> Vector3<f64> {
        self.t
    }
    fn set_t(&mut self, value: Vector3<f64>) {
        self.t = value;
    }
    fn r(&self) -> Matrix3<f64> {
        self.r
    }
    fn set_r(&mut self, value: Matrix3<f64>) {
        self.r = value;
    }
}
