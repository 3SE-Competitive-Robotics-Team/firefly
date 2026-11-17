use crate::absolute_pose::absolute_adapter_base::AbsoluteAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Borrowed XYZ points and bearings with independent declared counts.
pub struct MaCentralAbsolute<'a> {
    pub points: &'a [f64],
    pub bearing_vectors: &'a [f64],
    pub number_points: usize,
    pub number_bearing_vectors: usize,
    pub t: Vector3<f64>,
    pub r: Matrix3<f64>,
}
impl<'a> MaCentralAbsolute<'a> {
    /// # Errors
    /// A declared point or bearing count exceeds its XYZ backing storage.
    pub fn new(
        points: &'a [f64],
        bearing_vectors: &'a [f64],
        number_points: usize,
        number_bearing_vectors: usize,
    ) -> Result<Self, String> {
        if points.len() / 3 < number_points || bearing_vectors.len() / 3 < number_bearing_vectors {
            return Err("adapter storage shape violates correspondence bounds".into());
        }
        Ok(Self {
            points,
            bearing_vectors,
            number_points,
            number_bearing_vectors,
            t: Vector3::zeros(),
            r: Matrix3::identity(),
        })
    }
}
impl AbsoluteAdapterBase for MaCentralAbsolute<'_> {
    fn bearing_vector(&self, index: usize) -> Vector3<f64> {
        assert!(index < self.number_bearing_vectors);
        Vector3::from_column_slice(&self.bearing_vectors[index * 3..index * 3 + 3])
    }
    fn point(&self, index: usize) -> Vector3<f64> {
        assert!(index < self.number_points);
        Vector3::from_column_slice(&self.points[index * 3..index * 3 + 3])
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
        self.number_bearing_vectors
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
