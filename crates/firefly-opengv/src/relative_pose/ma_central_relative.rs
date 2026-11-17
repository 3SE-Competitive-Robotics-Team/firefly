use crate::relative_pose::relative_adapter_base::RelativeAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Borrowed XYZ bearing arrays with independent declared counts.
pub struct MaCentralRelative<'a> {
    pub bearing_vectors1: &'a [f64],
    pub bearing_vectors2: &'a [f64],
    pub number_bearing_vectors1: usize,
    pub number_bearing_vectors2: usize,
    pub t12: Vector3<f64>,
    pub r12: Matrix3<f64>,
}
impl<'a> MaCentralRelative<'a> {
    /// # Errors
    /// A declared bearing count exceeds its XYZ backing storage.
    pub fn new(
        bearing_vectors1: &'a [f64],
        bearing_vectors2: &'a [f64],
        number_bearing_vectors1: usize,
        number_bearing_vectors2: usize,
    ) -> Result<Self, String> {
        if bearing_vectors1.len() / 3 < number_bearing_vectors1
            || bearing_vectors2.len() / 3 < number_bearing_vectors2
        {
            return Err("adapter storage shape violates correspondence bounds".into());
        }
        Ok(Self {
            bearing_vectors1,
            bearing_vectors2,
            number_bearing_vectors1,
            number_bearing_vectors2,
            t12: Vector3::zeros(),
            r12: Matrix3::identity(),
        })
    }
}
impl RelativeAdapterBase for MaCentralRelative<'_> {
    fn bearing_vector1(&self, index: usize) -> Vector3<f64> {
        assert!(index < self.number_bearing_vectors1);
        Vector3::from_column_slice(&self.bearing_vectors1[index * 3..index * 3 + 3])
    }
    fn bearing_vector2(&self, index: usize) -> Vector3<f64> {
        assert!(index < self.number_bearing_vectors2);
        Vector3::from_column_slice(&self.bearing_vectors2[index * 3..index * 3 + 3])
    }
    fn cam_offset1(&self, _: usize) -> Vector3<f64> {
        Vector3::zeros()
    }
    fn cam_rotation1(&self, _: usize) -> Matrix3<f64> {
        Matrix3::identity()
    }
    fn cam_offset2(&self, _: usize) -> Vector3<f64> {
        Vector3::zeros()
    }
    fn cam_rotation2(&self, _: usize) -> Matrix3<f64> {
        Matrix3::identity()
    }
    fn weight(&self, _: usize) -> f64 {
        1.0
    }
    fn number_correspondences(&self) -> usize {
        self.number_bearing_vectors2
    }
    fn t12(&self) -> Vector3<f64> {
        self.t12
    }
    fn set_t12(&mut self, value: Vector3<f64>) {
        self.t12 = value;
    }
    fn r12(&self) -> Matrix3<f64> {
        self.r12
    }
    fn set_r12(&mut self, value: Matrix3<f64>) {
        self.r12 = value;
    }
}
