use crate::relative_pose::relative_adapter_base::RelativeAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Borrowed bearing pairs with a relative pose prior.
pub struct CentralRelativeAdapter<'a> {
    pub bearing_vectors1: &'a [Vector3<f64>],
    pub bearing_vectors2: &'a [Vector3<f64>],
    pub t12: Vector3<f64>,
    pub r12: Matrix3<f64>,
}
impl<'a> CentralRelativeAdapter<'a> {
    /// # Errors
    /// The two bearing arrays have different lengths.
    pub fn new(
        bearing_vectors1: &'a [Vector3<f64>],
        bearing_vectors2: &'a [Vector3<f64>],
    ) -> Result<Self, String> {
        if bearing_vectors1.len() != bearing_vectors2.len() {
            return Err("adapter storage shape violates correspondence bounds".into());
        }
        Ok(Self {
            bearing_vectors1,
            bearing_vectors2,
            t12: Vector3::zeros(),
            r12: Matrix3::identity(),
        })
    }
    #[must_use]
    pub fn with_rotation(mut self, rotation: Matrix3<f64>) -> Self {
        self.r12 = rotation;
        self
    }
    #[must_use]
    pub fn with_pose(mut self, translation: Vector3<f64>, rotation: Matrix3<f64>) -> Self {
        self.t12 = translation;
        self.r12 = rotation;
        self
    }
}
impl RelativeAdapterBase for CentralRelativeAdapter<'_> {
    fn bearing_vector1(&self, index: usize) -> Vector3<f64> {
        self.bearing_vectors1[index]
    }
    fn bearing_vector2(&self, index: usize) -> Vector3<f64> {
        self.bearing_vectors2[index]
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
        self.bearing_vectors2.len()
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
