use crate::point_cloud::point_cloud_adapter_base::PointCloudAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Borrowed point pairs with a relative pose prior.
pub struct PointCloudAdapter<'a> {
    pub points1: &'a [Vector3<f64>],
    pub points2: &'a [Vector3<f64>],
    pub t12: Vector3<f64>,
    pub r12: Matrix3<f64>,
}
impl<'a> PointCloudAdapter<'a> {
    /// # Errors
    /// The two point clouds have different lengths.
    pub fn new(
        points1: &'a [Vector3<f64>],
        points2: &'a [Vector3<f64>],
    ) -> firefly_error::Result<Self> {
        if points1.len() != points2.len() {
            return Err(crate::invalid_correspondences());
        }
        Ok(Self {
            points1,
            points2,
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
impl PointCloudAdapterBase for PointCloudAdapter<'_> {
    fn point1(&self, index: usize) -> Vector3<f64> {
        self.points1[index]
    }
    fn point2(&self, index: usize) -> Vector3<f64> {
        self.points2[index]
    }
    fn weight(&self, _: usize) -> f64 {
        1.0
    }
    fn number_correspondences(&self) -> usize {
        self.points2.len()
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
