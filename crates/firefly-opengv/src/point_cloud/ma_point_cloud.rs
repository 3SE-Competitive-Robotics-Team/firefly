use crate::point_cloud::point_cloud_adapter_base::PointCloudAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Borrowed XYZ point clouds with independent declared counts.
pub struct MaPointCloud<'a> {
    pub points1: &'a [f64],
    pub points2: &'a [f64],
    pub number_points1: usize,
    pub number_points2: usize,
    pub t12: Vector3<f64>,
    pub r12: Matrix3<f64>,
}
impl<'a> MaPointCloud<'a> {
    /// # Errors
    /// A declared point count exceeds its XYZ backing storage.
    pub fn new(
        points1: &'a [f64],
        points2: &'a [f64],
        number_points1: usize,
        number_points2: usize,
    ) -> Result<Self, String> {
        if points1.len() / 3 < number_points1 || points2.len() / 3 < number_points2 {
            return Err("adapter storage shape violates correspondence bounds".into());
        }
        Ok(Self {
            points1,
            points2,
            number_points1,
            number_points2,
            t12: Vector3::zeros(),
            r12: Matrix3::identity(),
        })
    }
}
impl PointCloudAdapterBase for MaPointCloud<'_> {
    fn point1(&self, index: usize) -> Vector3<f64> {
        assert!(index < self.number_points1);
        Vector3::from_column_slice(&self.points1[index * 3..index * 3 + 3])
    }
    fn point2(&self, index: usize) -> Vector3<f64> {
        assert!(index < self.number_points2);
        Vector3::from_column_slice(&self.points2[index * 3..index * 3 + 3])
    }
    fn weight(&self, _: usize) -> f64 {
        1.0
    }
    fn number_correspondences(&self) -> usize {
        self.number_points2
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
