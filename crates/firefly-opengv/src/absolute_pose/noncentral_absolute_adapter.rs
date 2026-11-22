use crate::absolute_pose::absolute_adapter_base::AbsoluteAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Borrowed bearing/point pairs with per-correspondence camera extrinsics.
pub struct NoncentralAbsoluteAdapter<'a> {
    pub points: &'a [Vector3<f64>],
    pub bearing_vectors: &'a [Vector3<f64>],
    pub cam_correspondences: &'a [usize],
    pub cam_offsets: &'a [Vector3<f64>],
    pub cam_rotations: &'a [Matrix3<f64>],
    pub t: Vector3<f64>,
    pub r: Matrix3<f64>,
}
impl<'a> NoncentralAbsoluteAdapter<'a> {
    /// # Errors
    /// Correspondence lengths differ or a camera index exceeds its calibration storage.
    pub fn new(
        bearing_vectors: &'a [Vector3<f64>],
        cam_correspondences: &'a [usize],
        points: &'a [Vector3<f64>],
        cam_offsets: &'a [Vector3<f64>],
        cam_rotations: &'a [Matrix3<f64>],
    ) -> firefly_error::Result<Self> {
        if points.len() != bearing_vectors.len()
            || cam_correspondences.len() != bearing_vectors.len()
            || cam_correspondences
                .iter()
                .any(|&i| i >= cam_offsets.len() || i >= cam_rotations.len())
        {
            return Err(crate::invalid_correspondences());
        }
        Ok(Self {
            points,
            bearing_vectors,
            cam_correspondences,
            cam_offsets,
            cam_rotations,
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
impl AbsoluteAdapterBase for NoncentralAbsoluteAdapter<'_> {
    fn bearing_vector(&self, index: usize) -> Vector3<f64> {
        self.bearing_vectors[index]
    }
    fn point(&self, index: usize) -> Vector3<f64> {
        assert!(index < self.bearing_vectors.len());
        self.points[index]
    }
    fn cam_offset(&self, index: usize) -> Vector3<f64> {
        self.cam_offsets[self.cam_correspondences[index]]
    }
    fn cam_rotation(&self, index: usize) -> Matrix3<f64> {
        self.cam_rotations[self.cam_correspondences[index]]
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
