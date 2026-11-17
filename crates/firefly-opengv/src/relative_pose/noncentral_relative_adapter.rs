use crate::relative_pose::relative_adapter_base::RelativeAdapterBase;
use nalgebra::{Matrix3, Vector3};

/// Borrowed bearing pairs with per-correspondence camera extrinsics.
pub struct NoncentralRelativeAdapter<'a> {
    pub bearing_vectors1: &'a [Vector3<f64>],
    pub bearing_vectors2: &'a [Vector3<f64>],
    pub cam_correspondences1: &'a [usize],
    pub cam_correspondences2: &'a [usize],
    pub cam_offsets: &'a [Vector3<f64>],
    pub cam_rotations: &'a [Matrix3<f64>],
    pub t12: Vector3<f64>,
    pub r12: Matrix3<f64>,
}
impl<'a> NoncentralRelativeAdapter<'a> {
    /// # Errors
    /// Correspondence lengths differ or a camera index exceeds its calibration storage.
    pub fn new(
        bearing_vectors1: &'a [Vector3<f64>],
        bearing_vectors2: &'a [Vector3<f64>],
        cam_correspondences1: &'a [usize],
        cam_correspondences2: &'a [usize],
        cam_offsets: &'a [Vector3<f64>],
        cam_rotations: &'a [Matrix3<f64>],
    ) -> Result<Self, String> {
        if bearing_vectors1.len() != bearing_vectors2.len()
            || cam_correspondences1.len() != bearing_vectors1.len()
            || cam_correspondences1
                .iter()
                .any(|&i| i >= cam_offsets.len() || i >= cam_rotations.len())
            || cam_correspondences2.len() != bearing_vectors2.len()
            || cam_correspondences2
                .iter()
                .any(|&i| i >= cam_offsets.len() || i >= cam_rotations.len())
        {
            return Err("adapter storage shape violates correspondence bounds".into());
        }
        Ok(Self {
            bearing_vectors1,
            bearing_vectors2,
            cam_correspondences1,
            cam_correspondences2,
            cam_offsets,
            cam_rotations,
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
impl RelativeAdapterBase for NoncentralRelativeAdapter<'_> {
    fn bearing_vector1(&self, index: usize) -> Vector3<f64> {
        self.bearing_vectors1[index]
    }
    fn bearing_vector2(&self, index: usize) -> Vector3<f64> {
        self.bearing_vectors2[index]
    }
    fn cam_offset1(&self, index: usize) -> Vector3<f64> {
        self.cam_offsets[self.cam_correspondences1[index]]
    }
    fn cam_rotation1(&self, index: usize) -> Matrix3<f64> {
        self.cam_rotations[self.cam_correspondences1[index]]
    }
    fn cam_offset2(&self, index: usize) -> Vector3<f64> {
        self.cam_offsets[self.cam_correspondences2[index]]
    }
    fn cam_rotation2(&self, index: usize) -> Matrix3<f64> {
        self.cam_rotations[self.cam_correspondences2[index]]
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
