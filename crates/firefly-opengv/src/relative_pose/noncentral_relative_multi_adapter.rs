use crate::relative_pose::{
    relative_adapter_base::RelativeAdapterBase,
    relative_multi_adapter_base::RelativeMultiAdapterBase,
};
use nalgebra::{Matrix3, Vector3};

pub struct NoncentralRelativeMultiAdapter<'a> {
    pub bearing_vectors1: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
    pub bearing_vectors2: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
    pub cam_offsets: &'a [Vector3<f64>],
    pub cam_rotations: &'a [Matrix3<f64>],
    pub t12: Vector3<f64>,
    pub r12: Matrix3<f64>,
    offsets: Vec<usize>,
    groups: Vec<usize>,
    correspondences: Vec<usize>,
}
impl<'a> NoncentralRelativeMultiAdapter<'a> {
    /// # Errors
    /// Camera group counts, per-group bearing counts, or calibration counts differ.
    pub fn new(
        bearing_vectors1: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
        bearing_vectors2: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
        cam_offsets: &'a [Vector3<f64>],
        cam_rotations: &'a [Matrix3<f64>],
    ) -> Result<Self, String> {
        if bearing_vectors1.len() != bearing_vectors2.len()
            || bearing_vectors1
                .iter()
                .zip(&bearing_vectors2)
                .any(|(a, b)| a.len() != b.len())
            || bearing_vectors2.len() != cam_offsets.len()
            || cam_offsets.len() != cam_rotations.len()
        {
            return Err("adapter storage shape violates correspondence bounds".into());
        }

        let mut offsets = Vec::new();
        let mut groups = Vec::new();
        let mut correspondences = Vec::new();
        let mut offset = 0;
        for (g, n) in bearing_vectors2.iter().map(|v| v.len()).enumerate() {
            offsets.push(offset);
            for i in 0..n {
                groups.push(g);
                correspondences.push(i);
            }
            offset += n;
        }
        Ok(Self {
            bearing_vectors1,
            bearing_vectors2,
            cam_offsets,
            cam_rotations,
            t12: Vector3::zeros(),
            r12: Matrix3::identity(),
            offsets,
            groups,
            correspondences,
        })
    }
}
impl RelativeMultiAdapterBase for NoncentralRelativeMultiAdapter<'_> {
    fn bearing_vector1_at(&self, g: usize, i: usize) -> Vector3<f64> {
        self.bearing_vectors1[g][i]
    }
    fn bearing_vector2_at(&self, g: usize, i: usize) -> Vector3<f64> {
        self.bearing_vectors2[g][i]
    }
    fn weight_at(&self, _: usize, _: usize) -> f64 {
        1.0
    }
    fn cam_offset(&self, g: usize) -> Vector3<f64> {
        self.cam_offsets[g]
    }
    fn cam_rotation(&self, g: usize) -> Matrix3<f64> {
        self.cam_rotations[g]
    }
    fn number_correspondences_at(&self, g: usize) -> usize {
        self.bearing_vectors2[g].len()
    }
    fn number_pairs(&self) -> usize {
        self.cam_offsets.len()
    }
    fn convert_multiindex(&self, g: usize, i: usize) -> usize {
        self.offsets[g] + i
    }
    fn multi_pairindex(&self, i: usize) -> usize {
        self.groups[i]
    }
    fn multicorrespondence_index(&self, i: usize) -> usize {
        self.correspondences[i]
    }
}
impl RelativeAdapterBase for NoncentralRelativeMultiAdapter<'_> {
    fn bearing_vector1(&self, i: usize) -> Vector3<f64> {
        self.bearing_vector1_at(self.multi_pairindex(i), self.multicorrespondence_index(i))
    }
    fn bearing_vector2(&self, i: usize) -> Vector3<f64> {
        self.bearing_vector2_at(self.multi_pairindex(i), self.multicorrespondence_index(i))
    }
    fn weight(&self, i: usize) -> f64 {
        self.weight_at(self.multi_pairindex(i), self.multicorrespondence_index(i))
    }
    fn cam_offset1(&self, i: usize) -> Vector3<f64> {
        self.cam_offset(self.multi_pairindex(i))
    }
    fn cam_rotation1(&self, i: usize) -> Matrix3<f64> {
        self.cam_rotation(self.multi_pairindex(i))
    }
    fn cam_offset2(&self, i: usize) -> Vector3<f64> {
        self.cam_offset(self.multi_pairindex(i))
    }
    fn cam_rotation2(&self, i: usize) -> Matrix3<f64> {
        self.cam_rotation(self.multi_pairindex(i))
    }
    fn number_correspondences(&self) -> usize {
        (0..self.number_pairs())
            .map(|g| self.number_correspondences_at(g))
            .sum()
    }
    fn t12(&self) -> Vector3<f64> {
        self.t12
    }
    fn set_t12(&mut self, v: Vector3<f64>) {
        self.t12 = v;
    }
    fn r12(&self) -> Matrix3<f64> {
        self.r12
    }
    fn set_r12(&mut self, v: Matrix3<f64>) {
        self.r12 = v;
    }
}
