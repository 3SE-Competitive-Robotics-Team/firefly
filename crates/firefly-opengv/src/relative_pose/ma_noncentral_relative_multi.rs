use crate::relative_pose::{
    relative_adapter_base::RelativeAdapterBase,
    relative_multi_adapter_base::RelativeMultiAdapterBase,
};
use nalgebra::{Matrix3, Vector3};

pub struct MaNoncentralRelativeMulti<'a> {
    pub bearing_vectors1: &'a [&'a [f64]],
    pub bearing_vectors2: &'a [&'a [f64]],
    pub cam_offsets: &'a [f64],
    pub number_bearing_vectors: &'a [usize],
    pub t12: Vector3<f64>,
    pub r12: Matrix3<f64>,
    offsets: Vec<usize>,
    groups: Vec<usize>,
    correspondences: Vec<usize>,
}
impl<'a> MaNoncentralRelativeMulti<'a> {
    /// # Errors
    /// Group counts differ, camera offsets are missing, or a bearing count exceeds its XYZ storage.
    pub fn new(
        bearing_vectors1: &'a [&'a [f64]],
        bearing_vectors2: &'a [&'a [f64]],
        cam_offsets: &'a [f64],
        number_bearing_vectors: &'a [usize],
    ) -> firefly_error::Result<Self> {
        if bearing_vectors1.len() != number_bearing_vectors.len()
            || bearing_vectors2.len() != number_bearing_vectors.len()
            || cam_offsets.len() / 3 < number_bearing_vectors.len()
            || number_bearing_vectors.iter().enumerate().any(|(g, &n)| {
                bearing_vectors1[g].len() / 3 < n || bearing_vectors2[g].len() / 3 < n
            })
        {
            return Err(crate::invalid_correspondences());
        }

        let mut offsets = Vec::new();
        let mut groups = Vec::new();
        let mut correspondences = Vec::new();
        let mut offset = 0;
        for (g, n) in number_bearing_vectors.iter().copied().enumerate() {
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
            number_bearing_vectors,
            t12: Vector3::zeros(),
            r12: Matrix3::identity(),
            offsets,
            groups,
            correspondences,
        })
    }
}
impl RelativeMultiAdapterBase for MaNoncentralRelativeMulti<'_> {
    fn bearing_vector1_at(&self, g: usize, i: usize) -> Vector3<f64> {
        {
            assert!(i < self.number_bearing_vectors[g]);
            Vector3::from_column_slice(&self.bearing_vectors1[g][3 * i..3 * i + 3])
        }
    }
    fn bearing_vector2_at(&self, g: usize, i: usize) -> Vector3<f64> {
        {
            assert!(i < self.number_bearing_vectors[g]);
            Vector3::from_column_slice(&self.bearing_vectors2[g][3 * i..3 * i + 3])
        }
    }
    fn weight_at(&self, _: usize, _: usize) -> f64 {
        1.0
    }
    fn cam_offset(&self, g: usize) -> Vector3<f64> {
        Vector3::from_column_slice(&self.cam_offsets[g * 3..g * 3 + 3])
    }
    fn cam_rotation(&self, _: usize) -> Matrix3<f64> {
        Matrix3::identity()
    }
    fn number_correspondences_at(&self, g: usize) -> usize {
        self.number_bearing_vectors[g]
    }
    fn number_pairs(&self) -> usize {
        self.number_bearing_vectors.len()
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
impl RelativeAdapterBase for MaNoncentralRelativeMulti<'_> {
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
