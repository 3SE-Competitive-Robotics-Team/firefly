use crate::absolute_pose::{
    absolute_adapter_base::AbsoluteAdapterBase,
    absolute_multi_adapter_base::AbsoluteMultiAdapterBase,
};
use nalgebra::{Matrix3, Vector3};

pub struct NoncentralAbsoluteMultiAdapter<'a> {
    pub bearing_vectors: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
    pub points: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
    pub cam_offsets: &'a [Vector3<f64>],
    pub cam_rotations: &'a [Matrix3<f64>],
    pub t: Vector3<f64>,
    pub r: Matrix3<f64>,
    offsets: Vec<usize>,
    groups: Vec<usize>,
    correspondences: Vec<usize>,
}
impl<'a> NoncentralAbsoluteMultiAdapter<'a> {
    /// # Errors
    /// Camera group counts, per-group pair counts, or calibration counts differ.
    pub fn new(
        bearing_vectors: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
        points: Vec<std::sync::Arc<Vec<Vector3<f64>>>>,
        cam_offsets: &'a [Vector3<f64>],
        cam_rotations: &'a [Matrix3<f64>],
    ) -> firefly_error::Result<Self> {
        if bearing_vectors.len() != points.len()
            || bearing_vectors.len() != cam_offsets.len()
            || cam_offsets.len() != cam_rotations.len()
            || bearing_vectors
                .iter()
                .zip(&points)
                .any(|(b, p)| b.len() != p.len())
        {
            return Err(crate::invalid_correspondences());
        }

        let mut offsets = Vec::new();
        let mut groups = Vec::new();
        let mut correspondences = Vec::new();
        let mut offset = 0;
        for (g, n) in bearing_vectors.iter().map(|v| v.len()).enumerate() {
            offsets.push(offset);
            for i in 0..n {
                groups.push(g);
                correspondences.push(i);
            }
            offset += n;
        }
        Ok(Self {
            bearing_vectors,
            points,
            cam_offsets,
            cam_rotations,
            t: Vector3::zeros(),
            r: Matrix3::identity(),
            offsets,
            groups,
            correspondences,
        })
    }
}
impl AbsoluteMultiAdapterBase for NoncentralAbsoluteMultiAdapter<'_> {
    fn bearing_vector_at(&self, g: usize, i: usize) -> Vector3<f64> {
        self.bearing_vectors[g][i]
    }
    fn point_at(&self, g: usize, i: usize) -> Vector3<f64> {
        self.points[g][i]
    }
    fn weight_at(&self, _: usize, _: usize) -> f64 {
        1.0
    }
    fn multi_cam_offset(&self, g: usize) -> Vector3<f64> {
        self.cam_offsets[g]
    }
    fn multi_cam_rotation(&self, g: usize) -> Matrix3<f64> {
        self.cam_rotations[g]
    }
    fn number_correspondences_at(&self, g: usize) -> usize {
        self.bearing_vectors[g].len()
    }
    fn number_frames(&self) -> usize {
        self.cam_offsets.len()
    }
    fn convert_multiindex(&self, g: usize, i: usize) -> usize {
        self.offsets[g] + i
    }
    fn multi_frameindex(&self, i: usize) -> usize {
        self.groups[i]
    }
    fn multicorrespondence_index(&self, i: usize) -> usize {
        self.correspondences[i]
    }
}
impl AbsoluteAdapterBase for NoncentralAbsoluteMultiAdapter<'_> {
    fn bearing_vector(&self, i: usize) -> Vector3<f64> {
        self.bearing_vector_at(self.multi_frameindex(i), self.multicorrespondence_index(i))
    }
    fn point(&self, i: usize) -> Vector3<f64> {
        self.point_at(self.multi_frameindex(i), self.multicorrespondence_index(i))
    }
    fn weight(&self, i: usize) -> f64 {
        self.weight_at(self.multi_frameindex(i), self.multicorrespondence_index(i))
    }
    fn cam_offset(&self, i: usize) -> Vector3<f64> {
        self.multi_cam_offset(self.multi_frameindex(i))
    }
    fn cam_rotation(&self, i: usize) -> Matrix3<f64> {
        self.multi_cam_rotation(self.multi_frameindex(i))
    }
    fn number_correspondences(&self) -> usize {
        (0..self.number_frames())
            .map(|g| self.number_correspondences_at(g))
            .sum()
    }
    fn t(&self) -> Vector3<f64> {
        self.t
    }
    fn set_t(&mut self, v: Vector3<f64>) {
        self.t = v;
    }
    fn r(&self) -> Matrix3<f64> {
        self.r
    }
    fn set_r(&mut self, v: Matrix3<f64>) {
        self.r = v;
    }
}
