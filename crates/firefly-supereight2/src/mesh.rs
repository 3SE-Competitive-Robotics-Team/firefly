/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! Uncoloured, unlabelled surface mesh geometry.
use crate::image::invalid;
use firefly_error::Result;
use nalgebra::{Matrix4, Vector3};
#[derive(Clone, Debug, PartialEq)]
pub struct MeshFace<const N: usize = 3> {
    pub vertices: [Vector3<f32>; N],
    pub scale: i8,
}
pub type Mesh<const N: usize = 3> = Vec<MeshFace<N>>;
pub type TriangleMesh = Mesh<3>;
pub type QuadMesh = Mesh<4>;
#[must_use]
pub fn quad_to_triangle_mesh(input: &[MeshFace<4>]) -> TriangleMesh {
    input
        .iter()
        .flat_map(|q| {
            [[0, 1, 2], [0, 2, 3]].map(|i| MeshFace {
                vertices: i.map(|j| q.vertices[j]),
                scale: q.scale,
            })
        })
        .collect()
}
pub fn transform<const N: usize>(mesh: &mut [MeshFace<N>], t: &Matrix4<f32>) {
    for face in mesh {
        for p in &mut face.vertices {
            *p = (t * p.push(1.0)).fixed_rows::<3>(0).into_owned();
        }
    }
}
#[derive(Clone, Debug, PartialEq)]
pub struct Vertex {
    pub position: Vector3<f32>,
    pub normal: Option<Vector3<f32>>,
}
#[derive(Clone, Debug, Default, PartialEq)]
pub struct VertexIndexMesh<const N: usize = 3> {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<usize>,
}
fn normalize(v: Vector3<f32>) -> Vector3<f32> {
    let n = v.norm();
    if n > 0.0 { v / n } else { v }
}
impl<const N: usize> VertexIndexMesh<N> {
    pub fn merge(&mut self, other: &Self) {
        let offset = self.vertices.len();
        self.vertices.extend_from_slice(&other.vertices);
        self.indices
            .extend(other.indices.iter().map(|i| i + offset));
    }
    /// # Errors
    /// Returns an error unless faces have three or four vertices, indices form complete faces,
    /// and every index addresses an existing vertex.
    pub fn compute_normals(&mut self) -> Result<()> {
        if (N != 3 && N != 4)
            || !self.indices.len().is_multiple_of(N)
            || self.indices.iter().any(|&i| i >= self.vertices.len())
        {
            return Err(invalid("invalid face indices"));
        }
        for face in self.indices.as_chunks::<N>().0 {
            let p = |i: usize| self.vertices[face[i]].position;
            let mut normal = normalize((p(1) - p(0)).cross(&(p(2) - p(0))));
            if N == 4 {
                normal += normalize((p(2) - p(0)).cross(&(p(3) - p(0))));
            }
            for &i in face {
                let n = &mut self.vertices[i].normal;
                *n = Some(n.unwrap_or(Vector3::zeros()) + normal);
            }
        }
        for v in &mut self.vertices {
            if let Some(n) = v.normal.as_mut() {
                *n = normalize(*n);
            }
        }
        Ok(())
    }
}
