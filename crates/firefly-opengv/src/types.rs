//! Geometric vision types. Transformations map viewpoint 2 coordinates into viewpoint 1.
use nalgebra::{Matrix3, Matrix4, SMatrix, Vector3, Vector4};
use num_complex::Complex64;
pub type BearingVector = Vector3<f64>;
pub type BearingVectors = Vec<BearingVector>;
pub type Translation = Vector3<f64>;
pub type Translations = Vec<Translation>;
pub type Rotation = Matrix3<f64>;
pub type Rotations = Vec<Rotation>;
pub type Transformation = SMatrix<f64, 3, 4>;
pub type Transformations = Vec<Transformation>;
pub type Cayley = Vector3<f64>;
pub type Quaternion = Vector4<f64>;
pub type Essential = Matrix3<f64>;
pub type Essentials = Vec<Essential>;
pub type ComplexEssential = Matrix3<Complex64>;
pub type ComplexEssentials = Vec<ComplexEssential>;
pub type Point = Vector3<f64>;
pub type Points = Vec<Point>;
pub type Eigenvalues = Vector3<f64>;
pub type Eigenvectors = Matrix3<f64>;
#[derive(Clone, Debug)]
pub struct EigensolverOutput {
    pub translation: Translation,
    pub rotation: Rotation,
    pub eigenvalues: Eigenvalues,
    pub eigenvectors: Eigenvectors,
}
#[derive(Clone, Debug)]
pub struct GeOutput {
    pub translation: Vector4<f64>,
    pub rotation: Rotation,
    pub eigenvalues: Vector4<f64>,
    pub eigenvectors: Matrix4<f64>,
}
