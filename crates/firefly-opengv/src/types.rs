/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

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
