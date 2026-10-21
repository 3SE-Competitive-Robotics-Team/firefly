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
use super::{
    eigensolver::modules as eigen,
    fivept_kneip::{code, init},
    fivept_nister::modules as nister,
    fivept_stewenius::modules as stewenius,
    ge::modules as ge,
    sixpt::modules2,
};
use crate::{
    math::{
        cayley::{cayley2rot, rot2cayley},
        sturm::Sturm,
    },
    optimization_functor::{LmParameters, NumericalBackend},
    types::{ComplexEssentials, EigensolverOutput, Essentials, GeOutput, Rotation, Rotations},
};
use nalgebra::{DMatrix, DVector, Matrix3, Matrix4, SMatrix, SVector, Vector3};
use num_complex::Complex64;
use rand::RngExt;
fn dynamic<const R: usize, const C: usize>(m: &SMatrix<f64, R, C>) -> DMatrix<f64> {
    DMatrix::from_column_slice(R, C, m.as_slice())
}
pub fn fivept_stewenius_main(
    ee: &SMatrix<f64, 9, 4>,
    backend: &dyn NumericalBackend,
) -> Option<ComplexEssentials> {
    let mut a = SMatrix::<f64, 10, 20>::zeros();
    stewenius::compose_a(ee, &mut a);
    let a1 = a.fixed_view::<10, 10>(0, 0).into_owned();
    let a2 = a.fixed_view::<10, 10>(0, 10).into_owned();
    let a3 = a1.full_piv_lu().try_inverse()? * a2;
    let mut m = SMatrix::<f64, 10, 10>::zeros();
    m.fixed_view_mut::<3, 10>(0, 0)
        .copy_from(&(-a3.fixed_view::<3, 10>(0, 0)));
    m.fixed_view_mut::<2, 10>(3, 0)
        .copy_from(&(-a3.fixed_view::<2, 10>(4, 0)));
    m.row_mut(5).copy_from(&(-a3.row(7)));
    for (r, c) in [(6, 0), (7, 1), (8, 3), (9, 6)] {
        m[(r, c)] = 1.0;
    }
    let (_, v) = backend.eigenpairs(&dynamic(&m))?;
    if v.shape() != (10, 10) {
        return None;
    }
    let mut sols = SMatrix::<Complex64, 4, 10>::from_element(Complex64::new(1.0, 0.0));
    for r in 0..3 {
        for c in 0..10 {
            sols[(r, c)] = v[(6 + r, c)] / v[(9, c)];
        }
    }
    let mut evec = ee.map(|x| Complex64::new(x, 0.0)) * sols;
    for c in 0..10 {
        let mut norm = Complex64::new(0.0, 0.0);
        for r in 0..9 {
            norm += evec[(r, c)].powi(2);
        }
        norm = norm.sqrt();
        for r in 0..9 {
            evec[(r, c)] /= norm;
        }
    }
    Some(
        (0..10)
            .map(|c| Matrix3::from_fn(|r, k| evec[(3 * r + k, c)]))
            .collect(),
    )
}
fn b<const C: usize>(a: &SMatrix<f64, 10, 10>, row: usize, col: usize) -> SMatrix<f64, 1, C> {
    let mut p = SMatrix::<f64, 1, C>::zeros();
    let mut q = p;
    for j in 0..C - 1 {
        p[j + 1] = a[(row, col + j)];
        q[j] = a[(row + 1, col + j)];
    }
    p - q
}
fn mul73(p: &SMatrix<f64, 1, 5>, q: &SMatrix<f64, 1, 4>) -> SMatrix<f64, 1, 8> {
    let mut v = SMatrix::zeros();
    nister::compute_seventh_order_polynomial(p, q, &mut v);
    v
}
fn mul64(p: &SMatrix<f64, 1, 4>, q: &SMatrix<f64, 1, 4>) -> SMatrix<f64, 1, 7> {
    let mut v = SMatrix::zeros();
    nister::compute_sixth_order_polynomial(p, q, &mut v);
    v
}
fn val<const C: usize>(p: &SMatrix<f64, 1, C>, x: f64) -> f64 {
    nister::poly_val(&dynamic(p), x)
}
pub fn fivept_nister_main(
    ee: &SMatrix<f64, 9, 4>,
    backend: &dyn NumericalBackend,
) -> Option<Essentials> {
    let mut a = SMatrix::<f64, 10, 20>::zeros();
    nister::compose_a(ee, &mut a);
    let a3 = a
        .fixed_view::<10, 10>(0, 0)
        .into_owned()
        .full_piv_lu()
        .try_inverse()?
        * a.fixed_view::<10, 10>(0, 10);
    let (b11, b21, b31, b12, b22, b32) = (
        b::<4>(&a3, 4, 0),
        b::<4>(&a3, 6, 0),
        b::<4>(&a3, 8, 0),
        b::<4>(&a3, 4, 3),
        b::<4>(&a3, 6, 3),
        b::<4>(&a3, 8, 3),
    );
    let (b13, b23, b33) = (b::<5>(&a3, 4, 6), b::<5>(&a3, 6, 6), b::<5>(&a3, 8, 6));
    let p1 = mul73(&b23, &b12) - mul73(&b13, &b22);
    let p2 = mul73(&b13, &b21) - mul73(&b23, &b11);
    let p3 = mul64(&b11, &b22) - mul64(&b12, &b21);
    let (mut po1, mut po2, mut po3) = (
        SMatrix::<f64, 1, 11>::zeros(),
        SMatrix::<f64, 1, 11>::zeros(),
        SMatrix::<f64, 1, 11>::zeros(),
    );
    nister::compute_tenth_order_polynomial_from73(&p1, &b31, &mut po1);
    nister::compute_tenth_order_polynomial_from73(&p2, &b32, &mut po2);
    nister::compute_tenth_order_polynomial_from64(&p3, &b33, &mut po3);
    let po = po1 + po2 + po3;
    let roots = Sturm::new(po.as_slice())?.find_roots();
    let mut evec = DMatrix::zeros(9, roots.len());
    for (i, z) in roots.iter().copied().enumerate() {
        let mut coeff = Vector3::new(val(&p1, z) / val(&p3, z), val(&p2, z) / val(&p3, z), z);
        nister::pollish_coefficients(&a, &mut coeff, backend)?;
        evec.set_column(
            i,
            &(coeff[0] * ee.column(0)
                + coeff[1] * ee.column(1)
                + coeff[2] * ee.column(2)
                + ee.column(3)),
        );
    }
    for c in 0..roots.len() {
        let mut norm = 0.0;
        for r in 0..9 {
            norm += evec[(r, c)].powf(2.0);
        }
        norm = norm.sqrt();
        for r in 0..9 {
            evec[(r, c)] /= norm;
        }
    }
    Some(
        (0..roots.len())
            .map(|c| Matrix3::from_fn(|r, k| evec[(3 * r + k, c)]))
            .collect(),
    )
}
pub fn fivept_kneip_main(
    f1: &SMatrix<f64, 3, 5>,
    f2: &SMatrix<f64, 3, 5>,
    backend: &dyn NumericalBackend,
) -> Option<Rotations> {
    let mut gm = SMatrix::<f64, 66, 197>::zeros();
    let mut current = 0;
    for first in 0..5 {
        for second in first + 1..5 {
            let temp1 = f1.column(first) * f1.column(second).transpose();
            let temp2 = f2.column(first) * f2.column(second).transpose();
            for third in second + 1..5 {
                let c1 = std::array::from_fn(|rotation| temp1 * f1[(rotation, third)]);
                let c2 = std::array::from_fn(|rotation| temp2 * f2[(rotation, third)]);
                gm.row_mut(current)
                    .copy_from(&init::init_epncp_row_r(&c1, &c2));
                current += 1;
            }
        }
    }
    init::init_matrix(&mut gm);
    code::compute_basis(&mut gm);
    let mut matrix = SMatrix::<f64, 20, 20>::zeros();
    matrix
        .fixed_view_mut::<10, 20>(0, 0)
        .copy_from(&(-gm.fixed_view::<10, 20>(51, 177)));
    for i in 10..19 {
        matrix[(i, i - 9)] = 1.0;
    }
    matrix[(19, 18)] = 1.0;
    let (constant, right_vectors) = backend.eigenpairs(&dynamic(&matrix))?;
    if constant.len() != 20 || right_vectors.shape() != (20, 20) {
        return None;
    }
    let mut rotations = Vec::new();
    for i in 0..20 {
        if constant[i].im.abs() < 0.1 {
            let mut rotation = Matrix3::from_fn(|leading, quadratic| {
                (right_vectors[(18 - 3 * leading - quadratic, i)] / right_vectors[(19, i)]).re
            });
            for j in 1..3 {
                let norm = rotation.row(j).norm();
                let row = rotation.row(j) / norm;
                rotation.row_mut(j).copy_from(&row);
            }
            if (rotation.determinant() - 1.0).abs() < 0.1 {
                let normal: SMatrix<f64, 3, 5> = SMatrix::from_fn(|leading, quadratic| {
                    f1.column(quadratic)
                        .cross(&(rotation * f2.column(quadratic)))[leading]
                });
                let total = kneip_consistency_error(&normal, &rotation);
                if total < 0.001 {
                    let mut positive = true;
                    'points: for j in 0..5 {
                        for leading in 0..5 {
                            for quadratic in leading + 1..5 {
                                let translation =
                                    normal.column(leading).cross(&normal.column(quadratic));
                                let tv1 = translation.cross(&f1.column(j));
                                let tv2 = translation.cross(&(rotation * f2.column(j)));
                                if tv1.dot(&tv2) < 0.0 {
                                    positive = false;
                                    break 'points;
                                }
                            }
                        }
                    }
                    if positive {
                        rotations.push(rotation);
                    }
                }
            }
        }
    }
    Some(rotations)
}
fn abs(x: f64) -> f64 {
    x.abs()
}
#[derive(Clone, Debug)]
pub struct EigensolverTerms {
    pub xx: Matrix3<f64>,
    pub yy: Matrix3<f64>,
    pub zz: Matrix3<f64>,
    pub xy: Matrix3<f64>,
    pub yz: Matrix3<f64>,
    pub zx: Matrix3<f64>,
}
impl EigensolverTerms {
    #[must_use]
    pub fn zeros() -> Self {
        Self {
            xx: Matrix3::zeros(),
            yy: Matrix3::zeros(),
            zz: Matrix3::zeros(),
            xy: Matrix3::zeros(),
            yz: Matrix3::zeros(),
            zx: Matrix3::zeros(),
        }
    }
    #[must_use]
    pub fn compose(&self, c: &Vector3<f64>) -> Matrix3<f64> {
        eigen::compose_m(
            &eigen::BearingMoments {
                xx_f: &self.xx,
                yy_f: &self.yy,
                zz_f: &self.zz,
                xy_f: &self.xy,
                yz_f: &self.yz,
                zx_f: &self.zx,
            },
            c,
        )
    }
}
pub fn eigensolver_main(
    translation_value: &EigensolverTerms,
    starting_rotation: &Rotation,
    backend: &dyn NumericalBackend,
) -> Option<EigensolverOutput> {
    let linear = rot2cayley(starting_rotation);
    let mut x = DVector::from_column_slice(linear.as_slice());
    let mut residual = |x: &DVector<f64>, out: &mut DVector<f64>| {
        if x.len() != 3 || out.len() != 3 {
            return None;
        }
        let linear = Vector3::from_column_slice(x.as_slice());
        let mut jac = SMatrix::<f64, 1, 3>::zeros();
        eigen::get_smallest_ev_with_jacobian(
            &eigen::BearingMoments {
                xx_f: &translation_value.xx,
                yy_f: &translation_value.yy,
                zz_f: &translation_value.zz,
                xy_f: &translation_value.xy,
                yz_f: &translation_value.yz,
                zx_f: &translation_value.zx,
            },
            &linear,
            &mut jac,
        );
        out.copy_from(&jac.transpose());
        Some(())
    };
    backend.levenberg_marquardt(
        &mut x,
        3,
        &mut residual,
        LmParameters {
            ftol: 0.00005,
            xtol: 1E1 * f64::EPSILON,
            maxfev: 100,
            ..Default::default()
        },
    )?;
    let linear = Vector3::from_column_slice(x.as_slice());
    let (constant, right_vectors) =
        backend.eigenpairs(&dynamic(&translation_value.compose(&linear)))?;
    let (constant, right_vectors) = sorted_eigen::<3>(&constant, &right_vectors)?;
    let translation =
        (constant[0].powf(2.0) + constant[1].powf(2.0)).sqrt() * right_vectors.column(2);
    Some(EigensolverOutput {
        translation,
        rotation: cayley2rot(&linear),
        eigenvalues: constant,
        eigenvectors: right_vectors,
    })
}
fn sorted_eigen<const N: usize>(
    d: &DVector<Complex64>,
    v: &DMatrix<Complex64>,
) -> Option<(SVector<f64, N>, SMatrix<f64, N, N>)> {
    if d.len() != N || v.shape() != (N, N) {
        return None;
    }
    let mut order: Vec<_> = (0..N).collect();
    if d.iter().any(|x| x.re.is_nan()) {
        return None;
    }
    order.sort_by(|&a, &b| d[b].re.partial_cmp(&d[a].re).unwrap());
    Some((
        SVector::from_fn(|r, _| d[order[r]].re),
        SMatrix::from_fn(|r, c| v[(r, order[c])].re),
    ))
}
pub fn sixpt_main(
    l1: &SMatrix<f64, 6, 6>,
    l2: &SMatrix<f64, 6, 6>,
    backend: &dyn NumericalBackend,
) -> Option<Rotations> {
    let l1 = std::array::from_fn(|c| l1.column(c).into_owned());
    let l2 = std::array::from_fn(|c| l2.column(c).into_owned());
    let action = modules2::setup_action(&l1, &l2)?;
    let (_, ev) = backend.eigenpairs(&dynamic(&action))?;
    if ev.shape() != (64, 64) {
        return None;
    }
    Some(
        (0..64)
            .map(|c| {
                let solution = Vector3::from_fn(|r, _| (ev[(60 + r, c)] / ev[(63, c)]).re);
                cayley2rot(&solution).transpose()
            })
            .collect(),
    )
}
#[derive(Clone, Debug)]
pub struct GeTerms {
    pub central: EigensolverTerms,
    pub x1: SMatrix<f64, 3, 9>,
    pub y1: SMatrix<f64, 3, 9>,
    pub z1: SMatrix<f64, 3, 9>,
    pub x2: SMatrix<f64, 3, 9>,
    pub y2: SMatrix<f64, 3, 9>,
    pub z2: SMatrix<f64, 3, 9>,
    pub m11: SMatrix<f64, 9, 9>,
    pub m12: SMatrix<f64, 9, 9>,
    pub m22: SMatrix<f64, 9, 9>,
}
impl GeTerms {
    #[must_use]
    pub fn zeros() -> Self {
        Self {
            central: EigensolverTerms::zeros(),
            x1: SMatrix::zeros(),
            y1: SMatrix::zeros(),
            z1: SMatrix::zeros(),
            x2: SMatrix::zeros(),
            y2: SMatrix::zeros(),
            z2: SMatrix::zeros(),
            m11: SMatrix::zeros(),
            m12: SMatrix::zeros(),
            m22: SMatrix::zeros(),
        }
    }
    #[must_use]
    pub fn compose(&self, c: &Vector3<f64>) -> Matrix4<f64> {
        ge::compose_g(
            &ge::GeneralizedMoments {
                xx_f: &self.central.xx,
                yy_f: &self.central.yy,
                zz_f: &self.central.zz,
                xy_f: &self.central.xy,
                yz_f: &self.central.yz,
                zx_f: &self.central.zx,
                x1_p: &self.x1,
                y1_p: &self.y1,
                z1_p: &self.z1,
                x2_p: &self.x2,
                y2_p: &self.y2,
                z2_p: &self.z2,
                m11_p: &self.m11,
                m12_p: &self.m12,
                m22_p: &self.m22,
            },
            c,
        )
    }
    #[must_use]
    pub fn cost(&self, c: &Vector3<f64>, step: i32) -> f64 {
        ge::get_cost(
            &ge::GeneralizedMoments {
                xx_f: &self.central.xx,
                yy_f: &self.central.yy,
                zz_f: &self.central.zz,
                xy_f: &self.central.xy,
                yz_f: &self.central.yz,
                zx_f: &self.central.zx,
                x1_p: &self.x1,
                y1_p: &self.y1,
                z1_p: &self.z1,
                x2_p: &self.x2,
                y2_p: &self.y2,
                z2_p: &self.z2,
                m11_p: &self.m11,
                m12_p: &self.m12,
                m22_p: &self.m22,
            },
            c,
            step,
        )
    }
    #[must_use]
    pub fn jacobian(&self, c: &Vector3<f64>) -> SMatrix<f64, 1, 3> {
        let mut jac = SMatrix::zeros();
        ge::get_cost_with_jacobian(
            &ge::GeneralizedMoments {
                xx_f: &self.central.xx,
                yy_f: &self.central.yy,
                zz_f: &self.central.zz,
                xy_f: &self.central.xy,
                yz_f: &self.central.yz,
                zx_f: &self.central.zx,
                x1_p: &self.x1,
                y1_p: &self.y1,
                z1_p: &self.z1,
                x2_p: &self.x2,
                y2_p: &self.y2,
                z2_p: &self.z2,
                m11_p: &self.m11,
                m12_p: &self.m12,
                m22_p: &self.m22,
            },
            c,
            &mut jac,
            1,
        );
        jac
    }
    #[must_use]
    pub fn quick_jacobian(&self, c: &Vector3<f64>, current: f64) -> SMatrix<f64, 1, 3> {
        let mut jac = SMatrix::zeros();
        ge::get_quick_jacobian(
            &ge::GeneralizedMoments {
                xx_f: &self.central.xx,
                yy_f: &self.central.yy,
                zz_f: &self.central.zz,
                xy_f: &self.central.xy,
                yz_f: &self.central.yz,
                zx_f: &self.central.zx,
                x1_p: &self.x1,
                y1_p: &self.y1,
                z1_p: &self.z1,
                x2_p: &self.x2,
                y2_p: &self.y2,
                z2_p: &self.z2,
                m11_p: &self.m11,
                m12_p: &self.m12,
                m22_p: &self.m22,
            },
            c,
            current,
            &mut jac,
            1,
        );
        jac
    }
}
fn ge_output(t: &GeTerms, c: &Vector3<f64>, backend: &dyn NumericalBackend) -> Option<GeOutput> {
    let (d, v) = backend.eigenpairs(&dynamic(&t.compose(c)))?;
    let (d, v) = sorted_eigen::<4>(&d, &v)?;
    let factor = v[(3, 3)];
    let translation = (1.0 / factor) * v.column(3);
    Some(GeOutput {
        translation,
        rotation: cayley2rot(c),
        eigenvalues: d,
        eigenvectors: v,
    })
}
pub fn ge_main(
    t: &GeTerms,
    start: &Vector3<f64>,
    backend: &dyn NumericalBackend,
) -> Option<GeOutput> {
    let mut x = DVector::from_column_slice(start.as_slice());
    let mut residual = |x: &DVector<f64>, out: &mut DVector<f64>| {
        if x.len() != 3 || out.len() != 3 {
            return None;
        }
        let c = Vector3::from_column_slice(x.as_slice());
        out.copy_from(&t.jacobian(&c).transpose());
        Some(())
    };
    backend.levenberg_marquardt(
        &mut x,
        3,
        &mut residual,
        LmParameters {
            ftol: 0.000_001,
            xtol: 1E1 * f64::EPSILON,
            maxfev: 100,
            ..Default::default()
        },
    )?;
    ge_output(t, &Vector3::from_column_slice(x.as_slice()), backend)
}
pub fn ge_main2(
    t: &GeTerms,
    start: &Vector3<f64>,
    backend: &dyn NumericalBackend,
) -> Option<GeOutput> {
    let mut amplitude = 0.3;
    let (mut found, mut trial) = (false, 0);
    let mut c = *start;
    while !found && trial < 5 {
        if trial > 2 {
            amplitude = 0.6;
        }
        c = *start;
        if trial != 0 {
            for j in 0..3 {
                c[j] += (f64::from(rand::rng().random_range(0..=i32::MAX)) / f64::from(i32::MAX)
                    - 0.5)
                    * 2.0
                    * amplitude;
            }
        }
        let mut lambda = 0.01;
        let mut iterations = 0;
        let mut smallest = t.cost(&c, 1);
        while iterations < 50 {
            let jac = t.quick_jacobian(&c, smallest);
            let norm = (jac[0].powf(2.0) + jac[1].powf(2.0) + jac[2].powf(2.0)).sqrt();
            let normalized = (1.0 / norm) * jac.transpose();
            let mut sample = c - lambda * normalized;
            let mut ev = t.cost(&sample, 1);
            if iterations == 0 {
                while ev < smallest {
                    smallest = ev;
                    if lambda * 2.0 > 0.08 {
                        break;
                    }
                    lambda *= 2.0;
                    sample = c - lambda * normalized;
                    ev = t.cost(&sample, 1);
                }
            }
            while ev > smallest {
                lambda /= 2.0;
                sample = c - lambda * normalized;
                ev = t.cost(&sample, 1);
            }
            c = sample;
            smallest = ev;
            if lambda < 0.00001 {
                break;
            }
            iterations += 1;
        }
        if c.norm() < 0.01 {
            if t.cost(&c, 0) > 0.001 {
                trial += 1;
            } else {
                found = true;
            }
        } else {
            found = true;
        }
    }
    ge_output(t, &c, backend)
}
pub fn ge_plot(t: &GeTerms, output: &mut GeOutput) {
    let c = rot2cayley(&output.rotation);
    ge::get_ev(
        &ge::GeneralizedMoments {
            xx_f: &t.central.xx,
            yy_f: &t.central.yy,
            zz_f: &t.central.zz,
            xy_f: &t.central.xy,
            yz_f: &t.central.yz,
            zx_f: &t.central.zx,
            x1_p: &t.x1,
            y1_p: &t.y1,
            z1_p: &t.z1,
            x2_p: &t.x2,
            y2_p: &t.y2,
            z2_p: &t.z2,
            m11_p: &t.m11,
            m12_p: &t.m12,
            m22_p: &t.m22,
        },
        &c,
        &mut output.eigenvalues,
    );
    output.eigenvectors = t.compose(&c);
}

fn kneip_consistency_error(normal: &SMatrix<f64, 3, 5>, rotation: &Matrix3<f64>) -> f64 {
    let mut total = 0.0;
    for leading in 0..5 {
        for quadratic in leading + 1..5 {
            for linear in quadratic + 1..5 {
                let eval = Matrix3::from_columns(&[
                    normal.column(leading).into_owned(),
                    normal.column(quadratic).into_owned(),
                    normal.column(linear).into_owned(),
                ]);
                total += eval.determinant().abs();
            }
        }
    }
    total += abs(rotation[(0, 0)] * rotation[(0, 0)]
        + rotation[(0, 1)] * rotation[(0, 1)]
        + rotation[(0, 2)] * rotation[(0, 2)]
        - 1.0);
    total += abs(rotation[(0, 0)] * rotation[(1, 0)]
        + rotation[(0, 1)] * rotation[(1, 1)]
        + rotation[(0, 2)] * rotation[(1, 2)]);
    total += abs(rotation[(0, 0)] * rotation[(2, 0)]
        + rotation[(0, 1)] * rotation[(2, 1)]
        + rotation[(0, 2)] * rotation[(2, 2)]);
    total += abs(rotation[(1, 0)] * rotation[(1, 0)]
        + rotation[(1, 1)] * rotation[(1, 1)]
        + rotation[(1, 2)] * rotation[(1, 2)]
        - 1.0);
    total += abs(rotation[(1, 0)] * rotation[(2, 0)]
        + rotation[(1, 1)] * rotation[(2, 1)]
        + rotation[(1, 2)] * rotation[(2, 2)]);
    total += abs(rotation[(2, 0)] * rotation[(2, 0)]
        + rotation[(2, 1)] * rotation[(2, 1)]
        + rotation[(2, 2)] * rotation[(2, 2)]
        - 1.0);
    total += abs(rotation[(1, 0)] * rotation[(2, 1)]
        - rotation[(2, 0)] * rotation[(1, 1)]
        - rotation[(0, 2)]);
    total += abs(rotation[(2, 0)] * rotation[(0, 1)]
        - rotation[(0, 0)] * rotation[(2, 1)]
        - rotation[(1, 2)]);
    total += abs(rotation[(0, 0)] * rotation[(1, 1)]
        - rotation[(1, 0)] * rotation[(0, 1)]
        - rotation[(2, 2)]);

    total
}
