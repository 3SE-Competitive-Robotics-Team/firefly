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

//! `EPnP` formulation by the EPFL Computer Vision Laboratory authors.

use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, SVector, Vector3};

#[derive(Clone, Debug)]
pub struct Epnp {
    maximum_number_of_correspondences: usize,
    number_of_correspondences: usize,
    pws: Vec<f64>,
    us: Vec<f64>,
    alphas: Vec<f64>,
    pcs: Vec<f64>,
    signs: Vec<i32>,
    cws: [[f64; 3]; 4],
    ccs: [[f64; 3]; 4],
    uc: f64,
    vc: f64,
    fu: f64,
    fv: f64,
}
impl Default for Epnp {
    fn default() -> Self {
        Self::new()
    }
}
impl Epnp {
    #[must_use]
    pub fn new() -> Self {
        Self {
            maximum_number_of_correspondences: 0,
            number_of_correspondences: 0,
            pws: vec![],
            us: vec![],
            alphas: vec![],
            pcs: vec![],
            signs: vec![],
            cws: [[0.0; 3]; 4],
            ccs: [[0.0; 3]; 4],
            uc: 0.0,
            vc: 0.0,
            fu: 1.0,
            fv: 1.0,
        }
    }
    pub fn set_maximum_number_of_correspondences(&mut self, n: usize) {
        if self.maximum_number_of_correspondences < n {
            self.maximum_number_of_correspondences = n;
            self.pws = vec![0.0; 3 * n];
            self.us = vec![0.0; 2 * n];
            self.alphas = vec![0.0; 4 * n];
            self.pcs = vec![0.0; 3 * n];
            self.signs = vec![0; n];
        }
    }
    pub fn reset_correspondences(&mut self) {
        self.number_of_correspondences = 0;
    }
    pub fn add_correspondence(
        &mut self,
        xw: f64,
        yw: f64,
        zw: f64,
        x: f64,
        y: f64,
        z: f64,
    ) -> Option<()> {
        let i = self.number_of_correspondences;
        if i >= self.maximum_number_of_correspondences {
            return None;
        }
        self.pws[3 * i] = xw;
        self.pws[3 * i + 1] = yw;
        self.pws[3 * i + 2] = zw;
        self.us[2 * i] = x / z;
        self.us[2 * i + 1] = y / z;
        self.signs[i] = if z > 0.0 { 1 } else { -1 };
        self.number_of_correspondences += 1;
        Some(())
    }
    pub fn choose_control_points(&mut self) -> Option<()> {
        let n = self.number_of_correspondences;
        if n == 0 {
            return None;
        }
        self.cws[0] = [0.0; 3];
        for i in 0..n {
            for j in 0..3 {
                self.cws[0][j] += self.pws[3 * i + j];
            }
        }
        for j in 0..3 {
            self.cws[0][j] /= n as f64;
        }
        let pw0 = DMatrix::from_fn(n, 3, |i, j| self.pws[3 * i + j] - self.cws[0][j]);
        let svd = (pw0.transpose() * pw0).svd(true, true);
        let d = svd.singular_values;
        let ut = svd.u?.transpose();
        for i in 1..4 {
            let k = (d[i - 1] / n as f64).sqrt();
            for j in 0..3 {
                self.cws[i][j] = self.cws[0][j] + k * ut[(i - 1, j)];
            }
        }
        Some(())
    }
    pub fn compute_barycentric_coordinates(&mut self) -> Option<()> {
        let cc = Matrix3::from_fn(|i, j| self.cws[j + 1][i] - self.cws[0][i]);
        let inv = cc.try_inverse()?;
        for i in 0..self.number_of_correspondences {
            for j in 0..3 {
                self.alphas[4 * i + 1 + j] = inv[(j, 0)] * (self.pws[3 * i] - self.cws[0][0])
                    + inv[(j, 1)] * (self.pws[3 * i + 1] - self.cws[0][1])
                    + inv[(j, 2)] * (self.pws[3 * i + 2] - self.cws[0][2]);
            }
            self.alphas[4 * i] =
                1.0 - self.alphas[4 * i + 1] - self.alphas[4 * i + 2] - self.alphas[4 * i + 3];
        }
        Some(())
    }
    pub fn fill_m(
        &self,
        m: &mut DMatrix<f64>,
        row: usize,
        a: &[f64],
        u: f64,
        v: f64,
    ) -> Option<()> {
        if a.len() < 4 || row + 1 >= m.nrows() || m.ncols() < 12 {
            return None;
        }
        for i in 0..4 {
            m[(row, 3 * i)] = a[i] * self.fu;
            m[(row, 3 * i + 1)] = 0.0;
            m[(row, 3 * i + 2)] = a[i] * (self.uc - u);
            m[(row + 1, 3 * i)] = 0.0;
            m[(row + 1, 3 * i + 1)] = a[i] * self.fv;
            m[(row + 1, 3 * i + 2)] = a[i] * (self.vc - v);
        }
        Some(())
    }
    pub fn compute_ccs(&mut self, betas: &[f64; 4], ut: &DMatrix<f64>) -> Option<()> {
        if ut.nrows() < 12 || ut.ncols() < 12 {
            return None;
        }
        self.ccs = [[0.0; 3]; 4];
        for i in 0..4 {
            for j in 0..4 {
                for k in 0..3 {
                    self.ccs[j][k] += betas[i] * ut[(11 - i, 3 * j + k)];
                }
            }
        }
        Some(())
    }
    pub fn compute_pcs(&mut self) {
        for i in 0..self.number_of_correspondences {
            for j in 0..3 {
                self.pcs[3 * i + j] = self.alphas[4 * i] * self.ccs[0][j]
                    + self.alphas[4 * i + 1] * self.ccs[1][j]
                    + self.alphas[4 * i + 2] * self.ccs[2][j]
                    + self.alphas[4 * i + 3] * self.ccs[3][j];
            }
        }
    }
    pub fn compute_pose(&mut self) -> Option<(f64, Matrix3<f64>, Vector3<f64>)> {
        if self.number_of_correspondences == 0 {
            return None;
        }
        self.choose_control_points()?;
        self.compute_barycentric_coordinates()?;
        let mut m = DMatrix::zeros(2 * self.number_of_correspondences, 12);
        for i in 0..self.number_of_correspondences {
            self.fill_m(
                &mut m,
                2 * i,
                &self.alphas[4 * i..4 * i + 4],
                self.us[2 * i],
                self.us[2 * i + 1],
            )?;
        }
        let ut = (m.transpose() * m).svd(true, true).u?.transpose();
        let l = Self::compute_l_6x10(&ut)?;
        let rho = self.compute_rho();
        let mut results = Vec::with_capacity(3);
        for approximation in 1..=3 {
            let mut betas = Self::find_betas(&l, &rho, approximation)?;
            Self::gauss_newton(&l, &rho, &mut betas)?;
            results.push(self.compute_r_and_t(&ut, &betas)?);
        }
        let mut n = 0;
        if results[1].0 < results[0].0 {
            n = 1;
        }
        if results[2].0 < results[n].0 {
            n = 2;
        }
        Some(results[n])
    }
    pub fn copy_r_and_t(
        r: &Matrix3<f64>,
        t: &Vector3<f64>,
        r_dst: &mut Matrix3<f64>,
        t_dst: &mut Vector3<f64>,
    ) {
        *r_dst = *r;
        *t_dst = *t;
    }
    #[must_use]
    pub fn dist2(p1: &[f64; 3], p2: &[f64; 3]) -> f64 {
        (p1[0] - p2[0]) * (p1[0] - p2[0])
            + (p1[1] - p2[1]) * (p1[1] - p2[1])
            + (p1[2] - p2[2]) * (p1[2] - p2[2])
    }
    #[must_use]
    pub fn dot(v1: &[f64], v2: &[f64]) -> f64 {
        v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
    }
    #[must_use]
    pub fn reprojection_error(&self, r: &Matrix3<f64>, t: &Vector3<f64>) -> f64 {
        let mut sum2 = 0.0;
        for i in 0..self.number_of_correspondences {
            let pw = &self.pws[3 * i..3 * i + 3];
            let xc = r[(0, 0)] * pw[0] + r[(0, 1)] * pw[1] + r[(0, 2)] * pw[2] + t[0];
            let yc = r[(1, 0)] * pw[0] + r[(1, 1)] * pw[1] + r[(1, 2)] * pw[2] + t[1];
            let inv_zc = 1.0 / (r[(2, 0)] * pw[0] + r[(2, 1)] * pw[1] + r[(2, 2)] * pw[2] + t[2]);
            let ue = self.uc + self.fu * xc * inv_zc;
            let ve = self.vc + self.fv * yc * inv_zc;
            let u = self.us[2 * i];
            let v = self.us[2 * i + 1];
            sum2 += ((u - ue) * (u - ue) + (v - ve) * (v - ve)).sqrt();
        }
        sum2 / self.number_of_correspondences as f64
    }
    #[must_use]
    pub fn estimate_r_and_t(&self) -> Option<(Matrix3<f64>, Vector3<f64>)> {
        let dimension = self.number_of_correspondences;
        let mut pc0 = [0.0; 3];
        let mut pw0 = [0.0; 3];
        for i in 0..dimension {
            for j in 0..3 {
                pc0[j] += self.pcs[3 * i + j];
                pw0[j] += self.pws[3 * i + j];
            }
        }
        for j in 0..3 {
            pc0[j] /= dimension as f64;
            pw0[j] /= dimension as f64;
        }
        let mut abt = Matrix3::<f64>::zeros();
        for i in 0..dimension {
            for j in 0..3 {
                abt[(j, 0)] += (self.pcs[3 * i + j] - pc0[j]) * (self.pws[3 * i] - pw0[0]);
                abt[(j, 1)] += (self.pcs[3 * i + j] - pc0[j]) * (self.pws[3 * i + 1] - pw0[1]);
                abt[(j, 2)] += (self.pcs[3 * i + j] - pc0[j]) * (self.pws[3 * i + 2] - pw0[2]);
            }
        }
        let svd = abt.svd(true, true);
        let left_vectors = svd.u?;
        let right_vectors = svd.v_t?.transpose();
        let mut rotation = left_vectors * right_vectors.transpose();
        let det = rotation[(0, 0)] * rotation[(1, 1)] * rotation[(2, 2)]
            + rotation[(0, 1)] * rotation[(1, 2)] * rotation[(2, 0)]
            + rotation[(0, 2)] * rotation[(1, 0)] * rotation[(2, 1)]
            - rotation[(0, 2)] * rotation[(1, 1)] * rotation[(2, 0)]
            - rotation[(0, 1)] * rotation[(1, 0)] * rotation[(2, 2)]
            - rotation[(0, 0)] * rotation[(1, 2)] * rotation[(2, 1)];
        if det < 0.0 {
            let mut vp = right_vectors;
            vp.column_mut(2).neg_mut();
            rotation = left_vectors * vp.transpose();
        }
        let translation = Vector3::from_fn(|i, _| {
            pc0[i]
                - (rotation[(i, 0)] * pw0[0]
                    + rotation[(i, 1)] * pw0[1]
                    + rotation[(i, 2)] * pw0[2])
        });
        Some((rotation, translation))
    }
    pub fn print_pose(r: &Matrix3<f64>, t: &Vector3<f64>) {
        for i in 0..3 {
            log::info!("{} {} {} {}", r[(i, 0)], r[(i, 1)], r[(i, 2)], t[i]);
        }
    }
    pub fn solve_for_sign(&mut self) -> Option<()> {
        if self.number_of_correspondences == 0 {
            return None;
        }
        if (self.pcs[2] < 0.0 && self.signs[0] > 0) || (self.pcs[2] > 0.0 && self.signs[0] < 0) {
            for i in 0..4 {
                for j in 0..3 {
                    self.ccs[i][j] = -self.ccs[i][j];
                }
            }
            for i in 0..self.number_of_correspondences {
                self.pcs[3 * i] = -self.pcs[3 * i];
                self.pcs[3 * i + 1] = -self.pcs[3 * i + 1];
                self.pcs[3 * i + 2] = -self.pcs[3 * i + 2];
            }
        }
        Some(())
    }
    pub fn compute_r_and_t(
        &mut self,
        ut: &DMatrix<f64>,
        betas: &[f64; 4],
    ) -> Option<(f64, Matrix3<f64>, Vector3<f64>)> {
        self.compute_ccs(betas, ut)?;
        self.compute_pcs();
        self.solve_for_sign()?;
        let (r, t) = self.estimate_r_and_t()?;
        Some((self.reprojection_error(&r, &t), r, t))
    }
    fn find_betas(
        l: &SMatrix<f64, 6, 10>,
        rho: &SVector<f64, 6>,
        approximation: usize,
    ) -> Option<[f64; 4]> {
        let columns: &[usize] = match approximation {
            1 => &[0, 1, 3, 6],
            2 => &[0, 1, 2],
            3 => &[0, 1, 2, 3, 4],
            _ => return None,
        };
        let m = DMatrix::from_fn(6, columns.len(), |i, j| l[(i, columns[j])]);
        let svd = m.svd(true, true);
        let epsilon = f64::EPSILON * svd.singular_values.len() as f64 * svd.singular_values[0];
        let b = svd
            .solve(&DVector::from_column_slice(rho.as_slice()), epsilon)
            .ok()?;
        let mut betas = [0.0; 4];
        if approximation == 1 {
            if b[0] < 0.0 {
                betas[0] = (-b[0]).sqrt();
                for i in 1..4 {
                    betas[i] = -b[i] / betas[0];
                }
            } else {
                betas[0] = b[0].sqrt();
                for i in 1..4 {
                    betas[i] = b[i] / betas[0];
                }
            }
        } else {
            if b[0] < 0.0 {
                betas[0] = (-b[0]).sqrt();
                betas[1] = if b[2] < 0.0 { (-b[2]).sqrt() } else { 0.0 }
            } else {
                betas[0] = b[0].sqrt();
                betas[1] = if b[2] > 0.0 { b[2].sqrt() } else { 0.0 }
            }
            if b[1] < 0.0 {
                betas[0] = -betas[0];
            }
            if approximation == 3 {
                betas[2] = b[3] / betas[0];
            }
        }
        Some(betas)
    }
    #[must_use]
    pub fn find_betas_approx_1(l: &SMatrix<f64, 6, 10>, rho: &SVector<f64, 6>) -> Option<[f64; 4]> {
        Self::find_betas(l, rho, 1)
    }
    #[must_use]
    pub fn find_betas_approx_2(l: &SMatrix<f64, 6, 10>, rho: &SVector<f64, 6>) -> Option<[f64; 4]> {
        Self::find_betas(l, rho, 2)
    }
    #[must_use]
    pub fn find_betas_approx_3(l: &SMatrix<f64, 6, 10>, rho: &SVector<f64, 6>) -> Option<[f64; 4]> {
        Self::find_betas(l, rho, 3)
    }
    #[must_use]
    pub fn compute_l_6x10(ut: &DMatrix<f64>) -> Option<SMatrix<f64, 6, 10>> {
        if ut.nrows() < 12 || ut.ncols() < 12 {
            return None;
        }
        let mut dv = [[[0.0; 3]; 6]; 4];
        for i in 0..4 {
            let (mut a, mut b) = (0, 1);
            for difference in &mut dv[i] {
                for (k, coordinate) in difference.iter_mut().enumerate() {
                    *coordinate = ut[(11 - i, 3 * a + k)] - ut[(11 - i, 3 * b + k)];
                }
                b += 1;
                if b > 3 {
                    a += 1;
                    b = a + 1;
                }
            }
        }
        let mut l = SMatrix::zeros();
        for i in 0..6 {
            l[(i, 0)] = Self::dot(&dv[0][i], &dv[0][i]);
            l[(i, 1)] = 2.0 * Self::dot(&dv[0][i], &dv[1][i]);
            l[(i, 2)] = Self::dot(&dv[1][i], &dv[1][i]);
            l[(i, 3)] = 2.0 * Self::dot(&dv[0][i], &dv[2][i]);
            l[(i, 4)] = 2.0 * Self::dot(&dv[1][i], &dv[2][i]);
            l[(i, 5)] = Self::dot(&dv[2][i], &dv[2][i]);
            l[(i, 6)] = 2.0 * Self::dot(&dv[0][i], &dv[3][i]);
            l[(i, 7)] = 2.0 * Self::dot(&dv[1][i], &dv[3][i]);
            l[(i, 8)] = 2.0 * Self::dot(&dv[2][i], &dv[3][i]);
            l[(i, 9)] = Self::dot(&dv[3][i], &dv[3][i]);
        }
        Some(l)
    }
    #[must_use]
    pub fn compute_rho(&self) -> SVector<f64, 6> {
        SVector::from_row_slice(&[
            Self::dist2(&self.cws[0], &self.cws[1]),
            Self::dist2(&self.cws[0], &self.cws[2]),
            Self::dist2(&self.cws[0], &self.cws[3]),
            Self::dist2(&self.cws[1], &self.cws[2]),
            Self::dist2(&self.cws[1], &self.cws[3]),
            Self::dist2(&self.cws[2], &self.cws[3]),
        ])
    }
    #[must_use]
    pub fn compute_a_and_b_gauss_newton(
        l: &SMatrix<f64, 6, 10>,
        rho: &SVector<f64, 6>,
        betas: &[f64; 4],
    ) -> (SMatrix<f64, 6, 4>, SVector<f64, 6>) {
        let mut a = SMatrix::zeros();
        let mut b = SVector::zeros();
        for i in 0..6 {
            a[(i, 0)] = 2.0 * l[(i, 0)] * betas[0]
                + l[(i, 1)] * betas[1]
                + l[(i, 3)] * betas[2]
                + l[(i, 6)] * betas[3];
            a[(i, 1)] = l[(i, 1)] * betas[0]
                + 2.0 * l[(i, 2)] * betas[1]
                + l[(i, 4)] * betas[2]
                + l[(i, 7)] * betas[3];
            a[(i, 2)] = l[(i, 3)] * betas[0]
                + l[(i, 4)] * betas[1]
                + 2.0 * l[(i, 5)] * betas[2]
                + l[(i, 8)] * betas[3];
            a[(i, 3)] = l[(i, 6)] * betas[0]
                + l[(i, 7)] * betas[1]
                + l[(i, 8)] * betas[2]
                + 2.0 * l[(i, 9)] * betas[3];

            b[(i, 0)] = rho[i]
                - (l[(i, 0)] * betas[0] * betas[0]
                    + l[(i, 1)] * betas[0] * betas[1]
                    + l[(i, 2)] * betas[1] * betas[1]
                    + l[(i, 3)] * betas[0] * betas[2]
                    + l[(i, 4)] * betas[1] * betas[2]
                    + l[(i, 5)] * betas[2] * betas[2]
                    + l[(i, 6)] * betas[0] * betas[3]
                    + l[(i, 7)] * betas[1] * betas[3]
                    + l[(i, 8)] * betas[2] * betas[3]
                    + l[(i, 9)] * betas[3] * betas[3]);
        }

        (a, b)
    }
    pub fn gauss_newton(
        l: &SMatrix<f64, 6, 10>,
        rho: &SVector<f64, 6>,
        betas: &mut [f64; 4],
    ) -> Option<()> {
        for _ in 0..5 {
            let (mut a, mut b) = Self::compute_a_and_b_gauss_newton(l, rho, betas);
            let x = Self::qr_solve(&mut a, &mut b)?;
            for i in 0..4 {
                betas[i] += x[i];
            }
        }
        Some(())
    }
    pub fn qr_solve(
        a_orig: &mut SMatrix<f64, 6, 4>,
        b: &mut SVector<f64, 6>,
    ) -> Option<SVector<f64, 4>> {
        let mut a = a_orig.transpose();
        let data = a.as_mut_slice();
        let (mut a1, mut a2) = ([0.0; 6], [0.0; 6]);
        let (nr, nc) = (6, 4);
        let mut kk = 0;
        for k in 0..nc {
            let mut ik = kk;
            let mut eta = data[ik].abs();
            for _ in k + 1..nr {
                let elt = data[ik].abs();
                if eta < elt {
                    eta = elt;
                }
                ik += nc;
            }
            if eta == 0.0 {
                return None;
            }
            let mut ik = kk;
            let mut sum = 0.0;
            let inv_eta = 1.0 / eta;
            for _ in k..nr {
                data[ik] *= inv_eta;
                sum += data[ik] * data[ik];
                ik += nc;
            }
            let mut sigma = sum.sqrt();
            if data[kk] < 0.0 {
                sigma = -sigma;
            }
            data[kk] += sigma;
            a1[k] = sigma * data[kk];
            a2[k] = -eta * sigma;
            for j in k + 1..nc {
                let mut ik = kk;
                let mut sum = 0.0;
                for _ in k..nr {
                    sum += data[ik] * data[ik + j - k];
                    ik += nc;
                }
                let tau = sum / a1[k];
                let mut ik = kk;
                for _ in k..nr {
                    data[ik + j - k] -= tau * data[ik];
                    ik += nc;
                }
            }
            kk += nc + 1;
        }
        let mut jj = 0;
        for (j, divisor) in a1.iter().enumerate().take(nc) {
            let mut ij = jj;
            let mut tau = 0.0;
            for i in j..nr {
                tau += data[ij] * b[i];
                ij += nc;
            }
            tau /= *divisor;
            let mut ij = jj;
            for i in j..nr {
                b[i] -= tau * data[ij];
                ij += nc;
            }
            jj += nc + 1;
        }
        let mut x = SVector::<f64, 4>::zeros();
        x[nc - 1] = b[nc - 1] / a2[nc - 1];
        for i in (0..nc - 1).rev() {
            let first = i * nc + i + 1;
            let mut sum = 0.0;
            for (offset, j) in (i + 1..nc).enumerate() {
                sum += data[first + offset] * x[j];
            }
            x[i] = (b[i] - sum) / a2[i];
        }
        Some(x)
    }
    #[must_use]
    pub fn mat_to_quat(r: &Matrix3<f64>) -> [f64; 4] {
        let tr = r[(0, 0)] + r[(1, 1)] + r[(2, 2)];
        let mut q = [0.0; 4];

        let n4 = if tr > 0.0 {
            q[0] = r[(1, 2)] - r[(2, 1)];
            q[1] = r[(2, 0)] - r[(0, 2)];
            q[2] = r[(0, 1)] - r[(1, 0)];
            q[3] = tr + 1.0;
            q[3]
        } else if r[(0, 0)] > r[(1, 1)] && r[(0, 0)] > r[(2, 2)] {
            q[0] = 1.0 + r[(0, 0)] - r[(1, 1)] - r[(2, 2)];
            q[1] = r[(1, 0)] + r[(0, 1)];
            q[2] = r[(2, 0)] + r[(0, 2)];
            q[3] = r[(1, 2)] - r[(2, 1)];
            q[0]
        } else if r[(1, 1)] > r[(2, 2)] {
            q[0] = r[(1, 0)] + r[(0, 1)];
            q[1] = 1.0 + r[(1, 1)] - r[(0, 0)] - r[(2, 2)];
            q[2] = r[(2, 1)] + r[(1, 2)];
            q[3] = r[(2, 0)] - r[(0, 2)];
            q[1]
        } else {
            q[0] = r[(2, 0)] + r[(0, 2)];
            q[1] = r[(2, 1)] + r[(1, 2)];
            q[2] = 1.0 + r[(2, 2)] - r[(0, 0)] - r[(1, 1)];
            q[3] = r[(0, 1)] - r[(1, 0)];
            q[2]
        };
        let scale = 0.5 / n4.sqrt();
        for v in &mut q {
            *v *= scale;
        }
        q
    }
    #[must_use]
    pub fn relative_error(
        r_true: &Matrix3<f64>,
        t_true: &Vector3<f64>,
        r_est: &Matrix3<f64>,
        t_est: &Vector3<f64>,
    ) -> (f64, f64) {
        let a = Self::mat_to_quat(r_true);
        let b = Self::mat_to_quat(r_est);
        let norm = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]).sqrt();
        let err1 = ((a[0] - b[0]) * (a[0] - b[0])
            + (a[1] - b[1]) * (a[1] - b[1])
            + (a[2] - b[2]) * (a[2] - b[2])
            + (a[3] - b[3]) * (a[3] - b[3]))
            .sqrt()
            / norm;
        let err2 = ((a[0] + b[0]) * (a[0] + b[0])
            + (a[1] + b[1]) * (a[1] + b[1])
            + (a[2] + b[2]) * (a[2] + b[2])
            + (a[3] + b[3]) * (a[3] + b[3]))
            .sqrt()
            / norm;
        let rotation = if err2 < err1 { err2 } else { err1 };
        let translation = ((t_true[0] - t_est[0]) * (t_true[0] - t_est[0])
            + (t_true[1] - t_est[1]) * (t_true[1] - t_est[1])
            + (t_true[2] - t_est[2]) * (t_true[2] - t_est[2]))
            .sqrt()
            / (t_true[0] * t_true[0] + t_true[1] * t_true[1] + t_true[2] * t_true[2]).sqrt();
        (rotation, translation)
    }
}
