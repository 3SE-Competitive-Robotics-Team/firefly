//! 带形矩阵 LU 分解（论文 O(M) 线性复杂度）。
//!
//! MINCO 系统 M(T)c = b(q) 的 M 是带形矩阵（补充材料 S6 + MINCO 论文 Eq. 53）：
//! 下带宽 lowerBw = 8、上带宽 upperBw = 5（块结构推导）。
//!
//! 参考官方实现（EGO-Planner-v2 `poly_traj_utils.hpp` `BandedSystem`）：
//! - 无部分主元（T > 0 时 M 非奇异且结构稳定，官方注释
//!   "NO PIVOT is applied on the matrix A for efficiency"），
//!   避免行交换导致的带宽扩展，分解 O(lowerBw·upperBw·N)
//! - 存储为 Golub & Van Loan 建议的列主序带布局
//! - Aᵀ 求解直接用 U/L 元素（Aᵀ = UᵀLᵀ，无主元时 P = I），
//!   零额外分解成本

use firefly_error::{Error, ErrorKind, Result};
use nalgebra::DMatrix;

/// 带形矩阵：列主序带存储 `data[(i - j + upper_bw) * n + j]`。
#[derive(Debug, Clone)]
pub struct BandedMatrix {
    n: usize,
    lower_bw: usize,
    upper_bw: usize,
    data: Vec<f64>,
}

impl BandedMatrix {
    pub fn new(n: usize, lower_bw: usize, upper_bw: usize) -> Self {
        Self {
            n,
            lower_bw,
            upper_bw,
            data: vec![0.0; n * (lower_bw + upper_bw + 1)],
        }
    }

    /// 复位到指定形状：形状不变时只清零，复用已有存储。
    pub fn reset(&mut self, n: usize, lower_bw: usize, upper_bw: usize) {
        if self.n == n && self.lower_bw == lower_bw && self.upper_bw == upper_bw {
            self.data.fill(0.0);
        } else {
            *self = Self::new(n, lower_bw, upper_bw);
        }
    }

    fn idx(&self, i: usize, j: usize) -> usize {
        (i + self.upper_bw - j) * self.n + j
    }

    /// 设置元素；带外位置静默忽略。
    pub fn set(&mut self, i: usize, j: usize, v: f64) {
        if i < self.n && j < self.n && i + self.upper_bw >= j && i <= j + self.lower_bw {
            let idx = self.idx(i, j);
            self.data[idx] = v;
        }
    }

    #[cfg(test)]
    fn get(&self, i: usize, j: usize) -> f64 {
        if i < self.n && j < self.n && i + self.upper_bw >= j && i <= j + self.lower_bw {
            self.data[self.idx(i, j)]
        } else {
            0.0
        }
    }
}

/// 带形 LU 分解（无部分主元）：A = LU，乘数/上三角就地存储。
#[derive(Debug)]
pub struct BandedPlu {
    n: usize,
    lower_bw: usize,
    upper_bw: usize,
    data: Vec<f64>,
}

impl BandedPlu {
    /// 带形 LU 分解（官方 `BandedSystem::factorizeLU` 逻辑）。
    /// 生产路径用 [`Self::factorize_owned`]；借用版本留给测试。
    #[cfg(test)]
    pub fn factorize(a: &BandedMatrix) -> Result<Self> {
        Self::factor_in_place(a.n, a.lower_bw, a.upper_bw, a.data.clone())
    }

    /// 同一分解，但接管矩阵存储，省掉一次 `n×(带宽)` 的克隆。
    ///
    /// 调用方（`Minco`）每次时长变化都会新建带形矩阵，克隆纯属浪费。
    pub fn factorize_owned(a: BandedMatrix) -> Result<Self> {
        Self::factor_in_place(a.n, a.lower_bw, a.upper_bw, a.data)
    }

    fn factor_in_place(
        n: usize,
        lower_bw: usize,
        upper_bw: usize,
        mut data: Vec<f64>,
    ) -> Result<Self> {
        factor_data(n, lower_bw, upper_bw, &mut data)?;
        Ok(Self {
            n,
            lower_bw,
            upper_bw,
            data,
        })
    }

    /// 在已有分解上重算，复用 `data` 缓冲；带形维度必须一致。
    pub fn refactorize(&mut self, a: &BandedMatrix) -> Result<()> {
        assert_eq!(self.n, a.n);
        assert_eq!(self.lower_bw, a.lower_bw);
        assert_eq!(self.upper_bw, a.upper_bw);
        self.data.clone_from(&a.data);
        factor_data(self.n, self.lower_bw, self.upper_bw, &mut self.data)
    }

    fn get(&self, i: usize, j: usize) -> f64 {
        if i < self.n && j < self.n && i + self.upper_bw >= j && i <= j + self.lower_bw {
            self.data[(i + self.upper_bw - j) * self.n + j]
        } else {
            0.0
        }
    }

    /// 解 Ax = b（前代 Ly=b + 回代 Ux=y，官方 solve 逻辑）。
    fn solve_into(&self, b: &mut [f64]) {
        let n = self.n;
        for j in 0..n {
            let i_max = (j + self.lower_bw).min(n - 1);
            for i in j + 1..=i_max {
                let l = self.get(i, j);
                if l != 0.0 {
                    b[i] -= l * b[j];
                }
            }
        }
        for j in (0..n).rev() {
            b[j] /= self.get(j, j);
            let i_min = j.saturating_sub(self.upper_bw);
            for i in i_min..j {
                let u = self.get(i, j);
                if u != 0.0 {
                    b[i] -= u * b[j];
                }
            }
        }
    }

    /// 解 Aᵀx = b：Aᵀ = UᵀLᵀ（无主元时 P = I），
    /// 先解 Uᵀy = b 再解 Lᵀx = y（官方 solveAdj 逻辑）。
    fn solve_transpose_into(&self, b: &mut [f64]) {
        let n = self.n;
        for j in 0..n {
            b[j] /= self.get(j, j);
            let i_max = (j + self.upper_bw).min(n - 1);
            for i in j + 1..=i_max {
                let u = self.get(j, i);
                if u != 0.0 {
                    b[i] -= u * b[j];
                }
            }
        }
        for j in (0..n).rev() {
            let i_min = j.saturating_sub(self.lower_bw);
            for i in i_min..j {
                let l = self.get(j, i);
                if l != 0.0 {
                    b[i] -= l * b[j];
                }
            }
        }
    }
}

/// 带形 LU 分解主体：就地改写 `data`，供构造与重分解共用。
fn factor_data(n: usize, lower_bw: usize, upper_bw: usize, data: &mut [f64]) -> Result<()> {
    let get = |data: &[f64], i: usize, j: usize| {
        if i + upper_bw >= j && i <= j + lower_bw {
            data[(i + upper_bw - j) * n + j]
        } else {
            0.0
        }
    };

    for k in 0..n.saturating_sub(1) {
        let i_max = (k + lower_bw).min(n - 1);
        let pivot = get(data, k, k);
        if pivot == 0.0 {
            return Err(Error::new(
                ErrorKind::Convergence,
                "banded matrix is singular",
            ));
        }
        for i in k + 1..=i_max {
            let v = get(data, i, k);
            if v != 0.0 {
                data[(i + upper_bw - k) * n + k] = v / pivot;
            }
        }
        let j_max = (k + upper_bw).min(n - 1);
        for j in k + 1..=j_max {
            let c = get(data, k, j);
            if c != 0.0 {
                for i in k + 1..=i_max {
                    let lik = get(data, i, k);
                    if lik != 0.0 {
                        let idx = (i + upper_bw - j) * n + j;
                        data[idx] -= lik * c;
                    }
                }
            }
        }
    }

    Ok(())
}

/// 带形矩阵求解助手：构造 → 分解 → 求解多右端。
#[derive(Debug)]
pub struct BandedSolver {
    plu: BandedPlu,
}

impl BandedSolver {
    #[cfg(test)]
    pub fn new(a: &BandedMatrix) -> Result<Self> {
        Ok(Self {
            plu: BandedPlu::factorize(a)?,
        })
    }

    /// 接管矩阵存储的构造：`Minco` 的带形矩阵是一次性产物。
    pub fn new_owned(a: BandedMatrix) -> Result<Self> {
        Ok(Self {
            plu: BandedPlu::factorize_owned(a)?,
        })
    }

    /// 方阵维数（右端向量的行数）。
    #[must_use]
    pub fn rows(&self) -> usize {
        self.plu.n
    }

    /// 在已有分解上重算，复用分解缓冲。
    pub fn refactorize(&mut self, a: &BandedMatrix) -> Result<()> {
        self.plu.refactorize(a)
    }

    pub fn solve_in_place(&self, rhs: &mut DMatrix<f64>) {
        assert_eq!(rhs.nrows(), self.plu.n);
        for mut col in rhs.column_iter_mut() {
            self.plu.solve_into(col.as_mut_slice());
        }
    }

    #[cfg(test)]
    pub fn solve_transpose(&self, rhs: &DMatrix<f64>) -> DMatrix<f64> {
        let mut out = rhs.clone();
        self.solve_transpose_in_place(&mut out);
        out
    }

    /// 解 Aᵀx = b 就地；调用方自备缓冲，不覆盖入参。
    pub fn solve_transpose_in_place(&self, rhs: &mut DMatrix<f64>) {
        assert_eq!(rhs.nrows(), self.plu.n);
        for mut col in rhs.column_iter_mut() {
            self.plu.solve_transpose_into(col.as_mut_slice());
        }
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    fn random_banded(n: usize, kl: usize, ku: usize) -> BandedMatrix {
        let mut m = BandedMatrix::new(n, kl, ku);
        let mut seed = 42u64;
        let mut next = || {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5
        };
        for i in 0..n {
            for j in i.saturating_sub(kl)..=(i + ku).min(n - 1) {
                let v = next();
                m.set(i, j, if i == j { v + 5.0 } else { v });
            }
        }
        m
    }

    fn to_dense(m: &BandedMatrix) -> DMatrix<f64> {
        DMatrix::from_fn(m.n, m.n, |i, j| m.get(i, j))
    }

    #[test]
    fn solve_matches_nalgebra() {
        for (n, kl, ku) in [(30, 8, 5), (20, 4, 2), (12, 3, 1)] {
            let banded = random_banded(n, kl, ku);
            let dense = to_dense(&banded);
            let solver = BandedSolver::new(&banded).unwrap();
            let rhs = DMatrix::from_fn(n, 3, |i, c| (i * 3 + c) as f64 * 0.1 + 1.0);
            // 普通求解
            let mut got = rhs.clone();
            solver.solve_in_place(&mut got);
            let expected = dense.clone().lu().solve(&rhs).unwrap();
            let err = (&got - &expected).abs().max();
            assert!(err < 1e-8, "n={n} err={err}");
            // 转置求解
            let got = solver.solve_transpose(&rhs);
            let expected = dense.transpose().lu().solve(&rhs).unwrap();
            let err = (&got - &expected).abs().max();
            assert!(err < 1e-8, "transpose n={n} err={err}");
        }
    }

    #[test]
    fn singular_matrix_reports_error() {
        let mut m = BandedMatrix::new(4, 1, 1);
        m.set(0, 0, 1.0);
        m.set(1, 1, 0.0);
        m.set(2, 2, 1.0);
        m.set(3, 3, 1.0);
        let r = BandedPlu::factorize(&m);
        assert!(r.is_err());
        assert_eq!(r.unwrap_err().kind(), ErrorKind::Convergence);
    }
}
