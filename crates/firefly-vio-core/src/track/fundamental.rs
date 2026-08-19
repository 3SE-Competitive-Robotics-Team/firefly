//! 基础矩阵 RANSAC（自实现，对照 `OpenCV modules/calib3d/src/fundam.cpp`、
//! `ptsetreg.cpp` 的 `findFundamentalMat(..., FM_RANSAC, param, confidence)`）。
//!
//! [`ransac_fundamental`] 用归一化 8 点法拟合基础矩阵 `F`，并以 **Sampson 距离**
//! 作为残差做 RANSAC（MSAC）离群剔除外点，迭代次数随置信度自适应增加。
//!
//! OpenVINS `TrackKLT.cpp` 第 873 行调用：
//! ```cpp
//! cv::findFundamentalMat(pts0_n, pts1_n, cv::FM_RANSAC, 2.0/max_focallength, 0.999, mask_rsc);
//! ```
//! 其中 `pts0_n`/`pts1_n` 为去畸变归一化坐标，阈值 `2.0/max_focallength`、
//! 置信度 `0.999`（`max_focallength` 为两相机焦距最大值）。

#![allow(
    clippy::doc_markdown,
    clippy::many_single_char_names,
    clippy::similar_names
)]
use nalgebra::Vector2;

/// 8 点最小求解所需的最小点对数量。
pub const MIN_SAMPLES: usize = 8;

/// RANSAC 的基础矩阵拟合。
///
/// - `pts0`/`pts1`：归一化坐标对应点；
/// - `threshold`：内点阈值（Sampson 距离，OpenVINS 传入 `2.0/max_focallength`）；
/// - `confidence`：期望置信度（如 `0.999`）；
/// - 返回 `(在点数掩码, 最佳 F)`；`inlier_mask[i]` 表示第 i 对点是内点。
///
/// 计分用 MSAC：内点以 Sampson 平方距离累加，外点给固定 `threshold²` 惩罚，
/// 内点数最大者胜出；迭代次数按 `K = log(1-c)/log(1-w⁸)` 自适应更新
/// （`w` = 内点占比），上限 1000 次，与 OpenCV `RANSACUpdateNumIters` 一致。
#[must_use]
pub fn ransac_fundamental(
    pts0: &[Vector2<f64>],
    pts1: &[Vector2<f64>],
    threshold: f64,
    confidence: f64,
) -> (Vec<bool>, Option<[f64; 9]>) {
    let n = pts0.len().min(pts1.len());
    let mut mask = vec![false; n];
    if n < MIN_SAMPLES {
        return (mask, None);
    }

    let threshold_sq = threshold * threshold;
    let mut rng = Rng::seed(0x9E37_79B9_7F4A_7C15);
    let mut best_f: Option<[f64; 9]> = None;
    let mut best_score = f64::INFINITY;
    let mut best_inliers = 0usize;
    let mut iterations = 1000usize;

    while iterations > 0 {
        iterations -= 1;
        // 采样 8 对不重复索引
        let Some(sample) = sample_indices(&mut rng, n, MIN_SAMPLES) else {
            break;
        };
        let s0 = sample.iter().map(|&i| pts0[i]).collect::<Vec<_>>();
        let s1 = sample.iter().map(|&i| pts1[i]).collect::<Vec<_>>();
        let Some(f) = estimate_fundamental_8point(&s0, &s1) else {
            continue;
        };

        let mut inliers = vec![false; n];
        let mut score = 0.0f64;
        let mut inlier_count = 0usize;
        for i in 0..n {
            let e = sampson_distance(&f, pts0[i], pts1[i]);
            if e < threshold_sq {
                inliers[i] = true;
                inlier_count += 1;
                score += e;
            } else {
                score += threshold_sq;
            }
        }
        if inlier_count > best_inliers || (inlier_count == best_inliers && score < best_score) {
            best_inliers = inlier_count;
            best_score = score;
            best_f = Some(f);
            mask = inliers;
        }

        // 自适应迭代次数：直接替换剩余迭代数（OpenCV `RANSACUpdateNumIters` 语义，
        // 内点占比高时 K 变小 → 提前终止）；`None` 仅当 0 个点（不会发生）。
        match update_iterations(confidence, best_inliers, n) {
            Some(it) => iterations = it.min(iterations),
            None => break,
        }
        if best_inliers == n {
            break;
        }
    }

    (mask, best_f)
}

/// 计算所需的 RANSAC 迭代次数（`RANSACUpdateNumIters` 语义）。
///
/// ```text
/// w = 内点占比 = inliers / n
/// K = ceil( |ln(1 - confidence)| / |ln(1 - w⁸)| )，夹取到 [1, 1000]
/// ```
/// 内点占比低时 `K` 超过上限 → 返回 1000（维持大迭代数继续找）；内点占比高时
/// `K` 小 → 提前终止。`None` 仅当 `n == 0`。
#[must_use]
fn update_iterations(confidence: f64, inliers: usize, n: usize) -> Option<usize> {
    if n == 0 {
        return None;
    }
    let w = inliers as f64 / n as f64;
    if w <= 0.0 {
        // 尚无内点：无法收敛出置信度，维持最大迭代上限继续尝试。
        return Some(1000);
    }
    let denom = 1.0 - w.powf(MIN_SAMPLES as f64);
    if denom <= 1e-12 {
        return Some(1);
    }
    let it = ((1.0 - confidence).ln() / denom.ln()).abs().ceil();
    Some(it.clamp(1.0, 1000.0) as usize)
}

/// 确定性采样：从 `[0, n)` 中取 `k` 个不重复索引。
#[must_use]
fn sample_indices(rng: &mut Rng, n: usize, k: usize) -> Option<Vec<usize>> {
    if n < k {
        return None;
    }
    let mut chosen = Vec::with_capacity(k);
    let mut used = vec![false; n];
    while chosen.len() < k {
        if !used.is_empty() {
            let mut guard = 0;
            loop {
                let idx = rng.next() % n;
                if !used[idx] {
                    used[idx] = true;
                    chosen.push(idx);
                    break;
                }
                guard += 1;
                if guard > 10_000 {
                    return None; // 极端拥塞保护
                }
            }
        }
    }
    Some(chosen)
}

/// 极小线性同余伪随机数生成器（测试与采样用，确定性、非加密）。
struct Rng(u64);

impl Rng {
    fn seed(s: u64) -> Self {
        Self(s)
    }

    #[allow(clippy::unreadable_literal)] // LCG 乘法/加法常量，按惯例不带分隔符
    fn next(&mut self) -> usize {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 33) as usize
    }
}

/// Sampson 距离（一阶几何残差）。
///
/// 对对应点对 `(x₁, x₂)` 与 `F`：
/// ```text
/// ε = (x₂ᵀ F x₁)² / ( (Fx₁)₁² + (Fx₁)₂² + (Fᵀx₂)₁² + (Fᵀx₂)₂² )
/// ```
/// 返回该平方距离（OpenCV RANSAC 阈值以该平方距离比较）。
#[must_use]
pub fn sampson_distance(f: &[f64; 9], x1: Vector2<f64>, x2: Vector2<f64>) -> f64 {
    let fx = [
        f[0] * x1.x + f[1] * x1.y + f[2],
        f[3] * x1.x + f[4] * x1.y + f[5],
        f[6] * x1.x + f[7] * x1.y + f[8],
    ];
    let ftx2 = [
        f[0] * x2.x + f[3] * x2.y + f[6],
        f[1] * x2.x + f[4] * x2.y + f[7],
        f[2] * x2.x + f[5] * x2.y + f[8],
    ];
    let num = fx[0] * x2.x + fx[1] * x2.y + fx[2];
    let denom = fx[0] * fx[0] + fx[1] * fx[1] + ftx2[0] * ftx2[0] + ftx2[1] * ftx2[1];
    if denom < 1e-12 {
        return f64::MAX;
    }
    num * num / denom
}

/// 归一化 8 点法：由两组归一化坐标对求基础矩阵 `F`（行主序 3×3）。
///
/// 返回 `Option<[f64; 9]>`；点数不足或退化输入返回 `None`。
/// 流程：Hartley 归一化 → 构造 9×9 齐次方程组，用 `AᵀA` 的最小特征向量
/// （SVD 的 `V` 最后一列）求解 → 施加强制 rank-2 约束 → 还原归一化。
#[must_use]
pub fn estimate_fundamental_8point(
    pts0: &[Vector2<f64>],
    pts1: &[Vector2<f64>],
) -> Option<[f64; 9]> {
    if pts0.len() < 8 || pts1.len() < 8 || pts0.len() != pts1.len() {
        return None;
    }
    let (m0, s0) = normalize_points(pts0);
    let (m1, s1) = normalize_points(pts1);
    // 归一化变换 N：x' = N·x = [[s,0,-s·mx],[0,s,-s·my],[0,0,1]]·x
    let norm0 = [
        [s0, 0.0, -s0 * m0.x],
        [0.0, s0, -s0 * m0.y],
        [0.0, 0.0, 1.0],
    ];
    let norm1 = [
        [s1, 0.0, -s1 * m1.x],
        [0.0, s1, -s1 * m1.y],
        [0.0, 0.0, 1.0],
    ];

    let n = pts0.len();
    // 齐次坐标
    let mut n0 = Vec::with_capacity(n);
    let mut n1 = Vec::with_capacity(n);
    for i in 0..n {
        let x0 = (pts0[i].x - m0.x) * s0;
        let y0 = (pts0[i].y - m0.y) * s0;
        let x1 = (pts1[i].x - m1.x) * s1;
        let y1 = (pts1[i].y - m1.y) * s1;
        n0.push([x0, y0, 1.0]);
        n1.push([x1, y1, 1.0]);
    }

    // 构造 A（n×9），A行 = [x1x0, x1y0, x1, y1x0, y1y0, y1, x0, y0, 1]
    let mut a = vec![0.0f64; n * 9];
    for i in 0..n {
        let (x0, y0) = (n0[i][0], n0[i][1]);
        let (x1, y1) = (n1[i][0], n1[i][1]);
        let row = &mut a[i * 9..i * 9 + 9];
        row[0] = x1 * x0;
        row[1] = x1 * y0;
        row[2] = x1;
        row[3] = y1 * x0;
        row[4] = y1 * y0;
        row[5] = y1;
        row[6] = x0;
        row[7] = y0;
        row[8] = 1.0;
    }

    // 最小特征向量 of AᵀA（即 SVD 的 V 最后一列）；m 采用带位移的逆幂迭代
    let ata = ata_from_a(&a, n, 9);
    let v = smallest_eigenvector(&ata, 9)?;
    let mut f = [[0.0f64; 3]; 3];
    for (row, chunk) in f.iter_mut().zip(v.chunks_exact(3)) {
        row.copy_from_slice(chunk);
    }

    // rank-2 约束：f 的最小奇异方向投影为零
    let f2 = enforce_rank2(&f);

    // 还原归一化：x1'ᵀ·f2·x0' = 0 且 x0'=N0·x0、x1'=N1·x1
    // → x1ᵀ·(N1ᵀ·f2·N0)·x0 = 0 → F = N1ᵀ·f2·N0
    let tmp = mul3(&f2, &norm0);
    let f_final = mul3(&transpose3(&norm1), &tmp);
    Some(flatten3(&f_final))
}

/// 计算 `AᵀA`（对称，`cols×cols`）。`a` 为 `rows×cols` 行主序。
#[must_use]
fn ata_from_a(a: &[f64], rows: usize, cols: usize) -> Vec<f64> {
    let mut ata = vec![0.0f64; cols * cols];
    for r in 0..rows {
        for i in 0..cols {
            let ai = a[r * cols + i];
            for j in 0..cols {
                ata[i * cols + j] += ai * a[r * cols + j];
            }
        }
    }
    ata
}

/// 求对称半正定矩阵的最小特征向量（带位移逆幂迭代）。
///
/// 用 `(M + δI)⁻¹` 做逆幂迭代：δ 取一个小的正则化（相对谱范数），使矩阵正定，
/// 从而逆幂迭代收敛到最小特征向量。matching OpenCV 数值大体语义（最小奇异值
/// 方向）。返回单位范数向量。
#[must_use]
fn smallest_eigenvector(m: &[f64], n: usize) -> Option<Vec<f64>> {
    // 对称实矩阵的最小特征向量：用 `SymmetricEigen` 一次性精确求出，
    // 替代原 200 次位移逆幂迭代（每次 9×9 线性求解 + 分配）——
    // 后者被 RANSAC 1000 次调用时是 O(1000×200×n³) 病态慢（实测单帧
    // RANSAC 0.65s，直接导致相机帧掉到 ~1Hz、滤波器饿死）。
    let mat = nalgebra::DMatrix::from_row_slice(n, n, m);
    let eig = mat.symmetric_eigen();
    let vals = eig.eigenvalues;
    let mut imin = 0usize;
    for i in 1..n {
        if vals[i] < vals[imin] {
            imin = i;
        }
    }
    let col = eig.eigenvectors.column(imin);
    let mut v: Vec<f64> = (0..n).map(|i| col[i]).collect();
    let norm = v.iter().map(|x| x * x).sum::<f64>().sqrt();
    if !norm.is_finite() || norm < 1e-14 {
        return None;
    }
    for x in &mut v {
        *x /= norm;
    }
    Some(v)
}

/// 强制 3×3 基础矩阵满足 rank-2：把其最后一个右奇异方向的投影清零。
///
/// 通过对 `FᵀF` 求最小特征向量 `v`（即 SVD 的右奇异向量），做
/// `F' = F (I − vvᵀ)`。
#[must_use]
#[allow(clippy::needless_range_loop)] // 3×3 矩阵元素索引比迭代器更直白
fn enforce_rank2(f: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    // FᵀF（3×3 对称）
    let mut ftf = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            let mut s = 0.0;
            for k in 0..3 {
                s += f[k][i] * f[k][j];
            }
            ftf[i][j] = s;
        }
    }
    let flat = [
        ftf[0][0], ftf[0][1], ftf[0][2], ftf[1][0], ftf[1][1], ftf[1][2], ftf[2][0], ftf[2][1],
        ftf[2][2],
    ];
    let Some(v) = smallest_eigenvector(&flat, 3) else {
        return *f;
    };
    let nrm2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if nrm2 < 1e-12 {
        return *f;
    }
    let mut fr = [[0.0f64; 3]; 3];
    for r in 0..3 {
        for c in 0..3 {
            let mut acc = f[r][c];
            for k in 0..3 {
                acc -= f[r][k] * v[k] * v[c] / nrm2;
            }
            fr[r][c] = acc;
        }
    }
    fr
}

fn mul3(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut r = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            let mut s = 0.0;
            for k in 0..3 {
                s += a[i][k] * b[k][j];
            }
            r[i][j] = s;
        }
    }
    r
}

fn transpose3(a: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut r = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[j][i];
        }
    }
    r
}

fn flatten3(a: &[[f64; 3]; 3]) -> [f64; 9] {
    let mut r = [0.0f64; 9];
    for i in 0..3 {
        for j in 0..3 {
            r[i * 3 + j] = a[i][j];
        }
    }
    r
}

/// Hartley 归一化：返回 `(质心, 缩放系数)`，使平均范数距离为 `sqrt(2)`。
#[must_use]
fn normalize_points(pts: &[Vector2<f64>]) -> (Vector2<f64>, f64) {
    let n = pts.len() as f64;
    let cx = pts.iter().map(|p| p.x).sum::<f64>() / n;
    let cy = pts.iter().map(|p| p.y).sum::<f64>() / n;
    let dist_sum = pts
        .iter()
        .map(|p| ((p.x - cx).powi(2) + (p.y - cy).powi(2)).sqrt())
        .sum::<f64>()
        / n;
    let scale = if dist_sum > 1e-12 {
        (2.0_f64).sqrt() / dist_sum
    } else {
        1.0
    };
    (Vector2::new(cx, cy), scale)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;

    fn rand_src() -> impl FnMut() -> f64 {
        // 确定性 LCG
        let mut s = 0x1234_5678_9abc_def0u64;
        move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s as f64 / u64::MAX as f64) * 2.0 - 1.0
        }
    }

    /// 由给定基础矩阵生成干净对应点。
    fn gen_clean(f: &[f64; 9], count: usize) -> (Vec<Vector2<f64>>, Vec<Vector2<f64>>) {
        let mut rnd = rand_src();
        let mut pts0 = Vec::with_capacity(count);
        let mut pts1 = Vec::with_capacity(count);
        while pts0.len() < count {
            let x = rnd();
            let y = rnd();
            let fx = [
                f[0] * x + f[1] * y + f[2],
                f[3] * x + f[4] * y + f[5],
                f[6] * x + f[7] * y + f[8],
            ];
            let fx2 = fx[0] * fx[0] + fx[1] * fx[1] + fx[2] * fx[2];
            if fx2 < 1e-6 {
                continue;
            }
            let mut x2 = Vector3::new(rnd(), rnd(), rnd());
            let dot = fx[0] * x2.x + fx[1] * x2.y + fx[2] * x2.z;
            x2 -= (dot / fx2) * Vector3::new(fx[0], fx[1], fx[2]);
            let x2 = x2 / x2.z;
            pts0.push(Vector2::new(x, y));
            pts1.push(Vector2::new(x2.x, x2.y));
        }
        (pts0, pts1)
    }

    #[test]
    fn epipolar_holds_on_clean_data() {
        let tx = 0.1_f64;
        let ty = -0.02;
        let tz = 0.03;
        let f = [0.0, -tz, ty, tz, 0.0, -tx, -ty, tx, 0.0];
        let (pts0, pts1) = gen_clean(&f, 12);
        let f_est = estimate_fundamental_8point(&pts0, &pts1);
        assert!(f_est.is_some());
        let f = f_est.unwrap();
        for i in 0..pts0.len() {
            let d = sampson_distance(&f, pts0[i], pts1[i]);
            assert!(d < 1e-6, "sampson {d} too large at {i}");
        }
    }

    /// 由一般（非斜对称）基础矩阵 `F=[t]ₓ·R` 生成像素量级、质心远离原点的点对，
    /// 断言去归一化还原正确（锁住 `F=N1ᵀ·f2·N0` 公式）。
    #[test]
    fn denormalization_with_general_f_and_far_centroid() {
        // 一般旋转 R（绕 z(+y) 复合）与非零平移 → F 非斜对称
        let r = [0.999_8, -0.02, 0.0, 0.02, 0.999_8, 0.0, 0.0, 0.0, 1.0];
        let t = [0.5, 0.3, 0.1];
        let tx_ = t[2];
        let ty_ = t[0];
        let tz_ = t[1];
        // [t]× 与 R 相乘得到一般 F（行主序）
        let mut f = [0.0f64; 9];
        // f = [t]× · R
        // [t]× = [[0,-tz,ty],[tz,0,-tx],[-ty,tx,0]]
        let txm = tx_;
        let tym = ty_;
        let tzm = tz_;
        for c in 0..3 {
            // 列 c 为 [t]× * R 的第 c 列
            let r0 = r[c];
            let r1 = r[3 + c];
            let r2 = r[6 + c];
            f[c] = -tzm * r1 + tym * r2;
            f[3 + c] = tzm * r0 - txm * r2;
            f[6 + c] = -tym * r0 + txm * r1;
        }
        // 点对：像素量级、质心远离原点（~ (330,250)），平移范围 [100,600]
        let mut rnd0 = rand_src();
        let mut pts0 = Vec::with_capacity(16);
        let mut pts1 = Vec::with_capacity(16);
        while pts0.len() < 16 {
            let x = 250.0 + rnd0() * 350.0; // ∈ [100,600]
            let y = 250.0 + rnd0() * 350.0;
            let fx = [
                f[0] * x + f[1] * y + f[2],
                f[3] * x + f[4] * y + f[5],
                f[6] * x + f[7] * y + f[8],
            ];
            let fx2 = fx[0] * fx[0] + fx[1] * fx[1] + fx[2] * fx[2];
            if fx2 < 1e-3 {
                continue;
            }
            let mut x2 = Vector3::new(rnd0(), rnd0(), 2.0 + rnd0().abs());
            let dot = fx[0] * x2.x + fx[1] * x2.y + fx[2] * x2.z;
            x2 -= (dot / fx2) * Vector3::new(fx[0], fx[1], fx[2]);
            let w = if x2.z.abs() < 1e-6 { 1.0 } else { x2.z };
            pts0.push(Vector2::new(x, y));
            pts1.push(Vector2::new(x2.x / w, x2.y / w));
        }
        let f_est = estimate_fundamental_8point(&pts0, &pts1);
        assert!(f_est.is_some(), "8-point failed on general F");
        let f_est = f_est.unwrap();
        for i in 0..pts0.len() {
            let d = sampson_distance(&f_est, pts0[i], pts1[i]);
            assert!(
                d < 1e-6,
                "sampson {d} too large at {i}; denormalization incorrect"
            );
        }
    }

    #[test]
    fn ransac_rejects_outliers_and_recovers_inliers() {
        let tx = 0.1_f64;
        let ty = -0.02;
        let tz = 0.03;
        let f = [0.0, -tz, ty, tz, 0.0, -tx, -ty, tx, 0.0];
        // 约 67% 内点率的合成集合（8 点法在中等内点率下应能可靠恢复）
        let (mut pts0, mut pts1) = gen_clean(&f, 80);
        let inlier_gt = pts0.len();
        let mut rnd = rand_src();
        for _ in 0..40 {
            pts0.push(Vector2::new(rnd(), rnd()));
            pts1.push(Vector2::new(rnd(), rnd()));
        }
        let (mask, _best) = ransac_fundamental(&pts0, &pts1, 1e-4, 0.999);
        let true_pos = mask[..inlier_gt].iter().filter(|&&b| b).count();
        assert!(
            true_pos >= (inlier_gt as f64 * 0.9) as usize,
            "only {true_pos}/{inlier_gt} inliers"
        );
        let leaked = mask[inlier_gt..].iter().filter(|&&b| b).count();
        assert!(leaked < 8, "{leaked} outliers leaked");
    }

    #[test]
    fn ransac_adaptive_iterations_reduce_on_high_inlier_rate() {
        // 高内点率 → K 应远小于 1000；全内点 → K≈1
        let hi = update_iterations(0.999, 198, 200).unwrap();
        assert!(hi < 100, "high-inlier ratio should give small K, got {hi}");
        let full = update_iterations(0.999, 200, 200).unwrap();
        assert_eq!(full, 1);
        // 低/零内点 → 维持 1000（不因单次坏采样直接终止）
        let zero = update_iterations(0.999, 0, 200).unwrap();
        assert_eq!(zero, 1000);
        let low = update_iterations(0.999, 1, 200).unwrap();
        assert_eq!(low, 1000);
    }
}
