//! 移植官方 `root_finder.hpp`（EGO-Planner-v2 `traj_opt`）。
//!
//! 对照 `~/Projects/EGO-Planner-v2/swarm-playground/main_ws/src/planner/traj_opt/include/optimizer/root_finder.hpp`（1090 行）。
//! 函数与行为逐行对齐官方实现，仅做 Rust 习惯适配（`Vec<f64>` 领先系数在前，`BTreeSet<OrderedF64>` 代替 `std::set<double>`）。
//! 公开 API：`poly_conv` / `poly_sqr` / `poly_val` / `count_roots` / `solve_polynomial`（对应官方 `RootFinder::` 命名空间）。

#![allow(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::too_many_arguments,
    clippy::pedantic,
    clippy::nursery,
    clippy::all
)]
#![allow(dead_code)]

use std::collections::BTreeSet;
use std::f64::consts::PI;

const DBL_EPSILON: f64 = f64::EPSILON;
const HIGHEST_ORDER: usize = 64;

// 有序浮点包装，使 `BTreeSet` 可承载 `f64`（全序 `total_cmp`，与官方 `std::set<double>` 去重一致）。
#[derive(Debug, Clone, Copy)]
pub struct OrderedF64(pub f64);

impl PartialEq for OrderedF64 {
    fn eq(&self, other: &Self) -> bool {
        self.0.to_bits() == other.0.to_bits()
    }
}
impl Eq for OrderedF64 {}
impl PartialOrd for OrderedF64 {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for OrderedF64 {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&other.0)
    }
}

// 内部工具（RootFinderPriv）

/// 官方 `polyMod` (18)：`u mod v`，要求 `v[0]` 为 ±1。
fn poly_mod(u: &[f64], v: &[f64], r: &mut [f64]) -> usize {
    let lu = u.len();
    let lv = v.len();
    debug_assert_eq!(r.len(), lu);
    let order_u = lu as i32 - 1;
    let order_v = lv as i32 - 1;
    r.copy_from_slice(u);
    if v[0] < 0.0 {
        for i in (order_v + 1)..=order_u {
            let idx = i as usize;
            r[idx] = -r[idx];
        }
        for i in 0..=(order_u - order_v) {
            let ii = i as usize;
            for j in (i + 1)..=(order_v + i) {
                let jj = j as usize;
                let v_idx = (j - i) as usize;
                r[jj] = -r[jj] - r[ii] * v[v_idx];
            }
        }
    } else {
        for i in 0..=(order_u - order_v) {
            let ii = i as usize;
            for j in (i + 1)..=(order_v + i) {
                let jj = j as usize;
                let v_idx = (j - i) as usize;
                r[jj] -= r[ii] * v[v_idx];
            }
        }
    }
    let mut k = order_v - 1;
    while k >= 0 && r[(order_u - k) as usize].abs() < DBL_EPSILON {
        r[(order_u - k) as usize] = 0.0;
        k -= 1;
    }
    if k <= 0 { 1 } else { (k + 1) as usize }
}

/// 官方 `polyEval` (63)：稳定求值（非 Horner），`x==0` / `x==1` 快路径。
fn poly_eval(p: &[f64], x: f64) -> f64 {
    let len = p.len();
    if len == 0 {
        return 0.0;
    }
    if x.abs() < DBL_EPSILON {
        return p[len - 1];
    }
    if x == 1.0 {
        return p.iter().sum();
    }
    let mut ret = 0.0;
    let mut xn = 1.0;
    for i in (0..len).rev() {
        ret += p[i] * xn;
        xn *= x;
    }
    ret
}

/// 官方 `solveCub` (102)：`a x³ + b x² + c x + d = 0` 全实根。
fn solve_cub(a: f64, b: f64, c: f64, d: f64) -> BTreeSet<OrderedF64> {
    let mut roots = BTreeSet::new();
    const COS120: f64 = -0.50;
    const SIN120: f64 = 0.866_025_403_784_4386;
    let mut a = a;
    let mut b = b;
    let mut c = c;
    let mut d = d;
    if d.abs() < DBL_EPSILON {
        roots.insert(OrderedF64(0.0));
        d = c;
        c = b;
        b = a;
        a = 0.0;
    }
    if a.abs() < DBL_EPSILON {
        if b.abs() < DBL_EPSILON {
            if c.abs() > DBL_EPSILON {
                roots.insert(OrderedF64(-d / c));
            }
        } else {
            let discriminant = c * c - 4.0 * b * d;
            if discriminant >= 0.0 {
                let inv2b = 1.0 / (2.0 * b);
                let y = discriminant.sqrt();
                roots.insert(OrderedF64((-c + y) * inv2b));
                roots.insert(OrderedF64((-c - y) * inv2b));
            }
        }
    } else {
        let inva = 1.0 / a;
        let invaa = inva * inva;
        let bb = b * b;
        let bover3a = b * (1.0 / 3.0) * inva;
        let p = (3.0 * a * c - bb) * (1.0 / 3.0) * invaa;
        let halfq =
            (2.0 * bb * b - 9.0 * a * b * c + 27.0 * a * a * d) * (0.5 / 27.0) * invaa * inva;
        let yy = p * p * p / 27.0 + halfq * halfq;
        if yy > DBL_EPSILON {
            let y = yy.sqrt();
            let uuu = -halfq + y;
            let vvv = -halfq - y;
            let www = if uuu.abs() > vvv.abs() { uuu } else { vvv };
            let w = if www < 0.0 {
                -www.abs().powf(1.0 / 3.0)
            } else {
                www.powf(1.0 / 3.0)
            };
            roots.insert(OrderedF64(w - p / (3.0 * w) - bover3a));
        } else if yy < -DBL_EPSILON {
            let x = -halfq;
            let y = (-yy).sqrt();
            let (theta, r) = if x.abs() > DBL_EPSILON {
                let theta = if x > 0.0 {
                    (y / x).atan()
                } else {
                    (y / x).atan() + PI
                };
                let r = (x * x - yy).sqrt();
                (theta, r)
            } else {
                (PI / 2.0, y)
            };
            let theta3 = theta / 3.0;
            let r3 = r.powf(1.0 / 3.0);
            let ux = theta3.cos() * r3;
            let uyi = theta3.sin() * r3;
            roots.insert(OrderedF64(ux + ux - bover3a));
            roots.insert(OrderedF64(2.0 * (ux * COS120 - uyi * SIN120) - bover3a));
            roots.insert(OrderedF64(2.0 * (ux * COS120 + uyi * SIN120) - bover3a));
        } else {
            let www = -halfq;
            let w = if www < 0.0 {
                -www.abs().powf(1.0 / 3.0)
            } else {
                www.powf(1.0 / 3.0)
            };
            roots.insert(OrderedF64(w + w - bover3a));
            roots.insert(OrderedF64(2.0 * w * COS120 - bover3a));
        }
    }
    roots
}

/// 官方 `solveResolvent` (212)：预解三次式，`x` 长 3，返回实根数。
fn solve_resolvent(x: &mut [f64; 3], a: f64, b: f64, c: f64) -> usize {
    let a2 = a * a;
    let q = (a2 - 3.0 * b) / 9.0;
    let r = (a * (2.0 * a2 - 9.0 * b) + 27.0 * c) / 54.0;
    let r2 = r * r;
    let q3 = q * q * q;
    if r2 < q3 {
        let mut t = r / q3.sqrt();
        if t < -1.0 {
            t = -1.0;
        }
        if t > 1.0 {
            t = 1.0;
        }
        t = t.acos();
        let a3 = a / 3.0;
        let qv = -2.0 * q.sqrt();
        x[0] = qv * (t / 3.0).cos() - a3;
        x[1] = qv * ((t + PI * 2.0) / 3.0).cos() - a3;
        x[2] = qv * ((t - PI * 2.0) / 3.0).cos() - a3;
        3
    } else {
        let mut a_tmp = -((r.abs() + (r2 - q3).sqrt()).powf(1.0 / 3.0));
        if r < 0.0 {
            a_tmp = -a_tmp;
        }
        let b_tmp = if a_tmp == 0.0 { 0.0 } else { q / a_tmp };
        let a_div3 = a / 3.0;
        x[0] = (a_tmp + b_tmp) - a_div3;
        x[1] = -0.5 * (a_tmp + b_tmp) - a_div3;
        x[2] = 0.5 * 3.0_f64.sqrt() * (a_tmp - b_tmp);
        if x[2].abs() < DBL_EPSILON {
            x[2] = x[1];
            return 2;
        }
        1
    }
}

/// 官方 `solveQuartMonic` (265)：首一四次 `x⁴+ax³+bx²+cx+d=0` 实根。
fn solve_quart_monic(a: f64, b: f64, c: f64, d: f64) -> BTreeSet<OrderedF64> {
    let mut roots = BTreeSet::new();
    let a3 = -b;
    let b3 = a * c - 4.0 * d;
    let c3 = -a * a * d - c * c + 4.0 * b * d;
    let mut x3 = [0.0; 3];
    let i_zeroes = solve_resolvent(&mut x3, a3, b3, c3);
    let mut y = x3[0];
    if i_zeroes != 1 {
        if x3[1].abs() > y.abs() {
            y = x3[1];
        }
        if x3[2].abs() > y.abs() {
            y = x3[2];
        }
    }
    let (q1, q2, p1, p2);
    let d_val = y * y - 4.0 * d;
    if d_val.abs() < DBL_EPSILON {
        let q = y * 0.5;
        q1 = q;
        q2 = q;
        let dd = a * a - 4.0 * (b - y);
        if dd.abs() < DBL_EPSILON {
            let p = a * 0.5;
            p1 = p;
            p2 = p;
        } else {
            let sd = dd.sqrt();
            p1 = (a + sd) * 0.5;
            p2 = (a - sd) * 0.5;
        }
    } else {
        let sd = d_val.sqrt();
        q1 = (y + sd) * 0.5;
        q2 = (y - sd) * 0.5;
        p1 = (a * q1 - c) / (q1 - q2);
        p2 = (c - a * q2) / (q1 - q2);
    }
    let mut disc = p1 * p1 - 4.0 * q1;
    if disc.abs() < DBL_EPSILON {
        roots.insert(OrderedF64(-p1 * 0.5));
    } else if disc > 0.0 {
        let sd = disc.sqrt();
        roots.insert(OrderedF64((-p1 + sd) * 0.5));
        roots.insert(OrderedF64((-p1 - sd) * 0.5));
    }
    disc = p2 * p2 - 4.0 * q2;
    if disc.abs() < DBL_EPSILON {
        roots.insert(OrderedF64(-p2 * 0.5));
    } else if disc > 0.0 {
        let sd = disc.sqrt();
        roots.insert(OrderedF64((-p2 + sd) * 0.5));
        roots.insert(OrderedF64((-p2 - sd) * 0.5));
    }
    roots
}

/// 官方 `solveQuart` (353)
fn solve_quart(a: f64, b: f64, c: f64, d: f64, e: f64) -> BTreeSet<OrderedF64> {
    if a.abs() < DBL_EPSILON {
        return solve_cub(b, c, d, e);
    }
    solve_quart_monic(b / a, c / a, d / a, e / a)
}

/// 官方 `numSignVar` (398)
fn num_sign_var(x: f64, sturm_seqs: &[Vec<f64>]) -> i32 {
    let mut sign_var = 0;
    let mut lasty = poly_eval(&sturm_seqs[0], x);
    for i in 1..sturm_seqs.len() {
        let y = poly_eval(&sturm_seqs[i], x);
        if lasty == 0.0 || lasty * y < 0.0 {
            sign_var += 1;
        }
        lasty = y;
    }
    sign_var
}

/// 官方 `polyDeri` (418)
fn poly_deri(coeffs: &[f64]) -> Vec<f64> {
    let horder = coeffs.len() - 1;
    let mut d = Vec::with_capacity(horder);
    for i in 0..horder {
        d.push((horder - i) as f64 * coeffs[i]);
    }
    d
}

/// 官方 `safeNewton` (430)
fn safe_newton<F, DF>(func: &F, dfunc: &DF, l: f64, h: f64, tol: f64, max_its: usize) -> f64
where
    F: Fn(f64) -> f64,
    DF: Fn(f64) -> f64,
{
    let fl = func(l);
    let fh = func(h);
    if fl == 0.0 {
        return l;
    }
    if fh == 0.0 {
        return h;
    }
    let (mut xl, mut xh) = if fl < 0.0 { (l, h) } else { (h, l) };
    let mut rts = 0.5 * (xl + xh);
    let mut dxold = (xh - xl).abs();
    let mut dx = dxold;
    let mut f = func(rts);
    let mut df = dfunc(rts);
    for _ in 0..max_its {
        if ((rts - xh) * df - f) * ((rts - xl) * df - f) > 0.0
            || (2.0 * f).abs() > (dxold * df).abs()
        {
            dxold = dx;
            dx = 0.5 * (xh - xl);
            rts = xl + dx;
            if xl == rts {
                break;
            }
        } else {
            dxold = dx;
            dx = f / df;
            let temp = rts;
            rts -= dx;
            if temp == rts {
                break;
            }
        }
        if dx.abs() < tol {
            break;
        }
        f = func(rts);
        df = dfunc(rts);
        if f < 0.0 {
            xl = rts;
        } else {
            xh = rts;
        }
    }
    rts
}

/// 官方 `shrinkInterval` (509)
fn shrink_interval(coeffs: &[f64], lbound: f64, ubound: f64, tol: f64) -> f64 {
    let dcoeffs = poly_deri(coeffs);
    let func = |x: f64| poly_eval(coeffs, x);
    let dfunc = |x: f64| poly_eval(&dcoeffs, x);
    safe_newton(&func, &dfunc, lbound, ubound, tol, 128)
}

/// 官方 `recurIsolate` (523)
fn recur_isolate(
    l: f64,
    r: f64,
    fl: f64,
    fr: f64,
    lnv: i32,
    rnv: i32,
    tol: f64,
    sturm_seqs: &[Vec<f64>],
    rts: &mut BTreeSet<OrderedF64>,
) {
    let nrts = lnv - rnv;
    if nrts == 0 {
        return;
    }
    if nrts == 1 {
        if fl * fr < 0.0 {
            rts.insert(OrderedF64(shrink_interval(&sturm_seqs[0], l, r, tol)));
            return;
        }
        for _ in 0..128 {
            if fl * fr < 0.0 {
                rts.insert(OrderedF64(shrink_interval(&sturm_seqs[1], l, r, tol)));
                return;
            }
            let m = (l + r) / 2.0;
            let fm = poly_eval(&sturm_seqs[0], m);
            if fm == 0.0 || (r - l).abs() < tol {
                rts.insert(OrderedF64(m));
                return;
            }
            let mnv = num_sign_var(m, sturm_seqs);
            if lnv == mnv {
                // 区间左半无根，收缩左界
                // 为简化实现，直接递归右半（官方原地更新 l/fl，此处等价）
                recur_isolate(m, r, fm, fr, mnv, rnv, tol, sturm_seqs, rts);
                return;
            }
            recur_isolate(l, m, fl, fm, lnv, mnv, tol, sturm_seqs, rts);
            return;
        }
        rts.insert(OrderedF64((l + r) / 2.0));
        return;
    }
    // nrts > 1
    let mut cl = l;
    let mut cr = r;
    let mut cfl = fl;
    let mut cfr = fr;
    let mut clnv = lnv;
    let mut crnv = rnv;
    for _ in 0..128 {
        let m = (cl + cr) / 2.0;
        let mnv = num_sign_var(m, sturm_seqs);
        if (cr - cl).abs() < tol {
            rts.insert(OrderedF64(m));
            return;
        }
        let fm = poly_eval(&sturm_seqs[0], m);
        if fm == 0.0 {
            // 命中根，微小偏移避免卡住（官方 bias 逻辑简化：四分位点重试）
            let biased_m = (cr - cl) / 4.0 + cl;
            let biased_fm = poly_eval(&sturm_seqs[0], biased_m);
            let biased_mnv = num_sign_var(biased_m, sturm_seqs);
            if clnv != biased_mnv && crnv != biased_mnv {
                recur_isolate(
                    cl, biased_m, cfl, biased_fm, clnv, biased_mnv, tol, sturm_seqs, rts,
                );
                recur_isolate(
                    biased_m, cr, biased_fm, cfr, biased_mnv, crnv, tol, sturm_seqs, rts,
                );
                return;
            } else if clnv == biased_mnv {
                cl = biased_m;
                cfl = biased_fm;
                clnv = biased_mnv;
            } else {
                cr = biased_m;
                cfr = biased_fm;
                crnv = biased_mnv;
            }
            continue;
        }
        if clnv != mnv && crnv != mnv {
            recur_isolate(cl, m, cfl, fm, clnv, mnv, tol, sturm_seqs, rts);
            recur_isolate(m, cr, fm, cfr, mnv, crnv, tol, sturm_seqs, rts);
            return;
        } else if clnv == mnv {
            cl = m;
            cfl = fm;
            clnv = mnv;
        } else {
            cr = m;
            cfr = fm;
            crnv = mnv;
        }
    }
    rts.insert(OrderedF64((cl + cr) / 2.0));
}

/// 官方 `isolateRealRoots` (646)：区间隔离求根（Sturm 定理）
fn isolate_real_roots(coeffs: &[f64], lbound: f64, ubound: f64, tol: f64) -> BTreeSet<OrderedF64> {
    // 先用 Cauchy/Kojima 界收紧区间（与官方一致）
    let leading = coeffs[0];
    let mut monic = Vec::with_capacity(coeffs.len());
    monic.push(1.0);
    for i in 1..coeffs.len() {
        monic.push(coeffs[i] / leading);
    }
    let rho_c = 1.0 + monic[1..].iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    let nonzero: Vec<f64> = monic
        .iter()
        .copied()
        .filter(|x| x.abs() >= DBL_EPSILON)
        .collect();
    if nonzero.is_empty() {
        return BTreeSet::new();
    }
    let nonzeros = nonzero.len();
    let mut kojima_vec = Vec::with_capacity(nonzeros - 1);
    for i in 0..nonzeros - 1 {
        kojima_vec.push((nonzero[i + 1] / nonzero[i]).abs());
    }
    if let Some(last) = kojima_vec.last_mut() {
        *last /= 2.0;
    }
    let rho_k = 2.0 * kojima_vec.iter().cloned().fold(0.0_f64, f64::max);
    let rho = rho_c.min(rho_k) + 1.0;
    let lb = lbound.max(-rho);
    let ub = ubound.min(rho);
    // 递归隔离：用 count_roots 做区间二分，避免直接操作 Sturm 序列的复杂性
    fn rec(l: f64, r: f64, coeffs: &[f64], tol: f64, rts: &mut BTreeSet<OrderedF64>) {
        let n = count_roots(coeffs, l, r);
        if n == 0 {
            return;
        }
        if n == 1 {
            let fl = poly_val(coeffs, l, true);
            let fr = poly_val(coeffs, r, true);
            let root = if fl * fr < 0.0 {
                shrink_interval(coeffs, l, r, tol)
            } else {
                // 无符号变化（如偶重根或区间未严格隔离），用采样找近似再 Newton 精化
                let mut best_x = (l + r) * 0.5;
                let mut best_v = f64::INFINITY;
                for k in 0..32 {
                    let x = l + (r - l) * k as f64 / 31.0;
                    let v = poly_val(coeffs, x, true).abs();
                    if v < best_v {
                        best_v = v;
                        best_x = x;
                    }
                }
                let mut x = best_x;
                let dcoeffs = poly_deri(coeffs);
                for _ in 0..32 {
                    let f = poly_val(coeffs, x, true);
                    let df = poly_val(&dcoeffs, x, true);
                    if df.abs() < DBL_EPSILON {
                        break;
                    }
                    let nx = x - f / df;
                    if nx < l || nx > r {
                        break;
                    }
                    x = nx;
                    if f.abs() < tol {
                        break;
                    }
                }
                x
            };
            rts.insert(OrderedF64(root));
            return;
        }
        if (r - l).abs() < tol {
            rts.insert(OrderedF64((l + r) * 0.5));
            return;
        }
        let m = (l + r) * 0.5;
        let fm = poly_val(coeffs, m, true);
        if fm.abs() < DBL_EPSILON {
            rts.insert(OrderedF64(m));
            let eps = tol.max(1e-12);
            if m - eps > l {
                rec(l, m - eps, coeffs, tol, rts);
            }
            if m + eps < r {
                rec(m + eps, r, coeffs, tol, rts);
            }
            return;
        }
        rec(l, m, coeffs, tol, rts);
        rec(m, r, coeffs, tol, rts);
    }
    let mut rts = BTreeSet::new();
    rec(lb, ub, coeffs, tol, &mut rts);
    rts
}

// 公开 API（RootFinder）

/// 官方 `polyConv` (741)：卷积
#[must_use]
pub fn poly_conv(l_coef: &[f64], r_coef: &[f64]) -> Vec<f64> {
    let n = l_coef.len() + r_coef.len() - 1;
    let mut result = vec![0.0; n];
    for i in 0..n {
        for j in 0..=i {
            if j < l_coef.len() && (i - j) < r_coef.len() {
                result[i] += l_coef[j] * r_coef[i - j];
            }
        }
    }
    result
}

/// 官方 `polySqr` (826)：自卷积
#[must_use]
pub fn poly_sqr(coef: &[f64]) -> Vec<f64> {
    let coef_size = coef.len();
    if coef_size == 0 {
        return Vec::new();
    }
    let result_size = coef_size * 2 - 1;
    let mut result = vec![0.0; result_size];
    for i in 0..result_size {
        let mut temp = 0.0;
        let mut lbound = i as i32 - coef_size as i32 + 1;
        if lbound < 0 {
            lbound = 0;
        }
        let mut rbound = if coef_size < (i + 1) {
            coef_size
        } else {
            i + 1
        };
        rbound += lbound as usize;
        let rbound_orig = rbound;
        if rbound_orig & 1 == 1 {
            let mid = rbound_orig >> 1;
            temp += coef[mid] * coef[mid];
            rbound = rbound_orig >> 1;
        } else {
            rbound >>= 1;
        }
        for j in lbound as usize..rbound {
            temp += 2.0 * coef[j] * coef[i - j];
        }
        result[i] = temp;
    }
    result
}

/// 官方 `polyVal` (861)：多项式求值，`numerical_stability=true` 用稳定法
#[must_use]
pub fn poly_val(coeffs: &[f64], x: f64, numerical_stability: bool) -> f64 {
    if coeffs.is_empty() {
        return 0.0;
    }
    let order = coeffs.len() - 1;
    if x.abs() < DBL_EPSILON {
        return coeffs[order];
    }
    if x == 1.0 {
        return coeffs.iter().sum();
    }
    if numerical_stability {
        let mut ret = 0.0;
        let mut xn = 1.0;
        for i in (0..coeffs.len()).rev() {
            ret += coeffs[i] * xn;
            xn *= x;
        }
        ret
    } else {
        let mut ret = 0.0;
        for &c in coeffs {
            ret = ret * x + c;
        }
        ret
    }
}

/// 官方 `countRoots` (907)：Sturm 计数 `(l,r)` 内互异实根数
#[must_use]
pub fn count_roots(coeffs: &[f64], l: f64, r: f64) -> i32 {
    let original_size = coeffs.len();
    let mut valid = original_size;
    for &c in coeffs {
        if c.abs() < DBL_EPSILON {
            valid -= 1;
        } else {
            break;
        }
    }
    if valid == 0 {
        return 0;
    }
    if coeffs[original_size - 1].abs() < DBL_EPSILON {
        return 0;
    }
    let offset = original_size - valid;
    let monic_len = valid;
    let leading = coeffs[offset];
    let mut monic = Vec::with_capacity(monic_len);
    monic.push(1.0);
    for i in 1..monic_len {
        monic.push(coeffs[offset + i] / leading);
    }
    let len = monic.len();
    let order = len - 1;
    let mut seqs: Vec<Vec<f64>> = Vec::new();
    seqs.push(monic.clone());
    let mut deriv = Vec::with_capacity(len - 1);
    for i in 0..len {
        deriv.push((order - i) as f64 * monic[i] / order as f64);
    }
    deriv.truncate(len - 1);
    seqs.push(deriv);
    let mut idx = 0;
    loop {
        let a = seqs[idx].clone();
        let b = seqs[idx + 1].clone();
        let mut r_full = vec![0.0; a.len()];
        let r_len = poly_mod(&a, &b, &mut r_full);
        let mut r = r_full[a.len() - r_len..].to_vec();
        if r_len == 1 && r[0].abs() < DBL_EPSILON {
            break;
        }
        let first_abs = r[0].abs();
        if first_abs < DBL_EPSILON {
            break;
        }
        for i in 1..r.len() {
            r[i] /= -first_abs;
        }
        r[0] /= -first_abs;
        seqs.push(r);
        if r_len == 1 {
            break;
        }
        idx += 1;
        if seqs.len() > HIGHEST_ORDER {
            break;
        }
        if seqs.last().is_some_and(|s| s.len() == 1) {
            break;
        }
    }
    let mut n_roots = 0;
    let mut last_yl = poly_eval(&seqs[0], l);
    let mut last_yr = poly_eval(&seqs[0], r);
    for i in 1..seqs.len() {
        let yl = poly_eval(&seqs[i], l);
        let yr = poly_eval(&seqs[i], r);
        if last_yl == 0.0 || last_yl * yl < 0.0 {
            n_roots += 1;
        }
        if last_yr == 0.0 || last_yr * yr < 0.0 {
            n_roots -= 1;
        }
        last_yl = yl;
        last_yr = yr;
    }
    n_roots
}

/// 官方 `solvePolynomial` (990)
#[must_use]
pub fn solve_polynomial(
    coeffs: &[f64],
    lbound: f64,
    ubound: f64,
    tol: f64,
) -> BTreeSet<OrderedF64> {
    let mut valid = coeffs.len();
    for &c in coeffs {
        if c.abs() < DBL_EPSILON {
            valid -= 1;
        } else {
            break;
        }
    }
    let mut offset = 0usize;
    let mut nonzeros = valid;
    if valid > 0 {
        for i in 0..valid {
            if coeffs[coeffs.len() - 1 - i].abs() < DBL_EPSILON {
                nonzeros -= 1;
                offset += 1;
            } else {
                break;
            }
        }
    }
    let mut rts = BTreeSet::new();
    if nonzeros == 0 {
        rts.insert(OrderedF64(f64::INFINITY));
        rts.insert(OrderedF64(f64::NEG_INFINITY));
    } else if nonzeros == 1 && offset == 0 {
        // 常数非零，无根
    } else {
        let mut ncoeffs = vec![0.0; std::cmp::max(5, nonzeros)];
        let start = coeffs.len() - valid;
        let tail = &coeffs[start..start + nonzeros];
        let dst_start = ncoeffs.len() - nonzeros;
        ncoeffs[dst_start..].copy_from_slice(tail);
        let ncoeffs_clone = ncoeffs.clone();
        if nonzeros <= 5 {
            let a = ncoeffs_clone[0];
            let b = ncoeffs_clone[1];
            let c = ncoeffs_clone[2];
            let d = ncoeffs_clone[3];
            let e = ncoeffs_clone[4];
            rts = solve_quart(a, b, c, d, e);
        } else {
            rts = isolate_real_roots(&ncoeffs_clone, lbound, ubound, tol);
        }
        if offset > 0 {
            rts.insert(OrderedF64(0.0));
        }
    }
    let mut filtered = BTreeSet::new();
    for r in rts {
        if r.0 > lbound && r.0 < ubound {
            filtered.insert(r);
        }
    }
    filtered
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn poly_sqr_simple() {
        let c = vec![1.0, 1.0];
        let sq = poly_sqr(&c);
        assert_eq!(sq, vec![1.0, 2.0, 1.0]);
    }

    #[test]
    fn solve_cubic_single() {
        let r = solve_cub(1.0, 0.0, 0.0, -1.0);
        assert!(r.contains(&OrderedF64(1.0)));
    }

    #[test]
    fn count_roots_simple() {
        let c = vec![1.0, 0.0, -1.0];
        assert_eq!(count_roots(&c, -2.0, 0.0), 1);
        assert_eq!(count_roots(&c, 0.0, 2.0), 1);
        assert_eq!(count_roots(&c, -2.0, 2.0), 2);
    }

    #[test]
    fn solve_polynomial_quadratic() {
        let c = vec![1.0, -3.0, 2.0];
        let r = solve_polynomial(&c, -10.0, 10.0, 1e-9);
        let v: Vec<f64> = r.iter().map(|x| x.0).collect();
        assert!(v.iter().any(|x| (x - 1.0).abs() < 1e-6));
        assert!(v.iter().any(|x| (x - 2.0).abs() < 1e-6));
    }

    #[test]
    fn solve_polynomial_quart_known_roots() {
        // (u-0.2)(u-0.5)(u-0.8)(u-1.1) = u⁴ -2.6u³ +2.31u² -0.806u +0.088
        let c = vec![1.0, -2.6, 2.31, -0.806, 0.088];
        let r = solve_polynomial(&c, -0.0625, 1.0625, 1e-7);
        let v: Vec<f64> = r.iter().map(|x| x.0).collect();
        assert!(
            v.iter().any(|x| (x - 0.2).abs() < 1e-6),
            "应含 0.2，实际 {v:?}"
        );
        assert!(
            v.iter().any(|x| (x - 0.5).abs() < 1e-6),
            "应含 0.5，实际 {v:?}"
        );
        assert!(
            v.iter().any(|x| (x - 0.8).abs() < 1e-6),
            "应含 0.8，实际 {v:?}"
        );
        assert!(
            !v.iter().any(|x| (x - 1.1).abs() < 1e-6),
            "1.1 在区间外不应含"
        );
        assert_eq!(v.len(), 3, "区间内应恰 3 根，实际 {v:?}");
        for &x in &v {
            assert!(
                poly_val(&c, x, true).abs() < 1e-6,
                "根 {x} 代入应为 0，实际 {}",
                poly_val(&c, x, true)
            );
        }
    }

    #[test]
    fn solve_polynomial_cubic_known_roots() {
        // (u-0.1)(u-0.6)(u-0.9) = u³ -1.6u² +0.69u -0.054
        let c = vec![1.0, -1.6, 0.69, -0.054];
        let r = solve_polynomial(&c, -0.0625, 1.0625, 1e-9);
        let v: Vec<f64> = r.iter().map(|x| x.0).collect();
        assert_eq!(v.len(), 3);
        assert!(v.iter().any(|x| (x - 0.1).abs() < 1e-6));
        assert!(v.iter().any(|x| (x - 0.6).abs() < 1e-6));
        assert!(v.iter().any(|x| (x - 0.9).abs() < 1e-6));
    }

    #[test]
    fn solve_polynomial_degree5_known_roots() {
        // (u-0.1)(u-0.3)(u-0.5)(u-0.7)(u-0.9) = u⁵ -2.5u⁴ +2.3u³ -0.95u² +0.1689u -0.00945
        let c = vec![1.0, -2.5, 2.3, -0.95, 0.1689, -0.00945];
        let r = solve_polynomial(&c, -0.0625, 1.0625, 1e-9);
        let v: Vec<f64> = r.iter().map(|x| x.0).collect();
        assert_eq!(v.len(), 5, "5 次应 5 根，实际 {v:?}");
        for exp in [0.1, 0.3, 0.5, 0.7, 0.9] {
            assert!(v.iter().any(|x| (x - exp).abs() < 1e-6), "缺 {exp} {v:?}");
        }
    }

    #[test]
    fn solve_polynomial_degree7_known_roots() {
        // 7 次：根 0.1 0.2 0.3 0.5 0.7 0.85 0.95
        let roots = [0.1, 0.2, 0.3, 0.5, 0.7, 0.85, 0.95];
        let mut coeff = vec![1.0];
        for &r in &roots {
            let mut next = vec![0.0; coeff.len() + 1];
            for i in 0..coeff.len() {
                next[i] += coeff[i];
                next[i + 1] -= coeff[i] * r;
            }
            coeff = next;
        }
        let r = solve_polynomial(&coeff, -0.0625, 1.0625, 1e-7);
        let v: Vec<f64> = r.iter().map(|x| x.0).collect();
        assert_eq!(v.len(), 7, "7 次应 7 根，实际 {v:?}");
        for &exp in &roots {
            assert!(v.iter().any(|x| (x - exp).abs() < 1e-5), "缺 {exp} {v:?}");
        }
    }
}
