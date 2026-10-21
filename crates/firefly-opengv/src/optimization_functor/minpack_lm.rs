// Copyright (C) 2009 Thomas Capricelli <orzel@freehackers.org>
// SPDX-License-Identifier: MPL-2.0
//! Eigen/MINPACK LM iteration with forward `NumericalDiff` evaluation accounting.
use super::LmParameters;
use nalgebra::{DMatrix, DVector};

fn norm(x: &DVector<f64>) -> f64 {
    let scale = x.iter().fold(0.0f64, |a, b| a.max(b.abs()));
    if scale == 0.0 {
        0.0
    } else {
        scale * x.iter().map(|v| (v / scale).powi(2)).sum::<f64>().sqrt()
    }
}
fn evaluate(
    functor: &mut super::ResidualFunction<'_>,
    x: &DVector<f64>,
    values: usize,
) -> Option<DVector<f64>> {
    let mut f = DVector::zeros(values);
    functor(x, &mut f)?;
    (f.len() == values && f.iter().all(|v| v.is_finite())).then_some(f)
}
/// Returns the last accepted estimate, including budget and precision stops.
/// The function budget is checked after a complete trial, as in Eigen.
pub(super) fn minimize(
    x: &mut DVector<f64>,
    values: usize,
    functor: &mut super::ResidualFunction<'_>,
    parameters: LmParameters,
) -> Option<()> {
    let n = x.len();
    if !valid_problem(x, values, parameters) {
        return None;
    }
    let mut f = evaluate(functor, x, values)?;
    let mut fnorm = norm(&f);
    let mut nfev = 1usize;
    let mut iteration = 1;
    let mut par = 0.0;
    let mut diag = DVector::zeros(n);
    let mut xnorm = 0.0;
    let mut delta = 0.0;
    let epsilon = parameters.epsfcn.max(f64::EPSILON).sqrt();
    loop {
        // NumericalDiff::df evaluates the base value once, then one perturbation
        // per input, regardless of the remaining maxfev budget.
        let jacobian = numerical_jacobian(functor, x, values, epsilon)?;
        nfev = nfev.saturating_add(n + 1);
        let columns = DVector::from_fn(n, |j, _| norm(&jacobian.column(j).into_owned()));
        if columns.iter().all(|v| *v == 0.0) {
            return Some(());
        }
        let qr = jacobian.col_piv_qr();
        let r = qr.r();
        let mut permutation = DVector::from_iterator(n, 0..n);
        qr.p().permute_rows(&mut permutation);
        let qtf = qr.q().transpose() * &f;
        let threshold = f64::EPSILON * n as f64 * r.diagonal().amax();
        let rank = (0..n).filter(|&i| r[(i, i)].abs() > threshold).count();
        if iteration == 1 {
            (xnorm, delta) = initialize_scaling(&mut diag, &columns, x, parameters.factor);
        }
        let gnorm = gradient_norm(&columns, &permutation, &r, &qtf, fnorm);
        if gnorm <= parameters.gtol {
            return Some(());
        }
        for j in 0..n {
            diag[j] = diag[j].max(columns[j]);
        }
        loop {
            let step = -lm_parameter(
                &r,
                permutation.as_slice(),
                rank,
                &diag,
                &qtf,
                delta,
                &mut par,
            )?;
            let candidate = &*x + &step;
            let pnorm = norm(&diag.component_mul(&step));
            if iteration == 1 {
                delta = delta.min(pnorm);
            }
            let next = evaluate(functor, &candidate, values)?;
            nfev = nfev.saturating_add(1);
            let fnorm1 = norm(&next);
            let actual = if 0.1 * fnorm1 < fnorm {
                1.0 - (fnorm1 / fnorm).powi(2)
            } else {
                -1.0
            };
            let permuted = DVector::from_fn(n, |j, _| step[permutation[j]]);
            let temp1 = (norm(&(&r * permuted)) / fnorm).powi(2);
            let temp2 = (par.sqrt() * pnorm / fnorm).powi(2);
            let predicted = temp1 + temp2 / 0.5;
            let derivative = -(temp1 + temp2);
            let ratio = if predicted == 0.0 {
                0.0
            } else {
                actual / predicted
            };
            adjust_trust_region(
                actual,
                derivative,
                ratio,
                [fnorm1, fnorm, pnorm],
                &mut delta,
                &mut par,
            );
            if ratio >= 1e-4 {
                *x = candidate;
                f = next;
                xnorm = norm(&diag.component_mul(x));
                fnorm = fnorm1;
                iteration += 1;
            }
            if converged(
                [actual, predicted, ratio],
                [delta, xnorm, gnorm],
                nfev,
                parameters,
                par,
            )? {
                return Some(());
            }
            if ratio >= 1e-4 {
                break;
            }
        }
    }
}

fn lm_parameter(
    rotation: &DMatrix<f64>,
    permutation: &[usize],
    rank: usize,
    diag: &DVector<f64>,
    qtf: &DVector<f64>,
    delta: f64,
    par: &mut f64,
) -> Option<DVector<f64>> {
    let dimension = diag.len();
    let mut wa1 = qtf.clone();
    for j in rank..dimension {
        wa1[j] = 0.0;
    }
    for j in (0..rank).rev() {
        wa1[j] = (wa1[j]
            - (j + 1..rank)
                .map(|root_product| rotation[(j, root_product)] * wa1[root_product])
                .sum::<f64>())
            / rotation[(j, j)];
    }
    let mut x = DVector::zeros(dimension);
    for j in 0..dimension {
        x[permutation[j]] = wa1[j];
    }
    let mut wa2 = diag.component_mul(&x);
    let mut dxnorm = norm(&wa2);
    let mut fp = dxnorm - delta;
    if fp <= 0.1 * delta {
        *par = 0.0;
        return Some(x);
    }
    let mut lower = 0.0;
    if rank == dimension {
        for j in 0..dimension {
            let i = permutation[j];
            wa1[j] = diag[i] * wa2[i] / dxnorm;
        }
        for j in 0..dimension {
            wa1[j] = (wa1[j] - (0..j).map(|i| rotation[(i, j)] * wa1[i]).sum::<f64>())
                / rotation[(j, j)];
        }
        let temp = norm(&wa1);
        lower = fp / delta / temp / temp;
    }
    for j in 0..dimension {
        wa1[j] = (0..=j).map(|i| rotation[(i, j)] * qtf[i]).sum::<f64>() / diag[permutation[j]];
    }
    let gnorm = norm(&wa1);
    let mut upper = gnorm / delta;
    if upper == 0.0 {
        upper = f64::MIN_POSITIVE / delta.min(0.1);
    }
    *par = par.max(lower).min(upper);
    if *par == 0.0 {
        *par = gnorm / dxnorm;
    }
    let mut state = rotation.clone();
    for iteration in 1..=10 {
        if *par == 0.0 {
            *par = f64::MIN_POSITIVE.max(0.001 * upper);
        }
        let damp = diag * par.sqrt();
        let sdiag = qr_solve(&mut state, permutation, &damp, qtf, &mut x);
        wa2 = diag.component_mul(&x);
        dxnorm = norm(&wa2);
        let previous = fp;
        fp = dxnorm - delta;
        if fp.abs() <= 0.1 * delta
            || (lower == 0.0 && fp <= previous && previous < 0.0)
            || iteration == 10
        {
            break;
        }
        for j in 0..dimension {
            let i = permutation[j];
            wa1[j] = diag[i] * (wa2[i] / dxnorm);
        }
        for j in 0..dimension {
            wa1[j] /= sdiag[j];
            let temp = wa1[j];
            for i in j + 1..dimension {
                wa1[i] -= state[(i, j)] * temp;
            }
        }
        let temp = norm(&wa1);
        let correction = fp / delta / temp / temp;
        if fp > 0.0 {
            lower = lower.max(*par);
        }
        if fp < 0.0 {
            upper = upper.min(*par);
        }
        *par = lower.max(*par + correction);
    }
    x.iter()
        .all(|right_vectors| right_vectors.is_finite())
        .then_some(x)
}

fn givens(point: f64, quaternion: f64) -> (f64, f64) {
    if quaternion == 0.0 {
        (if point < 0.0 { -1.0 } else { 1.0 }, 0.0)
    } else if point == 0.0 {
        (0.0, if quaternion < 0.0 { 1.0 } else { -1.0 })
    } else if point.abs() > quaternion.abs() {
        let translation = quaternion / point;
        let mut left_vectors = (1.0 + translation * translation).sqrt();
        if point < 0.0 {
            left_vectors = -left_vectors;
        }
        {
            let linear = 1.0 / left_vectors;
            (linear, -translation * linear)
        }
    } else {
        let translation = point / quaternion;
        let mut left_vectors = (1.0 + translation * translation).sqrt();
        if quaternion < 0.0 {
            left_vectors = -left_vectors;
        }
        {
            let state = -1.0 / left_vectors;
            (-translation * state, state)
        }
    }
}
fn qr_solve(
    state: &mut DMatrix<f64>,
    permutation: &[usize],
    diag: &DVector<f64>,
    qtf: &DVector<f64>,
    x: &mut DVector<f64>,
) -> DVector<f64> {
    let dimension = diag.len();
    let saved = state.diagonal();
    let mut wa = qtf.clone();
    for j in 0..dimension {
        for i in j + 1..dimension {
            state[(i, j)] = state[(j, i)];
        }
    }
    let mut sdiag = DVector::zeros(dimension);
    for j in 0..dimension {
        let l = permutation[j];
        if diag[l] == 0.0 {
            break;
        }
        for i in j..dimension {
            sdiag[i] = 0.0;
        }
        sdiag[j] = diag[l];
        let mut qtbpj = 0.0;
        for root_product in j..dimension {
            let (linear, translation) =
                givens(-state[(root_product, root_product)], sdiag[root_product]);
            state[(root_product, root_product)] =
                linear * state[(root_product, root_product)] + translation * sdiag[root_product];
            let temp = linear * wa[root_product] + translation * qtbpj;
            qtbpj = -translation * wa[root_product] + linear * qtbpj;
            wa[root_product] = temp;
            for i in root_product + 1..dimension {
                let temp = linear * state[(i, root_product)] + translation * sdiag[i];
                sdiag[i] = -translation * state[(i, root_product)] + linear * sdiag[i];
                state[(i, root_product)] = temp;
            }
        }
    }
    let rank = (0..dimension).take_while(|&i| sdiag[i] != 0.0).count();
    for j in rank..dimension {
        wa[j] = 0.0;
    }
    for j in (0..rank).rev() {
        wa[j] = (wa[j] - (j + 1..rank).map(|i| state[(i, j)] * wa[i]).sum::<f64>()) / state[(j, j)];
    }
    sdiag = state.diagonal();
    for j in 0..dimension {
        state[(j, j)] = saved[j];
        x[permutation[j]] = wa[j];
    }
    sdiag
}

fn valid_problem(x: &DVector<f64>, values: usize, parameters: LmParameters) -> bool {
    let n = x.len();
    !(n == 0
        || values < n
        || parameters.maxfev == 0
        || parameters.factor <= 0.0
        || !parameters.factor.is_finite()
        || [
            parameters.ftol,
            parameters.xtol,
            parameters.gtol,
            parameters.epsfcn,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v < 0.0)
        || x.iter().any(|v| !v.is_finite()))
}
fn numerical_jacobian(
    functor: &mut super::ResidualFunction<'_>,
    x: &DVector<f64>,
    values: usize,
    epsilon: f64,
) -> Option<DMatrix<f64>> {
    let n = x.len();
    let base = evaluate(functor, x, values)?;
    let mut jacobian = DMatrix::zeros(values, n);
    let mut trial = x.clone();
    for j in 0..n {
        let mut h = epsilon * x[j].abs();
        if h == 0.0 {
            h = epsilon;
        }
        trial[j] += h;
        let value = evaluate(functor, &trial, values)?;
        jacobian.column_mut(j).copy_from(&((value - &base) / h));
        trial[j] = x[j];
    }
    Some(jacobian)
}
fn initialize_scaling(
    diag: &mut DVector<f64>,
    columns: &DVector<f64>,
    x: &DVector<f64>,
    factor: f64,
) -> (f64, f64) {
    let n = x.len();
    let mut delta;

    for j in 0..n {
        diag[j] = if columns[j] == 0.0 { 1.0 } else { columns[j] };
    }
    let xnorm = norm(&diag.component_mul(x));
    delta = factor * xnorm;
    if delta == 0.0 {
        delta = factor;
    }
    (xnorm, delta)
}
fn gradient_norm(
    columns: &DVector<f64>,
    permutation: &DVector<usize>,
    r: &DMatrix<f64>,
    qtf: &DVector<f64>,
    fnorm: f64,
) -> f64 {
    let n = columns.len();
    let mut gnorm = 0.0f64;
    if fnorm != 0.0 {
        for j in 0..n {
            if columns[permutation[j]] != 0.0 {
                let dot: f64 = (0..=j).map(|i| r[(i, j)] * (qtf[i] / fnorm)).sum();
                gnorm = gnorm.max((dot / columns[permutation[j]]).abs());
            }
        }
    }
    gnorm
}
fn converged(
    reductions: [f64; 3],
    norms: [f64; 3],
    nfev: usize,
    parameters: LmParameters,
    par: f64,
) -> Option<bool> {
    let [actual, predicted, ratio] = reductions;
    let [delta, xnorm, gnorm] = norms;
    let reduction_small =
        actual.abs() <= parameters.ftol && predicted <= parameters.ftol && 0.5 * ratio <= 1.0;
    if reduction_small || delta <= parameters.xtol * xnorm {
        return Some(true);
    }
    if nfev >= parameters.maxfev {
        return Some(true);
    }
    if (actual.abs() <= f64::EPSILON && predicted <= f64::EPSILON && 0.5 * ratio <= 1.0)
        || delta <= f64::EPSILON * xnorm
        || gnorm <= f64::EPSILON
    {
        return Some(true);
    }
    if !delta.is_finite() || !par.is_finite() {
        None
    } else {
        Some(false)
    }
}

fn adjust_trust_region(
    actual: f64,
    derivative: f64,
    ratio: f64,
    norms: [f64; 3],
    delta: &mut f64,
    par: &mut f64,
) {
    let [fnorm1, fnorm, pnorm] = norms;
    if ratio <= 0.25 {
        let mut temp = if actual >= 0.0 {
            0.5
        } else {
            0.5 * derivative / (derivative + 0.5 * actual)
        };
        if 0.1 * fnorm1 >= fnorm || temp < 0.1 {
            temp = 0.1;
        }
        (*delta) = temp * (*delta).min(pnorm / 0.1);
        (*par) /= temp;
    } else if !((*par) != 0.0 && ratio < 0.75) {
        (*delta) = pnorm / 0.5;
        (*par) *= 0.5;
    }
}
