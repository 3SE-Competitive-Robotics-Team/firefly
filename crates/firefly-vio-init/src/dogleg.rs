//! Traditional DOGLEG with Ceres 2.2 trust-region scaling and step-quality rules.
//! `OpenVINS` uses function tolerance 1e-5 and gradient tolerance 1e-9.
//! Dense normal-equation factorization gives the same model step as Schur elimination;
//! only accepted, finite candidates can become the returned initialization state.
use nalgebra::{DMatrix, DVector};
use std::time::Instant;

pub(crate) struct Linearization {
    pub residual: DVector<f64>,
    pub jacobian: DMatrix<f64>,
    /// True objective, including 1/2 and the robust loss, not the IRLS squared norm.
    pub cost: f64,
}

impl Linearization {
    fn valid(&self) -> bool {
        self.cost.is_finite()
            && self.cost >= 0.
            && self.jacobian.nrows() == self.residual.len()
            && self.jacobian.ncols() > 0
            && self
                .residual
                .iter()
                .chain(self.jacobian.iter())
                .all(|x| x.is_finite())
    }
}

/// CauchyLoss(1): rho(s) = log(1+s). Ceres Corrector uses sqrt(rho') for
/// residuals and Jacobians because rho'' is negative; cost remains rho(s)/2.
pub(crate) fn cauchy(squared_norm: f64) -> (f64, f64) {
    (
        0.5 * squared_norm.ln_1p(),
        (1. + squared_norm).sqrt().recip(),
    )
}

/// Intersection of the traditional dogleg path with a spherical trust region.
fn step(gradient: &DVector<f64>, gn: &DVector<f64>, alpha: f64, radius: f64) -> DVector<f64> {
    if gn.norm() <= radius {
        return gn.clone();
    }
    if alpha * gradient.norm() >= radius {
        return gradient * (-radius / gradient.norm());
    }
    let cauchy = -alpha * gradient;
    let direction = gn - &cauchy;
    let cross = cauchy.dot(&direction);
    let remaining = radius * radius - cauchy.norm_squared();
    let root = (cross * cross + direction.norm_squared() * remaining).sqrt();
    let beta = if cross <= 0. {
        (root - cross) / direction.norm_squared()
    } else {
        remaining / (root + cross)
    };
    cauchy + beta * direction
}

fn regularized_gauss_newton(
    normal: &DMatrix<f64>,
    gradient: &DVector<f64>,
) -> Option<DVector<f64>> {
    let mut mu = 1e-8;
    while mu < 1. {
        let mut regularized = normal.clone();
        for i in 0..gradient.len() {
            regularized[(i, i)] += mu;
        }
        if let Some(chol) = regularized.cholesky() {
            let step = chol.solve(&(-gradient));
            if step.iter().all(|x| x.is_finite()) {
                return Some(step);
            }
        }
        mu *= 10.;
    }
    None
}

/// Rejected trials leave state and linearization untouched. Exhaustion, invalid
/// initial data and a collapsed trust region reject initialization rather than
/// certifying convergence. `plus` supplies the state's manifold retraction.
pub(crate) fn minimize<S>(
    mut state: S,
    max_iterations: usize,
    max_seconds: f64,
    evaluate: impl Fn(&S) -> Option<Linearization>,
    plus: impl Fn(&S, &DVector<f64>) -> S,
    parameters: impl Fn(&S) -> DVector<f64>,
) -> Option<(S, Linearization)> {
    let start = Instant::now();
    if !max_seconds.is_finite() || max_seconds <= 0. {
        return None;
    }
    let mut current = evaluate(&state).filter(Linearization::valid)?;
    if max_iterations == 0 {
        return Some((state, current));
    }
    let mut values = parameters(&state);
    let mut accepted = false;
    let mut radius = 1e4_f64;
    let mut iterations = 0;
    loop {
        if start.elapsed().as_secs_f64() >= max_seconds {
            log::warn!("[init-d]: DOGLEG time budget exhausted");
            return None;
        }
        let gradient = current.jacobian.transpose() * &current.residual;
        if gradient.amax() <= 1e-9 {
            return Some((state, current));
        }
        let diagonal = DVector::from_iterator(
            gradient.len(),
            current
                .jacobian
                .column_iter()
                .map(|column| column.norm_squared().clamp(1e-6, 1e32).sqrt()),
        );
        let mut scaled_j = current.jacobian.clone();
        for (i, mut column) in scaled_j.column_iter_mut().enumerate() {
            column /= diagonal[i];
        }
        let scaled_g = gradient.component_div(&diagonal);
        let normal = scaled_j.transpose() * &scaled_j;
        let gn = regularized_gauss_newton(&normal, &scaled_g)?;
        let alpha = scaled_g.norm_squared() / (&scaled_j * &scaled_g).norm_squared();
        if !alpha.is_finite() {
            return None;
        }
        loop {
            if iterations >= max_iterations
                || start.elapsed().as_secs_f64() >= max_seconds
                || radius <= 1e-32
            {
                log::warn!(
                    "[init-d]: DOGLEG did not converge (iterations={iterations}, radius={radius})"
                );
                return None;
            }
            iterations += 1;
            let scaled_step = step(&scaled_g, &gn, alpha, radius);
            let delta = scaled_step.component_div(&diagonal);
            let projected = &current.jacobian * &delta;
            let predicted = -gradient.dot(&delta) - 0.5 * projected.norm_squared();
            if !predicted.is_finite() || predicted <= 0. {
                return None;
            }
            let candidate = plus(&state, &delta);
            let candidate_values = parameters(&candidate);
            // Ceres parameter tolerance uses ambient coordinates after a successful step.
            if accepted
                && candidate_values.iter().all(|v| v.is_finite())
                && (&candidate_values - &values).norm() <= 1e-8 * (values.norm() + 1e-8)
            {
                return Some((state, current));
            }
            let trial = evaluate(&candidate).filter(Linearization::valid);
            if let Some(trial) = trial {
                let decrease = current.cost - trial.cost;
                // Ceres checks true relative cost change before accepting the trial.
                // Keep the accepted state, including when the trial cost rises slightly.
                if decrease.abs() <= 1e-5 * current.cost {
                    return Some((state, current));
                }
                let quality = decrease / predicted;
                if decrease >= 0. && quality > 1e-3 {
                    log::debug!(
                        "[init-d]: DOGLEG cost={} => {}, quality={quality}",
                        current.cost,
                        trial.cost
                    );
                    accepted = true;
                    values = candidate_values;
                    state = candidate;
                    current = trial;
                    if quality < 0.25 {
                        radius *= 0.5;
                    } else if quality > 0.75 {
                        radius = radius.max(3. * scaled_step.norm()).min(1e16);
                    }
                    break;
                }
            }
            radius *= 0.5;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cauchy_cost_and_gradient_match_analytic_values_and_differences() {
        let (cost, weight) = cauchy(1.);
        assert!((cost - 2_f64.ln() / 2.).abs() < 1e-15);
        assert!((weight - 1. / 2_f64.sqrt()).abs() < 1e-15);
        for x in [0.1_f64, 1., 100.] {
            let eps = 1e-5;
            let fd = (cauchy((x + eps).powi(2)).0 - cauchy((x - eps).powi(2)).0) / (2. * eps);
            assert!((fd - x / (1. + x * x)).abs() < 1e-10);
            assert!((cauchy(x * x).1.powi(2) * x - fd).abs() < 1e-10);
        }
        // Large residuals must not saturate at a unit cost.
        assert!(cauchy(101_f64.powi(2)).0 - cauchy(100_f64.powi(2)).0 > 0.009);
    }

    #[test]
    fn dogleg_has_gauss_newton_gradient_and_interpolated_branches() {
        let g = DVector::from_vec(vec![1., 0.]);
        let gn = DVector::from_vec(vec![-1., -2.]);
        assert!((step(&g, &gn, 0.5, 3.) - &gn).norm() < 1e-15);
        assert!((step(&g, &gn, 0.5, 0.25) - DVector::from_vec(vec![-0.25, 0.])).norm() < 1e-15);
        // (-.5-.5b)^2 + (-2b)^2 = 1.
        let beta = (-0.5 + 13_f64.sqrt()) / 8.5;
        let expected = DVector::from_vec(vec![-0.5 - 0.5 * beta, -2. * beta]);
        assert!((step(&g, &gn, 0.5, 1.) - expected).norm() < 1e-14);
    }

    #[allow(clippy::trivially_copy_pass_by_ref)] // Solver callback contract.
    fn scalar_parameters(x: &f64) -> DVector<f64> {
        DVector::from_element(1, *x)
    }

    #[allow(clippy::trivially_copy_pass_by_ref, clippy::unnecessary_wraps)] // Solver callback contract.
    fn nonlinear(x: &f64) -> Option<Linearization> {
        let r = x * x - 1.;
        Some(Linearization {
            residual: DVector::from_element(1, r),
            jacobian: DMatrix::from_element(1, 1, 2. * x),
            cost: 0.5 * r * r,
        })
    }

    #[test]
    fn rejected_and_invalid_trials_cannot_be_returned_as_converged() {
        let plus = |x: &f64, dx: &DVector<f64>| x + dx[0];
        // The first Newton step .1 -> 5.05 increases the true objective.
        assert!(minimize(0.1, 1, 5., nonlinear, plus, scalar_parameters).is_none());
        let (x, final_eval) = minimize(0.1, 100, 5., nonlinear, plus, scalar_parameters).unwrap();
        assert!((x - 1.).abs() < 1e-8);
        assert!(final_eval.cost < 1e-16);
        assert!((final_eval.cost - nonlinear(&x).unwrap().cost).abs() < 1e-30);
        let bounded = |x: &f64| if *x > 2. { None } else { nonlinear(x) };
        let (x, _) = minimize(0.1, 100, 5., bounded, plus, scalar_parameters).unwrap();
        assert!((x - 1.).abs() < 1e-8);
        assert!(minimize(f64::NAN, 100, 5., nonlinear, plus, scalar_parameters).is_none());
        assert!(minimize(0.1, 100, 0., nonlinear, plus, scalar_parameters).is_none());
    }

    #[test]
    fn robust_optimization_has_independent_expected_minimum() {
        // Nine inliers at zero plus a distant outlier: the optimum solves
        // 9x/(1+x²) + (x-100)/(1+(x-100)²) = 0, near 0.001111.
        let evaluate = |x: &f64| {
            let mut r = DVector::zeros(10);
            let mut j = DMatrix::zeros(10, 1);
            let mut cost = 0.;
            for i in 0..10 {
                let raw = x - if i == 9 { 100. } else { 0. };
                let (c, w) = cauchy(raw * raw);
                r[i] = w * raw;
                j[(i, 0)] = w;
                cost += c;
            }
            Some(Linearization {
                residual: r,
                jacobian: j,
                cost,
            })
        };
        let (x, final_eval) =
            minimize(0.5, 100, 5., evaluate, |x, d| x + d[0], scalar_parameters).unwrap();
        let expected = 0.001_111_013_7_f64;
        let minimum = f64::midpoint(
            9. * (1. + expected.powi(2)).ln(),
            (1. + (expected - 100.).powi(2)).ln(),
        );
        assert!(x.abs() < 0.002); // Ordinary least squares gives 10.
        assert!((final_eval.cost - minimum).abs() <= 1e-5 * minimum);
    }
}
