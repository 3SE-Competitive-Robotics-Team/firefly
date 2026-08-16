//! L-BFGS 优化器。
//!
//! 两循环递归 + BB 初始 Hessian + 强 Wolfe 线搜索
//! （EGO-Planner v1 Sec. IV-B，论文对比表明 L-BFGS 优于
//! Barzilai-Borwein 与截断牛顿）。

use std::collections::VecDeque;

use firefly_error::{Error, ErrorKind, Result};
use nalgebra::DVector;

pub trait Objective {
    fn evaluate(&mut self, x: &DVector<f64>) -> f64;
    fn gradient(&mut self, x: &DVector<f64>) -> DVector<f64>;
    /// 目标函数是否请求提前终止（官方 earlyExitCallback：优化中动态更新约束后
    /// 目标已变，当前搜索方向无效，终止后由外层重新优化）。
    fn early_exit(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone)]
pub struct LbfgsConfig {
    pub memory: usize,
    pub max_iterations: usize,
    /// 梯度收敛：‖g‖∞ / max(1, ‖x‖∞) < ε（LBFGS-Lite `g_epsilon`）。
    pub gradient_epsilon: f64,
    /// 相对改进停止：`|f_prev − f| / max(1, |f|) < δ`，连续 `past` 次满足即收敛
    /// （官方 EGO-Planner v2：`past = 3, delta = 1e-2`）。
    pub delta: f64,
    pub past: usize,
    /// Armijo 系数 c1。
    pub f_dec_coeff: f64,
    /// 弱 Wolfe 曲率系数 c2。
    pub s_curv_coeff: f64,
    pub max_line_search: usize,
}

impl Default for LbfgsConfig {
    fn default() -> Self {
        Self {
            memory: 16,
            max_iterations: 200,
            gradient_epsilon: 1e-5,
            delta: 1e-2,
            past: 3,
            f_dec_coeff: 1e-4,
            s_curv_coeff: 0.9,
            max_line_search: 200,
        }
    }
}

#[derive(Debug, Clone)]
pub struct LbfgsReport {
    pub iterations: usize,
    pub final_cost: f64,
    pub gradient_norm: f64,
    pub converged: bool,
    /// 目标函数请求提前终止（约束在优化中动态更新）。
    pub early_exit: bool,
    pub final_x: DVector<f64>,
}

pub struct Lbfgs {
    config: LbfgsConfig,
}

impl Lbfgs {
    #[must_use]
    pub fn new(config: LbfgsConfig) -> Self {
        Self { config }
    }

    /// # Errors
    ///
    /// `Convergence`：搜索方向非下降方向或线搜索无法满足 Armijo 条件。
    #[fastrace::trace]
    // objective 为 trait 对象无 Debug：只记录初始点
    #[logcall::logcall("debug", input = "x0 = {x0:?}", output = "")]
    pub fn minimize(&self, objective: &mut dyn Objective, x0: DVector<f64>) -> Result<LbfgsReport> {
        let mut x = x0;
        let mut fx = objective.evaluate(&x);
        let mut g = objective.gradient(&x);

        let mut s_history: VecDeque<DVector<f64>> = VecDeque::with_capacity(self.config.memory);
        let mut y_history: VecDeque<DVector<f64>> = VecDeque::with_capacity(self.config.memory);
        let mut rho_history: VecDeque<f64> = VecDeque::with_capacity(self.config.memory);
        // past 收敛判据：连续 past 次相对改进小于 delta（官方 lbfgs `past/delta`）
        let mut past_improved = 0usize;

        for iter in 0..self.config.max_iterations {
            if objective.early_exit() {
                return Ok(LbfgsReport {
                    iterations: iter,
                    final_cost: fx,
                    gradient_norm: g.iter().fold(0.0f64, |m, v| m.max(v.abs())),
                    converged: false,
                    early_exit: true,
                    final_x: x.clone(),
                });
            }

            // 梯度收敛：‖g‖∞ / max(1, ‖x‖∞) < ε
            let gnorm_inf = g.iter().fold(0.0f64, |m, v| m.max(v.abs()));
            let xnorm_inf = x.iter().fold(0.0f64, |m, v| m.max(v.abs()));
            if gnorm_inf / xnorm_inf.max(1.0) < self.config.gradient_epsilon {
                return Ok(LbfgsReport {
                    iterations: iter,
                    final_cost: fx,
                    gradient_norm: gnorm_inf,
                    converged: true,
                    early_exit: false,
                    final_x: x.clone(),
                });
            }

            let direction = Self::search_direction(&g, &s_history, &y_history, &rho_history);
            let mut step = 1.0 / direction.norm().max(1e-300);

            let (x_next, f_next, g_next, step_taken) =
                self.line_search(objective, &x, fx, &g, &direction, &mut step)?;

            // 线搜索无进展（软失败）：立即返回，避免后续迭代空转
            if step_taken < 1e-14 {
                return Ok(LbfgsReport {
                    iterations: iter,
                    final_cost: fx,
                    gradient_norm: g.iter().fold(0.0f64, |m, v| m.max(v.abs())),
                    converged: false,
                    early_exit: false,
                    final_x: x.clone(),
                });
            }

            // 相对改进停止（官方 liblbfgs）：连续 past 次 |Δf|/max(1,|f|) < δ
            // 且步长 < xtol×max(1,|x|)——缺步长条件会在轨迹大步移动但改进
            // 缓慢时假收敛（障碍梯度未被推离）
            let rate = (fx - f_next).abs() / f_next.abs().max(1.0);
            if rate < self.config.delta {
                past_improved += 1;
            } else {
                past_improved = 0;
            }
            let xtol = 1e-16 * x_next.norm().max(1.0);
            if past_improved >= self.config.past && step_taken < xtol && iter > 0 {
                return Ok(LbfgsReport {
                    iterations: iter,
                    final_cost: f_next,
                    gradient_norm: g_next.iter().fold(0.0f64, |m, v| m.max(v.abs())),
                    converged: true,
                    early_exit: false,
                    final_x: x_next.clone(),
                });
            }

            if step_taken > 1e-14 {
                let s_vec = step_taken * &direction;
                let y_vec = &g_next - &g;
                let ys = y_vec.dot(&s_vec);
                if ys > 1e-10 {
                    let rho = 1.0 / ys;
                    s_history.push_back(s_vec);
                    y_history.push_back(y_vec);
                    rho_history.push_back(rho);
                    if s_history.len() > self.config.memory {
                        s_history.pop_front();
                        y_history.pop_front();
                        rho_history.pop_front();
                    }
                }
            }

            x = x_next;
            fx = f_next;
            g = g_next;
        }

        // 迭代上限：带解返回（调用方可接受继续），而非硬错误
        Ok(LbfgsReport {
            iterations: self.config.max_iterations,
            final_cost: fx,
            gradient_norm: g.iter().fold(0.0f64, |m, v| m.max(v.abs())),
            converged: false,
            early_exit: false,
            final_x: x,
        })
    }

    fn search_direction(
        grad: &DVector<f64>,
        s_history: &VecDeque<DVector<f64>>,
        y_history: &VecDeque<DVector<f64>>,
        rho_history: &VecDeque<f64>,
    ) -> DVector<f64> {
        let mut q = grad.clone();
        let mut alphas = Vec::with_capacity(s_history.len());
        for i in (0..s_history.len()).rev() {
            let alpha = rho_history[i] * s_history[i].dot(&q);
            q -= alpha * &y_history[i];
            alphas.push(alpha);
        }
        alphas.reverse();

        let h0 = Self::initial_hessian(s_history, y_history);
        let mut r = h0 * &q;
        for i in 0..s_history.len() {
            let beta = rho_history[i] * y_history[i].dot(&r);
            r += &s_history[i] * (alphas[i] - beta);
        }
        -r
    }

    fn initial_hessian(
        s_history: &VecDeque<DVector<f64>>,
        y_history: &VecDeque<DVector<f64>>,
    ) -> f64 {
        if let (Some(s), Some(y)) = (s_history.back(), y_history.back()) {
            let ys = y.dot(s);
            let yy = y.dot(y);
            if yy > 1e-12 {
                return ys / yy;
            }
        }
        1.0
    }

    /// Lewis-Overton 线搜索（LBFGS-Lite）：弱 Wolfe 条件 + 区间二分。
    /// 返回 (新 x, 新 f, 新 g, 实际步长)。
    #[allow(clippy::too_many_arguments)]
    fn line_search(
        &self,
        objective: &mut dyn Objective,
        x: &DVector<f64>,
        fx: f64,
        g: &DVector<f64>,
        direction: &DVector<f64>,
        stp: &mut f64,
    ) -> Result<(DVector<f64>, f64, DVector<f64>, f64)> {
        let dginit = g.dot(direction);
        if dginit >= 0.0 {
            return Err(Error::new(
                ErrorKind::Convergence,
                "search direction is not a descent direction",
            ));
        }

        let finit = fx;
        let dgtest = self.config.f_dec_coeff * dginit;
        let curv_test = self.config.s_curv_coeff * dginit;
        let mut brackt = false;
        let mut mu = 0.0;
        let mut nu = 1.0e20;
        let mut count = 0usize;

        loop {
            let candidate = x + *stp * direction;
            let cf = objective.evaluate(&candidate);
            if !cf.is_finite() {
                return Err(Error::new(
                    ErrorKind::Convergence,
                    "line search: non-finite objective",
                ));
            }
            log::debug!(
                "ls: stp={stp:.3e} f={fx:.6e} cf={cf:.6e} armijo_ok={}",
                cf <= finit + *stp * dgtest
            );
            if cf > finit + *stp * dgtest {
                // Armijo 失败：上界收缩
                nu = *stp;
                brackt = true;
            } else {
                let cg = objective.gradient(&candidate);
                if cg.dot(direction) < curv_test {
                    // 曲率不足：下界提升
                    mu = *stp;
                } else {
                    // 弱 Wolfe 满足
                    return Ok((candidate, cf, cg, *stp));
                }
            }
            count += 1;
            // 软失败：超限/区间收缩到数值极限时返回原地（步长 0），
            // 让优化器的收敛判据（相对改进停止）接管，而非整体报错。
            if count >= self.config.max_line_search
                || (brackt && (nu - mu) < 1e-16 * nu.max(1e-300))
            {
                return Ok((x.clone(), fx, g.clone(), 0.0));
            }
            *stp = if brackt { 0.5 * (mu + nu) } else { *stp * 2.0 };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Quadratic {
        a: f64,
    }

    impl Objective for Quadratic {
        fn evaluate(&mut self, x: &DVector<f64>) -> f64 {
            self.a * x.dot(x)
        }

        fn gradient(&mut self, x: &DVector<f64>) -> DVector<f64> {
            2.0 * self.a * x
        }
    }

    struct Rosenbrock;

    impl Objective for Rosenbrock {
        fn evaluate(&mut self, x: &DVector<f64>) -> f64 {
            let (a, b) = (x[0], x[1]);
            (1.0 - a).powi(2) + 100.0 * (b - a * a).powi(2)
        }

        fn gradient(&mut self, x: &DVector<f64>) -> DVector<f64> {
            let (a, b) = (x[0], x[1]);
            DVector::from_vec(vec![
                -2.0 * (1.0 - a) - 400.0 * a * (b - a * a),
                200.0 * (b - a * a),
            ])
        }
    }

    #[test]
    fn converges_on_quadratic() {
        let lbfgs = Lbfgs::new(LbfgsConfig::default());
        let mut obj = Quadratic { a: 1.0 };
        let report = lbfgs
            .minimize(&mut obj, DVector::from_vec(vec![3.0, -2.0, 1.0]))
            .unwrap();
        assert!(report.converged);
        // 官方判据：梯度相对收敛（‖g‖∞/max(1,‖x‖∞) < 1e-5）或相对改进停止
        assert!(report.final_cost < 1e-3, "cost: {}", report.final_cost);
    }

    #[test]
    fn converges_on_rosenbrock() {
        let config = LbfgsConfig {
            max_iterations: 1000,
            delta: 1e-6,
            ..LbfgsConfig::default()
        };
        let lbfgs = Lbfgs::new(config);
        let mut obj = Rosenbrock;
        let report = lbfgs
            .minimize(&mut obj, DVector::from_vec(vec![-1.2, 1.0]))
            .unwrap();
        assert!(report.converged);
        // 官方判据下 Rosenbrock 收敛到极小点附近（相对改进停止）
        assert!(report.final_cost < 1e-2, "cost: {}", report.final_cost);
    }

    #[test]
    fn iteration_limit_returns_unconverged_report() {
        let config = LbfgsConfig {
            max_iterations: 2,
            ..LbfgsConfig::default()
        };
        let lbfgs = Lbfgs::new(config);
        let mut obj = Rosenbrock;
        let report = lbfgs
            .minimize(&mut obj, DVector::from_vec(vec![5.0, 5.0]))
            .unwrap();
        assert!(!report.converged);
        assert_eq!(report.iterations, 2);
        assert_eq!(report.final_x.len(), 2, "解必须随报告返回");
    }
}
