//! L-BFGS 优化器。
//!
//! liblbfgs 衍生版（More-Thuente 线搜索 + 两循环递归）的忠实移植，
//! 严格对照 EGO-Planner-v2 捆绑的
//! `swarm-playground/*/src/planner/traj_opt/include/optimizer/lbfgs.hpp`：
//! 算法、数值公式、收敛判据与返回码语义一一对应。
//! EGO 实际参数覆盖见 `LbfgsConfig::default`。

use firefly_error::{Error, ErrorKind, Result};
use nalgebra::DVector;

/// 对照官方返回码枚举（负值为错误）。
const CONVERGENCE: i32 = 0;
const STOP: i32 = 1;
const ALREADY_MINIMIZED: i32 = 2;
const ERR_OUTOFINTERVAL: i32 = -1038;
const ERR_INCORRECT_TMINMAX: i32 = -1039;
const ERR_ROUNDING_ERROR: i32 = -1040;
const ERR_MINIMUMSTEP: i32 = -1041;
const ERR_MAXIMUMSTEP: i32 = -1042;
const ERR_MAXIMUMLINESEARCH: i32 = -1043;
const ERR_MAXIMUMITERATION: i32 = -1044;
const ERR_WIDTHTOOSMALL: i32 = -1045;
const ERR_INVALIDPARAMETERS: i32 = -1046;
const ERR_INCREASEGRADIENT: i32 = -1047;

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
    /// 梯度收敛：‖g‖₂ / max(1, ‖x‖₂) ≤ ε（欧氏 L2 范数，官方 `g_epsilon`）。
    pub gradient_epsilon: f64,
    /// 相对改进停止（官方单次比较）：`|(pf[k%past] − f) / f| < δ` 即停，
    /// `past > 0` 时启用；`k < past` 期间不检测。
    pub delta: f64,
    pub past: usize,
    /// 充分下降系数（Armijo），官方 `f_dec_coeff`。
    pub f_dec_coeff: f64,
    /// 曲率系数，官方 `s_curv_coeff`；须满足 `f_dec_coeff < s_curv_coeff < 1`。
    pub s_curv_coeff: f64,
    /// 曲率条件选择：true 为强 Wolfe（|dg| 有界），false 为弱 Wolfe（dg 有下界）。
    pub abs_curv_cond: bool,
    pub max_line_search: usize,
    /// 线搜索步长下界（EGO 覆盖默认 1e-20 为 1e-32）。
    pub min_step: f64,
    /// 线搜索步长上界。
    pub max_step: f64,
    /// 不确定区间相对宽度终止阈值（官方 `xtol`）。
    pub xtol: f64,
}

impl Default for LbfgsConfig {
    /// EGO-Planner v2 实际使用值（`poly_traj_optimizer.cpp` 在官方默认基础上
    /// 覆盖 `memory`、`max_iterations`、`min_step`、`past`、`delta`），
    /// 其余取官方默认。
    fn default() -> Self {
        Self {
            memory: 16,
            max_iterations: 200,
            gradient_epsilon: 1e-5,
            delta: 1e-2,
            past: 3,
            f_dec_coeff: 1e-4,
            s_curv_coeff: 0.9,
            abs_curv_cond: true,
            max_line_search: 40,
            min_step: 1e-32,
            max_step: 1e20,
            xtol: 1e-16,
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

/// 单次修正对（s, y）及其内积缓存（官方 `iteration_data_t`）。
struct IterationData {
    alpha: f64,
    s: DVector<f64>,
    y: DVector<f64>,
    ys: f64,
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
    /// `InvalidArgument`：参数不满足官方校验约束。
    /// `Convergence`：线搜索失败（舍入误差、区间塌缩、评估次数超限等）。
    #[fastrace::trace]
    // objective 为 trait 对象无 Debug：只记录初始点
    #[logcall::logcall("debug", input = "x0 = {x0:?}", output = "")]
    // 变量名（x/g/d/m/k/end/pf）对照官方 lbfgs_optimize，便于逐行核对
    #[allow(clippy::many_single_char_names)]
    pub fn minimize(&self, objective: &mut dyn Objective, x0: DVector<f64>) -> Result<LbfgsReport> {
        let cfg = self.config.clone();
        Self::validate_config(&cfg)?;

        let n = x0.len();
        let mut x = x0;
        let mut fx = objective.evaluate(&x);
        let mut g = objective.gradient(&x);

        // 官方工作区：环形缓冲 lm[m]（end 指针推进）、过去 past 次目标值 pf
        let m = cfg.memory;
        let mut lm: Vec<IterationData> = (0..m)
            .map(|_| IterationData {
                alpha: 0.0,
                s: DVector::zeros(n),
                y: DVector::zeros(n),
                ys: 0.0,
            })
            .collect();
        let mut pf: Vec<f64> = vec![0.0; cfg.past];
        if cfg.past > 0 {
            pf[0] = fx;
        }

        // 初始方向 H₀ = I：d = −g；初始步长 1/‖d‖（vec2norminv）
        let mut d = -g.clone();
        let mut step = 1.0 / d.norm();

        let mut xnorm = x.norm();
        if xnorm < 1.0 {
            xnorm = 1.0;
        }
        let gnorm = g.norm();
        if gnorm / xnorm <= cfg.gradient_epsilon {
            return Ok(report(0, fx, &g, true, false, x));
        }

        // k/end 与官方一致：k 从 1 起、每次有效更新 ++k；end 为环形缓冲写指针
        let mut k = 1usize;
        let mut end = 0usize;
        let mut iterations = 0usize;

        let ret = loop {
            // firefly 语义：每次迭代前询问目标是否请求提前终止
            // （对应官方 proc_progress 返回非零取消优化的外层重优化路径）
            if objective.early_exit() {
                return Ok(report(iterations, fx, &g, false, true, x));
            }

            let xp = x.clone();
            let gp = g.clone();
            let step_min = cfg.min_step;
            let step_max = cfg.max_step;

            match line_search_morethuente(
                objective, &mut x, &mut fx, &mut g, &d, &mut step, &xp, step_min, step_max, &cfg,
            ) {
                Ok(_) => {}
                Err(code) => {
                    // 回退到线搜索前的点与梯度（官方语义）
                    x = xp;
                    g = gp;
                    break code;
                }
            }
            iterations += 1;

            // 梯度收敛：‖g‖₂ / max(1, ‖x‖₂) ≤ ε
            let mut xnorm = x.norm();
            if xnorm < 1.0 {
                xnorm = 1.0;
            }
            let gnorm = g.norm();
            if gnorm / xnorm <= cfg.gradient_epsilon {
                break CONVERGENCE;
            }

            // 相对改进停止（单次比较）：|(pf[k%past] − f) / f| < δ
            if cfg.past > 0 {
                if cfg.past <= k {
                    let rate = (pf[k % cfg.past] - fx) / fx;
                    if rate.abs() < cfg.delta {
                        break STOP;
                    }
                }
                pf[k % cfg.past] = fx;
            }

            if cfg.max_iterations != 0 && cfg.max_iterations < k + 1 {
                break ERR_MAXIMUMITERATION;
            }

            // s/y 写入环形缓冲槽位 end
            let slot = &mut lm[end];
            slot.s = &x - &xp;
            slot.y = &g - &gp;
            let ys = slot.y.dot(&slot.s);
            let yy = slot.y.dot(&slot.y);
            slot.ys = ys;

            d = -g.clone();

            // ys 过小跳过 L-BFGS 更新（Ceres 式），保持 d = −g
            if ys > f64::EPSILON {
                let bound = m.min(k);
                k += 1;
                end = (end + 1) % m;
                two_loop_recursion(&mut lm, end, bound, &mut d, ys, yy);
            }

            step = 1.0;
        };

        match ret {
            ERR_MAXIMUMITERATION => Ok(report(iterations, fx, &g, false, false, x)),
            CONVERGENCE | STOP | ALREADY_MINIMIZED => {
                Ok(report(iterations, fx, &g, true, false, x))
            }
            code => Err(Error::new(ErrorKind::Convergence, strerror(code))),
        }
    }

    /// 参数校验，逐条对照 `lbfgs_optimize` 入口检查。
    fn validate_config(cfg: &LbfgsConfig) -> Result<()> {
        if cfg.memory == 0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "lbfgs: memory must be positive",
            ));
        }
        if cfg.gradient_epsilon < 0.0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "lbfgs: gradient_epsilon must be non-negative",
            ));
        }
        if cfg.delta < 0.0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "lbfgs: delta must be non-negative",
            ));
        }
        if cfg.min_step < 0.0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "lbfgs: min_step must be non-negative",
            ));
        }
        if cfg.max_step < cfg.min_step {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "lbfgs: max_step must be >= min_step",
            ));
        }
        if cfg.f_dec_coeff < 0.0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "lbfgs: f_dec_coeff must be non-negative",
            ));
        }
        if cfg.s_curv_coeff <= cfg.f_dec_coeff || 1.0 <= cfg.s_curv_coeff {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "lbfgs: requires f_dec_coeff < s_curv_coeff < 1",
            ));
        }
        if cfg.xtol < 0.0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "lbfgs: xtol must be non-negative",
            ));
        }
        if cfg.max_line_search == 0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "lbfgs: max_line_search must be positive",
            ));
        }
        Ok(())
    }
}

fn strerror(code: i32) -> &'static str {
    match code {
        ERR_OUTOFINTERVAL => "line-search step went out of the interval of uncertainty",
        ERR_INCORRECT_TMINMAX => "logic error; the interval of uncertainty became too small",
        ERR_ROUNDING_ERROR => "rounding error prevents further progress",
        ERR_MINIMUMSTEP => "line-search step became smaller than min_step",
        ERR_MAXIMUMSTEP => "line-search step became larger than max_step",
        ERR_MAXIMUMLINESEARCH => "line-search reached the maximum number of evaluations",
        ERR_MAXIMUMITERATION => "maximum number of iterations reached",
        ERR_WIDTHTOOSMALL => "relative width of the interval of uncertainty is at most xtol",
        ERR_INVALIDPARAMETERS => "invalid line-search step",
        ERR_INCREASEGRADIENT => "search direction increases the objective function value",
        _ => "unknown error",
    }
}

fn report(
    iterations: usize,
    final_cost: f64,
    g: &DVector<f64>,
    converged: bool,
    early_exit: bool,
    final_x: DVector<f64>,
) -> LbfgsReport {
    LbfgsReport {
        iterations,
        final_cost,
        gradient_norm: g.norm(),
        converged,
        early_exit,
        final_x,
    }
}

/// 两循环递归计算 d ← −H·g（Nocedal 1980，p.779）；
/// `j_start` 为最新修正对槽位，逆序遍历至最老修正对。
fn two_loop_recursion(
    lm: &mut [IterationData],
    j_start: usize,
    bound: usize,
    d: &mut DVector<f64>,
    ys: f64,
    yy: f64,
) {
    let m = lm.len();
    let mut j = j_start;
    for _ in 0..bound {
        j = (j + m - 1) % m;
        let it = &mut lm[j];
        it.alpha = d.dot(&it.s) / it.ys;
        d.axpy(-it.alpha, &it.y, 1.0);
    }
    // H₀ 缩放：γ = ys/yy
    *d *= ys / yy;
    for _ in 0..bound {
        let it = &lm[j];
        let beta = it.y.dot(&*d) / it.ys;
        d.axpy(it.alpha - beta, &it.s, 1.0);
        j = (j + 1) % m;
    }
}

/// 三次插值极小点（对照 `CUBIC_MINIMIZER_LBFGS`）。
// 变量名对照宏展开（u/fu/du/v/fv/dv），便于逐行核对
#[allow(clippy::many_single_char_names)]
fn cubic_minimizer(u: f64, fu: f64, du: f64, v: f64, fv: f64, dv: f64) -> f64 {
    let d = v - u;
    let theta = (fu - fv) * 3.0 / d + du + dv;
    let p = theta.abs();
    let q = du.abs();
    let r = dv.abs();
    let s = p.max(q).max(r);
    // gamm = s*sqrt((theta/s)**2 - (du/s)*(dv/s))
    let a = theta / s;
    let mut gamm = s * (a * a - (du / s) * (dv / s)).sqrt();
    if v < u {
        gamm = -gamm;
    }
    let p = gamm - du + theta;
    let q = gamm - du + gamm + dv;
    let r = p / q;
    u + r * d
}

/// 带区间保护的三次插值极小点（对照 `CUBIC_MINIMIZER2_LBFGS`）。
// 变量名与参数表对照宏展开，便于逐行核对
#[allow(clippy::many_single_char_names, clippy::too_many_arguments)]
fn cubic_minimizer2(
    u: f64,
    fu: f64,
    du: f64,
    v: f64,
    fv: f64,
    dv: f64,
    xmin: f64,
    xmax: f64,
) -> f64 {
    let d = v - u;
    let theta = (fu - fv) * 3.0 / d + du + dv;
    let p = theta.abs();
    let q = du.abs();
    let r = dv.abs();
    let s = p.max(q).max(r);
    let a = theta / s;
    let discr = a * a - (du / s) * (dv / s);
    let mut gamm = if discr > 0.0 { s * discr.sqrt() } else { 0.0 };
    if u < v {
        gamm = -gamm;
    }
    let p = gamm - dv + theta;
    let q = gamm - dv + gamm + du;
    let r = p / q;
    if r < 0.0 && gamm != 0.0 {
        v - r * d
    } else if d > 0.0 {
        xmax
    } else {
        xmin
    }
}

/// 二次插值极小点（对照 `QUARD_MINIMIZER_LBFGS`）。
fn quad_minimizer(u: f64, fu: f64, du: f64, v: f64, fv: f64) -> f64 {
    let a = v - u;
    u + du / ((fu - fv) / a + du) / 2.0 * a
}

/// 割线二次插值极小点（对照 `QUARD_MINIMIZER2_LBFGS`）。
fn quad_minimizer2(u: f64, du: f64, v: f64, dv: f64) -> f64 {
    let a = u - v;
    v + dv / (dv - du) * a
}

/// 更新保护区间与试探步长（More & Thuente 1994，对照 `update_trial_interval`）。
/// 返回 0 表示正常；非零为官方错误码，调用方记入 uinfo 而不立即中止。
#[allow(clippy::too_many_arguments)]
fn update_trial_interval(
    x: &mut f64,
    fx: &mut f64,
    dx: &mut f64,
    y: &mut f64,
    fy: &mut f64,
    dy: &mut f64,
    t: &mut f64,
    ft: &mut f64,
    dt: &mut f64,
    tmin: f64,
    tmax: f64,
    brackt: &mut bool,
) -> i32 {
    let dsign = *dt * (*dx / dx.abs()) < 0.0;
    let bound: bool;

    if *brackt {
        let (lo, hi) = if *x <= *y { (*x, *y) } else { (*y, *x) };
        if *t <= lo || hi <= *t {
            return ERR_OUTOFINTERVAL;
        }
        // 函数值必须从 x 起下降
        if 0.0 <= *dx * (*t - *x) {
            return ERR_INCREASEGRADIENT;
        }
        if tmax < tmin {
            return ERR_INCORRECT_TMINMAX;
        }
    }

    // 试探值选择：4 种 case 的 cubic/quadratic 插值取舍
    let newt;
    if *fx < *ft {
        // Case 1：函数值更高，最小值被夹住；cubic 极小点更近取 cubic，否则取两者均值
        *brackt = true;
        bound = true;
        let mc = cubic_minimizer(*x, *fx, *dx, *t, *ft, *dt);
        let mq = quad_minimizer(*x, *fx, *dx, *t, *ft);
        newt = if (mc - *x).abs() < (mq - *x).abs() {
            mc
        } else {
            mc + 0.5 * (mq - mc)
        };
    } else if dsign {
        // Case 2：函数值更低且导数变号；cubic 极小点更远取 cubic，否则取割线二次
        *brackt = true;
        bound = false;
        let mc = cubic_minimizer(*x, *fx, *dx, *t, *ft, *dt);
        let mq = quad_minimizer2(*x, *dx, *t, *dt);
        newt = if (mc - *t).abs() > (mq - *t).abs() {
            mc
        } else {
            mq
        };
    } else if dt.abs() < dx.abs() {
        // Case 3：函数值更低、导数同号且幅值减小；
        // brackt 时取离 t 近的极小点，否则取远的
        bound = true;
        let mc = cubic_minimizer2(*x, *fx, *dx, *t, *ft, *dt, tmin, tmax);
        let mq = quad_minimizer2(*x, *dx, *t, *dt);
        newt = if *brackt {
            if (*t - mc).abs() < (*t - mq).abs() {
                mc
            } else {
                mq
            }
        } else {
            if (*t - mc).abs() > (*t - mq).abs() {
                mc
            } else {
                mq
            }
        };
    } else {
        // Case 4：函数值更低、导数同号且幅值不减；
        // 未夹住时直接取区间端点，已夹住时取对端点的 cubic 极小点
        bound = false;
        newt = if *brackt {
            cubic_minimizer(*t, *ft, *dt, *y, *fy, *dy)
        } else if *x < *t {
            tmax
        } else {
            tmin
        };
    }

    // 区间更新（与试探值选择无关）：
    // Case a: f(x) < f(t) → y ← t；Case c: 导数变号 → y ← x；
    // 随后一律 x ← t
    if *fx < *ft {
        *y = *t;
        *fy = *ft;
        *dy = *dt;
    } else {
        if dsign {
            *y = *x;
            *fy = *fx;
            *dy = *dx;
        }
        *x = *t;
        *fx = *ft;
        *dx = *dt;
    }

    // clip 到 [tmin, tmax]
    let mut newt = newt.min(tmax).max(tmin);

    // 已夹住且本 case 收缩了区间时，避免试探值贴上界
    if *brackt && bound {
        let mq = *x + 0.66 * (*y - *x);
        if *x < *y {
            if mq < newt {
                newt = mq;
            }
        } else if newt < mq {
            newt = mq;
        }
    }

    // 已夹住时避免试探值贴下界
    if *brackt {
        let mq = if *x < *y {
            *x + 0.01 * (*y - *x)
        } else {
            *y + 0.01 * (*x - *y)
        };
        if newt < mq {
            newt = mq;
        }
    }

    *t = newt;
    0
}

/// More-Thuente 线搜索（对照 `line_search_morethuente`）。
/// 成功返回评估次数；Err 携带官方错误码。
// 步长被 clip 成恰好等于边界，官方用精确相等判定触界；stx/sty、dgxm/dgym
// 等成对变量名保留官方命名，便于逐行核对；主循环为官方控制流的逐行移植，
// 拆分会破坏对照性
#[allow(clippy::float_cmp, clippy::similar_names, clippy::too_many_lines)]
#[allow(clippy::too_many_arguments)]
fn line_search_morethuente(
    objective: &mut dyn Objective,
    x: &mut DVector<f64>,
    f: &mut f64,
    g: &mut DVector<f64>,
    s: &DVector<f64>,
    stp: &mut f64,
    xp: &DVector<f64>,
    stpmin: f64,
    stpmax: f64,
    cfg: &LbfgsConfig,
) -> std::result::Result<usize, i32> {
    if *stp <= 0.0 {
        return Err(ERR_INVALIDPARAMETERS);
    }

    let dginit = g.dot(s);
    // 必须是下降方向
    if 0.0 < dginit {
        return Err(ERR_INCREASEGRADIENT);
    }

    let finit = *f;
    let dgtest = cfg.f_dec_coeff * dginit;
    let mut brackt = false;
    let mut stage1 = true;
    let mut uinfo = 0i32;
    let mut count = 0usize;
    let mut width = stpmax - stpmin;
    let mut prev_width = 2.0 * width;

    // (stx, fx, dgx)：较优点；(sty, fy, dgy)：区间另一端；(stp, f, dg)：当前试探点
    let mut stx = 0.0f64;
    let mut fx = finit;
    let mut dgx = dginit;
    let mut sty = 0.0f64;
    let mut fy = finit;
    let mut dgy = dginit;

    loop {
        // 区间端点随 brackt 状态切换
        let (stmin, stmax) = if brackt {
            (stx.min(sty), stx.max(sty))
        } else {
            (stx, *stp + 4.0 * (*stp - stx))
        };

        // clip 到 [stpmin, stpmax]
        if *stp < stpmin {
            *stp = stpmin;
        }
        if stpmax < *stp {
            *stp = stpmax;
        }

        // 异常终止在即：退回目前最优点
        if brackt
            && (*stp <= stmin || stmax <= *stp || uinfo != 0 || stmax - stmin <= cfg.xtol * stmax)
        {
            *stp = stx;
        }

        // x ← xp + stp·s，评估目标与梯度
        x.copy_from(xp);
        x.axpy(*stp, s, 1.0);
        *f = objective.evaluate(x);
        *g = objective.gradient(x);
        let mut dg = g.dot(s);

        let ftest1 = finit + *stp * dgtest;
        count += 1;

        // 错误与收敛判定（顺序与官方一致）
        if !f.is_finite() || (brackt && (*stp <= stmin || stmax <= *stp || uinfo != 0)) {
            return Err(ERR_ROUNDING_ERROR);
        }
        if *stp == stpmax && *f <= ftest1 && dg <= dgtest {
            return Err(ERR_MAXIMUMSTEP);
        }
        if *stp == stpmin && (ftest1 < *f || dgtest <= dg) {
            return Err(ERR_MINIMUMSTEP);
        }
        if brackt && (stmax - stmin) <= cfg.xtol * stmax {
            return Err(ERR_WIDTHTOOSMALL);
        }
        if cfg.max_line_search <= count {
            return Err(ERR_MAXIMUMLINESEARCH);
        }
        // 充分下降 + 曲率条件（强/弱由 abs_curv_cond 决定）
        let curv_ok = if cfg.abs_curv_cond {
            dg.abs() <= cfg.s_curv_coeff * (-dginit)
        } else {
            -dg <= cfg.s_curv_coeff * (-dginit)
        };
        if *f <= ftest1 && curv_ok {
            return Ok(count);
        }

        // 第一阶段寻找修正函数 fm = f − stp·dgtest 的非正值的非负导数点
        if stage1 && *f <= ftest1 && cfg.f_dec_coeff.min(cfg.s_curv_coeff) * dginit <= dg {
            stage1 = false;
        }

        // 修正函数预测仅在「下降不足但函数值更低」时启用
        if stage1 && ftest1 < *f && *f <= fx {
            let mut fm = *f - *stp * dgtest;
            let mut fxm = fx - stx * dgtest;
            let mut fym = fy - sty * dgtest;
            let mut dgm = dg - dgtest;
            let mut dgxm = dgx - dgtest;
            let mut dgym = dgy - dgtest;

            uinfo = update_trial_interval(
                &mut stx,
                &mut fxm,
                &mut dgxm,
                &mut sty,
                &mut fym,
                &mut dgym,
                stp,
                &mut fm,
                &mut dgm,
                stmin,
                stmax,
                &mut brackt,
            );

            // 由修正值还原 f/dg 缓存
            fx = fxm + stx * dgtest;
            fy = fym + sty * dgtest;
            dgx = dgxm + dgtest;
            dgy = dgym + dgtest;
        } else {
            uinfo = update_trial_interval(
                &mut stx,
                &mut fx,
                &mut dgx,
                &mut sty,
                &mut fy,
                &mut dgy,
                stp,
                f,
                &mut dg,
                stmin,
                stmax,
                &mut brackt,
            );
        }

        // 夹住后强制区间充分收缩
        if brackt {
            if 0.66 * prev_width <= (sty - stx).abs() {
                *stp = stx + 0.5 * (sty - stx);
            }
            prev_width = width;
            width = (sty - stx).abs();
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

    struct AlwaysExit;

    impl Objective for AlwaysExit {
        fn evaluate(&mut self, x: &DVector<f64>) -> f64 {
            x.dot(x)
        }

        fn gradient(&mut self, x: &DVector<f64>) -> DVector<f64> {
            2.0 * x
        }

        fn early_exit(&self) -> bool {
            true
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
        // 官方判据：‖g‖₂/max(1,‖x‖₂) ≤ 1e-5 或相对改进停止
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
    fn strong_and_weak_wolfe_both_converge() {
        for abs_curv_cond in [true, false] {
            let config = LbfgsConfig {
                abs_curv_cond,
                max_iterations: 1000,
                delta: 1e-6,
                ..LbfgsConfig::default()
            };
            let lbfgs = Lbfgs::new(config);
            let mut obj = Rosenbrock;
            let report = lbfgs
                .minimize(&mut obj, DVector::from_vec(vec![-1.2, 1.0]))
                .unwrap();
            assert!(report.converged, "abs_curv_cond={abs_curv_cond}");
            assert!(
                report.final_cost < 1e-2,
                "abs_curv_cond={abs_curv_cond}, cost: {}",
                report.final_cost
            );
        }
    }

    #[test]
    fn invalid_memory_rejected() {
        let config = LbfgsConfig {
            memory: 0,
            ..LbfgsConfig::default()
        };
        let lbfgs = Lbfgs::new(config);
        let mut obj = Quadratic { a: 1.0 };
        let err = lbfgs
            .minimize(&mut obj, DVector::from_vec(vec![1.0]))
            .unwrap_err();
        assert_eq!(err.kind(), ErrorKind::InvalidArgument);
    }

    #[test]
    fn invalid_s_curv_coeff_rejected() {
        let config = LbfgsConfig {
            s_curv_coeff: 1.0,
            ..LbfgsConfig::default()
        };
        let lbfgs = Lbfgs::new(config);
        let mut obj = Quadratic { a: 1.0 };
        let err = lbfgs
            .minimize(&mut obj, DVector::from_vec(vec![1.0]))
            .unwrap_err();
        assert_eq!(err.kind(), ErrorKind::InvalidArgument);
    }

    #[test]
    fn already_minimized_at_stationary_point() {
        let lbfgs = Lbfgs::new(LbfgsConfig::default());
        let mut obj = Quadratic { a: 1.0 };
        let report = lbfgs
            .minimize(&mut obj, DVector::from_vec(vec![0.0, 0.0]))
            .unwrap();
        // ALREADY_MINIMIZED 语义：初始点即极小点 → converged 且零迭代
        assert!(report.converged);
        assert_eq!(report.iterations, 0);
        assert!(report.final_cost.abs() < 1e-12);
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

    #[test]
    fn early_exit_reports_flag() {
        let lbfgs = Lbfgs::new(LbfgsConfig::default());
        let mut obj = AlwaysExit;
        let report = lbfgs
            .minimize(&mut obj, DVector::from_vec(vec![1.0, 1.0]))
            .unwrap();
        assert!(report.early_exit);
        assert!(!report.converged);
        assert_eq!(report.iterations, 0);
        assert_eq!(report.final_x.len(), 2);
    }
}
