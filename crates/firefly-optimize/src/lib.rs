//! 数值优化。
//!
//! L-BFGS（有限内存 BFGS）求解无约束问题，配合强 Wolfe 线搜索。
//! 论文对比表明 L-BFGS 优于 Barzilai-Borwein 与截断牛顿（EGO-Planner v1 Sec. VI-B）。

mod lbfgs;

pub use lbfgs::{Lbfgs, LbfgsConfig, LbfgsReport, Objective};
