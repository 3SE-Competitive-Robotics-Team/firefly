//! 无约束 L-BFGS，配合 More–Thuente 线搜索；默认使用强 Wolfe 条件。

mod lbfgs;

pub use lbfgs::{Lbfgs, LbfgsConfig, LbfgsReport, Objective};
