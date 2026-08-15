//! MINCO（Minimum Control）轨迹领域。
//!
//! 轨迹由中间点 `q` 与分段时长 `T` 参数化，映射到分段多项式系数 `c`：
//! `M(T) c = b(q)`，带形矩阵，O(M) 线性复杂度求解与梯度传播。
//! 数学细节见 MINCO 论文（arXiv:2103.00190）与 EGO-Planner v2 补充材料 S6。

mod banded;
mod minco;

pub use minco::{Endpoint, Minco, MincoBuilder, Sample, SolverOrder, Trajectory};
