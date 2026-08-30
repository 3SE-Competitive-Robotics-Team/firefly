//! small-gicp 忠实纯 Rust 移植（ICRA 2024, Koide3）。
//!
//! 模块组织 1:1 镜像官方 `include/small_gicp`，点云面向 trait 编程，
//! 并行统一用 `rayon`（替代 C++ OpenMP/TBB），线性代数用 `nalgebra`。
//!
//! - `points`：`traits` / `point_cloud` / `Vec<Vector3/4>` 特化（对照 `points/`）。
//! - `util`：体素降采样、法向/协方差估计、李代数、快速取整、空间哈希（对照 `util/`）。
//! - `ann`：`KdTree` / `KnnResult` / `FlatContainer` / `GaussianVoxel` /
//!   `IncrementalVoxelMap` / `ProjectiveSearch` / `SequentialAccessor`（对照 `ann/`）。
//! - `factors`：`ICPFactor` / `PlaneICPFactor` / `GICPFactor` / `RobustKernel` /
//!   `GeneralFactor`（对照 `factors/`）。
//! - `registration`：`SerialReduction` / `ParallelReduction` / `GaussNewton` /
//!   `LevenbergMarquardt` / `TerminationCriteria` / `Rejector` / `Registration` /
//!   `helper`（对照 `registration/`）。

#![allow(clippy::pedantic)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_possible_wrap)]
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::cast_sign_loss)]

pub mod ann;
pub mod factors;
pub mod points;
pub mod registration;
pub mod util;
