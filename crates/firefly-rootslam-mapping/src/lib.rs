//! `RootSLAM` mapping reconstructed from the paper, not a translation of its
//! unreleased implementation. Original Basalt is a separate dependency.
pub mod descriptor;
pub mod global_optimization;
pub mod global_problem;
pub mod loop_closure;
pub mod map;
pub mod pose_graph;
