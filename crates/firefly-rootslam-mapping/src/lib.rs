//! Independent implementation of the `RootSLAM` mapping methods.
//! Local estimation uses the Basalt dependency.
pub mod descriptor;
pub mod edge_information;
pub mod global_optimization;
pub mod global_problem;
pub mod loop_closure;
pub mod map;
pub mod pose_graph;
