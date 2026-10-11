//! 有界会话路标地图；原始 VIO 快照只读，长期观测与回环几何独立维护。
pub mod descriptor;
pub mod geometry;
pub mod map;
pub use map::{Feature, Frame, Map, Options};
