//! 视觉先验地图：关键帧位姿 + 3D 路标 + 描述子（对照 VINS-Fusion 的
//! `pose_graph.txt` + `briefdes.dat` + `KeyFrame`）。
//!
//! 约定：地图离线一次建成、在线冻结（VINS-Fusion 的 sequence 0 语义）；
//! 在线查询用 VIO 位姿先验做空间邻域短名单（有先验，无需全局检索模型）。

pub mod io;
pub mod map;

pub use io::{load_map, save_map};
pub use map::{VisionKeyFrame, VisionMap, VisionMapPoint};
