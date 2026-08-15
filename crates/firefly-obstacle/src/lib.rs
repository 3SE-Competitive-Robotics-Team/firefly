//! 动态障碍领域：运动模型与轨迹预测。
//!
//! 对应论文 Dynamic obstacle avoidance 节 + Swarm Playground
//! 官方 `moving_obstacles.cpp`：标准自行车模型（加速度响应 + 阻尼 + 限速），
//! 用当前控制输入外推未来轨迹，再以 MINCO 拟合为与无人机同构的预测轨迹，
//! 经 `firefly-cost::Peer` 通道让 planner 统一避让。

mod motion;
mod predict;

pub use motion::MovingObstacle;
pub use predict::{ObstaclePredictor, PredictorConfig};
