//! 动态障碍预测轨迹生成（官方 `predict_traj` 语义）。
//!
//! 用当前控制输入外推未来 horizon 秒，采样预测点，
//! 以 MINCO 拟合为与无人机轨迹同构的 3D 轨迹（z 固定）。

use firefly_error::{Error, ErrorKind, Result};
use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder, Trajectory};
use nalgebra::{Point3, Vector3};

use crate::motion::MovingObstacle;

#[derive(Debug, Clone)]
pub struct PredictorConfig {
    /// 预测时长（官方 `PRED_TIME` = 5.0s）。
    pub horizon: f64,
    /// MINCO 段数（官方 `SEG_NUM` = 10）。
    pub segments: usize,
    /// 外推步长。
    pub step: f64,
}

impl Default for PredictorConfig {
    fn default() -> Self {
        Self {
            horizon: 5.0,
            segments: 10,
            step: 0.1,
        }
    }
}

pub struct ObstaclePredictor {
    config: PredictorConfig,
}

impl ObstaclePredictor {
    #[must_use]
    pub fn new(config: PredictorConfig) -> Self {
        Self { config }
    }

    /// 生成预测轨迹：起点为当前位置/速度，中间点为外推采样，终点为外推末态。
    ///
    /// # Errors
    ///
    /// `InvalidArgument`：配置非法（horizon/segments 非正）。
    ///
    /// # Panics
    ///
    /// MINCO 求解奇异（理论上 T > 0 时不会发生）。
    #[fastrace::trace]
    #[logcall::logcall("debug", output = "")]
    pub fn predict_trajectory(
        &self,
        obstacle: &MovingObstacle,
        acc: f64,
        dir: f64,
        height: f64,
    ) -> Result<Trajectory> {
        let seg = self.config.segments;
        if self.config.horizon <= 0.0 || seg == 0 {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "predictor config must be positive",
            ));
        }

        let head_p = Vector3::new(obstacle.position().x, obstacle.position().y, height);
        let head_v = Vector3::new(obstacle.velocity().x, obstacle.velocity().y, 0.0);
        let start = Endpoint {
            position: head_p,
            velocity: head_v,
            acceleration: Vector3::zeros(),
        };

        // 中间点：horizon/segments 间隔的外推位置
        let dt = self.config.horizon / seg as f64;
        let mut waypoints = Vec::with_capacity(seg - 1);
        for i in 1..seg {
            let (p, _) = obstacle.predict(acc, dir, dt * i as f64);
            waypoints.push(Point3::new(p.x, p.y, height));
        }

        // 终点：外推末态
        let (tail_p, tail_v) = obstacle.predict(acc, dir, self.config.horizon);
        let end = Endpoint {
            position: Vector3::new(tail_p.x, tail_p.y, height),
            velocity: Vector3::new(tail_v.x, tail_v.y, 0.0),
            acceleration: Vector3::zeros(),
        };

        let durations = vec![dt; seg];
        MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&waypoints, &durations)
            .map(|m| m.solve().expect("nonsingular"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector2;

    fn moving_right() -> MovingObstacle {
        MovingObstacle::new(Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0), 0.0, 5.0)
    }

    #[test]
    fn predicted_trajectory_geometry() {
        let o = MovingObstacle::new(Vector2::new(1.0, 2.0), Vector2::new(1.0, 0.0), 0.0, 1.0);
        let p = ObstaclePredictor::new(PredictorConfig::default());
        let traj = p.predict_trajectory(&o, 0.0, 0.0, 1.0).unwrap();
        // 起点为当前位置
        let s0 = traj.eval(0.0);
        assert!((s0.position - Vector3::new(1.0, 2.0, 1.0)).norm() < 1e-6);
        // 沿预测方向运动（+x）
        let sf = traj.eval(traj.duration());
        assert!(sf.position.x > 1.0, "应沿预测方向运动");
    }

    #[test]
    fn invalid_config_rejected() {
        let p = ObstaclePredictor::new(PredictorConfig {
            horizon: 0.0,
            ..PredictorConfig::default()
        });
        let r = p.predict_trajectory(&moving_right(), 0.0, 0.0, 1.0);
        assert_eq!(r.unwrap_err().kind(), ErrorKind::InvalidArgument);
    }
}
