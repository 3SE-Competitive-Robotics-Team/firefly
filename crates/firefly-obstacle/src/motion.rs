//! 移动障碍运动模型（官方 `moving_obstacle::dyn_update` 语义）。
//!
//! 2D 自行车模型近似：加速度沿航向（yaw），转向率 dir 控制航向变化，
//! 阻尼使障碍像真实物体一样逐渐减速，限速防止发散。

use nalgebra::Vector2;

#[derive(Debug, Clone)]
pub struct MovingObstacle {
    position: Vector2<f64>,
    velocity: Vector2<f64>,
    yaw: f64,
    max_velocity: f64,
    damping: f64,
}

impl MovingObstacle {
    #[must_use]
    pub fn new(
        position: Vector2<f64>,
        velocity: Vector2<f64>,
        yaw: f64,
        max_velocity: f64,
    ) -> Self {
        Self {
            position,
            velocity,
            yaw,
            max_velocity,
            damping: 0.9,
        }
    }

    /// 设置阻尼系数（官方 0.9：每步保留 90% 速度，逐渐停止）。
    #[must_use]
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping;
        self
    }

    #[must_use]
    pub const fn position(&self) -> Vector2<f64> {
        self.position
    }

    #[must_use]
    pub const fn velocity(&self) -> Vector2<f64> {
        self.velocity
    }

    #[must_use]
    pub const fn yaw(&self) -> f64 {
        self.yaw
    }

    /// 一步推进（控制输入：油门 acc、转向率 dir）。
    pub fn step(&mut self, acc: f64, dir: f64, dt: f64) {
        self.yaw += dir * dt;
        let acc_vec = Vector2::new(acc * self.yaw.cos(), acc * self.yaw.sin());
        self.velocity += acc_vec * dt;
        self.velocity *= self.damping;
        let speed = self.velocity.norm();
        if speed > self.max_velocity {
            self.velocity *= self.max_velocity / speed;
        }
        self.position += self.velocity * dt + 0.5 * acc_vec * dt * dt;
    }

    /// 外推 t 秒（不改变自身状态），返回 (位置, 速度)。
    #[must_use]
    pub fn predict(&self, acc: f64, dir: f64, horizon: f64) -> (Vector2<f64>, Vector2<f64>) {
        const STEP: f64 = 0.1;
        let mut probe = Self {
            position: self.position,
            velocity: self.velocity,
            yaw: self.yaw,
            max_velocity: self.max_velocity,
            damping: self.damping,
        };
        let mut t = 0.0;
        while t < horizon {
            let dt = STEP.min(horizon - t);
            probe.step(acc, dir, dt);
            t += dt;
        }
        (probe.position, probe.velocity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dynamics() {
        // 静止保持静止
        let o = MovingObstacle::new(Vector2::new(1.0, 2.0), Vector2::zeros(), 0.0, 1.0);
        let (p, _) = o.predict(0.0, 0.0, 0.1);
        assert_eq!(p, Vector2::new(1.0, 2.0));
        // 油门沿 yaw 方向加速
        let mut o = MovingObstacle::new(
            Vector2::new(1.0, 2.0),
            Vector2::zeros(),
            core::f64::consts::FRAC_PI_2,
            1.0,
        );
        o.step(0.5, 1.0, 1.0);
        assert!(o.velocity().y > 0.0, "加速度应沿 yaw 方向");
        // 阻尼减速（无油门时速度衰减）
        let mut o = MovingObstacle::new(Vector2::new(1.0, 2.0), Vector2::new(2.0, 0.0), 0.0, 1.0);
        let v0 = o.velocity().norm();
        o.step(1.0, 0.0, 1.0);
        assert!(o.velocity().norm() < v0, "无油门时应减速");
        // 限速
        let mut o = MovingObstacle::new(
            Vector2::new(1.0, 2.0),
            Vector2::new(0.0, 100.0),
            core::f64::consts::FRAC_PI_2,
            1.0,
        );
        o.step(1.0, 0.0, 1.0);
        assert!(o.velocity().norm() <= 4.0, "速度应被限幅");
    }

    #[test]
    fn predict_extrapolates_without_mutation() {
        let o = MovingObstacle::new(Vector2::zeros(), Vector2::new(1.0, 0.0), 0.0, 5.0);
        let before = o.position();
        let (pos, _) = o.predict(0.0, 0.0, 2.0);
        assert_eq!(o.position(), before, "predict 不改变自身");
        assert!(pos.x > 0.0, "外推向前");
    }
}
