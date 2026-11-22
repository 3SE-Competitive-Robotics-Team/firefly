//! 静止平台降落：对准、下降、目标锁存后的末段；由 FC 独占状态。
use crate::{QuadState, px4::velocity_smoothing::VelocitySmoothing};
use nalgebra::Vector3;
use serde::Deserialize;

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PrecisionParams {
    pub align_radius: f32,
    pub align_hold: f32,
    pub final_height: f32,
    pub target_timeout: f32,
    pub reacquire_timeout: f32,
    pub final_timeout: f32,
    pub horizontal_speed: f32,
    pub horizontal_accel: f32,
    pub horizontal_jerk: f32,
}
impl Default for PrecisionParams {
    fn default() -> Self {
        Self {
            align_radius: 0.03,
            align_hold: 0.5,
            final_height: 0.35,
            target_timeout: 0.4,
            reacquire_timeout: 2.,
            final_timeout: 8.,
            horizontal_speed: 0.15,
            horizontal_accel: 0.3,
            horizontal_jerk: 1.,
        }
    }
}
impl PrecisionParams {
    /// # Errors
    /// 非有限或非正控制限值。
    pub fn validate(&self) -> firefly_error::Result<()> {
        if [
            self.align_radius,
            self.align_hold,
            self.final_height,
            self.target_timeout,
            self.reacquire_timeout,
            self.final_timeout,
            self.horizontal_speed,
            self.horizontal_accel,
            self.horizontal_jerk,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v <= 0.)
            || self.final_timeout <= self.target_timeout
        {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "invalid precision landing limits",
            ));
        }
        Ok(())
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrecisionStage {
    Align,
    Descend,
    Final,
    Reacquire,
}

#[derive(Clone, Copy, Debug)]
enum Progress {
    Align {
        held: f32,
    },
    Descend,
    Final {
        elapsed: f32,
        contact_candidate: bool,
    },
    Reacquire,
}

#[derive(Clone, Copy, Debug)]
pub struct PrecisionLanding {
    params: PrecisionParams,
    target: Vector3<f32>,
    progress: Progress,
    age: f32,
    horizontal: [VelocitySmoothing; 2],
}
impl PrecisionLanding {
    #[must_use]
    pub fn new(params: PrecisionParams, target: Vector3<f32>, state: &QuadState) -> Self {
        let horizontal = std::array::from_fn(|i| {
            let mut value = VelocitySmoothing::default();
            value.reset(0., state.velocity[i], state.position[i]);
            value.set_max_vel(params.horizontal_speed);
            value.set_max_accel(params.horizontal_accel);
            value.set_max_jerk(params.horizontal_jerk);
            value
        });
        Self {
            params,
            target,
            progress: Progress::Align { held: 0. },
            age: 0.,
            horizontal,
        }
    }
    #[must_use]
    pub const fn target(&self) -> &Vector3<f32> {
        &self.target
    }

    #[must_use]
    pub const fn stage(&self) -> PrecisionStage {
        match self.progress {
            Progress::Align { .. } => PrecisionStage::Align,
            Progress::Descend => PrecisionStage::Descend,
            Progress::Final { .. } => PrecisionStage::Final,
            Progress::Reacquire => PrecisionStage::Reacquire,
        }
    }

    pub fn observe(&mut self, target: Vector3<f32>) {
        if !matches!(self.progress, Progress::Final { .. }) && (target - self.target).norm() <= 0.15
        {
            self.target = target;
            self.age = 0.;
            if matches!(self.progress, Progress::Reacquire) {
                self.progress = Progress::Align { held: 0. };
            }
        }
    }
    /// 失效由 FC 转为 Hold；末段锁定目标并限时完成。
    /// `near_contact` 由调用方按平台高度与低运动门限给出；低推力仍由 FC 独立确认。
    pub fn advance(&mut self, dt: f32, state: &QuadState, near_contact: bool) -> bool {
        self.age += dt;
        let error = (self.target - state.position).xy().norm();
        if let Progress::Final {
            elapsed,
            contact_candidate,
        } = &mut self.progress
        {
            *elapsed += dt;
            *contact_candidate |= near_contact && error <= 2. * self.params.align_radius;
            if *elapsed > self.params.final_timeout
                || (!*contact_candidate && error > 2. * self.params.align_radius)
            {
                return false;
            }
        } else {
            if self.age > self.params.target_timeout {
                if self.age > self.params.target_timeout + self.params.reacquire_timeout {
                    return false;
                }
                self.progress = Progress::Reacquire;
                for axis in &mut self.horizontal {
                    axis.update(dt, 0.);
                }
                return true;
            }
            let aligned = error <= self.params.align_radius && state.velocity.xy().norm() <= 0.08;
            if let Progress::Align { held } = &mut self.progress {
                *held = if aligned { *held + dt } else { 0. };
                if *held >= self.params.align_hold {
                    self.progress = Progress::Descend;
                }
            }
            if matches!(self.progress, Progress::Descend)
                && aligned
                && state.position.z - self.target.z <= self.params.final_height
            {
                self.progress = Progress::Final {
                    elapsed: 0.,
                    contact_candidate: false,
                };
            }
        }
        for i in 0..2 {
            let velocity = (0.8 * (self.target[i] - self.horizontal[i].current_position()))
                .clamp(-self.params.horizontal_speed, self.params.horizontal_speed);
            self.horizontal[i].update(dt, velocity);
        }
        true
    }
    #[must_use]
    pub fn contact_candidate(&self) -> bool {
        matches!(
            self.progress,
            Progress::Final {
                contact_candidate: true,
                ..
            }
        )
    }

    #[must_use]
    pub fn may_descend(&self, state: &QuadState) -> bool {
        matches!(self.progress, Progress::Final { .. })
            || (matches!(self.progress, Progress::Descend)
                && (self.target - state.position).xy().norm() <= self.params.align_radius)
    }
    #[must_use]
    pub fn horizontal_reference(&self) -> ([f32; 2], [f32; 2], [f32; 2]) {
        (
            self.horizontal.map(|s| s.current_position()),
            self.horizontal.map(|s| s.current_velocity()),
            self.horizontal.map(|s| s.current_acceleration()),
        )
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn descent_requires_alignment_and_final_loss_is_bounded() {
        let mut state = QuadState {
            position: Vector3::new(0.1, 0., 1.),
            ..QuadState::default()
        };
        let mut landing =
            PrecisionLanding::new(PrecisionParams::default(), Vector3::zeros(), &state);
        for _ in 0..100 {
            landing.observe(Vector3::zeros());
            assert!(landing.advance(0.01, &state, false));
        }
        assert!(!landing.may_descend(&state));
        state.position.x = 0.;
        for _ in 0..60 {
            landing.observe(Vector3::zeros());
            assert!(landing.advance(0.01, &state, false));
        }
        assert!(landing.may_descend(&state));
        state.position.z = 0.1;
        landing.advance(0.01, &state, false);
        assert_eq!(landing.stage(), PrecisionStage::Final);
        assert!(landing.advance(0.5, &state, false));
        assert!(!landing.advance(8., &state, false));
    }
    #[test]
    fn contact_candidate_allows_local_frame_drift_but_keeps_final_deadline() {
        let state = QuadState {
            position: Vector3::new(0., 0., 0.03),
            ..QuadState::default()
        };
        let mut landing =
            PrecisionLanding::new(PrecisionParams::default(), Vector3::zeros(), &state);
        for _ in 0..2 {
            landing.observe(Vector3::zeros());
            assert!(landing.advance(0.25, &state, false));
        }
        assert_eq!(landing.stage(), PrecisionStage::Final);
        let mut drifting = state;
        drifting.position.x = 0.08;
        assert!(!landing.advance(0.01, &drifting, false));
        assert!(landing.advance(0.01, &state, true));
        assert!(landing.advance(0.01, &drifting, true));
        assert!(!landing.advance(9., &state, true));
    }

    #[test]
    fn target_jumps_do_not_refresh_missing_observation() {
        let mut landing = PrecisionLanding::new(
            PrecisionParams::default(),
            Vector3::new(0., 0., -1.),
            &QuadState::default(),
        );
        landing.observe(Vector3::new(1., 0., -1.));
        assert_eq!(landing.target.x, 0.);
        assert!(!landing.advance(3., &QuadState::default(), false));
    }
}
