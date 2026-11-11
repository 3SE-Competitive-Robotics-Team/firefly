//! Coordinate bounded position/yaw commands and a stopping tail on one time axis.
use firefly_map::{GridMap, ObservedSpace};
use firefly_trajectory::{Endpoint, MincoBuilder, SolverOrder, Trajectory};
use nalgebra::{DMatrix, DVector, Point3, Vector3};

use crate::{Planner, PlannerConfig};

/// EGO-derived angular envelope, in rad/s and rad/s².
pub const MAX_YAW_RATE: f64 = 2. * std::f64::consts::PI;
pub const MAX_YAW_ACCELERATION: f64 = 5. * std::f64::consts::PI;

#[derive(Clone)]
pub struct MotionContext {
    /// Coverage diagnostics only; execution uses the same optimistic policy as EGO search.
    pub observed: ObservedSpace,
    pub measured_yaw: f64,
    /// Half of the calibrated horizontal field of view, in radians.
    pub view_half_angle: f64,
}

pub(crate) struct Motion {
    pub position: Trajectory,
    pub yaw: Trajectory,
    pub reaches_goal: bool,
}

pub(crate) fn unwrap_near(previous: f64, angle: f64) -> f64 {
    previous + (angle - previous + std::f64::consts::PI).rem_euclid(2. * std::f64::consts::PI)
        - std::f64::consts::PI
}

fn endpoint(position: Vector3<f64>) -> Endpoint {
    Endpoint {
        position,
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    }
}

fn segment(start: Endpoint, end: Endpoint, duration: f64) -> Option<Trajectory> {
    MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&[], &[duration])
        .ok()?
        .solve()
        .ok()
}

/// Exact prefix: cutting a polynomial changes its duration, not its coefficients.
pub(crate) fn prefix(trajectory: &Trajectory, duration: f64) -> Option<Trajectory> {
    if !duration.is_finite() || duration <= 0. || duration > trajectory.duration() + 1e-9 {
        return None;
    }
    let mut left = duration;
    let mut times = Vec::new();
    for &time in trajectory.durations().iter() {
        times.push(time.min(left));
        left -= time;
        if left <= 1e-9 {
            break;
        }
    }
    Trajectory::from_coefficients(
        trajectory
            .coefficients()
            .rows(0, times.len() * 6)
            .into_owned(),
        DVector::from_vec(times),
    )
    .ok()
}

fn append(first: &Trajectory, second: &Trajectory) -> Option<Trajectory> {
    let rows = first.coefficients().nrows();
    let coefficients = DMatrix::from_fn(rows + second.coefficients().nrows(), 3, |r, c| {
        if r < rows {
            first.coefficients()[(r, c)]
        } else {
            second.coefficients()[(r - rows, c)]
        }
    });
    Trajectory::from_coefficients(
        coefficients,
        DVector::from_iterator(
            first.pieces() + second.pieces(),
            first
                .durations()
                .iter()
                .chain(second.durations().iter())
                .copied(),
        ),
    )
    .ok()
}

/// Discrete centerline check at at most half a voxel of travel and at least 100 Hz.
/// Inflation covers known obstacles; continuous swept-volume validation is separate.
/// Unknown space remains traversable, matching the position optimizer and EGO map queries.
pub(crate) fn executable(
    map: &GridMap,
    trajectory: &Trajectory,
    from: f64,
    config: &PlannerConfig,
) -> bool {
    let dt = (map.resolution() / (2. * config.max_velocity)).min(0.01);
    let duration = trajectory.duration();
    if !dt.is_finite() || dt <= 0. || !duration.is_finite() || duration / dt > 20_000. {
        return false;
    }
    let count = ((duration - from.max(0.)).max(0.) / dt).ceil() as usize;
    (0..=count).all(|i| {
        let sample = trajectory.eval((from.max(0.) + i as f64 * dt).min(duration));
        let p = sample.position;
        p.iter()
            .chain(sample.velocity.iter())
            .chain(sample.acceleration.iter())
            .all(|x| x.is_finite())
            && map.index_of(p).is_some()
            && !map.is_occupied_inflated(p)
            && sample.velocity.norm() <= config.max_velocity + 1e-6
            && sample.acceleration.norm() <= config.max_acceleration + 1e-6
            && sample.jerk.norm() <= config.max_jerk + 1e-6
    })
}

fn angular_feasible(yaw: &Trajectory) -> bool {
    let n = (yaw.duration() / 0.01).ceil() as usize;
    n <= 20_000
        && (0..=n).all(|i| {
            let state = yaw.eval((i as f64 * 0.01).min(yaw.duration()));
            state.position.x.is_finite()
                && state.velocity.x.abs() <= MAX_YAW_RATE + 1e-6
                && state.acceleration.x.abs() <= MAX_YAW_ACCELERATION + 1e-6
        })
}

pub(crate) fn yaw_trajectory(
    position: &Trajectory,
    initial: Vector3<f64>,
    goal: Option<f64>,
) -> Option<Trajectory> {
    let direction = look_direction(position, 0., 1.);
    let desired = goal.unwrap_or_else(|| {
        if direction.xy().norm() > 0.1 {
            direction.y.atan2(direction.x)
        } else {
            initial.x
        }
    });
    let angle = unwrap_near(initial.x, desired);
    let start = Endpoint {
        position: Vector3::new(initial.x, 0., 0.),
        velocity: Vector3::new(initial.y, 0., 0.),
        acceleration: Vector3::new(initial.z, 0., 0.),
    };
    let end = endpoint(Vector3::new(angle, 0., 0.));
    let curve = segment(start, end, position.duration())?;
    let mut time = 0.;
    let waypoints: Vec<_> = position
        .durations()
        .iter()
        .take(position.pieces() - 1)
        .map(|duration| {
            time += duration;
            Point3::from(curve.eval(time).position)
        })
        .collect();
    MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
        .build(&waypoints, position.durations().as_slice())
        .ok()?
        .solve()
        .ok()
}

fn brake(start: Endpoint, duration: f64) -> Option<Trajectory> {
    let end = start.position
        + start.velocity * (duration / 2.)
        + start.acceleration * (duration * duration / 12.);
    segment(start, endpoint(end), duration)
}

pub(crate) fn stopping_motion(
    start: Endpoint,
    map: &GridMap,
    initial_yaw: Vector3<f64>,
    config: &PlannerConfig,
) -> Option<Motion> {
    for i in 0..8 {
        let position = brake(start, 0.25 * 1.5_f64.powi(i))?;
        let yaw = yaw_trajectory(&position, initial_yaw, Some(initial_yaw.x))?;
        if executable(map, &position, 0., config) && angular_feasible(&yaw) {
            return Some(Motion {
                position,
                yaw,
                reaches_goal: false,
            });
        }
    }
    None
}

/// Commit a bounded prefix and complete stopping tail checked against known obstacles.
#[allow(clippy::too_many_arguments)]
pub(crate) fn coordinate(
    proposed: &Trajectory,
    map: &GridMap,
    context: &MotionContext,
    initial_yaw: Vector3<f64>,
    goal_yaw: Option<f64>,
    config: &PlannerConfig,
    horizon_seconds: f64,
    reaches_goal: bool,
) -> Option<Motion> {
    let timed = retime(proposed, config)?;
    let proposed = &timed;
    let state = proposed.eval(0.);
    let start = Endpoint {
        position: state.position,
        velocity: state.velocity,
        acceleration: state.acceleration,
    };
    let direction = proposed.eval(proposed.duration()).position - start.position;
    let desired = if direction.xy().norm() > 1e-3 {
        direction.y.atan2(direction.x)
    } else {
        goal_yaw.unwrap_or(initial_yaw.x)
    };
    let turn = (unwrap_near(context.measured_yaw, desired) - context.measured_yaw).abs()
        > context.view_half_angle;
    log_proposal(
        proposed,
        map,
        context,
        config,
        horizon_seconds,
        desired,
        turn,
    );
    let heading_only = direction.norm() < 1e-6;
    if turn || (heading_only && goal_yaw.is_some()) {
        for i in 0..8 {
            let duration = 0.25 * 1.5_f64.powi(i);
            let stop = brake(start, duration)?;
            let end = endpoint(stop.eval(stop.duration()).position);
            let angle = unwrap_near(initial_yaw.x, desired);
            let turn_duration = (2. * (angle - initial_yaw.x).abs() / MAX_YAW_RATE)
                .max((6. * (angle - initial_yaw.x).abs() / MAX_YAW_ACCELERATION).sqrt())
                .max(0.5);
            let position = append(&stop, &segment(end, end, turn_duration)?)?;
            let yaw = yaw_trajectory(&position, initial_yaw, Some(angle))?;
            if executable(map, &position, 0., config) && angular_feasible(&yaw) {
                return Some(Motion {
                    position,
                    yaw,
                    reaches_goal: reaches_goal && heading_only,
                });
            }
        }
        return None;
    }
    if reaches_goal
        && proposed.duration() <= horizon_seconds
        && executable(map, proposed, 0., config)
    {
        let yaw = yaw_trajectory(proposed, initial_yaw, goal_yaw)?;
        if angular_feasible(&yaw) {
            return Some(Motion {
                position: proposed.clone(),
                yaw,
                reaches_goal: true,
            });
        }
    }
    // Bound retries and memory independently of the length of the global mission.
    let max_prefix = proposed.duration().min(horizon_seconds);
    for cut in (1..=12).rev() {
        let duration = max_prefix * f64::from(cut) / 12.;
        let head = prefix(proposed, duration)?;
        if !executable(map, &head, 0., config) {
            continue;
        }
        let end = head.eval(duration);
        let start = Endpoint {
            position: end.position,
            velocity: end.velocity,
            acceleration: end.acceleration,
        };
        for i in 0..8 {
            let tail = brake(start, 0.25 * 1.5_f64.powi(i))?;
            let position = append(&head, &tail)?;
            let yaw = yaw_trajectory(&position, initial_yaw, Some(desired))?;
            if executable(map, &position, 0., config) && angular_feasible(&yaw) {
                return Some(Motion {
                    position,
                    yaw,
                    reaches_goal: false,
                });
            }
        }
    }
    None
}

fn log_proposal(
    proposed: &Trajectory,
    map: &GridMap,
    context: &MotionContext,
    config: &PlannerConfig,
    horizon_seconds: f64,
    desired: f64,
    turn: bool,
) {
    if log::log_enabled!(target: "planner-motion", log::Level::Debug) {
        let blocked = (0..=200).find_map(|i| {
            let t = proposed.duration().min(horizon_seconds) * f64::from(i) / 200.;
            let sample = proposed.eval(t);
            let p = sample.position;
            let reason = if map.index_of(p).is_none() {
                "window"
            } else if map.is_occupied_inflated(p) {
                "occupied"
            } else if sample.velocity.norm() > config.max_velocity + 1e-6 {
                "velocity"
            } else if sample.acceleration.norm() > config.max_acceleration + 1e-6 {
                "acceleration"
            } else if sample.jerk.norm() > config.max_jerk + 1e-6 {
                "jerk"
            } else {
                return None;
            };
            Some((t, p, reason))
        });
        let first_unobserved = (0..=200).find_map(|i| {
            let t = proposed.duration().min(horizon_seconds) * f64::from(i) / 200.;
            let p = proposed.eval(t).position;
            (!context.observed.is_free(p)).then_some((t, p))
        });
        log::debug!(target: "planner-motion", "proposal start={:?} yaw={} desired={} turn={turn} first_block={blocked:?} first_unobserved={first_unobserved:?}", proposed.eval(0.).position, context.measured_yaw, desired);
    }
}

/// Orient the camera along the spatial lookahead beyond short braking transients.
fn look_direction(trajectory: &Trajectory, from: f64, distance: f64) -> Vector3<f64> {
    let origin = trajectory.eval(from).position;
    let mut direction = Vector3::zeros();
    for i in 1..=100 {
        let t = from + (trajectory.duration() - from) * f64::from(i) / 100.;
        direction = trajectory.eval(t).position - origin;
        if direction.xy().norm() >= distance {
            break;
        }
    }
    direction
}

/// Soft optimization penalties may exceed the envelope slightly. Refit shared
/// MINCO knots with longer times while preserving boundary PVA, then collision-check
/// the resulting curve.
fn retime(proposed: &Trajectory, config: &PlannerConfig) -> Option<Trajectory> {
    if !proposed.duration().is_finite()
        || proposed.duration() <= 0.
        || proposed.coefficients().iter().any(|x| !x.is_finite())
    {
        return None;
    }
    let first = proposed.eval(0.);
    let last = proposed.eval(proposed.duration());
    let start = Endpoint {
        position: first.position,
        velocity: first.velocity,
        acceleration: first.acceleration,
    };
    let end = Endpoint {
        position: last.position,
        velocity: last.velocity,
        acceleration: last.acceleration,
    };
    let mut time = 0.;
    let waypoints: Vec<_> = proposed
        .durations()
        .iter()
        .take(proposed.pieces() - 1)
        .map(|duration| {
            time += duration;
            Point3::from(proposed.eval(time).position)
        })
        .collect();
    let mut trajectory = proposed.clone();
    for _ in 0..8 {
        if !trajectory.duration().is_finite()
            || trajectory.coefficients().iter().any(|x| !x.is_finite())
        {
            return None;
        }
        let factor = (Planner::trajectory_max_vel(&trajectory) / config.max_velocity)
            .max((Planner::trajectory_max_acc(&trajectory) / config.max_acceleration).sqrt())
            .max((Planner::trajectory_max_jerk(&trajectory) / config.max_jerk).cbrt());
        if !factor.is_finite() {
            return None;
        }
        if factor <= 1. + 1e-9 {
            return Some(trajectory);
        }
        let times: Vec<_> = trajectory
            .durations()
            .iter()
            .map(|duration| duration * factor.max(1.01))
            .collect();
        trajectory = MincoBuilder::new(SolverOrder::MinimumJerk, start, end)
            .build(&waypoints, &times)
            .ok()?
            .solve()
            .ok()?;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_map::GridMapBuilder;

    fn scene(free_x: f64) -> (GridMap, MotionContext) {
        let map = GridMapBuilder::new(0.1, [100, 20, 20])
            .with_obstacles_inflation(0.)
            .build()
            .unwrap();
        let free: Vec<_> = (0..40_000)
            .map(|i| f64::from(i % 100) * 0.1 < free_x)
            .collect();
        let context = MotionContext {
            observed: ObservedSpace::from_buffer(0.1, [0; 3], [100, 20, 20], [0; 3], &free),
            measured_yaw: 0.,
            view_half_angle: 0.6,
        };
        (map, context)
    }

    fn proposal() -> Trajectory {
        segment(
            endpoint(Vector3::new(0.5, 1., 1.)),
            endpoint(Vector3::new(6.5, 1., 1.)),
            10.,
        )
        .unwrap()
    }

    #[test]
    fn unobserved_air_uses_the_same_obstacle_policy_as_position_planning() {
        let (mut map, context) = scene(0.);
        let config = PlannerConfig::default();
        let target = Vector3::new(1.5, 1., 1.1);
        let proposed =
            segment(endpoint(Vector3::new(0.5, 1., 0.989)), endpoint(target), 4.).unwrap();
        let motion = coordinate(
            &proposed,
            &map,
            &context,
            Vector3::zeros(),
            None,
            &config,
            5.,
            true,
        )
        .expect("unobserved air is traversable under EGO's optimistic policy");
        assert!(motion.reaches_goal);
        assert!((motion.position.eval(motion.position.duration()).position - target).norm() < 1e-9);
        // Unknown may be traversed, but a measured obstacle must still block the curve.
        map.set_state(
            map.index_of(proposed.eval(2.).position).unwrap(),
            firefly_map::VoxelState::Occupied,
        );
        map.inflate_obstacles();
        assert!(!executable(&map, &proposed, 0., &config));
    }

    #[test]
    fn obstacle_boundary_requires_a_complete_stop_before_it() {
        let (mut map, context) = scene(1.4);
        for x in 14..100 {
            for y in 0..20 {
                for z in 0..20 {
                    map.set_state([x, y, z], firefly_map::VoxelState::Occupied);
                }
            }
        }
        map.inflate_obstacles();
        let config = PlannerConfig::default();
        let proposed = proposal();
        assert!(!executable(&map, &proposed, 0., &config));
        let motion = coordinate(
            &proposed,
            &map,
            &context,
            Vector3::zeros(),
            None,
            &config,
            2.,
            false,
        )
        .unwrap();
        assert!(!motion.reaches_goal);
        assert!(motion.position.eval(motion.position.duration()).position.x > 0.6);
        for i in 0..=1000 {
            let state = motion
                .position
                .eval(motion.position.duration() * f64::from(i) / 1000.);
            assert!(state.position.x < 1.4);
            assert!(state.velocity.norm() <= config.max_velocity + 1e-6);
            assert!(state.acceleration.norm() <= config.max_acceleration + 1e-6);
            assert!(state.jerk.norm() <= config.max_jerk + 1e-6);
        }
        let end = motion.position.eval(motion.position.duration());
        assert!(end.velocity.norm() < 1e-9 && end.acceleration.norm() < 1e-9);
        let join = motion.position.durations()[0];
        let before = motion.position.eval(join - 1e-8);
        let after = motion.position.eval(join + 1e-8);
        assert!((before.velocity - after.velocity).norm() < 1e-6);
        assert!((before.acceleration - after.acceleration).norm() < 1e-6);
    }

    #[test]
    fn large_turn_keeps_position_and_wraps_heading_continuously() {
        let (map, mut context) = scene(10.);
        context.measured_yaw = std::f64::consts::PI;
        let initial = Vector3::new(context.measured_yaw, 0., 0.);
        let motion = coordinate(
            &proposal(),
            &map,
            &context,
            initial,
            None,
            &PlannerConfig::default(),
            2.,
            false,
        )
        .unwrap();
        assert!(!motion.reaches_goal);
        assert!(angular_feasible(&motion.yaw));
        for i in 0..=100 {
            let t = motion.position.duration() * f64::from(i) / 100.;
            assert!(
                (motion.position.eval(t).position - proposal().eval(0.).position).norm() < 1e-9
            );
        }
        let yaw = motion.yaw.eval(motion.yaw.duration());
        assert!(unwrap_near(0., yaw.position.x).abs() < 1e-9);
        assert!(yaw.velocity.norm() < 1e-9);
        assert!(
            (unwrap_near(179_f64.to_radians(), -179_f64.to_radians()) - 181_f64.to_radians()).abs()
                < 1e-12
        );
    }

    #[test]
    fn emergency_brake_preserves_measured_velocity_and_acceleration() {
        let (map, _) = scene(10.);
        let start = Endpoint {
            position: Vector3::new(1., 1., 1.),
            velocity: Vector3::new(1., 0., 0.),
            acceleration: Vector3::new(0.2, 0., 0.),
        };
        let motion =
            stopping_motion(start, &map, Vector3::zeros(), &PlannerConfig::default()).unwrap();
        let first = motion.position.eval(0.);
        assert!((first.velocity - start.velocity).norm() < 1e-9);
        assert!((first.acceleration - start.acceleration).norm() < 1e-9);
        let end = motion.position.eval(motion.position.duration());
        assert!(end.position.x > start.position.x);
        assert!(end.velocity.norm() < 1e-9 && end.acceleration.norm() < 1e-9);
    }

    #[test]
    fn stationary_goal_accepts_explicit_heading_without_translation() {
        let (map, context) = scene(0.);
        let pose = endpoint(Vector3::new(0.5, 1., 1.));
        let position = segment(pose, pose, 2.).unwrap();
        let motion = coordinate(
            &position,
            &map,
            &context,
            Vector3::zeros(),
            Some(1.5),
            &PlannerConfig::default(),
            2.,
            true,
        )
        .unwrap();
        assert!(motion.reaches_goal);
        assert!((motion.yaw.eval(motion.yaw.duration()).position.x - 1.5).abs() < 1e-9);
        assert!(
            (motion.position.eval(motion.position.duration()).position - pose.position).norm()
                < 1e-9
        );
    }

    #[test]
    fn retiming_preserves_moving_boundary_and_enforces_extrema() {
        let start = Endpoint {
            position: Vector3::new(1., 1., 1.),
            velocity: Vector3::new(0.5, 0., 0.),
            acceleration: Vector3::new(0.1, 0., 0.),
        };
        let end = endpoint(Vector3::new(3., 1., 1.));
        let original = segment(start, end, 1.).unwrap();
        let config = PlannerConfig::default();
        let timed = retime(&original, &config).unwrap();
        assert!(timed.duration() > original.duration());
        let first = timed.eval(0.);
        assert!((first.velocity - start.velocity).norm() < 1e-9);
        assert!((first.acceleration - start.acceleration).norm() < 1e-9);
        let last = timed.eval(timed.duration());
        assert!((last.position - end.position).norm() < 1e-9);
        for i in 0..=2000 {
            let s = timed.eval(timed.duration() * f64::from(i) / 2000.);
            assert!(s.velocity.norm() <= config.max_velocity + 1e-6);
            assert!(s.acceleration.norm() <= config.max_acceleration + 1e-6);
            assert!(s.jerk.norm() <= config.max_jerk + 1e-6);
        }
    }

    #[test]
    fn yaw_replan_keeps_turning_through_position_piece_boundaries() {
        let pose = endpoint(Vector3::new(1., 1., 1.));
        let position = append(
            &segment(pose, pose, 0.25).unwrap(),
            &segment(pose, pose, 1.75).unwrap(),
        )
        .unwrap();
        let yaw = yaw_trajectory(&position, Vector3::new(0., 1., 0.), Some(1.5)).unwrap();
        for i in 0..=200 {
            let t = f64::from(i) * 0.01;
            // Independent quintic with yaw(0)=0, rate(0)=1, yaw(2)=1.5,
            // zero endpoint accelerations and zero final rate.
            let expected = t + 0.375 * t.powi(3) - 0.40625 * t.powi(4) + 0.09375 * t.powi(5);
            assert!((yaw.eval(t).position.x - expected).abs() < 1e-9);
            assert!(yaw.eval(t).velocity.x >= -1e-9);
        }
    }

    #[test]
    fn nearby_position_goal_is_not_mistaken_for_heading_only() {
        let (map, context) = scene(10.);
        let target = Vector3::new(1.05, 1., 1.);
        let position = segment(endpoint(Vector3::new(1., 1., 1.)), endpoint(target), 2.).unwrap();
        let motion = coordinate(
            &position,
            &map,
            &context,
            Vector3::zeros(),
            Some(0.5),
            &PlannerConfig::default(),
            2.,
            true,
        )
        .unwrap();
        assert!(motion.reaches_goal);
        assert!((motion.position.eval(motion.position.duration()).position - target).norm() < 1e-9);
    }
}
