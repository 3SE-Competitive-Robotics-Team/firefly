// SPDX-License-Identifier: GPL-3.0-only
//! `raycast.cpp`: integer-cell direction, strict tie comparisons, end cell excluded.
use nalgebra::Vector3;

pub(crate) struct RayCaster {
    cell: Vector3<i32>,
    end: Vector3<i32>,
    step: Vector3<i32>,
    t_max: Vector3<f64>,
    t_delta: Vector3<f64>,
}
fn intbound(s: f64, ds: f64) -> f64 {
    if ds < 0. {
        return intbound(-s, -ds);
    }
    let s = ((s % 1.) + 1.) % 1.;
    (1. - s) / ds
}
impl RayCaster {
    pub fn new(start: Vector3<f64>, end: Vector3<f64>) -> Self {
        let cell = start.map(|v| v.floor() as i32);
        let end = end.map(|v| v.floor() as i32);
        let direction = end - cell;
        let step = direction.map(i32::signum);
        Self {
            cell,
            end,
            step,
            t_max: Vector3::from_fn(|i, _| intbound(start[i], f64::from(direction[i]))),
            t_delta: Vector3::from_fn(|i, _| f64::from(step[i]) / f64::from(direction[i])),
        }
    }
}
impl Iterator for RayCaster {
    type Item = Vector3<i32>;
    fn next(&mut self) -> Option<Self::Item> {
        if self.cell == self.end {
            return None;
        }
        let result = self.cell;
        let axis = if self.t_max.x < self.t_max.y {
            if self.t_max.x < self.t_max.z { 0 } else { 2 }
        } else if self.t_max.y < self.t_max.z {
            1
        } else {
            2
        };
        self.cell[axis] += self.step[axis];
        self.t_max[axis] += self.t_delta[axis];
        Some(result)
    }
}
