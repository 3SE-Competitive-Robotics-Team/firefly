// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See docs/licenses/README.md for the complete notice and advertising acknowledgement.
//! Original Edge.h/Edge.cc.
use super::{
    float_image::FloatImage,
    math_util::{mod_2pi, mod_2pi_near},
    union_find_simple::UnionFindSimple,
};
#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub cost: i32,
    pub a: usize,
    pub b: usize,
}
impl Edge {
    pub const MIN_MAG: f32 = 0.004;
    pub const MAX_EDGE_COST: f32 = 30. * std::f32::consts::PI / 180.;
    #[must_use]
    pub fn edge_cost(theta0: f32, theta1: f32, magnitude: f32) -> i32 {
        if magnitude < Self::MIN_MAG {
            return -1;
        }
        let error = mod_2pi(theta1 - theta0).abs();
        if error > Self::MAX_EDGE_COST {
            return -1;
        }
        (error / Self::MAX_EDGE_COST * 100.) as i32
    }
    pub fn calculate(
        theta0: f32,
        x: usize,
        y: usize,
        theta: &FloatImage,
        magnitude: &FloatImage,
        output: &mut Vec<Self>,
    ) {
        let a = y * theta.width + x;
        for (next_x, next_y) in [(x + 1, y), (x, y + 1), (x + 1, y + 1)] {
            let cost = Self::edge_cost(
                theta0,
                theta.get(next_x, next_y),
                magnitude.get(next_x, next_y),
            );
            if cost >= 0 {
                output.push(Self {
                    cost,
                    a,
                    b: next_y * theta.width + next_x,
                });
            }
        }
        if x > 0 {
            let cost =
                Self::edge_cost(theta0, theta.get(x - 1, y + 1), magnitude.get(x - 1, y + 1));
            if cost >= 0 {
                output.push(Self {
                    cost,
                    a,
                    b: (y + 1) * theta.width + x - 1,
                });
            }
        }
    }
    #[expect(
        clippy::similar_names,
        clippy::manual_midpoint,
        reason = "Preserve source range names and f32 addition then division"
    )]
    pub fn merge(
        edges: &[Self],
        forest: &mut UnionFindSimple,
        tmin: &mut [f32],
        tmax: &mut [f32],
        mmin: &mut [f32],
        mmax: &mut [f32],
    ) {
        for edge in edges {
            let a = forest.representative(edge.a);
            let b = forest.representative(edge.b);
            if a == b {
                continue;
            }
            let size = (forest.sizes[a] + forest.sizes[b]) as f32;
            let costa = tmax[a] - tmin[a];
            let costb = tmax[b] - tmin[b];
            let center_b = (tmin[b] + tmax[b]) / 2.;
            let shift = mod_2pi_near((tmin[a] + tmax[a]) / 2., center_b) - center_b;
            let minimum = tmin[a].min(tmin[b] + shift);
            let mut maximum = tmax[a].max(tmax[b] + shift);
            if maximum - minimum > 2. * std::f32::consts::PI {
                maximum = minimum + 2. * std::f32::consts::PI;
            }
            let min_magnitude = mmin[a].min(mmin[b]);
            let max_magnitude = mmax[a].max(mmax[b]);
            if maximum - minimum <= costa.min(costb) + 100. / size
                && max_magnitude - min_magnitude
                    <= (mmax[a] - mmin[a]).min(mmax[b] - mmin[b]) + 1200. / size
            {
                let root = forest.connect(a, b);
                tmin[root] = minimum;
                tmax[root] = maximum;
                mmin[root] = min_magnitude;
                mmax[root] = max_magnitude;
            }
        }
    }
}
