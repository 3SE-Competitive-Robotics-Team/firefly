// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See docs/licenses/README.md for the complete notice and advertising acknowledgement.
//! Original Segment.h/Segment.cc.
use std::sync::atomic::{AtomicUsize, Ordering};
static NEXT_ID: AtomicUsize = AtomicUsize::new(0);
#[derive(Clone, Debug)]
pub struct Segment {
    pub p0: (f32, f32),
    pub p1: (f32, f32),
    pub theta: f32,
    pub length: f32,
    pub children: Vec<usize>,
    pub id: usize,
}
impl Default for Segment {
    fn default() -> Self {
        Self {
            p0: (0., 0.),
            p1: (0., 0.),
            theta: 0.,
            length: 0.,
            children: Vec::new(),
            id: NEXT_ID.fetch_add(1, Ordering::Relaxed) + 1,
        }
    }
}
impl Segment {
    pub const MINIMUM_SEGMENT_SIZE: usize = 4;
    pub const MINIMUM_LINE_LENGTH: f32 = 4.;
    #[must_use]
    pub fn segment_length(&self) -> f32 {
        super::math_util::distance_2d(self.p0, self.p1)
    }
}
