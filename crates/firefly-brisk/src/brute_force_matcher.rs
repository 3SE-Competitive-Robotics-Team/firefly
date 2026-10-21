/*
Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
Stefan Leutenegger, Simon Lynen and Margarita Chli.

Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
Stefan Leutenegger and Simon Lynen.

Copyright (C) 2020  Smart Robotics Lab, Imperial College London
Stefan Leutenegger

Copyright (C) 2024  Smart Robotics Lab, Technical University of Munich
Stefan Leutenegger

All rights reserved.

This is the Author's implementation of BRISK: Binary Robust Invariant
Scalable Keypoints [1]. Various (partly unpublished) extensions are provided,
some of which are described in [2].

[1] Stefan Leutenegger, Margarita Chli and Roland Siegwart. BRISK: Binary
    Robust Invariant Scalable Keypoints. In Proceedings of the IEEE
    International Conference on Computer Vision (ICCV), 2011.

[2] Stefan Leutenegger. Unmanned Solar Airplanes: Design and Algorithms for
    Efficient and Robust Autonomous Operation. Doctoral dissertation, 2014.

This file is part of BRISK.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//! Brute-force Hamming matching with source ordering, masking and integer sentinels.
use crate::{
    hamming::Hamming,
    vectorized_filters::{check, invalid},
};
use firefly_error::Result;
use purecv::core::Matrix;
use purecv::features2d::DMatch;
#[derive(Default)]
pub struct BruteForceMatcher {
    pub train_descriptors: Vec<Matrix<u8>>,
    pub distance: Hamming,
}
impl BruteForceMatcher {
    #[must_use]
    pub fn new(distance: Hamming) -> Self {
        Self {
            train_descriptors: Vec::new(),
            distance,
        }
    }
    #[must_use]
    pub fn is_mask_supported(&self) -> bool {
        true
    }
    /// A nonempty source clone writes through an empty destination iterator.
    /// # Errors
    /// Returns an error if cloning train data is requested; only an empty matcher can be cloned.
    pub fn try_clone(&self, empty_train_data: bool) -> Result<Self> {
        if !empty_train_data && !self.train_descriptors.is_empty() {
            return Err(invalid(
                "source matcher clone has no destination train storage",
            ));
        }
        Ok(Self::new(self.distance))
    }
    fn validate(&self, query: &Matrix<u8>, masks: &[Matrix<u8>]) -> Result<()> {
        check(query)?;
        if query.rows == 0 || query.cols == 0 {
            return Err(invalid("nonempty query descriptors required"));
        }
        if !masks.is_empty() && masks.len() != self.train_descriptors.len() {
            return Err(invalid("one match mask per train image required"));
        }
        for (i, t) in self.train_descriptors.iter().enumerate() {
            check(t)?;
            if t.rows != 0 && t.cols != query.cols {
                return Err(invalid("descriptor dimensions differ"));
            }
            if let Some(m) = masks.get(i) {
                check(m)?;
                if !m.data.is_empty() && (m.rows != query.rows || m.cols != t.rows) {
                    return Err(invalid("match mask shape mismatch"));
                }
            }
        }
        Ok(())
    }
    fn possible(masks: &[Matrix<u8>], image: usize, q: usize, t: usize) -> bool {
        masks
            .get(image)
            .is_none_or(|m| m.data.is_empty() || m.data[q * m.cols + t] != 0)
    }
    fn masked_out(masks: &[Matrix<u8>], q: usize) -> bool {
        !masks.is_empty()
            && masks.iter().all(|m| {
                !m.data.is_empty() && m.data[q * m.cols..(q + 1) * m.cols].iter().all(|v| *v == 0)
            })
    }
    /// # Errors
    /// Returns an error for empty/malformed query descriptors, incompatible train dimensions,
    /// invalid masks, or an unsupported requested neighbour count.
    pub fn knn_match(
        &self,
        query: &Matrix<u8>,
        matches: &mut Vec<Vec<DMatch>>,
        knn: usize,
        masks: &[Matrix<u8>],
        compact_result: bool,
    ) -> Result<()> {
        self.validate(query, masks)?;
        for q in 0..query.rows {
            if Self::masked_out(masks, q) {
                if !compact_result {
                    matches.push(Vec::new());
                }
                continue;
            }
            let a = &query.data[q * query.cols..(q + 1) * query.cols];
            let mut dists = Vec::with_capacity(self.train_descriptors.len());
            for (i, t) in self.train_descriptors.iter().enumerate() {
                let mut d = vec![i32::MAX; t.rows];
                for (j, v) in d.iter_mut().enumerate() {
                    if Self::possible(masks, i, q, j) {
                        *v = self.distance.distance(
                            a,
                            &t.data[j * t.cols..(j + 1) * t.cols],
                            query.cols as i32,
                        )?;
                    }
                }
                dists.push(d);
            }
            let mut found = Vec::new();
            for _ in 0..knn {
                let mut best = None;
                let mut distance = f32::MAX;
                for (i, d) in dists.iter().enumerate() {
                    if let Some((j, &v)) = d.iter().enumerate().min_by_key(|(j, v)| (**v, *j))
                        && f64::from(v) < f64::from(distance)
                    {
                        best = Some((i, j));
                        distance = v as f32;
                    }
                }
                let Some((i, j)) = best else {
                    break;
                };
                found.push(DMatch {
                    query_idx: q as i32,
                    train_idx: j as i32,
                    train_img_idx: i as i32,
                    distance,
                });
                dists[i][j] = i32::MAX;
            }
            found.sort_unstable_by(|a, b| a.distance.total_cmp(&b.distance));
            matches.push(found);
        }
        Ok(())
    }
    /// # Errors
    /// Returns an error for empty/malformed query descriptors, incompatible train dimensions,
    /// invalid masks, or an unsupported requested neighbour count.
    pub fn radius_match(
        &self,
        query: &Matrix<u8>,
        matches: &mut Vec<Vec<DMatch>>,
        max_distance: f32,
        masks: &[Matrix<u8>],
        compact_result: bool,
    ) -> Result<()> {
        self.validate(query, masks)?;
        for q in 0..query.rows {
            if Self::masked_out(masks, q) {
                if !compact_result {
                    matches.push(Vec::new());
                }
                continue;
            }
            let a = &query.data[q * query.cols..(q + 1) * query.cols];
            let mut found = Vec::new();
            for (i, t) in self.train_descriptors.iter().enumerate() {
                for j in 0..t.rows {
                    if Self::possible(masks, i, q, j) {
                        let distance = self.distance.distance(
                            a,
                            &t.data[j * t.cols..(j + 1) * t.cols],
                            query.cols as i32,
                        )?;
                        if (distance as f32) < max_distance {
                            found.push(DMatch {
                                query_idx: q as i32,
                                train_idx: j as i32,
                                train_img_idx: i as i32,
                                distance: distance as f32,
                            });
                        }
                    }
                }
            }
            found.sort_unstable_by(|a, b| a.distance.total_cmp(&b.distance));
            matches.push(found);
        }
        Ok(())
    }
}
