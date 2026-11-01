// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See LICENSE for the complete notice and advertising acknowledgement.
//! Original TagFamily.h/TagFamily.cc.
use super::tag_detection::TagDetection;
pub struct TagFamily {
    pub black_border: usize,
    pub bits: usize,
    pub dimension: usize,
    pub minimum_hamming_distance: i32,
    pub error_recovery_bits: i32,
    pub codes: Vec<u64>,
}
impl TagFamily {
    #[must_use]
    pub fn new(
        bits: usize,
        minimum_hamming_distance: i32,
        codes: &[u64],
        black_border: usize,
    ) -> Self {
        let dimension = (bits as f32).sqrt() as usize;
        if bits != dimension * dimension {
            log::error!("TagFamily bits must be a square number");
        }
        Self {
            black_border,
            bits,
            dimension,
            minimum_hamming_distance,
            error_recovery_bits: 1,
            codes: codes.to_vec(),
        }
    }
    pub fn set_error_recovery_bits(&mut self, bits: i32) {
        self.error_recovery_bits = bits;
    }
    pub fn set_error_recovery_fraction(&mut self, fraction: f32) {
        self.error_recovery_bits =
            (((self.minimum_hamming_distance - 1) / 2) as f32 * fraction) as i32;
    }
    #[must_use]
    pub fn rotate90(word: u64, dimension: usize) -> u64 {
        let mut rotated = 0;
        for r in (0..dimension).rev() {
            for c in 0..dimension {
                rotated <<= 1;
                if word & (1 << (r + dimension * c)) != 0 {
                    rotated |= 1;
                }
            }
        }
        rotated
    }
    #[must_use]
    pub fn decode(&self, word: u64) -> TagDetection {
        let mut rotations = [word; 4];
        for i in 1..4 {
            rotations[i] = Self::rotate90(rotations[i - 1], self.dimension);
        }
        let mut detection = TagDetection {
            id: -1,
            hamming_distance: i32::MAX,
            observed_code: word,
            ..Default::default()
        };
        for (id, &code) in self.codes.iter().enumerate() {
            for (rotation, &observed) in rotations.iter().enumerate() {
                let distance = (observed ^ code).count_ones() as i32;
                if distance < detection.hamming_distance {
                    detection.hamming_distance = distance;
                    detection.rotation = rotation;
                    detection.id = id as i32;
                    detection.code = code;
                }
            }
        }
        detection.good = detection.hamming_distance <= self.error_recovery_bits;
        detection
    }
    #[must_use]
    /// # Panics
    /// Panics if a code contains set bits outside the declared family width.
    pub fn hamming_distances(&self) -> Vec<usize> {
        let mut counts = vec![0; self.bits + 1];
        for (i, &word) in self.codes.iter().enumerate() {
            let r1 = Self::rotate90(word, self.dimension);
            let r2 = Self::rotate90(r1, self.dimension);
            let r3 = Self::rotate90(r2, self.dimension);
            for &other in self.codes.iter().skip(i + 1) {
                let distance = [word, r1, r2, r3]
                    .into_iter()
                    .map(|r| (r ^ other).count_ones())
                    .min()
                    .expect("four rotations");
                counts[distance as usize] += 1;
            }
        }
        counts
    }
}
