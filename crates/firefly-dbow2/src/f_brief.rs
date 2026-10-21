/*
 * File: FBrief.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: functions for BRIEF descriptors
 * License: see the LICENSE.txt file
 *
 */

use super::templated_vocabulary::{DescriptorOps, StorageResult};
use nalgebra::DMatrix;
pub const L: usize = 256;
/// Bit i has the same meaning as `std::bitset`'s index i.
pub type Descriptor = [bool; L];
#[derive(Clone, Copy, Debug, Default)]
pub struct FBrief;
impl FBrief {
    #[must_use]
    pub fn mean_value(descriptors: &[&Descriptor]) -> Descriptor {
        let mut mean = [false; L];
        let mut counts = [0_usize; L];
        for d in descriptors {
            for i in 0..L {
                counts[i] += usize::from(d[i]);
            }
        }
        for i in 0..L {
            mean[i] = counts[i] > descriptors.len() / 2;
        }
        mean
    }
    #[must_use]
    pub fn distance(a: &Descriptor, b: &Descriptor) -> f64 {
        a.iter().zip(b).filter(|(a, b)| a != b).count() as f64
    }
    #[must_use]
    pub fn to_string(a: &Descriptor) -> String {
        a.iter().rev().map(|&b| if b { '1' } else { '0' }).collect()
    }
    /// Stream extraction accepts a nonempty binary prefix of at most 256 digits.
    ///
    /// # Errors
    /// Returns an error if no binary digit can be extracted from the input.
    pub fn from_string(s: &str) -> StorageResult<Descriptor> {
        let bits: Vec<_> = s
            .trim_start_matches(|c: char| c.is_ascii_whitespace())
            .chars()
            .take(L)
            .take_while(|&c| c == '0' || c == '1')
            .collect();
        if bits.is_empty() {
            return Err("BRIEF descriptor requires binary digits".into());
        }
        let mut out = [false; L];
        for (i, &c) in bits.iter().rev().enumerate() {
            out[i] = c == '1';
        }
        Ok(out)
    }
    #[must_use]
    pub fn to_mat32f(descriptors: &[Descriptor]) -> DMatrix<f32> {
        if descriptors.is_empty() {
            return DMatrix::zeros(0, 0);
        }
        DMatrix::from_fn(descriptors.len(), L, |r, c| {
            if descriptors[r][c] { 1.0 } else { 0.0 }
        })
    }
}
impl DescriptorOps for FBrief {
    type Descriptor = Descriptor;
    fn mean_value(d: &[&Descriptor]) -> Descriptor {
        Self::mean_value(d)
    }
    fn distance(a: &Descriptor, b: &Descriptor) -> f64 {
        Self::distance(a, b)
    }
    fn to_string(d: &Descriptor) -> String {
        Self::to_string(d)
    }
    fn from_string(s: &str) -> StorageResult<Descriptor> {
        Self::from_string(s)
    }
}
