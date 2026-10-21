/*
 * File: FORB.cpp
 * Date: June 2012
 * Author: Dorian Galvez-Lopez
 * Description: functions for ORB descriptors
 * License: see the LICENSE.txt file
 *
 */

use super::templated_vocabulary::{DescriptorOps, StorageResult};
use nalgebra::DMatrix;
use std::fmt::Write;
pub const L: usize = 32;
pub type Descriptor = Vec<u8>;
#[derive(Clone, Copy, Debug, Default)]
pub struct FOrb;
impl FOrb {
    /// Empty input releases the descriptor; ties set a bit as in FORB.cpp.
    #[must_use]
    pub fn mean_value(descriptors: &[&Descriptor]) -> Descriptor {
        if descriptors.is_empty() {
            return Vec::new();
        }
        if descriptors.len() == 1 {
            return descriptors[0].clone();
        }
        let mut counts = [0_usize; L * 8];
        for d in descriptors {
            for (j, &b) in d.iter().enumerate() {
                for bit in 0..8 {
                    if b & (1 << (7 - bit)) != 0 {
                        counts[j * 8 + bit] += 1;
                    }
                }
            }
        }
        let threshold = descriptors.len() / 2 + descriptors.len() % 2;
        let mut mean = vec![0; L];
        for (i, &n) in counts.iter().enumerate() {
            if n >= threshold {
                mean[i / 8] |= 1 << (7 - i % 8);
            }
        }
        mean
    }
    /// Only complete uint64 groups of a's byte columns participate.
    #[must_use]
    pub fn distance(a: &Descriptor, b: &Descriptor) -> f64 {
        f64::from(
            a.as_chunks::<8>()
                .0
                .iter()
                .enumerate()
                .map(|(i, ac)| {
                    ac.iter()
                        .zip(&b[i * 8..i * 8 + 8])
                        .map(|(&a, &b)| (a ^ b).count_ones())
                        .sum::<u32>()
                })
                .sum::<u32>(),
        )
    }
    #[must_use]
    pub fn to_string(a: &Descriptor) -> String {
        let mut out = String::new();
        for b in a {
            write!(&mut out, "{b} ").expect("String write");
        }
        out
    }
    /// Missing/invalid bytes are uninitialized in the source and return an error.
    ///
    /// # Errors
    /// Returns an error if fewer than 32 signed integers can be extracted or an integer
    /// overflows `i32`.
    pub fn from_string(s: &str) -> StorageResult<Descriptor> {
        let mut rest = s;
        let mut out = Vec::with_capacity(L);
        for _ in 0..L {
            rest = rest.trim_start_matches(|c: char| c.is_ascii_whitespace());
            let bytes = rest.as_bytes();
            let mut end = usize::from(bytes.first().is_some_and(|b| *b == b'+' || *b == b'-'));
            let digits = end;
            while end < bytes.len() && bytes[end].is_ascii_digit() {
                end += 1;
            }
            if end == digits {
                return Err("ORB descriptor requires 32 integers".into());
            }
            out.push(rest[..end].parse::<i32>()? as u8);
            rest = &rest[end..];
        }
        Ok(out)
    }
    /// Expands vector descriptors in most-significant-bit-first order.
    ///
    /// # Errors
    /// Returns an error if a descriptor does not contain exactly 32 bytes.
    pub fn to_mat32f(descriptors: &[Descriptor]) -> StorageResult<DMatrix<f32>> {
        if descriptors.is_empty() {
            return Ok(DMatrix::zeros(0, 0));
        }
        if descriptors.iter().any(|d| d.len() != L) {
            return Err("ORB bit matrix requires 32 byte descriptors".into());
        }
        Ok(DMatrix::from_fn(descriptors.len(), L * 8, |r, c| {
            if descriptors[r][c / 8] & (1 << (7 - c % 8)) != 0 {
                1.0
            } else {
                0.0
            }
        }))
    }
    /// The matrix overload converts byte values without expanding bits.
    pub fn matrix_to_mat32f(descriptors: &DMatrix<u8>) -> DMatrix<f32> {
        descriptors.map(f32::from)
    }
    /// # Errors
    /// Returns an error if a descriptor contains fewer than 32 bytes.
    pub fn to_mat8u(descriptors: &[Descriptor]) -> StorageResult<DMatrix<u8>> {
        if descriptors.iter().any(|d| d.len() < L) {
            return Err("ORB byte matrix requires 32 bytes per descriptor".into());
        }
        Ok(DMatrix::from_fn(descriptors.len(), L, |r, c| {
            descriptors[r][c]
        }))
    }
}
impl DescriptorOps for FOrb {
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
