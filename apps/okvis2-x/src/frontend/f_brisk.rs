/*
 * BRISK descriptor operations corresponding to FBrisk.cpp.
 * Authors: Dorian Galvez-Lopez, Stefan Leutenegger.
 * License: BSD, see external/DBoW2/LICENSE.txt.
 */

use nalgebra::DMatrix;
use std::fmt::Write;

pub const DESCRIPTOR_LENGTH: usize = 48;
pub type Descriptor = [u8; DESCRIPTOR_LENGTH];

#[derive(Clone, Copy, Debug, Default)]
pub struct FBrisk;

impl FBrisk {
    pub fn mean_value(descriptors: &[&Descriptor], mean: &mut Descriptor) {
        mean.fill(0);
        let threshold = descriptors.len() as u64 / 2;
        let mut sum = [0_u64; DESCRIPTOR_LENGTH * 8];
        for descriptor in descriptors {
            for i in 0..DESCRIPTOR_LENGTH {
                for bit in 0..8 {
                    sum[i * 8 + bit] += u64::from(descriptor[i] & (1 << bit) != 0);
                }
            }
        }
        for i in 0..DESCRIPTOR_LENGTH {
            for bit in 0..8 {
                if sum[i * 8 + bit] > threshold {
                    mean[i] |= 1 << bit;
                }
            }
        }
    }

    #[must_use]
    pub fn distance(a: &Descriptor, b: &Descriptor) -> f64 {
        f64::from(
            a.iter()
                .zip(b)
                .map(|(&left, &right)| (left ^ right).count_ones())
                .sum::<u32>(),
        )
    }

    #[must_use]
    pub fn to_string(descriptor: &Descriptor) -> String {
        let mut result = String::new();
        for byte in descriptor {
            write!(&mut result, "{byte} ").expect("writing to a string cannot fail");
        }
        result
    }

    /// Reads 48 decimal integers, with the unsigned-byte conversion used by C++.
    /// Panics if any required integer is missing or invalid.
    /// # Panics
    /// Panics if a required resource/invariant is absent: `descriptor integer is missing`,
    /// `descriptor integer is invalid`.
    pub fn from_string(descriptor: &mut Descriptor, text: &str) {
        let mut values = text.split_whitespace();
        for byte in descriptor {
            let value: i32 = values
                .next()
                .expect("descriptor integer is missing")
                .parse()
                .expect("descriptor integer is invalid");
            *byte = value as u8;
        }
    }

    /// Expands each byte into eight bit masks: values are `0` or `1 << bit`.
    pub fn to_mat32f(descriptors: &[Descriptor], matrix: &mut DMatrix<f32>) {
        if descriptors.is_empty() {
            *matrix = DMatrix::zeros(0, 0);
            return;
        }
        *matrix = DMatrix::zeros(descriptors.len(), DESCRIPTOR_LENGTH * 8);
        for (i, descriptor) in descriptors.iter().enumerate() {
            for j in (0..DESCRIPTOR_LENGTH * 8).step_by(8) {
                for bit in 0..8 {
                    matrix[(i, j + bit)] = f32::from(descriptor[j / 8] & (1 << bit));
                }
            }
        }
    }
}

impl firefly_dbow2::templated_vocabulary::DescriptorOps for FBrisk {
    type Descriptor = Descriptor;
    fn mean_value(d: &[&Self::Descriptor]) -> Self::Descriptor {
        let mut mean = [0; 48];
        Self::mean_value(d, &mut mean);
        mean
    }
    fn distance(a: &Self::Descriptor, b: &Self::Descriptor) -> f64 {
        Self::distance(a, b)
    }
    fn to_string(d: &Self::Descriptor) -> String {
        Self::to_string(d)
    }
    fn from_string(
        s: &str,
    ) -> firefly_dbow2::templated_vocabulary::StorageResult<Self::Descriptor> {
        let values = s
            .split_whitespace()
            .take(DESCRIPTOR_LENGTH)
            .map(str::parse::<i32>)
            .collect::<std::result::Result<Vec<_>, _>>()?;
        if values.len() < 48 {
            return Err("BRISK descriptor requires 48 integers".into());
        }
        let mut d = [0; 48];
        for (out, x) in d.iter_mut().zip(values) {
            *out = x as u8;
        }
        Ok(d)
    }
}
