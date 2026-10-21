/*
 * File: FSurf64.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: functions for Surf64 descriptors
 * License: see the LICENSE.txt file
 *
 */

use super::templated_vocabulary::{DescriptorOps, StorageResult};
use nalgebra::DMatrix;
pub const L: usize = 64;
pub type Descriptor = Vec<f32>;
#[derive(Clone, Copy, Debug, Default)]
pub struct FSurf64;
impl FSurf64 {
    #[must_use]
    pub fn dimensions() -> usize {
        L
    }
    #[must_use]
    pub fn mean_value(descriptors: &[&Descriptor]) -> Descriptor {
        let mut mean = vec![0.0; L];
        let s = descriptors.len() as f32;
        for d in descriptors {
            for i in (0..L).step_by(4) {
                mean[i] += d[i] / s;
                mean[i + 1] += d[i + 1] / s;
                mean[i + 2] += d[i + 2] / s;
                mean[i + 3] += d[i + 3] / s;
            }
        }
        mean
    }
    #[must_use]
    pub fn distance(a: &Descriptor, b: &Descriptor) -> f64 {
        let mut sqd = 0.0_f64;
        for i in (0..L).step_by(4) {
            for j in i..i + 4 {
                let d = a[j] - b[j];
                sqd += f64::from(d * d);
            }
        }
        sqd
    }
    #[must_use]
    pub fn to_string(a: &Descriptor) -> String {
        let mut out = String::new();
        for &x in &a[..L] {
            out.push_str(&default_float(x));
            out.push(' ');
        }
        out
    }
    /// Resizing value-initializes new slots; failed extraction leaves later slots intact.
    pub fn from_string_into(a: &mut Descriptor, s: &str) {
        a.resize(L, 0.0);
        let mut rest = s;
        for out in a.iter_mut() {
            rest = rest.trim_start_matches(|c: char| c.is_ascii_whitespace());
            if rest.is_empty() {
                break;
            }
            let bytes = rest.as_bytes();
            let mut end = usize::from(bytes[0] == b'+' || bytes[0] == b'-');
            let mut digits = 0;
            while end < bytes.len() && bytes[end].is_ascii_digit() {
                end += 1;
                digits += 1;
            }
            if end < bytes.len() && bytes[end] == b'.' {
                end += 1;
                while end < bytes.len() && bytes[end].is_ascii_digit() {
                    end += 1;
                    digits += 1;
                }
            }
            if digits == 0 {
                *out = 0.0;
                break;
            }
            if end < bytes.len() && (bytes[end] == b'e' || bytes[end] == b'E') {
                end += 1;
                if end < bytes.len() && (bytes[end] == b'+' || bytes[end] == b'-') {
                    end += 1;
                }
                let exponent_start = end;
                while end < bytes.len() && bytes[end].is_ascii_digit() {
                    end += 1;
                }
                if exponent_start == end {
                    *out = 0.0;
                    break;
                }
            }
            match rest[..end].parse::<f32>() {
                Ok(value) if value.is_finite() => *out = value,
                Ok(value) => {
                    *out = if value.is_sign_negative() {
                        -f32::MAX
                    } else {
                        f32::MAX
                    };
                    break;
                }
                Err(_) => {
                    *out = 0.0;
                    break;
                }
            }
            rest = &rest[end..];
        }
    }
    /// # Errors
    /// This implementation returns `Ok`; malformed fields follow the stream-extraction rules in
    /// `Self::from_string_into`.
    pub fn from_string(s: &str) -> StorageResult<Descriptor> {
        let mut a = Vec::new();
        Self::from_string_into(&mut a, s);
        Ok(a)
    }
    #[must_use]
    pub fn to_mat32f(descriptors: &[Descriptor]) -> DMatrix<f32> {
        if descriptors.is_empty() {
            return DMatrix::zeros(0, 0);
        }
        DMatrix::from_fn(descriptors.len(), L, |r, c| descriptors[r][c])
    }
}
/// C++ defaultfloat's six significant decimal digits and exponent formatting.
fn default_float(value: f32) -> String {
    if value.is_nan() {
        return if value.is_sign_negative() {
            "-nan"
        } else {
            "nan"
        }
        .into();
    }
    if value.is_infinite() {
        return if value.is_sign_negative() {
            "-inf"
        } else {
            "inf"
        }
        .into();
    }
    let scientific = format!("{value:.5e}");
    let (mantissa, exponent) = scientific.split_once('e').expect("scientific exponent");
    let exponent: i32 = exponent.parse().expect("numeric exponent");
    if (-4..6).contains(&exponent) {
        let precision = (5 - exponent).max(0) as usize;
        let rounded = scientific.parse::<f64>().expect("finite scientific number");
        let fixed = format!("{rounded:.precision$}");
        if fixed.contains('.') {
            fixed.trim_end_matches('0').trim_end_matches('.').into()
        } else {
            fixed
        }
    } else {
        let m = mantissa.trim_end_matches('0').trim_end_matches('.');
        format!("{m}e{exponent:+03}")
    }
}
impl DescriptorOps for FSurf64 {
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
