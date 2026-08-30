//! 快速向下取整（对照 official `util/fast_floor.hpp`）。
//!
//! 等价 `std::floor`，但避免浮点取整函数开销：先截断为整数，再对负值修正。
//! 用于体素坐标计算，必须与点坐标的下取整语义严格一致。

use nalgebra::Vector4;

/// 对四维齐次点逐分量快速下取整，返回 `(i32, i32, i32, i32)`。
///
/// `ncoord = trunc(pt)`；若 `pt < ncoord`（截断偏向零，负尾数被高估），再减一。
/// 例：`-0.5 → trunc 0`，因 `-0.5 < 0` 修正为 `-1`，即 `floor(-0.5)`。
pub fn fast_floor(pt: &Vector4<f64>) -> [i32; 4] {
    let ncoord = [pt[0] as i32, pt[1] as i32, pt[2] as i32, pt[3] as i32];
    let mut out = ncoord;
    for k in 0..4 {
        if pt[k] < f64::from(ncoord[k]) {
            out[k] -= 1;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_std_floor() {
        let mut rng = splitmix64_seed(20240827);
        for _ in 0..1000 {
            let v = Vector4::new(
                rand_f64(&mut rng, -1000.0, 1000.0),
                rand_f64(&mut rng, -1000.0, 1000.0),
                rand_f64(&mut rng, -1000.0, 1000.0),
                1.0,
            );
            let f = fast_floor(&v);
            assert_eq!(f[0], pt_floor(v[0]));
            assert_eq!(f[1], pt_floor(v[1]));
            assert_eq!(f[2], pt_floor(v[2]));
            assert_eq!(f[3], pt_floor(v[3]));
        }
    }

    fn pt_floor(x: f64) -> i32 {
        x.floor() as i32
    }

    fn rand_f64(rng: &mut u64, lo: f64, hi: f64) -> f64 {
        let u = splitmix64_next(rng);
        lo + (u as f64 / u64::MAX as f64) * (hi - lo)
    }

    fn splitmix64_seed(seed: u64) -> u64 {
        seed ^ 0x9E37_9B97_F4A7_C15B
    }

    fn splitmix64_next(state: &mut u64) -> u64 {
        *state = state.wrapping_add(0x9E37_9B97_F4A7_C15B);
        let mut z = *state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
}
