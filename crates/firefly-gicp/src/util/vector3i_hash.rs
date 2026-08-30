//! 空间哈希（对照 official `util/vector3i_hash.hpp`）。
//!
//! Teschner et al., "Optimized Spatial Hashing for Collision Detection of
//! Deformable Objects", VMV2003。把体素整数坐标异或散列到 `usize`。

/// 三维整数坐标异或哈希（对照 `XORVector3iHash`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct XorVector3iHash;

impl XorVector3iHash {
    /// 哈希 `x`（体素整数坐标）。
    ///
    /// 三个轴乘互异素数后异或折叠，碰撞率低且计算廉价。
    pub fn hash(x: [i32; 3]) -> usize {
        const P1: usize = 73_856_093;
        const P2: usize = 19_349_669; // 19_349_663 非素数，官方已修正
        const P3: usize = 83_492_791;
        // 先把 i32 视为无符号再乘，避免溢出回绕掩盖符号位差异
        ((x[0] as i64 as u64 as usize).wrapping_mul(P1))
            ^ ((x[1] as i64 as u64 as usize).wrapping_mul(P2))
            ^ ((x[2] as i64 as u64 as usize).wrapping_mul(P3))
    }

    /// 相等判断（体素坐标逐分量相等）。
    pub fn equal(x1: [i32; 3], x2: [i32; 3]) -> bool {
        x1 == x2
    }

    /// 函数调用语义（同 `operator()`）。
    pub fn call(&self, x: [i32; 3]) -> usize {
        Self::hash(x)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn consistent_and_equal() {
        let mut rng = 0x1234_5678_u64;
        for _ in 0..1000 {
            let v = [
                splitmix_i32(&mut rng),
                splitmix_i32(&mut rng),
                splitmix_i32(&mut rng),
            ];
            let h = XorVector3iHash;
            assert_eq!(XorVector3iHash::hash(v), h.call(v));
            assert!(XorVector3iHash::equal(v, v));
        }
    }

    fn splitmix_i32(state: &mut u64) -> i32 {
        *state = state.wrapping_add(0x9E37_9B97_F4A7_C15B);
        let mut z = *state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        (z ^ (z >> 31)) as i32
    }
}
