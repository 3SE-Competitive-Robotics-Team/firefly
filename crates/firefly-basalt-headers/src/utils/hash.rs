// SPDX-License-Identifier: BSD-3-Clause
// Source: basalt-headers aa441ba3e51050c47ba1902537792a2e4db7e43d,
// include/basalt/utils/hash.h. See the crate LICENSE.

/// Combines the hash of a value into the seed with native `size_t` arithmetic.
/// Pass the upstream `std::hash<T>` result, not a Rust randomized hash.
pub fn hash_combine(seed: &mut usize, value_hash: usize) {
    #[cfg(target_pointer_width = "64")]
    {
        *seed = hash_combine64(*seed as u64, value_hash as u64) as usize;
    }
    #[cfg(target_pointer_width = "32")]
    {
        *seed = hash_combine32(*seed as u32, value_hash as u32) as usize;
    }
}

#[must_use]
pub const fn hash_combine64(seed: u64, value_hash: u64) -> u64 {
    seed ^ value_hash
        .wrapping_add(0x9e37_79b9_7f4a_7c15)
        .wrapping_add(seed << 12)
        .wrapping_add(seed >> 4)
}

#[must_use]
pub const fn hash_combine32(seed: u32, value_hash: u32) -> u32 {
    seed ^ value_hash
        .wrapping_add(0x9e37_79b9)
        .wrapping_add(seed << 6)
        .wrapping_add(seed >> 2)
}
