use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
pub struct TimeCamId {
    #[serde(rename = "value0")]
    pub frame_id: i64,
    #[serde(rename = "value1")]
    pub cam_id: usize,
}
impl TimeCamId {
    /// Numeric value of the upstream integer-based `std::hash<TimeCamId>`.
    #[must_use]
    pub fn hash_value(self) -> usize {
        let mut seed = 0;
        hash_combine(&mut seed, self.frame_id as usize);
        hash_combine(&mut seed, self.cam_id);
        seed
    }

    #[must_use]
    pub const fn new(frame_id: i64, cam_id: usize) -> Self {
        Self { frame_id, cam_id }
    }
}

impl fmt::Display for TimeCamId {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{}_{}", self.frame_id, self.cam_id)
    }
}

fn hash_combine(seed: &mut usize, value_hash: usize) {
    #[cfg(target_pointer_width = "64")]
    {
        *seed = (*seed as u64
            ^ (value_hash as u64)
                .wrapping_add(0x9e37_79b9_7f4a_7c15)
                .wrapping_add((*seed as u64) << 12)
                .wrapping_add((*seed as u64) >> 4)) as usize;
    }
    #[cfg(target_pointer_width = "32")]
    {
        *seed = (*seed as u32
            ^ (value_hash as u32)
                .wrapping_add(0x9e37_79b9)
                .wrapping_add((*seed as u32) << 6)
                .wrapping_add((*seed as u32) >> 2)) as usize;
    }
}
