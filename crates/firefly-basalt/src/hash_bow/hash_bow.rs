// SPDX-License-Identifier: BSD-3-Clause
// Source: VladyslavUsenko/basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc,
// include/basalt/hash_bow/hash_bow.h. Basalt project LICENSE-BSD-3-Clause.
use crate::utils::common_types::{FEATURE_HASH_MAX_SIZE, FeatureHash, FrameId, HashBowVector};
use firefly_calibration::observation::TimeCamId;
use std::{collections::HashMap, sync::RwLock};

/// Inverted-index histogram retrieval with Basalt's fixed random bit permutation.
pub struct HashBow<const N: usize> {
    num_bits: usize,
    word_bit_permutation: [usize; FEATURE_HASH_MAX_SIZE],
    inverted_index: RwLock<HashMap<FeatureHash, Vec<(TimeCamId, f64)>>>,
}
impl<const N: usize> HashBow<N> {
    /// # Panics
    /// Panics unless the descriptor has fewer than 512 bits.
    #[must_use]
    pub fn new(num_bits: usize) -> Self {
        assert!(N < 512, "descriptor length must be below 512");
        Self {
            num_bits: num_bits.min(32),
            word_bit_permutation: Self::compute_permutation(),
            inverted_index: RwLock::new(HashMap::new()),
        }
    }

    const fn compute_permutation() -> [usize; FEATURE_HASH_MAX_SIZE] {
        let mut result = [0; FEATURE_HASH_MAX_SIZE];
        let mut j = 0;
        let mut i = 0;
        while i < 512 && j < FEATURE_HASH_MAX_SIZE {
            if RANDOM_BIT_PERMUTATION[i] < N {
                result[j] = RANDOM_BIT_PERMUTATION[i];
                j += 1;
            }
            i += 1;
        }
        result
    }

    /// Reads words in increasing bit-index order; unused high bits are ignored.
    /// # Panics
    /// Panics if the array does not have exactly ceil(N/64) words, or a zero-bit
    /// descriptor is read with a nonzero hash width.
    #[must_use]
    pub fn compute_hash<const WORDS: usize>(&self, descriptor: &[u64; WORDS]) -> FeatureHash {
        assert_eq!(WORDS, N.div_ceil(64));
        let mut result = 0;
        for (i, &bit) in self.word_bit_permutation[..self.num_bits]
            .iter()
            .enumerate()
        {
            result |= (((descriptor[bit / 64] >> (bit % 64)) & 1) as u32) << i;
        }
        result
    }

    /// Replaces output hashes and the L1-normalized histogram; histogram order is unspecified.
    /// # Panics
    /// Panics if a descriptor has an incompatible word count.
    pub fn compute_bow<const WORDS: usize>(
        &self,
        descriptors: &[[u64; WORDS]],
        hashes: &mut Vec<FeatureHash>,
        bow: &mut HashBowVector,
    ) {
        hashes.resize(descriptors.len(), 0);
        let mut histogram = HashMap::with_capacity(descriptors.len());
        for (i, descriptor) in descriptors.iter().enumerate() {
            hashes[i] = self.compute_hash(descriptor);
            *histogram.entry(hashes[i]).or_insert(0.0_f64) += 1.;
        }
        bow.clear();
        let mut l1_sum = 0.;
        for (hash, count) in histogram {
            bow.push((hash, count));
            l1_sum += count.abs();
        }
        for (_, weight) in bow {
            *weight /= l1_sum;
        }
    }

    /// Appends all histogram entries, including repeated additions for the same image.
    /// # Panics
    /// Panics if a database writer has poisoned the lock.
    pub fn add_to_database(&self, image: TimeCamId, bow: &HashBowVector) {
        let mut index = self
            .inverted_index
            .write()
            .expect("HashBoW database lock poisoned");
        for &(word, weight) in bow {
            index.entry(word).or_default().push((image, weight));
        }
    }

    /// Scores matching entries by histogram intersection with a strict timestamp cutoff.
    /// Results are sorted only when truncated; tied result ordering is unspecified.
    /// # Panics
    /// Panics if a database writer has poisoned the lock.
    pub fn query_database(
        &self,
        bow: &HashBowVector,
        num_results: usize,
        results: &mut Vec<(TimeCamId, f64)>,
        max_t_ns: Option<FrameId>,
    ) {
        results.clear();
        let index = self
            .inverted_index
            .read()
            .expect("HashBoW database lock poisoned");
        let mut scores = HashMap::new();
        for &(word, weight) in bow {
            if let Some(entries) = index.get(&word) {
                for &(image, other) in entries {
                    if max_t_ns.is_none_or(|limit| image.frame_id < limit) {
                        *scores.entry(image).or_insert(0.) +=
                            (weight - other).abs() - weight.abs() - other.abs();
                    }
                }
            }
        }
        results.reserve(scores.len());
        results.extend(
            scores
                .into_iter()
                .map(|(image, score)| (image, -score / 2.)),
        );
        if results.len() > num_results {
            results.select_nth_unstable_by(num_results, |a, b| b.1.total_cmp(&a.1));
            results.truncate(num_results);
            results.sort_unstable_by(|a, b| b.1.total_cmp(&a.1));
        }
    }
}

const RANDOM_BIT_PERMUTATION: [usize; 512] = [
    484, 458, 288, 170, 215, 424, 41, 38, 293, 96, 172, 428, 508, 52, 370, 1, 182, 472, 89, 339,
    273, 234, 98, 217, 73, 195, 307, 306, 113, 429, 161, 443, 364, 439, 301, 247, 325, 24, 490,
    366, 75, 7, 464, 232, 49, 196, 144, 69, 470, 387, 3, 86, 361, 313, 396, 356, 94, 201, 291, 360,
    107, 251, 413, 393, 296, 124, 308, 146, 298, 160, 121, 302, 151, 345, 336, 26, 63, 238, 79,
    267, 262, 437, 433, 350, 53, 134, 194, 452, 114, 54, 82, 214, 191, 242, 482, 37, 432, 311, 130,
    460, 422, 221, 271, 192, 474, 46, 289, 34, 20, 95, 463, 499, 159, 272, 481, 129, 448, 173, 323,
    258, 416, 229, 334, 510, 461, 263, 362, 346, 39, 500, 381, 401, 492, 299, 33, 169, 241, 11,
    254, 449, 199, 486, 400, 365, 70, 436, 108, 19, 233, 505, 152, 6, 480, 468, 278, 426, 253, 471,
    328, 327, 139, 29, 27, 488, 332, 290, 412, 164, 259, 352, 222, 186, 32, 319, 410, 211, 405,
    187, 213, 507, 205, 395, 62, 178, 36, 140, 87, 491, 351, 450, 314, 77, 342, 132, 133, 477, 103,
    389, 206, 197, 324, 485, 425, 297, 231, 123, 447, 126, 9, 64, 181, 40, 14, 5, 261, 431, 333,
    223, 4, 138, 220, 76, 44, 300, 331, 78, 193, 497, 403, 435, 275, 147, 66, 368, 141, 451, 225,
    250, 61, 18, 444, 208, 380, 109, 255, 337, 372, 212, 359, 457, 31, 398, 354, 219, 117, 248,
    392, 203, 88, 479, 509, 149, 120, 145, 51, 15, 367, 190, 163, 417, 454, 329, 183, 390, 83, 404,
    249, 81, 264, 445, 317, 179, 244, 473, 71, 111, 118, 209, 171, 224, 459, 446, 104, 13, 377,
    200, 414, 198, 420, 226, 153, 384, 25, 441, 305, 338, 316, 483, 184, 402, 48, 131, 502, 252,
    469, 12, 167, 243, 373, 35, 127, 341, 455, 379, 210, 340, 128, 430, 57, 434, 330, 415, 494,
    142, 355, 282, 322, 65, 105, 421, 68, 409, 466, 245, 59, 269, 112, 386, 257, 256, 93, 174, 16,
    60, 143, 343, 115, 506, 276, 10, 496, 489, 235, 47, 136, 22, 165, 204, 42, 465, 440, 498, 312,
    504, 116, 419, 185, 303, 218, 353, 283, 374, 2, 177, 137, 240, 102, 309, 292, 85, 453, 388,
    397, 438, 281, 279, 442, 110, 55, 101, 100, 150, 375, 406, 157, 23, 0, 237, 376, 236, 216, 8,
    154, 91, 456, 423, 176, 427, 284, 30, 84, 349, 335, 56, 270, 227, 286, 168, 239, 122, 478, 162,
    475, 166, 17, 348, 285, 175, 155, 266, 382, 304, 268, 180, 295, 125, 371, 467, 277, 294, 58,
    347, 72, 280, 50, 287, 511, 80, 260, 326, 495, 45, 106, 399, 369, 503, 357, 315, 418, 487, 99,
    43, 320, 188, 407, 246, 501, 119, 158, 274, 408, 230, 358, 90, 148, 363, 207, 344, 265, 462,
    189, 310, 385, 67, 28, 383, 378, 156, 394, 97, 476, 493, 321, 411, 228, 21, 391, 202, 92, 318,
    74, 135,
];

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;
    #[test]
    fn matches_cpp_oracle() {
        let expected: serde_json::Value =
            serde_json::from_str(include_str!("../../tests/reference/utils.json")).unwrap();
        let bow = HashBow::<256>::new(20);
        let mut descriptors = [[0_u64; 4]; 4];
        descriptors[1] = [u64::MAX; 4];
        for bit in (0..256).step_by(3) {
            descriptors[2][bit / 64] |= 1 << (bit % 64);
        }
        descriptors[3] = descriptors[2];
        let mut hashes = Vec::new();
        let mut histogram = Vec::new();
        bow.compute_bow(&descriptors, &mut hashes, &mut histogram);
        histogram.sort_unstable_by_key(|word| word.0);
        assert_eq!(serde_json::json!(hashes), expected["hashes"]);
        assert_eq!(serde_json::json!(histogram), expected["bow"]);
        bow.add_to_database(TimeCamId::new(10, 0), &histogram);
        bow.add_to_database(TimeCamId::new(20, 1), &vec![(hashes[0], 1.)]);
        for (index, cutoff) in [None, Some(20), Some(21)].into_iter().enumerate() {
            let mut results = Vec::new();
            bow.query_database(&histogram, 10, &mut results, cutoff);
            results.sort_unstable_by_key(|r| r.0);
            let actual: Vec<_> = results
                .into_iter()
                .map(|(image, score)| serde_json::json!([image.frame_id, image.cam_id, score]))
                .collect();
            let rows = expected["query"][index]["results"].as_array().unwrap();
            assert_eq!(actual.len(), rows.len());
            for (actual, row) in actual.iter().zip(rows) {
                assert_eq!(actual[0], row[0]);
                assert_eq!(actual[1], row[1]);
                assert_eq!(actual[2].as_f64(), row[2].as_f64());
            }
        }
    }
    #[test]
    fn bits_histogram_and_timestamp_boundary_match_source() {
        let bow = HashBow::<256>::new(2);
        let mut a = [0; 4];
        a[170 / 64] |= 1 << (170 % 64);
        let mut b = [0; 4];
        b[215 / 64] |= 1 << (215 % 64);
        assert_eq!(bow.compute_hash(&a), 1);
        assert_eq!(bow.compute_hash(&b), 2);
        let mut hashes = Vec::new();
        let mut histogram = Vec::new();
        bow.compute_bow(&[a, a, b], &mut hashes, &mut histogram);
        assert_eq!(hashes, vec![1, 1, 2]);
        assert_eq!(
            histogram.iter().copied().collect::<BTreeMap<_, _>>(),
            BTreeMap::from([(1, 2. / 3.), (2, 1. / 3.)])
        );
        bow.add_to_database(TimeCamId::new(10, 0), &histogram);
        bow.add_to_database(TimeCamId::new(20, 0), &vec![(1, 1.)]);
        let mut results = Vec::new();
        bow.query_database(&histogram, 1, &mut results, None);
        assert_eq!(results, vec![(TimeCamId::new(10, 0), 1.)]);
        bow.query_database(&histogram, 10, &mut results, Some(10));
        assert_eq!(results, Vec::<(TimeCamId, f64)>::new());
        bow.query_database(&histogram, 10, &mut results, Some(20));
        assert_eq!(results, vec![(TimeCamId::new(10, 0), 1.)]);
        bow.query_database(&histogram, 0, &mut results, None);
        assert_eq!(results, Vec::<(TimeCamId, f64)>::new());
        bow.compute_bow::<4>(&[], &mut hashes, &mut histogram);
        assert!(hashes.is_empty() && histogram.is_empty());
    }

    #[test]
    fn concurrent_inserts_and_repeated_images_preserve_all_contributions() {
        let bow = HashBow::<256>::new(64);
        std::thread::scope(|scope| {
            for camera in 0..4 {
                let db = &bow;
                scope.spawn(move || db.add_to_database(TimeCamId::new(1, camera), &vec![(1, 1.)]));
            }
        });
        bow.add_to_database(TimeCamId::new(1, 0), &vec![(1, 1.)]);
        let mut results = Vec::new();
        bow.query_database(&vec![(1, 1.)], 4, &mut results, None);
        assert_eq!(
            results.into_iter().collect::<BTreeMap<_, _>>(),
            BTreeMap::from([
                (TimeCamId::new(1, 0), 2.),
                (TimeCamId::new(1, 1), 1.),
                (TimeCamId::new(1, 2), 1.),
                (TimeCamId::new(1, 3), 1.)
            ])
        );
        assert_eq!(bow.compute_hash(&[u64::MAX; 4]), u32::MAX);
        assert_eq!(HashBow::<1>::new(32).compute_hash(&[1]), u32::MAX);
        assert_eq!(HashBow::<256>::new(0).compute_hash(&[u64::MAX; 4]), 0);
        assert_eq!(HashBow::<0>::new(0).compute_hash(&[]), 0);
    }
}
