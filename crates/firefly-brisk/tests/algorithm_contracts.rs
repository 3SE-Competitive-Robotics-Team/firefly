// SPDX-License-Identifier: BSD-3-Clause

//! Independent bit/rectangle oracles and synthetic feature contracts; no image assets or golden data.
use firefly_brisk::{
    agast::agast5_8::AgastDetector58,
    brisk_descriptor_extractor::{BriskDescriptorExtractor, BriskVersion},
    brute_force_matcher::BruteForceMatcher,
    hamming::Hamming,
    integral_image::integral_image_8,
};
use proptest::prelude::*;
use purecv::{
    core::{Matrix, Point2f},
    features2d::KeyPoint,
};

proptest! {
    #![proptest_config(ProptestConfig::with_failure_persistence(
        proptest::test_runner::FileFailurePersistence::WithSource("proptest-regressions")
    ))]
    #[test]
    fn hamming_384_bits_matches_bitwise_definition(a in any::<[u8; 48]>(), b in any::<[u8; 48]>()) {
        let expected: i32 = (0..384).map(|bit| i32::from((a[bit / 8] >> (bit % 8)) & 1 != (b[bit / 8] >> (bit % 8)) & 1)).sum();
        prop_assert_eq!(Hamming.distance(&a, &b, 48).unwrap(), expected);
        prop_assert_eq!(Hamming.distance(&b, &a, 48).unwrap(), expected);
        prop_assert_eq!(Hamming.distance(&a, &a, 48).unwrap(), 0);
        prop_assert_eq!(Hamming::neon_popcnt_of_xored(&a, &b, 3).unwrap(), expected as u32);
        prop_assert_eq!(Hamming::ssse3_popcnt_of_xored(&a, &b, 3).unwrap(), expected as u32);
    }

    #[test]
    fn integral_rectangles_equal_direct_pixel_sums(pixels in prop::collection::vec(any::<u8>(), 35)) {
        let mut image = Matrix::<u8>::new(5, 7, 1);
        image.data = pixels;
        let integral = integral_image_8(&image).unwrap();
        prop_assert!(integral.data[..8].iter().all(|&v| v == 0));
        for y0 in 0..5 {
            for y1 in y0+1..=5 {
                for x0 in 0..7 {
                    for x1 in x0+1..=7 {
                        let expected: i32 = (y0..y1).flat_map(|y| &image.data[y*7+x0..y*7+x1]).map(|&v| i32::from(v)).sum();
                        let actual = integral.data[y1*8+x1] - integral.data[y0*8+x1] - integral.data[y1*8+x0] + integral.data[y0*8+x0];
                        prop_assert_eq!(actual, expected);
                    }
                }
            }
        }
    }
}

#[test]
fn agast_exhaustive_ring_neighbours_cyclic_five_pixel_definition() {
    // A 3x3 image has exactly one eligible centre. Enumerate every ring and both polarities.
    let mut detector = AgastDetector58::new(3, 3, 20);
    for polarity in [-1, 1] {
        for pattern in 0u16..256 {
            let mut image = vec![128u8; 9];
            for (bit, &(dx, dy)) in AgastDetector58::PATTERN.iter().enumerate() {
                image[((1 + dy) * 3 + 1 + dx) as usize] = if pattern & (1 << bit) != 0 {
                    (128 + polarity * 21) as u8
                } else {
                    128
                };
            }
            let expected = (0..8)
                .any(|start| (0..5).all(|offset| pattern & (1 << ((start + offset) % 8)) != 0));
            let mut corners = Vec::new();
            detector.detect(&image, &mut corners, None).unwrap();
            assert_eq!(
                corners.len(),
                usize::from(expected),
                "pattern={pattern:08b}, polarity={polarity}"
            );
        }
    }
    let mut image = vec![148; 9];
    image[4] = 128;
    let mut corners = Vec::new();
    detector.detect(&image, &mut corners, None).unwrap();
    assert!(
        corners.is_empty(),
        "contrast equal to threshold is not a corner"
    );
}

#[test]
fn malformed_images_and_short_descriptors_return_errors() {
    let mut image = Matrix::<u8>::new(2, 3, 1);
    image.data.pop();
    assert!(integral_image_8(&image).is_err());
    assert!(Hamming.distance(&[0; 47], &[0; 48], 48).is_err());
    assert!(
        AgastDetector58::new(3, 3, 20)
            .detect(&[0; 8], &mut Vec::new(), None)
            .is_err()
    );
    let empty = integral_image_8(&Matrix::<u8>::new(0, 0, 1)).unwrap();
    assert_eq!((empty.rows, empty.cols, empty.data), (1, 1, vec![0]));
}

#[test]
fn matcher_orders_neighbours_and_uses_strict_radius_and_masks() {
    let query = Matrix::<u8>::new(1, 48, 1);
    let mut train = Matrix::<u8>::new(3, 48, 1);
    train.data[48] = 1;
    train.data[96] = 3;
    let mut matcher = BruteForceMatcher::new(Hamming);
    matcher.train_descriptors.push(train);
    let mut neighbours = Vec::new();
    matcher
        .knn_match(&query, &mut neighbours, 3, &[], false)
        .unwrap();
    assert_eq!(
        neighbours[0]
            .iter()
            .map(|m| (m.train_idx, m.distance))
            .collect::<Vec<_>>(),
        [(0, 0.0), (1, 1.0), (2, 2.0)]
    );
    let mut neighbours = Vec::new();
    matcher
        .radius_match(&query, &mut neighbours, 1.0, &[], false)
        .unwrap();
    assert_eq!(neighbours[0].len(), 1);
    assert_eq!(neighbours[0][0].train_idx, 0);
    let mut mask = Matrix::<u8>::new(1, 3, 1);
    mask.data[2] = 1;
    let mut neighbours = Vec::new();
    matcher
        .radius_match(&query, &mut neighbours, 3.0, &[mask], false)
        .unwrap();
    assert_eq!(neighbours[0].len(), 1);
    assert_eq!(neighbours[0][0].train_idx, 2);
    let mut neighbours = Vec::new();
    matcher
        .knn_match(
            &query,
            &mut neighbours,
            1,
            &[Matrix::<u8>::new(1, 3, 1)],
            true,
        )
        .unwrap();
    assert_eq!(neighbours.len(), 0);
    assert!(
        matcher
            .knn_match(
                &query,
                &mut neighbours,
                1,
                &[Matrix::<u8>::new(2, 3, 1)],
                false
            )
            .is_err()
    );
}

#[test]
fn descriptor_preserves_integer_translation_and_rejects_invalid_keypoints() {
    let extractor = BriskDescriptorExtractor::new(false, true, BriskVersion::V2, 1.0).unwrap();
    let mut image = Matrix::<u8>::new(160, 160, 1);
    let mut shifted = image.clone();
    for y in 0..160 {
        for x in 0..160 {
            image.data[y * 160 + x] = ((x * 17 + y * 29 + x * y * 3) % 256) as u8;
            if x >= 4 {
                shifted.data[y * 160 + x] = image.data[y * 160 + x - 4];
            }
        }
    }
    let mut points = vec![KeyPoint::new(
        Point2f::new(75.0, 80.0),
        12.0,
        -1.0,
        1.0,
        0,
        -1,
    )];
    let first = extractor.compute(&image, &mut points).unwrap();
    assert_eq!(points.len(), 1);
    assert_eq!((first.rows, first.cols), (1, extractor.descriptor_size()));
    assert!(first.data.iter().any(|&v| v != 0));
    points[0].pt.x += 4.0;
    let second = extractor.compute(&shifted, &mut points).unwrap();
    assert_eq!(first.data, second.data);
    let mut invalid = vec![KeyPoint::new(
        Point2f::new(f32::NAN, 80.0),
        12.0,
        -1.0,
        1.0,
        0,
        -1,
    )];
    assert!(extractor.compute(&image, &mut invalid).is_err());
    let mut border = vec![KeyPoint::new(
        Point2f::new(0.0, 0.0),
        12.0,
        -1.0,
        1.0,
        0,
        -1,
    )];
    let empty = extractor.compute(&image, &mut border).unwrap();
    assert_eq!(border.len(), 0);
    assert_eq!(empty.rows, 0);
}
