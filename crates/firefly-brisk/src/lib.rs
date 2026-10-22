// SPDX-License-Identifier: BSD-3-Clause

//! BRISK descriptors and feature detection with the bundled AGAST corner detectors.

pub mod agast;
pub mod brisk_descriptor_extractor;
pub mod brisk_feature;
pub mod brisk_feature_detector;
pub mod brisk_layer;
pub mod brisk_scale_space;
pub mod brute_force_matcher;
pub mod hamming;
pub mod harris_feature_detector;
pub mod harris_score_calculator;
pub mod harris_score_calculator_float;
pub mod harris_scores;
pub mod helper_structures;
pub mod image_down_sampling;
pub mod integral_image;
pub mod pattern_provider;
pub mod scale_space_feature_detector;
pub mod scale_space_layer;
pub mod uniformity_enforcement;
pub mod vectorized_filters;
