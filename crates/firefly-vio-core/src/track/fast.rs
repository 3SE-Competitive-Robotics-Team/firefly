//! FAST-9 角点检测（基于 purecv `FastFeatureDetector`）。
//!
//! purecv 内部使用 Edward Rosten 的优化算法 + `parallel` feature 的 Rayon
//! 行并行扫描，对齐 OpenCV `cv::FAST` 的检测语义。
//!
//! 与自实现版本的差异：
//! - NMS 用严格 `>`（purecv）vs 扫描序 `>=`（自实现）：合成同分场景行为不同，
//!   自然图像无影响；
//! - 得分类型 `u8`（purecv Rosten 算法）vs `f32`（自实现连续段差之和）：
//!   值域不同但排序语义一致。

use crate::sensor::GrayImage;
use crate::track::KeyPoint;
use purecv::features2d::{FastFeatureDetector, FastType};

/// 将 `GrayImage` 转为 purecv `Matrix<u8>`（单通道）。
fn gray_to_matrix(img: &GrayImage) -> purecv::core::Matrix<u8> {
    purecv::core::Matrix::from_vec(img.height, img.width, 1, img.data.clone())
}

/// FAST-9 角点检测。
///
/// - `threshold`：强度阈值（像素差，0–255）；
/// - `nonmax_suppression`：是否做非极大值抑制。
///
/// 返回角点列表。边界 3 像素内不检测（窗口半径 3 无法完整采样）。
#[must_use]
pub fn fast(img: &GrayImage, threshold: i32, nonmax_suppression: bool) -> Vec<KeyPoint> {
    fast_with_score(img, threshold, nonmax_suppression).0
}

/// 与 [`fast`] 相同，但额外返回每个角点的得分（供 grider 排序复用）。
#[must_use]
pub fn fast_with_score(
    img: &GrayImage,
    threshold: i32,
    nonmax_suppression: bool,
) -> (Vec<KeyPoint>, Vec<f32>) {
    if img.width < 4 || img.height < 4 {
        return (Vec::new(), Vec::new());
    }
    let mat = gray_to_matrix(img);
    let detector = FastFeatureDetector::new(
        threshold.clamp(0, 255) as u8,
        nonmax_suppression,
        FastType::Type9_16,
    );
    let kps = detector.detect(&mat).unwrap_or_default();
    let scores: Vec<f32> = kps.iter().map(|kp| kp.response).collect();
    let out: Vec<KeyPoint> = kps
        .into_iter()
        .map(|kp| KeyPoint {
            x: kp.pt.x,
            y: kp.pt.y,
            response: kp.response,
        })
        .collect();
    (out, scores)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 构造黑色背景 + 单个白色方块角点的合成图。
    fn corner_img(cx: usize, cy: usize, w: usize, h: usize) -> GrayImage {
        let mut data = vec![0u8; w * h];
        for y in cy..(cy + 2).min(h) {
            for x in cx..(cx + 2).min(w) {
                data[y * w + x] = 255;
            }
        }
        GrayImage {
            width: w,
            height: h,
            data,
        }
    }

    #[test]
    fn fast_detects_corner_at_expected_position() {
        let (w, h) = (40, 40);
        let img = corner_img(20, 20, w, h);
        let kpts = fast(&img, 20, false);
        assert!(!kpts.is_empty(), "no corners detected");
        let found = kpts
            .iter()
            .any(|k| (k.x - 20.0).abs() <= 3.0 && (k.y - 20.0).abs() <= 3.0);
        assert!(found, "corner not at expected position; got {kpts:?}");
    }

    #[test]
    fn fast_nms_does_not_increase_count() {
        // NMS 不应增加点数（purecv 严格 > 比较：同分互相抑制，允许为 0）
        let (w, h) = (48, 48);
        let img = corner_img(16, 16, w, h);
        let with_nms = fast(&img, 20, true);
        let without_nms = fast(&img, 20, false);
        assert!(!without_nms.is_empty());
        assert!(
            with_nms.len() <= without_nms.len(),
            "NMS should not add points"
        );
    }

    #[test]
    fn fast_flat_image_no_corners() {
        let img = gray_const(100, 32, 32);
        let kpts = fast(&img, 20, true);
        assert!(kpts.is_empty());
    }

    #[test]
    fn fast_gradient_no_corners() {
        let w = 32;
        let h = 32;
        let mut data = vec![0u8; w * h];
        for y in 0..h {
            for x in 0..w {
                data[y * w + x] = x.midpoint(y).min(255) as u8;
            }
        }
        let img = GrayImage {
            width: w,
            height: h,
            data,
        };
        let kpts = fast(&img, 25, true);
        assert!(kpts.is_empty());
    }

    #[test]
    fn fast_threshold_affects_count() {
        let img = corner_img(20, 20, 40, 40);
        let strict = fast(&img, 120, false);
        let lenient = fast(&img, 20, false);
        assert!(strict.len() <= lenient.len());
    }

    #[test]
    fn fast_tiny_image_no_panic() {
        let img = gray_const(0, 3, 3);
        assert!(fast(&img, 10, true).is_empty());
    }

    fn gray_const(v: u8, w: usize, h: usize) -> GrayImage {
        GrayImage {
            width: w,
            height: h,
            data: vec![v; w * h],
        }
    }
}
