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
use crate::track::pyramid::gray_to_matrix;
use purecv::features2d::{FastFeatureDetector, FastType};

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
mod purecv_tests {
    use super::*;

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
    fn purecv_fast_with_score_detects_corner() {
        // 2x2 白块（自研 fast() 已知能检出）
        let img = corner_img(20, 20, 40, 40);
        let (kps, _) = fast_with_score(&img, 20, false);
        assert!(!kps.is_empty(), "purecv FAST 对 2x2 白块检出 0");
    }

    #[test]
    fn purecv_fast_with_score_detects_blob() {
        // 3x3 亮点（150）在黑背景（0）上：圆周 16 点全暗 → 应检出
        let (w, h) = (64, 64);
        let mut data = vec![0u8; w * h];
        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                data[((20 + dy) as usize) * w + (20 + dx) as usize] = 150;
            }
        }
        let img = GrayImage {
            width: w,
            height: h,
            data,
        };
        let (kps, _) = fast_with_score(&img, 10, false);
        assert!(!kps.is_empty(), "purecv FAST 对 3x3 亮点检出 0: {kps:?}");
    }

    #[test]
    fn purecv_fast_with_score_blob_on_checker() {
        // 3x3 亮点（150）在棋盘背景（60/100）上：圆周 16 点 60/100 < 140
        // → 应检出（合成测试场景）
        let (w, h) = (64, 64);
        let mut data = vec![0u8; w * h];
        for y in 0..h {
            for x in 0..w {
                data[y * w + x] = if ((x / 16) + (y / 16)) % 2 == 0 {
                    60
                } else {
                    100
                };
            }
        }
        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                data[((20 + dy) as usize) * w + (20 + dx) as usize] = 150;
            }
        }
        let img = GrayImage {
            width: w,
            height: h,
            data,
        };
        let (kps, _) = fast_with_score(&img, 10, false);
        println!(
            "棋盘亮点 NMS关: {} 个, response={:?}",
            kps.len(),
            kps.iter().map(|k| k.response).take(5).collect::<Vec<_>>()
        );
        assert!(!kps.is_empty(), "purecv FAST 棋盘上亮点检出 0: {kps:?}");
        // NMS 开：平台斑（3x3 同 level）响应平坦，"严格大于"8 邻域 → 全抑制
        // （OpenCV 同行为，数学事实）——合成测试因此用高斯斑（单峰）
        let (kps_nms, _) = fast_with_score(&img, 10, true);
        println!("棋盘亮点 NMS开: {} 个", kps_nms.len());
        assert!(kps_nms.is_empty(), "平台斑 NMS 应全抑制（响应平坦）");
    }
}
