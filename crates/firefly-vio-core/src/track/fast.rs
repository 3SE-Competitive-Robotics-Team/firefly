//! FAST-9 角点检测（purecv `FastFeatureDetector`）。
//! 角点得分存于 `KeyPoint::response`；NMS 要求得分严格大于八邻域。

use crate::track::KeyPoint;
use purecv::core::Matrix;
use purecv::features2d::{FastFeatureDetector, FastType};

/// 检测单通道矩阵中的角点；对齐 OpenCV CPU FAST，关闭 NMS 时响应值为零。
#[must_use]
pub fn detect(img: &Matrix<u8>, threshold: i32, nonmax_suppression: bool) -> Vec<KeyPoint> {
    let detector = FastFeatureDetector::new(
        threshold.clamp(0, 255) as u8,
        nonmax_suppression,
        FastType::Type9_16,
    );
    detector
        .detect(img)
        .unwrap_or_default()
        .into_iter()
        .map(|kp| KeyPoint {
            x: kp.pt.x,
            y: kp.pt.y,
            // 仅在启用 NMS 时使用响应值；未启用时 purecv 不提供有效分数。
            response: if nonmax_suppression { kp.response } else { 0.0 },
        })
        .collect()
}

#[cfg(test)]
mod purecv_tests {
    use super::*;
    use crate::sensor::GrayImage;
    use crate::track::pyramid::gray_to_matrix;

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
    fn isolated_corner_has_analytic_fast_response() {
        let mut img = GrayImage {
            width: 15,
            height: 15,
            data: vec![0; 15 * 15],
        };
        img.data[7 * 15 + 7] = 150;
        let result = detect(&gray_to_matrix(&img), 20, true);
        assert_eq!(result.len(), 1);
        assert_eq!(
            (result[0].x, result[0].y, result[0].response),
            (7., 7., 149.)
        );
        assert_eq!(
            detect(&gray_to_matrix(&img), 150, true),
            [] as [KeyPoint; 0]
        );
    }

    #[test]
    fn purecv_fast_detects_corner() {
        // 2x2 白块在关闭 NMS 时包含多个角点。
        let img = corner_img(20, 20, 40, 40);
        let kps = detect(&gray_to_matrix(&img), 20, false);
        assert!(!kps.is_empty(), "purecv FAST 对 2x2 白块检出 0");
        assert!(kps.iter().all(|kp| kp.response == 0.0));
    }

    #[test]
    fn purecv_fast_detects_blob() {
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
        let kps = detect(&gray_to_matrix(&img), 10, false);
        assert!(!kps.is_empty(), "purecv FAST 对 3x3 亮点检出 0: {kps:?}");
    }

    #[test]
    fn purecv_fast_blob_on_checker() {
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
        let kps = detect(&gray_to_matrix(&img), 10, false);
        assert!(!kps.is_empty(), "purecv FAST 棋盘上亮点检出 0: {kps:?}");
        // NMS 开：平台斑（3x3 同 level）响应平坦，"严格大于"8 邻域 → 全抑制
        // （OpenCV 同行为，数学事实）——合成测试因此用高斯斑（单峰）
        let kps_nms = detect(&gray_to_matrix(&img), 10, true);
        assert!(kps_nms.is_empty(), "平台斑 NMS 应全抑制（响应平坦）");
    }
}
