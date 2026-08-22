//! 光流图像金字塔（基于 purecv 的 SIMD 加速 `pyr_down`）。
//!
//! 金字塔等级：level 0 为输入图；每一级由上一级做 OpenCV 5×5 高斯核
//! `[1,4,6,4,1]/256` 分离卷积 + 隔行降采样得到（尺寸约为上一级的一半）。
//! OpenCV `buildOpticalFlowPyramid` 的金字塔高度受 `winSize` 约束，
//! 且每级随 `max_levels` 上限封顶。

use crate::sensor::GrayImage;
use purecv::core::Matrix;
use purecv::core::types::BorderTypes;

/// 将 `GrayImage` 转为 purecv `Matrix<u8>`（单通道）。
fn gray_to_matrix(img: &GrayImage) -> Matrix<u8> {
    Matrix::from_vec(img.height, img.width, 1, img.data.clone())
}

/// 将 purecv `Matrix<u8>` 转回 `GrayImage`。
fn matrix_to_gray(m: &Matrix<u8>) -> GrayImage {
    GrayImage {
        width: m.cols,
        height: m.rows,
        data: m.data.clone(),
    }
}

/// 构建光流金字塔（对照 `cv::buildOpticalFlowPyramid` 的数值语义）。
///
/// - 返回值第 0 级为输入图（未经处理）；
/// - 第 `k` 级 = `pyramid[k-1]` 的 OpenCV 5×5 高斯模糊 + 隔行降采样
///   （purecv `pyr_down`，NEON/SSE SIMD 加速）；
/// - 最多 `max_levels` 级；且每级最小边长不小于 `min_side`（保证 LK 窗口在
///   最顶层仍能完整取值，对应 OpenCV 用 `winSize` 上限金字塔高度）。
///
/// `min_side` 需 ≥ LK 窗口边长 + 导数边界（TrackKLT 的窗口 15×15 → 取 22）。
///
/// # Panics
/// 内部 `pyr_down` 失败时 panic（合法输入不应发生，对照 purecv 契约）。
#[must_use]
pub fn build_optical_flow_pyramid(
    img: &GrayImage,
    max_levels: usize,
    min_side: usize,
) -> Vec<GrayImage> {
    let mut pyramid = Vec::with_capacity(max_levels);
    pyramid.push(img.clone());

    // purecv pyr_down：SIMD 加速 [1,4,6,4,1] 分离卷积 + 降采样
    // BorderTypes::Reflect101 = OpenCV BORDER_DEFAULT（不含边缘复制）
    for _ in 1..max_levels {
        let prev = &pyramid[pyramid.len() - 1];
        // 下一级边长不足最小阈值，则不再生成更高层
        if prev.width < 2 * min_side || prev.height < 2 * min_side {
            break;
        }
        let mat = gray_to_matrix(prev);
        let down_mat = purecv::imgproc::pyramid::pyr_down(&mat, None, BorderTypes::Reflect101)
            .expect("pyr_down should not fail for valid input");
        let down = matrix_to_gray(&down_mat);
        if down.width < min_side || down.height < min_side {
            break;
        }
        pyramid.push(down);
    }
    pyramid
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pyramid_starts_with_input() {
        let img = GrayImage {
            width: 64,
            height: 64,
            data: vec![0u8; 64 * 64],
        };
        let pyr = build_optical_flow_pyramid(&img, 5, 3);
        assert_eq!(pyr.len(), 5);
        assert_eq!(pyr[0].width, 64);
        assert_eq!(pyr[0].height, 64);
    }

    #[test]
    fn pyramid_downsamples_by_two() {
        let mut data = vec![0u8; 128 * 128];
        for y in 0..128usize {
            for x in 0..128usize {
                data[y * 128 + x] = ((x + y) % 256) as u8;
            }
        }
        let img = GrayImage {
            width: 128,
            height: 128,
            data,
        };
        let pyr = build_optical_flow_pyramid(&img, 5, 3);
        assert_eq!(pyr[1].width, 64);
        assert_eq!(pyr[1].height, 64);
        assert_eq!(pyr[2].width, 32);
        assert_eq!(pyr[4].width, 8);
    }

    #[test]
    fn pyramid_stops_at_min_size() {
        let img = GrayImage {
            width: 7,
            height: 7,
            data: vec![0u8; 49],
        };
        let pyr = build_optical_flow_pyramid(&img, 5, 3);
        assert!(pyr.last().unwrap().width >= 3);
        assert_eq!(pyr.len(), 2);
    }

    #[test]
    fn pyramid_respects_lk_window_cap() {
        let img = GrayImage {
            width: 128,
            height: 128,
            data: vec![0u8; 128 * 128],
        };
        let pyr = build_optical_flow_pyramid(&img, 6, 22);
        assert!(!pyr.is_empty());
        assert!(
            pyr.last().unwrap().width >= 22,
            "top level {} < 22",
            pyr.last().unwrap().width
        );
    }

    #[test]
    fn empty_image_no_panic() {
        let img = GrayImage {
            width: 0,
            height: 0,
            data: vec![],
        };
        let pyr = build_optical_flow_pyramid(&img, 5, 3);
        assert_eq!(pyr.len(), 1);
        assert_eq!(pyr[0].width, 0);
    }

    #[test]
    fn pyr_down_preserves_flat_region() {
        // 平坦区域 pyr_down 不应改变像素值（核和为 256，除后不变）
        let img = gray_const(80u8, 20, 20);
        let mat = gray_to_matrix(&img);
        let down = purecv::imgproc::pyramid::pyr_down(&mat, None, BorderTypes::Reflect101).unwrap();
        let down_gray = matrix_to_gray(&down);
        assert!(down_gray.data.iter().all(|&p| p == 80));
    }

    fn gray_const(v: u8, w: usize, h: usize) -> GrayImage {
        GrayImage {
            width: w,
            height: h,
            data: vec![v; w * h],
        }
    }
}
