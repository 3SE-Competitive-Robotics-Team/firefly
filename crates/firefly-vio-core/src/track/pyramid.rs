//! 光流图像金字塔（自实现，对照 `OpenCV modules/video/src/lkpyramid.cpp` 的
//! `buildOpticalFlowPyramid`）。
//!
//! 金字塔等级：level 0 为输入图；每一级由上一级做 5×5 高斯模糊后隔行降采样
//! 得到（尺寸约为上一级的一半）。OpenCV `buildOpticalFlowPyramid` 的金字塔
//! 高度受 `winSize` 约束（保证最顶层窗口仍落在图像内），且每级随
//! [`build_optical_flow_pyramid`] 的 `max_levels` 上限封顶。

#![allow(
    clippy::doc_markdown,
    clippy::many_single_char_names,
    clippy::similar_names
)]
use crate::sensor::GrayImage;

/// 5×5 高斯核（`[1,4,6,4,1]/16`，分离核，与 OpenCV `pyrDown` 一致）。
const GAUSS: [f64; 5] = [1.0, 4.0, 6.0, 4.0, 1.0];
const GAUSS_NORM: f64 = 16.0;

/// 高斯模糊：对 `src` 做 5×5 高斯卷积（镜像边界，同 OpenCV `BORDER_DEFAULT`）。
///
/// 分离实现：先横向再纵向；边界越界处镜像采样（反射），保证奇偶均为 5 点。
#[must_use]
fn gaussian_5x5(src: &GrayImage) -> GrayImage {
    let (w, h) = (src.width, src.height);
    let idx = |v: i64, n: usize| -> usize {
        // 镜像反射索引（OpenCV BORDER_REFLECT_101 近似；处理 5 点时用反射）
        let n_i = n as i64;
        let mut i = v;
        let wrap = 2 * n_i - 2;
        if wrap == 0 {
            return 0;
        }
        loop {
            if i >= 0 && i < n_i {
                break;
            }
            i = if i < 0 { -i } else { wrap - i };
        }
        i as usize
    };

    // 横向
    let mut tmp = vec![0f32; w * h];
    for y in 0..h {
        let row = y * w;
        for x in 0..w {
            let mut acc = 0.0f32;
            for (k, &gk) in GAUSS.iter().enumerate() {
                let off = x as i64 + k as i64 - 2;
                let xi = idx(off, w);
                acc += f64::from(src.data[row + xi]) as f32 * gk as f32;
            }
            tmp[row + x] = acc / GAUSS_NORM as f32;
        }
    }

    // 纵向
    let mut out = vec![0u8; w * h];
    for y in 0..h {
        for x in 0..w {
            let mut acc = 0.0f32;
            for (k, &gk) in GAUSS.iter().enumerate() {
                let off = y as i64 + k as i64 - 2;
                let yi = idx(off, h);
                acc += tmp[yi * w + x] * gk as f32;
            }
            out[y * w + x] = (acc / GAUSS_NORM as f32).round().clamp(0.0, 255.0) as u8;
        }
    }
    GrayImage {
        width: w,
        height: h,
        data: out,
    }
}

/// 隔行降采样：取高斯模糊后图像的偶数行偶数列像素，尺寸 `(w/2, h/2)`。
///
/// 对照 OpenCV `pyrDown` 的行为（GaussianBlur + 隔点采样）。OpenVINS 的
/// `buildOpticalFlowPyramid` 内部即 `pyrDown`。
#[must_use]
fn downsample2(blurred: &GrayImage) -> GrayImage {
    let nw = blurred.width / 2;
    let nh = blurred.height / 2;
    let mut data = Vec::with_capacity(nw * nh);
    for y in 0..nh {
        for x in 0..nw {
            data.push(blurred.data[(2 * y) * blurred.width + 2 * x]);
        }
    }
    GrayImage {
        width: nw,
        height: nh,
        data,
    }
}

/// 构建光流金字塔（对照 `cv::buildOpticalFlowPyramid` 的数值语义）。
///
/// - 返回值第 0 级为输入图（未经处理）；
/// - 第 `k` 级 = `pyramid[k-1]` 的 5×5 高斯模糊 + 隔行降采样；
/// - 最多 `max_levels` 级；且每级最小边长不小于 `min_side`（保证 LK 窗口在
///   最顶层仍能完整取值，对应 OpenCV 用 `winSize` 上限金字塔高度）。
///
/// `min_side` 需 ≥ LK 窗口边长 + 导数边界（TrackKLT 的窗口 15×15 → 取 22）。
#[must_use]
pub fn build_optical_flow_pyramid(
    img: &GrayImage,
    max_levels: usize,
    min_side: usize,
) -> Vec<GrayImage> {
    let mut pyramid = Vec::with_capacity(max_levels);
    pyramid.push(img.clone());
    for _ in 1..max_levels {
        let prev = &pyramid[pyramid.len() - 1];
        // 下一级边长不足最小阈值，则不再生成更高层
        if prev.width < 2 * min_side || prev.height < 2 * min_side {
            break;
        }
        let blurred = gaussian_5x5(prev);
        let down = downsample2(&blurred);
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
        // min_side=3，7×7 → [7,3]，不再降采样到 1
        assert!(pyr.last().unwrap().width >= 3);
        assert_eq!(pyr.len(), 2);
    }

    #[test]
    fn pyramid_respects_lk_window_cap() {
        // min_side 为大时（LK 窗口 15×15 → 22），128×128 顶部只到 64/32
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
    fn gaussian_5x5_preserves_flat_region() {
        let img = gray_const(80u8, 20, 20);
        let blurred = gaussian_5x5(&img);
        // 平坦区域高斯模糊不应改变像素值（核和为 1）
        assert!(blurred.data.iter().all(|&p| p == 80));
    }

    fn gray_const(v: u8, w: usize, h: usize) -> GrayImage {
        GrayImage {
            width: w,
            height: h,
            data: vec![v; w * h],
        }
    }
}
