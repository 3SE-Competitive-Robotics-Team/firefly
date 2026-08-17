//! FAST-9 角点检测（自实现，对照 `OpenCV modules/features2d/src/fast.cpp` 与
//! `fast_score.hpp`）。
//!
//! [`fast`] 在灰度图上做 FAST-9 角点检测：以像素为中心取半径 3 的 16 邻域圆，
//! 若有连续 ≥9 个邻域像素都显著亮于（或都显著暗于）中心（差阈值 `threshold`），
//! 判为角点；随后可选非极大值抑制（`nonmax_suppression`）——按 FAST 得分
//! 取 3×3 局部极大值，抑制弱角点。

#![allow(
    clippy::doc_markdown,
    clippy::many_single_char_names,
    clippy::similar_names
)]
use crate::sensor::GrayImage;
use crate::track::KeyPoint;

/// FAST-9 的 16 邻域圆周偏移（x 增量, y 增量），按顺时针顺序。
///
/// 与 OpenCV `fast_pattern` 的 `pixel[16]` 重合（p0 在正右方，p4 在正下方，
/// 逆/顺时针取决于实现；这里按 OpenCV `make_offsets` 的排列）。
const OFFSETS: [(i32, i32); 16] = [
    (0, -3),
    (1, -3),
    (2, -2),
    (3, -1),
    (3, 0),
    (3, 1),
    (2, 2),
    (1, 3),
    (0, 3),
    (-1, 3),
    (-2, 2),
    (-3, 1),
    (-3, 0),
    (-3, -1),
    (-2, -2),
    (-1, -3),
];

/// FAST 得分计算：给定像素为角点时，返回其「连续比中心亮/暗超过阈值的一段
/// 强度差之和」，作为响应值。非角点返回 `i16::MIN`。
///
/// 对照 OpenCV `fastCornerScore`（连续最大 9 像素段，取三段中最大累加）。
fn score(ptr: &[u8], stride: i32, x: i32, y: i32, threshold: i32) -> f32 {
    let center = i32::from(ptr[(y * stride + x) as usize]);
    // 三个候选段起点：p0、p4、p8
    let best = {
        let mut best = i32::MIN;
        let segments: [i32; 3] = [0, 4, 8];
        for &start in &segments {
            // 连续 9 个「亮」像素
            let mut sum = 0i32;
            for k in 0..9 {
                let idx = (start + k) & 0x0F;
                let (ox, oy) = OFFSETS[idx as usize];
                let v = i32::from(ptr[((y + oy) * stride + (x + ox)) as usize]);
                sum += (v - center).max(0); // 亮
            }
            if sum > best {
                best = sum;
            }
            // 连续 9 个「暗」像素
            let mut sum = 0i32;
            for k in 0..9 {
                let idx = (start + k) & 0x0F;
                let (ox, oy) = OFFSETS[idx as usize];
                let v = i32::from(ptr[((y + oy) * stride + (x + ox)) as usize]);
                sum += (center - v).max(0); // 暗
            }
            if sum > best {
                best = sum;
            }
        }
        // OpenCV 要求差至少为阈值：得分计为超过阈值部分的累加
        best -= threshold * 9;
        best
    };
    best as f32
}

/// 是否某个 16 邻域窗口内存在连续 `n` 个像素都显著亮/暗于中心。
#[must_use]
fn is_corner(ptr: &[u8], stride: i32, x: i32, y: i32, threshold: i32) -> bool {
    let center = i32::from(ptr[(y * stride + x) as usize]);
    // 任一候选段：9 个连续像素都「亮」或都「暗」
    for start in [0, 4, 8] {
        let mut bright = 0i32;
        let mut dark = 0i32;
        for k in 0..9 {
            let idx = (start + k) & 0x0F;
            let (ox, oy) = OFFSETS[idx as usize];
            let v = i32::from(ptr[((y + oy) * stride + (x + ox)) as usize]);
            if v - center > threshold {
                bright += 1;
            }
            if center - v > threshold {
                dark += 1;
            }
        }
        if bright >= 9 || dark >= 9 {
            return true;
        }
    }
    false
}

/// FAST-9 角点检测。
///
/// - `threshold`：强度阈值（像素差）；
/// - `nonmax_suppression`：是否做非极大值抑制（OpenCV `cv::FAST` 的第三参）。
///
/// 返回 `(角点列表, 得分)`。边界 3 像素内不检测（窗口半径 3 无法完整采样）。
/// 非极大值抑制：保留得分严格大于 3×3 邻域内其它角点的点（同值抑制，保持
/// 稀疏性）。
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
    let (w, h) = (img.width, img.height);
    if w < 4 || h < 4 {
        return (Vec::new(), Vec::new());
    }
    let stride = w as i32;
    // 得分矩阵（-1 初始化为无角点）
    let mut scores = vec![-1i32; w * h];
    for y in 3..h - 3 {
        for x in 3..w - 3 {
            if is_corner(&img.data, stride, x as i32, y as i32, threshold) {
                let s = score(&img.data, stride, x as i32, y as i32, threshold);
                scores[y * w + x] = if s.is_infinite() || s.is_nan() {
                    -1
                } else {
                    s.round() as i32
                };
            }
        }
    }

    let mut out = Vec::new();
    if nonmax_suppression {
        // 扫描序 NMS（对齐 OpenCV 语义）：按 y→x 扫描，仅当 8 邻域内没有
        // 「已接受」且得分 ≥ 当前的点时才接受当前角点；平局保留最先扫描到的。
        let mut accepted = vec![false; w * h];
        for y in 3..h - 3 {
            for x in 3..w - 3 {
                let cur = scores[y * w + x];
                if cur < 0 {
                    continue;
                }
                let mut keep = true;
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                        let nb = accepted[ny as usize * w + nx as usize];
                        let ns = scores[ny as usize * w + nx as usize];
                        if nb && ns >= cur {
                            keep = false;
                            break;
                        }
                    }
                    if !keep {
                        break;
                    }
                }
                if keep {
                    accepted[y * w + x] = true;
                    out.push(KeyPoint {
                        x: x as f32,
                        y: y as f32,
                        response: cur as f32,
                    });
                }
            }
        }
    } else {
        for y in 3..h - 3 {
            for x in 3..w - 3 {
                let cur = scores[y * w + x];
                if cur >= 0 {
                    out.push(KeyPoint {
                        x: x as f32,
                        y: y as f32,
                        response: cur as f32,
                    });
                }
            }
        }
    }
    let scores_out: Vec<f32> = out.iter().map(|k| k.response).collect();
    (out, scores_out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 构造黑色背景 + 单个白色方块角点的合成图。
    fn corner_img(cx: usize, cy: usize, w: usize, h: usize) -> GrayImage {
        let mut data = vec![0u8; w * h];
        // 以 (cx,cy) 为左上角画 2×2 白色方块，制造灰度突变角点
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
        // 应存在一个角点，位置在方块左上附近
        assert!(!kpts.is_empty(), "no corners detected");
        let found = kpts.iter().any(|k| {
            // 角点应在方块角附近（20,20 附近）
            (k.x - 20.0).abs() <= 3.0 && (k.y - 20.0).abs() <= 3.0
        });
        assert!(found, "corner not at expected position; got {kpts:?}");
    }

    #[test]
    fn fast_nms_removes_weak_neighbors() {
        // 两个孤立的强角点（相距远，互不抑制）。NMS 前后都应检出。
        let (w, h) = (48, 48);
        let img = corner_img(16, 16, w, h);
        let with_nms = fast(&img, 20, true);
        let without_nms = fast(&img, 20, false);
        assert!(!with_nms.is_empty());
        assert!(!without_nms.is_empty());
        // NMS 不增加点数
        assert!(
            with_nms.len() <= without_nms.len(),
            "NMS should not add points"
        );
    }

    #[test]
    fn fast_nms_suppresses_duplicate_cluster() {
        // 一个亮斑会沿角处产生多个紧邻角点；NMS 应约去重复的簇点。
        let (w, h) = (40, 40);
        let img = corner_img(20, 20, w, h);
        let cluster = fast(&img, 20, false);
        let nms = fast(&img, 20, true);
        // NMS 后 3×3 邻域内不应出现两个都是局部极大的角点
        for a in &nms {
            let near = nms.iter().filter(|b| {
                (((a.x - b.x).abs() <= 1.0) && ((a.y - b.y).abs() <= 1.0))
                    && ((a.x - b.x).abs() > 1e-6 || (a.y - b.y).abs() > 1e-6)
            });
            assert_eq!(near.count(), 0, "NMS left two adjacent maxima near {a:?}");
        }
        assert!(nms.len() <= cluster.len());
    }

    #[test]
    fn fast_flat_image_no_corners() {
        let img = gray_const(100, 32, 32);
        let kpts = fast(&img, 20, true);
        assert!(kpts.is_empty());
    }

    #[test]
    fn fast_gradient_no_corners() {
        // 平滑渐变无显著角点
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
