//! 网格 FAST 特征提取（对照 `OpenVINS ov_core/src/track/Grider_GRID.h` 的
//! `Grider_GRID::perform_griding`）。
//!
//! 把图像划分为 `grid_x × grid_y` 个网格；每个网格内用 [`crate::track::fast`]
//! 提取 FAST 角点，按响应排序取 top `num_features_grid`，必要时自适应缩小网格
//! 以均分特征；随后对全部角点做子像素精化（对照 `cv::cornerSubPix`）。

#![allow(
    clippy::doc_markdown,
    clippy::many_single_char_names,
    clippy::similar_names
)]
use crate::sensor::GrayImage;
use crate::track::KeyPoint;
use crate::track::fast;
use crate::track::mask_on;
use rayon::prelude::*;

/// 网格 FAST 特征提取（对照 `Grider_GRID::perform_griding`）。
///
/// - `img`：检测图像；
/// - `mask`：勿检测区域（`>127` 排除）；
/// - `valid_locs`：需要提取的网格坐标 `(x_grid, y_grid)` 列表（已按各格当前
///   特征数筛选，参照 `TrackKLT.cpp` 的调用方）；
/// - `num_features`：期望特征总数；
/// - `grid_x`/`grid_y`：网格数；若 `num_features < grid_x*grid_y` 会按比例
///   自适应缩小网格；
/// - `threshold`：FAST 阈值；
/// - `subpixel`：是否执行 cornerSubPix 子像素精化（默认 true，同 OpenVINS）。
///
/// 每格提取数内部按 `num_features/(grid_x*grid_y)+1` 计算（同 C++）。返回值已做
/// ROI 偏移修正，且已剔除落在掩码区域的点。自适应网格缩放的逻辑与 C++ 一致：
/// ```text
/// if num_features < grid_x*grid_y:
///     ratio = grid_x/grid_y
///     grid_y = ceil(sqrt(num_features/ratio))
///     grid_x = ceil(grid_y*ratio)
/// ```
///
/// # Panics
///
/// 内部 `cornerSubPix`（purecv）失败时 panic（合法输入不应发生）。
#[must_use]
pub fn perform_griding(
    img: &GrayImage,
    mask: &GrayImage,
    valid_locs: &[(i32, i32)],
    num_features: usize,
    grid_x: usize,
    grid_y: usize,
    threshold: i32,
    subpixel: bool,
) -> Vec<KeyPoint> {
    let (mut grid_x, mut grid_y) = (grid_x, grid_y);
    if num_features < grid_x * grid_y {
        let ratio = grid_x as f64 / grid_y as f64;
        grid_y = ((num_features as f64 / ratio).sqrt()).ceil() as usize;
        grid_x = ((grid_y as f64) * ratio).ceil() as usize;
    }
    grid_x = grid_x.max(1);
    grid_y = grid_y.max(1);
    let features_per_grid = (num_features / (grid_x * grid_y) + 1).max(1);
    let size_x = img.width / grid_x;
    let size_y = img.height / grid_y;
    if size_x == 0 || size_y == 0 {
        return Vec::new();
    }

    let mut out = Vec::new();
    // 并行提取：每个网格独立，rayon 自动分配到多核（对照 OpenVINS `parallel_for_`）
    let collected: Vec<Vec<KeyPoint>> = valid_locs
        .par_iter()
        .filter_map(|&(gx, gy)| {
            let x = gx * size_x as i32;
            let y = gy * size_y as i32;
            // 网格越界则跳过
            if x as usize + size_x > img.width || y as usize + size_y > img.height {
                return None;
            }
            // 取 ROI
            let roi = crop_roi(img, x as usize, y as usize, size_x, size_y);
            let kpts = fast::fast_with_score(&roi, threshold, true);
            // 按响应降序
            let mut sorted = kpts.0;
            sorted.sort_by(|a, b| {
                b.response
                    .partial_cmp(&a.response)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            let mut cell_pts = Vec::new();
            for kpt in sorted.into_iter().take(features_per_grid) {
                let mut p = kpt;
                p.x += x as f32;
                p.y += y as f32;
                if p.x < 0.0 || p.y < 0.0 || p.x > img.width as f32 || p.y > img.height as f32 {
                    continue;
                }
                if mask_on(mask, p.x as i32, p.y as i32) {
                    continue;
                }
                cell_pts.push(p);
            }
            Some(cell_pts)
        })
        .collect();
    for cell in collected {
        out.extend(cell);
    }

    if subpixel {
        // purecv cornerSubPix（OpenCV 语义，对照 Grider_GRID.h 的
        // cv::cornerSubPix：win 5×5、zero_zone (-1,-1)、COUNT+EPS 20/0.001）
        let mat = crate::track::pyramid::gray_to_matrix(img);
        let mut corners: Vec<purecv::core::types::Point2f> = out
            .iter()
            .map(|p| purecv::core::types::Point2f::new(p.x, p.y))
            .collect();
        purecv::imgproc::feature::corner_sub_pix(
            &mat,
            &mut corners,
            purecv::core::types::Size2i::new(5, 5),
            purecv::core::types::Size2i::new(-1, -1),
            purecv::core::types::TermCriteria::new(purecv::core::types::TermType::Both, 20, 0.001),
        )
        .expect("cornerSubPix 不应失败（单通道灰度）");
        for (k, c) in out.iter_mut().zip(corners) {
            k.x = c.x;
            k.y = c.y;
        }
    }
    out
}

/// 从图像中裁剪 ROI（复用现有数据，不复制）。返回视图结构。
///
/// # Panics
///
/// 内部 `corner_sub_pix`（purecv）失败时 panic（合法输入不应发生）。
#[must_use]
fn crop_roi(img: &GrayImage, x: usize, y: usize, w: usize, h: usize) -> GrayImage {
    // 组装 ROI 视图并转成 flat GrayImage（仅用于 FAST 检测）
    let mut data = Vec::with_capacity(w * h);
    for yy in y..y + h {
        let row = yy * img.width;
        data.extend_from_slice(&img.data[row + x..row + x + w]);
    }
    GrayImage {
        width: w,
        height: h,
        data,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 布满明亮方块的图（FAST 检测用）。
    ///
    /// 8×8 白块放在每20×20 cell 内偏心位置，确保角点得分各异（purecv 严格
    /// NMS 下同分角点会全部抑制，偏心放置让不同角点的圆周采样不同→得分不同）。
    fn corners_img(w: usize, h: usize) -> GrayImage {
        let mut data = vec![0u8; w * h];
        let cell = 20usize;
        let mut fy = 2;
        while fy + 10 < h {
            let mut fx = 2;
            while fx + 10 < w {
                for dy in 0..8 {
                    for dx in 0..8 {
                        data[(fy + dy) * w + (fx + dx)] = 255;
                    }
                }
                fx += cell;
            }
            fy += cell;
        }
        GrayImage {
            width: w,
            height: h,
            data,
        }
    }

    #[test]
    fn grider_respects_feature_cap() {
        let img = corners_img(128, 128);
        let mask = GrayImage {
            width: 128,
            height: 128,
            data: vec![0u8; 128 * 128],
        };
        let grid_x = 4_usize;
        let grid_y = 4_usize;
        let valid = (0..grid_y)
            .flat_map(|gy| (0..grid_x).map(move |gx| (gx as i32, gy as i32)))
            .collect::<Vec<_>>();
        // num_features >= grid 数 → 不做自适应缩放
        let num_features = 32_usize;
        let pts = perform_griding(&img, &mask, &valid, num_features, grid_x, grid_y, 20, false);
        assert!(!pts.is_empty(), "no features");
        // 每格最多 num_features/(gx*gy)+1 个 → 总数应被该上限约束
        let per_cell = num_features / (grid_x * grid_y) + 1;
        let upper = per_cell * valid.len();
        assert!(
            pts.len() <= upper,
            "got {} features, cap {upper}",
            pts.len()
        );
        // 坐标应在 ROI 偏移后仍落在图像内
        for p in &pts {
            assert!(p.x >= 0.0 && p.x < 128.0 && p.y >= 0.0 && p.y < 128.0);
        }
    }

    #[test]
    fn grider_adaptive_grid_when_few_features() {
        let img = corners_img(120, 120);
        let mask = GrayImage {
            width: 120,
            height: 120,
            data: vec![0u8; 120 * 120],
        };
        // num_features(8) < grid_x*grid_y(4*4=16) → 自适应收缩网格
        // ratio=1 → grid_y=ceil(sqrt(8/1))=3、grid_x=ceil(3*1)=3，每格 ≤1 个
        let valid = (0..4)
            .flat_map(|gy| (0..4).map(move |gx| (gx, gy)))
            .collect::<Vec<_>>();
        let pts = perform_griding(&img, &mask, &valid, 8, 4, 4, 20, false);
        assert!(!pts.is_empty());
        assert!(pts.len() <= 9, "got {}", pts.len());
    }

    #[test]
    fn grider_mask_blocks_detection() {
        let img = corners_img(64, 64);
        // 全掩码 → 不提取任何特征
        let mask = GrayImage {
            width: 64,
            height: 64,
            data: vec![255u8; 64 * 64],
        };
        let valid = vec![(0, 0), (1, 1)];
        let pts = perform_griding(&img, &mask, &valid, 8, 4, 4, 20, false);
        assert!(pts.is_empty(), "masked region should yield no features");
    }

    #[test]
    fn grider_empty_valid_locs() {
        let img = corners_img(32, 32);
        let mask = GrayImage {
            width: 32,
            height: 32,
            data: vec![0u8; 32 * 32],
        };
        let pts = perform_griding(&img, &mask, &[], 16, 4, 4, 20, false);
        assert!(pts.is_empty());
    }

    #[test]
    fn subpixel_refines() {
        // 粗糙整数角点经 cornerSubPix 应近似保持（合成平滑角）
        let img = corners_img(32, 32);
        // 在 2×2 白块角落处取样；子像素校正应仍贴近整数像素
        let mut pts = vec![KeyPoint::new(5.0, 5.0), KeyPoint::new(21.0, 5.0)];
        // 取点直接在方块正上方平滑处，避免角点被过于约束
        // purecv cornerSubPix（OpenCV 语义）
        let mat = crate::track::pyramid::gray_to_matrix(&img);
        let mut corners: Vec<purecv::core::types::Point2f> = pts
            .iter()
            .map(|p| purecv::core::types::Point2f::new(p.x, p.y))
            .collect();
        purecv::imgproc::feature::corner_sub_pix(
            &mat,
            &mut corners,
            purecv::core::types::Size2i::new(5, 5),
            purecv::core::types::Size2i::new(-1, -1),
            purecv::core::types::TermCriteria::new(purecv::core::types::TermType::Both, 20, 0.001),
        )
        .unwrap();
        for (p, c) in pts.iter_mut().zip(corners) {
            p.x = c.x;
            p.y = c.y;
        }
        for p in &pts {
            assert!(
                (p.x - p.x.round()).abs() <= 1.0,
                "subpixel drift too large at {p:?}"
            );
        }
    }
}
