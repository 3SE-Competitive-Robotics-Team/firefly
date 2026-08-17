//! 图像直方图增强（自实现，对照 `OpenCV modules/imgproc/src/clahe.cpp` 与
//! `cv::equalizeHist`）。
//!
//! - [`equalize_hist`]：全局直方图均衡（`cv::equalizeHist`）；在 8 位灰度图上构建
//!   全局灰度直方图，累计分布做映射。
//! - [`clahe`]：对比度受限自适应直方图均衡（`cv::CLAHE::apply`）；把图像分成
//!   `tiles_x × tiles_y` 个瓦片，每个瓦片做裁剪限制 + 直方图均衡的 LUT，再按
//!   像素位置做双线性插值合成（边界瓦片退化为就近瓦片）。

#![allow(
    clippy::doc_markdown,
    clippy::many_single_char_names,
    clippy::similar_names
)]
use crate::sensor::GrayImage;

/// 灰度级数（8 位）。
const HIST_SIZE: usize = 256;

/// 全局直方图均衡（对照 `cv::equalizeHist`）。
///
/// 1. 统计全局灰度直方图 `hist[0..256]`；
/// 2. 累计分布 `cumsum`：
///    ```text
///    scale = 255 / (width × height)
///    lut[g] = (g == 0) ? round(cumsum(0)·scale)
///                       : round(cumsum(g)·scale)
///    ```
/// 3. 逐像素查 LUT 映射输出。
///
/// 按 OpenCV 语义，第 0 级（灰度为 0）直接映射为 0，其余按 `round(cdf·scale)`
/// （四舍五入），与 C++ `equalizeHist` 的取整一致。
#[must_use]
pub fn equalize_hist(img: &GrayImage) -> GrayImage {
    let n = img.width * img.height;
    let mut hist = [0u64; HIST_SIZE];
    for &px in &img.data {
        hist[px as usize] += 1;
    }

    let scale = 255.0 / n as f64;
    let mut lut = [0u8; HIST_SIZE];
    let mut cumsum = 0u64;
    for g in 0..HIST_SIZE {
        cumsum += hist[g];
        if g == 0 {
            lut[g] = 0;
        } else {
            lut[g] = (cumsum as f64 * scale).round() as u8;
        }
    }

    let data = img.data.iter().map(|&px| lut[px as usize]).collect();
    GrayImage {
        width: img.width,
        height: img.height,
        data,
    }
}

/// CLAHE 参数（对照 `cv::createCLAHE` + `CLAHE::apply`）。
#[derive(Debug, Clone, Copy)]
pub struct ClaheParams {
    /// 裁剪限制（`clipLimit`），相对每个瓦片平均直方图 bin 数的倍数。
    pub clip_limit: f64,
    /// 网格在 x 方向（水平）上的瓦片数（`tilesGridSize.width`）。
    pub tiles_x: usize,
    /// 网格在 y 方向（垂直）上的瓦片数（`tilesGridSize.height`）。
    pub tiles_y: usize,
}

impl Default for ClaheParams {
    fn default() -> Self {
        Self {
            clip_limit: 10.0,
            tiles_x: 8,
            tiles_y: 8,
        }
    }
}

/// CLAHE 自适应直方图均衡（对照 `OpenCV clahe.cpp`）。
///
/// 算法：
/// 1. 每个瓦片尺寸 `tile_w = ceil(cols/tiles_x)`、`tile_h = ceil(rows/tiles_y)`；
/// 2. 裁剪限制（`clipLimit_ = clip_limit * tile_area / 256`）：
///    每个瓦片统计直方图后，把超过限制的 bin 计数截到限制值，并把超出总量
///    平均摊回所有 bin（余数逐个 +1），保证总面积守恒；
/// 3. 对每个瓦片按裁剪后的直方图做 CDF 映射生成 256 长的 LUT：
///    `lut[g] = round(255 · cdf(g) / tile_area)`；
/// 4. 输出：逐像素找所在瓦片 `(ty, tx)` 与瓦片内归一化位置，用相邻 2×2 瓦片
///    的 LUT 做双线性插值；边界瓦片回退到最近瓦片（`clamp` 语义同 OpenCV）。
///
/// 8×8 网格 + `clip_limit=10.0` 即 `TrackKLT.cpp` 第 61-63 行所用值。
#[must_use]
// 与 OpenCV CLAHE 1:1 移植的长流程函数，拆分会破坏对照可审计性。
#[allow(clippy::too_many_lines)]
pub fn clahe(img: &GrayImage, params: &ClaheParams) -> GrayImage {
    let (cols, rows) = (img.width, img.height);
    let tile_w = cols.div_ceil(params.tiles_x);
    let tile_h = rows.div_ceil(params.tiles_y);
    let tile_area = (tile_w * tile_h) as f64;

    let clip_limit = if params.clip_limit > 0.0 {
        params.clip_limit * tile_area / HIST_SIZE as f64
    } else {
        0.0
    };
    let clip_value = if params.clip_limit > 0.0 {
        clip_limit.ceil()
    } else {
        0.0
    };

    // 每个瓦片生成 LUT[256]
    let mut luts: Vec<[u8; HIST_SIZE]> = Vec::with_capacity(params.tiles_x * params.tiles_y);
    for ty in 0..params.tiles_y {
        for tx in 0..params.tiles_x {
            let x0 = tx * tile_w;
            let y0 = ty * tile_h;
            let x1 = (x0 + tile_w).min(cols);
            let y1 = (y0 + tile_h).min(rows);
            if x1 <= x0 || y1 <= y0 {
                // 瓦片完全落在图像外：置恒等 LUT，保证后续插值安全。
                let mut lut = [0u8; HIST_SIZE];
                for (g, slot) in lut.iter_mut().enumerate() {
                    *slot = g as u8;
                }
                luts.push(lut);
                continue;
            }

            let mut hist = [0u64; HIST_SIZE];
            for y in y0..y1 {
                let row = y * cols;
                for x in x0..x1 {
                    let g = img.data[row + x] as usize;
                    hist[g] += 1;
                }
            }

            // 裁剪并再分配
            let mut total_excess = 0u64;
            for h in &mut hist {
                if *h > clip_value as u64 {
                    total_excess += *h - clip_value as u64;
                    *h = clip_value as u64;
                }
            }
            let step = total_excess / HIST_SIZE as u64;
            let mut rem = total_excess % HIST_SIZE as u64;
            for h in &mut hist {
                *h += step;
                if rem > 0 {
                    *h += 1;
                    rem -= 1;
                }
            }

            // CDF → LUT
            let mut lut = [0u8; HIST_SIZE];
            let area = ((x1 - x0) * (y1 - y0)) as f64;
            let mut sum = 0u64;
            for g in 0..HIST_SIZE {
                sum += hist[g];
                let val = (sum as f64 * (HIST_SIZE - 1) as f64 / area).round();
                lut[g] = val as u8;
            }
            luts.push(lut);
        }
    }

    // 双线性插值合成输出
    let mut data = vec![0u8; cols * rows];
    let lut_at =
        |ty: usize, tx: usize, g: usize| -> f64 { f64::from(luts[ty * params.tiles_x + tx][g]) };
    for y in 0..rows {
        // 瓦片行
        let ty_raw = if y == 0 {
            0
        } else {
            (y / tile_h).min(params.tiles_y - 1)
        };
        // 大于 0 的行，位置归一化到 [0,1) 内
        let ty_pos = if tile_h > 0 {
            let f = (y % tile_h) as f64 / tile_h as f64;
            f.min(1.0)
        } else {
            0.0
        };
        let ty0 = ty_raw;
        // 相邻瓦片行（行内最后一个瓦片回退到自身）
        let ty1 = (ty_raw + 1).min(params.tiles_y - 1);

        for x in 0..cols {
            let tx_raw = if x == 0 {
                0
            } else {
                (x / tile_w).min(params.tiles_x - 1)
            };
            let tx_pos = if tile_w > 0 {
                let f = (x % tile_w) as f64 / tile_w as f64;
                f.min(1.0)
            } else {
                0.0
            };
            let tx0 = tx_raw;
            let tx1 = (tx_raw + 1).min(params.tiles_x - 1);

            let g = usize::from(img.data[y * cols + x]);
            let t00 = lut_at(ty0, tx0, g);
            let t01 = lut_at(ty0, tx1, g);
            let t10 = lut_at(ty1, tx0, g);
            let t11 = lut_at(ty1, tx1, g);
            // 先沿 x、再沿 y 双线性
            let top = t00 * (1.0 - tx_pos) + t01 * tx_pos;
            let bot = t10 * (1.0 - tx_pos) + t11 * tx_pos;
            let val = top * (1.0 - ty_pos) + bot * ty_pos;
            data[y * cols + x] = val.round().clamp(0.0, 255.0) as u8;
        }
    }

    GrayImage {
        width: cols,
        height: rows,
        data,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gray(image: &[u8], w: usize, h: usize) -> GrayImage {
        GrayImage {
            width: w,
            height: h,
            data: image.to_vec(),
        }
    }

    #[test]
    fn equalize_hist_low_contrast_improves_range() {
        // 灰度挤在 60..140 的低对比图
        let mut data = Vec::with_capacity(64 * 64);
        let mut v = 60u8;
        for _ in 0..(64 * 64) {
            data.push(v);
            v = if v >= 140 { 60 } else { v + 1 };
        }
        let img = gray(&data, 64, 64);
        let out = equalize_hist(&img);
        let (mut minv, mut maxv) = (u8::MAX, 0u8);
        for &px in &out.data {
            minv = minv.min(px);
            maxv = maxv.max(px);
        }
        // 均衡后动态范围应显著扩大
        assert!(maxv - minv > 100, "range only {minv}..{maxv}");
        // 输出仍为 8 位灰度图
        assert_eq!(out.width, 64);
        assert_eq!(out.height, 64);
    }

    #[test]
    fn equalize_hist_constant_image_stable() {
        // 常值图：均衡后仍为常值（不引入伪对比），且映射到最高 bin 对应的值。
        let img = gray(&[100u8; 100], 10, 10);
        let out = equalize_hist(&img);
        assert!(
            out.data.iter().all(|&px| px == 255 || px == 100),
            "constant image should stay constant"
        );
        let first = out.data[0];
        assert!(
            out.data.iter().all(|&px| px == first),
            "constant image should map to a single value"
        );
    }

    #[test]
    fn equalize_hist_edges_preserved() {
        // 全黑 + 全白两半
        let mut data = vec![0u8; 32 * 16];
        data[..32 * 8].fill(0);
        data[32 * 8..].fill(255);
        let img = gray(&data, 32, 16);
        let out = equalize_hist(&img);
        let (mut has_low, mut has_high) = (false, false);
        for &px in &out.data {
            if px < 10 {
                has_low = true;
            }
            if px > 245 {
                has_high = true;
            }
        }
        assert!(has_low && has_high);
    }

    #[test]
    fn clahe_changes_contrast() {
        let mut data = vec![0u8; 40 * 40];
        // 低频亮斑 + 少量噪声
        for y in 0..40usize {
            for x in 0..40usize {
                let base = if x < 20 && y < 20 { 200u8 } else { 160u8 };
                data[y * 40 + x] = base;
            }
        }
        let img = gray(&data, 40, 40);
        let out = clahe(
            &img,
            &ClaheParams {
                clip_limit: 10.0,
                tiles_x: 8,
                tiles_y: 8,
            },
        );
        // CLAHE 后局部对比被增强：极差/方差应不同于输入
        let mean =
            |d: &[u8]| -> f64 { d.iter().map(|&v| f64::from(v)).sum::<f64>() / d.len() as f64 };
        let var = |d: &[u8]| -> f64 {
            let m = mean(d);
            d.iter()
                .map(|&v| {
                    let d = f64::from(v) - m;
                    d * d
                })
                .sum::<f64>()
                / d.len() as f64
        };
        assert!((var(&img.data) - var(&out.data)).abs() > 1e-3);
        assert_eq!(out.width, 40);
        assert_eq!(out.height, 40);
    }

    #[test]
    fn clahe_constant_image_stable() {
        let img = gray(&[128u8; 25], 5, 5);
        let out = clahe(&img, &ClaheParams::default());
        // 常数图 → 每瓦片直方图仅一个 bin，CDF 满格映射到单一值
        //（OpenCV CLAHE 对常值图同样输出单一值，具体取值取决于 CDF 归一化）
        let first = out.data[0];
        assert!(
            out.data.iter().all(|&p| p == first),
            "constant image should map to a single value, got {first}"
        );
        assert_eq!(out.width, 5);
        assert_eq!(out.height, 5);
    }

    #[test]
    fn clahe_manual_grid_8x8() {
        // 网格数不整除时也应正确（非 8 倍尺寸）
        let mut data = vec![0u8; 30 * 30];
        data[15 * 30 + 15] = 200;
        let img = gray(&data, 30, 30);
        let out = clahe(
            &img,
            &ClaheParams {
                clip_limit: 2.0,
                tiles_x: 8,
                tiles_y: 8,
            },
        );
        assert_eq!(out.data.len(), 30 * 30);
    }

    #[test]
    fn empty_image_no_panic() {
        let img = gray(&[], 0, 0);
        let out_eq = equalize_hist(&img);
        assert_eq!(out_eq.data.len(), 0);
        let out_cl = clahe(&img, &ClaheParams::default());
        assert_eq!(out_cl.data.len(), 0);
    }
}
