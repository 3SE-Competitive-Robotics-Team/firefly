//! 8 位灰度直方图增强，对照 OpenCV CPU `equalizeHist` 与 `CLAHE::apply`。

use crate::sensor::GrayImage;

const HIST_SIZE: usize = 256;

/// 全局均衡：最低占用灰度映射为零，常值图保持原值。
#[must_use]
pub fn equalize_hist(img: &GrayImage) -> GrayImage {
    let mut hist = [0usize; HIST_SIZE];
    for &px in &img.data {
        hist[usize::from(px)] += 1;
    }
    let Some(first) = hist.iter().position(|&count| count != 0) else {
        return img.clone();
    };
    if hist[first] == img.data.len() {
        return img.clone();
    }
    let scale = 255.0_f32 / (img.data.len() - hist[first]) as f32;
    let mut lut = [0u8; HIST_SIZE];
    let mut sum = 0;
    for i in first + 1..HIST_SIZE {
        sum += hist[i];
        lut[i] = (sum as f32 * scale).round_ties_even() as u8;
    }
    GrayImage {
        width: img.width,
        height: img.height,
        data: img.data.iter().map(|&px| lut[usize::from(px)]).collect(),
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

/// CLAHE：REFLECT_101 补边、均匀裁剪再分配、瓦片中心双线性插值。
/// `clip_limit <= 0` 禁用裁剪；空图返回空图。
///
/// # Panics
/// 网格任一维为零。
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn clahe(img: &GrayImage, params: &ClaheParams) -> GrayImage {
    assert!(
        params.tiles_x > 0 && params.tiles_y > 0,
        "CLAHE grid must be nonzero"
    );
    let (cols, rows) = (img.width, img.height);
    if cols == 0 || rows == 0 {
        return img.clone();
    }
    // OpenCV 在任一维不整除时同时补两维，整除的另一维也补一个网格周期。
    let (padded_cols, padded_rows) =
        if cols.is_multiple_of(params.tiles_x) && rows.is_multiple_of(params.tiles_y) {
            (cols, rows)
        } else {
            (
                cols + params.tiles_x - cols % params.tiles_x,
                rows + params.tiles_y - rows % params.tiles_y,
            )
        };
    let tile_w = padded_cols / params.tiles_x;
    let tile_h = padded_rows / params.tiles_y;
    let tile_area = tile_w * tile_h;
    let scale = 255.0_f32 / tile_area as f32;
    let clip_value = ((params.clip_limit * tile_area as f64 / HIST_SIZE as f64) as usize).max(1);
    let reflect = |pos: usize, len: usize| {
        if len == 1 {
            return 0;
        }
        let period = 2 * (len - 1);
        let folded = pos % period;
        folded.min(period - folded)
    };
    let mut luts = Vec::with_capacity(params.tiles_x * params.tiles_y);
    for ty in 0..params.tiles_y {
        for tx in 0..params.tiles_x {
            let mut hist = [0usize; HIST_SIZE];
            for y in ty * tile_h..(ty + 1) * tile_h {
                for x in tx * tile_w..(tx + 1) * tile_w {
                    hist[usize::from(img.data[reflect(y, rows) * cols + reflect(x, cols)])] += 1;
                }
            }
            if params.clip_limit > 0.0 {
                let mut clipped = 0;
                for count in &mut hist {
                    clipped += count.saturating_sub(clip_value);
                    *count = (*count).min(clip_value);
                }
                for count in &mut hist {
                    *count += clipped / HIST_SIZE;
                }
                let residual = clipped % HIST_SIZE;
                if let Some(step) = HIST_SIZE.checked_div(residual) {
                    for bin in (0..HIST_SIZE).step_by(step.max(1)).take(residual) {
                        hist[bin] += 1;
                    }
                }
            }
            let mut lut = [0u8; HIST_SIZE];
            let mut sum = 0;
            for (i, count) in hist.into_iter().enumerate() {
                sum += count;
                lut[i] = (sum as f32 * scale).round_ties_even() as u8;
            }
            luts.push(lut);
        }
    }
    let weights = |pos: usize, tile_size: usize, tiles: usize| {
        let coordinate = pos as f32 * (1.0 / tile_size as f32) - 0.5;
        let low = coordinate.floor() as isize;
        let weight = coordinate - low as f32;
        (
            low.max(0) as usize,
            (low + 1).min(tiles as isize - 1) as usize,
            weight,
        )
    };
    let x_weights: Vec<_> = (0..cols)
        .map(|x| weights(x, tile_w, params.tiles_x))
        .collect();
    let mut data = vec![0; cols * rows];
    for y in 0..rows {
        let (y0, y1, wy) = weights(y, tile_h, params.tiles_y);
        for (x, &(x0, x1, wx)) in x_weights.iter().enumerate() {
            let g = usize::from(img.data[y * cols + x]);
            let at = |ty: usize, tx: usize| f32::from(luts[ty * params.tiles_x + tx][g]);
            let value = (at(y0, x0) * (1.0 - wx) + at(y0, x1) * wx) * (1.0 - wy)
                + (at(y1, x0) * (1.0 - wx) + at(y1, x1) * wx) * wy;
            data[y * cols + x] = value.round_ties_even() as u8;
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
        // 常值图必须保留输入灰度，不能只检查输出是否为常数。
        let img = gray(&[100u8; 100], 10, 10);
        let out = equalize_hist(&img);
        assert!(
            out.data.iter().all(|&px| px == 100),
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
    fn equalization_subtracts_the_first_occupied_bin() {
        let img = gray(&[60, 60, 80, 100, 120, 140], 6, 1);
        assert_eq!(equalize_hist(&img).data, [0, 0, 64, 128, 191, 255]);
    }

    #[test]
    fn clahe_matches_opencv_cpu_fixtures() {
        // OpenCV 4.12.0 CPU，2×2 网格；覆盖零裁剪、瓦片中心和单轴补边。
        let cases: &[(usize, usize, f64, &[u8])] = &[
            (
                8,
                6,
                0.,
                &[
                    21, 64, 128, 160, 191, 250, 42, 85, 85, 149, 212, 255, 53, 122, 149, 212, 188,
                    234, 35, 102, 168, 222, 21, 99, 244, 64, 117, 196, 255, 74, 138, 212, 82, 166,
                    212, 24, 101, 179, 18, 88, 170, 234, 64, 160, 234, 48, 128, 234,
                ],
            ),
            (
                8,
                6,
                10.,
                &[
                    21, 64, 128, 160, 191, 250, 42, 85, 85, 149, 212, 255, 53, 122, 149, 212, 184,
                    230, 38, 106, 168, 224, 24, 102, 234, 64, 128, 191, 250, 85, 149, 223, 82, 149,
                    194, 42, 119, 188, 35, 106, 149, 212, 64, 149, 223, 64, 149, 234,
                ],
            ),
            (
                7,
                6,
                2.,
                &[
                    32, 64, 112, 151, 191, 223, 64, 80, 143, 207, 247, 64, 108, 143, 175, 223, 48,
                    96, 159, 207, 48, 235, 68, 124, 193, 241, 90, 136, 80, 143, 191, 50, 108, 183,
                    32, 143, 199, 76, 141, 197, 79, 135,
                ],
            ),
            (
                7,
                5,
                2.,
                &[
                    21, 64, 128, 160, 191, 234, 64, 85, 149, 212, 250, 64, 122, 149, 184, 230, 42,
                    106, 166, 208, 46, 234, 74, 128, 199, 234, 101, 138, 99, 149, 212, 61, 119,
                    192, 57,
                ],
            ),
        ];
        for &(w, h, clip_limit, expected) in cases {
            let pixels: Vec<_> = (0..h)
                .flat_map(|y| (0..w).map(move |x| ((x * 17 + y * 29 + x * y * 3) % 96 + 60) as u8))
                .collect();
            let out = clahe(
                &gray(&pixels, w, h),
                &ClaheParams {
                    clip_limit,
                    tiles_x: 2,
                    tiles_y: 2,
                },
            );
            assert_eq!(out.data, expected, "{w}×{h}, clip={clip_limit}");
        }
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
