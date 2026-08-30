//! 图像补丁金字塔与 NCC（论文 V-C/V-D，对照 `vio.cpp:203` `getImagePatch`
//! 与 `vio.cpp:333` `calculateNCC`）。
//!
//! 补丁金字塔：3 层同尺寸 11×11，每层从上一层半采样（`scale = 1<<level`），
//! 像素取双线性插值（`vio.cpp:212-223` 的 `w_ref_*` 加权）。
//!
//! 双线性权重四角记号（`w_tl`/`w_tr`/`w_bl`/`w_br`）为图像插值惯例，
//! 予以模块级允许（对照 `firefly-vio-core/src/cam.rs` 既有先例）。
#![allow(clippy::similar_names)]

use firefly_void_types::visual::GrayImage;
use nalgebra::Vector2;

use crate::options::VoxelMapOptions;

/// 补丁金字塔：`levels[level]` 为 `patch_size × patch_size` 灰度。
#[derive(Debug, Clone, PartialEq)]
pub struct PatchPyramid {
    /// 每层数据（行主序 `patch_size×patch_size`）。
    pub levels: Vec<Vec<f64>>,
    /// 每层采样间隔（`1 << level`）。
    pub scale: Vec<u32>,
    /// 补丁边长（像素）。
    pub patch_size: usize,
}

impl PatchPyramid {
    /// 第 0 层数据切片。
    #[must_use]
    pub fn level0(&self) -> &[f64] {
        &self.levels[0]
    }

    /// 第 0 层均值。
    #[must_use]
    pub fn level0_mean(&self) -> f64 {
        let data = &self.levels[0];
        if data.is_empty() {
            return 0.0;
        }
        data.iter().sum::<f64>() / data.len() as f64
    }

    /// 两补丁第 0 层 NCC（均值中心化，对照 `vio.cpp:333`）。
    #[must_use]
    pub fn ncc_level0(&self, other: &Self) -> f64 {
        let a = &self.levels[0];
        let b = &other.levels[0];
        let n = a.len().min(b.len());
        if n == 0 {
            return 0.0;
        }
        let mean_a = a.iter().take(n).sum::<f64>() / n as f64;
        let mean_b = b.iter().take(n).sum::<f64>() / n as f64;
        let mut up = 0.0;
        let mut down1 = 0.0;
        let mut down2 = 0.0;
        for i in 0..n {
            let da = a[i] - mean_a;
            let db = b[i] - mean_b;
            up += da * db;
            down1 += da * da;
            down2 += db * db;
        }
        let denom = (down1 * down2).sqrt();
        if denom < 1e-12 { 0.0 } else { up / denom }
    }
}

/// 从图像提取补丁金字塔（双线性插值，越界像素截断为 0）。
///
/// 对照 `vio.cpp:203` `getImagePatch`：`scale = 1<<level`，以 `(px - half·scale)`
/// 为起点按 `scale` 步长采样。论文 V-C 要求补丁尺寸 11、层数 3。
#[must_use]
pub fn extract_patch_pyramid(
    image: &GrayImage,
    px: Vector2<f64>,
    opts: &VoxelMapOptions,
) -> PatchPyramid {
    let patch_size = opts.patch_size;
    let half = (patch_size as i64) / 2;
    let mut levels = Vec::with_capacity(opts.patch_pyramid_level);
    let mut scale_list = Vec::with_capacity(opts.patch_pyramid_level);
    for level in 0..opts.patch_pyramid_level {
        let scale = 1u32 << level;
        let start_u = (px[0] / f64::from(scale)).floor() * f64::from(scale);
        let start_v = (px[1] / f64::from(scale)).floor() * f64::from(scale);
        let subpix_u = (px[0] - start_u) / f64::from(scale);
        let subpix_v = (px[1] - start_v) / f64::from(scale);
        let w_tl = (1.0 - subpix_u) * (1.0 - subpix_v);
        let w_tr = subpix_u * (1.0 - subpix_v);
        let w_bl = (1.0 - subpix_u) * subpix_v;
        let w_br = subpix_u * subpix_v;

        let w = [w_tl, w_tr, w_bl, w_br];

        let origin_u = start_u - (half * i64::from(scale)) as f64;
        let origin_v = start_v - (half * i64::from(scale)) as f64;
        let mut data = Vec::with_capacity(patch_size * patch_size);
        for y in 0..patch_size {
            for x in 0..patch_size {
                let u = origin_u + (x as i64 * i64::from(scale)) as f64;
                let v = origin_v + (y as i64 * i64::from(scale)) as f64;
                data.push(sample_bilinear(image, u, v, f64::from(scale), w));
            }
        }
        levels.push(data);
        scale_list.push(scale);
    }
    PatchPyramid {
        levels,
        scale: scale_list,
        patch_size,
    }
}

/// 双线性采样（`vio.cpp:218-222`：四角权重）。
fn sample_bilinear(image: &GrayImage, u: f64, v: f64, scale: f64, w: [f64; 4]) -> f64 {
    let u_i = u.floor() as usize;
    let v_i = v.floor() as usize;
    let width = image.width();
    let height = image.height();
    let at = |x: usize, y: usize| -> f64 {
        if x < width && y < height {
            f64::from(image.get(x, y).unwrap_or(0))
        } else {
            0.0
        }
    };
    let tl = at(u_i, v_i);
    let tr = at(u_i + scale as usize, v_i);
    let bl = at(u_i, v_i + scale as usize);
    let br = at(u_i + scale as usize, v_i + scale as usize);
    w[0] * tl + w[1] * tr + w[2] * bl + w[3] * br
}
