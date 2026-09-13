//! 传感器像素数学：灰度转换 + 深度线性化 + 深度噪声 + 调试显示映射。
//!
//! 口径对照 `packages/firefly-mujoco/src/firefly_mujoco/env.py`（`DroneEnv`）：
//! 灰度用 BT.601 加权；深度噪声三步（视差域高斯、边缘膨胀 1px、随机丢点），
//! 仅作用于有效命中（`0.05 < z < 100` 米），其余保持无效标记 `0.0`。
//!
//! 深度线性化依据：`Bevy` 相机投影恒为无限远反向 Z
//! （`PerspectiveProjection::get_clip_from_view` 用
//! `perspective_infinite_reverse_rh`），深度预通道纹理存 NDC `d ∈ [0, 1]`，
//! `d = 0` 为清空值（无几何体，判无效），`dist = near / d` 精确还原视距。

use rand::RngExt;
use rand::rngs::StdRng;

/// 深度噪声强度（对照 `DroneEnv(depth_noise=0.02)` 默认值，无单位）。
pub const DEPTH_NOISE: f32 = 0.02;
/// 双目基线（米，对照 MJCF `cam_left/right` 沿 y 相距 0.05）。
pub const STEREO_BASELINE: f32 = 0.05;
/// 深度有效下限（米，对照 `env.py` 有效掩码 `depth > 0.05`）。
pub const DEPTH_MIN: f32 = 0.05;
/// 深度有效上限（米，对照 `env.py` 有效掩码 `depth < 100.0`）。
pub const DEPTH_MAX: f32 = 100.0;
/// 调试图深度显示上限（米，超出显示为最远色）。
pub const DISPLAY_DEPTH_RANGE: f32 = 20.0;

/// RGB（sRGB 字节序）→ 灰度（BT.601 加权，对照 `DroneEnv::_to_gray`）。
///
/// `rgb.len() == 4 * gray.len()`（RGBA 行主序转单通道行主序）。
pub fn rgb_to_gray(rgb: &[u8], gray: &mut [u8]) {
    debug_assert_eq!(rgb.len(), 4 * gray.len());
    for (dst, src) in gray.iter_mut().zip(rgb.as_chunks::<4>().0) {
        *dst = (0.299 * f32::from(src[0]) + 0.587 * f32::from(src[1]) + 0.114 * f32::from(src[2]))
            as u8;
    }
}

/// NDC 深度（小端 f32 字节）→ 米制视距（行主序）。
///
/// `dist = near / d`；`d <= 0`（清空值）或解码越界一律记无效 `0.0`，
/// 与 `env.py` 的无效语义一致（planner 以 `z <= 0.05` 判无效）。
pub fn linearize_depth(raw: &[u8], out: &mut [f32], near: f32) {
    debug_assert_eq!(raw.len(), 4 * out.len());
    for (dst, src) in out.iter_mut().zip(raw.as_chunks::<4>().0) {
        let d = f32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        let z = if d > 0.0 && d.is_finite() {
            near / d
        } else {
            0.0
        };
        *dst = if z > DEPTH_MIN && z < DEPTH_MAX {
            z
        } else {
            0.0
        };
    }
}

/// 深度噪声（原地，对照 `DroneEnv::render_depth` 三步；`fov_y` 为弧度）。
///
/// 1. 视差域高斯：`disp = f·B/z`，`σ_disp = 4·DEPTH_NOISE` 像素，`σ_z ∝ z²`；
/// 2. 边缘膨胀：四邻深度差超 `max(0.12, 0.04·z)` 判边缘，前景向外扩 1 像素；
/// 3. 随机丢点：每帧均匀抽 `5~15%` 有把效像素置 `0.0`。
pub fn add_depth_noise(
    depth: &mut [f32],
    width: usize,
    height: usize,
    fov_y: f32,
    rng: &mut StdRng,
) {
    debug_assert_eq!(depth.len(), width * height);
    let focal = (height as f32 / 2.0) / (fov_y / 2.0).tan();
    let fb = focal * STEREO_BASELINE;
    let sigma_disp = DEPTH_NOISE * 4.0;

    // 1. 视差域高斯（仅有效命中）。
    for z in depth.iter_mut() {
        if *z > DEPTH_MIN && *z < DEPTH_MAX {
            let disp = fb / *z;
            let noisy = (disp + gaussian(rng) * sigma_disp).max(0.1);
            *z = fb / noisy;
        }
    }

    // 2. 边缘膨胀 1px（四邻差分，阈值近距 12cm、远距 4%·z）。
    let edge = edge_mask(depth, width, height);
    if edge.iter().any(|e| *e) {
        dilate_foreground(depth, &edge, width, height);
    }

    // 3. 随机丢点（仅有效像素）。
    let hole_rate: f32 = rng.random_range(0.05..0.15);
    for z in depth.iter_mut() {
        if *z > DEPTH_MIN && *z < DEPTH_MAX && rng.random::<f32>() < hole_rate {
            *z = 0.0;
        }
    }
}

/// 标准高斯采样（Box-Muller，避免为单分布引入 `rand_distr` 依赖）。
fn gaussian(rng: &mut StdRng) -> f32 {
    let u1 = rng.random::<f32>().max(f32::EPSILON);
    let u2 = rng.random::<f32>();
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos()
}

/// 边缘掩码：有效像素四邻存在深度差超阈值的邻居。
fn edge_mask(depth: &[f32], width: usize, height: usize) -> Vec<bool> {
    let mut edge = vec![false; depth.len()];
    for y in 0..height {
        for x in 0..width {
            let i = y * width + x;
            let z = depth[i];
            if z <= DEPTH_MIN || z >= DEPTH_MAX {
                continue;
            }
            let thresh = 0.12f32.max(0.04 * z);
            let neighbors = [
                x.checked_sub(1).map(|nx| y * width + nx),
                (x + 1 < width).then(|| y * width + x + 1),
                y.checked_sub(1).map(|ny| ny * width + x),
                (y + 1 < height).then(|| (y + 1) * width + x),
            ];
            for n in neighbors.into_iter().flatten() {
                let nz = depth[n];
                if nz > DEPTH_MIN && nz < DEPTH_MAX && (z - nz).abs() > thresh {
                    edge[i] = true;
                    break;
                }
            }
        }
    }
    edge
}

/// 前景膨胀：非边缘有效像素若 3×3 邻域内有更近的前景，取前景深度。
fn dilate_foreground(depth: &mut [f32], edge: &[bool], width: usize, height: usize) {
    let src = depth.to_vec();
    for y in 0..height {
        for x in 0..width {
            let i = y * width + x;
            if edge[i] || src[i] <= DEPTH_MIN || src[i] >= DEPTH_MAX {
                continue;
            }
            let mut nearest = f32::INFINITY;
            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    if nx < 0 || ny < 0 || nx >= width as i32 || ny >= height as i32 {
                        continue;
                    }
                    let nz = src[ny as usize * width + nx as usize];
                    if nz > DEPTH_MIN && nz < DEPTH_MAX && edge[ny as usize * width + nx as usize] {
                        nearest = nearest.min(nz);
                    }
                }
            }
            if nearest.is_finite() && src[i] > nearest + 1e-9 {
                depth[i] = nearest;
            }
        }
    }
}

/// 灰度 → 调试显示（RGBA 行主序，三通道复制，`alpha = 255`）。
pub fn gray_to_display(gray: &[u8], rgba: &mut [u8]) {
    debug_assert_eq!(rgba.len(), 4 * gray.len());
    for (dst, g) in rgba.as_chunks_mut::<4>().0.iter_mut().zip(gray.iter()) {
        dst[0] = *g;
        dst[1] = *g;
        dst[2] = *g;
        dst[3] = 255;
    }
}

/// 深度 → 调试显示（近白远黑，`max_range` 外与无效记黑）。
pub fn depth_to_display(depth: &[f32], rgba: &mut [u8], max_range: f32) {
    debug_assert_eq!(rgba.len(), 4 * depth.len());
    for (dst, z) in rgba.as_chunks_mut::<4>().0.iter_mut().zip(depth.iter()) {
        let v = if *z > DEPTH_MIN && *z < DEPTH_MAX {
            (255.0 * (1.0 - (*z / max_range).clamp(0.0, 1.0))) as u8
        } else {
            0
        };
        dst[0] = v;
        dst[1] = v;
        dst[2] = v;
        dst[3] = 255;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::Mat4;
    use rand::SeedableRng;

    /// BT.601 三原色与白场定点值。
    #[test]
    fn gray_matches_bt601() {
        let rgb = [
            255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255,
        ];
        let mut gray = [0u8; 4];
        rgb_to_gray(&rgb, &mut gray);
        assert_eq!(gray[0], (0.299 * 255.0) as u8);
        assert_eq!(gray[1], (0.587 * 255.0) as u8);
        assert_eq!(gray[2], (0.114 * 255.0) as u8);
        assert_eq!(gray[3], 255);
    }

    /// 线性化公式与 `Bevy` 实际投影矩阵互逆（近/中/远三点）。
    #[test]
    fn depth_linearize_matches_projection() {
        let near = 0.05f32;
        let proj = Mat4::perspective_infinite_reverse_rh(70.88f32.to_radians(), 4.0 / 3.0, near);
        // 恰为近平面时按有效掩码（`z > 0.05`，对照 `env.py`）判无效，故从内侧取值。
        for z in [0.06, 1.0, 10.0, 99.0] {
            let clip = proj * bevy::math::Vec4::new(0.0, 0.0, -z, 1.0);
            let ndc = clip.z / clip.w;
            let mut out = [0.0f32; 1];
            linearize_depth(&ndc.to_le_bytes(), &mut out, near);
            assert!((out[0] - z).abs() < 1e-3, "z={z} decoded={}", out[0]);
        }
    }

    /// 清空值（`d = 0`）与非法值判无效（精确 `0.0` 赋值，逐位比较成立）。
    #[allow(clippy::float_cmp)]
    #[test]
    fn depth_invalid_stays_zero() {
        let mut out = [1.0f32; 3];
        let raw: Vec<u8> = [0.0f32, f32::NAN, f32::INFINITY]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        linearize_depth(&raw, &mut out, 0.05);
        assert_eq!(out, [0.0, 0.0, 0.0]);
    }

    /// 噪声只改有效区形状不变：无效像素保持 `0.0`，有效值仍为正有限。
    #[allow(clippy::float_cmp)]
    #[test]
    fn depth_noise_preserves_invalid() {
        let mut rng = StdRng::seed_from_u64(7);
        let mut depth = vec![0.0f32; 32 * 32];
        for y in 8..24 {
            for x in 8..24 {
                depth[y * 32 + x] = 5.0;
            }
        }
        add_depth_noise(&mut depth, 32, 32, 70.88f32.to_radians(), &mut rng);
        assert_eq!(depth[0], 0.0);
        assert_eq!(depth[31 * 32 + 31], 0.0);
        let inner: Vec<f32> = depth[8 * 32 + 8..24 * 32].to_vec();
        assert!(inner.iter().all(|z| *z >= 0.0 && z.is_finite()));
        assert!(inner.iter().any(|z| *z > 0.0));
    }
}
