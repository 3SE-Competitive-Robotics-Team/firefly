//! 传感器像素数学：灰度转换、深度线性化与调试显示映射。
//!
//! 深度线性化依据：`Bevy` 相机投影恒为无限远反向 Z
//! （`PerspectiveProjection::get_clip_from_view` 用
//! `perspective_infinite_reverse_rh`），深度预通道纹理存 NDC `d ∈ [0, 1]`，
//! `d = 0` 为清空值（无几何体，判无效），`dist = near / d` 精确还原光轴 Z 深度。

/// 深度有效下限（米）。
pub const DEPTH_MIN: f32 = 0.05;
/// 深度有效上限（米）。
pub const DEPTH_MAX: f32 = 100.0;
/// 调试图深度显示上限（米，超出显示为最远色）。
pub const DISPLAY_DEPTH_RANGE: f32 = 20.0;

/// RGB（sRGB 字节序）→ 灰度（BT.601 加权）。
///
/// `rgb.len() == 4 * gray.len()`（RGBA 行主序转单通道行主序）。
pub fn rgb_to_gray(rgb: &[u8], gray: &mut [u8]) {
    debug_assert_eq!(rgb.len(), 4 * gray.len());
    for (dst, src) in gray.iter_mut().zip(rgb.as_chunks::<4>().0) {
        *dst = (0.299 * f32::from(src[0]) + 0.587 * f32::from(src[1]) + 0.114 * f32::from(src[2]))
            as u8;
    }
}

/// NDC 深度（小端 f32 字节）→ 米制光轴 Z 深度（行主序）。
///
/// `dist = near / d`；`d <= 0`（清空值）或解码越界一律记无效 `0.0`，
/// planner 以 `z <= 0.05` 判无效。
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
        // 恰为近平面时按有效掩码（`z > 0.05`）判无效，故从内侧取值。
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
}
