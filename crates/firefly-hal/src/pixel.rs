//! 采集帧换算：设备像素格式 → 总线消息载荷（纯计算，无 I/O）。
//!
//! 深度无效值沿用 `planner` 的判据（`z <= 0.05` 视为无效），因此 `Z16` 的 0
//! 直接缩放成 0 米，不额外编码成 `NaN`。

use serde::Deserialize;

/// 相机采集格式，与配置里的 `format` 字段同名（小写）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum FrameFormat {
    /// 8 位灰度（`V4L2_PIX_FMT_GREY`）：载荷可直接拷贝。
    Grey,
    /// 打包 `YUYV` 4:2:2（`V4L2_PIX_FMT_YUYV`）：只取亮度分量。
    Yuyv,
    /// 16 位小端深度（`V4L2_PIX_FMT_Z16`）：按配置缩放成米。
    Z16,
}

impl FrameFormat {
    /// 每像素占用的源字节数。
    #[must_use]
    pub const fn bytes_per_pixel(self) -> usize {
        match self {
            Self::Grey => 1,
            Self::Yuyv | Self::Z16 => 2,
        }
    }
}

/// 把 `YUYV` 的亮度分量写入 `out`，返回写入的像素数。
///
/// `YUYV` 每四字节携带两个像素：`Y0 U Y1 V`。任一侧先耗尽即停止，调用方用
/// 返回的像素数与期望值比较，判断帧长度是否符合协商的格式。
#[must_use]
pub fn gray_from_yuyv(yuyv: &[u8], out: &mut [u8]) -> usize {
    let mut written = 0;
    for pair in yuyv.as_chunks::<4>().0 {
        if written + 2 > out.len() {
            break;
        }
        out[written] = pair[0];
        out[written + 1] = pair[2];
        written += 2;
    }
    written
}

/// 把 `Z16` 深度写入 `out`（米），返回写入的像素数。
#[must_use]
pub fn meters_from_z16(z16: &[u8], out: &mut [f32], scale: f32) -> usize {
    let pixels = (z16.len() / 2).min(out.len());
    for (index, slot) in out.iter_mut().take(pixels).enumerate() {
        let raw = u16::from_le_bytes([z16[index * 2], z16[index * 2 + 1]]);
        *slot = f32::from(raw) * scale;
    }
    pixels
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn yuyv_keeps_luma_of_both_pixels() {
        let source = [10, 128, 20, 128, 30, 128, 40, 128];
        let mut out = [0u8; 4];
        assert_eq!(gray_from_yuyv(&source, &mut out), 4);
        assert_eq!(out, [10, 20, 30, 40]);
    }

    #[test]
    fn yuyv_stops_at_short_target() {
        let source = [10, 128, 20, 128, 30, 128, 40, 128];
        let mut out = [0u8; 3];
        assert_eq!(gray_from_yuyv(&source, &mut out), 2);
        assert_eq!(out, [10, 20, 0]);
    }

    #[test]
    fn yuyv_ignores_trailing_partial_pair() {
        let source = [10, 128, 20, 128, 30];
        let mut out = [0u8; 4];
        assert_eq!(gray_from_yuyv(&source, &mut out), 2);
        assert_eq!(out, [10, 20, 0, 0]);
    }

    #[test]
    fn z16_scales_millimetres_to_metres() {
        let mut source = Vec::new();
        for raw in [0u16, 500, 1500] {
            source.extend_from_slice(&raw.to_le_bytes());
        }
        let mut out = [f32::NAN; 3];
        assert_eq!(meters_from_z16(&source, &mut out, 0.001), 3);
        assert!((out[0] - 0.0).abs() < 1e-6);
        assert!((out[1] - 0.5).abs() < 1e-6);
        assert!((out[2] - 1.5).abs() < 1e-6);
    }

    #[test]
    fn z16_stops_at_short_target() {
        let mut source = Vec::new();
        for raw in [100u16, 200] {
            source.extend_from_slice(&raw.to_le_bytes());
        }
        let mut out = [0f32; 1];
        assert_eq!(meters_from_z16(&source, &mut out, 1.0), 1);
        assert!((out[0] - 100.0).abs() < 1e-6);
    }

    #[test]
    fn bytes_per_pixel_matches_layout() {
        assert_eq!(FrameFormat::Grey.bytes_per_pixel(), 1);
        assert_eq!(FrameFormat::Yuyv.bytes_per_pixel(), 2);
        assert_eq!(FrameFormat::Z16.bytes_per_pixel(), 2);
    }
}
