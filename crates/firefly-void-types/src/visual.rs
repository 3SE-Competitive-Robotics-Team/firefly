//! 视觉/标定基础类型（P2 起供 firefly-void-map 使用）。
//!
//! - [`GrayImage`]：单通道 8bit 灰度图（行主序）；
//! - [`Intrinsics`]：针孔相机内参（无畸变，畸变模型在 P3 测量 crate 处理）；
//! - [`VisualState`]：视觉更新所需的每帧状态（帧号 + 逆曝光时间）。

use nalgebra::{Vector2, Vector3};

/// 单通道 8bit 灰度图（行主序，`data[y*width+x]`）。
///
/// 对照 `cv::Mat` 灰度图像；P2 地图 crate 从中提取图像补丁。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GrayImage {
    width: usize,
    height: usize,
    data: Vec<u8>,
}

impl GrayImage {
    /// 构造灰度图。`data.len()` 必须等于 `width*height`。
    ///
    /// # Panics
    /// `data.len() != width*height` 时 panic。
    #[must_use]
    pub fn new(width: usize, height: usize, data: Vec<u8>) -> Self {
        assert_eq!(
            data.len(),
            width * height,
            "灰度图数据长度 {} 与宽高 {}x{} 不匹配",
            data.len(),
            width,
            height
        );
        Self {
            width,
            height,
            data,
        }
    }

    /// 图像宽度（像素）。
    #[must_use]
    pub const fn width(&self) -> usize {
        self.width
    }

    /// 图像高度（像素）。
    #[must_use]
    pub const fn height(&self) -> usize {
        self.height
    }

    /// 原始数据切片（行主序）。
    #[must_use]
    pub fn as_slice(&self) -> &[u8] {
        &self.data
    }

    /// 取像素（越界返回 `None`）。
    #[must_use]
    pub fn get(&self, x: usize, y: usize) -> Option<u8> {
        if x < self.width && y < self.height {
            Some(self.data[y * self.width + x])
        } else {
            None
        }
    }
}

/// 针孔相机内参（单位像素；`cx`,`cy` 为主点）。
///
/// 对照论文 (13) 式的投影矩阵 `P`；仿真相机 320×240、f=300（DESIGN.md §3）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Intrinsics {
    /// 焦距 x（像素）。
    pub fx: f64,
    /// 焦距 y（像素）。
    pub fy: f64,
    /// 主点 x（像素）。
    pub cx: f64,
    /// 主点 y（像素）。
    pub cy: f64,
}

impl Intrinsics {
    /// 构造。
    #[must_use]
    pub const fn new(fx: f64, fy: f64, cx: f64, cy: f64) -> Self {
        Self { fx, fy, cx, cy }
    }

    /// 相机系点 → 像素坐标（`z <= 0` 时返回 `None`）。
    #[must_use]
    pub fn project(&self, p_cam: &Vector3<f64>) -> Option<Vector2<f64>> {
        if p_cam[2] <= 0.0 {
            return None;
        }
        let inv_z = 1.0 / p_cam[2];
        Some(Vector2::new(
            self.fx * p_cam[0] * inv_z + self.cx,
            self.fy * p_cam[1] * inv_z + self.cy,
        ))
    }

    /// 像素坐标 → 相机系单位方向（z=1 平面上的点）。
    #[must_use]
    pub fn unproject(&self, px: &Vector2<f64>) -> Vector3<f64> {
        Vector3::new(
            (px[0] - self.cx) / self.fx,
            (px[1] - self.cy) / self.fy,
            1.0,
        )
    }
}

/// 视觉更新所需的每帧状态。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VisualState {
    /// 帧号（单调递增，用于"距上次增补 >20 帧"判据）。
    pub frame_id: u32,
    /// 逆曝光时间 `τ`（相对首帧，无单位）。
    pub inv_expo_time: f64,
}

impl VisualState {
    /// 构造。
    #[must_use]
    pub const fn new(frame_id: u32, inv_expo_time: f64) -> Self {
        Self {
            frame_id,
            inv_expo_time,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gray_image_access() {
        let img = GrayImage::new(2, 2, vec![0, 1, 2, 3]);
        assert_eq!(img.get(0, 0), Some(0));
        assert_eq!(img.get(1, 1), Some(3));
        assert_eq!(img.get(2, 0), None);
        assert_eq!(img.get(0, 2), None);
    }

    #[test]
    fn intrinsics_project_unproject_roundtrip() {
        let k = Intrinsics::new(300.0, 300.0, 160.0, 120.0);
        let p = Vector3::new(1.0, -2.0, 5.0);
        let px = k.project(&p).unwrap();
        assert!((px[0] - 220.0).abs() < 1e-9);
        assert!((px[1] - 0.0).abs() < 1e-9);
        let dir = k.unproject(&px);
        assert!((dir.normalize() - p.normalize()).norm() < 1e-9);
        // 相机后方的点投影失败
        assert!(k.project(&Vector3::new(0.0, 0.0, -1.0)).is_none());
    }
}
