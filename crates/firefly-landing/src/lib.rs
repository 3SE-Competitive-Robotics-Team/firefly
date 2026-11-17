//! 下视降落的图像坐标与检测契约；所有几何使用像素中心坐标。

use firefly_error::{Error, ErrorKind, Result};
pub mod geometry;

/// 原图坐标下的检测框，边界采用像素边缘坐标。
#[derive(Clone, Copy, Debug)]
pub struct Detection {
    pub bounds: [f32; 4],
    pub confidence: f32,
}

/// 固定形状 letterbox；缩放后宽高与整数填充同时用于正反变换。
#[derive(Clone, Copy, Debug)]
pub struct Letterbox {
    source: [usize; 2],
    target: [usize; 2],
    resized: [usize; 2],
    padding: [usize; 2],
}

impl Letterbox {
    /// 宽、高均须为正。返回值同时描述图像预处理和检测坐标反变换。
    /// # Errors
    /// 尺寸为空或溢出。
    pub fn new(source: [usize; 2], target: [usize; 2]) -> Result<Self> {
        if source.contains(&0)
            || target.contains(&0)
            || source[0].checked_mul(source[1]).is_none()
            || target[0]
                .checked_mul(target[1])
                .and_then(|n| n.checked_mul(3))
                .is_none()
        {
            return Err(invalid("invalid letterbox dimensions"));
        }
        let scale = (target[0] as f64 / source[0] as f64).min(target[1] as f64 / source[1] as f64);
        let resized = source.map(|n| (n as f64 * scale).round().max(1.) as usize);
        Ok(Self {
            source,
            target,
            resized,
            padding: [(target[0] - resized[0]) / 2, (target[1] - resized[1]) / 2],
        })
    }

    /// 灰度 → 三通道 NCHW FP32，双线性缩放，边界复制，填充 114/255。
    /// # Errors
    /// 图像或张量长度与声明尺寸不符。
    pub fn write_gray(&self, gray: &[u8], tensor: &mut [f32]) -> Result<()> {
        let [sw, sh] = self.source;
        let [tw, th] = self.target;
        let [rw, rh] = self.resized;
        if gray.len() != sw * sh || tensor.len() != 3 * tw * th {
            return Err(invalid(
                "image/tensor length differs from letterbox dimensions",
            ));
        }
        tensor.fill(114. / 255.);
        for y in 0..rh {
            let sy = ((y as f64 + 0.5) * sh as f64 / rh as f64 - 0.5).clamp(0., (sh - 1) as f64);
            let y0 = sy.floor() as usize;
            let y1 = (y0 + 1).min(sh - 1);
            let fy = (sy - y0 as f64) as f32;
            for x in 0..rw {
                let sx =
                    ((x as f64 + 0.5) * sw as f64 / rw as f64 - 0.5).clamp(0., (sw - 1) as f64);
                let x0 = sx.floor() as usize;
                let x1 = (x0 + 1).min(sw - 1);
                let fx = (sx - x0 as f64) as f32;
                let a =
                    f32::from(gray[y0 * sw + x0]) * (1. - fx) + f32::from(gray[y0 * sw + x1]) * fx;
                let b =
                    f32::from(gray[y1 * sw + x0]) * (1. - fx) + f32::from(gray[y1 * sw + x1]) * fx;
                let value = (a * (1. - fy) + b * fy) / 255.;
                let index = (y + self.padding[1]) * tw + x + self.padding[0];
                for channel in 0..3 {
                    tensor[channel * tw * th + index] = value;
                }
            }
        }
        Ok(())
    }

    /// YOLO 像素边缘坐标反变换到原图；裁掉填充中的框。
    #[must_use]
    pub fn detection(&self, bounds: [f32; 4], confidence: f32) -> Option<Detection> {
        if !confidence.is_finite()
            || !(0.0..=1.0).contains(&confidence)
            || !bounds.iter().all(|v| v.is_finite())
        {
            return None;
        }
        let bounds = std::array::from_fn(|i| {
            let axis = i % 2;
            ((bounds[i] - self.padding[axis] as f32) * self.source[axis] as f32
                / self.resized[axis] as f32)
                .clamp(0., self.source[axis] as f32)
        });
        (bounds[2] > bounds[0] && bounds[3] > bounds[1]).then_some(Detection { bounds, confidence })
    }
}

/// 单类别原始检测头 `[1,5,N]`，xywh/score；有界 CPU 后处理使 TopK/NMS 位于 NPU 图外。
/// # Errors
/// 输出形状、阈值或数据长度无效。
#[allow(clippy::many_single_char_names)]
pub fn decode_raw_detections(
    shape: &[i64],
    values: &[f32],
    letterbox: &Letterbox,
    threshold: f32,
) -> Result<Vec<Detection>> {
    if shape.len() != 3
        || shape[0] != 1
        || shape[1] != 5
        || shape[2] <= 0
        || shape[2] > 10000
        || values.len() != 5 * shape[2] as usize
        || !threshold.is_finite()
        || !(0.0..=1.0).contains(&threshold)
    {
        return Err(invalid(
            "expected single-class YOLO raw [1,5,N], N in 1..=10000",
        ));
    }
    let n = shape[2] as usize;
    let mut candidates: Vec<_> = (0..n)
        .filter_map(|i| {
            let score = values[4 * n + i];
            if score < threshold {
                return None;
            }
            let (x, y, w, h) = (
                values[i],
                values[n + i],
                values[2 * n + i],
                values[3 * n + i],
            );
            if w <= 0. || h <= 0. {
                return None;
            }
            letterbox.detection([x - w / 2., y - h / 2., x + w / 2., y + h / 2.], score)
        })
        .collect();
    candidates.sort_by(|a, b| b.confidence.total_cmp(&a.confidence));
    candidates.truncate(300);
    let mut accepted: Vec<Detection> = Vec::new();
    for candidate in candidates {
        if accepted
            .iter()
            .all(|previous| intersection_over_union(previous.bounds, candidate.bounds) < 0.5)
        {
            accepted.push(candidate);
        }
    }
    Ok(accepted)
}

fn intersection_over_union(a: [f32; 4], b: [f32; 4]) -> f32 {
    let intersection =
        (a[2].min(b[2]) - a[0].max(b[0])).max(0.) * (a[3].min(b[3]) - a[1].max(b[1])).max(0.);
    intersection / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersection)
}

fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cif_camera_enters_cif_network_without_resizing_or_padding() {
        let transform = Letterbox::new([352, 288], [352, 288]).unwrap();
        let image: Vec<_> = (0..352 * 288).map(|i| (i % 256) as u8).collect();
        let mut tensor = vec![0.; 3 * image.len()];
        transform.write_gray(&image, &mut tensor).unwrap();
        for (channel, pixels) in tensor.chunks_exact(image.len()).enumerate() {
            assert!(
                pixels
                    .iter()
                    .zip(&image)
                    .all(|(&a, &b)| a == f32::from(b) / 255.),
                "channel {channel}"
            );
        }
        assert_eq!(
            transform
                .detection([10., 20., 100., 200.], 0.8)
                .unwrap()
                .bounds,
            [10., 20., 100., 200.]
        );
    }

    #[test]
    fn letterbox_preserves_full_frame_and_removes_padding() {
        let transform = Letterbox::new([640, 480], [416, 416]).unwrap();
        let detection = transform.detection([0., 52., 416., 364.], 0.9).unwrap();
        assert_eq!(detection.bounds, [0., 0., 640., 480.]);
        assert!(transform.detection([0., 0., 416., 40.], 0.9).is_none());
    }

    #[test]
    fn grayscale_letterbox_keeps_channel_and_pixel_center_contract() {
        let transform = Letterbox::new([2, 2], [4, 6]).unwrap();
        let mut tensor = vec![0.; 3 * 4 * 6];
        transform
            .write_gray(&[0, 255, 255, 0], &mut tensor)
            .unwrap();
        assert_eq!(&tensor[..4], &[114. / 255.; 4]);
        assert_eq!(&tensor[..24], &tensor[24..48]);
        assert_eq!(tensor[4], 0.);
        assert_eq!(tensor[7], 1.);
        assert!((tensor[9] - 0.375).abs() < 1e-6);
    }

    #[test]
    fn decoder_rejects_other_model_heads_and_invalid_detections() {
        let transform = Letterbox::new([640, 480], [416, 416]).unwrap();
        assert!(decode_raw_detections(&[1, 300, 6], &[], &transform, 0.3).is_err());
        // Two overlapping boxes and a NaN center: retain the strongest valid box.
        let values = [
            208.,
            209.,
            f32::NAN,
            208.,
            208.,
            208.,
            416.,
            416.,
            416.,
            312.,
            312.,
            312.,
            0.9,
            0.8,
            0.95,
        ];
        let detections = decode_raw_detections(&[1, 5, 3], &values, &transform, 0.3).unwrap();
        assert_eq!(detections.len(), 1);
        assert_eq!(detections[0].bounds, [0., 0., 640., 480.]);
        assert_eq!(detections[0].confidence, 0.9);
        assert!(decode_raw_detections(&[1, 5, 3], &values, &transform, f32::NAN).is_err());
    }
}
