//! 深度重投影到左目像素网格；深度单位为米，值为相机轴向深度。
#![allow(clippy::many_single_char_names)]
use firefly_base::{FrameId, RigidTransform};
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector3;

/// 针孔模型；图像原点在左上，相机轴为右、上、后。
#[derive(Clone, Copy, Debug)]
pub struct Pinhole {
    pub width: usize,
    pub height: usize,
    pub focal: f64,
    pub cx: f64,
    pub cy: f64,
}
impl Pinhole {
    /// 像素与轴向深度反投影到相机系。
    #[must_use]
    pub fn point(self, uv: [f64; 2], z: f64) -> Vector3<f64> {
        Vector3::new(
            (uv[0] - self.cx) * z / self.focal,
            -(uv[1] - self.cy) * z / self.focal,
            -z,
        )
    }
    /// 相机点投影；相机后方与非法点没有像素。
    #[must_use]
    pub fn pixel(self, p: Vector3<f64>) -> Option<[f64; 2]> {
        (p.iter().all(|v| v.is_finite()) && p.z < -0.2).then(|| {
            [
                self.cx - self.focal * p.x / p.z,
                self.cy + self.focal * p.y / p.z,
            ]
        })
    }
}

/// 对齐后的轴向深度；空洞保持 NaN，遮挡竞争保留最近表面。
pub struct RegisteredDepth {
    camera: Pinhole,
    values: Vec<f64>,
}
impl RegisteredDepth {
    /// 在连续像素处采样；要求四邻点有效且属于连续表面，禁止跨边缘插值。
    #[must_use]
    pub fn point_at(&self, uv: [f64; 2]) -> Option<Vector3<f64>> {
        if !uv.iter().all(|v| v.is_finite()) || uv[0] < 0. || uv[1] < 0. {
            return None;
        }
        let x = uv[0].floor() as usize;
        let y = uv[1].floor() as usize;
        if uv[0] >= (self.camera.width - 1) as f64 || uv[1] >= (self.camera.height - 1) as f64 {
            return None;
        }
        let w = self.camera.width;
        let z = [
            self.values[y * w + x],
            self.values[y * w + x + 1],
            self.values[(y + 1) * w + x],
            self.values[(y + 1) * w + x + 1],
        ];
        if !z.iter().all(|v| v.is_finite()) {
            return None;
        }
        let near = z.iter().copied().fold(f64::INFINITY, f64::min);
        let far = z.iter().copied().fold(0., f64::max);
        // 三厘米或深度的 2%：只允许连续表面的小范围插值。
        if far - near > 0.03_f64.max(near * 0.02) {
            return None;
        }
        let a = uv[0] - x as f64;
        let b = uv[1] - y as f64;
        let depth = (1. - b) * ((1. - a) * z[0] + a * z[1]) + b * ((1. - a) * z[2] + a * z[3]);
        Some(self.camera.point(uv, depth))
    }
}

/// 深度相机反投影 → 刚体变换 → 左目投影 → z-buffer。
/// # Errors
/// 尺寸、内参或坐标方向不满足契约。
pub fn register_depth(
    depth: &[f32],
    source: Pinhole,
    target: Pinhole,
    transform: &RigidTransform,
) -> Result<RegisteredDepth> {
    if source.width.checked_mul(source.height) != Some(depth.len())
        || source.width == 0
        || source.height == 0
        || target.width < 2
        || target.height < 2
        || target
            .width
            .checked_mul(target.height)
            .is_none_or(|n| n > 16_777_216)
        || [source.focal, target.focal]
            .iter()
            .any(|v| !v.is_finite() || *v <= 0.)
        || ![source.cx, source.cy, target.cx, target.cy]
            .iter()
            .all(|v| v.is_finite())
        || transform.source() != FrameId::DEPTH_CAMERA
        || transform.target() != FrameId::LEFT_CAMERA
    {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "invalid depth registration calibration or dimensions",
        ));
    }
    let mut values = vec![f64::NAN; target.width * target.height];
    for (i, &depth) in depth.iter().enumerate() {
        let z = f64::from(depth);
        if !(0.2..=20.).contains(&z) {
            continue;
        }
        let source_point = source.point([(i % source.width) as f64, (i / source.width) as f64], z);
        let p = transform.point(source_point.into()).coords;
        let Some(uv) = target.pixel(p) else {
            continue;
        };
        let x = uv[0].round();
        let y = uv[1].round();
        if x < 0. || y < 0. || x >= target.width as f64 || y >= target.height as f64 {
            continue;
        }
        let value = &mut values[y as usize * target.width + x as usize];
        if !value.is_finite() || -p.z < *value {
            *value = -p.z;
        }
    }
    Ok(RegisteredDepth {
        camera: target,
        values,
    })
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    fn camera() -> Pinhole {
        Pinhole {
            width: 32,
            height: 24,
            focal: 100.,
            cx: 16.,
            cy: 12.,
        }
    }
    fn shift(x: f64) -> RigidTransform {
        RigidTransform::from_parts(
            FrameId::LEFT_CAMERA,
            FrameId::DEPTH_CAMERA,
            [x, 0., 0.],
            [0., 0., 0., 1.],
        )
        .unwrap()
    }
    #[test]
    fn baseline_projection_has_known_value_and_derivative() {
        let k = camera();
        let p = k.point([16., 12.], 1.);
        assert_eq!(
            k.pixel(shift(-0.025).point(p.into()).coords).unwrap(),
            [13.5, 12.]
        );
        let eps = 1e-6;
        let a = k.pixel(shift(-0.025 + eps).point(p.into()).coords).unwrap()[0];
        let b = k.pixel(shift(-0.025 - eps).point(p.into()).coords).unwrap()[0];
        assert!(((a - b) / (2. * eps) - 100.).abs() < 1e-7);
    }
    #[test]
    fn target_rays_recover_plane_and_do_not_fill_disocclusions() {
        let k = camera();
        let d = register_depth(&vec![1.; 32 * 24], k, k, &shift(-0.025)).unwrap();
        let p = d.point_at([15.25, 12.5]).unwrap();
        assert!((p - Vector3::new(-0.0075, -0.005, -1.)).norm() < 1e-12);
        assert!(d.point_at([30., 12.]).is_none());
    }
    #[test]
    fn z_buffer_selects_front_surface_and_edges_are_rejected() {
        let k = camera();
        let mut src = vec![f32::NAN; 32 * 24];
        src[12 * 32 + 17] = 2.;
        src[12 * 32 + 18] = 1.;
        let d = register_depth(&src, k, k, &shift(-0.02)).unwrap();
        assert_eq!(d.values[12 * 32 + 16], 1.);
        let mut src = vec![1.; 32 * 24];
        for y in 0..24 {
            for x in 16..32 {
                src[y * 32 + x] = 2.;
            }
        }
        let d = register_depth(&src, k, k, &shift(0.)).unwrap();
        assert!(d.point_at([15.5, 12.]).is_none());
        assert!(d.point_at([10., 12.]).is_some());
        assert!(d.point_at([f64::NAN, 0.]).is_none());
    }
    #[test]
    fn rotated_plane_matches_independent_plane_equation() {
        let k = camera();
        let theta = 0.08_f64;
        let transform = RigidTransform::from_parts(
            FrameId::LEFT_CAMERA,
            FrameId::DEPTH_CAMERA,
            [0.; 3],
            [0., (theta / 2.).sin(), 0., (theta / 2.).cos()],
        )
        .unwrap();
        let registered = register_depth(&vec![2.; 32 * 24], k, k, &transform).unwrap();
        let p = registered.point_at([16.2, 12.2]).unwrap();
        let expected_z = 2. / (theta.cos() - theta.sin() * 0.2 / 100.);
        // 最近像素重投影的 0.5px 量化误差对应此平面小于 1mm 轴向误差。
        assert!((-p.z - expected_z).abs() < 0.001);
        assert!(registered.point_at([f64::MAX, 0.]).is_none());
        assert!(register_depth(&[1.], k, k, &transform).is_err());
        assert!(register_depth(&vec![2.; 32 * 24], k, k, &transform.inverse()).is_err());
    }
}
