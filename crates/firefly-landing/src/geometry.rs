//! 共面 H 图案角点与标定针孔几何；图案外接矩形具有四个实际白色凸角。
use crate::{Detection, Result, invalid};
use nalgebra::{Isometry3, Point3, Vector2, Vector3};
use purecv::{
    core::{Matrix, Point2f, Size2i, TermCriteria, TermType},
    imgproc::corner_sub_pix,
};
use serde::Deserialize;

#[derive(Clone, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Geometry {
    /// fx、fy、cx、cy，像素；默认匹配仿真 CIF、70.88° 垂直视场，实机使用标定值。
    pub intrinsics: [f64; 4],
    /// Brown–Conrady k1、k2、p1、p2、k3。
    pub distortion: [f64; 5],
    /// H 白色图案外接矩形尺寸（米），来自 rmuc2027 CAD 的共面顶点。
    pub marker_size: [f64; 2],
    pub max_reprojection_px: f64,
}

impl Default for Geometry {
    fn default() -> Self {
        let focal = 144. / (70.88_f64.to_radians() / 2.).tan();
        Self {
            intrinsics: [focal, focal, 175.5, 143.5],
            distortion: [0.; 5],
            marker_size: [0.163_496_61, 0.194_825_32],
            max_reprojection_px: 2.,
        }
    }
}

impl Geometry {
    /// Fit the rectangle on a horizontal support plane using odometry gravity.
    /// Orthogonal Procrustes keeps front-facing planar depth stable under pixel noise.
    /// The H pattern retains its 180-degree heading ambiguity.
    #[must_use]
    #[allow(clippy::many_single_char_names)]
    pub fn horizontal_target(
        &self,
        corners: [[f64; 2]; 4],
        world_camera: &Isometry3<f64>,
    ) -> Option<(Vector3<f64>, f64)> {
        let [w, h] = self.marker_size;
        let object = [
            Vector2::new(-w / 2., -h / 2.),
            Vector2::new(w / 2., -h / 2.),
            Vector2::new(w / 2., h / 2.),
            Vector2::new(-w / 2., h / 2.),
        ];
        let mut rays = [Vector2::zeros(); 4];
        for (i, pixel) in corners.into_iter().enumerate() {
            let pixel = self.ray(Vector2::from(pixel))?;
            let direction = world_camera.rotation * Vector3::new(pixel.x, pixel.y, 1.);
            if direction.z >= -0.2 {
                return None;
            }
            rays[i] = direction.xy() / direction.z;
        }
        let center = rays.iter().copied().sum::<Vector2<f64>>() / 4.;
        let origin = world_camera.translation.vector;
        let mut best: Option<(Vector3<f64>, f64)> = None;
        for shift in 0..2 {
            let mut covariance = nalgebra::Matrix2::zeros();
            for i in 0..4 {
                covariance -= (rays[(i + shift) % 4] - center) * object[i].transpose();
            }
            let svd = covariance.svd(true, true);
            let rotation = svd.u? * svd.v_t?;
            let scale =
                svd.singular_values.sum() / object.iter().map(Vector2::norm_squared).sum::<f64>();
            if !scale.is_finite() || scale <= 0. {
                continue;
            }
            let depth = 1. / scale;
            let position = origin - Vector3::new(center.x, center.y, 1.) * depth;
            let mut squared = 0.;
            for i in 0..4 {
                let xy = rotation * object[i];
                let world = position + Vector3::new(xy.x, xy.y, 0.);
                let camera = world_camera.inverse_transform_point(&Point3::from(world));
                if camera.z <= 0. {
                    squared = f64::INFINITY;
                    break;
                }
                let measured = self.ray(Vector2::from(corners[(i + shift) % 4]))?;
                squared += ((camera.x / camera.z - measured.x) * self.intrinsics[0]).powi(2)
                    + ((camera.y / camera.z - measured.y) * self.intrinsics[1]).powi(2);
            }
            let rms = (squared / 4.).sqrt();
            if rms <= self.max_reprojection_px && best.as_ref().is_none_or(|(_, old)| rms < *old) {
                best = Some((position, rms));
            }
        }
        best
    }

    /// # Errors
    /// 非有限标定、非正焦距/尺寸/门限。
    pub fn validate(&self) -> Result<()> {
        if !self
            .intrinsics
            .iter()
            .chain(self.distortion.iter())
            .chain(self.marker_size.iter())
            .chain([&self.max_reprojection_px])
            .all(|v| v.is_finite())
            || self.intrinsics[..2]
                .iter()
                .chain(self.marker_size.iter())
                .any(|&v| v <= 0.)
            || self.max_reprojection_px <= 0.
        {
            return Err(invalid("invalid landing camera/marker geometry"));
        }
        Ok(())
    }

    fn ray(&self, pixel: Vector2<f64>) -> Option<Vector2<f64>> {
        let [fx, fy, cx, cy] = self.intrinsics;
        let distorted = Vector2::new((pixel.x - cx) / fx, (pixel.y - cy) / fy);
        let mut point = distorted;
        let [k1, k2, p1, p2, k3] = self.distortion;
        for _ in 0..12 {
            let r2 = point.norm_squared();
            let radial = 1. + r2 * (k1 + r2 * (k2 + r2 * k3));
            if !radial.is_finite() || radial.abs() < 1e-9 {
                return None;
            }
            let tangent = Vector2::new(
                2. * p1 * point.x * point.y + p2 * (r2 + 2. * point.x * point.x),
                p1 * (r2 + 2. * point.y * point.y) + 2. * p2 * point.x * point.y,
            );
            point = (distorted - tangent) / radial;
        }
        point.iter().all(|v| v.is_finite()).then_some(point)
    }
}

/// Three bounded brightness thresholds separate white paint from reflection/shadow.
/// # Errors
/// Invalid image dimensions or failed subpixel refinement.
pub fn corner_candidates(
    gray: &[u8],
    width: usize,
    height: usize,
    detection: Detection,
) -> Result<Vec<[[f64; 2]; 4]>> {
    let mut candidates = Vec::new();
    for percent in [85, 95, 65] {
        if let Some(corners) = refine_at_threshold(gray, width, height, detection, percent)?
            && !candidates.contains(&corners)
        {
            candidates.push(corners);
        }
    }
    Ok(candidates)
}

/// Return the first refined white-H candidate in brightness-priority order.
/// # Errors
/// Invalid image dimensions or failed subpixel refinement.
pub fn refine_corners(
    gray: &[u8],
    width: usize,
    height: usize,
    detection: Detection,
) -> Result<Option<[[f64; 2]; 4]>> {
    Ok(corner_candidates(gray, width, height, detection)?
        .first()
        .copied())
}

/// ROI 内提取连通白色 H，凸包化简到四个实际凸角，再作亚像素精化。
/// 残缺目标、低对比度和实心亮斑返回 None。
/// # Errors
/// 图像尺寸无效或角点精化失败。
#[allow(clippy::too_many_lines)]
fn refine_at_threshold(
    gray: &[u8],
    width: usize,
    height: usize,
    detection: Detection,
    percent: u16,
) -> Result<Option<[[f64; 2]; 4]>> {
    if gray.len() != width * height {
        return Err(invalid("corner image dimensions"));
    }
    let [x0, y0, x1, y1] = detection.bounds;
    let margin = ((x1 - x0).max(y1 - y0) * 0.1).ceil();
    let left = (x0 - margin).max(0.) as usize;
    let top = (y0 - margin).max(0.) as usize;
    let right = (x1 + margin).min(width as f32) as usize;
    let bottom = (y1 + margin).min(height as f32) as usize;
    if right <= left + 12 || bottom <= top + 12 {
        return Ok(None);
    }
    let (rw, rh) = (right - left, bottom - top);
    let pixels: Vec<u8> = (top..bottom)
        .flat_map(|y| gray[y * width + left..y * width + right].iter().copied())
        .collect();
    let low = *pixels.iter().min().unwrap_or(&0);
    let high = *pixels.iter().max().unwrap_or(&0);
    if high.saturating_sub(low) < 40 {
        return Ok(None);
    }
    // Keep the white paint separate from midtone specular texture on the board.
    let threshold = u16::from(low) + (u16::from(high) - u16::from(low)) * percent / 100;
    let mut visited = vec![false; pixels.len()];
    let mut best: Option<(usize, Vec<Vector2<f64>>)> = None;
    for seed in 0..pixels.len() {
        if visited[seed] || u16::from(pixels[seed]) <= threshold {
            continue;
        }
        let mut stack = vec![seed];
        let mut points = Vec::new();
        let mut boundary = false;
        visited[seed] = true;
        while let Some(index) = stack.pop() {
            let (x, y) = (index % rw, index / rw);
            boundary |= x == 0 || y == 0 || x + 1 == rw || y + 1 == rh;
            points.push(Vector2::new(x as f64, y as f64));
            for neighbor in [
                x.checked_sub(1).map(|nx| y * rw + nx),
                (x + 1 < rw).then_some(index + 1),
                y.checked_sub(1).map(|ny| ny * rw + x),
                (y + 1 < rh).then_some(index + rw),
            ]
            .into_iter()
            .flatten()
            {
                if !visited[neighbor] && u16::from(pixels[neighbor]) > threshold {
                    visited[neighbor] = true;
                    stack.push(neighbor);
                }
            }
        }
        if boundary || points.len() < 40 {
            continue;
        }
        let count = points.len();
        let mut hull = convex_hull(points);
        if hull.len() < 4 {
            continue;
        }
        let area = polygon_area(&hull);
        let fill = count as f64 / area;
        if !(0.25..0.85).contains(&fill) {
            continue;
        }
        while hull.len() > 4 {
            let i = (0..hull.len())
                .min_by(|&a, &b| removal_area(&hull, a).total_cmp(&removal_area(&hull, b)))
                .unwrap_or(0);
            hull.remove(i);
        }
        if polygon_area(&hull) < 0.9 * area {
            continue;
        }
        if best.as_ref().is_none_or(|(old, _)| count > *old) {
            best = Some((count, hull));
        }
    }
    let Some((_, hull)) = best else {
        return Ok(None);
    };
    let mut corners: Vec<_> = hull
        .iter()
        .map(|p| Point2f::new(p.x as f32, p.y as f32))
        .collect();
    let before = corners.clone();
    let image = Matrix::from_vec(rh, rw, 1, pixels);
    corner_sub_pix(
        &image,
        &mut corners,
        Size2i::new(3, 3),
        Size2i::new(-1, -1),
        TermCriteria::new(TermType::Both, 20, 0.01),
    )
    .map_err(|_| invalid("landing subpixel corner refinement"))?;
    if corners
        .iter()
        .zip(&before)
        .any(|(a, b)| !a.x.is_finite() || !a.y.is_finite() || (a.x - b.x).hypot(a.y - b.y) > 3.)
    {
        return Ok(None);
    }
    Ok(Some(std::array::from_fn(|i| {
        [
            f64::from(corners[i].x) + left as f64,
            f64::from(corners[i].y) + top as f64,
        ]
    })))
}

fn cross(a: Vector2<f64>, b: Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}
fn convex_hull(mut points: Vec<Vector2<f64>>) -> Vec<Vector2<f64>> {
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    let mut hull: Vec<Vector2<f64>> = Vec::new();
    for point in &points {
        while hull.len() >= 2
            && cross(
                hull[hull.len() - 1] - hull[hull.len() - 2],
                *point - hull[hull.len() - 1],
            ) <= 0.
        {
            hull.pop();
        }
        hull.push(*point);
    }
    let lower = hull.len();
    for point in points.iter().rev().skip(1) {
        while hull.len() > lower
            && cross(
                hull[hull.len() - 1] - hull[hull.len() - 2],
                *point - hull[hull.len() - 1],
            ) <= 0.
        {
            hull.pop();
        }
        hull.push(*point);
    }
    hull.pop();
    hull
}
fn polygon_area(points: &[Vector2<f64>]) -> f64 {
    (0..points.len())
        .map(|i| cross(points[i], points[(i + 1) % points.len()]))
        .sum::<f64>()
        .abs()
        / 2.
}
fn removal_area(points: &[Vector2<f64>], i: usize) -> f64 {
    cross(
        points[i] - points[(i + points.len() - 1) % points.len()],
        points[(i + 1) % points.len()] - points[i],
    )
    .abs()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn horizontal_fit_recovers_center_with_tilt_and_pixel_noise() {
        use nalgebra::{Translation3, UnitQuaternion};
        let geometry = Geometry::default();
        for tilt in [0., 0.12] {
            let camera = Isometry3::from_parts(
                Translation3::new(0.07, -0.03, 0.8),
                UnitQuaternion::from_euler_angles(std::f64::consts::PI + tilt, -0.08, 0.4),
            );
            let [w, h] = geometry.marker_size;
            let corners = [
                [-w / 2., -h / 2.],
                [w / 2., -h / 2.],
                [w / 2., h / 2.],
                [-w / 2., h / 2.],
            ]
            .map(|[x, y]| {
                let p = camera.inverse_transform_point(&Point3::new(x, y, 0.));
                [
                    geometry.intrinsics[0] * p.x / p.z + geometry.intrinsics[2],
                    geometry.intrinsics[1] * p.y / p.z + geometry.intrinsics[3],
                ]
            });
            let (position, error) = geometry.horizontal_target(corners, &camera).unwrap();
            assert!(position.norm() < 1e-8);
            assert!(error < 1e-8);
            let noisy = std::array::from_fn(|i| {
                [
                    corners[i][0] + if i % 2 == 0 { 0.12 } else { -0.1 },
                    corners[i][1] + if i < 2 { 0.08 } else { -0.1 },
                ]
            });
            let (position, error) = geometry.horizontal_target(noisy, &camera).unwrap();
            assert!(position.norm() < 0.005);
            assert!(error < 0.2);
            assert!(
                geometry
                    .horizontal_target([[10., 10.]; 4], &camera)
                    .is_none()
            );
        }
    }
    #[test]
    fn bright_solid_and_partial_h_are_rejected() {
        let detection = Detection {
            bounds: [5., 5., 95., 95.],
            confidence: 1.,
        };
        assert!(
            refine_corners(&[127; 10000], 100, 100, detection)
                .unwrap()
                .is_none()
        );
        let mut image = vec![0; 10000];
        for y in 20..80 {
            for x in 25..75 {
                image[y * 100 + x] = 255;
            }
        }
        assert!(
            refine_corners(&image, 100, 100, detection)
                .unwrap()
                .is_none()
        );
        for y in 30..70 {
            for x in 35..65 {
                if !(47..53).contains(&y) {
                    image[y * 100 + x] = 0;
                }
            }
        }
        assert!(
            refine_corners(&image, 100, 100, detection)
                .unwrap()
                .is_some()
        );
    }
}
