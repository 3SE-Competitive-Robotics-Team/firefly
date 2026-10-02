//! 金字塔逆组合 Lucas–Kanade；残差和空间梯度均用灰度值/像素单位。
//! 对照 OpenVINS TrackKLT 调用的 OpenCV LK：Scharr 导数、粗到细、初值传播。

use crate::sensor::GrayImage;
use nalgebra::Vector2;
use purecv::core::{
    Matrix,
    types::{BorderTypes, Size2i},
};
use purecv::imgproc::derivatives::scharr;
use purecv::video::optical_flow::build_optical_flow_pyramid;
use rayon::prelude::*;

fn sample(im: &Matrix<f32>, x: f32, y: f32) -> f64 {
    let x = x.clamp(0., (im.cols - 1) as f32);
    let y = y.clamp(0., (im.rows - 1) as f32);
    let (ix, iy) = (x as usize, y as usize);
    let (jx, jy) = ((ix + 1).min(im.cols - 1), (iy + 1).min(im.rows - 1));
    let (a, b) = ((x - ix as f32) as f64, (y - iy as f32) as f64);
    (1. - b)
        * ((1. - a) * im.data[iy * im.cols + ix] as f64 + a * im.data[iy * im.cols + jx] as f64)
        + b * ((1. - a) * im.data[jy * im.cols + ix] as f64 + a * im.data[jy * im.cols + jx] as f64)
}

/// Scharr 对单位斜坡的响应为 32，除以 32 才是灰度值/像素。
fn gradient(im: &Matrix<f32>, dx: i32, dy: i32) -> Matrix<f32> {
    scharr(im, dx, dy, 1. / 32., 0., BorderTypes::Reflect101).expect("valid grayscale derivative")
}

fn refine(
    prev: &Matrix<f32>,
    next: &Matrix<f32>,
    gx: &Matrix<f32>,
    gy: &Matrix<f32>,
    p: Vector2<f32>,
    mut q: Vector2<f32>,
    iterations: usize,
) -> Option<Vector2<f32>> {
    let initial = q;
    let mut patch = Vec::with_capacity(441);
    let (mut xx, mut xy, mut yy) = (0., 0., 0.);
    for y in -10..=10 {
        for x in -10..=10 {
            let (u, v) = (p.x + x as f32, p.y + y as f32);
            let (ix, iy) = (sample(gx, u, v), sample(gy, u, v));
            patch.push((x as f32, y as f32, sample(prev, u, v), ix, iy));
            xx += ix * ix;
            xy += ix * iy;
            yy += iy * iy;
        }
    }
    let determinant = xx * yy - xy * xy;
    let eigen = 0.5 * (xx + yy - ((xx - yy).powi(2) + 4. * xy * xy).sqrt()) / 441.;
    if determinant <= f64::EPSILON || eigen < 1e-4 {
        return None;
    }
    let mut previous = Vector2::<f32>::zeros();
    for i in 0..iterations {
        if q.x < 0. || q.y < 0. || q.x >= next.cols as f32 || q.y >= next.rows as f32 {
            return None;
        }
        let (mut bx, mut by) = (0., 0.);
        for &(x, y, value, ix, iy) in &patch {
            let residual = value - sample(next, q.x + x, q.y + y);
            bx += ix * residual;
            by += iy * residual;
        }
        let delta = Vector2::new(
            ((yy * bx - xy * by) / determinant) as f32,
            ((xx * by - xy * bx) / determinant) as f32,
        );
        q += delta;
        if (q - initial).norm() > 10. {
            return None;
        }
        if delta.norm_squared() < 0.01_f32.powi(2) {
            break;
        }
        if i > 0 && (delta + previous).abs().max() < 0.01 {
            q -= 0.5 * delta;
            break;
        }
        previous = delta;
    }
    (q.iter().all(|v| v.is_finite())
        && q.x >= 0.
        && q.y >= 0.
        && q.x < next.cols as f32
        && q.y < next.rows as f32)
        .then_some(q)
}

#[fastrace::trace]
fn track_one_way(
    prev: &GrayImage,
    next: &GrayImage,
    points: &[Vector2<f32>],
    initial: &[Vector2<f32>],
) -> (Vec<Vector2<f32>>, Vec<bool>) {
    assert_eq!(points.len(), initial.len());
    let build = |im: &GrayImage| {
        build_optical_flow_pyramid(
            &super::pyramid::gray_to_matrix(im),
            Size2i::new(21, 21),
            3,
            false,
            BorderTypes::Reflect101,
            BorderTypes::Reflect101,
        )
        .expect("valid grayscale pyramid")
        .levels
    };
    let (pyr0, pyr1) = (build(prev), build(next));
    let levels = pyr0.len().min(pyr1.len());
    let gradients: Vec<_> = pyr0
        .par_iter()
        .map(|im| (gradient(im, 1, 0), gradient(im, 0, 1)))
        .collect();
    points
        .par_iter()
        .zip(initial)
        .map(|(&p, &init)| {
            let mut q = init / (1 << (levels - 1)) as f32;
            let mut valid = false;
            for level in (0..levels).rev() {
                let scale = (1 << level) as f32;
                if level + 1 < levels {
                    q *= 2.;
                }
                let (gx, gy) = &gradients[level];
                if let Some(result) = refine(&pyr0[level], &pyr1[level], gx, gy, p / scale, q, 30) {
                    q = result;
                    valid = true;
                } else {
                    valid = false;
                }
            }
            (q, valid)
        })
        .unzip()
}

/// 双向一致性拒绝遮挡、越界与不稳定匹配；阈值 0.5 像素。
pub(super) fn optical_flow(
    prev: &GrayImage,
    next: &GrayImage,
    points: &[Vector2<f32>],
    initial: &[Vector2<f32>],
) -> (Vec<Vector2<f32>>, Vec<bool>) {
    let (out, mut valid) = track_one_way(prev, next, points, initial);
    let (back, reverse_valid) = track_one_way(next, prev, &out, points);
    for i in 0..valid.len() {
        valid[i] &= reverse_valid[i] && (back[i] - points[i]).norm() <= 0.5;
    }
    (out, valid)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn texture(x: f64, y: f64) -> f64 {
        120. + 35. * (0.07 * x).sin() + 30. * (0.09 * y).cos() + 20. * (0.05 * x + 0.06 * y).sin()
    }
    fn image(shift: Vector2<f64>) -> Matrix<f32> {
        Matrix::from_vec(
            80,
            80,
            1,
            (0..6400)
                .map(|i| texture((i % 80) as f64 - shift.x, (i / 80) as f64 - shift.y) as f32)
                .collect(),
        )
    }
    #[test]
    fn derivative_matches_independent_finite_difference_and_unit_ramp() {
        let ramp = Matrix::from_vec(
            80,
            80,
            1,
            (0..6400)
                .map(|i| (2 * (i % 80) + 3 * (i / 80)) as f32)
                .collect(),
        );
        assert!((sample(&gradient(&ramp, 1, 0), 40., 40.) - 2.).abs() < 1e-6);
        assert!((sample(&gradient(&ramp, 0, 1), 40., 40.) - 3.).abs() < 1e-6);
        let im = image(Vector2::zeros());
        let h = 1e-3;
        let exact = (texture(40. + h, 40.) - texture(40. - h, 40.)) / (2. * h);
        assert!((sample(&gradient(&im, 1, 0), 40., 40.) - exact).abs() < 0.005);
    }
    #[test]
    fn one_iteration_recovers_small_translation_in_pixel_units() {
        let p = Vector2::new(40., 40.);
        let d = Vector2::new(0.08, -0.06);
        let a = image(Vector2::zeros());
        let b = image(d);
        let q = refine(&a, &b, &gradient(&a, 1, 0), &gradient(&a, 0, 1), p, p, 1).unwrap();
        assert!(
            ((q - p).cast::<f64>() - d).norm() < 0.001,
            "step={:?}, expected={d:?}",
            q - p
        );
    }
    #[test]
    fn pyramid_recovers_signed_subpixel_and_large_translations() {
        for d in [
            Vector2::new(0.25, -0.4),
            Vector2::new(-3.2, 2.4),
            Vector2::new(12.5, -9.25),
        ] {
            let make = |shift: Vector2<f64>| GrayImage {
                width: 320,
                height: 240,
                data: (0..76800)
                    .map(|i| {
                        texture(
                            2. * ((i % 320) as f64 - shift.x),
                            2. * ((i / 320) as f64 - shift.y),
                        )
                        .round() as u8
                    })
                    .collect(),
            };
            let pts = vec![
                Vector2::new(96., 88.),
                Vector2::new(180., 144.),
                Vector2::new(120., 176.),
            ];
            let (out, status) = optical_flow(&make(Vector2::zeros()), &make(d), &pts, &pts);
            for ((p, q), ok) in pts.iter().zip(out).zip(status) {
                assert!(ok);
                assert!(
                    ((q - p).cast::<f64>() - d).norm() < 0.08,
                    "shift={d:?} measured={:?}",
                    q - p
                );
            }
        }
    }
}
