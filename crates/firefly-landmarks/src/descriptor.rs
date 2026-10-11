//! 在已跟踪的原始像素上计算 ORB；不做最近像素的三维关联。
use firefly_error::{Error, ErrorKind, Result};
use purecv::{
    core::{Matrix, Point2f},
    features2d::{
        KeyPoint, Orb,
        orb::{compute_orientation, precompute_umax},
    },
};

pub type Descriptor = [u8; 32];

/// # Errors
/// 图像尺寸无效或上游描述子计算失败。
#[fastrace::trace]
pub fn extract(
    image: &[u8],
    width: usize,
    height: usize,
    pixels: &[[f32; 2]],
) -> Result<Vec<Option<Descriptor>>> {
    if width.checked_mul(height) != Some(image.len()) {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "descriptor image dimensions",
        ));
    }
    let matrix = Matrix::from_vec(height, width, 1, image.to_vec());
    let umax = precompute_umax(15);
    let mut selected = Vec::new();
    let mut keys = Vec::new();
    for (i, &[x, y]) in pixels.iter().enumerate() {
        // 旋转后的 31px 描述子采样区域必须完整落在图像内。
        if !x.is_finite()
            || !y.is_finite()
            || x < 23.
            || y < 23.
            || x >= width as f32 - 23.
            || y >= height as f32 - 23.
        {
            continue;
        }
        let angle = compute_orientation(&matrix, x.round() as usize, y.round() as usize, 31, &umax)
            .map_err(|e| Error::new(ErrorKind::Internal, e.to_string()))?;
        keys.push(KeyPoint {
            pt: Point2f::new(x, y),
            size: 31.,
            angle,
            ..Default::default()
        });
        selected.push(i);
    }
    let mut orb = Orb::default();
    orb.set_nlevels(1);
    // ORB-SLAM3 ORBextractor：方向来自原图，BRIEF 采样使用 7×7、σ=2 平滑图。
    let smooth = purecv::imgproc::gaussian_blur(
        &matrix,
        purecv::core::Size2i::new(7, 7),
        2.,
        2.,
        purecv::core::BorderTypes::Reflect101,
    )
    .map_err(|e| Error::new(ErrorKind::Internal, e.to_string()))?;
    let descriptors = orb
        .compute(&smooth, &keys)
        .map_err(|e| Error::new(ErrorKind::Internal, e.to_string()))?;
    let mut output = vec![None; pixels.len()];
    for (i, bytes) in selected
        .into_iter()
        .zip(descriptors.data.as_chunks::<32>().0)
    {
        output[i] = Some(*bytes);
    }
    Ok(output)
}

pub struct Detected {
    pub pixel: [f32; 2],
    pub descriptor: Descriptor,
}

/// 四层 ORB 金字塔，保留尺度与方向对应的描述子，坐标统一回原始图像。
/// # Errors
/// 图像尺寸非法或上游检测、平滑、描述子计算失败。
#[fastrace::trace]
pub fn detect(image: &[u8], width: usize, height: usize) -> Result<Vec<Detected>> {
    use purecv::features2d::orb::{build_orb_pyramid, compute_orb_descriptor};
    if width.checked_mul(height) != Some(image.len()) {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "descriptor image dimensions",
        ));
    }
    let error =
        |e: purecv::core::error::PureCvError| Error::new(ErrorKind::Internal, e.to_string());
    let matrix = Matrix::from_vec(height, width, 1, image.to_vec());
    let mut orb = Orb::default();
    orb.set_nfeatures(600);
    orb.set_nlevels(4);
    orb.set_edge_threshold(23);
    let keys = orb.detect(&matrix).map_err(error)?;
    let pyramid = build_orb_pyramid(&matrix, 4, 1.2).map_err(error)?;
    let smooth = pyramid
        .iter()
        .map(|level| {
            purecv::imgproc::gaussian_blur(
                level,
                purecv::core::Size2i::new(7, 7),
                2.,
                2.,
                purecv::core::BorderTypes::Reflect101,
            )
            .map_err(error)
        })
        .collect::<Result<Vec<_>>>()?;
    keys.into_iter()
        .map(|key| {
            let pixel = [key.pt.x, key.pt.y];
            let level = key.octave as usize;
            let mut scaled = key;
            let scale = 1.2_f32.powi(scaled.octave);
            scaled.pt.x /= scale;
            scaled.pt.y /= scale;
            let descriptor = compute_orb_descriptor(
                &smooth[level],
                &scaled,
                31,
                &purecv::features2d::BIT_PATTERN_31,
            )
            .map_err(error)?;
            Ok(Detected { pixel, descriptor })
        })
        .collect()
}

#[must_use]
pub fn distance(a: &Descriptor, b: &Descriptor) -> u32 {
    a.iter().zip(b).map(|(a, b)| (a ^ b).count_ones()).sum()
}

/// ORB-SLAM3 的 Hamming 低阈值 50 与比率检查；双向唯一，拒绝等距歧义。
#[must_use]
pub fn matches(query: &[Descriptor], train: &[Descriptor]) -> Vec<(usize, usize)> {
    let mut reverse = vec![(usize::MAX, u32::MAX); train.len()];
    let mut best = vec![(usize::MAX, u32::MAX, u32::MAX); query.len()];
    for (i, a) in query.iter().enumerate() {
        for (j, b) in train.iter().enumerate() {
            let d = distance(a, b);
            if d < best[i].1 {
                best[i] = (j, d, best[i].1);
            } else if d < best[i].2 {
                best[i].2 = d;
            }
            if d < reverse[j].1 {
                reverse[j] = (i, d);
            }
        }
    }
    best.into_iter()
        .enumerate()
        .filter_map(|(i, (j, a, b))| {
            (a <= 50
                && u64::from(a) * 10 < u64::from(b) * 8
                && j < train.len()
                && reverse[j].0 == i)
                .then_some((i, j))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pyramid_matches_preserve_known_scale_geometry() {
        let mut random = 42u64;
        let data = (0..384 * 288)
            .map(|_| {
                random = random
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1);
                (random >> 32) as u8
            })
            .collect();
        let image = Matrix::from_vec(288, 384, 1, data);
        let smaller = purecv::imgproc::resize(&image, purecv::core::Size::new(320, 240)).unwrap();
        let a = detect(&image.data, 384, 288).unwrap();
        let b = detect(&smaller.data, 320, 240).unwrap();
        let pairs = matches(
            &a.iter().map(|p| p.descriptor).collect::<Vec<_>>(),
            &b.iter().map(|p| p.descriptor).collect::<Vec<_>>(),
        );
        let good = pairs
            .iter()
            .filter(|&&(i, j)| {
                (a[i].pixel[0] / 1.2 - b[j].pixel[0]).hypot(a[i].pixel[1] / 1.2 - b[j].pixel[1])
                    < 2.
            })
            .count();
        assert!(
            good >= 30,
            "known-scale inliers={good}, matches={}",
            pairs.len()
        );
        assert!(good * 10 >= pairs.len() * 9);
    }
    #[test]
    fn exact_pixels_row_order_and_ambiguous_matches() {
        let image: Vec<u8> = (0..80 * 80)
            .map(|i| ((i * 17 + i / 80 * 43) % 251) as u8)
            .collect();
        let pixels = [[2., 2.], [40., 40.], [f32::NAN, 40.], [30., 35.]];
        let descriptors = extract(&image, 80, 80, &pixels).unwrap();
        assert!(descriptors[0].is_none() && descriptors[2].is_none());
        let single = extract(&image, 80, 80, &pixels[3..]).unwrap();
        assert_eq!(descriptors[3], single[0]);
        assert!(extract(&image, 79, 80, &pixels).is_err());
        assert_eq!(distance(&[0; 32], &[255; 32]), 256);
        assert_eq!(
            matches(&[[0; 32]], &[[0; 32], [0; 32]]),
            [] as [(usize, usize); 0]
        );
        assert_eq!(matches(&[[0; 32]], &[[255; 32], [0; 32]]), [(0, 1)]);
    }
}
