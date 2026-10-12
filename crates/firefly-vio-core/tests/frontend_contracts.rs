//! 前端回归：FAST/NMS/LK 在合成点阵上的行为锚点。
//!
//! FAST NMS 使用八邻域严格 `>` 比较，常值亮度平台的等分角点会互相抑制。
//! LK 测试使用已知位移检查光流方向和幅度。

use firefly_vio_core::sensor::GrayImage;
use firefly_vio_core::track::fast::detect;
use firefly_vio_core::track::{grider, pyramid};

/// purecv 金字塔 LK 测试配置：2 层金字塔、15×15 窗口、initial flow。
fn lk_flow(
    prev: &GrayImage,
    next: &GrayImage,
    prev_pts: &[nalgebra::Vector2<f32>],
    init_pts: &[nalgebra::Vector2<f32>],
) -> (Vec<nalgebra::Vector2<f32>>, Vec<bool>) {
    use purecv::core::types::{Point2f, Size2i, TermCriteria, TermType};
    let p0 = pyramid::gray_to_matrix(prev);
    let p1 = pyramid::gray_to_matrix(next);
    let p0f: Vec<Point2f> = prev_pts.iter().map(|p| Point2f::new(p.x, p.y)).collect();
    let p1f: Vec<Point2f> = init_pts.iter().map(|p| Point2f::new(p.x, p.y)).collect();
    let (out, status, _) = purecv::video::calc_optical_flow_pyramid_lk(
        &p0,
        &p1,
        &p0f,
        Some(&p1f),
        Size2i::new(15, 15),
        1,
        TermCriteria::new(TermType::Both, 30, 0.01),
        purecv::video::OPTFLOW_USE_INITIAL_FLOW,
        1e-4,
    )
    .expect("purecv LK 不应失败");
    (
        out.into_iter()
            .map(|p| nalgebra::Vector2::new(p.x, p.y))
            .collect(),
        status.into_iter().map(|s| s != 0).collect(),
    )
}

fn dotted_image(w: usize, h: usize, level_jitter: bool) -> (GrayImage, Vec<(f32, f32)>) {
    let mut data = vec![0u8; w * h];
    let mut dots = Vec::new();
    let mut n = 0usize;
    for yi in 1..6 {
        for xi in 1..8 {
            let cx = xi * 40;
            let cy = yi * 40;
            // 亮度差异化：NMS 严格 > 比较下同分角点互相抑制
            let level = if level_jitter {
                150u8 + ((n * 37) % 100) as u8
            } else {
                200
            };
            n += 1;
            for dy in -2i32..=2 {
                for dx in -2i32..=2 {
                    data[(cy + dy) as usize * w + (cx + dx) as usize] = level;
                }
            }
            dots.push((cx as f32, cy as f32));
        }
    }
    // 噪声最后叠加：若先加噪再被方块覆盖，角点仍处常值平台（NMS 归零）
    for (idx, d) in data.iter_mut().enumerate() {
        *d = d.saturating_add(((idx.wrapping_mul(2_654_435_761)) >> 24) as u8 % 7);
    }
    (
        GrayImage {
            width: w,
            height: h,
            data,
        },
        dots,
    )
}

/// 无 NMS 时点阵全部可检（检测器本身健康）。
#[test]
fn fast_without_nms_detects_all_dots() {
    let (img, dots) = dotted_image(320, 240, true);
    let kps = detect(&pyramid::gray_to_matrix(&img), 10, false);
    assert!(
        kps.len() >= dots.len(),
        "检出 {} 少于点数 {}",
        kps.len(),
        dots.len()
    );
}

/// 无噪声的常值平台图：NMS 等分互斥归零（记录 purecv 行为锚点）。
#[test]
fn fast_nms_tie_plateau_suppresses_all() {
    // 纯色方块、无任何噪声：所有角点得分完全相同
    let mut data = vec![0u8; 320 * 240];
    for yi in 1..6 {
        for xi in 1..8 {
            for dy in -2i32..=2 {
                for dx in -2i32..=2 {
                    data[(yi * 40 + dy) as usize * 320 + (xi * 40 + dx) as usize] = 200;
                }
            }
        }
    }
    let img = GrayImage {
        width: 320,
        height: 240,
        data,
    };
    let with_nms = detect(&pyramid::gray_to_matrix(&img), 10, true);
    assert_eq!(
        with_nms.len(),
        0,
        "等分平台的 NMS 行为发生变化，请更新本锚点"
    );
}

/// grider 路径在差异化亮度点阵上可检出。
#[test]
fn grider_extracts_on_varied_dots() {
    let (img, _) = dotted_image(320, 240, true);
    let valid: Vec<(i32, i32)> = (0..5).flat_map(|x| (0..5).map(move |y| (x, y))).collect();
    let zero_mask = GrayImage {
        width: 320,
        height: 240,
        data: vec![0u8; 320 * 240],
    };
    let got = grider::perform_griding(&img, &zero_mask, &valid, 200, 5, 5, 10, true);
    assert!(!got.is_empty(), "grider 零检出");
}

/// LK 对已知整数位移的跟踪精度（防零位移/反向回退）。
#[test]
fn lk_tracks_known_shift() {
    let w = 320usize;
    let h = 240usize;
    let draw = |data: &mut Vec<u8>, cx: f32, cy: f32| {
        for dy in -2i32..=2 {
            for dx in -2i32..=2 {
                data[(cy as i32 + dy) as usize * w + (cx as i32 + dx) as usize] = 200;
            }
        }
    };
    let mut img0 = vec![0u8; w * h];
    let mut img1 = vec![0u8; w * h];
    let pts0 = [
        (60.0f32, 60.0),
        (150.0, 80.0),
        (240.0, 120.0),
        (90.0, 180.0),
    ];
    for &(x, y) in &pts0 {
        draw(&mut img0, x, y);
        draw(&mut img1, x + 4.0, y + 2.0);
    }
    let img0_g = GrayImage {
        width: w,
        height: h,
        data: img0,
    };
    let img1_g = GrayImage {
        width: w,
        height: h,
        data: img1,
    };
    let prev: Vec<nalgebra::Vector2<f32>> = pts0
        .iter()
        .map(|&(x, y)| nalgebra::Vector2::new(x, y))
        .collect();
    let init = prev.clone();
    let (out, status) = lk_flow(&img0_g, &img1_g, &prev, &init);
    for i in 0..pts0.len() {
        assert!(status[i], "点 {i} 跟踪失败");
        let du = out[i].x - prev[i].x;
        let dv = out[i].y - prev[i].y;
        assert!(
            (du - 4.0).abs() < 1.0 && (dv - 2.0).abs() < 1.0,
            "点 {i} 位移 ({du},{dv}) ≠ (4,2)"
        );
    }
}

/// 全图与含角点的 ROI 均可检测，NMS 只从候选中筛选。
#[test]
fn fast_roi_nms_preserves_candidate_coordinates() {
    let (img, _) = dotted_image(320, 240, true);
    let full = detect(&pyramid::gray_to_matrix(&img), 10, true);
    assert!(!full.is_empty(), "全图必须检出角点");
    // 单格 ROI（cell 64×48）：取含点的一格
    let (cx0, cy0, cw, ch) = (40usize, 40usize, 64usize, 48usize);
    let mut roi_data = Vec::with_capacity(cw * ch);
    for yy in cy0..cy0 + ch {
        let row = yy * 320;
        roi_data.extend_from_slice(&img.data[row + cx0..row + cx0 + cw]);
    }
    let roi = GrayImage {
        width: cw,
        height: ch,
        data: roi_data,
    };
    let roi_kps = detect(&pyramid::gray_to_matrix(&roi), 10, true);
    assert!(!roi_kps.is_empty(), "含角点 ROI 必须检出角点");
    let roi_no = detect(&pyramid::gray_to_matrix(&roi), 10, false);
    assert!(roi_kps.len() <= roi_no.len());
    for point in &roi_kps {
        assert!(
            roi_no
                .iter()
                .any(|candidate| candidate.x.to_bits() == point.x.to_bits()
                    && candidate.y.to_bits() == point.y.to_bits())
        );
    }
}

/// 一像素负向位移与帧间噪声下，每个坐标误差小于 0.1 像素。
#[test]
fn lk_small_shift_with_flicker_noise() {
    use firefly_vio_core::sensor::GrayImage;
    let w = 320usize;
    let h = 240usize;
    let draw = |data: &mut Vec<u8>, cx: f32, cy: f32| {
        for dy in -2i32..=2 {
            for dx in -2i32..=2 {
                data[(cy as i32 + dy) as usize * w + (cx as i32 + dx) as usize] = 200;
            }
        }
    };
    let mk = |seed: usize, shift: (f32, f32)| -> GrayImage {
        let mut data = vec![0u8; w * h];
        for &(x, y) in &[
            (171.0f32, 117.0f32),
            (60.0, 60.0),
            (150.0, 80.0),
            (240.0, 120.0),
        ] {
            draw(&mut data, x + shift.0, y + shift.1);
        }
        for (i, d) in data.iter_mut().enumerate() {
            *d = d.saturating_add((((i.wrapping_mul(2_654_435_761)) >> 24) + seed * 7) as u8 % 7);
        }
        GrayImage {
            width: w,
            height: h,
            data,
        }
    };
    let img0 = mk(1, (0.0, 0.0));
    let img1 = mk(2, (-1.0, -1.0));
    let prev = vec![nalgebra::Vector2::new(171.0f32, 117.0f32)];
    let init = prev.clone();
    let (out, status) = lk_flow(&img0, &img1, &prev, &init);
    assert!(status[0], "含噪小位移必须保持跟踪");
    let error = out[0] - prev[0] - nalgebra::Vector2::new(-1.0, -1.0);
    assert!(
        error.x.abs() < 0.1 && error.y.abs() < 0.1,
        "误差 {error:?} px"
    );
}

/// 六种整数位移、四种起始位置与帧间噪声下，全点跟踪误差小于 0.1 像素。
#[test]
fn lk_integer_shift_sweep_preserves_tracks_and_accuracy() {
    use firefly_vio_core::sensor::GrayImage;
    let w = 320usize;
    let h = 240usize;
    let base_dots: Vec<(f32, f32)> = (1..6)
        .flat_map(|yi| (1..8).map(move |xi| ((xi * 40) as f32, (yi * 40) as f32)))
        .collect();
    let draw_all = |shift: (f64, f64), seed: usize| -> GrayImage {
        let mut data = vec![0u8; w * h];
        for &(cx, cy) in &base_dots {
            let cxf = cx + shift.0 as f32;
            let cyf = cy + shift.1 as f32;
            for dy in -2i32..=2 {
                for dx in -2i32..=2 {
                    data[(cyf as i32 + dy) as usize * w + (cxf as i32 + dx) as usize] = 200;
                }
            }
        }
        for (i, d) in data.iter_mut().enumerate() {
            let h = (i as u64).wrapping_mul(2_654_435_761) ^ (seed as u64).wrapping_mul(40503);
            *d = d.saturating_add(((h >> 24) & 0x7) as u8);
        }
        GrayImage {
            width: w,
            height: h,
            data,
        }
    };
    // 方块按整数像素绘制，因此只验证整数真值位移。
    let shifts: [(f64, f64); 6] = [
        (0.0, 0.0),
        (1.0, 1.0),
        (2.0, -1.0),
        (-1.0, 2.0),
        (-2.0, -2.0),
        (4.0, 2.0),
    ];
    for (dx, dy) in shifts {
        for trial in 0..4u64 {
            // 平移点阵整体位置避免同一初始点集的系统效应
            let off = trial as f64 * 1.0;
            let img0 = draw_all((off, off), trial as usize * 2 + 1);
            let img1 = draw_all((off + dx, off + dy), trial as usize * 2 + 2);
            let prev: Vec<nalgebra::Vector2<f32>> = base_dots
                .iter()
                .map(|&(x, y)| nalgebra::Vector2::new(x + off as f32, y + off as f32))
                .collect();
            let init = prev.clone();
            let (out, status) = lk_flow(&img0, &img1, &prev, &init);
            assert_eq!(out.len(), prev.len());
            assert_eq!(status.len(), prev.len());
            for (i, (point, previous)) in out.iter().zip(&prev).enumerate() {
                assert!(status[i], "位移 ({dx},{dy}) trial={trial} 点 {i} 丢失");
                let error = (f64::from(point.x - previous.x) - dx)
                    .hypot(f64::from(point.y - previous.y) - dy);
                assert!(
                    error < 0.1,
                    "位移 ({dx},{dy}) trial={trial} 点 {i} 误差 {error} px"
                );
            }
        }
    }
}
