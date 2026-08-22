//! 前端回归：FAST/NMS/LK 在合成点阵上的行为锚点。
//!
//! 背景（2026-08 调试）：purecv NMS 为 8 邻域严格 `>` 比较——常值亮度
//! 平台（纯色方块、规则点阵）会整片互斥归零；真实帧因传感器噪声极少
//! 同分，故仅影响极端合成图。LK 已知位移回归防止光流方向/步长回退。

use firefly_vio_core::sensor::GrayImage;
use firefly_vio_core::track::fast::fast_with_score;
use firefly_vio_core::track::{grider, lk, pyramid};

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
    let kps = fast_with_score(&img, 10, false).0;
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
    let with_nms = fast_with_score(&img, 10, true).0;
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
    let p0 = pyramid::build_optical_flow_pyramid(
        &GrayImage {
            width: w,
            height: h,
            data: img0,
        },
        3,
        lk::MIN_PYR_SIDE,
    );
    let p1 = pyramid::build_optical_flow_pyramid(
        &GrayImage {
            width: w,
            height: h,
            data: img1,
        },
        3,
        lk::MIN_PYR_SIDE,
    );
    let prev: Vec<nalgebra::Vector2<f32>> = pts0
        .iter()
        .map(|&(x, y)| nalgebra::Vector2::new(x, y))
        .collect();
    let init = prev.clone();
    let (out, status) = lk::calc_optical_flow_pyr_lk(
        &p0,
        &p1,
        &prev,
        &init,
        &lk::TermCriteria::default_lk(),
        true,
        lk::MIN_EIG_THRESHOLD,
    );
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

/// 分层诊断：全图 FAST → 单格 ROI FAST → grider，定位检出断点。
#[test]
fn grider_stage_diagnosis() {
    let (img, _) = dotted_image(320, 240, true);
    // 全图
    let full = fast_with_score(&img, 10, true).0;
    println!("stage1 全图NMS={}", full.len());
    // 单格 ROI（cell 64×48）：取含点的一格
    // 复制 crop_roi 逻辑（私有函数，测试内联同款实现）
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
    let roi_kps = fast_with_score(&roi, 10, true).0;
    println!("stage2 ROI NMS={}", roi_kps.len());
    let roi_no = fast_with_score(&roi, 10, false).0;
    println!("stage2b ROI 无NMS={}", roi_no.len());
}

/// 小位移(-0.9,-0.9)+帧间独立噪声：复现现场"跟踪反向滞后"现象。
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
    // 真实观测：真值 +0.15px/帧向右，跟踪却 -0.9px/帧向左
    let img0 = mk(1, (0.0, 0.0));
    let img1 = mk(2, (-0.9, -0.9));
    let p0 = pyramid::build_optical_flow_pyramid(&img0, 5, lk::MIN_PYR_SIDE);
    let p1 = pyramid::build_optical_flow_pyramid(&img1, 5, lk::MIN_PYR_SIDE);
    let prev = vec![nalgebra::Vector2::new(171.0f32, 117.0f32)];
    let init = prev.clone();
    let (out, status) = lk::calc_optical_flow_pyr_lk(
        &p0,
        &p1,
        &prev,
        &init,
        &lk::TermCriteria::default_lk(),
        true,
        lk::MIN_EIG_THRESHOLD,
    );
    println!(
        "小位移(-0.9,-0.9): out=({:.2},{:.2}) status={}",
        out[0].x - prev[0].x,
        out[0].y - prev[0].y,
        status[0]
    );
}

/// 最终探针：不同小位移 × 有无帧间噪声，量化 purecv LK 系统偏差。
/// 真值位移已知，输出「测量-真值」误差向量；多 seed 取均值分离系统项。
#[test]
fn lk_systematic_error_sweep() {
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
    // 注意：亚像素部分用面积近似不可行，这里仅测整数+半像素级位移；
    // 点阵渲染为整像素方块，亚像素真值需重心插值——超出本探针范围。
    let shifts: [(f64, f64); 6] = [
        (0.0, 0.0),
        (1.0, 1.0),
        (2.0, -1.0),
        (-1.0, 2.0),
        (-2.0, -2.0),
        (4.0, 2.0),
    ];
    println!("位移(px)      | 无噪误差px | 有噪(amp7)误差px");
    println!("--------------+------------+-----------------");
    for (dx, dy) in shifts {
        let mut err_clean = Vec::new();
        for trial in 0..4u64 {
            // 平移点阵整体位置避免同一初始点集的系统效应
            let off = trial as f64 * 1.0;
            let img0 = draw_all((off, off), trial as usize * 2 + 1);
            let img1 = draw_all((off + dx, off + dy), trial as usize * 2 + 2);
            let p0 = pyramid::build_optical_flow_pyramid(&img0, 5, lk::MIN_PYR_SIDE);
            let p1 = pyramid::build_optical_flow_pyramid(&img1, 5, lk::MIN_PYR_SIDE);
            let prev: Vec<nalgebra::Vector2<f32>> = base_dots
                .iter()
                .map(|&(x, y)| nalgebra::Vector2::new(x + off as f32, y + off as f32))
                .collect();
            let init = prev.clone();
            let (out, status) = lk::calc_optical_flow_pyr_lk(
                &p0,
                &p1,
                &prev,
                &init,
                &lk::TermCriteria::default_lk(),
                true,
                lk::MIN_EIG_THRESHOLD,
            );
            let e: f64 = out
                .iter()
                .zip(prev.iter())
                .enumerate()
                .filter(|(i, _)| status[*i])
                .map(|(_, (o, p))| {
                    (((f64::from(o.x) - f64::from(p.x)) - dx).powi(2)
                        + ((f64::from(o.y) - f64::from(p.y)) - dy).powi(2))
                    .sqrt()
                })
                .sum::<f64>()
                / status.iter().filter(|s| **s).count().max(1) as f64;
            err_clean.push(e);
        }
        let ec = err_clean.iter().sum::<f64>() / err_clean.len() as f64;
        println!("({dx:+.1},{dy:+.1})   | {ec:.3}      |");
    }
}
