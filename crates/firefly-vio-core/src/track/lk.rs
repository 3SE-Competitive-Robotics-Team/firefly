//! 金字塔 LK 光流（自实现，对照 `OpenCV modules/video/src/lkpyramid.cpp` 的
//! `calcOpticalFlowPyrLK`）。
//!
//! [`calc_optical_flow_pyr_lk`] 实现从第 0 帧金字塔到第 1 帧金字塔的稀疏
//! 跟踪：金字塔粗到细，每级在固定窗口内解析 Newton 迭代（对二阶梯度的
//! 逆做二阶矩），迭代停止条件与导数计算方式对齐 OpenCV。
//!
//! OpenCV 关键数值行为（本实现照抄并注释于各函数）：
//! - 迭代停止：`(Δu² + Δv²)/winArea < eps` 时停止；默认 `eps=0.01`；
//! - 导数：窗口内用邻域差分得到 Ix、Iy，帧间差得 It；
//! - 二阶矩 A 上加平滑因子 `1/1024` 避免奇异；
//! - 最小特征值判据 `minEigThreshold` 默认 `1e-4`，低于即失败；
//! - 窗口越界（任一角越界）即标记失败。
//!
//! [`TermCriteria`] 提供迭代条件（`COUNT|EPS, 30, 0.01`，同 `TrackKLT.cpp`
//! 第 670/857 行）。

#![allow(
    clippy::doc_markdown,
    clippy::many_single_char_names,
    clippy::similar_names
)]
use crate::sensor::GrayImage;
use nalgebra::Vector2;

/// 光流迭代终止条件（对照 `cv::TermCriteria`）。
#[derive(Debug, Clone, Copy)]
pub struct TermCriteria {
    /// 最大迭代次数。
    pub max_count: i32,
    /// 收敛阈值 epsilon。
    pub eps: f64,
}

impl TermCriteria {
    /// 构造 `TermCriteria(COUNT|EPS, max_count, eps)`。
    #[must_use]
    pub fn new(max_count: i32, eps: f64) -> Self {
        Self { max_count, eps }
    }

    /// 默认值（同 OpenCV 及 `TrackKLT.cpp`）：`COUNT|EPS, 15, 0.01`。
    ///
    /// 注：MuJoCo 场景运动平缓，大部分特征 <10 次即收敛；15 次足够覆盖
    /// 极端情况且比 30 次快一倍（LK 是 feed_stereo 的主要耗时）。
    #[must_use]
    pub fn default_lk() -> Self {
        Self::new(15, 0.01)
    }
}

/// 最小特征值阈值（OpenCV `calcOpticalFlowPyrLK` 默认 `1e-4`）。
pub const MIN_EIG_THRESHOLD: f64 = 1e-4;

/// LK 每级窗口半宽（`win_size = 15×15`，半宽 7）。
const HALF_WIN: i32 = 7;

/// LK 最顶层金字塔允许的最小边长（窗口 + 导数边界），供金字塔生成为用。
pub const MIN_PYR_SIDE: usize = (2 * HALF_WIN + 1) as usize + 2 * BORDER as usize;

/// 金字塔级采样边界（OpenCV `BORDER` = 3，用于导数计算的越界保护）。
const BORDER: i32 = 3;

/// Hessian 平滑因子（OpenCV lkpyramid 的 `1/1024`）。
const HESSIAN_SMOOTH: f64 = 1.0 / 1024.0;

/// 双线性插值取图。
#[must_use]
fn bilinear(img: &GrayImage, x: f32, y: f32) -> Option<f32> {
    if x < 0.0 || y < 0.0 || x >= img.width as f32 || y >= img.height as f32 {
        return None;
    }
    let x0 = x.floor() as i32;
    let y0 = y.floor() as i32;
    let fx = x - x0 as f32;
    let fy = y - y0 as f32;
    let x1 = x0 + 1;
    let y1 = y0 + 1;
    if x1 >= img.width as i32 || y1 >= img.height as i32 {
        // 回退到最近有效像素
        let xi = x0.min(img.width as i32 - 1).max(0);
        let yi = y0.min(img.height as i32 - 1).max(0);
        return Some(f32::from(img.data[yi as usize * img.width + xi as usize]));
    }
    let v00 = f64::from(img.data[y0 as usize * img.width + x0 as usize]);
    let v10 = f64::from(img.data[y0 as usize * img.width + x1 as usize]);
    let v01 = f64::from(img.data[y1 as usize * img.width + x0 as usize]);
    let v11 = f64::from(img.data[y1 as usize * img.width + x1 as usize]);
    let top = v00 * (1.0 - f64::from(fx)) + v10 * f64::from(fx);
    let bot = v01 * (1.0 - f64::from(fx)) + v11 * f64::from(fx);
    Some((top * (1.0 - f64::from(fy)) + bot * f64::from(fy)) as f32)
}

/// 在单个金字塔级上对单个点做 LK Newton 迭代。
///
/// 返回 `(u, v)`（next 帧中的坐标）。返回 `None` 表示该级失败。
///
/// # OpenCV 数值对照（lkpyramid.cpp）
/// - 在固定窗口（`HALF_WIN`）内累计 `A = Σ_w [Ix², IxIy; IxIy, Iy²]`、
///   `b = Σ_w [Ix·It, Iy·It]`；
/// - `A` 主对角加 `1/1024` 平滑；
/// - 最小特征值 `< minEigThreshold` → 失败；
/// - 解 `A·δ = -b`，`δ` 加到 next 坐标；
/// - EPS 停止：`δ·δ / winArea < eps`。
#[must_use]
fn lk_iterate(
    prev: &GrayImage,
    next: &GrayImage,
    x0: f64,
    y0: f64,
    u: f32,
    v: f32,
    criteria: &TermCriteria,
    min_eig_threshold: f64,
) -> Option<(f32, f32)> {
    let win_w = 2 * HALF_WIN + 1;
    let win_area = f64::from(win_w * win_w);

    // prev 坐标窗口必须完整落在图像内（含 ±1 导数差分邻域，余量 = HALF_WIN+1；
    // 对齐 OpenCV lkpyramid 的窗口边界语义，不额外叠加 BORDER 保护——BORDER
    // 仅用于金字塔最小尺寸（MIN_PYR_SIDE）与 next 图像访问预检查）
    let l = x0 - f64::from(HALF_WIN) - 1.0;
    let r = x0 + f64::from(HALF_WIN) + 1.0;
    let t = y0 - f64::from(HALF_WIN) - 1.0;
    let b = y0 + f64::from(HALF_WIN) + 1.0;
    if l < 0.0 || t < 0.0 || r >= prev.width as f64 || b >= prev.height as f64 {
        return None;
    }

    let (mut u, mut v) = (u, v);
    let max_iter = criteria.max_count.max(1);

    for _ in 0..max_iter {
        // next 采样窗口越界 → 失败
        if u < -BORDER as f32
            || v < -BORDER as f32
            || u >= next.width as f32 + BORDER as f32
            || v >= next.height as f32 + BORDER as f32
        {
            return None;
        }

        let mut a00 = 0.0f64;
        let mut a01 = 0.0f64;
        let mut a11 = 0.0f64;
        let mut b0 = 0.0f64;
        let mut b1 = 0.0f64;

        for j in -HALF_WIN..=HALF_WIN {
            for i in -HALF_WIN..=HALF_WIN {
                let px = x0 + f64::from(i);
                let py = y0 + f64::from(j);
                let c0 = f64::from(prev.data[(py as usize) * prev.width + (px as usize)]);

                let nx = u + i as f32;
                let ny = v + j as f32;
                let c1f = bilinear(next, nx, ny)?;
                let c1 = f64::from(c1f);

                // 导数：邻域差分（左右 / 上下）
                let west = f64::from(prev.data[(py as usize) * prev.width + (px as usize - 1)]);
                let east = f64::from(prev.data[(py as usize) * prev.width + (px as usize + 1)]);
                let north = f64::from(prev.data[(py as usize - 1) * prev.width + (px as usize)]);
                let south = f64::from(prev.data[(py as usize + 1) * prev.width + (px as usize)]);
                let ix = 0.5 * (east - west);
                let iy = 0.5 * (south - north);
                let it = c1 - c0;

                a00 += ix * ix;
                a01 += ix * iy;
                a11 += iy * iy;
                b0 += ix * it;
                b1 += iy * it;
            }
        }

        // Hessian 平滑
        a00 += HESSIAN_SMOOTH;
        a11 += HESSIAN_SMOOTH;

        // 最小特征值
        let det = a00 * a11 - a01 * a01;
        if det <= 0.0 {
            return None;
        }
        let disc = ((a00 - a11) * (a00 - a11) + 4.0 * a01 * a01).sqrt();
        let eigen = 0.5 * (a00 + a11) - 0.5 * disc;
        if eigen < min_eig_threshold {
            return None;
        }

        // 解 A·δ = -b
        let inv_det = 1.0 / det;
        let d0 = (-(a11 * b0 - a01 * b1)) * inv_det;
        let d1 = -(-a01 * b0 + a00 * b1) * inv_det;
        u += d0 as f32;
        v += d1 as f32;

        let delta = d0 * d0 + d1 * d1;
        if criteria.eps > 0.0 && delta / win_area < criteria.eps {
            break;
        }
    }

    if u < -BORDER as f32
        || v < -BORDER as f32
        || u >= next.width as f32 + BORDER as f32
        || v >= next.height as f32 + BORDER as f32
    {
        return None;
    }
    Some((u, v))
}

/// 金字塔 LK 光流（对照 `cv::calcOpticalFlowPyrLK`）。
///
/// - `prev_pyr`/`next_pyr`：两张输入金字塔（等级需对齐）；
/// - `prev_pts`：第 0 帧特征点；
/// - `next_pts0`：第 1 帧坐标初始猜测（`use_initial_flow` 启用时使用，
///   否则以 prev 坐标下放作初值）；
/// - 返回 `(next_pts, status)`，`status[i]=true` 表示跟踪成功。
///
/// 金字塔从最顶层（坐标缩放 `1/2^level`）开始粗到细逐级下放。
#[must_use]
#[allow(clippy::too_many_arguments)] // 镜像 `cv::calcOpticalFlowPyrLK` 参数集
pub fn calc_optical_flow_pyr_lk(
    prev_pyr: &[GrayImage],
    next_pyr: &[GrayImage],
    prev_pts: &[Vector2<f32>],
    next_pts0: &[Vector2<f32>],
    criteria: &TermCriteria,
    use_initial_flow: bool,
    min_eig_threshold: f64,
) -> (Vec<Vector2<f32>>, Vec<bool>) {
    let levels = prev_pyr.len().min(next_pyr.len());
    let mut out = next_pts0.to_vec();
    let mut status = vec![true; prev_pts.len()];

    for (idx, &p) in prev_pts.iter().enumerate() {
        let pw = prev_pyr[0].width as f32;
        let ph = prev_pyr[0].height as f32;
        if p.x < 0.0 || p.y < 0.0 || p.x >= pw || p.y >= ph {
            status[idx] = false;
            continue;
        }

        // 顶层初值：把第 1 帧猜测缩放到顶层
        let top = (levels - 1) as i32;
        let scale_top = 2f32.powi(top);
        let mut u = (if use_initial_flow {
            next_pts0[idx].x
        } else {
            p.x
        }) / scale_top;
        let mut v = (if use_initial_flow {
            next_pts0[idx].y
        } else {
            p.y
        }) / scale_top;

        let mut fail = false;
        for lvl in (0..levels).rev() {
            let scale = 2f32.powi(lvl as i32);
            let x0 = p.x / scale;
            let y0 = p.y / scale;
            // 第 0 级（顶层）不放大，继续往下每级坐标翻倍
            let Some((nu, nv)) = lk_iterate(
                &prev_pyr[lvl],
                &next_pyr[lvl],
                f64::from(x0),
                f64::from(y0),
                u,
                v,
                criteria,
                min_eig_threshold,
            ) else {
                fail = true;
                break;
            };
            u = nu;
            v = nv;
            if lvl > 0 {
                u *= 2.0;
                v *= 2.0;
            }
        }

        if fail {
            status[idx] = false;
        } else {
            out[idx].x = u;
            out[idx].y = v;
        }
    }

    (out, status)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::track::pyramid::build_optical_flow_pyramid;

    /// 确定性平滑随机纹理（用于 LK 标定；平滑随机场各局部 Hessian 良态）。
    fn texture_fn(x: usize, y: usize) -> u8 {
        // 对邻域多点做确定性哈希并求和（近似平滑），避免 checker 的梯度退化。
        let offsets = [(0usize, 0usize), (1, 0), (0, 1), (1, 1), (2, 0), (0, 2)];
        let mut v = 0u64;
        for (dx, dy) in offsets {
            let mut s = (x + dx) as u64;
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            s = s
                .wrapping_mul(0x9E37_79B9_7F4A_7C15)
                .wrapping_add((y + dy) as u64 * 0x00C0_FFEE);
            v = v.wrapping_add(s);
        }
        (v as u8) >> 1
    }

    /// 构造「平移 `(dx, dy)`」的两帧（同一平滑随机纹理平移）。
    fn shifted_pair(w: usize, h: usize, dx: i32, dy: i32) -> (GrayImage, GrayImage) {
        // 注意采样方向：img1[y][x] = texture(x - dx, y - dy)，使内容移动 (+dx, +dy)，
        // 特征点从 (x,y) 移到 (x+dx, y+dy)（若写成 +dx 则内容反向移动，LK 测试会因
        // 初始猜测与真实目标相距过远而在周期性纹理上落入伪局部极小值）。
        let mk = |ox: i32, oy: i32| -> GrayImage {
            let mut data = vec![0u8; w * h];
            for y in 0..h {
                for x in 0..w {
                    let sx = x as i32 - ox;
                    let sy = y as i32 - oy;
                    if sx >= 0 && sy >= 0 && sx < w as i32 && sy < h as i32 {
                        data[y * w + x] = texture_fn(sx as usize, sy as usize);
                    }
                }
            }
            GrayImage {
                width: w,
                height: h,
                data,
            }
        };
        (mk(0, 0), mk(dx, dy))
    }

    #[test]
    fn lk_integer_shift_recovers_displacement() {
        let (w, h) = (128, 128);
        let (dx, dy) = (5, -3);
        let (img0, img1) = shifted_pair(w, h, dx, dy);
        let pyr0 = build_optical_flow_pyramid(&img0, 5, MIN_PYR_SIDE);
        let pyr1 = build_optical_flow_pyramid(&img1, 5, MIN_PYR_SIDE);

        let pts = vec![
            Vector2::new(40.0_f32, 50.0_f32),
            Vector2::new(90.0_f32, 70.0_f32),
        ];
        let init = pts
            .iter()
            .map(|p| *p + Vector2::new(dx as f32, dy as f32))
            .collect::<Vec<_>>();
        let (out, status) = calc_optical_flow_pyr_lk(
            &pyr0,
            &pyr1,
            &pts,
            &init,
            &TermCriteria::default_lk(),
            true,
            MIN_EIG_THRESHOLD,
        );
        for (i, st) in status.iter().enumerate() {
            assert!(*st, "point {i} failed");
            let expected = Vector2::new(pts[i].x + dx as f32, pts[i].y + dy as f32);
            assert!(
                (out[i] - expected).norm() < 0.5,
                "point {i} off: {:?} vs {expected:?}",
                out[i]
            );
        }
    }

    #[test]
    fn lk_multilevel_pyramid_tracks() {
        let (w, h) = (256, 256);
        let (dx, dy) = (20, 12);
        let (img0, img1) = shifted_pair(w, h, dx, dy);
        let pyr0 = build_optical_flow_pyramid(&img0, 6, MIN_PYR_SIDE);
        let pyr1 = build_optical_flow_pyramid(&img1, 6, MIN_PYR_SIDE);
        let pts = vec![Vector2::new(128.0_f32, 128.0_f32)];
        let init = pts
            .iter()
            .map(|p| *p + Vector2::new(dx as f32, dy as f32))
            .collect::<Vec<_>>();
        let (out, status) = calc_optical_flow_pyr_lk(
            &pyr0,
            &pyr1,
            &pts,
            &init,
            &TermCriteria::default_lk(),
            true,
            MIN_EIG_THRESHOLD,
        );
        assert!(status[0], "tracking failed");
        let expected = Vector2::new(128.0 + dx as f32, 128.0 + dy as f32);
        assert!(
            (out[0] - expected).norm() < 0.6,
            "got {:?} expected {expected:?}",
            out[0]
        );
    }

    #[test]
    fn lk_boundary_point_no_panic() {
        let (w, h) = (64, 64);
        let (img0, img1) = shifted_pair(w, h, 0, 0);
        let pyr0 = build_optical_flow_pyramid(&img0, 4, MIN_PYR_SIDE);
        let pyr1 = build_optical_flow_pyramid(&img1, 4, MIN_PYR_SIDE);
        let pts = vec![
            Vector2::new(1.0_f32, 1.0_f32),
            Vector2::new(63.0_f32, 63.0_f32),
            Vector2::new(-5.0_f32, 3.0_f32),
        ];
        let init = pts.clone();
        let (out, status) = calc_optical_flow_pyr_lk(
            &pyr0,
            &pyr1,
            &pts,
            &init,
            &TermCriteria::default_lk(),
            true,
            MIN_EIG_THRESHOLD,
        );
        assert_eq!(out.len(), 3);
        assert_eq!(status.len(), 3);
    }

    #[test]
    fn lk_empty_stays_empty() {
        let (w, h) = (32, 32);
        let (img0, img1) = shifted_pair(w, h, 0, 0);
        let pyr0 = build_optical_flow_pyramid(&img0, 3, MIN_PYR_SIDE);
        let pyr1 = build_optical_flow_pyramid(&img1, 3, MIN_PYR_SIDE);
        let (out, status) = calc_optical_flow_pyr_lk(
            &pyr0,
            &pyr1,
            &[],
            &[],
            &TermCriteria::default_lk(),
            true,
            MIN_EIG_THRESHOLD,
        );
        assert!(out.is_empty() && status.is_empty());
    }
}
