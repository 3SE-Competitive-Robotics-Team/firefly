//! 几何驱动的双目深度退化：视差量化、相关破面与边缘位置抖动。
//! 模型依据及未建模的物理因素见 `docs/depth_sensor.md`；默认值不是实机标定。

use firefly_error::{Error, ErrorKind, Result};
use rand::{RngExt, SeedableRng, rngs::StdRng};
use serde::Deserialize;

/// 通用双目代理模型；深度为光轴 Z（米），无效测量统一为零。
#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DepthNoiseOptions {
    /// 是否施加退化；关闭时仍执行有效量程裁剪。
    pub enabled: bool,
    /// 深度测量等效基线，米；默认与仿真双目 5cm 一致。
    pub baseline_m: f32,
    /// 有效光轴深度下限，米；0.2m 为未标定的仿真设定。
    pub min_depth_m: f32,
    /// 有效光轴深度上限，米；8m 与在线建图感知上限一致。
    pub max_depth_m: f32,
    /// 总视差标准差，像素；0.08 为通用仿真设定，参照 D400 亚像素噪声量级。
    pub disparity_sigma_px: f32,
    /// 视差量化步长，像素；1/32 参照 `RealSense` D400 文档。
    pub disparity_step_px: f32,
    /// 视差方差中空间/时间相关部分的比例，无量纲 [0,1]。
    pub correlated_fraction: f32,
    /// 横向采样抖动标准差，像素；0.5 参照 `SimKinect`。
    pub lateral_sigma_px: f32,
    /// 相关高斯场格点间距，像素；8px 为仿真设定，非块状硬分割。
    pub patch_size_px: usize,
    /// 高斯场时间格点间隔，秒；0.4s 为仿真设定，不复用历史深度。
    pub correlation_time_s: f64,
    /// 相关场中随时间变化的方差比例；2% 保留静态缺陷形状，仅边界轻微闪动。
    pub temporal_fraction: f32,
    /// 平面基础丢点概率；默认 3% 为仿真设定。
    pub hole_probability: f32,
    /// 量程上限处额外丢点概率，按 (z/max)^2 插值；默认 20% 为仿真设定。
    pub far_hole_probability: f32,
    /// 几何边缘额外失效概率；默认 65% 为仿真设定。
    pub edge_hole_probability: f32,
    /// 几何边缘检测半径，像素；默认 2px 为仿真设定。
    pub edge_radius_px: usize,
    /// 深度跳变绝对门限，米；与相对门限取较大值。
    pub edge_threshold_m: f32,
    /// 深度跳变相对门限，无量纲；默认 4% 为仿真设定。
    pub edge_threshold_ratio: f32,
    /// 相同种子、测量时间与几何产生相同输出，不依赖任务排队顺序。
    pub seed: u64,
}

impl Default for DepthNoiseOptions {
    fn default() -> Self {
        Self {
            enabled: true,
            baseline_m: 0.05,
            min_depth_m: 0.2,
            max_depth_m: 8.0,
            disparity_sigma_px: 0.08,
            disparity_step_px: 1.0 / 32.0,
            correlated_fraction: 0.5,
            lateral_sigma_px: 0.5,
            patch_size_px: 8,
            correlation_time_s: 0.4,
            temporal_fraction: 0.02,
            hole_probability: 0.03,
            far_hole_probability: 0.2,
            edge_hole_probability: 0.65,
            edge_radius_px: 2,
            edge_threshold_m: 0.12,
            edge_threshold_ratio: 0.04,
            seed: 20_261_002,
        }
    }
}

impl DepthNoiseOptions {
    pub fn validate(&self) -> Result<()> {
        let positive = [
            self.baseline_m,
            self.min_depth_m,
            self.max_depth_m,
            self.edge_threshold_m,
        ];
        let nonnegative = [
            self.disparity_sigma_px,
            self.disparity_step_px,
            self.lateral_sigma_px,
            self.edge_threshold_ratio,
        ];
        let probabilities = [
            self.correlated_fraction,
            self.temporal_fraction,
            self.hole_probability,
            self.far_hole_probability,
            self.edge_hole_probability,
        ];
        if positive.iter().any(|v| !v.is_finite() || *v <= 0.0)
            || nonnegative.iter().any(|v| !v.is_finite() || *v < 0.0)
            || probabilities
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
            || self.min_depth_m < crate::sensors::DEPTH_MIN
            || self.min_depth_m >= self.max_depth_m
            || self.max_depth_m >= 100.0
            || !(1..=128).contains(&self.patch_size_px)
            || self.edge_radius_px > 8
            || !self.correlation_time_s.is_finite()
            || self.correlation_time_s < 0.001
        {
            return Err(
                Error::new(ErrorKind::InvalidArgument, "invalid depth noise parameters")
                    .with_context("operation", "validate render.depth_noise"),
            );
        }
        Ok(())
    }

    fn valid(&self, z: f32) -> bool {
        z.is_finite() && z >= self.min_depth_m && z <= self.max_depth_m
    }
}

fn gaussian(rng: &mut StdRng) -> f32 {
    let u = rng.random::<f32>().max(f32::EPSILON);
    (-2.0 * u.ln()).sqrt() * (std::f32::consts::TAU * rng.random::<f32>()).cos()
}

/// 双线性/时间插值后按权重平方和归一化，保持各点方差为一。
struct Field {
    values: Vec<f32>,
    cols: usize,
    scale: usize,
}

impl Field {
    fn new(
        width: usize,
        height: usize,
        options: &DepthNoiseOptions,
        stamp: f64,
        channel: u64,
    ) -> Self {
        let scale = options.patch_size_px;
        let cols = width / scale + 2;
        let rows = height / scale + 2;
        let t = stamp / options.correlation_time_s;
        let epoch = t.floor() as u64;
        let a = (t - t.floor()) as f32;
        let norm = ((1.0 - a).powi(2) + a * a).sqrt();
        let seed = |e: u64| {
            options.seed
                ^ channel.wrapping_mul(0x9e37_79b9_7f4a_7c15)
                ^ e.wrapping_mul(0xbf58_476d_1ce4_e5b9)
        };
        let mut left = StdRng::seed_from_u64(seed(epoch));
        let mut right = StdRng::seed_from_u64(seed(epoch.wrapping_add(1)));
        let mut fixed = StdRng::seed_from_u64(seed(u64::MAX));
        let values = (0..cols * rows)
            .map(|_| {
                let dynamic = ((1.0 - a) * gaussian(&mut left) + a * gaussian(&mut right)) / norm;
                (1.0 - options.temporal_fraction).sqrt() * gaussian(&mut fixed)
                    + options.temporal_fraction.sqrt() * dynamic
            })
            .collect();
        Self {
            values,
            cols,
            scale,
        }
    }

    #[allow(clippy::many_single_char_names)]
    fn at(&self, x: usize, y: usize) -> f32 {
        let u = (x % self.scale) as f32 / self.scale as f32;
        let v = (y % self.scale) as f32 / self.scale as f32;
        let i = (y / self.scale) * self.cols + x / self.scale;
        let weights = [(1.0 - u) * (1.0 - v), u * (1.0 - v), (1.0 - u) * v, u * v];
        let values = [
            self.values[i],
            self.values[i + 1],
            self.values[i + self.cols],
            self.values[i + self.cols + 1],
        ];
        weights.iter().zip(values).map(|(w, z)| w * z).sum::<f32>()
            / weights.iter().map(|w| w * w).sum::<f32>().sqrt()
    }
}

/// 标准正态 CDF，Abramowitz–Stegun 26.2.17（绝对误差约 1e-7）。
fn normal_cdf(x: f32) -> f32 {
    let t = 1.0 / (1.0 + 0.231_641_9 * x.abs());
    let tail = 0.398_942_3
        * (-0.5 * x * x).exp()
        * t
        * (0.319_381_54
            + t * (-0.356_563_78 + t * (1.781_477_9 + t * (-1.821_255_9 + t * 1.330_274_5))));
    if x >= 0.0 { 1.0 - tail } else { tail }
}

fn measured_depth(z: f32, fb: f32, error: f32, step: f32) -> f32 {
    let disparity = fb / z + error;
    let quantized = if step > 0.0 {
        (disparity / step).round() * step
    } else {
        disparity
    };
    if quantized > 0.0 { fb / quantized } else { 0.0 }
}

/// 原地生成测量；从干净几何检测边缘，空洞不填补、不转换为无穷远自由射线。
#[fastrace::trace]
pub fn apply(
    depth: &mut [f32],
    width: usize,
    height: usize,
    focal_px: f32,
    stamp: f64,
    options: &DepthNoiseOptions,
) {
    assert_eq!(depth.len(), width * height);
    assert!(stamp.is_finite() && stamp >= 0.0 && focal_px.is_finite() && focal_px > 0.0);
    for z in depth.iter_mut() {
        if !options.valid(*z) {
            *z = 0.0;
        }
    }
    if !options.enabled {
        return;
    }
    let clean = depth.to_vec();
    let holes = Field::new(width, height, options, stamp, 1);
    let axial = Field::new(width, height, options, stamp, 2);
    let lateral_x = Field::new(width, height, options, stamp, 3);
    let lateral_y = Field::new(width, height, options, stamp, 4);
    let mut rng = StdRng::seed_from_u64(options.seed ^ stamp.to_bits().rotate_left(17));
    let fb = focal_px * options.baseline_m;
    for y in 0..height {
        for x in 0..width {
            let i = y * width + x;
            let z = clean[i];
            if z == 0.0 {
                continue;
            }
            let threshold = options
                .edge_threshold_m
                .max(options.edge_threshold_ratio * z);
            let r = options.edge_radius_px;
            let edge = (y.saturating_sub(r)..=(y + r).min(height - 1)).any(|ny| {
                (x.saturating_sub(r)..=(x + r).min(width - 1)).any(|nx| {
                    let neighbor = clean[ny * width + nx];
                    neighbor == 0.0 || (neighbor - z).abs() > threshold
                })
            });
            let base = (options.hole_probability
                + options.far_hole_probability * (z / options.max_depth_m).powi(2))
            .min(1.0);
            let probability = if edge {
                1.0 - (1.0 - base) * (1.0 - options.edge_hole_probability)
            } else {
                base
            };
            if probability >= 1.0 || normal_cdf(holes.at(x, y)) < probability {
                depth[i] = 0.0;
                continue;
            }
            // 最近邻跨边界采样，避免线性插值凭空生成前后景之间的表面。
            let sx = (x as f32 + options.lateral_sigma_px * lateral_x.at(x, y)).round() as isize;
            let sy = (y as f32 + options.lateral_sigma_px * lateral_y.at(x, y)).round() as isize;
            if sx < 0 || sy < 0 || sx >= width as isize || sy >= height as isize {
                depth[i] = 0.0;
                continue;
            }
            let sample = clean[sy as usize * width + sx as usize];
            if sample == 0.0 {
                depth[i] = 0.0;
                continue;
            }
            let error = options.disparity_sigma_px
                * (options.correlated_fraction.sqrt() * axial.at(x, y)
                    + (1.0 - options.correlated_fraction).sqrt() * gaussian(&mut rng));
            let measured = measured_depth(sample, fb, error, options.disparity_step_px);
            depth[i] = if options.valid(measured) {
                measured
            } else {
                0.0
            };
        }
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    fn isolated() -> DepthNoiseOptions {
        DepthNoiseOptions {
            hole_probability: 0.0,
            far_hole_probability: 0.0,
            edge_hole_probability: 0.0,
            lateral_sigma_px: 0.0,
            disparity_sigma_px: 0.0,
            disparity_step_px: 0.0,
            ..DepthNoiseOptions::default()
        }
    }

    #[test]
    fn disparity_formula_quantization_and_derivative() {
        // f=160px、B=0.05m、z=2m → d=4px；+0.25px → z=32/17m。
        let z = measured_depth(2.0, 8.0, 0.25, 0.0);
        assert!((z - 32.0 / 17.0).abs() < 1e-6);
        assert!((measured_depth(2.0, 8.0, 0.02, 1.0 / 32.0) - 256.0 / 129.0).abs() < 1e-6);
        assert_eq!(measured_depth(2.0, 8.0, -4.0, 0.0), 0.0);
        let h = 0.001;
        let derivative =
            (measured_depth(2.0, 8.0, h, 0.0) - measured_depth(2.0, 8.0, -h, 0.0)) / (2.0 * h);
        assert!((derivative - (-2.0_f32.powi(2) / 8.0)).abs() < 1e-4);
        for (x, expected) in [(0.0, 0.5), (1.0, 0.841_344_7), (-2.0, 0.022_750_13)] {
            assert!((normal_cdf(x) - expected).abs() < 2e-7);
        }
    }

    #[test]
    fn axial_noise_scales_quadratically_with_range() {
        let options = DepthNoiseOptions {
            disparity_sigma_px: 0.01,
            correlated_fraction: 0.0,
            ..isolated()
        };
        let rms = |z: f32| {
            let mut image = vec![z; 256 * 256];
            apply(&mut image, 256, 256, 160.0, 1.0, &options);
            (image.iter().map(|v| (*v - z).powi(2)).sum::<f32>() / image.len() as f32).sqrt()
        };
        let near = rms(1.0);
        let far = rms(2.0);
        // σ_z ≈ z² σ_d/(fB)，固定种子 65536 样本允许 3% 统计误差。
        assert!((near / 0.00125 - 1.0).abs() < 0.03);
        assert!((far / 0.005 - 1.0).abs() < 0.03);
        assert!((far / near - 4.0).abs() < 0.03);
    }

    #[test]
    fn holes_form_correlated_patches_and_persist_in_measurement_time() {
        let options = DepthNoiseOptions {
            hole_probability: 0.2,
            ..isolated()
        };
        let make = |stamp| {
            let mut image = vec![2.0; 256 * 256];
            apply(&mut image, 256, 256, 160.0, stamp, &options);
            image
        };
        let image = make(1.0);
        let next = make(1.01);
        let future = make(4.0);
        let holes = image.iter().filter(|v| **v == 0.0).count();
        assert!((0.14..0.26).contains(&(holes as f32 / image.len() as f32)));
        let mut adjacent = 0;
        let mut eligible = 0;
        for y in 0..256 {
            for x in 0..255 {
                if image[y * 256 + x] == 0.0 {
                    eligible += 1;
                    adjacent += usize::from(image[y * 256 + x + 1] == 0.0);
                }
            }
        }
        // 独立 20% 丢点的条件概率为 0.2；相关破面应显著高于它。
        assert!(adjacent as f32 / eligible as f32 > 0.7);
        let overlap = |other: &[f32]| {
            image
                .iter()
                .zip(other)
                .filter(|(a, b)| **a == 0.0 && **b == 0.0)
                .count() as f32
                / holes as f32
        };
        assert!(overlap(&next) > 0.9);
        assert!(overlap(&future) > 0.75);
        assert_eq!(image, make(1.0));
    }

    #[test]
    fn clean_geometry_edges_drive_dropout_including_invalid_silhouette() {
        let options = DepthNoiseOptions {
            edge_hole_probability: 1.0,
            ..isolated()
        };
        let mut image = vec![2.0; 64 * 64];
        for y in 0..64 {
            for x in 32..64 {
                image[y * 64 + x] = 4.0;
            }
        }
        apply(&mut image, 64, 64, 160.0, 1.0, &options);
        assert_eq!(image[20 * 64 + 29], 2.0);
        assert_eq!(image[20 * 64 + 34], 4.0);
        assert!(image[20 * 64 + 30..20 * 64 + 34].iter().all(|v| *v == 0.0));
        let mut silhouette = vec![2.0; 64 * 64];
        for y in 0..64 {
            for x in 32..64 {
                silhouette[y * 64 + x] = 0.0;
            }
        }
        apply(&mut silhouette, 64, 64, 160.0, 1.0, &options);
        assert_eq!(silhouette[20 * 64 + 29], 2.0);
        assert!(
            silhouette[20 * 64 + 30..20 * 64 + 64]
                .iter()
                .all(|v| *v == 0.0)
        );
    }

    #[test]
    fn lateral_jitter_does_not_invent_intermediate_surfaces() {
        let options = DepthNoiseOptions {
            lateral_sigma_px: 1.0,
            ..isolated()
        };
        let mut image = vec![2.0; 128 * 128];
        for y in 0..128 {
            for x in 64..128 {
                image[y * 128 + x] = 4.0;
            }
        }
        let original = image.clone();
        apply(&mut image, 128, 128, 160.0, 1.0, &options);
        assert!(image.iter().all(|z| [0.0, 2.0, 4.0].contains(z)));
        assert!(image.iter().zip(original).any(|(a, b)| *a > 0.0 && *a != b));
    }

    #[test]
    fn invalid_range_output_and_disabled_model_contract() {
        let mut image = vec![0.0, f32::NAN, f32::INFINITY, -1.0, 0.1, 20.0, 2.0, 4.0];
        let options = DepthNoiseOptions {
            enabled: false,
            ..DepthNoiseOptions::default()
        };
        apply(&mut image, 4, 2, 160.0, 0.0, &options);
        assert_eq!(image, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 4.0]);
        let mut full = vec![4.0; 128 * 128];
        full[0] = 0.0;
        apply(
            &mut full,
            128,
            128,
            160.0,
            0.0,
            &DepthNoiseOptions::default(),
        );
        assert_eq!(full[0], 0.0);
        assert!(
            full.iter()
                .all(|z| *z == 0.0 || z.is_finite() && (0.2..=8.0).contains(z))
        );
    }

    #[test]
    fn configuration_defaults_and_invalid_values() {
        let options: DepthNoiseOptions = toml::from_str("").unwrap();
        options.validate().unwrap();
        for invalid in [
            DepthNoiseOptions {
                correlation_time_s: 0.0,
                ..options
            },
            DepthNoiseOptions {
                disparity_sigma_px: f32::NAN,
                ..options
            },
            DepthNoiseOptions {
                hole_probability: 1.1,
                ..options
            },
            DepthNoiseOptions {
                min_depth_m: 9.0,
                ..options
            },
            DepthNoiseOptions {
                patch_size_px: 0,
                ..options
            },
        ] {
            assert!(invalid.validate().is_err());
        }
        assert!(toml::from_str::<DepthNoiseOptions>("hole_probabilty = 0.1").is_err());
    }
}
