//! 相机模型（对照 `OpenVINS` `ov_core/src/cam`：`CamBase`/`CamRadtan`/`CamEqui`）。
//!
//! 本模块提供针孔（pinhole）相机模型的抽象 [`CameraModel`] trait 与两个具体
//! 实现：
//! - [`CamRadtan`]：pinhole-radtan（Brown–Conrady 径向+切向畸变），对应 `CamRadtan.h`；
//! - [`CamEqui`]：pinhole-equi（等距鱼眼），对应 `CamEqui.h`。
//!
//! 内参向量（8 元素，顺序同 OpenVINS `set_value`）：
//! `[f_x, f_y, c_x, c_y, k_1, k_2, k_3, k_4]`。
//!
//! ## undistort 的数值行为
//!
//! `undistort_*` 复刻 OpenCV 的数值方案（OpenVINS 的 `undistort_f` 直接调用
//! `cv::undistortPoints` / `cv::fisheye::undistortPoints`）：
//! - `CamRadtan`：复刻 `OpenCV modules/imgproc/src/undistort.cpp` 中
//!   `undistortPoints` 对 pinhole 模型的固定点迭代反解；
//! - `CamEqui`：复刻 `OpenCV modules/calib3d/src/fisheye.cpp` 中
//!   `undistortPoints` 的 Newton 迭代。
//!
//! 两处数值行为细节见各 `undistort_f` 的文档注释。
//!
//! 说明：数学/几何转录代码中单字符标量（`x`,`y`,`r`,`s`）与类名文档引用
//! （`OpenCV`/`OpenVINS`）属于固有风格，予以模块级允许。
#![allow(
    clippy::doc_markdown,
    clippy::many_single_char_names,
    clippy::similar_names
)]

use nalgebra::{Matrix2, Matrix3, Vector2};
use std::sync::Arc;

/// 内参向量的长度（`[f_x f_y c_x c_y k_1 k_2 k_3 k_4]`）。
pub const CALIB_LEN: usize = 8;

/// 2×8 的「畸变后像素对 8 个内参」雅可比。
pub type CalibJacobian = nalgebra::SMatrix<f64, 2, 8>;

/// 相机模型抽象（对照 `CamBase.h` 的公开接口）。
///
/// 所有实现均为针孔模型，仅畸变建模不同。原始像素坐标 `uv_dist` 经
/// `undistort_*` → 归一化坐标 `uv_norm`；归一化坐标经 `distort_*` →
/// 原始像素坐标。C++ 用 `float` 处用 `f32`、用 `double` 处用 `f64`
/// （`undistort_f`/`distort_f` 是 `float` 接口）。
pub trait CameraModel: std::fmt::Debug + Send + Sync {
    /// 更新并校验相机内参（必须正好 8 个元素：
    /// `[f_x f_y c_x c_y k_1 k_2 k_3 k_4]`）。
    ///
    /// 对照 `CamBase::set_value`。
    /// # Panics
    /// 当 `calib.len() != 8` 时 panic（与 C++ 的 `assert` 一致）。
    fn set_value(&mut self, calib: &[f64]);

    /// 给定原始像素坐标 `uv_dist`，去畸变为归一化坐标（`float` 接口）。
    ///
    /// 见 `CamBase::undistort_f`。
    fn undistort_f(&self, uv_dist: Vector2<f32>) -> Vector2<f32>;

    /// 给定原始像素坐标 `uv_dist`，去畸变为归一化坐标（`double` 接口）。
    ///
    /// C++ 语义：先 `cast<float>` 后调用 `undistort_f`，再 `cast<double>`
    /// （即内部以 `float` 精度计算）。见 `CamBase::undistort_d`。
    #[must_use]
    fn undistort_d(&self, uv_dist: Vector2<f64>) -> Vector2<f64> {
        let out = self.undistort_f(uv_dist.cast::<f32>());
        out.cast::<f64>()
    }

    /// 给定归一化坐标 `uv_norm`，畸变为原始像素坐标（`float` 接口）。
    ///
    /// 见 `CamBase::distort_f`。
    fn distort_f(&self, uv_norm: Vector2<f32>) -> Vector2<f32>;

    /// 给定归一化坐标 `uv_norm`，畸变为原始像素坐标（`double` 接口）。
    ///
    /// C++ 语义：先 `cast<float>` 后调用 `distort_f`，再 `cast<double>`。
    /// 见 `CamBase::distort_d`。
    #[must_use]
    fn distort_d(&self, uv_norm: Vector2<f64>) -> Vector2<f64> {
        let out = self.distort_f(uv_norm.cast::<f32>());
        out.cast::<f64>()
    }

    /// 计算「畸变后像素对归一化坐标」的雅可比，以及对 8 个内参的雅可比。
    ///
    /// 对照 `CamBase::compute_distort_jacobian`：
    /// - 返回 `.0 = ∂z/∂zn`（2×2）；
    /// - 返回 `.1 = ∂z/∂ζ`（2×8，ζ 为内参向量）。
    fn compute_distort_jacobian(&self, uv_norm: Vector2<f64>) -> (Matrix2<f64>, CalibJacobian);

    /// 归一化坐标 → 原始像素（投影），并附带对归一化坐标的 2×2 雅可比。
    ///
    /// 等价于 `distort_f` + `compute_distort_jacobian().0`。OpenVINS `CamBase.h`
    /// 未直接暴露 `project`（仅靠 `distort_f` + 雅可比），此为惯用便捷封装。
    #[must_use]
    fn project(&self, uv_norm: Vector2<f32>) -> (Vector2<f32>, Matrix2<f64>) {
        let uv = self.distort_f(uv_norm);
        let jac = self.compute_distort_jacobian(uv_norm.cast()).0;
        (uv, jac)
    }

    /// 图像宽度（原始像素）。
    fn width(&self) -> usize;

    /// 图像高度（原始像素）。
    fn height(&self) -> usize;

    /// 原始内参向量（`[f_x f_y c_x c_y k_1 k_2 k_3 k_4]`）。
    fn value(&self) -> [f64; CALIB_LEN];

    /// 相机内参矩阵 `K = [[f_x,0,c_x],[0,f_y,c_y],[0,0,1]]`
    /// （对照 `CamBase::get_K`）。
    #[must_use]
    fn camera_matrix(&self) -> Matrix3<f64> {
        let v = self.value();
        Matrix3::new(v[0], 0.0, v[2], 0.0, v[1], v[3], 0.0, 0.0, 1.0)
    }
}

/// 共享相机句柄（跟踪器内部用 `Arc<dyn CameraModel>` 保存多相机标定）。
pub type SharedCamera = Arc<dyn CameraModel>;

/// 内参容器：持有 8 个内参值，供两种畸变模型共享（避免重复字段）。
#[derive(Debug, Clone, Copy, PartialEq)]
struct Intrinsics {
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    k: [f64; 4],
}

impl Intrinsics {
    fn from_slice(calib: &[f64]) -> Self {
        assert!(
            calib.len() == CALIB_LEN,
            "camera calibration must have exactly {CALIB_LEN} values, got {}",
            calib.len()
        );
        // 内参顺序 [fx fy cx cy k1 k2 k3 k4]
        Self {
            fx: calib[0],
            fy: calib[1],
            cx: calib[2],
            cy: calib[3],
            k: [calib[4], calib[5], calib[6], calib[7]],
        }
    }

    fn as_array(&self) -> [f64; CALIB_LEN] {
        [
            self.fx, self.fy, self.cx, self.cy, self.k[0], self.k[1], self.k[2], self.k[3],
        ]
    }

    /// 把原始像素坐标换算为「归一化的畸变坐标」`((u-cx)/fx, (v-cy)/fy)`。
    #[must_use]
    fn normalized_point(&self, uv: Vector2<f32>) -> Vector2<f64> {
        Vector2::new(
            f64::from(uv.x - self.cx as f32) / self.fx,
            f64::from(uv.y - self.cy as f32) / self.fy,
        )
    }

    /// 归一化坐标经内参映射到原始像素。
    #[must_use]
    fn pixel_from_norm(&self, n: Vector2<f64>) -> Vector2<f32> {
        Vector2::new(
            (self.fx * n.x + self.cx) as f32,
            (self.fy * n.y + self.cy) as f32,
        )
    }
}

/// pinhole-radtan（Brown–Conrady）相机模型。
///
/// 畸变闭式公式（对照 `CamRadtan.h`）：
/// ```text
/// x = x_n(1 + k1 r² + k2 r⁴) + 2 p1 x_n y_n + p2 (r² + 2 x_n²)
/// y = y_n(1 + k1 r² + k2 r⁴) + p1 (r² + 2 y_n²) + 2 p2 x_n y_n
/// u = f_x x + c_x , v = f_y y + c_y      (r² = x_n² + y_n²)
/// ```
/// 内参顺序 `[fx fy cx cy k1 k2 k3 k4]` 中 `k3`/`k4` 即 OpenCV 的切向项
/// `p1`/`p2`，径向只到 `k2`（四阶）。`CamRadtan.h` 中 `cam_d(6)`→`p1`、
/// `cam_d(7)`→`p2`。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CamRadtan {
    intrinsics: Intrinsics,
    width: usize,
    height: usize,
}

impl CamRadtan {
    /// 构造：`width`/`height` 为原始图像尺寸，`calib` 为 8 个内参。
    #[must_use]
    pub fn new(width: usize, height: usize, calib: &[f64]) -> Self {
        let intrinsics = Intrinsics::from_slice(calib);
        Self {
            intrinsics,
            width,
            height,
        }
    }
}

impl CameraModel for CamRadtan {
    fn set_value(&mut self, calib: &[f64]) {
        self.intrinsics = Intrinsics::from_slice(calib);
    }

    /// 去畸变：复刻 `cv::undistortPoints`（pinhole，无 R/P 的归一化分支）的
    /// 固定点迭代反解，来源 `OpenCV modules/imgproc/src/undistort.cpp`：
    ///
    /// 1. 用内参把原始像素归一化到「畸变空间」`(x0,y0)=((u-cx)/fx,(v-cy)/fy)`；
    /// 2. 令 `(x,y)=(x0,y0)` 作初值，按默认
    ///    `TermCriteria(COUNT+EPS, 5, DBL_EPSILON)` 固定迭代 5 次：
    ///    ```text
    ///    r² = x² + y²
    ///    icdist = 1 / (1 + k1 r² + k2 r⁴)      // pinhole 径向仅 k1,k2
    ///    dx = 2 p1xy + p2 (r² + 2x²)
    ///    dy = p1 (r² + 2y²) + 2 p2xy
    ///    x = (x0 - dx)·icdist
    ///    y = (y0 - dy)·icdist
    ///    ```
    ///
    /// OpenVINS `CamRadtan.h` 直接把单点交给 `cv::undistortPoints`；此处以精确
    /// 等价数值复刻（阈值为 `DBL_EPSILON` 在 `f32` 分量下几乎永假，故等价于
    /// 固定 5 次迭代）。
    fn undistort_f(&self, uv_dist: Vector2<f32>) -> Vector2<f32> {
        let p = self.intrinsics;
        let n0 = p.normalized_point(uv_dist);
        let (x0, y0) = (n0.x, n0.y);
        let (k1, k2, p1, p2) = (p.k[0], p.k[1], p.k[2], p.k[3]);
        let (mut x, mut y) = (x0, y0);
        for _ in 0..5 {
            let r2 = x * x + y * y;
            let r4 = r2 * r2;
            let icdist = 1.0 / (1.0 + k1 * r2 + k2 * r4);
            let delta_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            let delta_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
            x = (x0 - delta_x) * icdist;
            y = (y0 - delta_y) * icdist;
        }
        Vector2::new(x as f32, y as f32)
    }

    /// 畸变闭式公式（对照 `CamRadtan.h::distort_f`），输出去畸变前的像素坐标。
    fn distort_f(&self, uv_norm: Vector2<f32>) -> Vector2<f32> {
        let p = self.intrinsics;
        let xn = f64::from(uv_norm.x);
        let yn = f64::from(uv_norm.y);
        let r2 = xn * xn + yn * yn;
        let r4 = r2 * r2;
        let (k1, k2, p1, p2) = (p.k[0], p.k[1], p.k[2], p.k[3]);
        let x1 = xn * (1.0 + k1 * r2 + k2 * r4) + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn);
        let y1 = yn * (1.0 + k1 * r2 + k2 * r4) + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn;
        p.pixel_from_norm(Vector2::new(x1, y1))
    }

    fn compute_distort_jacobian(&self, uv_norm: Vector2<f64>) -> (Matrix2<f64>, CalibJacobian) {
        let p = self.intrinsics;
        let (x, y) = (uv_norm.x, uv_norm.y);
        let r2 = x * x + y * y;
        let r4 = r2 * r2;
        let x2 = x * x;
        let y2 = y * y;
        let xy = x * y;
        let (k1, k2, p1, p2) = (p.k[0], p.k[1], p.k[2], p.k[3]);

        // ∂z/∂zn（对照 CamRadtan.h 的 H_dz_dzn）
        let j00 = p.fx
            * ((1.0 + k1 * r2 + k2 * r4)
                + (2.0 * k1 * x2 + 4.0 * k2 * x2 * r2)
                + 2.0 * p1 * y
                + 6.0 * p2 * x);
        let j01 = p.fx * (2.0 * k1 * xy + 4.0 * k2 * xy * r2 + 2.0 * p1 * x + 2.0 * p2 * y);
        let j10 = p.fy * (2.0 * k1 * xy + 4.0 * k2 * xy * r2 + 2.0 * p1 * x + 2.0 * p2 * y);
        let j11 = p.fy
            * ((1.0 + k1 * r2 + k2 * r4)
                + (2.0 * k1 * y2 + 4.0 * k2 * y2 * r2)
                + 6.0 * p1 * y
                + 2.0 * p2 * x);
        let dz_dzn = Matrix2::new(j00, j01, j10, j11);

        // 畸变后归一化解（对照 CamRadtan.h 第 179-182 行）
        let x1 = x * (1.0 + k1 * r2 + k2 * r4) + 2.0 * p1 * xy + p2 * (r2 + 2.0 * x2);
        let y1 = y * (1.0 + k1 * r2 + k2 * r4) + p1 * (r2 + 2.0 * y2) + 2.0 * p2 * xy;

        // ∂z/∂ζ（2×8，列序 [fx fy cx cy k1 k2 k3=p1 k4=p2]；对照 H_dz_dzeta）
        let mut dz_dzeta = CalibJacobian::zeros();
        dz_dzeta[(0, 0)] = x1;
        dz_dzeta[(0, 2)] = 1.0;
        dz_dzeta[(0, 4)] = p.fx * x * r2;
        dz_dzeta[(0, 5)] = p.fx * x * r4;
        dz_dzeta[(0, 6)] = 2.0 * p.fx * xy;
        dz_dzeta[(0, 7)] = p.fx * (r2 + 2.0 * x2);
        dz_dzeta[(1, 1)] = y1;
        dz_dzeta[(1, 3)] = 1.0;
        dz_dzeta[(1, 4)] = p.fy * y * r2;
        dz_dzeta[(1, 5)] = p.fy * y * r4;
        dz_dzeta[(1, 6)] = p.fy * (r2 + 2.0 * y2);
        dz_dzeta[(1, 7)] = 2.0 * p.fy * xy;

        (dz_dzn, dz_dzeta)
    }

    fn width(&self) -> usize {
        self.width
    }

    fn height(&self) -> usize {
        self.height
    }

    fn value(&self) -> [f64; CALIB_LEN] {
        self.intrinsics.as_array()
    }
}

/// pinhole-equi（等距鱼眼）相机模型。
///
/// 畸变闭式公式（对照 `CamEqui.h`）：
/// ```text
/// x = (x_n/r)·θ_d , y = (y_n/r)·θ_d
/// θ_d = θ(1 + k1 θ² + k2 θ⁴ + k3 θ⁶ + k4 θ⁸)
/// r² = x_n² + y_n² , θ = atan(r)
/// u = f_x x + c_x , v = f_y y + c_y
/// ```
/// 当 `r ≤ 1e-8` 时按 `θ_d/r → 1` 处理（中心点无缩放），同 `CamEqui.h`。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CamEqui {
    intrinsics: Intrinsics,
    width: usize,
    height: usize,
}

impl CamEqui {
    /// 构造：`width`/`height` 为原始图像尺寸，`calib` 为 8 个内参。
    #[must_use]
    pub fn new(width: usize, height: usize, calib: &[f64]) -> Self {
        let intrinsics = Intrinsics::from_slice(calib);
        Self {
            intrinsics,
            width,
            height,
        }
    }
}

impl CameraModel for CamEqui {
    fn set_value(&mut self, calib: &[f64]) {
        self.intrinsics = Intrinsics::from_slice(calib);
    }

    /// 去畸变：复刻 `cv::fisheye::undistortPoints`（无 R/P 归一化分支）的
    /// Newton 迭代，来源 `OpenCV modules/calib3d/src/fisheye.cpp`：
    ///
    /// 以归一化畸变坐标 `(x0,y0)=((u-cx)/fx,(v-cy)/fy)` 为初值，固定 10 次
    /// Newton 迭代求解满足 `distort(mx,my) = (x0,y0)` 的 `(mx,my)`。每次：
    /// ```text
    /// r      = sqrt(mx² + my²)
    /// θ      = atan(r)
    /// θ_d    = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸)
    /// s      = θ_d / r                      // r 甚小时 s→1
    /// gx     = s·mx - x0 ,  gy = s·my - y0  // 残差
    /// J      = ∂(gx,gy)/∂(mx,my)            // 2×2 解析雅可比
    /// (mx,my) -=  J⁻¹ · (gx,gy)
    /// ```
    ///
    /// OpenCV fisheye `undistortPoints` 的默认 `TermCriteria(MAX_ITER+EPS,
    /// 100, FLT_EPSILON)` 中，`FLT_EPSILON` 对常规像素过大/过小不定，实际主循环
    /// 以 Newton 步 `(J⁻¹g)` 的模长判停；本实现固定 10 次迭代并在残差模长
    /// 小于 `1e-8` 时提前收敛（等价且更稳）。单点 `f32` 语义与 C++ 一致。
    fn undistort_f(&self, uv_dist: Vector2<f32>) -> Vector2<f32> {
        let p = self.intrinsics;
        let n0 = p.normalized_point(uv_dist);
        let mut mx = n0.x;
        let mut my = n0.y;
        let (k1, k2, k3, k4) = (p.k[0], p.k[1], p.k[2], p.k[3]);
        for _ in 0..10 {
            let r = (mx * mx + my * my).sqrt();
            if r < 1e-8 {
                break;
            }
            let th = r.atan();
            let th2 = th * th;
            let th4 = th2 * th2;
            let th6 = th4 * th2;
            let th8 = th4 * th4;
            let thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
            let s = thd / r;

            let gx = s * mx - n0.x;
            let gy = s * my - n0.y;
            // J = ∂(s·m)/∂m，链式展开（r² 而非 r）
            let dash = 1.0 + 3.0 * k1 * th2 + 5.0 * k2 * th4 + 7.0 * k3 * th6 + 9.0 * k4 * th8;
            let drdx = mx / r;
            let drdy = my / r;
            // ∂s/∂r 由 θ_d 与 r 的关系导出
            let dsdr = (dash - s) / r;
            let dgx_dmx = s + mx * dsdr * drdx;
            let dgx_dmy = mx * dsdr * drdy;
            let dgy_dmx = my * dsdr * drdx;
            let dgy_dmy = s + my * dsdr * drdy;
            let det = dgx_dmx * dgy_dmy - dgx_dmy * dgy_dmx;
            if det.abs() < 1e-16 {
                break;
            }
            let inv = 1.0 / det;
            let dx = (dgy_dmy * gx - dgy_dmx * gy) * inv; // 注意到 2×2 逆
            let dy = (-dgx_dmy * gx + dgx_dmx * gy) * inv;
            mx -= dx;
            my -= dy;
            if (dx * dx + dy * dy) < 1e-16 {
                break;
            }
        }
        Vector2::new(mx as f32, my as f32)
    }

    /// 畸变闭式公式（对照 `CamEqui.h::distort_f`），输出去畸变前的像素坐标。
    fn distort_f(&self, uv_norm: Vector2<f32>) -> Vector2<f32> {
        let p = self.intrinsics;
        let xn = f64::from(uv_norm.x);
        let yn = f64::from(uv_norm.y);
        let r = (xn * xn + yn * yn).sqrt();
        let theta = r.atan();
        let theta_d = theta
            + p.k[0] * theta.powi(3)
            + p.k[1] * theta.powi(5)
            + p.k[2] * theta.powi(7)
            + p.k[3] * theta.powi(9);
        // r 甚小时 cdist→1
        let inv_r = if r > 1e-8 { 1.0 / r } else { 1.0 };
        let cdist = if r > 1e-8 { theta_d * inv_r } else { 1.0 };
        let x1 = xn * cdist;
        let y1 = yn * cdist;
        p.pixel_from_norm(Vector2::new(x1, y1))
    }

    fn compute_distort_jacobian(&self, uv_norm: Vector2<f64>) -> (Matrix2<f64>, CalibJacobian) {
        let p = self.intrinsics;
        // 对照 CamEqui.h compute_distort_jacobian
        let (x, y) = (uv_norm.x, uv_norm.y);
        let r = (x * x + y * y).sqrt();
        let theta = r.atan();
        let theta_d = theta
            + p.k[0] * theta.powi(3)
            + p.k[1] * theta.powi(5)
            + p.k[2] * theta.powi(7)
            + p.k[3] * theta.powi(9);
        let inv_r = if r > 1e-8 { 1.0 / r } else { 1.0 };
        let cdist = if r > 1e-8 { theta_d * inv_r } else { 1.0 };
        let x1 = x * cdist;
        let y1 = y * cdist;

        let duv_dxy = Matrix2::new(p.fx, 0.0, 0.0, p.fy);
        let dxy_dxyn = Matrix2::new(theta_d * inv_r, 0.0, 0.0, theta_d * inv_r);
        let dxy_dr =
            nalgebra::Vector2::new(-x * theta_d * inv_r * inv_r, -y * theta_d * inv_r * inv_r);
        let dr_dxyn = nalgebra::RowVector2::new(x * inv_r, y * inv_r);
        let dxy_dthd = nalgebra::Vector2::new(x * inv_r, y * inv_r);
        let dthd_dth = 1.0
            + 3.0 * p.k[0] * theta.powi(2)
            + 5.0 * p.k[1] * theta.powi(4)
            + 7.0 * p.k[2] * theta.powi(6)
            + 9.0 * p.k[3] * theta.powi(8);
        let dth_dr = 1.0 / (r * r + 1.0);
        let inner = dxy_dxyn + (dxy_dr + dxy_dthd * (dthd_dth * dth_dr)) * dr_dxyn;
        let dz_dzn = duv_dxy * inner;

        let mut dz_dzeta = CalibJacobian::zeros();
        dz_dzeta[(0, 0)] = x1;
        dz_dzeta[(0, 2)] = 1.0;
        dz_dzeta[(0, 4)] = p.fx * x * inv_r * theta.powi(3);
        dz_dzeta[(0, 5)] = p.fx * x * inv_r * theta.powi(5);
        dz_dzeta[(0, 6)] = p.fx * x * inv_r * theta.powi(7);
        dz_dzeta[(0, 7)] = p.fx * x * inv_r * theta.powi(9);
        dz_dzeta[(1, 1)] = y1;
        dz_dzeta[(1, 3)] = 1.0;
        dz_dzeta[(1, 4)] = p.fy * y * inv_r * theta.powi(3);
        dz_dzeta[(1, 5)] = p.fy * y * inv_r * theta.powi(5);
        dz_dzeta[(1, 6)] = p.fy * y * inv_r * theta.powi(7);
        dz_dzeta[(1, 7)] = p.fy * y * inv_r * theta.powi(9);

        (dz_dzn, dz_dzeta)
    }

    fn width(&self) -> usize {
        self.width
    }

    fn height(&self) -> usize {
        self.height
    }

    fn value(&self) -> [f64; CALIB_LEN] {
        self.intrinsics.as_array()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn radtan_calib() -> [f64; CALIB_LEN] {
        [600.0, 599.0, 320.0, 240.0, -0.2, 0.05, 0.001, -0.001]
    }

    fn equi_calib() -> [f64; CALIB_LEN] {
        [300.0, 300.0, 320.0, 240.0, -0.01, 0.0, 0.0, 0.0]
    }

    #[test]
    fn radtan_distort_undistort_roundtrip() {
        let cam = CamRadtan::new(640, 480, &radtan_calib());
        for (ux, uy) in [
            (320.0, 240.0),
            (400.0, 300.0),
            (300.0, 200.0),
            (500.0, 260.0),
            (200.0, 400.0),
        ] {
            let uv = Vector2::new(ux as f32, uy as f32);
            let n = cam.undistort_f(uv);
            let back = cam.distort_f(n);
            let err = (back - uv).norm();
            assert!(err < 1e-3, "roundtrip err={err} at ({ux},{uy})");
        }
    }

    #[test]
    fn radtan_distort_matches_closed_form() {
        let cam = CamRadtan::new(640, 480, &radtan_calib());
        let n = Vector2::new(0.3f32, -0.2f32);
        let uv = cam.distort_f(n);
        let (fx, fy, cx, cy) = (600.0, 599.0, 320.0, 240.0);
        let (k1, k2, p1, p2) = (-0.2, 0.05, 0.001, -0.001);
        let (x, y) = (0.3_f64, -0.2_f64);
        let r2 = x * x + y * y;
        let r4 = r2 * r2;
        let x1 = x * (1.0 + k1 * r2 + k2 * r4) + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let y1 = y * (1.0 + k1 * r2 + k2 * r4) + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        assert!((uv.x - (fx * x1 + cx) as f32).abs() < 1e-4);
        assert!((uv.y - (fy * y1 + cy) as f32).abs() < 1e-4);
    }

    #[test]
    fn radtan_identity_distortion() {
        let cam = CamRadtan::new(640, 480, &[600.0, 600.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0]);
        let n = Vector2::new(0.1f32, 0.2f32);
        let uv = cam.distort_f(n);
        assert!((uv.x - 380.0_f32).abs() < 1e-4);
        assert!((uv.y - 360.0_f32).abs() < 1e-4);
        let back = cam.undistort_f(uv);
        assert!((back - n).norm() < 1e-5);
    }

    #[test]
    fn equi_distort_undistort_roundtrip() {
        let cam = CamEqui::new(640, 480, &equi_calib());
        for (ux, uy) in [
            (320.0, 240.0),
            (350.0, 270.0),
            (300.0, 210.0),
            (500.0, 350.0),
        ] {
            let uv = Vector2::new(ux as f32, uy as f32);
            let n = cam.undistort_f(uv);
            let back = cam.distort_f(n);
            let err = (back - uv).norm();
            assert!(err < 1e-2, "equi roundtrip err={err} at ({ux},{uy})");
        }
    }

    #[test]
    fn equi_distort_matches_closed_form() {
        let cam = CamEqui::new(640, 480, &equi_calib());
        let n = Vector2::new(0.2f32, 0.1f32);
        let uv = cam.distort_f(n);
        let (fx, fy, cx, cy) = (300.0, 300.0, 320.0, 240.0);
        let k = equi_calib();
        let (x, y) = (0.2_f64, 0.1_f64);
        let r = (x * x + y * y).sqrt();
        let theta = r.atan();
        let theta_d = theta + k[4] * theta.powi(3);
        let cdist = theta_d / r;
        assert!((uv.x - (fx * x * cdist + cx) as f32).abs() < 1e-4);
        assert!((uv.y - (fy * y * cdist + cy) as f32).abs() < 1e-4);
    }

    #[test]
    fn equi_center_no_scaling() {
        let cam = CamEqui::new(640, 480, &equi_calib());
        let uv = cam.distort_f(Vector2::new(0.0f32, 0.0f32));
        assert!((uv - Vector2::new(320.0_f32, 240.0_f32)).norm() < 1e-4);
    }

    #[test]
    fn set_value_rejects_wrong_length() {
        let mut cam = CamRadtan::new(640, 480, &[0.0; 8]);
        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            cam.set_value(&[0.0; 4]);
        }));
        assert!(r.is_err());
    }

    #[test]
    fn project_matches_distort() {
        let cam = CamRadtan::new(640, 480, &radtan_calib());
        let n = Vector2::new(0.4f32, -0.3f32);
        let (uv, jac) = cam.project(n);
        assert!((uv - cam.distort_f(n)).norm() < 1e-6);
        let (hjac, _) = cam.compute_distort_jacobian(n.cast());
        assert!((jac - hjac).norm() < 1e-10);
    }
}
