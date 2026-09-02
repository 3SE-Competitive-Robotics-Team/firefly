//! 静态先验平面容器（P11.2：LIOP 式紧耦合先验测量的只读地图侧）。
//!
//! 语义对照参考实现 `~/Projects/liop_prior/Lidar_IMU_Localization/
//! src/loc/map_location.cpp`：先验全局地图是**只读 kdtree**
//! （`kdtreeGlobal`，:303-306 装载后不再增量更新；先验匹配分支
//! :1701-1766 每帧对当前帧 surf 点做 5 近邻 → 平面拟合 → 点面残差）。
//! 本容器是其"离线已拟合成平面"版本：装载即固定（无 register/refit/
//! 成熟演化），每帧查询产出现成平面候选，由测量模型做残差。
//!
//! 存储：平面分两类索引——
//! - **大平面**（半径超过单个根体素，如地面/墙面）：全局 `Vec`，逐点
//!   线性扫描候选（测量模型侧用径向判据 `radius_k·radius` 自然截断，
//!   面数少，代价可忽略）；
//! - **局部平面**（半径 ≤ 根体素边长，如箱体小面）：`VoxelKey` 根体素
//!   哈希索引，与在线 [`crate::voxel::VoxelMap`] 的 `root_at` 查询同构。
//!
//! 坐标系：与在线地图一致的世界系（MuJoCo 场景系）。首帧即全局系，
//! 先验面不需要 `T_map_odom` 状态（不动状态维度，调研 §4.4）。

use std::path::Path;

use firefly_error::{Error, ErrorKind};
use nalgebra::{Matrix3, Matrix6, Vector3};

use crate::options::ROOT_SIZE;
use crate::plane::VoxelPlane;
use crate::voxel::VoxelKey;

/// 大平面阈值：半径超过该值的平面进入全局索引（地面/墙面），否则按
/// 根体素哈希索引。取值 = 根体素边长：显著大于根体素的面不可能只落
/// 在单个体素内，全局扫描在面数少时更直接。
pub const LARGE_PLANE_RADIUS: f64 = ROOT_SIZE;

/// 静态先验平面容器。
#[derive(Debug, Clone, Default)]
pub struct PriorPlaneMap {
    /// 根体素哈希索引（局部平面；一个根体素可有多面——箱体多面共角）。
    roots: std::collections::HashMap<VoxelKey, Vec<usize>>,
    /// 平面池（`roots` 与 `global` 只存池索引）。
    planes: Vec<VoxelPlane>,
    /// 大平面索引（半径 > [`LARGE_PLANE_RADIUS`]，直接指向池）。
    global: Vec<usize>,
}

impl PriorPlaneMap {
    /// 由平面列表构造（装载即固定——先验容器无插入路径）。
    ///
    /// 平面须为 `is_plane == true` 的有效平面。局部平面（半径 ≤
    /// [`LARGE_PLANE_RADIUS`]）登记到其**外接正方盒（center ± radius，
    /// 保守各向同性）覆盖的所有根体素**：面跨体素边界时中心所在体素
    /// 的查询会漏点——与在线图八叉不同，先验面是"离线整体"，粗筛须
    /// 按几何覆盖登记（对照 LIOP `kdtree` 的邻域查询语义：点到面距离由
    /// kNN 半径截断，不按体素归属；精判据径向 `radius_k·radius` 在测量侧）。
    #[must_use]
    pub fn from_planes(planes: Vec<VoxelPlane>) -> Self {
        let mut roots: std::collections::HashMap<VoxelKey, Vec<usize>> =
            std::collections::HashMap::new();
        let mut global = Vec::new();
        for (idx, plane) in planes.iter().enumerate() {
            if plane.radius > LARGE_PLANE_RADIUS {
                global.push(idx);
                continue;
            }
            // 外接正方盒 [center−r, center+r] 的体素键区间
            let lo =
                VoxelKey::from_point(&(plane.center - Vector3::repeat(plane.radius)), ROOT_SIZE);
            let hi =
                VoxelKey::from_point(&(plane.center + Vector3::repeat(plane.radius)), ROOT_SIZE);
            for x in lo.x..=hi.x {
                for y in lo.y..=hi.y {
                    for z in lo.z..=hi.z {
                        roots.entry(VoxelKey { x, y, z }).or_default().push(idx);
                    }
                }
            }
        }
        Self {
            roots,
            planes,
            global,
        }
    }

    /// 平面总数。
    #[must_use]
    pub fn len(&self) -> usize {
        self.planes.len()
    }

    /// 是否为空。
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.planes.is_empty()
    }

    /// 直接访问平面池。
    #[must_use]
    pub fn planes(&self) -> &[VoxelPlane] {
        &self.planes
    }

    /// 点 `p`（世界系）处的候选平面池索引：同根体素的局部平面 + 全部
    /// 大平面（径向/卡方判据由测量侧执行）。
    #[must_use]
    pub fn candidate_indices(&self, p: &Vector3<f64>) -> Vec<usize> {
        let mut out = self.global.clone();
        let key = VoxelKey::from_point(p, ROOT_SIZE);
        if let Some(local) = self.roots.get(&key) {
            out.extend_from_slice(local);
        }
        out
    }

    /// 候选平面引用（[`candidate_indices`] 的借用版本，供测量模型直接消费）。
    #[must_use]
    pub fn candidates_at<'a>(&'a self, p: &Vector3<f64>) -> Vec<&'a VoxelPlane> {
        self.candidate_indices(p)
            .into_iter()
            .map(|i| &self.planes[i])
            .collect()
    }

    /// 由文件装载（文本格式，见 [`to_text`]）。
    ///
    /// # Errors
    /// 文件不可读（`NotFound`）或格式/数值非法（`InvalidArgument`）。
    pub fn load_text(path: impl AsRef<Path>) -> firefly_error::Result<Self> {
        let path = path.as_ref();
        let raw = std::fs::read_to_string(path).map_err(|e| {
            Error::new(ErrorKind::NotFound, "prior map file not found").with_source(e)
        })?;
        Self::parse_text(&raw)
    }

    /// 解析文本格式。
    ///
    /// 行格式（每行一个平面，空格分隔）：
    /// `cx cy cz nx ny nz d var_scale radius npts`
    /// `Σ_nq ≈ var_scale · I₆`（装载时展开为 6×6 各向同性近似）；
    /// `#` 开头为注释，空行跳过。
    ///
    /// # Errors
    /// 行数/数值非法时 `InvalidArgument`。
    pub fn parse_text(raw: &str) -> firefly_error::Result<Self> {
        let mut planes = Vec::new();
        for (lineno, line) in raw.lines().enumerate() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let nums: Vec<f64> = line
                .split_whitespace()
                .map(|s| {
                    s.parse::<f64>().map_err(|_| {
                        Error::new(ErrorKind::InvalidArgument, "prior map 数值非法")
                            .with_context("line", lineno + 1)
                    })
                })
                .collect::<Result<_, _>>()?;
            if nums.len() != 10 {
                return Err(Error::new(
                    ErrorKind::InvalidArgument,
                    "prior map 每行须 10 个数（c n d var_scale radius npts）",
                )
                .with_context("line", lineno + 1));
            }
            let v = |i: usize| nums[i];
            let center = Vector3::new(v(0), v(1), v(2));
            let normal = Vector3::new(v(3), v(4), v(5));
            let d = v(6);
            let var_scale = v(7);
            let radius = v(8);
            let points_count = v(9) as usize;
            if var_scale <= 0.0 || radius <= 0.0 || normal.norm() < 1e-9 {
                return Err(Error::new(
                    ErrorKind::InvalidArgument,
                    "prior map 非法值（var_scale/radius>0，normal 非零）",
                )
                .with_context("line", lineno + 1));
            }
            planes.push(VoxelPlane {
                center,
                normal: normal.normalize(),
                d,
                plane_var: Matrix6::identity() * var_scale,
                covariance: Matrix3::identity() * radius,
                radius,
                eigen_min: 1e-4,
                eigen_mid: radius * radius,
                eigen_max: radius * radius,
                points_count,
                is_plane: true,
                is_mature: true,
            });
        }
        Ok(Self::from_planes(planes))
    }

    /// 写出文本格式（[`parse_text`] 的逆）。
    ///
    /// 平面不确定度取 `Σ_nq` 的近似对角标量 `trace/6` 作各向同性还原。
    #[must_use]
    pub fn to_text(&self) -> String {
        let mut lines = vec!["# 静态先验平面（c n d var_scale radius npts）".to_owned()];
        for p in &self.planes {
            let var_scale = p.plane_var.trace() / 6.0;
            lines.push(format!(
                "{:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6e} {:.6} {}",
                p.center[0],
                p.center[1],
                p.center[2],
                p.normal[0],
                p.normal[1],
                p.normal[2],
                p.d,
                var_scale,
                p.radius,
                p.points_count,
            ));
        }
        lines.join("\n") + "\n"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Matrix3;

    fn ground_plane() -> VoxelPlane {
        VoxelPlane {
            center: Vector3::new(0.0, 0.0, 0.0),
            normal: Vector3::z_axis().into_inner(),
            d: 0.0,
            plane_var: Matrix6::identity() * 1e-4,
            covariance: Matrix3::identity() * 10.0,
            radius: 10.0,
            eigen_min: 1e-6,
            eigen_mid: 10.0,
            eigen_max: 10.0,
            points_count: 1000,
            is_plane: true,
            is_mature: true,
        }
    }

    fn small_plane(center: Vector3<f64>) -> VoxelPlane {
        VoxelPlane {
            center,
            normal: Vector3::x_axis().into_inner(),
            d: -center[0],
            plane_var: Matrix6::identity() * 1e-5,
            covariance: Matrix3::identity(),
            radius: 0.3,
            eigen_min: 1e-6,
            eigen_mid: 0.09,
            eigen_max: 0.09,
            points_count: 50,
            is_plane: true,
            is_mature: true,
        }
    }

    #[test]
    fn large_plane_visible_from_any_root() {
        let map = PriorPlaneMap::from_planes(vec![ground_plane()]);
        // 大平面（地面 radius=10）在远处也能查到候选（径向判据由测量侧裁）
        for p in [Vector3::new(1.0, 4.0, 1.0), Vector3::new(-3.0, 20.0, 2.0)] {
            assert_eq!(map.candidates_at(&p).len(), 1, "p={p} 应命中全局地面");
        }
    }

    #[test]
    fn small_plane_indexed_by_covering_root_voxels() {
        let map = PriorPlaneMap::from_planes(vec![small_plane(Vector3::new(0.2, 0.2, 0.3))]);
        // 中心同体素 (0,0,0) 内命中
        assert_eq!(map.candidates_at(&Vector3::new(0.25, 0.25, 0.35)).len(), 1);
        // 半径跨体素边界：相邻体素 (1,1,1) 也在登记覆盖内（径向裁在测量侧）
        assert_eq!(map.candidates_at(&Vector3::new(0.9, 0.9, 0.9)).len(), 1);
        // 真远点（超出外接盒登记范围）无候选
        assert!(map.candidates_at(&Vector3::new(5.0, 5.0, 5.0)).is_empty());
    }

    #[test]
    fn roundtrip_text() {
        let map = PriorPlaneMap::from_planes(vec![
            ground_plane(),
            small_plane(Vector3::new(0.2, 0.2, 0.3)),
        ]);
        let text = map.to_text();
        let back = PriorPlaneMap::parse_text(&text).expect("roundtrip 可解析");
        assert_eq!(back.len(), 2);
        for p in back.planes() {
            assert!(p.is_plane && p.is_mature);
            assert!((p.normal.norm() - 1.0).abs() < 1e-9);
        }
        // 法向/中心还原
        let b0 = &back.planes()[0];
        assert!((b0.center - Vector3::zeros()).norm() < 1e-6);
        assert!((b0.normal - Vector3::z_axis().into_inner()).norm() < 1e-6);
    }

    #[test]
    fn parse_rejects_bad_line() {
        assert!(PriorPlaneMap::parse_text("1 2 3\n").is_err());
        assert!(PriorPlaneMap::parse_text("").is_ok());
        assert!(PriorPlaneMap::parse_text("# 只有注释\n").is_ok());
    }
}
