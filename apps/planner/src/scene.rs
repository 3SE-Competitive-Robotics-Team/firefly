//! `MuJoCo` 场景数据：默认场景静态地图（与 `firefly_mujoco/scene.py` 同构）、
//! 空地图、动态人形体素渲染。

use firefly_error::{Error, ErrorKind, Result};
use firefly_map::MapFile;

/// `MuJoCo` 闭环模式的空地图（无静态先验，由深度感知填充）。
/// 范围覆盖仓库场景：x∈[-2,48]、y∈[-9,9]、z∈[0,5.2]。
#[must_use]
pub fn empty_map_file() -> MapFile {
    MapFile {
        resolution: 0.4,
        origin: [-2.0, -9.0, 0.0],
        dims: [125, 45, 13],
        occupied: Vec::new(),
        decor: Vec::new(),
        motions: Vec::new(),
    }
}

/// 仓库场景静态地图：与 `firefly_mujoco/scene.py` 的 `_WAREHOUSE_COLLIDERS`
/// 同构（box 中心 + 半尺寸），体素化后作先验，保证**全局路径**在空地图上
/// 也会沿走廊飞行（纯深度感知在航线上才看到障碍，全局路径会是直线）。
///
/// 布局：走廊带 y∈[-4,4] 全程净空（起点 (2,0,1) → 终点 (40,0,1)）；
/// 碰撞盒即两侧货架墙/两端墙/顶/地面。
#[must_use]
pub fn mujoco_map_file() -> MapFile {
    let mut map = empty_map_file();
    let boxes: [[f64; 6]; 6] = [
        [0.0, 0.0, 2.5, 0.15, 8.1, 2.5],
        [46.0, 0.0, 2.5, 0.15, 8.1, 2.5],
        [23.0, 0.0, 5.0, 23.2, 8.1, 0.1],
        [23.0, -6.5, 1.5, 23.2, 0.1, 1.5],
        [23.0, 6.5, 1.5, 23.2, 0.1, 1.5],
        [23.0, 0.0, -0.05, 23.2, 8.1, 0.05],
    ];
    let res = map.resolution;
    let o = map.origin;
    for [cx, cy, cz, hx, hy, hz] in boxes {
        for x in 0..map.dims[0] {
            for y in 0..map.dims[1] {
                for z in 0..map.dims[2] {
                    let p = [
                        o[0] + (x as f64 + 0.5) * res,
                        o[1] + (y as f64 + 0.5) * res,
                        o[2] + (z as f64 + 0.5) * res,
                    ];
                    if (p[0] - cx).abs() <= hx && (p[1] - cy).abs() <= hy && (p[2] - cz).abs() <= hz
                    {
                        map.occupied.push(p);
                    }
                }
            }
        }
    }
    map
}

/// 人形体素（0.1m 格）：双腿 + 躯干 + 头，脚底 z=0，中心对齐 (cx, cy)。
#[must_use]
pub fn human_voxels(cx: f64, cy: f64) -> Vec<(i32, i32, i32)> {
    let ox = (cx / 0.1).round() as i32 - 1;
    let oy = (cy / 0.1).round() as i32 - 1;
    let mut out = Vec::with_capacity(40);
    // 双腿：1×1×8
    for z in 0..=7 {
        out.push((ox - 1, oy, z));
        out.push((ox, oy, z));
    }
    // 躯干：3×2×4
    for x in -1..=1 {
        for y in 0..=1 {
            for z in 8..=11 {
                out.push((ox + x, oy + y, z));
            }
        }
    }
    // 头：2×2×2
    for x in 0..=1 {
        for y in 0..=1 {
            for z in 14..=15 {
                out.push((ox + x, oy + y, z));
            }
        }
    }
    out
}

/// 解析 `--xxx x y z` 形式的三维参数。
///
/// # Errors
///
/// 参数缺失或非数字。
pub fn parse_vec3(it: &mut impl Iterator<Item = String>, name: &str) -> Result<[f64; 3]> {
    let mut v = [0.0; 3];
    for c in &mut v {
        *c = it
            .next()
            .ok_or_else(|| Error::new(ErrorKind::InvalidArgument, format!("missing {name} value")))?
            .parse()
            .map_err(|e| {
                Error::new(ErrorKind::InvalidArgument, format!("invalid {name} value"))
                    .with_source(e)
            })?;
    }
    Ok(v)
}
