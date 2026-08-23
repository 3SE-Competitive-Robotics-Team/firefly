//! `MuJoCo` 场景数据：默认场景静态地图（与 `firefly_mujoco/scene.py` 同构）、
//! 空地图、动态人形体素渲染。

use firefly_error::{Error, ErrorKind, Result};
use firefly_map::MapFile;

/// `MuJoCo` 闭环模式的空地图（无静态先验，由深度感知填充）。
/// 范围覆盖 `firefly-mujoco` 场景：x∈[0,32]、y∈[-5,9]、z∈[0,5.2]。
#[must_use]
pub fn empty_map_file() -> MapFile {
    MapFile {
        resolution: 0.4,
        origin: [0.0, -5.0, 0.0],
        dims: [80, 35, 13],
        occupied: Vec::new(),
        decor: Vec::new(),
        motions: Vec::new(),
    }
}

/// `MuJoCo` 默认场景静态地图：与 `firefly_mujoco/scene.py` 的障碍布局
/// 同构（box 中心 + 半尺寸），体素化后作先验，保证**全局路径**在空地图上
/// 也会绕柱蛇形（纯深度感知在航线上才看到障碍，全局路径会是直线）。
///
/// 布局：中线上一串孤立高柱（约 0.8~1.2m 见方），逼小幅左右绕行；
/// x=12/19 为走廊外小障（装饰）；起点盒两侧 x∈{0.5,2,3.5}×y{1.5,6.5}
/// 为 VIO 验证轨迹侧翼柱（`--script` 盒 x∈[-2,4]、y∈[3,5] 的近场特征源）。
#[must_use]
pub fn mujoco_map_file() -> MapFile {
    let mut map = empty_map_file();
    let boxes: [[f64; 6]; 11] = [
        [9.0, 4.0, 1.5, 0.4, 0.5, 1.5],
        [12.0, 6.5, 1.0, 0.4, 0.7, 1.0],
        [16.0, 4.0, 1.5, 0.4, 0.6, 1.5],
        [19.0, 1.8, 0.9, 0.4, 0.5, 0.9],
        [22.0, 3.6, 1.5, 0.4, 0.5, 1.5],
        // --script 轨迹侧翼柱（scene.py 同步）
        [0.5, 1.5, 1.5, 0.35, 0.35, 1.5],
        [2.0, 1.5, 1.5, 0.35, 0.35, 1.5],
        [3.5, 1.5, 1.5, 0.35, 0.35, 1.5],
        [0.5, 6.5, 1.5, 0.35, 0.35, 1.5],
        [2.0, 6.5, 1.5, 0.35, 0.35, 1.5],
        [3.5, 6.5, 1.5, 0.35, 0.35, 1.5],
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
