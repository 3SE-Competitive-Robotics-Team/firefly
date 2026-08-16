//! `FFMap` 占据地图标准文件格式（见 `docs/map-format.md`）。
//!
//! 静态环境为占据体素列表，动态障碍为形状 + 时间航点；
//! 文本行式指令，`#` 开头为注释。

use std::fs;

use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector3;

use crate::grid::{GridMap, GridMapBuilder, VoxelState};

/// 动态障碍形状。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Shape {
    Box { center: [f64; 3], size: [f64; 3] },
    Sphere { center: [f64; 3], radius: f64 },
}

impl Shape {
    /// 障碍在 `t` 时刻的占据体素索引集合（形状中心在 `pos`）。
    pub(crate) fn voxels_at(&self, pos: Vector3<f64>, map: &GridMap) -> Vec<[usize; 3]> {
        let mut out = Vec::new();
        let (lo, hi) = match *self {
            Shape::Box { size, .. } => {
                let s = Vector3::new(size[0], size[1], size[2]) / 2.0;
                (pos - s, pos + s)
            }
            Shape::Sphere { radius, .. } => {
                let r = Vector3::repeat(radius);
                (pos - r, pos + r)
            }
        };
        let origin = map.origin();
        let res = map.resolution();
        let clamp =
            |v: f64, dim: usize| v.clamp(origin[dim], origin[dim] + map.dims()[dim] as f64 * res);
        let i0 = ((clamp(lo.x, 0) - origin.x) / res).floor() as usize;
        let i1 = ((clamp(hi.x, 0) - origin.x) / res).ceil() as usize;
        let j0 = ((clamp(lo.y, 1) - origin.y) / res).floor() as usize;
        let j1 = ((clamp(hi.y, 1) - origin.y) / res).ceil() as usize;
        let k0 = ((clamp(lo.z, 2) - origin.z) / res).floor() as usize;
        let k1 = ((clamp(hi.z, 2) - origin.z) / res).ceil() as usize;
        for i in i0..i1.min(map.dims()[0]) {
            for j in j0..j1.min(map.dims()[1]) {
                for k in k0..k1.min(map.dims()[2]) {
                    let center = Vector3::new(
                        origin.x + (i as f64 + 0.5) * res,
                        origin.y + (j as f64 + 0.5) * res,
                        origin.z + (k as f64 + 0.5) * res,
                    );
                    let inside = match self {
                        Shape::Box { .. } => true,
                        Shape::Sphere { radius, .. } => (center - pos).norm() <= *radius,
                    };
                    if inside {
                        out.push([i, j, k]);
                    }
                }
            }
        }
        out
    }
}

/// 动态障碍：形状 + 时间航点序列。
#[derive(Debug, Clone)]
pub struct Motion {
    pub shape: Shape,
    pub waypoints: Vec<(f64, [f64; 3])>,
    pub loop_back: bool,
}

impl Motion {
    /// 障碍中心在 `t` 时刻的位置（相邻航点线性插值，`loop_back` 按周期取模）。
    #[must_use]
    pub fn position_at(&self, t: f64) -> [f64; 3] {
        let wp = &self.waypoints;
        let t_last = wp[wp.len() - 1].0;
        // 循环运动先取模（rem_euclid 保证负时间也安全），再按段插值
        let t = if self.loop_back {
            t.rem_euclid(t_last)
        } else {
            t
        };
        if t <= wp[0].0 {
            return wp[0].1;
        }
        if t >= t_last {
            return wp[wp.len() - 1].1;
        }
        for w in wp.windows(2) {
            let (ta, pa) = w[0];
            let (tb, pb) = w[1];
            if t >= ta && t <= tb {
                let s = (t - ta) / (tb - ta);
                return [
                    pa[0] + (pb[0] - pa[0]) * s,
                    pa[1] + (pb[1] - pa[1]) * s,
                    pa[2] + (pb[2] - pa[2]) * s,
                ];
            }
        }
        wp[wp.len() - 1].1
    }
}

/// `FFMap` 文件。
#[derive(Debug, Clone)]
pub struct MapFile {
    pub resolution: f64,
    pub origin: [f64; 3],
    pub dims: [usize; 3],
    /// 占据体素世界坐标（米）。
    pub occupied: Vec<[f64; 3]>,
    /// 装饰体素（不参与规划的视觉元素，如草丛）。
    pub decor: Vec<[f64; 3]>,
    pub motions: Vec<Motion>,
}

impl MapFile {
    /// 从文件加载。
    ///
    /// # Errors
    ///
    /// `NotFound`：文件不存在；`InvalidData`：解析失败。
    pub fn from_file(path: impl AsRef<std::path::Path>) -> Result<Self> {
        let text = fs::read_to_string(path.as_ref())
            .map_err(|e| Error::new(ErrorKind::NotFound, "map file not found").with_source(e))?;
        text.parse()
    }

    /// 静态环境栅格化。
    ///
    /// # Errors
    ///
    /// `InvalidData`：占据体素超出地图范围。
    pub fn to_grid_map(&self) -> Result<GridMap> {
        let origin = Vector3::new(self.origin[0], self.origin[1], self.origin[2]);
        let mut map = GridMapBuilder::new(self.resolution, self.dims)
            .with_origin(origin)
            .build()?;
        for p in &self.occupied {
            let idx = map
                .index_of(Vector3::new(p[0], p[1], p[2]))
                .ok_or_else(|| {
                    Error::new(ErrorKind::InvalidArgument, "occupied voxel outside map")
                        .with_context("occupied voxel", format!("{p:?}"))
                })?;
            map.set_state(idx, VoxelState::Occupied);
        }
        Ok(map)
    }

    /// 障碍在 `t` 时刻占据的体素索引。
    #[must_use]
    pub fn motion_voxels(&self, t: f64, map: &GridMap) -> Vec<[usize; 3]> {
        self.motions
            .iter()
            .flat_map(|m| {
                let pos = m.position_at(t);
                m.shape.voxels_at(Vector3::new(pos[0], pos[1], pos[2]), map)
            })
            .collect()
    }
}

impl std::str::FromStr for MapFile {
    type Err = Error;

    fn from_str(text: &str) -> Result<Self> {
        Parser::parse(text)
    }
}

impl std::fmt::Display for MapFile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "FORMAT     firefly-map   1")?;
        writeln!(f, "RESOLUTION {:.3}", self.resolution)?;
        writeln!(
            f,
            "ORIGIN     {:.3} {:.3} {:.3}",
            self.origin[0], self.origin[1], self.origin[2]
        )?;
        writeln!(
            f,
            "DIMS       {} {} {}",
            self.dims[0], self.dims[1], self.dims[2]
        )?;
        if !self.occupied.is_empty() {
            writeln!(f, "OCCUPANCY")?;
            for p in &self.occupied {
                writeln!(f, "{:.3} {:.3} {:.3}", p[0], p[1], p[2])?;
            }
        }
        if !self.decor.is_empty() {
            writeln!(f, "DECOR")?;
            for p in &self.decor {
                writeln!(f, "{:.3} {:.3} {:.3}", p[0], p[1], p[2])?;
            }
        }
        for m in &self.motions {
            match m.shape {
                Shape::Box { center, size } => {
                    writeln!(
                        f,
                        "MOTION box    {:.3} {:.3} {:.3} {:.3} {:.3} {:.3}",
                        center[0], center[1], center[2], size[0], size[1], size[2]
                    )?;
                }
                Shape::Sphere { center, radius } => {
                    writeln!(
                        f,
                        "MOTION sphere {:.3} {:.3} {:.3} {:.3}",
                        center[0], center[1], center[2], radius
                    )?;
                }
            }
            for &(t, p) in &m.waypoints {
                writeln!(f, "{:.3} {:.3} {:.3} {:.3}", t, p[0], p[1], p[2])?;
            }
            if m.loop_back {
                writeln!(f, "LOOP")?;
            }
            writeln!(f)?;
        }
        Ok(())
    }
}

struct Parser;

impl Parser {
    fn parse(text: &str) -> Result<MapFile> {
        let mut resolution = None;
        let mut origin = None;
        let mut dims = None;
        let mut occupied = Vec::new();
        let mut decor = Vec::new();
        let mut motions = Vec::new();
        let mut current: Option<Motion> = None;
        // 数据段：占据（默认）或装饰
        let mut in_decor = false;
        let mut line_no = 0usize;

        for raw in text.lines() {
            line_no += 1;
            let line = raw.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let mut words = line.split_whitespace();
            let Some(first) = words.next() else { continue };
            let rest: Vec<&str> = words.collect();
            let err = |msg: &str| -> Error {
                Error::new(ErrorKind::InvalidArgument, msg).with_context("line", line_no)
            };
            match first {
                "FORMAT" => {
                    if rest.len() != 2 || rest[0] != "firefly-map" || rest[1] != "1" {
                        return Err(err("unsupported FORMAT"));
                    }
                }
                "RESOLUTION" => {
                    resolution = Some(parse_f64(&rest, line_no, "RESOLUTION")?);
                }
                "ORIGIN" => {
                    origin = Some(parse_xyz(&rest, line_no)?);
                }
                "DIMS" => {
                    dims = Some(parse_usize3(&rest, line_no, "DIMS")?);
                }
                "OCCUPANCY" => {
                    finish_motion(&mut motions, &mut current, line_no)?;
                    in_decor = false;
                }
                "DECOR" => {
                    finish_motion(&mut motions, &mut current, line_no)?;
                    in_decor = true;
                }
                "MOTION" => {
                    finish_motion(&mut motions, &mut current, line_no)?;
                    current = Some(parse_motion(&rest, line_no)?);
                }
                "LOOP" => {
                    let m = current.as_mut().ok_or_else(|| err("LOOP outside MOTION"))?;
                    m.loop_back = true;
                }
                _ if current.is_some() => {
                    let m = current.as_mut().unwrap();
                    let words = line.split_whitespace().collect::<Vec<_>>();
                    parse_waypoint(&words, line_no, &mut m.waypoints)?;
                }
                _ => {
                    // 数据行（整行三个坐标），归属当前段
                    let nums =
                        parse_f64_n(&line.split_whitespace().collect::<Vec<_>>(), 3, line_no)?;
                    if in_decor {
                        decor.push([nums[0], nums[1], nums[2]]);
                    } else {
                        occupied.push([nums[0], nums[1], nums[2]]);
                    }
                }
            }
        }
        finish_motion(&mut motions, &mut current, line_no)?;

        let resolution = resolution
            .ok_or_else(|| Error::new(ErrorKind::InvalidArgument, "missing RESOLUTION"))?;
        let origin =
            origin.ok_or_else(|| Error::new(ErrorKind::InvalidArgument, "missing ORIGIN"))?;
        let dims = dims.ok_or_else(|| Error::new(ErrorKind::InvalidArgument, "missing DIMS"))?;
        validate_waypoints(&motions)?;
        Ok(MapFile {
            resolution,
            origin,
            dims,
            occupied,
            decor,
            motions,
        })
    }
}

fn parse_motion(rest: &[&str], line_no: usize) -> Result<Motion> {
    let err = |msg: &str| -> Error {
        Error::new(ErrorKind::InvalidArgument, msg).with_context("line", line_no)
    };
    let shape = match rest.first() {
        Some(&"box") => {
            let nums = parse_f64_n(&rest[1..], 6, line_no)?;
            Shape::Box {
                center: [nums[0], nums[1], nums[2]],
                size: [nums[3], nums[4], nums[5]],
            }
        }
        Some(&"sphere") => {
            let nums = parse_f64_n(&rest[1..], 4, line_no)?;
            Shape::Sphere {
                center: [nums[0], nums[1], nums[2]],
                radius: nums[3],
            }
        }
        _ => return Err(err("unknown MOTION shape")),
    };
    Ok(Motion {
        shape,
        waypoints: Vec::new(),
        loop_back: false,
    })
}

fn parse_waypoint(
    rest: &[&str],
    line_no: usize,
    waypoints: &mut Vec<(f64, [f64; 3])>,
) -> Result<()> {
    let nums = parse_f64_n(rest, 4, line_no)?;
    waypoints.push((nums[0], [nums[1], nums[2], nums[3]]));
    Ok(())
}

fn finish_motion(
    motions: &mut Vec<Motion>,
    current: &mut Option<Motion>,
    line_no: usize,
) -> Result<()> {
    if let Some(m) = current.take() {
        if m.waypoints.is_empty() {
            return Err(
                Error::new(ErrorKind::InvalidArgument, "MOTION without waypoints")
                    .with_context("line", line_no),
            );
        }
        motions.push(m);
    }
    Ok(())
}

fn validate_waypoints(motions: &[Motion]) -> Result<()> {
    for m in motions {
        for w in m.waypoints.windows(2) {
            if w[1].0 < w[0].0 {
                return Err(Error::new(
                    ErrorKind::InvalidArgument,
                    "MOTION waypoint time must be monotonic",
                ));
            }
        }
    }
    Ok(())
}

fn parse_f64(rest: &[&str], line_no: usize, key: &str) -> Result<f64> {
    if rest.len() != 1 {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            format!("{key} expects one value"),
        )
        .with_context("line", line_no));
    }
    rest[0].parse().map_err(|e| {
        Error::new(ErrorKind::InvalidArgument, format!("invalid {key} value"))
            .with_context("line", line_no)
            .with_source(e)
    })
}

fn parse_xyz(rest: &[&str], line_no: usize) -> Result<[f64; 3]> {
    let nums = parse_f64_n(rest, 3, line_no)?;
    Ok([nums[0], nums[1], nums[2]])
}

fn parse_usize3(rest: &[&str], line_no: usize, key: &str) -> Result<[usize; 3]> {
    let mut out = [0usize; 3];
    if rest.len() != 3 {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            format!("{key} expects 3 values"),
        )
        .with_context("line", line_no));
    }
    for (i, v) in rest.iter().enumerate() {
        out[i] = v.parse().map_err(|e| {
            Error::new(ErrorKind::InvalidArgument, format!("invalid {key} value"))
                .with_context("line", line_no)
                .with_source(e)
        })?;
    }
    Ok(out)
}

fn parse_f64_n(rest: &[&str], n: usize, line_no: usize) -> Result<Vec<f64>> {
    if rest.len() != n {
        return Err(
            Error::new(ErrorKind::InvalidArgument, format!("expects {n} values"))
                .with_context("line", line_no),
        );
    }
    rest.iter()
        .map(|v| {
            v.parse::<f64>().map_err(|e| {
                Error::new(ErrorKind::InvalidArgument, "invalid number")
                    .with_context("line", line_no)
                    .with_source(e)
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(a: [f64; 3], b: [f64; 3]) {
        for (x, y) in a.iter().zip(b) {
            assert!((x - y).abs() < 1e-9, "{a:?} != {b:?}");
        }
    }

    const SAMPLE: &str = "\
# demo
FORMAT     firefly-map   1
RESOLUTION 0.4
ORIGIN     0 0 0
DIMS       50 20 8
OCCUPANCY
1.2 0.6 1.4
2.8 3.4 2.2
MOTION box    9 2.5 1.5 0.8 3.0 1.2
0    1 2 1
4    18 2 1
8    1 2 1
LOOP
MOTION sphere 15 4 1 0.5
0    15 2 1
6    15 6 1
";

    #[test]
    fn parse_static_and_motion() {
        let map: MapFile = SAMPLE.parse().unwrap();
        assert!((map.resolution - 0.4).abs() < 1e-9);
        assert_eq!(map.dims, [50, 20, 8]);
        assert_eq!(map.occupied.len(), 2);
        assert_eq!(map.motions.len(), 2);
        assert!(map.motions[0].loop_back);
        assert!(!map.motions[1].loop_back);
        let grid = map.to_grid_map().unwrap();
        assert!(grid.is_occupied(Vector3::new(1.2, 0.6, 1.4)));
    }

    #[test]
    fn roundtrip() {
        let map: MapFile = SAMPLE.parse().unwrap();
        let text = map.to_string();
        let reparsed: MapFile = text.parse().unwrap();
        assert_eq!(reparsed.occupied.len(), map.occupied.len());
        for (a, b) in reparsed.occupied.iter().zip(&map.occupied) {
            assert_close(*a, *b);
        }
        assert_eq!(reparsed.motions.len(), map.motions.len());
        assert_eq!(reparsed.motions[0].loop_back, map.motions[0].loop_back);
    }

    #[test]
    fn motion_interpolation_and_loop() {
        let map: MapFile = SAMPLE.parse().unwrap();
        let m = &map.motions[0];
        assert_close(m.position_at(0.0), [1.0, 2.0, 1.0]);
        assert_close(m.position_at(2.0), [9.5, 2.0, 1.0]); // (1+18)/2
        assert_close(m.position_at(4.0), [18.0, 2.0, 1.0]);
        // t=10 取模到 t=2（周期 8）：(1+18)/2 处继续插值，而非瞬移回起点
        assert_close(m.position_at(10.0), [9.5, 2.0, 1.0]);
        // 非循环障碍：t 超过末航点停在终点
        assert_close(map.motions[1].position_at(100.0), [15.0, 6.0, 1.0]);
    }

    #[test]
    fn motion_voxels_in_map() {
        let map: MapFile = SAMPLE.parse().unwrap();
        let grid = map.to_grid_map().unwrap();
        let voxels = map.motion_voxels(0.0, &grid);
        assert!(!voxels.is_empty());
        // 每个体素都在界内
        for idx in voxels {
            assert!(idx.iter().zip(grid.dims()).all(|(i, d)| *i < d));
        }
    }

    #[test]
    fn missing_header_rejected() {
        let e = "RESOLUTION 0.4\n".parse::<MapFile>().unwrap_err();
        assert_eq!(e.kind(), ErrorKind::InvalidArgument);
    }

    #[test]
    fn unknown_shape_rejected() {
        let e = "FORMAT firefly-map 1\nRESOLUTION 0.4\nORIGIN 0 0 0\nDIMS 10 10 10\nMOTION cube 1 2 3\n"
            .parse::<MapFile>()
            .unwrap_err();
        assert_eq!(e.kind(), ErrorKind::InvalidArgument);
    }

    #[test]
    fn nonmonotonic_waypoint_rejected() {
        let e = "FORMAT firefly-map 1\nRESOLUTION 0.4\nORIGIN 0 0 0\nDIMS 10 10 10\nMOTION box 1 2 3 1 1 1\n5 1 1 1\n3 2 2 2\n"
            .parse::<MapFile>()
            .unwrap_err();
        assert_eq!(e.kind(), ErrorKind::InvalidArgument);
    }
}
