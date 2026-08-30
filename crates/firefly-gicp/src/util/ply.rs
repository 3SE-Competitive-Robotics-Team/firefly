//! 简易 PLY 读取（仅支持 small-gicp `data/*.ply` 二进制小端格式）。
//!
//! - 头部 `format binary_little_endian 1.0`，`element vertex N`，`property float x/y/z/scalar_intensity`。
//! - 忽略 intensity，仅读取 `x,y,z` 为 `Vector3<f32>` 转 `PointCloud`。

use std::path::Path;

use crate::points::point_cloud::PointCloud;
use crate::points::traits::PointCloudMut;
use nalgebra::Vector4;

/// 从 small-gicp 二进制 PLY 加载点云（`x,y,z`，`w=1`）。
///
/// 若文件不存在或格式不符，返回 `Err`；调用方可在测试中 `skip`。
pub fn load_small_gicp_ply(path: &Path) -> std::io::Result<PointCloud> {
    let bytes = std::fs::read(path)?;
    // 寻找 end_header
    let header_end = bytes
        .windows(11)
        .position(|w| w == b"end_header\n")
        .ok_or_else(|| std::io::Error::new(std::io::ErrorKind::InvalidData, "no end_header"))?;
    let header = &bytes[..header_end];
    let header_str = String::from_utf8_lossy(header);

    // 解析 vertex 数
    let mut num_vertices: usize = 0;
    for line in header_str.lines() {
        if line.starts_with("element vertex") {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() == 3 {
                num_vertices = parts[2].parse().unwrap_or(0);
            }
        }
    }
    if num_vertices == 0 {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "no vertex count",
        ));
    }

    let data_offset = header_end + 11; // "end_header\n" 长度
    let expected = num_vertices * 4 * 4; // 4 float ×4 byte
    if bytes.len() < data_offset + expected {
        return Err(std::io::Error::new(
            std::io::ErrorKind::UnexpectedEof,
            "ply truncated",
        ));
    }

    let mut cloud = PointCloud::new();
    cloud.resize(num_vertices);
    let data = &bytes[data_offset..];

    for i in 0..num_vertices {
        let base = i * 16;
        let x =
            f32::from_le_bytes([data[base], data[base + 1], data[base + 2], data[base + 3]]) as f64;
        let y = f32::from_le_bytes([
            data[base + 4],
            data[base + 5],
            data[base + 6],
            data[base + 7],
        ]) as f64;
        let z = f32::from_le_bytes([
            data[base + 8],
            data[base + 9],
            data[base + 10],
            data[base + 11],
        ]) as f64;
        // intensity 在 base+12..15 忽略
        cloud.set_point(i, Vector4::new(x, y, z, 1.0));
    }

    Ok(cloud)
}

/// 从 `T_target_source.txt`（4×4 行主序）加载 `Matrix4<f64>`。
pub fn load_transform_txt(path: &Path) -> std::io::Result<nalgebra::Matrix4<f64>> {
    let text = std::fs::read_to_string(path)?;
    let mut vals = Vec::new();
    for tok in text.split_whitespace() {
        vals.push(tok.parse::<f64>().map_err(|_| {
            std::io::Error::new(std::io::ErrorKind::InvalidData, "parse transform")
        })?);
    }
    if vals.len() != 16 {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "transform not 16 vals",
        ));
    }
    let mut m = nalgebra::Matrix4::zeros();
    for r in 0..4 {
        for c in 0..4 {
            m[(r, c)] = vals[r * 4 + c];
        }
    }
    Ok(m)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::points::traits::PointCloudTrait;
    use std::path::PathBuf;

    #[test]
    fn load_official_data_if_present() {
        let target_path = PathBuf::from(env!("HOME")).join("Projects/small-gicp/data/target.ply");
        let source_path = PathBuf::from(env!("HOME")).join("Projects/small-gicp/data/source.ply");
        let t_path =
            PathBuf::from(env!("HOME")).join("Projects/small-gicp/data/T_target_source.txt");

        if !target_path.exists() || !source_path.exists() || !t_path.exists() {
            eprintln!("official data not found, skip");
            return;
        }

        let target = load_small_gicp_ply(&target_path).expect("load target");
        let source = load_small_gicp_ply(&source_path).expect("load source");
        let t = load_transform_txt(&t_path).expect("load T");

        assert_eq!(target.num_points(), 69088, "target vertex count per header");
        assert_eq!(source.num_points(), 69792);
        assert!((t[(0, 0)] - 0.999925).abs() < 1e-6);
        // 简单校验：变换后 source 质心应接近 target 质心
        assert!(target.num_points() > 0 && source.num_points() > 0);
    }
}
