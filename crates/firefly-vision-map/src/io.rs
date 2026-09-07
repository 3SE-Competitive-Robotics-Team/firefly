//! 地图文件存取：小端二进制（`MAGIC` + 版本 + 帧流），无第三方依赖。
//!
//! 布局（v2）：`magic[8] version u32 count u64`，每帧
//! `id u64 ts f64 pos 3f64 quat 4f64 n u64`，
//! 每点 `xyz 3f64 uv 2f32 desc 128f32 score f32`。

use std::io::{Read, Write};
use std::path::Path;

use firefly_error::{Error, ErrorKind};

use crate::map::{VisionKeyFrame, VisionMap, VisionMapPoint};

/// 文件魔数。
const MAGIC: &[u8; 8] = b"FFVMAP01";
/// 文件版本。
const VERSION: u32 = 2;

fn read_exact<const N: usize>(r: &mut impl Read) -> Result<[u8; N], Error> {
    let mut buf = [0u8; N];
    r.read_exact(&mut buf)
        .map_err(|e| Error::new(ErrorKind::InvalidArgument, format!("视觉地图读取失败: {e}")))?;
    Ok(buf)
}

fn read_u32(r: &mut impl Read) -> Result<u32, Error> {
    Ok(u32::from_le_bytes(read_exact(r)?))
}

fn read_u64(r: &mut impl Read) -> Result<u64, Error> {
    Ok(u64::from_le_bytes(read_exact(r)?))
}

fn read_f32(r: &mut impl Read) -> Result<f32, Error> {
    Ok(f32::from_le_bytes(read_exact(r)?))
}

fn read_f64(r: &mut impl Read) -> Result<f64, Error> {
    Ok(f64::from_le_bytes(read_exact(r)?))
}

/// 保存地图到文件（父目录需存在）。
///
/// # Errors
///
/// IO 失败。
pub fn save_map(map: &VisionMap, path: &Path) -> Result<(), Error> {
    let mut f = std::fs::File::create(path).map_err(|e| {
        Error::new(
            ErrorKind::InvalidArgument,
            format!("视觉地图创建失败 {}: {e}", path.display()),
        )
    })?;
    let fail =
        |e: std::io::Error| Error::new(ErrorKind::Internal, format!("视觉地图写入失败: {e}"));
    f.write_all(MAGIC).map_err(&fail)?;
    f.write_all(&VERSION.to_le_bytes()).map_err(&fail)?;
    f.write_all(&(map.frames.len() as u64).to_le_bytes())
        .map_err(&fail)?;
    for frame in &map.frames {
        f.write_all(&frame.id.to_le_bytes()).map_err(&fail)?;
        f.write_all(&frame.timestamp.to_le_bytes()).map_err(&fail)?;
        for v in frame.position {
            f.write_all(&v.to_le_bytes()).map_err(&fail)?;
        }
        for v in frame.quat_xyzw {
            f.write_all(&v.to_le_bytes()).map_err(&fail)?;
        }
        f.write_all(&(frame.points.len() as u64).to_le_bytes())
            .map_err(&fail)?;
        for p in &frame.points {
            for v in p.position {
                f.write_all(&v.to_le_bytes()).map_err(&fail)?;
            }
            for v in p.uv {
                f.write_all(&v.to_le_bytes()).map_err(&fail)?;
            }
            for v in p.descriptor {
                f.write_all(&v.to_le_bytes()).map_err(&fail)?;
            }
            f.write_all(&p.score.to_le_bytes()).map_err(&fail)?;
        }
    }
    Ok(())
}

/// 由文件加载地图（魔数/版本校验）。
///
/// # Errors
///
/// 文件缺失、魔数或版本不匹配、内容截断。
pub fn load_map(path: &Path) -> Result<VisionMap, Error> {
    let mut f = std::fs::File::open(path).map_err(|e| {
        Error::new(
            ErrorKind::InvalidArgument,
            format!("视觉地图打开失败 {}: {e}", path.display()),
        )
    })?;
    if read_exact::<8>(&mut f)? != *MAGIC {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            format!("视觉地图魔数不匹配: {}", path.display()),
        ));
    }
    if read_u32(&mut f)? != VERSION {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            format!("视觉地图版本不支持: {}", path.display()),
        ));
    }
    let count = read_u64(&mut f)?;
    let mut frames = Vec::new();
    for _ in 0..count {
        let id = read_u64(&mut f)?;
        let timestamp = read_f64(&mut f)?;
        let position = [read_f64(&mut f)?, read_f64(&mut f)?, read_f64(&mut f)?];
        let quat_xyzw = [
            read_f64(&mut f)?,
            read_f64(&mut f)?,
            read_f64(&mut f)?,
            read_f64(&mut f)?,
        ];
        let n = read_u64(&mut f)?;
        let mut points = Vec::new();
        for _ in 0..n {
            let position = [read_f64(&mut f)?, read_f64(&mut f)?, read_f64(&mut f)?];
            let uv = [read_f32(&mut f)?, read_f32(&mut f)?];
            let mut descriptor = [0f32; 128];
            for d in &mut descriptor {
                *d = read_f32(&mut f)?;
            }
            let score = read_f32(&mut f)?;
            points.push(VisionMapPoint {
                position,
                uv,
                descriptor,
                score,
            });
        }
        frames.push(VisionKeyFrame {
            id,
            timestamp,
            position,
            quat_xyzw,
            points,
        });
    }
    Ok(VisionMap { frames })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::map::VisionMapPoint;

    #[test]
    fn roundtrip() {
        let map = VisionMap {
            frames: vec![VisionKeyFrame {
                id: 7,
                timestamp: 1.5,
                position: [1.0, 2.0, 3.0],
                quat_xyzw: [0.0, 0.0, 0.0, 1.0],
                points: vec![VisionMapPoint {
                    position: [4.0, 5.0, 6.0],
                    uv: [10.0, 20.0],
                    descriptor: [0.5; 128],
                    score: 0.9,
                }],
            }],
        };
        let dir = std::env::temp_dir().join("firefly-vision-map-test");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("map.ffvmap");
        save_map(&map, &path).unwrap();
        let back = load_map(&path).unwrap();
        assert_eq!(back.frames.len(), 1);
        assert_eq!(back.frames[0].id, 7);
        assert!((back.frames[0].points[0].position[0] - 4.0).abs() < 1e-12);
        assert!((back.frames[0].points[0].descriptor[3] - 0.5).abs() < 1e-9);
        std::fs::remove_file(&path).ok();
    }

    #[test]
    fn rejects_bad_magic() {
        let dir = std::env::temp_dir().join("firefly-vision-map-test");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("bad.ffvmap");
        std::fs::write(&path, b"not a map file................").unwrap();
        assert!(load_map(&path).is_err());
        std::fs::remove_file(&path).ok();
    }
}
