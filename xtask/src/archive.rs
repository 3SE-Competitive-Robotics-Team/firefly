//! 部署包格式：zstd 压缩的简单容器。
//!
//! 布局（小端）：
//! `magic "FFPKG001" | u32 条目数 | { u16 路径长度 | 路径 | u32 权限 | u64 大小 | 内容 }*`
//!
//! 只接受相对路径且不含 `..`；解包先写临时文件再改名，避免覆盖正在执行的二进制时
//! 触发 `ETXTBSY`。

use std::fs::{self, File};
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::os::unix::fs::PermissionsExt;
use std::path::{Component, Path, PathBuf};

const MAGIC: &[u8; 8] = b"FFPKG001";
const MAX_ENTRIES: u32 = 4096;
const MAX_PATH_BYTES: usize = 1024;
const ZSTD_LEVEL: i32 = 3;

pub struct Entry {
    /// 包内路径，使用 `/` 分隔。
    pub path: String,
    /// Unix 权限位，只保留低 9 位。
    pub mode: u32,
    pub source: PathBuf,
}

/// 写出部署包并返回文件大小（字节）。
pub fn write(package: &Path, entries: &[Entry]) -> io::Result<u64> {
    let file = File::create(package)?;
    let mut encoder = zstd::stream::write::Encoder::new(BufWriter::new(file), ZSTD_LEVEL)?;
    encoder.write_all(MAGIC)?;
    let count = u32::try_from(entries.len()).map_err(|_| invalid("too many entries"))?;
    encoder.write_all(&count.to_le_bytes())?;

    for entry in entries {
        if !safe_path(&entry.path) {
            return Err(invalid(format!("unsafe entry path: {}", entry.path)));
        }
        let path = entry.path.as_bytes();
        let length = u16::try_from(path.len()).map_err(|_| invalid("entry path too long"))?;
        encoder.write_all(&length.to_le_bytes())?;
        encoder.write_all(path)?;
        encoder.write_all(&entry.mode.to_le_bytes())?;

        let size = fs::metadata(&entry.source)?.len();
        encoder.write_all(&size.to_le_bytes())?;
        let mut source = BufReader::new(File::open(&entry.source)?);
        let copied = io::copy(&mut source.by_ref().take(size), &mut encoder)?;
        if copied != size {
            return Err(invalid(format!(
                "{} changed while packaging",
                entry.source.display()
            )));
        }
    }

    let mut writer = encoder.finish()?;
    writer.flush()?;
    writer.get_ref().sync_all()?;
    Ok(fs::metadata(package)?.len())
}

/// 解包到目标目录并返回条目数。
pub fn unpack(package: &Path, destination: &Path) -> io::Result<u32> {
    let mut decoder = zstd::stream::read::Decoder::new(BufReader::new(File::open(package)?))?;
    let mut magic = [0u8; 8];
    decoder.read_exact(&mut magic)?;
    if &magic != MAGIC {
        return Err(invalid("not a firefly package"));
    }

    let count = read_u32(&mut decoder)?;
    if count > MAX_ENTRIES {
        return Err(invalid("package has too many entries"));
    }

    for _ in 0..count {
        let path = read_path(&mut decoder)?;
        let mode = read_u32(&mut decoder)?;
        let size = read_u64(&mut decoder)?;
        let target = destination.join(&path);
        let parent = target
            .parent()
            .ok_or_else(|| invalid(format!("entry has no parent directory: {path}")))?;
        fs::create_dir_all(parent)?;

        let file_name = target
            .file_name()
            .ok_or_else(|| invalid(format!("entry has no file name: {path}")))?;
        let temp = target.with_file_name(format!(".{}.ffpkg-tmp", file_name.to_string_lossy()));
        let mut output = File::create(&temp)?;
        let copied = io::copy(&mut decoder.by_ref().take(size), &mut output)?;
        if copied != size {
            return Err(invalid(format!("truncated entry: {path}")));
        }
        output.set_permissions(fs::Permissions::from_mode(mode & 0o777))?;
        output.sync_all()?;
        drop(output);
        fs::rename(&temp, &target)?;
    }

    Ok(count)
}

fn read_u16(reader: &mut impl Read) -> io::Result<u16> {
    let mut bytes = [0u8; 2];
    reader.read_exact(&mut bytes)?;
    Ok(u16::from_le_bytes(bytes))
}

fn read_u32(reader: &mut impl Read) -> io::Result<u32> {
    let mut bytes = [0u8; 4];
    reader.read_exact(&mut bytes)?;
    Ok(u32::from_le_bytes(bytes))
}

fn read_u64(reader: &mut impl Read) -> io::Result<u64> {
    let mut bytes = [0u8; 8];
    reader.read_exact(&mut bytes)?;
    Ok(u64::from_le_bytes(bytes))
}

fn read_path(reader: &mut impl Read) -> io::Result<String> {
    let length = usize::from(read_u16(reader)?);
    if length == 0 || length > MAX_PATH_BYTES {
        return Err(invalid("invalid entry path length"));
    }
    let mut bytes = vec![0u8; length];
    reader.read_exact(&mut bytes)?;
    let path = String::from_utf8(bytes).map_err(|_| invalid("entry path is not UTF-8"))?;
    if !safe_path(&path) {
        return Err(invalid(format!("unsafe entry path: {path}")));
    }
    Ok(path)
}

fn safe_path(path: &str) -> bool {
    !path.is_empty()
        && !path.contains('\0')
        && Path::new(path)
            .components()
            .all(|component| matches!(component, Component::Normal(_)))
}

fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("xtask-archive-{}-{name}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn roundtrip_preserves_paths_modes_and_contents() {
        let dir = scratch("roundtrip");
        let binary = dir.join("fc");
        fs::write(&binary, b"binary").unwrap();
        let config = dir.join("vio.toml");
        fs::write(&config, b"key = 1\n").unwrap();
        let entries = vec![
            Entry {
                path: "bin/fc".to_owned(),
                mode: 0o755,
                source: binary,
            },
            Entry {
                path: "configs/vio.toml".to_owned(),
                mode: 0o644,
                source: config,
            },
        ];

        let package = dir.join("package.zst");
        assert!(write(&package, &entries).unwrap() > 0);

        let out = dir.join("out");
        assert_eq!(unpack(&package, &out).unwrap(), 2);
        assert_eq!(fs::read(out.join("bin/fc")).unwrap(), b"binary");
        assert_eq!(
            fs::read_to_string(out.join("configs/vio.toml")).unwrap(),
            "key = 1\n"
        );
        assert_eq!(
            fs::metadata(out.join("bin/fc"))
                .unwrap()
                .permissions()
                .mode()
                & 0o777,
            0o755
        );
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn rejects_unsafe_or_empty_paths() {
        let dir = scratch("unsafe");
        let file = dir.join("file");
        fs::write(&file, b"x").unwrap();
        for path in ["../escape", "/absolute", "", "a/../b"] {
            let entries = [Entry {
                path: path.to_owned(),
                mode: 0o644,
                source: file.clone(),
            }];
            assert!(
                write(&dir.join("package.zst"), &entries).is_err(),
                "accepted {path:?}"
            );
        }
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn rejects_foreign_package() {
        let dir = scratch("foreign");
        let package = dir.join("package.zst");
        fs::write(
            &package,
            zstd::encode_all(&b"not a package"[..], 3).unwrap(),
        )
        .unwrap();
        assert!(unpack(&package, &dir.join("out")).is_err());
        fs::remove_dir_all(&dir).unwrap();
    }
}
