//! 命令行坐标解析。

use firefly_error::{Error, ErrorKind, Result};

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
