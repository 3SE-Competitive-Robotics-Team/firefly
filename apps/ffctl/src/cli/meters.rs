//! 有限米数：一个在解析期拒绝 `NaN`/`±inf` 的 `f64` 包装。
//!
//! `f64` 的 `FromStr` 会收下 `NaN`、`inf`、`-inf`，而传给飞控/规划器的坐标与高度必须是
//! 有限值。按 `usage-rs` 的划分这属于「用 `FromStr` 做类型化转换」的一侧——它只保证值的
//! 类型，不描述取值范围；范围规则另在字段上用 `validate` 声明（见 `fc::Takeoff`）。

use std::str::FromStr;

/// 有限的米数。
#[derive(Clone, Copy, Debug)]
pub(crate) struct Meters(f64);

impl Meters {
    /// 取出内部的 `f64`。
    #[must_use]
    pub(crate) const fn get(self) -> f64 {
        self.0
    }
}

impl FromStr for Meters {
    type Err = String;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        let value: f64 = text.parse().map_err(|_| format!("`{text}` 不是数值"))?;
        if value.is_finite() {
            Ok(Self(value))
        } else {
            Err(format!("`{text}` 不是有限数值"))
        }
    }
}
