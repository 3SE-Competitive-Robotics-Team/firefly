use std::str::FromStr;

/// 有限的米数。
#[derive(Clone, Copy, Debug)]
pub(crate) struct Meters(f64);

impl Meters {
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
