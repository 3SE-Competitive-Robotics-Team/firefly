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
    type Err = firefly_error::Error;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        let value: f64 = text.parse().map_err(|e| {
            firefly_error::Error::caused_by(
                firefly_error::ErrorKind::InvalidArgument,
                format!("`{text}` 不是数值"),
                e,
            )
        })?;
        if value.is_finite() {
            Ok(Self(value))
        } else {
            Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                format!("`{text}` 不是有限数值"),
            ))
        }
    }
}
