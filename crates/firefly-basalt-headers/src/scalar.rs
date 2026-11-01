//! Floating-point types and constants used by the upstream Basalt instantiations.
mod sealed {
    pub trait Sealed {}
    impl Sealed for f32 {}
    impl Sealed for f64 {}
}

/// Scalar operations shared by camera, geometry and inertial models.
pub trait BasaltScalar: sealed::Sealed + nalgebra::RealField + Copy {
    fn as_f64(self) -> f64;
    fn from_f64_lossy(value: f64) -> Self;
    fn from_i64_lossy(value: i64) -> Self;
    fn min_positive_value() -> Self;
    /// Geometric validity threshold from Sophus, independent of machine epsilon.
    fn epsilon() -> Self;
    fn epsilon_sqrt() -> Self;
}
impl BasaltScalar for f32 {
    fn from_i64_lossy(value: i64) -> Self {
        value as Self
    }
    fn min_positive_value() -> Self {
        Self::MIN_POSITIVE
    }
    fn from_f64_lossy(value: f64) -> Self {
        value as Self
    }
    fn as_f64(self) -> f64 {
        f64::from(self)
    }
    fn epsilon() -> Self {
        1e-5
    }
    fn epsilon_sqrt() -> Self {
        1e-5_f32.sqrt()
    }
}
impl BasaltScalar for f64 {
    fn from_i64_lossy(value: i64) -> Self {
        value as Self
    }
    fn min_positive_value() -> Self {
        Self::MIN_POSITIVE
    }
    fn from_f64_lossy(value: f64) -> Self {
        value
    }
    fn as_f64(self) -> f64 {
        self
    }
    fn epsilon() -> Self {
        1e-10
    }
    fn epsilon_sqrt() -> Self {
        1e-10_f64.sqrt()
    }
}
