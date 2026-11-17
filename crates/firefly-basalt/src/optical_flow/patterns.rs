use nalgebra::{RealField, SMatrix};

/// A sampling pattern with one offset per matrix column.
pub trait Pattern<S: RealField + Copy, const N: usize> {
    /// Number of samples (including the historical 50/51 names with 52 samples).
    const PATTERN_SIZE: usize = N;

    /// Offsets in the exact upstream order.
    fn pattern2() -> SMatrix<S, 2, N>;
}

/// The original 24-point sampling pattern.
pub struct Pattern24;
/// The original 52-point sampling pattern.
pub struct Pattern52;
/// The 52-point pattern scaled by one half.
pub struct Pattern51;
/// The 52-point pattern scaled by three quarters.
pub struct Pattern50;

impl Pattern24 {
    /// Integer offsets before scalar conversion.
    pub const PATTERN_RAW: [[i16; 2]; 24] = [
        [-1, 5],
        [1, 5],
        [-3, 3],
        [-1, 3],
        [1, 3],
        [3, 3],
        [-5, 1],
        [-3, 1],
        [-1, 1],
        [1, 1],
        [3, 1],
        [5, 1],
        [-5, -1],
        [-3, -1],
        [-1, -1],
        [1, -1],
        [3, -1],
        [5, -1],
        [-3, -3],
        [-1, -3],
        [1, -3],
        [3, -3],
        [-1, -5],
        [1, -5],
    ];
}

impl Pattern52 {
    /// Integer offsets before scalar conversion.
    pub const PATTERN_RAW: [[i16; 2]; 52] = [
        [-3, 7],
        [-1, 7],
        [1, 7],
        [3, 7],
        [-5, 5],
        [-3, 5],
        [-1, 5],
        [1, 5],
        [3, 5],
        [5, 5],
        [-7, 3],
        [-5, 3],
        [-3, 3],
        [-1, 3],
        [1, 3],
        [3, 3],
        [5, 3],
        [7, 3],
        [-7, 1],
        [-5, 1],
        [-3, 1],
        [-1, 1],
        [1, 1],
        [3, 1],
        [5, 1],
        [7, 1],
        [-7, -1],
        [-5, -1],
        [-3, -1],
        [-1, -1],
        [1, -1],
        [3, -1],
        [5, -1],
        [7, -1],
        [-7, -3],
        [-5, -3],
        [-3, -3],
        [-1, -3],
        [1, -3],
        [3, -3],
        [5, -3],
        [7, -3],
        [-5, -5],
        [-3, -5],
        [-1, -5],
        [1, -5],
        [3, -5],
        [5, -5],
        [-3, -7],
        [-1, -7],
        [1, -7],
        [3, -7],
    ];
}

impl<S: RealField + Copy> Pattern<S, 24> for Pattern24 {
    fn pattern2() -> SMatrix<S, 2, 24> {
        SMatrix::from_fn(|row, col| S::from_i16(Self::PATTERN_RAW[col][row]).unwrap())
    }
}

impl<S: RealField + Copy> Pattern<S, 52> for Pattern52 {
    fn pattern2() -> SMatrix<S, 2, 52> {
        SMatrix::from_fn(|row, col| S::from_i16(Self::PATTERN_RAW[col][row]).unwrap())
    }
}

impl<S: RealField + Copy> Pattern<S, 52> for Pattern51 {
    fn pattern2() -> SMatrix<S, 2, 52> {
        <Pattern52 as Pattern<S, 52>>::pattern2() * S::from_f64(0.5).unwrap()
    }
}

impl<S: RealField + Copy> Pattern<S, 52> for Pattern50 {
    fn pattern2() -> SMatrix<S, 2, 52> {
        <Pattern52 as Pattern<S, 52>>::pattern2() * S::from_f64(0.75).unwrap()
    }
}
