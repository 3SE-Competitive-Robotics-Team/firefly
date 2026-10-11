//! `GrayImage` ↔ purecv `Matrix<u8>` 转换。
//!
//! purecv 的 LK 接收自有 `Matrix<u8>`，内部构建 f32 金字塔和 Scharr 导数。

use crate::sensor::GrayImage;
use purecv::core::Matrix;

/// 将 `GrayImage` 转为 purecv `Matrix<u8>`（单通道）。
#[must_use]
pub fn gray_to_matrix(img: &GrayImage) -> Matrix<u8> {
    Matrix::from_vec(img.height, img.width, 1, img.data.clone())
}
