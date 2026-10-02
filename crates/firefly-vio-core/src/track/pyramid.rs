//! `GrayImage` ↔ purecv `Matrix<u8>` 转换。
//!
//! 为 FAST 和 LK 金字塔入口提供灰度矩阵。

use crate::sensor::GrayImage;
use purecv::core::Matrix;

/// 将 `GrayImage` 转为 purecv `Matrix<u8>`（单通道）。
#[must_use]
pub fn gray_to_matrix(img: &GrayImage) -> Matrix<u8> {
    Matrix::from_vec(img.height, img.width, 1, img.data.clone())
}
