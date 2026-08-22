//! `GrayImage` ↔ purecv `Matrix<u8>` 转换。
//!
//! purecv 的 LK/FAST 接收 `Matrix<u8>`；u8 金字塔已随自研 LK 移除
//! （purecv LK 内部自建 f32 金字塔 + Sobel）。

use crate::sensor::GrayImage;
use purecv::core::Matrix;

/// 将 `GrayImage` 转为 purecv `Matrix<u8>`（单通道）。
#[must_use]
pub fn gray_to_matrix(img: &GrayImage) -> Matrix<u8> {
    Matrix::from_vec(img.height, img.width, 1, img.data.clone())
}
