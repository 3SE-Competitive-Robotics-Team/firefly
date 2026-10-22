/* OKVIS2-X — Copyright (c) 2015 ETH Zurich; 2020 Imperial College London;
 * 2025 Technical University of Munich and ETH Zurich. SPDX-License-Identifier: BSD-3-Clause */
//! Owned contiguous tensor/image conversions and the scalar visualization transforms in `utils.hpp`.

use firefly_error::{Error, ErrorKind, Result};
use nalgebra::DMatrix;
use purecv::core::{DynamicData, DynamicMatrix, Matrix};
use std::sync::Arc;

pub fn invalid(message: impl Into<String>, operation: &'static str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation(operation)
}

/// Float32 tensor in contiguous row-major order. Shape is preserved across the inference boundary.
#[derive(Clone, Debug)]
pub struct Tensor {
    pub shape: Vec<usize>,
    pub data: Arc<[f32]>,
}
impl Tensor {
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn new(shape: Vec<usize>, data: Vec<f32>) -> Result<Self> {
        let count = shape.iter().try_fold(1usize, |n, d| n.checked_mul(*d));
        if count != Some(data.len()) {
            return Err(invalid(
                "tensor dimensions and data length disagree",
                "Tensor::new",
            ));
        }
        Ok(Self {
            shape,
            data: data.into(),
        })
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn matrix(&self) -> Result<Matrix<f32>> {
        if self.shape.len() != 2 {
            return Err(invalid(
                "expected a two-dimensional tensor",
                "Tensor::matrix",
            ));
        }
        Ok(Matrix::from_vec(
            self.shape[0],
            self.shape[1],
            1,
            self.data.to_vec(),
        ))
    }
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    pub fn plane(&self, index: usize) -> Result<Self> {
        if self.shape.len() != 3 || index >= self.shape[0] {
            return Err(invalid(
                "expected a valid plane in [C,H,W]",
                "Tensor::plane",
            ));
        }
        let n = self.shape[1] * self.shape[2];
        Self::new(
            self.shape[1..].to_vec(),
            self.data[index * n..(index + 1) * n].to_vec(),
        )
    }
    #[must_use]
    pub fn map(&self, f: impl Fn(f32) -> f32) -> Self {
        Self {
            shape: self.shape.clone(),
            data: self.data.iter().copied().map(f).collect::<Vec<_>>().into(),
        }
    }
}

/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn byte_image(image: &DynamicMatrix) -> Result<&Matrix<u8>> {
    match &image.data {
        DynamicData::U8(matrix) => Ok(matrix),
        _ => Err(invalid("expected an unsigned-byte image", "byte_image")),
    }
}
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn float_image(image: &DynamicMatrix) -> Result<&Matrix<f32>> {
    match &image.data {
        DynamicData::F32(matrix) => Ok(matrix),
        _ => Err(invalid("expected a float32 image", "float_image")),
    }
}
#[must_use]
pub fn image_u8(matrix: Matrix<u8>) -> Arc<DynamicMatrix> {
    Arc::new(DynamicMatrix {
        data: DynamicData::U8(matrix),
    })
}
#[must_use]
pub fn image_f32(matrix: Matrix<f32>) -> Arc<DynamicMatrix> {
    Arc::new(DynamicMatrix {
        data: DynamicData::F32(matrix),
    })
}
/// Byte values remain unscaled; the Torch model consumes a `[H,W]` byte input.
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn cv_mat_to_tensor(image: &DynamicMatrix) -> Result<Matrix<u8>> {
    let image = byte_image(image)?;
    if image.channels != 1 || image.rows == 0 || image.cols == 0 {
        return Err(invalid("expected grayscale input", "cv_mat_to_tensor"));
    }
    Ok(image.clone())
}
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn tensor_to_cv_mat_float(tensor: &Tensor) -> Result<Matrix<f32>> {
    tensor.matrix()
}
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn tensor_to_cv_mat_byte(tensor: &Tensor) -> Result<Matrix<u8>> {
    let matrix = tensor.matrix()?;
    Ok(Matrix::from_vec(
        matrix.rows,
        matrix.cols,
        1,
        matrix.data.iter().map(|x| *x as u8).collect(),
    ))
}
/// Converts CHW to owned contiguous HWC, retaining the allocation with `Arc`.
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn tensor_to_shared_ptr(tensor: &Tensor) -> Result<Arc<[f32]>> {
    if tensor.shape.len() != 3 {
        return Err(invalid("expected [C,H,W]", "tensor_to_shared_ptr"));
    }
    let (c, h, w) = (tensor.shape[0], tensor.shape[1], tensor.shape[2]);
    let mut output = vec![0.; tensor.data.len()];
    for y in 0..h {
        for x in 0..w {
            for ch in 0..c {
                output[(y * w + x) * c + ch] = tensor.data[(ch * h + y) * w + x];
            }
        }
    }
    Ok(output.into())
}
#[must_use]
pub fn eigen_matrix_to_torch(matrix: &DMatrix<f32>) -> Tensor {
    let mut data = Vec::with_capacity(matrix.len());
    for row in 0..matrix.nrows() {
        for col in 0..matrix.ncols() {
            data.push(matrix[(row, col)]);
        }
    }
    Tensor {
        shape: vec![matrix.nrows(), matrix.ncols()],
        data: data.into(),
    }
}
#[derive(Clone, Copy, Debug)]
pub enum ColourMap {
    Inferno,
    Jet,
    Parula,
}
/// Raster operations unavailable in the pinned `PureCV` API. Implementations must
/// use the named `OpenCV` LUT, Hershey Complex font and AA/thickness parameters.
pub trait VisualizationRaster: Send + Sync {
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn colour_map(&self, values: &Matrix<u8>, map: ColourMap) -> Result<Matrix<u8>>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn line(
        &self,
        image: &mut Matrix<u8>,
        a: [f32; 2],
        b: [f32; 2],
        bgr: [u8; 3],
        thickness: i32,
        aa: bool,
    ) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn circle(
        &self,
        image: &mut Matrix<u8>,
        point: [f32; 2],
        radius: i32,
        bgr: [u8; 3],
        thickness: i32,
        aa: bool,
    ) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn text(
        &self,
        image: &mut Matrix<u8>,
        text: &str,
        point: [f32; 2],
        scale: f64,
        bgr: [u8; 3],
        aa: bool,
    ) -> Result<()>;
    /// # Errors
    /// Returns an error for invalid/missing calibration or tensor/image data, incompatible
    /// model outputs, or a model, processing callback, raster or worker failure.
    fn resize_cubic(&self, image: &Matrix<u8>, rows: usize, cols: usize) -> Result<Matrix<u8>>;
}
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn colour_values(
    tensor: &Tensor,
    scale: f32,
    map: ColourMap,
    raster: &dyn VisualizationRaster,
) -> Result<Matrix<u8>> {
    let values = tensor.map(|x| (x / scale).clamp(0., 1.) * 255.);
    raster.colour_map(&tensor_to_cv_mat_byte(&values)?, map)
}
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn tensor_to_depth_vis(
    tensor: &Tensor,
    raster: &dyn VisualizationRaster,
) -> Result<Matrix<u8>> {
    colour_values(tensor, 6., ColourMap::Inferno, raster)
}
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn tensor_to_i_depth_vis(
    tensor: &Tensor,
    raster: &dyn VisualizationRaster,
) -> Result<Matrix<u8>> {
    colour_values(&tensor.map(|x| 1. / x), 2., ColourMap::Inferno, raster)
}
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn tensor_to_i_sigma_vis(
    depth: &Tensor,
    sigma: &Tensor,
    raster: &dyn VisualizationRaster,
) -> Result<Matrix<u8>> {
    if depth.shape != sigma.shape {
        return Err(invalid(
            "depth/sigma shape mismatch",
            "tensor_to_i_sigma_vis",
        ));
    }
    let tensor = Tensor::new(
        depth.shape.clone(),
        depth
            .data
            .iter()
            .zip(sigma.data.iter())
            .map(|(d, s)| (1.0_f32 / (d * d)) * s)
            .collect(),
    )?;
    colour_values(&tensor, 1., ColourMap::Jet, raster)
}
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn gray_to_bgr(image: &Matrix<u8>) -> Result<Matrix<u8>> {
    purecv::imgproc::color::cvt_color_gray_to_bgr(image).map_err(|e| invalid(e, "gray_to_bgr"))
}
/// # Errors
/// Returns an error for invalid/missing calibration or tensor/image data, incompatible
/// model outputs, or a model, processing callback, raster or worker failure.
pub fn add_weighted(a: &Matrix<u8>, alpha: f64, b: &Matrix<u8>, beta: f64) -> Result<Matrix<u8>> {
    if (a.rows, a.cols, a.channels) != (b.rows, b.cols, b.channels) {
        return Err(invalid("blend image shape mismatch", "add_weighted"));
    }
    Ok(Matrix::from_vec(
        a.rows,
        a.cols,
        a.channels,
        a.data
            .iter()
            .zip(&b.data)
            .map(|(a, b)| {
                (alpha * f64::from(*a) + beta * f64::from(*b))
                    .round_ties_even()
                    .clamp(0., 255.) as u8
            })
            .collect(),
    ))
}
