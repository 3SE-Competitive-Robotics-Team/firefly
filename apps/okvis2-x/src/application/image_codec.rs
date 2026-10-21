// SPDX-License-Identifier: BSD-3-Clause

//! PNG/JPEG decoding with BGR/BGRA channel order at the estimator boundary.

use crate::datasets::dataset_reader::{ImageCodec, ImageReadMode};
use image::{DynamicImage, ImageBuffer};
use purecv::{
    core::{DynamicData, DynamicMatrix, Matrix},
    imgproc::{ColorConversionCode, cvt_color},
};
use std::{io, path::Path};

pub struct RustImageCodec;

fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}

fn swap_red_blue<T>(pixels: &mut [T], channels: usize) {
    if channels >= 3 {
        for pixel in pixels.chunks_exact_mut(channels) {
            pixel.swap(0, 2);
        }
    }
}

impl ImageCodec for RustImageCodec {
    fn read(&self, path: &Path, mode: ImageReadMode) -> io::Result<DynamicMatrix> {
        let image = image::ImageReader::open(path)?
            .decode()
            .map_err(|error| invalid(error.to_string()))?;
        let image = if matches!(mode, ImageReadMode::Unchanged) {
            image
        } else {
            to_u8(image)?
        };
        if matches!(mode, ImageReadMode::Grayscale) {
            let (cols, rows) = (image.width() as usize, image.height() as usize);
            let matrix = match image {
                DynamicImage::ImageLuma8(m) => Matrix::from_vec(rows, cols, 1, m.into_raw()),
                DynamicImage::ImageLumaA8(m) => Matrix::from_vec(
                    rows,
                    cols,
                    1,
                    m.into_raw()
                        .as_chunks::<2>()
                        .0
                        .iter()
                        .map(|pixel| pixel[0])
                        .collect(),
                ),
                image => {
                    let rgb = image.into_rgb8();
                    cvt_color(
                        &Matrix::from_vec(rows, cols, 3, rgb.into_raw()),
                        ColorConversionCode::COLOR_RGB2GRAY,
                    )
                    .map_err(invalid)?
                }
            };
            return Ok(DynamicMatrix {
                data: DynamicData::U8(matrix),
            });
        }
        let image = if matches!(mode, ImageReadMode::Colour) {
            DynamicImage::ImageRgb8(image.into_rgb8())
        } else {
            image
        };
        let (cols, rows) = (image.width() as usize, image.height() as usize);
        macro_rules! pixels {
            ($image:expr, $variant:ident, $channels:expr) => {{
                let mut data = $image.into_raw();
                swap_red_blue(&mut data, $channels);
                DynamicData::$variant(Matrix::from_vec(rows, cols, $channels, data))
            }};
        }
        let data = match image {
            DynamicImage::ImageLuma8(m) => pixels!(m, U8, 1),
            DynamicImage::ImageLumaA8(m) => {
                pixels!(DynamicImage::ImageLumaA8(m).into_rgba8(), U8, 4)
            }
            DynamicImage::ImageRgb8(m) => pixels!(m, U8, 3),
            DynamicImage::ImageRgba8(m) => pixels!(m, U8, 4),
            DynamicImage::ImageLuma16(m) => pixels!(m, U16, 1),
            DynamicImage::ImageLumaA16(m) => {
                pixels!(DynamicImage::ImageLumaA16(m).into_rgba16(), U16, 4)
            }
            DynamicImage::ImageRgb16(m) => pixels!(m, U16, 3),
            DynamicImage::ImageRgba16(m) => pixels!(m, U16, 4),
            DynamicImage::ImageRgb32F(m) => pixels!(m, F32, 3),
            DynamicImage::ImageRgba32F(m) => pixels!(m, F32, 4),
            _ => return Err(invalid("unsupported decoded image format")),
        };
        Ok(DynamicMatrix { data })
    }

    fn write(&self, path: &Path, image: &DynamicMatrix) -> io::Result<()> {
        let cols = u32::try_from(image.cols()).map_err(|error| invalid(error.to_string()))?;
        let rows = u32::try_from(image.rows()).map_err(|error| invalid(error.to_string()))?;
        macro_rules! encode {
            ($m:expr, $pixel:ty, $variant:ident) => {{
                let mut pixels = $m.data.clone();
                swap_red_blue(&mut pixels, $m.channels);
                let image = ImageBuffer::<$pixel, _>::from_raw(cols, rows, pixels)
                    .ok_or_else(|| invalid("image storage does not match its dimensions"))?;
                DynamicImage::$variant(image)
            }};
        }
        let encoded = match &image.data {
            DynamicData::U8(m) => match m.channels {
                1 => encode!(m, image::Luma<u8>, ImageLuma8),
                2 => encode!(m, image::LumaA<u8>, ImageLumaA8),
                3 => encode!(m, image::Rgb<u8>, ImageRgb8),
                4 => encode!(m, image::Rgba<u8>, ImageRgba8),
                _ => return Err(invalid("unsupported channel count")),
            },
            DynamicData::U16(m) => match m.channels {
                1 => encode!(m, image::Luma<u16>, ImageLuma16),
                2 => encode!(m, image::LumaA<u16>, ImageLumaA16),
                3 => encode!(m, image::Rgb<u16>, ImageRgb16),
                4 => encode!(m, image::Rgba<u16>, ImageRgba16),
                _ => return Err(invalid("unsupported channel count")),
            },
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::Unsupported,
                    "PNG/JPEG output requires u8 or u16 pixels",
                ));
            }
        };
        encoded
            .save(path)
            .map_err(|error| io::Error::other(error.to_string()))
    }
}

/// `OpenCV`'s default integer image decoding keeps the high byte of u16 samples.
fn to_u8(image: DynamicImage) -> io::Result<DynamicImage> {
    let (cols, rows) = (image.width(), image.height());
    macro_rules! convert {
        ($m:expr, $pixel:ty, $variant:ident) => {{
            let pixels: Vec<u8> = $m
                .into_raw()
                .into_iter()
                .map(|value| (value >> 8) as u8)
                .collect();
            DynamicImage::$variant(
                ImageBuffer::<$pixel, _>::from_raw(cols, rows, pixels)
                    .ok_or_else(|| invalid("image dimensions disagree"))?,
            )
        }};
    }
    Ok(match image {
        DynamicImage::ImageLuma16(m) => convert!(m, image::Luma<u8>, ImageLuma8),
        DynamicImage::ImageLumaA16(m) => convert!(m, image::LumaA<u8>, ImageLumaA8),
        DynamicImage::ImageRgb16(m) => convert!(m, image::Rgb<u8>, ImageRgb8),
        DynamicImage::ImageRgba16(m) => convert!(m, image::Rgba<u8>, ImageRgba8),
        DynamicImage::ImageRgb32F(_) | DynamicImage::ImageRgba32F(_) => {
            return Err(invalid("float input requires unchanged decoding"));
        }
        image => image,
    })
}
