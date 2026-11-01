// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Native cereal overloads from official `include/basalt/io/dataset_io.h`.
use super::dataset_io::{AccelData, GyroData};
use crate::utils::common_types::CerealBinary;
use firefly_calibration::image::image::ManagedImage;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector3;
use std::io::{Read, Write};

fn error(source: std::io::Error) -> Error {
    Error::new(
        ErrorKind::InvalidArgument,
        "Cannot process dataset cereal archive",
    )
    .with_operation("basalt.cereal.binary")
    .with_source(source)
}
fn bytes<const N: usize>(reader: &mut impl Read) -> Result<[u8; N]> {
    let mut data = [0; N];
    reader.read_exact(&mut data).map_err(error)?;
    Ok(data)
}
macro_rules! imu_archive {
    ($ty:ty) => {
        impl CerealBinary for $ty {
            fn read_cereal_binary(reader: &mut impl Read) -> Result<Self> {
                let timestamp_ns = i64::from_ne_bytes(bytes(reader)?);
                let mut data = Vector3::zeros();
                for value in data.iter_mut() {
                    *value = f64::from_ne_bytes(bytes(reader)?);
                }
                Ok(Self { timestamp_ns, data })
            }
            fn write_cereal_binary(&self, writer: &mut impl Write) -> Result<()> {
                writer
                    .write_all(&self.timestamp_ns.to_ne_bytes())
                    .map_err(error)?;
                for value in self.data.iter() {
                    writer.write_all(&value.to_ne_bytes()).map_err(error)?;
                }
                Ok(())
            }
        }
    };
}
imu_archive!(GyroData);
imu_archive!(AccelData);
impl CerealBinary for ManagedImage<u8> {
    fn read_cereal_binary(reader: &mut impl Read) -> Result<Self> {
        let width = usize::from_ne_bytes(bytes(reader)?);
        let height = usize::from_ne_bytes(bytes(reader)?);
        width.checked_mul(height).ok_or_else(|| {
            error(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Image dimensions overflow",
            ))
        })?;
        let mut image = Self::new(width, height);
        reader.read_exact(image.data_mut()).map_err(error)?;
        Ok(image)
    }
    fn write_cereal_binary(&self, writer: &mut impl Write) -> Result<()> {
        writer.write_all(&self.w.to_ne_bytes()).map_err(error)?;
        writer.write_all(&self.h.to_ne_bytes()).map_err(error)?;
        // Original serialize reinitialises dense dimensions before binary_data.
        // Dense images are its defined useful input; reject a padded view rather
        // than silently archive padding as pixels.
        if self.stride != self.w {
            return Err(error(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "Cereal image requires dense storage",
            )));
        }
        writer.write_all(self.data()).map_err(error)
    }
}
