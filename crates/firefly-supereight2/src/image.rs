/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Image storage uses `PureCV` matrices; indices retain row-major pixel order.
use firefly_error::{Error, ErrorKind, Result};
pub type Image<T> = purecv::core::Matrix<T>;
pub(crate) fn invalid(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("supereight2")
}
/// # Errors
/// Returns an error unless the single-channel image dimensions match its contiguous storage.
pub fn validate_image<T>(image: &Image<T>) -> Result<()> {
    if image.channels != 1 || image.rows.checked_mul(image.cols) != Some(image.data.len()) {
        return Err(invalid("single-channel contiguous image storage required"));
    }
    Ok(())
}
/// # Errors
/// Returns an error for invalid image storage, output/remap shape mismatch, or a remap index
/// outside the input.
pub fn remap<T: Copy>(input: &Image<T>, output: &mut Image<T>, map: &Image<usize>) -> Result<()> {
    validate_image(input)?;
    validate_image(output)?;
    validate_image(map)?;
    if output.rows != map.rows || output.cols != map.cols {
        return Err(invalid("output/remap dimensions differ"));
    }
    for (pixel, &index) in output.data.iter_mut().zip(&map.data) {
        *pixel = *input
            .data
            .get(index)
            .ok_or_else(|| invalid("remap index outside input"))?;
    }
    Ok(())
}
