//! Shared parsing and image-codec adaptation for the original dataset readers.
use super::dataset_io::{AccelData, GyroData, ImageData};
use firefly_calibration::image::image::ManagedImage;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use std::{fs, path::Path, str::FromStr, sync::Arc};

#[derive(Default)]
pub(super) struct Records {
    pub image_timestamps: Vec<i64>,
    pub accel: Vec<AccelData>,
    pub gyro: Vec<GyroData>,
    pub gt_timestamps: Vec<i64>,
    pub gt_poses: Vec<Isometry3<f64>>,
}
pub(super) fn failure(path: &Path, message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message)
        .with_operation("basalt.dataset.read")
        .with_context("path", path.display())
}
pub(super) fn lines(path: &Path) -> Result<Vec<(usize, String)>> {
    let text = fs::read_to_string(path).map_err(|e| {
        Error::new(
            if e.kind() == std::io::ErrorKind::NotFound {
                ErrorKind::NotFound
            } else {
                ErrorKind::Internal
            },
            "Cannot read dataset metadata",
        )
        .with_operation("basalt.dataset.read")
        .with_context("path", path.display())
        .with_source(e)
    })?;
    Ok(text
        .lines()
        .enumerate()
        .filter(|(_, line)| !line.trim().is_empty() && !line.starts_with('#'))
        .map(|(n, line)| (n + 1, line.to_owned()))
        .collect())
}
pub(super) fn parse<T: FromStr>(value: &str, path: &Path, line: usize) -> Result<T> {
    value.trim().parse().map_err(|_| {
        failure(path, "Invalid dataset field")
            .with_context("line", line)
            .with_context("value", value)
    })
}
pub(super) fn fields<'a>(
    line: &'a str,
    path: &Path,
    number: usize,
    count: usize,
    csv: bool,
) -> Result<Vec<&'a str>> {
    let values: Vec<_> = if csv {
        line.split(',').map(str::trim).collect()
    } else {
        line.split_whitespace().collect()
    };
    if values.len() < count {
        return Err(failure(path, "Missing dataset fields")
            .with_context("line", number)
            .with_context("required", count)
            .with_context("actual", values.len()));
    }
    Ok(values)
}
pub(super) fn seconds_ns(value: f64, path: &Path, line: usize) -> Result<i64> {
    let scaled = value * 1e9;
    if !scaled.is_finite() || scaled < i64::MIN as f64 || scaled >= -(i64::MIN as f64) {
        return Err(
            failure(path, "Timestamp is outside the nanosecond range").with_context("line", line)
        );
    }
    Ok(scaled as i64)
}
pub(super) fn numbers<const N: usize>(
    values: &[&str],
    path: &Path,
    line: usize,
) -> Result<[f64; N]> {
    if values.len() < N {
        return Err(failure(path, "Missing numeric fields").with_context("line", line));
    }
    let mut result = [0.; N];
    for (i, item) in result.iter_mut().enumerate() {
        *item = parse(values[i], path, line)?;
    }
    Ok(result)
}
pub(super) fn pose(
    position: [f64; 3],
    rotation: [f64; 4],
    path: &Path,
    line: usize,
) -> Result<Isometry3<f64>> {
    let q = Quaternion::new(rotation[0], rotation[1], rotation[2], rotation[3]);
    if !q.norm_squared().is_finite() || q.norm_squared() == 0. {
        return Err(failure(path, "Invalid ground-truth quaternion").with_context("line", line));
    }
    Ok(Isometry3::from_parts(
        Translation3::from(position),
        UnitQuaternion::new_normalize(q),
    ))
}
pub(super) fn image(path: &Path, only_gray8: bool) -> Result<ImageData> {
    if !path.exists() {
        return Ok(ImageData::default());
    }
    let decoded = image::ImageReader::open(path)
        .map_err(|e| failure(path, "Cannot open dataset image").with_source(e))?
        .decode()
        .map_err(|e| failure(path, "Cannot decode dataset image").with_source(e))?;
    let (w, h) = (decoded.width() as usize, decoded.height() as usize);
    let pixels: Vec<u16> = match decoded {
        image::DynamicImage::ImageLuma8(img) => img
            .into_raw()
            .into_iter()
            .map(|v| u16::from(v) << 8)
            .collect(),
        image::DynamicImage::ImageRgb8(img) if !only_gray8 => img
            .into_raw()
            .as_chunks::<3>()
            .0
            .iter()
            .map(|v| u16::from(v[2]) << 8)
            .collect(),
        image::DynamicImage::ImageLuma16(img) if !only_gray8 => img.into_raw(),
        _ => return Err(failure(path, "Unsupported dataset image pixel format")),
    };
    let mut image = ManagedImage::new(w, h);
    image.data_mut().copy_from_slice(&pixels);
    Ok(ImageData {
        img: Some(Arc::new(image)),
        exposure: 0.,
    })
}
