// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are indexed in docs/licenses/README.md.
//! Original `calibration/aprilgrid.h` and `src/calibration/aprilgrid.cpp`.
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::Vector4;
use serde::Deserialize;
use std::path::Path;

#[derive(Clone, Debug)]
pub struct AprilGrid {
    pub aprilgrid_corner_pos_3d: Vec<Vector4<f64>>,
    pub aprilgrid_vignette_pos_3d: Vec<Vector4<f64>>,
    tag_cols: usize,
    tag_rows: usize,
}
#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
#[expect(
    clippy::struct_field_names,
    reason = "Fields mirror the original JSON configuration"
)]
struct GridConfig {
    tag_cols: usize,
    tag_rows: usize,
    tag_size: f64,
    tag_spacing: f64,
}
impl AprilGrid {
    /// Reads the upstream JSON fields `tagCols`, `tagRows`, `tagSize`, `tagSpacing`.
    /// # Errors
    /// Returns file/JSON errors or invalid target dimensions.
    pub fn new(path: &Path) -> Result<Self> {
        let input = std::fs::File::open(path)
            .map_err(|e| Error::from(e).with_operation("basalt.aprilgrid"))?;
        let config: GridConfig = serde_json::from_reader(input).map_err(|e| {
            Error::new(ErrorKind::InvalidArgument, "Invalid AprilGrid JSON")
                .with_operation("basalt.aprilgrid")
                .with_source(e)
        })?;
        Self::from_parameters(
            config.tag_cols,
            config.tag_rows,
            config.tag_size,
            config.tag_spacing,
        )
    }
    /// # Errors
    /// Returns an error for empty, overflowing, nonfinite or nonpositive target geometry.
    pub fn from_parameters(cols: usize, rows: usize, size: f64, spacing: f64) -> Result<Self> {
        let tags = cols
            .checked_mul(rows)
            .filter(|n| *n <= usize::MAX / 20)
            .ok_or_else(|| {
                Error::new(ErrorKind::InvalidArgument, "AprilGrid size overflow")
                    .with_operation("basalt.aprilgrid")
            })?;
        if cols == 0
            || rows == 0
            || !size.is_finite()
            || size <= 0.
            || !spacing.is_finite()
            || spacing < 0.
        {
            return Err(
                Error::new(ErrorKind::InvalidArgument, "Invalid AprilGrid dimensions")
                    .with_operation("basalt.aprilgrid"),
            );
        }
        let mut corners = vec![Vector4::zeros(); tags * 4];
        let x_offsets = [0., size, size, 0.];
        let y_offsets = [0., 0., size, size];
        for y in 0..rows {
            for x in 0..cols {
                let tag = cols * y + x;
                let ox = x as f64 * size * (1. + spacing);
                let oy = y as f64 * size * (1. + spacing);
                for i in 0..4 {
                    corners[4 * tag + i] =
                        Vector4::new(ox + x_offsets[i], oy + y_offsets[i], 0., 1.);
                }
            }
        }
        let blocks = tags * 2;
        let mut vignette = vec![Vector4::zeros(); (blocks + cols + rows) * 5];
        for k in 0..5 {
            let coefficient = (k + 1) as f64 / 6.;
            for i in 0..tags {
                let (p1, p2, p3) = (corners[4 * i + 1], corners[4 * i + 2], corners[4 * i + 3]);
                vignette[k * blocks + 2 * i] = p1 + coefficient * (p2 - p1);
                vignette[k * blocks + 2 * i + 1] = p2 + coefficient * (p3 - p2);
                vignette[k * blocks + 2 * i].x += size * spacing / 2.;
                vignette[k * blocks + 2 * i + 1].y += size * spacing / 2.;
            }
        }
        let mut offset = blocks * 5;
        for k in 0..5 {
            for i in 0..cols {
                let (p0, p1) = (corners[4 * i], corners[4 * i + 1]);
                vignette[offset + k * cols + i] = p0 + (k + 1) as f64 / 6. * (p1 - p0);
                vignette[offset + k * cols + i].y -= size * spacing / 2.;
            }
        }
        offset += cols * 5;
        for k in 0..5 {
            for i in 0..rows {
                let (p0, p3) = (corners[4 * i * cols], corners[4 * i * cols + 3]);
                vignette[offset + k * rows + i] = p0 + (k + 1) as f64 / 6. * (p3 - p0);
                vignette[offset + k * rows + i].x -= size * spacing / 2.;
            }
        }
        Ok(Self {
            aprilgrid_corner_pos_3d: corners,
            aprilgrid_vignette_pos_3d: vignette,
            tag_cols: cols,
            tag_rows: rows,
        })
    }
    #[must_use]
    pub fn tag_cols(&self) -> usize {
        self.tag_cols
    }
    #[must_use]
    pub fn tag_rows(&self) -> usize {
        self.tag_rows
    }
}
