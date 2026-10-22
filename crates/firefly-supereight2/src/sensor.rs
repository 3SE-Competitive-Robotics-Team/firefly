/*
 * SPDX-FileCopyrightText: 2020-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2022-2024 Simon Boche
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Shared depth-sensor calibration and integration-scale thresholds.
use crate::image::invalid;
use firefly_error::Result;
use nalgebra::IsometryMatrix3;
#[derive(Clone, Debug)]
pub struct SensorConfig {
    pub width: i32,
    pub height: i32,
    pub near_plane: f32,
    pub far_plane: f32,
    pub t_bs: IsometryMatrix3<f32>,
    pub pixel_voxel_ratio_per_scale: Vec<f32>,
}
impl Default for SensorConfig {
    fn default() -> Self {
        Self {
            width: 0,
            height: 0,
            near_plane: 0.01,
            far_plane: 10.0,
            t_bs: IsometryMatrix3::identity(),
            pixel_voxel_ratio_per_scale: vec![1.5, 3.0, 6.0],
        }
    }
}
impl SensorConfig {
    /// # Errors
    /// Returns an error if scaled sensor dimensions are non-finite or outside the `i32` range.
    pub fn downsampled(&self, factor: f32) -> Result<Self> {
        let w = self.width as f32 / factor;
        let h = self.height as f32 / factor;
        if !w.is_finite()
            || !h.is_finite()
            || w < i32::MIN as f32
            || w >= i32::MAX as f32
            || h < i32::MIN as f32
            || h >= i32::MAX as f32
        {
            return Err(invalid("sensor image downsampling outside integer range"));
        }
        Ok(Self {
            width: w as i32,
            height: h as i32,
            ..self.clone()
        })
    }
}
#[derive(Clone, Debug)]
pub struct Sensor {
    pub left_hand_frame: bool,
    pub near_plane: f32,
    pub far_plane: f32,
    pub t_bs: IsometryMatrix3<f32>,
    pub pixel_voxel_ratio_per_scale: Vec<f32>,
}
impl Sensor {
    #[must_use]
    pub fn new(config: &SensorConfig) -> Self {
        Self {
            left_hand_frame: false,
            near_plane: config.near_plane,
            far_plane: config.far_plane,
            t_bs: config.t_bs,
            pixel_voxel_ratio_per_scale: config.pixel_voxel_ratio_per_scale.clone(),
        }
    }
}

/// Section access preserves an existing configuration when its section is absent.
pub(crate) fn config_section<'a>(
    document: &'a toml::Value,
    name: &str,
) -> Result<Option<&'a toml::Table>> {
    match document.get(name) {
        None => {
            log::warn!("No {name} section; retaining configuration");
            Ok(None)
        }
        Some(v) => v
            .as_table()
            .map(Some)
            .ok_or_else(|| invalid(format!("{name} must be a table"))),
    }
}
pub(crate) fn config_float(table: &toml::Table, name: &str, value: &mut f32) -> Result<()> {
    if let Some(v) = table.get(name) {
        *value = v
            .as_float()
            .or_else(|| v.as_integer().map(|n| n as f64))
            .ok_or_else(|| invalid(format!("{name} must be numeric")))? as f32;
    }
    Ok(())
}
pub(crate) fn config_integer(table: &toml::Table, name: &str, value: &mut i32) -> Result<()> {
    if let Some(v) = table.get(name) {
        *value = i32::try_from(
            v.as_integer()
                .ok_or_else(|| invalid(format!("{name} must be an integer")))?,
        )
        .map_err(|_| invalid(format!("{name} outside i32 range")))?;
    }
    Ok(())
}
pub(crate) fn config_array<const N: usize>(
    table: &toml::Table,
    name: &str,
) -> Result<Option<[f32; N]>> {
    let Some(v) = table.get(name) else {
        return Ok(None);
    };
    let a = v
        .as_array()
        .ok_or_else(|| invalid(format!("{name} must be a flat array")))?;
    if a.len() != N {
        return Err(invalid(format!("{name} requires {N} elements")));
    }
    let mut out = [0.0; N];
    for (i, v) in a.iter().enumerate() {
        out[i] =
            v.as_float()
                .or_else(|| v.as_integer().map(|n| n as f64))
                .ok_or_else(|| invalid(format!("{name} must contain numbers")))? as f32;
    }
    Ok(Some(out))
}
pub(crate) fn config_transform(
    table: &toml::Table,
    full: &str,
    translation: &str,
    rotation: &str,
    default: nalgebra::Matrix4<f32>,
) -> Result<IsometryMatrix3<f32>> {
    let mut matrix = if let Some(a) = config_array::<16>(table, full)? {
        nalgebra::Matrix4::from_row_slice(&a)
    } else {
        default
    };
    if let Some(a) = config_array::<3>(table, translation)? {
        matrix
            .fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&nalgebra::Vector3::from(a));
    }
    if let Some(a) = config_array::<9>(table, rotation)? {
        matrix
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&nalgebra::Matrix3::from_row_slice(&a));
    }
    let r = matrix.fixed_view::<3, 3>(0, 0).into_owned();
    if !matrix.iter().all(|x| x.is_finite())
        || (r.transpose() * r - nalgebra::Matrix3::identity()).norm() > 1e-4
        || (r.determinant() - 1.0).abs() > 1e-4
        || matrix.row(3) != nalgebra::RowVector4::new(0.0, 0.0, 0.0, 1.0)
    {
        return Err(invalid(
            "sensor/map transform must be a finite rigid homogeneous transform",
        ));
    }
    Ok(IsometryMatrix3::from_parts(
        nalgebra::Translation3::from(matrix.fixed_view::<3, 1>(0, 3).into_owned()),
        nalgebra::Rotation3::from_matrix_unchecked(r),
    ))
}
impl SensorConfig {
    /// # Errors
    /// Returns an error if sensor parameters have incompatible TOML types or the configured
    /// transform is not finite and rigid.
    pub fn read_toml(&mut self, document: &toml::Value) -> Result<()> {
        let Some(t) = config_section(document, "sensor")? else {
            return Ok(());
        };
        config_integer(t, "width", &mut self.width)?;
        config_integer(t, "height", &mut self.height)?;
        config_float(t, "near_plane", &mut self.near_plane)?;
        config_float(t, "far_plane", &mut self.far_plane)?;
        self.t_bs = config_transform(t, "T_BS", "t_BS", "R_BS", nalgebra::Matrix4::identity())?;
        Ok(())
    }
}
impl std::fmt::Display for SensorConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "width: {}\nheight: {}\nnear_plane: {}\nfar_plane: {}\nT_BS: {}",
            self.width,
            self.height,
            self.near_plane,
            self.far_plane,
            self.t_bs.to_homogeneous()
        )
    }
}
