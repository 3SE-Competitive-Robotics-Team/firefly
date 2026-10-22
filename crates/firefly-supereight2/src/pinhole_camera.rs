/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Pinhole depth sensor, frustum tests and hysteretic integration-scale selection.
use crate::{
    image::invalid,
    math_util::plane_normal,
    sensor::{Sensor, SensorConfig},
};
use firefly_error::Result;
use firefly_srl_projection::ProjectionPinholeCamera;
use nalgebra::{Vector2, Vector3, Vector4};
#[derive(Clone, Debug)]
pub struct PinholeConfig {
    pub sensor: SensorConfig,
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
}
impl Default for PinholeConfig {
    fn default() -> Self {
        Self {
            sensor: SensorConfig::default(),
            fx: f32::NAN,
            fy: f32::NAN,
            cx: f32::NAN,
            cy: f32::NAN,
        }
    }
}
impl PinholeConfig {
    /// # Errors
    /// Returns an error if scaled sensor dimensions are non-finite or outside the `i32` range.
    pub fn downsampled(&self, factor: f32) -> Result<Self> {
        Ok(Self {
            sensor: self.sensor.downsampled(factor)?,
            fx: self.fx / factor,
            fy: self.fy / factor,
            cx: (self.cx + 0.5f32) / factor - 0.5f32,
            cy: (self.cy + 0.5f32) / factor - 0.5f32,
        })
    }
}
#[derive(Clone, Debug)]
pub struct PinholeCamera {
    pub sensor: Sensor,
    pub model: ProjectionPinholeCamera,
    pub scaled_pixel: f32,
    pub horizontal_fov: f32,
    pub vertical_fov: f32,
    pub frustum_vertices_s: [Vector3<f32>; 8],
    pub frustum_normals_s: [Vector4<f32>; 6],
}
impl std::ops::Deref for PinholeCamera {
    type Target = Sensor;
    fn deref(&self) -> &Self::Target {
        &self.sensor
    }
}
impl std::ops::DerefMut for PinholeCamera {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.sensor
    }
}
impl PinholeCamera {
    /// # Errors
    /// Returns an error for nonpositive image dimensions, invalid near/far ordering (including
    /// NaN), or NaN intrinsics.
    pub fn new(c: &PinholeConfig) -> Result<Self> {
        if c.sensor.width <= 0
            || c.sensor.height <= 0
            || c.sensor.near_plane.partial_cmp(&0.0) != Some(std::cmp::Ordering::Greater)
            || c.sensor.far_plane.partial_cmp(&c.sensor.near_plane)
                != Some(std::cmp::Ordering::Greater)
            || [c.fx, c.fy, c.cx, c.cy].iter().any(|v| v.is_nan())
        {
            return Err(invalid("invalid pinhole camera calibration"));
        }
        let mut sensor = Sensor::new(&c.sensor);
        sensor.left_hand_frame = (c.fx < 0.0) ^ (c.fy < 0.0);
        let mut camera = Self {
            sensor,
            model: ProjectionPinholeCamera::new(
                c.sensor.width as usize,
                c.sensor.height as usize,
                c.fx,
                c.fy,
                c.cx,
                c.cy,
            ),
            scaled_pixel: 1.0 / c.fx,
            horizontal_fov: 2.0 * (c.sensor.width as f32 / (2.0f32 * c.fx)).atan(),
            vertical_fov: 2.0 * (c.sensor.height as f32 / (2.0f32 * c.fy)).atan(),
            frustum_vertices_s: [Vector3::zeros(); 8],
            frustum_normals_s: [Vector4::zeros(); 6],
        };
        camera.compute_frustum();
        Ok(camera)
    }
    /// # Errors
    /// Returns an error if scaled sensor dimensions are unrepresentable or the resulting
    /// camera calibration is invalid.
    pub fn downsampled(c: &PinholeConfig, factor: f32) -> Result<Self> {
        Self::new(&c.downsampled(factor)?)
    }
    /// # Errors
    /// Returns an error if a scaled dimension is non-finite, negative, or outside the
    /// supported nonnegative `i32` range.
    pub fn from_camera(camera: &Self, factor: f32) -> Result<Self> {
        let width = camera.model.width as f32 / factor;
        let height = camera.model.height as f32 / factor;
        if !width.is_finite()
            || !height.is_finite()
            || width < 0.0
            || height < 0.0
            || width >= i32::MAX as f32
            || height >= i32::MAX as f32
        {
            return Err(invalid(
                "pinhole downsampling outside image dimension range",
            ));
        }
        let mut result = Self {
            sensor: camera.sensor.clone(),
            model: ProjectionPinholeCamera::new(
                width as usize,
                height as usize,
                camera.model.fx / factor,
                camera.model.fy / factor,
                (camera.model.cx + 0.5) / factor - 0.5,
                (camera.model.cy + 0.5) / factor - 0.5,
            ),
            scaled_pixel: 1.0 / (camera.model.fx / factor),
            horizontal_fov: 2.0 * (camera.model.width as f32 / (2.0 * camera.model.fx)).atan(),
            vertical_fov: 2.0 * (camera.model.height as f32 / (2.0 * camera.model.fy)).atan(),
            frustum_vertices_s: [Vector3::zeros(); 8],
            frustum_normals_s: [Vector4::zeros(); 6],
        };
        result.compute_frustum();
        Ok(result)
    }
    #[must_use]
    pub fn block_integration_scale(
        &self,
        centre: &Vector3<f32>,
        map_res: f32,
        last_scale: i32,
        min_scale: i32,
        max_scale: i32,
    ) -> i32 {
        let pixel_dim = centre.z * self.scaled_pixel;
        let ratio = pixel_dim / map_res;
        let mut scale = 0;
        for threshold in &self.pixel_voxel_ratio_per_scale {
            if ratio < *threshold {
                break;
            }
            scale += 1;
        }
        scale = scale.min(max_scale);
        let mut shifted = *centre;
        if scale > last_scale && min_scale != -1 {
            shifted.z = (f64::from(shifted.z) - 0.25) as f32;
        } else if scale < last_scale && min_scale != -1 {
            shifted.z = (f64::from(shifted.z) + 0.25) as f32;
        } else {
            return scale;
        }
        self.block_integration_scale(&shifted, map_res, last_scale, -1, max_scale)
    }
    #[must_use]
    pub fn measurement_from_point(&self, point: &Vector3<f32>) -> f32 {
        point.z
    }
    #[must_use]
    pub fn near_dist(&self, ray: &Vector3<f32>) -> f32 {
        self.near_plane / normalized(ray).z
    }
    #[must_use]
    pub fn far_dist(&self, ray: &Vector3<f32>) -> f32 {
        self.far_plane / normalized(ray).z
    }
    #[must_use]
    pub fn point_in_frustum(&self, point: &Vector3<f32>) -> bool {
        self.sphere_in_frustum(point, 0.0)
    }
    #[must_use]
    pub fn point_in_frustum_inf(&self, point: &Vector3<f32>) -> bool {
        self.sphere_in_frustum_inf(point, 0.0)
    }
    #[must_use]
    pub fn sphere_in_frustum(&self, centre: &Vector3<f32>, radius: f32) -> bool {
        self.frustum_test(centre, radius, 6)
    }
    #[must_use]
    pub fn sphere_in_frustum_inf(&self, centre: &Vector3<f32>, radius: f32) -> bool {
        self.frustum_test(centre, radius, 5)
    }
    #[must_use]
    pub const fn type_name() -> &'static str {
        "PinholeCamera"
    }
    fn frustum_test(&self, centre: &Vector3<f32>, radius: f32, count: usize) -> bool {
        let point = centre.push(1.0);
        for normal in &self.frustum_normals_s[..count] {
            if point.dot(normal) < -radius {
                return false;
            }
        }
        true
    }
    fn compute_frustum(&mut self) {
        for (i, pixel) in [
            Vector2::new(0.0, 0.0),
            Vector2::new(self.model.width as f32, 0.0),
            Vector2::new(self.model.width as f32, self.model.height as f32),
            Vector2::new(0.0, self.model.height as f32),
        ]
        .iter()
        .enumerate()
        {
            let ray = self.model.back_project(pixel);
            self.frustum_vertices_s[i] = self.near_plane * ray;
            self.frustum_vertices_s[i + 4] = self.far_plane * ray;
        }
        let v = self.frustum_vertices_s;
        let normals = [
            plane_normal(&v[4], &v[0], &v[3]),
            plane_normal(&v[1], &v[5], &v[6]),
            plane_normal(&v[7], &v[3], &v[2]),
            plane_normal(&v[5], &v[1], &v[0]),
        ];
        let product = self.model.fx * self.model.fy;
        let sign = product / product.abs();
        for (i, n) in normals.iter().enumerate() {
            self.frustum_normals_s[i] = (*n * sign).push(0.0);
        }
        self.frustum_normals_s[4] = Vector4::new(0.0, 0.0, 1.0, -self.near_plane);
        self.frustum_normals_s[5] = Vector4::new(0.0, 0.0, -1.0, self.far_plane);
    }
}
fn normalized(v: &Vector3<f32>) -> Vector3<f32> {
    let norm = v.norm();
    if norm > 0.0 { v / norm } else { *v }
}

impl PinholeConfig {
    /// # Errors
    /// Returns an error if camera/sensor options have incompatible TOML types or a configured
    /// transform is not finite and rigid.
    pub fn read_toml(&mut self, document: &toml::Value) -> Result<()> {
        self.sensor.read_toml(document)?;
        let Some(t) = crate::sensor::config_section(document, "sensor")? else {
            return Ok(());
        };
        for (name, value) in [
            ("fx", &mut self.fx),
            ("fy", &mut self.fy),
            ("cx", &mut self.cx),
            ("cy", &mut self.cy),
        ] {
            crate::sensor::config_float(t, name, value)?;
        }
        Ok(())
    }
}
impl std::fmt::Display for PinholeConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}fx: {}\nfy: {}\ncx: {}\ncy: {}\n",
            self.sensor, self.fx, self.fy, self.cx, self.cy
        )
    }
}
