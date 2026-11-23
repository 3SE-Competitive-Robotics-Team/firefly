use crate::model;
use serde::Deserialize;
use std::path::PathBuf;
#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub(crate) struct Config {
    pub(crate) manifest: Option<PathBuf>,
    pub(crate) backend: model::Backend,
    pub(crate) runtime: Option<PathBuf>,
    pub(crate) rate_hz: f64,
    pub(crate) confidence: f32,
    pub(crate) diagnostics: bool,
    pub(crate) target_radius_m: f64,
    /// 标记平面到机体触地原点的高度差（米）；仿真为 21mm 减 1mm 涂层。
    pub(crate) contact_offset_m: f64,
    pub(crate) geometry: firefly_landing::geometry::Geometry,
    /// 光心在机体中的米制位置。
    pub(crate) camera_position: [f64; 3],
    /// 机体 ← CV 光学坐标旋转，单位四元数 xyzw。
    pub(crate) camera_rotation_xyzw: [f64; 4],
}
impl Config {
    pub(crate) fn manifest(&self) -> PathBuf {
        self.manifest.clone().unwrap_or_else(|| {
            let file = match self.backend {
                model::Backend::Cpu => "manifest.json",
                model::Backend::Viplite => "viplite.json",
            };
            firefly_config::paths::models("h-marker-yolo26n-cif-v1").join(file)
        })
    }

    pub(crate) fn tracking(&self) -> crate::tracking::Settings {
        let q = self.camera_rotation_xyzw;
        crate::tracking::Settings {
            geometry: self.geometry.clone(),
            camera_in_body: nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::from(nalgebra::Vector3::from(self.camera_position)),
                nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
                    q[3], q[0], q[1], q[2],
                )),
            ),
            acquisition_radius_m: self.target_radius_m,
            contact_offset_m: self.contact_offset_m,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        let mounting = firefly_base::rig::body_from_optical(firefly_base::FrameId::DOWN_CAMERA);
        Self {
            manifest: None,
            backend: model::Backend::Viplite,
            runtime: None,
            rate_hz: 10.,
            confidence: 0.5,
            diagnostics: false,
            target_radius_m: 0.8,
            contact_offset_m: 0.020,
            geometry: firefly_landing::geometry::Geometry::default(),
            camera_position: mounting.isometry().translation.vector.into(),
            camera_rotation_xyzw: mounting.isometry().rotation.coords.into(),
        }
    }
}
pub(crate) fn validate(config: &Config) -> firefly_error::Result<()> {
    config.geometry.validate()?;
    if !config.rate_hz.is_finite()
        || !(5.0..=30.).contains(&config.rate_hz)
        || !config.confidence.is_finite()
        || !(0.5..=1.).contains(&config.confidence)
        || !config.target_radius_m.is_finite()
        || !(0.01..=1.5).contains(&config.target_radius_m)
        || !config.contact_offset_m.is_finite()
        || config.contact_offset_m.abs() > 0.2
        || !config
            .camera_position
            .iter()
            .chain(config.camera_rotation_xyzw.iter())
            .all(|v| v.is_finite())
        || (config
            .camera_rotation_xyzw
            .iter()
            .map(|v| v * v)
            .sum::<f64>()
            - 1.)
            .abs()
            > 1e-6
    {
        return Err(firefly_error::Error::new(
            firefly_error::ErrorKind::InvalidArgument,
            "invalid landing rate, confidence, acquisition radius or mounting",
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn shipped_defaults_use_quantized_npu_and_validate_measured_mounting() {
        let mut config: Config =
            toml::from_str(include_str!("../../../../configs/landing.toml")).unwrap();
        validate(&config).unwrap();
        assert_eq!(config.backend, model::Backend::Viplite);
        assert_eq!(config.camera_position, [-0.03, 0., 0.]);
        config.camera_rotation_xyzw = [0.; 4];
        assert!(validate(&config).is_err());
        assert!(toml::from_str::<Config>("unknown=1").is_err());
    }
}
