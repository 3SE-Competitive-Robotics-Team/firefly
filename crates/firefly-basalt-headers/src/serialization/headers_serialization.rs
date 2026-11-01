// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Cereal JSON field layout from original `serialization/headers_serialization.h`.
use crate::{
    calibration::{Calibration, MocapCalibration},
    camera::{
        double_sphere_camera::DoubleSphereCamera, extended_camera::ExtendedUnifiedCamera,
        generic_camera::GenericCamera, kannala_brandt_camera4::KannalaBrandtCamera4,
        pinhole_camera::PinholeCamera, pinhole_radtan8_camera::PinholeRadtan8Camera,
        unified_camera::UnifiedCamera,
    },
    scalar::BasaltScalar,
    spline::rd_spline::RdSpline,
};
use nalgebra::{Isometry3, Quaternion, SVector, Translation3, UnitQuaternion, Vector2};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde_json::{Value, json};

#[derive(Serialize, Deserialize)]
struct PoseArchive {
    px: f64,
    py: f64,
    pz: f64,
    qx: f64,
    qy: f64,
    qz: f64,
    qw: f64,
}

#[derive(Serialize, Deserialize)]
struct MocapArchive {
    #[serde(rename = "T_mocap_world")]
    world: PoseArchive,
    #[serde(rename = "T_imu_marker")]
    marker: PoseArchive,
    mocap_time_offset_ns: i64,
    mocap_to_imu_offset_ns: i64,
}
impl<S: BasaltScalar> Serialize for MocapCalibration<S> {
    fn serialize<T: Serializer>(&self, serializer: T) -> Result<T::Ok, T::Error> {
        MocapArchive {
            world: PoseArchive::from_pose(&self.t_moc_w),
            marker: PoseArchive::from_pose(&self.t_i_mark),
            mocap_time_offset_ns: self.mocap_time_offset_ns,
            mocap_to_imu_offset_ns: self.mocap_to_imu_offset_ns,
        }
        .serialize(serializer)
    }
}
impl<'de, S: BasaltScalar> Deserialize<'de> for MocapCalibration<S> {
    fn deserialize<T: Deserializer<'de>>(deserializer: T) -> Result<Self, T::Error> {
        let archive = MocapArchive::deserialize(deserializer)?;
        Ok(Self {
            t_moc_w: archive.world.pose(),
            t_i_mark: archive.marker.pose(),
            mocap_time_offset_ns: archive.mocap_time_offset_ns,
            mocap_to_imu_offset_ns: archive.mocap_to_imu_offset_ns,
        })
    }
}
impl PoseArchive {
    fn pose<S: BasaltScalar>(&self) -> Isometry3<S> {
        // Cereal writes Sophus coefficients directly, without renormalizing.
        Isometry3::from_parts(
            Translation3::new(
                S::from_f64_lossy(self.px),
                S::from_f64_lossy(self.py),
                S::from_f64_lossy(self.pz),
            ),
            UnitQuaternion::new_unchecked(Quaternion::new(
                S::from_f64_lossy(self.qw),
                S::from_f64_lossy(self.qx),
                S::from_f64_lossy(self.qy),
                S::from_f64_lossy(self.qz),
            )),
        )
    }
    fn from_pose<S: BasaltScalar>(p: &Isometry3<S>) -> Self {
        let t = p.translation.vector;
        let q = p.rotation.quaternion();
        Self {
            px: t.x.as_f64(),
            py: t.y.as_f64(),
            pz: t.z.as_f64(),
            qx: q.i.as_f64(),
            qy: q.j.as_f64(),
            qz: q.k.as_f64(),
            qw: q.w.as_f64(),
        }
    }
}
#[derive(Serialize, Deserialize)]
struct SplineArchive {
    value0: i64,
    value1: i64,
    value2: Vec<[f64; 1]>,
}
#[derive(Serialize, Deserialize)]
struct CalibrationArchive {
    #[serde(rename = "T_imu_cam")]
    transforms: Vec<PoseArchive>,
    intrinsics: Vec<Value>,
    resolution: Vec<[i32; 2]>,
    calib_accel_bias: [f64; 9],
    calib_gyro_bias: [f64; 12],
    imu_update_rate: f64,
    accel_noise_std: [f64; 3],
    gyro_noise_std: [f64; 3],
    accel_bias_std: [f64; 3],
    gyro_bias_std: [f64; 3],
    cam_time_offset_ns: i64,
    vignette: Vec<SplineArchive>,
}

fn number<S: BasaltScalar>(v: &Value, key: &str) -> Result<S, String> {
    v.get(key)
        .and_then(Value::as_f64)
        .map(S::from_f64_lossy)
        .ok_or_else(|| format!("missing numeric camera field {key}"))
}
fn parameters<S: BasaltScalar, const N: usize>(
    v: &Value,
    keys: [&str; N],
) -> Result<SVector<S, N>, String> {
    let mut p = SVector::zeros();
    for (i, key) in keys.iter().enumerate() {
        p[i] = number(v, key)?;
    }
    Ok(p)
}
fn read_camera<S: BasaltScalar>(v: &Value) -> Result<GenericCamera<S>, String> {
    let name = v
        .get("camera_type")
        .and_then(Value::as_str)
        .ok_or("missing camera_type")?;
    let p = v.get("intrinsics").ok_or("missing intrinsics")?;
    Ok(match name {
        "ds" => GenericCamera::DoubleSphere(DoubleSphereCamera::new(parameters(
            p,
            ["fx", "fy", "cx", "cy", "xi", "alpha"],
        )?)),
        "kb4" => GenericCamera::KannalaBrandt4(KannalaBrandtCamera4::new(parameters(
            p,
            ["fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4"],
        )?)),
        "ucm" => GenericCamera::Unified(UnifiedCamera::new(parameters(
            p,
            ["fx", "fy", "cx", "cy", "alpha"],
        )?)),
        "pinhole" => {
            GenericCamera::Pinhole(PinholeCamera::new(parameters(p, ["fx", "fy", "cx", "cy"])?))
        }
        "pinhole-radtan8" => GenericCamera::PinholeRadtan8(PinholeRadtan8Camera::with_rpmax(
            parameters(
                p,
                [
                    "fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6",
                ],
            )?,
            number(p, "rpmax")?,
        )),
        _ => GenericCamera::ExtendedUnified(ExtendedUnifiedCamera::new(parameters(
            p,
            ["fx", "fy", "cx", "cy", "alpha", "beta"],
        )?)),
    })
}
fn write_camera<S: BasaltScalar>(camera: &GenericCamera<S>) -> Value {
    let keys: &[&str] = match camera {
        GenericCamera::ExtendedUnified(_) => &["fx", "fy", "cx", "cy", "alpha", "beta"],
        GenericCamera::DoubleSphere(_) => &["fx", "fy", "cx", "cy", "xi", "alpha"],
        GenericCamera::KannalaBrandt4(_) => &["fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4"],
        GenericCamera::Unified(_) => &["fx", "fy", "cx", "cy", "alpha"],
        GenericCamera::Pinhole(_) => &["fx", "fy", "cx", "cy"],
        GenericCamera::PinholeRadtan8(_) => &[
            "fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6",
        ],
    };
    let mut parameters = serde_json::Map::new();
    for (key, value) in keys.iter().zip(camera.param().iter()) {
        parameters.insert((*key).to_owned(), json!(value.as_f64()));
    }
    if let GenericCamera::PinholeRadtan8(camera) = camera {
        parameters.insert("rpmax".to_owned(), json!(camera.rpmax().as_f64()));
    }
    json!({"camera_type":camera.name(),"intrinsics":parameters})
}
impl<S: BasaltScalar> Serialize for Calibration<S> {
    fn serialize<T: Serializer>(&self, serializer: T) -> Result<T::Ok, T::Error> {
        CalibrationArchive {
            transforms: self.t_i_c.iter().map(PoseArchive::from_pose).collect(),
            intrinsics: self.intrinsics.iter().map(write_camera).collect(),
            resolution: self.resolution.iter().map(|v| [v.x, v.y]).collect(),
            calib_accel_bias: std::array::from_fn(|i| self.calib_accel_bias.param()[i].as_f64()),
            calib_gyro_bias: std::array::from_fn(|i| self.calib_gyro_bias.param()[i].as_f64()),
            imu_update_rate: self.imu_update_rate.as_f64(),
            accel_noise_std: std::array::from_fn(|i| self.accel_noise_std[i].as_f64()),
            gyro_noise_std: std::array::from_fn(|i| self.gyro_noise_std[i].as_f64()),
            accel_bias_std: std::array::from_fn(|i| self.accel_bias_std[i].as_f64()),
            gyro_bias_std: std::array::from_fn(|i| self.gyro_bias_std[i].as_f64()),
            cam_time_offset_ns: self.cam_time_offset_ns,
            vignette: self
                .vignette
                .iter()
                .map(|s| SplineArchive {
                    value0: s.min_time_ns(),
                    value1: s.time_interval_ns(),
                    value2: s.knots().iter().map(|k| [k[0].as_f64()]).collect(),
                })
                .collect(),
        }
        .serialize(serializer)
    }
}
impl<'de, S: BasaltScalar> Deserialize<'de> for Calibration<S> {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let a = CalibrationArchive::deserialize(deserializer)?;
        let mut result = Self {
            t_i_c: a.transforms.iter().map(PoseArchive::pose).collect(),
            intrinsics: a
                .intrinsics
                .iter()
                .map(read_camera)
                .collect::<Result<_, _>>()
                .map_err(serde::de::Error::custom)?,
            resolution: a.resolution.into_iter().map(Vector2::from).collect(),
            imu_update_rate: S::from_f64_lossy(a.imu_update_rate),
            accel_noise_std: SVector::from(a.accel_noise_std).map(S::from_f64_lossy),
            gyro_noise_std: SVector::from(a.gyro_noise_std).map(S::from_f64_lossy),
            accel_bias_std: SVector::from(a.accel_bias_std).map(S::from_f64_lossy),
            gyro_bias_std: SVector::from(a.gyro_bias_std).map(S::from_f64_lossy),
            cam_time_offset_ns: a.cam_time_offset_ns,
            vignette: a
                .vignette
                .into_iter()
                .map(|s| {
                    let mut spline = RdSpline::new(s.value1, s.value0);
                    for k in s.value2 {
                        spline.knots_push_back(SVector::from(k).map(S::from_f64_lossy));
                    }
                    spline
                })
                .collect(),
            ..Self::default()
        };
        *result.calib_accel_bias.param_mut() =
            SVector::from(a.calib_accel_bias).map(S::from_f64_lossy);
        *result.calib_gyro_bias.param_mut() =
            SVector::from(a.calib_gyro_bias).map(S::from_f64_lossy);
        Ok(result)
    }
}
