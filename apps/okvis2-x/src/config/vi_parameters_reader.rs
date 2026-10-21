/*
 * OKVIS2-X - Open Keyframe-based Visual-Inertial SLAM Configurable with Dense
 * Depth or LiDAR, and GNSS
 * Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 * Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 * Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich
 * and ETH Zurich
 * SPDX-License-Identifier: BSD-3-Clause
 */
//! TOML configuration reader preserving OKVIS parameter keys and validation.
//! Matrices are flat row-major arrays. Boolean integers and strings retain the
//! source spelling rules; native TOML booleans are accepted as well. YAML input
//! and `OpenCV` `FileStorage` tags are not accepted.
use crate::{
    cameras::{
        camera_base::{CameraBase, RectifyMaps},
        equidistant_distortion::EquidistantDistortion,
        eucm_camera::EucmCamera,
        n_camera_system::{CameraType, DistortionType, NCameraSystem},
        no_distortion::NoDistortion,
        pinhole_camera::PinholeCamera,
        radial_tangential_distortion::RadialTangentialDistortion,
        radial_tangential_distortion8::RadialTangentialDistortion8,
    },
    config::parameters::{
        CameraCalibration, CameraParameters, EstimatorParameters, FrontendParameters,
        GpsParameters, ImuParameters, LidarParameters, OnlineCalibrationParameters,
        OutputParameters, ViParameters,
    },
    kinematics::transformation::{Transformation, normalized_quaternion},
};
use nalgebra::{DVector, Matrix3, Matrix4, SMatrix, Vector2, Vector3};
use std::{collections::BTreeSet, io, path::Path, sync::Arc};
use toml::Value;
fn invalid(s: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, s.into())
}
fn entry<'a>(node: &'a Value, name: &str) -> io::Result<&'a Value> {
    node.get(name)
        .ok_or_else(|| invalid(format!("missing parameter {name}")))
}
/// # Errors
/// Returns an error for missing/malformed configuration data, unsupported camera models,
/// inconsistent calibration dimensions or a failed rectification/configuration backend.
pub fn parse_integer(node: &Value, name: &str) -> io::Result<i32> {
    i32::try_from(
        entry(node, name)?
            .as_integer()
            .ok_or_else(|| invalid(format!("missing integer parameter {name}")))?,
    )
    .map_err(|_| invalid(format!("integer out of range: {name}")))
}
/// # Errors
/// Returns an error for missing/malformed configuration data, unsupported camera models,
/// inconsistent calibration dimensions or a failed rectification/configuration backend.
pub fn parse_real(node: &Value, name: &str) -> io::Result<f64> {
    entry(node, name)?
        .as_float()
        .ok_or_else(|| invalid(format!("missing real parameter {name}")))
}
/// # Errors
/// Returns an error for missing/malformed configuration data, unsupported camera models,
/// inconsistent calibration dimensions or a failed rectification/configuration backend.
pub fn parse_string(node: &Value, name: &str) -> io::Result<String> {
    entry(node, name)?
        .as_str()
        .map(str::to_owned)
        .ok_or_else(|| invalid(format!("missing string parameter {name}")))
}
/// # Errors
/// Returns an error for missing/malformed configuration data, unsupported camera models,
/// inconsistent calibration dimensions or a failed rectification/configuration backend.
pub fn parse_bool(node: &Value, name: &str) -> io::Result<bool> {
    let v = entry(node, name)?;
    if let Some(b) = v.as_bool() {
        return Ok(b);
    }
    if let Some(n) = v.as_integer() {
        return Ok(n != 0);
    }
    if let Some(s) = v.as_str() {
        return match s.split(' ').next().unwrap_or("").to_lowercase().as_str() {
            "false" | "no" | "n" | "off" => Ok(false),
            "true" | "yes" | "y" | "on" => Ok(true),
            _ => Err(invalid(format!("Boolean with uninterpretable value {s}"))),
        };
    }
    Err(invalid(format!("missing boolean parameter {name}")))
}
fn array<'a>(node: &'a Value, name: &str) -> io::Result<&'a [Value]> {
    entry(node, name)?
        .as_array()
        .map(Vec::as_slice)
        .ok_or_else(|| invalid(format!("missing real array parameter {name}")))
}
fn numeric(v: &Value) -> io::Result<f64> {
    v.as_float()
        .or_else(|| v.as_integer().map(|x| x as f64))
        .ok_or_else(|| invalid("expected array number"))
}
fn vector<const N: usize>(node: &Value, name: &str) -> io::Result<[f64; N]> {
    let a = array(node, name)?;
    if a.len() < N {
        return Err(invalid(format!("array {name} requires {N} entries")));
    }
    let mut out = [0.0; N];
    for (i, v) in out.iter_mut().enumerate() {
        *v = numeric(&a[i])?;
    }
    Ok(out)
}
/// # Errors
/// Returns an error for missing/malformed configuration data, unsupported camera models,
/// inconsistent calibration dimensions or a failed rectification/configuration backend.
pub fn parse_matrix4(node: &Value, name: &str) -> io::Result<Matrix4<f64>> {
    Ok(Matrix4::from_row_slice(&vector::<16>(node, name)?))
}
/// # Errors
/// Returns an error for missing/malformed configuration data, unsupported camera models,
/// inconsistent calibration dimensions or a failed rectification/configuration backend.
pub fn parse_vector3(node: &Value, name: &str) -> io::Result<Vector3<f64>> {
    Ok(Vector3::from_row_slice(&vector::<3>(node, name)?))
}
/// The external stereo primitive has the `OpenCV` stereoRectify contract. Both
/// paths use zero disparity, alpha/balance zero and signed-16 fixed-point maps.
pub trait StereoRectification {
    /// # Errors
    /// Returns an error for missing/malformed configuration data, unsupported camera models,
    /// inconsistent calibration dimensions or a failed rectification/configuration backend.
    fn rectify(&self, request: &RectificationRequest) -> io::Result<RectificationResult>;
}
pub struct RectificationRequest {
    pub left_k: Matrix3<f64>,
    pub right_k: Matrix3<f64>,
    pub left_distortion: [f64; 4],
    pub right_distortion: [f64; 4],
    pub raw_size: (i32, i32),
    pub rect_size: (i32, i32),
    pub rotation: Matrix3<f64>,
    pub translation: Vector3<f64>,
    pub equidistant: bool,
    pub fov_scale: f64,
    pub zero_disparity: bool,
    pub alpha_or_balance: f64,
}
pub struct RectificationResult {
    pub left_rotation: Matrix3<f64>,
    pub right_rotation: Matrix3<f64>,
    pub left_projection: SMatrix<f64, 3, 4>,
    pub right_projection: SMatrix<f64, 3, 4>,
    pub left_maps: RectifyMaps,
    pub right_maps: RectifyMaps,
}
#[derive(Default)]
pub struct ViParametersReader {
    parameters: Option<ViParameters>,
}
impl ViParametersReader {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
    #[must_use]
    pub fn get_parameters(&self) -> Option<&ViParameters> {
        self.parameters.as_ref()
    }
    #[must_use]
    pub fn into_parameters(self) -> Option<ViParameters> {
        self.parameters
    }
    /// # Errors
    /// Returns an error for missing/malformed configuration data, unsupported camera models,
    /// inconsistent calibration dimensions or a failed rectification/configuration backend.
    pub fn read_config_file(
        &mut self,
        path: &Path,
        rectifier: Option<&dyn StereoRectification>,
    ) -> io::Result<()> {
        let text = std::fs::read_to_string(path)?;
        self.read_config_str(&text, rectifier)
    }
    /// # Errors
    /// Returns an error for missing/malformed configuration data, unsupported camera models,
    /// inconsistent calibration dimensions or a failed rectification/configuration backend.
    pub fn read_config_str(
        &mut self,
        text: &str,
        rectifier: Option<&dyn StereoRectification>,
    ) -> io::Result<()> {
        let file: Value = toml::from_str(text).map_err(|e| invalid(e.to_string()))?;
        let calibrations = Self::get_camera_calibration(&file)?;
        let camera = entry(&file, "camera_parameters")?;
        let rectify_count = calibrations
            .iter()
            .filter(|c| c.camera_type.depth_type.need_rectify)
            .count();
        let fov_scale = if camera.get("fov_scale").is_some_and(Value::is_float) {
            Some(parse_real(camera, "fov_scale")?)
        } else {
            None
        };
        if rectify_count != 0 && fov_scale.is_none() {
            return Err(invalid("Please set fov_scale for the stereo rectification"));
        }
        let sidx = if let Some(v) = camera.get("deep_stereo_indices").and_then(Value::as_array) {
            v.iter()
                .map(|x| {
                    let i = x
                        .as_integer()
                        .ok_or_else(|| invalid("deep_stereo_indices must be integers"))?;
                    usize::try_from(i).map_err(|_| invalid("negative stereo index"))
                })
                .collect::<io::Result<Vec<_>>>()?
        } else {
            if rectify_count != 0 {
                return Err(invalid(
                    "Please set deep_stereo_indices for the stereo depth network",
                ));
            }
            Vec::new()
        };
        let mut system = self
            .parameters
            .as_ref()
            .map(|parameters| parameters.n_camera_system.clone())
            .unwrap_or_default();
        if sidx.len() == 2 {
            let left = calibrations
                .get(sidx[0])
                .ok_or_else(|| invalid("left stereo index out of range"))?;
            let right = calibrations
                .get(sidx[1])
                .ok_or_else(|| invalid("right stereo index out of range"))?;
            Self::compute_rectify_map(&mut system, left, right, &sidx, fov_scale, rectifier)?;
        } else {
            log::warn!("No stereo rectification");
        }
        Self::append_camera_calibrations(&mut system, &calibrations)?;
        let online = entry(camera, "online_calibration")?;
        let imu = entry(&file, "imu_parameters")?;
        let frontend = entry(&file, "frontend_parameters")?;
        let estimator = entry(&file, "estimator_parameters")?;
        let output = entry(&file, "output_parameters")?;
        let mut sync_cameras = self
            .parameters
            .as_ref()
            .map(|parameters| parameters.camera.sync_cameras.clone())
            .unwrap_or_default();
        let configured_sync_cameras: BTreeSet<usize> = array(camera, "sync_cameras")?
            .iter()
            .map(|v| {
                v.as_integer()
                    .and_then(|n| usize::try_from(n).ok())
                    .ok_or_else(|| invalid("sync_cameras must contain nonnegative integers"))
            })
            .collect::<io::Result<_>>()?;
        sync_cameras.extend(configured_sync_cameras);
        let parameters = self.parse_parameters(
            &file,
            camera,
            (online, imu, frontend, estimator, output),
            system,
            (sidx, sync_cameras),
        )?;
        self.parameters = Some(parameters);
        Ok(())
    }
    fn parse_parameters(
        &self,
        file: &Value,
        camera: &Value,
        (online, imu, frontend, estimator, output): (&Value, &Value, &Value, &Value, &Value),
        system: NCameraSystem,
        (sidx, sync_cameras): (Vec<usize>, BTreeSet<usize>),
    ) -> io::Result<ViParameters> {
        let parameters = ViParameters {
            n_camera_system: system,
            camera: CameraParameters {
                timestamp_tolerance: parse_real(camera, "timestamp_tolerance")?,
                sync_cameras,
                stereo_indices: sidx,
                image_delay: parse_real(camera, "image_delay")?,
                online_calibration: OnlineCalibrationParameters {
                    do_extrinsics: parse_bool(online, "do_extrinsics")?,
                    do_extrinsics_final_ba: parse_bool(online, "do_extrinsics_final_ba")?,
                    sigma_r: parse_real(online, "sigma_r")?,
                    sigma_alpha: parse_real(online, "sigma_alpha")?,
                    sigma_r_final_ba: parse_real(online, "sigma_r_final_ba")?,
                    sigma_alpha_final_ba: parse_real(online, "sigma_alpha_final_ba")?,
                },
            },
            imu: Self::parse_imu_parameters(imu)?,
            frontend: FrontendParameters {
                detection_threshold: parse_real(frontend, "detection_threshold")?,
                absolute_threshold: parse_real(frontend, "absolute_threshold")?,
                matching_threshold: parse_real(frontend, "matching_threshold")?,
                octaves: parse_integer(frontend, "octaves")?,
                max_num_keypoints: parse_integer(frontend, "max_num_keypoints")?,
                keyframe_overlap: parse_real(frontend, "keyframe_overlap")?,
                use_cnn: parse_bool(frontend, "use_cnn")?,
                parallelise_detection: parse_bool(frontend, "parallelise_detection")?,
                num_matching_threads: parse_integer(frontend, "num_matching_threads")?,
            },
            estimator: Self::parse_estimator_parameters(estimator)?,
            output: OutputParameters {
                display: crate::config::parameters::DisplayOptions {
                    display_topview: parse_bool(output, "display_topview")?,
                    display_matches: parse_bool(output, "display_matches")?,
                    display_overhead: parse_bool(output, "display_overhead")?,
                },

                enable_submapping: parse_bool(output, "enable_submapping")?,
            },
            gps: file
                .get("gps_parameters")
                .filter(|v| v.is_table())
                .map(Self::get_gps_calibration)
                .transpose()?
                .or_else(|| {
                    self.parameters
                        .as_ref()
                        .and_then(|parameters| parameters.gps.clone())
                }),
            lidar: file
                .get("lidar")
                .filter(|v| v.is_table())
                .map(Self::get_lidar_calibration)
                .transpose()?
                .or_else(|| {
                    self.parameters
                        .as_ref()
                        .and_then(|parameters| parameters.lidar.clone())
                }),
            rgb: None,
        };
        Ok(parameters)
    }
    fn parse_estimator_parameters(estimator: &Value) -> io::Result<EstimatorParameters> {
        Ok(EstimatorParameters {
            num_keyframes: parse_integer(estimator, "num_keyframes")?,
            num_loop_closure_frames: parse_integer(estimator, "num_loop_closure_frames")?,
            num_imu_frames: parse_integer(estimator, "num_imu_frames")?,
            do_loop_closures: parse_bool(estimator, "do_loop_closures")?,
            do_final_ba: parse_bool(estimator, "do_final_ba")?,
            enforce_realtime: parse_bool(estimator, "enforce_realtime")?,
            realtime_min_iterations: parse_integer(estimator, "realtime_min_iterations")?,
            realtime_max_iterations: parse_integer(estimator, "realtime_max_iterations")?,
            realtime_time_limit: parse_real(estimator, "realtime_time_limit")?,
            realtime_num_threads: parse_integer(estimator, "realtime_num_threads")?,
            full_graph_iterations: parse_integer(estimator, "full_graph_iterations")?,
            full_graph_num_threads: parse_integer(estimator, "full_graph_num_threads")?,
            p_dbow: parse_real(estimator, "p_dbow")?,
            drift_percentage_heuristic: parse_real(estimator, "drift_percentage_heuristic")?,
        })
    }
    fn parse_imu_parameters(imu: &Value) -> io::Result<ImuParameters> {
        Ok(ImuParameters {
            r#use: parse_bool(imu, "use")?,
            t_bs: Transformation::from_matrix(&parse_matrix4(imu, "T_BS")?),
            a_max: parse_real(imu, "a_max")?,
            g_max: parse_real(imu, "g_max")?,
            sigma_g_c: parse_real(imu, "sigma_g_c")?,
            sigma_bg: parse_real(imu, "sigma_bg")?,
            sigma_a_c: parse_real(imu, "sigma_a_c")?,
            sigma_ba: parse_real(imu, "sigma_ba")?,
            sigma_gw_c: parse_real(imu, "sigma_gw_c")?,
            sigma_aw_c: parse_real(imu, "sigma_aw_c")?,
            g: parse_real(imu, "g")?,
            a0: parse_vector3(imu, "a0")?,
            g0: parse_vector3(imu, "g0")?,
            s_a: parse_vector3(imu, "s_a")?,
        })
    }
    fn append_camera_calibrations(
        system: &mut NCameraSystem,
        calibrations: &[CameraCalibration],
    ) -> io::Result<()> {
        for c in calibrations {
            let pose = Arc::new(Transformation::new(
                c.t_sc.translation(),
                normalized_quaternion(c.t_sc.quaternion()),
            ));
            let dims = c.image_dimension;
            let f = c.focal_length;
            let p = c.principal_point;
            let d = &c.distortion_coefficients;
            let (geometry, distortion): (Arc<dyn CameraBase>, DistortionType) = if c.camera_model
                == "eucm"
            {
                let mut cam = EucmCamera::new(
                    (dims.x, dims.y),
                    (f.x, f.y, p.x, p.y, c.eucm_parameters.x, c.eucm_parameters.y),
                    u64::MAX,
                );
                cam.initialise_camera_awareness_maps();
                (Arc::new(cam), DistortionType::NoDistortion)
            } else {
                macro_rules! pinhole {
                    ($dist:expr,$kind:expr) => {{
                        let mut cam = PinholeCamera::new(
                            (dims.x, dims.y),
                            (f.x, f.y, p.x, p.y),
                            $dist,
                            u64::MAX,
                        );
                        cam.initialise_undistort_maps();
                        cam.initialise_camera_awareness_maps();
                        (Arc::new(cam) as Arc<dyn CameraBase>, $kind)
                    }};
                }
                match c.distortion_type.as_str() {
                    "equidistant" => pinhole!(
                        EquidistantDistortion::new(d[0], d[1], d[2], d[3]),
                        DistortionType::Equidistant
                    ),
                    "radialtangential" | "plumb_bob" => pinhole!(
                        RadialTangentialDistortion::new(d[0], d[1], d[2], d[3]),
                        DistortionType::RadialTangential
                    ),
                    "radialtangential8" | "plumb_bob8" => {
                        if d.len() < 8 {
                            return Err(invalid("radialtangential8 requires eight coefficients"));
                        }
                        pinhole!(
                            RadialTangentialDistortion8::new((
                                d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]
                            )),
                            DistortionType::RadialTangential8
                        )
                    }
                    _ => {
                        log::error!("unrecognized distortion type {}", c.distortion_type);
                        continue;
                    }
                }
            };
            system.add_camera(&pose, geometry, distortion, true, c.camera_type.clone());
        }
        Ok(())
    }

    /// # Errors
    /// Returns an error for missing/malformed configuration data, unsupported camera models,
    /// inconsistent calibration dimensions or a failed rectification/configuration backend.
    pub fn get_camera_calibration(file: &Value) -> io::Result<Vec<CameraCalibration>> {
        Self::get_calibration_via_config(entry(file, "cameras")?)
    }
    /// # Errors
    /// Returns an error for missing/malformed configuration data, unsupported camera models,
    /// inconsistent calibration dimensions or a failed rectification/configuration backend.
    pub fn get_calibration_via_config(cameras: &Value) -> io::Result<Vec<CameraCalibration>> {
        let cameras = cameras
            .as_array()
            .filter(|a| !a.is_empty())
            .ok_or_else(|| invalid("Did not find any calibration"))?;
        let mut result = Vec::new();
        for c in cameras {
            if array(c, "image_dimension")?.len() != 2
                || array(c, "distortion_coefficients")?.len() < 4
                || array(c, "focal_length")?.len() != 2
                || array(c, "principal_point")?.len() != 2
            {
                return Err(invalid("incomplete camera calibration"));
            }
            let source_distortion = parse_string(c, "distortion_type")?;
            let model = parse_string(c, "cam_model")?;
            if model != "pinhole" && model != "eucm" {
                return Err(invalid(
                    "unsupported camera model with unassigned calibration fields",
                ));
            }
            let dims = vector::<2>(c, "image_dimension")?;
            let mut camera_type = CameraType::default();
            if let Some(value) = c.get("camera_type").and_then(Value::as_str) {
                camera_type.is_colour = !value.starts_with("gray");
                if value.len() >= 6 {
                    camera_type.depth_type.is_depth_camera = value.ends_with("+depth");
                }
            }
            if let Some(value) = c.get("slam_use").and_then(Value::as_str) {
                camera_type.is_used = value.starts_with("okvis");
                if value.len() >= 6 {
                    camera_type.depth_type.creation.create_depth = value.ends_with("-depth");
                }
                if value.len() >= 8 {
                    camera_type.depth_type.creation.create_virtual = value.ends_with("-virtual");
                }
            }
            if c.get("sigma_pixels").is_some_and(Value::is_float) {
                camera_type.depth_type.sigma_pixels = parse_real(c, "sigma_pixels")?;
            }
            if c.get("sigma_depth").is_some_and(Value::is_float) {
                camera_type.depth_type.sigma_pixels = parse_real(c, "sigma_depth")?;
            }
            let enabled = |name| {
                c.get(name).is_some_and(|value: &Value| {
                    value
                        .as_bool()
                        .unwrap_or_else(|| value.as_str() == Some("true"))
                })
            };
            if enabled("mapping") {
                camera_type.is_used_mapping = true;
            }
            if enabled("mapping_rectification") {
                camera_type.depth_type.need_rectify = true;
            }
            let distortion_coefficients = if model == "pinhole" {
                DVector::from_vec(
                    array(c, "distortion_coefficients")?
                        .iter()
                        .map(numeric)
                        .collect::<io::Result<_>>()?,
                )
            } else {
                DVector::zeros(0)
            };
            let eucm_parameters = if model == "eucm" {
                Vector2::from_row_slice(&vector::<2>(c, "eucm_parameters")?)
            } else {
                Vector2::zeros()
            };
            result.push(CameraCalibration {
                t_sc: Transformation::from_matrix(&parse_matrix4(c, "T_SC")?),
                image_dimension: Vector2::new(dims[0] as i32, dims[1] as i32),
                distortion_coefficients,
                focal_length: Vector2::from_row_slice(&vector::<2>(c, "focal_length")?),
                principal_point: Vector2::from_row_slice(&vector::<2>(c, "principal_point")?),
                camera_model: model.clone(),
                eucm_parameters,
                distortion_type: if model == "eucm" {
                    "None".into()
                } else {
                    source_distortion
                },
                camera_type,
            });
        }
        Ok(result)
    }
    /// # Errors
    /// Returns an error for missing/malformed configuration data, unsupported camera models,
    /// inconsistent calibration dimensions or a failed rectification/configuration backend.
    pub fn get_lidar_calibration(node: &Value) -> io::Result<LidarParameters> {
        Ok(LidarParameters {
            t_sl: Transformation::from_matrix(&parse_matrix4(node, "T_SL")?),
            elevation_resolution_angle: parse_real(node, "elevation_resolution_angle")? as f32,
            azimuth_resolution_angle: parse_real(node, "azimuth_resolution_angle")? as f32,
        })
    }
    /// # Errors
    /// Returns an error for missing/malformed configuration data, unsupported camera models,
    /// inconsistent calibration dimensions or a failed rectification/configuration backend.
    pub fn get_gps_calibration(node: &Value) -> io::Result<GpsParameters> {
        Ok(GpsParameters {
            r#type: parse_string(node, "data_type")?,
            r_sa: parse_vector3(node, "r_SA")?,
            yaw_error_threshold: parse_real(node, "yaw_error_threshold")?,
            robust_gps_init: parse_bool(node, "robust_gps_init")?,
        })
    }
    /// # Errors
    /// Returns an error for missing/malformed configuration data, unsupported camera models,
    /// inconsistent calibration dimensions or a failed rectification/configuration backend.
    pub fn compute_rectify_map(
        system: &mut NCameraSystem,
        left: &CameraCalibration,
        right: &CameraCalibration,
        indices: &[usize],
        fov_scale: Option<f64>,
        rectifier: Option<&dyn StereoRectification>,
    ) -> io::Result<bool> {
        if !(left.camera_type.depth_type.need_rectify
            && right.camera_type.depth_type.need_rectify
            && left.camera_model == "pinhole"
            && right.camera_model == "pinhole")
        {
            return Ok(false);
        }
        if left.distortion_type != "radialtangential" && left.distortion_type != "equidistant" {
            log::error!(
                "Radial-tangential or equidistant distortion models are supported in stereo rectification"
            );
            return Ok(false);
        }
        let width = left.image_dimension.x;
        let height = left.image_dimension.y;
        let aspect = f64::from(width) / f64::from(height);
        let mut rect_width = 512;
        if aspect > 1.6 {
            rect_width = (aspect * 384.0) as i32;
            if rect_width % 8 != 0 {
                rect_width += 8 - rect_width % 8;
            }
        }

        if left.distortion_coefficients.len() < 4
            || right.distortion_coefficients.len() < 4
            || indices.len() != 2
        {
            return Err(invalid("invalid stereo calibration"));
        }
        let relative = right
            .t_sc
            .matrix()
            .try_inverse()
            .ok_or_else(|| invalid("singular stereo extrinsics"))?
            * left.t_sc.matrix();
        let request = RectificationRequest {
            left_k: Self::calibration_matrix(left),
            right_k: Self::calibration_matrix(right),
            left_distortion: std::array::from_fn(|i| left.distortion_coefficients[i]),
            right_distortion: std::array::from_fn(|i| right.distortion_coefficients[i]),
            raw_size: (width, height),
            rect_size: (rect_width, 384),
            rotation: relative.fixed_view::<3, 3>(0, 0).into_owned(),
            translation: relative.fixed_view::<3, 1>(0, 3).into_owned(),
            equidistant: left.distortion_type == "equidistant",
            fov_scale: fov_scale.ok_or_else(|| invalid("unassigned stereo fov_scale"))?,
            zero_disparity: true,
            alpha_or_balance: 0.0,
        };
        let backend = rectifier.ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::Unsupported,
                "stereoRectify/fisheye stereoRectify capability required",
            )
        })?;
        let result = backend.rectify(&request)?;
        for (index, calibration, rotation, projection, maps) in [
            (
                indices[0],
                left,
                result.left_rotation,
                result.left_projection,
                result.left_maps,
            ),
            (
                indices[1],
                right,
                result.right_rotation,
                result.right_projection,
                result.right_maps,
            ),
        ] {
            let mut transform = Matrix4::identity();
            transform.fixed_view_mut::<3, 3>(0, 0).copy_from(&rotation);
            let t_sc = calibration
                .t_sc
                .compose(&Transformation::from_matrix(&transform).inverse());
            let pose = Arc::new(Transformation::new(
                t_sc.translation(),
                normalized_quaternion(t_sc.quaternion()),
            ));
            let mut camera = PinholeCamera::new(
                (rect_width, 384),
                (
                    projection[(0, 0)],
                    projection[(1, 1)],
                    projection[(0, 2)],
                    projection[(1, 2)],
                ),
                NoDistortion,
                u64::MAX,
            );
            camera.set_rectify_map(maps);
            system.add_rectify_camera(index, pose, Arc::new(camera));
        }
        Ok(true)
    }
    fn calibration_matrix(c: &CameraCalibration) -> Matrix3<f64> {
        let k = |c: &CameraCalibration| {
            Matrix3::new(
                c.focal_length.x,
                0.0,
                c.principal_point.x,
                0.0,
                c.focal_length.y,
                c.principal_point.y,
                0.0,
                0.0,
                1.0,
            )
        };
        k(c)
    }
}
