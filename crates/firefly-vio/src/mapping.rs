//! 只读关键帧快照；地图生命周期独立于滤波器的特征清理与边缘化。
use firefly_vio_core::triangulation::{single_gaussnewton, single_triangulation};
use nalgebra::{Matrix3, Vector3, Vector4};

use crate::vio_manager::VioManager;

#[derive(Debug, Clone)]
pub struct Observation {
    pub id: usize,
    pub pixel: [f32; 2],
    pub normalized: [f64; 2],
    pub right_normalized: Option<[f64; 2]>,
    /// 当前机体系中的初值；无可靠三角化时仍保留二维观测。
    pub point_body: Option<[f64; 3]>,
}

#[derive(Debug, Clone)]
pub struct CameraCalibration {
    pub rotation: Matrix3<f64>,
    pub translation: Vector3<f64>,
    pub intrinsics: [f64; 8],
    pub fisheye: bool,
}

#[derive(Debug, Clone)]
pub struct Snapshot {
    pub camera_time: f64,
    pub imu_time: f64,
    pub position: Vector3<f64>,
    pub quaternion: Vector4<f64>,
    /// `OpenCV` 相机轴：右、下、前。方向为 camera←body。
    pub camera_rotation: Matrix3<f64>,
    pub camera_translation: Vector3<f64>,
    pub intrinsics: [f64; 8],
    pub fisheye: bool,
    pub right_camera: Option<CameraCalibration>,
    pub observations: Vec<Observation>,
}

/// 在清理特征前创建快照；不向滤波器写回重三角化结果。
#[fastrace::trace]
pub fn capture(vio: &VioManager) -> Option<Snapshot> {
    let state = &vio.state;
    let camera = state.cameras.get(&0)?;
    let calib = state.calib_imu_to_cam.get(&0)?;
    let intrinsics = camera.value();
    let clones = crate::updater_slam::build_clones_cam(state);
    let valid_times: Vec<_> = state.clones_imu.iter().map(|(t, _)| *t).collect();
    let rotation = state.imu.pose().rot();
    let position = state.imu.pos();
    let mut observations = Vec::new();
    for feature in vio.track_feats.database().iter_features() {
        let Some(times) = feature.timestamps.get(&0) else {
            continue;
        };
        let Some(index) = times
            .iter()
            .position(|t| t.total_cmp(&state.timestamp).is_eq())
        else {
            continue;
        };
        let pixel = feature.uvs.get(&0)?[index];
        let normalized = camera.undistort_f(pixel);
        let right_normalized = feature.timestamps.get(&1).and_then(|times| {
            let index = times
                .iter()
                .position(|t| t.total_cmp(&state.timestamp).is_eq())?;
            let pixel = *feature.uvs.get(&1)?.get(index)?;
            let uv = state.cameras.get(&1)?.undistort_f(pixel);
            Some([f64::from(uv.x), f64::from(uv.y)])
        });
        let mut triangulated = feature.clone();
        triangulated.clean_old_measurements(&valid_times);
        for (id, pixels) in &triangulated.uvs {
            if let Some(model) = state.cameras.get(id) {
                triangulated
                    .uvs_norm
                    .insert(*id, pixels.iter().map(|&p| model.undistort_f(p)).collect());
            }
        }
        let options = &vio.params.triangulation_options;
        let point = state
            .features_slam
            .get(&feature.featid)
            .filter(|p| p.representation == crate::options::FeatRepresentation::Global3D)
            .map(|p| p.get_xyz(false))
            .or_else(|| {
                (single_triangulation(&mut triangulated, &clones, options)
                    && single_gaussnewton(&mut triangulated, &clones, options))
                .then_some(triangulated.p_FinG)
            });
        // 所有可用观测必须解释该初值；像素门限不替代三角化的条件数与视差检查。
        let point = point.filter(|p| {
            triangulated.timestamps.iter().all(|(cam, times)| {
                times.iter().enumerate().all(|(j, t)| {
                    let Some((_, pose)) = clones
                        .get(cam)
                        .and_then(|poses| poses.iter().find(|(s, _)| s.total_cmp(t).is_eq()))
                    else {
                        return false;
                    };
                    let q = pose.rot * (p - pose.pos);
                    let Some(model) = state.cameras.get(cam) else {
                        return false;
                    };
                    let k = model.value();
                    let uv = triangulated.uvs_norm[cam][j];
                    q.iter().all(|v| v.is_finite())
                        && q.z > 0.1
                        && ((q.x / q.z - f64::from(uv.x)) * k[0])
                            .hypot((q.y / q.z - f64::from(uv.y)) * k[1])
                            <= 3.
                })
            })
        });
        observations.push(Observation {
            id: feature.featid,
            pixel: pixel.into(),
            normalized: [f64::from(normalized.x), f64::from(normalized.y)],
            right_normalized,
            point_body: point.map(|p| (rotation * (p - position)).into()),
        });
    }
    observations.sort_by_key(|o| o.id);
    Some(Snapshot {
        camera_time: state.timestamp,
        imu_time: vio.state_imu_timestamp(),
        position,
        quaternion: state.imu.quat(),
        camera_rotation: calib.rot(),
        camera_translation: calib.pos(),
        intrinsics,
        fisheye: camera.is_fisheye(),
        right_camera: state
            .cameras
            .get(&1)
            .zip(state.calib_imu_to_cam.get(&1))
            .map(|(camera, pose)| CameraCalibration {
                rotation: pose.rot(),
                translation: pose.pos(),
                intrinsics: camera.value(),
                fisheye: camera.is_fisheye(),
            }),
        observations,
    })
}
