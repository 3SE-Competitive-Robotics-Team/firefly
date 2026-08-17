//! 初始化编排器（对照 `OpenVINS` `ov_init/src/init/InertialInitializer.cpp`）。
//!
//! 持有 IMU 缓冲，按视差/急动检测选择静态或动态初始化路径：
//! - 静止或检测到急动 → [`StaticInitializer`]；
//! - 移动中且启用动态初始化 → [`DynamicInitializer`]；
//! - 其余情况返回 `None`（等待更多测量）。

use firefly_vio_core::feat::FeatureDatabase;
use firefly_vio_core::sensor::ImuData;
use nalgebra::Vector2;

use crate::InitResult;
use crate::dynamic_init::DynamicInitializer;
use crate::options::InitOptions;
use crate::static_init::StaticInitializer;

/// 初始化编排器（对照 `InertialInitializer`）。
#[derive(Debug, Clone)]
pub struct InertialInitializer {
    /// 初始化器选项。
    pub params: InitOptions,
    /// IMU 测量缓冲（跨静态/动态初始化器共享；对照 C++ 的 `imu_data`）。
    pub imu_data: Vec<ImuData>,
    /// 静态初始化器。
    init_static: StaticInitializer,
    /// 动态初始化器。
    init_dynamic: DynamicInitializer,
}

impl InertialInitializer {
    /// 构造（对照 `InertialInitializer` 构造函数）。
    #[must_use]
    pub fn new(params: InitOptions) -> Self {
        let init_static = StaticInitializer::new(params.clone());
        let init_dynamic = DynamicInitializer::new(params.clone());
        Self {
            params,
            imu_data: Vec::new(),
            init_static,
            init_dynamic,
        }
    }

    /// 喂入 IMU 测量并按 `oldest_time` 清理（对照 `InertialInitializer::feed_imu`）。
    pub fn feed_imu(&mut self, message: &ImuData, oldest_time: f64) {
        self.imu_data.push(*message);
        if (oldest_time - (-1.0)).abs() > f64::EPSILON {
            self.imu_data.retain(|d| d.timestamp >= oldest_time);
        }
    }

    /// 尝试初始化（对照 `InertialInitializer::initialize`）。
    ///
    /// `wait_for_jerk`：是否等待急动（无 ZUPT 时为 true，有 ZUPT 时 false）。
    /// 成功返回 [`InitResult`]；数据不足/未通过视差与激励检查返回 `None`。
    // 与 C++ 1:1 移植的编排流程，拆分会破坏对照可审计性。
    #[allow(clippy::too_many_lines)]
    pub fn initialize(
        &mut self,
        db: &mut FeatureDatabase,
        wait_for_jerk: bool,
    ) -> Option<InitResult> {
        // 0. 相机时刻窗口（对照 C++ 的 newest_cam_time/oldest_time）
        let mut newest_cam_time = -1.0f64;
        for feat in db.iter_features() {
            for ts in feat.timestamps.values() {
                for t in ts {
                    newest_cam_time = newest_cam_time.max(*t);
                }
            }
        }
        let oldest_time = newest_cam_time - self.params.init_window_time - 0.10;
        if newest_cam_time < 0.0 || oldest_time < 0.0 {
            return None;
        }

        // 1. 清理过旧测量（对照 C++ 的 cleanup_measurements 与 imu erase）
        db.cleanup_measurements(oldest_time);
        self.imu_data
            .retain(|d| d.timestamp >= oldest_time + self.params.calib_camimu_dt);

        // 2. 视差检测（对照 C++ 的 disparity_detected_moving_*）
        let mut disparity_detected_moving_1to0 = false;
        let mut disparity_detected_moving_2to1 = false;
        if self.params.init_max_disparity > 0.0 {
            let newest_time_allowed = newest_cam_time - 0.5 * self.params.init_window_time;
            let (avg_disp0, _var_disp0, num_features0) =
                compute_disparity(db, newest_time_allowed, -1.0);
            let (avg_disp1, _var_disp1, num_features1) =
                compute_disparity(db, newest_cam_time, newest_time_allowed);

            // 特征数不足则等待（对照 C++ feat_thresh=15）
            let feat_thresh = 15;
            if num_features0 < feat_thresh || num_features1 < feat_thresh {
                log::debug!(
                    "初始化：特征不足无法计算视差（{num_features0},{num_features1} < {feat_thresh}）"
                );
                return None;
            }
            log::debug!(
                "[init]: disparity is {avg_disp0:.3},{avg_disp1:.3} (thresh {:.2})",
                self.params.init_max_disparity
            );
            disparity_detected_moving_1to0 = avg_disp0 > self.params.init_max_disparity;
            disparity_detected_moving_2to1 = avg_disp1 > self.params.init_max_disparity;
        }

        // 3. 选择路径（对照 C++ 的 has_jerk/is_still 分支）
        let static_detected = !disparity_detected_moving_1to0;
        let has_jerk = static_detected && disparity_detected_moving_2to1;
        let is_still = static_detected && !disparity_detected_moving_2to1;
        let use_static = (has_jerk && wait_for_jerk) || (is_still && !wait_for_jerk);
        if use_static && self.params.init_imu_thresh > 0.0 {
            log::debug!("[init]: USING STATIC INITIALIZER METHOD!");
            // 同步 IMU 缓冲（静态初始化器持有自己的副本；对照 C++ 共享指针）
            self.init_static.imu_data = self.imu_data.clone();
            return self.init_static.initialize(wait_for_jerk);
        }
        if self.params.init_dyn_use
            && (disparity_detected_moving_1to0 || disparity_detected_moving_2to1)
        {
            log::debug!("[init]: USING DYNAMIC INITIALIZER METHOD!");
            self.init_dynamic.imu_data = self.imu_data.clone();
            return self.init_dynamic.initialize(db);
        }
        let msg = if has_jerk {
            String::new()
        } else if is_still {
            "no accel jerk detected".to_string()
        } else {
            "no accel jerk detected, platform moving too much".to_string()
        };
        log::info!("[init]: failed static init: {msg}");
        None
    }
}

/// 计算所有特征的平均视差（对照 `FeatureHelper::compute_disparity` 的
/// "all features" 重载；`newest_time`/`oldest_time` 为 -1 时不设界）。
///
/// 返回 `(均值, 样本标准差, 参与特征数)`；少于 2 个视差时返回 `(-1,-1,0)`。
#[must_use]
pub fn compute_disparity(
    db: &FeatureDatabase,
    newest_time: f64,
    oldest_time: f64,
) -> (f64, f64, usize) {
    let mut disparities: Vec<f64> = Vec::new();
    for feat in db.iter_features() {
        for (cam_id, ts) in &feat.timestamps {
            // 至少两个观测才可能有视差（对照 C++ 的 size()<2 continue）
            if ts.len() < 2 {
                continue;
            }
            let uvs = &feat.uvs[cam_id];
            let mut found0 = false;
            let mut found1 = false;
            let mut uv0 = Vector2::zeros();
            let mut uv1 = Vector2::zeros();
            for (idx, time) in ts.iter().enumerate() {
                let old_ok = (oldest_time - (-1.0)).abs() < f64::EPSILON || *time > oldest_time;
                if old_ok && !found0 {
                    uv0 = uvs[idx];
                    found0 = true;
                } else {
                    let new_ok = (newest_time - (-1.0)).abs() < f64::EPSILON || *time < newest_time;
                    if new_ok && found0 {
                        uv1 = uvs[idx];
                        found1 = true;
                    }
                }
            }
            if found0 && found1 {
                disparities.push(f64::from((uv1 - uv0).norm()));
            }
        }
    }

    if disparities.len() < 2 {
        return (-1.0, -1.0, 0);
    }
    let n = disparities.len() as f64;
    let mean = disparities.iter().sum::<f64>() / n;
    let var = (disparities.iter().map(|d| (d - mean).powi(2)).sum::<f64>() / (n - 1.0)).sqrt();
    (mean, var, disparities.len())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::options::InitOptions;
    use firefly_vio_core::feat::FeatureDatabase;

    #[test]
    fn feed_imu_appends_and_cleans() {
        let mut init = InertialInitializer::new(InitOptions::default());
        for t in 0..10 {
            init.feed_imu(
                &ImuData {
                    timestamp: f64::from(t),
                    wm: nalgebra::Vector3::zeros(),
                    am: nalgebra::Vector3::zeros(),
                },
                3.0,
            );
        }
        // 只保留 >= 3.0 的测量
        assert!(init.imu_data.iter().all(|d| d.timestamp >= 3.0));
        assert_eq!(init.imu_data.len(), 7);
    }

    #[test]
    fn initialize_with_empty_db_returns_none() {
        let mut init = InertialInitializer::new(InitOptions::default());
        let mut db = FeatureDatabase::new();
        assert!(init.initialize(&mut db, true).is_none());
    }

    #[test]
    fn compute_disparity_empty_db() {
        let db = FeatureDatabase::new();
        assert_eq!(compute_disparity(&db, -1.0, -1.0), (-1.0, -1.0, 0));
    }
}
