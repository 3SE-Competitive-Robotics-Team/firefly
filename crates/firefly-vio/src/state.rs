//! MSCKF 滤波器状态（对照 `OpenVINS` `ov_msckf/state/State.h/.cpp`）。
//!
//! `State` 持有滑动窗口克隆、IMU 状态、标定变量与全协方差矩阵。
//! 协方差的增广/边缘化等操作由 [`crate::state_helper`] 完成（C++ 中
//! `StateHelper` 是 `State` 的 friend，Rust 中通过公开方法访问）。

use std::collections::{BTreeMap, HashMap, VecDeque};

use firefly_vio_core::imu_model::{dm, tg};
use firefly_vio_types::var::{ImuState, JplQuat, PoseJpl, Variable, VecVar};
use nalgebra::{DMatrix, DVector, Matrix3, SVector};

use crate::landmark::Landmark;
use crate::options::{ImuModel, StateOptions};

/// 滤波器状态（对照 `State`）。
#[derive(Debug, Clone)]
pub struct State {
    /// 当前时间戳（相机时钟系，最后一次更新时刻；对照 `_timestamp`）。
    pub timestamp: f64,
    /// 滤波器选项。
    pub options: StateOptions,
    /// 活动 IMU 状态（`q_GtoI`、`p_IinG`、`v_IinG`、`bg`、`ba`；对照 `_imu`）。
    pub imu: ImuState,
    /// 滑动窗口克隆（成像时刻 → 位姿；对照 `_clones_IMU`）。
    ///
    /// 按时间升序的 `VecDeque`（尾部追加、头部边缘化）；窗口 ≤ `max_clone_size`
    /// 个克隆，线性查找代价可忽略。f64 时间戳无 `Ord`，故不用 `BTreeMap`
    /// （C++ `std::map` 的 `operator<` 对 NaN 未定义，此处语义更安全）。
    pub clones_imu: VecDeque<(f64, PoseJpl)>,
    /// 相机到 IMU 时间偏移（对照 `_calib_dt_CAMtoIMU`；未校准时为 None）。
    pub calib_dt_cam_to_imu: Option<VecVar>,
    /// 各相机 IMU→相机 外参（对照 `_calib_IMUtoCAM`）。
    pub calib_imu_to_cam: BTreeMap<usize, PoseJpl>,
    /// 各相机内参（8 维；对照 `_cam_intrinsics`）。
    pub cam_intrinsics: BTreeMap<usize, VecVar>,
    /// 各相机的畸变模型对象（对照 `_cam_intrinsics_cameras`；由 `VioManager`
    /// 构造时从标定文件加载，EKF 更新与三角化使用其投影/雅可比）。
    pub cameras: BTreeMap<usize, firefly_vio_core::cam::SharedCamera>,
    /// 当前 SLAM 特征集（featid → Landmark；对照 `_features_SLAM`）。
    pub features_slam: HashMap<usize, Landmark>,
    /// 陀螺尺度/错切（对照 `_calib_imu_dw`）。
    pub calib_imu_dw: VecVar,
    /// 加速度计尺度/错切（对照 `_calib_imu_da`）。
    pub calib_imu_da: VecVar,
    /// 陀螺重力敏感阵（对照 `_calib_imu_tg`）。
    pub calib_imu_tg: VecVar,
    /// 陀螺系到 IMU 系旋转（kalibr 模型标定；对照 `_calib_imu_GYROtoIMU`）。
    pub calib_imu_gyro_to_imu: JplQuat,
    /// 加速度计系到 IMU 系旋转（rpng 模型标定；对照 `_calib_imu_ACCtoIMU`）。
    pub calib_imu_acc_to_imu: JplQuat,
    /// 全协方差（对照 `_Cov`）。
    pub cov: DMatrix<f64>,
}

impl State {
    /// 构造（对照 `State::State(StateOptions&)`：变量布局 + 初始协方差）。
    // 与 C++ 构造函数 1:1 移植的长流程，拆分会破坏对照可审计性。
    #[allow(clippy::too_many_lines)]
    #[must_use]
    pub fn new(options: StateOptions) -> Self {
        let mut st = Self {
            timestamp: -1.0,
            options,
            imu: ImuState::default(),
            clones_imu: VecDeque::new(),
            calib_dt_cam_to_imu: None,
            calib_imu_to_cam: BTreeMap::new(),
            cam_intrinsics: BTreeMap::new(),
            cameras: BTreeMap::new(),
            features_slam: HashMap::new(),
            calib_imu_dw: VecVar::new(6),
            calib_imu_da: VecVar::new(6),
            calib_imu_tg: VecVar::new(9),
            calib_imu_gyro_to_imu: JplQuat::default(),
            calib_imu_acc_to_imu: JplQuat::default(),
            cov: DMatrix::zeros(0, 0),
        };

        // IMU 固有标定默认值（对照 State.cpp 的 `_imu_default`：
        // 尺度 1、错切 0；`Dm()` 按模型解释布局）
        let imu_default = DVector::from_column_slice(&[1.0, 0.0, 0.0, 1.0, 0.0, 1.0]);
        st.calib_imu_dw.set_value(imu_default.clone());
        st.calib_imu_dw.set_fej(imu_default.clone());
        st.calib_imu_da.set_value(imu_default.clone());
        st.calib_imu_da.set_fej(imu_default);

        // 变量布局（对照 State.cpp 的 current_id 顺序）
        let mut current_id = 0usize;
        st.imu.set_local_id(current_id as i32);
        current_id += st.imu.size();

        if st.options.do_calib_imu_intrinsics {
            st.calib_imu_dw.set_local_id(current_id as i32);
            current_id += 6;
            st.calib_imu_da.set_local_id(current_id as i32);
            current_id += 6;
            if st.options.do_calib_imu_g_sensitivity {
                st.calib_imu_tg.set_local_id(current_id as i32);
                current_id += 9;
            }
            // kalibr 标定 R_GYROtoIMU；rpng 标定 R_ACCtoIMU
            if st.options.imu_model == ImuModel::Kalibr {
                st.calib_imu_gyro_to_imu.set_local_id(current_id as i32);
                current_id += 3;
            } else {
                st.calib_imu_acc_to_imu.set_local_id(current_id as i32);
                current_id += 3;
            }
        }

        if st.options.do_calib_camera_timeoffset {
            let mut dt = VecVar::new(1);
            dt.set_local_id(current_id as i32);
            current_id += 1;
            st.calib_dt_cam_to_imu = Some(dt);
        }

        for i in 0..st.options.num_cameras {
            let mut pose = PoseJpl::default();
            let mut intrin = VecVar::new(8);
            if st.options.do_calib_camera_pose {
                pose.set_local_id(current_id as i32);
                current_id += 6;
            }
            if st.options.do_calib_camera_intrinsics {
                intrin.set_local_id(current_id as i32);
                current_id += 8;
            }
            st.calib_imu_to_cam.insert(i, pose);
            st.cam_intrinsics.insert(i, intrin);
        }

        // 初始协方差：1e-3² 单位阵 + 各标定先验（对照 State.cpp）
        st.cov = 1e-6 * DMatrix::identity(current_id, current_id);
        if st.options.do_calib_imu_intrinsics {
            let set_block = |cov: &mut DMatrix<f64>, id: i32, n: usize, sigma: f64| {
                let id = id as usize;
                let mut block = DMatrix::identity(n, n);
                block *= sigma * sigma;
                cov.view_mut((id, id), (n, n)).copy_from(&block);
            };
            set_block(&mut st.cov, st.calib_imu_dw.id(), 6, 0.005);
            set_block(&mut st.cov, st.calib_imu_da.id(), 6, 0.008);
            if st.options.do_calib_imu_g_sensitivity {
                set_block(&mut st.cov, st.calib_imu_tg.id(), 9, 0.005);
            }
            if st.options.imu_model == ImuModel::Kalibr {
                set_block(&mut st.cov, st.calib_imu_gyro_to_imu.id(), 3, 0.005);
            } else {
                set_block(&mut st.cov, st.calib_imu_acc_to_imu.id(), 3, 0.005);
            }
        }
        if let Some(dt) = &st.calib_dt_cam_to_imu {
            let id = dt.id() as usize;
            st.cov[(id, id)] = 0.01 * 0.01;
        }
        if st.options.do_calib_camera_pose {
            for pose in st.calib_imu_to_cam.values() {
                let id = pose.id() as usize;
                st.cov
                    .view_mut((id, id), (3, 3))
                    .copy_from(&(0.005 * 0.005 * Matrix3::identity()));
                st.cov
                    .view_mut((id + 3, id + 3), (3, 3))
                    .copy_from(&(0.015 * 0.015 * Matrix3::identity()));
            }
        }
        if st.options.do_calib_camera_intrinsics {
            for intrin in st.cam_intrinsics.values() {
                let id = intrin.id() as usize;
                st.cov
                    .view_mut((id, id), (4, 4))
                    .copy_from(&DMatrix::identity(4, 4));
                st.cov
                    .view_mut((id + 4, id + 4), (4, 4))
                    .copy_from(&(0.005 * 0.005 * DMatrix::identity(4, 4)));
            }
        }

        st
    }

    /// 下一次将被边缘化的克隆时刻（对照 `State::margtimestep`；无克隆返回 -1）。
    #[must_use]
    pub fn marg_timestep(&self) -> f64 {
        self.clones_imu.front().map_or(-1.0, |(t, _)| *t)
    }

    /// 当前协方差维度（对照 `State::max_covariance_size`）。
    #[must_use]
    pub fn max_covariance_size(&self) -> usize {
        self.cov.nrows()
    }

    /// 陀螺尺度/错切矩阵（对照 `State::Dm` 的 kalibr/rpng 布局）。
    #[must_use]
    pub fn dm(&self) -> Matrix3<f64> {
        let model = match self.options.imu_model {
            ImuModel::Kalibr => firefly_vio_core::imu_model::ImuModel::Kalibr,
            ImuModel::Rpng => firefly_vio_core::imu_model::ImuModel::Rpng,
        };
        let v = SVector::<f64, 6>::from_column_slice(self.calib_imu_dw.vec().as_slice());
        dm(model, &v)
    }

    /// 加速度计尺度/错切矩阵（对照 `State::Dm` 的 kalibr/rpng 布局）。
    #[must_use]
    pub fn da(&self) -> Matrix3<f64> {
        let model = match self.options.imu_model {
            ImuModel::Kalibr => firefly_vio_core::imu_model::ImuModel::Kalibr,
            ImuModel::Rpng => firefly_vio_core::imu_model::ImuModel::Rpng,
        };
        let v = SVector::<f64, 6>::from_column_slice(self.calib_imu_da.vec().as_slice());
        dm(model, &v)
    }

    /// 重力敏感矩阵（对照 `State::Tg`，列主序填充）。
    #[must_use]
    pub fn tg(&self) -> Matrix3<f64> {
        let v = SVector::<f64, 9>::from_column_slice(self.calib_imu_tg.vec().as_slice());
        tg(&v)
    }

    /// 所有在协方差中的变量 `(id, size)`，按协方差顺序（对照 `_variables`）。
    ///
    /// 顺序：IMU(15) → IMU 标定（dw/da/tg/旋转）→ 时间偏移 → 相机外参/内参
    /// → 滑动窗口克隆。供 `StateHelper` 的 EKF 循环使用。
    #[must_use]
    pub fn variable_order(&self) -> Vec<(i32, usize)> {
        let mut order = Vec::new();
        order.push((self.imu.id(), self.imu.size()));
        if self.options.do_calib_imu_intrinsics {
            order.push((self.calib_imu_dw.id(), 6));
            order.push((self.calib_imu_da.id(), 6));
            if self.options.do_calib_imu_g_sensitivity {
                order.push((self.calib_imu_tg.id(), 9));
            }
            if self.options.imu_model == ImuModel::Kalibr {
                order.push((self.calib_imu_gyro_to_imu.id(), 3));
            } else {
                order.push((self.calib_imu_acc_to_imu.id(), 3));
            }
        }
        if let Some(dt) = &self.calib_dt_cam_to_imu {
            order.push((dt.id(), 1));
        }
        for i in 0..self.options.num_cameras {
            if let Some(pose) = self.calib_imu_to_cam.get(&i)
                && pose.id() >= 0
            {
                order.push((pose.id(), 6));
            }
            if let Some(intrin) = self.cam_intrinsics.get(&i)
                && intrin.id() >= 0
            {
                order.push((intrin.id(), 8));
            }
        }
        for (_, clone) in &self.clones_imu {
            order.push((clone.id(), 6));
        }
        // SLAM 特征（对照 C++ 的 `_variables` 末尾）。
        // 注意：遍历 HashMap 顺序不定，但调用方只按 (id,size) 索引进协方差，
        // 与顺序无关。
        for landmark in self.features_slam.values() {
            if landmark.id() >= 0 {
                order.push((landmark.id(), landmark.size()));
            }
        }
        order
    }

    /// 用误差增量 `dx` 更新所有变量（对照 `EKFUpdate` 末尾的逐变量
    /// `update` 循环：四元数 boxplus / 向量加法）。
    pub fn update_all(&mut self, dx: &DVector<f64>) {
        let update_var = |id: i32, size: usize, v: &mut dyn Variable| {
            if id < 0 {
                return;
            }
            let id = id as usize;
            v.update(&dx.rows_range(id..id + size).into_owned());
        };
        update_var(self.imu.id(), 15, &mut self.imu);
        if self.options.do_calib_imu_intrinsics {
            update_var(self.calib_imu_dw.id(), 6, &mut self.calib_imu_dw);
            update_var(self.calib_imu_da.id(), 6, &mut self.calib_imu_da);
            if self.options.do_calib_imu_g_sensitivity {
                update_var(self.calib_imu_tg.id(), 9, &mut self.calib_imu_tg);
            }
            if self.options.imu_model == ImuModel::Kalibr {
                update_var(
                    self.calib_imu_gyro_to_imu.id(),
                    3,
                    &mut self.calib_imu_gyro_to_imu,
                );
            } else {
                update_var(
                    self.calib_imu_acc_to_imu.id(),
                    3,
                    &mut self.calib_imu_acc_to_imu,
                );
            }
        }
        if let Some(dt) = &mut self.calib_dt_cam_to_imu {
            update_var(dt.id(), 1, dt);
        }
        for i in 0..self.options.num_cameras {
            if let Some(pose) = self.calib_imu_to_cam.get_mut(&i) {
                update_var(pose.id(), 6, pose);
            }
            if let Some(intrin) = self.cam_intrinsics.get_mut(&i) {
                update_var(intrin.id(), 8, intrin);
            }
        }
        for (_, clone) in &mut self.clones_imu {
            update_var(clone.id(), 6, clone);
        }
        // SLAM 特征（对照 C++ EKFUpdate 末尾的 landmark 更新循环）
        for landmark in self.features_slam.values_mut() {
            if landmark.id() < 0 {
                continue;
            }
            let id = landmark.id() as usize;
            landmark.update(&dx.rows_range(id..id + landmark.size()).into_owned());
        }
    }

    /// 把 `id > marg_id` 的变量 id 前移 `marg_size`（对照 `marginalize` 的
    /// 变量重排段；仅作用于克隆与标定变量——IMU 固定位于 0..15）。
    pub(crate) fn renumber_after(&mut self, marg_id: i32, marg_size: usize) {
        let shift = |v: &mut dyn Variable| {
            let id = v.id();
            if id > marg_id {
                v.set_local_id(id - marg_size as i32);
            }
        };
        shift(&mut self.calib_imu_dw);
        shift(&mut self.calib_imu_da);
        shift(&mut self.calib_imu_tg);
        shift(&mut self.calib_imu_gyro_to_imu);
        shift(&mut self.calib_imu_acc_to_imu);
        if let Some(dt) = &mut self.calib_dt_cam_to_imu {
            shift(dt);
        }
        for pose in self.calib_imu_to_cam.values_mut() {
            shift(pose);
        }
        for intrin in self.cam_intrinsics.values_mut() {
            shift(intrin);
        }
        for (_, clone) in &mut self.clones_imu {
            shift(clone);
        }
        for landmark in self.features_slam.values_mut() {
            if landmark.id() > marg_id {
                landmark.set_local_id(landmark.id() - marg_size as i32);
            }
        }
    }

    /// 当前 IMU 标定（供 `firefly-vio-core` 的传播使用）。
    #[must_use]
    pub fn imu_calibration(&self) -> firefly_vio_core::imu_model::ImuCalibration {
        firefly_vio_core::imu_model::ImuCalibration {
            bias_a: self.imu.bias_a(),
            bias_g: self.imu.bias_g(),
            r_acc_to_imu: self.calib_imu_acc_to_imu.rot(),
            r_gyro_to_imu: self.calib_imu_gyro_to_imu.rot(),
            da: self.da(),
            dw: self.dm(),
            tg: self.tg(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Vector3, Vector4};

    #[test]
    fn variable_layout_matches_cpp() {
        // 无标定：只有 IMU 15 维
        let s = State::new(StateOptions::default());
        assert_eq!(s.cov.nrows(), 15);
        assert_eq!(s.imu.id(), 0);
        assert_eq!(s.imu.size(), 15);

        // 开 IMU 内参 + 重力敏感（kalibr）：15 + 6 + 6 + 9 + 3 = 39
        let opts = StateOptions {
            do_calib_imu_intrinsics: true,
            do_calib_imu_g_sensitivity: true,
            ..StateOptions::default()
        };
        let s = State::new(opts);
        assert_eq!(s.cov.nrows(), 39);
        assert_eq!(s.calib_imu_dw.id(), 15);
        assert_eq!(s.calib_imu_da.id(), 21);
        assert_eq!(s.calib_imu_tg.id(), 27);
        assert_eq!(s.calib_imu_gyro_to_imu.id(), 36);

        // rpng：标定 R_ACCtoIMU 而非 R_GYROtoIMU
        let opts = StateOptions {
            do_calib_imu_intrinsics: true,
            imu_model: ImuModel::Rpng,
            ..StateOptions::default()
        };
        let s = State::new(opts);
        assert_eq!(s.calib_imu_acc_to_imu.id(), 27);
        assert_eq!(s.calib_imu_gyro_to_imu.id(), -1);

        // 双相机 + 外参/内参标定
        let opts = StateOptions {
            num_cameras: 2,
            do_calib_camera_pose: true,
            do_calib_camera_intrinsics: true,
            ..StateOptions::default()
        };
        let s = State::new(opts);
        // 15 + 2*(6+8) = 43
        assert_eq!(s.cov.nrows(), 43);
        assert_eq!(s.calib_imu_to_cam.get(&0).unwrap().id(), 15);
        assert_eq!(s.cam_intrinsics.get(&0).unwrap().id(), 21);
    }

    #[test]
    fn calibration_defaults_match_cpp() {
        let s = State::new(StateOptions::default());
        // Dm 默认（kalibr 行主序解释）：[[1,0,0],[0,1,0],[0,0,1]]
        assert_eq!(s.dm(), Matrix3::identity());
        assert_eq!(s.da(), Matrix3::identity());
        assert_eq!(s.tg(), Matrix3::zeros());
    }

    #[test]
    fn initial_covariance_blocks() {
        let opts = StateOptions {
            do_calib_imu_intrinsics: true,
            ..StateOptions::default()
        };
        let s = State::new(opts);
        assert!((s.cov[(15, 15)] - 0.005f64.powi(2)).abs() < 1e-12);
        assert!((s.cov[(21, 21)] - 0.008f64.powi(2)).abs() < 1e-12);
        // IMU 主块 1e-6
        assert!((s.cov[(0, 0)] - 1e-6).abs() < 1e-12);
    }

    #[test]
    fn imu_set_value_roundtrip() {
        let mut s = State::new(StateOptions::default());
        s.imu.set_value(
            Vector4::new(0.0, 0.0, 0.0, 1.0),
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::new(0.1, 0.2, 0.3),
            Vector3::new(0.01, 0.02, 0.03),
            Vector3::new(0.001, 0.002, 0.003),
        );
        assert_eq!(s.imu.pos(), Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(s.imu.vel(), Vector3::new(0.1, 0.2, 0.3));
        assert_eq!(s.imu.bias_g(), Vector3::new(0.01, 0.02, 0.03));
        assert_eq!(s.imu.bias_a(), Vector3::new(0.001, 0.002, 0.003));
        // set_value 只更新 value；FEJ 需显式设置（对照 C++）
        let fej = s.imu.pose().pos_fej();
        assert!(fej.norm() < 1e-12);
    }

    #[test]
    fn marg_timestep_empty() {
        let s = State::new(StateOptions::default());
        assert!((s.marg_timestep() - (-1.0)).abs() < 1e-12);
    }
}
