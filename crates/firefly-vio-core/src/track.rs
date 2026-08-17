//! KLT 视觉前端（对照 `OpenVINS` `ov_core/src/track`：`TrackBase`/`TrackKLT`/`Grider_GRID`）。
//!
//! 本模块实现 OpenVINS 的 KLT 稀疏特征跟踪前端，并把所依赖的 OpenCV 算法全部
//! 自实现（不引入 `opencv` crate）：
//! - [`histogram`]：直方图均衡（`equalizeHist`）与 CLAHE（`createCLAHE`）；
//! - [`pyramid`]：光流图像金字塔（`buildOpticalFlowPyramid`）；
//! - [`fast`]：FAST-9 角点检测 + 非极大值抑制（`cv::FAST`）；
//! - [`grider`]：网格自适应 FAST 提取 + `cornerSubPix`（`Grider_GRID::perform_griding`）；
//! - [`lk`]：金字塔 LK 光流（`calcOpticalFlowPyrLK`）；
//! - [`fundamental`]：归一化 8 点法 + RANSAC 基础矩阵（`findFundamentalMat`）。
//!
//! [`TrackKlt`] 提供完整跟踪器（对照 `TrackKLT.cpp`），消费 `crate::feat` 的
//! `FeatureDatabase`，按时间跟踪（单目）与双目一致性跟踪，并把去畸变归一化
//! 坐标写入特征库。
//!
//! 可视化（`display_active`/`display_history`）无 GUI，不移植，见 [`TrackerBase`]。
//!
//! 说明：双目/网格代码中左右（`_l`/`_r`）变量与数学单字符名、OpenCV/OpenVINS
//! 文档引用、镜像 C++ 的多参数签名属于固有结构，予以模块级允许。
#![allow(
    clippy::doc_markdown,
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::too_many_arguments
)]

use crate::cam::CameraModel;
use crate::feat::FeatureDatabase;
use crate::sensor::{CameraData, GrayImage};
use std::collections::HashMap;

pub mod fast;
pub mod fundamental;
pub mod grider;
pub mod histogram;
pub mod lk;
pub mod pyramid;

/// 图像预处理方法（对照 `TrackBase.h` 的 `HistogramMethod` 枚举）。
///
/// 送入跟踪前的图像增强方式：
/// - [`HistogramMethod::None`]：不做处理；
/// - [`HistogramMethod::Histogram`]：全局直方图均衡（`cv::equalizeHist`）；
/// - [`HistogramMethod::Clahe`]：CLAHE（`cv::CLAHE`，8×8 网格）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistogramMethod {
    /// 不做直方图预处理。
    None,
    /// 全局直方图均衡（`cv::equalizeHist`）。
    Histogram,
    /// 对比度受限自适应直方图均衡（`cv::createCLAHE(10.0, Size(8,8))`）。
    Clahe,
}

/// 二维角点（对照 `cv::KeyPoint`：位置 + FAST 响应强度）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KeyPoint {
    /// u 坐标（像素）。
    pub x: f32,
    /// v 坐标（像素）。
    pub y: f32,
    /// FAST / 角点响应强度（用于按强度排序与非极大值抑制）。
    pub response: f32,
}

impl KeyPoint {
    /// 由像素坐标构造角点（响应默认 0）。
    #[must_use]
    pub fn new(x: f32, y: f32) -> Self {
        Self {
            x,
            y,
            response: 0.0,
        }
    }
}

/// 像素 u、v 是否落在图像边界内（`0 <= x < width`，`0 <= y < height`）。
#[must_use]
fn in_bounds(img: &GrayImage, x: i32, y: i32) -> bool {
    x >= 0 && y >= 0 && x < img.width as i32 && y < img.height as i32
}

/// 掩码查询 helper：返回该像素是否被掩码覆盖（`>127`，勿提取特征）。
///
/// 区分 C++ 的 `cv::Mat::at<uint8_t>` 语义；边界外视为未掩码处 0 处理，不 panic。
#[must_use]
fn mask_on(mask: &GrayImage, x: i32, y: i32) -> bool {
    in_bounds(mask, x, y) && mask.data[(y as usize) * mask.width + (x as usize)] > 127
}

/// 把相机的 `i32` 传感器 id 换算为特征库使用的 `usize` 键。
#[must_use]
fn sensor_key(id: i32) -> usize {
    id.max(0) as usize
}

// ============================================================================
// 跟踪器（对照 `ov_core/src/track/TrackBase.h` / `TrackKLT.cpp`）
// ============================================================================

/// 跟踪器基础结构（对照 `TrackBase.h` 的公共状态与接口）。
///
/// 字段含义（对照 `TrackBase.h`）：
/// - `camera_calib`：相机 id → 相机标定（`std::map<size_t, shared_ptr<CamBase>>`）；
/// - `database`：特征库（`FeatureDatabase`，存储所有跟踪测量）；
/// - `use_stereo`：是否双目一致性跟踪（`TrackBase::use_stereo`）；
/// - `histogram_method`：直方图预处理方式（`TrackBase::histogram_method`）。
///
/// `currid`（全局特征 id 游标）在 C++ 中为 `std::atomic<size_t>`（支持多线程）；
/// 本单线程移植用 [`TrackKlt::currid`] 的 `usize` 字段表示。
///
/// 可视化接口（`display_active`/`display_history`）依赖 GUI，超出本 crate 范畴
/// 不移植；`change_feat_id` 依赖特征库内部重排接口，见 [`TrackerBase::change_feat_id`] 文档。
#[derive(Debug)]
pub struct TrackerBase {
    /// 相机 id → 相机标定（对照 `TrackBase::camera_calib`）。
    pub camera_calib: HashMap<usize, std::sync::Arc<dyn CameraModel>>,
    /// 特征库（对照 `TrackBase::database`）。
    pub database: FeatureDatabase,
    /// 是否双目（对照 `TrackBase::use_stereo`）。
    pub use_stereo: bool,
    /// 直方图预处理方法（对照 `TrackBase::histogram_method`）。
    pub histogram_method: HistogramMethod,
}

impl TrackerBase {
    /// 构造 `TrackerBase`（对照 `TrackBase` 构造函数；创建空的 `FeatureDatabase`）。
    #[must_use]
    pub fn new(
        camera_calib: HashMap<usize, std::sync::Arc<dyn CameraModel>>,
        use_stereo: bool,
        histogram_method: HistogramMethod,
    ) -> Self {
        Self {
            camera_calib,
            database: FeatureDatabase::new(),
            use_stereo,
            histogram_method,
        }
    }

    /// 特征库只读引用（对照 `TrackBase::get_feature_database`）。
    #[must_use]
    pub fn database(&self) -> &FeatureDatabase {
        &self.database
    }

    /// 特征库可变引用。
    pub fn database_mut(&mut self) -> &mut FeatureDatabase {
        &mut self.database
    }

    /// 更改特征库中特征 id（对照 `TrackBase::change_feat_id` 的数据库部分）。
    ///
    /// 经 [`FeatureDatabase::remap_feature_id`] 重排 id（冲突时保留新 id 已有
    /// 特征）；跟踪器侧的 `ids_last` 更新见 [`TrackKlt::change_feat_id`]。
    pub fn change_feat_id(&mut self, old: usize, new: usize) {
        self.database.remap_feature_id(old, new);
    }
}

/// 单相机的一帧跟踪状态（聚合 `TrackBase` 的 `img_last/pts_last/ids_last`
/// 与 `TrackKLT` 的 `img_curr/img_pyramid_curr`）。
#[derive(Debug)]
struct CamState {
    /// 当前帧预处理图像（`img_curr`）。
    img_curr: GrayImage,
    /// 当前帧金字塔（`img_pyramid_curr`）。
    pyramid_curr: Vec<GrayImage>,
    /// 上一帧图像（`img_last`）。
    img_last: GrayImage,
    /// 上一帧金字塔（`img_pyramid_last`）。
    pyramid_last: Vec<GrayImage>,
    /// 上一帧掩码（`img_mask_last`）。
    mask_last: GrayImage,
    /// 上一帧成功跟踪点（`pts_last`）。
    pts_last: Vec<KeyPoint>,
    /// 上一帧各点特征 id（`ids_last`）。
    ids_last: Vec<usize>,
}

impl Default for CamState {
    fn default() -> Self {
        fn empty() -> GrayImage {
            GrayImage {
                width: 0,
                height: 0,
                data: Vec::new(),
            }
        }
        Self {
            img_curr: empty(),
            pyramid_curr: Vec::new(),
            img_last: empty(),
            pyramid_last: Vec::new(),
            mask_last: empty(),
            pts_last: Vec::new(),
            ids_last: Vec::new(),
        }
    }
}

/// KLT 特征跟踪器（对照 `TrackKLT.cpp` / `TrackKLT.h`）。
///
/// 构造参数对照 `TrackKLT`：`numfeats`→`num_features`、`numaruco`→`num_aruco`、
/// `stereo`→`use_stereo`、`histmethod`→`histogram_method`、`fast_threshold`、
/// `gridx`/`gridy`、`minpxdist`。
///
/// 跟踪流程对照 `TrackKLT.cpp`：
/// - [`TrackKlt::feed_new_camera`]：预处理 + 分发单目/双目；
/// - [`TrackKlt::feed_monocular`]：mesh 补点 → 时间 LK+RANSAC → 入库；
/// - [`TrackKlt::feed_stereo`]：左右时间跟踪 → 双目一致性 → 入库；
/// - [`TrackKlt::perform_detection_monocular`]：对应 `TrackKLT::perform_detection_monocular`；
/// - [`TrackKlt::perform_matching`]：对应 `TrackKLT::perform_matching`（LK + 基础矩阵 RANSAC）。
#[derive(Debug)]
pub struct TrackKlt {
    base: TrackerBase,
    num_features: usize,
    fast_threshold: i32,
    grid_x: usize,
    grid_y: usize,
    min_px_dist: i32,
    /// 全局特征 id 游标（C++ `std::atomic<size_t> currid`；单线程 `usize`）。
    currid: usize,
    /// 每相机跟踪状态。
    cams: HashMap<usize, CamState>,
}

impl TrackKlt {
    /// 构造 KLT 跟踪器（对照 `TrackKLT` 构造函数）。
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        camera_calib: HashMap<usize, std::sync::Arc<dyn CameraModel>>,
        num_features: usize,
        num_aruco: usize,
        use_stereo: bool,
        histogram_method: HistogramMethod,
        fast_threshold: i32,
        grid_x: usize,
        grid_y: usize,
        min_px_dist: i32,
    ) -> Self {
        // 起始 id 大于 aruco 角点数（每个标签 4 角），同 TrackBase 构造函数。
        let currid = 4 * num_aruco + 1;
        Self {
            base: TrackerBase::new(camera_calib, use_stereo, histogram_method),
            num_features,
            fast_threshold,
            grid_x,
            grid_y,
            min_px_dist,
            currid,
            cams: HashMap::new(),
        }
    }

    /// 当前特征 id 游标（对照 `TrackBase::currid`）。
    #[must_use]
    pub fn currid(&self) -> usize {
        self.currid
    }

    /// 特征库只读引用。
    #[must_use]
    pub fn database(&self) -> &FeatureDatabase {
        self.base.database()
    }

    /// 特征库可变引用。
    pub fn database_mut(&mut self) -> &mut FeatureDatabase {
        self.base.database_mut()
    }

    /// 设置跟踪特征数上限（对照 `TrackBase::set_num_features`）。
    pub fn set_num_features(&mut self, num_features: usize) {
        self.num_features = num_features;
    }

    /// 更改当前跟踪特征 id（对照 `TrackBase::change_feat_id` 的完整语义）。
    ///
    /// 同时更新 `FeatureDatabase`（经 `TrackerBase::change_feat_id`）与各相机
    /// 上一帧的 `ids_last` 记录。
    pub fn change_feat_id(&mut self, old: usize, new: usize) {
        self.base.change_feat_id(old, new);
        for state in self.cams.values_mut() {
            for id in &mut state.ids_last {
                if *id == old {
                    *id = new;
                }
            }
        }
    }

    /// 上一帧某相机的跟踪点（对照 `TrackBase::get_last_obs`）。
    #[must_use]
    pub fn last_points(&self, cam_sensor_id: i32) -> &[KeyPoint] {
        self.cams
            .get(&sensor_key(cam_sensor_id))
            .map_or(&[], |s| s.pts_last.as_slice())
    }

    /// 上一帧某相机的特征 id（对照 `TrackBase::get_last_ids`）。
    #[must_use]
    pub fn last_ids(&self, cam_sensor_id: i32) -> &[usize] {
        self.cams
            .get(&sensor_key(cam_sensor_id))
            .map_or(&[], |s| s.ids_last.as_slice())
    }

    /// 处理新相机数据（对照 `TrackKLT::feed_new_camera`）。
    ///
    /// 分发逻辑：单图 → `feed_monocular`；双图且 `use_stereo` → `feed_stereo`；
    /// 否则对每张图逐个 `feed_monocular`（C++ 中配双目但 `use_stereo=false` 时并行）。
    pub fn feed_new_camera(&mut self, msg: &CameraData) {
        let valid = !msg.sensor_ids.is_empty()
            && msg.sensor_ids.len() == msg.images.len()
            && msg.images.len() == msg.masks.len();
        if !valid {
            return; // 数据不一致：忽略（C++ 直接 `exit`，这里取安全策略）
        }
        self.preprocess_into_curr(msg);
        match (msg.images.len(), self.base.use_stereo) {
            (1, _) => self.feed_monocular(msg, 0),
            (2, true) => self.feed_stereo(msg, 0, 1),
            (_, false) => {
                for i in 0..msg.images.len() {
                    self.feed_monocular(msg, i);
                }
            }
            (_, true) => { /* 非法输入，忽略（C++ exit） */ }
        }
        // 可观测性：当前帧处理后特征库大小（诊断跟踪是否产出特征）
        log::debug!(
            "tracker feed t={:.3} db_size={}",
            msg.timestamp,
            self.base.database.size()
        );
    }

    /// 对 `msg` 中所有相机做直方图预处理 + 金字塔，并将结果写入 `img_curr`/
    /// `pyramid_curr`（对照 `TrackKLT.cpp` 第 49-76 行）。
    fn preprocess_into_curr(&mut self, msg: &CameraData) {
        for (i, &sid) in msg.sensor_ids.iter().enumerate() {
            let key = sensor_key(sid);
            let src = &msg.images[i];
            let processed = match self.base.histogram_method {
                HistogramMethod::Histogram => histogram::equalize_hist(src),
                HistogramMethod::Clahe => histogram::clahe(src, &histogram::ClaheParams::default()),
                HistogramMethod::None => src.clone(),
            };
            let pyr = pyramid::build_optical_flow_pyramid(&processed, 5, lk::MIN_PYR_SIDE);
            let state = self.cams.entry(key).or_default();
            state.img_curr = processed;
            state.pyramid_curr = pyr;
        }
    }

    /// 单目跟踪（对照 `TrackKLT::feed_monocular`）。
    fn feed_monocular(&mut self, msg: &CameraData, msg_id: usize) {
        let key = sensor_key(msg.sensor_ids[msg_id]);
        let mask = msg.masks[msg_id].clone();
        let timestamp = msg.timestamp;
        let cam_sensor_id = msg.sensor_ids[msg_id];

        // 取当前帧数据（克隆副本，避免跨 `&mut self` 调用的借用冲突）
        let (img_curr, pyr_curr) = {
            let s = self.cams.get(&key).expect("camera state after preprocess");
            (s.img_curr.clone(), s.pyramid_curr.clone())
        };

        // 首帧或上帧无点 → 直接重检测
        let first_frame = self.cams.get(&key).is_none_or(|s| s.pts_last.is_empty());
        if first_frame {
            let mut good = Vec::new();
            let mut ids = Vec::new();
            self.perform_detection_monocular(&pyr_curr, &mask, &img_curr, &mut good, &mut ids);
            let s = self.cams.get_mut(&key).unwrap();
            s.pts_last = good;
            s.ids_last = ids;
            s.img_last = img_curr;
            s.pyramid_last = pyr_curr;
            s.mask_last = mask;
            return;
        }

        // 补点：在上一帧（last）上检测，保证点数足够
        let (mut pts_old, mut ids_old) = {
            let s = self.cams.get(&key).unwrap();
            (s.pts_last.clone(), s.ids_last.clone())
        };
        let (pyr_last, mask_last, img_last) = {
            let s = self.cams.get(&key).unwrap();
            (
                s.pyramid_last.clone(),
                s.mask_last.clone(),
                s.img_last.clone(),
            )
        };
        self.perform_detection_monocular(
            &pyr_last,
            &mask_last,
            &img_last,
            &mut pts_old,
            &mut ids_old,
        );

        // 时间 LK + RANSAC
        let mut pts_new = pts_old.clone();
        let mut mask_ll = Vec::new();
        self.perform_matching(
            &pyr_last,
            &pyr_curr,
            &mut pts_old,
            &mut pts_new,
            key,
            key,
            &mut mask_ll,
        );

        // RANSAC 失败 → 复位（同 C++ 的 mask_ll.empty() 分支）
        if mask_ll.is_empty() {
            let s = self.cams.get_mut(&key).unwrap();
            s.pts_last.clear();
            s.ids_last.clear();
            s.img_last = img_curr;
            s.pyramid_last = pyr_curr;
            s.mask_last = mask;
            return;
        }

        let (w, h) = (img_curr.width as i32, img_curr.height as i32);
        let mut good = Vec::new();
        let mut good_ids = Vec::new();
        for i in 0..pts_new.len() {
            let p = pts_new[i];
            if p.x < 0.0 || p.y < 0.0 || (p.x as i32) >= w || (p.y as i32) >= h {
                continue;
            }
            if mask_on(&mask, p.x as i32, p.y as i32) {
                continue;
            }
            if mask_ll[i] {
                good.push(p);
                good_ids.push(ids_old[i]);
            }
        }

        // 入库（去畸变 → 归一化坐标）
        self.insert_to_db(timestamp, cam_sensor_id, &good, &good_ids);

        let s = self.cams.get_mut(&key).unwrap();
        s.pts_last = good;
        s.ids_last = good_ids;
        s.img_last = img_curr;
        s.pyramid_last = pyr_curr;
        s.mask_last = mask;
    }

    /// 双目跟踪（对照 `TrackKLT::feed_stereo`，具体双目一致性逻辑见注释）。
    // 与 C++ 1:1 移植的长流程函数，拆分会破坏对照可审计性（仓库惯例：
    // 移植代码允许带注释的 targeted allow，同 firefly-cost 的 too_many_arguments）。
    #[allow(clippy::too_many_lines)]
    fn feed_stereo(&mut self, msg: &CameraData, idl: usize, idr: usize) {
        let keyl = sensor_key(msg.sensor_ids[idl]);
        let keyr = sensor_key(msg.sensor_ids[idr]);
        let mask_l = msg.masks[idl].clone();
        let mask_r = msg.masks[idr].clone();
        let timestamp = msg.timestamp;
        let sid_l = msg.sensor_ids[idl];
        let sid_r = msg.sensor_ids[idr];

        // 当前帧数据
        let (img_l, img_r, pyr_l, pyr_r) = {
            let sl = self.cams.get(&keyl).unwrap();
            let sr = self.cams.get(&keyr).unwrap();
            (
                sl.img_curr.clone(),
                sr.img_curr.clone(),
                sl.pyramid_curr.clone(),
                sr.pyramid_curr.clone(),
            )
        };

        let first_frame = {
            let sl = self.cams.get(&keyl).is_none_or(|s| s.pts_last.is_empty());
            let sr = self.cams.get(&keyr).is_none_or(|s| s.pts_last.is_empty());
            sl && sr
        };
        if first_frame {
            let mut gl = Vec::new();
            let mut gr = Vec::new();
            let mut il = Vec::new();
            let mut ir = Vec::new();
            self.perform_detection_stereo(
                &pyr_l, &pyr_r, &mask_l, &mask_r, &img_l, &img_r, &mut gl, &mut gr, &mut il,
                &mut ir,
            );
            let sl = self.cams.get_mut(&keyl).unwrap();
            sl.pts_last = gl;
            sl.ids_last = il;
            sl.img_last = img_l;
            sl.pyramid_last = pyr_l;
            sl.mask_last = mask_l;
            let sr = self.cams.get_mut(&keyr).unwrap();
            sr.pts_last = gr;
            sr.ids_last = ir;
            sr.img_last = img_r;
            sr.pyramid_last = pyr_r;
            sr.mask_last = mask_r;
            return;
        }

        // 补点（last 图）
        let (mut pts_l, mut pts_r, mut ids_l, mut ids_r) = {
            let sl = self.cams.get(&keyl).unwrap();
            let sr = self.cams.get(&keyr).unwrap();
            (
                sl.pts_last.clone(),
                sr.pts_last.clone(),
                sl.ids_last.clone(),
                sr.ids_last.clone(),
            )
        };
        let (pyr_l_last, pyr_r_last, mask_l_last, mask_r_last) = {
            let sl = self.cams.get(&keyl).unwrap();
            let sr = self.cams.get(&keyr).unwrap();
            (
                sl.pyramid_last.clone(),
                sr.pyramid_last.clone(),
                sl.mask_last.clone(),
                sr.mask_last.clone(),
            )
        };
        self.perform_detection_stereo(
            &pyr_l_last,
            &pyr_r_last,
            &mask_l_last,
            &mask_r_last,
            &img_l,
            &img_r,
            &mut pts_l,
            &mut pts_r,
            &mut ids_l,
            &mut ids_r,
        );

        // 左右时间 LK + RANSAC
        let (mut pts_l_new, mut pts_r_new) = (pts_l.clone(), pts_r.clone());
        let mut mask_ll = Vec::new();
        let mut mask_rr = Vec::new();
        self.perform_matching(
            &pyr_l_last,
            &pyr_l,
            &mut pts_l,
            &mut pts_l_new,
            keyl,
            keyl,
            &mut mask_ll,
        );
        self.perform_matching(
            &pyr_r_last,
            &pyr_r,
            &mut pts_r,
            &mut pts_r_new,
            keyr,
            keyr,
            &mut mask_rr,
        );

        if mask_ll.is_empty() && mask_rr.is_empty() {
            let sl = self.cams.get_mut(&keyl).unwrap();
            sl.pts_last.clear();
            sl.ids_last.clear();
            let sr = self.cams.get_mut(&keyr).unwrap();
            sr.pts_last.clear();
            sr.ids_last.clear();
            return;
        }

        let (w_l, h_l) = (img_l.width as i32, img_l.height as i32);
        let (w_r, h_r) = (img_r.width as i32, img_r.height as i32);
        let mut good_l = Vec::new();
        let mut good_r = Vec::new();
        let mut good_ids_l = Vec::new();
        let mut good_ids_r = Vec::new();

        // LEFT loop（对照 TrackKLT.cpp 第 306-338 行）
        for i in 0..pts_l_new.len() {
            let p = pts_l_new[i];
            if p.x < 0.0 || p.y < 0.0 || (p.x as i32) > w_l || (p.y as i32) > h_l {
                continue;
            }
            let mut found_r = false;
            let mut idx_r = 0usize;
            for (n, &idr) in ids_r.iter().enumerate() {
                if ids_l[i] == idr {
                    found_r = true;
                    idx_r = n;
                    break;
                }
            }
            if mask_ll[i] && found_r && mask_rr[idx_r] {
                let pr = pts_r_new[idx_r];
                if pr.x < 0.0 || pr.y < 0.0 || (pr.x as i32) >= w_r || (pr.y as i32) >= h_r {
                    continue;
                }
                good_l.push(p);
                good_r.push(pr);
                good_ids_l.push(ids_l[i]);
                good_ids_r.push(ids_r[idx_r]);
            } else if mask_ll[i] {
                good_l.push(p);
                good_ids_l.push(ids_l[i]);
            }
        }
        // RIGHT loop（对照 TrackKLT.cpp 第 341-354 行）
        for i in 0..pts_r_new.len() {
            let p = pts_r_new[i];
            if p.x < 0.0 || p.y < 0.0 || (p.x as i32) >= w_r || (p.y as i32) >= h_r {
                continue;
            }
            let already = good_ids_r.iter().any(|&x| x == ids_r[i]);
            if mask_rr[i] && !already {
                good_r.push(p);
                good_ids_r.push(ids_r[i]);
            }
        }

        // 入库
        self.insert_to_db(timestamp, sid_l, &good_l, &good_ids_l);
        self.insert_to_db(timestamp, sid_r, &good_r, &good_ids_r);

        let sl = self.cams.get_mut(&keyl).unwrap();
        sl.pts_last = good_l;
        sl.ids_last = good_ids_l;
        sl.img_last = img_l;
        sl.pyramid_last = pyr_l;
        sl.mask_last = mask_l;
        let sr = self.cams.get_mut(&keyr).unwrap();
        sr.pts_last = good_r;
        sr.ids_last = good_ids_r;
        sr.img_last = img_r;
        sr.pyramid_last = pyr_r;
        sr.mask_last = mask_r;
    }

    /// 把 `(点,id)` 对经当前相机去畸变后写入特征库（对照 `TrackKLT.cpp` 入库段）。
    fn insert_to_db(
        &mut self,
        timestamp: f64,
        cam_sensor_id: i32,
        pts: &[KeyPoint],
        ids: &[usize],
    ) {
        let key = sensor_key(cam_sensor_id);
        let Some(cam) = self.base.camera_calib.get(&key).cloned() else {
            return;
        };
        for (p, &id) in pts.iter().zip(ids) {
            let uv = nalgebra::Vector2::new(p.x, p.y);
            let n = cam.undistort_f(uv);
            self.base
                .database_mut()
                .update_feature(id, timestamp, key, p.x, p.y, n.x, n.y);
        }
    }

    /// KLT 时间/双目匹配 + RANSAC（对照 `TrackKLT::perform_matching`）。
    ///
    /// `cam_key0`/`cam_key1` 为两条图像流的相机 id（时间跟踪时相同）。`mask_out`
    /// 长度与 `kpts0` 相同，`mask_out[i]=true` 表示第 i 对点 KLT 成功且通过
    /// 基础矩阵 RANSAC。`pts1` 传入时作初始猜测（`OPTFLOW_USE_INITIAL_FLOW`）。
    fn perform_matching(
        &mut self,
        img0pyr: &[GrayImage],
        img1pyr: &[GrayImage],
        kpts0: &mut [KeyPoint],
        kpts1: &mut [KeyPoint],
        cam_key0: usize,
        cam_key1: usize,
        mask_out: &mut Vec<bool>,
    ) {
        assert_eq!(kpts0.len(), kpts1.len(), "kpts0/1 size mismatch");
        if kpts0.is_empty() {
            return;
        }
        let pts0: Vec<nalgebra::Vector2<f32>> = kpts0
            .iter()
            .map(|k| nalgebra::Vector2::new(k.x, k.y))
            .collect();
        let pts1: Vec<nalgebra::Vector2<f32>> = kpts1
            .iter()
            .map(|k| nalgebra::Vector2::new(k.x, k.y))
            .collect();

        // 点数不足做 RANSAC → 全部失败（同 C++ 第 846-852 行）
        if pts0.len() < 10 {
            mask_out.extend(std::iter::repeat_n(false, pts0.len()));
            return;
        }

        // LK（OPTFLOW_USE_INITIAL_FLOW → use_initial_flow=true）
        let (out, status) = lk::calc_optical_flow_pyr_lk(
            img0pyr,
            img1pyr,
            &pts0,
            &pts1,
            &lk::TermCriteria::default_lk(),
            true,
            lk::MIN_EIG_THRESHOLD,
        );

        // 去畸变归一化（RANSAC 需要在规范坐标上进行，同 C++ 第 860-866 行）
        let cam0 = self.base.camera_calib.get(&cam_key0).cloned();
        let cam1 = self.base.camera_calib.get(&cam_key1).cloned();
        let (Some(cam0), Some(cam1)) = (cam0, cam1) else {
            return;
        };
        let pts0_n: Vec<nalgebra::Vector2<f64>> =
            pts0.iter().map(|p| cam0.undistort_d(p.cast())).collect();
        let pts1_n: Vec<nalgebra::Vector2<f64>> =
            out.iter().map(|p| cam1.undistort_d(p.cast())).collect();

        // max_focallength = 两相机焦距最大值（同 C++ 第 870-872 行）
        let max_focal = {
            let k0 = cam0.camera_matrix();
            let k1 = cam1.camera_matrix();
            k0[(0, 0)].max(k0[(1, 1)]).max(k1[(0, 0)]).max(k1[(1, 1)])
        };
        let threshold = 2.0 / max_focal;

        // RANSAC（同 C++ 第 873 行）
        let (mask_rsc, _) = fundamental::ransac_fundamental(&pts0_n, &pts1_n, threshold, 0.999);

        // 合并 KLT 与 RANSAC 掩码（同 C++ 第 876-879 行）
        for i in 0..status.len() {
            let ok = status.get(i).copied().unwrap_or(false)
                && mask_rsc.get(i).copied().unwrap_or(false);
            mask_out.push(ok);
        }

        // 回写位置（pts0 未变，pts1 取 LK 结果；同 C++ 第 882-885 行）
        for (i, k) in kpts1.iter_mut().enumerate() {
            k.x = out[i].x;
            k.y = out[i].y;
        }
    }

    /// 单目检测（对照 `TrackKLT::perform_detection_monocular`）。
    // 与 C++ 1:1 移植的长流程函数，拆分会破坏对照可审计性。
    #[allow(clippy::too_many_lines)]
    fn perform_detection_monocular(
        &mut self,
        imgpyr: &[GrayImage],
        mask0: &GrayImage,
        img0: &GrayImage,
        pts0: &mut Vec<KeyPoint>,
        ids0: &mut Vec<usize>,
    ) {
        let min_px = self.min_px_dist.max(1) as usize;
        let (w, h) = (img0.width, img0.height);
        let _ = imgpyr;
        // 距离占用网格（每 cell = min_px² 像素）
        let close_w = w.div_ceil(min_px);
        let close_h = h.div_ceil(min_px);
        let mut grid_close = vec![0u8; close_w * close_h];
        let grid_x = self.grid_x.max(1);
        let grid_y = self.grid_y.max(1);
        let size_x = w as f32 / grid_x as f32;
        let size_y = h as f32 / grid_y as f32;
        let mut grid_count = vec![0u8; grid_x * grid_y];
        let edge = 10i32;
        let (w_i, h_i) = (w as i32, h as i32);

        // 遍历既有特征：剔除越界/掩码/过近，并用之占用 close-grid 与网格计数
        let mut keep = Vec::with_capacity(pts0.len());
        let mut keep_ids = Vec::with_capacity(ids0.len());
        for (p, &fid) in pts0.iter().zip(ids0.iter()) {
            if p.x < edge as f32
                || p.y < edge as f32
                || p.x >= (w_i - edge) as f32
                || p.y >= (h_i - edge) as f32
            {
                continue;
            }
            let xc = (p.x / min_px as f32) as usize;
            let yc = (p.y / min_px as f32) as usize;
            if xc >= close_w || yc >= close_h {
                continue;
            }
            if grid_close[yc * close_w + xc] > 127 {
                continue;
            }
            if mask_on(mask0, p.x as i32, p.y as i32) {
                continue;
            }
            let xg = (p.x / size_x).floor() as usize;
            let yg = (p.y / size_y).floor() as usize;
            if xg >= grid_x || yg >= grid_y {
                continue;
            }
            grid_close[yc * close_w + xc] = 255;
            let cell = &mut grid_count[yg * grid_x + xg];
            *cell = cell.saturating_add(1);
            keep.push(*p);
            keep_ids.push(fid);
        }
        pts0.clear();
        ids0.clear();
        pts0.extend(keep);
        ids0.extend(keep_ids);

        // 需要的补点数
        let min_feat_percent = 0.50;
        let num_needed = self.num_features.saturating_sub(pts0.len());
        let min_threshold =
            std::cmp::min(20, (min_feat_percent * self.num_features as f64) as usize);
        if num_needed < min_threshold {
            return;
        }

        // 掩码下采样到网格（INTER_NEAREST）
        let mut mask_grid = vec![0u8; grid_x * grid_y];
        for yg in 0..grid_y {
            for xg in 0..grid_x {
                let sy = ((yg * h) / grid_y).min(h.saturating_sub(1));
                let sx = ((xg * w) / grid_x).min(w.saturating_sub(1));
                mask_grid[yg * grid_x + xg] = mask0.data[sy * w + sx];
            }
        }

        // 有效网格（该格未满 + 该格掩码未全遮）
        let features_per_grid = (num_needed / (grid_x * grid_y)).max(1) + 1;
        let mut valid = Vec::new();
        for xg in 0..grid_x {
            for yg in 0..grid_y {
                if (grid_count[yg * grid_x + xg] as usize) < features_per_grid
                    && mask_grid[yg * grid_x + xg] != 255
                {
                    valid.push((xg as i32, yg as i32));
                }
            }
        }

        // 网格提取 + 去畸变/掩码过滤（对照 Grider_GRID）
        let extracted = grider::perform_griding(
            img0,
            mask0,
            &valid,
            num_needed,
            grid_x,
            grid_y,
            self.fast_threshold,
            true,
        );
        for p in extracted {
            let xc = (p.x / min_px as f32) as usize;
            let yc = (p.y / min_px as f32) as usize;
            if xc >= close_w || yc >= close_h {
                continue;
            }
            if grid_close[yc * close_w + xc] > 127 {
                continue;
            }
            if mask_on(mask0, p.x as i32, p.y as i32) {
                continue;
            }
            grid_close[yc * close_w + xc] = 255;
            self.currid += 1;
            pts0.push(p);
            ids0.push(self.currid);
        }
    }

    /// 双目检测（对照 `TrackKLT::perform_detection_stereo`）。
    ///
    /// OpenVINS 的双目版还做左→右 LK 投影校验 + 右图去重；本移植以
    /// 「左右分别单目检测，双目特征共享 id 的对齐逻辑见 `feed_stereo`」
    /// 简化（详情见完成报告偏差项）。
    #[allow(clippy::too_many_arguments)]
    fn perform_detection_stereo(
        &mut self,
        img0pyr: &[GrayImage],
        img1pyr: &[GrayImage],
        mask0: &GrayImage,
        mask1: &GrayImage,
        img0: &GrayImage,
        img1: &GrayImage,
        pts0: &mut Vec<KeyPoint>,
        pts1: &mut Vec<KeyPoint>,
        ids0: &mut Vec<usize>,
        ids1: &mut Vec<usize>,
    ) {
        self.perform_detection_monocular(img0pyr, mask0, img0, pts0, ids0);
        self.perform_detection_monocular(img1pyr, mask1, img1, pts1, ids1);
        // 立体耦合：横向基线（平行前向相机，沿 y 分开）下，同一物理点在左右
        // 图仅水平平移一小段视差。对每个左角点取右侧最近邻（|dx|≤12、|dy|≤3），
        // 命中则赋**同一特征 id** → 该特征同时含左右目测量 → 三角化有立体视差
        //（否则左右独立 id → 特征单目 → 无立体深度 → 低运动时三角化全败）。
        let mut used = vec![false; pts1.len()];
        for i in 0..pts0.len() {
            let (x0, y0) = (pts0[i].x, pts0[i].y);
            let mut best: Option<usize> = None;
            let mut best_d = 169.0f32; // 13²
            for (j, pt) in pts1.iter().enumerate() {
                if used[j] {
                    continue;
                }
                let dx = pt.x - x0;
                let dy = pt.y - y0;
                if dy.abs() <= 3.0 && dx.abs() <= 12.0 {
                    let d = dx * dx + dy * dy;
                    if d < best_d {
                        best_d = d;
                        best = Some(j);
                    }
                }
            }
            if let Some(j) = best {
                ids1[j] = ids0[i];
                used[j] = true;
            }
        }
    }
}
