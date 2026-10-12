//! KLT 视觉前端（对照 `OpenVINS` `ov_core/src/track`：`TrackBase`/`TrackKLT`/`Grider_GRID`）。
//!
//! 本模块实现 OpenVINS 的 KLT 稀疏特征跟踪前端。直方图均衡由本模块实现；
//! FAST、子像素精化、LK 光流与基础矩阵 RANSAC 使用 purecv：
//! - [`histogram`]：直方图均衡（`equalizeHist`）与 CLAHE（`createCLAHE`）；
//! - [`pyramid`]：灰度图 ↔ purecv Matrix 转换（`gray_to_matrix`）；
//! - [`fast`]：FAST-9 角点检测 + 非极大值抑制（`cv::FAST`）；
//! - [`grider`]：网格自适应 FAST 提取 + `cornerSubPix`（`Grider_GRID::perform_griding`）。
//!
//! [`TrackKlt`] 提供完整跟踪器（对照 `TrackKLT.cpp`），消费 `crate::feat` 的
//! `FeatureDatabase`，按时间跟踪（单目）与双目一致性跟踪，并把去畸变归一化
//! 坐标写入特征库。
//!
//! [`TrackerBase`] 不提供 `display_active`/`display_history` GUI 可视化接口。
//!
//! 跟踪器仅输出 FAST/KLT 特征。
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
use std::{
    collections::{HashMap, HashSet},
    sync::Arc,
};

pub mod fast;
pub mod grider;
pub mod histogram;
pub mod pyramid;

/// purecv 金字塔 LK（对照 OpenCV `calcOpticalFlowPyrLK`：内部 f32 金字塔 +
/// Scharr 导数、`OPTFLOW_USE_INITIAL_FLOW` 初值）。
///
/// 21×21 窗口、四层前向金字塔；按 VINS-Fusion `FLOW_BACK` 用两层反向跟踪，
/// 仅接受往返误差 ≤0.5px 的对应。反向初值为原始点，不用预测位移。
/// # Panics
/// 图像必须是完整、同尺寸的灰度缓冲，初值数量必须与输入点一致。
#[must_use]
pub fn lk_optical_flow(
    prev: &GrayImage,
    next: &GrayImage,
    prev_pts: &[nalgebra::Vector2<f32>],
    init_pts: &[nalgebra::Vector2<f32>],
) -> (Vec<nalgebra::Vector2<f32>>, Vec<bool>) {
    let mut buffers = FlowBuffers::default();
    lk_optical_flow_with_buffers(prev, next, prev_pts, init_pts, &mut buffers);
    (buffers.tracked, buffers.accepted)
}

#[derive(Debug, Default)]
struct FlowBuffers {
    original: Vec<purecv::core::types::Point2f>,
    initial: Vec<purecv::core::types::Point2f>,
    tracked: Vec<nalgebra::Vector2<f32>>,
    accepted: Vec<bool>,
}

fn lk_optical_flow_with_buffers<'a>(
    prev: &GrayImage,
    next: &GrayImage,
    prev_pts: &[nalgebra::Vector2<f32>],
    init_pts: &[nalgebra::Vector2<f32>],
    buffers: &'a mut FlowBuffers,
) -> (&'a [nalgebra::Vector2<f32>], &'a [bool]) {
    use purecv::core::types::{Point2f, Size2i, TermCriteria, TermType};
    let prev_mat = pyramid::gray_to_matrix(prev);
    let next_mat = pyramid::gray_to_matrix(next);
    let p0 = &mut buffers.original;
    let p1 = &mut buffers.initial;
    p0.clear();
    p1.clear();
    p0.extend(prev_pts.iter().map(|p| Point2f::new(p.x, p.y)));
    p1.extend(init_pts.iter().map(|p| Point2f::new(p.x, p.y)));
    let (out, status, _err) = purecv::video::calc_optical_flow_pyramid_lk(
        &prev_mat,
        &next_mat,
        p0,
        Some(p1),
        Size2i::new(21, 21),
        3,
        TermCriteria::new(TermType::Both, 30, 0.01),
        purecv::video::OPTFLOW_USE_INITIAL_FLOW,
        1e-4,
    )
    .expect("purecv LK 不应失败（灰度单通道、同尺寸）");
    let (reverse, reverse_status, _) = purecv::video::calc_optical_flow_pyramid_lk(
        &next_mat,
        &prev_mat,
        &out,
        Some(p0),
        Size2i::new(21, 21),
        1,
        TermCriteria::new(TermType::Both, 30, 0.01),
        purecv::video::OPTFLOW_USE_INITIAL_FLOW,
        1e-4,
    )
    .expect("purecv reverse LK uses the same valid image pair");
    buffers.accepted.clear();
    buffers.accepted.extend(
        status
            .iter()
            .zip(&reverse_status)
            .zip(&reverse)
            .zip(p0.iter())
            .map(|(((forward, backward), returned), original)| {
                *forward != 0
                    && *backward != 0
                    && (returned.x - original.x).hypot(returned.y - original.y) <= 0.5
            }),
    );
    buffers.tracked.clear();
    buffers
        .tracked
        .extend(out.into_iter().map(|p| nalgebra::Vector2::new(p.x, p.y)));
    (&buffers.tracked, &buffers.accepted)
}

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
/// 全局特征 id 游标 [`TrackKlt::currid`] 在单线程跟踪器中以 `usize` 表示。
///
/// 本 crate 不提供 GUI 可视化；特征 id 更新见 [`TrackerBase::change_feat_id`]。
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
    img_curr: Arc<GrayImage>,
    /// 上一帧图像（`img_last`）。
    img_last: Arc<GrayImage>,
    /// 上一帧掩码（`img_mask_last`）。
    mask_last: Arc<GrayImage>,
    /// 上一帧成功跟踪点（`pts_last`）。
    pts_last: Vec<KeyPoint>,
    /// 上一帧各点特征 id（`ids_last`）。
    ids_last: Vec<usize>,
    /// 下一帧输出、LK 初值和匹配掩码复用容量。
    pts_next: Vec<KeyPoint>,
    ids_next: Vec<usize>,
    pts_flow: Vec<KeyPoint>,
    matching_mask: Vec<bool>,
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
            img_curr: Arc::new(empty()),
            img_last: Arc::new(empty()),
            mask_last: Arc::new(empty()),
            pts_last: Vec::new(),
            ids_last: Vec::new(),
            pts_next: Vec::new(),
            ids_next: Vec::new(),
            pts_flow: Vec::new(),
            matching_mask: Vec::new(),
        }
    }
}

impl CamState {
    fn prepare(&mut self) {
        self.pts_next.clear();
        self.ids_next.clear();
        self.matching_mask.clear();
    }

    fn advance(&mut self, image: Arc<GrayImage>, mask: Arc<GrayImage>) {
        std::mem::swap(&mut self.pts_last, &mut self.pts_next);
        std::mem::swap(&mut self.ids_last, &mut self.ids_next);
        self.pts_next.clear();
        self.ids_next.clear();
        self.img_last = image;
        self.mask_last = mask;
    }
}

#[derive(Debug)]
struct DetectionBuffers {
    grid_close: Vec<u8>,
    grid_count: Vec<u8>,
    valid: Vec<(i32, i32)>,
    stereo_ids: HashSet<usize>,
    mask: GrayImage,
}

impl Default for DetectionBuffers {
    fn default() -> Self {
        Self {
            grid_close: Vec::new(),
            grid_count: Vec::new(),
            valid: Vec::new(),
            stereo_ids: HashSet::new(),
            mask: GrayImage {
                width: 0,
                height: 0,
                data: Vec::new(),
            },
        }
    }
}

#[derive(Debug, Default)]
struct MatchingBuffers {
    pts0: Vec<nalgebra::Vector2<f32>>,
    pts1: Vec<nalgebra::Vector2<f32>>,
    valid: Vec<usize>,
    normalized0: Vec<purecv::core::types::Point2f>,
    normalized1: Vec<purecv::core::types::Point2f>,
    ransac_mask: Vec<u8>,
    flow: FlowBuffers,
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
    detection: DetectionBuffers,
    matching: MatchingBuffers,
    right_index: HashMap<usize, usize>,
    right_ids: HashSet<usize>,
    stereo_points: Vec<nalgebra::Vector2<f32>>,
    stereo_flow: FlowBuffers,
}

impl TrackKlt {
    /// 同步在线内参标定后的相机快照；已有观测保留采集时的归一化坐标。
    pub fn set_camera_calibration(&mut self, id: usize, camera: crate::cam::SharedCamera) {
        self.base.camera_calib.insert(id, camera);
    }

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
            detection: DetectionBuffers::default(),
            matching: MatchingBuffers::default(),
            right_index: HashMap::new(),
            right_ids: HashSet::new(),
            stereo_points: Vec::new(),
            stereo_flow: FlowBuffers::default(),
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

    /// 直接写入某相机的上一帧观测点与 id（合成算法评测夹具用；正常路径由
    /// [`TrackKlt::feed_new_camera`] 从图像填充）。`pts` 与 `ids` 长度须一致。
    ///
    /// # Panics
    /// `pts` 与 `ids` 长度不一致。
    pub fn set_last_observations(
        &mut self,
        cam_sensor_id: i32,
        pts: Vec<KeyPoint>,
        ids: Vec<usize>,
    ) {
        assert_eq!(pts.len(), ids.len(), "观测点与 id 数量必须一致");
        let state = self.cams.entry(sensor_key(cam_sensor_id)).or_default();
        state.pts_last = pts;
        state.ids_last = ids;
    }

    /// 处理新相机数据（对照 `TrackKLT::feed_new_camera`）。
    ///
    /// 分发逻辑：单图 → `feed_monocular`；双图且 `use_stereo` → `feed_stereo`；
    /// 否则对每张图逐个 `feed_monocular`（C++ 中配双目但 `use_stereo=false` 时并行）。
    #[fastrace::trace]
    pub fn feed_new_camera(&mut self, msg: &CameraData) {
        if !msg.is_valid() || (self.base.use_stereo && msg.images.len() > 2) {
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

    /// 对 `msg` 中所有相机做直方图预处理，将结果写入 `img_curr`
    /// （对照 `TrackKLT.cpp` 第 49-76 行；purecv LK 自建金字塔，此处只做
    /// 直方图）。None 直接共享输入，其余方法按相机用 rayon 并行处理。
    #[fastrace::trace]
    fn preprocess_into_curr(&mut self, msg: &CameraData) {
        use rayon::prelude::*;

        let hist_method = self.base.histogram_method;
        if hist_method == HistogramMethod::None {
            for (&sid, image) in msg.sensor_ids.iter().zip(&msg.images) {
                self.cams.entry(sensor_key(sid)).or_default().img_curr = Arc::clone(image);
            }
            return;
        }

        // 并行：每相机独立做直方图预处理（purecv LK 内部自建 f32 金字塔）
        let results: Vec<_> = msg
            .sensor_ids
            .par_iter()
            .enumerate()
            .map(|(i, &sid)| {
                let key = sensor_key(sid);
                let src = &msg.images[i];
                let processed = match hist_method {
                    HistogramMethod::Histogram => Arc::new(histogram::equalize_hist(src)),
                    HistogramMethod::Clahe => {
                        Arc::new(histogram::clahe(src, &histogram::ClaheParams::default()))
                    }
                    HistogramMethod::None => src.clone(),
                };
                (key, processed)
            })
            .collect();

        // 串行写入 self.cams（避免 &mut self 并发冲突）
        for (key, processed) in results {
            let state = self.cams.entry(key).or_default();
            state.img_curr = processed;
        }
    }

    /// 单目跟踪（对照 `TrackKLT::feed_monocular`）。
    #[fastrace::trace]
    fn feed_monocular(&mut self, msg: &CameraData, msg_id: usize) {
        let key = sensor_key(msg.sensor_ids[msg_id]);
        let mut state = self
            .cams
            .remove(&key)
            .expect("camera state after preprocess");
        state.prepare();
        self.track_monocular(msg, msg_id, &mut state);
        self.cams.insert(key, state);
    }

    fn track_monocular(&mut self, msg: &CameraData, msg_id: usize, state: &mut CamState) {
        let key = sensor_key(msg.sensor_ids[msg_id]);
        let mask = msg.masks[msg_id].clone();
        let timestamp = msg.timestamp;
        let cam_sensor_id = msg.sensor_ids[msg_id];

        let img_curr = Arc::clone(&state.img_curr);
        if state.pts_last.is_empty() {
            self.perform_detection_monocular(
                &mask,
                &img_curr,
                &mut state.pts_last,
                &mut state.ids_last,
                &[],
            );
            state.img_last = img_curr;
            state.mask_last = mask;
            return;
        }

        let (pts_old, ids_old) = (&mut state.pts_last, &mut state.ids_last);
        let (mask_last, img_last) = (&state.mask_last, &state.img_last);
        self.perform_detection_monocular(mask_last, img_last, pts_old, ids_old, &[]);

        // 时间 LK + RANSAC
        state.pts_flow.clone_from(pts_old);
        let pts_new = &mut state.pts_flow;
        let mask_ll = &mut state.matching_mask;
        self.perform_matching(img_last, &img_curr, pts_old, pts_new, key, key, mask_ll);

        // RANSAC 失败 → 复位（同 C++ 的 mask_ll.empty() 分支）
        if mask_ll.is_empty() {
            state.pts_last.clear();
            state.ids_last.clear();
            state.advance(img_curr, mask);
            return;
        }

        let (w, h) = (img_curr.width as i32, img_curr.height as i32);
        let good = &mut state.pts_next;
        let good_ids = &mut state.ids_next;
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
        self.insert_to_db(timestamp, cam_sensor_id, good, good_ids);

        state.advance(img_curr, mask);
    }

    /// 双目跟踪（对照 `TrackKLT::feed_stereo`，具体双目一致性逻辑见注释）。
    // 双目处理流程与 `TrackKLT::feed_stereo` 对应。
    #[allow(clippy::too_many_lines)]
    #[fastrace::trace]
    fn feed_stereo(&mut self, msg: &CameraData, idl: usize, idr: usize) {
        let keyl = sensor_key(msg.sensor_ids[idl]);
        let keyr = sensor_key(msg.sensor_ids[idr]);
        if keyl == keyr {
            return;
        }
        let mut left = self
            .cams
            .remove(&keyl)
            .expect("left camera after preprocess");
        let mut right = self
            .cams
            .remove(&keyr)
            .expect("right camera after preprocess");
        left.prepare();
        right.prepare();
        self.track_stereo(msg, idl, idr, &mut left, &mut right);
        self.cams.insert(keyl, left);
        self.cams.insert(keyr, right);
    }

    #[allow(clippy::too_many_lines)]
    fn track_stereo(
        &mut self,
        msg: &CameraData,
        idl: usize,
        idr: usize,
        sl: &mut CamState,
        sr: &mut CamState,
    ) {
        let keyl = sensor_key(msg.sensor_ids[idl]);
        let keyr = sensor_key(msg.sensor_ids[idr]);
        let mask_l = msg.masks[idl].clone();
        let mask_r = msg.masks[idr].clone();
        let timestamp = msg.timestamp;
        let sid_l = msg.sensor_ids[idl];
        let sid_r = msg.sensor_ids[idr];

        let (img_l, img_r) = (Arc::clone(&sl.img_curr), Arc::clone(&sr.img_curr));
        let (img_l_last, img_r_last) = (Arc::clone(&sl.img_last), Arc::clone(&sr.img_last));
        if sl.pts_last.is_empty() && sr.pts_last.is_empty() {
            self.perform_detection_stereo(
                &mask_l,
                &mask_r,
                &img_l,
                &img_r,
                &mut sl.pts_last,
                &mut sr.pts_last,
                &mut sl.ids_last,
                &mut sr.ids_last,
            );
            sl.img_last = img_l;
            sl.mask_last = mask_l;
            sr.img_last = img_r;
            sr.mask_last = mask_r;
            return;
        }

        let (pts_l, pts_r, ids_l, ids_r) = (
            &mut sl.pts_last,
            &mut sr.pts_last,
            &mut sl.ids_last,
            &mut sr.ids_last,
        );
        let (mask_l_last, mask_r_last) = (&sl.mask_last, &sr.mask_last);
        self.perform_detection_stereo(
            mask_l_last,
            mask_r_last,
            // 新点在上一帧检测，再由时间 LK 跟踪到当前帧。
            &img_l_last,
            &img_r_last,
            pts_l,
            pts_r,
            ids_l,
            ids_r,
        );

        // 左右时间 LK + RANSAC
        sl.pts_flow.clone_from(pts_l);
        sr.pts_flow.clone_from(pts_r);
        let (pts_l_new, pts_r_new) = (&mut sl.pts_flow, &mut sr.pts_flow);
        // 运动外推初值：LK 初值 = 上一帧位置 + 最近两帧位移（按特征 id 查
        // 库），减小大位移（10Hz 下近地面特征 ~31px/帧）下 LK 收敛滞后。
        for (i, id) in ids_l.iter().enumerate() {
            if let Some([prev, last]) = self.base.database.latest_uv_pair(*id, keyl) {
                pts_l_new[i].x += last.x - prev.x;
                pts_l_new[i].y += last.y - prev.y;
            }
        }
        for (i, id) in ids_r.iter().enumerate() {
            if let Some([prev, last]) = self.base.database.latest_uv_pair(*id, keyr) {
                pts_r_new[i].x += last.x - prev.x;
                pts_r_new[i].y += last.y - prev.y;
            }
        }
        let (mask_ll, mask_rr) = (&mut sl.matching_mask, &mut sr.matching_mask);
        self.perform_matching(&img_l_last, &img_l, pts_l, pts_l_new, keyl, keyl, mask_ll);
        self.perform_matching(&img_r_last, &img_r, pts_r, pts_r_new, keyr, keyr, mask_rr);

        if mask_ll.is_empty() && mask_rr.is_empty() {
            sl.pts_last.clear();
            sl.ids_last.clear();
            sl.img_last = img_l;
            sl.mask_last = mask_l;
            sr.pts_last.clear();
            sr.ids_last.clear();
            sr.img_last = img_r;
            sr.mask_last = mask_r;
            return;
        }

        let (w_l, h_l) = (img_l.width as i32, img_l.height as i32);
        let (w_r, h_r) = (img_r.width as i32, img_r.height as i32);
        let (good_l, good_r) = (&mut sl.pts_next, &mut sr.pts_next);
        let (good_ids_l, good_ids_r) = (&mut sl.ids_next, &mut sr.ids_next);

        // LEFT loop（对照 TrackKLT.cpp 第 306-338 行）
        // 右目 id → 位置预建表：O(1) 替代 O(n) 线性扫描（同 RIGHT loop 的 HashSet
        // 做法；`or_insert` 保留首个下标，与线性扫描找到首个即停同语义）。
        let r_index = &mut self.right_index;
        r_index.clear();
        for (n, &idr) in ids_r.iter().enumerate() {
            r_index.entry(idr).or_insert(n);
        }
        for i in 0..pts_l_new.len() {
            let p = pts_l_new[i];
            if p.x < 0.0 || p.y < 0.0 || (p.x as i32) > w_l || (p.y as i32) > h_l {
                continue;
            }
            let (found_r, idx_r) = match r_index.get(&ids_l[i]) {
                Some(&n) => (true, n),
                None => (false, 0),
            };
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
        // 用 HashSet 加速重复 id 查找（O(1) 替代 O(n) 线性扫描）
        let good_ids_r_set = &mut self.right_ids;
        good_ids_r_set.clear();
        good_ids_r_set.extend(good_ids_r.iter().copied());
        for i in 0..pts_r_new.len() {
            let p = pts_r_new[i];
            if p.x < 0.0 || p.y < 0.0 || (p.x as i32) >= w_r || (p.y as i32) >= h_r {
                continue;
            }
            if mask_rr[i] && !good_ids_r_set.contains(&ids_r[i]) {
                good_r.push(p);
                good_ids_r.push(ids_r[i]);
            }
        }

        // 入库
        self.insert_to_db(timestamp, sid_l, good_l, good_ids_l);
        self.insert_to_db(timestamp, sid_r, good_r, good_ids_r);
        sl.advance(img_l, mask_l);
        sr.advance(img_r, mask_r);
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
        img0: &GrayImage,
        img1: &GrayImage,
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
        let workspace = &mut self.matching;
        let pts0 = &mut workspace.pts0;
        let pts1 = &mut workspace.pts1;
        pts0.clear();
        pts1.clear();
        pts0.extend(kpts0.iter().map(|k| nalgebra::Vector2::new(k.x, k.y)));
        pts1.extend(kpts1.iter().map(|k| nalgebra::Vector2::new(k.x, k.y)));

        // 点数不足做 RANSAC → 全部失败（同 C++ 第 846-852 行）
        if pts0.len() < 10 {
            mask_out.extend(std::iter::repeat_n(false, pts0.len()));
            return;
        }

        // LK（OPTFLOW_USE_INITIAL_FLOW → use_initial_flow=true；purecv 金字塔 LK）
        let (out, status) =
            lk_optical_flow_with_buffers(img0, img1, pts0, pts1, &mut workspace.flow);
        let valid = &mut workspace.valid;
        valid.clear();
        valid.extend(
            status
                .iter()
                .enumerate()
                .filter_map(|(i, &ok)| ok.then_some(i)),
        );
        if valid.len() < 10 {
            mask_out.resize(pts0.len(), false);
            return;
        }

        // 去畸变归一化（RANSAC 需要在规范坐标上进行，同 C++ 第 860-866 行）
        let cam0 = self.base.camera_calib.get(&cam_key0).cloned();
        let cam1 = self.base.camera_calib.get(&cam_key1).cloned();
        let (Some(cam0), Some(cam1)) = (cam0, cam1) else {
            return;
        };
        let pts0_p = &mut workspace.normalized0;
        let pts1_p = &mut workspace.normalized1;
        pts0_p.clear();
        pts1_p.clear();
        pts0_p.extend(valid.iter().map(|&i| {
            let p = cam0.undistort_d(pts0[i].cast());
            purecv::core::types::Point2f::new(p.x as f32, p.y as f32)
        }));
        pts1_p.extend(valid.iter().map(|&i| {
            let p = cam1.undistort_d(out[i].cast());
            purecv::core::types::Point2f::new(p.x as f32, p.y as f32)
        }));

        // max_focallength = 两相机焦距最大值（同 C++ 第 870-872 行）
        let max_focal = {
            let k0 = cam0.camera_matrix();
            let k1 = cam1.camera_matrix();
            k0[(0, 0)].max(k0[(1, 1)]).max(k1[(0, 0)]).max(k1[(1, 1)])
        };
        let threshold = 2.0 / max_focal;

        // RANSAC（同 C++ 第 873 行；purecv `findFundamentalMat` = OpenCV 语义，
        // 替代手搓 ransac——退化时（内点 <8）返回 Err，等同 OpenCV 空 mask 全拒）
        let mask_rsc = &mut workspace.ransac_mask;
        mask_rsc.clear();
        let f_res = purecv::calib3d::find_fundamental_mat(
            pts0_p,
            pts1_p,
            purecv::calib3d::FundamentalMatMethod::FM_RANSAC,
            threshold,
            0.999,
            1000,
            Some(mask_rsc),
        );
        if f_res.is_err() {
            mask_rsc.clear();
        }

        // RANSAC 只使用双向 LK 有效的对应，掩码再还原到原始点索引。
        mask_out.resize(status.len(), false);
        mask_out.fill(false);
        for (&i, &accepted) in valid.iter().zip(mask_rsc.iter()) {
            mask_out[i] = accepted != 0;
        }

        // 回写位置（pts0 未变，pts1 取 LK 结果；同 C++ 第 882-885 行）
        for (i, k) in kpts1.iter_mut().enumerate() {
            k.x = out[i].x;
            k.y = out[i].y;
        }
    }

    /// 单目检测，返回剔除旧点后的新增点起始下标（对照 `TrackKlt::perform_detection_monocular`）。
    // 单目检测流程与 `TrackKLT::perform_detection_monocular` 对应。
    #[allow(clippy::too_many_lines)]
    #[fastrace::trace]
    fn perform_detection_monocular(
        &mut self,
        mask0: &GrayImage,
        img0: &GrayImage,
        pts0: &mut Vec<KeyPoint>,
        ids0: &mut Vec<usize>,
        stereo_ids: &[usize],
    ) -> usize {
        let workspace = &mut self.detection;
        workspace.stereo_ids.clear();
        workspace.stereo_ids.extend(stereo_ids.iter().copied());
        let stereo_ids = &workspace.stereo_ids;
        let min_px = self.min_px_dist.max(1) as usize;
        let (w, h) = (img0.width, img0.height);
        // 距离占用网格（每 cell = min_px² 像素）
        let close_w = w.div_ceil(min_px);
        let close_h = h.div_ceil(min_px);
        let grid_close = &mut workspace.grid_close;
        grid_close.resize(close_w * close_h, 0);
        grid_close.fill(0);
        let grid_x = self.grid_x.max(1);
        let grid_y = self.grid_y.max(1);
        let size_x = w as f32 / grid_x as f32;
        let size_y = h as f32 / grid_y as f32;
        let grid_count = &mut workspace.grid_count;
        grid_count.resize(grid_x * grid_y, 0);
        grid_count.fill(0);
        let edge = 10i32;
        let (w_i, h_i) = (w as i32, h as i32);

        // 遍历既有特征：剔除越界/掩码/过近，并用之占用 close-grid 与网格计数
        let mut kept = 0;
        for i in 0..pts0.len() {
            let p = pts0[i];
            let fid = ids0[i];
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
            // 对照 OpenVINS：右目已配对的点必须保留，网格去重只剔除单目点。
            if grid_close[yc * close_w + xc] > 127 && !stereo_ids.contains(&fid) {
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
            pts0[kept] = p;
            ids0[kept] = fid;
            kept += 1;
        }
        pts0.truncate(kept);
        ids0.truncate(kept);

        let new_start = pts0.len();

        // 需要的补点数
        let min_feat_percent = 0.50;
        let num_needed = self.num_features.saturating_sub(pts0.len());
        let min_threshold =
            std::cmp::min(20, (min_feat_percent * self.num_features as f64) as usize);
        if num_needed < min_threshold {
            return new_start;
        }

        // 有效网格（该格未满 + 该格掩码未全遮）
        let features_per_grid = self.num_features / (grid_x * grid_y) + 1;
        let required_per_grid = ((min_feat_percent * features_per_grid as f64) as usize).max(1);
        let valid = &mut workspace.valid;
        valid.clear();
        for xg in 0..grid_x {
            for yg in 0..grid_y {
                if (grid_count[yg * grid_x + xg] as usize) < required_per_grid
                    && mask0.data
                        [((yg * h) / grid_y).min(h - 1) * w + ((xg * w) / grid_x).min(w - 1)]
                        != 255
                {
                    valid.push((xg as i32, yg as i32));
                }
            }
        }

        // 更新掩码覆盖已有特征周围的 `2 * min_px_dist` 区域，维持特征间距。
        let (w_i, h_i) = (w as i32, h as i32);
        let r = min_px as i32;
        let mut mask_updated = mask0;
        let mut copied = false;
        for p in pts0.iter() {
            let (x, y) = (p.x as i32, p.y as i32);
            if x - r >= 0 && x + r < w_i && y - r >= 0 && y + r < h_i {
                if !copied {
                    workspace.mask.width = w;
                    workspace.mask.height = h;
                    workspace.mask.data.clone_from(&mask0.data);
                    copied = true;
                }
                for y0 in (y - r)..=(y + r) {
                    let row = y0 as usize * w;
                    workspace.mask.data[row + (x - r) as usize..=row + (x + r) as usize].fill(255);
                }
            }
        }
        if copied {
            mask_updated = &workspace.mask;
        }

        // 网格提取 + 去畸变/掩码过滤（对照 Grider_GRID）
        let extracted = grider::perform_griding(
            img0,
            mask_updated,
            valid,
            self.num_features,
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
        new_start
    }

    /// 双目检测（对照 `TrackKLT::perform_detection_stereo`）。
    ///
    /// 新增左目点通过左→右 LK 建立同一 feature id 的双目观测。
    /// 右目补点只对单目点执行邻近去重，必须保留已有双目对应。
    #[allow(clippy::too_many_arguments)]
    #[fastrace::trace]
    fn perform_detection_stereo(
        &mut self,
        mask0: &GrayImage,
        mask1: &GrayImage,
        img0: &GrayImage,
        img1: &GrayImage,
        pts0: &mut Vec<KeyPoint>,
        pts1: &mut Vec<KeyPoint>,
        ids0: &mut Vec<usize>,
        ids1: &mut Vec<usize>,
    ) {
        // 1. 左目单目检测（过滤旧的 + 增补新左特征）→ pts0/ids0
        let new_start = self.perform_detection_monocular(mask0, img0, pts0, ids0, &[]);

        // 2. 新增左特征左→右 LK 匹配（对照 OpenVINS：kpts1_new=kpts0_new →
        //    calcOpticalFlowPyrLK(left→right)，左右目同 id 双测量）
        let new_len = pts0.len();
        if new_len > new_start {
            let left = &mut self.stereo_points;
            left.clear();
            left.extend(
                pts0[new_start..]
                    .iter()
                    .map(|k| nalgebra::Vector2::new(k.x, k.y)),
            );
            let (right_out, status) =
                lk_optical_flow_with_buffers(img0, img1, left, left, &mut self.stereo_flow);
            let (w, h) = (img1.width as f32, img1.height as f32);
            for i in new_start..new_len {
                let si = i - new_start;
                let p = right_out[si];
                if status[si] && p.x >= 0.0 && p.y >= 0.0 && p.x < w && p.y < h {
                    // 双目耦合：右目点 + 同一 id → 特征同时有左右目测量
                    pts1.push(KeyPoint {
                        x: p.x,
                        y: p.y,
                        response: pts0[i].response,
                    });
                    ids1.push(ids0[i]);
                }
            }
        }

        // 3. 右目单目增补（close-grid 自动避开已耦合的右目点）
        self.perform_detection_monocular(mask1, img1, pts1, ids1, ids0);
    }
}

#[cfg(test)]
mod lk_bias_tests {
    use super::*;
    use nalgebra::Vector2;

    /// 非周期棋盘（伪随机格子宽 4-12px，有大量角点且无周期混淆）
    fn checkerboard(w: usize, h: usize, _cell: usize) -> GrayImage {
        // xorshift 伪随机（确定性，不依赖 rand）
        let mut seed = 0x1234_5678u32;
        let mut next = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed
        };
        // 水平/垂直分界线位置（伪随机非周期格子）
        let mut xs = vec![0usize];
        let mut x = 0usize;
        while x < w {
            x += 4 + (next() % 9) as usize;
            xs.push(x.min(w));
        }
        let mut ys = vec![0usize];
        let mut y = 0usize;
        while y < h {
            y += 4 + (next() % 9) as usize;
            ys.push(y.min(h));
        }
        let mut data = vec![0u8; w * h];
        for yy in 0..ys.len() - 1 {
            for xx in 0..xs.len() - 1 {
                let v = if (xx + yy) % 2 == 0 { 30u8 } else { 220u8 };
                for py in ys[yy]..ys[yy + 1] {
                    for px in xs[xx]..xs[xx + 1] {
                        data[py * w + px] = v;
                    }
                }
            }
        }
        // 3×3 均值模糊（抗锯齿，避免硬边阶跃的 LK 亚像素局限）
        let mut blurred = vec![0u8; w * h];
        for y in 0..h {
            for x in 0..w {
                let mut s = 0u32;
                let mut n = 0u32;
                for dy in -1i32..=1 {
                    for dx in -1i32..=1 {
                        let (sx, sy) = (x as i32 + dx, y as i32 + dy);
                        if sx >= 0 && sx < w as i32 && sy >= 0 && sy < h as i32 {
                            s += u32::from(data[sy as usize * w + sx as usize]);
                            n += 1;
                        }
                    }
                }
                blurred[y * w + x] = (s / n) as u8;
            }
        }
        GrayImage {
            width: w,
            height: h,
            data: blurred,
        }
    }

    /// 整数平移后的图像（平移后空白区填噪声，避免边界影响）
    fn shift(img: &GrayImage, dx: i32, dy: i32) -> GrayImage {
        let (w, h) = (img.width, img.height);
        let mut data = vec![0u8; w * h];
        for y in 0..h as i32 {
            for x in 0..w as i32 {
                let (sx, sy) = (x - dx, y - dy);
                if sx >= 0 && sx < w as i32 && sy >= 0 && sy < h as i32 {
                    data[(y as usize) * w + x as usize] = img.data[(sy as usize) * w + sx as usize];
                }
            }
        }
        GrayImage {
            width: w,
            height: h,
            data,
        }
    }

    /// 已知整数平移：同时约束保留点数和平均位移，拒绝全丢点的空通过。
    #[test]
    fn lk_displacement_bias() {
        let img0 = checkerboard(320, 240, 8);
        // 位移 ≤15px，避免随机格纹理的周期混淆。
        for (dx, dy) in [
            (0i32, 0i32),
            (10, 0),
            (0, 10),
            (0, -10),
            (10, 10),
            (-10, 5),
            (0, 12),
        ] {
            let img1 = shift(&img0, dx, dy);
            // 采样点：避开边界 40px
            let mut pts0: Vec<Vector2<f32>> = Vec::new();
            for y in (40..200).step_by(16) {
                for x in (40..280).step_by(16) {
                    pts0.push(Vector2::new(x as f32, y as f32));
                }
            }
            let pts1 = pts0.clone();
            let (out, status) = lk_optical_flow(&img0, &img1, &pts0, &pts1);
            let mut errs = Vec::new();
            let mut ok = 0usize;
            for (i, (p, s)) in out.iter().zip(status.iter()).enumerate() {
                if !s {
                    continue;
                }
                ok += 1;
                errs.push((p.x - pts0[i].x - dx as f32, p.y - pts0[i].y - dy as f32));
            }
            let n = errs.len() as f32;
            let mx = errs.iter().map(|e| e.0).sum::<f32>() / n;
            let my = errs.iter().map(|e| e.1).sum::<f32>() / n;
            assert!(ok >= pts0.len() * 9 / 10);
            assert!(
                mx.abs() < 0.25 && my.abs() < 0.25,
                "LK 位移系统性偏置 ({mx:.3},{my:.3})px @ shift=({dx},{dy})"
            );
        }
    }

    #[test]
    fn empty_tile_detection_budget_is_independent_of_other_tiles_occupancy() {
        let image = checkerboard(320, 240, 8);
        let mask = GrayImage {
            width: 320,
            height: 240,
            data: vec![0; 320 * 240],
        };
        let detect = |mut points: Vec<KeyPoint>| {
            let mut tracker = TrackKlt::new(
                HashMap::new(),
                200,
                0,
                false,
                HistogramMethod::None,
                20,
                4,
                4,
                10,
            );
            let mut ids = (1..=points.len()).collect();
            tracker.currid = points.len();
            tracker.perform_detection_monocular(&mask, &image, &mut points, &mut ids, &[]);
            points
                .into_iter()
                .filter(|p| p.x < 80. && p.y < 60.)
                .collect::<Vec<_>>()
        };
        let empty = detect(Vec::new());
        let occupied = detect(
            (0..100)
                .map(|i| KeyPoint::new(100. + (i % 10) as f32 * 20., 20. + (i / 10) as f32 * 20.))
                .collect(),
        );
        assert_ne!(empty, [] as [KeyPoint; 0]);
        assert_eq!(empty, occupied);
    }

    #[test]
    fn detection_workspace_resets_mask_and_grids_across_frames_and_sizes() {
        let make_tracker = || {
            TrackKlt::new(
                HashMap::new(),
                80,
                0,
                false,
                HistogramMethod::None,
                20,
                4,
                4,
                10,
            )
        };
        let mut tracker = make_tracker();
        for (w, h) in [(320, 240), (320, 240), (160, 120), (320, 240)] {
            let image = checkerboard(w, h, 8);
            let mask = GrayImage {
                width: w,
                height: h,
                data: vec![0; w * h],
            };
            let mut points = vec![KeyPoint::new(50., 50.)];
            let mut ids = vec![1];
            tracker.currid = 1;
            tracker.perform_detection_monocular(&mask, &image, &mut points, &mut ids, &[]);
            assert_eq!(tracker.detection.mask.data.len(), w * h);
            let grid_ptr = tracker.detection.grid_close.as_ptr();
            let mask_ptr = tracker.detection.mask.data.as_ptr();
            let mut fresh = make_tracker();
            fresh.currid = tracker.currid;
            let mut expected = Vec::new();
            let mut expected_ids = Vec::new();
            fresh.perform_detection_monocular(&mask, &image, &mut expected, &mut expected_ids, &[]);
            points.clear();
            ids.clear();
            tracker.perform_detection_monocular(&mask, &image, &mut points, &mut ids, &[]);
            assert_eq!(points, expected);
            assert_eq!(ids, expected_ids);
            assert_eq!(grid_ptr, tracker.detection.grid_close.as_ptr());
            assert_eq!(mask_ptr, tracker.detection.mask.data.as_ptr());
        }
    }

    #[test]
    fn stereo_detection_keeps_paired_right_points_in_the_same_grid_cell() {
        let image = GrayImage {
            width: 100,
            height: 100,
            data: vec![0; 10_000],
        };
        let mut tracker = TrackKlt::new(
            HashMap::new(),
            2,
            0,
            true,
            HistogramMethod::None,
            20,
            5,
            5,
            10,
        );
        let mut left = vec![KeyPoint::new(50., 50.), KeyPoint::new(65., 50.)];
        let mut right = vec![KeyPoint::new(50., 50.), KeyPoint::new(55., 50.)];
        let mut ids_left = vec![1, 2];
        let mut ids_right = ids_left.clone();
        tracker.perform_detection_stereo(
            &image,
            &image,
            &image,
            &image,
            &mut left,
            &mut right,
            &mut ids_left,
            &mut ids_right,
        );
        assert_eq!(ids_left, vec![1, 2]);
        assert_eq!(ids_right, ids_left);
    }

    #[test]
    fn stereo_replenishment_matches_new_points_after_pruning_old_tracks() {
        let image = checkerboard(320, 240, 8);
        let mask = GrayImage {
            width: 320,
            height: 240,
            data: vec![0; 320 * 240],
        };
        let mut tracker = TrackKlt::new(
            HashMap::new(),
            80,
            0,
            true,
            HistogramMethod::None,
            20,
            5,
            5,
            10,
        );
        tracker.currid = 80;
        let mut left = vec![KeyPoint::new(-1., -1.); 80];
        let mut ids_left: Vec<_> = (1..=80).collect();
        let mut right = Vec::new();
        let mut ids_right = Vec::new();
        tracker.perform_detection_stereo(
            &mask,
            &mask,
            &image,
            &image,
            &mut left,
            &mut right,
            &mut ids_left,
            &mut ids_right,
        );
        assert_ne!(left, [] as [KeyPoint; 0]);
        let mut reference = TrackKlt::new(
            HashMap::new(),
            80,
            0,
            true,
            HistogramMethod::None,
            20,
            5,
            5,
            10,
        );
        reference.currid = 80;
        let (mut ref_left, mut ref_right, mut ref_ids_left, mut ref_ids_right) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        reference.perform_detection_stereo(
            &mask,
            &mask,
            &image,
            &image,
            &mut ref_left,
            &mut ref_right,
            &mut ref_ids_left,
            &mut ref_ids_right,
        );
        assert!(ref_ids_left.iter().any(|id| ref_ids_right.contains(id)));
        assert_eq!(
            (&left, &right, &ids_left, &ids_right),
            (&ref_left, &ref_right, &ref_ids_left, &ref_ids_right)
        );
        for (p, id) in left.iter().zip(&ids_left) {
            assert!(*id > 80);
            if let Some(i) = ids_right.iter().position(|r| r == id) {
                assert!((p.x - right[i].x).hypot(p.y - right[i].y) < 0.01);
            }
        }
    }

    #[test]
    fn bidirectional_lk_rejects_occluded_points_and_keeps_visible_motion() {
        let first = checkerboard(320, 240, 8);
        let mut second = shift(&first, 4, 3);
        for y in 70..170 {
            second.data[y * 320 + 70..y * 320 + 170].fill(0);
        }
        let original = vec![Vector2::new(110., 110.), Vector2::new(240., 110.)];
        let guess: Vec<_> = original.iter().map(|p| p + Vector2::new(4., 3.)).collect();
        let (tracked, accepted) = lk_optical_flow(&first, &second, &original, &guess);
        assert!(
            !accepted[0],
            "occluded texture must not enter the estimator"
        );
        assert!(accepted[1]);
        assert!((tracked[1] - guess[1]).norm() < 0.05);
    }
}

#[cfg(test)]
mod shared_frame_tests {
    use super::*;

    fn frame(stereo: bool, value: u8) -> CameraData {
        let image = Arc::new(GrayImage {
            width: 64,
            height: 64,
            data: vec![value; 64 * 64],
        });
        let mask = Arc::new(GrayImage {
            width: 64,
            height: 64,
            data: vec![0; 64 * 64],
        });
        let n = if stereo { 2 } else { 1 };
        CameraData {
            timestamp: f64::from(value),
            sensor_ids: (0..n).collect(),
            images: vec![image; n as usize],
            masks: vec![mask; n as usize],
        }
    }

    #[test]
    fn shared_frames_advance_on_first_empty_and_rejected_tracks() {
        for stereo in [false, true] {
            let mut tracker = TrackKlt::new(
                HashMap::new(),
                2,
                0,
                stereo,
                HistogramMethod::None,
                20,
                2,
                2,
                5,
            );
            let first = frame(stereo, 0);
            tracker.feed_new_camera(&first);
            for (index, _) in first.sensor_ids.iter().enumerate() {
                let state = &tracker.cams[&index];
                assert!(Arc::ptr_eq(&state.img_curr, &first.images[index]));
                assert!(Arc::ptr_eq(&state.img_last, &first.images[index]));
                assert!(Arc::ptr_eq(&state.mask_last, &first.masks[index]));
            }
            // 越界点在补点时被移除，匹配的空结果必须推进图像和掩码。
            for id in &first.sensor_ids {
                tracker.set_last_observations(*id, vec![KeyPoint::new(-1.0, -1.0)], vec![1]);
            }
            let second = frame(stereo, 1);
            tracker.feed_new_camera(&second);
            assert!(
                tracker
                    .cams
                    .values()
                    .all(|state| state.pts_last.is_empty() && state.ids_last.is_empty())
            );
            assert!(Arc::ptr_eq(&tracker.cams[&0].img_last, &second.images[0]));
            // 单侧有效点不足 RANSAC 的十点门槛；非空拒绝结果走正常提交路径。
            tracker.set_last_observations(0, vec![KeyPoint::new(32.0, 32.0)], vec![2]);
            let third = frame(stereo, 2);
            tracker.feed_new_camera(&third);
            for (index, _) in third.sensor_ids.iter().enumerate() {
                let state = &tracker.cams[&index];
                assert!(state.pts_last.is_empty() && state.ids_last.is_empty());
                assert!(Arc::ptr_eq(&state.img_last, &third.images[index]));
                assert!(Arc::ptr_eq(&state.mask_last, &third.masks[index]));
            }
            assert!(first.images[0].data.iter().all(|pixel| *pixel == 0));
            assert!(first.masks[0].data.iter().all(|pixel| *pixel == 0));
        }
    }

    #[test]
    fn invalid_input_leaves_all_camera_state_untouched() {
        let mut tracker = TrackKlt::new(
            HashMap::new(),
            2,
            0,
            true,
            HistogramMethod::None,
            20,
            2,
            2,
            5,
        );
        let valid = frame(true, 0);
        tracker.feed_new_camera(&valid);
        tracker.set_last_observations(0, vec![KeyPoint::new(32.0, 32.0)], vec![7]);
        for case in 0..6 {
            let mut invalid = frame(true, 1);
            match case {
                0 => invalid.masks.clear(),
                1 => Arc::make_mut(&mut invalid.masks[1]).data.clear(),
                2 => Arc::make_mut(&mut invalid.masks[1]).width = 32,
                3 => {
                    Arc::make_mut(&mut invalid.images[1]).data.pop();
                }
                4 => invalid.sensor_ids[1] = invalid.sensor_ids[0],
                _ => Arc::make_mut(&mut invalid.images[1]).width = usize::MAX,
            }
            assert!(!invalid.is_valid());
            tracker.feed_new_camera(&invalid);
            assert_eq!(tracker.last_ids(0), &[7]);
            for index in 0..2 {
                assert!(Arc::ptr_eq(
                    &tracker.cams[&index].img_curr,
                    &valid.images[index]
                ));
                assert!(Arc::ptr_eq(
                    &tracker.cams[&index].img_last,
                    &valid.images[index]
                ));
                assert!(Arc::ptr_eq(
                    &tracker.cams[&index].mask_last,
                    &valid.masks[index]
                ));
            }
        }
    }
}
