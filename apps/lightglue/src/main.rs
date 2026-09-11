//! `lightglue` 进程：视觉全局定位（`ort` ONNX Runtime 推理）。
//!
//! 订阅当前帧特征（`Firefly/Features`）+ 矫正后里程计（先验，
//! `Firefly/CorrectedOdometry`，无则回退 `Firefly/Odometry`）→ 库图空间短名单 →
//! `models/lightglue-aliked-k512.onnx` 图-图匹配 → `PnP` 解全局位姿 →
//! 经 `Firefly/PoseObservation` 发布视觉观测（`gicp` 内同一 `FusionFilter` 融合）。
//! 对照 VINS-Fusion `loop_fusion` 的检环 + 位姿边（`findConnection` 用 VIO
//! 位姿作初值、`pose_graph` 以位姿边做联合优化），只是描述子换成 ALIKED，
//! 且观测与 GICP 共用误差态 EKF 而非位姿图。
//!
//! 运行：`cargo run -p lightglue -- --map <map.ffvmap> [-- --model ...]`。

use firefly_localization::convert::odom_to_matrix;
use firefly_pubsub::event::TopicListener;
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::odom::{CORRECTED_ODOM_TOPIC, ODOM_TOPIC};
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::{CorrectedOdomSubscriber, Subscriber};
use firefly_pubsub::vision::{
    DESC_DIM, FEATURE_TOPIC, FeatureMessage, MAX_FEATURES as NUM_POINTS, OBS_SOURCE_VISUAL,
    POSE_OBS_TOPIC, PoseObservation,
};
use firefly_vision_map::VisionMap;
use firefly_vision_match::calibration::{MUJOCO_FOCAL, body_pose_to_cam, cam_pose_to_body};
use firefly_vision_match::{CameraIntrinsics, pose_covariance, solve_visual_pose};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetAttachmentId;
use nalgebra::{Rotation3, UnitQuaternion};
use ort::session::Session;
use ort::value::Tensor;

/// 缺省权重路径（相对运行目录，通常为仓库根）。
const DEFAULT_MODEL: &str = "models/lightglue-aliked-k512.onnx";
/// 相机分辨率（`image_size` 输入，`[W, H]` int64）。
const WIDTH: i64 = 320;
const HEIGHT: i64 = 240;
/// 库图查询半径（米，VIO 先验周围）。
const QUERY_RADIUS: f64 = 3.0;
/// 单次查询候选帧数：拼多帧对应破平面退化（单帧共面时 `PnP` 有翻转二义性）。
const QUERY_TOPK: usize = 3;
/// 匹配分数阈值（图内建 0.1 过滤，此处再收紧）。
const SCORE_THRESHOLD: f32 = 0.2;
/// odom 先验新鲜度（秒），超时跳过本次查询。
const ODOM_FRESH_TIMEOUT: f64 = 1.0;

/// 解析 `--model/--map`（缺省见 [`DEFAULT_MODEL`]，地图必填）。
fn parse_args() -> Result<(String, String), String> {
    let mut it = std::env::args().skip(1);
    let mut model = DEFAULT_MODEL.to_owned();
    let mut map: Option<String> = None;
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--model" => {
                model = it
                    .next()
                    .ok_or_else(|| "missing --model value".to_owned())?;
            }
            "--map" => {
                map = Some(it.next().ok_or_else(|| "missing --map value".to_owned())?);
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    let map = map.ok_or_else(|| "missing --map <map.ffvmap>".to_owned())?;
    Ok((model, map))
}

/// 加载 ONNX 会话（文件缺失即报错，不静默；同 aliked 锁 1 intra-op 线程，
/// 防 ORT 抢占 vio/sim 的 CPU 预算）。
fn load_session(model: &str) -> Result<Session, Box<dyn std::error::Error>> {
    if !std::path::Path::new(model).is_file() {
        return Err(format!("权重缺失：{model}（见 models/，离线导出，不进 git）").into());
    }
    let session = Session::builder()?
        .with_intra_threads(1)?
        .commit_from_file(model)?;
    log::info!("lightglue 会话就绪：{model}");
    Ok(session)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    let (model, map_path) = parse_args().map_err(|e| {
        eprintln!(
            "{e}\n用法：lightglue --map <map.ffvmap> [--model models/lightglue-aliked-k512.onnx]"
        );
        std::process::exit(2);
    })?;
    let map = firefly_vision_map::load_map(std::path::Path::new(&map_path)).map_err(|e| {
        eprintln!("视觉库图加载失败: {e}");
        std::process::exit(2);
    })?;
    log::info!(
        "视觉库图就绪（{} 帧，{} 点）",
        map.frames.len(),
        map.num_points()
    );
    let mut session = load_session(&model)?;
    smoke_inference(&mut session)?;
    run_loop(&mut session, &map)?;
    Ok(())
}

/// 冒烟推理：零特征自匹配，校验输出形状（`[1,512]` 静态）。
fn smoke_inference(session: &mut Session) -> Result<(), Box<dyn std::error::Error>> {
    let zeros = |n: usize| {
        Tensor::from_array((
            [1usize, NUM_POINTS, n],
            vec![0f32; NUM_POINTS * n].into_boxed_slice(),
        ))
    };
    let size = Tensor::from_array(([1usize, 2], vec![WIDTH, HEIGHT].into_boxed_slice()))?;
    let outputs = session.run(ort::inputs![
        "k0" => zeros(2)?,
        "d0" => zeros(128)?,
        "k1" => zeros(2)?,
        "d1" => zeros(128)?,
        "s0" => size,
        "s1" => Tensor::from_array(([1usize, 2], vec![WIDTH, HEIGHT].into_boxed_slice()))?,
    ])?;
    for name in ["matches0", "matches1"] {
        let (shape, _) = outputs[name].try_extract_tensor::<i64>()?;
        assert_eq!(&shape[..], &[1, NUM_POINTS as i64], "{name} 形状");
    }
    for name in ["scores0", "scores1"] {
        let (shape, _) = outputs[name].try_extract_tensor::<f32>()?;
        assert_eq!(&shape[..], &[1, NUM_POINTS as i64], "{name} 形状");
    }
    log::info!("lightglue 冒烟推理通过（输出 [1,{NUM_POINTS}] 静态）");
    Ok(())
}

/// 主循环：特征事件唤醒 → 取最新特征 + 先验 odom → 查库匹配 + `PnP` → 发观测。
///
/// 先验来源：优先矫正后里程计（`Firefly/CorrectedOdometry`，`gicp` 融合输出，
/// 含漂移修正，查询半径罩得住），无则回退原始 `Firefly/Odometry`（对照
/// `VINS-Fusion pose_graph.cpp` 用 `w_r_vio/w_t_vio` 矫正后位姿做查询与可视化）。
#[allow(clippy::too_many_lines)] // 订阅装配 + WaitSet 编排 + 先验回退，结构由进程接线驱动
fn run_loop(session: &mut Session, map: &VisionMap) -> Result<(), firefly_error::Error> {
    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "lightglue");
    let feat_sub = Subscriber::<FeatureMessage>::with_topic(&node, FEATURE_TOPIC)?;
    log::info!("已订阅特征话题 {FEATURE_TOPIC}");
    let odom_sub = Subscriber::<OdomMessage>::with_topic(&node, ODOM_TOPIC)?;
    log::info!("已订阅 odom 话题 {ODOM_TOPIC}（查询先验回退）");
    let corrected_sub = match CorrectedOdomSubscriber::new(&node) {
        Ok(s) => {
            log::info!("已订阅矫正后里程计 {CORRECTED_ODOM_TOPIC}（查询先验）");
            Some(s)
        }
        Err(e) => {
            log::warn!("矫正后里程计订阅不可用，回退原始 odom：{e}");
            None
        }
    };
    let obs_pub = Publisher::<PoseObservation>::with_topic(&node, POSE_OBS_TOPIC)?;
    log::info!("已打开位姿观测话题 {POSE_OBS_TOPIC}");

    let feat_events = TopicListener::with_topic(&node, FEATURE_TOPIC)?;
    let waitset = WaitSetBuilder::new()
        .create::<ipc::Service>()
        .map_err(|e| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::Internal,
                format!("创建 WaitSet 失败: {e:?}"),
            )
        })?;
    let _feat_guard = waitset.attach_notification(&feat_events).map_err(|e| {
        firefly_error::Error::new(
            firefly_error::ErrorKind::Internal,
            format!("挂载特征事件监听失败: {e:?}"),
        )
    })?;
    let tick_guard = waitset
        .attach_interval(std::time::Duration::from_millis(500))
        .map_err(|e| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::Internal,
                format!("挂载心跳定时器失败: {e:?}"),
            )
        })?;

    // 特征发布端带 notify（`aliked` 为 `with_topic_notify`）：特征到即醒；
    // 心跳仅兜底无事件时的 odom 缓存更新与断流自愈。
    let mut latest_odom: Option<OdomMessage> = None;
    let mut latest_corrected: Option<OdomMessage> = None;
    let on_event = |attachment_id: WaitSetAttachmentId<ipc::Service>| {
        let _ = attachment_id.has_event_from(&tick_guard);
        let _ = feat_events.drain();
        while let Ok(Some(sample)) = odom_sub.receive() {
            latest_odom = Some(*sample);
        }
        if let Some(sub) = &corrected_sub {
            while let Ok(Some(sample)) = sub.receive() {
                latest_corrected = Some(*sample);
            }
        }
        let mut latest_feat: Option<FeatureMessage> = None;
        while let Ok(Some(sample)) = feat_sub.receive() {
            latest_feat = Some(*sample);
        }
        // 先验优先级：矫正后（新鲜）> 原始 odom。
        let prior = latest_corrected
            .filter(|m| m.is_initialized)
            .or(latest_odom)
            .filter(|m| m.is_initialized);
        let (Some(feat), Some(odom)) = (latest_feat, prior) else {
            log::debug!(
                "视觉触发跳过（feat={} prior={}）",
                latest_feat.is_some(),
                prior.is_some()
            );
            return CallbackProgression::Continue;
        };
        if feat.timestamp - odom.timestamp > ODOM_FRESH_TIMEOUT {
            log::debug!(
                "视觉触发跳过（先验过期 feat_t={:.2} odom_t={:.2}）",
                feat.timestamp,
                odom.timestamp
            );
            return CallbackProgression::Continue;
        }
        // 特征时间戳即 sim 时钟；查询前后各 pump 一次（匹配阻塞下积压排空）。
        firefly_observability::set_sim_time(feat.timestamp);
        firefly_observability::pump_log_ipc(&log_ipc);
        match query_once(session, map, &feat, &odom) {
            Ok(Some(obs)) => {
                if let Err(e) = obs_pub.publish(obs) {
                    log::warn!("位姿观测发布失败: {e}");
                } else {
                    log::info!(
                        "视觉观测 t={:.2} pos=({:.2},{:.2},{:.2}) inliers {}/{} err {:.2}px",
                        obs.timestamp,
                        obs.position_x,
                        obs.position_y,
                        obs.position_z,
                        obs.num_inliers,
                        obs.total_points,
                        obs.error
                    );
                }
            }
            Ok(None) => log::debug!("视觉查询无有效位姿（拒收）"),
            Err(e) => log::warn!("视觉查询失败: {e}"),
        }
        firefly_observability::pump_log_ipc(&log_ipc);
        // 单查询/唤醒：特征 1Hz 节流，连续帧不堆积（corrected 缓存保留，
        // 其 100Hz 发布节拍保证新鲜度判定有效）
        latest_odom = None;
        CallbackProgression::Continue
    };
    match waitset.wait_and_process(on_event) {
        Ok(
            iceoryx2::waitset::WaitSetRunResult::Interrupt
            | iceoryx2::waitset::WaitSetRunResult::TerminationRequest,
        ) => {
            log::info!("收到终止信号，优雅退出");
            firefly_observability::pump_log_ipc(&log_ipc);
        }
        Ok(_) => {}
        Err(e) => {
            return Err(firefly_error::Error::temporary(
                firefly_error::ErrorKind::Internal,
                format!("WaitSet 事件等待失败: {e:?}"),
            ));
        }
    }
    firefly_observability::pump_log_ipc(&log_ipc);
    Ok(())
}

/// 单帧匹配：query 特征 vs 库图一帧 → 2D-3D 对应（丢弃填充/低分匹配）。
#[allow(clippy::type_complexity)]
fn match_frame(
    session: &mut Session,
    map: &VisionMap,
    frame_idx: usize,
    feat: &FeatureMessage,
    k0: &[f32],
    d0: &[f32],
) -> Result<(Vec<[f32; 2]>, Vec<[f64; 3]>), Box<dyn std::error::Error>> {
    let frame = &map.frames[frame_idx];
    let n_map = frame.points.len().min(NUM_POINTS);
    if n_map < 6 {
        return Ok((Vec::new(), Vec::new()));
    }
    // 库侧 0 填充（后过滤填充匹配）。
    let mut k1 = vec![0f32; NUM_POINTS * 2];
    let mut d1 = vec![0f32; NUM_POINTS * DESC_DIM];
    for (i, p) in frame.points.iter().take(n_map).enumerate() {
        k1[2 * i] = p.uv[0];
        k1[2 * i + 1] = p.uv[1];
        d1[i * DESC_DIM..(i + 1) * DESC_DIM].copy_from_slice(&p.descriptor);
    }
    let outputs = session.run(ort::inputs![
        "k0" => Tensor::from_array(([1usize, NUM_POINTS, 2], k0.to_vec().into_boxed_slice()))?,
        "d0" => Tensor::from_array(([1usize, NUM_POINTS, DESC_DIM], d0.to_vec().into_boxed_slice()))?,
        "k1" => Tensor::from_array(([1usize, NUM_POINTS, 2], k1.into_boxed_slice()))?,
        "d1" => Tensor::from_array(([1usize, NUM_POINTS, DESC_DIM], d1.into_boxed_slice()))?,
        "s0" => Tensor::from_array(([1usize, 2], vec![WIDTH, HEIGHT].into_boxed_slice()))?,
        "s1" => Tensor::from_array(([1usize, 2], vec![WIDTH, HEIGHT].into_boxed_slice()))?,
    ])?;
    let (_, matches) = outputs["matches0"].try_extract_tensor::<i64>()?;
    let (_, scores) = outputs["scores0"].try_extract_tensor::<f32>()?;
    let mut pairs_2d = Vec::new();
    let mut pairs_3d = Vec::new();
    for qi in 0..NUM_POINTS {
        let mi = matches[qi];
        if mi < 0 || mi as usize >= n_map || scores[qi] < SCORE_THRESHOLD {
            continue;
        }
        pairs_2d.push(feat.keypoints[qi]);
        pairs_3d.push(frame.points[mi as usize].position);
    }
    Ok((pairs_2d, pairs_3d))
}

/// 单次查询：先验邻域多帧匹配拼对应 → `PnP` → 机体位姿观测。
fn query_once(
    session: &mut Session,
    map: &VisionMap,
    feat: &FeatureMessage,
    odom: &OdomMessage,
) -> Result<Option<PoseObservation>, Box<dyn std::error::Error>> {
    let t_body_prior = odom_to_matrix(odom);
    let prior_pos = [
        t_body_prior[(0, 3)],
        t_body_prior[(1, 3)],
        t_body_prior[(2, 3)],
    ];
    let candidates = map.query_neighbors(prior_pos, QUERY_RADIUS, QUERY_TOPK);
    log::debug!(
        "视觉查询 t={:.2} prior=({:.1},{:.1},{:.1}) candidates={candidates:?}",
        feat.timestamp,
        prior_pos[0],
        prior_pos[1],
        prior_pos[2]
    );
    if candidates.is_empty() {
        return Ok(None);
    }
    // 多帧拼对应：单帧共面时 PnP 有翻转二义性，多视角点集破退化。
    // query 侧输入各帧复用（一次组装，多次推理）。
    let mut k0 = vec![0f32; NUM_POINTS * 2];
    let mut d0 = vec![0f32; NUM_POINTS * DESC_DIM];
    for i in 0..NUM_POINTS {
        k0[2 * i] = feat.keypoints[i][0];
        k0[2 * i + 1] = feat.keypoints[i][1];
        d0[i * DESC_DIM..(i + 1) * DESC_DIM].copy_from_slice(&feat.descriptors[i]);
    }
    let mut pairs_2d = Vec::new();
    let mut pairs_3d = Vec::new();
    for &frame_idx in &candidates {
        let (p2, p3) = match_frame(session, map, frame_idx, feat, &k0, &d0)?;
        pairs_2d.extend(p2);
        pairs_3d.extend(p3);
    }
    let total = pairs_2d.len();
    log::debug!(
        "视觉查询 t={:.2} 匹配对应 {} 组（TOPK={QUERY_TOPK}）",
        feat.timestamp,
        total
    );
    let intrinsics = CameraIntrinsics {
        focal: MUJOCO_FOCAL,
        cx: 160.0,
        cy: 120.0,
    };
    let t_cam_prior = body_pose_to_cam(&t_body_prior);
    let Some(pose) = solve_visual_pose(&pairs_2d, &pairs_3d, intrinsics, Some(t_cam_prior))
        .map_err(|e| format!("PnP: {e}"))?
    else {
        return Ok(None);
    };
    let t_body = cam_pose_to_body(&pose.t_global);
    let cov = pose_covariance(&pose);
    let mut covariance = [0f64; 36];
    for r in 0..6 {
        for c in 0..6 {
            covariance[r * 6 + c] = cov[(r, c)];
        }
    }
    let rot = t_body.fixed_view::<3, 3>(0, 0).into_owned();
    let quat = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix(&rot));
    let q = quat.quaternion();
    Ok(Some(PoseObservation {
        timestamp: feat.timestamp,
        source: OBS_SOURCE_VISUAL,
        position_x: t_body[(0, 3)],
        position_y: t_body[(1, 3)],
        position_z: t_body[(2, 3)],
        quat_x: q.i,
        quat_y: q.j,
        quat_z: q.k,
        quat_w: q.w,
        covariance,
        num_inliers: pose.num_inliers as u32,
        total_points: total as u32,
        error: pose.mean_reproj_px,
        converged: true,
    }))
}

#[cfg(test)]
mod tests {
    use super::DEFAULT_MODEL;

    /// 测试用权重路径：先 CWD相对，再相对 `CARGO_MANIFEST_DIR` 回仓库根；
    /// 都缺失时跳过（`models/` 已 ignore，CI 无权重）。
    fn test_model_path() -> Option<String> {
        let cwd = std::path::PathBuf::from(DEFAULT_MODEL);
        if cwd.is_file() {
            return cwd.to_str().map(str::to_owned);
        }
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join(DEFAULT_MODEL);
        if root.is_file() {
            return root.to_str().map(str::to_owned);
        }
        None
    }

    /// 端到端冒烟：权重缺失时跳过。
    #[test]
    fn onnx_smoke_or_skip() {
        let Some(path) = test_model_path() else {
            println!("skip: 无权重 {DEFAULT_MODEL}");
            return;
        };
        let mut session = super::load_session(&path).unwrap();
        super::smoke_inference(&mut session).unwrap();
    }
}
