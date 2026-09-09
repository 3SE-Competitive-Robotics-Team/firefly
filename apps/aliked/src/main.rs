//! `aliked` 进程：ALIKED-N16 特征提取（`ort` ONNX Runtime 推理）。
//!
//! 两种模式：
//! - 在线：订阅左目灰度 → `models/aliked-n16-k512.onnx`（静态 320×240，K=512）→
//!   经 `Firefly/Features` 发布特征（关键点/描述子/score）；
//! - 离线建库：`--build-map <frames_dir> --out <map.ffvmap>`——读摆拍 bin
//!   （`gen_vision_eval_data.py` 布局），提特征 + 深度反投影（-Z 相机系，
//!   与 `firefly-vision-match::calibration` 同惯例）写 `ffvmap` 库图。
//!
//! 权重在 `models/`（已 ignore，不进 git）。
//!
//! 运行：`cargo run -p aliked [--model models/aliked-n16-k512.onnx]`。

use firefly_pubsub::camera::{CAMERA_LEFT_TOPIC, GrayImageMessage};
use firefly_pubsub::event::{CAMERA_PAIR_TOPIC, TopicListener};
use firefly_pubsub::node::create_node;
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::vision::{DESC_DIM, FEATURE_TOPIC, FeatureMessage, MAX_FEATURES as NUM_POINTS};
use firefly_vision_map::{VisionKeyFrame, VisionMap, VisionMapPoint};
use firefly_vision_match::calibration::{MUJOCO_FOCAL, body_pose_to_cam};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetAttachmentId;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector4};
use ort::session::Session;
use std::path::{Path, PathBuf};

/// 缺省权重路径（相对运行目录，通常为仓库根）。
const DEFAULT_MODEL: &str = "models/aliked-n16-k512.onnx";
/// 导出约定：静态分辨率（见导出脚本）。
const WIDTH: usize = 320;
const HEIGHT: usize = 240;
/// 特征发布节流（秒）：ORT CPU 单帧约数百毫秒，视觉定位 1Hz 足够。
const FEATURE_PERIOD: f64 = 1.0;
/// 心跳周期（无事件时兜底）。
const HEARTBEAT: std::time::Duration = std::time::Duration::from_millis(500);

/// 相机主点（与导出约定一致）。
const CX: f64 = 160.0;
const CY: f64 = 120.0;
/// 深度反投影有效区间（米；摆拍深度背景值为上千，需截断）。
const MIN_DEPTH: f64 = 0.2;
const MAX_DEPTH: f64 = 20.0;

/// 解析命令行参数：`--model`（在线/离线通用）、`--build-map <dir>` +
/// `--out <map.ffvmap>`（离线建库，两者必须同时出现）。
fn parse_args() -> Result<(String, Option<PathBuf>, Option<PathBuf>), String> {
    let mut it = std::env::args().skip(1);
    let mut model = DEFAULT_MODEL.to_owned();
    let mut build_map: Option<PathBuf> = None;
    let mut out: Option<PathBuf> = None;
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--model" => {
                model = it
                    .next()
                    .ok_or_else(|| "missing --model value".to_owned())?;
            }
            "--build-map" => {
                build_map = Some(PathBuf::from(
                    it.next()
                        .ok_or_else(|| "missing --build-map value".to_owned())?,
                ));
            }
            "--out" => {
                out = Some(PathBuf::from(
                    it.next().ok_or_else(|| "missing --out value".to_owned())?,
                ));
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    match (&build_map, &out) {
        (Some(_), None) => return Err("--build-map 需同时给 --out <map.ffvmap>".to_owned()),
        (None, Some(_)) => return Err("--out 需同时给 --build-map <frames_dir>".to_owned()),
        _ => {}
    }
    Ok((model, build_map, out))
}

/// 加载 ONNX 会话（文件缺失即报错，不静默）。
///
/// 线程约束：ORT 缺省占满全核（实测 aliked 常驻 200%+，饿死 vio 的 IMU
/// 消费致断流）；在线推理节流 1Hz，锁 1 intra-op 线程不影响输出。
fn load_session(model: &str) -> Result<Session, Box<dyn std::error::Error>> {
    if !std::path::Path::new(model).is_file() {
        return Err(format!("权重缺失：{model}（见 models/，离线导出，不进 git）").into());
    }
    let session = Session::builder()?
        .with_intra_threads(1)?
        .commit_from_file(model)?;
    log::info!("aliked 会话就绪：{model}");
    Ok(session)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    let (model, build_map, out) = parse_args().map_err(|e| {
        eprintln!("{e}\n用法：aliked [--model models/aliked-n16-k512.onnx]\n       aliked --build-map <frames_dir> --out <map.ffvmap> [--model ...]");
        std::process::exit(2);
    })?;
    let mut session = load_session(&model)?;
    if let (Some(dir), Some(out)) = (build_map, out) {
        build_map_offline(&mut session, &dir, &out)?;
        return Ok(());
    }
    smoke_inference(&mut session)?;
    run_loop(&mut session)?;
    Ok(())
}

/// 主循环：相机对事件唤醒 → 取最新左目帧 → 节流 1Hz 推理 → 发布特征。
fn run_loop(session: &mut Session) -> Result<(), firefly_error::Error> {
    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "aliked");
    let left_sub = Subscriber::<GrayImageMessage>::with_topic(&node, CAMERA_LEFT_TOPIC)?;
    log::info!("已订阅左目话题 {CAMERA_LEFT_TOPIC}");
    let feat_pub = Publisher::<FeatureMessage>::with_topic_notify(&node, FEATURE_TOPIC)?;
    log::info!("已打开特征话题 {FEATURE_TOPIC}（带事件唤醒）");

    let cam_events = TopicListener::with_topic(&node, CAMERA_PAIR_TOPIC)?;
    let waitset = WaitSetBuilder::new()
        .create::<ipc::Service>()
        .map_err(|e| {
            firefly_error::Error::new(
                firefly_error::ErrorKind::Internal,
                format!("创建 WaitSet 失败: {e:?}"),
            )
        })?;
    let _cam_guard = waitset.attach_notification(&cam_events).map_err(|e| {
        firefly_error::Error::new(
            firefly_error::ErrorKind::Internal,
            format!("挂载相机事件监听失败: {e:?}"),
        )
    })?;
    let tick_guard = waitset.attach_interval(HEARTBEAT).map_err(|e| {
        firefly_error::Error::new(
            firefly_error::ErrorKind::Internal,
            format!("挂载心跳定时器失败: {e:?}"),
        )
    })?;

    let mut next_feat = 0.0f64;
    let on_event = |attachment_id: WaitSetAttachmentId<ipc::Service>| {
        let _ = attachment_id.has_event_from(&tick_guard);
        let _ = cam_events.drain();
        // 排空左目队列，只取最新帧
        let mut latest: Option<GrayImageMessage> = None;
        while let Ok(Some(sample)) = left_sub.receive() {
            latest = Some(*sample);
        }
        let Some(frame) = latest else {
            return CallbackProgression::Continue;
        };
        if frame.timestamp + 1e-9 < next_feat {
            return CallbackProgression::Continue;
        }
        next_feat = frame.timestamp + FEATURE_PERIOD;
        // 相机帧时间戳即 sim 时钟（传感器时钟 = 仿真时钟）；推理前后各 pump
        // 一次（推理阻塞数百 ms，积压日志及时排空，不等下一帧）。
        firefly_observability::set_sim_time(frame.timestamp);
        firefly_observability::pump_log_ipc(&log_ipc);
        match infer_frame(session, &frame) {
            Ok(msg) => {
                if let Err(e) = feat_pub.publish(msg) {
                    log::warn!("特征发布失败: {e}");
                }
            }
            Err(e) => log::warn!("特征推理失败: {e}"),
        }
        firefly_observability::pump_log_ipc(&log_ipc);
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

/// 模型推理：u8 灰度 → f32 NCHW（/255，三通道复用）→ keypoints/descriptors/scores
/// （`[1, K, 2]`/`[1, K, 128]`/`[1, K]`，展平）。在线/离线建库共用。
#[allow(clippy::type_complexity)]
fn run_model(
    session: &mut Session,
    gray: &[u8],
    width: usize,
    height: usize,
) -> Result<(Vec<f32>, Vec<f32>, Vec<f32>), Box<dyn std::error::Error>> {
    use ort::value::Tensor;
    if width != WIDTH || height != HEIGHT {
        return Err(format!("分辨率不匹配: {width}x{height}（导出约定 {WIDTH}x{HEIGHT}）").into());
    }
    let n = width * height;
    let mut input = vec![0f32; 3 * n];
    for (i, &px) in gray.iter().enumerate() {
        let v = f32::from(px) / 255.0;
        input[i] = v;
        input[n + i] = v;
        input[2 * n + i] = v;
    }
    let tensor = Tensor::from_array(([1usize, 3, height, width], input.into_boxed_slice()))?;
    let outputs = session.run(ort::inputs!["image" => tensor])?;
    let (shape, kpts) = outputs["keypoints"].try_extract_tensor::<f32>()?;
    assert_eq!(&shape[..], &[1, NUM_POINTS as i64, 2]);
    let (_, descs) = outputs["descriptors"].try_extract_tensor::<f32>()?;
    let (_, scores) = outputs["scores"].try_extract_tensor::<f32>()?;
    Ok((kpts.to_vec(), descs.to_vec(), scores.to_vec()))
}

/// 单帧推理（在线）：灰度帧 → 特征消息（定长 512，大数组是设计使然）。
#[allow(clippy::large_stack_arrays)]
fn infer_frame(
    session: &mut Session,
    frame: &GrayImageMessage,
) -> Result<FeatureMessage, Box<dyn std::error::Error>> {
    let (kpts, descs, scores) = run_model(
        session,
        &frame.data,
        frame.width as usize,
        frame.height as usize,
    )?;
    let mut msg = FeatureMessage {
        timestamp: frame.timestamp,
        count: NUM_POINTS as u32,
        keypoints: [[0.0; 2]; NUM_POINTS],
        descriptors: [[0.0; DESC_DIM]; NUM_POINTS],
        scores: [0.0; NUM_POINTS],
    };
    for i in 0..NUM_POINTS {
        msg.keypoints[i] = [kpts[2 * i], kpts[2 * i + 1]];
        msg.scores[i] = scores[i];
        for d in 0..DESC_DIM {
            msg.descriptors[i][d] = descs[i * DESC_DIM + d];
        }
    }
    Ok(msg)
}

/// 离线建库：摆拍 bin（`gen_vision_eval_data.py` 布局）→ 每帧提特征 +
/// 深度反投影（`Xc=(dx·d, dy·d, -d)`，`dy=-(v-cy)/f` 与
/// `firefly-map::DepthCamera::update_from_depth` 同式）→ 写 `ffvmap` 库图。
fn build_map_offline(
    session: &mut Session,
    frames_dir: &Path,
    out: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut bins: Vec<PathBuf> = std::fs::read_dir(frames_dir)?
        .collect::<std::io::Result<Vec<_>>>()?
        .into_iter()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "bin"))
        .collect();
    bins.sort();
    if bins.is_empty() {
        return Err(format!("建库目录无 bin 帧: {}", frames_dir.display()).into());
    }
    let mut map = VisionMap::new();
    for (idx, path) in bins.iter().enumerate() {
        let buf = std::fs::read(path)?;
        let width = u64::from_le_bytes(buf[0..8].try_into()?) as usize;
        let height = u64::from_le_bytes(buf[8..16].try_into()?) as usize;
        let img_off = 16;
        let depth_off = img_off + width * height;
        let pos_off = depth_off + 4 * width * height;
        if buf.len() < pos_off + 64 {
            return Err(format!("帧文件截断: {}", path.display()).into());
        }
        let rd = |at: usize| f64::from_le_bytes(buf[at..at + 8].try_into().unwrap());
        let pos = [rd(pos_off), rd(pos_off + 8), rd(pos_off + 16)];
        let quat = [
            rd(pos_off + 24),
            rd(pos_off + 32),
            rd(pos_off + 40),
            rd(pos_off + 48),
        ];
        let t = rd(pos_off + 56);
        let (kpts, descs, scores) = run_model(session, &buf[img_off..depth_off], width, height)?;
        let t_body = Isometry3::from_parts(
            Translation3::new(pos[0], pos[1], pos[2]),
            UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                quat[3], quat[0], quat[1], quat[2],
            )),
        )
        .to_homogeneous();
        let t_cam = body_pose_to_cam(&t_body);
        let mut points = Vec::new();
        for i in 0..NUM_POINTS {
            let u = kpts[2 * i] as usize;
            let v = kpts[2 * i + 1] as usize;
            if u >= width || v >= height {
                continue;
            }
            let d = f64::from(f32::from_le_bytes(
                buf[depth_off + 4 * (v * width + u)..depth_off + 4 * (v * width + u) + 4]
                    .try_into()
                    .unwrap(),
            ));
            if !(MIN_DEPTH..=MAX_DEPTH).contains(&d) {
                continue;
            }
            let dx = (f64::from(kpts[2 * i]) - CX) / MUJOCO_FOCAL;
            let dy = -(f64::from(kpts[2 * i + 1]) - CY) / MUJOCO_FOCAL;
            let xw = t_cam * Vector4::new(dx * d, dy * d, -d, 1.0);
            points.push(VisionMapPoint {
                position: [xw[0], xw[1], xw[2]],
                uv: [kpts[2 * i], kpts[2 * i + 1]],
                descriptor: descs[i * DESC_DIM..(i + 1) * DESC_DIM].try_into().unwrap(),
                score: scores[i],
            });
        }
        let n_pts = points.len();
        map.frames.push(VisionKeyFrame {
            id: idx as u64,
            timestamp: t,
            position: pos,
            quat_xyzw: quat,
            points,
        });
        log::info!(
            "建库帧 {}: {n_pts} 点（深度有效 {n_pts}/{NUM_POINTS}）",
            path.file_name().unwrap_or_default().to_string_lossy()
        );
    }
    firefly_vision_map::save_map(&map, out)?;
    log::info!(
        "库图已写 {}（{} 帧，{} 点）",
        out.display(),
        map.frames.len(),
        map.num_points()
    );
    Ok(())
}

/// 冒烟推理：零图单帧，校验输出形状（K=512、描述子 128 维）。
fn smoke_inference(session: &mut Session) -> Result<(), Box<dyn std::error::Error>> {
    use ort::value::Tensor;
    let input = Tensor::from_array((
        [1usize, 3, HEIGHT, WIDTH],
        vec![0f32; 3 * HEIGHT * WIDTH].into_boxed_slice(),
    ))?;
    let outputs = session.run(ort::inputs!["image" => input])?;
    let (shape, _) = outputs["keypoints"].try_extract_tensor::<f32>()?;
    assert_eq!(&shape[..], &[1, NUM_POINTS as i64, 2], "keypoints 形状");
    let (shape, _) = outputs["descriptors"].try_extract_tensor::<f32>()?;
    assert_eq!(
        &shape[..],
        &[1, NUM_POINTS as i64, DESC_DIM as i64],
        "descriptors 形状"
    );
    let (shape, _) = outputs["scores"].try_extract_tensor::<f32>()?;
    assert_eq!(&shape[..], &[1, NUM_POINTS as i64], "scores 形状");
    log::info!("aliked 冒烟推理通过（输出 K={NUM_POINTS}，描述子 {DESC_DIM} 维）");
    Ok(())
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
