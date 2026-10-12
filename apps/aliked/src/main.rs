//! `aliked` 进程：ALIKED-N16 特征提取（`ort` ONNX Runtime 推理）。
//!
//! 两种模式：
//! - 在线：订阅左目灰度 → `models/aliked-n16-k512.onnx`（静态 320×240，K=512）→
//!   经 `Firefly/Features` 发布特征（关键点/描述子/score）；
//! - 离线建库：`--build-map <frames_dir> --out <map.ffvmap>`——读摆拍 bin
//!   （`collect_vision_map.py` 布局），提特征 + 深度反投影（-Z 相机系，
//!   与 `firefly-vision-match::calibration` 同惯例）写 `ffvmap` 库图。
//!
//! 权重在 `models/`（已 ignore，不进 git）。
//!
//! 运行：`cargo run --release -p aliked [--model models/aliked-n16-k512.onnx]`。

use firefly_pubsub::camera::{CAMERA_LEFT_TOPIC, GrayImageMessage};
use firefly_pubsub::event::{CAMERA_PAIR_TOPIC, TopicListener};
use firefly_pubsub::node::create_node;
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::{Received, Subscriber};
use firefly_pubsub::vision::{DESC_DIM, FEATURE_TOPIC, FeatureMessage, MAX_FEATURES as NUM_POINTS};
use firefly_vision_map::{VisionKeyFrame, VisionMap, VisionMapPoint};
use firefly_vision_match::calibration::{body_pose_to_cam, left_from_depth, pinhole};
use iceoryx2::prelude::*;
use iceoryx2::waitset::WaitSetAttachmentId;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use ort::session::{Session, SessionOutputs};
use ort::value::Tensor;
use std::path::{Path, PathBuf};

/// 缺省权重路径（相对运行目录，通常为仓库根）。
const DEFAULT_MODEL: &str = "models/aliked-n16-k512.onnx";
/// 导出约定：静态分辨率（见导出脚本）。
const WIDTH: usize = 320;
const HEIGHT: usize = 240;
/// 特征处理最小间隔（传感器秒）；只处理最新帧，推理慢时丢帧而不积压。
/// 默认特征推理频率（Hz）。地图查询节流 1Hz、在线回环关键帧 1s，供给侧 2Hz 已足够：
/// 查询侧要求特征与里程计时间差 ≤ `ODOM_FRESH_TIMEOUT`（1s），2Hz 每 0.5s 一帧。
/// 更高频率只增加重复推理——消费者按各自节流取最新帧。
const DEFAULT_RATE_HZ: f64 = 2.0;
/// 允许的频率范围（Hz）：低于 0.5 会顶破查询侧 1s 新鲜度门，高于 10 超过相机帧率。
const RATE_HZ_RANGE: std::ops::RangeInclusive<f64> = 0.5..=10.0;
/// 时钟回拨重整阈值（秒）：帧时间戳比节流门限早超此值即视为传感器时钟重启
///（`sim` 重启后任务时钟归零），重整节流门限而非永久静默（否则重启后无特征）。
const CLOCK_REWIND_MARGIN: f64 = 2.0;
/// 心跳周期（无事件时兜底）。
const HEARTBEAT: std::time::Duration = std::time::Duration::from_millis(500);

/// 命令行参数。
struct Args {
    model: String,
    build_map: Option<PathBuf>,
    out: Option<PathBuf>,
    /// 特征推理频率（Hz，见 [`DEFAULT_RATE_HZ`]）。
    rate_hz: f64,
}

/// 解析命令行参数：`--model`（在线/离线通用）、`--build-map <dir>` +
/// `--out <map.ffvmap>`（离线建库，两者必须同时出现）、`--rate-hz`（在线推理频率）。
fn parse_args() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    let mut model = DEFAULT_MODEL.to_owned();
    let mut build_map: Option<PathBuf> = None;
    let mut out: Option<PathBuf> = None;
    let mut rate_hz = DEFAULT_RATE_HZ;
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
            "--rate-hz" => {
                let value = it
                    .next()
                    .ok_or_else(|| "missing --rate-hz value".to_owned())?;
                rate_hz = value
                    .parse::<f64>()
                    .map_err(|e| format!("--rate-hz 不是数值（{value}）：{e}"))?;
                if !RATE_HZ_RANGE.contains(&rate_hz) {
                    return Err(format!(
                        "--rate-hz 须在 {RATE_HZ_RANGE:?} 内，得到 {rate_hz}"
                    ));
                }
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    match (&build_map, &out) {
        (Some(_), None) => return Err("--build-map 需同时给 --out <map.ffvmap>".to_owned()),
        (None, Some(_)) => return Err("--out 需同时给 --build-map <frames_dir>".to_owned()),
        _ => {}
    }
    Ok(Args {
        model,
        build_map,
        out,
        rate_hz,
    })
}

/// ONNX 会话与可复用的 CPU 输入张量；每次推理前完整覆盖三个通道。
struct Model {
    session: Session,
    input: Tensor<f32>,
}

/// 加载 ONNX 会话（文件缺失即报错）。四条算子线程，空闲休眠。
fn load_model(model: &str) -> Result<Model, Box<dyn std::error::Error>> {
    if !std::path::Path::new(model).is_file() {
        return Err(format!("权重缺失：{model}（见 models/，离线导出，不进 git）").into());
    }
    let session = Session::builder()?
        .with_intra_threads(4)?
        .with_intra_op_spinning(false)?
        .commit_from_file(model)?;
    log::info!("aliked 会话就绪：{model}");
    let input = Tensor::from_array((
        [1usize, 3, HEIGHT, WIDTH],
        vec![0f32; 3 * HEIGHT * WIDTH].into_boxed_slice(),
    ))?;
    Ok(Model { session, input })
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    let Args {
        model,
        build_map,
        out,
        rate_hz,
    } = parse_args().map_err(|e| {
        eprintln!(
            "{e}\n用法：aliked [--model models/aliked-n16-k512.onnx] [--rate-hz 2.0]\n       \
             aliked --build-map <frames_dir> --out <map.ffvmap> [--model ...]"
        );
        std::process::exit(2);
    })?;
    let mut model = load_model(&model)?;
    if let (Some(dir), Some(out)) = (build_map, out) {
        build_map_offline(&mut model, &dir, &out)?;
        return Ok(());
    }
    smoke_inference(&mut model)?;
    run_loop(&mut model, 1.0 / rate_hz)?;
    Ok(())
}

/// 主循环：相机对事件唤醒 → 取最新左目帧 → 按 `period` 节流推理 → 发布特征。
fn run_loop(model: &mut Model, period: f64) -> Result<(), firefly_error::Error> {
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
        // 排空左目队列，只取最新帧；持样本句柄零拷贝（77KB 灰度不落地拷贝）。
        let mut latest: Option<Received<GrayImageMessage>> = None;
        while let Ok(Some(sample)) = left_sub.receive() {
            latest = Some(sample);
        }
        let Some(frame) = latest.as_deref() else {
            return CallbackProgression::Continue;
        };
        if frame.timestamp + 1e-9 < next_feat {
            // 时钟回拨（传感器重启）vs 正常节流：回拨则重整门限，否则跳过
            if next_feat - frame.timestamp > CLOCK_REWIND_MARGIN {
                log::info!(
                    "传感器时钟回拨（{:.2} → {:.2}），特征节流重整",
                    next_feat - period,
                    frame.timestamp
                );
            } else {
                return CallbackProgression::Continue;
            }
        }
        next_feat = frame.timestamp + period;
        // 相机帧时间戳即 sim 时钟（传感器时钟 = 仿真时钟）；推理前后各 pump
        // 一次（推理阻塞数百 ms，积压日志及时排空，不等下一帧）。
        firefly_observability::set_sim_time(frame.timestamp);
        firefly_observability::pump_log_ipc(&log_ipc);
        let root = fastrace::Span::root("aliked", fastrace::prelude::SpanContext::random());
        let trace_guard = root.set_local_parent();
        match infer_frame(model, frame) {
            Ok(msg) => {
                log::debug!(
                    "特征发布 t={:.2}（{} 点，含低分）",
                    msg.timestamp,
                    NUM_POINTS
                );
                if let Err(e) = feat_pub.publish(msg) {
                    log::warn!("特征发布失败: {e}");
                }
            }
            Err(e) => log::warn!("特征推理失败: {e}"),
        }
        firefly_observability::pump_log_ipc(&log_ipc);
        drop(trace_guard);
        drop(root);
        fastrace::flush();
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

impl Model {
    /// u8 灰度 → f32 NCHW（/255，三通道复用）；输出持有至本帧消费完成。
    #[fastrace::trace]
    fn run(
        &mut self,
        gray: &[u8],
        width: usize,
        height: usize,
    ) -> Result<SessionOutputs<'_>, Box<dyn std::error::Error>> {
        if width != WIDTH || height != HEIGHT || gray.len() != WIDTH * HEIGHT {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                format!(
                    "输入不匹配: {width}x{height}, {} 像素（约定 {WIDTH}x{HEIGHT}）",
                    gray.len()
                ),
            )
            .with_operation("aliked input")
            .into());
        }
        let n = width * height;
        let (_, input) = self.input.extract_tensor_mut();
        for (i, &px) in gray.iter().enumerate() {
            let v = f32::from(px) / 255.0;
            input[i] = v;
            input[n + i] = v;
            input[2 * n + i] = v;
        }
        Ok(self.session.run(ort::inputs!["image" => &self.input])?)
    }
}

/// 借用 ONNX 输出的展平切片；消息和库图只在最终存储处取得自有数据。
struct Features<'a> {
    keypoints: &'a [f32],
    descriptors: &'a [f32],
    scores: &'a [f32],
}

impl<'a> Features<'a> {
    fn from_outputs(outputs: &'a SessionOutputs<'_>) -> Result<Self, Box<dyn std::error::Error>> {
        let (shape, keypoints) = outputs["keypoints"].try_extract_tensor::<f32>()?;
        assert_eq!(&shape[..], &[1, NUM_POINTS as i64, 2], "keypoints 形状");
        let (shape, descriptors) = outputs["descriptors"].try_extract_tensor::<f32>()?;
        assert_eq!(
            &shape[..],
            &[1, NUM_POINTS as i64, DESC_DIM as i64],
            "descriptors 形状"
        );
        let (shape, scores) = outputs["scores"].try_extract_tensor::<f32>()?;
        assert_eq!(&shape[..], &[1, NUM_POINTS as i64], "scores 形状");
        Ok(Self {
            keypoints,
            descriptors,
            scores,
        })
    }
}

/// 单帧推理（在线）：灰度帧 → 特征消息（定长 512，大数组是设计使然）。
#[allow(clippy::large_stack_arrays)]
fn infer_frame(
    model: &mut Model,
    frame: &GrayImageMessage,
) -> Result<FeatureMessage, Box<dyn std::error::Error>> {
    let outputs = model.run(&frame.data, frame.width as usize, frame.height as usize)?;
    let features = Features::from_outputs(&outputs)?;
    let mut msg = FeatureMessage {
        timestamp: frame.timestamp,
        count: NUM_POINTS as u32,
        keypoints: [[0.0; 2]; NUM_POINTS],
        descriptors: [[0.0; DESC_DIM]; NUM_POINTS],
        scores: [0.0; NUM_POINTS],
    };
    for i in 0..NUM_POINTS {
        msg.keypoints[i] = [features.keypoints[2 * i], features.keypoints[2 * i + 1]];
        msg.scores[i] = features.scores[i];
        msg.descriptors[i].copy_from_slice(&features.descriptors[i * DESC_DIM..(i + 1) * DESC_DIM]);
    }
    Ok(msg)
}

/// 离线建库：摆拍 bin（`collect_vision_map.py` 布局）→ 每帧提特征 +
/// 深度向左目重投影、遮挡筛选与连续表面采样 → 地图路标 → 写 `ffvmap`。
fn build_map_offline(
    model: &mut Model,
    frames_dir: &Path,
    out: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "aliked-build");
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
        firefly_observability::pump_log_ipc(&log_ipc);
        let buf = std::fs::read(path)?;
        if buf.len() != 16 + 320 * 240 * 5 + 64 {
            return Err(format!("invalid capture size: {}", path.display()).into());
        }
        let width = u64::from_le_bytes(buf[0..8].try_into()?) as usize;
        let height = u64::from_le_bytes(buf[8..16].try_into()?) as usize;
        if width != 320 || height != 240 {
            return Err("capture calibration requires 320x240".into());
        }
        let img_off = 16;
        let depth_off = img_off + width * height;
        let pos_off = depth_off + 4 * width * height;
        let rd = |at: usize| f64::from_le_bytes(buf[at..at + 8].try_into().unwrap());
        let pos = [rd(pos_off), rd(pos_off + 8), rd(pos_off + 16)];
        let quat = [24, 32, 40, 48].map(|offset| rd(pos_off + offset));
        let t = rd(pos_off + 56);
        let outputs = model.run(&buf[img_off..depth_off], width, height)?;
        let features = Features::from_outputs(&outputs)?;
        let t_body = Isometry3::from_parts(
            Translation3::new(pos[0], pos[1], pos[2]),
            UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                quat[3], quat[0], quat[1], quat[2],
            )),
        )
        .to_homogeneous();
        let t_cam = body_pose_to_cam(&t_body);
        let depths: Vec<_> = buf[depth_off..pos_off]
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| f32::from_le_bytes(*b))
            .collect();
        let registered = firefly_vision_match::depth::register_depth(
            &depths,
            pinhole(),
            pinhole(),
            &left_from_depth(),
        )?;
        let mut points = Vec::new();
        for i in 0..NUM_POINTS {
            let Some(point) = registered.point_at([
                f64::from(features.keypoints[2 * i]),
                f64::from(features.keypoints[2 * i + 1]),
            ]) else {
                continue;
            };
            let xw = t_cam * point.push(1.);
            points.push(VisionMapPoint {
                position: [xw[0], xw[1], xw[2]],
                uv: [features.keypoints[2 * i], features.keypoints[2 * i + 1]],
                descriptor: features.descriptors[i * DESC_DIM..(i + 1) * DESC_DIM]
                    .try_into()
                    .unwrap(),
                score: features.scores[i],
            });
        }
        let n_pts = points.len();
        if !landmark_coverage(&points) {
            log::info!("拒绝低覆盖库帧 {}: {n_pts} landmarks", path.display());
            continue;
        }
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
    if map.frames.is_empty() {
        return Err("no visual map frame meets landmark count and spatial coverage".into());
    }
    firefly_vision_map::save_map(&map, out)?;
    log::info!(
        "库图已写 {}（{} 帧，{} 点）",
        out.display(),
        map.frames.len(),
        map.num_points()
    );
    firefly_observability::pump_log_ipc(&log_ipc);
    Ok(())
}

/// 至少 64 个有效路标，覆盖 4×4 网格中的至少 6 格。
fn landmark_coverage(points: &[VisionMapPoint]) -> bool {
    let mut occupied = [false; 16];
    for point in points {
        let x = (point.uv[0] / 80.) as usize;
        let y = (point.uv[1] / 60.) as usize;
        if x < 4 && y < 4 {
            occupied[y * 4 + x] = true;
        }
    }
    points.len() >= 64 && occupied.iter().filter(|filled| **filled).count() >= 6
}

/// 冒烟推理：零图单帧，校验输出形状（K=512、描述子 128 维）。
fn smoke_inference(model: &mut Model) -> Result<(), Box<dyn std::error::Error>> {
    let outputs = model.run(&vec![0; WIDTH * HEIGHT], WIDTH, HEIGHT)?;
    Features::from_outputs(&outputs)?;
    log::info!("aliked 冒烟推理通过（输出 K={NUM_POINTS}，描述子 {DESC_DIM} 维）");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

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
            log::warn!("skip: 无权重 {DEFAULT_MODEL}");
            return;
        };
        let mut model = super::load_model(&path).unwrap();
        super::smoke_inference(&mut model).unwrap();
    }

    #[test]
    fn reused_input_matches_fresh_tensor_across_frames_or_skip() {
        let Some(path) = test_model_path() else {
            log::warn!("skip: 无权重 {DEFAULT_MODEL}");
            return;
        };
        let mut model = load_model(&path).unwrap();
        let input_ptr = model.input.extract_tensor().1.as_ptr();
        let mut frame = GrayImageMessage {
            width: WIDTH as u32,
            height: HEIGHT as u32,
            ..GrayImageMessage::default()
        };
        assert!(model.run(&frame.data, WIDTH - 1, HEIGHT).is_err());
        assert!(model.run(&frame.data[..WIDTH], WIDTH, HEIGHT).is_err());
        let mut retained = None;
        for seed in [17usize, 0, 93] {
            frame.timestamp += 0.5;
            for (i, px) in frame.data.iter_mut().enumerate() {
                *px = ((i * seed + (i / WIDTH) * 31 * seed) % 256) as u8;
            }
            let msg = infer_frame(&mut model, &frame).unwrap();
            assert_eq!(model.input.extract_tensor().1.as_ptr(), input_ptr);

            let input: Vec<f32> = frame.data.iter().map(|&px| f32::from(px) / 255.0).collect();
            let fresh = Tensor::from_array((
                [1usize, 3, HEIGHT, WIDTH],
                input.repeat(3).into_boxed_slice(),
            ))
            .unwrap();
            let outputs = model.session.run(ort::inputs!["image" => fresh]).unwrap();
            let expected = Features::from_outputs(&outputs).unwrap();
            assert_eq!(msg.timestamp, frame.timestamp);
            assert_eq!(msg.count, NUM_POINTS as u32);
            assert_eq!(msg.keypoints.as_flattened(), expected.keypoints);
            assert_eq!(msg.descriptors.as_flattened(), expected.descriptors);
            assert_eq!(&msg.scores, expected.scores);
            if retained.is_none() {
                retained = Some((
                    Box::new(msg),
                    expected.keypoints.to_vec(),
                    expected.descriptors.to_vec(),
                    expected.scores.to_vec(),
                ));
            }
        }
        let (msg, keypoints, descriptors, scores) = retained.unwrap();
        assert_eq!(msg.keypoints.as_flattened(), keypoints);
        assert_eq!(msg.descriptors.as_flattened(), descriptors);
        assert_eq!(msg.scores.as_slice(), scores);
    }
}
