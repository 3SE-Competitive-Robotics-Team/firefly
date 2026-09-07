//! 视觉全局定位离线评测：库图 × 摆拍帧全链（`ALIKED` → `LightGlue` → `PnP`）位姿误差。
//!
//! 数据由 `scripts/gen_vision_eval_data.py` 生成（`logs/bench/vision_eval/`，
//! `gitignored`）：`frames/<traj>_<idx>.bin`（left 灰度 + 深度 + 真值位姿）。
//! 库图为 `apps/planner/maps/straight_forward.ffvmap`（`collect_vision_frames.py`
//! + 离线建库产物）。
//!
//! 每帧：`ALIKED` 提特征 → 真值邻域 `TOPK` 帧匹配（排除自身帧，
//! 测闭环式泛化而非自匹配）→ 多帧拼对应 → `PnP` → 与真值比位置/姿态误差。
//! 另写 `eval_rows.csv` 供离线分析。
//!
//! 运行：`cargo test --release -p lightglue --test vision_traj_eval -- --nocapture`
//! 本文件是评测 harness（输出即交付），不是门禁单测：只统计、不设断言。

use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

use firefly_pubsub::vision::{DESC_DIM, FeatureMessage, MAX_FEATURES as NUM_POINTS};
use firefly_vision_map::VisionMap;
use firefly_vision_match::calibration::{MUJOCO_FOCAL, body_pose_to_cam, cam_pose_to_body};
use firefly_vision_match::{CameraIntrinsics, solve_visual_pose};
use nalgebra::{Isometry3, Matrix4, Translation3, UnitQuaternion};
use ort::session::Session;
use ort::value::Tensor;

const WIDTH: i64 = 320;
const HEIGHT: i64 = 240;
/// 真值邻域查询半径（米）与候选帧数（与 `apps/lightglue` 在线一致）。
const QUERY_RADIUS: f64 = 3.0;
const QUERY_TOPK: usize = 3;
/// 匹配分数阈值（与 `apps/lightglue` 在线一致）。
const SCORE_THRESHOLD: f32 = 0.2;
/// 成功判据两档：修正档（对照 GICP eval）与定位档。
const STRICT_T: f64 = 0.1;
const STRICT_R_DEG: f64 = 2.0;
const LOOSE_T: f64 = 0.3;
const LOOSE_R_DEG: f64 = 5.0;

fn default_aliked_model() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("models")
        .join("aliked-n16-k512.onnx")
}

fn default_lg_model() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("models")
        .join("lightglue-aliked-k512.onnx")
}

fn default_map() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("apps")
        .join("planner")
        .join("maps")
        .join("straight_forward.ffvmap")
}

fn data_dir() -> PathBuf {
    if let Ok(dir) = std::env::var("VISION_EVAL_DIR") {
        return PathBuf::from(dir);
    }
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("logs")
        .join("bench")
        .join("vision_eval")
}

fn env_path(name: &str, default: &Path) -> PathBuf {
    std::env::var(name).map_or_else(|_| default.to_path_buf(), PathBuf::from)
}

/// 帧记录。
#[derive(Clone)]
struct EvalRow {
    name: String,
    terr: f64,
    rerr_deg: f64,
    inliers: usize,
    total: usize,
    reproj_px: f64,
}

fn read_le_u64(buf: &[u8], at: usize) -> usize {
    u64::from_le_bytes(buf[at..at + 8].try_into().unwrap()) as usize
}

fn read_le_f64(buf: &[u8], at: usize) -> f64 {
    f64::from_le_bytes(buf[at..at + 8].try_into().unwrap())
}

fn load_session(model: &PathBuf) -> Result<Session, Box<dyn std::error::Error>> {
    if !model.is_file() {
        return Err(format!("权重缺失：{}", model.display()).into());
    }
    Ok(Session::builder()?.commit_from_file(model)?)
}

/// 单帧 ALIKED 提取：u8 灰度 → f32 NCHW（/255，三通道复用）→ 特征消息。
#[allow(clippy::large_stack_arrays)]
fn extract_aliked(
    session: &mut Session,
    gray: &[u8],
) -> Result<FeatureMessage, Box<dyn std::error::Error>> {
    let image_size = (WIDTH * HEIGHT) as usize;
    let mut input = vec![0f32; 3 * image_size];
    for (i, &px) in gray.iter().enumerate() {
        let v = f32::from(px) / 255.0;
        input[i] = v;
        input[image_size + i] = v;
        input[2 * image_size + i] = v;
    }
    let tensor = Tensor::from_array((
        [1usize, 3, HEIGHT as usize, WIDTH as usize],
        input.into_boxed_slice(),
    ))?;
    let outputs = session.run(ort::inputs!["image" => tensor])?;
    let (shape, kpts) = outputs["keypoints"].try_extract_tensor::<f32>()?;
    assert_eq!(&shape[..], &[1, NUM_POINTS as i64, 2]);
    let (_, descs) = outputs["descriptors"].try_extract_tensor::<f32>()?;
    let (_, scores) = outputs["scores"].try_extract_tensor::<f32>()?;
    let mut msg = FeatureMessage {
        timestamp: 0.0,
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

/// 库帧匹配（与 `apps/lightglue` 在线同逻辑）：query 特征 vs 库图一帧 → 2D-3D 对应。
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

fn rot_err_deg(est: &Matrix4<f64>, gt: &Matrix4<f64>) -> f64 {
    let rot = est.fixed_view::<3, 3>(0, 0).into_owned().transpose()
        * gt.fixed_view::<3, 3>(0, 0).into_owned();
    ((rot.trace() - 1.0) / 2.0)
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees()
}

fn trans_err(est: &Matrix4<f64>, gt: &Matrix4<f64>) -> f64 {
    (est.fixed_view::<3, 1>(0, 3) - gt.fixed_view::<3, 1>(0, 3)).norm()
}

/// 单帧查询：真值邻域多帧匹配拼对应 → `PnP` → 机体位姿误差。
fn eval_frame(
    lg: &mut Session,
    map: &VisionMap,
    feat: &FeatureMessage,
    t_gt: &Matrix4<f64>,
    self_id: u64,
) -> Result<Option<EvalRow>, Box<dyn std::error::Error>> {
    let gt_pos = [t_gt[(0, 3)], t_gt[(1, 3)], t_gt[(2, 3)]];
    let candidates: Vec<usize> = map
        .query_neighbors(gt_pos, QUERY_RADIUS, QUERY_TOPK)
        .into_iter()
        .filter(|&i| map.frames[i].id != self_id)
        .collect();
    if candidates.is_empty() {
        return Ok(None);
    }
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
        let (p2, p3) = match_frame(lg, map, frame_idx, feat, &k0, &d0)?;
        pairs_2d.extend(p2);
        pairs_3d.extend(p3);
    }
    let total = pairs_2d.len();
    if total < 6 {
        return Ok(None);
    }
    let intrinsics = CameraIntrinsics {
        focal: MUJOCO_FOCAL,
        cx: 160.0,
        cy: 120.0,
    };
    let t_cam_prior = body_pose_to_cam(t_gt);
    let Some(pose) = solve_visual_pose(&pairs_2d, &pairs_3d, intrinsics, Some(t_cam_prior))
        .map_err(|e| format!("PnP: {e}"))?
    else {
        return Ok(None);
    };
    let t_body = cam_pose_to_body(&pose.t_global);
    Ok(Some(EvalRow {
        name: String::new(),
        terr: trans_err(&t_body, t_gt),
        rerr_deg: rot_err_deg(&t_body, t_gt),
        inliers: pose.num_inliers,
        total,
        reproj_px: pose.mean_reproj_px,
    }))
}

fn pct(ok: usize, all: usize) -> f64 {
    100.0 * ok as f64 / all.max(1) as f64
}

fn median(v: &[f64]) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    let mut s = v.to_vec();
    s.sort_by(f64::total_cmp);
    s[s.len() / 2]
}

#[test]
#[allow(clippy::too_many_lines)] // 评测 harness：读帧→提特征→匹配→PnP→统计→落盘，一次性流程
fn vision_traj_eval() {
    let aliked_model = env_path("VISION_ALIKED_MODEL", &default_aliked_model());
    let lg_model = env_path("VISION_LG_MODEL", &default_lg_model());
    let map_path = env_path("VISION_MAP", &default_map());
    let Ok(mut aliked) = load_session(&aliked_model) else {
        println!("skip: 无权重 {}", aliked_model.display());
        return;
    };
    let Ok(mut lg) = load_session(&lg_model) else {
        println!("skip: 无权重 {}", lg_model.display());
        return;
    };
    let map = firefly_vision_map::load_map(&map_path).unwrap();
    println!(
        "map: {} frames, {} points",
        map.frames.len(),
        map.num_points()
    );

    let frames_dir = data_dir().join("frames");
    let mut frames: Vec<PathBuf> = fs::read_dir(&frames_dir)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|p| p.extension().is_some_and(|e| e == "bin"))
        .collect();
    frames.sort();
    println!("frames: {}", frames.len());

    let mut rows: Vec<(String, EvalRow)> = Vec::new();
    let (mut n_skip, mut ms_total) = (0usize, 0u128);
    let clock = Instant::now();
    for (idx, path) in frames.iter().enumerate() {
        let stem = path.file_stem().unwrap().to_string_lossy().to_string();
        let self_id: u64 = stem
            .rsplit_once('_')
            .and_then(|(_, n)| n.parse().ok())
            .unwrap_or(u64::MAX);
        let buf = fs::read(path).unwrap();
        let width = read_le_u64(&buf, 0);
        let height = read_le_u64(&buf, 8);
        let img_off = 16;
        let depth_off = img_off + width * height;
        let pos_off = depth_off + 4 * width * height;
        let pos = [
            read_le_f64(&buf, pos_off),
            read_le_f64(&buf, pos_off + 8),
            read_le_f64(&buf, pos_off + 16),
        ];
        let quat = [
            read_le_f64(&buf, pos_off + 24),
            read_le_f64(&buf, pos_off + 32),
            read_le_f64(&buf, pos_off + 40),
            read_le_f64(&buf, pos_off + 48),
        ];
        let t_gt = Isometry3::from_parts(
            Translation3::new(pos[0], pos[1], pos[2]),
            UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                quat[3], quat[0], quat[1], quat[2],
            )),
        )
        .to_homogeneous();
        let tick = Instant::now();
        let feat = match extract_aliked(&mut aliked, &buf[img_off..depth_off]) {
            Ok(f) => f,
            Err(e) => {
                println!("  {stem}: aliked 失败 {e}");
                n_skip += 1;
                continue;
            }
        };
        match eval_frame(&mut lg, &map, &feat, &t_gt, self_id) {
            Ok(Some(mut row)) => {
                row.name = stem.clone();
                ms_total += tick.elapsed().as_millis();
                rows.push((stem, row));
            }
            Ok(None) => {
                n_skip += 1;
                ms_total += tick.elapsed().as_millis();
                println!("  {stem}: 无有效位姿（匹配/内点不足）");
            }
            Err(e) => {
                n_skip += 1;
                ms_total += tick.elapsed().as_millis();
                println!("  {stem}: 查询失败 {e}");
            }
        }
        if idx % 6 == 0 {
            println!(
                "... {idx}/{} ({:.0}s)",
                frames.len(),
                clock.elapsed().as_secs_f64()
            );
        }
    }

    let n_all = rows.len();
    let strict = rows
        .iter()
        .filter(|(_, r)| r.terr < STRICT_T && r.rerr_deg < STRICT_R_DEG)
        .count();
    let loose = rows
        .iter()
        .filter(|(_, r)| r.terr < LOOSE_T && r.rerr_deg < LOOSE_R_DEG)
        .count();
    let terrs: Vec<f64> = rows.iter().map(|(_, r)| r.terr).collect();
    let rerrs: Vec<f64> = rows.iter().map(|(_, r)| r.rerr_deg).collect();
    let inls: Vec<usize> = rows.iter().map(|(_, r)| r.inliers).collect();
    println!("\n=== vision eval ===");
    println!(
        "frames={} solved={} skip={} | strict(<{STRICT_T}m/{STRICT_R_DEG}°) {strict}/{} {:.1}% | loose(<{LOOSE_T}m/{LOOSE_R_DEG}°) {loose}/{} {:.1}%",
        frames.len(),
        n_all,
        n_skip,
        n_all,
        pct(strict, n_all),
        n_all,
        pct(loose, n_all),
    );
    if n_all > 0 {
        println!(
            "terr  med={:.4} mean={:.4} max={:.4} m | rerr med={:.3} mean={:.3} max={:.3}° | inliers med={} min={} | avg={:.0}ms/query",
            median(&terrs),
            terrs.iter().sum::<f64>() / n_all as f64,
            terrs.iter().copied().fold(0.0, f64::max),
            median(&rerrs),
            rerrs.iter().sum::<f64>() / n_all as f64,
            rerrs.iter().copied().fold(0.0, f64::max),
            median(&inls.iter().map(|&v| v as f64).collect::<Vec<_>>()),
            inls.iter().copied().min().unwrap_or(0),
            ms_total as f64 / n_all as f64,
        );
    }
    let mut worst: Vec<&(String, EvalRow)> = rows
        .iter()
        .filter(|(_, r)| !(r.terr < STRICT_T && r.rerr_deg < STRICT_R_DEG))
        .collect();
    worst.sort_by(|a, b| b.1.terr.partial_cmp(&a.1.terr).unwrap());
    println!("--- worst failures (frame terr rerr inliers/total reproj) ---");
    for (name, r) in worst.iter().take(10) {
        println!(
            "  {:<28} {:.3}m {:.2}° {}/{} {:.2}px",
            name, r.terr, r.rerr_deg, r.inliers, r.total, r.reproj_px
        );
    }
    let mut csv = String::new();
    for (name, r) in &rows {
        let _ = writeln!(
            csv,
            "{},{:.4},{:.3},{},{},{:.3}",
            name, r.terr, r.rerr_deg, r.inliers, r.total, r.reproj_px
        );
    }
    let csv_path = data_dir().join("eval_rows.csv");
    fs::write(&csv_path, csv).unwrap();
    println!("rows -> {}", csv_path.display());
}
