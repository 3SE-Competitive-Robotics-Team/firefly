//! GICP 轨迹离线评测：先验云 × 轨迹深度帧（10000 点/帧）配准误差率。
//!
//! 数据由 `scripts/gen_gicp_eval_data.py` 生成（`logs/bench/gicp_eval/`，
//! gitignored）：`prior_cloud.bin` + `frames/<traj>_<idx>.bin`。
//! 初值扰动模拟 odom 漂移（0.2m/2°、0.5m/5°、0.05m/0.5° × x/y/z 轴）；
//! 成功判据 terr<0.1m && rerr<2°（融合器修正档口径）。另写
//! `eval_rows.csv`（帧, terr, rerr, 收敛, 内点, 降采样点数, 有效点数）供离线分析。
//!
//! 运行：`cargo test --release -p firefly-localization --test gicp_traj_eval -- --nocapture`
//! 本文件是评测 harness（输出即交付），不是门禁单测：只统计、不设断言。

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::fs;
use std::path::PathBuf;
use std::time::Instant;

use firefly_gicp::points::point_cloud::PointCloud;
use firefly_gicp::points::traits::{PointCloudMut, PointCloudTrait};
use firefly_gicp::util::lie::se3_exp;
use firefly_localization::reloc::{GlobalRelocalizer, RelocOptions};
use firefly_map::DepthCamera;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3, Vector4, Vector6};

const N_POINTS: usize = 10_000;
const MIN_VALID: usize = 2_000;
const SUCCESS_T: f64 = 0.1;
const SUCCESS_R_DEG: f64 = 2.0;

fn env_f64(name: &str, default: f64) -> f64 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn data_dir() -> PathBuf {
    if let Ok(dir) = std::env::var("GICP_EVAL_DIR") {
        return PathBuf::from(dir);
    }
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("logs")
        .join("bench")
        .join("gicp_eval")
}

fn read_le_f32(buf: &[u8], at: usize) -> f32 {
    f32::from_le_bytes(buf[at..at + 4].try_into().unwrap())
}

fn read_le_u64(buf: &[u8], at: usize) -> usize {
    u64::from_le_bytes(buf[at..at + 8].try_into().unwrap()) as usize
}

fn load_prior() -> PointCloud {
    let buf = fs::read(data_dir().join("prior_cloud.bin")).unwrap();
    let num = read_le_u64(&buf, 0);
    let mut cloud = PointCloud::new();
    cloud.resize(num);
    for i in 0..num {
        let off = 8 + 12 * i;
        cloud.set_point(
            i,
            Vector4::new(
                f64::from(read_le_f32(&buf, off)),
                f64::from(read_le_f32(&buf, off + 4)),
                f64::from(read_le_f32(&buf, off + 8)),
                1.0,
            ),
        );
    }
    cloud
}

fn stride_sample(full: &PointCloud, num: usize) -> PointCloud {
    let total = full.num_points();
    let mut out = PointCloud::new();
    if total == 0 {
        return out;
    }
    let take = num.min(total);
    let step = total as f64 / take as f64;
    out.resize(take);
    for i in 0..take {
        let src = ((i as f64) * step).floor() as usize;
        out.set_point(i, full.point(src.min(total - 1)));
    }
    out
}

fn rot_err_deg(est: &nalgebra::Matrix4<f64>, gt: &nalgebra::Matrix4<f64>) -> f64 {
    let rot = est.fixed_view::<3, 3>(0, 0).into_owned().transpose()
        * gt.fixed_view::<3, 3>(0, 0).into_owned();
    ((rot.trace() - 1.0) / 2.0)
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees()
}

fn trans_err(est: &nalgebra::Matrix4<f64>, gt: &nalgebra::Matrix4<f64>) -> f64 {
    let col_est = est.fixed_view::<3, 1>(0, 3);
    let col_gt = gt.fixed_view::<3, 1>(0, 3);
    (col_est - col_gt).norm()
}

/// 单帧：深度转点云 → 抽 10k → 6 组扰动配准 → 行记录。
#[allow(clippy::too_many_lines)]
fn eval_frame(
    reloc: &GlobalRelocalizer,
    cam: &DepthCamera,
    path: &PathBuf,
    rows: &mut Vec<(String, f64, f64, bool, usize, usize, usize)>,
    tally_traj: &mut BTreeMap<String, (usize, usize)>,
    tally_level: &mut BTreeMap<String, (usize, usize)>,
    ms_total: &mut u128,
) -> (usize, usize, usize) {
    let stem = path.file_stem().unwrap().to_string_lossy().to_string();
    let traj = stem
        .rsplit_once('_')
        .map(|(name, _)| name.to_string())
        .unwrap_or_default();
    let buf = fs::read(path).unwrap();
    let width = read_le_u64(&buf, 0);
    let height = read_le_u64(&buf, 8);
    let depth: Vec<f32> = (0..width * height)
        .map(|i| read_le_f32(&buf, 16 + 4 * i))
        .collect();
    let pos_off = 16 + 4 * width * height;
    let pos = Vector3::new(
        f64::from_le_bytes(buf[pos_off..pos_off + 8].try_into().unwrap()),
        f64::from_le_bytes(buf[pos_off + 8..pos_off + 16].try_into().unwrap()),
        f64::from_le_bytes(buf[pos_off + 16..pos_off + 24].try_into().unwrap()),
    );

    let identity =
        Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), UnitQuaternion::identity());
    let full = GlobalRelocalizer::depth_to_cloud(&depth, cam, &identity);
    let n_valid = full.num_points();
    if n_valid < MIN_VALID {
        return (0, 0, 1);
    }
    let source = stride_sample(&full, N_POINTS);
    let t_gt =
        Isometry3::from_parts(Translation3::from(pos), UnitQuaternion::identity()).to_homogeneous();

    // 初值扰动档 × x/y/z 轴（body 系，odom 漂移形态）
    let levels = [
        (0.2, 2.0_f64.to_radians()),
        (0.5, 5.0_f64.to_radians()),
        (0.05, 0.5_f64.to_radians()),
    ];
    let axes = [Vector3::x(), Vector3::y(), Vector3::z()];
    let (mut ok_count, mut all_count) = (0, 0);
    for (level_idx, (disturb_t, disturb_r)) in levels.into_iter().enumerate() {
        for (axis_idx, axis) in axes.into_iter().enumerate() {
            let mut delta = Vector6::zeros();
            delta.fixed_rows_mut::<3>(0).copy_from(&(axis * disturb_r));
            delta.fixed_rows_mut::<3>(3).copy_from(&(axis * disturb_t));
            let init = t_gt * se3_exp(&delta);
            let tick = Instant::now();
            let res = reloc.relocalize(&source, &init);
            *ms_total += tick.elapsed().as_millis();
            let terr = trans_err(&res.result.t_target_source, &t_gt);
            let rerr = rot_err_deg(&res.result.t_target_source, &t_gt);
            let ok = terr < SUCCESS_T && rerr < SUCCESS_R_DEG;
            all_count += 1;
            if ok {
                ok_count += 1;
            }
            rows.push((
                format!("{stem} L{level_idx}A{axis_idx}"),
                terr,
                rerr,
                res.result.converged,
                res.result.num_inliers,
                res.total_points,
                n_valid,
            ));
            for key in [traj.clone(), format!("L{level_idx}")] {
                let tally = if key == traj {
                    &mut *tally_traj
                } else {
                    &mut *tally_level
                };
                let entry = tally.entry(key).or_insert((0, 0));
                entry.1 += 1;
                if ok {
                    entry.0 += 1;
                }
            }
        }
    }
    (ok_count, all_count, 0)
}

struct Report<'a> {
    rows: &'a [(String, f64, f64, bool, usize, usize, usize)],
    tally_traj: &'a BTreeMap<String, (usize, usize)>,
    tally_level: &'a BTreeMap<String, (usize, usize)>,
    n_ok: usize,
    n_all: usize,
    n_skip: usize,
    n_frames: usize,
    ms_total: u128,
}

fn print_report(rep: &Report) {
    let n_all_max = rep.n_all.max(1);
    println!(
        "=== frames={} skipped(low-valid)={} regs={} ok={} rate={:.1}% avg={:.0}ms/reg ===",
        rep.n_frames,
        rep.n_skip,
        rep.n_all,
        rep.n_ok,
        100.0 * rep.n_ok as f64 / n_all_max as f64,
        rep.ms_total as f64 / n_all_max as f64
    );
    println!("--- per trajectory (ok/total) ---");
    for (name, (ok, all)) in rep.tally_traj {
        println!(
            "  {name:<18} {ok:>4}/{all:<4} {:5.1}%",
            100.0 * *ok as f64 / *all as f64
        );
    }
    println!("--- per level ---");
    for (level, (ok, all)) in rep.tally_level {
        println!(
            "  {level:<4} {ok:>4}/{all:<4} {:5.1}%",
            100.0 * *ok as f64 / *all as f64
        );
    }
    let mut fails: Vec<_> = rep
        .rows
        .iter()
        .filter(|(_, terr, rerr, _, _, _, _)| *terr >= SUCCESS_T || *rerr >= SUCCESS_R_DEG)
        .collect();
    fails.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    println!("--- worst 15 failures (frame terr rerr valid residual) ---");
    for (name, terr, rerr, _, _, _, n_valid) in fails.iter().take(15) {
        println!("  {name:<24} {terr:.3}m {rerr:.2}deg valid={n_valid}");
    }
    let mut csv = String::new();
    for (name, terr, rerr, converged, inliers, total, n_valid) in rep.rows {
        let _ = writeln!(
            csv,
            "{name},{terr:.4},{rerr:.3},{converged},{inliers},{total},{n_valid}"
        );
    }
    let csv_path = data_dir().join("eval_rows.csv");
    fs::write(&csv_path, csv).unwrap();
    println!("rows -> {}", csv_path.display());
}

#[test]
fn gicp_traj_eval() {
    let opts = RelocOptions {
        downsampling_resolution: 0.2,
        num_neighbors: 10,
        max_correspondence_distance: env_f64("GICP_MAX_CORR", 2.0),
        max_iterations: 20,
        voxel_resolution: 1.0,
    };
    let reloc = GlobalRelocalizer::from_cloud(load_prior(), opts);
    println!("target points: {}", reloc.target().num_points());

    let mut cam = DepthCamera::mujoco_default();
    cam.pixel_step = 1;
    cam.max_range = env_f64("GICP_MAX_RANGE", cam.max_range);

    let frames_dir = data_dir().join("frames");
    let mut frames: Vec<PathBuf> = fs::read_dir(&frames_dir)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .collect();
    frames.sort();
    println!("frames: {}", frames.len());

    let mut rows = Vec::new();
    let mut tally_traj = BTreeMap::new();
    let mut tally_level = BTreeMap::new();
    let (mut n_ok, mut n_all, mut n_skip, mut ms_total) = (0usize, 0usize, 0usize, 0u128);
    let clock = Instant::now();
    for (idx, path) in frames.iter().enumerate() {
        let (ok_count, all_count, skip) = eval_frame(
            &reloc,
            &cam,
            path,
            &mut rows,
            &mut tally_traj,
            &mut tally_level,
            &mut ms_total,
        );
        n_ok += ok_count;
        n_all += all_count;
        n_skip += skip;
        if idx % 30 == 0 {
            println!(
                "... {idx}/{} ({:.0}s)",
                frames.len(),
                clock.elapsed().as_secs_f64()
            );
        }
    }
    print_report(&Report {
        rows: &rows,
        tally_traj: &tally_traj,
        tally_level: &tally_level,
        n_ok,
        n_all,
        n_skip,
        n_frames: frames.len(),
        ms_total,
    });
}
