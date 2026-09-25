//! 逐像素处理工作线程：灰度化 + 深度线性化/噪声 + 调试显示映射。
//!
//! 这些计算与 IPC 发布**解耦**：发布端口是主线程独占的（iceoryx2 端口非
//! `Send`），但像素数学不是——放到独立线程做，主线程只做「发出去」这一步，
//! 不再被逐像素循环占住（对照 `link` 的流水：主世界装配 → 本线程算 →
//! 主世界发布）。
//!
//! 口径对照 `packages/firefly-mujoco/src/firefly_mujoco/env.py`（`DroneEnv`）：
//! 灰度用 BT.601 加权；深度噪声三步（视差域高斯、边缘膨胀 1px、随机丢点），
//! 仅作用于有效命中（`0.05 < z < 100` 米），其余保持无效标记 `0.0`。

use std::sync::mpsc::{Receiver, SyncSender};

use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH};
use firefly_pubsub::trace::TraceContext;
use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::rig::{FOV_Y_DEG, SENSOR_NEAR};
use crate::sensors;

/// 一拍的原始输入（主世界装配后交给工作线程）。
#[derive(Debug)]
pub struct CaptureJob {
    /// 单调序号。
    pub seq: u64,
    /// 位姿时间戳（秒，sim 时间）。
    pub stamp: f64,
    /// 上游 trace 上下文（发布时续接）。
    pub trace: TraceContext,
    /// 左目紧凑 `RGBA8`（行主序无填充）。
    pub left_rgb: Vec<u8>,
    /// 右目紧凑 `RGBA8`。
    pub right_rgb: Vec<u8>,
    /// 深度预通道原始字节（小端 `f32`）。
    pub depth_raw: Vec<u8>,
}

/// 一拍的全部输出（工作线程算好后交回主世界发布/显示）。
#[derive(Debug)]
pub struct ProcessedCapture {
    /// 单调序号。
    pub seq: u64,
    /// 位姿时间戳（秒）。
    pub stamp: f64,
    /// 上游 trace 上下文。
    pub trace: TraceContext,
    /// 左目 RGB（直显用）。
    pub left_rgb: Vec<u8>,
    /// 右目 RGB（直显用）。
    pub right_rgb: Vec<u8>,
    /// 左目灰度（发布）。
    pub left_gray: Vec<u8>,
    /// 右目灰度（发布）。
    pub right_gray: Vec<u8>,
    /// 深度（米，行主序，已加噪声；发布）。
    pub depth: Vec<f32>,
    /// 左目灰度显示图（`RGBA8`）。
    pub left_gray_rgba: Vec<u8>,
    /// 右目灰度显示图（`RGBA8`）。
    pub right_gray_rgba: Vec<u8>,
    /// 深度伪彩显示图（`RGBA8`）。
    pub depth_rgba: Vec<u8>,
}

/// 工作线程主循环：阻塞收任务，算完阻塞回结果；通道关闭即退出。
pub fn run_worker(jobs: &Receiver<CaptureJob>, results: &SyncSender<ProcessedCapture>) {
    let mut rng = StdRng::from_rng(&mut rand::rng());
    while let Ok(job) = jobs.recv() {
        let out = process(job, &mut rng);
        if results.send(out).is_err() {
            break;
        }
    }
}

/// 单拍像素管线（对照 `firefly_mujoco.env.DroneEnv` 的灰度/深度口径）。
fn process(job: CaptureJob, rng: &mut StdRng) -> ProcessedCapture {
    let mut left_gray = vec![0u8; IMAGE_SIZE];
    let mut right_gray = vec![0u8; IMAGE_SIZE];
    sensors::rgb_to_gray(&job.left_rgb, &mut left_gray);
    sensors::rgb_to_gray(&job.right_rgb, &mut right_gray);

    let mut depth = vec![0.0f32; IMAGE_SIZE];
    sensors::linearize_depth(&job.depth_raw, &mut depth, SENSOR_NEAR);
    sensors::add_depth_noise(
        &mut depth,
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
        FOV_Y_DEG.to_radians(),
        rng,
    );

    let mut left_gray_rgba = vec![0u8; 4 * IMAGE_SIZE];
    sensors::gray_to_display(&left_gray, &mut left_gray_rgba);
    let mut right_gray_rgba = vec![0u8; 4 * IMAGE_SIZE];
    sensors::gray_to_display(&right_gray, &mut right_gray_rgba);
    let mut depth_rgba = vec![0u8; 4 * IMAGE_SIZE];
    sensors::depth_to_display(&depth, &mut depth_rgba, sensors::DISPLAY_DEPTH_RANGE);

    ProcessedCapture {
        seq: job.seq,
        stamp: job.stamp,
        trace: job.trace,
        left_rgb: job.left_rgb,
        right_rgb: job.right_rgb,
        left_gray,
        right_gray,
        depth,
        left_gray_rgba,
        right_gray_rgba,
        depth_rgba,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 单拍管线产出各缓冲尺寸正确；全清空深度保持无效 `0.0`（显示全黑）。
    #[test]
    fn process_produces_expected_buffers() {
        let mut rng = StdRng::seed_from_u64(7);
        let job = CaptureJob {
            seq: 3,
            stamp: 1.5,
            trace: TraceContext::empty(),
            left_rgb: vec![128u8; 4 * IMAGE_SIZE],
            right_rgb: vec![64u8; 4 * IMAGE_SIZE],
            depth_raw: vec![0u8; 4 * IMAGE_SIZE],
        };
        let out = process(job, &mut rng);
        assert_eq!(out.seq, 3);
        assert!((out.stamp - 1.5).abs() < 1e-9);
        assert_eq!(out.left_gray.len(), IMAGE_SIZE);
        assert_eq!(out.right_gray.len(), IMAGE_SIZE);
        assert_eq!(out.depth.len(), IMAGE_SIZE);
        assert_eq!(out.left_gray_rgba.len(), 4 * IMAGE_SIZE);
        assert_eq!(out.right_gray_rgba.len(), 4 * IMAGE_SIZE);
        assert_eq!(out.depth_rgba.len(), 4 * IMAGE_SIZE);
        // 原始深度全为清空值 → 全无效，显示为黑。
        assert!(out.depth.iter().all(|z| *z == 0.0));
        assert!(
            out.depth_rgba
                .as_chunks::<4>()
                .0
                .iter()
                .all(|px| px[0] == 0)
        );
    }
}
