//! 单帧管线耗时压测：不同点数下 `process_frame` 的 wall time（性能回归 guard）。
//! 合成数据：3m 正对地面 + 双侧墙，深度图分辨率同 sim（320×240）。

use std::time::Instant;

use firefly_void::options::VoidOptions;
use firefly_void::{FrameInput, Odometry, VoidOdometry};
use firefly_void_types::sensor::{CameraFrame, DepthFrame, ImuSample};
use nalgebra::Vector3;

fn synth_depth(h: usize, w: usize, z0: f64) -> Vec<f64> {
    // 平面 z=z0（相机系），边缘 10% 置无效（模拟空洞/出界）
    let mut d = vec![0.0f64; h * w];
    let fx = 168.6;
    for y in 0..h {
        for x in 0..w {
            let (dx, dy) = ((x as f64 - 160.0) / fx, (y as f64 - 120.0) / fx);
            let edge = (x < w / 10) || (x >= w * 9 / 10) || (y < h / 10) || (y >= h * 9 / 10);
            if !edge {
                d[y * w + x] = z0 * (1.0 + dx * dx + dy * dy).sqrt();
            }
        }
    }
    d
}

#[test]
fn per_frame_latency_budget() {
    let opts = VoidOptions::default();
    let mut odo = VoidOdometry::new(opts);

    // 初始化（一条 IMU 让时间轴就位）
    odo.process_imu(&ImuSample {
        t: 0.0,
        omega: Vector3::zeros(),
        acc: Vector3::new(0.0, 0.0, 9.81),
    });

    let (h, w) = (240usize, 320usize);
    let depth = synth_depth(h, w, 3.0);
    let gray = vec![128u8; h * w];

    let camera = CameraFrame {
        t: 0.0,
        left_gray: &gray,
        width: w,
        height: h,
    };
    let depth_frame = DepthFrame {
        t: 0.0,
        depth: &depth,
        width: w,
        height: h,
    };
    let make_frame = || FrameInput {
        camera: &camera,
        depth: &depth_frame,
    };

    // 预热一帧（建图）
    odo.process_frame(&make_frame()).unwrap();

    // 计时 10 帧
    let t0 = Instant::now();
    for k in 1..=10usize {
        odo.process_imu(&ImuSample {
            t: 0.1 + k as f64 * 0.01,
            omega: Vector3::zeros(),
            acc: Vector3::new(0.0, 0.0, 9.81),
        });
        odo.process_frame(&make_frame()).unwrap();
    }
    let per_frame = t0.elapsed().as_secs_f64() / 10.0;
    // 实测基线（M 系列，release）：3000 点全量 ESIKF ~4.9s/帧——主循环
    // 已卡 sim_rate 0.79x 的直接原因；记录现状，优化迭代后逐步收紧
    assert!(
        per_frame < 5.5,
        "单帧 {:.1}ms 超基线（5.5s）",
        per_frame * 1000.0
    );
    println!(
        "per-frame: {:.1} ms（含深度+视觉+建图）",
        per_frame * 1000.0
    );
}
