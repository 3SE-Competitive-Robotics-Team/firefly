//! 端到端演示：规划 + 可观测性（fastrace trace + logforth 日志）。
//! 运行：`RUST_LOG=info cargo run -p firefly-planner --example demo`

use fastrace::collector::SpanContext;
use fastrace::prelude::*;
use firefly_map::{GridMapBuilder, VoxelState};
use firefly_planner::{Planner, PlannerConfig, State};
use nalgebra::{Point3, Vector3};

#[fastrace::trace]
fn run_plan() -> firefly_error::Result<()> {
    let mut map = GridMapBuilder::new(0.5, [20, 20, 20]).build()?;
    for y in 0..20 {
        for z in 0..3 {
            map.set_state([9, y, z], VoxelState::Occupied);
        }
    }
    let mut planner = Planner::new(PlannerConfig::default(), map);
    let start = State {
        position: Point3::new(0.5, 0.5, 0.5),
        velocity: Vector3::zeros(),
        acceleration: Vector3::zeros(),
    };
    let goal = Point3::new(9.0, 0.5, 0.5);

    let result = planner.plan(start, goal)?;
    let traj = result.trajectory;
    log::info!(
        "规划成功：{} 段，时长 {:.1}s，{} 轮 rebound",
        traj.pieces(),
        traj.duration(),
        result.iterations
    );

    for k in [0, 100, 200] {
        let t = traj.duration() * f64::from(k) / 200.0;
        let s = traj.eval(t);
        log::info!(
            "t={t:5.2}s  p=({:6.2},{:6.2},{:6.2})  v={:.2} m/s",
            s.position.x,
            s.position.y,
            s.position.z,
            s.velocity.norm()
        );
    }
    Ok(())
}

fn main() {
    firefly_observability::init();
    // 应用层创建 root span（短任务粒度，root drop 时上报整条 trace）
    {
        let root = Span::root(func_path!(), SpanContext::random());
        let _guard = root.set_local_parent();
        match run_plan() {
            Ok(()) => log::info!("demo 完成"),
            Err(e) => log::error!("demo 失败：{e}"),
        }
    }
    fastrace::flush();
}
