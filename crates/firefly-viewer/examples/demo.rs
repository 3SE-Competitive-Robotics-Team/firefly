//! 规划过程可视化演示：地图 + 轨迹 + 障碍平面 → rerun viewer。
//!
//! 运行：
//! - 有 viewer：`cargo run -p firefly-viewer --example demo`（自动连接/spawn）
//! - 离线记录：`cargo run -p firefly-viewer --example demo -- --save out.rrd`
//!   （之后用 `rerun out.rrd` 打开）

use firefly_map::{GridMapBuilder, VoxelState};
use firefly_planner::{Planner, PlannerConfig, State};
use firefly_viewer::Viewer;
use nalgebra::{Point3, Vector3};

fn main() -> firefly_error::Result<()> {
    // 场景：x=4.5 处一堵墙
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

    // 有 --save 参数则离线记录，否则连接 viewer
    let save = std::env::args().any(|a| a == "--save");
    let viewer = if save {
        Viewer::save("firefly-demo", "out.rrd")?
    } else {
        Viewer::spawn("firefly-demo")?
    };

    viewer.log_map("world/obstacles", planner.map_ref())?;
    viewer.log_trajectory(
        "planner/trajectory",
        &result.trajectory,
        (80, 160, 255),
        (255, 200, 80),
    )?;
    viewer.log_planes("planner/planes", &result.planes)?;
    viewer.log_path(
        "planner/start_goal",
        &[start.position.coords, goal.coords],
        (80, 200, 120),
    )?;

    // 轨迹概要
    let traj = &result.trajectory;
    log::info!(
        "规划完成：{} 段，{:.1}s，{} 轮 rebound，{} 个平面",
        traj.pieces(),
        traj.duration(),
        result.iterations,
        result.planes.len()
    );
    Ok(())
}
