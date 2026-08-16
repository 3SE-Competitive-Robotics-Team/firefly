//! 生成演示地图（3 静态 + 2 动态）为 `FFMap` 标准格式。
//!
//! 用法：`cargo run -p firefly-map --example gen_maps [输出目录]`，
//! 默认输出到 `apps/firefly-demo/maps/`。

use std::path::PathBuf;

use firefly_map::{Motion, Obstacle, Scene, Shape};

fn main() {
    let out = std::env::args()
        .nth(1)
        .map_or_else(|| PathBuf::from("apps/firefly-demo/maps"), PathBuf::from);
    std::fs::create_dir_all(&out).expect("create output dir");

    let maps = [
        ("gate", gate()),
        ("corridor", corridor()),
        ("maze", maze()),
        ("forest_dyn", forest_dyn()),
        ("slalom_dyn", slalom_dyn()),
    ];
    for (name, scene) in maps {
        let file = scene.to_map_file().expect("export map");
        let path = out.join(format!("{name}.ffmap"));
        std::fs::write(&path, file.to_string()).expect("write map file");
        println!(
            "{}: {} 占据体素, {} 动态障碍",
            path.display(),
            file.occupied.len(),
            file.motions.len()
        );
    }
}

/// 统一环境：0.1m 分辨率，28×8×3.2m，起点 (1,4,1)，终点 (27,4,1)。
fn base() -> Scene {
    Scene {
        resolution: 0.1,
        dims: [280, 80, 32],
        start: [1.0, 4.0, 1.0],
        goal: [27.0, 4.0, 1.0],
        ..Scene::default()
    }
}

fn cuboid(cx: f64, cy: f64, cz: f64, sx: f64, sy: f64, sz: f64) -> Obstacle {
    Obstacle::Box {
        center: [cx, cy, cz],
        size: [sx, sy, sz],
    }
}

/// 双柱门 + 穿门航线。
fn gate() -> Scene {
    let mut s = base();
    s.obstacles = vec![
        cuboid(9.0, 2.5, 1.5, 0.8, 0.8, 3.0),
        cuboid(9.0, 5.5, 1.5, 0.8, 0.8, 3.0),
    ];
    s
}

/// S 型交错门洞（中间开 / 两侧开交替）。
fn corridor() -> Scene {
    let mut s = base();
    s.obstacles = vec![
        cuboid(6.0, 1.25, 1.5, 0.6, 2.5, 3.0),
        cuboid(6.0, 6.75, 1.5, 0.6, 2.5, 3.0),
        cuboid(12.0, 4.0, 1.5, 0.6, 3.0, 3.0),
        cuboid(18.0, 1.25, 1.5, 0.6, 2.5, 3.0),
        cuboid(18.0, 6.75, 1.5, 0.6, 2.5, 3.0),
        cuboid(24.0, 4.0, 1.5, 0.6, 3.0, 3.0),
    ];
    s
}

/// 迷宫：错位短墙。
fn maze() -> Scene {
    let mut s = base();
    s.obstacles = vec![
        cuboid(4.0, 1.25, 1.5, 0.6, 2.5, 3.0),
        cuboid(4.0, 6.75, 1.5, 0.6, 2.5, 3.0),
        cuboid(4.0, 4.0, 2.5, 0.6, 3.0, 3.0),
        cuboid(8.0, 4.0, 1.5, 0.6, 4.0, 3.0),
        cuboid(8.0, 1.25, 2.5, 0.6, 2.5, 3.0),
        cuboid(8.0, 6.75, 2.5, 0.6, 2.5, 3.0),
        cuboid(12.0, 1.25, 1.5, 0.6, 2.5, 3.0),
        cuboid(12.0, 6.75, 1.5, 0.6, 2.5, 3.0),
        cuboid(12.0, 4.0, 2.5, 0.6, 3.0, 3.0),
        cuboid(16.0, 4.0, 1.5, 0.6, 4.0, 3.0),
        cuboid(16.0, 1.25, 2.5, 0.6, 2.5, 3.0),
        cuboid(16.0, 6.75, 2.5, 0.6, 2.5, 3.0),
        cuboid(20.0, 1.25, 1.5, 0.6, 2.5, 3.0),
        cuboid(20.0, 6.75, 1.5, 0.6, 2.5, 3.0),
        cuboid(20.0, 4.0, 2.5, 0.6, 3.0, 3.0),
        cuboid(24.0, 4.0, 1.5, 0.6, 4.0, 3.0),
        cuboid(24.0, 1.25, 2.5, 0.6, 2.5, 3.0),
        cuboid(24.0, 6.75, 2.5, 0.6, 2.5, 3.0),
    ];
    s
}

/// 森林 + 两个横向穿行的动态障碍。
fn forest_dyn() -> Scene {
    let mut s = base();
    s.obstacles = vec![
        cuboid(4.0, 4.0, 1.5, 0.8, 0.8, 3.0),
        cuboid(4.0, 6.5, 1.5, 0.8, 0.8, 3.0),
        cuboid(8.0, 2.0, 1.5, 0.8, 0.8, 3.0),
        cuboid(8.0, 5.5, 1.5, 0.8, 0.8, 3.0),
        cuboid(12.0, 4.0, 1.5, 0.8, 0.8, 3.0),
        cuboid(12.0, 6.5, 1.5, 0.8, 0.8, 3.0),
        cuboid(16.0, 2.0, 1.5, 0.8, 0.8, 3.0),
        cuboid(16.0, 5.5, 1.5, 0.8, 0.8, 3.0),
        cuboid(20.0, 4.0, 1.5, 0.8, 0.8, 3.0),
        cuboid(20.0, 6.5, 1.5, 0.8, 0.8, 3.0),
        Obstacle::Sphere {
            center: [24.0, 3.0, 1.2],
            radius: 0.6,
        },
        Obstacle::Sphere {
            center: [24.0, 6.0, 1.2],
            radius: 0.6,
        },
    ];
    s.motions = vec![
        sweep(10.0, 0.5, 14.0, 7.5, 0.0),
        sweep(16.0, 7.5, 14.0, 0.5, 3.5),
        sweep(22.0, 0.5, 14.0, 7.5, 7.0),
        sweep(26.0, 7.5, 14.0, 0.5, 10.5),
    ];
    s
}

/// 交错门 + 动态障碍在门间横穿。
fn slalom_dyn() -> Scene {
    let mut s = base();
    s.obstacles = vec![
        cuboid(6.0, 4.0, 1.5, 0.6, 3.0, 3.0),
        cuboid(12.0, 1.25, 1.5, 0.6, 2.5, 3.0),
        cuboid(12.0, 6.75, 1.5, 0.6, 2.5, 3.0),
        cuboid(18.0, 4.0, 1.5, 0.6, 3.0, 3.0),
        cuboid(24.0, 1.25, 1.5, 0.6, 2.5, 3.0),
        cuboid(24.0, 6.75, 1.5, 0.6, 2.5, 3.0),
    ];
    s.motions = vec![
        sweep(9.0, 0.5, 10.0, 7.5, 0.0),
        sweep(15.0, 7.5, 10.0, 0.5, 2.5),
        sweep(21.0, 0.5, 10.0, 7.5, 5.0),
        sweep(27.0, 7.5, 10.0, 0.5, 7.5),
    ];
    s
}

/// 动态障碍：沿 x 固定、y 往返横扫的盒子（周期 `period` 秒）。
fn sweep(x: f64, y0: f64, period: f64, y1: f64, phase: f64) -> Motion {
    Motion {
        shape: Shape::Box {
            center: [x, 0.0, 1.5],
            size: [0.8, 0.8, 3.0],
        },
        waypoints: vec![
            (phase, [x, y0, 1.5]),
            (phase + period / 2.0, [x, y1, 1.5]),
            (phase + period, [x, y0, 1.5]),
        ],
        loop_back: true,
    }
}
