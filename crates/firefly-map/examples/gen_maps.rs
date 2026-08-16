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

/// 草丛：低矮球形灌木（仅装饰，不参与规划）。
fn bush(cx: f64, cy: f64) -> Obstacle {
    Obstacle::Sphere {
        center: [cx, cy, 0.25],
        radius: 0.25,
    }
}

/// 树：细树干 + 球形树冠（树冠下压至飞行层 z≈1.0，体素化后呈树形）。
fn tree(cx: f64, cy: f64) -> [Obstacle; 2] {
    [
        Obstacle::Box {
            center: [cx, cy, 0.75],
            size: [0.3, 0.3, 1.5],
        },
        Obstacle::Sphere {
            center: [cx, cy, 1.3],
            radius: 0.55,
        },
    ]
}

/// 森林：连续墙 + 随机树 + 10 个靠中间往返的动态障碍。
fn forest_dyn() -> Scene {
    // 连续墙：地面到顶全挡（厚 0.3m，高 3m）。
    // 中间墙挡 y=4 直线（缝两侧），两侧墙挡上下（缝中间），交替成 S 型。
    const WALLS: &[[f64; 3]] = &[
        // [x, 墙段中心, 墙段长]
        [4.0, 4.0, 3.0],   // 挡中间：缝 y<2.5 或 y>5.5
        [9.0, 1.25, 2.5],  // 挡下侧：缝 y>2.5
        [9.0, 6.75, 2.5],  // 挡上侧：缝 y<5.5
        [14.0, 4.0, 3.0],  // 挡中间
        [19.0, 1.25, 2.5], // 挡下侧
        [19.0, 6.75, 2.5], // 挡上侧
        [24.0, 4.0, 3.0],  // 挡中间
    ];
    // 地面草丛（装饰层）
    const BUSHES: &[[f64; 2]] = &[
        [2.0, 3.0],
        [4.0, 2.0],
        [4.5, 5.5],
        [6.5, 5.0],
        [9.0, 3.5],
        [9.5, 6.5],
        [12.0, 2.5],
        [13.0, 4.5],
        [14.0, 6.0],
        [16.5, 3.5],
        [17.5, 5.5],
        [19.0, 5.0],
        [21.5, 2.5],
        [24.0, 4.5],
    ];
    let mut s = base();
    let mut obstacles = Vec::new();
    for [x, yc, len] in WALLS {
        obstacles.push(Obstacle::Box {
            center: [*x, *yc, 1.5],
            size: [0.3, *len, 3.0],
        });
    }
    // 树：固定 seed 伪随机散布（避开墙位），树冠间留缝
    let mut seed = 0x5eed_u64;
    let mut placed = 0;
    let mut guard = 0;
    while placed < 12 && guard < 400 {
        guard += 1;
        let x = 5.5 + lcg(&mut seed) * 20.0;
        let y = 0.8 + lcg(&mut seed) * 6.4;
        if WALLS.iter().any(|[wx, ..]| (x - wx).abs() < 1.2) {
            continue;
        }
        if obstacles
            .iter()
            .filter(|o| matches!(o, Obstacle::Sphere { .. }))
            .any(|o| {
                let Obstacle::Sphere { center, .. } = o else {
                    unreachable!()
                };
                (x - center[0]).hypot(y - center[1]) < 1.2
            })
        {
            continue;
        }
        obstacles.extend(tree(x, y));
        placed += 1;
    }
    s.obstacles = obstacles;
    s.decor = BUSHES.iter().map(|[x, y]| bush(*x, *y)).collect();
    // 10 个动态障碍：x 均匀分布，y 3.0~5.0 短往返（周期 5s，相位错开）
    s.motions = (0..10)
        .map(|i| {
            let x = 3.5 + f64::from(i) * 2.5;
            sweep(x, 3.0, 5.0, 5.0, f64::from(i) * 0.5)
        })
        .collect();
    s
}

/// 固定 seed 线性同余伪随机（0..1）。
fn lcg(state: &mut u64) -> f64 {
    *state = state
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1_442_695_040_888_963_407);
    ((*state >> 33) as f64) / (u64::MAX >> 33) as f64
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
            center: [x, 0.0, 0.9],
            size: [0.5, 0.5, 1.8],
        },
        waypoints: vec![
            (phase, [x, y0, 1.5]),
            (phase + period / 2.0, [x, y1, 1.5]),
            (phase + period, [x, y0, 1.5]),
        ],
        loop_back: true,
    }
}
