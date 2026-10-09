//! 体素可视化（渲染调试窗口）：静态场地 + 实时感知。
//!
//! - **静态场地**：本地读 `.ffmap`（复用 `firefly-map` 的解析器；场地占据与
//!   位姿估计无关），启动时建一次网格。
//! - **实时感知**：订阅 `Firefly/Viz` 的 `kind = VOXELS` 消息——planner 发布的
//!   就是它正在规划的那张图（`plan/perceived`），用的是**估计位姿**，所以定位
//!   一漂，视图里障碍的位置就会跟着错，能直接看出来。
//!
//! 体素约定与 `apps/planner::log_map`、`FFMap` 文档一致：
//! `world = voxel_origin + (idx + 0.5) * voxel_size`。
//!
//! 观感对照宇树 `RViz` 配置（`unitree_slam`/`point_lio_unilidar`）：**小方块 +
//! 整层纯色不透明 + 深色背景**，靠方块间的缝看清体素粒度；不画自由/未知空间。
//! 静态先验只画障碍（平铺的地板层按占据格数自动识别后剔除）。
//!
//! 性能：每层一个**合并网格**（每体素 6 面，单 draw call）；实时层按**增量**
//! 协议累计（新增并入、删除移出），只在集合变化且距上次重建 ≥
//! [`REBUILD_INTERVAL`] 时按原句柄原地更新，不每帧重建。

use std::collections::{HashMap, HashSet};

use bevy::asset::RenderAssetUsages;
use bevy::camera::visibility::RenderLayers;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use firefly_map::{MapFile, VoxelState};
use firefly_pubsub::subscriber::Subscriber;
use firefly_pubsub::viz::{VIZ_TOPIC, VizMessage, kind};

use crate::config::VoxelsConfig;
use crate::link::IpcPorts;

/// 实时层最小重建间隔（秒）：话题频率高于此，按此节流。
/// 实时层重建间隔（秒，10Hz）：增量消息按拍到达，重建比这更快没有意义。
const REBUILD_INTERVAL: f64 = 0.1;
/// 方块边长占格子的比例：1.0 = 满格，占据体积与实际一致（判断净空必须如此）。
/// 缩小方块能让粒度更好看，但会让人以为占据体积比实际小，所以缩放在这里恒为 1，
/// 粒度改由每格明度抖动体现。
const VOXEL_FILL: f32 = 1.0;
/// 静态层边长占比：比感知层小一点点，两层重叠格不会共面闪烁（相差 3mm，
/// 净空判读不受影响）。
const FIELD_FILL: f32 = 0.98;
/// 每格明度抖动的最大幅度（±10%）：满格方块贴在一起时靠这点差异看出体素粒度。
const VOXEL_SHADE_JITTER: f32 = 0.10;

/// 体素→逐格明度系数（确定性哈希，同一格每次渲染相同）。
fn cube_shade(idx: [i32; 3]) -> [f32; 4] {
    let h = (idx[0].wrapping_mul(73_856_093)
        ^ idx[1].wrapping_mul(19_349_663)
        ^ idx[2].wrapping_mul(83_492_791)) as u32;
    let f = 1.0 - VOXEL_SHADE_JITTER + (h % 21) as f32 * VOXEL_SHADE_JITTER / 10.0;
    [f, f, f, 1.0]
}
/// 静态场地（先验障碍）颜色：冷灰纯色。
const FIELD_COLOR: Color = Color::srgb(0.34, 0.37, 0.40);
/// 实时感知颜色：青绿纯色（青位偏离路径的亮绿，避免混淆）。
const LIVE_COLOR: Color = Color::srgb(0.05, 0.85, 0.80);

/// 立方体一个面：邻居索引偏移、四角（相对体素中心，单位为体素尺寸）、法线。
type Face = ([i32; 3], [[f32; 3]; 4], [f32; 3]);

/// 静态场地：窗口索引、网格世界原点、体素尺寸。
type FieldVoxels = (Vec<[i32; 3]>, [f32; 3], [f32; 3]);

/// 立方体六个面。
const FACES: [Face; 6] = [
    (
        [1, 0, 0],
        [
            [0.5, -0.5, -0.5],
            [0.5, 0.5, -0.5],
            [0.5, 0.5, 0.5],
            [0.5, -0.5, 0.5],
        ],
        [1.0, 0.0, 0.0],
    ),
    (
        [-1, 0, 0],
        [
            [-0.5, -0.5, 0.5],
            [-0.5, 0.5, 0.5],
            [-0.5, 0.5, -0.5],
            [-0.5, -0.5, -0.5],
        ],
        [-1.0, 0.0, 0.0],
    ),
    (
        [0, 1, 0],
        [
            [0.5, 0.5, -0.5],
            [-0.5, 0.5, -0.5],
            [-0.5, 0.5, 0.5],
            [0.5, 0.5, 0.5],
        ],
        [0.0, 1.0, 0.0],
    ),
    (
        [0, -1, 0],
        [
            [0.5, -0.5, 0.5],
            [-0.5, -0.5, 0.5],
            [-0.5, -0.5, -0.5],
            [0.5, -0.5, -0.5],
        ],
        [0.0, -1.0, 0.0],
    ),
    (
        [0, 0, 1],
        [
            [-0.5, -0.5, 0.5],
            [-0.5, 0.5, 0.5],
            [0.5, 0.5, 0.5],
            [0.5, -0.5, 0.5],
        ],
        [0.0, 0.0, 1.0],
    ),
    (
        [0, 0, -1],
        [
            [0.5, -0.5, -0.5],
            [0.5, 0.5, -0.5],
            [-0.5, 0.5, -0.5],
            [-0.5, -0.5, -0.5],
        ],
        [0.0, 0.0, -1.0],
    ),
];

/// 体素显示层标记。
#[derive(Component)]
pub struct VoxelLayer;

/// 已收到的规划路径（实体名 → 点集 + 颜色），用折线 gizmo 画。
#[derive(Resource, Default)]
pub struct Paths {
    lines: HashMap<String, (Vec<Vec3>, Color)>,
}

/// 折线 gizmo 的渲染层：**必须**限到专用路径层，否则默认 layer 0 会被传感器
/// 相机一并画进发布给 VIO/深度的图像里。
#[allow(clippy::needless_pass_by_value)] // `SystemParam` 契约，与同 crate 其它系统一致
pub fn setup_gizmo_layers(mut store: ResMut<GizmoConfigStore>) {
    let (config, _) = store.config_mut::<DefaultGizmoConfigGroup>();
    config.render_layers = RenderLayers::layer(crate::rig::PATH_LAYER);
    // 规划路径要盖在地形/体素之上看得见：默认 2px 且 `depth_bias = 0`，会被
    // 地面挡住或在体素缝里闪烁。
    config.line.width = 4.0;
    config.depth_bias = -0.01;
}

/// 画规划路径（`plan/global_path`、`plan/local_traj`）。
#[allow(clippy::needless_pass_by_value)] // `SystemParam` 契约，与同 crate 其它系统一致
pub fn draw_paths(paths: Res<Paths>, mut gizmos: Gizmos) {
    for (points, color) in paths.lines.values() {
        if points.len() >= 2 {
            gizmos.linestrip(points.iter().copied(), *color);
        }
    }
}

/// 实时层指纹与节流状态。
#[derive(Resource, Default)]
pub struct LiveVoxelAccum {
    /// 当前应显示的体素集合（增量协议累计：全量替换 / 并入 / 移除）。
    set: HashSet<[i32; 3]>,
    /// 体素尺寸（米）与网格原点（世界坐标，米）。
    size: [f32; 3],
    origin: [f32; 3],
    /// 集合自上次重建以来是否变化。
    dirty: bool,
    /// 是否已重建过（首帧日志用）。
    built: bool,
    /// 上次重建时刻（`Time::elapsed_secs_f64`）。
    last_rebuild: f64,
}

/// 体素网格句柄（材质由 Bevy 复用；静态层建好后不再改动，只留实时层句柄）。
#[derive(Resource)]
pub struct VoxelMeshes {
    live: Handle<Mesh>,
}

/// 由体素索引构建**方块**合并网格（每个体素 6 面，不做内部面剔除）。
///
/// `fill` 是边长占格子的比例（[`VOXEL_FILL`] = 1 为满格）；每格带一点明度抖动，
/// 满格相邻时也能看出体素粒度。
#[must_use]
fn cube_mesh(indices: &[[i32; 3]], origin: [f32; 3], size: [f32; 3], fill: f32) -> Mesh {
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(indices.len() * 24);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(indices.len() * 24);
    let mut colors: Vec<[f32; 4]> = Vec::with_capacity(indices.len() * 24);
    let mut triangles: Vec<u32> = Vec::with_capacity(indices.len() * 36);
    for idx in indices {
        let center = [
            origin[0] + (idx[0] as f32 + 0.5) * size[0],
            origin[1] + (idx[1] as f32 + 0.5) * size[1],
            origin[2] + (idx[2] as f32 + 0.5) * size[2],
        ];
        let shade = cube_shade(*idx);
        for (_, corners, normal) in FACES {
            let base = positions.len() as u32;
            for corner in corners {
                positions.push([
                    center[0] + corner[0] * size[0] * fill,
                    center[1] + corner[1] * size[1] * fill,
                    center[2] + corner[2] * size[2] * fill,
                ]);
                normals.push(normal);
                colors.push(shade);
            }
            triangles.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
        }
    }
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    mesh.insert_indices(Indices::U32(triangles));
    mesh
}

/// 静态场地占据体素：`(窗口索引, 世界原点, 体素尺寸)`；载入失败返回 `None`。
fn load_field(path: &str) -> Option<FieldVoxels> {
    let file = match MapFile::from_file(path) {
        Ok(file) => file,
        Err(e) => {
            log::warn!("体素视图：读取 {path} 失败，静态场地不显示：{e}");
            return None;
        }
    };
    let grid = match file.to_grid_map() {
        Ok(grid) => grid,
        Err(e) => {
            log::warn!("体素视图：{path} 转占据栅格失败，静态场地不显示：{e}");
            return None;
        }
    };
    let dims = grid.dims();
    let mut per_z = vec![0usize; dims[2]];
    let mut occupied: Vec<[i32; 3]> = Vec::new();
    for x in 0..dims[0] {
        for y in 0..dims[1] {
            for (z, count) in per_z.iter_mut().enumerate() {
                if grid.state([x, y, z]) == VoxelState::Occupied {
                    *count += 1;
                    occupied.push([x as i32, y as i32, z as i32]);
                }
            }
        }
    }
    // 地板层识别：一整层几乎铺满 x-y 面积的 z 层就是平铺地板（场地实测 63–78%，
    // 下一层只有 30%，余量一倍以上）。不剔的话地板铺满整个视图、把感知层压住。
    let area = dims[0] * dims[1];
    let total: usize = per_z.iter().sum();
    let ground: Vec<usize> = (0..dims[2]).filter(|&z| per_z[z] * 2 > area).collect();
    let indices: Vec<[i32; 3]> = occupied
        .into_iter()
        .filter(|idx| !ground.contains(&(idx[2] as usize)))
        .collect();
    let origin = *grid.origin();
    let resolution = grid.resolution() as f32;
    log::info!(
        "体素视图：静态场地 {} 格（{:.2}m 分辨率）；剔除地板层 {ground:?}（原 {} 格）",
        indices.len(),
        resolution,
        total
    );
    Some((
        indices,
        [origin.x as f32, origin.y as f32, origin.z as f32],
        [resolution; 3],
    ))
}

/// 启动时建静态场地网格 + 空的实时网格，并订阅 `Firefly/Viz`。
#[allow(clippy::needless_pass_by_value)] // `SystemParam` 契约，与同 crate 其它系统一致
pub fn setup_voxels(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    config: Res<crate::config::RenderConfig>,
) {
    let VoxelsConfig { field_map } = &config.voxels;
    let field_indices = load_field(field_map);
    let field_mesh = match field_indices {
        Some((indices, origin, size)) => meshes.add(cube_mesh(&indices, origin, size, FIELD_FILL)),
        None => meshes.add(cube_mesh(&[], [0.0; 3], [1.0; 3], FIELD_FILL)),
    };
    // 初始给一个退化方块而不是空 mesh：空 mesh 会触发 Bevy `slab_allocator`
    // 的 use-after-free 报错（实测）。
    let live_mesh = meshes.add(cube_mesh(&[[0, 0, 0]], [0.0; 3], [f32::EPSILON; 3], 1.0));
    let field_material = materials.add(StandardMaterial {
        base_color: FIELD_COLOR,
        unlit: true,
        cull_mode: None,
        alpha_mode: AlphaMode::Opaque,
        ..default()
    });
    let live_material = materials.add(StandardMaterial {
        base_color: LIVE_COLOR,
        unlit: true,
        cull_mode: None,
        alpha_mode: AlphaMode::Opaque,
        ..default()
    });
    for (mesh, material) in [
        (field_mesh.clone(), field_material),
        (live_mesh.clone(), live_material),
    ] {
        commands.spawn((
            Mesh3d(mesh),
            MeshMaterial3d(material),
            Transform::default(),
            Visibility::default(),
            RenderLayers::layer(crate::rig::VOXEL_LAYER),
            VoxelLayer,
        ));
    }
    commands.insert_resource(VoxelMeshes { live: live_mesh });
    commands.insert_resource(LiveVoxelAccum::default());
}

/// 消费 `Firefly/Viz` 的体素消息，按需重建实时层网格。
#[allow(clippy::needless_pass_by_value)] // `SystemParam` 契约，与同 crate 其它系统一致
pub fn update_live_voxels(
    time: Res<Time>,
    ports: NonSend<IpcPorts>,
    mut meshes: ResMut<Assets<Mesh>>,
    layer: Res<VoxelMeshes>,
    mut accum: ResMut<LiveVoxelAccum>,
    mut paths: ResMut<Paths>,
) {
    let Some(sub) = &ports.viz_sub else {
        return;
    };
    // 增量消息必须**按到达顺序全部应用**——丢一条就会与 planner 的图分叉，
    // 不能像全量快照那样"只留最新"。
    loop {
        match sub.receive() {
            Ok(Some(sample)) => {
                let msg: &VizMessage = &sample;
                let entity = &msg.entity[..msg.entity_len as usize];
                if msg.kind == kind::LINE_STRIP {
                    // 规划路径：只收 planner 发的 `plan/*`，直接并存（点数少，无需节流）。
                    if msg.point_count == 0 || !entity.starts_with(b"plan/") {
                        continue;
                    }
                    let name = String::from_utf8_lossy(entity).into_owned();
                    let n = (msg.point_count as usize).min(msg.points.len());
                    let points: Vec<Vec3> = msg.points[..n]
                        .iter()
                        .map(|p| Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32))
                        .collect();
                    let color = Color::srgb_u8(msg.color[0], msg.color[1], msg.color[2]);
                    if paths.lines.insert(name.clone(), (points, color)).is_none() {
                        log::info!("体素视图：收到规划路径 {name}");
                    }
                    continue;
                }
                if entity != b"plan/perceived"
                    || !matches!(
                        msg.kind,
                        kind::VOXELS | kind::VOXELS_ADD | kind::VOXELS_REMOVE
                    )
                {
                    continue;
                }
                let count = msg.voxel_count.min(firefly_pubsub::viz::VOXELS_MAX as u32);
                let indices = &msg.voxels[..count as usize];
                accum.size = msg.voxel_size;
                accum.origin = msg.voxel_origin;
                match msg.kind {
                    kind::VOXELS => {
                        accum.set.clear();
                        accum.set.extend(indices.iter().copied());
                        accum.dirty = true;
                    }
                    kind::VOXELS_ADD => {
                        for idx in indices {
                            accum.dirty |= accum.set.insert(*idx);
                        }
                    }
                    kind::VOXELS_REMOVE => {
                        for idx in indices {
                            accum.dirty |= accum.set.remove(idx);
                        }
                    }
                    _ => {}
                }
            }
            Ok(None) => break,
            Err(e) => {
                log::warn!("体素视图：可视化消息接收失败：{e}");
                break;
            }
        }
    }
    if !accum.dirty || time.elapsed_secs_f64() - accum.last_rebuild < REBUILD_INTERVAL {
        return;
    }
    accum.dirty = false;
    accum.last_rebuild = time.elapsed_secs_f64();
    if !accum.built {
        accum.built = true;
        log::info!("体素视图：实时感知层首次重建（{} 格）", accum.set.len());
    }
    if let Some(mut mesh) = meshes.get_mut(&layer.live) {
        let indices: Vec<[i32; 3]> = accum.set.iter().copied().collect();
        *mesh = cube_mesh(&indices, accum.origin, accum.size, VOXEL_FILL);
    }
}

/// 打开可视化订阅（失败降级为不显示实时层，不影响出图链路）。
#[must_use]
pub fn open_subscription(node: &firefly_pubsub::node::IpcNode) -> Option<Subscriber<VizMessage>> {
    // planner 一次 replan 会连着发多条不同实体（global_path/local_traj/planes…），
    // 缓冲 1 只会留下最后一条；给一个小缓冲覆盖突发，同时不冒犯"先建服务"那端的
    // `subscriber_max_buffer_size` 上限（要太大时会
    // `BufferSizeExceedsMaxSupportedBufferSizeOfService`）。
    match Subscriber::with_topic_and_buffer(node, VIZ_TOPIC, 8) {
        Ok(sub) => {
            log::info!("体素视图：已订阅 {VIZ_TOPIC}（实时感知地图）");
            Some(sub)
        }
        Err(e) => {
            log::warn!("体素视图：订阅 {VIZ_TOPIC} 失败，实时层不显示：{e}");
            None
        }
    }
}
