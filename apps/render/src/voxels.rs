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
//! 性能：每层一个**合并表面网格**（只发射与空邻居相邻的面，内部面不发），
//! 单 draw call；实时层只在内容指纹变化且距上次重建 ≥ [`REBUILD_INTERVAL`]
//! 时按原句柄原地更新，不每帧重建、不每帧分配。

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
const REBUILD_INTERVAL: f64 = 0.25;
/// 静态场地颜色（灰蓝）。
const FIELD_COLOR: Color = Color::srgb(0.38, 0.42, 0.52);
/// 实时感知颜色（青绿）。
const LIVE_COLOR: Color = Color::srgb(0.15, 0.85, 0.55);

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

/// 实时层一帧数据：时间戳、体素数、体素尺寸、网格原点、索引。
type LiveVoxels = (f64, u32, [f32; 3], [f32; 3], Vec<[i32; 3]>);

/// 已收到的规划路径（实体名 → 点集 + 颜色），用折线 gizmo 画。
#[derive(Resource, Default)]
pub struct Paths {
    lines: HashMap<String, (Vec<Vec3>, Color)>,
}

/// 折线 gizmo 的渲染层：**必须**限到主视角层，否则默认 layer 0 会被传感器
/// 相机一并画进发布给 VIO/深度的图像里。
#[allow(clippy::needless_pass_by_value)] // `SystemParam` 契约，与同 crate 其它系统一致
pub fn setup_gizmo_layers(mut store: ResMut<GizmoConfigStore>) {
    let (config, _) = store.config_mut::<DefaultGizmoConfigGroup>();
    config.render_layers = RenderLayers::layer(crate::rig::VOXEL_LAYER);
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
pub struct LiveStamp {
    /// `(时间戳, 体素数)`：与上次相同则跳过重建。
    stamp: Option<(f64, u32)>,
    /// 上次重建时刻（`Time::elapsed_secs_f64`）。
    last_rebuild: f64,
}

/// 体素网格句柄（材质由 Bevy 复用；静态层建好后不再改动，只留实时层句柄）。
#[derive(Resource)]
pub struct VoxelMeshes {
    live: Handle<Mesh>,
}

/// 由体素索引构建**表面**合并网格（只发射暴露面）。
///
/// 内部面不发：两个相邻体素之间的面看不见，去掉能把面数降低一个数量级。
#[must_use]
fn surface_mesh(indices: &[[i32; 3]], origin: [f32; 3], size: [f32; 3]) -> Mesh {
    let occupied: HashSet<[i32; 3]> = indices.iter().copied().collect();
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut normals: Vec<[f32; 3]> = Vec::new();
    let mut triangles: Vec<u32> = Vec::new();
    for idx in indices {
        let center = [
            origin[0] + (idx[0] as f32 + 0.5) * size[0],
            origin[1] + (idx[1] as f32 + 0.5) * size[1],
            origin[2] + (idx[2] as f32 + 0.5) * size[2],
        ];
        for (dir, corners, normal) in FACES {
            if occupied.contains(&[idx[0] + dir[0], idx[1] + dir[1], idx[2] + dir[2]]) {
                continue;
            }
            let base = positions.len() as u32;
            for corner in corners {
                positions.push([
                    center[0] + corner[0] * size[0],
                    center[1] + corner[1] * size[1],
                    center[2] + corner[2] * size[2],
                ]);
                normals.push(normal);
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
    let mut indices = Vec::new();
    for x in 0..dims[0] {
        for y in 0..dims[1] {
            for z in 0..dims[2] {
                if grid.state([x, y, z]) == VoxelState::Occupied {
                    indices.push([x as i32, y as i32, z as i32]);
                }
            }
        }
    }
    let origin = *grid.origin();
    let resolution = grid.resolution() as f32;
    log::info!(
        "体素视图：静态场地 {} 格（{:.2}m 分辨率）",
        indices.len(),
        resolution
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
        Some((indices, origin, size)) => meshes.add(surface_mesh(&indices, origin, size)),
        None => meshes.add(surface_mesh(&[], [0.0; 3], [1.0; 3])),
    };
    // 初始给一片退化三角形而不是空 mesh：空 mesh 会触发 Bevy `slab_allocator`
    // 的 use-after-free 报错（实测）。
    let live_mesh = meshes.add(surface_mesh(&[[0, 0, 0]], [0.0; 3], [f32::EPSILON; 3]));
    let field_material = materials.add(StandardMaterial {
        base_color: FIELD_COLOR,
        unlit: true,
        cull_mode: None,
        ..default()
    });
    let live_material = materials.add(StandardMaterial {
        base_color: LIVE_COLOR,
        unlit: true,
        cull_mode: None,
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
    commands.insert_resource(LiveStamp::default());
}

/// 消费 `Firefly/Viz` 的体素消息，按需重建实时层网格。
#[allow(clippy::needless_pass_by_value)] // `SystemParam` 契约，与同 crate 其它系统一致
pub fn update_live_voxels(
    time: Res<Time>,
    ports: NonSend<IpcPorts>,
    mut meshes: ResMut<Assets<Mesh>>,
    layer: Res<VoxelMeshes>,
    mut stamp: ResMut<LiveStamp>,
    mut paths: ResMut<Paths>,
) {
    let Some(sub) = &ports.viz_sub else {
        return;
    };
    // 只保留最新一条感知地图消息（每帧清空队列，避免积压）。
    let mut newest: Option<LiveVoxels> = None;
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
                if msg.kind != kind::VOXELS || entity != b"plan/perceived" {
                    continue;
                }
                let count = msg.voxel_count.min(firefly_pubsub::viz::VOXELS_MAX as u32);
                let indices = msg.voxels[..count as usize].to_vec();
                newest = Some((
                    msg.timestamp,
                    count,
                    msg.voxel_size,
                    msg.voxel_origin,
                    indices,
                ));
            }
            Ok(None) => break,
            Err(e) => {
                log::warn!("体素视图：可视化消息接收失败：{e}");
                break;
            }
        }
    }
    let Some((timestamp, count, size, origin, indices)) = newest else {
        return;
    };
    let fingerprint = (timestamp, count);
    if stamp.stamp == Some(fingerprint)
        || time.elapsed_secs_f64() - stamp.last_rebuild < REBUILD_INTERVAL
    {
        return;
    }
    if stamp.stamp.is_none() {
        log::info!("体素视图：实时感知层首次重建（{count} 格）");
    }
    stamp.stamp = Some(fingerprint);
    stamp.last_rebuild = time.elapsed_secs_f64();
    if let Some(mut mesh) = meshes.get_mut(&layer.live) {
        let rebuilt = surface_mesh(&indices, origin, size);
        *mesh = rebuilt;
    }
}

/// 打开可视化订阅（失败降级为不显示实时层，不影响出图链路）。
#[must_use]
pub fn open_subscription(node: &firefly_pubsub::node::IpcNode) -> Option<Subscriber<VizMessage>> {
    // 只保留最新一条体素消息，缓冲 1 即可；也避免超过“先建服务”那端的上限
    // （`open_or_create` 由先到者定 `subscriber_max_buffer_size`）。
    match Subscriber::with_topic(node, VIZ_TOPIC) {
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
