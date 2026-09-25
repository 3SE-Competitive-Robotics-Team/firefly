//! GPU 纹理回读：传感器颜色目标 + 深度预通道纹理 → CPU。
//!
//! 链路（每相机节拍一次，由 `link` 按固定 10Hz 发起请求）：
//! 主世界 `CaptureHub::request` → 渲染世界 `copy_sensors`（`Cleanup` 阶段，
//! 图执行之后同队列提交拷贝）→ `map_async` 回调经通道回传字节 →
//! 主世界 `link` 配对三份同 `seq` 回复后发布。
//!
//! 缓冲**双缓冲**（[`CAPTURE_SETS`] 组）：上一拍还在途时即可发起下一拍，
//! 供图节奏不再被单拍完成时间卡住（单缓冲时拍长一旦超过节拍就丢拍）。
//!
//! 行距约束：`320 × 4 = 1280` 字节恰为 256 对齐（`wgpu` 拷贝要求），
//! 无填充行可言；颜色与深度（`Depth32Float`）同为每像素 4 字节。

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

use bevy::image::Image;
use bevy::prelude::*;
use bevy::render::GpuResourceAppExt;
use bevy::render::Render;
use bevy::render::render_asset::RenderAssets;
use bevy::render::render_resource::{
    Buffer, BufferDescriptor, BufferUsages, CommandEncoderDescriptor, Extent3d, MapMode, Origin3d,
    PollType, TexelCopyBufferInfo, TexelCopyBufferLayout, TexelCopyTextureInfo, TextureAspect,
};
use bevy::render::renderer::{RenderDevice, RenderQueue};
use bevy::render::sync_world::RenderEntity;
use bevy::render::texture::GpuImage;
use bevy::render::view::ViewDepthTexture;
use bevy::render::{Extract, RenderSystems};
use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_WIDTH};
use firefly_pubsub::trace::TraceContext;

use crate::rig::Eye;

/// 单帧字节数（`RGBA8` / `Depth32Float` 皆 4 字节每像素）。
pub const FRAME_BYTES: usize = 4 * IMAGE_WIDTH * IMAGE_HEIGHT;
/// 拷贝行距（字节，256 对齐，无填充）。
pub const BYTES_PER_ROW: u32 = 4 * IMAGE_WIDTH as u32;
/// 回读缓冲组数：双缓冲，允许上一拍在途时发起下一拍。
pub const CAPTURE_SETS: usize = 2;

/// 传感器种类（回读回复的归属标记）。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SensorKind {
    /// 左目颜色。
    Left,
    /// 右目颜色。
    Right,
    /// 深度预通道。
    Depth,
}

/// 回读请求（主世界→渲染世界，`seq` 关联三份回复与位姿时间戳）。
#[derive(Clone, Copy, Debug)]
pub struct CaptureRequest {
    /// 单调序号。
    pub seq: u64,
    /// 位姿时间戳（秒，发布时原样透传）。
    pub stamp: f64,
    /// 上游 trace 上下文（发布时续接）。
    pub trace: TraceContext,
    /// 左目颜色目标。
    pub left: AssetId<Image>,
    /// 右目颜色目标。
    pub right: AssetId<Image>,
}

/// 回读回复（渲染世界→主世界）。
#[derive(Debug)]
pub struct CapturedFrame {
    /// 对应请求的序号。
    pub seq: u64,
    /// 传感器种类。
    pub kind: SensorKind,
    /// 时间戳（透传自请求）。
    pub stamp: f64,
    /// 上游 trace 上下文（透传自请求）。
    pub trace: TraceContext,
    /// 紧凑像素（颜色 `RGBA8` / 深度小端 f32，行主序无填充）。
    pub bytes: Vec<u8>,
}

/// 跨世界回读通道（主世界与渲染世界各持同一 `Arc`）。
#[derive(Resource, Clone, Default)]
pub struct CaptureHub {
    /// 共享内核。
    pub inner: Arc<HubInner>,
}

/// 回读通道内核。
#[derive(Debug, Default)]
pub struct HubInner {
    /// 待执行的最新请求（新位姿覆盖旧请求，渲染永远追最新）。
    pub request: Mutex<Option<CaptureRequest>>,
    /// 已完成的回复队列。
    pub done: Mutex<VecDeque<CapturedFrame>>,
    /// 在途缓冲组数（`0..=CAPTURE_SETS`；回调清零一份回复时递减）。
    pub inflight_sets: AtomicU32,
    /// 最近一次被渲染世界**取走**（提交拷贝）的请求序号；主世界据此收起
    /// 传感器相机（单调持久值，不受帧率/时序影响，见 `link::poll_pose`）。
    pub taken_seq: AtomicU64,
}

impl CaptureHub {
    /// 发起回读请求（新位姿到达时调用；上个请求若在途则等下帧）。
    pub fn request(&self, req: CaptureRequest) {
        *self.inner.request.lock().expect("hub poisoned") = Some(req);
    }

    /// 取出全部已完成的回复。
    pub fn drain(&self) -> Vec<CapturedFrame> {
        self.inner
            .done
            .lock()
            .expect("hub poisoned")
            .drain(..)
            .collect()
    }

    /// 是否有请求待渲染世界取走。
    pub fn has_pending(&self) -> bool {
        self.inner.request.lock().expect("hub poisoned").is_some()
    }

    /// 是否还能发起新一拍（有空闲缓冲组）。
    pub fn can_issue(&self) -> bool {
        (self.inner.inflight_sets.load(Ordering::SeqCst) as usize) < CAPTURE_SETS
    }

    /// 最近被取走的请求序号（未被取走时保持旧值）。
    pub fn taken_seq(&self) -> u64 {
        self.inner.taken_seq.load(Ordering::SeqCst)
    }
}

/// 单组回读暂存（三份纹理各一块；回调内 `unmap` 后归还）。
struct CaptureSet {
    /// 左目暂存。
    left: Buffer,
    /// 右目暂存。
    right: Buffer,
    /// 深度暂存。
    depth: Buffer,
    /// 在途回复数（3 → 0）；0 表示本组空闲，可再取一拍。
    pending: Arc<AtomicU32>,
}

/// 回读暂存缓冲组池（`init_gpu_resource` 创建，设备丢失时重建）。
#[derive(Resource)]
struct CaptureBuffers {
    /// 双缓冲组。
    sets: Vec<CaptureSet>,
}

impl FromWorld for CaptureBuffers {
    fn from_world(world: &mut World) -> Self {
        let device = world.resource::<RenderDevice>();
        let desc = BufferDescriptor {
            label: Some("sensor-capture"),
            size: FRAME_BYTES as u64,
            usage: BufferUsages::COPY_DST | BufferUsages::MAP_READ,
            mapped_at_creation: false,
        };
        let sets = (0..CAPTURE_SETS)
            .map(|_| CaptureSet {
                left: device.create_buffer(&desc),
                right: device.create_buffer(&desc),
                depth: device.create_buffer(&desc),
                pending: Arc::new(AtomicU32::new(0)),
            })
            .collect();
        Self { sets }
    }
}

/// 透传 rig 标记到渲染世界（渲染世界的相机实体靠它辨认深度相机）。
#[allow(clippy::needless_pass_by_value)]
fn extract_eyes(query: Extract<Query<(RenderEntity, &Eye)>>, mut commands: Commands) {
    for (entity, eye) in &query {
        if let Ok(mut target) = commands.get_entity(entity) {
            target.insert(*eye);
        }
    }
}

/// 拷贝传感器纹理 → 空闲暂存组（`Render` 之后同队列提交，顺序保证新于渲染）。
///
/// 请求在无空闲组或纹理未就绪时**放回**而非丢弃：主世界据此保持相机激活、
/// 下帧重试（相机按需激活，见 `link::poll_pose`）。
#[allow(clippy::needless_pass_by_value, clippy::too_many_arguments)]
fn copy_sensors(
    hub: Res<CaptureHub>,
    images: Res<RenderAssets<GpuImage>>,
    depth_cams: Query<(Entity, &Eye, &ViewDepthTexture)>,
    buffers: Res<CaptureBuffers>,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
) {
    if hub.inner.inflight_sets.load(Ordering::SeqCst) == 0 && !hub.has_pending() {
        return;
    }
    let _ = device.poll(PollType::Poll);

    // 先找空闲组：全忙时把请求留在槽里（主世界不会顶掉），下帧再试。
    let Some(set) = buffers
        .sets
        .iter()
        .find(|set| set.pending.load(Ordering::SeqCst) == 0)
    else {
        return;
    };
    let Some(req) = hub.inner.request.lock().expect("hub poisoned").take() else {
        return;
    };
    let Some(left) = images.get(req.left) else {
        log::debug!("左目 GPU 纹理尚未就绪，下帧重试");
        requeue(&hub, req);
        return;
    };
    let Some(right) = images.get(req.right) else {
        log::debug!("右目 GPU 纹理尚未就绪，下帧重试");
        requeue(&hub, req);
        return;
    };
    let mut depth_tex = None;
    for (_, eye, view) in &depth_cams {
        if *eye == Eye::Depth {
            depth_tex = Some(&view.texture);
            break;
        }
    }
    let Some(depth_tex) = depth_tex else {
        log::debug!("深度预通道纹理尚未就绪，下帧重试");
        requeue(&hub, req);
        return;
    };

    // 请求被真正消费（拷贝已编码）：主世界以 `taken_seq` 判据收起相机。
    set.pending.store(3, Ordering::SeqCst);
    hub.inner.inflight_sets.fetch_add(1, Ordering::SeqCst);
    hub.inner.taken_seq.store(req.seq, Ordering::SeqCst);
    let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor {
        label: Some("sensor-capture"),
    });
    encode_copy(&mut encoder, &left.texture, &set.left);
    encode_copy(&mut encoder, &right.texture, &set.right);
    encode_copy(&mut encoder, depth_tex, &set.depth);
    queue.submit(std::iter::once(encoder.finish()));
    // 映射必须在提交之后：提交校验要求被引用的缓冲处于 Idle，预映射
    // 即 `BufferStillMapped` 校验错误（默认错误策略直接退出进程）。
    map_for_readback(
        &set.left,
        "left",
        &hub,
        set.pending.clone(),
        SensorKind::Left,
        req,
    );
    map_for_readback(
        &set.right,
        "right",
        &hub,
        set.pending.clone(),
        SensorKind::Right,
        req,
    );
    map_for_readback(
        &set.depth,
        "depth",
        &hub,
        set.pending.clone(),
        SensorKind::Depth,
        req,
    );
}

/// 把未消费的请求放回待执行槽（下帧重试）。
fn requeue(hub: &CaptureHub, req: CaptureRequest) {
    *hub.inner.request.lock().expect("hub poisoned") = Some(req);
}

/// 单份纹理拷贝编码（只编码不映射，映射见 `map_for_readback`）。
fn encode_copy(
    encoder: &mut bevy::render::render_resource::CommandEncoder,
    texture: &bevy::render::render_resource::Texture,
    buffer: &Buffer,
) {
    encoder.copy_texture_to_buffer(
        TexelCopyTextureInfo {
            texture: &**texture,
            mip_level: 0,
            origin: Origin3d::ZERO,
            aspect: TextureAspect::All,
        },
        TexelCopyBufferInfo {
            buffer,
            layout: TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(BYTES_PER_ROW),
                rows_per_image: Some(IMAGE_HEIGHT as u32),
            },
        },
        Extent3d {
            width: IMAGE_WIDTH as u32,
            height: IMAGE_HEIGHT as u32,
            depth_or_array_layers: 1,
        },
    );
}

/// 暂存缓冲映射回传（提交后调用，回调内拷贝出字节即 `unmap` 归还缓冲组）。
#[allow(clippy::too_many_arguments)]
fn map_for_readback(
    buffer: &Buffer,
    label: &'static str,
    hub: &CaptureHub,
    pending: Arc<AtomicU32>,
    kind: SensorKind,
    req: CaptureRequest,
) {
    let hub = hub.inner.clone();
    let owned = buffer.clone();
    buffer.slice(..).map_async(MapMode::Read, move |result| {
        if let Err(e) = result {
            log::warn!("{label} 回读映射失败：{e:?}");
        } else {
            let bytes = owned.slice(..).get_mapped_range().to_vec();
            owned.unmap();
            hub.done
                .lock()
                .expect("hub poisoned")
                .push_back(CapturedFrame {
                    seq: req.seq,
                    kind,
                    stamp: req.stamp,
                    trace: req.trace,
                    bytes,
                });
        }
        if pending.fetch_sub(1, Ordering::SeqCst) == 1 {
            hub.inflight_sets.fetch_sub(1, Ordering::SeqCst);
        }
    });
}

/// 回读插件（渲染世界侧系统注册）。
pub struct CapturePlugin;

impl Plugin for CapturePlugin {
    fn build(&self, app: &mut App) {
        let hub = app.world().resource::<CaptureHub>().clone();
        let render_app = app.sub_app_mut(bevy::render::RenderApp);
        // 暂存缓冲必须等渲染设备就绪后创建（`RenderDevice` 在插件
        // `build` 时尚不存在，`init_gpu_resource` 将其推迟到
        // `RenderStartup`，设备丢失重建时亦会自动重建）。
        render_app
            .insert_resource(hub)
            .init_gpu_resource::<CaptureBuffers>()
            .add_systems(bevy::render::ExtractSchedule, extract_eyes)
            .add_systems(Render, copy_sensors.in_set(RenderSystems::Cleanup));
    }
}
