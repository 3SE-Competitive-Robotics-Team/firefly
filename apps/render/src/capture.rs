//! GPU 纹理回读：传感器颜色目标 + 深度预通道纹理 → CPU。
//!
//! 链路（每相机节拍一次，由 `link` 在新位姿到达时发起请求）：
//! 主世界 `CaptureHub::request` → 渲染世界 `copy_sensors`（`Cleanup` 阶段，
//! 图执行之后同队列提交拷贝）→ `map_async` 回调经通道回传字节 →
//! 主世界 `link` 配对三份同 `seq` 回复后发布。
//!
//! 行距约束：`320 × 4 = 1280` 字节恰为 256 对齐（`wgpu` 拷贝要求），
//! 无填充行可言；颜色与深度（`Depth32Float`）同为每像素 4 字节。

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

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
    /// 有拷贝在途（渲染世界置位，最后一份回复的回调清零）。
    pub inflight: AtomicBool,
    /// 待回复计数（请求分发时置 3，每份回复递减）。
    pub pending: AtomicU32,
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
}

/// 回读暂存缓冲（三份纹理各一块，复用；回调内 `unmap` 后归还）。
#[derive(Resource)]
struct CaptureBuffers {
    /// 左目暂存。
    left: Buffer,
    /// 右目暂存。
    right: Buffer,
    /// 深度暂存。
    depth: Buffer,
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
        Self {
            left: device.create_buffer(&desc),
            right: device.create_buffer(&desc),
            depth: device.create_buffer(&desc),
        }
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

/// 拷贝传感器纹理 → 暂存缓冲（`Render` 之后同队列提交，顺序保证新于渲染）。
#[allow(clippy::needless_pass_by_value, clippy::too_many_arguments)]
fn copy_sensors(
    hub: Res<CaptureHub>,
    images: Res<RenderAssets<GpuImage>>,
    depth_cams: Query<(Entity, &Eye, &ViewDepthTexture)>,
    buffers: Res<CaptureBuffers>,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
) {
    let _ = device.poll(PollType::Poll);
    let req = hub.inner.request.lock().expect("hub poisoned").take();
    let Some(req) = req else { return };
    if hub.inner.inflight.swap(true, Ordering::SeqCst) {
        *hub.inner.request.lock().expect("hub poisoned") = Some(req);
        return;
    }
    let Some(left) = images.get(req.left) else {
        log::debug!("左目 GPU 纹理尚未就绪，跳过本节拍");
        hub.inner.inflight.store(false, Ordering::SeqCst);
        return;
    };
    let Some(right) = images.get(req.right) else {
        log::debug!("右目 GPU 纹理尚未就绪，跳过本节拍");
        hub.inner.inflight.store(false, Ordering::SeqCst);
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
        log::debug!("深度预通道纹理尚未就绪，跳过本节拍");
        hub.inner.inflight.store(false, Ordering::SeqCst);
        return;
    };

    hub.inner.pending.store(3, Ordering::SeqCst);
    let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor {
        label: Some("sensor-capture"),
    });
    encode_copy(&mut encoder, &left.texture, &buffers.left);
    encode_copy(&mut encoder, &right.texture, &buffers.right);
    encode_copy(&mut encoder, depth_tex, &buffers.depth);
    queue.submit(std::iter::once(encoder.finish()));
    // 映射必须在提交之后：提交校验要求被引用的缓冲处于 Idle，预映射
    // 即 `BufferStillMapped` 校验错误（默认错误策略直接退出进程）。
    map_for_readback(&buffers.left, "left", &hub, SensorKind::Left, req);
    map_for_readback(&buffers.right, "right", &hub, SensorKind::Right, req);
    map_for_readback(&buffers.depth, "depth", &hub, SensorKind::Depth, req);
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

/// 暂存缓冲映射回传（提交后调用，回调内拷贝出字节即 `unmap` 归还缓冲）。
#[allow(clippy::too_many_arguments)]
fn map_for_readback(
    buffer: &Buffer,
    label: &'static str,
    hub: &CaptureHub,
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
        if hub.pending.fetch_sub(1, Ordering::SeqCst) == 1 {
            hub.inflight.store(false, Ordering::SeqCst);
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
