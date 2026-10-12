//! 传感器最终输出：GPU 上生成 BT.601 灰度，并打包为紧凑单字节回读。

use bevy::asset::{embedded_asset, load_embedded_asset};
use bevy::core_pipeline::blit::{BlitPipeline, init_blit_pipeline};
use bevy::prelude::*;
use bevy::render::render_resource::{
    BindGroupEntries, BindGroupLayout, BindGroupLayoutEntries, Buffer, CommandEncoder,
    ComputePassDescriptor, ComputePipeline, PipelineCompilationOptions, PipelineLayoutDescriptor,
    RawComputePipelineDescriptor, ShaderModuleDescriptor, ShaderSource, ShaderStages,
    TextureSampleType, TextureView,
    binding_types::{storage_buffer, texture_2d},
};
use bevy::render::renderer::RenderDevice;
use bevy::render::{RenderApp, RenderStartup};

use firefly_pubsub::camera::IMAGE_SIZE;

pub struct GrayscalePlugin;

impl Plugin for GrayscalePlugin {
    fn build(&self, app: &mut App) {
        embedded_asset!(app, "grayscale.wgsl");
        app.sub_app_mut(RenderApp).add_systems(
            RenderStartup,
            configure_gray_output.after(init_blit_pipeline),
        );
    }
}

#[cfg(test)]
mod tests;

#[allow(clippy::needless_pass_by_value)]
fn configure_gray_output(mut blit: ResMut<BlitPipeline>, assets: Res<AssetServer>) {
    // 本应用的颜色相机均为 R8 传感器，最终 blit 输出感知灰度。
    blit.fragment_shader = load_embedded_asset!(assets.as_ref(), "grayscale.wgsl");
}

#[derive(Resource)]
pub struct GrayPackPipeline {
    layout: BindGroupLayout,
    pipeline: ComputePipeline,
}

impl FromWorld for GrayPackPipeline {
    fn from_world(world: &mut World) -> Self {
        Self::new(world.resource::<RenderDevice>())
    }
}

impl GrayPackPipeline {
    pub fn new(device: &RenderDevice) -> Self {
        let layout = device.create_bind_group_layout(
            "gray-pack",
            &BindGroupLayoutEntries::sequential(
                ShaderStages::COMPUTE,
                (
                    texture_2d(TextureSampleType::Float { filterable: false }),
                    storage_buffer::<Vec<u32>>(false),
                ),
            ),
        );
        let module = device.create_and_validate_shader_module(ShaderModuleDescriptor {
            label: Some("gray-pack"),
            source: ShaderSource::Wgsl(include_str!("gray_pack.wgsl").into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: Some("gray-pack"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_compute_pipeline(&RawComputePipelineDescriptor {
            label: Some("gray-pack"),
            layout: Some(&pipeline_layout),
            module: &module,
            entry_point: Some("pack"),
            compilation_options: PipelineCompilationOptions::default(),
            cache: None,
        });
        Self { layout, pipeline }
    }

    pub fn encode(
        &self,
        device: &RenderDevice,
        encoder: &mut CommandEncoder,
        source: &TextureView,
        packed: &Buffer,
        staging: &Buffer,
    ) {
        let bind = device.create_bind_group(
            "gray-pack",
            &self.layout,
            &BindGroupEntries::sequential((source, packed.as_entire_binding())),
        );
        {
            let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
                label: Some("gray-pack"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups((IMAGE_SIZE / 4).div_ceil(64) as u32, 1, 1);
        }
        encoder.copy_buffer_to_buffer(packed, 0, staging, 0, IMAGE_SIZE as u64);
    }
}
