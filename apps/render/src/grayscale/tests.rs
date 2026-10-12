use super::*;
use bevy::app::PluginsState;
use bevy::render::render_resource::{
    BufferDescriptor, BufferUsages, ColorTargetState, ColorWrites, CommandEncoderDescriptor,
    Extent3d, LoadOp, MapMode, Operations, Origin3d, PollType, RawFragmentState,
    RawRenderPipelineDescriptor, RawVertexState, RenderPassColorAttachment, RenderPassDescriptor,
    SamplerDescriptor, StoreOp, TexelCopyBufferLayout, TexelCopyTextureInfo, TextureAspect,
    TextureDescriptor, TextureDimension, TextureFormat, TextureUsages, TextureViewDescriptor,
};
use bevy::render::renderer::RenderQueue;
use bevy::window::{ExitCondition, WindowPlugin};
use bevy::winit::WinitPlugin;
use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_WIDTH};
use std::time::{Duration, Instant};

fn gpu() -> (RenderDevice, RenderQueue) {
    let mut app = App::new();
    app.add_plugins(
        DefaultPlugins
            .build()
            .disable::<WinitPlugin>()
            .disable::<bevy::render::pipelined_rendering::PipelinedRenderingPlugin>()
            .set(WindowPlugin {
                primary_window: None,
                exit_condition: ExitCondition::DontExit,
                ..default()
            }),
    );
    let deadline = Instant::now() + Duration::from_secs(10);
    while app.plugins_state() != PluginsState::Ready {
        assert!(Instant::now() < deadline, "GPU initialization timed out");
        bevy::tasks::tick_global_task_pools_on_main_thread();
        std::thread::yield_now();
    }
    app.finish();
    app.cleanup();
    let world = app.sub_app(RenderApp).world();
    (
        world.resource::<RenderDevice>().clone(),
        world.resource::<RenderQueue>().clone(),
    )
}

#[test]
#[ignore = "requires a native GPU; run cargo test -p render gpu_grayscale -- --ignored"]
#[allow(clippy::too_many_lines)]
fn gpu_grayscale_matches_bt601_and_packs_rows_without_padding() {
    let (device, queue) = gpu();
    let colors = [
        [255, 0, 0, 255],
        [0, 255, 0, 255],
        [0, 0, 255, 255],
        [255; 4],
        [0, 0, 0, 255],
    ];
    let rgb: Vec<u8> = (0..IMAGE_SIZE)
        .flat_map(|i| {
            if i < colors.len() {
                colors[i]
            } else {
                [i as u8, (i * 17) as u8, (i * 73) as u8, 255]
            }
        })
        .collect();
    let desc = TextureDescriptor {
        label: Some("gray-test"),
        size: Extent3d {
            width: IMAGE_WIDTH as u32,
            height: IMAGE_HEIGHT as u32,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: TextureDimension::D2,
        format: TextureFormat::Rgba8Unorm,
        usage: TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST,
        view_formats: &[],
    };
    let source = device.create_texture(&desc);
    let gray = device.create_texture(&TextureDescriptor {
        format: TextureFormat::R8Unorm,
        usage: TextureUsages::TEXTURE_BINDING | TextureUsages::RENDER_ATTACHMENT,
        ..desc
    });
    queue.write_texture(
        TexelCopyTextureInfo {
            texture: &source,
            mip_level: 0,
            origin: Origin3d::ZERO,
            aspect: TextureAspect::All,
        },
        &rgb,
        TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some((IMAGE_WIDTH * 4) as u32),
            rows_per_image: Some(IMAGE_HEIGHT as u32),
        },
        desc.size,
    );
    let layout = device.create_bind_group_layout(
        "gray-test",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::FRAGMENT,
            (
                texture_2d(TextureSampleType::Float { filterable: false }),
                bevy::render::render_resource::binding_types::sampler(
                    bevy::render::render_resource::SamplerBindingType::NonFiltering,
                ),
            ),
        ),
    );
    let vertex = r"
@vertex fn vertex(@builtin(vertex_index) i: u32) -> FullscreenVertexOutput {
let p = array(vec2(-1.0, -1.0), vec2(3.0, -1.0), vec2(-1.0, 3.0));
return FullscreenVertexOutput(vec4(p[i], 0.0, 1.0), vec2(0.0));
}";
    let module = device.create_and_validate_shader_module(ShaderModuleDescriptor {
        label: Some("gray-test"),
        source: ShaderSource::Wgsl(
            format!("{}\n{vertex}", include_str!("../grayscale.wgsl")).into(),
        ),
    });
    let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
        label: None,
        bind_group_layouts: &[Some(&layout)],
        immediate_size: 0,
    });
    let pipeline = device.create_render_pipeline(&RawRenderPipelineDescriptor {
        label: Some("gray-test"),
        layout: Some(&pipeline_layout),
        vertex: RawVertexState {
            module: &module,
            entry_point: Some("vertex"),
            compilation_options: default(),
            buffers: &[],
        },
        fragment: Some(RawFragmentState {
            module: &module,
            entry_point: Some("fs_main"),
            compilation_options: default(),
            targets: &[Some(ColorTargetState {
                format: TextureFormat::R8Unorm,
                blend: None,
                write_mask: ColorWrites::ALL,
            })],
        }),
        primitive: default(),
        depth_stencil: None,
        multisample: default(),
        multiview_mask: None,
        cache: None,
    });
    let src_view = source.create_view(&TextureViewDescriptor::default());
    let gray_view = gray.create_view(&TextureViewDescriptor::default());
    let sampler = device.create_sampler(&SamplerDescriptor::default());
    let bind = device.create_bind_group(
        "gray-test",
        &layout,
        &BindGroupEntries::sequential((&src_view, &sampler)),
    );
    let buffer_desc = BufferDescriptor {
        label: None,
        size: IMAGE_SIZE as u64,
        usage: BufferUsages::STORAGE | BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    };
    let packed = device.create_buffer(&buffer_desc);
    let staging = device.create_buffer(&BufferDescriptor {
        usage: BufferUsages::COPY_DST | BufferUsages::MAP_READ,
        ..buffer_desc
    });
    let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor::default());
    {
        let mut pass = encoder.begin_render_pass(&RenderPassDescriptor {
            label: None,
            color_attachments: &[Some(RenderPassColorAttachment {
                view: &gray_view,
                depth_slice: None,
                resolve_target: None,
                ops: Operations {
                    load: LoadOp::Clear(default()),
                    store: StoreOp::Store,
                },
            })],
            depth_stencil_attachment: None,
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &bind, &[]);
        pass.draw(0..3, 0..1);
    }
    GrayPackPipeline::new(&device).encode(&device, &mut encoder, &gray_view, &packed, &staging);
    let submission = queue.submit([encoder.finish()]);
    let (done, received) = std::sync::mpsc::channel();
    staging
        .slice(..)
        .map_async(MapMode::Read, move |result| done.send(result).unwrap());
    device
        .poll(PollType::Wait {
            submission_index: Some(submission),
            timeout: Some(Duration::from_secs(10)),
        })
        .unwrap();
    received
        .recv_timeout(Duration::from_secs(10))
        .unwrap()
        .unwrap();
    let actual = staging.slice(..).get_mapped_range().unwrap();
    let mut expected = vec![0; IMAGE_SIZE];
    crate::sensors::rgb_to_gray(&rgb, &mut expected);
    assert_eq!(actual.len(), IMAGE_SIZE);
    for (i, (a, e)) in actual.iter().zip(&expected).enumerate() {
        assert!(a.abs_diff(*e) <= 1, "gray pixel {i}: GPU={a} CPU={e}");
    }
    assert_eq!(&actual[..colors.len()], &expected[..colors.len()]);
    drop(actual);
    staging.unmap();
}
