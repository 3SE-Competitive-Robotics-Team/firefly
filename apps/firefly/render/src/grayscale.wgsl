struct FullscreenVertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@group(0) @binding(0) var in_texture: texture_2d<f32>;
@group(0) @binding(1) var in_sampler: sampler;

// CompositingSpace::Srgb stores encoded RGB in the intermediate UNORM texture.
@fragment
fn fs_main(in: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let rgb = round(clamp(textureLoad(in_texture, vec2<i32>(in.position.xy), 0).rgb, vec3(0.0), vec3(1.0)) * 255.0);
    let gray = floor(0.299 * rgb.r + 0.587 * rgb.g + 0.114 * rgb.b) / 255.0;
    return vec4(gray, gray, gray, 1.0);
}
