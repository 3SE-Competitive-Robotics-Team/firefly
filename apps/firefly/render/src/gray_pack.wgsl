@group(0) @binding(0) var gray: texture_2d<f32>;
@group(0) @binding(1) var<storage, read_write> packed: array<u32>;

// Four R8 pixels per word avoid COPY_BYTES_PER_ROW_ALIGNMENT padding.
@compute @workgroup_size(64)
fn pack(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x >= arrayLength(&packed) { return; }
    let size = textureDimensions(gray);
    var word = 0u;
    for (var i = 0u; i < 4u; i += 1u) {
        let pixel = id.x * 4u + i;
        let xy = vec2<i32>(i32(pixel % size.x), i32(pixel / size.x));
        let value = u32(round(textureLoad(gray, xy, 0).r * 255.0));
        word |= value << (8u * i);
    }
    packed[id.x] = word;
}
