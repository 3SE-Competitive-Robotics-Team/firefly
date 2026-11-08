//! Minification for uncompressed material color and scalar textures.
use bevy::{
    prelude::*,
    render::render_resource::{TextureDimension, TextureFormat},
};

/// Complete PNG material mip chains before render extraction. Sensor targets,
/// normal maps, cubemaps and precomputed/compressed mip chains are untouched.
pub struct MaterialMipmapPlugin;
impl Plugin for MaterialMipmapPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, prepare_materials);
    }
}

#[allow(clippy::needless_pass_by_value)]
fn prepare_materials(
    mut events: MessageReader<AssetEvent<StandardMaterial>>,
    materials: Res<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
) {
    for event in events.read() {
        let (AssetEvent::Added { id }
        | AssetEvent::Modified { id }
        | AssetEvent::LoadedWithDependencies { id }) = event
        else {
            continue;
        };
        let Some(material) = materials.get(*id) else {
            continue;
        };
        for handle in [
            &material.base_color_texture,
            &material.metallic_roughness_texture,
        ]
        .into_iter()
        .flatten()
        {
            if let Some(mut image) = images.get_mut(handle) {
                complete_mips(&mut image);
            }
        }
    }
}

fn decode(value: u8, srgb: bool) -> f32 {
    let value = f32::from(value) / 255.;
    if !srgb {
        value
    } else if value <= 0.04045 {
        value / 12.92
    } else {
        ((value + 0.055) / 1.055).powf(2.4)
    }
}

fn encode(value: f32, srgb: bool) -> u8 {
    let value = if !srgb {
        value
    } else if value <= 0.003_130_8 {
        value * 12.92
    } else {
        1.055 * value.powf(1. / 2.4) - 0.055
    };
    (value * 255.).round().clamp(0., 255.) as u8
}

fn complete_mips(image: &mut Image) {
    let descriptor = &image.texture_descriptor;
    if descriptor.dimension != TextureDimension::D2
        || descriptor.size.depth_or_array_layers != 1
        || descriptor.mip_level_count != 1
        || !matches!(
            descriptor.format,
            TextureFormat::Rgba8Unorm | TextureFormat::Rgba8UnormSrgb
        )
    {
        return;
    }
    let srgb = descriptor.format == TextureFormat::Rgba8UnormSrgb;
    let (mut width, mut height) = (
        descriptor.size.width as usize,
        descriptor.size.height as usize,
    );
    let Some(data) = image
        .data
        .as_mut()
        .filter(|data| data.len() == width * height * 4)
    else {
        return;
    };
    // Keep linear values between levels; quantize only each stored mip.
    let mut source: Vec<f32> = data
        .iter()
        .enumerate()
        .map(|(i, &value)| decode(value, srgb && i % 4 != 3))
        .collect();
    while width > 1 || height > 1 {
        let (next_width, next_height) = ((width / 2).max(1), (height / 2).max(1));
        let mut next = vec![0.; next_width * next_height * 4];
        for y in 0..next_height {
            for x in 0..next_width {
                let (x0, x1) = (
                    x as f32 * width as f32 / next_width as f32,
                    (x + 1) as f32 * width as f32 / next_width as f32,
                );
                let (y0, y1) = (
                    y as f32 * height as f32 / next_height as f32,
                    (y + 1) as f32 * height as f32 / next_height as f32,
                );
                for sy in y0.floor() as usize..(y1.ceil() as usize).min(height) {
                    for sx in x0.floor() as usize..(x1.ceil() as usize).min(width) {
                        let weight = (x1.min((sx + 1) as f32) - x0.max(sx as f32))
                            * (y1.min((sy + 1) as f32) - y0.max(sy as f32))
                            / ((x1 - x0) * (y1 - y0));
                        for channel in 0..4 {
                            next[(y * next_width + x) * 4 + channel] +=
                                source[(sy * width + sx) * 4 + channel] * weight;
                        }
                    }
                }
            }
        }
        data.extend(
            next.iter()
                .enumerate()
                .map(|(i, &v)| encode(v, srgb && i % 4 != 3)),
        );
        source = next;
        (width, height) = (next_width, next_height);
        image.texture_descriptor.mip_level_count += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::{asset::RenderAssetUsages, render::render_resource::Extent3d};

    #[test]
    fn material_event_completes_non_power_of_two_texture() {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, AssetPlugin::default()))
            .init_asset::<Image>()
            .init_asset::<StandardMaterial>()
            .add_plugins(MaterialMipmapPlugin);
        let image = Image::new(
            Extent3d {
                width: 3,
                height: 1,
                depth_or_array_layers: 1,
            },
            TextureDimension::D2,
            vec![0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255, 255],
            TextureFormat::Rgba8Unorm,
            RenderAssetUsages::default(),
        );
        let handle = app.world_mut().resource_mut::<Assets<Image>>().add(image);
        let _material = app
            .world_mut()
            .resource_mut::<Assets<StandardMaterial>>()
            .add(StandardMaterial {
                metallic_roughness_texture: Some(handle.clone()),
                ..Default::default()
            });
        app.update();
        app.update();
        let images = app.world().resource::<Assets<Image>>();
        let image = images.get(&handle).unwrap();
        assert_eq!(image.texture_descriptor.mip_level_count, 2);
        assert_eq!(&image.data.as_ref().unwrap()[12..], [85, 85, 85, 255]);
    }

    #[test]
    fn mip_chain_filters_color_in_linear_light_and_keeps_base_pixels() {
        for (format, expected) in [
            (TextureFormat::Rgba8UnormSrgb, 188),
            (TextureFormat::Rgba8Unorm, 128),
        ] {
            let base = vec![
                0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 255,
            ];
            let mut image = Image::new(
                Extent3d {
                    width: 2,
                    height: 2,
                    depth_or_array_layers: 1,
                },
                TextureDimension::D2,
                base.clone(),
                format,
                RenderAssetUsages::default(),
            );
            complete_mips(&mut image);
            assert_eq!(image.texture_descriptor.mip_level_count, 2);
            assert_eq!(&image.data.as_ref().unwrap()[..16], base);
            assert_eq!(
                &image.data.as_ref().unwrap()[16..],
                [expected, expected, expected, 255]
            );
            let once = image.data.clone();
            complete_mips(&mut image);
            assert_eq!(image.data, once);
        }
    }
}
