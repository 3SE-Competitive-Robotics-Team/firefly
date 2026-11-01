use firefly_basalt_headers::image::{
    image::ManagedImage,
    image_pyr::{ManagedImagePyr, PyramidPixel},
};
use proptest::prelude::*;
use serde_json::Value;

fn compare<T: PyramidPixel>(fixture: &Value) {
    let width = fixture["width"].as_u64().unwrap() as usize;
    let height = fixture["height"].as_u64().unwrap() as usize;
    let levels = fixture["levels"].as_u64().unwrap() as usize;
    let max = (1_i32 << fixture["bits"].as_u64().unwrap()) - 1;
    let mut image = ManagedImage::<T>::new(width, height);
    for y in 0..height {
        for x in 0..width {
            let value = if fixture["constant"].as_bool().unwrap() {
                max
            } else {
                ((x * x * 13 + y * y * 17 + x * y * 5 + 100) % (max as usize + 1)) as i32
            };
            image.data_mut()[y * width + x] = T::from_filtered(value);
        }
    }
    let pyramid = ManagedImagePyr::new(&image, levels);
    let mipmap = pyramid.mipmap();
    let actual: Vec<i32> = (0..mipmap.h)
        .flat_map(|row| mipmap.row(row).iter().map(|&v| v.into()))
        .collect();
    let expected: Vec<i32> = serde_json::from_value(fixture["mipmap"].clone()).unwrap();
    assert_eq!(actual, expected);
    for level in 0..=levels {
        let expected: [usize; 2] =
            serde_json::from_value(fixture["offsets"][level].clone()).unwrap();
        assert_eq!(pyramid.lvl_offset(level), expected);
        assert_eq!(
            (pyramid.lvl(level).w, pyramid.lvl(level).h),
            (width >> level, height >> level)
        );
    }
}

#[test]
fn official_cpp_gaussian_mipmap_reference() {
    let fixtures: Value = serde_json::from_str(include_str!("reference/image_pyr.json")).unwrap();
    for fixture in fixtures.as_array().unwrap() {
        match fixture["bits"].as_u64().unwrap() {
            8 => compare::<u8>(fixture),
            16 => compare::<u16>(fixture),
            _ => panic!("unsupported oracle pixel type"),
        }
    }
}

#[test]
fn replacement_updates_mipmap_dimensions_and_pixels() {
    let mut source = ManagedImage::<u16>::new(8, 8);
    source.data_mut().fill(100);
    let mut pyramid = ManagedImagePyr::new(&source, 2);
    let mut replacement = ManagedImage::<u16>::new(7, 9);
    replacement.data_mut().fill(500);
    pyramid.set_from_image(&replacement, 2);
    assert_eq!((pyramid.mipmap().w, pyramid.mipmap().h), (10, 9));
    for level in 0..=2 {
        let image = pyramid.lvl(level);
        assert!(image.accumulate(true, |valid, &pixel| valid && pixel == 500));
    }
}

proptest! {
    #[test]
    fn separable_filter_matches_direct_integer_convolution(
        (width, height, pixels) in (3_usize..32, 3_usize..32)
            .prop_flat_map(|(width, height)| (Just(width), Just(height),
                proptest::collection::vec(any::<u16>(), width * height)))
    ) {
        let mut image = ManagedImage::<u16>::new(width, height);
        image.data_mut().copy_from_slice(&pixels);
        let pyramid = ManagedImagePyr::new(&image, 1);
        let filtered = pyramid.lvl(1);
        let weights = [1_u64, 4, 6, 4, 1];
        // Periodic reflection gives an independent definition of BORDER_REFLECT_101.
        let reflect = |coordinate: isize, size: usize| {
            let period = 2 * (size as isize - 1);
            let wrapped = coordinate.rem_euclid(period);
            wrapped.min(period - wrapped) as usize
        };
        for y in 0..height / 2 {
            for x in 0..width / 2 {
                let mut sum = 0_u64;
                for (ky, &wy) in weights.iter().enumerate() {
                    for (kx, &wx) in weights.iter().enumerate() {
                        let sx = reflect((2 * x + kx) as isize - 2, width);
                        let sy = reflect((2 * y + ky) as isize - 2, height);
                        sum += wx * wy * u64::from(pixels[sy * width + sx]);
                    }
                }
                prop_assert_eq!(u64::from(*filtered.get(x, y)), (sum + 128) / 256);
            }
        }
    }
}
