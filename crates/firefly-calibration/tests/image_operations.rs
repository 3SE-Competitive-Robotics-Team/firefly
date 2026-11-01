#![allow(clippy::float_cmp, clippy::cast_possible_truncation)]
use firefly_calibration::image::image::Image;
use firefly_calibration::image::image::ManagedImage;
use nalgebra::Vector2;
use serde_json::Value;

fn number(value: &Value) -> f64 {
    if value.as_str() == Some("NaN") {
        f64::NAN
    } else {
        value.as_f64().unwrap()
    }
}

#[test]
fn official_cpp_mixed_precision_and_nan_reductions() {
    let fixture: Value =
        serde_json::from_str(include_str!("reference/image_operations.json")).unwrap();
    let mixed = &fixture["mixed"];
    let pixels: Vec<f64> = mixed["pixels_bits"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| f64::from_bits(v.as_u64().unwrap()))
        .collect();
    let image = Image::new(&pixels, 5, 5, 6);
    let p = Vector2::new(
        number(&mixed["position"][0]) as f32,
        number(&mixed["position"][1]) as f32,
    );
    assert_eq!(f64::from(image.interp(&p)), number(&mixed["interp"]));
    for (actual, key) in [
        (image.interp_grad(&p), "gradient"),
        (image.interp_grad_bilinear_exact(&p), "exact_gradient"),
    ] {
        for i in 0..3 {
            assert_eq!(f64::from(actual[i]), number(&mixed[key][i]), "{key}[{i}]");
        }
    }
    for case in fixture["min_max"].as_array().unwrap() {
        let pixels: Vec<f64> = case["pixels"]
            .as_array()
            .unwrap()
            .iter()
            .map(number)
            .collect();
        let (min, max) = Image::new(&pixels, pixels.len(), 1, pixels.len())
            .min_max()
            .unwrap();
        for (actual, key) in [(min, "min"), (max, "max")] {
            let expected = number(&case[key]);
            if expected.is_nan() {
                assert!(actual.is_nan());
            } else {
                assert_eq!(actual, expected);
            }
            assert_eq!(
                actual.is_sign_negative(),
                case[format!("{key}_sign")].as_bool().unwrap()
            );
        }
    }
}

#[test]
fn pitched_image_operations_preserve_padding_and_cast_semantics() {
    let mut image = ManagedImage::<u16>::with_stride(2, 2, 4);
    image.view_mut().memset(0x12);
    assert_eq!(image.data(), &[0x1212; 8]);
    image.view_mut().fill(10);
    image.view_mut().col(1).fill(20);
    assert_eq!(
        image.data(),
        &[10, 20, 0x1212, 0x1212, 10, 20, 0x1212, 0x1212]
    );
    assert_eq!(image.view().sum(), 60);
    assert_eq!(image.view().mean::<f64>(), 15.);
    assert_eq!(
        image.view().col(1).pixels().copied().collect::<Vec<_>>(),
        vec![20, 20]
    );
    let source = Image::new(&[-1.9_f64, 2.9, 3.1, -4.1], 2, 2, 2);
    let mut converted = ManagedImage::<i16>::with_stride(2, 2, 3);
    converted.data_mut().fill(77);
    converted.convert_from(source);
    assert_eq!(converted.data(), &[-1, 2, 77, 3, -4, 77]);
    let mut subtract = ManagedImage::<i16>::new(2, 2);
    subtract.view_mut().fill(1);
    converted -= &subtract;
    assert_eq!(converted.data(), &[-2, 1, 77, 2, -5, 77]);
    converted.deallocate();
    assert_eq!(converted.view().area(), 0);
    assert_eq!(converted.data(), [] as [i16; 0]);
}
