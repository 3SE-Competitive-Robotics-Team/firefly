use firefly_calibration::image::image::Image;
use firefly_calibration::image::image::ImageScalar;
use firefly_calibration::image::image::ManagedImage;
use nalgebra::Vector2;
use serde_json::Value;

fn close(actual: f64, expected: &Value, tolerance: f64) {
    let expected = expected.as_f64().unwrap();
    assert!(
        (actual - expected).abs() <= tolerance * expected.abs().max(1.0),
        "{actual} != {expected}"
    );
}

fn compare<S: ImageScalar>(fixture: &Value, tolerance: f64) {
    let pixels: Vec<u16> = serde_json::from_value(fixture["pixels"].clone()).unwrap();
    let image = Image::new(&pixels, 7, 6, 10);
    for sample in fixture["samples"].as_array().unwrap() {
        let position = Vector2::new(
            S::from_f64(sample["position"][0].as_f64().unwrap()).unwrap(),
            S::from_f64(sample["position"][1].as_f64().unwrap()).unwrap(),
        );
        close(
            image.interp(&position).as_f64(),
            &sample["interp"],
            tolerance,
        );
        close(
            image.interp_cubic_splines(&position),
            &sample["cubic"],
            tolerance,
        );
        for (actual, key) in [
            (image.interp_grad(&position), "central_gradient"),
            (
                image.interp_grad_bilinear_exact(&position),
                "exact_gradient",
            ),
        ] {
            for axis in 0..3 {
                close(actual[axis].as_f64(), &sample[key][axis], tolerance);
            }
        }
        if sample["cubic_gradient"].is_array() {
            let actual = image.interp_grad_cubic_splines(&position);
            for axis in 0..3 {
                close(
                    actual[axis].as_f64(),
                    &sample["cubic_gradient"][axis],
                    tolerance,
                );
            }
        }
    }
    for sample in fixture["borders"].as_array().unwrap() {
        let position = Vector2::new(
            S::from_f64(sample["position"][0].as_f64().unwrap()).unwrap(),
            S::from_f64(sample["position"][1].as_f64().unwrap()).unwrap(),
        );
        close(
            image.interp_cubic_splines(&position),
            &sample["cubic"],
            tolerance,
        );
        for border in 0..3 {
            assert_eq!(
                image.in_bounds(&position, S::from_usize(border).unwrap()),
                sample[format!("in_bounds_{border}")].as_bool().unwrap()
            );
        }
        if sample["cubic_gradient"].is_array() {
            let actual = image.interp_grad_cubic_splines(&position);
            for axis in 0..3 {
                close(
                    actual[axis].as_f64(),
                    &sample["cubic_gradient"][axis],
                    tolerance,
                );
            }
        }
    }
}

#[test]
fn official_cpp_interpolation_reference() {
    let fixtures: Value = serde_json::from_str(include_str!("reference/image.json")).unwrap();
    compare::<f32>(&fixtures[0], 2e-6);
    compare::<f64>(&fixtures[1], 2e-14);
}

#[test]
fn pitched_views_exclude_padding_and_mutate_parent() {
    let mut image = ManagedImage::<u16>::with_stride(4, 3, 7);
    image.data_mut().fill(u16::MAX);
    image.view_mut().fill(5);
    image.view_mut().sub_image(1, 1, 2, 2).fill(9);
    assert_eq!(
        image
            .view()
            .accumulate(0_u32, |sum, &value| sum + u32::from(value)),
        76
    );
    assert_eq!(image.view().min_max(), Some((5, 9)));
    assert!(
        image
            .data()
            .chunks(7)
            .all(|row| row[4..].iter().all(|&v| v == u16::MAX))
    );
    let mut copy = ManagedImage::<u16>::new(0, 0);
    copy.copy_from(image.view().sub_image(1, 1, 2, 2));
    assert_eq!(copy.data(), &[9, 9, 9, 9]);
}

#[test]
fn nonfinite_coordinates_are_outside() {
    let pixels = [0_u16; 16];
    let image = Image::new(&pixels, 4, 4, 4);
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(!image.in_bounds(&Vector2::new(value, 1.0), 0.0));
        assert!(!image.in_bounds(&Vector2::new(1.0, value), 0.0));
    }
}
