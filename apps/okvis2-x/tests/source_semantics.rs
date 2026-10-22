use firefly_okvis_x::{
    cameras::{
        camera_base::{CameraBase, ProjectionStatus},
        distortion_base::DistortionBase,
        equidistant_distortion::EquidistantDistortion,
        eucm_camera::EucmCamera,
        no_distortion::NoDistortion,
        pinhole_camera::PinholeCamera,
        radial_tangential_distortion::RadialTangentialDistortion,
        radial_tangential_distortion8::RadialTangentialDistortion8,
    },
    config::vi_parameters_reader::ViParametersReader,
};
use nalgebra::{DVector, Matrix2, Matrix2xX, Matrix3xX, SMatrix, Vector2, Vector3};

#[test]
fn mapping_factor_options_preserve_flat_toml_keys_and_defaults() {
    use firefly_okvis_x::mapping::config_mapping::SubMapConfig;

    for map_to_map in [false, true] {
        for map_to_live in [false, true] {
            let config = SubMapConfig::from_toml(&format!(
                "[general]\nuse_map_to_map_factors = {map_to_map}\nuse_map_to_live_factors = {map_to_live}\n"
            ))
            .unwrap();
            assert_eq!(config.factors.use_map2map_factors, map_to_map);
            assert_eq!(config.factors.use_map2live_factors, map_to_live);
            assert!(
                config
                    .to_string()
                    .contains(&format!("use_map_to_map_factors: {map_to_map}"))
            );
            assert!(
                config
                    .to_string()
                    .contains(&format!("use_map_to_live_factors: {map_to_live}"))
            );
        }
    }
    let partial = SubMapConfig::from_toml("[general]\nuse_map_to_live_factors = true\n").unwrap();
    assert!(!partial.factors.use_map2map_factors);
    assert!(partial.factors.use_map2live_factors);
    let absent = SubMapConfig::from_toml("").unwrap();
    assert!(!absent.factors.use_map2map_factors);
    assert!(!absent.factors.use_map2live_factors);
}

#[test]
fn camera_mapping_flags_accept_native_toml_booleans() {
    for (value, expected) in [
        ("true", true),
        ("false", false),
        ("\"true\"", true),
        ("\"True\"", false),
    ] {
        let document: toml::Value = toml::from_str(&format!(
            r#"
[[cameras]]
T_SC = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
image_dimension = [640,480]
distortion_coefficients = [0.0,0.0,0.0,0.0]
focal_length = [320.0,320.0]
principal_point = [320.0,240.0]
distortion_type = "radialtangential"
cam_model = "pinhole"
camera_type = "gray+depth"
slam_use = "okvis"
mapping = {value}
mapping_rectification = {value}
"#
        ))
        .unwrap();
        let cameras = ViParametersReader::get_camera_calibration(&document).unwrap();
        assert_eq!(cameras[0].camera_type.is_used_mapping, expected);
        assert_eq!(cameras[0].camera_type.depth_type.need_rectify, expected);
    }
}

#[test]
fn distortion_external_parameter_errors_return_without_panicking() {
    let models: Vec<Box<dyn DistortionBase>> = vec![
        Box::new(EquidistantDistortion::test_object()),
        Box::new(RadialTangentialDistortion::test_object()),
        Box::new(RadialTangentialDistortion8::test_object()),
    ];
    for model in models {
        let mut output = Vector2::new(91.0, 92.0);
        let mut jacobian = Matrix2::zeros();
        let mut parameters = DVector::zeros(0);
        model.get_parameters(&mut parameters);
        assert!(!model.distort_with_external_parameters(
            &Vector2::new(0.1, 0.2),
            &parameters,
            &mut output,
            None,
            None
        ));
        assert_eq!(output, Vector2::new(91.0, 92.0));
        let short = DVector::zeros(model.num_distortion_intrinsics() - 1);
        assert!(!model.distort_with_external_parameters(
            &Vector2::new(0.1, 0.2),
            &short,
            &mut output,
            Some(&mut jacobian),
            None
        ));
    }
    let mut output = Vector2::zeros();
    assert!(
        !RadialTangentialDistortion8::test_object().undistort(&Vector2::new(4.0, 0.0), &mut output)
    );
}

#[test]
fn projection_jacobians_match_independent_central_differences() {
    let models: Vec<Box<dyn CameraBase>> = vec![
        Box::new(EucmCamera::test_object()),
        Box::new(PinholeCamera::<RadialTangentialDistortion8>::test_object()),
    ];
    let point = Vector3::new(0.2, -0.15, 2.0);
    for camera in models {
        let mut output = Vector2::zeros();
        let mut jacobian = SMatrix::<f64, 2, 3>::zeros();
        assert_eq!(
            camera.project_with_jacobians(&point, &mut output, &mut jacobian, None),
            ProjectionStatus::Successful
        );
        for axis in 0..3 {
            let mut a = point;
            let mut b = point;
            a[axis] += 1e-6;
            b[axis] -= 1e-6;
            let mut pa = Vector2::zeros();
            let mut pb = Vector2::zeros();
            assert_eq!(camera.project(&a, &mut pa), ProjectionStatus::Successful);
            assert_eq!(camera.project(&b, &mut pb), ProjectionStatus::Successful);
            assert!(((pa - pb) / 2e-6 - jacobian.column(axis)).norm() < 1e-6);
        }
        assert_eq!(
            camera.project_with_external_parameters(
                &point,
                &DVector::zeros(0),
                &mut output,
                Some(&mut jacobian),
                None
            ),
            ProjectionStatus::Invalid
        );
    }
}

#[test]
fn eucm_batch_projection_preserves_order_and_status_append() {
    let camera = EucmCamera::test_object();
    let points =
        Matrix3xX::from_columns(&[Vector3::new(0.1, 0.2, 2.0), Vector3::new(-0.3, 0.1, 1.0)]);
    let mut images = Matrix2xX::zeros(2);
    let mut statuses = vec![ProjectionStatus::Invalid];
    camera.project_batch(&points, &mut images, Some(&mut statuses));
    assert_eq!(
        statuses,
        vec![
            ProjectionStatus::Invalid,
            ProjectionStatus::Successful,
            ProjectionStatus::Successful
        ]
    );
    let with_status = images.clone();
    camera.project_batch(&points, &mut images, None);
    assert_eq!(images, with_status);
    for i in 0..2 {
        let mut scalar = Vector2::zeros();
        camera.project(&points.column(i).into_owned(), &mut scalar);
        assert_eq!(images.column(i), scalar);
    }
}

#[test]
fn pinhole_no_distortion_has_analytic_projection() {
    let camera = PinholeCamera::new((640, 480), (320.0, 300.0, 320.0, 240.0), NoDistortion, 0);
    let mut image = Vector2::zeros();
    assert_eq!(
        camera.project(&Vector3::new(1.0, 2.0, 4.0), &mut image),
        ProjectionStatus::Successful
    );
    assert_eq!(image, Vector2::new(400.0, 390.0));
}

#[test]
fn unchanged_gray_alpha_png_expands_bgra_like_opencv() {
    use firefly_okvis_x::{
        application::RustImageCodec,
        datasets::dataset_reader::{ImageCodec, ImageReadMode},
    };
    use image::{ImageBuffer, LumaA};
    use purecv::core::DynamicData;
    let directory =
        std::env::temp_dir().join(format!("okvis-alpha-semantics-{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();
    let byte_path = directory.join("gray-alpha8.png");
    ImageBuffer::<LumaA<u8>, _>::from_raw(2, 1, vec![17, 23, 240, 128])
        .unwrap()
        .save(&byte_path)
        .unwrap();
    let DynamicData::U8(byte) = RustImageCodec
        .read(&byte_path, ImageReadMode::Unchanged)
        .unwrap()
        .data
    else {
        panic!("expected u8");
    };
    assert_eq!(byte.channels, 4);
    assert_eq!(byte.data, vec![17, 17, 17, 23, 240, 240, 240, 128]);
    let short_path = directory.join("gray-alpha16.png");
    ImageBuffer::<LumaA<u16>, _>::from_raw(2, 1, vec![1000, 2000, 32768, 65535])
        .unwrap()
        .save(&short_path)
        .unwrap();
    let DynamicData::U16(short) = RustImageCodec
        .read(&short_path, ImageReadMode::Unchanged)
        .unwrap()
        .data
    else {
        panic!("expected u16");
    };
    assert_eq!(short.channels, 4);
    assert_eq!(
        short.data,
        vec![1000, 1000, 1000, 2000, 32768, 32768, 32768, 65535]
    );
    std::fs::remove_dir_all(directory).unwrap();
}
