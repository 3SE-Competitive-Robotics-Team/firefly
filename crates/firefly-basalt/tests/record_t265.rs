use firefly_basalt::{
    device::rs_t265::RsPoseData,
    io::dataset_io::ImageData,
    optical_flow::OpticalFlowInput,
    record_t265::{RecordingSession, encode_image, scalar, utc_date},
};
use firefly_basalt_headers::{
    image::image::ManagedImage, imu::imu_types::ImuData, utils::sophus_utils::so3_exp,
};
use nalgebra::{Isometry3, Translation3, Vector3};
use std::{
    path::{Path, PathBuf},
    sync::Arc,
};
struct Directory(PathBuf);
impl Directory {
    fn new(name: &str) -> Self {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../logs")
            .join(format!("basalt-record-{name}-{}", std::process::id()));
        std::fs::create_dir_all(&path).unwrap();
        Self(path)
    }
}
impl Drop for Directory {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.0).unwrap();
    }
}
fn image(camera: usize) -> ManagedImage<u16> {
    let mut image = ManagedImage::new(16, 16);
    for (i, pixel) in image.data_mut().iter_mut().enumerate() {
        *pixel = ((((i * 17 + camera * 31) % 256) << 8) | i) as u16;
    }
    image
}
fn fixture() -> serde_json::Value {
    serde_json::from_str(include_str!("reference/record_t265.json")).unwrap()
}
#[test]
fn original_webp_quality_and_csv_format() {
    assert_eq!(webpx::version(), (1, 6, 0));
    let expected = fixture();
    for quality in [0, 90, 100, 101] {
        let encoded = encode_image(&image(0), quality).unwrap();
        let original: Vec<u8> =
            serde_json::from_value(expected["images"][quality.to_string()]["bytes"].clone())
                .unwrap();
        assert_eq!(encoded, original, "Original OpenCV WebP quality {quality}");
    }
    for pair in expected["scalars"].as_array().unwrap() {
        assert_eq!(scalar(pair[0].as_f64().unwrap()), pair[1].as_str().unwrap());
    }
    assert_eq!(utc_date(0).unwrap(), "1970_01_01_00_00_00");
    assert_eq!(utc_date(951_782_400).unwrap(), "2000_02_29_00_00_00");
    assert!(utc_date(-1).is_err());
}

#[test]
fn concurrent_stop_preserves_every_accepted_sample() {
    let root = Directory::new("concurrent");
    let mut session = RecordingSession::start(&root.0, 3, true, None).unwrap();
    let input = session.inputs();
    let (ready_tx, ready_rx) = std::sync::mpsc::channel();
    let worker = std::thread::spawn(move || {
        let mut accepted = 0;
        for time in 0..50_000 {
            if !input.send_imu(Arc::new(ImuData {
                t_ns: time,
                ..Default::default()
            })) {
                break;
            }
            accepted += 1;
            if accepted == 10_001 {
                ready_tx.send(()).unwrap();
            }
        }
        accepted
    });
    ready_rx
        .recv_timeout(std::time::Duration::from_secs(10))
        .unwrap();
    session.stop().unwrap();
    let accepted = worker.join().unwrap();
    let output = std::fs::read_to_string(session.dataset_dir.join("mav0/imu0/data.csv")).unwrap();
    assert_eq!(output.lines().count(), accepted + 1);
    for (index, row) in output.lines().skip(1).enumerate() {
        assert_eq!(
            row.split(',').next().unwrap().parse::<usize>().unwrap(),
            index
        );
    }
}
#[test]
fn recording_drains_all_workers_and_matches_original_payload_files() {
    let root = Directory::new("drain");
    let mut session = RecordingSession::start(&root.0, 0, false, None).unwrap();
    let input = session.inputs();
    let frame = Arc::new(OpticalFlowInput {
        t_ns: 123_456_789,
        img_data: (0..2)
            .map(|camera| ImageData {
                img: Some(Arc::new(image(camera))),
                exposure: 0.00125 * (camera + 1) as f64,
            })
            .collect(),
    });
    assert!(input.send_image(Arc::clone(&frame)));
    assert!(input.send_imu(Arc::new(ImuData {
        t_ns: 123_456_780,
        gyro: Vector3::new(0.123_456_789, -0.000_012_345_678_9, 1_234_567.89),
        accel: Vector3::new(1., -0., 9.80665)
    })));
    assert!(input.send_pose(RsPoseData {
        t_ns: 123_456_781,
        data: Isometry3::from_parts(
            Translation3::new(0.123_456_789, 1_000_000.1, -0.000_001_234_567),
            so3_exp(&Vector3::new(0.1, -0.03, 0.02))
        )
    }));
    session.stop().unwrap();
    assert!(!input.send_image(frame));
    session.stop().unwrap();
    let expected = fixture();
    for (path, key) in [
        ("mav0/cam0/data.csv", "camera"),
        ("mav0/cam0/exposure.csv", "exposure"),
        ("mav0/imu0/data.csv", "imu"),
        ("mav0/realsense0/data.csv", "pose"),
    ] {
        let actual = std::fs::read_to_string(session.dataset_dir.join(path)).unwrap();
        assert_eq!(
            actual.lines().skip(1).collect::<Vec<_>>().join("\n") + "\n",
            expected[key].as_str().unwrap()
        );
    }
    for camera in 0..2 {
        assert!(
            session
                .dataset_dir
                .join(format!("mav0/cam{camera}/data/123456789.webp"))
                .exists()
        );
    }
    assert_eq!(session.imu_plot.try_recv().unwrap(), [1., -0., 9.80665_f32]);
}
#[test]
fn drop_drains_and_write_failure_is_reported_without_deadlocking() {
    let root = Directory::new("failure");
    let mut session = RecordingSession::start(&root.0, 1, true, None).unwrap();
    assert!(!session.needs_shutdown());
    assert!(!session.dataset_dir.join("mav0/realsense0").exists());
    let input = session.inputs();
    std::fs::remove_dir(session.dataset_dir.join("mav0/cam0/data")).unwrap();
    let frame = Arc::new(OpticalFlowInput {
        t_ns: 1,
        img_data: vec![
            ImageData {
                img: Some(Arc::new(image(0))),
                exposure: 0.001
            };
            2
        ],
    });
    assert!(input.send_image(frame));
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    while !session.needs_shutdown() {
        assert!(
            std::time::Instant::now() < deadline,
            "Write failure was not reported before stop"
        );
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    assert!(
        session.needs_shutdown(),
        "Status inspection must not consume the error"
    );
    assert!(session.stop().is_err());
    let session = RecordingSession::start(&root.0, 2, true, None).unwrap();
    let directory = session.dataset_dir.clone();
    let input = session.inputs();
    for time in 0..32 {
        assert!(input.send_imu(Arc::new(ImuData {
            t_ns: time,
            ..Default::default()
        })));
    }
    drop(session);
    assert_eq!(
        std::fs::read_to_string(directory.join("mav0/imu0/data.csv"))
            .unwrap()
            .lines()
            .count(),
        33
    );
    assert!(!input.send_imu(Arc::new(ImuData::default())));
}
