use firefly_basalt::io::{
    dataset_io::DatasetIoInterface, dataset_io_euroc::EurocIo, dataset_io_kitti::KittiIo,
    dataset_io_uzh::UzhIo,
};
use std::{
    path::{Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
};

struct Fixture(PathBuf);
impl Fixture {
    fn new() -> Self {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../logs")
            .join(format!(
                "basalt-dataset-test-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
        std::fs::create_dir_all(&path).unwrap();
        Self(path)
    }
    fn text(&self, relative: &str, content: &str) {
        let path = self.0.join(relative);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, content).unwrap();
    }
    fn gray(&self, relative: &str) {
        let path = self.0.join(relative);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        image::GrayImage::from_raw(2, 1, vec![1, 255])
            .unwrap()
            .save(path)
            .unwrap();
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.0).unwrap();
    }
}
fn cpp_reference(kind: &str, fixture: &Fixture, reader: &dyn DatasetIoInterface) {
    let Ok(binary) = std::env::var("BASALT_DATASET_REFERENCE") else {
        return;
    };
    let output = std::process::Command::new(binary)
        .arg(kind)
        .arg(&fixture.0)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let stdout = String::from_utf8(output.stdout).unwrap();
    let expected: serde_json::Value = serde_json::from_str(
        stdout
            .lines()
            .find_map(|s| s.strip_prefix("REFERENCE_JSON="))
            .unwrap(),
    )
    .unwrap();
    let shared = reader.data().unwrap();
    let mut data = shared.lock().unwrap();
    let timestamps = data.image_timestamps().clone();
    let gyro: Vec<_> = data
        .gyro_data()
        .iter()
        .map(|m| serde_json::json!([m.timestamp_ns, m.data.x, m.data.y, m.data.z]))
        .collect();
    let accel: Vec<_> = data
        .accel_data()
        .iter()
        .map(|m| serde_json::json!([m.timestamp_ns, m.data.x, m.data.y, m.data.z]))
        .collect();
    let poses: Vec<_> = data
        .gt_pose_data()
        .iter()
        .map(|p| {
            let t = p.translation.vector;
            let q = p.rotation.quaternion();
            serde_json::json!([t.x, t.y, t.z, q.i, q.j, q.k, q.w])
        })
        .collect();
    let images:Vec<_>=timestamps.iter().map(|t|data.image_data(*t).unwrap().iter().map(|im|im.img.as_ref().map_or(serde_json::Value::Null,|pixels|serde_json::json!({"width":pixels.w,"height":pixels.h,"pixels":pixels.data(),"exposure":im.exposure}))).collect::<Vec<_>>()).collect();
    assert_eq!(
        serde_json::json!({"timestamps":timestamps,"gt_timestamps":data.gt_timestamps(),"gyro":gyro,"accel":accel,"poses":poses,"images":images}),
        expected
    );
}
#[test]
fn euroc_loads_stereo_imu_exposure_and_selects_groundtruth() {
    let f = Fixture::new();
    f.text(
        "mav0/cam0/data.csv",
        "#timestamp,filename\n100,a.png\n200,b.png\n",
    );
    f.text("mav0/imu0/data.csv", "90,1,2,3,4,5,6\n");
    f.text(
        "mav0/state_groundtruth_estimate0/data.csv",
        "100,1,2,3,1,0,0,0,9,9,9\n",
    );
    f.text("mav0/mocap0/data.csv", "110,4,5,6,1,0,0,0\n");
    f.text("mav0/cam0/exposure.csv", "100,5000000\n");
    f.gray("mav0/cam0/data/a.png");
    f.gray("mav0/cam0/data/b.png");
    std::fs::create_dir_all(f.0.join("mav0/cam1/data")).unwrap();
    image::ImageBuffer::<image::Luma<u16>, _>::from_raw(2, 1, vec![123_u16, 65535])
        .unwrap()
        .save(f.0.join("mav0/cam1/data/a.png"))
        .unwrap();
    let mut reader = EurocIo::new(false);
    reader.read(&f.0).unwrap();
    let shared = reader.data().unwrap();
    let mut data = shared.lock().unwrap();
    assert_eq!(data.image_timestamps(), &[100, 200]);
    assert_eq!(data.gyro_data()[0].data.as_slice(), &[1., 2., 3.]);
    assert_eq!(data.accel_data()[0].data.as_slice(), &[4., 5., 6.]);
    assert_eq!(data.gt_timestamps(), &[100]);
    let imgs = data.image_data(100).unwrap();
    assert_eq!(imgs[0].img.as_ref().unwrap().data(), &[256, 65280]);
    assert_eq!(imgs[1].img.as_ref().unwrap().data(), &[123, 65535]);
    assert_eq!(imgs[0].exposure, 0.005);
    assert!(data.image_data(200).unwrap()[1].img.is_none());
    drop(data);
    cpp_reference("euroc", &f, &reader);
    reader.reset();
    assert!(reader.data().is_none());
    let mut mocap = EurocIo::new(true);
    mocap.read(&f.0).unwrap();
    cpp_reference("mocap", &f, &mocap);
    assert_eq!(
        mocap.data().unwrap().lock().unwrap().gt_timestamps(),
        &[110]
    );
}

#[test]
fn asl_preserves_camera_filenames_and_rejects_unpaired_timestamps() {
    let fixture = Fixture::new();
    fixture.text("aria/cam0/data.csv", "100,left.png\n");
    fixture.text("aria/cam1/data.csv", "100,right.png\n");
    fixture.text("aria/imu0/data.csv", "90,1,2,3,4,5,6\n110,2,3,4,5,6,7\n");
    fixture.gray("aria/cam0/data/left.png");
    fixture.gray("aria/cam1/data/right.png");
    let mut reader = EurocIo::new(false);
    reader.read_asl(&fixture.0.join("aria")).unwrap();
    let shared = reader.data().unwrap();
    let mut data = shared.lock().unwrap();
    assert_eq!(data.image_timestamps(), &[100]);
    assert!(
        data.image_data(100)
            .unwrap()
            .iter()
            .all(|image| image.img.is_some())
    );
    assert_eq!(data.gt_timestamps(), &[] as &[i64]);
    drop(data);
    reader.reset();
    fixture.text("aria/cam1/data.csv", "101,right.png\n");
    assert!(reader.read_asl(&fixture.0.join("aria")).is_err());
    assert!(reader.data().is_none());
}
#[test]
fn uzh_preserves_blue_channel_seconds_and_xyzw_quaternion() {
    let f = Fixture::new();
    f.text("left_images.txt", "0 1.25 left.png\n");
    f.text("right_images.txt", "0 1.25 right.png\n");
    f.text("imu.txt", "0 1.2 1 2 3 4 5 6\n");
    f.text("groundtruth.txt", "0 1.25 7 8 9 0 0 1 0\n");
    image::RgbImage::from_raw(1, 1, vec![11, 22, 33])
        .unwrap()
        .save(f.0.join("left.png"))
        .unwrap();
    f.gray("right.png");
    let mut reader = UzhIo::default();
    reader.read(&f.0).unwrap();
    let shared = reader.data().unwrap();
    let mut data = shared.lock().unwrap();
    assert_eq!(data.image_timestamps(), &[1_250_000_000]);
    assert_eq!(data.gyro_data()[0].timestamp_ns, 1_200_000_000);
    assert_eq!(
        data.image_data(1_250_000_000).unwrap()[0]
            .img
            .as_ref()
            .unwrap()
            .data(),
        &[8448]
    );
    assert_eq!(
        data.gt_pose_data()[0].translation.vector.as_slice(),
        &[7., 8., 9.]
    );
    assert_eq!(data.gt_pose_data()[0].rotation.quaternion().k.abs(), 1.);
    drop(data);
    cpp_reference("uzh", &f, &reader);
    f.text("groundtruth.txt", "1.25 7 8 9 0 0 1 0\n");
    reader.read(&f.0).unwrap();
    let shared = reader.data().unwrap();
    let data = shared.lock().unwrap();
    assert_eq!(data.gt_timestamps(), &[1_250_000_000]);
    assert_eq!(
        data.gt_pose_data()[0].translation.vector.as_slice(),
        &[7., 8., 9.]
    );
    assert_eq!(data.gt_pose_data()[0].rotation.quaternion().k.abs(), 1.);
}
#[test]
fn uzh_rejects_unpaired_and_duplicate_images_before_loading_pixels() {
    let f = Fixture::new();
    f.text("left_images.txt", "0 1.25 left.png\n");
    f.text("right_images.txt", "0 1.26 right.png\n");
    let mut reader = UzhIo::default();
    assert!(reader.read(&f.0).is_err());
    assert!(reader.data().is_none());
    f.text(
        "right_images.txt",
        "0 1.25 right.png\n1 1.25 duplicate.png\n",
    );
    assert!(reader.read(&f.0).is_err());
    assert!(reader.data().is_none());
}
#[test]
fn kitti_reads_indexed_images_and_row_major_poses() {
    let f = Fixture::new();
    f.text("times.txt", "0.0\n0.1\n");
    f.text("poses.txt", "1 0 0 2 0 1 0 3 0 0 1 4\n");
    f.gray("image_0/000000.png");
    f.gray("image_1/000000.png");
    let mut reader = KittiIo::default();
    reader.read(&f.0).unwrap();
    let shared = reader.data().unwrap();
    let mut data = shared.lock().unwrap();
    assert_eq!(data.image_timestamps(), &[0, 100_000_000]);
    assert_eq!(
        data.gt_pose_data()[0].translation.vector.as_slice(),
        &[2., 3., 4.]
    );
    assert_eq!(
        data.image_data(0).unwrap()[0].img.as_ref().unwrap().data(),
        &[256, 65280]
    );
    assert!(data.gyro_data().is_empty());
    assert!(data.image_data(100_000_000).unwrap()[0].img.is_none());
    drop(data);
    cpp_reference("kitti", &f, &reader);
}
