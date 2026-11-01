use firefly_basalt::io::dataset_io::DatasetIoFactory;
use std::path::Path;

struct Hash(u64);
impl Hash {
    fn new() -> Self {
        Self(14_695_981_039_346_656_037)
    }
    fn integer(&mut self, value: u64) {
        for byte in value.to_le_bytes() {
            self.0 ^= u64::from(byte);
            self.0 = self.0.wrapping_mul(1_099_511_628_211);
        }
    }
    fn scalar(&mut self, value: f64) {
        self.integer(value.to_bits());
    }
}

#[test]
#[ignore = "Requires real TUMVI room1 bag selected with BASALT_DATASET_BAG"]
fn official_tumvi_complete_metadata_and_sampled_images() {
    let path = std::env::var("BASALT_DATASET_BAG").expect("BASALT_DATASET_BAG");
    let reference: serde_json::Value =
        serde_json::from_str(include_str!("reference/dataset_rosbag.json")).unwrap();
    compare(Path::new(&path), &reference);
}

#[test]
fn official_compressed_bags_and_pixel_boundaries() {
    for mode in ["none", "bz2", "lz4"] {
        let base = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/reference");
        let reference: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(base.join(format!("dataset_rosbag_{mode}.json"))).unwrap(),
        )
        .unwrap();
        compare(&base.join(format!("dataset_rosbag_{mode}.bag")), &reference);
    }
}

fn compare(path: &Path, reference: &serde_json::Value) {
    let mut reader = DatasetIoFactory::get_dataset_io("bag", false).unwrap();
    reader.read(path).unwrap();
    let data = reader.data().unwrap();
    let mut data = data.lock().unwrap();
    assert_eq!(
        data.num_cams() as u64,
        reference["cameras"].as_u64().unwrap()
    );
    assert_eq!(
        data.mocap_to_imu_offset_ns(),
        reference["offset"].as_i64().unwrap()
    );
    let timestamps: Vec<i64> = serde_json::from_value(reference["timestamps"].clone()).unwrap();
    assert_eq!(*data.image_timestamps(), timestamps);
    for (name, samples) in [
        (
            "gyro",
            data.gyro_data()
                .iter()
                .map(|v| (v.timestamp_ns, v.data))
                .collect::<Vec<_>>(),
        ),
        (
            "accel",
            data.accel_data()
                .iter()
                .map(|v| (v.timestamp_ns, v.data))
                .collect(),
        ),
    ] {
        let expected = &reference[name];
        assert_eq!(samples.len() as u64, expected["count"].as_u64().unwrap());
        let mut hash = Hash::new();
        for (timestamp, value) in &samples {
            hash.integer(*timestamp as u64);
            for x in value.iter() {
                hash.scalar(*x);
            }
        }
        assert_eq!(
            hash.0,
            expected["hash"].as_u64().unwrap(),
            "{name} full-stream bit hash"
        );
        for row in expected["samples"].as_array().unwrap() {
            let (timestamp, value) = samples[row[0].as_u64().unwrap() as usize];
            assert_eq!(timestamp, row[1].as_i64().unwrap());
            for k in 0..3 {
                assert_eq!(value[k], row[k + 2].as_f64().unwrap());
            }
        }
    }
    let mut hash = Hash::new();
    assert_eq!(
        data.gt_pose_data().len() as u64,
        reference["gt"]["count"].as_u64().unwrap()
    );
    for (timestamp, pose) in data.gt_timestamps().iter().zip(data.gt_pose_data()) {
        hash.integer(*timestamp as u64);
        let t = pose.translation.vector;
        let q = pose.rotation.quaternion();
        for value in [t.x, t.y, t.z, q.i, q.j, q.k, q.w] {
            hash.scalar(value);
        }
    }
    assert_eq!(
        hash.0,
        reference["gt"]["hash"].as_u64().unwrap(),
        "GT full-stream bit hash"
    );
    for sample in reference["gt"]["samples"].as_array().unwrap() {
        let index = sample[0].as_u64().unwrap() as usize;
        assert_eq!(data.gt_timestamps()[index], sample[1].as_i64().unwrap());
        let pose = &data.gt_pose_data()[index];
        let t = pose.translation.vector;
        let q = pose.rotation.quaternion();
        for (column, value) in [t.x, t.y, t.z, q.i, q.j, q.k, q.w].into_iter().enumerate() {
            assert_eq!(value, sample[column + 2].as_f64().unwrap());
        }
    }
    compare_images(&mut *data, reference);
}

fn compare_images(
    data: &mut dyn firefly_basalt::io::dataset_io::VioDataset,
    reference: &serde_json::Value,
) {
    for frame in reference["images"].as_array().unwrap() {
        let timestamp = frame["timestamp"].as_i64().unwrap();
        let actual = data.image_data(timestamp).unwrap();
        let images = frame["images"].as_array().unwrap();
        assert_eq!(actual.len(), images.len());
        for (image, expected) in actual.iter().zip(images) {
            if expected.is_null() {
                assert!(image.img.is_none());
                continue;
            }
            let pixels = image.img.as_ref().unwrap();
            assert_eq!(pixels.w as u64, expected["width"].as_u64().unwrap());
            assert_eq!(pixels.h as u64, expected["height"].as_u64().unwrap());
            assert_eq!(image.exposure, expected["exposure"].as_f64().unwrap());
            let mut hash = Hash::new();
            for &pixel in pixels.data() {
                hash.integer(u64::from(pixel));
            }
            assert_eq!(
                hash.0,
                expected["hash"].as_u64().unwrap(),
                "image at {timestamp}"
            );
            assert_eq!(
                u64::from(*pixels.data().iter().min().unwrap()),
                expected["minimum"].as_u64().unwrap()
            );
            assert_eq!(
                u64::from(*pixels.data().iter().max().unwrap()),
                expected["maximum"].as_u64().unwrap()
            );
            for sample in expected["pixels"].as_array().unwrap() {
                let x = sample[0].as_u64().unwrap() as usize;
                let y = sample[1].as_u64().unwrap() as usize;
                assert_eq!(
                    u64::from(pixels.data()[y * pixels.w + x]),
                    sample[2].as_u64().unwrap()
                );
            }
        }
    }
}
