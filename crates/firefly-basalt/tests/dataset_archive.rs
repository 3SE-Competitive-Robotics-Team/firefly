use firefly_basalt::{
    io::dataset_io::{AccelData, GyroData},
    utils::common_types::CerealBinary,
};
use firefly_basalt_headers::image::image::ManagedImage;
use nalgebra::Vector3;

#[test]
#[cfg(all(target_endian = "little", target_pointer_width = "64"))]
fn official_cereal_native_binary_and_json() {
    fn compare<T: CerealBinary>(value: &T, expected: &str) -> Vec<u8> {
        let mut bytes = Vec::new();
        value.write_cereal_binary(&mut bytes).unwrap();
        let reference: Vec<u8> = expected
            .as_bytes()
            .as_chunks::<2>()
            .0
            .iter()
            .map(|p| u8::from_str_radix(std::str::from_utf8(p).unwrap(), 16).unwrap())
            .collect();
        assert_eq!(bytes, reference);
        bytes
    }
    let expected: serde_json::Value =
        serde_json::from_str(include_str!("reference/dataset_archive.json")).unwrap();
    let gyro = GyroData {
        timestamp_ns: -123,
        data: Vector3::new(1.25, -2.5, 3.75),
    };
    let accel = AccelData {
        timestamp_ns: 456,
        data: Vector3::new(-9.81, 0., 0.125),
    };
    let mut image = ManagedImage::<u8>::new(3, 2);
    image
        .data_mut()
        .copy_from_slice(&[0, 51, 102, 153, 204, 255]);
    let bytes = compare(&gyro, expected["gyro"].as_str().unwrap());
    let restored = GyroData::read_cereal_binary(&mut bytes.as_slice()).unwrap();
    assert_eq!(restored.timestamp_ns, gyro.timestamp_ns);
    assert_eq!(restored.data, gyro.data);
    let _ = compare(&accel, expected["accel"].as_str().unwrap());
    let bytes = compare(&image, expected["image"].as_str().unwrap());
    let restored = ManagedImage::<u8>::read_cereal_binary(&mut bytes.as_slice()).unwrap();
    assert_eq!(restored.data(), image.data());
    assert_eq!((restored.w, restored.h), (3, 2));
    assert!(ManagedImage::<u8>::read_cereal_binary(&mut &bytes[..bytes.len() - 1]).is_err());
    assert_eq!(
        serde_json::to_value(gyro).unwrap(),
        expected["json"]["value0"]
    );
    assert_eq!(
        serde_json::to_value(accel).unwrap(),
        expected["json"]["value1"]
    );
}
