use firefly_basalt::{
    io::dataset_io::DatasetIoFactory,
    optical_flow::{OpticalFlowFactory, OpticalFlowInput},
    utils::vio_config::VioConfig,
};
use firefly_basalt_headers::calibration::Calibration;
use serde::Deserialize;
use serde_json::{Value, json};
use std::{fs::File, io::BufReader, path::PathBuf, sync::Arc};

#[derive(Deserialize)]
struct CalibrationArchive {
    value0: Calibration<f64>,
}

/// Requires the official executable and external dataset; numeric artifacts are
/// written under the workspace's existing logs directory.
#[test]
#[ignore = "requires BASALT_SOURCE, BASALT_DATASET and BASALT_FLOW_REFERENCE"]
fn real_dataset_first_five_frames_match_official_tracker() {
    compare_frames(Some(5));
}

#[test]
#[ignore = "requires BASALT_SOURCE, BASALT_DATASET and BASALT_FLOW_REFERENCE"]
fn real_dataset_first_thirty_frames_match_official_tracker() {
    compare_frames(Some(30));
}

#[test]
#[ignore = "requires BASALT_SOURCE, BASALT_DATASET and BASALT_FLOW_REFERENCE"]
fn real_dataset_complete_matches_official_tracker() {
    compare_frames(None);
}

fn compare_frames(count: Option<usize>) {
    let root = PathBuf::from(std::env::var_os("BASALT_SOURCE").unwrap());
    let dataset = PathBuf::from(std::env::var_os("BASALT_DATASET").unwrap());
    let executable = PathBuf::from(std::env::var_os("BASALT_FLOW_REFERENCE").unwrap());
    let config_path = root.join("data/tumvi_512_config.json");
    let calib_path = root.join("data/tumvi_512_ds_calib.json");
    let logs = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../logs");
    let cpp_path = logs.join("basalt_optical_flow_cpp.json");
    let rust_path = logs.join("basalt_optical_flow_rust.json");
    let calib: CalibrationArchive =
        serde_json::from_reader(BufReader::new(File::open(&calib_path).unwrap())).unwrap();
    let mut config = VioConfig::default();
    config.load(&config_path).unwrap();
    let mut reader = DatasetIoFactory::get_dataset_io("bag", false).unwrap();
    reader.read(&dataset).unwrap();
    let data = reader.data().unwrap();
    let mut timestamps = data.lock().unwrap().image_timestamps().clone();
    if let Some(count) = count {
        assert!(count <= timestamps.len());
        timestamps.truncate(count);
    }
    assert!(
        std::process::Command::new(executable)
            .arg(&dataset)
            .arg(&config_path)
            .arg(&calib_path)
            .arg(&cpp_path)
            .arg(timestamps.len().to_string())
            .status()
            .unwrap()
            .success()
    );
    let flow = OpticalFlowFactory::get_optical_flow(&config, &calib.value0).unwrap();
    let mut frames = Vec::new();
    for timestamp in timestamps {
        let img_data = data.lock().unwrap().image_data(timestamp).unwrap();
        let images: Vec<_> = img_data
            .iter()
            .map(|camera| {
                let image = camera.img.as_ref().unwrap();
                let mut hash = 14_695_981_039_346_656_037_u64;
                for y in 0..image.h {
                    for x in 0..image.w {
                        hash ^= u64::from(image.data()[y * image.stride + x]);
                        hash = hash.wrapping_mul(1_099_511_628_211);
                    }
                }
                json!({"width":image.w,"height":image.h,"hash":hash,"exposure":camera.exposure})
            })
            .collect();
        flow.input_queue()
            .send(Some(Arc::new(OpticalFlowInput {
                t_ns: timestamp,
                img_data,
            })))
            .unwrap();
        let result = flow.output_queue().recv().unwrap().unwrap();
        let cameras: Vec<Vec<_>> = result
            .observations
            .iter()
            .map(|camera| {
                camera
                    .iter()
                    .map(|(id, affine)| {
                        let matrix = [
                            affine.linear[(0, 0)],
                            affine.linear[(0, 1)],
                            affine.translation.x,
                            affine.linear[(1, 0)],
                            affine.linear[(1, 1)],
                            affine.translation.y,
                        ];
                        json!({"id":id,"matrix":matrix})
                    })
                    .collect()
            })
            .collect();
        frames.push(json!({"timestamp":timestamp,"images":images,"cameras":cameras}));
    }
    flow.input_queue().send(None).unwrap();
    assert!(flow.output_queue().recv().unwrap().is_none());
    flow.shutdown().unwrap();
    serde_json::to_writer_pretty(File::create(rust_path).unwrap(), &frames).unwrap();
    let expected: Vec<Value> =
        serde_json::from_reader(BufReader::new(File::open(cpp_path).unwrap())).unwrap();
    assert_frames(&frames, &expected);
}

fn assert_frames(frames: &[Value], expected: &[Value]) {
    assert_eq!(frames.len(), expected.len());
    for (index, (actual, reference)) in frames.iter().zip(expected).enumerate() {
        assert_eq!(actual["timestamp"], reference["timestamp"]);
        assert_eq!(
            actual["images"], reference["images"],
            "frame {index} decoded pixels/exposure"
        );
        let num_cameras = actual["cameras"].as_array().unwrap().len();
        assert_eq!(num_cameras, reference["cameras"].as_array().unwrap().len());
        for camera in 0..num_cameras {
            let actual = actual["cameras"][camera].as_array().unwrap();
            let reference = reference["cameras"][camera].as_array().unwrap();
            assert_eq!(
                actual.len(),
                reference.len(),
                "frame {index} camera {camera} track count"
            );
            for (a, b) in actual.iter().zip(reference) {
                assert_eq!(
                    a["id"], b["id"],
                    "frame {index} camera {camera} keypoint IDs"
                );
                for component in 0..6 {
                    let av = a["matrix"][component].as_f64().unwrap();
                    let bv = b["matrix"][component].as_f64().unwrap();
                    assert!(
                        (av - bv).abs() <= 1e-5,
                        "frame {index} camera {camera} id {} component {component}: {av} vs {bv}",
                        a["id"]
                    );
                }
            }
        }
    }
}
