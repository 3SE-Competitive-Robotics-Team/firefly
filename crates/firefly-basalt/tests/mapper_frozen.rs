//! Independent original-controller comparison after freezing random matching.
use firefly_basalt::utils::sophus_utils::se3_matrix;
use firefly_basalt::{
    mapper::Mapper,
    utils::{common_types::KeypointsData, vio_config::VioConfig},
};
use firefly_calibration::observation::TimeCamId;
use nalgebra::Vector2;
use serde_json::{Value, json};
use std::{collections::BTreeMap, fs::File, io::BufReader, path::PathBuf};

fn snapshot(mapper: &Mapper) -> Value {
    let poses: BTreeMap<_, _> = mapper
        .mapper
        .base
        .frame_poses
        .iter()
        .map(|(&time, state)| {
            let matrix = se3_matrix(state.pose());
            let values: Vec<_> = (0..4)
                .flat_map(|row| (0..4).map(move |col| matrix[(row, col)]))
                .collect();
            (time.to_string(), values)
        })
        .collect();
    let points: BTreeMap<_, _> = mapper.mapper.base.lmdb.landmarks().iter().map(|(&id,point)| {
        (id.to_string(),json!({"parameters":[point.direction.x,point.direction.y,point.inv_dist],
            "host":[point.host_kf_id.frame_id,point.host_kf_id.cam_id],"observations":point.obs.len()}))
    }).collect();
    json!({"poses":poses,"points":points})
}

fn difference(actual: &Value, expected: &Value) -> (f64, usize) {
    match (actual, expected) {
        (Value::Number(a), Value::Number(b))
            if (a.is_i64() || a.is_u64()) && (b.is_i64() || b.is_u64()) =>
        {
            (0., usize::from(a != b))
        }
        (Value::Number(a), Value::Number(b)) => (
            (a.as_f64().unwrap() - b.as_f64().unwrap()).abs() / (1. + b.as_f64().unwrap().abs()),
            0,
        ),
        (Value::Array(a), Value::Array(b)) => a
            .iter()
            .zip(b)
            .map(|(a, b)| difference(a, b))
            .fold((0., a.len().abs_diff(b.len())), |(m, n), (x, y)| {
                (m.max(x), n + y)
            }),
        (Value::Object(a), Value::Object(b)) => {
            let missing = a.keys().filter(|key| !b.contains_key(*key)).count()
                + b.keys().filter(|key| !a.contains_key(*key)).count();
            a.iter()
                .filter_map(|(key, a)| b.get(key).map(|b| difference(a, b)))
                .fold((0., missing), |(m, n), (x, y)| (m.max(x), n + y))
        }
        _ => (0., usize::from(actual != expected)),
    }
}

#[test]
#[ignore = "requires BASALT_SOURCE, BASALT_MAPPER_CACHE and BASALT_MAPPER_FROZEN"]
fn frozen_original_tracks_match_setup_and_optimizer_stages() {
    let root = PathBuf::from(std::env::var_os("BASALT_SOURCE").unwrap());
    let cache = PathBuf::from(std::env::var_os("BASALT_MAPPER_CACHE").unwrap());
    let fixture = PathBuf::from(std::env::var_os("BASALT_MAPPER_FROZEN").unwrap());
    let expected: Value =
        serde_json::from_reader(BufReader::new(File::open(fixture).unwrap())).unwrap();
    let mut config = VioConfig::default();
    config
        .load(root.join("data/tumvi_512_config.json"))
        .unwrap();
    let mut mapper =
        Mapper::load_data(&root.join("data/tumvi_512_ds_calib.json"), &cache, &config).unwrap();
    let mut actual = json!({"loaded":snapshot(&mapper)});
    for frame in expected["corners"].as_array().unwrap() {
        let image = TimeCamId {
            frame_id: frame["image"][0].as_i64().unwrap(),
            cam_id: frame["image"][1].as_u64().unwrap() as usize,
        };
        let corners = frame["pixels"]
            .as_array()
            .unwrap()
            .iter()
            .map(|p| Vector2::new(p[0].as_f64().unwrap(), p[1].as_f64().unwrap()))
            .collect();
        mapper.mapper.feature_corners.write().unwrap().insert(
            image,
            KeypointsData {
                corners,
                ..KeypointsData::default()
            },
        );
    }
    for track in expected["tracks"].as_array().unwrap() {
        let observations = track["observations"]
            .as_array()
            .unwrap()
            .iter()
            .map(|observation| {
                (
                    TimeCamId {
                        frame_id: observation[0].as_i64().unwrap(),
                        cam_id: observation[1].as_u64().unwrap() as usize,
                    },
                    observation[2].as_i64().unwrap() as i32,
                )
            })
            .collect();
        mapper
            .mapper
            .feature_tracks
            .insert(track["id"].as_i64().unwrap(), observations);
    }
    mapper.mapper.setup_opt();
    actual["setup"] = snapshot(&mapper);
    mapper.optimize();
    actual["optimized"] = snapshot(&mapper);
    mapper.filter();
    actual["filtered"] = snapshot(&mapper);
    mapper.optimize();
    actual["final"] = snapshot(&mapper);
    let stages = ["loaded", "setup", "optimized", "filtered", "final"];
    let diagnostics: BTreeMap<_, _> = stages
        .iter()
        .map(|&name| (name, difference(&actual[name], &expected[name])))
        .collect();
    actual["differences"] = json!(diagnostics);
    let path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../logs/basalt_mapper_frozen_rust.json");
    serde_json::to_writer_pretty(File::create(path).unwrap(), &actual).unwrap();
    for name in stages {
        let (error, missing) = diagnostics[name];
        assert_eq!(
            missing, 0,
            "{name}: differing geometry/observation membership"
        );
        // The original parallel optimizer is not a deterministic intermediate
        // oracle: identical frozen inputs differ by 1.53e-9 after the first
        // optimization in repeated C++ runs. Keep those values in diagnostics,
        // while certifying deterministic setup and the final converged result.
        if matches!(name, "loaded" | "setup" | "final") {
            assert!(
                error <= 1e-10,
                "{name}: relative numeric difference {error}"
            );
        }
    }
}
