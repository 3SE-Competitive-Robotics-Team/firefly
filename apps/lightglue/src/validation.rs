//! 独立渲染视角的定位验收；查询只使用二维特征，三维查询标签只参与误差评分。
use super::*;

#[test]
#[ignore = "requires independent held-out captures, models and a frozen map"]
#[allow(clippy::large_stack_arrays)]
fn held_out_map_localization_contract() {
    firefly_observability::init();
    let node = create_node().unwrap();
    let log_ipc = firefly_observability::init_ipc(&node, "map-validation");
    let path = std::env::var("FIREFLY_QUERY_MAP").expect("FIREFLY_QUERY_MAP required");
    let report = std::env::var("FIREFLY_MAP_VALIDATION_REPORT").expect("report path required");
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let map =
        firefly_vision_map::load_map(&root.join("apps/planner/maps/rmuc2026.ffvmap")).unwrap();
    let queries = firefly_vision_map::load_map(std::path::Path::new(&path)).unwrap();
    let expected: usize = std::env::var("FIREFLY_QUERY_COUNT")
        .expect("expected capture count required")
        .parse()
        .unwrap();
    assert_eq!(
        queries.frames.len(),
        expected,
        "rejected query captures must not disappear from acceptance"
    );
    let mut session = load_session(root.join(DEFAULT_MODEL).to_str().unwrap()).unwrap();
    assert!(!queries.frames.is_empty());
    let mut results = Vec::new();
    for frame in &queries.frames {
        assert!(
            map.frames.iter().all(|f| {
                nalgebra::Vector3::from(f.position)
                    .metric_distance(&nalgebra::Vector3::from(frame.position))
                    > 0.1
            }),
            "validation pose must be absent from training map"
        );
        let n = frame.points.len().min(NUM_POINTS);
        let mut features = Box::new(FeatureMessage {
            timestamp: frame.timestamp,
            count: n as u32,
            keypoints: [[0.; 2]; NUM_POINTS],
            descriptors: [[0.; DESC_DIM]; NUM_POINTS],
            scores: [0.; NUM_POINTS],
        });
        for (i, point) in frame.points.iter().take(n).enumerate() {
            features.keypoints[i] = point.uv;
            features.descriptors[i] = point.descriptor;
            features.scores[i] = point.score;
        }
        let q = frame.quat_xyzw;
        let prior = OdomMessage {
            timestamp: frame.timestamp,
            is_initialized: true,
            position_x: frame.position[0] + 0.3,
            position_y: frame.position[1] - 0.2,
            position_z: frame.position[2] + 0.1,
            quat_x: q[0],
            quat_y: q[1],
            quat_z: q[2],
            quat_w: q[3],
            ..Default::default()
        };
        let observation = query_once(&mut session, &map, &features, &prior, || false).unwrap();
        let result = if let Some(pose) = observation {
            let error = (nalgebra::Vector3::new(pose.position_x, pose.position_y, pose.position_z)
                - nalgebra::Vector3::from(frame.position))
            .norm();
            let expected =
                UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q[3], q[0], q[1], q[2]));
            let estimated = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                pose.quat_w,
                pose.quat_x,
                pose.quat_y,
                pose.quat_z,
            ));
            let angle = expected.angle_to(&estimated).to_degrees();
            let quality = pose.num_inliers >= 30
                && f64::from(pose.num_inliers) / f64::from(pose.total_points) >= 0.3
                && pose.error <= 3.;
            serde_json::json!({"id": frame.id, "passed": quality && error <= 0.25 && angle <= 15.,
                "position_error_m": error, "rotation_error_deg": angle, "inliers": pose.num_inliers,
                "correspondences": pose.total_points, "reprojection_px": pose.error})
        } else {
            serde_json::json!({"id": frame.id, "passed": false, "reason": "no_pose"})
        };
        log::info!("held-out map validation: {result}");
        firefly_observability::pump_log_ipc(&log_ipc);
        results.push(result);
    }
    let passed = results.iter().all(|r| r["passed"] == true);
    let summary = serde_json::json!({"passed": passed, "queries": results,
        "position_tolerance_m": 0.25, "rotation_tolerance_deg": 15.,
        "prior_offset_m": [0.3, -0.2, 0.1], "alignment": "none"});
    std::fs::write(report, serde_json::to_vec_pretty(&summary).unwrap()).unwrap();
    assert!(passed, "some held-out poses failed; see validation report");
}
