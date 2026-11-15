use firefly_basalt::opt_flow::FlowStatistics;
use firefly_basalt::optical_flow::{AffineCompact2, OpticalFlowInput, OpticalFlowResult};
use std::{collections::BTreeMap, sync::Arc};

#[test]
fn original_result_collection_counts_each_camera_and_duplicate_time() {
    let expected: serde_json::Value =
        serde_json::from_str(include_str!("reference/opt_flow.json")).unwrap();
    let mut stats = FlowStatistics::default();
    assert!(stats.mean_track_length().is_nan());
    for frame in 0..3 {
        let mut result = OpticalFlowResult {
            t_ns: if frame == 2 { 0 } else { frame as i64 },
            observations: vec![BTreeMap::new(); 2],
            input_images: Some(Arc::new(OpticalFlowInput::default())),
            ..Default::default()
        };
        for (camera, observations) in result.observations.iter_mut().enumerate() {
            observations.insert(7, AffineCompact2::default());
            observations.insert(10 + frame + camera, AffineCompact2::default());
        }
        stats.read_result(result);
    }
    assert_eq!(
        stats.observations.len(),
        expected["stored"].as_u64().unwrap() as usize
    );
    assert!(
        stats
            .observations
            .values()
            .all(|r| r.input_images.is_none())
    );
    for (id, count) in expected["counts"].as_object().unwrap() {
        assert_eq!(
            stats.keypoint_stats[&id.parse::<usize>().unwrap()],
            count.as_u64().unwrap() as usize
        );
    }
    assert_eq!(stats.keypoint_stats.len(), 5);
    assert_eq!(stats.mean_track_length(), 12. / 5.);
    assert!(stats.observations[&0].observations[0].contains_key(&10));
    assert!(!stats.observations[&0].observations[0].contains_key(&12));
}
