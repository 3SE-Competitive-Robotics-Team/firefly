use firefly_basalt::{
    utils::{common_types::TimeCamId, imu_types::PoseStateWithLin},
    vi_estimator::{
        ba_base::BundleAdjustmentBase,
        landmark_database::{Keypoint, KeypointObservation},
        local_map::LocalMapSnapshot,
    },
};
use firefly_calibration::calibration::Calibration;
use nalgebra::{Isometry3, SVector, Vector2};
use std::collections::{BTreeMap, BTreeSet};

#[test]
fn snapshot_retains_optimized_geometry_after_window_removal() {
    let mut base = BundleAdjustmentBase::new(Calibration::<f64>::default(), 0.5, 1.0);
    let host = TimeCamId {
        frame_id: 10,
        cam_id: 1,
    };
    let target = TimeCamId {
        frame_id: 20,
        cam_id: 0,
    };
    let mut state = PoseStateWithLin::new(10, Isometry3::translation(1.0, 2.0, 3.0), true);
    state.apply_inc(&SVector::<f64, 6>::from_row_slice(&[
        0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
    ]));
    base.frame_poses.insert(10, state);
    base.lmdb
        .add_landmark(42, &Keypoint::new(Vector2::new(0.1, 0.2), 0.25, host));
    base.lmdb.add_observation(
        target,
        &KeypointObservation {
            kpt_id: 42,
            pos: Vector2::new(121.0, 92.0),
        },
    );
    let snapshot = LocalMapSnapshot::capture(20, &base, &BTreeSet::from([10]), &BTreeMap::new());
    base.lmdb.remove_frame(20);
    base.frame_poses.clear();
    assert_eq!(snapshot.landmarks[&42].host_kf_id, host);
    assert_eq!(
        snapshot.landmarks[&42].obs[&target],
        Vector2::new(121.0, 92.0)
    );
    assert_eq!(snapshot.landmarks[&42].inv_dist, 0.25);
    assert!((snapshot.poses[&10].translation.vector.x - 1.2).abs() < 1e-14);
}
