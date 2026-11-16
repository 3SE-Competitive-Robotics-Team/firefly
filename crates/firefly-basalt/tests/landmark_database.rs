use firefly_basalt::vi_estimator::landmark_database::{
    Keypoint, KeypointObservation, LandmarkDatabase,
};
use firefly_calibration::observation::TimeCamId;
use nalgebra::Vector2;
use serde_json::{Value, json};
use std::collections::BTreeSet;

fn snapshot(database: &LandmarkDatabase<f64>) -> Value {
    let mut ids: Vec<_> = database.landmarks().keys().copied().collect();
    ids.sort_unstable();
    let landmarks: Vec<_> = ids.into_iter().map(|id| {
        let landmark = database.landmark(id);
        let observations: Vec<_> = landmark.obs.iter().map(|(target, pixel)| json!([target.frame_id, target.cam_id, pixel.x, pixel.y])).collect();
        json!({"id":id, "host":[landmark.host_kf_id.frame_id,landmark.host_kf_id.cam_id], "direction":[landmark.direction.x,landmark.direction.y], "inv_dist":landmark.inv_dist, "observations":observations})
    }).collect();
    let mut hosts: Vec<_> = database.host_kfs().collect();
    hosts.sort_unstable();
    let hosts: Vec<_> = hosts
        .into_iter()
        .map(|host| {
            json!([
                host.frame_id,
                host.cam_id,
                database.landmarks_for_host(host).count()
            ])
        })
        .collect();
    json!({"num_observations":database.num_observations(),"landmarks":landmarks,"hosts":hosts})
}

#[test]
fn official_cpp_database_lifecycle() {
    let expected: Value =
        serde_json::from_str(include_str!("reference/landmark_database.json")).unwrap();
    let mut actual = Vec::new();
    let mut database = LandmarkDatabase::<f64>::default();
    for id in 1_i32..=3 {
        let host = TimeCamId::new(if id == 3 { 2 } else { 1 }, usize::from(id == 2));
        database.add_landmark(
            id as usize,
            &Keypoint::new(
                Vector2::new(f64::from(id) * 0.25, -f64::from(id) * 0.5),
                f64::from(id) * 0.125,
                host,
            ),
        );
        for frame in 1_i32..=4 {
            database.add_observation(
                TimeCamId::new(i64::from(frame), 0),
                &KeypointObservation {
                    kpt_id: id,
                    pos: Vector2::new(f64::from(id * 10 + frame), f64::from(id * 20 - frame)),
                },
            );
        }
    }
    actual.push(snapshot(&database));
    database.backup();
    database.landmark_mut(1).direction = Vector2::new(100.0, 200.0);
    database.landmark_mut(1).inv_dist = 99.0;
    actual.push(snapshot(&database));
    database.restore();
    actual.push(snapshot(&database));
    database.add_observation(
        TimeCamId::new(1, 0),
        &KeypointObservation {
            kpt_id: 2,
            pos: Vector2::new(7.0, 8.0),
        },
    );
    actual.push(snapshot(&database));
    database.remove_frame(4);
    actual.push(snapshot(&database));
    database.remove_observations(1, &BTreeSet::from([TimeCamId::new(3, 0)]));
    actual.push(snapshot(&database));
    database.remove_keyframes(&BTreeSet::from([2]), &BTreeSet::new(), &BTreeSet::new());
    actual.push(snapshot(&database));
    database.remove_landmark(2);
    actual.push(snapshot(&database));
    assert_eq!(Value::Array(actual), expected);
}

#[test]
fn host_queries_preserve_observation_order_and_repeated_landmarks() {
    let host = TimeCamId::new(1, 0);
    let other_host = TimeCamId::new(2, 0);
    let mut database = LandmarkDatabase::<f64>::default();
    assert_eq!(database.host_kfs().len(), 0);
    for id in 1..=3 {
        database.add_landmark(
            id,
            &Keypoint::new(
                Vector2::zeros(),
                id as f64,
                if id == 3 { other_host } else { host },
            ),
        );
    }
    for (frame, id) in [(9, 2), (5, 2), (5, 1), (9, 1), (5, 3)] {
        database.add_observation(
            TimeCamId::new(frame, 0),
            &KeypointObservation {
                kpt_id: id,
                pos: Vector2::zeros(),
            },
        );
    }
    assert_eq!(
        database.host_kfs().collect::<BTreeSet<_>>(),
        BTreeSet::from([host, other_host])
    );
    assert_eq!(
        database
            .landmarks_for_host(host)
            .map(|p| p.inv_dist)
            .collect::<Vec<_>>(),
        [1., 2., 1., 2.]
    );
    assert!(std::ptr::eq(
        database.landmarks_for_host(other_host).next().unwrap(),
        database.landmark(3)
    ));
}

#[test]
fn geometry_updates_preserve_observations_and_host_index_counts() {
    let host = TimeCamId::new(1, 0);
    let mut database = LandmarkDatabase::<f32>::default();
    let mut point = Keypoint::new(Vector2::new(1.0, 2.0), 0.5, host);
    point.obs.insert(TimeCamId::new(99, 0), Vector2::zeros());
    database.add_landmark(1, &point);
    assert_eq!(database.num_landmark_observations(1), 0);
    for frame in 1..=3 {
        database.add_observation(
            TimeCamId::new(frame, 0),
            &KeypointObservation {
                kpt_id: 1,
                pos: Vector2::zeros(),
            },
        );
    }
    point.direction = Vector2::new(3.0, 4.0);
    database.add_landmark(1, &point);
    assert_eq!(database.num_landmark_observations(1), 3);
    assert_eq!(database.landmarks_for_host(host).count(), 3);
    assert_eq!(database.landmark(1).direction, point.direction);
    database.remove_observations(
        1,
        &BTreeSet::from([
            TimeCamId::new(1, 0),
            TimeCamId::new(2, 0),
            TimeCamId::new(3, 0),
        ]),
    );
    assert_eq!(database.num_landmarks(), 0);
    assert!(database.observations().is_empty());
}
