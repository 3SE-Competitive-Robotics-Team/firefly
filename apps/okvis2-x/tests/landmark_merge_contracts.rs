// SPDX-License-Identifier: BSD-3-Clause

//! Landmark merge contracts from OKVIS2-X `ViGraphEstimator` and `ViSlamBackend`.
use firefly_okvis_x::{ceres::vi_slam_backend::ViSlamBackend, frames::frame_typedefs::LandmarkId};
use nalgebra::Vector4;

fn backend() -> ViSlamBackend {
    let mut backend = ViSlamBackend::default();
    for graph in [&mut backend.realtime_graph, &mut backend.full_graph] {
        for id in 1..=3 {
            graph
                .add_landmark(
                    LandmarkId::new(id),
                    Vector4::new(id as f64, 0., 1., 1.),
                    true,
                )
                .unwrap();
        }
    }
    backend
}

#[test]
fn single_merge_updates_both_graphs_and_rejects_missing_landmarks() {
    let mut backend = backend();
    let (a, b, missing) = (LandmarkId::new(1), LandmarkId::new(2), LandmarkId::new(99));
    assert!(backend.merge_landmark(a, b).unwrap());
    for graph in [&backend.realtime_graph, &backend.full_graph] {
        assert!(!graph.landmarks.contains_key(&a));
        assert!(graph.landmarks.contains_key(&b));
    }
    assert!(backend.merge_landmark(a, b).is_err());
    assert!(backend.merge_landmark(b, missing).is_err());
    assert!(backend.realtime_graph.landmarks.contains_key(&b));
    assert!(backend.full_graph.landmarks.contains_key(&b));
}

#[test]
fn batch_merge_resolves_reverse_pairs_and_chains_without_double_counting() {
    let mut backend = backend();
    let (a, b, c) = (LandmarkId::new(1), LandmarkId::new(2), LandmarkId::new(3));
    assert_eq!(
        backend
            .merge_landmarks(&[a, b, a, c], &[b, a, c, b])
            .unwrap(),
        2
    );
    for graph in [&backend.realtime_graph, &backend.full_graph] {
        assert_eq!(graph.landmarks.keys().copied().collect::<Vec<_>>(), vec![c]);
    }
}

#[test]
fn batch_merge_rejects_missing_landmarks_and_mismatched_vectors() {
    let mut backend = backend();
    assert!(
        backend
            .merge_landmarks(&[LandmarkId::new(99)], &[LandmarkId::new(1)])
            .is_err()
    );
    assert!(backend.merge_landmarks(&[LandmarkId::new(1)], &[]).is_err());
    assert_eq!(backend.realtime_graph.landmarks.len(), 3);
    assert_eq!(backend.full_graph.landmarks.len(), 3);
}

#[test]
fn frontend_queries_only_the_selected_landmark_and_observes_graph_updates() {
    use firefly_okvis_x::frontend::pipeline::FrontendGraph;

    let mut backend = backend();
    let (a, b) = (LandmarkId::new(1), LandmarkId::new(2));
    {
        let _unrelated = backend.realtime_graph.landmarks[&a].h_point.borrow_mut();
        let selected = FrontendGraph::get_landmark(&backend, b).unwrap();
        assert_eq!(selected.id, b);
        assert_eq!(selected.point, Vector4::new(2., 0., 1., 1.));
        assert_eq!(
            FrontendGraph::landmark(&backend, b).unwrap(),
            selected.point
        );
        assert!(FrontendGraph::is_landmark_initialised(&backend, b).unwrap());
    }
    let changed = Vector4::new(-1., 2., 3., 1.);
    backend.set_landmark(b, changed, false).unwrap();
    assert_eq!(
        FrontendGraph::get_landmark(&backend, b).unwrap().point,
        changed
    );
    assert!(!FrontendGraph::is_landmark_initialised(&backend, b).unwrap());
    backend.merge_landmark(a, b).unwrap();
    assert!(!FrontendGraph::is_landmark_added(&backend, a));
    assert!(FrontendGraph::get_landmark(&backend, a).is_err());
    assert!(FrontendGraph::is_landmark_added(&backend, b));
}
