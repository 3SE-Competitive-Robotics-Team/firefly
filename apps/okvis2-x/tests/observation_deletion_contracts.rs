// SPDX-License-Identifier: BSD-3-Clause

//! Missing observations violate the OKVIS2-X graph deletion precondition.
use firefly_okvis_x::{
    ceres::{vi_graph::ViGraph, vi_slam_backend::ViSlamBackend},
    frames::frame_typedefs::{KeypointIdentifier, StateId},
};
use std::sync::atomic::Ordering;

#[test]
fn graph_rejects_missing_observations() {
    let mut graph = ViGraph::new();
    assert!(
        graph
            .remove_observation(KeypointIdentifier::new(1, 0, 0))
            .is_err()
    );
}

#[test]
fn backend_rejects_missing_observations_in_idle_and_loop_closing_graphs() {
    for busy in [false, true] {
        let mut backend = ViSlamBackend::default();
        backend.is_loop_closing.store(busy, Ordering::Release);
        assert!(backend.remove_observation(StateId::new(1), 0, 0).is_err());
        assert!(backend.touched_states.is_empty());
        assert!(backend.touched_landmarks.is_empty());
    }
}
