//! The optimization edge set holds the measurement-bearing edges only.
//!
//! The covisibility graph and its spanning forest restate the relative poses
//! the map holds right now, so their residual is zero by construction and they
//! contribute stiffness only. Treating them as constraints would pin the map
//! to its present shape and suppress the loop correction that the pose-graph
//! optimizer exists to distribute.
use firefly_rootslam_mapping::{
    global_optimization::{OptimizationOptions, optimize_pose_graph},
    map::{MapKeyframe, MapSnapshot},
    pose_graph::{EdgeKind, PoseGraphEdge},
};
use nalgebra::{Isometry3, SMatrix};

fn build(covisibility_span: Option<i64>) -> MapSnapshot {
    let mut snapshot = MapSnapshot::default();
    let count = 12_i64;
    for id in 0..count {
        snapshot.keyframes.insert(
            id,
            MapKeyframe {
                id,
                t_w_b: Isometry3::translation(id as f64, 0., 0.),
                // The newest keyframe stays in the sliding window and is held
                // fixed, which anchors the gauge as in the paper.
                active: id == count - 1,
                features: Vec::new(),
                images: None,
            },
        );
    }
    let information = SMatrix::<f64, 6, 6>::identity();
    for id in 0..count - 1 {
        snapshot.edges.push(PoseGraphEdge {
            from: id,
            to: id + 1,
            measurement: Isometry3::translation(1., 0., 0.),
            kind: EdgeKind::Temporal,
            sqrt_information: information,
        });
    }
    if let Some(span) = covisibility_span {
        for id in 0..count - span {
            snapshot.edges.push(PoseGraphEdge {
                from: id,
                to: id + span,
                measurement: Isometry3::translation(span as f64, 0., 0.),
                kind: EdgeKind::Covisibility,
                sqrt_information: information,
            });
        }
    }
    // The loop claims the two ends are 12 m apart; the current estimates say
    // 11 m. This is the drift the pose-graph optimizer has to distribute.
    snapshot.edges.push(PoseGraphEdge {
        from: 0,
        to: count - 1,
        measurement: Isometry3::translation(12., 0., 0.),
        kind: EdgeKind::Loop,
        sqrt_information: information,
    });
    snapshot
}

#[test]
fn covisibility_edges_do_not_constrain_the_pose_graph_optimizer() {
    let options = OptimizationOptions {
        max_iterations: 50,
        ..Default::default()
    };
    let without = optimize_pose_graph(&build(None), &options).unwrap();
    let with = optimize_pose_graph(&build(Some(3)), &options).unwrap();
    for id in 0..12 {
        assert_eq!(
            without.poses[&id], with.poses[&id],
            "covisibility edges changed the pose graph solution for keyframe {id}"
        );
    }
    assert_eq!(without.final_cost, with.final_cost);
}

#[test]
fn the_odometry_network_distributes_the_loop_correction() {
    let options = OptimizationOptions {
        max_iterations: 50,
        ..Default::default()
    };
    let result = optimize_pose_graph(&build(None), &options).unwrap();
    // The loop residual is the only violated constraint, so the optimizer has
    // to remove most of it instead of leaving the map untouched.
    assert!(
        result.final_cost < result.initial_cost * 0.25,
        "loop correction removed only {}% of the drift",
        100.0 * (1.0 - result.final_cost / result.initial_cost)
    );
    // The correction travels backwards along the odometry chain.
    assert!(result.poses[&0].translation.vector.x < -0.5);
    assert_eq!(result.poses[&11].translation.vector.x, 11.);
}
