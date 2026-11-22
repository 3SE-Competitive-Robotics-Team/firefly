pub mod absolute_pose;
pub mod indices;
pub mod math;
pub mod optimization_functor;
pub mod point_cloud;
pub mod relative_pose;
pub mod sac;
pub mod sac_problems;
pub mod triangulation;
pub mod types;

#[track_caller]
fn invalid_correspondences() -> firefly_error::Error {
    firefly_error::Error::new(
        firefly_error::ErrorKind::InvalidArgument,
        "adapter storage shape violates correspondence bounds",
    )
}
