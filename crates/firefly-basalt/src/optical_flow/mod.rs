pub mod frame_to_frame_optical_flow;
pub mod multiscale_frame_to_frame_optical_flow;
pub mod patch;
pub mod patch_optical_flow;
#[path = "optical_flow.rs"]
mod types;
pub use types::{
    AffineCompact2, KeypointId, OpticalFlowBase, OpticalFlowFactory, OpticalFlowInput,
    OpticalFlowResult,
};
pub mod patterns;

struct TrackedFrame {
    pyramid: Vec<crate::image::image_pyr::ManagedImagePyr<u16>>,
    flow: std::sync::Arc<OpticalFlowResult>,
}
#[cfg(test)]
mod reference_tests;
