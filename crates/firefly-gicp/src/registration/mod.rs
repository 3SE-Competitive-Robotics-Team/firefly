//! 配准主循环（对照 `registration/`）。

pub mod helper;
pub mod optimizer;
pub mod reduction;
pub mod reduction_rayon;
pub mod registration;
pub mod registration_result;
pub mod rejector;
pub mod termination_criteria;

pub use helper::{
    RegistrationSetting, RegistrationType, align as align_points, align_vgicp,
    create_gaussian_voxelmap, preprocess_points,
};
pub use optimizer::{GaussNewtonOptimizer, LevenbergMarquardtOptimizer};
pub use reduction::SerialReduction;
pub use reduction_rayon::ParallelReduction;
pub use registration::Registration;
pub use registration_result::RegistrationResult;
pub use rejector::{CorrespondenceRejector, DistanceRejector, NullRejector};
pub use termination_criteria::TerminationCriteria;
