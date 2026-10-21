// SPDX-License-Identifier: BSD-3-Clause

//! SRL pinhole projection with f32 coordinates and identity distortion.
//!
//! ```
//! use firefly_srl_projection::{ProjectionPinholeCamera, ProjectionStatus};
//! use nalgebra::{Vector2, Vector3};
//!
//! let camera = ProjectionPinholeCamera::new(640, 480, 320.0, 320.0, 320.0, 240.0);
//! let (status, pixel) = camera.project(&Vector3::new(0.0, 0.0, 2.0));
//! assert_eq!(status, ProjectionStatus::Successful);
//! assert_eq!(pixel, Vector2::new(320.0, 240.0));
//! assert_eq!(camera.back_project(&pixel), Vector3::new(0.0, 0.0, 1.0));
//! ```

pub mod no_distortion;
pub mod pinhole_camera;
pub mod projection_base;

pub use no_distortion::NoDistortion;
pub use pinhole_camera::ProjectionPinholeCamera;
pub use projection_base::{ProjectionBase, ProjectionStatus};
