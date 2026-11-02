//! Pose-graph residual information scale, `RootSLAM` section III-C `S_ab`.
//!
//! The paper weights every edge of the pose graph with `S_ab`, the square root
//! of the edge information matrix, so that the metre- and radian-valued
//! pose-graph residuals of Equation (9) can be summed with the pixel-valued
//! reprojection residuals of the bundle adjustment. The paper does not
//! publish the numerical values of `S_ab`; this module derives a
//! dimensionally consistent scale from the camera model instead of assuming
//! one.
//!
//! The scale is expressed in pixel equivalents: a pose-graph translation
//! residual of one metre is multiplied by `translation_pixels_per_meter`, and
//! a rotation residual of one radian by `rotation_pixels_per_radian`. Both
//! numbers are the image-plane displacement that the corresponding rigid body
//! error produces at `reference_depth_m`, so the two residual families share
//! one unit and Equation (9) becomes a sum of comparable terms.

use firefly_calibration::calibration::Calibration;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{SMatrix, Vector2, Vector3, Vector4};

use crate::pose_graph::{adjoint, hat};

/// Reference depth used when no explicit weight is supplied.
pub const DEFAULT_REFERENCE_DEPTH_M: f64 = 5.0;

/// Pose-graph residual scale in pixel equivalents.
///
/// `translation_pixels_per_meter` and `rotation_pixels_per_radian` are
/// `None` together when the weights are derived from the camera model at
/// `reference_depth_m`; supplying only one of them is rejected because the
/// translation and rotation blocks of `S_ab` must share one unit.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct EdgeInformation {
    /// Depth in metres at which the rigid body error is projected onto the
    /// image plane. Only used for the camera-model calibration.
    pub reference_depth_m: f64,
    /// Translation weight in pixels per metre.
    pub translation_pixels_per_meter: Option<f64>,
    /// Rotation weight in pixels per radian.
    pub rotation_pixels_per_radian: Option<f64>,
}

impl Default for EdgeInformation {
    fn default() -> Self {
        Self::calibrated(DEFAULT_REFERENCE_DEPTH_M)
    }
}

impl EdgeInformation {
    /// Derive the weights from the camera model at the given reference depth.
    #[must_use]
    pub fn calibrated(reference_depth_m: f64) -> Self {
        Self {
            reference_depth_m,
            translation_pixels_per_meter: None,
            rotation_pixels_per_radian: None,
        }
    }

    /// Use the given pixel-equivalent weights verbatim.
    #[must_use]
    pub fn explicit(translation_pixels_per_meter: f64, rotation_pixels_per_radian: f64) -> Self {
        Self {
            reference_depth_m: DEFAULT_REFERENCE_DEPTH_M,
            translation_pixels_per_meter: Some(translation_pixels_per_meter),
            rotation_pixels_per_radian: Some(rotation_pixels_per_radian),
        }
    }

    /// # Errors
    /// Returns an error when the reference depth or an explicit weight is not
    /// finite and positive, or when only one of the two weights is given.
    pub fn validate(&self) -> Result<()> {
        let invalid = |message: &str| {
            Error::new(ErrorKind::InvalidArgument, message)
                .with_operation("rootslam.edge_information.validate")
        };
        if !self.reference_depth_m.is_finite() || self.reference_depth_m <= 0.0 {
            return Err(invalid("Edge information reference depth must be positive"));
        }
        match (
            self.translation_pixels_per_meter,
            self.rotation_pixels_per_radian,
        ) {
            (None, None) => Ok(()),
            (Some(translation), Some(rotation)) => {
                if !translation.is_finite() || translation <= 0.0 {
                    return Err(invalid(
                        "Edge information translation weight must be positive",
                    ));
                }
                if !rotation.is_finite() || rotation <= 0.0 {
                    return Err(invalid("Edge information rotation weight must be positive"));
                }
                Ok(())
            }
            _ => Err(invalid(
                "Edge information translation and rotation weights must be given together",
            )),
        }
    }

    /// Resolve the square root information matrix used by every edge.
    ///
    /// # Errors
    /// Returns an error when the policy is invalid or when the camera model
    /// cannot be calibrated at the reference depth.
    pub fn resolve(&self, calibration: &Calibration<f64>) -> Result<SMatrix<f64, 6, 6>> {
        self.validate()?;
        let (translation, rotation) = match (
            self.translation_pixels_per_meter,
            self.rotation_pixels_per_radian,
        ) {
            (Some(translation), Some(rotation)) => (translation, rotation),
            (None, None) => calibrate(calibration, self.reference_depth_m)?,
            _ => unreachable!("validated above"),
        };
        let mut matrix = SMatrix::<f64, 6, 6>::zeros();
        for index in 0..3 {
            matrix[(index, index)] = translation;
        }
        for index in 3..6 {
            matrix[(index, index)] = rotation;
        }
        Ok(matrix)
    }
}

/// Image-plane displacement per unit rigid body error, in pixel equivalents.
///
/// The camera model is differentiated analytically, so no assumed focal
/// length is needed and every supported model is handled the same way. The
/// reference point lies on the optical axis at the reference depth and the
/// largest of the six body-frame directions is taken, which is the operator
/// norm of the pixel response: inside a 45 degree half field of view no
/// rigid body error shifts the image further than that bound.
fn calibrate(calibration: &Calibration<f64>, reference_depth_m: f64) -> Result<(f64, f64)> {
    if calibration.intrinsics.is_empty() {
        // Without a camera model no bundle adjustment can run, so the
        // pose-graph scale is never used. Keep the unscaled unit weight and
        // say so instead of refusing to build a map that has no geometry.
        log::warn!(
            "Calibration has no camera model; the pose-graph residuals keep the unscaled unit weight"
        );
        return Ok((1.0, 1.0));
    }
    if calibration.intrinsics.len() != calibration.t_i_c.len() {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "Camera model and extrinsic counts differ",
        )
        .with_operation("rootslam.edge_information.calibrate"));
    }
    let mut translation = 0.0_f64;
    let mut rotation = 0.0_f64;
    for (index, intrinsic) in calibration.intrinsics.iter().enumerate() {
        let camera_from_body = calibration.t_i_c[index].inverse();
        let point = Vector3::new(0.0, 0.0, reference_depth_m);
        {
            let mut prediction = Vector2::zeros();
            let mut projection = SMatrix::<f64, 2, 4>::zeros();
            if !intrinsic.project(
                &Vector4::new(point.x, point.y, point.z, 1.0),
                &mut prediction,
                Some(&mut projection),
            ) {
                continue;
            }
            // The projection Jacobian maps a camera-frame point offset to a
            // pixel offset; a left increment of the body pose moves that point
            // by `[I | -hat(point)]`, and the extrinsic converts the body
            // increment into the camera one.
            let point_jacobian = projection.fixed_columns::<3>(0).into_owned();
            let mut motion = SMatrix::<f64, 3, 6>::zeros();
            motion
                .fixed_view_mut::<3, 3>(0, 0)
                .copy_from(&SMatrix::<f64, 3, 3>::identity());
            motion.fixed_view_mut::<3, 3>(0, 3).copy_from(&-hat(&point));
            let pose_jacobian = point_jacobian * motion * adjoint(&camera_from_body);
            for axis in 0..3 {
                translation = translation.max(pose_jacobian.column(axis).norm());
                rotation = rotation.max(pose_jacobian.column(axis + 3).norm());
            }
        }
    }
    if !translation.is_finite() || !rotation.is_finite() || translation <= 0.0 || rotation <= 0.0 {
        return Err(Error::new(
            ErrorKind::InvalidArgument,
            "Camera model cannot be calibrated at the reference depth",
        )
        .with_operation("rootslam.edge_information.calibrate"));
    }
    Ok((translation, rotation))
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_calibration::camera::generic_camera::GenericCamera;
    use firefly_calibration::camera::pinhole_camera::PinholeCamera;
    use nalgebra::Isometry3;

    fn calibration(focal: f64) -> Calibration<f64> {
        let camera =
            GenericCamera::Pinhole(PinholeCamera::new(Vector4::new(focal, focal, 320., 240.)));
        Calibration {
            intrinsics: vec![camera.clone(), camera],
            t_i_c: vec![Isometry3::identity(), Isometry3::translation(0.2, 0., 0.)],
            ..Calibration::default()
        }
    }

    #[test]
    fn calibrated_weights_match_the_pinhole_scaling_law() {
        let focal = 300.;
        let depth = 5.;
        let (translation, rotation) = calibrate(&calibration(focal), depth).unwrap();
        // A lateral translation of one metre at the reference depth shifts the
        // image by `focal / depth` pixels; a rotation of one radian shifts it
        // by `focal` pixels, independently of the depth.
        assert!((translation - focal / depth).abs() < 1e-9);
        assert!((rotation - focal).abs() < 1e-9);
    }

    #[test]
    fn explicit_weights_are_used_verbatim() {
        let information = EdgeInformation::explicit(7., 11.)
            .resolve(&calibration(300.))
            .unwrap();
        assert_eq!(information[(0, 0)], 7.);
        assert_eq!(information[(2, 2)], 7.);
        assert_eq!(information[(3, 3)], 11.);
        assert_eq!(information[(5, 5)], 11.);
        assert_eq!(information[(0, 3)], 0.);
    }

    #[test]
    fn half_given_weights_are_rejected() {
        let information = EdgeInformation {
            reference_depth_m: 5.,
            translation_pixels_per_meter: Some(1.),
            rotation_pixels_per_radian: None,
        };
        assert!(information.validate().is_err());
    }

    #[test]
    fn a_camera_model_without_a_usable_projection_is_rejected() {
        let broken = Calibration {
            intrinsics: vec![GenericCamera::default()],
            t_i_c: vec![Isometry3::identity()],
            ..Calibration::default()
        };
        assert!(calibrate(&broken, 5.).is_err());
    }
}
