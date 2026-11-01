use firefly_basalt_headers::image::image::ManagedImage;
use firefly_calibration::observation::{
    self, AprilTagDetector, CalibCornerData, CalibrationImageSource, TimeCamId,
};
use firefly_error::{Error, ErrorKind, Result};
use std::sync::Arc;

struct Source {
    fail: bool,
}

impl CalibrationImageSource for Source {
    fn image_timestamps(&self) -> Result<Vec<i64>> {
        Ok(vec![17])
    }

    fn images_at(&self, _: i64) -> Result<Vec<firefly_calibration::observation::CalibrationImage>> {
        if self.fail {
            return Err(Error::new(ErrorKind::Internal, "image read failed"));
        }
        Ok(vec![
            Some(Arc::new(ManagedImage::new(2, 2))),
            None,
            Some(Arc::new(ManagedImage::new(2, 2))),
        ])
    }
}

struct Detector;

impl AprilTagDetector for Detector {
    fn detect_tags(&mut self, _: &ManagedImage<u16>) -> Result<(CalibCornerData, CalibCornerData)> {
        Ok((CalibCornerData::default(), CalibCornerData::default()))
    }
}

#[test]
fn injected_image_source_preserves_camera_ids_and_propagates_read_errors() {
    let (accepted, rejected) =
        observation::detect_corners(&Source { fail: false }, 4, |_| Box::new(Detector)).unwrap();
    assert_eq!(accepted.len(), 2);
    assert!(accepted.contains_key(&TimeCamId::new(17, 0)));
    assert!(accepted.contains_key(&TimeCamId::new(17, 2)));
    assert_eq!(rejected.len(), 2);

    assert!(
        observation::detect_corners(&Source { fail: true }, 4, |_| Box::new(Detector)).is_err()
    );
}
