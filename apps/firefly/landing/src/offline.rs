use crate::{Result, config::Config, model};
use firefly_pubsub::camera::{DOWN_HEIGHT, DOWN_WIDTH};
use std::path::Path;

pub(crate) fn run(config: &Config, path: &Path) -> Result<()> {
    let environment = model::environment(config.backend, config.runtime.as_deref())?;
    let image = image::open(path)
        .map_err(|e| crate::external("landing.image.read", e).with_context("path", path.display()))?
        .to_luma8();
    if image.dimensions() != (DOWN_WIDTH as u32, DOWN_HEIGHT as u32) {
        return Err(crate::invalid("offline input must be 352x288"));
    }
    let mut model = model::Model::load(&config.manifest(), &environment)?;
    let mut results = Vec::new();
    let q = config.camera_rotation_xyzw;
    let camera = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::identity(),
        nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(q[3], q[0], q[1], q[2])),
    );
    for detection in model.detect(image.as_raw(), config.confidence)? {
        let best = firefly_landing::geometry::outline_candidates(
            image.as_raw(),
            DOWN_WIDTH,
            DOWN_HEIGHT,
            detection,
        )?
        .into_iter()
        .filter_map(|corners| {
            config
                .geometry
                .horizontal_target(corners.rectangle, &camera)
                .map(|pose| (corners, pose))
        })
        .min_by(|a, b| a.1.reprojection_px.total_cmp(&b.1.reprojection_px));
        let corners = best.as_ref().map(|(outline, _)| outline.rectangle);
        let visual = best
            .as_ref()
            .and_then(|(outline, _)| config.geometry.marker_pose(outline));
        let pose = best.map(|(_, pose)| pose);
        let translation = pose.as_ref().map(|target| {
            camera
                .inverse_transform_point(&nalgebra::Point3::from(target.center))
                .coords
        });
        results.push(
            serde_json::json!({"bounds":detection.bounds,"confidence":detection.confidence,
            "corners":corners,"camera_translation":translation.as_ref().map(nalgebra::Matrix::as_slice),
            "camera_from_h_mod_pi":visual.map(|v| serde_json::json!({"translation_m":v.camera_from_h.translation.vector.as_slice(),"xyzw":v.camera_from_h.rotation.coords.as_slice(),"reprojection_px":v.reprojection_px})),
            "reprojection_px":pose.map(|target|target.reprojection_px)}),
        );
    }
    println!(
        "{}",
        serde_json::to_string(&results).map_err(|e| crate::external("landing.image.export", e))?
    );

    Ok(())
}
