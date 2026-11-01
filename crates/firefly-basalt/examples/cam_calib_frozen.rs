//! Replay official corner and initial-pose caches without rerunning detection.
use firefly_basalt::calibration::cam_calib::CamCalib;
use std::{fs::File, path::Path};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    assert!(
        (6..=7).contains(&args.len()),
        "dataset-type dataset grid source-cache output-directory [initial-calibration]"
    );
    let dataset_type = &args[1];
    let mut source = CamCalib::new(
        Path::new(&args[2]),
        dataset_type,
        Path::new(&args[3]),
        Path::new(&args[4]),
        "calib-cam",
        1,
        vec!["ds".into(); 2],
    )?;
    source.load_dataset()?;
    let mut target = CamCalib::new(
        Path::new(&args[2]),
        dataset_type,
        Path::new(&args[3]),
        Path::new(&args[5]),
        "calib-cam",
        1,
        vec!["ds".into(); 2],
    )?;
    target.vio_dataset = source.vio_dataset;
    target.calib_corners = source.calib_corners;
    target.calib_corners_rejected = source.calib_corners_rejected;
    target.calib_init_poses = source.calib_init_poses;
    target.init_cam_intrinsics()?;
    target.init_cam_extrinsics()?;
    if let Some(path) = args.get(6) {
        #[derive(serde::Deserialize)]
        struct Archive {
            value0: firefly_calibration::calibration::Calibration<f64>,
        }
        let archive: Archive = serde_json::from_reader(File::open(path)?)?;
        target.calib_opt.as_mut().unwrap().calib = Some(archive.value0);
    }
    target.init_optimization()?;
    let initial = target.calib_opt.as_ref().unwrap().calib.as_ref().unwrap();
    serde_json::to_writer(
        File::create(Path::new(&args[5]).join("initial_calibration.json"))?,
        &serde_json::json!({"value0":initial}),
    )?;
    let projections: Vec<_> = target.reprojected_vignette.iter().map(|(id, data)| serde_json::json!({"time":id.frame_id, "camera":id.cam_id, "points":data.corners_proj.iter().map(|p|[p.x,p.y]).collect::<Vec<_>>(), "valid":data.corners_proj_success})).collect();
    serde_json::to_writer(
        File::create(Path::new(&args[5]).join("initial_vignette_projections.json"))?,
        &projections,
    )?;
    while !target.optimize_with_param(false, None)? {}
    target.compute_vign()?;
    target.save_calib()?;
    Ok(())
}
