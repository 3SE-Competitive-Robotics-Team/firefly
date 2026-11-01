use firefly_calibration::aprilgrid::AprilGrid;
use nalgebra::Vector4;

#[test]
fn corner_order_and_vignette_geometry() {
    let grid = AprilGrid::from_parameters(3, 2, 0.04, 0.25).unwrap();
    assert_eq!(grid.aprilgrid_corner_pos_3d.len(), 24);
    assert_eq!(grid.aprilgrid_vignette_pos_3d.len(), 85);
    for (id, point) in grid.aprilgrid_corner_pos_3d.iter().enumerate() {
        let tag = id / 4;
        let corner = id % 4;
        let expected = Vector4::new(
            (tag % 3) as f64 * 0.05 + [0., 0.04, 0.04, 0.][corner],
            (tag / 3) as f64 * 0.05 + [0., 0., 0.04, 0.04][corner],
            0.,
            1.,
        );
        assert!((point - expected).amax() < 1e-15);
    }
    assert!(
        (grid.aprilgrid_vignette_pos_3d[0] - Vector4::new(0.045, 0.04 / 6., 0., 1.)).amax() < 1e-15
    );
    assert!(AprilGrid::from_parameters(0, 2, 0.04, 0.25).is_err());
}
