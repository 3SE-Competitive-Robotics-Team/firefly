//! Independent SVD solution demonstrates the single-precision gradient floor
//! of a block-assembled inconsistent least-squares problem near an optimum.
use nalgebra::{DMatrix, DVector};

fn apply_float(matrix: &DMatrix<f32>, step: &DVector<f32>) -> DVector<f32> {
    DVector::from_fn(matrix.nrows(), |row, _| {
        let mut sum = 0.0_f32;
        for col in 0..matrix.ncols() {
            sum += matrix[(row, col)] * step[col];
        }
        sum
    })
}
fn transpose_float(matrix: &DMatrix<f32>, residual: &DVector<f32>) -> DVector<f32> {
    let mut gradient = DVector::zeros(matrix.ncols());
    for start in (0..matrix.nrows()).step_by(8) {
        for col in 0..matrix.ncols() {
            let mut sum = 0.0_f32;
            for row in start..(start + 8).min(matrix.nrows()) {
                sum += matrix[(row, col)] * residual[row];
            }
            gradient[col] += sum;
        }
    }
    gradient
}
#[test]
fn float_gradient_reduction_can_reject_an_independent_reference_solution() {
    let matrix = DMatrix::from_fn(512, 12, |row, col| {
        let x = (row as f64 + 1.0) * (col as f64 + 1.7);
        (x.sin() + 0.3 * (x * 0.173).cos()) as f32
    });
    let precise = matrix.map(f64::from);
    let disturbance = DVector::from_fn(512, |row, _| ((row as f64 + 0.3) * 0.971).cos());
    let svd = precise.clone().svd(true, true);
    let projected = svd.solve(&disturbance, 1e-13).unwrap();
    let orthogonal = disturbance - &precise * projected;
    let step = DVector::from_fn(12, |col, _| (col as f64 + 1.0) * 1e-7);
    let rhs = (&precise * step + orthogonal).map(|v| v as f32);
    let reference = svd.solve(&rhs.map(f64::from), 1e-13).unwrap();
    let initial = f64::from(transpose_float(&matrix, &rhs).norm());
    let threshold = 1e-5 * initial;
    let float_gradient = transpose_float(
        &matrix,
        &(apply_float(&matrix, &reference.map(|v| v as f32)) - &rhs),
    );
    let precise_gradient = precise.transpose() * (&precise * &reference - rhs.map(f64::from));
    let rounded_step_gradient = precise.transpose()
        * (&precise * reference.map(|v| f64::from(v as f32)) - rhs.map(f64::from));
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../logs/rootslam_lsqr_precision.json");
    serde_json::to_writer_pretty(
        std::fs::File::create(path).unwrap(),
        &serde_json::json!({
            "initial_gradient":initial,"threshold":threshold,
            "reference_gradient":precise_gradient.norm(),
            "rounded_step_gradient":rounded_step_gradient.norm(),
            "float_reduction_gradient":float_gradient.norm()
        }),
    )
    .unwrap();
    assert!(precise_gradient.norm() < threshold);
    assert!(
        rounded_step_gradient.norm() < threshold,
        "returned f32 step remains accurate"
    );
    assert!(
        f64::from(float_gradient.norm()) > threshold,
        "f32 reduction reports a false nonconvergence"
    );
}

#[test]
fn rounding_a_certified_double_step_to_float_can_violate_the_physical_gradient() {
    // This is a stored f32 operator promoted exactly to f64, as in the frozen
    // production failure. The two identity rows keep the problem full rank.
    let large = f64::from(1_000_000_f32);
    let matrix = DMatrix::from_row_slice(3, 2, &[large, large, 1., 0., 0., 1.]);
    let svd = matrix.clone().svd(true, true);
    for measurement in [0.02_f32, 0.03, 0.04, 0.05] {
        let rhs = DVector::from_vec(vec![f64::from(measurement), 1., -1.]);
        // In sum/difference coordinates the exact solution satisfies x-y=2
        // and (2*M*M+1)*(x+y)=2*M*b[0]. No normal matrix is inverted here.
        let correction = large * rhs[0] / (2. * large * large + 1.);
        let analytic = DVector::from_vec(vec![1. + correction, -1. + correction]);
        let reference = svd.solve(&rhs, 1e-14).unwrap();
        assert!((&reference - &analytic).norm() < 1e-9);
        let initial_gradient = (matrix.transpose() * &rhs).norm();
        let threshold = 1e-5 * initial_gradient;
        let gradient = |step: &DVector<f64>| (matrix.transpose() * (&matrix * step - &rhs)).norm();
        // Strong margins over several inputs avoid a fragile tie-to-even case.
        assert!(gradient(&analytic) < threshold * 0.01);
        assert!(gradient(&reference) < threshold * 0.01);
        let rounded = reference.map(|value| f64::from(value as f32));
        assert_eq!(rounded, DVector::from_vec(vec![1., -1.]));
        assert!(gradient(&rounded) > threshold * 1000.);
    }
}
