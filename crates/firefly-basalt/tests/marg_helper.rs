use firefly_basalt::utils::complete_orthogonal_decomposition::CompleteOrthogonalDecomposition;
use firefly_basalt::vi_estimator::marg_helper::MargHelper;
use firefly_basalt_headers::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector};
use serde_json::Value;
use std::collections::BTreeSet;

fn matrix<S: BasaltScalar>(value: &Value) -> DMatrix<S> {
    let values: Vec<S> = value["values"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| S::from_f64_lossy(v.as_f64().unwrap()))
        .collect();
    DMatrix::from_row_slice(
        value["rows"].as_u64().unwrap() as usize,
        value["cols"].as_u64().unwrap() as usize,
        &values,
    )
}
fn check<S: BasaltScalar>(actual: &DMatrix<S>, expected: &Value, tolerance: f64, label: &str) {
    let expected = matrix::<S>(expected);
    assert_eq!(actual.shape(), expected.shape(), "{label}");
    for r in 0..actual.nrows() {
        for c in 0..actual.ncols() {
            let a = actual[(r, c)].as_f64();
            let e = expected[(r, c)].as_f64();
            assert!(
                (a - e).abs() <= tolerance * (1. + e.abs()),
                "{label} [{r},{c}]: {a} != {e}"
            );
        }
    }
}
#[expect(
    clippy::too_many_lines,
    reason = "Checks each original marginalization overload against the same independent input"
)]
fn run<S: BasaltScalar>(text: &str, tolerance: f64) {
    let data: Value = serde_json::from_str(text).unwrap();
    for case in data["ldlt"].as_array().unwrap() {
        let factor = firefly_basalt::utils::ldlt::Ldlt::new(matrix::<S>(&case["input"]));
        check(
            &DMatrix::from_column_slice(factor.diagonal().len(), 1, factor.diagonal().as_slice()),
            &case["diagonal"],
            tolerance,
            "LDLT D",
        );
        check(&factor.lower(), &case["lower"], tolerance, "LDLT L");
        check(
            &factor.solve(&matrix::<S>(&case["rhs"])),
            &case["solution"],
            tolerance,
            "LDLT solve",
        );
    }
    {
        use firefly_basalt::linearization::block_diagonal::{
            BlockDiagonalAccumulator, scale_jacobians,
        };
        let mut accumulator = BlockDiagonalAccumulator::<S>::default();
        let mut other = BlockDiagonalAccumulator::<S>::default();
        let first = DMatrix::from_row_slice(2, 2, &[1., 2., 3., 4.].map(S::from_f64_lossy));
        accumulator.add(0, first.clone());
        other.add(0, DMatrix::identity(2, 2));
        other.add(2, first * S::from_f64_lossy(2.));
        accumulator.join(&mut other);
        accumulator.add_diag(
            3,
            2,
            &DVector::from_vec(
                [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
                    .map(S::from_f64_lossy)
                    .to_vec(),
            ),
        );
        scale_jacobians(
            &mut accumulator.block_diagonal,
            3,
            2,
            &DVector::from_vec(
                [2., -1., 0.5, 3., -2., 0.25]
                    .map(S::from_f64_lossy)
                    .to_vec(),
            ),
        );
        for i in 0..3 {
            check(
                &accumulator.block_diagonal[&i],
                &data["blocks"][i],
                tolerance,
                "block diagonal",
            );
        }
        check(
            &other.block_diagonal[&0],
            &data["joined_existing"],
            tolerance,
            "join existing",
        );
        check(
            &other.block_diagonal[&2],
            &data["joined_moved"],
            tolerance,
            "join moved",
        );
    }
    for (index, case) in data["cod"].as_array().unwrap().iter().enumerate() {
        let cod = CompleteOrthogonalDecomposition::new(matrix::<S>(&case["input"]));
        assert_eq!(cod.rank(), case["rank"].as_u64().unwrap() as usize);
        check(
            &cod.pseudo_inverse(),
            &case["inverse"],
            tolerance,
            &format!("COD {index}"),
        );
    }
    let keep = BTreeSet::from([1, 4, 5]);
    let marg = BTreeSet::from([0, 2, 3]);
    for case in data["marginalization"].as_array().unwrap() {
        for path in ["sq_sq", "sq_sqrt", "sqrt_sqrt"] {
            let (input, rhs) = if path == "sqrt_sqrt" {
                ("j", "r")
            } else {
                ("h", "b")
            };
            let mut h = matrix::<S>(&case[input]);
            let mut b = matrix::<S>(&case[rhs]).column(0).into_owned();
            let (result, rhs) = match path {
                "sq_sq" => MargHelper::marginalize_sq_to_sq(&mut h, &mut b, &keep, &marg),
                "sq_sqrt" => MargHelper::marginalize_sq_to_sqrt(&mut h, &mut b, &keep, &marg),
                _ => MargHelper::marginalize_sqrt_to_sqrt(&mut h, &mut b, &keep, &marg),
            };
            assert_eq!(h.shape(), (0, 0));
            assert!(b.is_empty());
            let label = format!("{} {path}", case["name"]);
            check(&result, &case[path]["h"], tolerance, &label);
            check(
                &DMatrix::from_column_slice(rhs.len(), 1, rhs.as_slice()),
                &case[path]["b"],
                tolerance,
                &label,
            );
        }
    }
}
#[test]
fn original_marginalization_f64() {
    run::<f64>(include_str!("reference/marg_helper_f64.json"), 1e-10);
}
#[test]
fn original_marginalization_f32() {
    run::<f32>(include_str!("reference/marg_helper_f32.json"), 1e-5);
}

#[test]
fn ldlt_pseudoinverse_cutoff() {
    use firefly_basalt::utils::ldlt::Ldlt;
    let diagonal = DVector::from_vec(vec![2., -3., f64::MIN_POSITIVE, f64::MIN_POSITIVE / 2., 0.]);
    let factor = Ldlt::new(DMatrix::from_diagonal(&diagonal));
    let solution = factor.solve_vector(&DVector::from_element(5, 1.));
    assert_eq!(solution, DVector::from_vec(vec![0.5, -1. / 3., 0., 0., 0.]));
}
