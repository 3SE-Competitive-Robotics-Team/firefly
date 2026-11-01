use firefly_basalt::optimization::accumulator::SparseHashAccumulator;
use nalgebra::{DVector, SMatrix, SVector};
use serde_json::Value;
fn check(actual: &DVector<f64>, expected: &Value) {
    for (i, value) in expected.as_array().unwrap().iter().enumerate() {
        let expected = value.as_f64().unwrap();
        assert!(
            (actual[i] - expected).abs() <= 1e-10 * (1. + expected.abs()),
            "index {i}: {} != {expected}",
            actual[i]
        );
    }
}
#[test]
fn original_sparse_solvers() {
    let data: Value =
        serde_json::from_str(include_str!("reference/sparse_accumulator.json")).unwrap();
    for case in data.as_array().unwrap() {
        let size = usize::try_from(case["size"].as_u64().unwrap()).unwrap();
        let mut accumulator = SparseHashAccumulator::<f64>::default();
        accumulator.reset(size);
        for triplet in case["additions"].as_array().unwrap() {
            accumulator.add_h(
                usize::try_from(triplet[0].as_u64().unwrap()).unwrap(),
                usize::try_from(triplet[1].as_u64().unwrap()).unwrap(),
                &SMatrix::<f64, 1, 1>::repeat(triplet[2].as_f64().unwrap()),
            );
        }
        for (i, value) in case["rhs"].as_array().unwrap().iter().enumerate() {
            accumulator.add_b(i, &SVector::<f64, 1>::repeat(value.as_f64().unwrap()));
        }
        accumulator.setup_solver();
        check(&accumulator.h_diagonal(), &case["diagonal"]);
        let damping = DVector::repeat(size, 0.25);
        check(&accumulator.solve(None), &case["direct"]);
        check(&accumulator.solve(Some(&damping)), &case["damped"]);
        accumulator.iterative_solver = true;
        check(&accumulator.solve(None), &case["cg"]);
        check(&accumulator.solve(Some(&damping)), &case["cg_damped"]);
    }
}
