use firefly_basalt::optimization::accumulator::DenseAccumulator;
use nalgebra::{DVector, Matrix2, Vector2};

#[test]
fn dense_accumulator_preserves_solve_sign_damping_and_join() {
    let mut first = DenseAccumulator::<f64>::default();
    first.reset(2);
    first.add_h(0, 0, &Matrix2::new(2.0, 0.5, 0.5, 1.5));
    first.add_b(0, &Vector2::new(0.5, 1.0));
    let second = first.clone();
    first.join(&second);
    first.setup_solver();
    assert_eq!(first.h_diagonal(), DVector::from_vec(vec![4.0, 3.0]));
    let solution = first.solve(None);
    assert!((solution - DVector::from_vec(vec![1.0 / 11.0, 7.0 / 11.0])).amax() < 1e-15);
    let solution = first.solve(Some(&DVector::from_vec(vec![1.0, 2.0])));
    assert!((solution - DVector::from_vec(vec![0.125, 0.375])).amax() < 1e-15);
    first.reset(3);
    assert_eq!(first.h().shape(), (3, 3));
    assert_eq!(first.h().amax(), 0.0);
    assert_eq!(first.b().amax(), 0.0);
}

#[test]
fn ldlt_solve_reads_lower_triangle_as_upstream() {
    let mut accumulator = DenseAccumulator::<f32>::default();
    accumulator.reset(2);
    accumulator.add_h(0, 0, &Matrix2::new(4.0, 999.0, 1.0, 3.0));
    accumulator.add_b(0, &Vector2::new(1.0, 2.0));
    let solution = accumulator.solve(None);
    assert!((solution - DVector::from_vec(vec![1.0 / 11.0, 7.0 / 11.0])).amax() < 1e-6);
}
