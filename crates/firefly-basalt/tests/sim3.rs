use firefly_basalt::utils::sophus_utils::right_jacobian_inv_sim3_decoupled;
use firefly_basalt::utils::sophus_utils::right_jacobian_sim3_decoupled;
use firefly_basalt::utils::sophus_utils::sim3_expd;
use firefly_basalt::utils::sophus_utils::sim3_logd;
use nalgebra::SVector;

#[test]
fn decoupled_similarity_jacobian_matches_group_finite_difference() {
    let x = SVector::<f64, 7>::from_row_slice(&[0.3, -0.2, 0.8, 0.15, -0.12, 0.07, 0.4]);
    let pose = sim3_expd(&x);
    assert!((sim3_logd(&pose) - x).norm() < 1e-14);
    let jacobian = right_jacobian_sim3_decoupled(&x);
    let inverse = right_jacobian_inv_sim3_decoupled(&x);
    assert!((jacobian * inverse - nalgebra::SMatrix::<f64, 7, 7>::identity()).norm() < 1e-14);
    for column in 0..7 {
        let mut delta = SVector::<f64, 7>::zeros();
        delta[column] = 1e-6;
        let direct = (sim3_expd(&(x + delta)).matrix() - sim3_expd(&(x - delta)).matrix()) / (2e-6);
        let right = pose.matrix()
            * (sim3_expd(&(jacobian * delta)).matrix() - sim3_expd(&(-jacobian * delta)).matrix())
            / (2e-6);
        assert!((direct - right).norm() < 1e-9, "column {column}");
    }
}

#[test]
fn similarity_scale_saturates_like_sophus() {
    for scale in [-1000.0, 1000.0] {
        let mut x = SVector::<f64, 7>::zeros();
        x[6] = scale;
        let y = sim3_logd(&sim3_expd(&x));
        let epsilon_plus = 1e-10 * (1.0 + 1e-10);
        let expected = if scale < 0.0 {
            epsilon_plus
        } else {
            1.0 / epsilon_plus
        };
        assert!((y[6].exp() / expected - 1.0).abs() < 1e-14);
    }
}
