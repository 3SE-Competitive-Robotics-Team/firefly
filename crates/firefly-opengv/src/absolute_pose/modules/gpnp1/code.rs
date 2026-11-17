use super::{
    reductors::groebner_row3_0_f,
    spolynomials::{s_polynomial3, s_polynomial4},
};
use nalgebra::SMatrix;

pub fn compute(groebner_matrix: &mut SMatrix<f64, 5, 3>) {
    s_polynomial3(groebner_matrix);

    s_polynomial4(groebner_matrix);
    groebner_row3_0_f(groebner_matrix, 4);
}
