use super::{
    reductors::{
        groebner_row5_00_f, groebner_row6_00_f, groebner_row7_00_f, groebner_row7_10_f,
        groebner_row8_00_f,
    },
    spolynomials::{s_polynomial5, s_polynomial6, s_polynomial7, s_polynomial8, s_polynomial9},
};
use nalgebra::SMatrix;

pub fn compute(groebner_matrix: &mut SMatrix<f64, 10, 6>) {
    s_polynomial5(groebner_matrix);

    s_polynomial6(groebner_matrix);
    groebner_row5_00_f(groebner_matrix, 6);

    s_polynomial7(groebner_matrix);
    groebner_row5_00_f(groebner_matrix, 7);
    groebner_row6_00_f(groebner_matrix, 7);

    s_polynomial8(groebner_matrix);
    groebner_row7_10_f(groebner_matrix, 8);
    groebner_row6_00_f(groebner_matrix, 8);
    groebner_row7_00_f(groebner_matrix, 8);

    s_polynomial9(groebner_matrix);
    groebner_row7_00_f(groebner_matrix, 9);
    groebner_row8_00_f(groebner_matrix, 9);
}
