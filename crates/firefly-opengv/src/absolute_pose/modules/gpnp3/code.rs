use super::{
    reductors::{
        groebner_row3_000_f, groebner_row3_100_f, groebner_row4_000_f, groebner_row4_010_f,
        groebner_row4_100_f, groebner_row5_000_f, groebner_row5_010_f, groebner_row5_100_f,
        groebner_row6_000_f, groebner_row6_100_f, groebner_row7_000_f, groebner_row8_000_f,
        groebner_row9_000_f, groebner_row9_100_f, groebner_row10_000_f, groebner_row10_100_f,
        groebner_row11_000_f, groebner_row11_010_f, groebner_row11_100_f, groebner_row12_000_f,
        groebner_row12_010_f, groebner_row12_100_f, groebner_row13_000_f,
    },
    spolynomials::{
        s_polynomial4, s_polynomial5, s_polynomial6, s_polynomial7, s_polynomial8, s_polynomial9,
        s_polynomial10, s_polynomial11, s_polynomial12, s_polynomial13, s_polynomial14,
    },
};
use nalgebra::SMatrix;

pub fn compute(groebner_matrix: &mut SMatrix<f64, 15, 18>) {
    s_polynomial4(groebner_matrix);

    s_polynomial5(groebner_matrix);
    groebner_row4_000_f(groebner_matrix, 5);

    s_polynomial6(groebner_matrix);
    groebner_row4_000_f(groebner_matrix, 6);
    groebner_row5_000_f(groebner_matrix, 6);

    s_polynomial7(groebner_matrix);
    groebner_row5_100_f(groebner_matrix, 7);
    groebner_row6_100_f(groebner_matrix, 7);
    groebner_row4_000_f(groebner_matrix, 7);
    groebner_row5_000_f(groebner_matrix, 7);
    groebner_row6_000_f(groebner_matrix, 7);

    s_polynomial8(groebner_matrix);
    groebner_row4_100_f(groebner_matrix, 8);
    groebner_row5_100_f(groebner_matrix, 8);
    groebner_row6_100_f(groebner_matrix, 8);
    groebner_row7_000_f(groebner_matrix, 8);
    groebner_row3_000_f(groebner_matrix, 8);
    groebner_row4_000_f(groebner_matrix, 8);
    groebner_row5_000_f(groebner_matrix, 8);
    groebner_row6_000_f(groebner_matrix, 8);

    s_polynomial9(groebner_matrix);
    groebner_row5_010_f(groebner_matrix, 9);
    groebner_row3_100_f(groebner_matrix, 9);
    groebner_row4_100_f(groebner_matrix, 9);
    groebner_row5_100_f(groebner_matrix, 9);
    groebner_row6_100_f(groebner_matrix, 9);
    groebner_row7_000_f(groebner_matrix, 9);
    groebner_row8_000_f(groebner_matrix, 9);
    groebner_row3_000_f(groebner_matrix, 9);
    groebner_row4_000_f(groebner_matrix, 9);
    groebner_row5_000_f(groebner_matrix, 9);
    groebner_row6_000_f(groebner_matrix, 9);

    s_polynomial10(groebner_matrix);
    groebner_row4_010_f(groebner_matrix, 10);
    groebner_row5_010_f(groebner_matrix, 10);
    groebner_row3_100_f(groebner_matrix, 10);
    groebner_row4_100_f(groebner_matrix, 10);
    groebner_row5_100_f(groebner_matrix, 10);
    groebner_row6_100_f(groebner_matrix, 10);
    groebner_row9_100_f(groebner_matrix, 10);
    groebner_row8_000_f(groebner_matrix, 10);
    groebner_row3_000_f(groebner_matrix, 10);
    groebner_row4_000_f(groebner_matrix, 10);
    groebner_row5_000_f(groebner_matrix, 10);
    groebner_row6_000_f(groebner_matrix, 10);
    groebner_row9_000_f(groebner_matrix, 10);

    s_polynomial11(groebner_matrix);
    groebner_row6_000_f(groebner_matrix, 11);
    groebner_row9_000_f(groebner_matrix, 11);
    groebner_row10_000_f(groebner_matrix, 11);

    s_polynomial12(groebner_matrix);
    groebner_row10_100_f(groebner_matrix, 12);
    groebner_row11_100_f(groebner_matrix, 12);
    groebner_row9_000_f(groebner_matrix, 12);
    groebner_row10_000_f(groebner_matrix, 12);
    groebner_row11_000_f(groebner_matrix, 12);

    s_polynomial13(groebner_matrix);
    groebner_row10_100_f(groebner_matrix, 13);
    groebner_row11_010_f(groebner_matrix, 13);
    groebner_row12_010_f(groebner_matrix, 13);
    groebner_row11_100_f(groebner_matrix, 13);
    groebner_row12_100_f(groebner_matrix, 13);
    groebner_row10_000_f(groebner_matrix, 13);
    groebner_row11_000_f(groebner_matrix, 13);
    groebner_row12_000_f(groebner_matrix, 13);

    s_polynomial14(groebner_matrix);
    groebner_row11_000_f(groebner_matrix, 14);
    groebner_row12_000_f(groebner_matrix, 14);
    groebner_row13_000_f(groebner_matrix, 14);
}
