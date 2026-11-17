use nalgebra::SMatrix;

pub fn s_polynomial3(groebner_matrix: &mut SMatrix<f64, 5, 3>) {
    groebner_matrix[(3, 1)] = groebner_matrix[(0, 1)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 1)] / (groebner_matrix[(1, 0)]);
    groebner_matrix[(3, 2)] = groebner_matrix[(0, 2)] / (groebner_matrix[(0, 0)])
        - groebner_matrix[(1, 2)] / (groebner_matrix[(1, 0)]);
}

pub fn s_polynomial4(groebner_matrix: &mut SMatrix<f64, 5, 3>) {
    groebner_matrix[(4, 1)] = groebner_matrix[(1, 1)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 1)] / (groebner_matrix[(2, 0)]);
    groebner_matrix[(4, 2)] = groebner_matrix[(1, 2)] / (groebner_matrix[(1, 0)])
        - groebner_matrix[(2, 2)] / (groebner_matrix[(2, 0)]);
}
