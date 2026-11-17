use nalgebra::SMatrix;

pub fn groebner_row3_0_f(groebner_matrix: &mut SMatrix<f64, 5, 3>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 1)] / groebner_matrix[(3, 1)];
    groebner_matrix[(target_row, 1)] = 0.0;
    groebner_matrix[(target_row, 2)] -= factor * groebner_matrix[(3, 2)];
}
