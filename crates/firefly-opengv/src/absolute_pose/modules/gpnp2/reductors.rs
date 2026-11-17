use nalgebra::SMatrix;

pub fn groebner_row5_00_f(groebner_matrix: &mut SMatrix<f64, 10, 6>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 1)] / groebner_matrix[(5, 1)];
    groebner_matrix[(target_row, 1)] = 0.0;
    groebner_matrix[(target_row, 2)] -= factor * groebner_matrix[(5, 2)];
    groebner_matrix[(target_row, 3)] -= factor * groebner_matrix[(5, 3)];
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(5, 4)];
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(5, 5)];
}

pub fn groebner_row6_00_f(groebner_matrix: &mut SMatrix<f64, 10, 6>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 2)] / groebner_matrix[(6, 2)];
    groebner_matrix[(target_row, 2)] = 0.0;
    groebner_matrix[(target_row, 3)] -= factor * groebner_matrix[(6, 3)];
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(6, 4)];
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(6, 5)];
}

pub fn groebner_row7_10_f(groebner_matrix: &mut SMatrix<f64, 10, 6>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 1)] / groebner_matrix[(7, 3)];
    groebner_matrix[(target_row, 1)] = 0.0;
    groebner_matrix[(target_row, 2)] -= factor * groebner_matrix[(7, 4)];
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(7, 5)];
}

pub fn groebner_row7_00_f(groebner_matrix: &mut SMatrix<f64, 10, 6>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 3)] / groebner_matrix[(7, 3)];
    groebner_matrix[(target_row, 3)] = 0.0;
    groebner_matrix[(target_row, 4)] -= factor * groebner_matrix[(7, 4)];
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(7, 5)];
}

pub fn groebner_row8_00_f(groebner_matrix: &mut SMatrix<f64, 10, 6>, target_row: usize) {
    let factor = groebner_matrix[(target_row, 4)] / groebner_matrix[(8, 4)];
    groebner_matrix[(target_row, 4)] = 0.0;
    groebner_matrix[(target_row, 5)] -= factor * groebner_matrix[(8, 5)];
}
