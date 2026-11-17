//! Sparse Gauss–Jordan elimination with partial row pivoting.
/// The left square block must be invertible. `exit_condition` defaults to zero
/// in the C++ interface and is the last row affected by upward elimination.
/// Returns false for an invalid shape or a pivot column containing only NaNs.
pub fn gauss_jordan(matrix: &mut [Vec<f64>], exit_condition: usize) -> bool {
    let precision = 1.0e-10;
    let rows = matrix.len();
    if rows == 0 {
        return false;
    }
    let cols = matrix[0].len();
    if cols < rows || matrix.iter().any(|r| r.len() != cols) {
        return false;
    }
    for front in 0..rows {
        let mut max_value = -1.0;
        let mut pivot = None;
        for (row, values) in matrix.iter().enumerate().skip(front) {
            let value = values[front].abs();
            if value > max_value {
                pivot = Some(row);
                max_value = value;
            }
        }
        let Some(pivot) = pivot else {
            return false;
        };
        matrix.swap(pivot, front);
        let inverse = 1.0 / matrix[front][front];
        matrix[front][front] = 1.0;
        for entry in matrix[front].iter_mut().take(cols).skip(front + 1) {
            *entry *= inverse;
        }
        let nonzero: Vec<_> = (front..cols)
            .filter(|&c| matrix[front][c].abs() > precision)
            .collect();
        for row in front + 1..rows {
            let leading = matrix[row][front];
            if leading.abs() > precision {
                for &col in &nonzero {
                    matrix[row][col] -= leading * matrix[front][col];
                }
            }
        }
    }
    let mut front = rows - 1;
    while front > exit_condition {
        let nonzero: Vec<_> = (front..cols)
            .filter(|&c| matrix[front][c].abs() > precision)
            .collect();
        for row in (exit_condition..front).rev() {
            let leading = matrix[row][front];
            if leading.abs() > precision {
                for &col in &nonzero {
                    matrix[row][col] -= leading * matrix[front][col];
                }
            }
        }
        front -= 1;
    }
    true
}
