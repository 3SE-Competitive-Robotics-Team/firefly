/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

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
