//! Deterministic synthetic linearizations with visual/IMU-sized blocks.
//! These exercise graph connectivity, not recorded trajectories or sensor accuracy.
use super::{DMatrix, DVector, Row, SchurSystem};

impl SchurSystem<'_> {
    pub fn dense(&self, regularization: &DVector<f64>) -> Option<DVector<f64>> {
        self.solve_dense_normal(&self.lower_normal(regularization))
    }
}

pub struct Fixture {
    pub rows: Vec<Row>,
    pub gradient: DVector<f64>,
    pub points: Vec<usize>,
    pub regularization: DVector<f64>,
}

impl Fixture {
    pub fn window(frames: usize, points: usize, views: usize) -> Self {
        let retained = 3 * points;
        let extrinsics = retained + 15 * frames;
        let count = extrinsics + 6;
        let mut rows = Vec::new();
        // Weak priors make the zero-damping cases independently well posed.
        for point in 0..points {
            rows.push(Row {
                residual: DVector::from_element(3, 0.01),
                blocks: vec![(3 * point, DMatrix::identity(3, 3) * 0.1)],
            });
        }
        for frame in 0..frames {
            rows.push(Row {
                residual: DVector::from_fn(15, |i, _| (i as f64).cos() * 0.02),
                blocks: vec![(retained + frame * 15, DMatrix::identity(15, 15))],
            });
            if frame > 0 {
                rows.push(Row {
                    residual: DVector::from_element(15, 0.03),
                    blocks: vec![
                        (retained + (frame - 1) * 15, matrix(15, 15, frame)),
                        (retained + frame * 15, matrix(15, 15, frame + 13)),
                    ],
                });
            }
        }
        rows.push(Row {
            residual: DVector::from_element(6, 0.02),
            blocks: vec![(extrinsics, DMatrix::identity(6, 6))],
        });
        for point in 0..points {
            for view in 0..views.min(frames) {
                let frame = (point + view) % frames;
                let seed = point * 17 + view * 7;
                rows.push(Row {
                    residual: DVector::from_vec(vec![(seed as f64).sin(), 0.1]),
                    blocks: vec![
                        (retained + frame * 15, matrix(2, 6, seed + 1)),
                        (point * 3, matrix(2, 3, seed + 2)),
                        (extrinsics, matrix(2, 6, seed + 3)),
                    ],
                });
            }
        }
        let mut fixture = Self {
            rows,
            gradient: DVector::zeros(count),
            points: (0..points).map(|p| 3 * p).collect(),
            regularization: DVector::from_fn(count, |i, _| 0.01 + (i % 7) as f64 * 0.003),
        };
        fixture.update_gradient();
        fixture
    }

    pub fn update_gradient(&mut self) {
        self.gradient.fill(0.0);
        for row in &self.rows {
            for (offset, block) in &row.blocks {
                self.gradient.rows_mut(*offset, block.ncols()).gemv_tr(
                    1.0,
                    block,
                    &row.residual,
                    1.0,
                );
            }
        }
    }

    pub fn system(&self) -> SchurSystem<'_> {
        SchurSystem {
            rows: &self.rows,
            gradient: &self.gradient,
            eliminated_points: &self.points,
        }
    }
}

fn matrix(rows: usize, columns: usize, seed: usize) -> DMatrix<f64> {
    DMatrix::from_fn(rows, columns, |r, c| {
        ((seed * 31 + r * 11 + c * 19) as f64 * 0.17).sin() + if r == c { 0.7 } else { 0.0 }
    })
}
