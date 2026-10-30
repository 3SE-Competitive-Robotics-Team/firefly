use super::*;
use faer::linalg::solvers::SolveLstsq;
use proptest::prelude::*;

#[path = "fixtures.rs"]
mod fixtures;
use fixtures::Fixture;

fn verify(fixture: &Fixture) {
    // Independently assemble the augmented least-squares problem, including
    // repeated parameter blocks, and solve with QR rather than normal equations.
    let n = fixture.gradient.len();
    let m: usize = fixture.rows.iter().map(|row| row.residual.len()).sum();
    let mut design = faer::Mat::<f64>::zeros(m + n, n);
    let mut rhs = faer::Mat::<f64>::zeros(m + n, 1);
    let mut start = 0;
    for row in &fixture.rows {
        for r in 0..row.residual.len() {
            rhs[(start + r, 0)] = -row.residual[r];
            for (offset, block) in &row.blocks {
                for c in 0..block.ncols() {
                    design[(start + r, offset + c)] += block[(r, c)];
                }
            }
        }
        start += row.residual.len();
    }
    for c in 0..n {
        design[(m + c, c)] = fixture.regularization[c].sqrt();
    }
    let expected = design.qr().solve_lstsq(rhs.as_ref());
    let expected = DVector::from_iterator(n, (0..n).map(|i| expected[(i, 0)]));
    // Exercise the production dispatch as well as the individual assemblers.
    let model = Linearization {
        parallel: false,
        cost: 0.0,
        fixed_cost: 0.0,
        rows: fixture
            .rows
            .iter()
            .map(|row| Row {
                residual: row.residual.clone(),
                blocks: row.blocks.clone(),
            })
            .collect(),
        gradient: fixture.gradient.clone(),
        diagonal: DVector::zeros(n),
        eliminated_points: fixture.system().eliminated_points.to_vec(),
    };
    for actual in [
        fixture.system().dense(&fixture.regularization),
        fixture.system().block(&fixture.regularization),
        fixture
            .system()
            .assemble()
            .solve(&fixture.regularization, false),
        model.solve(&fixture.regularization, "DENSE_SCHUR"),
    ] {
        let actual = actual.unwrap();
        assert!((&actual - &expected).amax() <= 1e-9 * (1.0 + expected.amax()));
        let mut normal_residual =
            fixture.gradient.clone() + fixture.regularization.component_mul(&actual);
        for row in &fixture.rows {
            let mut product = DVector::zeros(row.residual.len());
            for (offset, j) in &row.blocks {
                product.gemv(1.0, j, &actual.rows(*offset, j.ncols()), 1.0);
            }
            for (offset, j) in &row.blocks {
                normal_residual
                    .rows_mut(*offset, j.ncols())
                    .gemv_tr(1.0, j, &product, 1.0);
            }
        }
        assert!(normal_residual.amax() <= 1e-9 * (1.0 + fixture.gradient.amax()));
    }
}

#[test]
fn assembled_normal_reuses_only_undamped_values_after_failed_and_successful_solves() {
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(4)
        .build()
        .unwrap();
    for coupled in [false, true] {
        let mut fixture = Fixture::window(3, 65, 2);
        if coupled {
            fixture.rows.push(Row {
                residual: DVector::from_element(3, 0.7),
                blocks: vec![
                    (0, DMatrix::identity(3, 3)),
                    (3, DMatrix::identity(3, 3) * 0.4),
                ],
            });
            fixture.update_gradient();
        }
        let normal = fixture.system().assemble();
        let invalid = DVector::from_element(fixture.gradient.len(), f64::NAN);
        assert!(normal.solve(&invalid, false).is_none());
        for damping in [0.0, 1e-5, 2.0, 0.0, 0.1] {
            fixture.regularization.fill(damping);
            verify(&fixture);
            let expected = fixture.system().block(&fixture.regularization);
            assert_eq!(normal.solve(&fixture.regularization, false), expected);
            assert_eq!(
                pool.install(|| normal.solve(&fixture.regularization, true)),
                expected
            );
        }
    }
    let gradient = DVector::from_element(3, 1.0);
    let normal = SchurSystem {
        rows: &[],
        gradient: &gradient,
        eliminated_points: &[0],
    }
    .assemble();
    assert!(normal.solve(&DVector::zeros(3), false).is_none());
    assert_eq!(
        normal.solve(&DVector::from_element(3, 4.0), false),
        Some(DVector::from_element(3, -0.25))
    );
}

#[test]
fn parallel_landmark_elimination_and_back_substitution_match_serial_including_coupling() {
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(4)
        .build()
        .unwrap();
    for points in [0, 1, 65, 150] {
        let fixture = Fixture::window(4, points, 3);
        assert_eq!(
            fixture.system().block(&fixture.regularization),
            pool.install(|| fixture
                .system()
                .block_with_parallel(&fixture.regularization, true))
        );
    }
    let mut fixture = Fixture::window(2, 3, 2);
    fixture.rows.push(Row {
        residual: DVector::from_element(3, 0.7),
        blocks: vec![
            (0, DMatrix::identity(3, 3)),
            (3, DMatrix::identity(3, 3) * 0.4),
        ],
    });
    fixture.update_gradient();
    assert_eq!(
        fixture.system().dense(&fixture.regularization),
        pool.install(|| fixture
            .system()
            .block_with_parallel(&fixture.regularization, true))
    );
    fixture.rows[0].blocks[0].1.fill(1e200);
    assert!(
        pool.install(|| fixture
            .system()
            .block_with_parallel(&fixture.regularization, true))
            .is_none()
    );
}

#[test]
fn block_and_dense_match_independent_augmented_qr() {
    for frames in [1, 2, 4] {
        for points in [0, 1, 5] {
            for views in [1, frames] {
                for damping in [0.0, 1e-5, 0.5] {
                    let mut fixture = Fixture::window(frames, points, views);
                    fixture.regularization *= damping;
                    verify(&fixture);
                }
            }
        }
    }
}

#[test]
fn interleaved_points_repeated_blocks_and_scaling_match_qr() {
    let mut fixture = Fixture::window(3, 4, 2);
    // Move all landmarks between two retained frame states.
    let p = 12;
    let permutation = |c: usize| {
        if c < p {
            c + 15
        } else if c < p + 15 {
            c - p
        } else {
            c
        }
    };
    for row in &mut fixture.rows {
        for (offset, block) in &mut row.blocks {
            *offset = permutation(*offset);
            for c in 0..block.ncols() {
                block
                    .column_mut(c)
                    .scale_mut(0.2 + ((*offset + c) % 9) as f64 * 0.1);
            }
        }
        let (offset, block) = row.blocks[0].clone();
        row.blocks.push((offset, block * 0.3));
        row.blocks.reverse();
    }
    fixture
        .points
        .iter_mut()
        .for_each(|offset| *offset = permutation(*offset));
    fixture.points.reverse();
    fixture.update_gradient();
    verify(&fixture);
}

#[test]
fn coupled_landmarks_preserve_complete_system() {
    let mut fixture = Fixture::window(2, 3, 2);
    fixture.rows.push(Row {
        residual: DVector::from_element(3, 0.7),
        blocks: vec![
            (0, DMatrix::identity(3, 3)),
            (3, DMatrix::identity(3, 3) * 0.4),
        ],
    });
    fixture.update_gradient();
    verify(&fixture);
}

#[test]
fn empty_all_point_and_singular_systems() {
    let rows = vec![Row {
        residual: DVector::from_element(3, 1.0),
        blocks: vec![(0, DMatrix::identity(3, 3))],
    }];
    let fixture = Fixture {
        rows,
        gradient: DVector::from_element(3, 1.0),
        points: vec![0],
        regularization: DVector::zeros(3),
    };
    verify(&fixture);
    let empty = SchurSystem {
        rows: &[],
        gradient: &DVector::zeros(0),
        eliminated_points: &[],
    };
    assert_eq!(empty.block(&DVector::zeros(0)), Some(DVector::zeros(0)));
    assert_eq!(empty.dense(&DVector::zeros(0)), Some(DVector::zeros(0)));
    for points in [vec![], vec![0]] {
        let system = SchurSystem {
            rows: &[],
            gradient: &DVector::zeros(3),
            eliminated_points: &points,
        };
        assert!(system.block(&DVector::zeros(3)).is_none());
        assert!(system.dense(&DVector::zeros(3)).is_none());
        assert_eq!(
            system.block(&DVector::from_element(3, 1.0)),
            Some(DVector::zeros(3))
        );
    }
}

#[test]
fn damping_enters_landmark_blocks_before_elimination() {
    let rows = [Row {
        residual: DVector::zeros(4),
        blocks: vec![
            (
                0,
                DMatrix::from_row_slice(4, 3, &[2., 0., 0., 0., 2., 0., 0., 0., 2., 0., 0., 0.]),
            ),
            (
                3,
                DMatrix::from_column_slice(4, 1, &[0.5, 1., 0., 19.0_f64.sqrt() / 2.]),
            ),
        ],
    }];
    let system = SchurSystem {
        rows: &rows,
        gradient: &DVector::from_vec(vec![2., 4., 6., 8.]),
        eliminated_points: &[0],
    };
    // C = diag(5,6,7), E = (1,2,0), B = 10. S = 137/15, rhs = -94/15.
    let damping = DVector::from_vec(vec![1., 2., 3., 4.]);
    let expected = DVector::from_vec(vec![-36. / 137., -60. / 137., -6. / 7., -94. / 137.]);
    for actual in [system.dense(&damping), system.block(&damping)] {
        assert!((actual.unwrap() - &expected).amax() < 2e-15);
    }
}

#[test]
fn unobserved_points_require_damping_and_preserve_global_order() {
    let system = SchurSystem {
        rows: &[],
        gradient: &DVector::from_vec(vec![1., 2., 3., 4., 5., 6., 7.]),
        eliminated_points: &[4, 0],
    };
    let damping = DVector::from_vec(vec![2., 4., 6., 8., 10., 12., 14.]);
    for actual in [system.dense(&damping), system.block(&damping)] {
        assert!((actual.unwrap() - DVector::from_element(7, -0.5)).amax() < 2e-15);
    }
    let mut damping = damping;
    damping[5] = 0.;
    assert!(system.dense(&damping).is_none());
    assert!(system.block(&damping).is_none());
}

#[test]
fn singular_reduced_system_is_not_reported_as_a_valid_step() {
    let rows = [Row {
        residual: DVector::zeros(3),
        blocks: vec![
            (0, DMatrix::identity(3, 3)),
            (3, DMatrix::from_column_slice(3, 1, &[1., 0., 0.])),
        ],
    }];
    let system = SchurSystem {
        rows: &rows,
        gradient: &DVector::from_element(4, 1.),
        eliminated_points: &[0],
    };
    let damping = DVector::zeros(4);
    assert!(system.dense(&damping).is_none());
    assert!(system.block(&damping).is_none());
}

#[test]
fn finite_jacobians_with_overflowing_normal_diagonals_are_rejected() {
    for points in [vec![], vec![0]] {
        let rows = [Row {
            residual: DVector::from_element(3, 1e-200),
            blocks: vec![(0, DMatrix::identity(3, 3) * 1e200)],
        }];
        let system = SchurSystem {
            rows: &rows,
            gradient: &DVector::from_element(3, 1.),
            eliminated_points: &points,
        };
        assert!(
            system.dense(&DVector::zeros(3)).is_none(),
            "dense accepted an overflowed normal system"
        );
        assert!(
            system.block(&DVector::zeros(3)).is_none(),
            "block accepted an overflowed normal system"
        );
    }
}

#[test]
fn nonfinite_rhs_never_produces_a_successful_step() {
    for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for points in [vec![], vec![0]] {
            let rows = [Row {
                residual: DVector::zeros(3),
                blocks: vec![(0, DMatrix::identity(3, 3))],
            }];
            let system = SchurSystem {
                rows: &rows,
                gradient: &DVector::from_vec(vec![1., invalid, 2.]),
                eliminated_points: &points,
            };
            assert!(system.dense(&DVector::zeros(3)).is_none());
            assert!(system.block(&DVector::zeros(3)).is_none());
        }
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]
    #[test]
    fn randomized_jacobians_match_qr_and_normal_residual(
        frames in 1usize..5,
        points in 0usize..9,
        views in 1usize..5,
        damping in 0.01f64..1.0,
        values in proptest::collection::vec(-2.0f64..2.0, 512),
    ) {
        let mut fixture = Fixture::window(frames, points, views);
        let mut values = values.iter().cycle();
        for row in &mut fixture.rows {
            for (_, block) in &mut row.blocks {
                for value in block.iter_mut() {
                    *value = *values.next().unwrap();
                }
            }
        }
        fixture.regularization.fill(damping);
        fixture.update_gradient();
        verify(&fixture);
    }
}
