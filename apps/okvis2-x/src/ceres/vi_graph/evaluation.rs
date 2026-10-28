//! Exclusive factor batches, reusable result slots, and ordered numerical reduction.
use super::{
    BTreeMap, BTreeSet, DVector, FactorRecord, Linearization, LossFunction, ManifoldKind,
    ResidualId, Result, Row, Snapshot, Values, graph_error,
};
use rayon::prelude::*;

const FACTORS_PER_BATCH: usize = 32;

#[derive(Default)]
pub(super) struct Workspace {
    pub parallel: bool,
    pub evaluation_calls: usize,
    cache_fixed_factors: bool,
    slots: Vec<Slot>,
    recycled: Option<Linearization>,
}

#[derive(Default)]
struct Slot {
    active: bool,
    fixed: bool,
    cached: bool,
    evaluations: usize,
    cost: f64,
    mask: Vec<bool>,
    row: Option<Row>,
    contributions: Vec<(usize, DVector<f64>, DVector<f64>)>,
    outcome: Option<Result<()>>,
}

impl Workspace {
    pub fn begin(&mut self, parallel: bool, cache_fixed_factors: bool) {
        self.parallel = parallel;
        self.evaluation_calls = 0;
        self.cache_fixed_factors = cache_fixed_factors;
        for slot in &mut self.slots {
            slot.cached = false;
            slot.evaluations = 0;
        }
    }
    pub fn factor_evaluations(&self) -> usize {
        self.slots.iter().map(|slot| slot.evaluations).sum()
    }
    pub fn recycle(&mut self, mut model: Linearization) {
        model.rows.clear();
        self.recycled = Some(model);
    }
}

impl Slot {
    fn evaluate(
        &mut self,
        factor: &mut FactorRecord,
        values: &BTreeMap<usize, Values>,
        offsets: &BTreeMap<usize, usize>,
        jacobians: bool,
    ) -> Result<()> {
        self.mask.clear();
        self.mask
            .extend(factor.keys.iter().map(|k| offsets.contains_key(k)));
        self.fixed = !self.mask.iter().any(|x| *x);
        let parameters: Vec<_> = factor
            .keys
            .iter()
            .map(|k| values[k].data.as_slice())
            .collect();
        let mut evaluated = factor.factor.evaluate(
            &parameters,
            (jacobians && !self.fixed).then_some(self.mask.as_slice()),
        )?;
        if evaluated.residuals.iter().any(|x| !x.is_finite()) {
            return Err(graph_error("non-finite graph residual"));
        }
        let squared_norm = evaluated.residuals.norm_squared();
        let rho = factor
            .loss
            .map_or([squared_norm, 1.0, 0.0], |loss| loss.evaluate(squared_norm));
        if rho.iter().any(|x| !x.is_finite()) || rho[1] < 0.0 {
            return Err(graph_error("invalid robust loss"));
        }
        self.cost = 0.5 * rho[0];
        if self.fixed || !jacobians {
            return Ok(());
        }
        let mut blocks = Vec::with_capacity(factor.keys.len());
        for (index, key) in factor.keys.iter().enumerate() {
            let Some(&offset) = offsets.get(key) else {
                continue;
            };
            let j = evaluated
                .jacobians
                .get_mut(index)
                .and_then(Option::take)
                .ok_or_else(|| graph_error("ambient factor Jacobian absent"))?;
            if j.shape() != (evaluated.residuals.len(), values[key].data.len())
                || j.iter().any(|x| !x.is_finite())
            {
                return Err(graph_error("invalid ambient factor Jacobian"));
            }
            blocks.push((offset, values[key].apply_plus_jacobian(j)));
        }
        // Ceres Corrector disables curvature correction for concave losses.
        let sqrt = rho[1].sqrt();
        evaluated.residuals *= sqrt;
        self.contributions
            .resize_with(blocks.len(), || (0, DVector::zeros(0), DVector::zeros(0)));
        for ((offset, j), (stored_offset, gradient, diagonal)) in
            blocks.iter_mut().zip(&mut self.contributions)
        {
            *j *= sqrt;
            *stored_offset = *offset;
            if gradient.len() != j.ncols() {
                gradient.resize_vertically_mut(j.ncols(), 0.0);
            }
            gradient.gemv_tr(1.0, j, &evaluated.residuals, 0.0);
            if diagonal.len() != j.ncols() {
                diagonal.resize_vertically_mut(j.ncols(), 0.0);
            }
            for c in 0..j.ncols() {
                diagonal[c] = j.column(c).norm_squared();
            }
        }
        self.row = Some(Row {
            residual: evaluated.residuals,
            blocks,
        });
        Ok(())
    }
}

impl Snapshot {
    #[fastrace::trace]
    pub(super) fn evaluate_with(
        &mut self,
        values: &BTreeMap<usize, Values>,
        offsets: &BTreeMap<usize, usize>,
        size: usize,
        disabled: &BTreeSet<ResidualId>,
        jacobians: bool,
        workspace: &mut Workspace,
    ) -> Result<Linearization> {
        workspace.evaluation_calls += 1;
        workspace
            .slots
            .resize_with(self.factors.len(), Slot::default);
        let cache_fixed_factors = workspace.cache_fixed_factors;
        let batch = |(factors, slots): (&mut [FactorRecord], &mut [Slot])| {
            for (factor, slot) in factors.iter_mut().zip(slots) {
                slot.active = !disabled.contains(&factor.id);
                slot.row = None;
                slot.outcome = Some(if slot.active && !slot.cached {
                    slot.evaluations += 1;
                    let result = slot.evaluate(factor, values, offsets, jacobians);
                    slot.cached = cache_fixed_factors && slot.fixed && result.is_ok();
                    result
                } else {
                    Ok(())
                });
            }
        };
        // Only a solver's explicitly installed pool may execute parallel work.
        if workspace.parallel {
            self.factors
                .par_chunks_mut(FACTORS_PER_BATCH)
                .zip(workspace.slots.par_chunks_mut(FACTORS_PER_BATCH))
                .for_each(batch);
        } else {
            self.factors
                .chunks_mut(FACTORS_PER_BATCH)
                .zip(workspace.slots.chunks_mut(FACTORS_PER_BATCH))
                .for_each(batch);
        }
        let n = if jacobians { size } else { 0 };
        let mut model = if jacobians {
            workspace.recycled.take()
        } else {
            None
        }
        .unwrap_or_else(|| Linearization {
            parallel: workspace.parallel,
            cost: 0.0,
            fixed_cost: 0.0,
            rows: Vec::new(),
            gradient: DVector::zeros(0),
            diagonal: DVector::zeros(0),
            eliminated_points: Vec::new(),
        });
        model.parallel = workspace.parallel;
        model.cost = 0.0;
        model.fixed_cost = 0.0;
        model.rows.clear();
        if model.gradient.len() != n {
            model.gradient.resize_vertically_mut(n, 0.0);
        }
        model.gradient.fill(0.0);
        if model.diagonal.len() != n {
            model.diagonal.resize_vertically_mut(n, 0.0);
        }
        model.diagonal.fill(0.0);
        model.eliminated_points.clear();
        if jacobians {
            model
                .eliminated_points
                .extend(offsets.iter().filter_map(|(&key, &offset)| {
                    matches!(values[&key].manifold, ManifoldKind::HomogeneousPoint)
                        .then_some(offset)
                }));
        }
        // Scheduling cannot change the first reported error or floating-point sum order.
        for slot in &mut workspace.slots {
            slot.outcome.take().expect("completed factor task")?;
            if !slot.active {
                continue;
            }
            if slot.fixed {
                model.fixed_cost += slot.cost;
                continue;
            }
            model.cost += slot.cost;
            if let Some(row) = slot.row.take() {
                for (offset, gradient, diagonal) in &slot.contributions {
                    for c in 0..gradient.len() {
                        model.gradient[offset + c] += gradient[c];
                        model.diagonal[offset + c] += diagonal[c];
                    }
                }
                model.rows.push(row);
            }
        }
        if !model.cost.is_finite() || !model.fixed_cost.is_finite() {
            return Err(graph_error("graph cost overflow"));
        }
        Ok(model)
    }
}

#[cfg(test)]
mod tests {
    use super::super::{
        Arc, Execution, Factor, RobustLoss, SMatrix, SVector, SolverOptions, SpeedAndBiasError,
    };
    use super::*;

    fn fixture() -> Snapshot {
        Snapshot {
            values: BTreeMap::from([(
                1,
                Values {
                    data: vec![0.2; 9],
                    manifold: ManifoldKind::Euclidean,
                    fixed: false,
                },
            )]),
            factors: (0..257)
                .map(|id| FactorRecord {
                    id,
                    factor: Factor::SpeedBiasPrior(Box::new(SpeedAndBiasError::new(
                        SVector::repeat(id as f64 / 100.0),
                        SMatrix::identity(),
                    ))),
                    keys: vec![1],
                    loss: Some(RobustLoss::Huber(1.0)),
                })
                .collect(),
        }
    }

    #[test]
    fn parallel_evaluation_is_ordered_and_reuses_workspace_after_masks_and_errors() {
        let mut snapshot = fixture();
        let values = snapshot.values.clone();
        let offsets = BTreeMap::from([(1, 0)]);
        let mut workspace = Workspace {
            parallel: true,
            ..Default::default()
        };
        let mut execution = Execution::default();
        execution.configure(4).unwrap();
        let pool = execution.pool().unwrap();
        for disabled in [
            BTreeSet::new(),
            BTreeSet::from([0, 32, 256]),
            (0..257).collect(),
        ] {
            for jacobians in [true, false, true] {
                let serial = snapshot
                    .evaluate(&values, &offsets, 9, &disabled, jacobians)
                    .unwrap();
                let parallel = pool
                    .install(|| {
                        snapshot.evaluate_with(
                            &values,
                            &offsets,
                            9,
                            &disabled,
                            jacobians,
                            &mut workspace,
                        )
                    })
                    .unwrap();
                assert_eq!(serial.cost.to_bits(), parallel.cost.to_bits());
                assert_eq!(serial.gradient, parallel.gradient);
                assert_eq!(serial.diagonal, parallel.diagonal);
                assert_eq!(serial.rows.len(), parallel.rows.len());
                for (a, b) in serial.rows.iter().zip(&parallel.rows) {
                    assert_eq!(a.residual, b.residual);
                    assert_eq!(a.blocks, b.blocks);
                }
                if jacobians {
                    let gradient_ptr = parallel.gradient.as_ptr();
                    workspace.recycle(parallel);
                    let again = pool
                        .install(|| {
                            snapshot.evaluate_with(
                                &values,
                                &offsets,
                                9,
                                &disabled,
                                true,
                                &mut workspace,
                            )
                        })
                        .unwrap();
                    assert_eq!(gradient_ptr, again.gradient.as_ptr());
                    workspace.recycle(again);
                }
            }
        }
        let mut invalid = values.clone();
        invalid.get_mut(&1).unwrap().data[0] = f64::NAN;
        assert!(
            pool.install(|| snapshot.evaluate_with(
                &invalid,
                &offsets,
                9,
                &BTreeSet::new(),
                true,
                &mut workspace
            ))
            .is_err()
        );
        assert!(
            workspace
                .slots
                .iter()
                .all(|s| s.outcome.is_some() || s.active)
        );
        let recovered = pool
            .install(|| {
                snapshot.evaluate_with(&values, &offsets, 9, &BTreeSet::new(), true, &mut workspace)
            })
            .unwrap();
        assert_eq!(recovered.rows.len(), 257);
    }

    #[test]
    fn fixed_cost_cache_is_solve_local_and_preserves_full_solver_results() {
        for threads in [1, 4] {
            let mut reference = Vec::new();
            for cache in [false, true] {
                let mut snapshot = fixture();
                let mut fixed = snapshot.values[&1].clone();
                fixed.fixed = true;
                snapshot.values.insert(2, fixed);
                for factor in &mut snapshot.factors[..200] {
                    factor.keys[0] = 2;
                }
                let mut workspace = Workspace::default();
                let mut execution = Execution::default();
                execution.configure(threads).unwrap();
                let options = SolverOptions {
                    num_threads: threads,
                    cache_fixed_factors: cache,
                    max_iterations: 5,
                    ..Default::default()
                };
                // Reuse the workspace with changed fixed values and disabled factors.
                for stage in 0..2 {
                    snapshot.values.get_mut(&2).unwrap().data.fill(stage as f64);
                    let disabled = if stage == 0 {
                        BTreeSet::new()
                    } else {
                        BTreeSet::from([0])
                    };
                    let summary = execution
                        .install(|| snapshot.run_with(&options, &disabled, &mut workspace))
                        .unwrap();
                    let calls = workspace.evaluation_calls;
                    assert!(calls > 1);
                    let fixed = 200 - stage;
                    assert_eq!(
                        workspace.factor_evaluations(),
                        if cache {
                            fixed + 57 * calls
                        } else {
                            (fixed + 57) * calls
                        }
                    );
                    let result = (
                        snapshot.values[&1].data.clone(),
                        summary.initial_cost,
                        summary.final_cost,
                        summary.iterations,
                        summary.report,
                    );
                    if cache {
                        assert_eq!(result, reference[stage]);
                    } else {
                        reference.push(result);
                    }
                }
            }
        }
    }

    #[test]
    fn fixed_cache_resets_masks_and_rejects_invalid_constants() {
        let mut snapshot = fixture();
        let mut workspace = Workspace::default();
        let disabled = BTreeSet::new();
        for offsets in [BTreeMap::new(), BTreeMap::from([(1, 0)]), BTreeMap::new()] {
            workspace.begin(false, true);
            let values = snapshot.values.clone();
            for _ in 0..3 {
                let cached = snapshot
                    .evaluate_with(&values, &offsets, 9, &disabled, true, &mut workspace)
                    .unwrap();
                let baseline = snapshot
                    .evaluate(&values, &offsets, 9, &disabled, true)
                    .unwrap();
                assert_eq!(cached.fixed_cost, baseline.fixed_cost);
                assert_eq!(cached.cost, baseline.cost);
                assert_eq!(cached.gradient, baseline.gradient);
                assert_eq!(cached.diagonal, baseline.diagonal);
            }
            assert_eq!(
                workspace.factor_evaluations(),
                if offsets.is_empty() { 257 } else { 771 }
            );
        }
        workspace.begin(false, true);
        let mut values = snapshot.values.clone();
        values.get_mut(&1).unwrap().data[0] = f64::NAN;
        assert!(
            snapshot
                .evaluate_with(
                    &values,
                    &BTreeMap::new(),
                    0,
                    &disabled,
                    true,
                    &mut workspace
                )
                .is_err()
        );
        assert!(workspace.slots.iter().all(|slot| !slot.cached));
        values.get_mut(&1).unwrap().data[0] = 0.2;
        assert!(
            snapshot
                .evaluate_with(
                    &values,
                    &BTreeMap::new(),
                    0,
                    &disabled,
                    true,
                    &mut workspace
                )
                .is_ok()
        );
    }

    #[test]
    fn pool_is_reused_and_solver_results_do_not_depend_on_worker_count() {
        let mut execution = Execution::default();
        execution.configure(2).unwrap();
        let pool = execution.pool().unwrap();
        execution.configure(2).unwrap();
        assert!(Arc::ptr_eq(&pool, &execution.pool().unwrap()));
        assert!(execution.configure(0).is_err());
        let mut reference = None;
        for threads in [1, 2, 4] {
            let mut snapshot = fixture();
            let summary = snapshot
                .run(
                    &SolverOptions {
                        num_threads: threads,
                        max_iterations: 5,
                        ..Default::default()
                    },
                    &BTreeSet::new(),
                )
                .unwrap();
            let result = (
                snapshot.values[&1].data.clone(),
                summary.final_cost,
                summary.iterations,
            );
            if let Some(expected) = &reference {
                assert_eq!(&result, expected);
            } else {
                reference = Some(result);
            }
        }
    }
}
