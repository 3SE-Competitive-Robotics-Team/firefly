//! Isolated same-input benchmark for constant-factor evaluation; never changes a live graph.
use super::{
    BTreeMap, BTreeSet, Execution, GraphProblem, Instant, Result, RustGraphSolver, Snapshot,
    SolverOptions, Workspace, graph_error,
};

impl RustGraphSolver {
    /// Compare repeated constant-factor evaluation with one evaluation and cached cost reuse.
    /// Returns constant factor count and elapsed seconds (repeated, cached).
    /// Snapshot capture and thread-pool startup are outside the timed regions.
    /// # Errors
    /// Returns graph validation/evaluation errors or a constant cost mismatch.
    pub fn benchmark_fixed_factors(
        problem: &GraphProblem,
        threads: usize,
        evaluations: usize,
        reverse: bool,
    ) -> Result<(usize, f64, f64)> {
        if evaluations == 0 {
            return Err(graph_error("benchmark requires evaluations"));
        }
        let mut execution = Execution::default();
        execution.configure(threads)?;
        let mut snapshots = [Snapshot::capture(problem)?, Snapshot::capture(problem)?];
        for snapshot in &mut snapshots {
            let (offsets, _) =
                snapshot.variable_offsets(&SolverOptions::default(), &BTreeSet::new())?;
            snapshot
                .factors
                .retain(|f| f.keys.iter().all(|k| !offsets.contains_key(k)));
        }
        let count = snapshots[0].factors.len();
        execution.install(|| {
            let mut times = [0.0; 2];
            let mut reference: Option<Vec<f64>> = None;
            for variant in if reverse { [1, 0] } else { [0, 1] } {
                let snapshot = &mut snapshots[variant];
                let values = snapshot.values.clone();
                let mut workspace = Workspace::default();
                workspace.parallel = threads > 1;
                let disabled = BTreeSet::new();
                let offsets = BTreeMap::new();
                // Warm buffers and mutable integration caches identically on both sides.
                snapshot.evaluate_with(&values, &offsets, 0, &disabled, false, &mut workspace)?;
                let start = Instant::now();
                let mut costs = Vec::with_capacity(evaluations);
                let mut cached = None;
                for _ in 0..evaluations {
                    let cost = if variant == 1 {
                        if let Some(cost) = cached {
                            cost
                        } else {
                            let cost = snapshot
                                .evaluate_with(
                                    &values,
                                    &offsets,
                                    0,
                                    &disabled,
                                    false,
                                    &mut workspace,
                                )?
                                .fixed_cost;
                            cached = Some(cost);
                            cost
                        }
                    } else {
                        snapshot
                            .evaluate_with(&values, &offsets, 0, &disabled, false, &mut workspace)?
                            .fixed_cost
                    };
                    costs.push(std::hint::black_box(cost));
                }
                times[variant] = start.elapsed().as_secs_f64();
                if let Some(expected) = &reference {
                    if *expected != costs {
                        return Err(graph_error("constant factor costs changed"));
                    }
                } else {
                    reference = Some(costs);
                }
            }
            Ok((count, times[0], times[1]))
        })
    }
}
