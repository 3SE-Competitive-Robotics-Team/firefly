//! Evaluate a frozen square-root system without changing its coefficients,
//! damping, physical-gradient stopping rule, or the production optimizer.
use firefly_calibration::scalar::BasaltScalar;
use nalgebra::{DMatrix, DVector};
use serde::Deserialize;
use serde_json::json;
use std::{fs::File, io::BufReader, time::Instant};

#[derive(Deserialize)]
struct Block {
    columns: Vec<usize>,
    rows: usize,
    matrix_column_major: Vec<f64>,
    residual: Vec<f64>,
}
#[derive(Deserialize)]
struct Archive {
    scalar_bytes: usize,
    columns: usize,
    damping: f64,
    blocks: Vec<Block>,
    step: Vec<f64>,
    relative_gradient_tolerance: f64,
}
struct Operator<S: BasaltScalar> {
    columns: usize,
    rows: usize,
    damping: S,
    blocks: Vec<(Vec<usize>, DMatrix<S>, DVector<S>)>,
}
impl<S: BasaltScalar> Operator<S> {
    fn new(a: &Archive) -> Self {
        Self {
            columns: a.columns,
            rows: a.blocks.iter().map(|b| b.rows).sum::<usize>() + a.columns,
            damping: S::from_f64_lossy(a.damping),
            blocks: a
                .blocks
                .iter()
                .map(|b| {
                    (
                        b.columns.clone(),
                        DMatrix::from_iterator(
                            b.rows,
                            b.columns.len(),
                            b.matrix_column_major.iter().copied().map(S::from_f64_lossy),
                        ),
                        DVector::from_iterator(
                            b.residual.len(),
                            b.residual.iter().copied().map(S::from_f64_lossy),
                        ),
                    )
                })
                .collect(),
        }
    }
    fn rhs(&self) -> DVector<S> {
        let mut rhs = DVector::zeros(self.rows);
        let mut offset = 0;
        for (_, _, residual) in &self.blocks {
            rhs.rows_mut(offset, residual.len()).copy_from(&(-residual));
            offset += residual.len();
        }
        rhs
    }
    fn mul(&self, x: &DVector<S>) -> DVector<S> {
        let mut out = DVector::zeros(self.rows);
        let mut offset = 0;
        for (columns, matrix, _) in &self.blocks {
            for row in 0..matrix.nrows() {
                let mut sum = S::zero();
                for (col, &global) in columns.iter().enumerate() {
                    sum += matrix[(row, col)] * x[global];
                }
                out[offset + row] = sum;
            }
            offset += matrix.nrows();
        }
        for i in 0..self.columns {
            out[offset + i] = self.damping * x[i];
        }
        out
    }
    fn transpose(&self, x: &DVector<S>) -> DVector<S> {
        let mut out = DVector::zeros(self.columns);
        let mut offset = 0;
        for (columns, matrix, _) in &self.blocks {
            for (col, &global) in columns.iter().enumerate() {
                let mut sum = S::zero();
                for row in 0..matrix.nrows() {
                    sum += matrix[(row, col)] * x[offset + row];
                }
                out[global] += sum;
            }
            offset += matrix.nrows();
        }
        for i in 0..self.columns {
            out[i] += self.damping * x[offset + i];
        }
        out
    }
    fn scaling(&self) -> DVector<S> {
        let mut norms = DVector::from_element(self.columns, self.damping.abs());
        for (columns, matrix, _) in &self.blocks {
            for (col, &global) in columns.iter().enumerate() {
                for value in matrix.column(col).iter() {
                    norms[global] = norms[global].hypot(*value);
                }
            }
        }
        norms.map(|v| {
            if v > S::zero() {
                S::one() / v
            } else {
                S::one()
            }
        })
    }
    fn gradient(&self, x: &DVector<S>) -> f64 {
        self.transpose(&(self.mul(x) - self.rhs())).norm().as_f64()
    }
}

// Independent baseline and candidate; the physical-gradient reference stays separate.
fn row_multiply<S: BasaltScalar, W: BasaltScalar>(
    operator: &Operator<S>,
    input: &DVector<W>,
) -> DVector<W> {
    let mut output = DVector::zeros(operator.rows);
    let mut offset = 0;
    for (columns, matrix, _) in &operator.blocks {
        for row in 0..matrix.nrows() {
            let mut sum = W::zero();
            for (local, &global) in columns.iter().enumerate() {
                sum += W::from_f64_lossy(matrix[(row, local)].as_f64()) * input[global];
            }
            output[offset + row] = sum;
        }
        offset += matrix.nrows();
    }
    for i in 0..operator.columns {
        output[offset + i] = W::from_f64_lossy(operator.damping.as_f64()) * input[i];
    }
    output
}
fn column_multiply<S: BasaltScalar, W: BasaltScalar>(
    operator: &Operator<S>,
    input: &DVector<W>,
) -> DVector<W> {
    let mut output = DVector::zeros(operator.rows);
    let mut offset = 0;
    for (columns, matrix, _) in &operator.blocks {
        let rows = matrix.nrows();
        let target = &mut output.as_mut_slice()[offset..offset + rows];
        for (local, &global) in columns.iter().enumerate() {
            let scale = input[global];
            let coefficients = &matrix.as_slice()[local * rows..(local + 1) * rows];
            for (value, coefficient) in target.iter_mut().zip(coefficients) {
                *value += W::from_f64_lossy(coefficient.as_f64()) * scale;
            }
        }
        offset += rows;
    }
    for i in 0..operator.columns {
        output[offset + i] = W::from_f64_lossy(operator.damping.as_f64()) * input[i];
    }
    output
}
trait Bits {
    fn bits(self) -> u64;
}
impl Bits for f32 {
    fn bits(self) -> u64 {
        u64::from(self.to_bits())
    }
}
impl Bits for f64 {
    fn bits(self) -> u64 {
        self.to_bits()
    }
}
fn matvec_benchmark<S: BasaltScalar, W: BasaltScalar + Bits>(
    archive: &Archive,
) -> serde_json::Value {
    use std::hint::black_box;
    let operator = Operator::<S>::new(archive);
    let input = DVector::from_iterator(
        archive.step.len(),
        archive.step.iter().copied().map(W::from_f64_lossy),
    );
    let baseline = row_multiply(&operator, &input);
    let candidate = column_multiply(&operator, &input);
    assert!(
        baseline
            .iter()
            .zip(candidate.iter())
            .all(|(&a, &b)| a.bits() == b.bits())
    );
    for _ in 0..4 {
        black_box(row_multiply(&operator, black_box(&input)));
        black_box(column_multiply(&operator, black_box(&input)));
    }
    let mut row_seconds = Vec::new();
    let mut column_seconds = Vec::new();
    for trial in 0..30 {
        for candidate in if trial % 2 == 0 {
            [false, true]
        } else {
            [true, false]
        } {
            let started = Instant::now();
            for _ in 0..3 {
                black_box(if candidate {
                    column_multiply(&operator, black_box(&input))
                } else {
                    row_multiply(&operator, black_box(&input))
                });
            }
            let elapsed = started.elapsed().as_secs_f64() / 3.;
            if candidate {
                column_seconds.push(elapsed);
            } else {
                row_seconds.push(elapsed);
            }
        }
    }
    let mut sorted_rows = row_seconds.clone();
    sorted_rows.sort_by(f64::total_cmp);
    let mut sorted_columns = column_seconds.clone();
    sorted_columns.sort_by(f64::total_cmp);
    json!({"storage_bytes":std::mem::size_of::<S>(),"workspace_bytes":std::mem::size_of::<W>(),"bitwise_equal":true,
        "row_seconds":row_seconds,"column_seconds":column_seconds,
        "median_row_seconds":sorted_rows[15],"median_column_seconds":sorted_columns[15],"median_speedup":sorted_rows[15]/sorted_columns[15]})
}

fn solve(
    operator: &Operator<f64>,
    rhs: &DVector<f64>,
    threshold: f64,
    max_iterations: usize,
    reorthogonalize: bool,
) -> (DVector<f64>, usize, bool, f64) {
    let scaling = operator.scaling();
    let mut x = DVector::zeros(operator.columns);
    let mut u = rhs.clone();
    let mut beta = u.norm();
    if beta == 0.0 {
        return (x, 0, true, 0.0);
    }
    u /= beta;
    let mut v = operator.transpose(&u).component_mul(&scaling);
    let mut alpha = v.norm();
    if alpha == 0.0 {
        return (x, 0, true, 0.0);
    }
    v /= alpha;
    let mut w = v.clone();
    let mut phi_bar = beta;
    let mut rho_bar = alpha;
    let mut basis = vec![v.clone()];
    let mut left_basis = if reorthogonalize {
        vec![u.clone()]
    } else {
        Vec::new()
    };
    let mut max_overlap = 0.0_f64;
    for iteration in 1..=max_iterations {
        u = operator.mul(&v.component_mul(&scaling)) - u * alpha;
        if reorthogonalize {
            for _ in 0..2 {
                for previous in &left_basis {
                    u -= previous * previous.dot(&u);
                }
            }
        }
        beta = u.norm();
        if beta > 0.0 {
            u /= beta;
        }
        v = operator.transpose(&u).component_mul(&scaling) - v * beta;
        if reorthogonalize {
            for _ in 0..2 {
                for previous in &basis {
                    v -= previous * previous.dot(&v);
                }
            }
            left_basis.push(u.clone());
        }
        alpha = v.norm();
        if alpha > 0.0 {
            v /= alpha;
        }
        if iteration % 8 == 0 {
            for previous in &basis {
                max_overlap = max_overlap.max(previous.dot(&v).abs());
            }
        }
        basis.push(v.clone());
        let rho = rho_bar.hypot(beta);
        if rho == 0.0 {
            let physical = x.component_mul(&scaling);
            let good = operator.transpose(&(operator.mul(&physical) - rhs)).norm() <= threshold;
            return (physical, iteration, good, max_overlap);
        }
        let cosine = rho_bar / rho;
        let sine = beta / rho;
        let theta = sine * alpha;
        rho_bar = -cosine * alpha;
        let phi = cosine * phi_bar;
        phi_bar *= sine;
        x += &w * (phi / rho);
        w = &v - w * (theta / rho);
        if iteration % 8 == 0 || alpha == 0.0 || beta == 0.0 {
            let physical = x.component_mul(&scaling);
            if operator.transpose(&(operator.mul(&physical) - rhs)).norm() <= threshold {
                return (physical, iteration, true, max_overlap);
            }
        }
    }
    (
        x.component_mul(&scaling),
        max_iterations,
        false,
        max_overlap,
    )
}
fn float_trace(
    operator: &Operator<f32>,
    double: &Operator<f64>,
    threshold: f64,
    archived: &DVector<f32>,
) -> serde_json::Value {
    let scaling = operator.scaling();
    let rhs = operator.rhs();
    let mut x = DVector::zeros(operator.columns);
    let mut u = rhs.clone();
    let mut beta = u.norm();
    u /= beta;
    let mut v = operator.transpose(&u).component_mul(&scaling);
    let mut alpha = v.norm();
    v /= alpha;
    let mut w = v.clone();
    let mut phi_bar = beta;
    let mut rho_bar = alpha;
    let mut history = Vec::new();
    let mut first_float = None;
    let mut first_double = None;
    let mut basis = vec![v.clone()];
    for iteration in 1..=1000 {
        u = operator.mul(&v.component_mul(&scaling)) - u * alpha;
        beta = u.norm();
        if beta > 0.0 {
            u /= beta;
        }
        v = operator.transpose(&u).component_mul(&scaling) - v * beta;
        alpha = v.norm();
        if alpha > 0.0 {
            v /= alpha;
        }
        let rho = (rho_bar * rho_bar + beta * beta).sqrt();
        if rho == 0.0 {
            break;
        }
        let cosine = rho_bar / rho;
        let sine = beta / rho;
        let theta = sine * alpha;
        rho_bar = -cosine * alpha;
        let phi = cosine * phi_bar;
        phi_bar *= sine;
        x += &w * (phi / rho);
        w = &v - w * (theta / rho);
        if iteration % 8 == 0 || alpha == 0.0 || beta == 0.0 {
            let physical = x.component_mul(&scaling);
            let g32 = operator.gradient(&physical);
            let g64 = double.gradient(&physical.map(f64::from));
            if g32 <= threshold && first_float.is_none() {
                first_float = Some(iteration);
            }
            if g64 <= threshold && first_double.is_none() {
                first_double = Some(iteration);
            }
            let max_overlap = basis
                .iter()
                .map(|previous| previous.dot(&v).abs())
                .fold(0.0_f32, f32::max);
            history.push(json!({"iteration":iteration,"gradient_f32":g32,"gradient_f64":g64,"max_v_inner_product":max_overlap}));
        }
        basis.push(v.clone());
    }
    json!({"first_converged_f32_check":first_float,"first_converged_f64_check":first_double,"final_step_difference_from_archive":(x.component_mul(&scaling)-archived).norm(),"history":history})
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 3
        && !(args.len() == 4 && (args[3] == "--full-reorthogonalization" || args[3] == "--matvec"))
    {
        return Err(
            "usage: lsqr_archive INPUT.json OUTPUT.json [--full-reorthogonalization|--matvec]"
                .into(),
        );
    }
    let archive: Archive = serde_json::from_reader(BufReader::new(File::open(&args[1])?))?;
    if args.get(3).is_some_and(|flag| flag == "--matvec") {
        let output = json!({"rows":archive.blocks.iter().map(|block|block.rows).sum::<usize>()+archive.columns,
            "columns":archive.columns,"f32_f32":matvec_benchmark::<f32,f32>(&archive),
            "f32_f64":matvec_benchmark::<f32,f64>(&archive),"f64_f64":matvec_benchmark::<f64,f64>(&archive)});
        serde_json::to_writer_pretty(File::create(&args[2])?, &output)?;
        return Ok(());
    }
    analyze_archive(&archive, &args[2], args.len() == 4)
}
fn analyze_archive(
    archive: &Archive,
    output_path: &str,
    reorthogonalize: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    if archive.scalar_bytes != 4 {
        return Err("precision comparison requires an archived f32 system".into());
    }
    let float = Operator::<f32>::new(archive);
    let double = Operator::<f64>::new(archive);
    let step32 = DVector::from_iterator(archive.step.len(), archive.step.iter().map(|v| *v as f32));
    let step64 = DVector::from_column_slice(&archive.step);
    let initial32 = float.transpose(&float.rhs()).norm();
    let initial64 = double.transpose(&double.rhs()).norm();
    let mut output = json!({"scalar_bytes":archive.scalar_bytes,"columns":archive.columns,"rows":double.rows,"blocks":archive.blocks.len(),"damping":archive.damping,"initial_gradient_f32":initial32,"initial_gradient_f64":initial64,"threshold_f32":f64::from(initial32)*archive.relative_gradient_tolerance,"threshold_f64":initial64*archive.relative_gradient_tolerance,"archived_step_gradient_f32":float.gradient(&step32),"archived_step_gradient_f64":double.gradient(&step64),"residual_norm_f64":(double.mul(&step64)-double.rhs()).norm()});
    output["double_lsqr"] = json!({"status":"running"});
    serde_json::to_writer_pretty(File::create(output_path)?, &output)?;
    let start = Instant::now();
    let (solution, iterations, converged, max_overlap) = solve(
        &double,
        &double.rhs(),
        initial64 * archive.relative_gradient_tolerance,
        1000,
        false,
    );
    let elapsed = start.elapsed().as_secs_f64();
    let rounded32 = solution.map(|v| v as f32);
    let rounded64 = rounded32.map(f64::from);
    output["double_lsqr"] = json!({"iterations":iterations,"converged":converged,"elapsed_seconds":elapsed,"gradient_f64":double.gradient(&solution),"rounded_step_gradient_f64":double.gradient(&rounded64),"rounded_step_gradient_f32":float.gradient(&rounded32),"step_norm":solution.norm(),"difference_to_archived_step":(&solution-step64).norm(),"maximum_v_inner_product":max_overlap});
    // Reference-only extended work: keep the production threshold and budget
    // unchanged. Solve more accurately to expose rounding of a physical step.
    let threshold = initial64 * archive.relative_gradient_tolerance;
    let reference_target = threshold * 0.01;
    let start = Instant::now();
    let (mut reference, iterations, reference_converged, overlap) =
        solve(&double, &double.rhs(), reference_target, 5000, false);
    let rounded = reference.map(|v| f64::from(v as f32));
    output["extended_reference"] = json!({
        "maximum_iterations":5000,"iterations":iterations,
        "reference_target":reference_target,"reference_target_met":reference_converged,
        "gradient_f64":double.gradient(&reference),
        "rounded_step_gradient_f64":double.gradient(&rounded),
        "f64_passes_original_threshold":double.gradient(&reference)<=threshold,
        "rounded_passes_original_threshold":double.gradient(&rounded)<=threshold,
        "maximum_v_inner_product":overlap,"elapsed_seconds":start.elapsed().as_secs_f64()
    });
    serde_json::to_writer_pretty(File::create(output_path)?, &output)?;
    let mut refinements = Vec::new();
    for round in 0..3 {
        let residual_rhs = double.rhs() - double.mul(&reference);
        let start = Instant::now();
        let (correction, iterations, reached, overlap) =
            solve(&double, &residual_rhs, reference_target, 1000, false);
        reference += correction;
        let rounded = reference.map(|v| f64::from(v as f32));
        refinements.push(json!({"round":round,"iterations":iterations,
            "correction_target_met":reached,"maximum_v_inner_product":overlap,
            "gradient_f64":double.gradient(&reference),
            "rounded_step_gradient_f64":double.gradient(&rounded),
            "f64_passes_original_threshold":double.gradient(&reference)<=threshold,
            "rounded_passes_original_threshold":double.gradient(&rounded)<=threshold,
            "elapsed_seconds":start.elapsed().as_secs_f64()}));
    }
    output["residual_refinement"] = json!(refinements);
    if reorthogonalize {
        let start = Instant::now();
        let (reference, iterations, converged, overlap) =
            solve(&double, &double.rhs(), threshold, 1000, true);
        let rounded = reference.map(|v| f64::from(v as f32));
        output["full_reorthogonalization"] = json!({
            "iterations":iterations,"converged":converged,
            "gradient_f64":double.gradient(&reference),
            "rounded_step_gradient_f64":double.gradient(&rounded),
            "maximum_v_inner_product":overlap,
            "stored_basis_bytes":(iterations+1)*(double.columns+double.rows)*8,
            "elapsed_seconds":start.elapsed().as_secs_f64()
        });
    }
    output["interpretation"] = json!({
        "criterion":"All pass flags use the archived relative threshold on the original quantized physical operator.",
        "budget":"A failure at 1000 followed by a passing f64 reference shows the original budget did not deliver a certified step.",
        "orthogonality":"Sampled right-basis overlap measures orthogonality loss; it alone does not prove that loss caused slow convergence.",
        "rounding":"A certified f64 reference whose cast-f32 gradient fails shows rounding limits that reference step, not impossibility of every representable f32 step."
    });
    output["float_lsqr"] = json!({"status":"running"});
    serde_json::to_writer_pretty(File::create(output_path)?, &output)?;
    let start = Instant::now();
    output["float_lsqr"] = float_trace(
        &float,
        &double,
        f64::from(initial32) * archive.relative_gradient_tolerance,
        &step32,
    );
    output["float_lsqr"]["elapsed_seconds"] = json!(start.elapsed().as_secs_f64());
    serde_json::to_writer_pretty(File::create(output_path)?, &output)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_variants_and_residual_correction_match_augmented_svd() {
        let matrix =
            DMatrix::from_row_slice(4, 3, &[1., 2., 3., 4., 0., 1., 0., 2., -1., 1., 1., 1.]);
        let operator = Operator {
            columns: 3,
            rows: 7,
            damping: 0.03,
            blocks: vec![(
                vec![0, 1, 2],
                matrix,
                DVector::from_vec(vec![1., -2., 3., 0.5]),
            )],
        };
        let augmented = DMatrix::from_columns(
            &(0..3)
                .map(|axis| {
                    let mut unit = DVector::zeros(3);
                    unit[axis] = 1.;
                    operator.mul(&unit)
                })
                .collect::<Vec<_>>(),
        );
        let rhs = operator.rhs();
        let expected = augmented.svd(true, true).solve(&rhs, 1e-14).unwrap();
        for reorthogonalize in [false, true] {
            let (step, _, certified, _) = solve(&operator, &rhs, 1e-11, 40, reorthogonalize);
            assert!(certified);
            assert!((step - &expected).norm() < 1e-11);
            let initial = DVector::from_vec(vec![0.1, -0.2, 0.3]);
            let correction_rhs = &rhs - operator.mul(&initial);
            let (correction, _, certified, _) =
                solve(&operator, &correction_rhs, 1e-11, 40, reorthogonalize);
            assert!(certified);
            assert!((initial + correction - &expected).norm() < 1e-11);
        }
    }
}
