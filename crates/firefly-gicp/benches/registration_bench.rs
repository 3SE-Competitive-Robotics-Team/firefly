//! 配准基准（对照 `small_gicp/src/benchmark/odometry_benchmark*.cpp`）。
//!
//! - `GICP` 串行 vs `ParallelReduction` 并行。
//! - `VGICP`（`GaussianVoxelMap`）对照。
//! - 规模：5k / 20k 点（KITTI 单帧 ~10k-30k）。

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use firefly_gicp::ann::incremental_voxelmap::GaussianVoxelMap;
use firefly_gicp::ann::kdtree::KdTree;
use firefly_gicp::factors::GicpFactor;
use firefly_gicp::points::point_cloud::PointCloud;
use firefly_gicp::points::traits::{PointCloudMut, PointCloudTrait};
use firefly_gicp::registration::{ParallelReduction, Registration, SerialReduction};
use firefly_gicp::util::lie::se3_exp;
use nalgebra::{Matrix4, Vector3, Vector4, Vector6};
use std::hint::black_box;

fn make_cloud(n: usize) -> PointCloud {
    let mut rng = 0xCAFE_u64;
    let mut cloud = PointCloud::new();
    cloud.resize(n);
    for i in 0..n {
        let x = (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 60.0;
        let y = (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 60.0;
        let z = (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 8.0;
        cloud.set_point(i, Vector4::new(x, y, z, 1.0));
        let mut cov = Matrix4::zeros();
        cov[(0, 0)] = 1.0;
        cov[(1, 1)] = 1.0;
        cov[(2, 2)] = 1.0;
        cloud.set_cov(i, cov);
    }
    let tree: KdTree<PointCloud> = KdTree::new(cloud.clone());
    firefly_gicp::util::normal_estimation::estimate_normals_covariances_with_tree(
        &mut cloud, &tree, 10,
    );
    cloud
}

fn make_transform() -> Matrix4<f64> {
    let rot = Vector3::new(0.02 * 0.3, 0.02 * 0.5, 0.02 * 0.7);
    let mut a = Vector6::zeros();
    a.fixed_rows_mut::<3>(0).copy_from(&rot);
    a.fixed_rows_mut::<3>(3)
        .copy_from(&Vector3::new(0.15, 0.09, 0.06));
    se3_exp(&a)
}

fn transform_cloud(cloud: &PointCloud, t: &Matrix4<f64>) -> PointCloud {
    let mut out = PointCloud::new();
    out.resize(cloud.num_points());
    for i in 0..cloud.num_points() {
        out.set_point(i, t * cloud.point(i));
        let n = cloud.normal(i);
        let r = t.fixed_view::<3, 3>(0, 0).into_owned();
        let mut nr = Vector4::zeros();
        nr.fixed_rows_mut::<3>(0)
            .copy_from(&(r * n.fixed_rows::<3>(0).into_owned()));
        out.set_normal(i, nr);
        let cov = cloud.cov(i);
        let cov3 = cov.fixed_view::<3, 3>(0, 0).into_owned();
        let cov_rot = r * cov3 * r.transpose();
        let mut cov4 = Matrix4::zeros();
        cov4.fixed_view_mut::<3, 3>(0, 0).copy_from(&cov_rot);
        out.set_cov(i, cov4);
    }
    out
}

fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_9B97_F4A7_C15B);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn bench_registration(c: &mut Criterion) {
    let mut group = c.benchmark_group("registration");

    for &n in &[5_000usize, 20_000] {
        let target = make_cloud(n);
        let t_gt = make_transform();
        let source = transform_cloud(&target, &t_gt.try_inverse().unwrap());
        let target_tree: KdTree<PointCloud> = KdTree::new(target.clone());
        let source_tree: KdTree<PointCloud> = KdTree::new(source.clone());

        group.throughput(Throughput::Elements(n as u64));

        // GICP 串行 LM
        group.bench_with_input(
            criterion::BenchmarkId::new("gicp_serial_lm", n),
            &n,
            |b, _| {
                b.iter(|| {
                    let reg: Registration<GicpFactor, SerialReduction> = Registration::default();
                    let res = reg.align_serial(
                        black_box(&target),
                        black_box(&source),
                        black_box(&target_tree),
                        &Matrix4::identity(),
                    );
                    black_box(res);
                });
            },
        );

        // GICP 并行 LM（rayon）
        group.bench_with_input(
            criterion::BenchmarkId::new("gicp_parallel_lm", n),
            &n,
            |b, _| {
                let reg: Registration<GicpFactor, ParallelReduction> = Registration {
                    criteria: firefly_gicp::registration::TerminationCriteria::default(),
                    rejector: firefly_gicp::registration::DistanceRejector { max_dist_sq: 1.0 },
                    point_factor_setting: GicpFactor::default(),
                    general_factor: firefly_gicp::factors::NullFactor,
                    reduction: ParallelReduction { num_threads: 0 },
                    optimizer: firefly_gicp::registration::LevenbergMarquardtOptimizer::default(),
                    _phantom: std::marker::PhantomData,
                };
                b.iter(|| {
                    let res = reg.align_parallel(
                        black_box(&target),
                        black_box(&source),
                        black_box(&target_tree),
                        &Matrix4::identity(),
                    );
                    black_box(res);
                });
            },
        );

        // VGICP（GaussianVoxelMap）
        let voxelmap = {
            let mut vm = GaussianVoxelMap::new(1.0);
            vm.insert_identity(&target);
            vm
        };
        group.bench_with_input(
            criterion::BenchmarkId::new("vgicp_serial", n),
            &n,
            |b, _| {
                b.iter(|| {
                    let reg: Registration<GicpFactor, SerialReduction> = Registration::default();
                    // VGICP 复用同一 GicpFactor，target 为 voxelmap
                    let res = reg.align_serial(
                        black_box(&voxelmap),
                        black_box(&source),
                        black_box(&voxelmap),
                        &Matrix4::identity(),
                    );
                    let _ = black_box(&source_tree);
                    black_box(res);
                });
            },
        );
    }
    group.finish();
}

criterion_group!(benches, bench_registration);
criterion_main!(benches);
