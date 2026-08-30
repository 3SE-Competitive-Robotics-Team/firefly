//! `KdTree` 基准（对照 `small_gicp/scripts/run_kdtree_benchmark.sh`）。
//!
//! - 构建：`KdTree::new` 单线程建树（满二叉 2n 节点预分配）。
//! - 查询：串行 `knn_search` vs 并行 `par_knn_search_batch`（`rayon`）。

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use firefly_gicp::ann::kdtree::KdTree;
use firefly_gicp::points::point_cloud::PointCloud;
use firefly_gicp::points::traits::{PointCloudMut, PointCloudTrait};
use nalgebra::Vector4;
use std::hint::black_box;

fn make_cloud(n: usize) -> PointCloud {
    let mut rng = 0x1234_5678_u64;
    let mut cloud = PointCloud::new();
    cloud.resize(n);
    for i in 0..n {
        let x = (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 80.0;
        let y = (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 80.0;
        let z = (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 10.0;
        cloud.set_point(i, Vector4::new(x, y, z, 1.0));
    }
    cloud
}

fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_9B97_F4A7_C15B);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn bench_kdtree(c: &mut Criterion) {
    let mut group = c.benchmark_group("kdtree");

    for &n in &[5_000usize, 20_000, 80_000] {
        let cloud = make_cloud(n);
        let queries: Vec<Vector4<f64>> = {
            let mut rng = 0xABCD_u64;
            (0..1_000)
                .map(|_| {
                    let idx = (splitmix64(&mut rng) as usize) % n;
                    let base = cloud.point(idx);
                    Vector4::new(
                        base.x + (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 2.0,
                        base.y + (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 2.0,
                        base.z + (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * 2.0,
                        1.0,
                    )
                })
                .collect()
        };

        // 构建
        group.bench_with_input(criterion::BenchmarkId::new("build", n), &n, |b, _| {
            b.iter(|| {
                let tree: KdTree<PointCloud> = KdTree::new(black_box(cloud.clone()));
                black_box(tree);
            });
        });

        // 准备树用于查询
        let tree: KdTree<PointCloud> = KdTree::new(cloud.clone());
        group.throughput(Throughput::Elements(queries.len() as u64));

        // 串行查询
        group.bench_with_input(
            criterion::BenchmarkId::new("query_serial_k5", n),
            &n,
            |b, _| {
                b.iter(|| {
                    for q in &queries {
                        let mut idx = [0usize; 5];
                        let mut dist = [0.0; 5];
                        black_box(tree.knn_search(black_box(q), 5, &mut idx, &mut dist));
                    }
                });
            },
        );

        // 并行批量查询
        group.bench_with_input(
            criterion::BenchmarkId::new("query_par_batch_k5", n),
            &n,
            |b, _| {
                b.iter(|| {
                    let res = tree.par_knn_search_batch(black_box(&queries), 5);
                    black_box(res);
                });
            },
        );
    }
    group.finish();
}

criterion_group!(benches, bench_kdtree);
criterion_main!(benches);
