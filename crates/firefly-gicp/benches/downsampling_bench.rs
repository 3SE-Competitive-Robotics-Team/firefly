//! 体素降采样基准（对照 `small_gicp/scripts/run_downsampling_benchmark.sh`）。
//!
//! - `voxelgrid_sampling` 串行 vs `rayon` 并行（内部 `into_par_iter`）。
//! - 校验 KITTI 典型规模：2k–100k 点，leaf 0.1–0.5m。

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use firefly_gicp::points::point_cloud::PointCloud;
use firefly_gicp::points::traits::PointCloudMut;
use firefly_gicp::util::downsampling::voxelgrid_sampling;
use std::hint::black_box;

fn make_cloud(n: usize, scale: f64) -> PointCloud {
    let mut rng = 0x9E37_u64;
    let mut cloud = PointCloud::new();
    cloud.resize(n);
    for i in 0..n {
        let x = (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * scale;
        let y = (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * scale;
        let z = (splitmix64(&mut rng) as f64 / u64::MAX as f64 - 0.5) * scale * 0.3;
        cloud.set_point(i, nalgebra::Vector4::new(x, y, z, 1.0));
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

fn bench_downsampling(c: &mut Criterion) {
    let mut group = c.benchmark_group("downsampling");

    for &n in &[5_000usize, 20_000, 80_000] {
        let cloud = make_cloud(n, 100.0);
        group.throughput(Throughput::Elements(n as u64));

        for &leaf in &[0.1f64, 0.25, 0.5] {
            let id = format!("n{n}_leaf{leaf}");
            group.bench_with_input(
                criterion::BenchmarkId::new("voxelgrid", id),
                &leaf,
                |b, &leaf| {
                    b.iter(|| {
                        let out: PointCloud =
                            voxelgrid_sampling(black_box(&cloud), black_box(leaf));
                        black_box(out);
                    });
                },
            );
        }
    }
    group.finish();
}

criterion_group!(benches, bench_downsampling);
criterion_main!(benches);
