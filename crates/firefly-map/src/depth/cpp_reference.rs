//! Differential test against the fixed upstream C++ bodies, not a Rust-generated baseline.
//! Build the oracle with tests/reference/generate.py, then explicitly run this ignored test.
use super::{DepthImage, MappingParameters, grid::GridMap};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use std::fmt::Write as _;
use std::io::Write as _;
use std::process::{Command, Stdio};

enum Operation {
    Depth {
        now: f64,
        position: [f64; 3],
        angle: f64,
        pixels: Vec<u16>,
    },
    Tick(f64),
    Fade,
}

fn operations() -> Vec<Operation> {
    let mut result = vec![Operation::Tick(0.0), Operation::Tick(10.0)];
    for frame in 0..300 {
        let position = match frame / 50 {
            0 | 4 => [0.25, 0.25, 0.25],
            1 => [0.75, -0.25, 0.25],
            2 => [-0.75, 0.75, -0.25],
            3 => [-0.75, 0.75, 0.75],
            _ => [0.25, 0.25, 0.75],
        };
        let mut pixels = vec![1000; 16];
        if frame % 17 == 0 {
            pixels = vec![
                0, 99, 100, 199, 200, 1000, 1000, 1500, 2000, 8000, 65535, 500, 500, 1000, 0, 3000,
            ];
        }
        if frame % 29 == 0 {
            pixels.fill(0);
        }
        result.push(Operation::Depth {
            now: 10.0 + f64::from(frame) * 0.032,
            position,
            angle: if frame % 31 == 0 { 0.3 } else { 0.0 },
            pixels,
        });
        if frame % 13 == 0 {
            result.push(Operation::Fade);
        }
    }
    result.push(Operation::Tick(30.0));
    result.push(Operation::Depth {
        now: 30.1,
        position: [0.25; 3],
        angle: 0.0,
        pixels: vec![500; 16],
    });
    result
}

fn snapshot(map: &GridMap) -> Vec<f64> {
    let mut values = vec![
        f64::from(u8::from(map.initialized())),
        f64::from(u8::from(map.odom_depth_timeout())),
        map.projected_points().len() as f64,
    ];
    for point in map.projected_points() {
        values.extend_from_slice(point.as_slice());
    }
    values.extend_from_slice(map.occupancy_buffer());
    values.extend(map.inflation_buffer().iter().map(|&v| f64::from(v)));
    if let Some((low, high)) = map.bounds() {
        values.extend(low.map(f64::from));
        values.extend(high.map(f64::from));
        for point in [
            Vector3::zeros(),
            Vector3::new(0.25, 0.25, 1.25),
            Vector3::from(high.map(|v| f64::from(v + 1) * map.resolution())),
            Vector3::from(low.map(|v| f64::from(v) * map.resolution())),
            Vector3::new(100.0, 0.0, 3.0),
        ] {
            values.push(f64::from(map.occupancy_at(point)));
            values.push(f64::from(map.inflated_occupancy_at(point)));
        }
    }
    values
}

#[test]
#[ignore = "requires the pinned upstream C++ oracle built by tests/reference/generate.py"]
fn cpp_local_mapping_matches_each_update() {
    let binary =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/ego-map-reference");
    for variant in 0..3 {
        let config = MappingParameters {
            resolution: 0.5,
            local_update_range: [2.0, 2.0, 1.5],
            obstacles_inflation: 0.5,
            fx: 2.0,
            fy: 3.0,
            cx: 1.0,
            cy: 1.0,
            use_depth_filter: variant == 1,
            depth_filter_margin: i32::from(variant == 1),
            depth_filter_mindist: 0.2,
            k_depth_scaling_factor: 1000.0,
            skip_pixel: if variant == 2 { 2 } else { 1 },
            fading_time: 0.25,
            ..MappingParameters::default()
        };
        let output = crate::GridMapBuilder::new(config.resolution, [8; 3])
            .build()
            .unwrap();
        let mut map = super::DepthMapper::new(config, Isometry3::identity(), &output).unwrap();
        map.grid.reference_wall = Some((-0.1, 2.0));
        let mut input = format!("{variant}\n");
        let mut expected = Vec::new();
        for operation in operations() {
            match operation {
                Operation::Depth {
                    now,
                    position,
                    angle,
                    pixels,
                } => {
                    write!(
                        input,
                        "D {now:.17} {:.17} {:.17} {:.17} {angle:.17}",
                        position[0], position[1], position[2]
                    )
                    .unwrap();
                    for pixel in &pixels {
                        write!(input, " {pixel}").unwrap();
                    }
                    input.push('\n');
                    let pose = Isometry3::from_parts(
                        Translation3::from(Vector3::from(position)),
                        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle),
                    );
                    map.integrate(DepthImage::new(4, 4, pixels).unwrap(), pose, now, now)
                        .unwrap();
                }
                Operation::Tick(now) => {
                    writeln!(input, "T {now:.17}").unwrap();
                    map.timed_out_at(now).unwrap();
                }
                Operation::Fade => {
                    input.push_str("F\n");
                    map.fade();
                }
            }
            expected.push(snapshot(&map.grid));
        }
        let original = replay(&binary.join("reference"), input.clone());
        let corrected = replay(&binary.join("reference-boundaries"), input);
        let changed_steps = original
            .lines()
            .zip(corrected.lines())
            .filter(|(a, b)| a != b)
            .count();
        assert_eq!(original.lines().count(), expected.len());
        eprintln!(
            "variant {variant}: {changed_steps}/{} snapshots changed by the two boundary rules",
            expected.len()
        );
        assert!(changed_steps > 0);
        let actual: Vec<_> = corrected.lines().collect();
        assert_eq!(actual.len(), expected.len());
        for (step, (line, expected)) in actual.iter().zip(expected).enumerate() {
            let actual: Vec<f64> = line
                .split_whitespace()
                .map(|v| v.parse().unwrap())
                .collect();
            assert_eq!(
                actual.len(),
                expected.len(),
                "variant {variant}, step {step}"
            );
            for (field, (cpp, rust)) in actual.iter().zip(&expected).enumerate() {
                let tolerance = 2e-12 * cpp.abs().max(1.0);
                assert!(
                    (cpp - rust).abs() <= tolerance,
                    "variant {variant}, step {step}, field {field}: C++ {cpp}, Rust {rust}"
                );
            }
        }
    }
}

fn replay(binary: &std::path::Path, input: String) -> String {
    let mut child = Command::new(binary)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("start the C++ oracle built by tests/reference/generate.py");
    let mut stdin = child.stdin.take().unwrap();
    // Drain stdout concurrently with input to avoid filling either OS pipe.
    let writer = std::thread::spawn(move || stdin.write_all(input.as_bytes()));
    let output = child.wait_with_output().unwrap();
    writer.join().unwrap().unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8(output.stdout).unwrap()
}
