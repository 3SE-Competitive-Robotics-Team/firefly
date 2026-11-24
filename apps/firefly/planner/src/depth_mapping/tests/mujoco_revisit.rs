use super::*;
use crate::map_input::MapStatus;
use firefly_map::{Evidence, OnlineState, SegmentRule};
use std::{path::Path, process::Command};

#[test]
#[ignore = "requires the workspace MuJoCo Python environment; run in release mode"]
fn mujoco_depth_revisit() {
    let frames = mujoco_frames();
    for lifetime in [None, Some(3.)] {
        replay(&frames, lifetime);
    }
}

fn mujoco_frames() -> String {
    let app = Path::new(env!("CARGO_MANIFEST_DIR"));
    let output = Command::new(app.join("../../../.venv/bin/python"))
        .arg(app.join("tests/mujoco_depth.py"))
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let frames = String::from_utf8(output.stdout).unwrap();
    assert_eq!(frames.lines().count(), 4000);
    frames
}

#[test]
#[ignore = "requires the workspace MuJoCo scene; checks the first ray after a long absence"]
fn first_returning_ray_refreshes_free_space() {
    let frames = mujoco_frames();
    let forward = frames.lines().nth(10).unwrap();
    let backward = frames.lines().nth(60).unwrap();
    let mut failures = Vec::new();
    for gap in [255, 256, 257, 512] {
        let (_, mut fusion) = registered_scene(None);
        let start = Instant::now();
        let probe = Vector3::new(1.05, 0.05, 1.05);
        for i in 0..=gap + 3 {
            let (view, scene_index) = if i > 2 && i < gap + 2 {
                (backward, 60)
            } else {
                (forward, 10)
            };
            let t = f64::from(i) / 20.;
            let (image, pose) =
                decode_frame(view, scene_index, t, start + Duration::from_secs_f64(t));
            fusion.integrate(image, pose, t).unwrap();
            if i == 2 || i >= gap + 2 {
                let observation = fusion.snapshot().unwrap().unwrap().map.observed.sample(
                    probe,
                    t,
                    Duration::from_secs(1),
                );
                println!("gap={gap} frame={i} t={t} centerline={observation:?}");
                if observation != OnlineState::Free(t) {
                    failures.push((gap, i, observation));
                }
            }
        }
    }
    assert!(
        failures.is_empty(),
        "visible free cells remained stale: {failures:?}"
    );
}

fn registered_scene(lifetime: Option<f64>) -> (firefly_map::GridMap, DepthFusion) {
    let opts = DepthOptions {
        intrinsics: [16., 16., 8., 6.],
        translation: [0.; 3],
        rotation: [0., 0., 0., 1.],
        occupied_max_age: lifetime,
        ..Default::default()
    };
    let mut map = crate::config::MappingOptions {
        range: [5., 3., 2.],
        ..Default::default()
    }
    .build(0.4)
    .unwrap();
    let field: firefly_map::MapFile = "FORMAT firefly-map 1\nRESOLUTION 0.1\nORIGIN 0 0 0\nDIMS 50 2 20\nOCCUPANCY\n4.05 0.05 1.05\n".parse().unwrap();
    field
        .install_fixed(&mut map, &Isometry3::identity())
        .unwrap();
    let fusion = DepthFusion::new(&map, opts, 0.4).unwrap();
    (map, fusion)
}

fn decode_frame(line: &str, index: usize, t: f64, now: Instant) -> (DepthFrame, Isometry3<f64>) {
    let phase = index % 400;
    let mut fields = line.split_ascii_whitespace();
    assert_eq!(fields.next().unwrap().parse::<usize>().unwrap(), index);
    let y = fields.next().unwrap().parse::<f64>().unwrap();
    let facing = fields.next().unwrap().parse::<f64>().unwrap();
    let pixels: Vec<u16> = fields.map(|p| p.parse().unwrap()).collect();
    let expected_depth = if (324..350).contains(&phase) {
        0
    } else if phase < 40 {
        2000
    } else {
        4000
    };
    assert_eq!(
        pixels[6 * 17 + 8],
        expected_depth,
        "MuJoCo center ray at frame {index}"
    );
    let pose = Isometry3::new(
        Vector3::new(0.05, y, 1.05),
        Vector3::y() * (facing * std::f64::consts::FRAC_PI_2),
    );
    let frame = DepthFrame {
        timestamp: t,
        received: now,
        image: DepthImage::new(17, 13, pixels).unwrap(),
        trace: TraceContext::default(),
    };
    (frame, pose)
}

fn replay(frames: &str, lifetime: Option<f64>) {
    let (mut map, mut fusion) = registered_scene(lifetime);
    let mut input = MapInput::default();
    let box_at = Vector3::new(2.05, 0.05, 1.05);
    let wall = Vector3::new(4.05, 0.05, 1.05);
    let floor_lane = Vector3::new(1.05, 0.05, 1.05);
    let epoch = Instant::now();
    let max_age = Duration::from_secs(1);
    let mut hit_frames = [None; 10];
    let mut cleared_frames = [None; 10];
    let mut blackout_rejections = 0;
    let mut residual_face_counts = Vec::new();
    for (index, line) in frames.lines().enumerate() {
        let phase = index % 400;
        let lap = index / 400;
        let elapsed = Duration::from_millis(50 * index as u64);
        let t = elapsed.as_secs_f64();
        let now = epoch + elapsed;
        let (frame, pose) = decode_frame(line, index, t, now);
        fusion.integrate(frame, pose, t).unwrap();
        if index % 10 == 0 {
            fusion.map.fade();
        }
        let Some(snapshot) = fusion.snapshot().unwrap() else {
            assert_eq!(index, 0);
            continue;
        };
        input.observe_local(snapshot).unwrap();
        let nav = OdomMessage {
            position_y: pose.translation.y,
            position_z: 1.05,
            ..odometry(t, 0.05)
        };
        let status = input.synchronize(&mut map, &nav, now, max_age);
        if (344..350).contains(&phase) {
            assert_eq!(status, MapStatus::Unavailable, "blackout frame {index}");
            blackout_rejections += 1;
            continue;
        }
        assert_eq!(status, MapStatus::Ready, "frame {index}");
        if phase == 240 {
            assert!(
                map.cell_at(box_at, t, max_age).is_none(),
                "probe must leave window"
            );
        }
        let Some(sample) = map.cell_at(box_at, t, max_age) else {
            continue;
        };
        let fixed = map.cell_at(wall, t, max_age).unwrap();
        assert!(
            fixed.fixed_occupied && fixed.collision,
            "wall lost on frame {index}"
        );
        assert!(!sample.fixed_occupied);
        if phase < 40
            && let OnlineState::Occupied(_) = sample.online
        {
            hit_frames[lap].get_or_insert(phase);
        }
        if (120..160).contains(&phase)
            && matches!(sample.online, OnlineState::Free(_))
            && !sample.collision
        {
            cleared_frames[lap].get_or_insert(phase - 120);
        }
        if phase == 39 {
            assert!(matches!(sample.online, OnlineState::Occupied(_)) && sample.collision);
        }
        if phase == 119 {
            assert!(
                matches!(
                    map.cell_at(floor_lane, t, max_age).unwrap().online,
                    OnlineState::Expired(Evidence::Free(_))
                ),
                "old lane renewed at {index}"
            );
            assert_eq!(
                sample.collision,
                lifetime.is_none(),
                "unseen box at {index}"
            );
        }
        if phase == 159 || phase == 399 {
            if phase == 159 {
                residual_face_counts.push(box_face_counts(&map, box_at, t));
            }
            assert!(
                map.segment_is_clear(floor_lane, box_at, SegmentRule::KnownObstacles),
                "ghost on centerline at frame {index}: {sample:?}"
            );
        }
    }
    println!(
        "occupied_lifetime={lifetime:?} hits_in_frames={hit_frames:?} clears_after_revisit_in_frames={cleared_frames:?} blackout_rejections={blackout_rejections} residual_face_counts_raw_and_inflated={residual_face_counts:?}"
    );
    assert!(hit_frames.iter().all(Option::is_some));
    assert!(cleared_frames.iter().all(Option::is_some));
    assert_eq!(blackout_rejections, 60);
    assert_eq!(residual_face_counts, [(0, 0); 10]);
}

/// Sample the 11 × 11 voxel centers across the former front face of the box.
fn box_face_counts(map: &firefly_map::GridMap, center: Vector3<f64>, t: f64) -> (usize, usize) {
    let mut occupied = 0;
    let mut blocked = 0;
    for y in -5..=5 {
        for z in -5..=5 {
            let p = center + Vector3::new(0., f64::from(y), f64::from(z)) * map.resolution();
            let s = map.cell_at(p, t, Duration::from_secs(1)).unwrap();
            assert!(!s.fixed_occupied);
            if matches!(
                s.online,
                OnlineState::Occupied(_) | OnlineState::Expired(Evidence::Occupied(_))
            ) {
                occupied += 1;
            }
            blocked += usize::from(s.collision);
        }
    }
    (occupied, blocked)
}
