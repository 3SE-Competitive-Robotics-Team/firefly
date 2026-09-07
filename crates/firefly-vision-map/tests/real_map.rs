//! 真实库图文件加载（离线构建产物，缺失时跳过）。

use std::path::PathBuf;

fn map_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("apps")
        .join("planner")
        .join("maps")
        .join("straight_forward.ffvmap")
}

#[test]
fn loads_offline_map_or_skip() {
    let path = map_path();
    if !path.is_file() {
        println!("skip: 无离线库图 {}", path.display());
        return;
    }
    let map = firefly_vision_map::load_map(&path).unwrap();
    assert_eq!(map.frames.len(), 18);
    assert!(map.num_points() > 4000);
    // 远端多帧同位悬停：大窗口应含第 9 帧，小窗口只保证半径内
    let near = map.query_neighbors(map.frames[9].position, 2.0, 20);
    assert!(near.contains(&9));
    let near3 = map.query_neighbors(map.frames[9].position, 2.0, 3);
    assert_eq!(near3.len(), 3);
    for &i in &near3 {
        let f = &map.frames[i];
        let d = (f.position[0] - 4.2)
            .hypot(f.position[1] - 4.0)
            .hypot(f.position[2] - 1.4);
        assert!(d <= 2.0 + 1e-9);
    }
}
