use re_log_encoding::{Decodable, Encodable, StreamFooter};
use serde_json::Value;
use std::{fs, path::Path, process::Command};

fn rejection(path: &Path) -> String {
    let output = Command::new(env!("CARGO_BIN_EXE_firefly-verify"))
        .arg("--transport")
        .arg("board")
        .arg(path)
        .output()
        .unwrap();
    assert_eq!(output.status.code(), Some(1));
    let report: Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(report["passed"], false);
    report["failures"][0].as_str().unwrap().to_owned()
}

#[test]
fn requires_a_closed_intact_recording_store() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let blueprint = root.join("configs/firefly.rbl");
    assert!(rejection(&blueprint).contains("missing recording manifest"));
    let dir = tempfile::tempdir_in(root.join("target")).unwrap();
    let path = dir.path().join("damaged.rrd");
    let mut bytes = fs::read(blueprint).unwrap();
    let trailer = bytes.split_off(bytes.len() - StreamFooter::ENCODED_SIZE_BYTES);
    fs::write(&path, &bytes).unwrap();
    assert!(rejection(&path).starts_with("invalid recording:"));
    let mut footer = StreamFooter::from_rrd_bytes(&trailer).unwrap();
    footer.entries[0].crc_excluding_header ^= 1;
    footer.to_rrd_bytes(&mut bytes).unwrap();
    fs::write(&path, &bytes).unwrap();
    assert!(rejection(&path).to_ascii_lowercase().contains("crc"));
}
