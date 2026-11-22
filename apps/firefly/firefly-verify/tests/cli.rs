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
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../..");
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

#[test]
fn complete_scoring_rejects_an_evicted_or_unsealed_range_before_decoding() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../..");
    let run = tempfile::tempdir_in(root.join("target")).unwrap();
    let directory = run.path().join("rrd");
    fs::create_dir(&directory).unwrap();
    fs::File::create(run.path().join(".read.lock")).unwrap();
    let mut manifest = serde_json::json!({"format":1,"rrd_last":2,"gaps":{},"evicted":{}});
    let save =
        |value: &Value| fs::write(run.path().join("manifest.json"), value.to_string()).unwrap();
    save(&manifest);
    fs::write(directory.join("000001.rrd"), []).unwrap();
    assert!(rejection(run.path()).contains("missing recording segment"));
    let unfinished = directory.join("000002.rrd.partial");
    fs::write(&unfinished, []).unwrap();
    assert!(rejection(run.path()).contains("segment"));
    fs::rename(&unfinished, directory.join("000002.rrd")).unwrap();
    manifest["evicted"]["rrd"] = serde_json::json!({"first":1,"last":1});
    save(&manifest);
    assert!(rejection(&directory.join("000002.rrd")).contains("missing recording evidence"));
    let retained = firefly_archive::reader::RecordingFiles::retained(run.path()).unwrap();
    let lock = fs::File::open(run.path().join(".read.lock")).unwrap();
    assert!(matches!(
        lock.try_lock(),
        Err(std::fs::TryLockError::WouldBlock)
    ));
    drop(retained);
    // A concurrent subprocess can hold a forked copy of the descriptor until exec.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    loop {
        match lock.try_lock() {
            Ok(()) => break,
            Err(std::fs::TryLockError::WouldBlock) if std::time::Instant::now() < deadline => {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            Err(error) => panic!("reader lock still held after drop: {error}"),
        }
    }
}
