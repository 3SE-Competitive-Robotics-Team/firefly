/*
 * File: vocabulary_storage.rs
 * Description: DBoW2 vocabulary storage round-trip against the shipped OpenCV file.
 * License: see docs/licenses/crates/firefly-dbow2/LICENSE.txt.
 */
//! `OpenCV` `FileStorage` vocabulary storage contract.
//!
//! The shipped `DBoW2` vocabulary is an `OpenCV`-generated `%YAML:1.0` document whose
//! flow-map separators omit the space YAML requires. The storage layer must read it,
//! re-emit a document that still loads, and stay byte-stable across re-emission.

use firefly_dbow2::templated_vocabulary::TemplatedVocabulary;
use firefly_okvis_x::frontend::f_brisk::FBrisk;
use std::{
    path::{Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
};

const VOCABULARY: &[u8] = include_bytes!("../resources/small_voc.yml.gz");

struct Fixture(PathBuf);
impl Fixture {
    fn new() -> Self {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../logs")
            .join(format!(
                "okvis-vocabulary-test-{}-{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
        std::fs::create_dir_all(&path).unwrap();
        Self(path)
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        std::fs::remove_dir_all(&self.0).unwrap();
    }
}

#[test]
fn opencv_vocabulary_round_trips_through_storage() {
    let fixture = Fixture::new();
    let vocabulary = TemplatedVocabulary::<FBrisk>::load_bytes(VOCABULARY)
        .expect("the shipped OpenCV vocabulary must load");
    assert!(vocabulary.k > 0);
    assert!(vocabulary.size() > 0);

    let first = fixture.0.join("vocabulary.yml");
    vocabulary.save(&first).expect("vocabulary must save");

    let reloaded =
        TemplatedVocabulary::<FBrisk>::load(&first).expect("emitted vocabulary must load");
    assert_eq!(reloaded.k, vocabulary.k);
    assert_eq!(reloaded.levels, vocabulary.levels);
    assert_eq!(reloaded.size(), vocabulary.size());

    let second = fixture.0.join("vocabulary-again.yml");
    reloaded
        .save(&second)
        .expect("reloaded vocabulary must save");
    assert_eq!(
        std::fs::read(&first).unwrap(),
        std::fs::read(&second).unwrap(),
        "re-emitting the vocabulary must be byte-stable"
    );
}
