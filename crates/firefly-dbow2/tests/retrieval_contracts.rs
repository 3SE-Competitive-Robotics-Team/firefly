// SPDX-License-Identifier: BSD-3-Clause

//! Mathematical score oracles and an online train/add/query lifecycle, without a vocabulary asset.
use firefly_dbow2::{
    bow_vector::{BowVector, LNorm, ScoringType, WeightingType},
    f_brief::FBrief,
    feature_vector::FeatureVector,
    scoring_object::GeneralScoring,
    templated_database::TemplatedDatabase,
    templated_vocabulary::{SeededRandom, TemplatedVocabulary},
};
use proptest::prelude::*;

fn bow(values: &[f64]) -> BowVector {
    BowVector(
        values
            .iter()
            .enumerate()
            .filter(|(_, v)| **v != 0.0)
            .map(|(i, &v)| (i as u32, v))
            .collect(),
    )
}
fn vocabulary() -> TemplatedVocabulary<FBrief> {
    let mut v = TemplatedVocabulary::new(2, 1, WeightingType::Tf, ScoringType::L1Norm);
    v.create(
        &[vec![[false; 256]], vec![[true; 256]]],
        &mut SeededRandom::new(19),
    )
    .unwrap();
    assert_eq!(v.size(), 2);
    v
}
proptest! {
    #![proptest_config(ProptestConfig::with_failure_persistence(
        proptest::test_runner::FileFailurePersistence::WithSource("proptest-regressions")
    ))]
    #[test]
    fn normalized_l1_score_equals_histogram_intersection(
        a in prop::array::uniform8(0u16..1000), b in prop::array::uniform8(0u16..1000)
    ) {
        let a = a.map(f64::from); let b = b.map(f64::from);
        let sa = a.iter().sum::<f64>(); let sb = b.iter().sum::<f64>();
        let mut va = bow(&a); let mut vb = bow(&b);
        va.normalize(LNorm::L1); vb.normalize(LNorm::L1);
        // Empty mass has no intersection. Positive mass is normalized independently here.
        let expected = if sa == 0.0 || sb == 0.0 {0.0} else {(0..8).map(|i| (a[i]/sa).min(b[i]/sb)).sum()};
        let score = ScoringType::L1Norm.score(&va,&vb);
        prop_assert!((score-expected).abs() < 2e-15);
        prop_assert!((-2e-15..=1.0+2e-15).contains(&score));
        // The score is symmetric mathematically; subtracting the two masses in reversed order can round differently.
        prop_assert!((score - ScoringType::L1Norm.score(&vb,&va)).abs() < 2e-15);
    }

    #[test]
    fn brief_encoding_preserves_every_bit(bits in any::<[bool;256]>()) {
        let text = FBrief::to_string(&bits);
        prop_assert_eq!(text.len(),256);
        prop_assert_eq!(FBrief::from_string(&text).unwrap(),bits);
        let complement = bits.map(|bit| !bit);
        prop_assert_eq!(FBrief::distance(&bits,&complement),256.0);
    }
}

#[test]
fn score_families_have_independent_two_bin_analytic_values() {
    let a = bow(&[0.25, 0.75]);
    let b = bow(&[0.75, 0.25]);
    let expected = [
        (ScoringType::L1Norm, 0.5),
        (ScoringType::ChiSquare, 0.75),
        (ScoringType::Bhattacharyya, 3.0f64.sqrt() / 2.0),
        (ScoringType::Kl, 0.5 * 3.0f64.ln()),
        (ScoringType::DotProduct, 0.375),
    ];
    for (kind, expected) in expected {
        assert!((kind.score(&a, &b) - expected).abs() < 1e-15, "{kind:?}");
    }
    let mut a = bow(&[3.0, 4.0]);
    a.normalize(LNorm::L2);
    let mut b = bow(&[4.0, 3.0]);
    b.normalize(LNorm::L2);
    assert!((ScoringType::L2Norm.score(&a, &b) - 0.8).abs() < 1e-15);
    for norm in [LNorm::L1, LNorm::L2] {
        let mut empty = BowVector::new();
        empty.normalize(norm);
        assert!(empty.is_empty());
        let mut zero = bow(&[0.0]);
        zero.add_weight(0, 0.0);
        zero.normalize(norm);
        assert_eq!(zero[&0], 0.0);
    }
}

#[test]
fn online_retrieval_ranks_identity_limits_history_and_clears_index() {
    let v = vocabulary();
    let first = vec![[false; 256]; 4];
    let other = vec![[true; 256]; 4];
    let mixed = vec![[false; 256], [true; 256]];
    let mut db = TemplatedDatabase::new(&v, true, 0);
    assert_eq!(db.add(&first), 0);
    assert_eq!(db.add(&other), 1);
    assert_eq!(db.add(&mixed), 2);
    let result = db.query(&first, 10, -1);
    assert_eq!(result.iter().map(|r| r.id).collect::<Vec<_>>(), [0, 2]);
    assert_eq!(result[0].score, 1.0);
    assert_eq!(result[1].score, 0.5);
    assert_eq!(db.query(&first, 1, -1).len(), 1);
    assert_eq!(
        db.query(&first, 10, 1)
            .iter()
            .map(|r| r.id)
            .collect::<Vec<_>>(),
        [0]
    );
    assert!(db.query(&[], 10, -1).is_empty());
    db.clear();
    assert_eq!(db.size(), 0);
    assert!(db.query(&first, 10, -1).is_empty());
    assert_eq!(db.add(&other), 0);
}

#[test]
fn inverted_index_scores_match_dense_intersection_under_insert_permutation() {
    let v = vocabulary();
    let documents = [
        bow(&[1.0, 0.0]),
        bow(&[0.0, 1.0]),
        bow(&[0.25, 0.75]),
        bow(&[0.6, 0.4]),
    ];
    let query = bow(&[0.8, 0.2]);
    for order in [[0, 1, 2, 3], [3, 1, 0, 2]] {
        let mut db = TemplatedDatabase::new(&v, false, 0);
        for &i in &order {
            db.add_bow(&documents[i], &FeatureVector::new());
        }
        let results = db.query_bow(&query, 0, -1);
        assert_eq!(results.len(), 4);
        for row in results.iter() {
            let d = &documents[order[row.id as usize]];
            let expected: f64 = (0..2)
                .map(|i| {
                    query
                        .get(&i)
                        .copied()
                        .unwrap_or(0.0)
                        .min(d.get(&i).copied().unwrap_or(0.0))
                })
                .sum();
            assert!((row.score - expected).abs() < 1e-15);
        }
        assert!(
            results
                .windows(2)
                .all(|pair| pair[0].score >= pair[1].score)
        );
    }
}

#[test]
fn vocabulary_storage_rejects_cycles_and_invalid_descriptor_text() {
    let v = vocabulary();
    let mut storage = v.to_storage();
    storage.nodes[0].parent_id = storage.nodes[0].node_id;
    assert!(TemplatedVocabulary::<FBrief>::from_storage(storage).is_err());
    let mut storage = v.to_storage();
    storage.nodes[0].descriptor = "not binary".into();
    assert!(TemplatedVocabulary::<FBrief>::from_storage(storage).is_err());
    assert!(FBrief::from_string("").is_err());
    let empty: Vec<Vec<[bool; 256]>> = Vec::new();
    assert!(
        TemplatedVocabulary::<FBrief>::new(0, 1, WeightingType::Tf, ScoringType::L1Norm)
            .create(&empty, &mut SeededRandom::new(1))
            .is_err()
    );
}
