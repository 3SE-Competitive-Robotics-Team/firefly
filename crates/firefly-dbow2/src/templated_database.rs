/*
 * DBoW2 — Copyright (c) 2011–2013 Dorian Galvez-Lopez.
 * License: see LICENSE.txt.
 */
//! `DBoW2` inverted file, direct file, and six database query paths.
use super::{
    bow_vector::{BowVector, ScoringType, WeightingType},
    feature_vector::FeatureVector,
    query_results::{EntryId, QueryResults, Result as QueryResult},
    scoring_object::log_eps,
    templated_vocabulary::{
        DescriptorOps, StorageResult, TemplatedVocabulary, VocabularyStorage, read_storage,
        write_storage,
    },
};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, fmt, path::Path};
pub const MIN_COMMON_WORDS: usize = 5;
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct IfPair {
    #[serde(rename = "imageId")]
    pub entry_id: EntryId,
    #[serde(rename = "weight")]
    pub word_weight: f64,
}
pub struct TemplatedDatabase<F: DescriptorOps> {
    pub vocabulary: Option<TemplatedVocabulary<F>>,
    pub use_di: bool,
    pub di_levels: i32,
    pub inverted_file: Vec<Vec<IfPair>>,
    pub direct_file: Vec<FeatureVector>,
    pub n_entries: EntryId,
}
impl<F: DescriptorOps> Default for TemplatedDatabase<F> {
    fn default() -> Self {
        Self::new_empty(true, 0)
    }
}
impl<F: DescriptorOps> Clone for TemplatedDatabase<F> {
    fn clone(&self) -> Self {
        match &self.vocabulary {
            Some(v) => Self::from_loaded_vocabulary(v.clone(), self.use_di, self.di_levels),
            None => Self::new_empty(self.use_di, self.di_levels),
        }
    }
}
impl<F: DescriptorOps> TemplatedDatabase<F> {
    #[must_use]
    pub fn new(vocabulary: &TemplatedVocabulary<F>, use_di: bool, di_levels: i32) -> Self {
        Self::from_loaded_vocabulary(vocabulary.clone(), use_di, di_levels)
    }
    fn from_loaded_vocabulary(
        vocabulary: TemplatedVocabulary<F>,
        use_di: bool,
        di_levels: i32,
    ) -> Self {
        let inverted_file = vec![Vec::new(); vocabulary.size()];
        Self {
            vocabulary: Some(vocabulary),
            use_di,
            di_levels,
            inverted_file,
            direct_file: Vec::new(),
            n_entries: 0,
        }
    }
    pub fn set_vocabulary(&mut self, vocabulary: &TemplatedVocabulary<F>) {
        self.vocabulary = Some(vocabulary.clone());
        self.clear();
    }
    pub fn set_vocabulary_with_index(
        &mut self,
        vocabulary: &TemplatedVocabulary<F>,
        use_di: bool,
        levels: i32,
    ) {
        self.use_di = use_di;
        self.di_levels = levels;
        self.set_vocabulary(vocabulary);
    }
    #[must_use]
    pub fn get_vocabulary(&self) -> Option<&TemplatedVocabulary<F>> {
        self.vocabulary.as_ref()
    }
    fn vocabulary(&self) -> &TemplatedVocabulary<F> {
        self.vocabulary
            .as_ref()
            .expect("database operation requires a vocabulary")
    }
    #[must_use]
    pub fn new_empty(use_di: bool, di_levels: i32) -> Self {
        Self {
            vocabulary: None,
            use_di,
            di_levels,
            inverted_file: Vec::new(),
            direct_file: Vec::new(),
            n_entries: 0,
        }
    }
    #[must_use]
    pub fn size(&self) -> EntryId {
        self.n_entries
    }
    #[must_use]
    pub fn using_direct_index(&self) -> bool {
        self.use_di
    }
    #[must_use]
    pub fn get_direct_index_levels(&self) -> i32 {
        self.di_levels
    }
    pub fn clear(&mut self) {
        self.inverted_file = vec![Vec::new(); self.vocabulary().size()];
        self.direct_file.clear();
        self.n_entries = 0;
    }
    pub fn allocate(&mut self, nd: usize, ni: usize) {
        for row in &mut self.inverted_file {
            if ni > row.len() {
                row.reserve(ni - row.len());
            }
        }
        if self.use_di && self.direct_file.len() < nd {
            self.direct_file.resize_with(nd, FeatureVector::new);
        }
    }
    pub fn add(&mut self, features: &[F::Descriptor]) -> EntryId {
        let (bow, fv) = self
            .vocabulary()
            .transform_with_features(features, self.di_levels);
        self.add_bow(&bow, &fv)
    }
    pub fn add_with_vectors(
        &mut self,
        features: &[F::Descriptor],
    ) -> (EntryId, BowVector, FeatureVector) {
        let (bow, fv) = self
            .vocabulary()
            .transform_with_features(features, self.di_levels);
        let id = self.add_bow(&bow, &fv);
        (id, bow, fv)
    }
    pub fn add_bow(&mut self, bow: &BowVector, fv: &FeatureVector) -> EntryId {
        let id = self.n_entries;
        self.n_entries += 1;
        if self.use_di {
            if id as usize == self.direct_file.len() {
                self.direct_file.push(fv.clone());
            } else {
                self.direct_file[id as usize] = fv.clone();
            }
        }
        for (&word, &weight) in bow.iter() {
            self.inverted_file[word as usize].push(IfPair {
                entry_id: id,
                word_weight: weight,
            });
        }
        id
    }
    pub fn query(&self, features: &[F::Descriptor], max_results: i32, max_id: i32) -> QueryResults {
        self.query_bow(&self.vocabulary().transform(features), max_results, max_id)
    }
    #[must_use]
    pub fn query_bow(&self, bow: &BowVector, max_results: i32, max_id: i32) -> QueryResults {
        match self.vocabulary().scoring {
            ScoringType::L1Norm => self.query_l1(bow, max_results, max_id),
            ScoringType::L2Norm => self.query_l2(bow, max_results, max_id),
            ScoringType::ChiSquare => self.query_chi_square(bow, max_results, max_id),
            ScoringType::Kl => self.query_kl(bow, max_results, max_id),
            ScoringType::Bhattacharyya => self.query_bhattacharyya(bow, max_results, max_id),
            ScoringType::DotProduct => self.query_dot_product(bow, max_results, max_id),
        }
    }
    fn visit(&self, bow: &BowVector, max_id: i32, mut f: impl FnMut(EntryId, f64, f64)) {
        for (&word, &q) in bow.iter() {
            for pair in &self.inverted_file[word as usize] {
                if max_id == -1 || (pair.entry_id as i32) < max_id {
                    f(pair.entry_id, q, pair.word_weight);
                }
            }
        }
    }
    fn finish(pairs: BTreeMap<EntryId, f64>, max_results: i32, descending: bool) -> QueryResults {
        let mut out = QueryResults(
            pairs
                .into_iter()
                .map(|(id, s)| QueryResult::new(id, s))
                .collect(),
        );
        Self::sort_cut(&mut out, max_results, descending);
        out
    }
    fn sort_cut(out: &mut QueryResults, max_results: i32, descending: bool) {
        out.sort_by(|a, b| {
            if descending {
                b.score.partial_cmp(&a.score)
            } else {
                a.score.partial_cmp(&b.score)
            }
            .unwrap_or(std::cmp::Ordering::Equal)
        });
        if max_results > 0 {
            out.truncate(max_results as usize);
        }
    }
    #[must_use]
    pub fn query_l1(&self, bow: &BowVector, max_results: i32, max_id: i32) -> QueryResults {
        let mut pairs = BTreeMap::new();
        self.visit(bow, max_id, |id, q, d| {
            let value = (q - d).abs() - q.abs() - d.abs();
            pairs
                .entry(id)
                .and_modify(|score| *score += value)
                .or_insert(value);
        });
        let mut out = Self::finish(pairs, max_results, false);
        for r in &mut out.0 {
            r.score = -r.score / 2.0;
        }
        out
    }
    #[must_use]
    pub fn query_l2(&self, bow: &BowVector, max_results: i32, max_id: i32) -> QueryResults {
        let mut pairs = BTreeMap::new();
        self.visit(bow, max_id, |id, q, d| {
            let value = -q * d;
            pairs
                .entry(id)
                .and_modify(|score| *score += value)
                .or_insert(value);
        });
        let mut out = Self::finish(pairs, max_results, false);
        for r in &mut out.0 {
            r.score = if r.score <= -1.0 {
                1.0
            } else {
                1.0 - (1.0 + r.score).sqrt()
            };
        }
        out
    }
    #[must_use]
    pub fn query_chi_square(&self, bow: &BowVector, max_results: i32, max_id: i32) -> QueryResults {
        let mut pairs = BTreeMap::<EntryId, (f64, usize, f64, f64)>::new();
        self.visit(bow, max_id, |id, q, d| {
            let value = if q + d == 0.0 { 0.0 } else { -q * d / (q + d) };
            pairs
                .entry(id)
                .and_modify(|p| {
                    p.0 += value;
                    p.1 += 1;
                    p.2 += q;
                    p.3 += d;
                })
                .or_insert((value, 1, q, d));
        });
        let mut out = QueryResults::default();
        for (id, (score, n, vi, wi)) in pairs {
            if n >= MIN_COMMON_WORDS {
                let mut r = QueryResult::new(id, score);
                r.n_words = Some(n as i32);
                r.sum_common_vi = Some(vi);
                r.sum_common_wi = Some(wi);
                r.expected_chi_score = Some(2.0 * wi / (1.0 + wi));
                out.push(r);
            }
        }
        Self::sort_cut(&mut out, max_results, false);
        for r in &mut out.0 {
            r.score *= -2.0;
            r.chi_score = Some(r.score);
        }
        out
    }
    #[must_use]
    pub fn query_kl(&self, bow: &BowVector, max_results: i32, max_id: i32) -> QueryResults {
        let mut pairs = BTreeMap::new();
        self.visit(bow, max_id, |id, vi, wi| {
            let value = if vi != 0.0 && wi != 0.0 {
                vi * (vi / wi).ln()
            } else {
                0.0
            };
            pairs
                .entry(id)
                .and_modify(|score| *score += value)
                .or_insert(value);
        });
        for (&id, score) in &mut pairs {
            let mut missing = 0.0;
            for (&word, &vi) in bow.iter() {
                if vi != 0.0
                    && !self.inverted_file[word as usize]
                        .iter()
                        .any(|p| p.entry_id == id)
                {
                    missing += vi * (vi.ln() - log_eps());
                }
            }
            *score += missing;
        }
        Self::finish(pairs, max_results, false)
    }
    #[must_use]
    pub fn query_bhattacharyya(
        &self,
        bow: &BowVector,
        max_results: i32,
        max_id: i32,
    ) -> QueryResults {
        let mut pairs = BTreeMap::<EntryId, (f64, usize)>::new();
        self.visit(bow, max_id, |id, q, d| {
            let value = (q * d).sqrt();
            pairs
                .entry(id)
                .and_modify(|p| {
                    p.0 += value;
                    p.1 += 1;
                })
                .or_insert((value, 1));
        });
        let mut out = QueryResults::default();
        for (id, (score, n)) in pairs {
            if n >= MIN_COMMON_WORDS {
                let mut r = QueryResult::new(id, score);
                r.n_words = Some(n as i32);
                r.bhat_score = Some(score);
                out.push(r);
            }
        }
        Self::sort_cut(&mut out, max_results, true);
        out
    }
    #[must_use]
    pub fn query_dot_product(
        &self,
        bow: &BowVector,
        max_results: i32,
        max_id: i32,
    ) -> QueryResults {
        let mut pairs = BTreeMap::new();
        self.visit(bow, max_id, |id, q, d| {
            let value = if self.vocabulary().weighting == WeightingType::Binary {
                1.0
            } else {
                q * d
            };
            pairs
                .entry(id)
                .and_modify(|score| *score += value)
                .or_insert(value);
        });
        Self::finish(pairs, max_results, true)
    }
    /// # Panics
    /// Panics if `id` is outside the database or no direct-index row exists for it.
    #[must_use]
    pub fn retrieve_features(&self, id: EntryId) -> &FeatureVector {
        assert!(id < self.n_entries);
        &self.direct_file[id as usize]
    }
    /// # Errors
    /// Returns an error if serialization, file creation, compression, writing, or flushing
    /// fails.
    pub fn save(&self, path: impl AsRef<Path>) -> StorageResult<()> {
        let mut root = serde_yaml::Mapping::new();
        root.insert(
            "vocabulary".into(),
            serde_yaml::to_value(self.vocabulary().to_storage())?,
        );
        root.insert(
            "database".into(),
            serde_yaml::to_value(DatabaseStorage {
                n_entries: self.n_entries,
                using_di: i32::from(self.use_di),
                di_levels: self.di_levels,
                inverted_index: self.inverted_file.clone(),
                direct_index: self
                    .direct_file
                    .iter()
                    .map(|fv| {
                        fv.iter()
                            .map(|(&node, features)| DirectStorage {
                                node_id: node,
                                features: vec![features.clone()],
                            })
                            .collect()
                    })
                    .collect(),
            })?,
        );
        write_storage(path, &root)
    }
    /// # Errors
    /// Returns an error for unreadable or malformed storage, invalid vocabulary data, or
    /// inconsistent database indices.
    pub fn load(path: impl AsRef<Path>) -> StorageResult<Self> {
        let root = read_storage(path)?;
        let vocabulary =
            TemplatedVocabulary::from_storage(serde_yaml::from_value::<VocabularyStorage>(
                root.get("vocabulary").ok_or("missing vocabulary")?.clone(),
            )?)?;
        let storage: DatabaseStorage =
            serde_yaml::from_value(root.get("database").ok_or("missing database")?.clone())?;
        if storage.inverted_index.len() != vocabulary.size() {
            return Err("inverted index size differs from vocabulary".into());
        }
        for row in &storage.inverted_index {
            if row.iter().any(|p| p.entry_id >= storage.n_entries)
                || row.windows(2).any(|p| p[0].entry_id >= p[1].entry_id)
            {
                return Err("invalid inverted index entry ordering".into());
            }
        }
        let mut db =
            Self::from_loaded_vocabulary(vocabulary, storage.using_di != 0, storage.di_levels);
        db.n_entries = storage.n_entries;
        db.inverted_file = storage.inverted_index;
        if db.use_di {
            if storage.direct_index.len() != db.n_entries as usize {
                return Err("direct index entry count mismatch".into());
            }
            for row in storage.direct_index {
                let mut fv = FeatureVector::new();
                for item in row {
                    if fv
                        .insert(
                            item.node_id,
                            item.features
                                .into_iter()
                                .next()
                                .ok_or("missing feature vector sequence")?,
                        )
                        .is_some()
                    {
                        return Err("duplicate direct index node".into());
                    }
                }
                db.direct_file.push(fv);
            }
        }
        Ok(db)
    }
}
#[derive(Serialize, Deserialize)]
struct DatabaseStorage {
    #[serde(rename = "nEntries")]
    n_entries: EntryId,
    #[serde(rename = "usingDI")]
    using_di: i32,
    #[serde(rename = "diLevels")]
    di_levels: i32,
    #[serde(rename = "invertedIndex")]
    inverted_index: Vec<Vec<IfPair>>,
    #[serde(rename = "directIndex")]
    direct_index: Vec<Vec<DirectStorage>>,
}
#[derive(Serialize, Deserialize)]
struct DirectStorage {
    #[serde(rename = "nodeId")]
    node_id: u32,
    features: Vec<Vec<u32>>,
}
impl<F: DescriptorOps> fmt::Display for TemplatedDatabase<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Database: Entries = {}, Using direct index = {}",
            self.n_entries,
            if self.use_di { "yes" } else { "no" }
        )?;
        if self.use_di {
            write!(f, ", Direct index levels = {}", self.di_levels)?;
        }
        write!(f, ". {}", self.vocabulary())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::f_brief::FBrief;

    fn database(scoring: ScoringType, rows: Vec<Vec<IfPair>>) -> TemplatedDatabase<FBrief> {
        let mut database = TemplatedDatabase::new_empty(false, 0);
        database.vocabulary = Some(TemplatedVocabulary::new(1, 1, WeightingType::Tf, scoring));
        database.inverted_file = rows;
        database.n_entries = 1;
        database
    }

    #[test]
    fn query_preserves_first_score_and_reference_kl_accumulation() {
        let database = database(
            ScoringType::DotProduct,
            vec![vec![IfPair {
                entry_id: 0,
                word_weight: 1.0,
            }]],
        );
        let mut bow = BowVector::new();
        bow.add_weight(0, -0.0);
        assert_eq!(
            database.query_bow(&bow, 0, -1)[0].score.to_bits(),
            (-0.0_f64).to_bits()
        );

        let database = self::database(
            ScoringType::Kl,
            vec![
                vec![IfPair {
                    entry_id: 0,
                    word_weight: 1e-100,
                }],
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
            ],
        );
        let mut bow = BowVector::new();
        bow.add_weight(0, 1e15);
        for word in 1..5 {
            bow.add_weight(word, 1.0);
        }
        let common = 1e15_f64 * (1e15_f64 / 1e-100_f64).ln();
        let mut missing = 0.0;
        let mut interleaved = common;
        for _ in 0..4 {
            missing += -log_eps();
            interleaved += -log_eps();
        }
        let expected = common + missing;
        assert_ne!(expected.to_bits(), interleaved.to_bits());
        assert_eq!(
            database.query_bow(&bow, 0, -1)[0].score.to_bits(),
            expected.to_bits()
        );
    }
}
