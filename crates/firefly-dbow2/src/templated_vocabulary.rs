/*
 * DBoW2 — Copyright (c) 2011–2013 Dorian Galvez-Lopez.
 * License: see LICENSE.txt.
 */
//! Hierarchical k-means vocabulary and `OpenCV` `FileStorage` YAML schema.
//! YAML parsing uses `serde_yaml`; the `OpenCV` `%YAML:1.0` directive is a format envelope.
use super::{
    bow_vector::{BowVector, NodeId, ScoringType, WeightingType, WordId, WordValue},
    feature_vector::FeatureVector,
    scoring_object::GeneralScoring,
};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeSet,
    fmt,
    io::{Read, Write},
    marker::PhantomData,
    path::Path,
};
pub type StorageResult<T> = std::result::Result<T, Box<dyn std::error::Error + Send + Sync>>;
pub trait DescriptorOps: Clone {
    type Descriptor: Clone;
    fn mean_value(descriptors: &[&Self::Descriptor]) -> Self::Descriptor;
    fn distance(a: &Self::Descriptor, b: &Self::Descriptor) -> f64;
    fn to_string(d: &Self::Descriptor) -> String;
    /// # Errors
    /// Returns an error when the input cannot be parsed in this descriptor encoding.
    fn from_string(s: &str) -> StorageResult<Self::Descriptor>;
}
/// Random draws used by the reference k-means++ initialization.
pub trait RandomSource {
    fn unit(&mut self) -> f64;
    fn index(&mut self, n: usize) -> usize {
        (self.unit() * n as f64) as usize
    }
}
#[derive(Debug)]
pub struct SeededRandom(pub rand::rngs::StdRng);
impl SeededRandom {
    #[must_use]
    pub fn new(seed: u64) -> Self {
        use rand::SeedableRng;
        Self(rand::rngs::StdRng::seed_from_u64(seed))
    }
}
impl RandomSource for SeededRandom {
    fn unit(&mut self) -> f64 {
        use rand::RngExt;
        self.0.random::<f64>()
    }
}
#[derive(Clone, Debug)]
pub struct Node<D> {
    pub id: NodeId,
    pub weight: WordValue,
    pub children: Vec<NodeId>,
    pub parent: Option<NodeId>,
    pub descriptor: Option<D>,
    pub word_id: Option<WordId>,
}
impl<D> Node<D> {
    fn new(id: NodeId) -> Self {
        Self {
            id,
            weight: 0.0,
            children: Vec::new(),
            parent: None,
            descriptor: None,
            word_id: None,
        }
    }
    pub fn is_leaf(&self) -> bool {
        self.children.is_empty()
    }
}
pub struct TemplatedVocabulary<F: DescriptorOps> {
    pub k: usize,
    pub levels: usize,
    pub weighting: WeightingType,
    pub scoring: ScoringType,
    pub nodes: Vec<Node<F::Descriptor>>,
    pub words: Vec<NodeId>,
    ops: PhantomData<F>,
}
impl<F: DescriptorOps> Clone for TemplatedVocabulary<F> {
    fn clone(&self) -> Self {
        let mut vocabulary = Self {
            k: self.k,
            levels: self.levels,
            weighting: self.weighting,
            scoring: self.scoring,
            nodes: self.nodes.clone(),
            words: Vec::new(),
            ops: PhantomData,
        };
        vocabulary.create_words();
        vocabulary
    }
}
impl<F: DescriptorOps> Default for TemplatedVocabulary<F> {
    fn default() -> Self {
        Self::new(10, 5, WeightingType::TfIdf, ScoringType::L1Norm)
    }
}
impl<F: DescriptorOps> TemplatedVocabulary<F> {
    #[must_use]
    pub fn new(k: usize, levels: usize, weighting: WeightingType, scoring: ScoringType) -> Self {
        Self {
            k,
            levels,
            weighting,
            scoring,
            nodes: Vec::new(),
            words: Vec::new(),
            ops: PhantomData,
        }
    }
    #[must_use]
    pub fn size(&self) -> usize {
        self.words.len()
    }
    #[must_use]
    pub fn empty(&self) -> bool {
        self.words.is_empty()
    }
    #[must_use]
    pub fn get_branching_factor(&self) -> usize {
        self.k
    }
    #[must_use]
    pub fn get_depth_levels(&self) -> usize {
        self.levels
    }
    #[must_use]
    pub fn get_weighting_type(&self) -> WeightingType {
        self.weighting
    }
    #[must_use]
    pub fn get_scoring_type(&self) -> ScoringType {
        self.scoring
    }
    pub fn set_scoring_type(&mut self, s: ScoringType) {
        self.scoring = s;
    }
    pub fn set_weighting_type(&mut self, w: WeightingType) {
        self.weighting = w;
    }
    /// # Errors
    /// Returns an error if the branching factor or vocabulary depth is zero.
    pub fn create(
        &mut self,
        training: &[Vec<F::Descriptor>],
        random: &mut impl RandomSource,
    ) -> StorageResult<()> {
        if self.k == 0 || self.levels == 0 {
            return Err("vocabulary requires positive k and L".into());
        }
        self.nodes.clear();
        self.words.clear();
        self.nodes.push(Node::new(0));
        let descriptors = training.iter().flat_map(|d| d.iter()).collect::<Vec<_>>();
        self.hkmeans_step(0, &descriptors, 1, random);
        self.create_words();
        self.set_node_weights(training);
        Ok(())
    }
    /// # Errors
    /// Returns an error if `k` or `levels` is zero.
    pub fn create_with_parameters(
        &mut self,
        training: &[Vec<F::Descriptor>],
        k: usize,
        levels: usize,
        weighting: WeightingType,
        scoring: ScoringType,
        random: &mut impl RandomSource,
    ) -> StorageResult<()> {
        self.k = k;
        self.levels = levels;
        self.weighting = weighting;
        self.scoring = scoring;
        self.create(training, random)
    }
    fn hkmeans_step(
        &mut self,
        parent: NodeId,
        descriptors: &[&F::Descriptor],
        level: usize,
        random: &mut impl RandomSource,
    ) {
        if descriptors.is_empty() {
            return;
        }
        let mut clusters;
        let mut groups;
        if descriptors.len() <= self.k {
            clusters = descriptors.iter().map(|d| (*d).clone()).collect::<Vec<_>>();
            groups = (0..descriptors.len()).map(|i| vec![i]).collect::<Vec<_>>();
        } else {
            clusters = self.initiate_clusters_kmpp(descriptors, random);
            groups = vec![Vec::new(); clusters.len()];
            let mut last = None;
            loop {
                if last.is_some() {
                    for (c, g) in clusters.iter_mut().zip(&groups) {
                        let ds = g.iter().map(|&i| descriptors[i]).collect::<Vec<_>>();
                        *c = F::mean_value(&ds);
                    }
                }
                groups.iter_mut().for_each(Vec::clear);
                let mut association = Vec::with_capacity(descriptors.len());
                for (i, d) in descriptors.iter().enumerate() {
                    let mut best = 0;
                    let mut distance = F::distance(d, &clusters[0]);
                    for (c, center) in clusters.iter().enumerate().skip(1) {
                        let dc = F::distance(d, center);
                        if dc < distance {
                            best = c;
                            distance = dc;
                        }
                    }
                    groups[best].push(i);
                    association.push(best);
                }
                if last.as_ref() == Some(&association) {
                    break;
                }
                last = Some(association);
            }
        }
        let mut children = Vec::new();
        for descriptor in clusters {
            let id = self.nodes.len() as NodeId;
            let mut n = Node::new(id);
            n.parent = Some(parent);
            n.descriptor = Some(descriptor);
            self.nodes.push(n);
            self.nodes[parent as usize].children.push(id);
            children.push(id);
        }
        if level < self.levels {
            for (id, group) in children.into_iter().zip(groups) {
                if group.len() > 1 {
                    let ds = group.iter().map(|&i| descriptors[i]).collect::<Vec<_>>();
                    self.hkmeans_step(id, &ds, level + 1, random);
                }
            }
        }
    }
    /// # Panics
    /// Panics if the random source returns an index outside `features` or a descriptor
    /// operation panics on incompatible inputs.
    pub fn initiate_clusters_kmpp(
        &self,
        features: &[&F::Descriptor],
        random: &mut impl RandomSource,
    ) -> Vec<F::Descriptor> {
        if features.is_empty() {
            return Vec::new();
        }
        let mut clusters = vec![features[random.index(features.len())].clone()];
        let mut distances = features
            .iter()
            .map(|d| F::distance(d, &clusters[0]))
            .collect::<Vec<_>>();
        while clusters.len() < self.k {
            for (d, dist) in features.iter().zip(&mut distances) {
                if *dist > 0.0 {
                    *dist = dist.min(F::distance(d, clusters.last().unwrap()));
                }
            }
            let sum = distances.iter().sum::<f64>();
            if sum <= 0.0 || sum.is_nan() {
                break;
            }
            let mut cut = 0.0;
            while cut == 0.0 {
                cut = random.unit() * sum;
            }
            let mut accum = 0.0;
            let mut selected = features.len() - 1;
            for (i, &dist) in distances.iter().enumerate() {
                accum += dist;
                if accum >= cut {
                    selected = i;
                    break;
                }
            }
            clusters.push(features[selected].clone());
        }
        clusters
    }
    fn create_words(&mut self) {
        self.words.clear();
        for n in self.nodes.iter_mut().skip(1) {
            if n.is_leaf() {
                n.word_id = Some(self.words.len() as WordId);
                self.words.push(n.id);
            }
        }
    }
    fn set_node_weights(&mut self, training: &[Vec<F::Descriptor>]) {
        match self.weighting {
            WeightingType::Tf | WeightingType::Binary => {
                for &n in &self.words {
                    self.nodes[n as usize].weight = 1.0;
                }
            }
            _ => {
                let mut counts = vec![0usize; self.words.len()];
                for doc in training {
                    let mut counted = BTreeSet::new();
                    for feature in doc {
                        if let Some((w, _, _)) = self.transform_feature(feature, 0) {
                            counted.insert(w);
                        }
                    }
                    for w in counted {
                        counts[w as usize] += 1;
                    }
                }
                for (i, &n) in self.words.iter().enumerate() {
                    if counts[i] > 0 {
                        self.nodes[n as usize].weight =
                            (training.len() as f64 / counts[i] as f64).ln();
                    }
                }
            }
        }
    }
    /// # Panics
    /// Panics if the public node/word arrays contain invalid indices or a non-root ancestor has
    /// no parent.
    #[must_use]
    pub fn get_effective_levels(&self) -> f32 {
        let mut sum = 0_u64;
        for &id in &self.words {
            let mut n = id;
            while n != 0 {
                sum += 1;
                n = self.nodes[n as usize].parent.unwrap();
            }
        }
        (sum as f64 / self.words.len() as f64) as f32
    }
    /// # Panics
    /// Panics if `id` or its node index is invalid, or the word has no descriptor.
    #[must_use]
    pub fn get_word(&self, id: WordId) -> &F::Descriptor {
        self.nodes[self.words[id as usize] as usize]
            .descriptor
            .as_ref()
            .unwrap()
    }
    #[must_use]
    pub fn get_word_weight(&self, id: WordId) -> WordValue {
        self.nodes[self.words[id as usize] as usize].weight
    }
    /// `node` is absent when a shallow leaf never reaches L-levelsup, matching the unwritten C++ output.
    pub fn transform_feature(
        &self,
        feature: &F::Descriptor,
        levelsup: i32,
    ) -> Option<(WordId, WordValue, Option<NodeId>)> {
        if self.empty() {
            return None;
        }
        let target = self.levels as i32 - levelsup;
        let mut node = if target <= 0 { Some(0) } else { None };
        let mut current = 0;
        let mut level = 0;
        loop {
            level += 1;
            let children = &self.nodes[current].children;
            let mut final_id = *children.first()?;
            let mut best = F::distance(feature, self.nodes[final_id as usize].descriptor.as_ref()?);
            for &n in children.iter().skip(1) {
                let d = F::distance(feature, self.nodes[n as usize].descriptor.as_ref()?);
                if d < best {
                    best = d;
                    final_id = n;
                }
            }
            current = final_id as usize;
            if level == target {
                node = Some(final_id);
            }
            if self.nodes[current].is_leaf() {
                let n = &self.nodes[current];
                return Some((n.word_id?, n.weight, node));
            }
        }
    }
    pub fn transform_word(&self, feature: &F::Descriptor) -> WordId {
        self.transform_feature(feature, 0).map_or(0, |x| x.0)
    }
    pub fn transform(&self, features: &[F::Descriptor]) -> BowVector {
        self.transform_with_features(features, 0).0
    }
    pub fn transform_with_features(
        &self,
        features: &[F::Descriptor],
        levelsup: i32,
    ) -> (BowVector, FeatureVector) {
        let mut bow = BowVector::new();
        let mut fv = FeatureVector::new();
        for (i, d) in features.iter().enumerate() {
            if let Some((word, weight, node)) = self.transform_feature(d, levelsup)
                && weight > 0.0
            {
                match self.weighting {
                    WeightingType::Tf | WeightingType::TfIdf => bow.add_weight(word, weight),
                    _ => bow.add_if_not_exist(word, weight),
                }
                if let Some(n) = node {
                    fv.add_feature(n, i as u32);
                }
            }
        }
        if let Some(norm) = self.scoring.must_normalize() {
            bow.normalize(norm);
        } else if !bow.is_empty()
            && matches!(self.weighting, WeightingType::Tf | WeightingType::TfIdf)
        {
            let n = bow.len() as f64;
            for w in bow.values_mut() {
                *w /= n;
            }
        }
        (bow, fv)
    }
    #[must_use]
    pub fn score(&self, v: &BowVector, w: &BowVector) -> f64 {
        self.scoring.score(v, w)
    }
    /// # Panics
    /// Panics if the word/ancestor indices are invalid or a non-root ancestor has no parent.
    #[must_use]
    pub fn get_parent_node(&self, word: WordId, mut levelsup: usize) -> NodeId {
        let mut n = self.words[word as usize];
        while levelsup > 0 && n != 0 {
            n = self.nodes[n as usize].parent.unwrap();
            levelsup -= 1;
        }
        n
    }
    #[must_use]
    pub fn get_words_from_node(&self, id: NodeId) -> Vec<WordId> {
        let mut words = Vec::new();
        if self.nodes[id as usize].is_leaf() {
            if let Some(w) = self.nodes[id as usize].word_id {
                words.push(w);
            }
            return words;
        }
        let mut parents = vec![id];
        while let Some(p) = parents.pop() {
            for &id in &self.nodes[p as usize].children {
                let child = &self.nodes[id as usize];
                if child.is_leaf() {
                    if let Some(w) = child.word_id {
                        words.push(w);
                    }
                } else {
                    parents.push(id);
                }
            }
        }
        words
    }
    pub fn stop_words(&mut self, min_weight: f64) -> usize {
        let mut count = 0;
        for &n in &self.words {
            if self.nodes[n as usize].weight < min_weight {
                self.nodes[n as usize].weight = 0.0;
                count += 1;
            }
        }
        count
    }
    /// # Panics
    /// Panics if the public tree contains an invalid child index or a non-root node lacks a
    /// descriptor.
    #[must_use]
    pub fn to_storage(&self) -> VocabularyStorage {
        let mut nodes = Vec::new();
        let mut parents = if self.nodes.is_empty() {
            Vec::new()
        } else {
            vec![0]
        };
        while let Some(parent) = parents.pop() {
            for &id in &self.nodes[parent].children {
                let n = &self.nodes[id as usize];
                nodes.push(NodeStorage {
                    node_id: id,
                    parent_id: parent as u32,
                    weight: n.weight,
                    descriptor: F::to_string(n.descriptor.as_ref().unwrap()),
                });
                if !n.is_leaf() {
                    parents.push(id as usize);
                }
            }
        }
        VocabularyStorage {
            k: self.k,
            l: self.levels,
            scoring_type: self.scoring as i32,
            weighting_type: self.weighting as i32,
            nodes,
            words: self
                .words
                .iter()
                .enumerate()
                .map(|(word, &node)| WordStorage {
                    word_id: word as u32,
                    node_id: node,
                })
                .collect(),
        }
    }
    /// # Errors
    /// Returns an error for invalid scoring/weighting values, node or word IDs,
    /// disconnected/cyclic trees, or malformed descriptors.
    pub fn from_storage(s: VocabularyStorage) -> StorageResult<Self> {
        let weighting = match s.weighting_type {
            0 => WeightingType::TfIdf,
            1 => WeightingType::Tf,
            2 => WeightingType::Idf,
            3 => WeightingType::Binary,
            _ => return Err("invalid weightingType".into()),
        };
        let scoring = match s.scoring_type {
            0 => ScoringType::L1Norm,
            1 => ScoringType::L2Norm,
            2 => ScoringType::ChiSquare,
            3 => ScoringType::Kl,
            4 => ScoringType::Bhattacharyya,
            5 => ScoringType::DotProduct,
            _ => return Err("invalid scoringType".into()),
        };
        let mut v = Self::new(s.k, s.l, weighting, scoring);
        v.nodes = (0..=s.nodes.len()).map(|id| Node::new(id as u32)).collect();
        let mut seen = BTreeSet::new();
        for n in s.nodes {
            if n.node_id == 0
                || n.node_id as usize >= v.nodes.len()
                || n.parent_id as usize >= v.nodes.len()
                || n.node_id == n.parent_id
                || !seen.insert(n.node_id)
            {
                return Err("invalid vocabulary node ids".into());
            }
            v.nodes[n.node_id as usize].parent = Some(n.parent_id);
            v.nodes[n.node_id as usize].weight = n.weight;
            v.nodes[n.node_id as usize].descriptor = Some(F::from_string(&n.descriptor)?);
            v.nodes[n.parent_id as usize].children.push(n.node_id);
        }
        v.words = vec![0; s.words.len()];
        let mut seen = BTreeSet::new();
        for w in s.words {
            if w.word_id as usize >= v.words.len()
                || w.node_id == 0
                || w.node_id as usize >= v.nodes.len()
                || !seen.insert(w.word_id)
                || !v.nodes[w.node_id as usize].is_leaf()
            {
                return Err("invalid vocabulary words".into());
            }
            v.words[w.word_id as usize] = w.node_id;
            v.nodes[w.node_id as usize].word_id = Some(w.word_id);
        }
        for n in &v.nodes {
            let mut id = n.id;
            let mut steps = 0;
            while id != 0 {
                steps += 1;
                if steps > v.nodes.len() {
                    return Err("cyclic vocabulary tree".into());
                }
                id = v.nodes[id as usize]
                    .parent
                    .ok_or("disconnected vocabulary node")?;
            }
        }
        Ok(v)
    }
    /// # Errors
    /// Returns an error if serialization, file creation, compression, writing, or flushing
    /// fails.
    pub fn save(&self, path: impl AsRef<Path>) -> StorageResult<()> {
        let mut root = serde_yaml::Mapping::new();
        root.insert(
            "vocabulary".into(),
            serde_yaml::to_value(self.to_storage())?,
        );
        write_storage(path, &root)
    }
    /// # Errors
    /// Returns an error for unreadable or malformed storage, missing vocabulary data, or an
    /// invalid vocabulary.
    pub fn load(path: impl AsRef<Path>) -> StorageResult<Self> {
        let root = read_storage(path)?;
        let storage =
            serde_yaml::from_value(root.get("vocabulary").ok_or("missing vocabulary")?.clone())?;
        Self::from_storage(storage)
    }
    /// Loads a plain YAML or gzip-compressed vocabulary from an owned/embedded resource.
    /// # Errors
    /// Returns decompression, YAML schema or vocabulary validation errors.
    pub fn load_bytes(bytes: &[u8]) -> StorageResult<Self> {
        let mut input = String::new();
        if bytes.starts_with(&[0x1f, 0x8b]) {
            flate2::read::GzDecoder::new(bytes).read_to_string(&mut input)?;
        } else {
            std::io::Cursor::new(bytes).read_to_string(&mut input)?;
        }
        let root = parse_storage(&input)?;
        let storage =
            serde_yaml::from_value(root.get("vocabulary").ok_or("missing vocabulary")?.clone())?;
        Self::from_storage(storage)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VocabularyStorage {
    pub k: usize,
    #[serde(rename = "L")]
    pub l: usize,
    #[serde(rename = "scoringType")]
    pub scoring_type: i32,
    #[serde(rename = "weightingType")]
    pub weighting_type: i32,
    pub nodes: Vec<NodeStorage>,
    pub words: Vec<WordStorage>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NodeStorage {
    #[serde(rename = "nodeId")]
    pub node_id: u32,
    #[serde(rename = "parentId")]
    pub parent_id: u32,
    pub weight: f64,
    pub descriptor: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WordStorage {
    #[serde(rename = "wordId")]
    pub word_id: u32,
    #[serde(rename = "nodeId")]
    pub node_id: u32,
}
/// # Errors
/// Returns an error if reading/decompression fails, YAML is invalid, or the document root is
/// not a mapping.
pub fn read_storage(path: impl AsRef<Path>) -> StorageResult<serde_yaml::Mapping> {
    let path = path.as_ref();
    let file = std::fs::File::open(path)?;
    let mut input = String::new();
    if path.extension().is_some_and(|e| e == "gz") {
        flate2::read::GzDecoder::new(file).read_to_string(&mut input)?;
    } else {
        std::io::BufReader::new(file).read_to_string(&mut input)?;
    }
    parse_storage(&input)
}
fn parse_storage(input: &str) -> StorageResult<serde_yaml::Mapping> {
    let yaml = input
        .strip_prefix("%YAML:1.0")
        .map_or(input, |s| s.trim_start_matches(['\r', '\n']));
    Ok(serde_yaml::from_str(&normalize_opencv_flow_keys(yaml))?)
}
/// `OpenCV` emits flow-map separators without YAML's required following space.
/// Normalize only separators, leaving quoted descriptors and scalar contents intact.
fn normalize_opencv_flow_keys(yaml: &str) -> String {
    let mut out = String::with_capacity(yaml.len());
    let mut containers: Vec<Option<bool>> = Vec::new();
    let mut quote = None;
    let mut escaped = false;
    let mut chars = yaml.chars().peekable();
    while let Some(c) = chars.next() {
        out.push(c);
        if let Some(q) = quote {
            if q == '"' && escaped {
                escaped = false;
                continue;
            }
            if q == '"' && c == '\\' {
                escaped = true;
                continue;
            }
            if c == q {
                if q == '\'' && chars.peek() == Some(&'\'') {
                    out.push(chars.next().unwrap());
                } else {
                    quote = None;
                }
            }
            continue;
        }
        match c {
            '\'' | '"' => quote = Some(c),
            '{' => containers.push(Some(true)),
            '[' => containers.push(None),
            '}' | ']' => {
                containers.pop();
            }
            ',' => {
                if let Some(Some(key)) = containers.last_mut() {
                    *key = true;
                }
            }
            ':' => {
                if let Some(Some(key)) = containers.last_mut()
                    && *key
                {
                    *key = false;
                    if chars.peek().is_some_and(|c| !c.is_whitespace()) {
                        out.push(' ');
                    }
                }
            }
            _ => {}
        }
    }
    out
}

/// `OpenCV`'s reader accepts flow containers but rejects indentless block sequences.
/// Scalar strings use serde's JSON escaping, which is also valid in YAML flow style.
fn write_opencv_flow(
    value: &serde_yaml::Value,
    out: &mut String,
    depth: usize,
) -> StorageResult<()> {
    use serde_yaml::Value;
    match value {
        Value::Mapping(map) => {
            out.push('{');
            for (index, (key, value)) in map.iter().enumerate() {
                if index != 0 {
                    out.push(',');
                }
                out.push('\n');
                out.push_str(&"  ".repeat(depth + 1));
                write_opencv_key(key, out)?;
                out.push_str(": ");
                write_opencv_flow(value, out, depth + 1)?;
            }
            if !map.is_empty() {
                out.push('\n');
                out.push_str(&"  ".repeat(depth));
            }
            out.push('}');
        }
        Value::Sequence(values) => {
            out.push('[');
            for (index, value) in values.iter().enumerate() {
                if index != 0 {
                    out.push(',');
                }
                out.push('\n');
                out.push_str(&"  ".repeat(depth + 1));
                write_opencv_flow(value, out, depth + 1)?;
            }
            if !values.is_empty() {
                out.push('\n');
                out.push_str(&"  ".repeat(depth));
            }
            out.push(']');
        }
        Value::String(value) => out.push_str(&serde_json::to_string(value)?),
        Value::Number(value) => match value.as_f64() {
            Some(v) if v.is_nan() => out.push_str(".Nan"),
            Some(v) if v == f64::INFINITY => out.push_str(".Inf"),
            Some(v) if v == f64::NEG_INFINITY => out.push_str("-.Inf"),
            _ => out.push_str(&value.to_string()),
        },
        Value::Bool(value) => out.push_str(if *value { "true" } else { "false" }),
        Value::Null => out.push('~'),
        Value::Tagged(_) => {
            return Err("tagged YAML values are not part of the DBoW2 storage schema".into());
        }
    }
    Ok(())
}

fn write_opencv_key(key: &serde_yaml::Value, out: &mut String) -> StorageResult<()> {
    let key = key.as_str().ok_or("OpenCV storage keys must be strings")?;
    if key.is_empty() || !key.bytes().all(|c| c.is_ascii_alphanumeric() || c == b'_') {
        return Err("invalid OpenCV storage key".into());
    }
    out.push_str(key);
    Ok(())
}

/// # Errors
/// Returns an error for unsupported storage values or keys, or if file creation, compression,
/// writing, or flushing fails.
pub fn write_storage(path: impl AsRef<Path>, value: &serde_yaml::Mapping) -> StorageResult<()> {
    let path = path.as_ref();
    let mut yaml = String::from("%YAML:1.0\n---\n");
    for (key, value) in value {
        write_opencv_key(key, &mut yaml)?;
        yaml.push_str(": ");
        write_opencv_flow(value, &mut yaml, 1)?;
        yaml.push('\n');
    }
    let file = std::fs::File::create(path)?;
    if path.extension().is_some_and(|e| e == "gz") {
        let mut encoder = flate2::write::GzEncoder::new(file, flate2::Compression::default());
        encoder.write_all(yaml.as_bytes())?;
        encoder.finish()?;
    } else {
        let mut writer = std::io::BufWriter::new(file);
        writer.write_all(yaml.as_bytes())?;
        writer.flush()?;
    }
    Ok(())
}
impl<F: DescriptorOps> fmt::Display for TemplatedVocabulary<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Vocabulary: k = {}, L = {}, Weighting = {:?}, Scoring = {:?}, Number of words = {}",
            self.k,
            self.levels,
            self.weighting,
            self.scoring,
            self.size()
        )
    }
}
