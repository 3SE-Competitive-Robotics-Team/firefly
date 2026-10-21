/*
 * DBoW2 — Copyright (c) 2011–2013 Dorian Galvez-Lopez.
 * License: see LICENSE.txt.
 */
//! Sparse, ordered bag of visual words.
use std::{
    collections::BTreeMap,
    fmt,
    io::{self, Write},
    ops::{Deref, DerefMut},
    path::Path,
};
pub type WordId = u32;
pub type NodeId = u32;
pub type WordValue = f64;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LNorm {
    L1,
    L2,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WeightingType {
    TfIdf,
    Tf,
    Idf,
    Binary,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScoringType {
    L1Norm,
    L2Norm,
    ChiSquare,
    Kl,
    Bhattacharyya,
    DotProduct,
}
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BowVector(pub BTreeMap<WordId, WordValue>);
impl Deref for BowVector {
    type Target = BTreeMap<WordId, WordValue>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for BowVector {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl BowVector {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
    pub fn add_weight(&mut self, id: WordId, v: WordValue) {
        self.entry(id).and_modify(|value| *value += v).or_insert(v);
    }
    pub fn add_if_not_exist(&mut self, id: WordId, v: WordValue) {
        self.entry(id).or_insert(v);
    }
    pub fn normalize(&mut self, norm: LNorm) {
        let n = match norm {
            LNorm::L1 => self.values().map(|v| v.abs()).sum(),
            LNorm::L2 => self.values().map(|v| v * v).sum::<f64>().sqrt(),
        };
        if n > 0.0 {
            for v in self.values_mut() {
                *v /= n;
            }
        }
    }
    /// # Errors
    /// Returns an error if the file cannot be created, written, or flushed.
    pub fn save_m(&self, path: impl AsRef<Path>, words: usize) -> io::Result<()> {
        let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
        let mut last = 0;
        for (&id, &v) in &self.0 {
            for _ in last..id {
                write!(f, "0 ")?;
            }
            write!(f, "{v} ")?;
            last = id + 1;
        }
        for _ in last as usize..words {
            write!(f, "0 ")?;
        }
        f.flush()
    }
}
impl fmt::Display for BowVector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, (id, v)) in self.iter().enumerate() {
            if i > 0 {
                write!(f, ", ")?;
            }
            write!(f, "<{id}, {v}>")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::BowVector;

    #[test]
    fn inserting_a_word_preserves_its_value() {
        let mut vector = BowVector::new();
        vector.add_weight(4, -0.0);
        assert_eq!(vector[&4].to_bits(), (-0.0_f64).to_bits());
        vector.add_weight(4, 1.5);
        vector.add_weight(4, 2.5);
        assert_eq!(vector[&4], 4.0);
    }
}
