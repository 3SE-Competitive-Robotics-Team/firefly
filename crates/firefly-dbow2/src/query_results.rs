/*
 * DBoW2 — Copyright (c) 2011–2013 Dorian Galvez-Lopez.
 * License: see LICENSE.txt.
 */
//! Database query scores and optional scoring diagnostics.
use std::{
    fmt,
    io::{self, Write},
    ops::{Deref, DerefMut},
    path::Path,
};
pub type EntryId = u32;
#[derive(Clone, Debug, PartialEq)]
pub struct Result {
    pub id: EntryId,
    pub score: f64,
    pub n_words: Option<i32>,
    pub bhat_score: Option<f64>,
    pub chi_score: Option<f64>,
    pub sum_common_vi: Option<f64>,
    pub sum_common_wi: Option<f64>,
    pub expected_chi_score: Option<f64>,
}
impl Result {
    #[must_use]
    pub fn new(id: EntryId, score: f64) -> Self {
        Self {
            id,
            score,
            n_words: None,
            bhat_score: None,
            chi_score: None,
            sum_common_vi: None,
            sum_common_wi: None,
            expected_chi_score: None,
        }
    }
    #[must_use]
    pub fn gt(a: &Self, b: &Self) -> bool {
        a.score > b.score
    }
    #[must_use]
    pub fn ge(a: &Self, b: &Self) -> bool {
        a.score > b.score
    }
    #[must_use]
    pub fn geq(a: &Self, b: &Self) -> bool {
        a.score >= b.score
    }
    #[must_use]
    pub fn geqv(a: &Self, s: f64) -> bool {
        a.score >= s
    }
    #[must_use]
    pub fn lt_id(a: &Self, b: &Self) -> bool {
        a.id < b.id
    }
}
impl PartialEq<EntryId> for Result {
    fn eq(&self, id: &EntryId) -> bool {
        self.id == *id
    }
}
impl PartialEq<f64> for Result {
    fn eq(&self, value: &f64) -> bool {
        self.score == *value
    }
}
impl PartialOrd<f64> for Result {
    fn partial_cmp(&self, value: &f64) -> Option<std::cmp::Ordering> {
        self.score.partial_cmp(value)
    }
}
impl PartialOrd for Result {
    fn partial_cmp(&self, b: &Self) -> Option<std::cmp::Ordering> {
        self.score.partial_cmp(&b.score)
    }
}
impl fmt::Display for Result {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "<EntryId: {}, Score: {}>", self.id, self.score)
    }
}
#[derive(Clone, Debug, Default, PartialEq)]
pub struct QueryResults(pub Vec<Result>);
impl Deref for QueryResults {
    type Target = Vec<Result>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for QueryResults {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl QueryResults {
    pub fn scale_scores(&mut self, factor: f64) {
        for r in &mut self.0 {
            r.score *= factor;
        }
    }
    /// # Errors
    /// Returns an error if the file cannot be created, written, or flushed.
    pub fn save_m(&self, path: impl AsRef<Path>) -> io::Result<()> {
        let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
        for r in &self.0 {
            writeln!(f, "{} {}", r.id, r.score)?;
        }
        f.flush()
    }
}
impl fmt::Display for QueryResults {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "{} {}:",
            self.len(),
            if self.len() == 1 { "result" } else { "results" }
        )?;
        for (i, r) in self.iter().enumerate() {
            if i > 0 {
                writeln!(f)?;
            }
            write!(f, "{r}")?;
        }
        Ok(())
    }
}
