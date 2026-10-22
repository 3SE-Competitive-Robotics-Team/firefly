/*
 * DBoW2 — Copyright (c) 2011–2013 Dorian Galvez-Lopez.
 * License: see LICENSE.txt.
 */
//! Direct-index node-to-local-feature associations.
use super::bow_vector::NodeId;
use std::{
    collections::BTreeMap,
    fmt,
    ops::{Deref, DerefMut},
};
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FeatureVector(pub BTreeMap<NodeId, Vec<u32>>);
impl Deref for FeatureVector {
    type Target = BTreeMap<NodeId, Vec<u32>>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for FeatureVector {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl FeatureVector {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
    pub fn add_feature(&mut self, id: NodeId, index: u32) {
        self.entry(id).or_default().push(index);
    }
}
impl fmt::Display for FeatureVector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, (id, v)) in self.iter().enumerate() {
            if i > 0 {
                write!(f, ", ")?;
            }
            write!(f, "<{id}: [")?;
            for (j, x) in v.iter().enumerate() {
                if j > 0 {
                    write!(f, ", ")?;
                }
                write!(f, "{x}")?;
            }
            write!(f, "]>")?;
        }
        Ok(())
    }
}
