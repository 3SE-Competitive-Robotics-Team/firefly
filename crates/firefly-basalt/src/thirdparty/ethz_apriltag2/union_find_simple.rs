// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See LICENSE for the complete notice and advertising acknowledgement.
//! Original UnionFindSimple.h/UnionFindSimple.cc.
pub struct UnionFindSimple {
    pub parent: Vec<usize>,
    pub sizes: Vec<usize>,
}
impl UnionFindSimple {
    #[must_use]
    pub fn new(count: usize) -> Self {
        Self {
            parent: (0..count).collect(),
            sizes: vec![1; count],
        }
    }
    pub fn representative(&mut self, id: usize) -> usize {
        if self.parent[id] == id {
            return id;
        }
        let root = self.representative(self.parent[id]);
        self.parent[id] = root;
        root
    }
    pub fn connect(&mut self, a: usize, b: usize) -> usize {
        let a = self.representative(a);
        let b = self.representative(b);
        if a == b {
            return a;
        }
        if self.sizes[a] > self.sizes[b] {
            self.parent[b] = a;
            self.sizes[a] += self.sizes[b];
            a
        } else {
            self.parent[a] = b;
            self.sizes[b] += self.sizes[a];
            b
        }
    }
}
