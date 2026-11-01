// Copyright (C) 2010 Gael Guennebaud <gael.guennebaud@inria.fr>.
// Derived from CSparse, Copyright (c) 2006 Timothy A. Davis.
// SPDX-License-Identifier: MPL-2.0
// Eigen's CSparse derivative is distributed under MPL-2.0 by permission.
//! Translation of Eigen 5.0.1 `OrderingMethods/Amd.h` quotient-graph AMD.
use std::ops::{Index, IndexMut};

// Signed indexing retains the upstream -1/-2 sentinels while bounds-checking
// every actual access in safe Rust.
struct Indices(Vec<i64>);
impl Indices {
    fn filled(size: usize, value: i64) -> Self {
        Self(vec![value; size])
    }
}
impl Index<i64> for Indices {
    type Output = i64;
    fn index(&self, index: i64) -> &i64 {
        &self.0[usize::try_from(index).expect("AMD sentinel used as index")]
    }
}
impl IndexMut<i64> for Indices {
    fn index_mut(&mut self, index: i64) -> &mut i64 {
        &mut self.0[usize::try_from(index).expect("AMD sentinel used as index")]
    }
}
fn flip(value: i64) -> i64 {
    -value - 2
}
fn clear(mut mark: i64, maximum: i64, weights: &mut Indices, size: i64) -> i64 {
    if mark < 2 || mark.checked_add(maximum).is_none() {
        for i in 0..size {
            if weights[i] != 0 {
                weights[i] = 1;
            }
        }
        mark = 2;
    }
    mark
}

/// Compute the permutation of a complete symmetric compressed-column pattern.
/// Structural diagonal entries must be retained, including numeric zeros.
/// # Panics
/// Requires valid sorted CSC indices and a dimension representable by i32.
#[must_use]
#[expect(
    clippy::many_single_char_names,
    clippy::similar_names,
    clippy::too_many_lines,
    reason = "Preserves the upstream AMD quotient-graph state machine"
)]
pub fn minimum_degree_ordering(outer: &[usize], inner: &[usize]) -> Vec<usize> {
    assert_ne!(outer, [] as [usize; 0]);
    let size = outer.len() - 1;
    assert!(i32::try_from(size).is_ok());
    let n = i64::try_from(size).unwrap();
    let mut cnz = i64::try_from(inner.len()).unwrap();
    let capacity = cnz + cnz / 5 + 2 * n;
    let mut cp = Indices(outer.iter().map(|&v| i64::try_from(v).unwrap()).collect());
    let mut ci = Indices(
        inner
            .iter()
            .map(|&v| {
                assert!(v < size);
                i64::try_from(v).unwrap()
            })
            .collect(),
    );
    ci.0.resize(usize::try_from(capacity).unwrap(), 0);
    let dense = (16_i64.max((10. * (n as f64).sqrt()) as i64)).min(n - 2);
    let mut len = Indices::filled(size + 1, 0);
    let mut nv = Indices::filled(size + 1, 1);
    let mut next = Indices::filled(size + 1, -1);
    let mut head = Indices::filled(size + 1, -1);
    let mut elen = Indices::filled(size + 1, 0);
    let mut degree = Indices::filled(size + 1, 0);
    let mut w = Indices::filled(size + 1, 1);
    let mut hhead = Indices::filled(size + 1, -1);
    let mut last = Indices::filled(size + 1, -1);
    for k in 0..n {
        len[k] = cp[k + 1] - cp[k];
        degree[k] = len[k];
    }
    let mut mark = clear(0, 0, &mut w, n);
    let mut nel = 0;
    let mut mindeg = 0;
    let mut lemax = 0;
    for i in 0..n {
        let has_diagonal = (cp[i]..cp[i + 1]).any(|p| ci[p] == i);
        let d = degree[i];
        if d == 1 && has_diagonal {
            elen[i] = -2;
            nel += 1;
            cp[i] = -1;
            w[i] = 0;
        } else if d > dense || !has_diagonal {
            nv[i] = 0;
            elen[i] = -1;
            nel += 1;
            cp[i] = flip(n);
            nv[n] += 1;
        } else {
            if head[d] != -1 {
                last[head[d]] = i;
            }
            next[i] = head[d];
            head[d] = i;
        }
    }
    elen[n] = -2;
    cp[n] = -1;
    w[n] = 0;
    while nel < n {
        while mindeg < n && head[mindeg] == -1 {
            mindeg += 1;
        }
        let k = head[mindeg];
        if next[k] != -1 {
            last[next[k]] = -1;
        }
        head[mindeg] = next[k];
        let elenk = elen[k];
        let mut nvk = nv[k];
        nel += nvk;
        if elenk > 0 && cnz + mindeg >= capacity {
            for j in 0..n {
                let p = cp[j];
                if p >= 0 {
                    cp[j] = ci[p];
                    ci[p] = flip(j);
                }
            }
            let mut q = 0;
            let mut p = 0;
            while p < cnz {
                let j = flip(ci[p]);
                p += 1;
                if j >= 0 {
                    ci[q] = cp[j];
                    cp[j] = q;
                    q += 1;
                    for _ in 0..len[j] - 1 {
                        ci[q] = ci[p];
                        q += 1;
                        p += 1;
                    }
                }
            }
            cnz = q;
        }
        let mut dk = 0;
        nv[k] = -nvk;
        let mut p = cp[k];
        let pk1 = if elenk == 0 { p } else { cnz };
        let mut pk2 = pk1;
        for k1 in 1..=elenk + 1 {
            let (e, mut pj, ln) = if k1 > elenk {
                (k, p, len[k] - elenk)
            } else {
                let e = ci[p];
                p += 1;
                (e, cp[e], len[e])
            };
            for _ in 1..=ln {
                let i = ci[pj];
                pj += 1;
                let nvi = nv[i];
                if nvi <= 0 {
                    continue;
                }
                dk += nvi;
                nv[i] = -nvi;
                ci[pk2] = i;
                pk2 += 1;
                if next[i] != -1 {
                    last[next[i]] = last[i];
                }
                if last[i] == -1 {
                    head[degree[i]] = next[i];
                } else {
                    next[last[i]] = next[i];
                }
            }
            if e != k {
                cp[e] = flip(k);
                w[e] = 0;
            }
        }
        if elenk != 0 {
            cnz = pk2;
        }
        degree[k] = dk;
        cp[k] = pk1;
        len[k] = pk2 - pk1;
        elen[k] = -2;
        mark = clear(mark, lemax, &mut w, n);
        for pk in pk1..pk2 {
            let i = ci[pk];
            let eln = elen[i];
            if eln <= 0 {
                continue;
            }
            let nvi = -nv[i];
            let wnvi = mark - nvi;
            for p in cp[i]..cp[i] + eln {
                let e = ci[p];
                if w[e] >= mark {
                    w[e] -= nvi;
                } else if w[e] != 0 {
                    w[e] = degree[e] + wnvi;
                }
            }
        }
        for pk in pk1..pk2 {
            let i = ci[pk];
            let p1 = cp[i];
            let p2 = p1 + elen[i] - 1;
            let mut pn = p1;
            let mut h = 0;
            let mut d = 0;
            for p in p1..=p2 {
                let e = ci[p];
                if w[e] != 0 {
                    let dext = w[e] - mark;
                    if dext > 0 {
                        d += dext;
                        ci[pn] = e;
                        pn += 1;
                        h += e;
                    } else {
                        cp[e] = flip(k);
                        w[e] = 0;
                    }
                }
            }
            elen[i] = pn - p1 + 1;
            let p3 = pn;
            let p4 = p1 + len[i];
            for p in p2 + 1..p4 {
                let j = ci[p];
                let nvj = nv[j];
                if nvj <= 0 {
                    continue;
                }
                d += nvj;
                ci[pn] = j;
                pn += 1;
                h += j;
            }
            if d == 0 {
                cp[i] = flip(k);
                let nvi = -nv[i];
                dk -= nvi;
                nvk += nvi;
                nel += nvi;
                nv[i] = 0;
                elen[i] = -1;
            } else {
                degree[i] = degree[i].min(d);
                ci[pn] = ci[p3];
                ci[p3] = ci[p1];
                ci[p1] = k;
                len[i] = pn - p1 + 1;
                h %= n;
                next[i] = hhead[h];
                hhead[h] = i;
                last[i] = h;
            }
        }
        degree[k] = dk;
        lemax = lemax.max(dk);
        mark = clear(mark + lemax, lemax, &mut w, n);
        for pk in pk1..pk2 {
            let mut i = ci[pk];
            if nv[i] >= 0 {
                continue;
            }
            let h = last[i];
            i = hhead[h];
            hhead[h] = -1;
            while i != -1 && next[i] != -1 {
                let ln = len[i];
                let eln = elen[i];
                for p in cp[i] + 1..cp[i] + ln {
                    w[ci[p]] = mark;
                }
                let mut jlast = i;
                let mut j = next[i];
                while j != -1 {
                    let mut equal = len[j] == ln && elen[j] == eln;
                    let mut p = cp[j] + 1;
                    while equal && p < cp[j] + ln {
                        if w[ci[p]] != mark {
                            equal = false;
                        }
                        p += 1;
                    }
                    if equal {
                        cp[j] = flip(i);
                        nv[i] += nv[j];
                        nv[j] = 0;
                        elen[j] = -1;
                        j = next[j];
                        next[jlast] = j;
                    } else {
                        jlast = j;
                        j = next[j];
                    }
                }
                i = next[i];
                mark += 1;
            }
        }
        let mut p = pk1;
        for pk in pk1..pk2 {
            let i = ci[pk];
            let nvi = -nv[i];
            if nvi <= 0 {
                continue;
            }
            nv[i] = nvi;
            let d = (degree[i] + dk - nvi).min(n - nel - nvi);
            if head[d] != -1 {
                last[head[d]] = i;
            }
            next[i] = head[d];
            last[i] = -1;
            head[d] = i;
            mindeg = mindeg.min(d);
            degree[i] = d;
            ci[p] = i;
            p += 1;
        }
        nv[k] = nvk;
        len[k] = p - pk1;
        if len[k] == 0 {
            cp[k] = -1;
            w[k] = 0;
        }
        if elenk != 0 {
            cnz = p;
        }
    }
    for i in 0..n {
        cp[i] = flip(cp[i]);
    }
    for j in 0..=n {
        head[j] = -1;
    }
    for j in (0..=n).rev() {
        if nv[j] > 0 {
            continue;
        }
        next[j] = head[cp[j]];
        head[cp[j]] = j;
    }
    for e in (0..=n).rev() {
        if nv[e] <= 0 {
            continue;
        }
        if cp[e] != -1 {
            next[e] = head[cp[e]];
            head[cp[e]] = e;
        }
    }
    let mut post = Vec::with_capacity(size + 1);
    let mut stack = Vec::with_capacity(size + 1);
    for i in 0..=n {
        if cp[i] != -1 {
            continue;
        }
        stack.push(i);
        while let Some(&p) = stack.last() {
            let child = head[p];
            if child == -1 {
                stack.pop();
                post.push(usize::try_from(p).unwrap());
            } else {
                head[p] = next[child];
                stack.push(child);
            }
        }
    }
    post.truncate(size);
    post
}
