/*
 * DBoW2 — Copyright (c) 2011–2013 Dorian Galvez-Lopez.
 * License: see LICENSE.txt.
 */
//! The six `DBoW2` sparse-vector scores.
use super::bow_vector::{BowVector, LNorm, ScoringType};
pub trait GeneralScoring {
    fn score(&self, v: &BowVector, w: &BowVector) -> f64;
    fn must_normalize(&self) -> Option<LNorm>;
}
#[must_use]
pub fn log_eps() -> f64 {
    f64::EPSILON.ln()
}
impl GeneralScoring for ScoringType {
    fn must_normalize(&self) -> Option<LNorm> {
        match self {
            Self::L2Norm => Some(LNorm::L2),
            Self::DotProduct => None,
            _ => Some(LNorm::L1),
        }
    }
    fn score(&self, v: &BowVector, w: &BowVector) -> f64 {
        let mut score = 0.0;
        if *self == Self::Kl {
            let mut left = v.iter().peekable();
            let mut right = w.iter().peekable();
            while let (Some(&(&vi_id, &vi)), Some(&(&wi_id, &wi))) = (left.peek(), right.peek()) {
                match vi_id.cmp(&wi_id) {
                    std::cmp::Ordering::Equal => {
                        if vi != 0.0 && wi != 0.0 {
                            score += vi * (vi / wi).ln();
                        }
                        left.next();
                        right.next();
                    }
                    std::cmp::Ordering::Less => {
                        score += vi * (vi.ln() - log_eps());
                        left.next();
                    }
                    std::cmp::Ordering::Greater => {
                        right.next();
                    }
                }
            }
            for (_, &vi) in left {
                if vi != 0.0 {
                    score += vi * (vi.ln() - log_eps());
                }
            }
            return score;
        }
        for (&id, &vi) in &v.0 {
            if let Some(&wi) = w.get(&id) {
                score += match self {
                    Self::L1Norm => (vi - wi).abs() - vi.abs() - wi.abs(),
                    Self::L2Norm | Self::DotProduct => vi * wi,
                    Self::ChiSquare => {
                        if vi + wi == 0.0 {
                            0.0
                        } else {
                            vi * wi / (vi + wi)
                        }
                    }
                    Self::Bhattacharyya => (vi * wi).sqrt(),
                    Self::Kl => unreachable!(),
                };
            }
        }
        match self {
            Self::L1Norm => -score / 2.0,
            Self::L2Norm => {
                if score >= 1.0 {
                    1.0
                } else {
                    1.0 - (1.0 - score).sqrt()
                }
            }
            Self::ChiSquare => 2.0 * score,
            _ => score,
        }
    }
}
macro_rules! scoring {
    ($name:ident,$kind:ident) => {
        #[derive(Clone, Copy, Debug, Default)]
        pub struct $name;
        impl GeneralScoring for $name {
            fn score(&self, v: &BowVector, w: &BowVector) -> f64 {
                ScoringType::$kind.score(v, w)
            }
            fn must_normalize(&self) -> Option<LNorm> {
                ScoringType::$kind.must_normalize()
            }
        }
    };
}
scoring!(L1Scoring, L1Norm);
scoring!(L2Scoring, L2Norm);
scoring!(ChiSquareScoring, ChiSquare);
scoring!(KlScoring, Kl);
scoring!(BhattacharyyaScoring, Bhattacharyya);
scoring!(DotProductScoring, DotProduct);
