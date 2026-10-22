//
//    AGAST, an adaptive and generic corner detector based on the
//              accelerated segment test for a 8 pixel mask
//
//    Copyright (C) 2010  Elmar Mair
//    All rights reserved.
//
//    Redistribution and use in source and binary forms, with or without
//    modification, are permitted provided that the following conditions are met:
//        * Redistributions of source code must retain the above copyright
//          notice, this list of conditions and the following disclaimer.
//        * Redistributions in binary form must reproduce the above copyright
//          notice, this list of conditions and the following disclaimer in the
//          documentation and/or other materials provided with the distribution.
//        * Neither the name of the <organization> nor the
//          names of its contributors may be used to endorse or promote products
//          derived from this software without specific prior written permission.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//    DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//! Bisection corner score for agast7-12s.
use crate::agast::{
    agast7_12s::AgastDetector712s,
    ast_detector::{checked_image, neighborhood},
};
use firefly_error::Result;
impl AgastDetector712s {
    /// # Errors
    /// Returns an error for invalid image storage, a corner/support outside the image,
    /// or a score threshold outside [-1, 255].
    pub fn corner_score(&self, image: &[u8], index: usize) -> Result<i32> {
        checked_image(&self.state, image)?;
        let samples = neighborhood(image, index, self.state.width, &Self::PATTERN)?;
        let center =
            i32::from(*image.get(index).ok_or_else(|| {
                crate::vectorized_filters::invalid("corner center outside image")
            })?);
        self.score_values(&samples, center)
    }
    fn score_values(&self, samples: &[i32; 12], center: i32) -> Result<i32> {
        let (mut bmin, mut bmax) = (self.state.threshold, 255i32);
        if !(-1..=255).contains(&bmin) {
            return Err(crate::vectorized_filters::invalid(
                "corner-score threshold outside [-1,255]",
            ));
        }
        let mut b_test = bmax.midpoint(bmin);
        loop {
            let bright_limit = center + b_test;
            let dark_limit = center - b_test;
            if Self::score_classify(samples, bright_limit, dark_limit) {
                bmin = b_test;
            } else {
                bmax = b_test;
            }
            if bmin == bmax - 1 || bmin == bmax {
                return Ok(bmin);
            }
            b_test = bmin.midpoint(bmax);
        }
    }
    fn score_classify(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        Self::score_classify_branch_129(samples, bright_limit, dark_limit)
    }
    #[inline]
    fn score_classify_branch_0(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[6] < dark_limit {
            {
                (samples[8] > bright_limit)
                    && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
            }
        } else if samples[6] > bright_limit {
            {
                (samples[8] > bright_limit)
                    && (if samples[4] > bright_limit {
                        (samples[3] > bright_limit) || (samples[10] > bright_limit)
                    } else {
                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                    })
            }
        } else {
            {
                (samples[8] > bright_limit)
                    && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
            }
        }
    }
    #[inline]
    fn score_classify_branch_1(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && (if samples[4] > bright_limit {
                                    (samples[3] > bright_limit) || (samples[10] > bright_limit)
                                } else {
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        } else if samples[1] > bright_limit {
            Self::score_classify_branch_0(samples, bright_limit, dark_limit)
        } else {
            (samples[6] >= dark_limit)
                && ((samples[6] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && (if samples[4] > bright_limit {
                                (samples[3] > bright_limit) || ({ samples[10] > bright_limit })
                            } else {
                                (samples[10] > bright_limit) && ({ samples[11] > bright_limit })
                            })
                    }))
        }
    }
    #[inline]
    fn score_classify_branch_2(samples: &[i32; 12], bright_limit: i32) -> bool {
        if samples[6] > bright_limit {
            if samples[3] > bright_limit {
                {
                    (samples[4] > bright_limit)
                        || ((samples[10] > bright_limit) && (samples[11] > bright_limit))
                }
            } else {
                {
                    (samples[8] > bright_limit)
                        && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
                }
            }
        } else {
            {
                (samples[11] > bright_limit)
                    && (if samples[3] > bright_limit {
                        (samples[4] > bright_limit) || (samples[10] > bright_limit)
                    } else {
                        (samples[8] > bright_limit) && (samples[10] > bright_limit)
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_3(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] > bright_limit {
            {
                (samples[1] >= dark_limit)
                    && ((samples[1] > bright_limit)
                        && (if samples[6] < dark_limit {
                            {
                                (samples[11] > bright_limit)
                                    && (if samples[3] > bright_limit {
                                        {
                                            (samples[4] > bright_limit)
                                                || (samples[10] > bright_limit)
                                        }
                                    } else {
                                        {
                                            (samples[8] > bright_limit)
                                                && (samples[10] > bright_limit)
                                        }
                                    })
                            }
                        } else {
                            Self::score_classify_branch_2(samples, bright_limit)
                        }))
            }
        } else {
            {
                (samples[1] >= dark_limit)
                    && ((samples[1] > bright_limit)
                        && (if samples[6] > bright_limit {
                            (samples[3] > bright_limit) && (samples[4] > bright_limit)
                        } else {
                            {
                                (samples[3] > bright_limit)
                                    && ({
                                        (samples[4] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            }
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_4(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            if samples[6] > bright_limit {
                (samples[3] > bright_limit) && (samples[4] > bright_limit)
            } else {
                {
                    (samples[3] > bright_limit)
                        && ({ (samples[4] > bright_limit) && (samples[11] > bright_limit) })
                }
            }
        } else {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[3] > bright_limit)
                                && ({ (samples[4] > bright_limit) && (samples[8] > bright_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_5(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] > bright_limit {
            if samples[1] < dark_limit {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({
                                (samples[3] > bright_limit)
                                    && ({
                                        (samples[4] > bright_limit) && (samples[8] > bright_limit)
                                    })
                            }))
                }
            } else {
                Self::score_classify_branch_4(samples, bright_limit, dark_limit)
            }
        } else {
            {
                (samples[1] >= dark_limit)
                    && ((samples[1] > bright_limit)
                        && (if samples[6] > bright_limit {
                            (samples[3] > bright_limit) && (samples[4] > bright_limit)
                        } else {
                            {
                                (samples[3] > bright_limit)
                                    && ({
                                        (samples[4] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            }
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_6(samples: &[i32; 12], bright_limit: i32) -> bool {
        if samples[6] > bright_limit {
            if samples[3] > bright_limit {
                {
                    (samples[4] > bright_limit)
                        || ((samples[10] > bright_limit) && (samples[11] > bright_limit))
                }
            } else {
                {
                    (samples[8] > bright_limit)
                        && ({
                            (samples[10] > bright_limit)
                                && ({ (samples[4] > bright_limit) || (samples[11] > bright_limit) })
                        })
                }
            }
        } else {
            {
                (samples[11] > bright_limit)
                    && (if samples[3] > bright_limit {
                        (samples[4] > bright_limit) || (samples[10] > bright_limit)
                    } else {
                        (samples[8] > bright_limit) && (samples[10] > bright_limit)
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_7(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            if samples[6] < dark_limit {
                {
                    (samples[11] > bright_limit)
                        && (if samples[3] > bright_limit {
                            (samples[4] > bright_limit) || (samples[10] > bright_limit)
                        } else {
                            (samples[8] > bright_limit) && (samples[10] > bright_limit)
                        })
                }
            } else {
                Self::score_classify_branch_6(samples, bright_limit)
            }
        } else {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && (if samples[4] > bright_limit {
                                    (samples[3] > bright_limit) || (samples[10] > bright_limit)
                                } else {
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_8(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            if samples[6] > bright_limit {
                (samples[3] > bright_limit) && (samples[4] > bright_limit)
            } else {
                {
                    (samples[3] > bright_limit)
                        && ({ (samples[4] > bright_limit) && (samples[11] > bright_limit) })
                }
            }
        } else {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[3] > bright_limit)
                                && ({ (samples[4] > bright_limit) && (samples[8] > bright_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_9(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] > bright_limit {
            if samples[1] < dark_limit {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({
                                (samples[8] > bright_limit)
                                    && (if samples[4] > bright_limit {
                                        {
                                            (samples[3] > bright_limit)
                                                || (samples[10] > bright_limit)
                                        }
                                    } else {
                                        {
                                            (samples[10] > bright_limit)
                                                && (samples[11] > bright_limit)
                                        }
                                    })
                            }))
                }
            } else {
                Self::score_classify_branch_7(samples, bright_limit, dark_limit)
            }
        } else {
            if samples[1] < dark_limit {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({
                                (samples[3] > bright_limit)
                                    && ({
                                        (samples[4] > bright_limit) && (samples[8] > bright_limit)
                                    })
                            }))
                }
            } else {
                Self::score_classify_branch_8(samples, bright_limit, dark_limit)
            }
        }
    }
    #[inline]
    fn score_classify_branch_10(samples: &[i32; 12], bright_limit: i32) -> bool {
        if samples[6] > bright_limit {
            if samples[3] > bright_limit {
                {
                    (samples[4] > bright_limit)
                        || ((samples[10] > bright_limit) && (samples[11] > bright_limit))
                }
            } else {
                {
                    (samples[8] > bright_limit)
                        && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
                }
            }
        } else {
            {
                (samples[11] > bright_limit)
                    && (if samples[3] > bright_limit {
                        (samples[4] > bright_limit) || (samples[10] > bright_limit)
                    } else {
                        (samples[8] > bright_limit) && (samples[10] > bright_limit)
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_11(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] > bright_limit {
            {
                (samples[1] >= dark_limit)
                    && ((samples[1] > bright_limit)
                        && (if samples[6] < dark_limit {
                            {
                                (samples[11] > bright_limit)
                                    && (if samples[3] > bright_limit {
                                        {
                                            (samples[4] > bright_limit)
                                                || (samples[10] > bright_limit)
                                        }
                                    } else {
                                        {
                                            (samples[8] > bright_limit)
                                                && (samples[10] > bright_limit)
                                        }
                                    })
                            }
                        } else {
                            Self::score_classify_branch_10(samples, bright_limit)
                        }))
            }
        } else {
            {
                (samples[1] >= dark_limit)
                    && ((samples[1] > bright_limit)
                        && (if samples[6] > bright_limit {
                            (samples[3] > bright_limit) && (samples[4] > bright_limit)
                        } else {
                            {
                                (samples[3] > bright_limit)
                                    && ({
                                        (samples[4] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            }
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_12(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] < dark_limit {
            if samples[9] < dark_limit {
                {
                    (samples[1] >= dark_limit)
                        && ((samples[1] > bright_limit)
                            && (if samples[6] > bright_limit {
                                (samples[3] > bright_limit) && (samples[4] > bright_limit)
                            } else {
                                {
                                    (samples[3] > bright_limit)
                                        && ({
                                            (samples[4] > bright_limit)
                                                && (samples[11] > bright_limit)
                                        })
                                }
                            }))
                }
            } else {
                Self::score_classify_branch_3(samples, bright_limit, dark_limit)
            }
        } else {
            if samples[9] < dark_limit {
                Self::score_classify_branch_5(samples, bright_limit, dark_limit)
            } else if samples[7] > bright_limit {
                Self::score_classify_branch_9(samples, bright_limit, dark_limit)
            } else {
                Self::score_classify_branch_11(samples, bright_limit, dark_limit)
            }
        }
    }
    #[inline]
    fn score_classify_branch_13(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[6] < dark_limit {
            {
                (samples[8] > bright_limit)
                    && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
            }
        } else if samples[6] > bright_limit {
            {
                (samples[8] > bright_limit)
                    && (if samples[4] > bright_limit {
                        (samples[3] > bright_limit) || (samples[10] > bright_limit)
                    } else {
                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                    })
            }
        } else {
            {
                (samples[8] > bright_limit)
                    && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
            }
        }
    }
    #[inline]
    fn score_classify_branch_14(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            Self::score_classify_branch_13(samples, bright_limit, dark_limit)
        } else {
            (samples[6] >= dark_limit)
                && ((samples[6] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && (if samples[4] > bright_limit {
                                (samples[3] > bright_limit) || ({ samples[10] > bright_limit })
                            } else {
                                (samples[10] > bright_limit) && ({ samples[11] > bright_limit })
                            })
                    }))
        }
    }
    #[inline]
    fn score_classify_branch_15(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            (samples[9] >= dark_limit)
                && ((samples[9] > bright_limit)
                    && (if samples[1] < dark_limit {
                        {
                            (samples[6] >= dark_limit)
                                && ((samples[6] > bright_limit)
                                    && ({
                                        (samples[8] > bright_limit)
                                            && (if samples[4] > bright_limit {
                                                {
                                                    (samples[3] > bright_limit)
                                                        || (samples[10] > bright_limit)
                                                }
                                            } else {
                                                {
                                                    (samples[10] > bright_limit)
                                                        && (samples[11] > bright_limit)
                                                }
                                            })
                                    }))
                        }
                    } else {
                        Self::score_classify_branch_14(samples, bright_limit, dark_limit)
                    }))
        }
    }
    #[inline]
    fn score_classify_branch_16(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            {
                (samples[3] > bright_limit)
                    && ({
                        (samples[4] > bright_limit)
                            && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_17(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && (if samples[4] < dark_limit {
                            {
                                (samples[3] < dark_limit)
                                    || ((samples[8] < dark_limit) && (samples[10] < dark_limit))
                            }
                        } else {
                            {
                                (samples[8] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            }
                        }))
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && (if samples[4] < dark_limit {
                                    (samples[3] < dark_limit) || (samples[10] < dark_limit)
                                } else {
                                    (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_18(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[4] > bright_limit {
            if samples[10] > bright_limit {
                (samples[3] > bright_limit) && (samples[11] > bright_limit)
            } else {
                {
                    (samples[8] < dark_limit)
                        && ({ (samples[11] < dark_limit) && (samples[10] < dark_limit) })
                }
            }
        } else {
            {
                (samples[8] < dark_limit)
                    && (if samples[10] < dark_limit {
                        (samples[4] < dark_limit) || (samples[11] < dark_limit)
                    } else {
                        (samples[3] < dark_limit) && (samples[4] < dark_limit)
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_19(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[6] > bright_limit {
            {
                (samples[3] > bright_limit)
                    && ({
                        (samples[4] > bright_limit)
                            && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
                    })
            }
        } else if samples[6] < dark_limit {
            Self::score_classify_branch_18(samples, bright_limit, dark_limit)
        } else {
            (samples[3] > bright_limit)
                && ({
                    (samples[4] > bright_limit)
                        && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
                })
        }
    }
    #[inline]
    fn score_classify_branch_20(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && (if samples[4] < dark_limit {
                                    (samples[3] < dark_limit) || (samples[10] < dark_limit)
                                } else {
                                    (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                })
                        }))
            }
        } else if samples[1] > bright_limit {
            Self::score_classify_branch_19(samples, bright_limit, dark_limit)
        } else {
            (samples[6] <= bright_limit)
                && ((samples[6] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && (if samples[4] < dark_limit {
                                (samples[3] < dark_limit) || (samples[10] < dark_limit)
                            } else {
                                (samples[10] < dark_limit) && (samples[11] < dark_limit)
                            })
                    }))
        }
    }
    #[inline]
    fn score_classify_branch_21(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && (if samples[4] < dark_limit {
                                    (samples[3] < dark_limit) || (samples[10] < dark_limit)
                                } else {
                                    (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_22(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] < dark_limit {
            if samples[1] > bright_limit {
                {
                    (samples[6] <= bright_limit)
                        && ((samples[6] < dark_limit)
                            && ({
                                (samples[8] < dark_limit)
                                    && (if samples[4] < dark_limit {
                                        (samples[3] < dark_limit) || (samples[10] < dark_limit)
                                    } else {
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            }))
                }
            } else {
                Self::score_classify_branch_17(samples, bright_limit, dark_limit)
            }
        } else {
            if samples[2] > bright_limit {
                Self::score_classify_branch_20(samples, bright_limit, dark_limit)
            } else {
                Self::score_classify_branch_21(samples, bright_limit, dark_limit)
            }
        }
    }
    #[inline]
    fn score_classify_branch_23(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            {
                (samples[3] > bright_limit)
                    && ({
                        (samples[4] > bright_limit)
                            && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_24(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] > bright_limit {
            {
                (samples[2] >= dark_limit)
                    && ((samples[2] > bright_limit)
                        && ({
                            (samples[1] >= dark_limit)
                                && ((samples[1] > bright_limit)
                                    && (Self::score_classify_branch_16(samples, bright_limit)))
                        }))
            }
        } else if samples[7] < dark_limit {
            Self::score_classify_branch_22(samples, bright_limit, dark_limit)
        } else {
            (samples[2] >= dark_limit)
                && ((samples[2] > bright_limit)
                    && ({
                        (samples[1] >= dark_limit)
                            && ((samples[1] > bright_limit)
                                && (Self::score_classify_branch_23(samples, bright_limit)))
                    }))
        }
    }
    #[inline]
    fn score_classify_branch_25(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            (samples[2] < dark_limit)
                && (if samples[1] < dark_limit {
                    {
                        (samples[6] <= bright_limit)
                            && ((samples[6] < dark_limit)
                                && ({ (samples[3] < dark_limit) && (samples[4] < dark_limit) }))
                    }
                } else {
                    {
                        (samples[6] <= bright_limit)
                            && ((samples[6] < dark_limit)
                                && ({
                                    (samples[3] < dark_limit)
                                        && ({
                                            (samples[4] < dark_limit) && (samples[8] < dark_limit)
                                        })
                                }))
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_26(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] > bright_limit {
            {
                (samples[1] >= dark_limit)
                    && ((samples[1] > bright_limit)
                        && ({
                            {
                                (samples[10] > bright_limit)
                                    && ({
                                        (samples[11] > bright_limit)
                                            && ({
                                                (samples[3] > bright_limit)
                                                    || (samples[8] > bright_limit)
                                            })
                                    })
                            }
                        }))
            }
        } else {
            Self::score_classify_branch_25(samples, bright_limit, dark_limit)
        }
    }
    #[inline]
    fn score_classify_branch_27(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            {
                {
                    (samples[8] > bright_limit)
                        && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
                }
            }
        } else {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && ({
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_28(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            {
                {
                    (samples[10] > bright_limit)
                        && ({
                            (samples[11] > bright_limit)
                                && ({ (samples[3] > bright_limit) || (samples[8] > bright_limit) })
                        })
                }
            }
        } else {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && ({
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_29(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            {
                {
                    (samples[8] > bright_limit)
                        && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
                }
            }
        } else {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && ({
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_30(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] > bright_limit {
            if samples[1] < dark_limit {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({
                                (samples[8] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            }))
                }
            } else {
                Self::score_classify_branch_28(samples, bright_limit, dark_limit)
            }
        } else {
            if samples[1] < dark_limit {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({
                                (samples[8] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            }))
                }
            } else {
                Self::score_classify_branch_29(samples, bright_limit, dark_limit)
            }
        }
    }
    #[inline]
    fn score_classify_branch_31(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({ (samples[3] > bright_limit) || (samples[8] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_32(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] > bright_limit {
            if samples[2] < dark_limit {
                if samples[1] < dark_limit {
                    {
                        (samples[6] >= dark_limit)
                            && ((samples[6] > bright_limit)
                                && ({
                                    (samples[8] > bright_limit)
                                        && ({
                                            (samples[10] > bright_limit)
                                                && (samples[11] > bright_limit)
                                        })
                                }))
                    }
                } else {
                    Self::score_classify_branch_27(samples, bright_limit, dark_limit)
                }
            } else {
                Self::score_classify_branch_30(samples, bright_limit, dark_limit)
            }
        } else {
            {
                (samples[2] >= dark_limit)
                    && ((samples[2] > bright_limit)
                        && ({
                            (samples[1] >= dark_limit)
                                && ((samples[1] > bright_limit)
                                    && (Self::score_classify_branch_31(samples, bright_limit)))
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_33(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({ (samples[3] < dark_limit) && (samples[4] < dark_limit) }))
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[3] < dark_limit)
                                && ({ (samples[4] < dark_limit) && (samples[8] < dark_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_34(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            (samples[1] > bright_limit)
                && ({
                    {
                        (samples[3] > bright_limit)
                            && ({
                                (samples[4] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            })
                    }
                })
        }
    }

    #[inline]
    fn score_classify_branch_37(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            (samples[1] > bright_limit)
                && ({
                    {
                        (samples[3] > bright_limit)
                            && ({
                                (samples[4] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_38(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] < dark_limit {
            Self::score_classify_branch_24(samples, bright_limit, dark_limit)
        } else if samples[9] > bright_limit {
            if samples[7] < dark_limit {
                Self::score_classify_branch_26(samples, bright_limit, dark_limit)
            } else {
                Self::score_classify_branch_32(samples, bright_limit, dark_limit)
            }
        } else {
            if samples[2] < dark_limit {
                {
                    (samples[7] <= bright_limit)
                        && ((samples[7] < dark_limit)
                            && (Self::score_classify_branch_33(samples, bright_limit, dark_limit)))
                }
            } else {
                {
                    (samples[2] > bright_limit)
                        && (if samples[7] > bright_limit {
                            {
                                (samples[1] >= dark_limit)
                                    && (Self::score_classify_branch_34(samples, bright_limit))
                            }
                        } else {
                            {
                                (samples[1] >= dark_limit)
                                    && (Self::score_classify_branch_37(samples, bright_limit))
                            }
                        })
                }
            }
        }
    }
    #[inline]
    fn score_classify_branch_39(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            {
                {
                    (samples[8] > bright_limit)
                        && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
                }
            }
        } else {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && ({
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_40(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            (samples[1] > bright_limit)
                && ({
                    {
                        (samples[3] > bright_limit)
                            && ({
                                (samples[4] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_41(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            (samples[1] > bright_limit)
                && ({
                    {
                        (samples[10] > bright_limit)
                            && ({
                                (samples[11] > bright_limit)
                                    && ({
                                        (samples[3] > bright_limit) || (samples[8] > bright_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_42(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            (samples[1] > bright_limit)
                && ({
                    {
                        (samples[3] > bright_limit)
                            && ({
                                (samples[4] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            })
                    }
                })
        }
    }

    #[inline]
    fn score_classify_branch_45(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            (samples[1] > bright_limit)
                && ({
                    {
                        (samples[3] > bright_limit)
                            && ({
                                (samples[4] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_46(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            {
                {
                    (samples[10] > bright_limit)
                        && ({
                            (samples[11] > bright_limit)
                                && ({ (samples[3] > bright_limit) || (samples[8] > bright_limit) })
                        })
                }
            }
        } else {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && ({
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_47(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            (samples[1] > bright_limit)
                && ({
                    {
                        (samples[3] > bright_limit)
                            && ({
                                (samples[4] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_48(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({ (samples[3] > bright_limit) || (samples[8] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_49(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            (samples[1] > bright_limit)
                && ({
                    {
                        (samples[3] > bright_limit)
                            && ({
                                (samples[4] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_50(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] > bright_limit {
            if samples[9] > bright_limit {
                if samples[1] < dark_limit {
                    {
                        (samples[6] >= dark_limit)
                            && ((samples[6] > bright_limit)
                                && ({
                                    (samples[8] > bright_limit)
                                        && ({
                                            (samples[10] > bright_limit)
                                                && (samples[11] > bright_limit)
                                        })
                                }))
                    }
                } else {
                    Self::score_classify_branch_46(samples, bright_limit, dark_limit)
                }
            } else {
                {
                    (samples[1] >= dark_limit)
                        && (Self::score_classify_branch_47(samples, bright_limit))
                }
            }
        } else {
            if samples[9] > bright_limit {
                {
                    (samples[1] >= dark_limit)
                        && ((samples[1] > bright_limit)
                            && (Self::score_classify_branch_48(samples, bright_limit)))
                }
            } else {
                {
                    (samples[1] >= dark_limit)
                        && (Self::score_classify_branch_49(samples, bright_limit))
                }
            }
        }
    }
    #[inline]
    fn score_classify_branch_51(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            {
                {
                    (samples[8] > bright_limit)
                        && ({ (samples[10] > bright_limit) && (samples[11] > bright_limit) })
                }
            }
        } else {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && ({
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_52(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] > bright_limit {
            if samples[7] < dark_limit {
                if samples[9] < dark_limit {
                    {
                        (samples[1] >= dark_limit)
                            && (Self::score_classify_branch_40(samples, bright_limit))
                    }
                } else if samples[9] > bright_limit {
                    {
                        (samples[1] >= dark_limit)
                            && (Self::score_classify_branch_41(samples, bright_limit))
                    }
                } else {
                    {
                        (samples[1] >= dark_limit)
                            && (Self::score_classify_branch_42(samples, bright_limit))
                    }
                }
            } else {
                if samples[9] < dark_limit {
                    {
                        (samples[1] >= dark_limit)
                            && (Self::score_classify_branch_45(samples, bright_limit))
                    }
                } else {
                    Self::score_classify_branch_50(samples, bright_limit, dark_limit)
                }
            }
        } else {
            {
                (samples[7] > bright_limit)
                    && ({
                        (samples[9] >= dark_limit)
                            && ((samples[9] > bright_limit)
                                && (if samples[1] < dark_limit {
                                    {
                                        (samples[6] >= dark_limit)
                                            && ((samples[6] > bright_limit)
                                                && ({
                                                    (samples[8] > bright_limit)
                                                        && ({
                                                            (samples[10] > bright_limit)
                                                                && (samples[11] > bright_limit)
                                                        })
                                                }))
                                    }
                                } else {
                                    Self::score_classify_branch_51(
                                        samples,
                                        bright_limit,
                                        dark_limit,
                                    )
                                }))
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_53(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] < dark_limit {
            {
                (samples[7] > bright_limit)
                    && ({
                        (samples[9] >= dark_limit)
                            && ((samples[9] > bright_limit)
                                && (if samples[1] < dark_limit {
                                    {
                                        (samples[6] >= dark_limit)
                                            && ((samples[6] > bright_limit)
                                                && ({
                                                    (samples[8] > bright_limit)
                                                        && ({
                                                            (samples[10] > bright_limit)
                                                                && (samples[11] > bright_limit)
                                                        })
                                                }))
                                    }
                                } else {
                                    Self::score_classify_branch_39(
                                        samples,
                                        bright_limit,
                                        dark_limit,
                                    )
                                }))
                    })
            }
        } else {
            Self::score_classify_branch_52(samples, bright_limit, dark_limit)
        }
    }
    #[inline]
    fn score_classify_branch_54(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            if samples[6] < dark_limit {
                (samples[3] < dark_limit) && (samples[4] < dark_limit)
            } else {
                {
                    (samples[3] < dark_limit)
                        && ({ (samples[4] < dark_limit) && (samples[11] < dark_limit) })
                }
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[3] < dark_limit)
                                && ({ (samples[4] < dark_limit) && (samples[8] < dark_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_55(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] < dark_limit {
            if samples[1] > bright_limit {
                {
                    (samples[6] <= bright_limit)
                        && ((samples[6] < dark_limit)
                            && ({
                                (samples[3] < dark_limit)
                                    && ({ (samples[4] < dark_limit) && (samples[8] < dark_limit) })
                            }))
                }
            } else {
                Self::score_classify_branch_54(samples, bright_limit, dark_limit)
            }
        } else {
            {
                (samples[1] <= bright_limit)
                    && ((samples[1] < dark_limit)
                        && (if samples[6] < dark_limit {
                            (samples[3] < dark_limit) && (samples[4] < dark_limit)
                        } else {
                            {
                                (samples[3] < dark_limit)
                                    && ({ (samples[4] < dark_limit) && (samples[11] < dark_limit) })
                            }
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_56(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            (samples[2] <= bright_limit)
                && ((samples[2] < dark_limit)
                    && (if samples[7] > bright_limit {
                        {
                            (samples[1] <= bright_limit)
                                && ((samples[1] < dark_limit)
                                    && (if samples[6] < dark_limit {
                                        (samples[3] < dark_limit) && (samples[4] < dark_limit)
                                    } else {
                                        {
                                            (samples[3] < dark_limit)
                                                && ({
                                                    (samples[4] < dark_limit)
                                                        && (samples[11] < dark_limit)
                                                })
                                        }
                                    }))
                        }
                    } else {
                        Self::score_classify_branch_55(samples, bright_limit, dark_limit)
                    }))
        }
    }
    #[inline]
    fn score_classify_branch_57(samples: &[i32; 12], dark_limit: i32) -> bool {
        if samples[6] < dark_limit {
            if samples[3] < dark_limit {
                {
                    (samples[4] < dark_limit)
                        || ((samples[10] < dark_limit) && ({ samples[11] < dark_limit }))
                }
            } else {
                {
                    (samples[8] < dark_limit)
                        && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                }
            }
        } else {
            {
                (samples[11] < dark_limit)
                    && (if samples[3] < dark_limit {
                        (samples[4] < dark_limit) || (samples[10] < dark_limit)
                    } else {
                        (samples[8] < dark_limit) && (samples[10] < dark_limit)
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_58(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            (samples[1] < dark_limit)
                && (if samples[6] > bright_limit {
                    {
                        (samples[11] < dark_limit)
                            && (if samples[3] < dark_limit {
                                (samples[4] < dark_limit) || (samples[10] < dark_limit)
                            } else {
                                (samples[8] < dark_limit) && (samples[10] < dark_limit)
                            })
                    }
                } else {
                    Self::score_classify_branch_57(samples, dark_limit)
                })
        }
    }
    #[inline]
    fn score_classify_branch_59(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[6] > bright_limit {
            {
                (samples[8] < dark_limit)
                    && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
            }
        } else if samples[6] < dark_limit {
            {
                (samples[8] < dark_limit)
                    && (if samples[4] < dark_limit {
                        (samples[3] < dark_limit) || (samples[10] < dark_limit)
                    } else {
                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                    })
            }
        } else {
            {
                (samples[8] < dark_limit)
                    && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
            }
        }
    }
    #[inline]
    fn score_classify_branch_60(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && (if samples[4] < dark_limit {
                                    (samples[3] < dark_limit) || (samples[10] < dark_limit)
                                } else {
                                    (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                })
                        }))
            }
        } else if samples[1] < dark_limit {
            Self::score_classify_branch_59(samples, bright_limit, dark_limit)
        } else {
            (samples[6] <= bright_limit)
                && ((samples[6] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && (if samples[4] < dark_limit {
                                (samples[3] < dark_limit) || (samples[10] < dark_limit)
                            } else {
                                (samples[10] < dark_limit) && (samples[11] < dark_limit)
                            })
                    }))
        }
    }
    #[inline]
    fn score_classify_branch_61(samples: &[i32; 12], dark_limit: i32) -> bool {
        if samples[6] < dark_limit {
            if samples[3] < dark_limit {
                {
                    (samples[4] < dark_limit)
                        || ((samples[10] < dark_limit) && (samples[11] < dark_limit))
                }
            } else {
                {
                    (samples[8] < dark_limit)
                        && ({
                            (samples[10] < dark_limit)
                                && ({ (samples[4] < dark_limit) || (samples[11] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[11] < dark_limit)
                    && (if samples[3] < dark_limit {
                        (samples[4] < dark_limit) || (samples[10] < dark_limit)
                    } else {
                        (samples[8] < dark_limit) && (samples[10] < dark_limit)
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_62(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            if samples[6] > bright_limit {
                {
                    (samples[11] < dark_limit)
                        && (if samples[3] < dark_limit {
                            (samples[4] < dark_limit) || (samples[10] < dark_limit)
                        } else {
                            (samples[8] < dark_limit) && (samples[10] < dark_limit)
                        })
                }
            } else {
                Self::score_classify_branch_61(samples, dark_limit)
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && (if samples[4] < dark_limit {
                                    (samples[3] < dark_limit) || (samples[10] < dark_limit)
                                } else {
                                    (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_63(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[6] > bright_limit {
            {
                (samples[8] < dark_limit)
                    && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
            }
        } else if samples[6] < dark_limit {
            {
                (samples[8] < dark_limit)
                    && (if samples[4] < dark_limit {
                        (samples[3] < dark_limit) || (samples[10] < dark_limit)
                    } else {
                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                    })
            }
        } else {
            {
                (samples[8] < dark_limit)
                    && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
            }
        }
    }
    #[inline]
    fn score_classify_branch_64(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && (if samples[4] < dark_limit {
                                    (samples[3] < dark_limit) || (samples[10] < dark_limit)
                                } else {
                                    (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                })
                        }))
            }
        } else if samples[1] < dark_limit {
            Self::score_classify_branch_63(samples, bright_limit, dark_limit)
        } else {
            (samples[6] <= bright_limit)
                && ((samples[6] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && (if samples[4] < dark_limit {
                                (samples[3] < dark_limit) || (samples[10] < dark_limit)
                            } else {
                                (samples[10] < dark_limit) && (samples[11] < dark_limit)
                            })
                    }))
        }
    }
    #[inline]
    fn score_classify_branch_65(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] < dark_limit {
            if samples[1] > bright_limit {
                {
                    (samples[6] <= bright_limit)
                        && ((samples[6] < dark_limit)
                            && ({
                                (samples[8] < dark_limit)
                                    && (if samples[4] < dark_limit {
                                        (samples[3] < dark_limit) || (samples[10] < dark_limit)
                                    } else {
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            }))
                }
            } else {
                Self::score_classify_branch_62(samples, bright_limit, dark_limit)
            }
        } else {
            Self::score_classify_branch_64(samples, bright_limit, dark_limit)
        }
    }
    #[inline]
    fn score_classify_branch_66(samples: &[i32; 12], dark_limit: i32) -> bool {
        if samples[6] < dark_limit {
            if samples[3] < dark_limit {
                {
                    (samples[4] < dark_limit)
                        || ((samples[10] < dark_limit) && ({ samples[11] < dark_limit }))
                }
            } else {
                {
                    (samples[8] < dark_limit)
                        && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                }
            }
        } else {
            {
                (samples[11] < dark_limit)
                    && (if samples[3] < dark_limit {
                        (samples[4] < dark_limit) || (samples[10] < dark_limit)
                    } else {
                        (samples[8] < dark_limit) && (samples[10] < dark_limit)
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_67(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            (samples[1] < dark_limit)
                && (if samples[6] > bright_limit {
                    {
                        (samples[11] < dark_limit)
                            && (if samples[3] < dark_limit {
                                (samples[4] < dark_limit) || (samples[10] < dark_limit)
                            } else {
                                (samples[8] < dark_limit) && (samples[10] < dark_limit)
                            })
                    }
                } else {
                    Self::score_classify_branch_66(samples, dark_limit)
                })
        }
    }
    #[inline]
    fn score_classify_branch_68(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            if samples[6] < dark_limit {
                (samples[3] < dark_limit) && (samples[4] < dark_limit)
            } else {
                {
                    (samples[3] < dark_limit)
                        && ({ (samples[4] < dark_limit) && (samples[11] < dark_limit) })
                }
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[3] < dark_limit)
                                && ({ (samples[4] < dark_limit) && (samples[8] < dark_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_69(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] < dark_limit {
            if samples[1] > bright_limit {
                {
                    (samples[6] <= bright_limit)
                        && ((samples[6] < dark_limit)
                            && ({
                                (samples[3] < dark_limit)
                                    && ({ (samples[4] < dark_limit) && (samples[8] < dark_limit) })
                            }))
                }
            } else {
                Self::score_classify_branch_68(samples, bright_limit, dark_limit)
            }
        } else {
            {
                (samples[1] <= bright_limit)
                    && ((samples[1] < dark_limit)
                        && (if samples[6] < dark_limit {
                            (samples[3] < dark_limit) && (samples[4] < dark_limit)
                        } else {
                            {
                                (samples[3] < dark_limit)
                                    && ({ (samples[4] < dark_limit) && (samples[11] < dark_limit) })
                            }
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_70(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            (samples[2] < dark_limit)
                && (if samples[7] > bright_limit {
                    {
                        (samples[1] <= bright_limit)
                            && ((samples[1] < dark_limit)
                                && (if samples[6] < dark_limit {
                                    (samples[3] < dark_limit) && (samples[4] < dark_limit)
                                } else {
                                    {
                                        (samples[3] < dark_limit)
                                            && ({
                                                (samples[4] < dark_limit)
                                                    && (samples[11] < dark_limit)
                                            })
                                    }
                                }))
                    }
                } else {
                    Self::score_classify_branch_69(samples, bright_limit, dark_limit)
                })
        }
    }
    #[inline]
    fn score_classify_branch_71(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] < dark_limit {
            if samples[7] > bright_limit {
                {
                    (samples[2] <= bright_limit)
                        && ((samples[2] < dark_limit)
                            && ({
                                (samples[1] <= bright_limit)
                                    && (Self::score_classify_branch_58(
                                        samples,
                                        bright_limit,
                                        dark_limit,
                                    ))
                            }))
                }
            } else if samples[7] < dark_limit {
                if samples[2] > bright_limit {
                    Self::score_classify_branch_60(samples, bright_limit, dark_limit)
                } else {
                    Self::score_classify_branch_65(samples, bright_limit, dark_limit)
                }
            } else {
                {
                    (samples[2] <= bright_limit)
                        && ((samples[2] < dark_limit)
                            && ({
                                (samples[1] <= bright_limit)
                                    && (Self::score_classify_branch_67(
                                        samples,
                                        bright_limit,
                                        dark_limit,
                                    ))
                            }))
                }
            }
        } else {
            {
                (samples[2] <= bright_limit)
                    && (Self::score_classify_branch_70(samples, bright_limit, dark_limit))
            }
        }
    }
    #[inline]
    fn score_classify_branch_72(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                {
                    (samples[8] < dark_limit)
                        && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                }
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_73(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && (if samples[4] > bright_limit {
                            {
                                (samples[3] > bright_limit)
                                    || ((samples[8] > bright_limit) && (samples[10] > bright_limit))
                            }
                        } else {
                            {
                                (samples[8] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            }
                        }))
            }
        } else {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && (if samples[4] > bright_limit {
                                    (samples[3] > bright_limit) || (samples[10] > bright_limit)
                                } else {
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_74(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] > bright_limit {
            if samples[1] < dark_limit {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({
                                (samples[8] > bright_limit)
                                    && (if samples[4] > bright_limit {
                                        {
                                            (samples[3] > bright_limit)
                                                || (samples[10] > bright_limit)
                                        }
                                    } else {
                                        {
                                            (samples[10] > bright_limit)
                                                && (samples[11] > bright_limit)
                                        }
                                    })
                            }))
                }
            } else {
                Self::score_classify_branch_73(samples, bright_limit, dark_limit)
            }
        } else {
            if samples[1] > bright_limit {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({ (samples[3] > bright_limit) && (samples[4] > bright_limit) }))
                }
            } else {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({
                                (samples[3] > bright_limit)
                                    && ({
                                        (samples[4] > bright_limit) && (samples[8] > bright_limit)
                                    })
                            }))
                }
            }
        }
    }
    #[inline]
    fn score_classify_branch_75(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] < dark_limit {
            if samples[1] > bright_limit {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({ (samples[3] > bright_limit) && (samples[4] > bright_limit) }))
                }
            } else {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({
                                (samples[3] > bright_limit)
                                    && ({
                                        (samples[4] > bright_limit) && (samples[8] > bright_limit)
                                    })
                            }))
                }
            }
        } else {
            Self::score_classify_branch_74(samples, bright_limit, dark_limit)
        }
    }
    #[inline]
    fn score_classify_branch_76(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] < dark_limit {
            {
                (samples[9] <= bright_limit)
                    && ((samples[9] < dark_limit)
                        && (if samples[1] > bright_limit {
                            {
                                (samples[6] <= bright_limit)
                                    && ((samples[6] < dark_limit)
                                        && ({
                                            (samples[8] < dark_limit)
                                                && ({
                                                    (samples[10] < dark_limit)
                                                        && (samples[11] < dark_limit)
                                                })
                                        }))
                            }
                        } else {
                            Self::score_classify_branch_72(samples, bright_limit, dark_limit)
                        }))
            }
        } else {
            {
                (samples[7] > bright_limit)
                    && (Self::score_classify_branch_75(samples, bright_limit, dark_limit))
            }
        }
    }
    #[inline]
    fn score_classify_branch_77(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[1] < dark_limit)
                && ({
                    {
                        (samples[3] < dark_limit)
                            && ({
                                (samples[4] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_78(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                {
                    (samples[10] < dark_limit)
                        && ({
                            (samples[11] < dark_limit)
                                && ({ (samples[3] < dark_limit) || (samples[8] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_79(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[1] < dark_limit)
                && ({
                    {
                        (samples[3] < dark_limit)
                            && ({
                                (samples[4] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_80(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({ (samples[3] < dark_limit) || (samples[8] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_81(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[4] < dark_limit {
            if samples[10] > bright_limit {
                (samples[8] > bright_limit) && (samples[11] > bright_limit)
            } else {
                {
                    (samples[3] < dark_limit)
                        && ({ (samples[11] < dark_limit) && (samples[10] < dark_limit) })
                }
            }
        } else {
            {
                (samples[8] > bright_limit)
                    && (if samples[10] > bright_limit {
                        (samples[4] > bright_limit) || (samples[11] > bright_limit)
                    } else {
                        (samples[3] > bright_limit) && (samples[4] > bright_limit)
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_82(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[6] < dark_limit {
            {
                (samples[3] < dark_limit)
                    && ({
                        (samples[4] < dark_limit)
                            && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                    })
            }
        } else if samples[6] > bright_limit {
            Self::score_classify_branch_81(samples, bright_limit, dark_limit)
        } else {
            (samples[3] < dark_limit)
                && ({
                    (samples[4] < dark_limit)
                        && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                })
        }
    }
    #[inline]
    fn score_classify_branch_83(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && (if samples[4] > bright_limit {
                                    (samples[3] > bright_limit) || (samples[10] > bright_limit)
                                } else {
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        } else if samples[1] < dark_limit {
            Self::score_classify_branch_82(samples, bright_limit, dark_limit)
        } else {
            (samples[6] >= dark_limit)
                && ((samples[6] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && (if samples[4] > bright_limit {
                                (samples[3] > bright_limit) || (samples[10] > bright_limit)
                            } else {
                                (samples[10] > bright_limit) && (samples[11] > bright_limit)
                            })
                    }))
        }
    }
    #[inline]
    fn score_classify_branch_84(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[1] < dark_limit)
                && ({
                    {
                        (samples[3] < dark_limit)
                            && ({
                                (samples[4] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_85(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[1] < dark_limit)
                && ({
                    {
                        (samples[3] < dark_limit)
                            && ({
                                (samples[4] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_86(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({ (samples[3] < dark_limit) || (samples[8] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_87(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[1] < dark_limit)
                && ({
                    {
                        (samples[3] < dark_limit)
                            && ({
                                (samples[4] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_88(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] > bright_limit {
            if samples[9] < dark_limit {
                {
                    (samples[1] <= bright_limit)
                        && ((samples[1] < dark_limit)
                            && (Self::score_classify_branch_80(samples, dark_limit)))
                }
            } else if samples[9] > bright_limit {
                Self::score_classify_branch_83(samples, bright_limit, dark_limit)
            } else {
                (samples[1] <= bright_limit)
                    && (Self::score_classify_branch_84(samples, dark_limit))
            }
        } else {
            if samples[9] > bright_limit {
                {
                    (samples[1] <= bright_limit)
                        && (Self::score_classify_branch_85(samples, dark_limit))
                }
            } else if samples[9] < dark_limit {
                {
                    (samples[1] <= bright_limit)
                        && ((samples[1] < dark_limit)
                            && (Self::score_classify_branch_86(samples, dark_limit)))
                }
            } else {
                {
                    (samples[1] <= bright_limit)
                        && (Self::score_classify_branch_87(samples, dark_limit))
                }
            }
        }
    }
    #[inline]
    fn score_classify_branch_89(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] < dark_limit {
            if samples[9] > bright_limit {
                {
                    (samples[1] <= bright_limit)
                        && (Self::score_classify_branch_77(samples, dark_limit))
                }
            } else if samples[9] < dark_limit {
                if samples[1] > bright_limit {
                    {
                        (samples[6] <= bright_limit)
                            && ((samples[6] < dark_limit)
                                && ({
                                    (samples[8] < dark_limit)
                                        && ({
                                            (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                        })
                                }))
                    }
                } else {
                    Self::score_classify_branch_78(samples, bright_limit, dark_limit)
                }
            } else {
                {
                    (samples[1] <= bright_limit)
                        && (Self::score_classify_branch_79(samples, dark_limit))
                }
            }
        } else {
            Self::score_classify_branch_88(samples, bright_limit, dark_limit)
        }
    }
    #[inline]
    fn score_classify_branch_90(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && (if samples[4] > bright_limit {
                                    (samples[3] > bright_limit) || (samples[10] > bright_limit)
                                } else {
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_91(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                {
                    (samples[8] < dark_limit)
                        && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                }
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_92(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] > bright_limit {
            {
                (samples[9] >= dark_limit)
                    && ((samples[9] > bright_limit)
                        && (Self::score_classify_branch_90(samples, bright_limit, dark_limit)))
            }
        } else {
            {
                (samples[9] < dark_limit)
                    && ({
                        (samples[7] < dark_limit)
                            && (if samples[1] > bright_limit {
                                {
                                    (samples[6] <= bright_limit)
                                        && ((samples[6] < dark_limit)
                                            && ({
                                                (samples[8] < dark_limit)
                                                    && ({
                                                        (samples[10] < dark_limit)
                                                            && (samples[11] < dark_limit)
                                                    })
                                            }))
                                }
                            } else {
                                Self::score_classify_branch_91(samples, bright_limit, dark_limit)
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_93(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                {
                    (samples[8] < dark_limit)
                        && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                }
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_94(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[1] < dark_limit)
                && ({
                    {
                        (samples[3] < dark_limit)
                            && ({
                                (samples[4] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_95(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({ (samples[3] < dark_limit) || (samples[8] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_96(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[1] < dark_limit)
                && ({
                    {
                        (samples[3] < dark_limit)
                            && ({
                                (samples[4] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            })
                    }
                })
        }
    }

    #[inline]
    fn score_classify_branch_99(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[1] < dark_limit)
                && ({
                    {
                        (samples[3] < dark_limit)
                            && ({
                                (samples[4] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_100(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                {
                    (samples[10] < dark_limit)
                        && ({
                            (samples[11] < dark_limit)
                                && ({ (samples[3] < dark_limit) || (samples[8] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_101(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[1] < dark_limit)
                && ({
                    {
                        (samples[3] < dark_limit)
                            && ({
                                (samples[4] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_102(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({ (samples[3] < dark_limit) || (samples[8] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_103(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[1] < dark_limit)
                && ({
                    {
                        (samples[3] < dark_limit)
                            && ({
                                (samples[4] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_104(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] < dark_limit {
            if samples[9] < dark_limit {
                if samples[1] > bright_limit {
                    {
                        (samples[6] <= bright_limit)
                            && ((samples[6] < dark_limit)
                                && ({
                                    (samples[8] < dark_limit)
                                        && ({
                                            (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                        })
                                }))
                    }
                } else {
                    Self::score_classify_branch_100(samples, bright_limit, dark_limit)
                }
            } else {
                {
                    (samples[1] <= bright_limit)
                        && (Self::score_classify_branch_101(samples, dark_limit))
                }
            }
        } else {
            if samples[9] < dark_limit {
                {
                    (samples[1] <= bright_limit)
                        && ((samples[1] < dark_limit)
                            && (Self::score_classify_branch_102(samples, dark_limit)))
                }
            } else {
                {
                    (samples[1] <= bright_limit)
                        && (Self::score_classify_branch_103(samples, dark_limit))
                }
            }
        }
    }
    #[inline]
    fn score_classify_branch_105(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                {
                    (samples[8] < dark_limit)
                        && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                }
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_106(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            (samples[7] < dark_limit)
                && ({
                    (samples[9] <= bright_limit)
                        && ((samples[9] < dark_limit)
                            && (if samples[1] > bright_limit {
                                {
                                    (samples[6] <= bright_limit)
                                        && ((samples[6] < dark_limit)
                                            && ({
                                                (samples[8] < dark_limit)
                                                    && ({
                                                        (samples[10] < dark_limit)
                                                            && (samples[11] < dark_limit)
                                                    })
                                            }))
                                }
                            } else {
                                Self::score_classify_branch_105(samples, bright_limit, dark_limit)
                            }))
                })
        }
    }
    #[inline]
    fn score_classify_branch_107(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] < dark_limit {
            if samples[7] > bright_limit {
                if samples[9] > bright_limit {
                    {
                        (samples[1] <= bright_limit)
                            && (Self::score_classify_branch_94(samples, dark_limit))
                    }
                } else if samples[9] < dark_limit {
                    {
                        (samples[1] <= bright_limit)
                            && ((samples[1] < dark_limit)
                                && (Self::score_classify_branch_95(samples, dark_limit)))
                    }
                } else {
                    {
                        (samples[1] <= bright_limit)
                            && (Self::score_classify_branch_96(samples, dark_limit))
                    }
                }
            } else {
                if samples[9] > bright_limit {
                    {
                        (samples[1] <= bright_limit)
                            && (Self::score_classify_branch_99(samples, dark_limit))
                    }
                } else {
                    Self::score_classify_branch_104(samples, bright_limit, dark_limit)
                }
            }
        } else {
            Self::score_classify_branch_106(samples, bright_limit, dark_limit)
        }
    }
    #[inline]
    fn score_classify_branch_108(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] > bright_limit {
            {
                (samples[7] < dark_limit)
                    && ({
                        (samples[9] <= bright_limit)
                            && ((samples[9] < dark_limit)
                                && (if samples[1] > bright_limit {
                                    {
                                        (samples[6] <= bright_limit)
                                            && ((samples[6] < dark_limit)
                                                && ({
                                                    (samples[8] < dark_limit)
                                                        && ({
                                                            (samples[10] < dark_limit)
                                                                && (samples[11] < dark_limit)
                                                        })
                                                }))
                                    }
                                } else {
                                    Self::score_classify_branch_93(
                                        samples,
                                        bright_limit,
                                        dark_limit,
                                    )
                                }))
                    })
            }
        } else {
            Self::score_classify_branch_107(samples, bright_limit, dark_limit)
        }
    }
    #[inline]
    fn score_classify_branch_109(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            (samples[6] <= bright_limit)
                && ((samples[6] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && (if samples[4] < dark_limit {
                                (samples[3] < dark_limit) || ({ samples[10] < dark_limit })
                            } else {
                                (samples[10] < dark_limit) && ({ samples[11] < dark_limit })
                            })
                    }))
        }
    }

    #[inline]
    fn score_classify_branch_112(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            (samples[6] <= bright_limit)
                && ((samples[6] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && (if samples[4] < dark_limit {
                                (samples[3] < dark_limit) || ({ samples[10] < dark_limit })
                            } else {
                                (samples[10] < dark_limit) && ({ samples[11] < dark_limit })
                            })
                    }))
        }
    }
    #[inline]
    fn score_classify_branch_113(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({ (samples[3] < dark_limit) && (samples[4] < dark_limit) }))
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[3] < dark_limit)
                                && ({ (samples[4] < dark_limit) && (samples[8] < dark_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_114(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[6] < dark_limit)
                && (if samples[4] < dark_limit {
                    {
                        (samples[3] < dark_limit)
                            || ((samples[8] < dark_limit) && ({ samples[10] < dark_limit }))
                    }
                } else {
                    {
                        (samples[8] < dark_limit)
                            && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_115(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                (samples[6] <= bright_limit)
                    && (Self::score_classify_branch_114(samples, dark_limit))
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && (if samples[4] < dark_limit {
                                    (samples[3] < dark_limit) || (samples[10] < dark_limit)
                                } else {
                                    (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_116(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && (if samples[4] < dark_limit {
                                    (samples[3] < dark_limit) || (samples[10] < dark_limit)
                                } else {
                                    (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                })
                        }))
            }
        } else {
            Self::score_classify_branch_115(samples, bright_limit, dark_limit)
        }
    }
    #[inline]
    fn score_classify_branch_117(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({ (samples[3] < dark_limit) && (samples[4] < dark_limit) }))
            }
        } else {
            {
                (samples[6] <= bright_limit)
                    && ((samples[6] < dark_limit)
                        && ({
                            (samples[3] < dark_limit)
                                && ({ (samples[4] < dark_limit) && (samples[8] < dark_limit) })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_118(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            (samples[6] <= bright_limit)
                && ((samples[6] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && (if samples[4] < dark_limit {
                                (samples[3] < dark_limit) || ({ samples[10] < dark_limit })
                            } else {
                                (samples[10] < dark_limit) && ({ samples[11] < dark_limit })
                            })
                    }))
        }
    }

    #[inline]
    fn score_classify_branch_121(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            (samples[6] <= bright_limit)
                && ((samples[6] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && (if samples[4] < dark_limit {
                                (samples[3] < dark_limit) || ({ samples[10] < dark_limit })
                            } else {
                                (samples[10] < dark_limit) && ({ samples[11] < dark_limit })
                            })
                    }))
        }
    }
    #[inline]
    fn score_classify_branch_122(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] > bright_limit {
            {
                (samples[9] <= bright_limit)
                    && ((samples[9] < dark_limit)
                        && (if samples[1] > bright_limit {
                            Self::score_classify_branch_109(samples, bright_limit, dark_limit)
                        } else {
                            Self::score_classify_branch_112(samples, bright_limit, dark_limit)
                        }))
            }
        } else if samples[2] < dark_limit {
            if samples[9] > bright_limit {
                Self::score_classify_branch_113(samples, bright_limit, dark_limit)
            } else if samples[9] < dark_limit {
                Self::score_classify_branch_116(samples, bright_limit, dark_limit)
            } else {
                Self::score_classify_branch_117(samples, bright_limit, dark_limit)
            }
        } else {
            {
                (samples[9] <= bright_limit)
                    && ((samples[9] < dark_limit)
                        && (if samples[1] > bright_limit {
                            Self::score_classify_branch_118(samples, bright_limit, dark_limit)
                        } else {
                            Self::score_classify_branch_121(samples, bright_limit, dark_limit)
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_123(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && (if samples[4] > bright_limit {
                                    (samples[3] > bright_limit) || (samples[10] > bright_limit)
                                } else {
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_124(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && (if samples[4] > bright_limit {
                            {
                                (samples[3] > bright_limit)
                                    || ((samples[8] > bright_limit) && (samples[10] > bright_limit))
                            }
                        } else {
                            {
                                (samples[8] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            }
                        }))
            }
        } else {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && (if samples[4] > bright_limit {
                                    (samples[3] > bright_limit) || (samples[10] > bright_limit)
                                } else {
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_125(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] > bright_limit {
            if samples[1] < dark_limit {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({
                                (samples[8] > bright_limit)
                                    && (if samples[4] > bright_limit {
                                        {
                                            (samples[3] > bright_limit)
                                                || (samples[10] > bright_limit)
                                        }
                                    } else {
                                        {
                                            (samples[10] > bright_limit)
                                                && (samples[11] > bright_limit)
                                        }
                                    })
                            }))
                }
            } else {
                Self::score_classify_branch_124(samples, bright_limit, dark_limit)
            }
        } else {
            if samples[1] > bright_limit {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({ (samples[3] > bright_limit) && (samples[4] > bright_limit) }))
                }
            } else {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({
                                (samples[3] > bright_limit)
                                    && ({
                                        (samples[4] > bright_limit) && (samples[8] > bright_limit)
                                    })
                            }))
                }
            }
        }
    }
    #[inline]
    fn score_classify_branch_126(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] < dark_limit {
            if samples[1] > bright_limit {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({ (samples[3] > bright_limit) && (samples[4] > bright_limit) }))
                }
            } else {
                {
                    (samples[6] >= dark_limit)
                        && ((samples[6] > bright_limit)
                            && ({
                                (samples[3] > bright_limit)
                                    && ({
                                        (samples[4] > bright_limit) && (samples[8] > bright_limit)
                                    })
                            }))
                }
            }
        } else {
            Self::score_classify_branch_125(samples, bright_limit, dark_limit)
        }
    }
    #[inline]
    fn score_classify_branch_127(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        {
            {
                (samples[6] >= dark_limit)
                    && ((samples[6] > bright_limit)
                        && ({
                            (samples[8] > bright_limit)
                                && (if samples[4] > bright_limit {
                                    (samples[3] > bright_limit) || (samples[10] > bright_limit)
                                } else {
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                })
                        }))
            }
        }
    }
    #[inline]
    fn score_classify_branch_128(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[5] < dark_limit {
            {
                (samples[7] <= bright_limit)
                    && ((samples[7] < dark_limit)
                        && (Self::score_classify_branch_122(samples, bright_limit, dark_limit)))
            }
        } else {
            {
                (samples[5] > bright_limit)
                    && ({
                        (samples[7] > bright_limit)
                            && (if samples[2] < dark_limit {
                                {
                                    (samples[9] >= dark_limit)
                                        && ((samples[9] > bright_limit)
                                            && (Self::score_classify_branch_123(
                                                samples,
                                                bright_limit,
                                                dark_limit,
                                            )))
                                }
                            } else if samples[2] > bright_limit {
                                Self::score_classify_branch_126(samples, bright_limit, dark_limit)
                            } else {
                                (samples[9] >= dark_limit)
                                    && ((samples[9] > bright_limit)
                                        && (Self::score_classify_branch_127(
                                            samples,
                                            bright_limit,
                                            dark_limit,
                                        )))
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_129(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[0] > bright_limit {
            if samples[5] > bright_limit {
                if samples[2] < dark_limit {
                    {
                        (samples[7] > bright_limit)
                            && ({
                                (samples[9] >= dark_limit)
                                    && ((samples[9] > bright_limit)
                                        && (Self::score_classify_branch_1(
                                            samples,
                                            bright_limit,
                                            dark_limit,
                                        )))
                            })
                    }
                } else if samples[2] > bright_limit {
                    Self::score_classify_branch_12(samples, bright_limit, dark_limit)
                } else {
                    (samples[7] > bright_limit)
                        && (Self::score_classify_branch_15(samples, bright_limit, dark_limit))
                }
            } else {
                if samples[5] < dark_limit {
                    Self::score_classify_branch_38(samples, bright_limit, dark_limit)
                } else {
                    Self::score_classify_branch_53(samples, bright_limit, dark_limit)
                }
            }
        } else {
            if samples[0] < dark_limit {
                if samples[5] < dark_limit {
                    if samples[9] > bright_limit {
                        Self::score_classify_branch_56(samples, bright_limit, dark_limit)
                    } else {
                        Self::score_classify_branch_71(samples, bright_limit, dark_limit)
                    }
                } else {
                    if samples[5] > bright_limit {
                        if samples[2] > bright_limit {
                            Self::score_classify_branch_76(samples, bright_limit, dark_limit)
                        } else if samples[2] < dark_limit {
                            Self::score_classify_branch_89(samples, bright_limit, dark_limit)
                        } else {
                            Self::score_classify_branch_92(samples, bright_limit, dark_limit)
                        }
                    } else {
                        Self::score_classify_branch_108(samples, bright_limit, dark_limit)
                    }
                }
            } else {
                Self::score_classify_branch_128(samples, bright_limit, dark_limit)
            }
        }
    }
}
