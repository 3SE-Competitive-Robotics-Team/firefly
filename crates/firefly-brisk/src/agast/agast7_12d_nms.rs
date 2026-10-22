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

//! Bisection corner score for agast7-12d.
use crate::agast::{
    agast7_12d::AgastDetector712d,
    ast_detector::{checked_image, neighborhood},
};
use firefly_error::Result;
impl AgastDetector712d {
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
        Self::score_classify_branch_26(samples, bright_limit, dark_limit)
    }
    #[inline]
    fn score_classify_branch_0(samples: &[i32; 12], bright_limit: i32) -> bool {
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
                                && ({
                                    (samples[11] > bright_limit)
                                        || ((samples[4] > bright_limit)
                                            && (samples[7] > bright_limit))
                                })
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
    fn score_classify_branch_1(samples: &[i32; 12], bright_limit: i32) -> bool {
        if samples[9] > bright_limit {
            if samples[1] > bright_limit {
                Self::score_classify_branch_0(samples, bright_limit)
            } else {
                (samples[6] > bright_limit)
                    && ({
                        (samples[7] > bright_limit)
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
                            })
                    })
            }
        } else {
            {
                (samples[3] > bright_limit)
                    && ({
                        (samples[4] > bright_limit)
                            && (if samples[1] > bright_limit {
                                (samples[6] > bright_limit) || (samples[11] > bright_limit)
                            } else {
                                {
                                    (samples[6] > bright_limit)
                                        && ({
                                            (samples[7] > bright_limit)
                                                && (samples[8] > bright_limit)
                                        })
                                }
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_2(samples: &[i32; 12], bright_limit: i32) -> bool {
        if samples[1] > bright_limit {
            if samples[10] > bright_limit {
                {
                    (samples[11] > bright_limit)
                        || ((samples[6] > bright_limit) && (samples[4] > bright_limit))
                }
            } else {
                {
                    (samples[6] > bright_limit)
                        && ({ (samples[3] > bright_limit) && (samples[4] > bright_limit) })
                }
            }
        } else {
            {
                (samples[6] > bright_limit)
                    && (if samples[4] > bright_limit {
                        (samples[3] > bright_limit) || (samples[10] > bright_limit)
                    } else {
                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_3(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[1] > bright_limit {
            if samples[8] > bright_limit {
                {
                    (samples[10] > bright_limit)
                        && ({ (samples[2] > bright_limit) || (samples[7] > bright_limit) })
                }
            } else {
                {
                    (samples[6] < dark_limit)
                        && ({
                            (samples[2] < dark_limit)
                                && ({ (samples[7] < dark_limit) && (samples[8] < dark_limit) })
                        })
                }
            }
        } else {
            if samples[6] > bright_limit {
                {
                    (samples[7] > bright_limit)
                        && ({ (samples[8] > bright_limit) && (samples[10] > bright_limit) })
                }
            } else {
                {
                    (samples[6] < dark_limit)
                        && ({
                            (samples[2] < dark_limit)
                                && ({
                                    (samples[7] < dark_limit)
                                        && ({
                                            (samples[1] < dark_limit) || (samples[8] < dark_limit)
                                        })
                                })
                        })
                }
            }
        }
    }
    #[inline]
    fn score_classify_branch_4(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[4] < dark_limit {
            if samples[11] > bright_limit {
                Self::score_classify_branch_3(samples, bright_limit, dark_limit)
            } else {
                (samples[2] < dark_limit)
                    && ({
                        (samples[7] < dark_limit)
                            && (if samples[1] < dark_limit {
                                samples[6] < dark_limit
                            } else {
                                (samples[6] < dark_limit) && (samples[8] < dark_limit)
                            })
                    })
            }
        } else {
            {
                (samples[11] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && ({
                                (samples[10] > bright_limit)
                                    && (if samples[1] > bright_limit {
                                        {
                                            (samples[2] > bright_limit)
                                                || (samples[7] > bright_limit)
                                        }
                                    } else {
                                        {
                                            (samples[6] > bright_limit)
                                                && (samples[7] > bright_limit)
                                        }
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_5(samples: &[i32; 12], bright_limit: i32) -> bool {
        if samples[3] > bright_limit {
            if samples[1] > bright_limit {
                {
                    (samples[2] > bright_limit)
                        || ((samples[7] > bright_limit) && (samples[8] > bright_limit))
                }
            } else {
                {
                    (samples[6] > bright_limit)
                        && ({ (samples[7] > bright_limit) && (samples[8] > bright_limit) })
                }
            }
        } else {
            {
                (samples[8] > bright_limit)
                    && (if samples[1] > bright_limit {
                        (samples[2] > bright_limit) || (samples[7] > bright_limit)
                    } else {
                        (samples[6] > bright_limit) && (samples[7] > bright_limit)
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_6(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[4] > bright_limit {
            if samples[10] > bright_limit {
                (samples[3] > bright_limit) && (samples[11] > bright_limit)
            } else {
                {
                    (samples[6] < dark_limit)
                        && ({
                            (samples[7] < dark_limit)
                                && ({
                                    (samples[8] < dark_limit)
                                        && ({
                                            (samples[11] < dark_limit) && (samples[10] < dark_limit)
                                        })
                                })
                        })
                }
            }
        } else {
            {
                (samples[6] < dark_limit)
                    && ({
                        (samples[7] < dark_limit)
                            && ({
                                (samples[8] < dark_limit)
                                    && (if samples[10] < dark_limit {
                                        (samples[4] < dark_limit) || (samples[11] < dark_limit)
                                    } else {
                                        (samples[3] < dark_limit) && (samples[4] < dark_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_7(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[6] < dark_limit)
                && ({
                    (samples[7] < dark_limit)
                        && (if samples[8] < dark_limit {
                            if samples[4] < dark_limit {
                                (samples[3] < dark_limit) || (samples[10] < dark_limit)
                            } else {
                                (samples[10] < dark_limit) && (samples[11] < dark_limit)
                            }
                        } else {
                            {
                                (samples[2] < dark_limit)
                                    && ({
                                        (samples[1] < dark_limit)
                                            && ({
                                                (samples[3] < dark_limit)
                                                    && (samples[4] < dark_limit)
                                            })
                                    })
                            }
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_8(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] > bright_limit {
            {
                (samples[1] > bright_limit)
                    && ({
                        (samples[3] > bright_limit)
                            && ({
                                (samples[4] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[2] < dark_limit)
                    && ({
                        (samples[3] < dark_limit)
                            && ({
                                (samples[4] < dark_limit)
                                    && ({
                                        (samples[7] < dark_limit)
                                            && (if samples[1] < dark_limit {
                                                samples[6] < dark_limit
                                            } else {
                                                (samples[6] < dark_limit)
                                                    && (samples[8] < dark_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_9(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] < dark_limit {
            if samples[2] > bright_limit {
                if samples[1] > bright_limit {
                    Self::score_classify_branch_6(samples, bright_limit, dark_limit)
                } else {
                    (samples[6] < dark_limit)
                        && ({
                            (samples[7] < dark_limit)
                                && ({
                                    (samples[8] < dark_limit)
                                        && (if samples[4] < dark_limit {
                                            {
                                                (samples[3] < dark_limit)
                                                    || (samples[10] < dark_limit)
                                            }
                                        } else {
                                            {
                                                (samples[10] < dark_limit)
                                                    && (samples[11] < dark_limit)
                                            }
                                        })
                                })
                        })
                }
            } else {
                Self::score_classify_branch_7(samples, dark_limit)
            }
        } else {
            Self::score_classify_branch_8(samples, bright_limit, dark_limit)
        }
    }
    #[inline]
    fn score_classify_branch_10(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            (samples[10] > bright_limit)
                && ({
                    (samples[11] > bright_limit)
                        && (if samples[9] > bright_limit {
                            if samples[1] > bright_limit {
                                (samples[3] > bright_limit) || (samples[8] > bright_limit)
                            } else {
                                {
                                    (samples[6] > bright_limit)
                                        && ({
                                            (samples[7] > bright_limit)
                                                && (samples[8] > bright_limit)
                                        })
                                }
                            }
                        } else {
                            {
                                (samples[1] > bright_limit)
                                    && ({
                                        (samples[3] > bright_limit) && (samples[4] > bright_limit)
                                    })
                            }
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_11(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[5] < dark_limit {
            if samples[9] > bright_limit {
                if samples[3] < dark_limit {
                    Self::score_classify_branch_4(samples, bright_limit, dark_limit)
                } else {
                    (samples[11] > bright_limit)
                        && ({
                            (samples[10] > bright_limit)
                                && (Self::score_classify_branch_5(samples, bright_limit))
                        })
                }
            } else {
                Self::score_classify_branch_9(samples, bright_limit, dark_limit)
            }
        } else {
            if samples[2] > bright_limit {
                Self::score_classify_branch_10(samples, bright_limit)
            } else {
                (samples[9] > bright_limit)
                    && ({
                        (samples[7] > bright_limit)
                            && ({
                                (samples[8] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit)
                                            && ({
                                                (samples[11] > bright_limit)
                                                    && ({
                                                        (samples[1] > bright_limit)
                                                            || (samples[6] > bright_limit)
                                                    })
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_12(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            (samples[6] > bright_limit)
                && (if samples[4] > bright_limit {
                    if samples[3] > bright_limit {
                        (samples[1] > bright_limit) || (samples[8] > bright_limit)
                    } else {
                        {
                            (samples[9] > bright_limit)
                                && ({ (samples[8] > bright_limit) && (samples[10] > bright_limit) })
                        }
                    }
                } else {
                    {
                        (samples[9] > bright_limit)
                            && ({
                                (samples[8] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit) && (samples[11] > bright_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_13(samples: &[i32; 12], dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            if samples[10] < dark_limit {
                {
                    (samples[11] < dark_limit)
                        || ((samples[6] < dark_limit) && (samples[4] < dark_limit))
                }
            } else {
                {
                    (samples[6] < dark_limit)
                        && ({ (samples[3] < dark_limit) && (samples[4] < dark_limit) })
                }
            }
        } else {
            {
                (samples[6] < dark_limit)
                    && (if samples[4] < dark_limit {
                        (samples[3] < dark_limit) || (samples[10] < dark_limit)
                    } else {
                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_14(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[5] > bright_limit {
            if samples[7] > bright_limit {
                Self::score_classify_branch_12(samples, bright_limit)
            } else {
                (samples[9] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && ({
                                (samples[10] < dark_limit)
                                    && ({
                                        (samples[11] < dark_limit)
                                            && ({
                                                (samples[7] < dark_limit)
                                                    && ({
                                                        (samples[1] < dark_limit)
                                                            || (samples[6] < dark_limit)
                                                    })
                                            })
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[9] < dark_limit)
                    && ({
                        (samples[7] < dark_limit)
                            && ({
                                (samples[8] < dark_limit)
                                    && (if samples[5] < dark_limit {
                                        Self::score_classify_branch_13(samples, dark_limit)
                                    } else {
                                        (samples[10] < dark_limit)
                                            && ({
                                                (samples[11] < dark_limit)
                                                    && ({
                                                        (samples[1] < dark_limit)
                                                            || (samples[6] < dark_limit)
                                                    })
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_15(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[4] < dark_limit {
            if samples[10] < dark_limit {
                (samples[3] < dark_limit) && (samples[11] < dark_limit)
            } else {
                {
                    (samples[6] > bright_limit)
                        && ({
                            (samples[7] > bright_limit)
                                && ({
                                    (samples[8] > bright_limit)
                                        && ({
                                            (samples[11] > bright_limit)
                                                && (samples[10] > bright_limit)
                                        })
                                })
                        })
                }
            }
        } else {
            {
                (samples[6] > bright_limit)
                    && ({
                        (samples[7] > bright_limit)
                            && ({
                                (samples[8] > bright_limit)
                                    && (if samples[10] > bright_limit {
                                        {
                                            (samples[4] > bright_limit)
                                                || (samples[11] > bright_limit)
                                        }
                                    } else {
                                        {
                                            (samples[3] > bright_limit)
                                                && (samples[4] > bright_limit)
                                        }
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_16(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[3] < dark_limit)
                && ({
                    (samples[4] < dark_limit)
                        && (if samples[5] < dark_limit {
                            if samples[1] < dark_limit {
                                (samples[6] < dark_limit) || (samples[11] < dark_limit)
                            } else {
                                {
                                    (samples[6] < dark_limit)
                                        && ({
                                            (samples[7] < dark_limit) && (samples[8] < dark_limit)
                                        })
                                }
                            }
                        } else {
                            {
                                (samples[1] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            }
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_17(samples: &[i32; 12], dark_limit: i32) -> bool {
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
                                && ({
                                    (samples[11] < dark_limit)
                                        || ((samples[4] < dark_limit) && (samples[7] < dark_limit))
                                })
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
    fn score_classify_branch_18(samples: &[i32; 12], dark_limit: i32) -> bool {
        if samples[5] < dark_limit {
            if samples[1] < dark_limit {
                Self::score_classify_branch_17(samples, dark_limit)
            } else {
                (samples[6] < dark_limit)
                    && ({
                        (samples[7] < dark_limit)
                            && ({
                                (samples[8] < dark_limit)
                                    && (if samples[4] < dark_limit {
                                        (samples[3] < dark_limit) || (samples[10] < dark_limit)
                                    } else {
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && (if samples[1] < dark_limit {
                                (samples[3] < dark_limit) || (samples[8] < dark_limit)
                            } else {
                                {
                                    (samples[6] < dark_limit)
                                        && ({
                                            (samples[7] < dark_limit) && (samples[8] < dark_limit)
                                        })
                                }
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_19(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[3] < dark_limit)
                && ({
                    (samples[4] < dark_limit)
                        && (if samples[5] < dark_limit {
                            if samples[1] < dark_limit {
                                (samples[6] < dark_limit) || (samples[11] < dark_limit)
                            } else {
                                {
                                    (samples[6] < dark_limit)
                                        && ({
                                            (samples[7] < dark_limit) && (samples[8] < dark_limit)
                                        })
                                }
                            }
                        } else {
                            {
                                (samples[1] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                                    })
                            }
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_20(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] > bright_limit {
            if samples[5] > bright_limit {
                if samples[1] < dark_limit {
                    Self::score_classify_branch_15(samples, bright_limit, dark_limit)
                } else {
                    (samples[6] > bright_limit)
                        && ({
                            (samples[7] > bright_limit)
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
                                })
                        })
                }
            } else {
                Self::score_classify_branch_16(samples, dark_limit)
            }
        } else {
            if samples[9] < dark_limit {
                Self::score_classify_branch_18(samples, dark_limit)
            } else {
                Self::score_classify_branch_19(samples, dark_limit)
            }
        }
    }
    #[inline]
    fn score_classify_branch_21(samples: &[i32; 12], dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            if samples[10] < dark_limit {
                {
                    (samples[11] < dark_limit)
                        || ((samples[6] < dark_limit) && (samples[4] < dark_limit))
                }
            } else {
                {
                    (samples[6] < dark_limit)
                        && ({ (samples[3] < dark_limit) && (samples[4] < dark_limit) })
                }
            }
        } else {
            {
                (samples[6] < dark_limit)
                    && (if samples[4] < dark_limit {
                        (samples[3] < dark_limit) || (samples[10] < dark_limit)
                    } else {
                        (samples[10] < dark_limit) && (samples[11] < dark_limit)
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_22(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] < dark_limit {
            {
                (samples[7] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && (if samples[5] < dark_limit {
                                Self::score_classify_branch_21(samples, dark_limit)
                            } else {
                                (samples[10] < dark_limit)
                                    && ({
                                        (samples[11] < dark_limit)
                                            && ({
                                                (samples[1] < dark_limit)
                                                    || (samples[6] < dark_limit)
                                            })
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[5] > bright_limit)
                    && ({
                        (samples[9] > bright_limit)
                            && ({
                                (samples[6] > bright_limit)
                                    && ({
                                        (samples[7] > bright_limit)
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
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_23(samples: &[i32; 12], bright_limit: i32) -> bool {
        {
            (samples[6] > bright_limit)
                && ({
                    (samples[7] > bright_limit)
                        && (if samples[4] > bright_limit {
                            if samples[3] > bright_limit {
                                {
                                    (samples[8] > bright_limit)
                                        || ((samples[1] > bright_limit)
                                            && (samples[2] > bright_limit))
                                }
                            } else {
                                (samples[8] > bright_limit) && (samples[10] > bright_limit)
                            }
                        } else {
                            {
                                (samples[11] > bright_limit)
                                    && ({
                                        (samples[8] > bright_limit) && (samples[10] > bright_limit)
                                    })
                            }
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_24(samples: &[i32; 12], dark_limit: i32) -> bool {
        {
            (samples[6] < dark_limit)
                && ({
                    (samples[7] < dark_limit)
                        && (if samples[4] < dark_limit {
                            if samples[3] < dark_limit {
                                {
                                    (samples[8] < dark_limit)
                                        || ((samples[1] < dark_limit) && (samples[2] < dark_limit))
                                }
                            } else {
                                (samples[8] < dark_limit) && (samples[10] < dark_limit)
                            }
                        } else {
                            {
                                (samples[11] < dark_limit)
                                    && ({ (samples[8] < dark_limit) && (samples[10] < dark_limit) })
                            }
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_25(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[5] > bright_limit {
            if samples[9] > bright_limit {
                Self::score_classify_branch_23(samples, bright_limit)
            } else {
                (samples[2] > bright_limit)
                    && ({
                        (samples[3] > bright_limit)
                            && ({
                                (samples[4] > bright_limit)
                                    && ({
                                        (samples[7] > bright_limit)
                                            && (if samples[1] > bright_limit {
                                                samples[6] > bright_limit
                                            } else {
                                                (samples[6] > bright_limit)
                                                    && (samples[8] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[5] < dark_limit)
                    && (if samples[9] < dark_limit {
                        Self::score_classify_branch_24(samples, dark_limit)
                    } else {
                        (samples[2] < dark_limit)
                            && ({
                                (samples[3] < dark_limit)
                                    && ({
                                        (samples[4] < dark_limit)
                                            && ({
                                                (samples[7] < dark_limit)
                                                    && (if samples[1] < dark_limit {
                                                        samples[6] < dark_limit
                                                    } else {
                                                        (samples[6] < dark_limit)
                                                            && (samples[8] < dark_limit)
                                                    })
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_26(samples: &[i32; 12], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[0] > bright_limit {
            if samples[5] > bright_limit {
                if samples[2] > bright_limit {
                    Self::score_classify_branch_1(samples, bright_limit)
                } else {
                    (samples[9] > bright_limit)
                        && ({
                            (samples[7] > bright_limit)
                                && ({
                                    (samples[8] > bright_limit)
                                        && (Self::score_classify_branch_2(samples, bright_limit))
                                })
                        })
                }
            } else {
                Self::score_classify_branch_11(samples, bright_limit, dark_limit)
            }
        } else {
            if samples[0] < dark_limit {
                if samples[2] > bright_limit {
                    Self::score_classify_branch_14(samples, bright_limit, dark_limit)
                } else if samples[2] < dark_limit {
                    Self::score_classify_branch_20(samples, bright_limit, dark_limit)
                } else {
                    Self::score_classify_branch_22(samples, bright_limit, dark_limit)
                }
            } else {
                Self::score_classify_branch_25(samples, bright_limit, dark_limit)
            }
        }
    }
}
