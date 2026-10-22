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

//! Bisection corner score for oast9-16.
use crate::agast::{
    ast_detector::{checked_image, neighborhood},
    oast9_16::OastDetector916,
};
use firefly_error::Result;
impl OastDetector916 {
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
    fn score_values(&self, samples: &[i32; 16], center: i32) -> Result<i32> {
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
    fn score_classify(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        Self::score_classify_branch_55(samples, bright_limit, dark_limit)
    }
    #[inline]
    fn score_classify_branch_0(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[1] > bright_limit {
            if samples[6] > bright_limit {
                (samples[8] > bright_limit) || (samples[15] > bright_limit)
            } else {
                {
                    (samples[13] > bright_limit)
                        && ({ (samples[14] > bright_limit) && (samples[15] > bright_limit) })
                }
            }
        } else {
            {
                (samples[8] > bright_limit)
                    && ({
                        (samples[9] > bright_limit)
                            && ({
                                (samples[10] > bright_limit)
                                    && ({
                                        (samples[6] > bright_limit)
                                            || ((samples[11] > bright_limit)
                                                && ({
                                                    (samples[12] > bright_limit)
                                                        && ({
                                                            (samples[13] > bright_limit)
                                                                && ({
                                                                    (samples[14] > bright_limit)
                                                                        && (samples[15]
                                                                            > bright_limit)
                                                                })
                                                        })
                                                }))
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_1(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[8] > bright_limit {
            if samples[9] > bright_limit {
                {
                    (samples[6] > bright_limit)
                        || ((samples[13] > bright_limit)
                            && ({ (samples[14] > bright_limit) && (samples[15] > bright_limit) }))
                }
            } else {
                {
                    (samples[1] > bright_limit)
                        && ({
                            (samples[13] > bright_limit)
                                && ({
                                    (samples[14] > bright_limit) && (samples[15] > bright_limit)
                                })
                        })
                }
            }
        } else {
            {
                (samples[1] > bright_limit)
                    && ({
                        (samples[13] > bright_limit)
                            && ({ (samples[14] > bright_limit) && (samples[15] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_2(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[1] > bright_limit {
            if samples[3] > bright_limit {
                (samples[6] > bright_limit) || (samples[13] > bright_limit)
            } else {
                {
                    (samples[10] > bright_limit)
                        && ({
                            (samples[11] > bright_limit)
                                && ({
                                    (samples[12] > bright_limit) && (samples[13] > bright_limit)
                                })
                        })
                }
            }
        } else {
            {
                (samples[8] > bright_limit)
                    && ({
                        (samples[9] > bright_limit)
                            && ({
                                (samples[10] > bright_limit)
                                    && ({
                                        (samples[11] > bright_limit)
                                            && ({
                                                (samples[12] > bright_limit)
                                                    && (samples[13] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_3(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[14] > bright_limit {
            {
                (samples[15] > bright_limit)
                    && (Self::score_classify_branch_2(samples, bright_limit))
            }
        } else {
            {
                (samples[14] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && ({
                                (samples[9] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit)
                                            && ({
                                                (samples[11] < dark_limit)
                                                    && ({
                                                        (samples[12] < dark_limit)
                                                            && ({
                                                                (samples[13] < dark_limit)
                                                                    && ({
                                                                        (samples[6] < dark_limit)
                                                                            || (samples[15]
                                                                                < dark_limit)
                                                                    })
                                                            })
                                                    })
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_4(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[1] > bright_limit {
            if samples[3] > bright_limit {
                (samples[6] > bright_limit) || (samples[13] > bright_limit)
            } else {
                {
                    (samples[10] > bright_limit)
                        && ({
                            (samples[11] > bright_limit)
                                && ({
                                    (samples[12] > bright_limit) && (samples[13] > bright_limit)
                                })
                        })
                }
            }
        } else {
            {
                (samples[8] > bright_limit)
                    && ({
                        (samples[9] > bright_limit)
                            && ({
                                (samples[10] > bright_limit)
                                    && ({
                                        (samples[11] > bright_limit)
                                            && ({
                                                (samples[12] > bright_limit)
                                                    && (samples[13] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_5(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] > bright_limit {
            if samples[3] > bright_limit {
                Self::score_classify_branch_0(samples, bright_limit)
            } else {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({
                                (samples[12] > bright_limit)
                                    && (Self::score_classify_branch_1(samples, bright_limit))
                            })
                    })
            }
        } else {
            if samples[7] < dark_limit {
                Self::score_classify_branch_3(samples, bright_limit, dark_limit)
            } else {
                (samples[14] > bright_limit)
                    && ({
                        (samples[15] > bright_limit)
                            && (Self::score_classify_branch_4(samples, bright_limit))
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_6(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[15] > bright_limit {
            if samples[1] > bright_limit {
                {
                    (samples[3] > bright_limit)
                        || ((samples[10] > bright_limit) && (samples[11] > bright_limit))
                }
            } else {
                {
                    (samples[8] > bright_limit)
                        && ({
                            (samples[9] > bright_limit)
                                && ({
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
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
                                    && ({
                                        (samples[9] > bright_limit)
                                            && ({
                                                (samples[10] > bright_limit)
                                                    && (samples[11] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_7(samples: &[i32; 16], dark_limit: i32) -> bool {
        {
            (samples[12] < dark_limit)
                && ({
                    (samples[7] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && ({
                                    (samples[9] < dark_limit)
                                        && ({
                                            (samples[10] < dark_limit)
                                                && ({
                                                    (samples[11] < dark_limit)
                                                        && ({
                                                            (samples[13] < dark_limit)
                                                                && ({
                                                                    (samples[6] < dark_limit)
                                                                        || ((samples[14]
                                                                            < dark_limit)
                                                                            && (samples[15]
                                                                                < dark_limit))
                                                                })
                                                        })
                                                })
                                        })
                                })
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_8(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[15] > bright_limit {
            if samples[1] > bright_limit {
                {
                    (samples[3] > bright_limit)
                        || ((samples[10] > bright_limit) && (samples[11] > bright_limit))
                }
            } else {
                {
                    (samples[8] > bright_limit)
                        && ({
                            (samples[9] > bright_limit)
                                && ({
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
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
                                    && ({
                                        (samples[9] > bright_limit)
                                            && ({
                                                (samples[10] > bright_limit)
                                                    && (samples[11] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_9(samples: &[i32; 16], dark_limit: i32) -> bool {
        {
            (samples[12] < dark_limit)
                && ({
                    (samples[7] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && ({
                                    (samples[9] < dark_limit)
                                        && ({
                                            (samples[10] < dark_limit)
                                                && ({
                                                    (samples[11] < dark_limit)
                                                        && ({
                                                            (samples[13] < dark_limit)
                                                                && ({
                                                                    (samples[14] < dark_limit)
                                                                        && ({
                                                                            (samples[6]
                                                                                < dark_limit)
                                                                                || (samples[15]
                                                                                    < dark_limit)
                                                                        })
                                                                })
                                                        })
                                                })
                                        })
                                })
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_10(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[5] > bright_limit {
            Self::score_classify_branch_5(samples, bright_limit, dark_limit)
        } else if samples[5] < dark_limit {
            if samples[12] > bright_limit {
                {
                    (samples[13] > bright_limit)
                        && ({
                            (samples[14] > bright_limit)
                                && (Self::score_classify_branch_6(samples, bright_limit))
                        })
                }
            } else {
                Self::score_classify_branch_7(samples, dark_limit)
            }
        } else {
            if samples[12] > bright_limit {
                {
                    (samples[13] > bright_limit)
                        && ({
                            (samples[14] > bright_limit)
                                && (Self::score_classify_branch_8(samples, bright_limit))
                        })
                }
            } else {
                Self::score_classify_branch_9(samples, dark_limit)
            }
        }
    }
    #[inline]
    fn score_classify_branch_11(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[14] > bright_limit {
            if samples[15] > bright_limit {
                {
                    (samples[1] > bright_limit)
                        || ((samples[8] > bright_limit) && (samples[9] > bright_limit))
                }
            } else {
                {
                    (samples[6] > bright_limit)
                        && ({
                            (samples[7] > bright_limit)
                                && ({ (samples[8] > bright_limit) && (samples[9] > bright_limit) })
                        })
                }
            }
        } else {
            {
                (samples[5] > bright_limit)
                    && ({
                        (samples[6] > bright_limit)
                            && ({
                                (samples[7] > bright_limit)
                                    && ({
                                        (samples[8] > bright_limit) && (samples[9] > bright_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_12(samples: &[i32; 16], dark_limit: i32) -> bool {
        {
            (samples[10] < dark_limit)
                && (if samples[6] < dark_limit {
                    if samples[5] < dark_limit {
                        (samples[3] < dark_limit) || (samples[12] < dark_limit)
                    } else {
                        {
                            (samples[12] < dark_limit)
                                && ({ (samples[13] < dark_limit) && (samples[14] < dark_limit) })
                        }
                    }
                } else {
                    {
                        (samples[12] < dark_limit)
                            && ({
                                (samples[13] < dark_limit)
                                    && ({
                                        (samples[14] < dark_limit) && (samples[15] < dark_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_13(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[11] > bright_limit {
            {
                (samples[12] > bright_limit)
                    && ({
                        (samples[13] > bright_limit)
                            && (if samples[10] > bright_limit {
                                Self::score_classify_branch_11(samples, bright_limit)
                            } else {
                                (samples[1] > bright_limit)
                                    && ({
                                        (samples[3] > bright_limit)
                                            && ({
                                                (samples[14] > bright_limit)
                                                    && (samples[15] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[11] < dark_limit)
                    && ({
                        (samples[7] < dark_limit)
                            && ({
                                (samples[8] < dark_limit)
                                    && ({
                                        (samples[9] < dark_limit)
                                            && (Self::score_classify_branch_12(samples, dark_limit))
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_14(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[14] > bright_limit {
            if samples[15] > bright_limit {
                {
                    (samples[1] > bright_limit)
                        || ((samples[8] > bright_limit) && (samples[9] > bright_limit))
                }
            } else {
                {
                    (samples[6] > bright_limit)
                        && ({
                            (samples[7] > bright_limit)
                                && ({ (samples[8] > bright_limit) && (samples[9] > bright_limit) })
                        })
                }
            }
        } else {
            {
                (samples[5] > bright_limit)
                    && ({
                        (samples[6] > bright_limit)
                            && ({
                                (samples[7] > bright_limit)
                                    && ({
                                        (samples[8] > bright_limit) && (samples[9] > bright_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_15(samples: &[i32; 16], dark_limit: i32) -> bool {
        {
            (samples[7] < dark_limit)
                && ({
                    (samples[8] < dark_limit)
                        && ({
                            (samples[9] < dark_limit)
                                && ({
                                    (samples[10] < dark_limit)
                                        && ({
                                            (samples[12] < dark_limit)
                                                && ({
                                                    (samples[13] < dark_limit)
                                                        && (if samples[6] < dark_limit {
                                                            {
                                                                (samples[5] < dark_limit)
                                                                    || (samples[14] < dark_limit)
                                                            }
                                                        } else {
                                                            {
                                                                (samples[14] < dark_limit)
                                                                    && (samples[15] < dark_limit)
                                                            }
                                                        })
                                                })
                                        })
                                })
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_16(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[11] > bright_limit {
            {
                (samples[12] > bright_limit)
                    && ({
                        (samples[13] > bright_limit)
                            && (if samples[10] > bright_limit {
                                Self::score_classify_branch_14(samples, bright_limit)
                            } else {
                                (samples[1] > bright_limit)
                                    && ({
                                        (samples[3] > bright_limit)
                                            && ({
                                                (samples[14] > bright_limit)
                                                    && (samples[15] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        } else {
            (samples[11] < dark_limit) && (Self::score_classify_branch_15(samples, dark_limit))
        }
    }
    #[inline]
    fn score_classify_branch_17(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[13] > bright_limit {
            if samples[14] > bright_limit {
                {
                    (samples[15] > bright_limit)
                        || ((samples[6] > bright_limit) && (samples[7] > bright_limit))
                }
            } else {
                {
                    (samples[5] > bright_limit)
                        && ({ (samples[6] > bright_limit) && (samples[7] > bright_limit) })
                }
            }
        } else {
            {
                (samples[4] > bright_limit)
                    && ({
                        (samples[5] > bright_limit)
                            && ({ (samples[6] > bright_limit) && (samples[7] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_18(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[8] > bright_limit {
            if samples[12] > bright_limit {
                Self::score_classify_branch_17(samples, bright_limit)
            } else {
                (samples[3] > bright_limit)
                    && ({
                        (samples[4] > bright_limit)
                            && ({
                                (samples[5] > bright_limit)
                                    && ({
                                        (samples[6] > bright_limit) && (samples[7] > bright_limit)
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[1] > bright_limit)
                    && ({
                        (samples[12] > bright_limit)
                            && ({
                                (samples[13] > bright_limit)
                                    && ({
                                        (samples[14] > bright_limit) && (samples[15] > bright_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_19(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[5] < dark_limit {
            if samples[4] < dark_limit {
                if samples[3] < dark_limit {
                    (samples[1] < dark_limit) || (samples[10] < dark_limit)
                } else {
                    {
                        (samples[10] < dark_limit)
                            && ({ (samples[11] < dark_limit) && (samples[12] < dark_limit) })
                    }
                }
            } else {
                {
                    (samples[10] < dark_limit)
                        && ({
                            (samples[11] < dark_limit)
                                && ({ (samples[12] < dark_limit) && (samples[13] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({
                                (samples[12] < dark_limit)
                                    && ({
                                        (samples[13] < dark_limit) && (samples[14] < dark_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_20(samples: &[i32; 16], dark_limit: i32) -> bool {
        {
            (samples[7] < dark_limit)
                && ({
                    (samples[8] < dark_limit)
                        && (if samples[6] < dark_limit {
                            Self::score_classify_branch_19(samples, dark_limit)
                        } else {
                            (samples[10] < dark_limit)
                                && ({
                                    (samples[11] < dark_limit)
                                        && ({
                                            (samples[12] < dark_limit)
                                                && ({
                                                    (samples[13] < dark_limit)
                                                        && ({
                                                            (samples[14] < dark_limit)
                                                                && (samples[15] < dark_limit)
                                                        })
                                                })
                                        })
                                })
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_21(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[13] > bright_limit {
            if samples[14] > bright_limit {
                {
                    (samples[15] > bright_limit)
                        || ((samples[6] > bright_limit) && (samples[7] > bright_limit))
                }
            } else {
                {
                    (samples[5] > bright_limit)
                        && ({ (samples[6] > bright_limit) && (samples[7] > bright_limit) })
                }
            }
        } else {
            {
                (samples[4] > bright_limit)
                    && ({
                        (samples[5] > bright_limit)
                            && ({ (samples[6] > bright_limit) && (samples[7] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_22(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[8] > bright_limit {
            if samples[12] > bright_limit {
                Self::score_classify_branch_21(samples, bright_limit)
            } else {
                (samples[3] > bright_limit)
                    && ({
                        (samples[4] > bright_limit)
                            && ({
                                (samples[5] > bright_limit)
                                    && ({
                                        (samples[6] > bright_limit) && (samples[7] > bright_limit)
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[1] > bright_limit)
                    && ({
                        (samples[12] > bright_limit)
                            && ({
                                (samples[13] > bright_limit)
                                    && ({
                                        (samples[14] > bright_limit) && (samples[15] > bright_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_23(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[6] < dark_limit {
            if samples[5] < dark_limit {
                if samples[4] < dark_limit {
                    (samples[3] < dark_limit) || (samples[12] < dark_limit)
                } else {
                    (samples[12] < dark_limit) && (samples[13] < dark_limit)
                }
            } else {
                {
                    (samples[12] < dark_limit)
                        && ({ (samples[13] < dark_limit) && (samples[14] < dark_limit) })
                }
            }
        } else {
            {
                (samples[12] < dark_limit)
                    && ({
                        (samples[13] < dark_limit)
                            && ({ (samples[14] < dark_limit) && (samples[15] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_24(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] > bright_limit {
            {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && (Self::score_classify_branch_22(samples, bright_limit))
                    })
            }
        } else {
            {
                (samples[9] < dark_limit)
                    && ({
                        (samples[7] < dark_limit)
                            && ({
                                (samples[8] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit)
                                            && ({
                                                (samples[11] < dark_limit)
                                                    && (Self::score_classify_branch_23(
                                                        samples, dark_limit,
                                                    ))
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_25(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] > bright_limit {
            if samples[4] > bright_limit {
                Self::score_classify_branch_10(samples, bright_limit, dark_limit)
            } else if samples[4] < dark_limit {
                Self::score_classify_branch_13(samples, bright_limit, dark_limit)
            } else {
                Self::score_classify_branch_16(samples, bright_limit, dark_limit)
            }
        } else {
            if samples[2] < dark_limit {
                if samples[9] > bright_limit {
                    {
                        (samples[10] > bright_limit)
                            && ({
                                (samples[11] > bright_limit)
                                    && (Self::score_classify_branch_18(samples, bright_limit))
                            })
                    }
                } else {
                    {
                        (samples[9] < dark_limit)
                            && (Self::score_classify_branch_20(samples, dark_limit))
                    }
                }
            } else {
                Self::score_classify_branch_24(samples, bright_limit, dark_limit)
            }
        }
    }
    #[inline]
    fn score_classify_branch_26(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[5] > bright_limit {
            if samples[4] > bright_limit {
                if samples[3] > bright_limit {
                    (samples[1] > bright_limit) || (samples[10] > bright_limit)
                } else {
                    {
                        (samples[10] > bright_limit)
                            && ({ (samples[11] > bright_limit) && (samples[12] > bright_limit) })
                    }
                }
            } else {
                {
                    (samples[10] > bright_limit)
                        && ({
                            (samples[11] > bright_limit)
                                && ({
                                    (samples[12] > bright_limit) && (samples[13] > bright_limit)
                                })
                        })
                }
            }
        } else {
            {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({
                                (samples[12] > bright_limit)
                                    && ({
                                        (samples[13] > bright_limit) && (samples[14] > bright_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_27(samples: &[i32; 16], bright_limit: i32) -> bool {
        {
            (samples[7] > bright_limit)
                && ({
                    (samples[8] > bright_limit)
                        && (if samples[6] > bright_limit {
                            Self::score_classify_branch_26(samples, bright_limit)
                        } else {
                            (samples[10] > bright_limit)
                                && ({
                                    (samples[11] > bright_limit)
                                        && ({
                                            (samples[12] > bright_limit)
                                                && ({
                                                    (samples[13] > bright_limit)
                                                        && ({
                                                            (samples[14] > bright_limit)
                                                                && (samples[15] > bright_limit)
                                                        })
                                                })
                                        })
                                })
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_28(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[13] < dark_limit {
            if samples[14] < dark_limit {
                {
                    (samples[15] < dark_limit)
                        || ((samples[6] < dark_limit) && (samples[7] < dark_limit))
                }
            } else {
                {
                    (samples[5] < dark_limit)
                        && ({ (samples[6] < dark_limit) && (samples[7] < dark_limit) })
                }
            }
        } else {
            {
                (samples[4] < dark_limit)
                    && ({
                        (samples[5] < dark_limit)
                            && ({ (samples[6] < dark_limit) && (samples[7] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_29(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[8] < dark_limit {
            if samples[12] < dark_limit {
                Self::score_classify_branch_28(samples, dark_limit)
            } else {
                (samples[3] < dark_limit)
                    && ({
                        (samples[4] < dark_limit)
                            && ({
                                (samples[5] < dark_limit)
                                    && ({ (samples[6] < dark_limit) && (samples[7] < dark_limit) })
                            })
                    })
            }
        } else {
            {
                (samples[1] < dark_limit)
                    && ({
                        (samples[12] < dark_limit)
                            && ({
                                (samples[13] < dark_limit)
                                    && ({
                                        (samples[14] < dark_limit) && (samples[15] < dark_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_30(samples: &[i32; 16], bright_limit: i32) -> bool {
        {
            (samples[10] > bright_limit)
                && (if samples[6] > bright_limit {
                    if samples[5] > bright_limit {
                        (samples[3] > bright_limit) || (samples[12] > bright_limit)
                    } else {
                        {
                            (samples[12] > bright_limit)
                                && ({
                                    (samples[13] > bright_limit) && (samples[14] > bright_limit)
                                })
                        }
                    }
                } else {
                    {
                        (samples[12] > bright_limit)
                            && ({
                                (samples[13] > bright_limit)
                                    && ({
                                        (samples[14] > bright_limit) && (samples[15] > bright_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn score_classify_branch_31(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[14] < dark_limit {
            if samples[15] < dark_limit {
                {
                    (samples[1] < dark_limit)
                        || ((samples[8] < dark_limit) && (samples[9] < dark_limit))
                }
            } else {
                {
                    (samples[6] < dark_limit)
                        && ({
                            (samples[7] < dark_limit)
                                && ({ (samples[8] < dark_limit) && (samples[9] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[5] < dark_limit)
                    && ({
                        (samples[6] < dark_limit)
                            && ({
                                (samples[7] < dark_limit)
                                    && ({ (samples[8] < dark_limit) && (samples[9] < dark_limit) })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_32(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[11] > bright_limit {
            {
                (samples[7] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && ({
                                (samples[9] > bright_limit)
                                    && (Self::score_classify_branch_30(samples, bright_limit))
                            })
                    })
            }
        } else {
            {
                (samples[11] < dark_limit)
                    && ({
                        (samples[12] < dark_limit)
                            && ({
                                (samples[13] < dark_limit)
                                    && (if samples[10] < dark_limit {
                                        Self::score_classify_branch_31(samples, dark_limit)
                                    } else {
                                        (samples[1] < dark_limit)
                                            && ({
                                                (samples[3] < dark_limit)
                                                    && ({
                                                        (samples[14] < dark_limit)
                                                            && (samples[15] < dark_limit)
                                                    })
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_33(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[15] < dark_limit {
            if samples[1] < dark_limit {
                {
                    (samples[3] < dark_limit)
                        || ((samples[10] < dark_limit) && (samples[11] < dark_limit))
                }
            } else {
                {
                    (samples[8] < dark_limit)
                        && ({
                            (samples[9] < dark_limit)
                                && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
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
                                    && ({
                                        (samples[9] < dark_limit)
                                            && ({
                                                (samples[10] < dark_limit)
                                                    && (samples[11] < dark_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_34(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[12] > bright_limit {
            {
                (samples[7] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && ({
                                (samples[9] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit)
                                            && ({
                                                (samples[11] > bright_limit)
                                                    && ({
                                                        (samples[13] > bright_limit)
                                                            && ({
                                                                (samples[6] > bright_limit)
                                                                    || ((samples[14]
                                                                        > bright_limit)
                                                                        && (samples[15]
                                                                            > bright_limit))
                                                            })
                                                    })
                                            })
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[12] < dark_limit)
                    && ({
                        (samples[13] < dark_limit)
                            && ({
                                (samples[14] < dark_limit)
                                    && (Self::score_classify_branch_33(samples, dark_limit))
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_35(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            if samples[3] < dark_limit {
                (samples[6] < dark_limit) || (samples[13] < dark_limit)
            } else {
                {
                    (samples[10] < dark_limit)
                        && ({
                            (samples[11] < dark_limit)
                                && ({ (samples[12] < dark_limit) && (samples[13] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[8] < dark_limit)
                    && ({
                        (samples[9] < dark_limit)
                            && ({
                                (samples[10] < dark_limit)
                                    && ({
                                        (samples[11] < dark_limit)
                                            && ({
                                                (samples[12] < dark_limit)
                                                    && (samples[13] < dark_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_36(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[14] > bright_limit {
            {
                (samples[8] > bright_limit)
                    && ({
                        (samples[9] > bright_limit)
                            && ({
                                (samples[10] > bright_limit)
                                    && ({
                                        (samples[11] > bright_limit)
                                            && ({
                                                (samples[12] > bright_limit)
                                                    && ({
                                                        (samples[13] > bright_limit)
                                                            && ({
                                                                (samples[6] > bright_limit)
                                                                    || (samples[15] > bright_limit)
                                                            })
                                                    })
                                            })
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[14] < dark_limit)
                    && ({
                        (samples[15] < dark_limit)
                            && (Self::score_classify_branch_35(samples, dark_limit))
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_37(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            if samples[6] < dark_limit {
                (samples[8] < dark_limit) || (samples[15] < dark_limit)
            } else {
                {
                    (samples[13] < dark_limit)
                        && ({ (samples[14] < dark_limit) && (samples[15] < dark_limit) })
                }
            }
        } else {
            {
                (samples[8] < dark_limit)
                    && ({
                        (samples[9] < dark_limit)
                            && ({
                                (samples[10] < dark_limit)
                                    && ({
                                        (samples[6] < dark_limit)
                                            || ((samples[11] < dark_limit)
                                                && ({
                                                    (samples[12] < dark_limit)
                                                        && ({
                                                            (samples[13] < dark_limit)
                                                                && ({
                                                                    (samples[14] < dark_limit)
                                                                        && (samples[15]
                                                                            < dark_limit)
                                                                })
                                                        })
                                                }))
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_38(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[8] < dark_limit {
            if samples[9] < dark_limit {
                {
                    (samples[6] < dark_limit)
                        || ((samples[13] < dark_limit)
                            && ({ (samples[14] < dark_limit) && (samples[15] < dark_limit) }))
                }
            } else {
                {
                    (samples[1] < dark_limit)
                        && ({
                            (samples[13] < dark_limit)
                                && ({ (samples[14] < dark_limit) && (samples[15] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[1] < dark_limit)
                    && ({
                        (samples[13] < dark_limit)
                            && ({ (samples[14] < dark_limit) && (samples[15] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_39(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            if samples[3] < dark_limit {
                (samples[6] < dark_limit) || (samples[13] < dark_limit)
            } else {
                {
                    (samples[10] < dark_limit)
                        && ({
                            (samples[11] < dark_limit)
                                && ({ (samples[12] < dark_limit) && (samples[13] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[8] < dark_limit)
                    && ({
                        (samples[9] < dark_limit)
                            && ({
                                (samples[10] < dark_limit)
                                    && ({
                                        (samples[11] < dark_limit)
                                            && ({
                                                (samples[12] < dark_limit)
                                                    && (samples[13] < dark_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_40(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] > bright_limit {
            Self::score_classify_branch_36(samples, bright_limit, dark_limit)
        } else if samples[7] < dark_limit {
            if samples[3] < dark_limit {
                Self::score_classify_branch_37(samples, dark_limit)
            } else {
                {
                    (samples[10] < dark_limit)
                        && ({
                            (samples[11] < dark_limit)
                                && ({
                                    (samples[12] < dark_limit)
                                        && (Self::score_classify_branch_38(samples, dark_limit))
                                })
                        })
                }
            }
        } else {
            {
                (samples[14] < dark_limit)
                    && ({
                        (samples[15] < dark_limit)
                            && (Self::score_classify_branch_39(samples, dark_limit))
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_41(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[15] < dark_limit {
            if samples[1] < dark_limit {
                {
                    (samples[3] < dark_limit)
                        || ((samples[10] < dark_limit) && (samples[11] < dark_limit))
                }
            } else {
                {
                    (samples[8] < dark_limit)
                        && ({
                            (samples[9] < dark_limit)
                                && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
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
                                    && ({
                                        (samples[9] < dark_limit)
                                            && ({
                                                (samples[10] < dark_limit)
                                                    && (samples[11] < dark_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_42(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[12] > bright_limit {
            {
                (samples[7] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && ({
                                (samples[9] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit)
                                            && ({
                                                (samples[11] > bright_limit)
                                                    && ({
                                                        (samples[13] > bright_limit)
                                                            && ({
                                                                (samples[14] > bright_limit)
                                                                    && ({
                                                                        (samples[6] > bright_limit)
                                                                            || (samples[15]
                                                                                > bright_limit)
                                                                    })
                                                            })
                                                    })
                                            })
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[12] < dark_limit)
                    && ({
                        (samples[13] < dark_limit)
                            && ({
                                (samples[14] < dark_limit)
                                    && (Self::score_classify_branch_41(samples, dark_limit))
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_43(samples: &[i32; 16], bright_limit: i32) -> bool {
        {
            (samples[7] > bright_limit)
                && ({
                    (samples[8] > bright_limit)
                        && ({
                            (samples[9] > bright_limit)
                                && ({
                                    (samples[10] > bright_limit)
                                        && ({
                                            (samples[12] > bright_limit)
                                                && ({
                                                    (samples[13] > bright_limit)
                                                        && (if samples[6] > bright_limit {
                                                            {
                                                                (samples[5] > bright_limit)
                                                                    || (samples[14] > bright_limit)
                                                            }
                                                        } else {
                                                            {
                                                                (samples[14] > bright_limit)
                                                                    && (samples[15] > bright_limit)
                                                            }
                                                        })
                                                })
                                        })
                                })
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_44(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[14] < dark_limit {
            if samples[15] < dark_limit {
                {
                    (samples[1] < dark_limit)
                        || ((samples[8] < dark_limit) && (samples[9] < dark_limit))
                }
            } else {
                {
                    (samples[6] < dark_limit)
                        && ({
                            (samples[7] < dark_limit)
                                && ({ (samples[8] < dark_limit) && (samples[9] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[5] < dark_limit)
                    && ({
                        (samples[6] < dark_limit)
                            && ({
                                (samples[7] < dark_limit)
                                    && ({ (samples[8] < dark_limit) && (samples[9] < dark_limit) })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_45(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[11] > bright_limit {
            Self::score_classify_branch_43(samples, bright_limit)
        } else {
            (samples[11] < dark_limit)
                && ({
                    (samples[12] < dark_limit)
                        && ({
                            (samples[13] < dark_limit)
                                && (if samples[10] < dark_limit {
                                    Self::score_classify_branch_44(samples, dark_limit)
                                } else {
                                    {
                                        (samples[1] < dark_limit)
                                            && ({
                                                (samples[3] < dark_limit)
                                                    && ({
                                                        (samples[14] < dark_limit)
                                                            && (samples[15] < dark_limit)
                                                    })
                                            })
                                    }
                                })
                        })
                })
        }
    }
    #[inline]
    fn score_classify_branch_46(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[6] > bright_limit {
            if samples[5] > bright_limit {
                if samples[4] > bright_limit {
                    (samples[3] > bright_limit) || (samples[12] > bright_limit)
                } else {
                    (samples[12] > bright_limit) && (samples[13] > bright_limit)
                }
            } else {
                {
                    (samples[12] > bright_limit)
                        && ({ (samples[13] > bright_limit) && (samples[14] > bright_limit) })
                }
            }
        } else {
            {
                (samples[12] > bright_limit)
                    && ({
                        (samples[13] > bright_limit)
                            && ({ (samples[14] > bright_limit) && (samples[15] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_47(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[13] < dark_limit {
            if samples[14] < dark_limit {
                {
                    (samples[15] < dark_limit)
                        || ((samples[6] < dark_limit) && (samples[7] < dark_limit))
                }
            } else {
                {
                    (samples[5] < dark_limit)
                        && ({ (samples[6] < dark_limit) && (samples[7] < dark_limit) })
                }
            }
        } else {
            {
                (samples[4] < dark_limit)
                    && ({
                        (samples[5] < dark_limit)
                            && ({ (samples[6] < dark_limit) && (samples[7] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_48(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[8] < dark_limit {
            if samples[12] < dark_limit {
                Self::score_classify_branch_47(samples, dark_limit)
            } else {
                (samples[3] < dark_limit)
                    && ({
                        (samples[4] < dark_limit)
                            && ({
                                (samples[5] < dark_limit)
                                    && ({ (samples[6] < dark_limit) && (samples[7] < dark_limit) })
                            })
                    })
            }
        } else {
            {
                (samples[1] < dark_limit)
                    && ({
                        (samples[12] < dark_limit)
                            && ({
                                (samples[13] < dark_limit)
                                    && ({
                                        (samples[14] < dark_limit) && (samples[15] < dark_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_49(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] > bright_limit {
            {
                (samples[7] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && ({
                                (samples[10] > bright_limit)
                                    && ({
                                        (samples[11] > bright_limit)
                                            && (Self::score_classify_branch_46(
                                                samples,
                                                bright_limit,
                                            ))
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[9] < dark_limit)
                    && ({
                        (samples[10] < dark_limit)
                            && ({
                                (samples[11] < dark_limit)
                                    && (Self::score_classify_branch_48(samples, dark_limit))
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_50(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] > bright_limit {
            if samples[9] > bright_limit {
                Self::score_classify_branch_27(samples, bright_limit)
            } else {
                (samples[9] < dark_limit)
                    && ({
                        (samples[10] < dark_limit)
                            && ({
                                (samples[11] < dark_limit)
                                    && (Self::score_classify_branch_29(samples, dark_limit))
                            })
                    })
            }
        } else {
            if samples[2] < dark_limit {
                if samples[4] > bright_limit {
                    Self::score_classify_branch_32(samples, bright_limit, dark_limit)
                } else if samples[4] < dark_limit {
                    if samples[5] > bright_limit {
                        Self::score_classify_branch_34(samples, bright_limit, dark_limit)
                    } else if samples[5] < dark_limit {
                        Self::score_classify_branch_40(samples, bright_limit, dark_limit)
                    } else {
                        Self::score_classify_branch_42(samples, bright_limit, dark_limit)
                    }
                } else {
                    Self::score_classify_branch_45(samples, bright_limit, dark_limit)
                }
            } else {
                Self::score_classify_branch_49(samples, bright_limit, dark_limit)
            }
        }
    }
    #[inline]
    fn score_classify_branch_51(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[4] > bright_limit {
            if samples[3] > bright_limit {
                if samples[2] > bright_limit {
                    (samples[1] > bright_limit) || (samples[10] > bright_limit)
                } else {
                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                }
            } else {
                {
                    (samples[10] > bright_limit)
                        && ({ (samples[11] > bright_limit) && (samples[12] > bright_limit) })
                }
            }
        } else {
            {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({ (samples[12] > bright_limit) && (samples[13] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_52(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[6] > bright_limit {
            if samples[5] > bright_limit {
                Self::score_classify_branch_51(samples, bright_limit)
            } else {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({
                                (samples[12] > bright_limit)
                                    && ({
                                        (samples[13] > bright_limit) && (samples[14] > bright_limit)
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({
                                (samples[12] > bright_limit)
                                    && ({
                                        (samples[13] > bright_limit)
                                            && ({
                                                (samples[14] > bright_limit)
                                                    && (samples[15] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_53(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[4] < dark_limit {
            if samples[3] < dark_limit {
                if samples[2] < dark_limit {
                    (samples[1] < dark_limit) || (samples[10] < dark_limit)
                } else {
                    (samples[10] < dark_limit) && (samples[11] < dark_limit)
                }
            } else {
                {
                    (samples[10] < dark_limit)
                        && ({ (samples[11] < dark_limit) && (samples[12] < dark_limit) })
                }
            }
        } else {
            {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({ (samples[12] < dark_limit) && (samples[13] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_54(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[6] < dark_limit {
            if samples[5] < dark_limit {
                Self::score_classify_branch_53(samples, dark_limit)
            } else {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({
                                (samples[12] < dark_limit)
                                    && ({
                                        (samples[13] < dark_limit) && (samples[14] < dark_limit)
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({
                                (samples[12] < dark_limit)
                                    && ({
                                        (samples[13] < dark_limit)
                                            && ({
                                                (samples[14] < dark_limit)
                                                    && (samples[15] < dark_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn score_classify_branch_55(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[0] > bright_limit {
            Self::score_classify_branch_25(samples, bright_limit, dark_limit)
        } else if samples[0] < dark_limit {
            Self::score_classify_branch_50(samples, bright_limit, dark_limit)
        } else if samples[7] > bright_limit {
            {
                (samples[8] > bright_limit)
                    && ({
                        (samples[9] > bright_limit)
                            && (Self::score_classify_branch_52(samples, bright_limit))
                    })
            }
        } else {
            {
                (samples[7] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && ({
                                (samples[9] < dark_limit)
                                    && (Self::score_classify_branch_54(samples, dark_limit))
                            })
                    })
            }
        }
    }
}
impl OastDetector916 {
    /// # Errors
    /// Returns an error for invalid image storage or scaled sampling support/normalization.
    pub fn corner_score_scaled(
        &self,
        image: &purecv::core::Matrix<u8>,
        x: f32,
        y: f32,
        scale: f32,
    ) -> Result<i32> {
        crate::vectorized_filters::check(image)?;
        if !x.is_finite() || !y.is_finite() || !scale.is_finite() || scale <= 0.0 {
            return Err(crate::vectorized_filters::invalid(
                "finite positive corner scale required",
            ));
        }
        if (f64::from(x) - 3.5 * f64::from(scale)).floor() < 0.0
            || (f64::from(y) - 3.5 * f64::from(scale)).floor() < 0.0
            || (f64::from(x) + 3.5 * f64::from(scale)).ceil() >= image.cols as f64
            || (f64::from(y) + 3.5 * f64::from(scale)).ceil() >= image.rows as f64
        {
            return Ok(0);
        }
        let px = [0, -1, -2, -3, -3, -3, -2, -1, 0, 1, 2, 3, 3, 3, 2, 1, 0];
        let py = [-3, -3, -2, -1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, 0];
        let mut samples = [0i32; 17];
        for i in 0..17 {
            samples[i] = i32::from(pattern_value(
                image,
                x + px[i] as f32 * scale,
                y + py[i] as f32 * scale,
                scale,
            )?);
        }
        let (mut bmin, mut bmax) = (self.state.threshold, 255i32);
        if !(-1..=255).contains(&bmin) {
            return Err(crate::vectorized_filters::invalid(
                "corner threshold outside [-1,255]",
            ));
        }
        let mut b_test = bmax.midpoint(bmin);
        loop {
            let bright_limit = samples[16] + b_test;
            let dark_limit = bright_limit - 2 * b_test;
            let hit = {
                if samples[0] > bright_limit {
                    Self::corner_score_scaled_branch_25(&samples, bright_limit, dark_limit)
                } else {
                    Self::corner_score_scaled_branch_55(&samples, bright_limit, dark_limit)
                }
            };
            if hit {
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
    #[inline]
    fn corner_score_scaled_branch_0(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[1] > bright_limit {
            if samples[6] > bright_limit {
                (samples[8] > bright_limit) || (samples[15] > bright_limit)
            } else {
                {
                    (samples[13] > bright_limit)
                        && ({ (samples[14] > bright_limit) && (samples[15] > bright_limit) })
                }
            }
        } else {
            {
                (samples[8] > bright_limit)
                    && ({
                        (samples[9] > bright_limit)
                            && ({
                                (samples[10] > bright_limit)
                                    && ({
                                        (samples[6] > bright_limit)
                                            || ((samples[11] > bright_limit)
                                                && ({
                                                    (samples[12] > bright_limit)
                                                        && ({
                                                            (samples[13] > bright_limit)
                                                                && ({
                                                                    (samples[14] > bright_limit)
                                                                        && (samples[15]
                                                                            > bright_limit)
                                                                })
                                                        })
                                                }))
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_1(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[8] > bright_limit {
            if samples[9] > bright_limit {
                {
                    (samples[6] > bright_limit)
                        || ((samples[13] > bright_limit)
                            && ({ (samples[14] > bright_limit) && (samples[15] > bright_limit) }))
                }
            } else {
                {
                    (samples[1] > bright_limit)
                        && ({
                            (samples[13] > bright_limit)
                                && ({
                                    (samples[14] > bright_limit) && (samples[15] > bright_limit)
                                })
                        })
                }
            }
        } else {
            {
                (samples[1] > bright_limit)
                    && ({
                        (samples[13] > bright_limit)
                            && ({ (samples[14] > bright_limit) && (samples[15] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_2(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[1] > bright_limit {
            if samples[3] > bright_limit {
                (samples[6] > bright_limit) || (samples[13] > bright_limit)
            } else {
                {
                    (samples[10] > bright_limit)
                        && ({
                            (samples[11] > bright_limit)
                                && ({
                                    (samples[12] > bright_limit) && (samples[13] > bright_limit)
                                })
                        })
                }
            }
        } else {
            {
                (samples[8] > bright_limit)
                    && ({
                        (samples[9] > bright_limit)
                            && ({
                                (samples[10] > bright_limit)
                                    && ({
                                        (samples[11] > bright_limit)
                                            && ({
                                                (samples[12] > bright_limit)
                                                    && (samples[13] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_3(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[14] > bright_limit {
            {
                (samples[15] > bright_limit)
                    && (Self::corner_score_scaled_branch_2(samples, bright_limit))
            }
        } else {
            {
                (samples[14] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && ({
                                (samples[9] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit)
                                            && ({
                                                (samples[11] < dark_limit)
                                                    && ({
                                                        (samples[12] < dark_limit)
                                                            && ({
                                                                (samples[13] < dark_limit)
                                                                    && ({
                                                                        (samples[6] < dark_limit)
                                                                            || (samples[15]
                                                                                < dark_limit)
                                                                    })
                                                            })
                                                    })
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_4(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[1] > bright_limit {
            if samples[3] > bright_limit {
                (samples[6] > bright_limit) || (samples[13] > bright_limit)
            } else {
                {
                    (samples[10] > bright_limit)
                        && ({
                            (samples[11] > bright_limit)
                                && ({
                                    (samples[12] > bright_limit) && (samples[13] > bright_limit)
                                })
                        })
                }
            }
        } else {
            {
                (samples[8] > bright_limit)
                    && ({
                        (samples[9] > bright_limit)
                            && ({
                                (samples[10] > bright_limit)
                                    && ({
                                        (samples[11] > bright_limit)
                                            && ({
                                                (samples[12] > bright_limit)
                                                    && (samples[13] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_5(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[7] > bright_limit {
            if samples[3] > bright_limit {
                Self::corner_score_scaled_branch_0(samples, bright_limit)
            } else {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({
                                (samples[12] > bright_limit)
                                    && (Self::corner_score_scaled_branch_1(samples, bright_limit))
                            })
                    })
            }
        } else {
            if samples[7] < dark_limit {
                Self::corner_score_scaled_branch_3(samples, bright_limit, dark_limit)
            } else {
                (samples[14] > bright_limit)
                    && ({
                        (samples[15] > bright_limit)
                            && (Self::corner_score_scaled_branch_4(samples, bright_limit))
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_6(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[15] > bright_limit {
            if samples[1] > bright_limit {
                {
                    (samples[3] > bright_limit)
                        || ((samples[10] > bright_limit) && (samples[11] > bright_limit))
                }
            } else {
                {
                    (samples[8] > bright_limit)
                        && ({
                            (samples[9] > bright_limit)
                                && ({
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
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
                                    && ({
                                        (samples[9] > bright_limit)
                                            && ({
                                                (samples[10] > bright_limit)
                                                    && (samples[11] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_7(samples: &[i32; 17], dark_limit: i32) -> bool {
        {
            (samples[12] < dark_limit)
                && ({
                    (samples[7] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && ({
                                    (samples[9] < dark_limit)
                                        && ({
                                            (samples[10] < dark_limit)
                                                && ({
                                                    (samples[11] < dark_limit)
                                                        && ({
                                                            (samples[13] < dark_limit)
                                                                && ({
                                                                    (samples[6] < dark_limit)
                                                                        || ((samples[14]
                                                                            < dark_limit)
                                                                            && (samples[15]
                                                                                < dark_limit))
                                                                })
                                                        })
                                                })
                                        })
                                })
                        })
                })
        }
    }
    #[inline]
    fn corner_score_scaled_branch_8(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[15] > bright_limit {
            if samples[1] > bright_limit {
                {
                    (samples[3] > bright_limit)
                        || ((samples[10] > bright_limit) && (samples[11] > bright_limit))
                }
            } else {
                {
                    (samples[8] > bright_limit)
                        && ({
                            (samples[9] > bright_limit)
                                && ({
                                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
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
                                    && ({
                                        (samples[9] > bright_limit)
                                            && ({
                                                (samples[10] > bright_limit)
                                                    && (samples[11] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_9(samples: &[i32; 17], dark_limit: i32) -> bool {
        {
            (samples[12] < dark_limit)
                && ({
                    (samples[7] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && ({
                                    (samples[9] < dark_limit)
                                        && ({
                                            (samples[10] < dark_limit)
                                                && ({
                                                    (samples[11] < dark_limit)
                                                        && ({
                                                            (samples[13] < dark_limit)
                                                                && ({
                                                                    (samples[14] < dark_limit)
                                                                        && ({
                                                                            (samples[6]
                                                                                < dark_limit)
                                                                                || (samples[15]
                                                                                    < dark_limit)
                                                                        })
                                                                })
                                                        })
                                                })
                                        })
                                })
                        })
                })
        }
    }
    #[inline]
    fn corner_score_scaled_branch_10(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[5] > bright_limit {
            Self::corner_score_scaled_branch_5(samples, bright_limit, dark_limit)
        } else if samples[5] < dark_limit {
            if samples[12] > bright_limit {
                {
                    (samples[13] > bright_limit)
                        && ({
                            (samples[14] > bright_limit)
                                && (Self::corner_score_scaled_branch_6(samples, bright_limit))
                        })
                }
            } else {
                Self::corner_score_scaled_branch_7(samples, dark_limit)
            }
        } else {
            if samples[12] > bright_limit {
                {
                    (samples[13] > bright_limit)
                        && ({
                            (samples[14] > bright_limit)
                                && (Self::corner_score_scaled_branch_8(samples, bright_limit))
                        })
                }
            } else {
                Self::corner_score_scaled_branch_9(samples, dark_limit)
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_11(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[14] > bright_limit {
            if samples[15] > bright_limit {
                {
                    (samples[1] > bright_limit)
                        || ((samples[8] > bright_limit) && (samples[9] > bright_limit))
                }
            } else {
                {
                    (samples[6] > bright_limit)
                        && ({
                            (samples[7] > bright_limit)
                                && ({ (samples[8] > bright_limit) && (samples[9] > bright_limit) })
                        })
                }
            }
        } else {
            {
                (samples[5] > bright_limit)
                    && ({
                        (samples[6] > bright_limit)
                            && ({
                                (samples[7] > bright_limit)
                                    && ({
                                        (samples[8] > bright_limit) && (samples[9] > bright_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_12(samples: &[i32; 17], dark_limit: i32) -> bool {
        {
            (samples[10] < dark_limit)
                && (if samples[6] < dark_limit {
                    if samples[5] < dark_limit {
                        (samples[3] < dark_limit) || (samples[12] < dark_limit)
                    } else {
                        {
                            (samples[12] < dark_limit)
                                && ({ (samples[13] < dark_limit) && (samples[14] < dark_limit) })
                        }
                    }
                } else {
                    {
                        (samples[12] < dark_limit)
                            && ({
                                (samples[13] < dark_limit)
                                    && ({
                                        (samples[14] < dark_limit) && (samples[15] < dark_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn corner_score_scaled_branch_13(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[11] > bright_limit {
            {
                (samples[12] > bright_limit)
                    && ({
                        (samples[13] > bright_limit)
                            && (if samples[10] > bright_limit {
                                Self::corner_score_scaled_branch_11(samples, bright_limit)
                            } else {
                                (samples[1] > bright_limit)
                                    && ({
                                        (samples[3] > bright_limit)
                                            && ({
                                                (samples[14] > bright_limit)
                                                    && (samples[15] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[11] < dark_limit)
                    && ({
                        (samples[7] < dark_limit)
                            && ({
                                (samples[8] < dark_limit)
                                    && ({
                                        (samples[9] < dark_limit)
                                            && (Self::corner_score_scaled_branch_12(
                                                samples, dark_limit,
                                            ))
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_14(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[14] > bright_limit {
            if samples[15] > bright_limit {
                {
                    (samples[1] > bright_limit)
                        || ((samples[8] > bright_limit) && (samples[9] > bright_limit))
                }
            } else {
                {
                    (samples[6] > bright_limit)
                        && ({
                            (samples[7] > bright_limit)
                                && ({ (samples[8] > bright_limit) && (samples[9] > bright_limit) })
                        })
                }
            }
        } else {
            {
                (samples[5] > bright_limit)
                    && ({
                        (samples[6] > bright_limit)
                            && ({
                                (samples[7] > bright_limit)
                                    && ({
                                        (samples[8] > bright_limit) && (samples[9] > bright_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_15(samples: &[i32; 17], dark_limit: i32) -> bool {
        {
            (samples[7] < dark_limit)
                && ({
                    (samples[8] < dark_limit)
                        && ({
                            (samples[9] < dark_limit)
                                && ({
                                    (samples[10] < dark_limit)
                                        && ({
                                            (samples[12] < dark_limit)
                                                && ({
                                                    (samples[13] < dark_limit)
                                                        && (if samples[6] < dark_limit {
                                                            {
                                                                (samples[5] < dark_limit)
                                                                    || (samples[14] < dark_limit)
                                                            }
                                                        } else {
                                                            {
                                                                (samples[14] < dark_limit)
                                                                    && (samples[15] < dark_limit)
                                                            }
                                                        })
                                                })
                                        })
                                })
                        })
                })
        }
    }
    #[inline]
    fn corner_score_scaled_branch_16(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[11] > bright_limit {
            {
                (samples[12] > bright_limit)
                    && ({
                        (samples[13] > bright_limit)
                            && (if samples[10] > bright_limit {
                                Self::corner_score_scaled_branch_14(samples, bright_limit)
                            } else {
                                (samples[1] > bright_limit)
                                    && ({
                                        (samples[3] > bright_limit)
                                            && ({
                                                (samples[14] > bright_limit)
                                                    && (samples[15] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[11] < dark_limit)
                    && (Self::corner_score_scaled_branch_15(samples, dark_limit))
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_17(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[13] > bright_limit {
            if samples[14] > bright_limit {
                {
                    (samples[15] > bright_limit)
                        || ((samples[6] > bright_limit) && (samples[7] > bright_limit))
                }
            } else {
                {
                    (samples[5] > bright_limit)
                        && ({ (samples[6] > bright_limit) && (samples[7] > bright_limit) })
                }
            }
        } else {
            {
                (samples[4] > bright_limit)
                    && ({
                        (samples[5] > bright_limit)
                            && ({ (samples[6] > bright_limit) && (samples[7] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_18(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[8] > bright_limit {
            if samples[12] > bright_limit {
                Self::corner_score_scaled_branch_17(samples, bright_limit)
            } else {
                (samples[3] > bright_limit)
                    && ({
                        (samples[4] > bright_limit)
                            && ({
                                (samples[5] > bright_limit)
                                    && ({
                                        (samples[6] > bright_limit) && (samples[7] > bright_limit)
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[1] > bright_limit)
                    && ({
                        (samples[12] > bright_limit)
                            && ({
                                (samples[13] > bright_limit)
                                    && ({
                                        (samples[14] > bright_limit) && (samples[15] > bright_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_19(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[5] < dark_limit {
            if samples[4] < dark_limit {
                if samples[3] < dark_limit {
                    (samples[1] < dark_limit) || (samples[10] < dark_limit)
                } else {
                    {
                        (samples[10] < dark_limit)
                            && ({ (samples[11] < dark_limit) && (samples[12] < dark_limit) })
                    }
                }
            } else {
                {
                    (samples[10] < dark_limit)
                        && ({
                            (samples[11] < dark_limit)
                                && ({ (samples[12] < dark_limit) && (samples[13] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({
                                (samples[12] < dark_limit)
                                    && ({
                                        (samples[13] < dark_limit) && (samples[14] < dark_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_20(samples: &[i32; 17], dark_limit: i32) -> bool {
        {
            (samples[7] < dark_limit)
                && ({
                    (samples[8] < dark_limit)
                        && (if samples[6] < dark_limit {
                            Self::corner_score_scaled_branch_19(samples, dark_limit)
                        } else {
                            (samples[10] < dark_limit)
                                && ({
                                    (samples[11] < dark_limit)
                                        && ({
                                            (samples[12] < dark_limit)
                                                && ({
                                                    (samples[13] < dark_limit)
                                                        && ({
                                                            (samples[14] < dark_limit)
                                                                && (samples[15] < dark_limit)
                                                        })
                                                })
                                        })
                                })
                        })
                })
        }
    }
    #[inline]
    fn corner_score_scaled_branch_21(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[13] > bright_limit {
            if samples[14] > bright_limit {
                {
                    (samples[15] > bright_limit)
                        || ((samples[6] > bright_limit) && (samples[7] > bright_limit))
                }
            } else {
                {
                    (samples[5] > bright_limit)
                        && ({ (samples[6] > bright_limit) && (samples[7] > bright_limit) })
                }
            }
        } else {
            {
                (samples[4] > bright_limit)
                    && ({
                        (samples[5] > bright_limit)
                            && ({ (samples[6] > bright_limit) && (samples[7] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_22(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[8] > bright_limit {
            if samples[12] > bright_limit {
                Self::corner_score_scaled_branch_21(samples, bright_limit)
            } else {
                (samples[3] > bright_limit)
                    && ({
                        (samples[4] > bright_limit)
                            && ({
                                (samples[5] > bright_limit)
                                    && ({
                                        (samples[6] > bright_limit) && (samples[7] > bright_limit)
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[1] > bright_limit)
                    && ({
                        (samples[12] > bright_limit)
                            && ({
                                (samples[13] > bright_limit)
                                    && ({
                                        (samples[14] > bright_limit) && (samples[15] > bright_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_23(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[6] < dark_limit {
            if samples[5] < dark_limit {
                if samples[4] < dark_limit {
                    (samples[3] < dark_limit) || (samples[12] < dark_limit)
                } else {
                    (samples[12] < dark_limit) && (samples[13] < dark_limit)
                }
            } else {
                {
                    (samples[12] < dark_limit)
                        && ({ (samples[13] < dark_limit) && (samples[14] < dark_limit) })
                }
            }
        } else {
            {
                (samples[12] < dark_limit)
                    && ({
                        (samples[13] < dark_limit)
                            && ({ (samples[14] < dark_limit) && (samples[15] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_24(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[9] > bright_limit {
            {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && (Self::corner_score_scaled_branch_22(samples, bright_limit))
                    })
            }
        } else {
            {
                (samples[9] < dark_limit)
                    && ({
                        (samples[7] < dark_limit)
                            && ({
                                (samples[8] < dark_limit)
                                    && ({
                                        (samples[10] < dark_limit)
                                            && ({
                                                (samples[11] < dark_limit)
                                                    && (Self::corner_score_scaled_branch_23(
                                                        samples, dark_limit,
                                                    ))
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_25(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[2] > bright_limit {
            if samples[4] > bright_limit {
                Self::corner_score_scaled_branch_10(samples, bright_limit, dark_limit)
            } else if samples[4] < dark_limit {
                Self::corner_score_scaled_branch_13(samples, bright_limit, dark_limit)
            } else {
                Self::corner_score_scaled_branch_16(samples, bright_limit, dark_limit)
            }
        } else {
            if samples[2] < dark_limit {
                if samples[9] > bright_limit {
                    {
                        (samples[10] > bright_limit)
                            && ({
                                (samples[11] > bright_limit)
                                    && (Self::corner_score_scaled_branch_18(samples, bright_limit))
                            })
                    }
                } else {
                    {
                        (samples[9] < dark_limit)
                            && (Self::corner_score_scaled_branch_20(samples, dark_limit))
                    }
                }
            } else {
                Self::corner_score_scaled_branch_24(samples, bright_limit, dark_limit)
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_26(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[5] > bright_limit {
            if samples[4] > bright_limit {
                if samples[3] > bright_limit {
                    (samples[1] > bright_limit) || (samples[10] > bright_limit)
                } else {
                    {
                        (samples[10] > bright_limit)
                            && ({ (samples[11] > bright_limit) && (samples[12] > bright_limit) })
                    }
                }
            } else {
                {
                    (samples[10] > bright_limit)
                        && ({
                            (samples[11] > bright_limit)
                                && ({
                                    (samples[12] > bright_limit) && (samples[13] > bright_limit)
                                })
                        })
                }
            }
        } else {
            {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({
                                (samples[12] > bright_limit)
                                    && ({
                                        (samples[13] > bright_limit) && (samples[14] > bright_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_27(samples: &[i32; 17], bright_limit: i32) -> bool {
        {
            (samples[7] > bright_limit)
                && ({
                    (samples[8] > bright_limit)
                        && (if samples[6] > bright_limit {
                            Self::corner_score_scaled_branch_26(samples, bright_limit)
                        } else {
                            (samples[10] > bright_limit)
                                && ({
                                    (samples[11] > bright_limit)
                                        && ({
                                            (samples[12] > bright_limit)
                                                && ({
                                                    (samples[13] > bright_limit)
                                                        && ({
                                                            (samples[14] > bright_limit)
                                                                && (samples[15] > bright_limit)
                                                        })
                                                })
                                        })
                                })
                        })
                })
        }
    }
    #[inline]
    fn corner_score_scaled_branch_28(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[13] < dark_limit {
            if samples[14] < dark_limit {
                {
                    (samples[15] < dark_limit)
                        || ((samples[6] < dark_limit) && (samples[7] < dark_limit))
                }
            } else {
                {
                    (samples[5] < dark_limit)
                        && ({ (samples[6] < dark_limit) && (samples[7] < dark_limit) })
                }
            }
        } else {
            {
                (samples[4] < dark_limit)
                    && ({
                        (samples[5] < dark_limit)
                            && ({ (samples[6] < dark_limit) && (samples[7] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_29(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[8] < dark_limit {
            if samples[12] < dark_limit {
                Self::corner_score_scaled_branch_28(samples, dark_limit)
            } else {
                (samples[3] < dark_limit)
                    && ({
                        (samples[4] < dark_limit)
                            && ({
                                (samples[5] < dark_limit)
                                    && ({ (samples[6] < dark_limit) && (samples[7] < dark_limit) })
                            })
                    })
            }
        } else {
            {
                (samples[1] < dark_limit)
                    && ({
                        (samples[12] < dark_limit)
                            && ({
                                (samples[13] < dark_limit)
                                    && ({
                                        (samples[14] < dark_limit) && (samples[15] < dark_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_30(samples: &[i32; 17], bright_limit: i32) -> bool {
        {
            (samples[10] > bright_limit)
                && (if samples[6] > bright_limit {
                    if samples[5] > bright_limit {
                        (samples[3] > bright_limit) || (samples[12] > bright_limit)
                    } else {
                        {
                            (samples[12] > bright_limit)
                                && ({
                                    (samples[13] > bright_limit) && (samples[14] > bright_limit)
                                })
                        }
                    }
                } else {
                    {
                        (samples[12] > bright_limit)
                            && ({
                                (samples[13] > bright_limit)
                                    && ({
                                        (samples[14] > bright_limit) && (samples[15] > bright_limit)
                                    })
                            })
                    }
                })
        }
    }
    #[inline]
    fn corner_score_scaled_branch_31(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[14] < dark_limit {
            if samples[15] < dark_limit {
                {
                    (samples[1] < dark_limit)
                        || ((samples[8] < dark_limit) && (samples[9] < dark_limit))
                }
            } else {
                {
                    (samples[6] < dark_limit)
                        && ({
                            (samples[7] < dark_limit)
                                && ({ (samples[8] < dark_limit) && (samples[9] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[5] < dark_limit)
                    && ({
                        (samples[6] < dark_limit)
                            && ({
                                (samples[7] < dark_limit)
                                    && ({ (samples[8] < dark_limit) && (samples[9] < dark_limit) })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_32(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[11] > bright_limit {
            {
                (samples[7] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && ({
                                (samples[9] > bright_limit)
                                    && (Self::corner_score_scaled_branch_30(samples, bright_limit))
                            })
                    })
            }
        } else {
            {
                (samples[11] < dark_limit)
                    && ({
                        (samples[12] < dark_limit)
                            && ({
                                (samples[13] < dark_limit)
                                    && (if samples[10] < dark_limit {
                                        Self::corner_score_scaled_branch_31(samples, dark_limit)
                                    } else {
                                        (samples[1] < dark_limit)
                                            && ({
                                                (samples[3] < dark_limit)
                                                    && ({
                                                        (samples[14] < dark_limit)
                                                            && (samples[15] < dark_limit)
                                                    })
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_33(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[15] < dark_limit {
            if samples[1] < dark_limit {
                {
                    (samples[3] < dark_limit)
                        || ((samples[10] < dark_limit) && (samples[11] < dark_limit))
                }
            } else {
                {
                    (samples[8] < dark_limit)
                        && ({
                            (samples[9] < dark_limit)
                                && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
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
                                    && ({
                                        (samples[9] < dark_limit)
                                            && ({
                                                (samples[10] < dark_limit)
                                                    && (samples[11] < dark_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_34(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[12] > bright_limit {
            {
                (samples[7] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && ({
                                (samples[9] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit)
                                            && ({
                                                (samples[11] > bright_limit)
                                                    && ({
                                                        (samples[13] > bright_limit)
                                                            && ({
                                                                (samples[6] > bright_limit)
                                                                    || ((samples[14]
                                                                        > bright_limit)
                                                                        && (samples[15]
                                                                            > bright_limit))
                                                            })
                                                    })
                                            })
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[12] < dark_limit)
                    && ({
                        (samples[13] < dark_limit)
                            && ({
                                (samples[14] < dark_limit)
                                    && (Self::corner_score_scaled_branch_33(samples, dark_limit))
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_35(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            if samples[3] < dark_limit {
                (samples[6] < dark_limit) || (samples[13] < dark_limit)
            } else {
                {
                    (samples[10] < dark_limit)
                        && ({
                            (samples[11] < dark_limit)
                                && ({ (samples[12] < dark_limit) && (samples[13] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[8] < dark_limit)
                    && ({
                        (samples[9] < dark_limit)
                            && ({
                                (samples[10] < dark_limit)
                                    && ({
                                        (samples[11] < dark_limit)
                                            && ({
                                                (samples[12] < dark_limit)
                                                    && (samples[13] < dark_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_36(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[14] > bright_limit {
            {
                (samples[8] > bright_limit)
                    && ({
                        (samples[9] > bright_limit)
                            && ({
                                (samples[10] > bright_limit)
                                    && ({
                                        (samples[11] > bright_limit)
                                            && ({
                                                (samples[12] > bright_limit)
                                                    && ({
                                                        (samples[13] > bright_limit)
                                                            && ({
                                                                (samples[6] > bright_limit)
                                                                    || (samples[15] > bright_limit)
                                                            })
                                                    })
                                            })
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[14] < dark_limit)
                    && ({
                        (samples[15] < dark_limit)
                            && (Self::corner_score_scaled_branch_35(samples, dark_limit))
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_37(samples: &[i32; 17], dark_limit: i32) -> bool {
        {
            (samples[8] < dark_limit)
                && ({
                    (samples[9] < dark_limit)
                        && ({
                            (samples[10] < dark_limit)
                                && ({
                                    (samples[6] < dark_limit)
                                        || ((samples[11] < dark_limit)
                                            && ({
                                                (samples[12] < dark_limit)
                                                    && ({
                                                        (samples[13] < dark_limit)
                                                            && ({
                                                                (samples[14] < dark_limit)
                                                                    && (samples[15] < dark_limit)
                                                            })
                                                    })
                                            }))
                                })
                        })
                })
        }
    }
    #[inline]
    fn corner_score_scaled_branch_38(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[8] < dark_limit {
            if samples[9] < dark_limit {
                {
                    (samples[6] < dark_limit)
                        || ((samples[13] < dark_limit)
                            && ({ (samples[14] < dark_limit) && (samples[15] < dark_limit) }))
                }
            } else {
                {
                    (samples[1] < dark_limit)
                        && ({
                            (samples[13] < dark_limit)
                                && ({ (samples[14] < dark_limit) && (samples[15] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[1] < dark_limit)
                    && ({
                        (samples[13] < dark_limit)
                            && ({ (samples[14] < dark_limit) && (samples[15] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_39(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[3] < dark_limit {
            if samples[1] < dark_limit {
                if samples[6] < dark_limit {
                    (samples[8] < dark_limit) || (samples[15] < dark_limit)
                } else {
                    {
                        (samples[13] < dark_limit)
                            && ({ (samples[14] < dark_limit) && (samples[15] < dark_limit) })
                    }
                }
            } else {
                Self::corner_score_scaled_branch_37(samples, dark_limit)
            }
        } else {
            {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({
                                (samples[12] < dark_limit)
                                    && (Self::corner_score_scaled_branch_38(samples, dark_limit))
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_40(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[1] < dark_limit {
            if samples[3] < dark_limit {
                (samples[6] < dark_limit) || (samples[13] < dark_limit)
            } else {
                {
                    (samples[10] < dark_limit)
                        && ({
                            (samples[11] < dark_limit)
                                && ({ (samples[12] < dark_limit) && (samples[13] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[8] < dark_limit)
                    && ({
                        (samples[9] < dark_limit)
                            && ({
                                (samples[10] < dark_limit)
                                    && ({
                                        (samples[11] < dark_limit)
                                            && ({
                                                (samples[12] < dark_limit)
                                                    && (samples[13] < dark_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_41(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[15] < dark_limit {
            if samples[1] < dark_limit {
                {
                    (samples[3] < dark_limit)
                        || ((samples[10] < dark_limit) && (samples[11] < dark_limit))
                }
            } else {
                {
                    (samples[8] < dark_limit)
                        && ({
                            (samples[9] < dark_limit)
                                && ({ (samples[10] < dark_limit) && (samples[11] < dark_limit) })
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
                                    && ({
                                        (samples[9] < dark_limit)
                                            && ({
                                                (samples[10] < dark_limit)
                                                    && (samples[11] < dark_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_42(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[12] > bright_limit {
            {
                (samples[7] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && ({
                                (samples[9] > bright_limit)
                                    && ({
                                        (samples[10] > bright_limit)
                                            && ({
                                                (samples[11] > bright_limit)
                                                    && ({
                                                        (samples[13] > bright_limit)
                                                            && ({
                                                                (samples[14] > bright_limit)
                                                                    && ({
                                                                        (samples[6] > bright_limit)
                                                                            || (samples[15]
                                                                                > bright_limit)
                                                                    })
                                                            })
                                                    })
                                            })
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[12] < dark_limit)
                    && ({
                        (samples[13] < dark_limit)
                            && ({
                                (samples[14] < dark_limit)
                                    && (Self::corner_score_scaled_branch_41(samples, dark_limit))
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_43(samples: &[i32; 17], bright_limit: i32) -> bool {
        {
            (samples[7] > bright_limit)
                && ({
                    (samples[8] > bright_limit)
                        && ({
                            (samples[9] > bright_limit)
                                && ({
                                    (samples[10] > bright_limit)
                                        && ({
                                            (samples[12] > bright_limit)
                                                && ({
                                                    (samples[13] > bright_limit)
                                                        && (if samples[6] > bright_limit {
                                                            {
                                                                (samples[5] > bright_limit)
                                                                    || (samples[14] > bright_limit)
                                                            }
                                                        } else {
                                                            {
                                                                (samples[14] > bright_limit)
                                                                    && (samples[15] > bright_limit)
                                                            }
                                                        })
                                                })
                                        })
                                })
                        })
                })
        }
    }
    #[inline]
    fn corner_score_scaled_branch_44(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[14] < dark_limit {
            if samples[15] < dark_limit {
                {
                    (samples[1] < dark_limit)
                        || ((samples[8] < dark_limit) && (samples[9] < dark_limit))
                }
            } else {
                {
                    (samples[6] < dark_limit)
                        && ({
                            (samples[7] < dark_limit)
                                && ({ (samples[8] < dark_limit) && (samples[9] < dark_limit) })
                        })
                }
            }
        } else {
            {
                (samples[5] < dark_limit)
                    && ({
                        (samples[6] < dark_limit)
                            && ({
                                (samples[7] < dark_limit)
                                    && ({ (samples[8] < dark_limit) && (samples[9] < dark_limit) })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_45(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[11] > bright_limit {
            Self::corner_score_scaled_branch_43(samples, bright_limit)
        } else {
            (samples[11] < dark_limit)
                && ({
                    (samples[12] < dark_limit)
                        && ({
                            (samples[13] < dark_limit)
                                && (if samples[10] < dark_limit {
                                    Self::corner_score_scaled_branch_44(samples, dark_limit)
                                } else {
                                    {
                                        (samples[1] < dark_limit)
                                            && ({
                                                (samples[3] < dark_limit)
                                                    && ({
                                                        (samples[14] < dark_limit)
                                                            && (samples[15] < dark_limit)
                                                    })
                                            })
                                    }
                                })
                        })
                })
        }
    }
    #[inline]
    fn corner_score_scaled_branch_46(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[4] > bright_limit {
            Self::corner_score_scaled_branch_32(samples, bright_limit, dark_limit)
        } else if samples[4] < dark_limit {
            if samples[5] > bright_limit {
                Self::corner_score_scaled_branch_34(samples, bright_limit, dark_limit)
            } else if samples[5] < dark_limit {
                if samples[7] > bright_limit {
                    Self::corner_score_scaled_branch_36(samples, bright_limit, dark_limit)
                } else if samples[7] < dark_limit {
                    Self::corner_score_scaled_branch_39(samples, dark_limit)
                } else {
                    {
                        (samples[14] < dark_limit)
                            && ({
                                (samples[15] < dark_limit)
                                    && (Self::corner_score_scaled_branch_40(samples, dark_limit))
                            })
                    }
                }
            } else {
                Self::corner_score_scaled_branch_42(samples, bright_limit, dark_limit)
            }
        } else {
            Self::corner_score_scaled_branch_45(samples, bright_limit, dark_limit)
        }
    }
    #[inline]
    fn corner_score_scaled_branch_47(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[6] > bright_limit {
            if samples[5] > bright_limit {
                if samples[4] > bright_limit {
                    (samples[3] > bright_limit) || (samples[12] > bright_limit)
                } else {
                    (samples[12] > bright_limit) && (samples[13] > bright_limit)
                }
            } else {
                {
                    (samples[12] > bright_limit)
                        && ({ (samples[13] > bright_limit) && (samples[14] > bright_limit) })
                }
            }
        } else {
            {
                (samples[12] > bright_limit)
                    && ({
                        (samples[13] > bright_limit)
                            && ({ (samples[14] > bright_limit) && (samples[15] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_48(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[13] < dark_limit {
            if samples[14] < dark_limit {
                {
                    (samples[15] < dark_limit)
                        || ((samples[6] < dark_limit) && (samples[7] < dark_limit))
                }
            } else {
                {
                    (samples[5] < dark_limit)
                        && ({ (samples[6] < dark_limit) && (samples[7] < dark_limit) })
                }
            }
        } else {
            {
                (samples[4] < dark_limit)
                    && ({
                        (samples[5] < dark_limit)
                            && ({ (samples[6] < dark_limit) && (samples[7] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_49(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[8] < dark_limit {
            if samples[12] < dark_limit {
                Self::corner_score_scaled_branch_48(samples, dark_limit)
            } else {
                (samples[3] < dark_limit)
                    && ({
                        (samples[4] < dark_limit)
                            && ({
                                (samples[5] < dark_limit)
                                    && ({ (samples[6] < dark_limit) && (samples[7] < dark_limit) })
                            })
                    })
            }
        } else {
            {
                (samples[1] < dark_limit)
                    && ({
                        (samples[12] < dark_limit)
                            && ({
                                (samples[13] < dark_limit)
                                    && ({
                                        (samples[14] < dark_limit) && (samples[15] < dark_limit)
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_50(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[9] > bright_limit {
            {
                (samples[7] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && ({
                                (samples[10] > bright_limit)
                                    && ({
                                        (samples[11] > bright_limit)
                                            && (Self::corner_score_scaled_branch_47(
                                                samples,
                                                bright_limit,
                                            ))
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[9] < dark_limit)
                    && ({
                        (samples[10] < dark_limit)
                            && ({
                                (samples[11] < dark_limit)
                                    && (Self::corner_score_scaled_branch_49(samples, dark_limit))
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_51(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[4] > bright_limit {
            if samples[3] > bright_limit {
                if samples[2] > bright_limit {
                    (samples[1] > bright_limit) || (samples[10] > bright_limit)
                } else {
                    (samples[10] > bright_limit) && (samples[11] > bright_limit)
                }
            } else {
                {
                    (samples[10] > bright_limit)
                        && ({ (samples[11] > bright_limit) && (samples[12] > bright_limit) })
                }
            }
        } else {
            {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({ (samples[12] > bright_limit) && (samples[13] > bright_limit) })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_52(samples: &[i32; 17], bright_limit: i32) -> bool {
        if samples[6] > bright_limit {
            if samples[5] > bright_limit {
                Self::corner_score_scaled_branch_51(samples, bright_limit)
            } else {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({
                                (samples[12] > bright_limit)
                                    && ({
                                        (samples[13] > bright_limit) && (samples[14] > bright_limit)
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({
                                (samples[12] > bright_limit)
                                    && ({
                                        (samples[13] > bright_limit)
                                            && ({
                                                (samples[14] > bright_limit)
                                                    && (samples[15] > bright_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_53(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[4] < dark_limit {
            if samples[3] < dark_limit {
                if samples[2] < dark_limit {
                    (samples[1] < dark_limit) || (samples[10] < dark_limit)
                } else {
                    (samples[10] < dark_limit) && (samples[11] < dark_limit)
                }
            } else {
                {
                    (samples[10] < dark_limit)
                        && ({ (samples[11] < dark_limit) && (samples[12] < dark_limit) })
                }
            }
        } else {
            {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({ (samples[12] < dark_limit) && (samples[13] < dark_limit) })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_54(samples: &[i32; 17], dark_limit: i32) -> bool {
        if samples[6] < dark_limit {
            if samples[5] < dark_limit {
                Self::corner_score_scaled_branch_53(samples, dark_limit)
            } else {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({
                                (samples[12] < dark_limit)
                                    && ({
                                        (samples[13] < dark_limit) && (samples[14] < dark_limit)
                                    })
                            })
                    })
            }
        } else {
            {
                (samples[10] < dark_limit)
                    && ({
                        (samples[11] < dark_limit)
                            && ({
                                (samples[12] < dark_limit)
                                    && ({
                                        (samples[13] < dark_limit)
                                            && ({
                                                (samples[14] < dark_limit)
                                                    && (samples[15] < dark_limit)
                                            })
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn corner_score_scaled_branch_55(
        samples: &[i32; 17],
        bright_limit: i32,
        dark_limit: i32,
    ) -> bool {
        if samples[0] < dark_limit {
            if samples[2] > bright_limit {
                if samples[9] > bright_limit {
                    Self::corner_score_scaled_branch_27(samples, bright_limit)
                } else {
                    (samples[9] < dark_limit)
                        && ({
                            (samples[10] < dark_limit)
                                && ({
                                    (samples[11] < dark_limit)
                                        && (Self::corner_score_scaled_branch_29(
                                            samples, dark_limit,
                                        ))
                                })
                        })
                }
            } else {
                if samples[2] < dark_limit {
                    Self::corner_score_scaled_branch_46(samples, bright_limit, dark_limit)
                } else {
                    Self::corner_score_scaled_branch_50(samples, bright_limit, dark_limit)
                }
            }
        } else {
            if samples[7] > bright_limit {
                {
                    (samples[8] > bright_limit)
                        && ({
                            (samples[9] > bright_limit)
                                && (Self::corner_score_scaled_branch_52(samples, bright_limit))
                        })
                }
            } else {
                {
                    (samples[7] < dark_limit)
                        && ({
                            (samples[8] < dark_limit)
                                && ({
                                    (samples[9] < dark_limit)
                                        && (Self::corner_score_scaled_branch_54(
                                            samples, dark_limit,
                                        ))
                                })
                        })
                }
            }
        }
    }
}
pub(crate) fn pattern_value(
    image: &purecv::core::Matrix<u8>,
    xf: f32,
    yf: f32,
    scale: f32,
) -> Result<u8> {
    let sigma_half = (0.5f64).max(f64::from(scale) / 2.0) as f32;
    let area = 4.0f32 * sigma_half * sigma_half;
    let scaling = (4_194_304.0f64 / f64::from(area)) as i32;
    let scaling2 = (f64::from(scaling as f32 * area) / 1024.0) as i32;
    if scaling2 == 0 {
        return Err(crate::vectorized_filters::invalid(
            "zero pattern integration divisor",
        ));
    }
    let (x_1, x1, y_1, y1) = (
        xf - sigma_half,
        xf + sigma_half,
        yf - sigma_half,
        yf + sigma_half,
    );
    let (xl, yt, xr, yb) = (
        (f64::from(x_1) + 0.5) as i32,
        (f64::from(y_1) + 0.5) as i32,
        (f64::from(x1) + 0.5) as i32,
        (f64::from(y1) + 0.5) as i32,
    );
    if xl < 0 || yt < 0 || xr >= image.cols as i32 || yb >= image.rows as i32 {
        return Err(crate::vectorized_filters::invalid(
            "pattern integration outside image",
        ));
    }
    let (left_fraction, top_fraction, right_fraction, bottom_fraction) = (
        f64::from(xl as f32 - x_1) + 0.5,
        f64::from(yt as f32 - y_1) + 0.5,
        f64::from(x1 - xr as f32) + 0.5,
        f64::from(y1 - yb as f32) + 0.5,
    );
    let (left_fraction, top_fraction, right_fraction, bottom_fraction) = (
        left_fraction as f32,
        top_fraction as f32,
        right_fraction as f32,
        bottom_fraction as f32,
    );
    let (a, b, c, d) = (
        (left_fraction * top_fraction * scaling as f32) as i32,
        (right_fraction * top_fraction * scaling as f32) as i32,
        (right_fraction * bottom_fraction * scaling as f32) as i32,
        (left_fraction * bottom_fraction * scaling as f32) as i32,
    );
    let (left_weight, top_weight, right_weight, bottom_weight) = (
        (left_fraction * scaling as f32) as i32,
        (top_fraction * scaling as f32) as i32,
        (right_fraction * scaling as f32) as i32,
        (bottom_fraction * scaling as f32) as i32,
    );
    let mut value = 0i32;
    for y in yt..=yb {
        for x in xl..=xr {
            let weight = if y == yt {
                if x == xl {
                    a
                } else if x == xr {
                    b
                } else {
                    top_weight
                }
            } else if y == yb {
                if x == xl {
                    d
                } else if x == xr {
                    c
                } else {
                    bottom_weight
                }
            } else if x == xl {
                left_weight
            } else if x == xr {
                right_weight
            } else {
                scaling
            };
            value = value.wrapping_add(
                weight.wrapping_mul(i32::from(image.data[y as usize * image.cols + x as usize])),
            );
        }
    }
    Ok(((value.wrapping_add(scaling2 / 2) / scaling2 / 1024) & 0xff) as u8)
}
