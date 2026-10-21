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

//! oast9-16 adaptive segment-test detector.
use crate::agast::ast_detector::{
    AstDetector, AstDetectorState, checked_image, keypoint, neighborhood,
};
use firefly_error::Result;
use purecv::core::Matrix;
use purecv::features2d::KeyPoint;
#[derive(Clone, Debug, Default)]
pub struct OastDetector916 {
    pub(crate) state: AstDetectorState,
}
impl OastDetector916 {
    pub const BORDER: usize = 3;
    pub const PATTERN: [(isize, isize); 16] = [
        (-3, 0),
        (-3, -1),
        (-2, -2),
        (-1, -3),
        (0, -3),
        (1, -3),
        (2, -2),
        (3, -1),
        (3, 0),
        (3, 1),
        (2, 2),
        (1, 3),
        (0, 3),
        (-1, 3),
        (-2, 2),
        (-3, 1),
    ];
    #[must_use]
    pub fn new(width: usize, height: usize, threshold: i32) -> Self {
        Self {
            state: AstDetectorState::new(width, height, threshold),
        }
    }
    pub fn set_threshold(&mut self, threshold: i32, upper: i32, lower: i32) {
        self.state.set_threshold(threshold, upper, lower);
    }
    pub fn set_image_size(&mut self, width: usize, height: usize) {
        self.state.width = width;
        self.state.height = height;
    }
    /// # Errors
    /// Returns an error if image dimensions/storage or threshold-map geometry is invalid.
    pub fn detect(
        &mut self,
        image: &[u8],
        corners: &mut Vec<KeyPoint>,
        threshold_map: Option<&Matrix<u8>>,
    ) -> Result<()> {
        checked_image(&self.state, image)?;
        corners.clear();
        if let Some(m) = threshold_map {
            crate::vectorized_filters::check(m)?;
            if m.cols != self.state.width || m.rows != self.state.height {
                return Err(crate::vectorized_filters::invalid(
                    "threshold map shape mismatch",
                ));
            }
        }
        for y in Self::BORDER..self.state.height.saturating_sub(Self::BORDER) {
            for x in Self::BORDER..self.state.width.saturating_sub(Self::BORDER) {
                let index = y * self.state.width + x;
                let mut threshold = self.state.threshold;
                if let Some(map) = threshold_map {
                    let value = i32::from(map.data[index]);
                    let (upper, lower, cmp) = self.state.threshold_bounds()?;
                    if value < cmp {
                        continue;
                    }
                    let value = if value < lower { lower } else { value };
                    let value = if value > upper { upper } else { value };
                    threshold = value * threshold / 100;
                }
                let samples = neighborhood(image, index, self.state.width, &Self::PATTERN)?;
                let bright_limit = i32::from(image[index]) + threshold;
                let dark_limit = i32::from(image[index]) - threshold;
                if Self::classify(&samples, bright_limit, dark_limit) {
                    corners.push(keypoint(x, y));
                }
            }
        }
        Ok(())
    }
    fn classify(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        Self::classify_branch_55(samples, bright_limit, dark_limit)
    }
    #[inline]
    fn classify_branch_0(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_1(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_2(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_3(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[14] > bright_limit {
            (samples[15] > bright_limit) && (Self::classify_branch_2(samples, bright_limit))
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
    fn classify_branch_4(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_5(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] > bright_limit {
            if samples[3] > bright_limit {
                Self::classify_branch_0(samples, bright_limit)
            } else {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && ({
                                (samples[12] > bright_limit)
                                    && (Self::classify_branch_1(samples, bright_limit))
                            })
                    })
            }
        } else {
            if samples[7] < dark_limit {
                Self::classify_branch_3(samples, bright_limit, dark_limit)
            } else {
                (samples[14] > bright_limit)
                    && ({
                        (samples[15] > bright_limit)
                            && (Self::classify_branch_4(samples, bright_limit))
                    })
            }
        }
    }
    #[inline]
    fn classify_branch_6(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_7(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_8(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_9(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_10(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[5] > bright_limit {
            Self::classify_branch_5(samples, bright_limit, dark_limit)
        } else if samples[5] < dark_limit {
            if samples[12] > bright_limit {
                {
                    (samples[13] > bright_limit)
                        && ({
                            (samples[14] > bright_limit)
                                && (Self::classify_branch_6(samples, bright_limit))
                        })
                }
            } else {
                Self::classify_branch_7(samples, dark_limit)
            }
        } else {
            if samples[12] > bright_limit {
                {
                    (samples[13] > bright_limit)
                        && ({
                            (samples[14] > bright_limit)
                                && (Self::classify_branch_8(samples, bright_limit))
                        })
                }
            } else {
                Self::classify_branch_9(samples, dark_limit)
            }
        }
    }
    #[inline]
    fn classify_branch_11(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_12(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_13(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[11] > bright_limit {
            {
                (samples[12] > bright_limit)
                    && ({
                        (samples[13] > bright_limit)
                            && (if samples[10] > bright_limit {
                                Self::classify_branch_11(samples, bright_limit)
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
                                            && (Self::classify_branch_12(samples, dark_limit))
                                    })
                            })
                    })
            }
        }
    }
    #[inline]
    fn classify_branch_14(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_15(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_16(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[11] > bright_limit {
            {
                (samples[12] > bright_limit)
                    && ({
                        (samples[13] > bright_limit)
                            && (if samples[10] > bright_limit {
                                Self::classify_branch_14(samples, bright_limit)
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
            (samples[11] < dark_limit) && (Self::classify_branch_15(samples, dark_limit))
        }
    }
    #[inline]
    fn classify_branch_17(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_18(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[8] > bright_limit {
            if samples[12] > bright_limit {
                Self::classify_branch_17(samples, bright_limit)
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
    fn classify_branch_19(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_20(samples: &[i32; 16], dark_limit: i32) -> bool {
        {
            (samples[7] < dark_limit)
                && ({
                    (samples[8] < dark_limit)
                        && (if samples[6] < dark_limit {
                            Self::classify_branch_19(samples, dark_limit)
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
    fn classify_branch_21(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_22(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[8] > bright_limit {
            if samples[12] > bright_limit {
                Self::classify_branch_21(samples, bright_limit)
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
    fn classify_branch_23(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_24(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] > bright_limit {
            {
                (samples[10] > bright_limit)
                    && ({
                        (samples[11] > bright_limit)
                            && (Self::classify_branch_22(samples, bright_limit))
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
                                                    && (Self::classify_branch_23(
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
    fn classify_branch_25(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] > bright_limit {
            if samples[4] > bright_limit {
                Self::classify_branch_10(samples, bright_limit, dark_limit)
            } else if samples[4] < dark_limit {
                Self::classify_branch_13(samples, bright_limit, dark_limit)
            } else {
                Self::classify_branch_16(samples, bright_limit, dark_limit)
            }
        } else {
            if samples[2] < dark_limit {
                if samples[9] > bright_limit {
                    {
                        (samples[10] > bright_limit)
                            && ({
                                (samples[11] > bright_limit)
                                    && (Self::classify_branch_18(samples, bright_limit))
                            })
                    }
                } else {
                    (samples[9] < dark_limit) && (Self::classify_branch_20(samples, dark_limit))
                }
            } else {
                Self::classify_branch_24(samples, bright_limit, dark_limit)
            }
        }
    }
    #[inline]
    fn classify_branch_26(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_27(samples: &[i32; 16], bright_limit: i32) -> bool {
        {
            (samples[7] > bright_limit)
                && ({
                    (samples[8] > bright_limit)
                        && (if samples[6] > bright_limit {
                            Self::classify_branch_26(samples, bright_limit)
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
    fn classify_branch_28(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_29(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[8] < dark_limit {
            if samples[12] < dark_limit {
                Self::classify_branch_28(samples, dark_limit)
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
    fn classify_branch_30(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_31(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_32(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[11] > bright_limit {
            {
                (samples[7] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && ({
                                (samples[9] > bright_limit)
                                    && (Self::classify_branch_30(samples, bright_limit))
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
                                        Self::classify_branch_31(samples, dark_limit)
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
    fn classify_branch_33(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_34(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
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
                                    && (Self::classify_branch_33(samples, dark_limit))
                            })
                    })
            }
        }
    }
    #[inline]
    fn classify_branch_35(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_36(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
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
                            && (Self::classify_branch_35(samples, dark_limit))
                    })
            }
        }
    }
    #[inline]
    fn classify_branch_37(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_38(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_39(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_40(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[7] > bright_limit {
            Self::classify_branch_36(samples, bright_limit, dark_limit)
        } else if samples[7] < dark_limit {
            if samples[3] < dark_limit {
                Self::classify_branch_37(samples, dark_limit)
            } else {
                {
                    (samples[10] < dark_limit)
                        && ({
                            (samples[11] < dark_limit)
                                && ({
                                    (samples[12] < dark_limit)
                                        && (Self::classify_branch_38(samples, dark_limit))
                                })
                        })
                }
            }
        } else {
            {
                (samples[14] < dark_limit)
                    && ({
                        (samples[15] < dark_limit)
                            && (Self::classify_branch_39(samples, dark_limit))
                    })
            }
        }
    }
    #[inline]
    fn classify_branch_41(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_42(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
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
                                    && (Self::classify_branch_41(samples, dark_limit))
                            })
                    })
            }
        }
    }
    #[inline]
    fn classify_branch_43(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_44(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_45(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[11] > bright_limit {
            Self::classify_branch_43(samples, bright_limit)
        } else {
            (samples[11] < dark_limit)
                && ({
                    (samples[12] < dark_limit)
                        && ({
                            (samples[13] < dark_limit)
                                && (if samples[10] < dark_limit {
                                    Self::classify_branch_44(samples, dark_limit)
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
    fn classify_branch_46(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_47(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_48(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[8] < dark_limit {
            if samples[12] < dark_limit {
                Self::classify_branch_47(samples, dark_limit)
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
    fn classify_branch_49(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[9] > bright_limit {
            {
                (samples[7] > bright_limit)
                    && ({
                        (samples[8] > bright_limit)
                            && ({
                                (samples[10] > bright_limit)
                                    && ({
                                        (samples[11] > bright_limit)
                                            && (Self::classify_branch_46(samples, bright_limit))
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
                                    && (Self::classify_branch_48(samples, dark_limit))
                            })
                    })
            }
        }
    }
    #[inline]
    fn classify_branch_50(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[2] > bright_limit {
            if samples[9] > bright_limit {
                Self::classify_branch_27(samples, bright_limit)
            } else {
                (samples[9] < dark_limit)
                    && ({
                        (samples[10] < dark_limit)
                            && ({
                                (samples[11] < dark_limit)
                                    && (Self::classify_branch_29(samples, dark_limit))
                            })
                    })
            }
        } else {
            if samples[2] < dark_limit {
                if samples[4] > bright_limit {
                    Self::classify_branch_32(samples, bright_limit, dark_limit)
                } else if samples[4] < dark_limit {
                    if samples[5] > bright_limit {
                        Self::classify_branch_34(samples, bright_limit, dark_limit)
                    } else if samples[5] < dark_limit {
                        Self::classify_branch_40(samples, bright_limit, dark_limit)
                    } else {
                        Self::classify_branch_42(samples, bright_limit, dark_limit)
                    }
                } else {
                    Self::classify_branch_45(samples, bright_limit, dark_limit)
                }
            } else {
                Self::classify_branch_49(samples, bright_limit, dark_limit)
            }
        }
    }
    #[inline]
    fn classify_branch_51(samples: &[i32; 16], bright_limit: i32) -> bool {
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
    fn classify_branch_52(samples: &[i32; 16], bright_limit: i32) -> bool {
        if samples[6] > bright_limit {
            if samples[5] > bright_limit {
                Self::classify_branch_51(samples, bright_limit)
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
    fn classify_branch_53(samples: &[i32; 16], dark_limit: i32) -> bool {
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
    fn classify_branch_54(samples: &[i32; 16], dark_limit: i32) -> bool {
        if samples[6] < dark_limit {
            if samples[5] < dark_limit {
                Self::classify_branch_53(samples, dark_limit)
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
    fn classify_branch_55(samples: &[i32; 16], bright_limit: i32, dark_limit: i32) -> bool {
        if samples[0] > bright_limit {
            Self::classify_branch_25(samples, bright_limit, dark_limit)
        } else if samples[0] < dark_limit {
            Self::classify_branch_50(samples, bright_limit, dark_limit)
        } else if samples[7] > bright_limit {
            {
                (samples[8] > bright_limit)
                    && ({
                        (samples[9] > bright_limit)
                            && (Self::classify_branch_52(samples, bright_limit))
                    })
            }
        } else {
            {
                (samples[7] < dark_limit)
                    && ({
                        (samples[8] < dark_limit)
                            && ({
                                (samples[9] < dark_limit)
                                    && (Self::classify_branch_54(samples, dark_limit))
                            })
                    })
            }
        }
    }
}
impl AstDetector for OastDetector916 {
    fn state(&self) -> &AstDetectorState {
        &self.state
    }
    fn state_mut(&mut self) -> &mut AstDetectorState {
        &mut self.state
    }
    fn detect(
        &mut self,
        image: &[u8],
        corners: &mut Vec<KeyPoint>,
        threshold_map: Option<&Matrix<u8>>,
    ) -> Result<()> {
        Self::detect(self, image, corners, threshold_map)
    }
    fn border_width(&self) -> usize {
        Self::BORDER
    }
    fn corner_score(&self, image: &[u8], index: usize) -> Result<i32> {
        Self::corner_score(self, image, index)
    }
}
