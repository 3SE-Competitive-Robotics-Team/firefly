/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

/// Borrowed explicit indices or the complete contiguous correspondence range.
#[derive(Clone, Copy, Debug)]
pub enum Indices<'a> {
    Explicit(&'a [usize]),
    All(usize),
}
impl<'a> Indices<'a> {
    #[must_use]
    pub fn new(indices: &'a [usize]) -> Self {
        Self::Explicit(indices)
    }
    #[must_use]
    pub fn all(number_correspondences: usize) -> Self {
        Self::All(number_correspondences)
    }
    #[must_use]
    pub fn size(&self) -> usize {
        match self {
            Self::Explicit(i) => i.len(),
            Self::All(n) => *n,
        }
    }
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.size() == 0
    }
    #[must_use]
    pub fn get(&self, i: usize) -> usize {
        match self {
            Self::Explicit(indices) => indices[i],
            Self::All(_) => i,
        }
    }
}
impl<'a> From<&'a [usize]> for Indices<'a> {
    fn from(value: &'a [usize]) -> Self {
        Self::new(value)
    }
}
impl From<usize> for Indices<'_> {
    fn from(value: usize) -> Self {
        Self::all(value)
    }
}
