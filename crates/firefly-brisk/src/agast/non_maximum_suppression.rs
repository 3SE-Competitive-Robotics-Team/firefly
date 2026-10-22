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

//! Connected four-neighbour score maxima, retaining the source's tie direction.
use crate::vectorized_filters::invalid;
use firefly_error::Result;
use purecv::features2d::KeyPoint;
/// # Errors
/// Returns an error if score/corner counts differ, coordinates are invalid,
/// or corners are not in row-major order.
pub fn non_maximum_suppression(
    scores: &[i32],
    corners: &[KeyPoint],
    output: &mut Vec<KeyPoint>,
    flags: &mut Vec<i32>,
) -> Result<()> {
    if scores.len() != corners.len() {
        return Err(invalid("AGAST scores/corners length mismatch"));
    }
    if corners
        .windows(2)
        .any(|w| w[0].pt.y > w[1].pt.y || (w[0].pt.y == w[1].pt.y && w[0].pt.x > w[1].pt.x))
    {
        return Err(invalid("AGAST NMS requires row-major corners"));
    }
    flags.clear();
    flags.resize(corners.len(), -1);
    let (mut last_row, mut next_last_row) = (0i32, 0i32);
    let (mut last_index, mut next_index) = (0usize, 0usize);
    for i in 0..corners.len() {
        let p = &corners[i];
        if ((last_row + 1) as f32) < p.pt.y {
            last_row = next_last_row;
            last_index = next_index;
        }
        if next_last_row as f32 != p.pt.y {
            next_last_row = p.pt.y as i32;
            next_index = i;
        }
        if (last_row + 1) as f32 == p.pt.y {
            while last_index < corners.len()
                && corners[last_index].pt.x < p.pt.x
                && corners[last_index].pt.y == last_row as f32
            {
                last_index += 1;
            }
            if last_index < corners.len() && corners[last_index].pt.x == p.pt.x && last_index != i {
                let t = find_max(flags, last_index)?;
                if scores[i] < scores[t] {
                    flags[i] = t as i32;
                } else {
                    flags[t] = i as i32;
                }
            }
        }
        if i != 0 && corners[i - 1].pt.y == p.pt.y && corners[i - 1].pt.x + 1.0 == p.pt.x {
            let above = flags[i];
            let t = find_max(flags, i - 1)?;
            if above == -1 {
                if t != i {
                    if scores[i] < scores[t] {
                        flags[i] = t as i32;
                    } else {
                        flags[t] = i as i32;
                    }
                }
            } else if t != above as usize {
                let a = above as usize;
                if scores[a] < scores[t] {
                    flags[a] = t as i32;
                    flags[i] = t as i32;
                } else {
                    flags[t] = above;
                    flags[i] = above;
                }
            }
        }
    }
    output.clear();
    output.extend(
        corners
            .iter()
            .enumerate()
            .filter(|(i, _)| flags[*i] == -1)
            .map(|(_, p)| p.clone()),
    );
    Ok(())
}
fn find_max(flags: &[i32], mut i: usize) -> Result<usize> {
    for _ in 0..=flags.len() {
        let n = *flags
            .get(i)
            .ok_or_else(|| invalid("invalid AGAST NMS parent"))?;
        if n == -1 {
            return Ok(i);
        }
        if n < 0 {
            return Err(invalid("invalid AGAST NMS parent"));
        }
        i = n as usize;
    }
    Err(invalid("AGAST NMS parent cycle"))
}
