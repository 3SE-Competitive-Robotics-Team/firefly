/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2023 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

//! Morton encoding and allocation-key filtering for 19 coordinate bits per axis.
use crate::image::invalid;
use firefly_error::Result;
use nalgebra::Vector3;
pub const SCALE_OFFSET: u32 = 5;
pub const SCALE_MASK: u64 = 31;
pub const KEY_SCALE_LIMIT: i32 = 19;
fn mask(scale: i32) -> Result<u64> {
    if !(0..=KEY_SCALE_LIMIT).contains(&scale) {
        return Err(invalid("Morton scale outside [0,19]"));
    }
    Ok(0x1ff_ffff_ffff_ffffu64 & (!0u64 << (3 * scale)))
}
#[must_use]
pub fn is_valid_coord(c: &Vector3<i32>) -> bool {
    c.iter().all(|v| *v >= 0 && *v < 524_288)
}
#[must_use]
pub fn is_valid(key: u64, limit: i32) -> bool {
    let s = (key & SCALE_MASK) as i32;
    s <= limit && mask(s).is_ok_and(|m| (key >> SCALE_OFFSET) & !m == 0)
}
/// # Errors
/// Returns an error if `value` exceeds the 19-bit coordinate range.
pub fn expand(value: u64) -> Result<u64> {
    if value > 0x7ffff {
        return Err(invalid("Morton coordinate exceeds 19 bits"));
    }
    let mut x = value & 0x7ffff;
    x = (x | x << 32) & 0x001f_0000_0000_ffff;
    x = (x | x << 16) & 0x001f_0000_ff00_00ff;
    x = (x | x << 8) & 0x100f_00f0_0f00_f00f;
    x = (x | x << 4) & 0x10c3_0c30_c30c_30c3;
    Ok((x | x << 2) & 0x1249_2492_4924_9249)
}
/// # Errors
/// Returns an error if `value` exceeds the accepted interleaved-component mask.
pub fn compact(value: u64) -> Result<u64> {
    if value > 0x0049_2492_4924_9249 {
        return Err(invalid("Morton component outside mask"));
    }
    let mut x = value & 0x0049_2492_4924_9249;
    x = (x | x >> 2) & 0x10c3_0c30_c30c_30c3;
    x = (x | x >> 4) & 0x100f_00f0_0f00_f00f;
    x = (x | x >> 8) & 0x001f_0000_ff00_00ff;
    x = (x | x >> 16) & 0x001f_0000_0000_ffff;
    Ok((x | x >> 32) & 0x001f_ffff)
}
/// # Errors
/// Returns an error if any coordinate is negative or at least `2^19`.
pub fn encode_code(coord: &Vector3<i32>) -> Result<u64> {
    if !is_valid_coord(coord) {
        return Err(invalid("coordinate outside Morton range"));
    }
    Ok(expand(coord.x as u64)? | (expand(coord.y as u64)? << 1) | (expand(coord.z as u64)? << 2))
}
/// # Errors
/// Returns an error if an interleaved component exceeds the accepted Morton mask.
pub fn decode_code(code: u64) -> Result<Vector3<i32>> {
    Ok(Vector3::new(
        compact(code)? as i32,
        compact(code >> 1)? as i32,
        compact(code >> 2)? as i32,
    ))
}
/// # Errors
/// Returns an error for coordinates outside the 19-bit range or a scale outside `0..=19`.
pub fn encode_key(coord: &Vector3<i32>, scale: i32) -> Result<(u64, bool)> {
    let detailed = encode_code(coord)?;
    let code = detailed & mask(scale)?;
    Ok(((code << SCALE_OFFSET) | scale as u64, detailed == code))
}
/// # Errors
/// Returns an error if `scale` is outside `0..=19`.
pub fn encode_key_from_code(code: u64, scale: i32) -> Result<(u64, bool)> {
    let filtered = code & mask(scale)?;
    Ok(((code << SCALE_OFFSET) | scale as u64, code == filtered))
}
/// # Errors
/// Returns an error for an invalid Morton key or a scale outside `0..=19`.
pub fn key_to_code(key: u64) -> Result<u64> {
    if !is_valid(key, KEY_SCALE_LIMIT) {
        return Err(invalid("invalid Morton key"));
    }
    Ok(key >> SCALE_OFFSET)
}
/// # Errors
/// Returns an error for an invalid Morton key or a scale outside `0..=19`.
pub fn key_to_scale(key: u64) -> Result<i32> {
    key_to_code(key)?;
    Ok((key & SCALE_MASK) as i32)
}
/// # Errors
/// Returns an error for an invalid Morton key or a scale outside `0..=19`.
pub fn key_to_coord(key: u64) -> Result<Vector3<i32>> {
    decode_code(key_to_code(key)?)
}
/// # Errors
/// Returns an error for an invalid Morton key or a scale outside `0..=19`.
pub fn decode_key(key: u64) -> Result<(Vector3<i32>, i32)> {
    Ok((key_to_coord(key)?, key_to_scale(key)?))
}
/// # Errors
/// Returns an error if `scale` is outside `0..=19`.
pub fn code_to_child_index(code: u64, scale: i32) -> Result<usize> {
    mask(scale)?;
    Ok(((code >> (3 * scale)) & 7) as usize)
}
/// # Errors
/// Returns an error for an invalid Morton key or a scale outside `0..=19`.
pub fn key_at_scale(key: u64, scale: i32) -> Result<(u64, bool)> {
    let old = key_to_scale(key)?;
    Ok((
        (key & (mask(scale)? << SCALE_OFFSET)) | scale as u64,
        old <= scale,
    ))
}
/// # Errors
/// Returns an error for an invalid Morton key or a scale outside `0..=19`.
pub fn code_at_scale(key: u64, scale: i32) -> Result<(u64, bool)> {
    Ok((
        key_to_code(key)? & mask(scale)?,
        key_to_scale(key)? <= scale,
    ))
}
/// # Errors
/// Returns an error if `key` is invalid or has no encodable parent.
pub fn parent_key(key: u64) -> Result<u64> {
    if !is_valid(key, KEY_SCALE_LIMIT - 1) {
        return Err(invalid("key has no encodable parent"));
    }
    let scale = key_to_scale(key)? + 1;
    Ok((key & (mask(scale)? << SCALE_OFFSET)) | scale as u64)
}
/// # Errors
/// Returns an error for an invalid Morton key or a scale outside `0..=19`.
pub fn block_key(key: u64, scale: i32) -> Result<u64> {
    key_to_code(key)?;
    Ok(key & ((mask(scale)? << SCALE_OFFSET) | SCALE_MASK))
}
/// # Errors
/// Returns an error for an invalid Morton key or a scale outside `0..=19`.
pub fn block_code(key: u64, scale: i32) -> Result<u64> {
    Ok(key_to_code(key)? & mask(scale)?)
}
/// # Errors
/// Returns an error if the parent key is invalid or its scale has no encodable child.
pub fn parent_to_child_key(parent: u64, code: u64) -> Result<u64> {
    let scale = key_to_scale(parent)? - 1;
    mask(scale)?;
    Ok(((parent | (code << ((scale * 3) as u32 + SCALE_OFFSET))) & !SCALE_MASK) | scale as u64)
}
/// # Errors
/// Returns an error if either key is invalid or the parent violates the source scale
/// precondition.
pub fn is_child(parent: u64, child: u64) -> Result<bool> {
    if !is_valid(parent, KEY_SCALE_LIMIT - 1) {
        return Err(invalid("source parent-key scale precondition violated"));
    }
    let p = key_to_scale(parent)?;
    let c = key_to_scale(child)?;
    Ok(c < p && (key_to_code(parent)? ^ (key_to_code(child)? & mask(p)?)) == 0)
}
/// # Errors
/// Returns an error for invalid keys or an unencodable parent scale.
pub fn is_siblings(a: u64, b: u64) -> Result<bool> {
    let scale = key_to_scale(a)?;
    if scale != key_to_scale(b)? {
        return Ok(false);
    }
    let m = mask(scale + 1)?;
    Ok((key_to_code(a)? & m) == (key_to_code(b)? & m))
}
pub fn sort_keys(keys: &mut [u64], descending: bool) {
    keys.sort_unstable();
    if descending {
        keys.reverse();
    }
}
pub fn unique_keys(keys: &mut [u64], output: &mut Vec<u64>, sort: bool) {
    if sort {
        keys.sort_unstable();
    }
    let Some(&first) = keys.first() else {
        return;
    };
    output.push(first);
    for &key in keys.iter() {
        if output.last() != Some(&key) {
            output.push(key);
        }
    }
}
/// # Errors
/// Returns an error if a key cannot be decoded or the required last output key is absent.
pub fn unique_codes(keys: &mut [u64], output: &mut Vec<u64>, sort: bool) -> Result<()> {
    if sort {
        keys.sort_unstable();
    }
    let Some(&first) = keys.first() else {
        return Ok(());
    };
    output.push(first);
    for &key in keys.iter() {
        let last = *output
            .last()
            .ok_or_else(|| invalid("empty Morton output"))?;
        if key_to_code(last)? != key_to_code(key)? {
            output.push(key);
        } else if key_to_scale(last)? > key_to_scale(key)? {
            *output
                .last_mut()
                .ok_or_else(|| invalid("empty Morton output"))? = key;
        }
    }
    Ok(())
}
/// # Errors
/// Returns an error for invalid keys/scales or an absent required last output key.
pub fn unique_allocation(
    keys: &mut [u64],
    scale: i32,
    output: &mut Vec<u64>,
    sort: bool,
) -> Result<()> {
    if sort {
        keys.sort_unstable();
    }
    let Some(&first) = keys.first() else {
        return Ok(());
    };
    output.push(block_key(first, scale)?);
    for &key in keys.iter() {
        let k = block_key(key, scale)?;
        let last = *output
            .last()
            .ok_or_else(|| invalid("empty Morton output"))?;
        if is_child(last, k)? {
            *output
                .last_mut()
                .ok_or_else(|| invalid("empty Morton output"))? = k;
        } else if last != k {
            output.push(k);
        }
    }
    Ok(())
}
/// # Errors
/// Returns an error for invalid keys/scales or when nonempty input has no key eligible at the
/// requested scale.
pub fn unique_at_scale(
    keys: &mut [u64],
    scale: i32,
    output: &mut Vec<u64>,
    sort: bool,
) -> Result<()> {
    if sort {
        keys.sort_unstable();
    }
    if keys.is_empty() {
        return Ok(());
    }
    let mut restart = 1;
    let mut found = false;
    for &key in keys.iter() {
        let (k, valid) = key_at_scale(key, scale)?;
        if valid {
            output.push(k);
            found = true;
            break;
        }
        restart += 1;
    }
    if !found {
        return Err(invalid(
            "source unique_at_scale restart iterator exceeds end",
        ));
    }
    for &key in keys.iter().skip(restart) {
        let (k, valid) = key_at_scale(key, scale)?;
        if valid && output.last() != Some(&k) {
            output.push(k);
        }
    }
    Ok(())
}
