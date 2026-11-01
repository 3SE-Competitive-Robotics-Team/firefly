// Copyright (c) 2021 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original `utils/system_utils.h` and Linux `src/utils/system_utils.cpp`.

#[must_use]
pub fn ensure_trailing_slash(path: &str) -> String {
    if path.is_empty() || path.ends_with('/') {
        path.to_owned()
    } else {
        format!("{path}/")
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MemoryInfo {
    /// Resident bytes, including mapped dataset pages.
    pub resident_memory: u64,
    /// Peak resident bytes since process creation.
    pub resident_memory_peak: u64,
}

/// Samples process resident memory using the original Linux sources. Returns
/// false on unavailable counters; unsupported platforms leave `info` unchanged.
pub fn get_memory_info(info: &mut MemoryInfo) -> bool {
    #[cfg(target_os = "linux")]
    {
        use nix::{
            sys::resource::{UsageWho, getrusage},
            unistd::{SysconfVar, sysconf},
        };
        let Some(sample) = (|| {
            let statm = std::fs::read_to_string("/proc/self/statm").ok()?;
            let pages = statm.split_whitespace().nth(1)?.parse::<u64>().ok()?;
            let page_size = u64::try_from(sysconf(SysconfVar::PAGE_SIZE).ok()??).ok()?;
            let peak_kib = u64::try_from(getrusage(UsageWho::RUSAGE_SELF).ok()?.max_rss()).ok()?;
            Some(MemoryInfo {
                resident_memory: pages.checked_mul(page_size)?,
                resident_memory_peak: peak_kib.checked_mul(1024)?,
            })
        })() else {
            return false;
        };
        *info = sample;
        true
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = info;
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trailing_slash_preserves_empty_and_existing_separator() {
        assert_eq!(ensure_trailing_slash(""), "");
        assert_eq!(ensure_trailing_slash("data/"), "data/");
        assert_eq!(ensure_trailing_slash("data"), "data/");
    }

    #[test]
    #[cfg(target_os = "linux")]
    fn process_memory_uses_byte_units() {
        let mut info = MemoryInfo::default();
        assert!(get_memory_info(&mut info));
        // getrusage can retain the pre-exec high-water mark; VmHWM describes
        // the current address space and is not an upper bound on that counter.
        let subsequent_peak = u64::try_from(
            nix::sys::resource::getrusage(nix::sys::resource::UsageWho::RUSAGE_SELF)
                .unwrap()
                .max_rss(),
        )
        .unwrap()
            * 1024;
        assert!(info.resident_memory > 0);
        assert!(info.resident_memory_peak <= subsequent_peak);
        assert!(info.resident_memory_peak > 1024);
    }
}
