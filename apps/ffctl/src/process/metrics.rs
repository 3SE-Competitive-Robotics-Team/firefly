//! 进程占用采样：直接读 `/proc/<pid>/stat` 与 `/proc/<pid>/statm`。
//!
//! 只在 Linux 目标上有数据；macOS 上 `/proc` 不存在，读取失败时调用方显示 `-`。

use std::fs;
use std::time::Duration;

use nix::unistd::{SysconfVar, sysconf};

#[derive(Clone, Copy)]
pub(crate) struct Sample {
    ticks: u64,
    rss_kib: u64,
}

#[derive(Clone, Copy, Default)]
pub(crate) struct Usage {
    pub(crate) cpu_percent: f64,
    pub(crate) rss_mib: f64,
}

pub(crate) fn sample(pid: u32) -> Option<Sample> {
    let stat = fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
    let ticks = parse_stat(&stat)?;
    let statm = fs::read_to_string(format!("/proc/{pid}/statm")).ok()?;
    let pages: u64 = statm.split_whitespace().nth(1)?.parse().ok()?;
    let page_size = page_size()?;
    Some(Sample {
        ticks,
        rss_kib: pages.saturating_mul(page_size) / 1024,
    })
}

pub(crate) fn usage(previous: Option<Sample>, current: Sample, elapsed: Duration) -> Usage {
    let clock_ticks = clock_ticks().unwrap_or(100);
    let seconds = elapsed.as_secs_f64();
    let cpu_percent = match previous {
        Some(previous) if seconds > 0.0 => {
            (current.ticks.saturating_sub(previous.ticks) as f64 / clock_ticks as f64) / seconds
                * 100.0
        }
        _ => 0.0,
    };
    Usage {
        cpu_percent,
        rss_mib: current.rss_kib as f64 / 1024.0,
    }
}

/// 从 `/proc/<pid>/stat` 取 utime + stime（clock ticks）。
fn parse_stat(stat: &str) -> Option<u64> {
    // comm 在括号里，可能含空格；从最后一个 ')' 之后开始数字段。
    let rest = stat.rsplit_once(')')?.1;
    let fields: Vec<&str> = rest.split_whitespace().collect();
    let utime: u64 = fields.get(11)?.parse().ok()?;
    let stime: u64 = fields.get(12)?.parse().ok()?;
    Some(utime.saturating_add(stime))
}

fn clock_ticks() -> Option<i64> {
    sysconf(SysconfVar::CLK_TCK).ok().flatten()
}

fn page_size() -> Option<u64> {
    sysconf(SysconfVar::PAGE_SIZE)
        .ok()
        .flatten()
        .and_then(|value| u64::try_from(value).ok())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_utime_stime_after_parenthesized_comm() {
        // 字段 3..13 各一个 0，然后 utime=7、stime=9。
        let stat = "123 (planner with ) paren) S 0 0 0 0 0 0 0 0 0 0 7 9";
        assert_eq!(parse_stat(stat), Some(16));
    }

    #[test]
    fn rejects_truncated_stat() {
        assert_eq!(parse_stat("123 (fc) S 1 2"), None);
    }

    #[test]
    fn usage_is_zero_without_elapsed_time() {
        let sample = Sample {
            ticks: 10,
            rss_kib: 2048,
        };
        let usage = usage(Some(sample), sample, Duration::ZERO);
        assert!(usage.cpu_percent.abs() < f64::EPSILON);
        assert!((usage.rss_mib - 2.0).abs() < f64::EPSILON);
    }
}
