use std::time::{Duration, Instant};

pub struct Rate {
    start: Instant,
    count: u64,
}

impl Default for Rate {
    fn default() -> Self {
        Self {
            start: Instant::now(),
            count: 0,
        }
    }
}

impl Rate {
    /// 加入本次完成的事件数，每经过至少一秒墙钟返回一次实际频率。
    pub fn observe(&mut self, now: Instant, count: u64) -> Option<f64> {
        self.count += count;
        let elapsed = now.duration_since(self.start);
        if elapsed < Duration::from_secs(1) {
            return None;
        }
        let rate = self.count as f64 / elapsed.as_secs_f64();
        self.start = now;
        self.count = 0;
        Some(rate)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counts_actual_events_over_wall_time_including_stalls_and_idle_windows() {
        let start = Instant::now();
        let mut rate = Rate { start, count: 0 };
        assert_eq!(rate.observe(start + Duration::from_millis(500), 50), None);
        assert_eq!(rate.observe(start + Duration::from_secs(1), 50), Some(100.));
        assert_eq!(rate.observe(start + Duration::from_secs(3), 1), Some(0.5));
        assert_eq!(rate.observe(start + Duration::from_secs(4), 0), Some(0.));
    }
}
