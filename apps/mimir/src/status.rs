//! Bounded, low-rate display metrics derived from existing scalar observations.

#[derive(Default)]
pub(crate) struct Status {
    values: [Option<(f64, f64)>; 6],
    emitted: Option<f64>,
    camera: Rate,
}

/// Count publication intervals over at least one second of measurement time.
#[derive(Default)]
struct Rate {
    start: Option<f64>,
    latest: Option<f64>,
    intervals: u32,
}

impl Rate {
    fn observe(&mut self, stamp: f64) -> Option<f64> {
        if self.latest.is_some_and(|previous| stamp <= previous) {
            return None;
        }
        let gap = self.latest.map_or(0., |previous| stamp - previous);
        self.latest = Some(stamp);
        if self.start.is_none() || gap > 2. {
            self.start = Some(stamp);
            self.intervals = 0;
            return None;
        }
        self.intervals += 1;
        let elapsed = stamp - self.start?;
        if elapsed < 1. {
            return None;
        }
        let rate = f64::from(self.intervals) / elapsed;
        self.start = Some(stamp);
        self.intervals = 0;
        Some(rate)
    }
}

impl Status {
    pub fn observe(&mut self, name: &str, stamp: f64, values: &[f64]) -> Option<[String; 6]> {
        if !stamp.is_finite() {
            return None;
        }
        let sample = match name {
            "fc/debug/tick_rate" => Some((0, values.first().copied())),
            "rootslam/frame_processing_ms" => Some((1, values.first().copied())),
            "vio/prediction/posterior_age_s" => Some((2, values.first().map(|v| v * 1000.))),
            "render/capture/publish_s" => {
                if let Some(rate) = self.camera.observe(stamp) {
                    self.values[3] = Some((stamp, rate));
                }
                Some((4, values.get(2).map(|v| v * 1000.)))
            }
            "vio/diagnostics/tracks" => Some((5, values.get(3).copied())),
            _ => None,
        };
        if let Some((index, Some(value))) = sample
            && self.values[index].is_none_or(|(previous, _)| stamp >= previous)
        {
            self.values[index] = Some((stamp, value));
        }
        if self.emitted.is_some_and(|previous| stamp - previous < 0.5) {
            return None;
        }
        self.emitted = Some(stamp);
        Some(std::array::from_fn(|index| {
            let unit = ["Hz", "ms", "ms", "FPS", "ms", "IDs"][index];
            let value = match self.values[index] {
                Some((time, value))
                    if value.is_finite() && (0. ..=2.).contains(&(stamp - time)) =>
                {
                    if index == 5 {
                        format!("{value:.0}")
                    } else {
                        format!("{value:.1}")
                    }
                }
                _ => "—".to_owned(),
            };
            format!("**{value} {unit}**")
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn metrics_have_independent_values_units_and_freshness() {
        let mut status = Status::default();
        let texts = status.observe("fc/debug/tick_rate", 1., &[999.5]).unwrap();
        assert_eq!(texts[0], "**999.5 Hz**");
        assert_eq!(texts[1], "**— ms**");
        assert!(
            status
                .observe("rootslam/frame_processing_ms", 1.1, &[6.5])
                .is_none()
        );
        assert!(
            status
                .observe("vio/prediction/posterior_age_s", 1.2, &[0.012])
                .is_none()
        );
        assert!(status.observe("fc/debug/tick_rate", 0.9, &[1.]).is_none());
        let texts = status
            .observe("vio/diagnostics/tracks", 1.5, &[90., 80., 70., 60.])
            .unwrap();
        assert_eq!(&texts[..3], &["**999.5 Hz**", "**6.5 ms**", "**12.0 ms**"]);
        assert_eq!(texts[5], "**60 IDs**");
        let texts = status.observe("fc/debug/state", 4., &[1.]).unwrap();
        assert!(texts.iter().all(|text| text.contains('—')));
        assert_eq!(
            status
                .observe("fc/debug/tick_rate", 4.5, &[f64::NAN])
                .unwrap()[0],
            "**— Hz**"
        );
    }

    #[test]
    fn camera_rate_counts_unique_publications_and_restarts_after_a_gap() {
        let mut status = Status::default();
        for frame in 0..=60 {
            let stamp = 1. + f64::from(frame) / 60.;
            let texts = status.observe("render/capture/publish_s", stamp, &[0., 0., 0.008]);
            assert!(
                status
                    .observe("render/capture/publish_s", stamp, &[0., 0., 0.008])
                    .is_none()
            );
            if frame == 60 {
                let texts = texts.unwrap();
                assert_eq!(texts[3], "**60.0 FPS**");
                assert_eq!(texts[4], "**8.0 ms**");
            }
        }
        assert_eq!(
            status.observe("fc/debug/state", 5., &[1.]).unwrap()[3],
            "**— FPS**"
        );
        assert_eq!(
            status
                .observe("render/capture/publish_s", 6., &[0.; 3])
                .unwrap()[3],
            "**— FPS**"
        );
        assert_eq!(
            status
                .observe("render/capture/publish_s", 7., &[0.; 3])
                .unwrap()[3],
            "**1.0 FPS**"
        );
    }
}
