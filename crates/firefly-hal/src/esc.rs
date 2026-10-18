//! 电调 PWM：推力（N）→ 占空比（ns）的线性映射。
//!
//! 飞控在 `Firefly/Control` 里发的是牛顿，电调吃的是脉宽，两者之间只有这张
//! 单调映射表。参数来自部署配置；非线性推力曲线留给以后按实测标定替换。

/// 推力到占空比的线性映射。
#[derive(Debug, Clone, Copy)]
pub struct ThrustToDuty {
    /// PWM 周期（ns）。
    period_ns: u64,
    /// 零推力脉宽（ns），也是失效/上锁时的输出。
    min_duty_ns: u64,
    /// 满推力脉宽（ns）。
    max_duty_ns: u64,
    /// 单电机推力上限（N）。
    max_thrust_n: f64,
}

impl ThrustToDuty {
    /// 构造并校验参数。
    ///
    /// # Errors
    /// 周期为零、脉宽区间为空或超出周期、推力上限非正。
    pub fn new(
        period_ns: u64,
        min_duty_ns: u64,
        max_duty_ns: u64,
        max_thrust_n: f64,
    ) -> Result<Self, String> {
        if period_ns == 0 {
            return Err("PWM 周期必须为正".to_owned());
        }
        if min_duty_ns >= max_duty_ns {
            return Err(format!(
                "PWM 脉宽区间为空：min={min_duty_ns} ns，max={max_duty_ns} ns"
            ));
        }
        if max_duty_ns > period_ns {
            return Err(format!(
                "PWM 满推力脉宽 {max_duty_ns} ns 超过周期 {period_ns} ns"
            ));
        }
        if !max_thrust_n.is_finite() || max_thrust_n <= 0.0 {
            return Err(format!("单电机推力上限必须为正：{max_thrust_n}"));
        }
        Ok(Self {
            period_ns,
            min_duty_ns,
            max_duty_ns,
            max_thrust_n,
        })
    }

    /// 周期（ns）。
    #[must_use]
    pub const fn period_ns(&self) -> u64 {
        self.period_ns
    }

    /// 零推力脉宽（ns）。
    #[must_use]
    pub const fn min_duty_ns(&self) -> u64 {
        self.min_duty_ns
    }

    /// 推力（N）→ 脉宽（ns），超出 `[0, max_thrust]` 的部分夹紧。
    #[must_use]
    pub fn duty_ns(&self, thrust_n: f64) -> u64 {
        if !thrust_n.is_finite() || thrust_n <= 0.0 {
            return self.min_duty_ns;
        }
        let fraction = (thrust_n / self.max_thrust_n).min(1.0);
        let span = self.max_duty_ns - self.min_duty_ns;
        self.min_duty_ns + (span as f64 * fraction).round() as u64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mapping() -> ThrustToDuty {
        ThrustToDuty::new(2_500_000, 1_000_000, 2_000_000, 5.0).expect("valid mapping")
    }

    #[test]
    fn zero_and_negative_thrust_sit_at_min_duty() {
        assert_eq!(mapping().duty_ns(0.0), 1_000_000);
        assert_eq!(mapping().duty_ns(-1.0), 1_000_000);
        assert_eq!(mapping().duty_ns(f64::NAN), 1_000_000);
    }

    #[test]
    fn full_thrust_sits_at_max_duty() {
        assert_eq!(mapping().duty_ns(5.0), 2_000_000);
        assert_eq!(mapping().duty_ns(9.0), 2_000_000);
    }

    #[test]
    fn half_thrust_sits_in_the_middle() {
        assert_eq!(mapping().duty_ns(2.5), 1_500_000);
    }

    #[test]
    fn mapping_is_monotonic() {
        let mapping = mapping();
        let mut previous = 0;
        for step in 0..=50 {
            let duty = mapping.duty_ns(f64::from(step) / 10.0);
            assert!(duty >= previous, "{duty} < {previous}");
            previous = duty;
        }
    }

    #[test]
    fn invalid_ranges_are_rejected() {
        assert!(ThrustToDuty::new(0, 1, 2, 5.0).is_err());
        assert!(ThrustToDuty::new(2_500_000, 2_000_000, 2_000_000, 5.0).is_err());
        assert!(ThrustToDuty::new(1_000_000, 1_000_000, 2_000_000, 5.0).is_err());
        assert!(ThrustToDuty::new(2_500_000, 1_000_000, 2_000_000, 0.0).is_err());
        assert!(ThrustToDuty::new(2_500_000, 1_000_000, 2_000_000, f64::INFINITY).is_err());
    }
}
