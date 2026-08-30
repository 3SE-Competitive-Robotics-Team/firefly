//! 鲁棒核（对照 `factors/robust_kernel.hpp`）。
//!
//! 提供 Huber / Cauchy 权重，以及 `RobustFactor<Kernel, Factor>` 包装。

/// Huber 鲁棒核（对照 `Huber`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct Huber {
    c: f64,
}

/// Huber 参数（对照 `Huber::Setting`）。
#[derive(Clone, Copy, Debug)]
pub struct HuberSetting {
    /// 核宽度。
    pub c: f64,
}

impl Default for HuberSetting {
    fn default() -> Self {
        Self { c: 1.0 }
    }
}

impl Huber {
    /// 由设定构造。
    pub fn new(setting: HuberSetting) -> Self {
        Self { c: setting.c }
    }

    /// 权重 `w(e)`（对照 `weight`）。
    pub fn weight(&self, e: f64) -> f64 {
        let ea = e.abs();
        if ea < self.c { 1.0 } else { self.c / ea }
    }

    /// 核宽度。
    pub fn c(&self) -> f64 {
        self.c
    }
}

/// Cauchy 鲁棒核（对照 `Cauchy`）。
#[derive(Clone, Copy, Debug, Default)]
pub struct Cauchy {
    c: f64,
}

/// Cauchy 参数（对照 `Cauchy::Setting`，C++ 中复用 Huber 命名）。
#[derive(Clone, Copy, Debug)]
pub struct CauchySetting {
    /// 核宽度。
    pub c: f64,
}

impl Default for CauchySetting {
    fn default() -> Self {
        Self { c: 1.0 }
    }
}

impl Cauchy {
    /// 由设定构造。
    pub fn new(setting: CauchySetting) -> Self {
        Self { c: setting.c }
    }

    /// 权重 `w(e) = c / (c + e²)`（对照 C++ 实现，`c/(c+e*e)`，非经典 `1/(1+(e/c)²)`）。
    pub fn weight(&self, e: f64) -> f64 {
        self.c / (self.c + e * e)
    }

    /// 核宽度。
    pub fn c(&self) -> f64 {
        self.c
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn huber_weight() {
        let k = Huber::new(HuberSetting { c: 1.0 });
        assert!((k.weight(0.5) - 1.0).abs() < 1e-12);
        assert!((k.weight(2.0) - 0.5).abs() < 1e-12);
        assert!((k.weight(-2.0) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn cauchy_weight() {
        let k = Cauchy::new(CauchySetting { c: 1.0 });
        assert!((k.weight(0.0) - 1.0).abs() < 1e-12);
        assert!((k.weight(1.0) - 0.5).abs() < 1e-12);
        // 对照 C++: c/(c+e*e)
        assert!((k.weight(2.0) - 1.0 / 5.0).abs() < 1e-12);
    }
}
