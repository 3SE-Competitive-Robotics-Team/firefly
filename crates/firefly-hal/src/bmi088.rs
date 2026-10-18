//! `BMI088` 六轴 IMU 的寄存器表与单位换算（纯计算，无 I/O）。
//!
//! 加速度计与陀螺是同一封装内的两片独立 SPI 从机，各占一条片选；加速度计的
//! SPI 读在地址之后需要多一个哑字节，陀螺不需要。量程编码与灵敏度取自数据
//! 手册的灵敏度表，`WHO_AM_I` 用于在初始化阶段确认器件型号。

/// SPI 读操作地址位。
const SPI_READ: u8 = 0x80;
/// 加速度计 `WHO_AM_I` 寄存器。
pub const ACCEL_CHIP_ID_REGISTER: u8 = 0x00;
/// 加速度计 `WHO_AM_I` 期望值。
pub const ACCEL_CHIP_ID: u8 = 0x1E;
/// 陀螺 `WHO_AM_I` 寄存器。
pub const GYRO_CHIP_ID_REGISTER: u8 = 0x00;
/// 陀螺 `WHO_AM_I` 期望值。
pub const GYRO_CHIP_ID: u8 = 0x0F;
/// 加速度计数据起始寄存器（`ACC_X_LSB`，小端三轴依次排列）。
pub const ACCEL_DATA_REGISTER: u8 = 0x12;
/// 陀螺数据起始寄存器（`RATE_X_LSB`，小端三轴依次排列）。
pub const GYRO_DATA_REGISTER: u8 = 0x02;
/// 加速度计软复位寄存器。
pub const ACCEL_SOFTRESET_REGISTER: u8 = 0x7E;
/// 陀螺软复位寄存器。
pub const GYRO_SOFTRESET_REGISTER: u8 = 0x14;
/// 软复位写入值。
pub const SOFTRESET_COMMAND: u8 = 0xB6;
/// 加速度计电源控制寄存器。
pub const ACCEL_PWR_CTRL_REGISTER: u8 = 0x7D;
/// 加速度计上电。
pub const ACCEL_PWR_CTRL_ON: u8 = 0x04;
/// 加速度计电源配置寄存器。
pub const ACCEL_PWR_CONF_REGISTER: u8 = 0x7C;
/// 加速度计正常工作（非省电）。
pub const ACCEL_PWR_CONF_ACTIVE: u8 = 0x00;
/// 加速度计配置寄存器（带宽与输出数据率）。
pub const ACCEL_CONF_REGISTER: u8 = 0x40;
/// 加速度计量程寄存器。
pub const ACCEL_RANGE_REGISTER: u8 = 0x41;
/// 陀螺量程寄存器。
pub const GYRO_RANGE_REGISTER: u8 = 0x0F;
/// 陀螺带宽寄存器。
pub const GYRO_BANDWIDTH_REGISTER: u8 = 0x10;
/// 陀螺带宽：2000 Hz 输出数据率、532 Hz 滤波带宽。
pub const GYRO_BANDWIDTH_2000_HZ: u8 = 0x00;
/// 陀螺低功耗模式寄存器。
pub const GYRO_LPM1_REGISTER: u8 = 0x11;
/// 陀螺正常工作模式。
pub const GYRO_LPM1_NORMAL: u8 = 0x00;

/// 加速度计 `ACC_CONF` 的带宽编码：正常带宽。
const ACCEL_BANDWIDTH_NORMAL: u8 = 0x08;
/// 标准重力加速度（m/s²）。
const STANDARD_GRAVITY: f64 = 9.806_65;

/// 加速度计输出数据率（`ACC_CONF` 的 ODR 编码）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AccelOdr {
    /// 12.5 Hz。
    Hz12_5 = 0x05,
    /// 25 Hz。
    Hz25 = 0x06,
    /// 50 Hz。
    Hz50 = 0x07,
    /// 100 Hz。
    Hz100 = 0x08,
    /// 200 Hz。
    Hz200 = 0x09,
    /// 400 Hz。
    Hz400 = 0x0A,
    /// 800 Hz。
    Hz800 = 0x0B,
    /// 1600 Hz。
    Hz1600 = 0x0C,
}

impl AccelOdr {
    /// 最接近且不低于 `rate_hz` 的档位。
    ///
    /// # Errors
    /// `rate_hz` 不是正数，或高于最高档位。
    pub fn at_least(rate_hz: f64) -> Result<Self, String> {
        const LADDER: [(f64, AccelOdr); 8] = [
            (12.5, AccelOdr::Hz12_5),
            (25.0, AccelOdr::Hz25),
            (50.0, AccelOdr::Hz50),
            (100.0, AccelOdr::Hz100),
            (200.0, AccelOdr::Hz200),
            (400.0, AccelOdr::Hz400),
            (800.0, AccelOdr::Hz800),
            (1600.0, AccelOdr::Hz1600),
        ];
        if !rate_hz.is_finite() || rate_hz <= 0.0 {
            return Err(format!("加速度计采样率必须为正数：{rate_hz}"));
        }
        LADDER
            .iter()
            .find(|(hz, _)| *hz >= rate_hz)
            .map(|(_, odr)| *odr)
            .ok_or_else(|| format!("加速度计最高 1600 Hz，请求 {rate_hz}"))
    }
}

/// 加速度计量程（`ACC_RANGE`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccelRange {
    /// ±3 g。
    G3,
    /// ±6 g。
    G6,
    /// ±12 g。
    G12,
    /// ±24 g。
    G24,
}

impl AccelRange {
    /// 由配置的 `range_g` 解析。
    ///
    /// # Errors
    /// 不是支持的量程。
    pub fn from_g(value: u32) -> Result<Self, String> {
        match value {
            3 => Ok(Self::G3),
            6 => Ok(Self::G6),
            12 => Ok(Self::G12),
            24 => Ok(Self::G24),
            other => Err(format!("加速度计量程只支持 3/6/12/24 g，请求 {other}")),
        }
    }

    /// `ACC_RANGE` 编码。
    #[must_use]
    pub const fn bits(self) -> u8 {
        match self {
            Self::G3 => 0x00,
            Self::G6 => 0x01,
            Self::G12 => 0x02,
            Self::G24 => 0x03,
        }
    }

    /// 灵敏度（LSB/g）。
    #[must_use]
    pub const fn lsb_per_g(self) -> f64 {
        match self {
            Self::G3 => 10_920.0,
            Self::G6 => 5_460.0,
            Self::G12 => 2_730.0,
            Self::G24 => 1_365.0,
        }
    }
}

/// 陀螺量程（`RATE_RANGE`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GyroRange {
    /// ±2000 °/s。
    Dps2000,
    /// ±1000 °/s。
    Dps1000,
    /// ±500 °/s。
    Dps500,
    /// ±250 °/s。
    Dps250,
    /// ±125 °/s。
    Dps125,
}

impl GyroRange {
    /// 由配置的 `range_dps` 解析。
    ///
    /// # Errors
    /// 不是支持的量程。
    pub fn from_dps(value: u32) -> Result<Self, String> {
        match value {
            2000 => Ok(Self::Dps2000),
            1000 => Ok(Self::Dps1000),
            500 => Ok(Self::Dps500),
            250 => Ok(Self::Dps250),
            125 => Ok(Self::Dps125),
            other => Err(format!(
                "陀螺量程只支持 125/250/500/1000/2000 °/s，请求 {other}"
            )),
        }
    }

    /// `RATE_RANGE` 编码。
    #[must_use]
    pub const fn bits(self) -> u8 {
        match self {
            Self::Dps2000 => 0x00,
            Self::Dps1000 => 0x01,
            Self::Dps500 => 0x02,
            Self::Dps250 => 0x03,
            Self::Dps125 => 0x04,
        }
    }

    /// 灵敏度（LSB/(°/s)）。
    #[must_use]
    pub const fn lsb_per_dps(self) -> f64 {
        match self {
            Self::Dps2000 => 16.384,
            Self::Dps1000 => 32.768,
            Self::Dps500 => 65.536,
            Self::Dps250 => 131.072,
            Self::Dps125 => 262.144,
        }
    }
}

/// 加速度计 `ACC_CONF` 取值。
#[must_use]
pub const fn accel_conf(odr: AccelOdr) -> u8 {
    ((odr as u8) << 4) | ACCEL_BANDWIDTH_NORMAL
}

/// 加速度计读命令：`[地址|读位, 哑字节]`。
#[must_use]
pub const fn accel_read_command(register: u8) -> [u8; 2] {
    [register | SPI_READ, 0]
}

/// 陀螺读命令：`[地址|读位]`。
#[must_use]
pub const fn gyro_read_command(register: u8) -> [u8; 1] {
    [register | SPI_READ]
}

/// 小端六字节 → 三轴原始计数。
#[must_use]
pub const fn decode_axes(bytes: &[u8; 6]) -> [i16; 3] {
    [
        i16::from_le_bytes([bytes[0], bytes[1]]),
        i16::from_le_bytes([bytes[2], bytes[3]]),
        i16::from_le_bytes([bytes[4], bytes[5]]),
    ]
}

/// 加速度计原始计数 → 比力（m/s²）。
#[must_use]
pub fn accel_metres_per_second_squared(raw: [i16; 3], range: AccelRange) -> [f64; 3] {
    let scale = STANDARD_GRAVITY / range.lsb_per_g();
    [
        f64::from(raw[0]) * scale,
        f64::from(raw[1]) * scale,
        f64::from(raw[2]) * scale,
    ]
}

/// 陀螺原始计数 → 角速度（rad/s）。
#[must_use]
pub fn gyro_radians_per_second(raw: [i16; 3], range: GyroRange) -> [f64; 3] {
    let scale = std::f64::consts::PI / 180.0 / range.lsb_per_dps();
    [
        f64::from(raw[0]) * scale,
        f64::from(raw[1]) * scale,
        f64::from(raw[2]) * scale,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axes_decode_little_endian() {
        let bytes = [0x00, 0x80, 0xFF, 0x7F, 0x01, 0x00];
        assert_eq!(decode_axes(&bytes), [-32_768, 32_767, 1]);
    }

    #[test]
    fn one_g_reads_as_standard_gravity() {
        let lsb = 10_920.0;
        let accel = accel_metres_per_second_squared([lsb as i16, 0, 0], AccelRange::G3);
        assert!((accel[0] - STANDARD_GRAVITY).abs() < 1e-2, "{accel:?}");
        assert!(accel[1].abs() < 1e-9);
    }

    #[test]
    fn wider_accel_range_scales_same_counts_down() {
        let raw = [1_365i16, 0, 0];
        let g3 = accel_metres_per_second_squared(raw, AccelRange::G3);
        let g24 = accel_metres_per_second_squared(raw, AccelRange::G24);
        assert!(
            (g3[0] / g24[0] - 0.125).abs() < 1e-9,
            "{} {}",
            g3[0],
            g24[0]
        );
    }

    #[test]
    fn gyro_counts_scale_to_radians() {
        let gyro = gyro_radians_per_second([16, 0, 0], GyroRange::Dps2000);
        let expected = 16.0 / 16.384 * std::f64::consts::PI / 180.0;
        assert!((gyro[0] - expected).abs() < 1e-12, "{gyro:?} {expected}");
    }

    #[test]
    fn range_encodings_match_datasheet_order() {
        assert_eq!(AccelRange::G3.bits(), 0);
        assert_eq!(AccelRange::G24.bits(), 3);
        assert_eq!(GyroRange::Dps2000.bits(), 0);
        assert_eq!(GyroRange::Dps125.bits(), 4);
        assert_eq!(AccelRange::from_g(6).map(AccelRange::bits), Ok(1));
        assert!(AccelRange::from_g(5).is_err());
        assert!(GyroRange::from_dps(100).is_err());
    }

    #[test]
    fn read_commands_set_the_read_bit() {
        assert_eq!(accel_read_command(ACCEL_DATA_REGISTER), [0x92, 0x00]);
        assert_eq!(gyro_read_command(GYRO_DATA_REGISTER), [0x82]);
    }

    #[test]
    fn odr_ladder_picks_lowest_sufficient_step() {
        assert_eq!(AccelOdr::at_least(400.0), Ok(AccelOdr::Hz400));
        assert_eq!(AccelOdr::at_least(300.0), Ok(AccelOdr::Hz400));
        assert_eq!(AccelOdr::at_least(1.0), Ok(AccelOdr::Hz12_5));
        assert!(AccelOdr::at_least(0.0).is_err());
        assert!(AccelOdr::at_least(3_200.0).is_err());
    }

    #[test]
    fn accel_conf_packs_odr_above_bandwidth() {
        assert_eq!(accel_conf(AccelOdr::Hz400), 0xA8);
        assert_eq!(accel_conf(AccelOdr::Hz100), 0x88);
    }
}
