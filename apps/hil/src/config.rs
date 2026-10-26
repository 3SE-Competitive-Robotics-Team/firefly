//! 主机侧 HIL 链路配置。

use std::path::PathBuf;

use serde::Deserialize;

/// `configs/hil.toml` 缺省路径。
pub fn default_config() -> PathBuf {
    firefly_config::paths::config("hil.toml")
}

/// 主机侧 HIL 配置。
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// USB 设备匹配与端点发现。
    #[serde(default)]
    pub usb: Usb,
    /// 链路队列与批处理。
    #[serde(default)]
    pub link: Link,
}

/// USB 设备匹配。
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Usb {
    /// 设备 VID。
    #[serde(default = "default_vendor")]
    pub vendor: u16,
    /// 设备 PID。
    #[serde(default = "default_product")]
    pub product: u16,
    /// 序列号前缀匹配；空表示不校验。
    #[serde(default = "default_serial")]
    pub serial: String,
    /// HIL 接口子类。
    #[serde(default = "default_subclass")]
    pub subclass: u8,
}

impl Default for Usb {
    fn default() -> Self {
        Self {
            vendor: default_vendor(),
            product: default_product(),
            serial: default_serial(),
            subclass: default_subclass(),
        }
    }
}

/// 队列与批处理参数。
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Link {
    /// 单次 bulk IN 读缓冲（字节）。
    #[serde(default = "default_read_buffer")]
    pub read_buffer: usize,
    /// 单次 bulk OUT 写缓冲（字节）。
    #[serde(default = "default_write_buffer")]
    pub write_buffer: usize,
    /// 单个 IMU 批次的最大样本数。
    #[serde(default = "default_imu_batch")]
    pub imu_batch: usize,
    /// 图像待发队列深度（帧）。
    #[serde(default = "default_image_queue")]
    pub image_queue: usize,
    /// 下行图像节流周期（秒）；板端 VIO 跟不上时用它压低送帧率。
    #[serde(default = "default_image_period")]
    pub image_period_s: f64,
    /// 是否下深度图；悬停与纯 VIO 阶段关闭，减少板端图像队列突发。
    #[serde(default)]
    pub depth: bool,
}

impl Default for Link {
    fn default() -> Self {
        Self {
            read_buffer: default_read_buffer(),
            write_buffer: default_write_buffer(),
            imu_batch: default_imu_batch(),
            image_queue: default_image_queue(),
            image_period_s: default_image_period(),
            depth: false,
        }
    }
}

const fn default_vendor() -> u16 {
    0x18d1
}

const fn default_product() -> u16 {
    0x0002
}

fn default_serial() -> String {
    "firefly-0001".to_owned()
}

const fn default_subclass() -> u8 {
    1
}

const fn default_read_buffer() -> usize {
    64 * 1024
}

const fn default_write_buffer() -> usize {
    16 * 1024
}

const fn default_imu_batch() -> usize {
    8
}

const fn default_image_queue() -> usize {
    4
}

const fn default_image_period() -> f64 {
    0.1
}

impl Config {
    /// 参数边界检查。
    ///
    /// # Errors
    /// 队列为零或缓冲小于协议上限时返回错误。
    pub fn validate(&self) -> firefly_error::Result<()> {
        if self.link.read_buffer < firefly_hil::protocol::HEADER_LEN {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "read_buffer smaller than frame header",
            ));
        }
        if self.link.write_buffer < firefly_hil::protocol::HEADER_LEN {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "write_buffer smaller than frame header",
            ));
        }
        if self.link.imu_batch == 0 || self.link.imu_batch > firefly_hil::protocol::MAX_IMU_BATCH {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "imu_batch out of range",
            ));
        }
        if !self.link.image_period_s.is_finite() || self.link.image_period_s <= 0.0 {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "image_period_s must be positive",
            ));
        }
        if self.link.image_queue == 0 {
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::InvalidArgument,
                "image_queue must be positive",
            ));
        }
        Ok(())
    }
}
