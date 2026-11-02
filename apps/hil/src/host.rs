//! USB 设备发现：按 VID/PID/序列号匹配，按接口子类定位传感器与执行器端点。

use std::time::Duration;

use nusb::MaybeFuture;
use nusb::descriptors::TransferType;
use nusb::io::{EndpointRead, EndpointWrite};
use nusb::transfer::{Bulk, In, Out};

use crate::config::Config;

/// 主机侧已打开的 HIL 设备。
pub struct HostLink {
    /// 设备句柄；接口必须活到端点线程结束。
    pub device: nusb::Device,
    /// 唯一厂商接口。
    pub interface: nusb::Interface,
    /// 下行批量 OUT 端点地址。
    pub out_endpoint: u8,
    /// 上行批量 IN 端点地址。
    pub in_endpoint: u8,
}

impl HostLink {
    /// 打开设备并认领两个厂商接口。
    ///
    /// # Errors
    /// 设备缺席、描述符不含预期接口或端点、认领失败。
    pub fn open(config: &Config) -> Result<Self, String> {
        let devices = nusb::list_devices()
            .wait()
            .map_err(|error| format!("list USB devices: {error}"))?;
        let mut info = None;
        for device in devices {
            if device.vendor_id() != config.usb.vendor || device.product_id() != config.usb.product
            {
                continue;
            }
            if !config.usb.serial.is_empty()
                && device.serial_number() != Some(config.usb.serial.as_str())
            {
                continue;
            }
            info = Some(device);
            break;
        }
        let info = info.ok_or_else(|| {
            format!(
                "找不到 HIL 设备 {:04x}:{:04x} serial={}",
                config.usb.vendor, config.usb.product, config.usb.serial
            )
        })?;
        let device = info
            .open()
            .wait()
            .map_err(|error| format!("open USB device: {error}"))?;
        let active = device
            .active_configuration()
            .map_err(|error| format!("active configuration: {error}"))?;
        let mut found = None;
        for interface in active.interfaces() {
            let alt = interface.first_alt_setting();
            if alt.class() == firefly_hil::INTERFACE_CLASS && alt.subclass() == config.usb.subclass
            {
                let out = bulk_endpoint(&alt, true)?;
                let inside = bulk_endpoint(&alt, false)?;
                found = Some((interface.interface_number(), out, inside));
                break;
            }
        }
        let (number, out_endpoint, in_endpoint) = found.ok_or("缺少 HIL 厂商接口")?;
        let interface = device
            .claim_interface(number)
            .wait()
            .map_err(|error| format!("claim HIL interface: {error}"))?;
        Ok(Self {
            device,
            interface,
            out_endpoint,
            in_endpoint,
        })
    }

    /// 创建传感器下行写入器与执行器上行读取器。
    ///
    /// # Errors
    /// 端点地址在认领的接口上不存在。
    pub fn split(
        self,
        write_buffer: usize,
        read_buffer: usize,
    ) -> Result<
        (
            EndpointWrite<Bulk>,
            EndpointRead<Bulk>,
            nusb::Device,
            nusb::Interface,
        ),
        String,
    > {
        let writer = self
            .interface
            .endpoint::<Bulk, Out>(self.out_endpoint)
            .map_err(|error| format!("下行端点 0x{:02x}: {error}", self.out_endpoint))?
            .writer(write_buffer);
        let reader = self
            .interface
            .endpoint::<Bulk, In>(self.in_endpoint)
            .map_err(|error| format!("上行端点 0x{:02x}: {error}", self.in_endpoint))?
            .reader(read_buffer)
            .with_read_timeout(Duration::from_millis(100));
        Ok((writer, reader, self.device, self.interface))
    }
}

fn bulk_endpoint(
    alt: &nusb::descriptors::InterfaceDescriptor<'_>,
    out: bool,
) -> Result<u8, String> {
    for endpoint in alt.endpoints() {
        if endpoint.transfer_type() != TransferType::Bulk {
            continue;
        }
        let address = endpoint.address();
        let is_out = address & 0x80 == 0;
        if is_out == out {
            return Ok(address);
        }
    }
    Err(format!(
        "接口 {} 缺少批量 {} 端点",
        alt.interface_number(),
        if out { "OUT" } else { "IN" }
    ))
}

/// 打印设备、接口与端点描述符，用于出厂/接线核对。
///
/// # Errors
/// 设备无法列出或打开。
pub fn probe(config: &Config) -> Result<(), String> {
    let devices = nusb::list_devices()
        .wait()
        .map_err(|error| format!("list USB devices: {error}"))?;
    let mut found = false;
    for device in devices {
        println!(
            "设备 {:04x}:{:04x} serial={:?}",
            device.vendor_id(),
            device.product_id(),
            device.serial_number()
        );
        if device.vendor_id() != config.usb.vendor || device.product_id() != config.usb.product {
            continue;
        }
        found = true;
        println!(
            "设备 {:04x}:{:04x} serial={:?} speed={:?}",
            device.vendor_id(),
            device.product_id(),
            device.serial_number(),
            device.speed()
        );
        let opened = device
            .open()
            .wait()
            .map_err(|error| format!("open: {error}"))?;
        let active = opened
            .active_configuration()
            .map_err(|error| format!("active configuration: {error}"))?;
        println!(
            "配置 {} 接口 {}",
            active.configuration_value(),
            active.num_interfaces()
        );
        for interface in active.interfaces() {
            for alt in interface.alt_settings() {
                println!(
                    "  接口 {} alt {} class=0x{:02x} subclass=0x{:02x} endpoints={}",
                    alt.interface_number(),
                    alt.alternate_setting(),
                    alt.class(),
                    alt.subclass(),
                    alt.num_endpoints()
                );
                for endpoint in alt.endpoints() {
                    println!(
                        "    端点 0x{:02x} {:?} max_packet={}",
                        endpoint.address(),
                        endpoint.transfer_type(),
                        endpoint.max_packet_size()
                    );
                }
            }
        }
    }
    if found {
        Ok(())
    } else {
        Err(format!(
            "没有找到 {:04x}:{:04x}",
            config.usb.vendor, config.usb.product
        ))
    }
}
