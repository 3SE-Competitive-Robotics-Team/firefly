//! 板端 `FunctionFS` 功能接口：描述符与端点常量。
//!
//! 单一 HIL 功能接口与主机的一个 USB 厂商接口一一对应，接口号由 configfs 按
//! 函数链接顺序分配；主机侧按 `bInterfaceClass=0xFF` 与子类匹配，不依赖接口号。
//! 端点号 1、2 已被 `adb` 占用，这里只用 3、4。
//!
//! `FunctionFS` 要求描述符在 UDC 绑定前由用户态写入 `ep0`，因此本模块只负责
//! 生成描述符字节；创建 configfs 目录、挂载与绑定由板端 `hil-gadget` 进程完成。

use std::io::{self, Write};

pub use crate::{HIL_SUBCLASS, INTERFACE_CLASS};

/// HIL 功能名（configfs `functions/ffs.hil`，`FunctionFS` 实例名 `hil`）。
pub const HIL_FUNCTION: &str = "hil";

/// 下行端点地址（批量 OUT，主机 → 板端）。
pub const OUT_ENDPOINT: u8 = 0x03;
/// 上行端点地址（批量 IN，板端 → 主机）。
pub const IN_ENDPOINT: u8 = 0x84;

/// 全速端点单包长度。
const FS_PACKET: u16 = 64;
/// 高速端点单包长度。
const HS_PACKET: u16 = 512;
/// `FunctionFS` V2 描述符块头长度：`magic`/`length`/`flags`/`fs_count`/`hs_count`。
pub const DESCRIPTORS_HEAD: usize = 20;
/// `FUNCTIONFS_DESCRIPTORS_MAGIC_V2`。
const DESCRIPTORS_MAGIC_V2: u32 = 3;
/// `FUNCTIONFS_HAS_FS_DESC | FUNCTIONFS_HAS_HS_DESC`。
const HAS_FS_HS: u32 = 1 | 2;
/// 功能接口描述符总长度上限（接口 + 2 个端点），用于缓冲区定长。
const MAX_DESCRIPTORS: usize = 64;

/// 单个批量端点。
#[derive(Clone, Copy, Debug)]
pub struct Endpoint {
    /// 端点地址（含方向位）。
    pub address: u8,
}

/// 一个 `FunctionFS` 功能接口的端点与匹配信息。
#[derive(Clone, Copy, Debug)]
pub struct Function {
    /// `FunctionFS` 实例名，同时是 configfs 函数名的后缀。
    pub name: &'static str,
    /// 接口号；内核按函数链接顺序重写，用户态统一写 0。
    pub interface: u8,
    /// 接口子类，主机按此匹配。
    pub subclass: u8,
    /// 端点列表（按描述符顺序）。
    pub endpoints: &'static [Endpoint],
}

/// 单一 HIL 功能：下行（图像/IMU/指令）+ 上行（控制/遥测）。
pub const HIL: Function = Function {
    name: HIL_FUNCTION,
    interface: 0,
    subclass: HIL_SUBCLASS,
    endpoints: &[
        Endpoint {
            address: OUT_ENDPOINT,
        },
        Endpoint {
            address: IN_ENDPOINT,
        },
    ],
};

/// 生成 `FunctionFS` 描述符块（全速 + 高速）。
#[must_use]
pub fn descriptors(function: &Function) -> Vec<u8> {
    let fs = iface_block(function, FS_PACKET);
    let hs = iface_block(function, HS_PACKET);
    let mut out = Vec::with_capacity(DESCRIPTORS_HEAD + fs.len() + hs.len());
    // usb_functionfs_descs_head_v2：magic、总长、flags，随后按 flags 写 fs/hs 计数。
    out.extend_from_slice(&DESCRIPTORS_MAGIC_V2.to_le_bytes());
    out.extend_from_slice(
        &u32::try_from(DESCRIPTORS_HEAD + fs.len() + hs.len())
            .unwrap_or(0)
            .to_le_bytes(),
    );
    out.extend_from_slice(&HAS_FS_HS.to_le_bytes());
    // fs_count/hs_count 是**描述符个数**（接口 + 端点），不是字节数。
    let count = u32::try_from(1 + function.endpoints.len()).unwrap_or(0);
    out.extend_from_slice(&count.to_le_bytes());
    out.extend_from_slice(&count.to_le_bytes());
    out.extend_from_slice(&fs);
    out.extend_from_slice(&hs);
    out
}

fn iface_block(function: &Function, packet: u16) -> Vec<u8> {
    debug_assert!(MAX_DESCRIPTORS >= 9 + 7 * function.endpoints.len());
    let mut out = Vec::with_capacity(9 + 7 * function.endpoints.len());
    // 接口描述符：类 0xFF、子类区分角色、协议 0、无字符串。
    out.extend_from_slice(&[
        9,
        4,
        function.interface,
        0,
        u8::try_from(function.endpoints.len()).unwrap_or(0),
        INTERFACE_CLASS,
        function.subclass,
        0,
        0,
    ]);
    for endpoint in function.endpoints {
        out.extend_from_slice(&[
            7,
            5,
            endpoint.address,
            0x02, // 批量传输
            u8::try_from(packet & 0xFF).unwrap_or(0),
            u8::try_from(packet >> 8).unwrap_or(0),
            0,
        ]);
    }
    out
}

/// 空 strings 块：FunctionFS 在描述符之后必须收到它，才会创建 ep 文件。
#[must_use]
pub fn strings_block() -> [u8; 16] {
    let mut out = [0_u8; 16];
    out[0..4].copy_from_slice(&2_u32.to_le_bytes());
    out[4..8].copy_from_slice(&16_u32.to_le_bytes());
    // str_count = 0、lang_count = 0：接口不引用字符串。
    out
}

/// 把描述符与空 strings 块写入已打开的 `ep0`；两块都写完后内核才激活功能。
///
/// # Errors
/// `ep0` 写入失败（通常是功能未启用或描述符被内核拒绝）。
pub fn write_descriptors(ep0: &mut impl Write, function: &Function) -> io::Result<()> {
    ep0.write_all(&descriptors(function))?;
    ep0.write_all(&strings_block())?;
    ep0.flush()
}

/// 主机心跳文件：板端收到上行数据后刷新，看门狗据此判断主机是否还在。
pub const HOST_SEEN: &str = "/opt/firefly/logs/hil/host_seen";

/// 刷新主机心跳文件（只更新 mtime，内容为空）。
pub fn touch_host_seen() {
    if let Ok(file) = std::fs::OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(HOST_SEEN)
    {
        let _ = file.set_len(0);
    }
}

/// 返回已激活功能的第 `index` 个数据端点文件（按描述符顺序，跳过 `ep0`）。
///
/// 端点文件名由内核按描述符顺序生成，不能按描述符地址硬编码。
///
/// # Errors
/// 挂载目录不存在、端点数量不足或读取失败。
pub fn endpoint_file_at(instance: &str, index: usize) -> io::Result<std::path::PathBuf> {
    let mount = std::path::Path::new("/dev/usb-ffs").join(instance);
    let mut candidates = Vec::new();
    for entry in std::fs::read_dir(&mount)? {
        let entry = entry?;
        let name = entry.file_name().to_string_lossy().into_owned();
        if name.starts_with("ep") && name != "ep0" {
            candidates.push(entry.path());
        }
    }
    candidates.sort();
    candidates.get(index).cloned().ok_or_else(|| {
        std::io::Error::other(format!(
            "{} 下缺少第 {index} 个数据端点，实际 {candidates:?}",
            mount.display()
        ))
    })
}

/// 返回唯一数据端点文件（只用于单端点诊断模式）。
///
/// # Errors
/// 挂载目录不存在、端点不唯一或读取失败。
pub fn endpoint_file(instance: &str) -> io::Result<std::path::PathBuf> {
    let mount = std::path::Path::new("/dev/usb-ffs").join(instance);
    let mut candidates = Vec::new();
    for entry in std::fs::read_dir(&mount)? {
        let entry = entry?;
        let name = entry.file_name().to_string_lossy().into_owned();
        if name.starts_with("ep") && name != "ep0" {
            candidates.push(entry.path());
        }
    }
    candidates.sort();
    match candidates.as_slice() {
        [path] => Ok(path.clone()),
        other => Err(std::io::Error::other(format!(
            "{} 下期望唯一数据端点，实际 {other:?}",
            mount.display()
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptor_head_matches_block_lengths() {
        let raw = descriptors(&HIL);
        let magic = u32::from_le_bytes(raw[0..4].try_into().unwrap());
        let total = u32::from_le_bytes(raw[4..8].try_into().unwrap()) as usize;
        let flags = u32::from_le_bytes(raw[8..12].try_into().unwrap());
        let fs = u32::from_le_bytes(raw[12..16].try_into().unwrap()) as usize;
        let hs = u32::from_le_bytes(raw[16..20].try_into().unwrap()) as usize;
        assert_eq!(magic, DESCRIPTORS_MAGIC_V2);
        assert_eq!(flags, HAS_FS_HS);
        assert_eq!(total, raw.len());
        // HIL：接口 9 + 两个端点 14 = 23 字节，共 3 个描述符。
        assert_eq!(total, DESCRIPTORS_HEAD + 23 + 23);
        assert_eq!(fs, 3);
        assert_eq!(hs, 3);
    }

    #[test]
    fn interface_descriptor_identifies_role() {
        let raw = descriptors(&HIL);
        let base = DESCRIPTORS_HEAD;
        assert_eq!(raw[base], 9);
        assert_eq!(raw[base + 1], 4);
        assert_eq!(raw[base + 2], HIL.interface);
        assert_eq!(raw[base + 4], 2);
        assert_eq!(raw[base + 5], INTERFACE_CLASS);
        assert_eq!(raw[base + 6], HIL_SUBCLASS);
        assert_eq!(&raw[base + 9..base + 11], &[7, 5]);
    }

    #[test]
    fn high_speed_packet_is_512() {
        let raw = descriptors(&HIL);
        let hs = &raw[DESCRIPTORS_HEAD + 23..];
        assert_eq!(hs[0], 9);
        assert_eq!(hs[9], 7);
        assert_eq!(hs[10], 5);
        assert_eq!(hs[11], OUT_ENDPOINT);
        assert_eq!(hs[12], 0x02);
        assert_eq!(u16::from_le_bytes([hs[13], hs[14]]), 512);
    }
}
