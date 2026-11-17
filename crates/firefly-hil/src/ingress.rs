//! 统一 HIL 传感器入口：分块重组、校验、零拷贝发布和 IMU 批次唤醒。
use crate::{
    protocol::{IMAGE_CHUNK_HEADER_LEN, ImageChunkHeader, Kind, Link, Message},
    topics::{SensorPublishers, error},
};
use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH};
use firefly_pubsub::{
    node::IpcNode,
    publish::Publisher,
    viz::{SCALAR_BUFFER_SIZE, ScalarMessage, VIZ_MAX_PUBLISHERS},
};
use std::{cell::Cell, io};

type Result<T> = io::Result<T>;

fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

struct Checksums {
    publisher: Publisher<ScalarMessage>,
    sequence: Cell<u64>,
}
impl Checksums {
    fn publish(&self, mut message: ScalarMessage) -> Result<()> {
        message.sequence = self.sequence.get();
        self.publisher.publish(message).map_err(error)?;
        self.sequence.set(message.sequence.wrapping_add(1));
        Ok(())
    }
}
struct IngressPorts {
    sensors: SensorPublishers,
    scalars: Checksums,
}

/// 同一入口处理所有下行数据；USB、字节流回归测试共用此实现。
pub struct SensorIngress {
    ports: IngressPorts,
    assembly: Option<ChunkAssembly>,
}
impl SensorIngress {
    /// # Errors
    /// IPC 服务或端口创建失败。
    pub fn open(node: &IpcNode, prefix: &str) -> Result<Self> {
        Ok(Self {
            ports: IngressPorts {
                sensors: SensorPublishers::open(node, prefix)?,
                scalars: Checksums {
                    publisher: Publisher::with_topic_buffer_publishers(
                        node,
                        &crate::topics::topic(prefix, firefly_pubsub::viz::SCALAR_TOPIC),
                        Some(SCALAR_BUFFER_SIZE),
                        Some(VIZ_MAX_PUBLISHERS),
                    )
                    .map_err(error)?,
                    sequence: Cell::new(0),
                },
            },
            assembly: None,
        })
    }
    /// 返回本次发布的 IMU 样本数；坏格式或错误方向终止本次会话。
    /// # Errors
    /// 协议、图像重组或 IPC 发布失败。
    pub fn accept_raw(&mut self, kind: Kind, body: Vec<u8>) -> Result<usize> {
        if kind.link() != Link::Sensor {
            return Err(invalid("uplink message received on sensor endpoint"));
        }
        if kind == Kind::ImageChunk {
            let header = ImageChunkHeader::decode(&body)?;
            handle_chunk(&mut self.assembly, header, body, &self.ports)?;
            return Ok(0);
        }
        let message = Message::decode(kind, &body)?;
        match message {
            Message::Imu(samples) => Ok(self.ports.sensors.publish_imu(&samples)?),
            Message::Gray(image) => {
                let publisher = if image.sensor_id == 0 {
                    &self.ports.sensors.left
                } else {
                    &self.ports.sensors.right
                };
                publisher.publish(*image).map_err(error)?;
                publish_image_checksum(&self.ports, &image)?;
                Ok(0)
            }
            Message::Depth(image) => {
                self.ports.sensors.depth.publish(*image).map_err(error)?;
                Ok(0)
            }
            Message::Hello(_) | Message::Ping(_) | Message::Bye(_) => Ok(0),
            request => {
                self.ports.sensors.requests.publish(request)?;
                Ok(0)
            }
        }
    }
}

/// 一张图的块重组状态。
struct ChunkAssembly {
    image_id: u32,
    sensor_id: u8,
    timestamp: f64,
    chunks: Vec<Option<Vec<u8>>>,
    received: usize,
}

fn handle_chunk(
    assembly: &mut Option<ChunkAssembly>,
    header: ImageChunkHeader,
    body: Vec<u8>,
    ports: &IngressPorts,
) -> Result<()> {
    let restart = match assembly.as_ref() {
        Some(current) => {
            current.image_id != header.image_id || current.sensor_id != header.sensor_id
        }
        None => true,
    };
    if restart {
        *assembly = Some(ChunkAssembly {
            image_id: header.image_id,
            sensor_id: header.sensor_id,
            timestamp: header.timestamp,
            chunks: Vec::from_fn(usize::from(header.chunk_count), |_| None),
            received: 0,
        });
    }
    let Some(current) = assembly.as_mut() else {
        return Ok(());
    };
    if usize::from(header.chunk_count) != current.chunks.len()
        || header.timestamp != current.timestamp
    {
        return Err(invalid("inconsistent image chunk metadata"));
    }
    let index = usize::from(header.chunk_index);
    if current.chunks[index].is_none() {
        current.chunks[index] = Some(body);
        current.received += 1;
    }
    if current.received == current.chunks.len() {
        let completed = assembly.take().expect("收齐后取出");
        publish_assembled(ports, completed)?;
    }
    Ok(())
}

fn publish_assembled(ports: &IngressPorts, image: ChunkAssembly) -> Result<()> {
    let ChunkAssembly {
        sensor_id,
        timestamp,
        chunks,
        ..
    } = image;
    let payload_len = |chunk: &Option<Vec<u8>>| {
        chunk
            .as_ref()
            .map_or(0, |body| body.len().saturating_sub(IMAGE_CHUNK_HEADER_LEN))
    };
    let total: usize = chunks.iter().map(payload_len).sum();
    let width = u32::try_from(IMAGE_WIDTH).unwrap_or(0);
    let height = u32::try_from(IMAGE_HEIGHT).unwrap_or(0);
    match sensor_id {
        0 | 1 => {
            if total != IMAGE_SIZE {
                log::warn!("图像块总长 {total} != {IMAGE_SIZE}，丢弃");
                return Ok(());
            }
            let mut sum = 0_u64;
            let publisher = if sensor_id == 0 {
                &ports.sensors.left
            } else {
                &ports.sensors.right
            };
            publisher
                .publish_with(|message| {
                    message.timestamp = timestamp;
                    message.sensor_id = i32::from(sensor_id);
                    message.width = width;
                    message.height = height;
                    let mut offset = 0;
                    for chunk in &chunks {
                        let Some(body) = chunk.as_ref() else { continue };
                        let payload = &body[IMAGE_CHUNK_HEADER_LEN..];
                        let end = offset + payload.len();
                        message.data[offset..end].copy_from_slice(payload);
                        offset = end;
                        for &byte in payload {
                            sum += u64::from(byte);
                        }
                    }
                })
                .map_err(error)?;
            let side = if sensor_id == 0 { "left" } else { "right" };
            let mut scalar = ScalarMessage::base(timestamp, &format!("hil/camera/{side}"));
            scalar.scalars = [timestamp, sum as f64, f64::from(width), f64::from(height)];
            scalar.scalar_count = 4;
            ports.scalars.publish(scalar)?;
        }
        2 => {
            let expected = IMAGE_SIZE * 4;
            if total != expected {
                log::warn!("深度块总长 {total} != {expected}，丢弃");
                return Ok(());
            }
            let mut offset = 0_usize;
            ports
                .sensors
                .depth
                .publish_with(|message| {
                    message.timestamp = timestamp;
                    message.sensor_id = 0;
                    message.width = width;
                    message.height = height;
                    for chunk in &chunks {
                        let Some(body) = chunk.as_ref() else { continue };
                        for value in body[IMAGE_CHUNK_HEADER_LEN..].as_chunks::<4>().0 {
                            message.data[offset] = f32::from_le_bytes(*value);
                            offset += 1;
                        }
                    }
                })
                .map_err(error)?;
        }
        3 => publish_down(ports, timestamp, &chunks, total)?,
        other => log::warn!("未知图像传感器 {other}"),
    }
    Ok(())
}

fn publish_down(
    ports: &IngressPorts,
    timestamp: f64,
    chunks: &[Option<Vec<u8>>],
    total: usize,
) -> Result<()> {
    use firefly_pubsub::camera::{DOWN_HEIGHT, DOWN_SENSOR_ID, DOWN_SIZE, DOWN_WIDTH};
    if total != DOWN_SIZE {
        return Err(invalid("down image chunk length mismatch"));
    }
    let mut sum = 0_u64;
    ports
        .sensors
        .down
        .publish_with(|message| {
            message.timestamp = timestamp;
            message.sensor_id = DOWN_SENSOR_ID;
            message.width = DOWN_WIDTH as u32;
            message.height = DOWN_HEIGHT as u32;
            let mut offset = 0;
            for body in chunks.iter().flatten() {
                let payload = &body[IMAGE_CHUNK_HEADER_LEN..];
                let end = offset + payload.len();
                message.data[offset..end].copy_from_slice(payload);
                offset = end;
                sum += payload.iter().map(|&v| u64::from(v)).sum::<u64>();
            }
        })
        .map_err(error)?;
    let mut scalar = ScalarMessage::base(timestamp, "hil/camera/down");
    scalar.scalar_count = 4;
    scalar.scalars = [timestamp, sum as f64, DOWN_WIDTH as f64, DOWN_HEIGHT as f64];
    ports.scalars.publish(scalar)?;
    Ok(())
}

/// 发布图像校验和：`[时间戳, 像素和, 宽, 高]`，主机按同一算法比对。
fn publish_image_checksum(
    ports: &IngressPorts,
    image: &firefly_pubsub::camera::GrayImageMessage,
) -> Result<()> {
    let sum: u64 = image.data.iter().map(|&pixel| u64::from(pixel)).sum();
    let side = if image.sensor_id == 0 {
        "left"
    } else {
        "right"
    };
    let mut scalar = ScalarMessage::base(image.timestamp, &format!("hil/camera/{side}"));
    scalar.scalars = [
        image.timestamp,
        sum as f64,
        f64::from(image.width),
        f64::from(image.height),
    ];
    scalar.scalar_count = 4;
    ports.scalars.publish(scalar)?;
    Ok(())
}
