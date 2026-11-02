//! 帧收发：分片、重组与流式读取。
//!
//! 一个端点是一条字节流；每条消息按 [`MAX_FRAGMENT`] 切成若干帧，帧头记录
//! 分片下标与总数。接收端只在全部分片到齐后解码，任一分片缺失、乱序或跨会话
//! 都丢弃整条消息并计数，不把截断图像或半个 IMU 批次交给算法。

use std::io::{self, Read, Write};

use crate::protocol::{
    Header, Kind, MAX_FRAGMENT, MAX_MESSAGE, Message, decode_header, encode_header,
};

/// 一次完整读取的结果：帧头与解码后的消息。
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    /// 该消息首片的帧头（分片信息已归一化为单条消息）。
    pub header: Header,
    /// 解码后的消息。
    pub message: Message,
}

/// 会话内按消息序号发送的帧写入器。
pub struct FrameWriter<W: Write> {
    inner: W,
    session: u64,
    seq: u64,
}

impl<W: Write> FrameWriter<W> {
    /// 以默认会话号 `0` 创建写入器。
    pub const fn new(inner: W) -> Self {
        Self {
            inner,
            session: 0,
            seq: 0,
        }
    }

    /// 当前会话号。
    #[must_use]
    pub const fn session(&self) -> u64 {
        self.session
    }

    /// 切换会话号并重置序号；跨会话残留帧不会与新会话混用。
    pub const fn set_session(&mut self, session: u64) {
        self.session = session;
        self.seq = 0;
    }

    /// 解包底层写入器。
    pub fn into_inner(self) -> W {
        self.inner
    }

    /// 写入一条消息（必要时分片）。
    ///
    /// # Errors
    /// 消息体超过 [`MAX_MESSAGE`] 或底层写入失败。
    pub fn write_message(&mut self, message: &Message) -> io::Result<()> {
        let body = message.encode();
        self.write_parts(message.kind(), message.capture_time(), &[&body])
    }

    /// 写入一条由多段拼成的消息体，不构造整条消息的中间 `Vec`。
    ///
    /// 图像分块用「块头 + 像素切片」两段直接写，避免 16 KiB 块体再拷一次。
    ///
    /// # Errors
    /// 消息体超过 [`MAX_MESSAGE`]、分片过多或底层写入失败。
    pub fn write_parts(
        &mut self,
        kind: Kind,
        capture_time: f64,
        parts: &[&[u8]],
    ) -> io::Result<()> {
        let total: usize = parts.iter().map(|part| part.len()).sum();
        if total > MAX_MESSAGE {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "hil message exceeds maximum size",
            ));
        }
        let chunks = total.div_ceil(MAX_FRAGMENT).max(1);
        let fragment_count = u16::try_from(chunks).map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "hil message has too many fragments",
            )
        })?;
        let seq = self.seq;
        if total <= MAX_FRAGMENT {
            let header = Header {
                kind,
                session: self.session,
                seq,
                capture_time,
                fragment_index: 0,
                fragment_count,
                payload_len: u32::try_from(total).unwrap_or(0),
            };
            self.inner.write_all(&encode_header(&header))?;
            for part in parts {
                self.inner.write_all(part)?;
            }
        } else {
            let mut index = 0_u16;
            let mut remaining = total;
            let mut part_index = 0_usize;
            let mut offset = 0_usize;
            while remaining > 0 {
                let take = remaining.min(MAX_FRAGMENT);
                let header = Header {
                    kind,
                    session: self.session,
                    seq,
                    capture_time,
                    fragment_index: index,
                    fragment_count,
                    payload_len: u32::try_from(take).unwrap_or(0),
                };
                self.inner.write_all(&encode_header(&header))?;
                let mut left = take;
                while left > 0 {
                    while part_index < parts.len() && offset == parts[part_index].len() {
                        part_index += 1;
                        offset = 0;
                    }
                    let part = parts[part_index];
                    let available = part.len() - offset;
                    let n = left.min(available);
                    self.inner.write_all(&part[offset..offset + n])?;
                    offset += n;
                    left -= n;
                }
                remaining -= take;
                index = index.saturating_add(1);
            }
        }
        self.seq = self.seq.wrapping_add(1);
        self.inner.flush()
    }
}

struct Pending {
    kind: Kind,
    session: u64,
    seq: u64,
    capture_time: f64,
    fragment_count: u16,
    next_index: u16,
    body: Vec<u8>,
}

/// 流式帧读取器：内部缓冲不足时继续读取底层流。
pub struct FrameReader<R: Read> {
    inner: R,
    buffer: Vec<u8>,
    start: usize,
    pending: Option<Pending>,
    dropped: u64,
}

impl<R: Read> FrameReader<R> {
    /// 创建读取器。
    pub const fn new(inner: R) -> Self {
        Self {
            inner,
            buffer: Vec::new(),
            start: 0,
            pending: None,
            dropped: 0,
        }
    }

    /// 因分片缺失、乱序或解码失败丢弃的消息数。
    #[must_use]
    pub const fn dropped(&self) -> u64 {
        self.dropped
    }

    /// 解包底层读取器。
    pub fn into_inner(self) -> R {
        self.inner
    }

    /// 读取下一条完整消息的原始消息体（不在这里解码）。
    ///
    /// 图像分块走这条路径：调用方拿到 `Vec` 后直接把像素拷进目标缓冲，
    /// 避免 `Message::decode` 再复制一次。
    ///
    /// # Errors
    /// 底层读取失败、流提前结束或帧头/消息体非法。
    pub fn read_raw(&mut self) -> io::Result<(Header, Vec<u8>)> {
        loop {
            self.compact();
            let available = self.buffer.len().saturating_sub(self.start);
            if available < crate::protocol::HEADER_LEN {
                self.fill(crate::protocol::HEADER_LEN)?;
                continue;
            }
            let mut raw = [0_u8; crate::protocol::HEADER_LEN];
            raw.copy_from_slice(&self.buffer[self.start..self.start + crate::protocol::HEADER_LEN]);
            let header = decode_header(&raw)?;
            let frame_len = crate::protocol::HEADER_LEN + header.payload_len as usize;
            if self.buffer.len().saturating_sub(self.start) < frame_len {
                self.fill(frame_len)?;
                continue;
            }
            let payload = self.buffer
                [self.start + crate::protocol::HEADER_LEN..self.start + frame_len]
                .to_vec();
            self.start += frame_len;
            if let Some(result) = self.accept_raw(header, payload) {
                return Ok(result);
            }
        }
    }

    /// 读取下一条完整消息并解码。
    ///
    /// # Errors
    /// 底层读取失败、流提前结束或帧头/消息体非法。
    pub fn read_frame(&mut self) -> io::Result<Frame> {
        let (header, body) = self.read_raw()?;
        let message = Message::decode(header.kind, &body)
            .map_err(|error| decode_error(header.kind, &error))?;
        Ok(Frame { header, message })
    }

    fn accept_raw(&mut self, header: Header, payload: Vec<u8>) -> Option<(Header, Vec<u8>)> {
        if header.fragment_count == 1 {
            if self.pending.take().is_some() {
                self.dropped = self.dropped.saturating_add(1);
            }
            return Some((header, payload));
        }
        if header.fragment_index == 0 {
            if self.pending.take().is_some() {
                self.dropped = self.dropped.saturating_add(1);
            }
            self.pending = Some(Pending {
                kind: header.kind,
                session: header.session,
                seq: header.seq,
                capture_time: header.capture_time,
                fragment_count: header.fragment_count,
                next_index: 1,
                body: payload,
            });
            return None;
        }
        let Some(pending) = self.pending.as_mut() else {
            self.dropped = self.dropped.saturating_add(1);
            return None;
        };
        if pending.kind != header.kind
            || pending.session != header.session
            || pending.seq != header.seq
            || pending.fragment_count != header.fragment_count
            || pending.next_index != header.fragment_index
        {
            self.pending = None;
            self.dropped = self.dropped.saturating_add(1);
            return None;
        }
        if pending.body.len().saturating_add(payload.len()) > MAX_MESSAGE {
            self.pending = None;
            self.dropped = self.dropped.saturating_add(1);
            return None;
        }
        pending.body.extend_from_slice(&payload);
        pending.next_index = pending.next_index.saturating_add(1);
        if pending.next_index < pending.fragment_count {
            return None;
        }
        let pending = self.pending.take().expect("pending fragment set above");
        Some((
            Header {
                kind: pending.kind,
                session: pending.session,
                seq: pending.seq,
                capture_time: pending.capture_time,
                fragment_index: 0,
                fragment_count: pending.fragment_count,
                payload_len: u32::try_from(pending.body.len()).unwrap_or(0),
            },
            pending.body,
        ))
    }

    fn compact(&mut self) {
        if self.start == 0 {
            return;
        }
        self.buffer.drain(..self.start);
        self.start = 0;
    }

    fn fill(&mut self, wanted: usize) -> io::Result<()> {
        while self.buffer.len().saturating_sub(self.start) < wanted {
            let available = self.buffer.len().saturating_sub(self.start);
            let need = wanted - available;
            let start = self.buffer.len();
            // FunctionFS 端点要求读缓冲不小于端点最大包长（512B），否则返回
            // EOVERFLOW；按 64 KiB 读，顺便省掉「临时块 + extend」的第二次拷贝。
            self.buffer.resize(start + need.max(64 * 1024), 0);
            let read = self.inner.read(&mut self.buffer[start..])?;
            self.buffer.truncate(start + read);
            if read == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "hil stream closed",
                ));
            }
        }
        Ok(())
    }
}

fn decode_error(kind: Kind, error: &io::Error) -> io::Error {
    io::Error::new(error.kind(), format!("decode {}: {error}", kind.name()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_pubsub::camera::{GrayImageMessage, IMAGE_SIZE};
    use firefly_pubsub::control::ControlMessage;

    fn roundtrip(message: &Message, session: u64) -> Message {
        let mut bytes = Vec::new();
        let mut writer = FrameWriter::new(&mut bytes);
        writer.set_session(session);
        writer.write_message(message).unwrap();
        let mut reader = FrameReader::new(bytes.as_slice());
        let frame = reader.read_frame().unwrap();
        assert_eq!(frame.header.session, session);
        assert_eq!(&frame.message, message);
        frame.message
    }

    #[test]
    fn control_roundtrip() {
        let message = Message::Control(ControlMessage {
            state_time: 12.5,
            thrust: [1.0, 2.0, 3.0, 4.0],
            tick: 7,
        });
        roundtrip(&message, 42);
    }

    #[test]
    fn large_gray_image_is_fragmented_and_reassembled() {
        let mut image = GrayImageMessage {
            timestamp: 3.25,
            sensor_id: 1,
            width: 320,
            height: 240,
            ..GrayImageMessage::default()
        };
        for (index, pixel) in image.data.iter_mut().enumerate() {
            *pixel = (index % 251) as u8;
        }
        let message = Message::Gray(Box::new(image));
        let mut bytes = Vec::new();
        let mut writer = FrameWriter::new(&mut bytes);
        writer.set_session(9);
        writer.write_message(&message).unwrap();
        let frames = bytes
            .len()
            .div_ceil(MAX_FRAGMENT + crate::protocol::HEADER_LEN);
        assert!(frames > 1, "image should span multiple fragments");
        assert!(Message::Gray(Box::new(image)).encode().len() > IMAGE_SIZE);

        let mut reader = FrameReader::new(bytes.as_slice());
        let frame = reader.read_frame().unwrap();
        assert_eq!(frame.message, message);
        assert_eq!(frame.header.fragment_count as usize, frames);
        assert_eq!(reader.dropped(), 0);
    }

    #[test]
    fn incomplete_message_is_dropped_not_reassembled() {
        let message = Message::Imu(vec![firefly_pubsub::imu::ImuMessage::default()]);
        let body = message.encode();
        let header = Header {
            kind: Kind::Imu,
            session: 1,
            seq: 5,
            capture_time: 0.0,
            fragment_index: 0,
            fragment_count: 2,
            payload_len: 1,
        };
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&encode_header(&header));
        bytes.push(body[0]);
        // 下一条消息完整；读取器应丢弃上一条残缺消息后返回它。
        let mut writer = FrameWriter::new(&mut bytes);
        writer.set_session(1);
        writer
            .write_message(&Message::Control(ControlMessage::default()))
            .unwrap();

        let mut reader = FrameReader::new(bytes.as_slice());
        let frame = reader.read_frame().unwrap();
        assert!(matches!(frame.message, Message::Control(_)));
        assert_eq!(reader.dropped(), 1);
    }

    #[test]
    fn airframe_roundtrip_preserves_every_field() {
        let message = Message::Airframe(firefly_pubsub::plant::AirframeMessage {
            timestamp: 12.5,
            mass: 0.219,
            inertia_x: 2.71e-4,
            inertia_y: 3.28e-4,
            inertia_z: 5.57e-4,
            linear_drag: 0.11,
            angular_drag: 0.22,
            rotor_positions: [
                [0.1, 0.2, 0.3],
                [-0.1, 0.2, 0.3],
                [0.1, -0.2, 0.3],
                [-0.1, -0.2, 0.3],
            ],
            rotor_spins: [1.0, -1.0, -1.0, 1.0],
            max_thrust_per_motor: 1.61,
            torque_coefficient: 0.011,
        });
        let mut bytes = Vec::new();
        let mut writer = FrameWriter::new(&mut bytes);
        writer.set_session(3);
        writer.write_message(&message).unwrap();
        let mut reader = FrameReader::new(bytes.as_slice());
        let Message::Airframe(decoded) = reader.read_frame().unwrap().message else {
            panic!("expected airframe");
        };
        assert!((decoded.mass - 0.219).abs() < 1e-12);
        assert!((decoded.inertia_x - 2.71e-4).abs() < 1e-15);
        assert!((decoded.inertia_z - 5.57e-4).abs() < 1e-15);
        assert!((decoded.linear_drag - 0.11).abs() < 1e-12);
        assert!((decoded.angular_drag - 0.22).abs() < 1e-12);
        assert!((decoded.rotor_positions[2][1] + 0.2).abs() < 1e-12);
        assert!((decoded.rotor_spins[1] + 1.0).abs() < 1e-12);
        assert!((decoded.max_thrust_per_motor - 1.61).abs() < 1e-12);
        assert!((decoded.torque_coefficient - 0.011).abs() < 1e-12);
    }

    #[test]
    fn image_chunk_roundtrip_preserves_bytes() {
        let header = crate::protocol::ImageChunkHeader {
            timestamp: 3.5,
            sensor_id: 1,
            image_id: 9,
            chunk_index: 1,
            chunk_count: 3,
        };
        let data = vec![7_u8; 1024];
        let mut bytes = Vec::new();
        let mut writer = FrameWriter::new(&mut bytes);
        writer
            .write_parts(
                Kind::ImageChunk,
                header.timestamp,
                &[&header.encode(), &data],
            )
            .unwrap();
        let mut reader = FrameReader::new(bytes.as_slice());
        let (raw, body) = reader.read_raw().unwrap();
        assert_eq!(raw.kind, Kind::ImageChunk);
        assert_eq!(
            crate::protocol::ImageChunkHeader::decode(&body).unwrap(),
            header
        );
        assert_eq!(&body[crate::protocol::IMAGE_CHUNK_HEADER_LEN..], &data[..]);
    }

    #[test]
    fn write_parts_fragments_are_reassembled() {
        let data = vec![5_u8; 40 * 1024];
        let mut bytes = Vec::new();
        let mut writer = FrameWriter::new(&mut bytes);
        writer
            .write_parts(Kind::Scalar, 1.0, &[&data[..10 * 1024], &data[10 * 1024..]])
            .unwrap();
        let mut reader = FrameReader::new(bytes.as_slice());
        let (header, body) = reader.read_raw().unwrap();
        assert_eq!(header.fragment_count, 3);
        assert_eq!(body, data);
    }
}
