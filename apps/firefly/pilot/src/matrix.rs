use firefly_pubsub::imu::ImuMessage;
use std::io;

pub const ENTER: &[u8] = b"$X<\x00\x01\x3f\x00\x00\xab";
pub const ACK: &[u8] = b"$X>\x00\x01\x3f\x00\x00\xab";
pub const HEARTBEAT: &[u8] = b"FH\xa6\x6e";
pub const STATUS_BYTES: usize = 68;
pub type StatusFrame = [u8; STATUS_BYTES];

/// CDC can still deliver queued IMU bytes when a session is reopened.
pub fn ack_end(bytes: &[u8]) -> io::Result<Option<usize>> {
    for (offset, reply) in bytes.windows(ACK.len()).enumerate() {
        if reply == ACK {
            return Ok(Some(offset + reply.len()));
        }
        if reply == b"$X!\x00\x01\x3f\x00\x00\xab" {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "Matrix rejected ENTER",
            ));
        }
    }
    Ok(None)
}

pub fn control(channels: [u16; 4]) -> [u8; 12] {
    let mut frame = [0; 12];
    frame[..2].copy_from_slice(b"FC");
    for (field, channel) in frame[2..10].as_chunks_mut::<2>().0.iter_mut().zip(channels) {
        field.copy_from_slice(&channel.to_le_bytes());
    }
    let checksum = crc(&frame[..10]);
    frame[10..].copy_from_slice(&checksum.to_le_bytes());
    frame
}

pub fn crc(bytes: &[u8]) -> u16 {
    let mut value = 0u16;
    for &byte in bytes {
        value ^= u16::from(byte) << 8;
        for _ in 0..8 {
            value = if value & 0x8000 == 0 {
                value << 1
            } else {
                (value << 1) ^ 0x1021
            };
        }
    }
    value
}

/// The last frame in a USB read bounds the capture time of the entire batch.
/// Slew toward the host clock to follow oscillator drift without USB packet jitter.
#[derive(Default)]
pub struct SampleClock {
    anchor: Option<(u32, f64)>,
}

impl SampleClock {
    /// Project a diagnostic tick without changing the IMU clock anchor.
    pub fn status_time(&self, tick: u32, arrival: f64) -> f64 {
        self.anchor.map_or(arrival, |(previous, host)| {
            host + f64::from(tick.cast_signed().wrapping_sub(previous.cast_signed())) * 1e-6
        })
    }

    pub fn align(&mut self, tick: u32, arrival: f64) -> io::Result<f64> {
        let stamp = if let Some((previous, host)) = self.anchor {
            let dt = tick.wrapping_sub(previous);
            if dt == 0 || dt > i32::MAX as u32 {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "Matrix clock discontinuity",
                ));
            }
            let elapsed = f64::from(dt) * 1e-6;
            let nominal = host + elapsed;
            nominal
                + ((arrival - nominal) * elapsed.min(1.0)).clamp(-elapsed * 0.02, elapsed * 0.02)
        } else {
            arrival
        };
        self.anchor = Some((tick, stamp));
        Ok(stamp)
    }
}

pub struct Sample {
    pub tick: u32,
    axes: [i16; 6],
}

impl Sample {
    pub fn message(&self, timestamp: f64) -> ImuMessage {
        let [ax, ay, az, gx, gy, gz] = self.axes.map(f64::from);
        ImuMessage {
            timestamp,
            linear_acceleration_x: ax * 0.009_806_65,
            linear_acceleration_y: ay * 0.009_806_65,
            linear_acceleration_z: az * 0.009_806_65,
            angular_velocity_x: (gx * 0.1).to_radians(),
            angular_velocity_y: (gy * 0.1).to_radians(),
            angular_velocity_z: (gz * 0.1).to_radians(),
        }
    }
}

pub fn decode(buffer: &mut Vec<u8>, samples: &mut Vec<Sample>, status: &mut Vec<StatusFrame>) {
    let mut pos = 0;
    while buffer.len() - pos >= 2 {
        let size = match &buffer[pos..pos + 2] {
            b"FI" => 20,
            b"FT" => STATUS_BYTES,
            _ => {
                pos += 1;
                continue;
            }
        };
        let Some(frame) = buffer.get(pos..pos + size) else {
            break;
        };
        if crc(&frame[..size - 2]) != u16::from_le_bytes([frame[size - 2], frame[size - 1]]) {
            pos += 1;
            continue;
        }
        if let Ok(snapshot) = frame.try_into() {
            status.push(snapshot);
        } else {
            let axes = std::array::from_fn(|axis| {
                let i = 6 + 2 * axis;
                i16::from_le_bytes([frame[i], frame[i + 1]])
            });
            samples.push(Sample {
                tick: u32::from_le_bytes([frame[2], frame[3], frame[4], frame[5]]),
                axes,
            });
        }
        pos += size;
    }
    buffer.drain(..pos);
}

#[cfg(test)]
mod tests {
    #[test]
    fn acknowledge_after_old_session_bytes() {
        let stream = [b"FIold".as_slice(), ACK, b"FInew"].concat();
        assert_eq!(ack_end(&stream[..9]).unwrap(), None);
        assert_eq!(&stream[ack_end(&stream).unwrap().unwrap()..], b"FInew");
    }
    use super::*;

    #[test]
    fn matches_firmware_channel_order_and_checksum() {
        assert_eq!(
            control([1234, 1500, 1777, 1000]),
            [0x46, 0x43, 0xd2, 4, 0xdc, 5, 0xf1, 6, 0xe8, 3, 0x9c, 0x75]
        );
    }

    #[test]
    fn recovers_after_corruption_and_decodes_split_sample() {
        let mut frame =
            *b"FI\x12\x34\x56\x78\x00\x00\x00\x00\xe8\x03\xf6\xff\x00\x00\x00\x00\x00\x00";
        let sum = crc(&frame[..18]);
        frame[18..].copy_from_slice(&sum.to_le_bytes());
        let mut buffer = b"FIbad".to_vec();
        let mut samples = Vec::new();
        buffer.extend_from_slice(&frame[..16]);
        let mut status = Vec::new();
        decode(&mut buffer, &mut samples, &mut status);
        buffer.extend_from_slice(&frame[16..]);
        decode(&mut buffer, &mut samples, &mut status);
        assert_eq!(buffer, [] as [u8; 0]);
        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0].tick, 0x7856_3412);
        let m = samples[0].message(1.0);
        assert!((m.linear_acceleration_z - 9.80665).abs() < 1e-9);
        assert!((m.angular_velocity_x.to_degrees() + 1.).abs() < 1e-9);
    }

    #[test]
    fn clock_spans_wrap_and_rejects_reboot() {
        let mut clock = SampleClock::default();
        clock.align(u32::MAX - 499, 4.).unwrap();
        assert!((clock.status_time(500, 12.) - 4.001).abs() < 1e-9);
        assert!((clock.align(500, 4.001).unwrap() - 4.001).abs() < 1e-9);
        assert!(clock.align(0, 5.).is_err());
    }

    #[test]
    fn recovers_mixed_frames_at_every_split_after_bad_status_crc() {
        let mut imu = [0; 20];
        imu[..2].copy_from_slice(b"FI");
        let checksum = crc(&imu[..18]);
        imu[18..].copy_from_slice(&checksum.to_le_bytes());
        let mut status = [0; STATUS_BYTES];
        status[..2].copy_from_slice(b"FT");
        status[16..36].copy_from_slice(&imu);
        let checksum = crc(&status[..STATUS_BYTES - 2]);
        status[STATUS_BYTES - 2..].copy_from_slice(&checksum.to_le_bytes());
        let mut bad = [0; STATUS_BYTES];
        bad[..2].copy_from_slice(b"FT");
        let bytes = [&bad[..], &imu, &status, &imu].concat();
        for split in 0..=bytes.len() {
            let mut buffer = bytes[..split].to_vec();
            let (mut samples, mut snapshots) = (Vec::new(), Vec::new());
            decode(&mut buffer, &mut samples, &mut snapshots);
            buffer.extend_from_slice(&bytes[split..]);
            decode(&mut buffer, &mut samples, &mut snapshots);
            assert_eq!(buffer, [] as [u8; 0]);
            assert_eq!(samples.len(), 2);
            assert_eq!(snapshots, [status]);
        }
    }
}
