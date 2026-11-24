use firefly_pubsub::{
    node::create_node,
    publish::Publisher,
    viz::{SCALAR_BUFFER_SIZE, SCALAR_TOPIC, ScalarMessage, VIZ_MAX_PUBLISHERS},
};
use std::sync::mpsc::Receiver;
use std::time::Duration;

pub struct Snapshot {
    pub stamp: f64,
    pub frame: super::matrix::StatusFrame,
}

impl Snapshot {
    fn publish(
        &self,
        publisher: &Publisher<ScalarMessage>,
        sequence: &mut u64,
    ) -> firefly_error::Result<()> {
        let mut emit = |entity: &str, values: &[f64]| {
            let mut row = ScalarMessage::base(self.stamp, entity);
            row.scalars[..values.len()].copy_from_slice(values);
            row.scalar_count = values.len() as u32;
            row.sequence = *sequence;
            *sequence = sequence.wrapping_add(1);
            publisher.publish(row).map(|_| ())
        };
        let f = &self.frame;
        let unsigned = |i: usize| f64::from(u16::from_le_bytes([f[i], f[i + 1]]));
        let signed = |i: usize| f64::from(i16::from_le_bytes([f[i], f[i + 1]]));
        let word = |i: usize| f64::from(u32::from_le_bytes([f[i], f[i + 1], f[i + 2], f[i + 3]]));
        let valid = |mask, value| if f[53] & mask != 0 { value } else { f64::NAN };
        emit("matrix/link", &[word(6), word(2)])?;
        emit(
            "matrix/attitude",
            &[signed(10) * 0.1, signed(12) * 0.1, signed(14) * 0.1],
        )?;
        emit(
            "matrix/control/sticks",
            &[unsigned(16), unsigned(18), unsigned(20), unsigned(22)],
        )?;
        emit(
            "matrix/control/motors",
            &[unsigned(24), unsigned(26), unsigned(28), unsigned(30)],
        )?;
        emit(
            "matrix/control/state",
            &[
                f64::from(f[34] & 1),
                f64::from((f[34] >> 1) & 1),
                f64::from((f[34] >> 2) & 1),
                unsigned(32),
            ],
        )?;
        emit("matrix/faults/state", &[word(36), f64::from(f[40])])?;
        emit(
            "matrix/faults/last_disarm",
            &[valid(8, f64::from(f[41])), valid(8, word(42))],
        )?;
        let failsafe = if f[50] == 0 {
            [f64::NAN; 2]
        } else {
            [f64::from(f[50]), word(46)]
        };
        emit("matrix/faults/last_failsafe", &failsafe)?;
        emit("matrix/power/state", &[f64::from(f[51]), f64::from(f[52])])?;
        emit(
            "matrix/power/meter",
            &[
                valid(1, unsigned(54) * 0.01),
                valid(2, signed(56) * 0.01),
                valid(2, unsigned(58)),
            ],
        )?;
        emit(
            "matrix/power/extrema",
            &[valid(16, unsigned(62) * 0.01), valid(32, signed(64) * 0.01)],
        )?;
        emit("matrix/temperature", &[valid(4, signed(60))])
    }
}

fn publish(receiver: &Receiver<Snapshot>) -> firefly_error::Result<()> {
    let node = create_node()?;
    let publisher = Publisher::with_topic_buffer_publishers(
        &node,
        SCALAR_TOPIC,
        Some(SCALAR_BUFFER_SIZE),
        Some(VIZ_MAX_PUBLISHERS),
    )?;
    let mut sequence = 0_u64;
    for snapshot in receiver {
        snapshot.publish(&publisher, &mut sequence)?;
    }
    Ok(())
}

pub fn run(receiver: &Receiver<Snapshot>) {
    while super::RUNNING.load(std::sync::atomic::Ordering::Relaxed) {
        match publish(receiver) {
            Ok(()) => break,
            Err(error) => {
                log::warn!("Matrix telemetry publisher: {error}");
                std::thread::sleep(Duration::from_secs(1));
            }
        }
    }
}
