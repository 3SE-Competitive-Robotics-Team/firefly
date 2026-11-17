use std::process::Command;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use firefly_pubsub::event::TopicListener;
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::Publisher;
use firefly_pubsub::subscriber::Subscriber;

#[test]
#[ignore = "requires FIREFLY_TEST_PYTHON pointing to the synced workspace Python"]
fn viz_python_layout_matches_rust_offsets_and_sizes() {
    use firefly_pubsub::activity::ActivityMessage;
    use firefly_pubsub::landing::LandingTargetMessage;
    use firefly_pubsub::log::LogMessage;
    use firefly_pubsub::trajectory::PlannerStatusMessage;
    use firefly_pubsub::viz::{SCALAR_BUFFER_SIZE, ScalarMessage, VizMessage, VoxelMessage};
    use std::mem::{offset_of, size_of};

    let python = std::env::var("FIREFLY_TEST_PYTHON").expect("workspace Python path");
    let script = r"
import ctypes
from firefly_mujoco.messages import SCALAR_BUFFER_SIZE, ScalarMessage, VizMessage, VoxelMessage
from firefly_mujoco.messages import PlannerStatusMessage, LandingTargetMessage, ActivityMessage
from firefly_mujoco import LogMessage
from firefly_mujoco.activity import LANDING
for cls, fields in [
    (ScalarMessage, ['timestamp', 'scalars', 'sequence']),
    (VizMessage, ['timestamp', 'points', 'sequence']),
    (VoxelMessage, ['voxels', 'sequence', 'chunk_index', 'chunk_count']),
    (PlannerStatusMessage, ['timestamp', 'goal_timestamp', 'finished']),
    (LandingTargetMessage, ['timestamp', 'session', 'position', 'confidence', 'team', 'valid']),
    (ActivityMessage, ['timestamp', 'owner', 'sequence', 'mask', 'lease_ms']),
    (LogMessage, ['sim_time', 'sequence', 'dropped_records']),
]:
    print(ctypes.sizeof(cls), *(getattr(cls, name).offset for name in fields))
print(SCALAR_BUFFER_SIZE)
print(LANDING)
";
    let output = Command::new(python).args(["-c", script]).output().unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let actual: Vec<usize> = String::from_utf8(output.stdout)
        .unwrap()
        .split_whitespace()
        .map(|word| word.parse().unwrap())
        .collect();
    assert_eq!(
        actual,
        vec![
            size_of::<ScalarMessage>(),
            offset_of!(ScalarMessage, timestamp),
            offset_of!(ScalarMessage, scalars),
            offset_of!(ScalarMessage, sequence),
            size_of::<VizMessage>(),
            offset_of!(VizMessage, timestamp),
            offset_of!(VizMessage, points),
            offset_of!(VizMessage, sequence),
            size_of::<VoxelMessage>(),
            offset_of!(VoxelMessage, voxels),
            offset_of!(VoxelMessage, sequence),
            offset_of!(VoxelMessage, chunk_index),
            offset_of!(VoxelMessage, chunk_count),
            size_of::<PlannerStatusMessage>(),
            offset_of!(PlannerStatusMessage, timestamp),
            offset_of!(PlannerStatusMessage, goal_timestamp),
            offset_of!(PlannerStatusMessage, finished),
            size_of::<LandingTargetMessage>(),
            offset_of!(LandingTargetMessage, timestamp),
            offset_of!(LandingTargetMessage, session),
            offset_of!(LandingTargetMessage, position),
            offset_of!(LandingTargetMessage, confidence),
            offset_of!(LandingTargetMessage, team),
            offset_of!(LandingTargetMessage, valid),
            size_of::<ActivityMessage>(),
            offset_of!(ActivityMessage, timestamp),
            offset_of!(ActivityMessage, owner),
            offset_of!(ActivityMessage, sequence),
            offset_of!(ActivityMessage, mask),
            offset_of!(ActivityMessage, lease_ms),
            size_of::<LogMessage>(),
            offset_of!(LogMessage, sim_time),
            offset_of!(LogMessage, sequence),
            offset_of!(LogMessage, dropped_records),
            SCALAR_BUFFER_SIZE,
            firefly_pubsub::activity::LANDING as usize,
        ]
    );
}

const PEER: &str = r"
import ctypes
import sys
import time
import iceoryx2 as iox2
from firefly_mujoco import OdomMessage, TraceContext

iox2.set_log_level(iox2.LogLevel.Error)
node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
def service(topic):
    return node.service_builder(iox2.ServiceName.new(topic)).publish_subscribe(OdomMessage).user_header(TraceContext).open_or_create()
incoming = service(sys.argv[1]).subscriber_builder().create()
outgoing = service(sys.argv[2]).publisher_builder().create()
event = node.service_builder(iox2.ServiceName.new(sys.argv[2])).event().open_or_create().notifier_builder().create()
deadline = time.monotonic() + 5
sent = False
while time.monotonic() < deadline:
    sample = incoming.receive()
    if sample is None:
        time.sleep(0.01)
        continue
    msg = sample.payload().contents
    if msg.timestamp == 14.5 and sent:
        sys.exit(0)
    assert msg.timestamp == 12.5 and msg.position_x == 2.5
    assert msg.velocity_y == -3.25 and msg.is_initialized
    assert sample.user_header().contents.send_ts_secs > 0
    if not sent:
        reply = OdomMessage.from_buffer_copy(bytes(msg))
        reply.timestamp = 13.5
        loan = outgoing.loan_uninit()
        ctypes.memmove(ctypes.addressof(loan.user_header().contents), ctypes.addressof(sample.user_header().contents), ctypes.sizeof(TraceContext))
        loan.write_payload(reply).send()
        event.notify_with_custom_event_id(iox2.EventId.new(0))
        sent = True
raise RuntimeError('IPC roundtrip timed out')
";

#[test]
#[ignore = "requires FIREFLY_TEST_PYTHON pointing to the synced workspace Python"]
fn cif_down_image_roundtrip_preserves_every_pixel_and_dimensions() {
    use firefly_pubsub::camera::{DOWN_HEIGHT, DOWN_SENSOR_ID, DOWN_WIDTH, DownImageMessage};
    let python = std::env::var("FIREFLY_TEST_PYTHON").expect("workspace Python path");
    let unique = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let request = format!("Firefly/Test/Interop/Down/{unique}/Request");
    let response = format!("Firefly/Test/Interop/Down/{unique}/Response");
    let node = create_node().unwrap();
    let publisher = Publisher::<DownImageMessage>::with_topic(&node, &request).unwrap();
    let subscriber = Subscriber::<DownImageMessage>::with_topic(&node, &response).unwrap();
    let peer = PEER.replace("OdomMessage", "DownImageMessage")
        .replace("assert msg.timestamp == 12.5 and msg.position_x == 2.5", "assert msg.timestamp == 12.5 and (msg.width, msg.height, msg.sensor_id) == (352, 288, 3)")
        .replace("assert msg.velocity_y == -3.25 and msg.is_initialized", "assert bytes(msg.data) == bytes(range(256)) * 396");
    let mut child = Command::new(python)
        .args(["-c", &peer, &request, &response])
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut received = false;
    while Instant::now() < deadline {
        publisher
            .publish_with(|message| {
                message.timestamp = if received { 14.5 } else { 12.5 };
                message.width = DOWN_WIDTH as u32;
                message.height = DOWN_HEIGHT as u32;
                message.sensor_id = DOWN_SENSOR_ID;
                for (index, pixel) in message.data.iter_mut().enumerate() {
                    *pixel = (index % 256) as u8;
                }
            })
            .unwrap();
        if let Some(sample) = subscriber.receive().unwrap() {
            assert_eq!(
                (sample.width, sample.height, sample.sensor_id),
                (352, 288, 3)
            );
            assert_eq!(sample.timestamp, 13.5);
            assert!(
                sample
                    .data
                    .iter()
                    .enumerate()
                    .all(|(index, &pixel)| pixel == (index % 256) as u8)
            );
            received = true;
        }
        if let Some(status) = child.try_wait().unwrap() {
            assert!(status.success());
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    if child.try_wait().unwrap().is_none() {
        child.kill().unwrap();
    }
    assert!(
        child.wait().unwrap().success(),
        "Python CIF peer failed or timed out"
    );
    assert!(received, "CIF reply missing");
}

#[test]
#[ignore = "requires FIREFLY_TEST_PYTHON pointing to the synced workspace Python"]
fn rust_python_payload_header_and_event_roundtrip() {
    let python = std::env::var("FIREFLY_TEST_PYTHON").expect("workspace Python path");
    let unique = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let request = format!("Firefly/Test/Interop/Odom/{unique}/Request");
    let response = format!("Firefly/Test/Interop/Odom/{unique}/Response");
    let node = create_node().unwrap();
    let publisher = Publisher::<OdomMessage>::with_topic_notify(&node, &request).unwrap();
    let subscriber = Subscriber::<OdomMessage>::with_topic(&node, &response).unwrap();
    let listener = TopicListener::with_topic(&node, &response).unwrap();
    let mut child = Command::new(python)
        .args(["-c", PEER, &request, &response])
        .spawn()
        .unwrap();
    let message = OdomMessage {
        timestamp: 12.5,
        position_x: 2.5,
        velocity_y: -3.25,
        is_initialized: true,
        ..OdomMessage::default()
    };
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut received = None;
    let mut notifications = 0;
    while Instant::now() < deadline {
        publisher.publish(message).unwrap();
        if let Some(sample) = subscriber.receive().unwrap() {
            received = Some((*sample, *sample.user_header()));
        }
        notifications += listener.drain().unwrap();
        if received.is_some() && notifications > 0 {
            publisher
                .publish(OdomMessage {
                    timestamp: 14.5,
                    ..message
                })
                .unwrap();
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(child.wait().unwrap().success(), "Python peer failed");
    let (reply, header) = received.expect("Python reply");
    assert!((reply.timestamp - 13.5).abs() < f64::EPSILON);
    assert!((reply.position_x - message.position_x).abs() < f64::EPSILON);
    assert!((reply.velocity_y - message.velocity_y).abs() < f64::EPSILON);
    assert!(reply.is_initialized);
    assert!(header.send_ts_secs > 0);
    assert!(notifications > 0);
    assert_eq!(listener.drain().unwrap(), 0, "event queue must be drained");
}

#[test]
#[ignore = "requires FIREFLY_TEST_PYTHON pointing to the synced workspace Python"]
fn reference_alignment_snapshot_roundtrip() {
    let python = std::env::var("FIREFLY_TEST_PYTHON").expect("workspace Python path");
    let unique = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let request = format!("Firefly/Test/Interop/Reference/{unique}/Request");
    let response = format!("Firefly/Test/Interop/Reference/{unique}/Response");
    let node = create_node().unwrap();
    let publisher = firefly_pubsub::publish::Publisher::<firefly_pubsub::reference::ReferenceMessage>::with_topic(&node, &request).unwrap();
    let subscriber = firefly_pubsub::subscriber::Subscriber::<
        firefly_pubsub::reference::ReferenceMessage,
    >::with_topic(&node, &response)
    .unwrap();
    let listener = TopicListener::with_topic(&node, &response).unwrap();
    let peer = PEER.replace("OdomMessage", "ReferenceMessage").replace("msg.is_initialized", "msg.has_alignment == 1 and list(msg.alignment_translation) == [10., 20., 30.] and msg.alignment_rotation[3] == 1. and msg.goal_timestamp == 123.");
    let mut child = Command::new(python)
        .args(["-c", &peer, &request, &response])
        .spawn()
        .unwrap();
    let message = firefly_pubsub::reference::ReferenceMessage {
        timestamp: 12.5,
        position_x: 2.5,
        velocity_y: -3.25,
        has_alignment: 1,
        goal_timestamp: 123.,
        alignment_translation: [10., 20., 30.],
        ..firefly_pubsub::reference::ReferenceMessage::default()
    };
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut received = None;
    let mut notifications = 0;
    while Instant::now() < deadline {
        publisher.publish(message).unwrap();
        if let Some(sample) = subscriber.receive().unwrap() {
            received = Some((*sample, *sample.user_header()));
        }
        notifications += listener.drain().unwrap();
        if received.is_some() && notifications > 0 {
            publisher
                .publish(firefly_pubsub::reference::ReferenceMessage {
                    timestamp: 14.5,
                    ..message
                })
                .unwrap();
            break;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(child.wait().unwrap().success(), "Python peer failed");
    let (reply, header) = received.expect("Python reply");
    assert!((reply.timestamp - 13.5).abs() < f64::EPSILON);
    assert!((reply.position_x - message.position_x).abs() < f64::EPSILON);
    assert!((reply.velocity_y - message.velocity_y).abs() < f64::EPSILON);
    assert_eq!(reply.has_alignment, 1);
    assert_eq!(reply.alignment_translation, [10., 20., 30.]);
    assert_eq!(reply.alignment_rotation, [0., 0., 0., 1.]);
    assert_eq!(reply.goal_timestamp, 123.);
    assert!(header.send_ts_secs > 0);
    assert!(notifications > 0);
    assert_eq!(listener.drain().unwrap(), 0, "event queue must be drained");
}
