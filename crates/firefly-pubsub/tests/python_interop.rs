//! Rust/Python 消息、用户头与事件互通；使用隔离话题，不依赖仿真资产。

use std::process::Command;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use firefly_pubsub::event::TopicListener;
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::OdomPublisher;
use firefly_pubsub::subscriber::OdomSubscriber;

#[test]
#[ignore = "requires FIREFLY_TEST_PYTHON pointing to the synced workspace Python"]
fn viz_python_layout_matches_rust_offsets_and_sizes() {
    use firefly_pubsub::viz::{SCALAR_BUFFER_SIZE, ScalarMessage, VizMessage, VoxelMessage};
    use std::mem::{offset_of, size_of};

    let python = std::env::var("FIREFLY_TEST_PYTHON").expect("workspace Python path");
    let script = r"
import ctypes
from firefly_viz.messages import SCALAR_BUFFER_SIZE, ScalarMessage, VizMessage, VoxelMessage
for cls, fields in [
    (ScalarMessage, ['timestamp', 'scalars', 'sequence']),
    (VizMessage, ['timestamp', 'points', 'sequence']),
    (VoxelMessage, ['voxels', 'sequence', 'chunk_index', 'chunk_count']),
]:
    print(ctypes.sizeof(cls), *(getattr(cls, name).offset for name in fields))
print(SCALAR_BUFFER_SIZE)
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
            SCALAR_BUFFER_SIZE,
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
fn rust_python_payload_header_and_event_roundtrip() {
    let python = std::env::var("FIREFLY_TEST_PYTHON").expect("workspace Python path");
    let unique = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let request = format!("Firefly/Test/Interop/{unique}/Request");
    let response = format!("Firefly/Test/Interop/{unique}/Response");
    let node = create_node().unwrap();
    let publisher = OdomPublisher::with_topic(&node, &request).unwrap();
    let subscriber = OdomSubscriber::with_topic(&node, &response).unwrap();
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
    let request = format!("Firefly/Test/Interop/{unique}/Request");
    let response = format!("Firefly/Test/Interop/{unique}/Response");
    let node = create_node().unwrap();
    let publisher = firefly_pubsub::publish::Publisher::<firefly_pubsub::reference::ReferenceMessage>::with_topic(&node, &request).unwrap();
    let subscriber = firefly_pubsub::subscriber::Subscriber::<
        firefly_pubsub::reference::ReferenceMessage,
    >::with_topic(&node, &response)
    .unwrap();
    let listener = TopicListener::with_topic(&node, &response).unwrap();
    let peer = PEER.replace("OdomMessage", "ReferenceMessage").replace("msg.is_initialized", "msg.has_alignment == 1 and list(msg.alignment_translation) == [10., 20., 30.] and msg.alignment_rotation[3] == 1.");
    let mut child = Command::new(python)
        .args(["-c", &peer, &request, &response])
        .spawn()
        .unwrap();
    let message = firefly_pubsub::reference::ReferenceMessage {
        timestamp: 12.5,
        position_x: 2.5,
        velocity_y: -3.25,
        has_alignment: 1,
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
    assert!(header.send_ts_secs > 0);
    assert!(notifications > 0);
    assert_eq!(listener.drain().unwrap(), 0, "event queue must be drained");
}
