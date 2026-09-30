//! Rust/Python 消息、用户头与事件互通；使用隔离话题，不依赖仿真资产。

use std::process::Command;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use firefly_pubsub::event::TopicListener;
use firefly_pubsub::node::create_node;
use firefly_pubsub::odom::OdomMessage;
use firefly_pubsub::publish::OdomPublisher;
use firefly_pubsub::subscriber::OdomSubscriber;

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
