use firefly_hil::{
    FrameReader, FrameWriter, Message,
    ingress::SensorIngress,
    protocol::{IMAGE_CHUNK_DATA_MAX, ImageChunk, ImageChunkHeader},
    topics::{RequestsPublishers, RequestsSubscribers, TelemetryPublishers, TelemetrySubscribers},
};
use firefly_pubsub::{
    activity::{ActivityMessage, LANDING},
    attitude::AttitudeMessage,
    camera::{DOWN_SIZE, DownImageMessage},
    command::CommandMessage,
    control::ControlMessage,
    event::TopicListener,
    goal::GoalMessage,
    imu::ImuMessage,
    landing::LandingTargetMessage,
    log::LogMessage,
    navigation::NavigationMessage,
    node::create_node,
    odom::OdomMessage,
    plant::AirframeMessage,
    reference::ReferenceMessage,
    subscriber::{Subscriber, SubscriberOptions},
    trajectory::PlannerStatusMessage,
    viz::{self, ScalarMessage, VizMessage, VoxelMessage},
};
use std::time::{SystemTime, UNIX_EPOCH};

fn prefix(name: &str) -> String {
    format!(
        "Test/Hil/{}/{}/{name}",
        std::process::id(),
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    )
}

fn ingest(ingress: &mut SensorIngress, messages: &[Message]) -> usize {
    let mut writer = FrameWriter::new(Vec::new());
    writer.set_session(73);
    for message in messages {
        writer.write_message(message).unwrap();
    }
    let bytes = writer.into_inner();
    let mut reader = FrameReader::new(bytes.as_slice());
    let mut imu = 0;
    while let Ok((header, body)) = reader.read_raw() {
        imu += ingress.accept_raw(header.kind, body).unwrap();
    }
    imu
}

#[test]
fn downlink_requests_use_the_same_native_ipc_contract() {
    let node = create_node().unwrap();
    let prefix = prefix("requests");
    let subscribers = RequestsSubscribers::open(&node, &prefix).unwrap();
    let mut ingress = SensorIngress::open(&node, &prefix).unwrap();
    let requests = vec![
        Message::Activity(ActivityMessage {
            timestamp: 3.,
            owner: 4,
            sequence: 1,
            mask: LANDING,
            lease_ms: 2000,
        }),
        Message::Command(CommandMessage {
            sequence: 17,
            ..Default::default()
        }),
        Message::Goal(GoalMessage {
            position_x: 4.,
            ..Default::default()
        }),
        Message::Airframe(AirframeMessage::default()),
    ];
    let host_prefix = format!("{prefix}/Host");
    let host = RequestsSubscribers::open(&node, &host_prefix).unwrap();
    let source = RequestsPublishers::open(&node, &host_prefix).unwrap();
    for message in &requests {
        source.publish(message.clone()).unwrap();
    }
    let mut routed = Vec::new();
    host.drain(None, |message| {
        routed.push(message);
        Ok(())
    })
    .unwrap();
    assert_eq!(routed, requests);
    ingest(&mut ingress, &routed);
    let mut actual = Vec::new();
    subscribers
        .drain(None, |message| {
            actual.push(message);
            Ok(())
        })
        .unwrap();
    assert_eq!(actual, requests);
    let invalid = Message::Activity(ActivityMessage {
        timestamp: 3.,
        owner: 4,
        sequence: 2,
        mask: 4,
        lease_ms: 2000,
    });
    assert!(
        ingress
            .accept_raw(invalid.kind(), invalid.encode())
            .is_err()
    );
    assert!(
        ingress
            .accept_raw(firefly_hil::Kind::Control, vec![])
            .is_err()
    );
}

#[test]
fn image_ports_allow_native_consumers_to_start_before_the_bridge() {
    let node = create_node().unwrap();
    let prefix = prefix("startup");
    let _left = Subscriber::<firefly_pubsub::camera::GrayImageMessage>::with_topic_and_buffer(
        &node,
        &format!("{prefix}/CameraLeft"),
        2,
    )
    .unwrap();
    let _down = Subscriber::<DownImageMessage>::with_topic_and_buffer(
        &node,
        &format!("{prefix}/CameraDown"),
        1,
    )
    .unwrap();
    let _source = firefly_hil::topics::SensorPublishers::open(&node, &prefix).unwrap();
    let _relay = firefly_hil::topics::SensorSubscribers::open(&node, &prefix).unwrap();
}

#[test]
fn cif_chunks_preserve_pixels_and_interleaved_imu_wakes_consumers() {
    let node = create_node().unwrap();
    let prefix = prefix("images");
    let down =
        Subscriber::<DownImageMessage>::with_topic(&node, &format!("{prefix}/CameraDown")).unwrap();
    let imu = Subscriber::<ImuMessage>::with_options(
        &node,
        &format!("{prefix}/Imu"),
        SubscriberOptions {
            buffer_size: 128,
            service_max_buffer_size: Some(128),
            max_publishers: Some(2),
        },
    )
    .unwrap();
    let events = TopicListener::with_topic(&node, &format!("{prefix}/Imu")).unwrap();
    let mut ingress = SensorIngress::open(&node, &prefix).unwrap();
    let pixels: Vec<u8> = (0..DOWN_SIZE).map(|i| (i % 251) as u8).collect();
    let chunks: Vec<Message> = pixels
        .chunks(IMAGE_CHUNK_DATA_MAX)
        .enumerate()
        .map(|(i, bytes)| {
            Message::ImageChunk(ImageChunk {
                header: ImageChunkHeader {
                    timestamp: 8.5,
                    sensor_id: 3,
                    image_id: 11,
                    chunk_index: i as u16,
                    chunk_count: pixels.len().div_ceil(IMAGE_CHUNK_DATA_MAX) as u16,
                },
                data: bytes.to_vec(),
            })
        })
        .collect();
    ingest(&mut ingress, &chunks[..2]);
    assert!(down.receive().unwrap().is_none());
    assert_eq!(
        ingest(
            &mut ingress,
            &[Message::Imu(vec![ImuMessage {
                timestamp: 8.51,
                ..Default::default()
            }])]
        ),
        1
    );
    assert_eq!(imu.receive().unwrap().unwrap().timestamp, 8.51);
    assert_eq!(events.drain().unwrap(), 1);
    ingest(&mut ingress, &chunks[2..]);
    let frame = down.receive().unwrap().expect("complete CIF frame");
    assert_eq!(
        (frame.timestamp, frame.width, frame.height, frame.sensor_id),
        (8.5, 352, 288, 3)
    );
    assert_eq!(frame.data.as_slice(), pixels);
    assert!(down.receive().unwrap().is_none());
}

#[test]
fn every_uplink_route_roundtrips_through_ipc_and_wire() {
    let node = create_node().unwrap();
    let board = prefix("board");
    let host = prefix("host");
    let board_ports = TelemetrySubscribers::open(&node, &board).unwrap();
    let source = TelemetryPublishers::open(&node, &board).unwrap();
    let monitor = TelemetrySubscribers::open(&node, &host).unwrap();
    let destination = TelemetryPublishers::open(&node, &host).unwrap();
    let expected = vec![
        Message::FcStatus(firefly_pubsub::fc_status::FcStatusMessage {
            timestamp: 12.5,
            sequence: 0,
            mode: 3,
        }),
        Message::Control(ControlMessage::default()),
        Message::Odometry(OdomMessage::default()),
        Message::CorrectedOdometry(OdomMessage::default()),
        Message::VioAttitude(AttitudeMessage {
            timestamp: 1.,
            posterior_time: 1.,
            session: 9,
            quat_xyzw: [0., 0., 0., 1.],
            angular_velocity: [0.; 3],
            gyro_bias: [0.; 3],
        }),
        Message::Reference(ReferenceMessage::default()),
        Message::PlannerStatus(PlannerStatusMessage::default()),
        Message::Navigation(NavigationMessage::default()),
        Message::LandingTarget(LandingTargetMessage::default()),
        Message::Viz(Box::new(VizMessage::base(
            viz::kind::LINE_STRIP,
            1.,
            "test/lines",
        ))),
        Message::Voxels(Box::new(VoxelMessage::base(
            viz::kind::VOXELS_ADD,
            1.,
            "test/voxels",
        ))),
        Message::Scalar(ScalarMessage::base(1., "landing/runtime")),
        Message::Log(LogMessage::default()),
    ];
    for message in &expected {
        source.publish(message.clone()).unwrap();
    }
    let mut writer = FrameWriter::new(Vec::new());
    board_ports
        .drain(None, |message| writer.write_message(&message))
        .unwrap();
    let bytes = writer.into_inner();
    let mut reader = FrameReader::new(bytes.as_slice());
    while let Ok(frame) = reader.read_frame() {
        destination.publish(frame.message).unwrap();
    }
    let mut actual = Vec::new();
    monitor
        .drain(None, |message| {
            actual.push(message);
            Ok(())
        })
        .unwrap();
    assert_eq!(actual, expected);
}
