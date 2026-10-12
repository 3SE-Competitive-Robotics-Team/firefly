//! VIO 关键帧驱动的有界视觉地图；原始图像仅用于内存中的描述子提取。
mod backend;
mod frontend;
use firefly_error::{Error, ErrorKind, Result};
use firefly_landmarks::Options;
use firefly_pubsub::{
    landmarks::{self, Correction, Keyframe},
    odom::OdomMessage,
    publish::Publisher,
    subscriber::Subscriber,
    vision::{self, LoopConstraint},
    viz::{ScalarMessage, VizPublisher},
};
use std::time::Duration;

fn main() -> Result<()> {
    firefly_observability::init();
    let mut args = std::env::args().skip(1);
    let mut path = "configs/slam.toml".to_owned();
    while let Some(arg) = args.next() {
        if arg != "--config" {
            return Err(Error::new(
                ErrorKind::InvalidArgument,
                "usage: slam [--config path]",
            ));
        }
        path = args
            .next()
            .ok_or_else(|| Error::new(ErrorKind::InvalidArgument, "missing config"))?;
    }
    let config = std::fs::read_to_string(&path).map_err(|e| {
        Error::new(ErrorKind::InvalidArgument, e.to_string()).with_context("config", &path)
    })?;
    let options: Options = toml::from_str(&config).map_err(|e| {
        Error::new(ErrorKind::InvalidArgument, e.to_string()).with_context("config", &path)
    })?;
    let mut backend = backend::Backend::new(options)?;
    let node = firefly_pubsub::node::create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "slam");
    let frames = Subscriber::<Keyframe>::with_topic(&node, landmarks::TOPIC)?;
    let corrections = Subscriber::<Correction>::with_topic_and_buffer(
        &node,
        landmarks::CORRECTION_TOPIC,
        landmarks::CORRECTION_BUFFER,
    )?;
    let keyframes = Publisher::<OdomMessage>::with_topic_and_buffer(
        &node,
        vision::KEYFRAME_TOPIC,
        Some(landmarks::KEYFRAME_BUFFER),
    )?;
    let loops = Publisher::<LoopConstraint>::with_topic(&node, vision::LOOP_TOPIC)?;
    let viz = VizPublisher::new(&node)?;
    while node.wait(Duration::from_millis(10)).is_ok() {
        firefly_observability::pump_log_ipc(&log_ipc);
        while let Some(correction) = corrections.receive()? {
            backend.correct(&correction).map_err(|e| {
                e.with_context("operation", "apply landmark correction")
                    .with_context("to", correction.to)
            })?;
        }
        if backend.waiting()? {
            continue;
        }
        while let Some(message) = frames.receive()? {
            if !backend.wants(message.odom.timestamp) {
                continue;
            }
            let root =
                fastrace::Span::root("landmark_slam", fastrace::prelude::SpanContext::random());
            let guard = root.set_local_parent();
            firefly_observability::set_sim_time(message.camera_time);
            let frame = frontend::convert(&message).map_err(|e| {
                e.with_context("operation", "build landmark frame")
                    .with_context("camera_time", message.camera_time)
            })?;
            let result = backend
                .process(message.session, frame, message.odom)
                .map_err(|e| {
                    e.with_context("operation", "update landmark map")
                        .with_context("camera_time", message.camera_time)
                })?;
            keyframes.publish(message.odom)?;
            if let Some(edge) = result.edge {
                loops.publish(edge)?;
            }
            let mut msg =
                ScalarMessage::base(message.camera_time, "loop/debug/frontend");
            msg.scalar_count = 4;
            msg.scalars[..4].copy_from_slice(&result.diagnostics);
            viz.publish(msg)?;
            drop(guard);
            drop(root);
            fastrace::flush();
            if backend.waiting()? {
                break;
            }
        }
    }
    Ok(())
}
