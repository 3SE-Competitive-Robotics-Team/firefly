mod model;
mod pixels;

use firefly_pubsub::{
    camera::{
        CAMERA_LEFT_TOPIC, DEPTH_TOPIC, DepthImagePublisher, GrayImageMessage, IMAGE_HEIGHT,
        IMAGE_WIDTH,
    },
    node::create_node,
    subscriber::{Subscriber, SubscriberOptions},
    viz::{ScalarMessage, VizPublisher},
};
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalHandler};
use std::{
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};

static RUNNING: AtomicBool = AtomicBool::new(true);

#[derive(usage_rs::Cli)]
#[usage(bin = "depth")]
struct Arguments {
    /// Run the FP32 ONNX model on the SIL host.
    #[usage(long)]
    simulation: bool,
}

fn main() -> std::process::ExitCode {
    firefly_error::report("depth", run_app())
}

fn run_app() -> firefly_error::Result<()> {
    let args = Arguments::parse();
    firefly_observability::init();
    let _guard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_| RUNNING.store(false, Ordering::Relaxed),
    )
    .map_err(|e| {
        firefly_error::Error::caused_by(
            firefly_error::ErrorKind::Internal,
            "register shutdown signals",
            e,
        )
    })?;
    let node = create_node()?;
    let logs = firefly_observability::init_ipc(&node, "depth");
    let mut model = model::Model::open(args.simulation)?;
    let left = Subscriber::<GrayImageMessage>::with_options(
        &node,
        CAMERA_LEFT_TOPIC,
        SubscriberOptions {
            buffer_size: 1,
            service_max_buffer_size: Some(2),
            max_publishers: Some(2),
        },
    )?;
    let output = DepthImagePublisher::open(&node, DEPTH_TOPIC, None)?;
    let diagnostics = VizPublisher::new(&node)?;
    let mut next_report = Instant::now();
    let mut count = 0_u64;
    let mut pixels = pixels::Pixels::new();
    log::info!(
        "YOLO26n-Depth ready, 320x320, backend={}",
        if args.simulation {
            "ONNX CPU"
        } else {
            "VIPLite INT8"
        }
    );
    while RUNNING.load(Ordering::Relaxed) {
        let Some(sample) = left.receive()? else {
            std::thread::sleep(Duration::from_millis(1));
            continue;
        };
        let frame = sample.payload();
        if frame.sensor_id != 0
            || (frame.width as usize, frame.height as usize) != (IMAGE_WIDTH, IMAGE_HEIGHT)
            || !frame.timestamp.is_finite()
            || frame.timestamp < 0.
        {
            continue;
        }
        let span = sample.user_header().continue_span("depth-inference");
        let _parent = span.as_ref().map(fastrace::Span::set_local_parent);
        let started = Instant::now();
        let stamp = frame.timestamp;
        firefly_observability::set_sim_time(stamp);
        let (input, restore) = pixels.input(&frame.data);
        drop(sample);
        let center_depth = model.run(input, |side, values| {
            let row = restore.rows(side, values);
            let mut center = 0.;
            output.publish_rows(stamp, 0, |y| {
                let pixels = row(y);
                if y == IMAGE_HEIGHT / 2 {
                    center = pixels[IMAGE_WIDTH / 2];
                }
                pixels
            })?;
            Ok(center)
        })?;
        count += 1;
        if Instant::now() >= next_report {
            let mut report = ScalarMessage::base(stamp, "depth/inference");
            report.scalars = [
                count as f64,
                started.elapsed().as_secs_f64() * 1000.,
                f64::from(center_depth),
                f64::from(!args.simulation),
            ];
            report.scalar_count = 4;
            let _ = diagnostics.publish(report);
            firefly_observability::pump_log_ipc(&logs);
            next_report = Instant::now() + Duration::from_secs(1);
        }
    }
    Ok(())
}
