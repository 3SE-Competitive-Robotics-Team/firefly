mod model;
mod pixels;

use firefly_pubsub::{
    camera::{
        CAMERA_LEFT_TOPIC, CAMERA_RIGHT_TOPIC, DEPTH_TOPIC, DepthImagePublisher, FrameCadence,
        GrayImageMessage, IMAGE_HEIGHT, IMAGE_WIDTH,
    },
    node::create_node,
    subscriber::{Received, Subscriber, SubscriberOptions},
    viz::{ScalarMessage, VizPublisher},
};
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalHandler};
use std::{
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};

static RUNNING: AtomicBool = AtomicBool::new(true);
const STEREO_DEPTH_HZ: u32 = 15;

#[derive(usage_rs::Cli)]
#[usage(bin = "depth")]
struct Arguments {
    /// Run the FP32 ONNX model on the SIL host.
    #[usage(long)]
    simulation: bool,
    /// Use the HIL rig's rectified stereo pair with A733 `ZipDepth`.
    #[usage(long)]
    zipdepth: bool,
}

enum Backend {
    Mono(model::Model, pixels::Pixels),
    #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
    Stereo(Box<firefly_stereo_depth::Stereo>),
}
impl Backend {
    fn open(args: &Arguments) -> firefly_error::Result<Self> {
        if args.zipdepth {
            if args.simulation {
                return Err(firefly_error::Error::new(
                    firefly_error::ErrorKind::InvalidArgument,
                    "Select either --zipdepth or --simulation",
                ));
            }
            #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
            {
                use firefly_error::IoResultExt;
                let path = firefly_config::paths::models("zipdepth/relative-320-int8.nb");
                let bytes = std::fs::read(&path).at_path("read ZipDepth NBG", &path)?;
                return Ok(Self::Stereo(Box::new(firefly_stereo_depth::Stereo::open(
                    bytes,
                )?)));
            }
            #[cfg(not(all(target_os = "linux", target_arch = "aarch64")))]
            return Err(firefly_error::Error::new(
                firefly_error::ErrorKind::Unsupported,
                "ZipDepth requires the A733 board",
            ));
        }
        Ok(Self::Mono(
            model::Model::open(args.simulation)?,
            pixels::Pixels::new(),
        ))
    }
}

type Image = Received<GrayImageMessage>;
struct Cameras {
    left: Subscriber<GrayImageMessage>,
    right: Option<Subscriber<GrayImageMessage>>,
    pending: [Option<Image>; 2],
    cadence: FrameCadence,
}
impl Cameras {
    fn open(node: &firefly_pubsub::node::IpcNode, stereo: bool) -> firefly_error::Result<Self> {
        let subscribe = |topic| {
            Subscriber::<GrayImageMessage>::with_options(
                node,
                topic,
                SubscriberOptions {
                    // Stereo arrivals can straddle the previous pair's inference.
                    buffer_size: if stereo { 2 } else { 1 },
                    service_max_buffer_size: Some(2),
                    max_publishers: Some(2),
                },
            )
        };
        Ok(Self {
            left: subscribe(CAMERA_LEFT_TOPIC)?,
            right: stereo.then(|| subscribe(CAMERA_RIGHT_TOPIC)).transpose()?,
            pending: [None, None],
            cadence: FrameCadence::default(),
        })
    }
    fn next(&mut self) -> firefly_error::Result<Option<(Image, Option<Image>)>> {
        for (eye, receiver) in std::iter::once(&self.left)
            .chain(self.right.as_ref())
            .enumerate()
        {
            if self.pending[eye].is_none() {
                self.pending[eye] = receiver.receive()?.filter(|s| {
                    let f = s.payload();
                    f.sensor_id == i32::from(eye != 0)
                        && f.width as usize == IMAGE_WIDTH
                        && f.height as usize == IMAGE_HEIGHT
                        && f.timestamp.is_finite()
                        && f.timestamp >= 0.
                });
            }
        }
        if self.right.is_some() {
            let [Some(l), Some(r)] = &self.pending else {
                return Ok(None);
            };
            let stamp = l.payload().timestamp;
            match stamp.total_cmp(&r.payload().timestamp) {
                std::cmp::Ordering::Less => {
                    self.pending[0] = None;
                    return Ok(None);
                }
                std::cmp::Ordering::Greater => {
                    self.pending[1] = None;
                    return Ok(None);
                }
                std::cmp::Ordering::Equal => {}
            }
            if !self.cadence.admit(stamp, STEREO_DEPTH_HZ) {
                self.pending = [None, None];
                return Ok(None);
            }
        }
        Ok(self.pending[0]
            .take()
            .map(|left| (left, self.pending[1].take())))
    }
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
    let mut backend = Backend::open(&args)?;
    let mut cameras = Cameras::open(&node, args.zipdepth)?;
    let output = DepthImagePublisher::open(&node, DEPTH_TOPIC, None)?;
    let diagnostics = VizPublisher::new(&node)?;
    let mut next_report = Instant::now();
    let mut count = 0_u64;
    log::info!(
        "Depth ready: {}",
        if args.zipdepth {
            "ZipDepth INT8 + local stereo anchors, 320x240"
        } else if args.simulation {
            "YOLO26n ONNX CPU"
        } else {
            "YOLO26n VIPLite INT8"
        }
    );
    while RUNNING.load(Ordering::Relaxed) {
        let Some((sample, right)) = cameras.next()? else {
            std::thread::sleep(Duration::from_millis(1));
            continue;
        };
        let frame = sample.payload();
        let span = sample.user_header().continue_span("depth-inference");
        let _parent = span.as_ref().map(fastrace::Span::set_local_parent);
        let started = Instant::now();
        let stamp = frame.timestamp;
        firefly_observability::set_sim_time(stamp);
        let center_depth = match &mut backend {
            Backend::Mono(model, pixels) => {
                drop(right);
                let (input, restore) = pixels.input(&frame.data);
                drop(sample);
                model.run(input, |side, values| {
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
                })?
            }
            #[cfg(all(target_os = "linux", target_arch = "aarch64"))]
            Backend::Stereo(stereo) => {
                let right = right.ok_or_else(|| {
                    firefly_error::Error::new(
                        firefly_error::ErrorKind::Internal,
                        "Missing stereo partner",
                    )
                })?;
                let metres = stereo.run(&frame.data, &right.payload().data)?;
                let (rows, _) = metres.as_chunks::<IMAGE_WIDTH>();
                let center = rows[IMAGE_HEIGHT / 2][IMAGE_WIDTH / 2];
                output.publish_rows(stamp, 0, |y| rows[y])?;
                center
            }
        };
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
