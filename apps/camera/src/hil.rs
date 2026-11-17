//! HIL 传感器接入：从 USB `FunctionFS` 端点读取仿真传感器并发布到本机话题。
//!
//! 时间戳沿用主机下发的仿真时间轴；本进程只做转接，不重排测量时刻。
//! `SensorIngress` 统一解码、校验和组装图像分块，并将传感器消息发布到共享内存。

use std::fs::OpenOptions;
use std::io::ErrorKind;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use firefly_hil::frame::FrameReader;
use firefly_hil::gadget::{endpoint_file_at, touch_host_seen};
use firefly_hil::ingress::SensorIngress;
use firefly_pubsub::node::create_node;
use iceoryx2_bb_posix::signal::{FetchableSignal, SignalGuard, SignalHandler};
use nix::fcntl::{FcntlArg, OFlag, fcntl};

/// HIL 功能实例名。
const HIL_INSTANCE: &str = "hil";
/// 下行端点在功能内的序号（第一个数据端点）。
const DOWNLINK_ENDPOINT: usize = 0;
/// 无数据轮询间隔；比 1ms 小，减少 IMU 到达的额外驻留。
const POLL: Duration = Duration::from_micros(200);
/// 主机心跳刷新间隔。
const HOST_HEARTBEAT: Duration = Duration::from_millis(500);

static RUNNING: AtomicBool = AtomicBool::new(true);

pub fn run() -> Result<(), Box<dyn std::error::Error>> {
    firefly_observability::init();
    let _guard: SignalGuard = SignalHandler::register_multiple_signals(
        &vec![FetchableSignal::Interrupt, FetchableSignal::Terminate],
        &|_signal| RUNNING.store(false, Ordering::Relaxed),
    )?;
    let node = create_node()?;
    let log_ipc = firefly_observability::init_ipc(&node, "camera");
    let mut ingress = SensorIngress::open(&node, "Firefly")?;
    let path = endpoint_file_at(HIL_INSTANCE, DOWNLINK_ENDPOINT)?;
    let file = OpenOptions::new().read(true).open(&path)?;
    let flags = OFlag::from_bits_truncate(fcntl(&file, FcntlArg::F_GETFL)?);
    fcntl(&file, FcntlArg::F_SETFL(flags | OFlag::O_NONBLOCK))?;
    log::info!("HIL 传感器端点 {}", path.display());

    let mut reader = FrameReader::new(file);
    let mut last_heartbeat = Instant::now();
    let mut imu_published = 0_u64;
    let mut last_imu_log = Instant::now();
    while RUNNING.load(Ordering::Relaxed) {
        firefly_observability::pump_log_ipc(&log_ipc);
        match reader.read_raw() {
            Ok((header, body)) => {
                if last_imu_log.elapsed() >= Duration::from_secs(1) {
                    log::debug!("camera --hil 已发布 IMU {imu_published} 条");
                    last_imu_log = Instant::now();
                }
                if last_heartbeat.elapsed() >= HOST_HEARTBEAT {
                    touch_host_seen();
                    last_heartbeat = Instant::now();
                }
                imu_published += ingress.accept_raw(header.kind, body)? as u64;
            }
            Err(error) if firefly_hil::frame::is_retryable(&error) => std::thread::sleep(POLL),
            Err(error) if error.kind() == ErrorKind::UnexpectedEof => {
                log::error!("HIL 传感器端点关闭");
                break;
            }
            Err(error) => return Err(error.into()),
        }
    }
    log::info!("camera --hil 退出");
    Ok(())
}
