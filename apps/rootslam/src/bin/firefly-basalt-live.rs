//! Headless T265 recording and Basalt VO/VIO.
use firefly_error::{Error, ErrorKind, Result};
use firefly_rootslam::live::{Options, run};
use std::{
    io::Write,
    sync::{Arc, atomic::AtomicBool},
};
fn invalid(message: &str) -> Error {
    Error::new(ErrorKind::InvalidArgument, message).with_operation("basalt.live.cli")
}
struct SignalRegistrations(Vec<signal_hook::SigId>);
impl Drop for SignalRegistrations {
    fn drop(&mut self) {
        for registration in self.0.drain(..) {
            signal_hook::low_level::unregister(registration);
        }
    }
}
fn stop_signals(cancel: &Arc<AtomicBool>) -> Result<SignalRegistrations> {
    let mut registrations = SignalRegistrations(Vec::new());
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        registrations.0.push(
            signal_hook::flag::register(signal, Arc::clone(cancel))
                .map_err(|e| Error::from(e).with_operation("basalt.live.signal"))?,
        );
    }
    Ok(registrations)
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let first = args.next();
    if first.as_deref() == Some("--help") || first.as_deref() == Some("-h") {
        writeln!(std::io::stdout(),"firefly-basalt-live --config <path.toml>\nTasks: vio (VO/VIO and TUM trajectory), record (stereo/IMU/pose dataset).\nStop with SIGINT or SIGTERM.").map_err(|e|Error::from(e).with_operation("basalt.live.help"))?;
        return Ok(());
    }
    if first.as_deref() != Some("--config") {
        return Err(invalid("Usage: firefly-basalt-live --config <path.toml>"));
    }
    let path = args
        .next()
        .ok_or_else(|| invalid("Missing configuration path"))?;
    if args.next().is_some() {
        return Err(invalid("Unexpected argument"));
    }
    let options = firefly_config::startup(path, firefly_observability::init, Options::validate)?;
    let cancel = Arc::new(AtomicBool::new(false));
    let _signals = stop_signals(&cancel)?;
    let node = firefly_pubsub::node::create_node()?;
    let logs = firefly_observability::init_ipc(&node, "basalt-live");
    std::thread::scope(|scope| {
        let worker = scope.spawn(|| run(&options, &cancel));
        while !worker.is_finished() {
            firefly_observability::pump_log_ipc(&logs);
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        let result = worker
            .join()
            .map_err(|_| invalid("Live application worker panicked"))
            .and_then(|result| result);
        match &result {
            Ok(report) => log::info!("Live task complete: {report:?}"),
            Err(error) => log::error!("{error}"),
        }
        firefly_observability::pump_log_ipc(&logs);
        result.map(|_| ())
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn sigint_and_sigterm_request_the_same_orderly_shutdown() {
        let cancel = Arc::new(AtomicBool::new(false));
        let _signals = stop_signals(&cancel).unwrap();
        for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
            cancel.store(false, std::sync::atomic::Ordering::Release);
            signal_hook::low_level::raise(signal).unwrap();
            assert!(cancel.load(std::sync::atomic::Ordering::Acquire));
        }
    }
}
