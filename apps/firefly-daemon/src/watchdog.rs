use std::io;
use std::os::{
    linux::net::SocketAddrExt,
    unix::{
        ffi::OsStrExt,
        net::{SocketAddr, UnixDatagram},
    },
};
use std::time::{Duration, Instant};

pub struct Watchdog {
    socket: UnixDatagram,
    interval: Duration,
    last: Instant,
}

impl Watchdog {
    pub fn ready() -> io::Result<Option<Self>> {
        let timeout = sd_notify::watchdog_enabled();
        let Some(path) = std::env::var_os("NOTIFY_SOCKET") else {
            if timeout.is_some() {
                return Err(io::Error::other("systemd watchdog requires NOTIFY_SOCKET"));
            }
            return Ok(None);
        };
        let address = if let Some(name) = path.as_bytes().strip_prefix(b"@") {
            SocketAddr::from_abstract_name(name)?
        } else {
            SocketAddr::from_pathname(path)?
        };
        let socket = UnixDatagram::unbound()?;
        socket.set_nonblocking(true)?;
        socket.connect_addr(&address)?;
        socket.send(b"READY=1")?;
        Ok(timeout.map(|timeout| Self {
            socket,
            interval: timeout / 2,
            last: Instant::now(),
        }))
    }

    pub fn completed_iteration(&mut self) -> io::Result<()> {
        if self.last.elapsed() >= self.interval {
            self.socket.send(b"WATCHDOG=1")?;
            self.last = Instant::now();
        }
        Ok(())
    }
}
