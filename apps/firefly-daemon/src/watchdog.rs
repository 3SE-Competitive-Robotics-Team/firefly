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
    socket: Option<UnixDatagram>,
    interval: Option<Duration>,
    last: Instant,
}

impl Watchdog {
    pub fn ready() -> io::Result<Self> {
        let socket = std::env::var_os("NOTIFY_SOCKET")
            .map(|path| -> io::Result<_> {
                let address = if let Some(name) = path.as_bytes().strip_prefix(b"@") {
                    SocketAddr::from_abstract_name(name)?
                } else {
                    SocketAddr::from_pathname(path)?
                };
                let socket = UnixDatagram::unbound()?;
                socket.set_nonblocking(true)?;
                socket.connect_addr(&address)?;
                socket.send(b"READY=1")?;
                Ok(socket)
            })
            .transpose()?;
        let interval = sd_notify::watchdog_enabled().map(|timeout| timeout / 2);
        Ok(Self {
            socket,
            interval,
            last: Instant::now(),
        })
    }

    pub fn completed_iteration(&mut self) -> io::Result<()> {
        if self
            .interval
            .is_some_and(|period| self.last.elapsed() >= period)
        {
            if let Some(socket) = &self.socket {
                socket.send(b"WATCHDOG=1")?;
            }
            self.last = Instant::now();
        }
        Ok(())
    }
}
