use firefly_hil::protocol::Message;
use firefly_mission::{Request, read_frame, write_frame};
use std::fs::{File, Permissions};
use std::io;
use std::os::unix::{fs::PermissionsExt, net::UnixListener};
use std::sync::mpsc::{Receiver, SyncSender};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub fn serve(requests: SyncSender<Message>, replies: Receiver<(u64, Vec<u8>)>) -> io::Result<()> {
    let dir = firefly_mission::runtime_dir();
    std::fs::create_dir_all(&dir)?;
    let lock = File::options()
        .write(true)
        .create(true)
        .truncate(false)
        .open(dir.join("hil-mission.lock"))?;
    lock.try_lock()?;
    let path = dir.join("hil-mission.sock");
    if path.exists() {
        std::fs::remove_file(&path)?;
    }
    let listener = UnixListener::bind(&path)?;
    std::fs::set_permissions(path, Permissions::from_mode(0o600))?;
    std::thread::spawn(move || {
        let _lock = lock;
        let mut id = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let timeout = Duration::from_secs(4);
        for stream in listener.incoming().flatten() {
            if stream.set_read_timeout(Some(timeout)).is_err()
                || stream.set_write_timeout(Some(timeout)).is_err()
            {
                continue;
            }
            let reply = (|| -> Result<serde_json::Value, String> {
                let request: Request = read_frame(&stream).map_err(|e| e.to_string())?;
                let body = serde_json::to_vec(&request).map_err(|e| e.to_string())?;
                if body.len() > firefly_hil::protocol::MISSION_BODY_MAX {
                    return Err("oversize HIL mission request".into());
                }
                id = id.wrapping_add(1);
                requests
                    .try_send(Message::MissionRequest(id, body))
                    .map_err(|e| e.to_string())?;
                let deadline = Instant::now() + timeout;
                loop {
                    let (got, body) = replies
                        .recv_timeout(deadline.saturating_duration_since(Instant::now()))
                        .map_err(|e| e.to_string())?;
                    if got == id {
                        return serde_json::from_slice(&body).map_err(|e| e.to_string())?;
                    }
                }
            })();
            let _ = write_frame(&stream, &reply);
        }
    });
    Ok(())
}
