use firefly_archive::transfer::Query;
use firefly_hil::protocol::Message;
use firefly_mission::{Request, read_frame, write_frame};
use std::fs::{File, Permissions};
use std::io;
use std::os::unix::{fs::PermissionsExt, net::UnixListener};
use std::sync::mpsc::{Receiver, SyncSender, TrySendError};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

fn enqueue(
    requests: &SyncSender<Message>,
    mut request: Message,
    deadline: Instant,
) -> Result<(), &'static str> {
    loop {
        match requests.try_send(request) {
            Ok(()) => return Ok(()),
            Err(TrySendError::Full(pending)) => request = pending,
            Err(TrySendError::Disconnected(_)) => return Err("HIL link closed"),
        }
        std::thread::sleep(Duration::from_millis(1));
        if Instant::now() >= deadline {
            return Err("HIL mission enqueue timed out");
        }
    }
}

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
                #[derive(serde::Deserialize, serde::Serialize)]
                #[serde(untagged)]
                enum Rpc {
                    Archive(Query),
                    Mission(Request),
                }
                let request: Rpc = read_frame(&stream).map_err(|e| e.to_string())?;
                let body = serde_json::to_vec(&request).map_err(|e| e.to_string())?;
                if body.len() > firefly_hil::protocol::MISSION_BODY_MAX {
                    return Err("oversize HIL mission request".into());
                }
                id = id.wrapping_add(1);
                let deadline = Instant::now() + timeout;
                enqueue(&requests, Message::MissionRequest(id, body), deadline)?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc::sync_channel;

    #[test]
    fn congested_link_delivers_once_when_space_opens() {
        let (tx, rx) = sync_channel(1);
        tx.send(Message::MissionRequest(1, vec![])).unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        std::thread::scope(|scope| {
            let pending =
                scope.spawn(|| enqueue(&tx, Message::MissionRequest(2, vec![3]), deadline));
            assert!(matches!(rx.recv(), Ok(Message::MissionRequest(1, _))));
            assert_eq!(pending.join().unwrap(), Ok(()));
            assert!(matches!(rx.try_recv(), Ok(Message::MissionRequest(2, body)) if body == [3]));
            assert!(rx.try_recv().is_err());
        });
    }

    #[test]
    fn full_queue_expires_and_closed_queue_fails_immediately() {
        let (tx, rx) = sync_channel(1);
        tx.send(Message::MissionRequest(1, vec![])).unwrap();
        let result = enqueue(&tx, Message::MissionRequest(2, vec![]), Instant::now());
        assert_eq!(result, Err("HIL mission enqueue timed out"));
        assert!(matches!(rx.try_recv(), Ok(Message::MissionRequest(1, _))));
        assert!(rx.try_recv().is_err());
        drop(rx);
        let result = enqueue(&tx, Message::MissionRequest(3, vec![]), Instant::now());
        assert_eq!(result, Err("HIL link closed"));
    }
}
