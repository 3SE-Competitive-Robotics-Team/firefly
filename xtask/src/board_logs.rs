use firefly_archive::{
    Run,
    transfer::{BLOCK, Cursor, Query, Snapshot, source},
};
use firefly_mission::{read_frame, write_frame};
use serde::de::DeserializeOwned;
use std::{
    io::{self, Write},
    os::unix::net::UnixStream,
    path::Path,
    time::Duration,
};

fn call<T: DeserializeOwned>(socket: &Path, query: &Query) -> io::Result<T> {
    let stream = UnixStream::connect(socket)?;
    let timeout = Some(Duration::from_secs(5));
    stream.set_read_timeout(timeout)?;
    stream.set_write_timeout(timeout)?;
    write_frame(&stream, query)?;
    let reply: Result<T, String> = read_frame(&stream)?;
    reply.map_err(io::Error::other)
}

pub fn snapshot(socket: &Path) -> io::Result<Snapshot> {
    call(socket, &Query::Snapshot)
}

/// Freeze each end cursor before copying so growing board files cannot extend this session.
pub fn collect(run: &Run, socket: &Path, start: &Snapshot) -> io::Result<()> {
    let end = snapshot(socket)?;
    if start.boot != end.boot {
        return Err(io::Error::other("board rebooted during HIL session"));
    }
    for (name, count) in &end.gaps {
        if *count > start.gaps.get(name).copied().unwrap_or_default() {
            run.gap("board", &format!("{name}: board reported lost log records"))?;
        }
    }
    for (key, end_cursor) in end.streams {
        let begin = start.streams.get(&key).copied().unwrap_or(Cursor {
            first: 1,
            segment: 1,
            offset: 0,
        });
        if begin.segment < end_cursor.first {
            run.gap(
                "board",
                &format!(
                    "{key} segments {}..{} evicted",
                    begin.segment, end_cursor.first
                ),
            )?;
        }
        if let Err(error) = copy(run, socket, &end.boot, &key, begin, end_cursor) {
            run.gap("board", &format!("{key}: {error}"))?;
            if matches!(
                error.kind(),
                io::ErrorKind::ConnectionRefused
                    | io::ErrorKind::NotFound
                    | io::ErrorKind::TimedOut
                    | io::ErrorKind::BrokenPipe
            ) {
                return Err(error);
            }
        }
    }
    Ok(())
}

fn copy(
    run: &Run,
    socket: &Path,
    boot: &str,
    key: &str,
    begin: Cursor,
    end: Cursor,
) -> io::Result<()> {
    let (kind, name) = source(key)?;
    let name = if kind == "system" && name == "board" {
        name.into()
    } else {
        format!("board-{name}")
    };
    let mut out = run.stream(kind, &name)?;
    let mut pending = Vec::new();
    for segment in begin.segment.max(end.first)..=end.segment {
        let mut offset = if begin.segment == segment {
            begin.offset
        } else {
            0
        };
        let stop = if end.segment == segment {
            end.offset
        } else {
            u64::MAX
        };
        if stop < offset {
            return Err(io::Error::other("board log shrank during session"));
        }
        while offset < stop {
            let limit = (stop - offset).min(BLOCK as u64) as usize;
            let query = Query::Read {
                boot: boot.into(),
                stream: key.into(),
                segment,
                offset,
                limit,
            };
            let bytes: Vec<u8> = call(socket, &query)?;
            if bytes.len() > limit {
                return Err(io::Error::other("oversize board log reply"));
            }
            if bytes.is_empty() {
                if segment == end.segment {
                    return Err(io::Error::other("truncated board log"));
                }
                break;
            }
            offset += bytes.len() as u64;
            for part in bytes.split_inclusive(|b| *b == b'\n') {
                pending.extend_from_slice(part);
                if part.ends_with(b"\n") || pending.len() >= 128 << 10 {
                    out.write_all(&pending)?;
                    pending.clear();
                }
            }
        }
    }
    out.write_all(&pending)?;
    out.flush()
}

#[cfg(test)]
mod tests {
    use super::*;
    use firefly_archive::transfer::reply;
    use std::{fs, os::unix::net::UnixListener, thread};

    #[test]
    fn copies_only_the_session_across_board_rotation() {
        let temp =
            tempfile::tempdir_in(super::super::target::workspace_root().join("target")).unwrap();
        let board = Run::create_at(temp.path(), "board-test").unwrap();
        let manifest = board.path().join("manifest.json");
        let mut metadata: serde_json::Value =
            serde_json::from_slice(&fs::read(&manifest).unwrap()).unwrap();
        metadata["boot_id"] = "test-boot".into();
        fs::write(&manifest, serde_json::to_vec(&metadata).unwrap()).unwrap();
        let (text, system) = (board.path().join("text"), board.path().join("system"));
        fs::write(text.join("camera.000001.log"), b"before\n").unwrap();
        let first = text.join("camera.000001.log");
        let socket = temp.path().join("log.sock");
        let listener = UnixListener::bind(&socket).unwrap();
        let worker = thread::spawn(move || {
            for conn in listener.incoming() {
                let conn = conn.unwrap();
                let Ok(query) = read_frame::<Query>(&conn) else {
                    break;
                };
                let response = reply(&board, query).map_err(|e| e.to_string());
                write_frame(&conn, &response).unwrap();
            }
        });
        let start = snapshot(&socket).unwrap();
        fs::write(first, b"before\nafter\n").unwrap();
        let payload = format!("{}\n", "A".repeat(BLOCK * 3));
        fs::write(text.join("camera.000002.log"), &payload).unwrap();
        fs::write(system.join("board.000001.log"), b"journal\n").unwrap();
        let host = Run::create_at(temp.path(), "hil-test").unwrap();
        collect(&host, &socket, &start).unwrap();
        let copied = fs::read_to_string(host.path().join("text/board-camera.000001.log")).unwrap();
        assert_eq!(copied, format!("after\n{payload}"));
        assert_eq!(
            fs::read(host.path().join("system/board.000001.log")).unwrap(),
            b"journal\n"
        );
        let invalid = Query::Read {
            boot: start.boot,
            stream: "text/../../manifest.json".into(),
            segment: 1,
            offset: 0,
            limit: 1,
        };
        assert!(call::<Vec<u8>>(&socket, &invalid).is_err());
        drop(UnixStream::connect(&socket).unwrap());
        worker.join().unwrap();
    }
}
