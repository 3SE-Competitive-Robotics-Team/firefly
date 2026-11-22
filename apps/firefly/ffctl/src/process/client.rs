use firefly_error::{Error, ErrorKind, Result};
use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::time::Duration;

use crate::process::SOCKET;
use crate::process::protocol::{Request, Response};

const RESPONSE_TIMEOUT: Duration = Duration::from_secs(60);

pub(crate) fn send(request: &Request) -> Result<Response> {
    let mut stream = UnixStream::connect(SOCKET).map_err(|error| {
        Error::from(error)
            .with_operation("connect to supervisor")
            .with_context("socket", SOCKET)
            .with_context("hint", "先运行 systemctl start firefly")
    })?;
    stream
        .set_read_timeout(Some(RESPONSE_TIMEOUT))
        .map_err(|error| Error::from(error).with_operation("设置 supervisor 超时"))?;

    let mut text = serde_json::to_string(request)
        .map_err(|error| Error::caused_by(ErrorKind::Internal, "编码请求", error))?;
    text.push('\n');
    stream
        .write_all(text.as_bytes())
        .map_err(|error| Error::from(error).with_operation("发送请求"))?;
    stream
        .flush()
        .map_err(|error| Error::from(error).with_operation("发送请求"))?;

    let mut reader = BufReader::new(stream);
    let mut line = String::new();
    reader
        .read_line(&mut line)
        .map_err(|error| Error::from(error).with_operation("读取 supervisor 响应"))?;
    serde_json::from_str(&line)
        .map_err(|error| Error::caused_by(ErrorKind::InvalidData, "解析 supervisor 响应", error))
}
