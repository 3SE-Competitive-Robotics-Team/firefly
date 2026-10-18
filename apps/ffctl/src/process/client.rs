//! supervisor 控制客户端：一次性 CLI 通过 Unix socket 发请求。

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::time::Duration;

use crate::process::SOCKET;
use crate::process::protocol::{Request, Response};

const RESPONSE_TIMEOUT: Duration = Duration::from_secs(60);

pub(crate) fn send(request: &Request) -> Result<Response, String> {
    let mut stream = UnixStream::connect(SOCKET).map_err(|error| {
        format!("连接 supervisor 失败（{error}）；先运行 systemctl start firefly")
    })?;
    stream
        .set_read_timeout(Some(RESPONSE_TIMEOUT))
        .map_err(|error| format!("设置 supervisor 超时: {error}"))?;

    let mut text = serde_json::to_string(request).map_err(|error| format!("编码请求: {error}"))?;
    text.push('\n');
    stream
        .write_all(text.as_bytes())
        .map_err(|error| format!("发送请求: {error}"))?;
    stream
        .flush()
        .map_err(|error| format!("发送请求: {error}"))?;

    let mut reader = BufReader::new(stream);
    let mut line = String::new();
    reader
        .read_line(&mut line)
        .map_err(|error| format!("读取 supervisor 响应: {error}"))?;
    serde_json::from_str(&line).map_err(|error| format!("解析 supervisor 响应: {error}"))
}
