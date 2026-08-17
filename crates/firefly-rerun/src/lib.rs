//! Rerun 可视化连接层：多进程共享 viewer + 图像/深度/位姿记录。
//!
//! 一个 `rerun` viewer 可同时接受多个进程的 gRPC 流（默认转发端口
//! `127.0.0.1:9876`）：先 `rerun` 开 viewer，各进程 [`Stream::connect`]
//! 即可往同一个 viewer 写数据（vio 的传感器/odom、demo 的规划结果等）。
//! 未开 viewer 时 [`Stream::connect_or_spawn`] 自动起一个新 viewer。
//!
//! 所有进程共用 `sim_time` 时间轴（仿真秒，Duration 型），保证跨进程
//! 数据按同一时钟对齐回放。

use std::net::{SocketAddr, TcpStream};
use std::path::PathBuf;
use std::time::Duration;

use firefly_error::{Error, ErrorKind, Result};

/// 本机 `rerun` viewer 的 SDK 转发端口（`rerun::DEFAULT_SERVER_PORT`）。
const VIEWER_PORT: u16 = 9876;
/// 探测 viewer 是否在线的超时（秒）。
const PROBE_TIMEOUT: Duration = Duration::from_millis(150);

/// 一个已连接（或已 spawn）的 rerun 记录流。
pub struct Stream {
    rec: rerun::RecordingStream,
}

impl Stream {
    /// 连接已启动的 rerun viewer（默认 `127.0.0.1:9876` 转发端口）。
    ///
    /// # Errors
    ///
    /// `InvalidArgument`：应用名非法；`Internal`：无法建立记录流。
    pub fn connect(app_id: &str) -> Result<Self> {
        let app = rerun::ApplicationId::try_new(app_id).map_err(|e| {
            Error::new(ErrorKind::InvalidArgument, "invalid application id").with_source(e)
        })?;
        let rec = rerun::RecordingStreamBuilder::new(app)
            .connect_grpc()
            .map_err(|e| {
                Error::new(ErrorKind::Internal, "failed to connect rerun viewer").with_source(e)
            })?;
        Ok(Self { rec })
    }

    /// 自起一个新 rerun viewer 并连接（独立运行无 viewer 时使用）。
    ///
    /// # Errors
    ///
    /// `InvalidArgument`：应用名非法；`Internal`：无法 spawn viewer。
    pub fn spawn(app_id: &str) -> Result<Self> {
        let app = rerun::ApplicationId::try_new(app_id).map_err(|e| {
            Error::new(ErrorKind::InvalidArgument, "invalid application id").with_source(e)
        })?;
        let rec = rerun::RecordingStreamBuilder::new(app)
            .spawn()
            .map_err(|e| {
                Error::new(ErrorKind::Internal, "failed to spawn rerun viewer").with_source(e)
            })?;
        Ok(Self { rec })
    }

    /// 保存到 rrd 文件（无 viewer 时离线记录）。
    ///
    /// # Errors
    ///
    /// `InvalidArgument`：应用名非法；`Internal`：无法创建记录。
    pub fn save(app_id: &str, path: impl Into<PathBuf>) -> Result<Self> {
        let app = rerun::ApplicationId::try_new(app_id).map_err(|e| {
            Error::new(ErrorKind::InvalidArgument, "invalid application id").with_source(e)
        })?;
        let rec = rerun::RecordingStreamBuilder::new(app)
            .save(path)
            .map_err(|e| {
                Error::new(ErrorKind::Internal, "failed to create rerun recording").with_source(e)
            })?;
        Ok(Self { rec })
    }

    /// 已有 viewer 在监听则连接，否则自起一个（多进程闭环的默认入口）。
    ///
    /// # Errors
    ///
    /// 同 [`Self::connect`] / [`Self::spawn`]。
    pub fn connect_or_spawn(app_id: &str) -> Result<Self> {
        if viewer_listening() {
            log::debug!("rerun viewer 在线（127.0.0.1:{VIEWER_PORT}），连接共享 viewer");
            Self::connect(app_id)
        } else {
            log::debug!("无 rerun viewer，自动 spawn");
            Self::spawn(app_id)
        }
    }

    /// 底层记录流（firefly-viewer 等上层经此记录地图/轨迹等离散实体）。
    #[must_use]
    pub fn stream(&self) -> &rerun::RecordingStream {
        &self.rec
    }

    /// 设置统一仿真时间轴 `sim_time`（秒，Duration 型）。
    ///
    /// 本线程后续所有 log 都落在该时刻，直到下次调用。
    pub fn set_time(&self, seconds: f64) {
        self.rec.set_time(
            "sim_time",
            rerun::TimeCell::from_duration_nanos((seconds * 1e9) as i64),
        );
    }

    /// 8-bit 灰度图（如双目灰度）→ rerun 图像实体。
    ///
    /// # Errors
    ///
    /// `Internal`：rerun 记录失败。
    pub fn log_gray_image(
        &self,
        entity: &str,
        width: u32,
        height: u32,
        data: &[u8],
    ) -> Result<()> {
        self.rec
            .log(
                entity,
                &rerun::Image::from_l8(data.to_vec(), [width, height]),
            )
            .map_err(stream_err)
    }

    /// 单通道 f32 深度（米，行主序）→ rerun 深度图实体（Viridis 着色）。
    ///
    /// # Errors
    ///
    /// `Internal`：rerun 记录失败。
    pub fn log_depth_image(
        &self,
        entity: &str,
        width: u32,
        height: u32,
        data: &[f32],
    ) -> Result<()> {
        let bytes: Vec<u8> = data.iter().flat_map(|v| v.to_le_bytes()).collect();
        let depth = rerun::DepthImage::from_data_type_and_bytes(
            bytes,
            [width, height],
            rerun::datatypes::ChannelDatatype::F32,
        )
        .with_meter(1.0)
        .with_colormap(rerun::components::Colormap::Viridis);
        self.rec.log(entity, &depth).map_err(stream_err)
    }

    /// 位姿（平移 + 旋转）→ 3D 刚体变换实体（子实体挂其下即随之移动）。
    ///
    /// 四元数为 xyzw（分量顺序，viewer 内自动归一化）。
    ///
    /// # Errors
    ///
    /// `Internal`：rerun 记录失败。
    pub fn log_pose(
        &self,
        entity: &str,
        position: [f64; 3],
        quat_xyzw: [f64; 4],
    ) -> Result<()> {
        let translation = rerun::components::Translation3D::from([
            position[0] as f32,
            position[1] as f32,
            position[2] as f32,
        ]);
        let rotation = rerun::Rotation3D::from(rerun::datatypes::Quaternion([
            quat_xyzw[0] as f32,
            quat_xyzw[1] as f32,
            quat_xyzw[2] as f32,
            quat_xyzw[3] as f32,
        ]));
        self.rec
            .log(
                entity,
                &rerun::Transform3D::from_translation_rotation(translation, rotation),
            )
            .map_err(stream_err)
    }

    /// 清除实体（含子树）。
    ///
    /// # Errors
    ///
    /// `Internal`：rerun 记录失败。
    pub fn clear(&self, entity: &str) -> Result<()> {
        self.rec
            .log(entity, &rerun::Clear::recursive())
            .map_err(stream_err)
    }
}

/// 探测本机是否已有 rerun viewer 在监听转发端口。
fn viewer_listening() -> bool {
    let addr: SocketAddr = ([127, 0, 0, 1], VIEWER_PORT).into();
    TcpStream::connect_timeout(&addr, PROBE_TIMEOUT).is_ok()
}

fn stream_err(e: rerun::RecordingStreamError) -> Error {
    Error::new(ErrorKind::Internal, "rerun log failed").with_source(e)
}