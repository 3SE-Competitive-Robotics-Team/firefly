//! 规划过程可视化（rerun viewer）。
//!
//! 记录地图、路径、轨迹、障碍平面到 rerun viewer：
//! `cargo run -p firefly-viewer --example demo`（需先启动 `rerun` viewer）。

use firefly_error::{Error, ErrorKind, Result};
use firefly_map::{GridMap, Plane, VoxelState};
use firefly_trajectory::Trajectory;
use nalgebra::Vector3;

pub struct Viewer {
    rec: rerun::RecordingStream,
}

impl Viewer {
    /// 连接已启动的 rerun viewer（或自动 spawn）。
    ///
    /// # Errors
    ///
    /// `InvalidArgument`：应用名非法；`Internal`：无法连接 viewer。
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
    pub fn save(app_id: &str, path: impl Into<std::path::PathBuf>) -> Result<Self> {
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

    /// 占据栅格地图 → 占据体素点云。
    ///
    /// # Errors
    ///
    /// `Internal`：rerun 记录失败。
    pub fn log_map(&self, entity: &str, map: &GridMap) -> Result<()> {
        let res = map.resolution();
        let origin = map.origin();
        let dims = map.dims();
        let mut positions = Vec::new();
        for x in 0..dims[0] {
            for y in 0..dims[1] {
                for z in 0..dims[2] {
                    if map.state([x, y, z]) == VoxelState::Occupied {
                        positions.push([
                            origin.x + (x as f64 + 0.5) * res,
                            origin.y + (y as f64 + 0.5) * res,
                            origin.z + (z as f64 + 0.5) * res,
                        ]);
                    }
                }
            }
        }
        self.rec
            .log(entity, &rerun::Points3D::new(positions))
            .map_err(viewer_err)
    }

    /// 路径/折线 → 3D 线段。
    ///
    /// # Errors
    ///
    /// `Internal`：rerun 记录失败。
    pub fn log_path(&self, entity: &str, points: &[Vector3<f64>]) -> Result<()> {
        let pts: Vec<[f64; 3]> = points.iter().map(|p| [p.x, p.y, p.z]).collect();
        self.rec
            .log(entity, &rerun::LineStrips3D::new([pts]))
            .map_err(viewer_err)
    }

    /// 轨迹 → 采样折线（位置）+ 速度向量。
    ///
    /// # Errors
    ///
    /// `Internal`：rerun 记录失败。
    pub fn log_trajectory(&self, entity: &str, traj: &Trajectory) -> Result<()> {
        const SAMPLES: usize = 100;
        let mut pts = Vec::new();
        let mut arrows: Vec<[f64; 3]> = Vec::new();
        for k in 0..SAMPLES {
            let t = traj.duration() * k as f64 / SAMPLES as f64;
            let s = traj.eval(t);
            pts.push([s.position.x, s.position.y, s.position.z]);
            arrows.push([s.velocity.x, s.velocity.y, s.velocity.z]);
        }
        self.rec
            .log(entity, &rerun::LineStrips3D::new([pts]))
            .map_err(viewer_err)?;
        self.rec
            .log(
                format!("{entity}/velocity").as_str(),
                &rerun::Arrows3D::from_vectors(arrows),
            )
            .map_err(viewer_err)
    }

    /// 无人机当前位置 → 3D 点。
    ///
    /// # Errors
    ///
    /// `Internal`：rerun 记录失败。
    pub fn log_position(&self, entity: &str, position: [f64; 3]) -> Result<()> {
        self.rec
            .log(entity, &rerun::Points3D::new([position]))
            .map_err(viewer_err)
    }

    /// 障碍平面（{s, v}）→ 法线向量。
    ///
    /// # Errors
    ///
    /// `Internal`：rerun 记录失败。
    pub fn log_planes(&self, entity: &str, planes: &[Plane]) -> Result<()> {
        let origins: Vec<[f64; 3]> = planes
            .iter()
            .map(|p| {
                let s = p.point();
                [s.x, s.y, s.z]
            })
            .collect();
        let vectors: Vec<[f64; 3]> = planes
            .iter()
            .map(|p| {
                let v = p.normal() * 0.3;
                [v.x, v.y, v.z]
            })
            .collect();
        self.rec
            .log(
                entity,
                &rerun::Arrows3D::from_vectors(vectors).with_origins(origins),
            )
            .map_err(viewer_err)
    }

    /// 清除实体。
    ///
    /// # Errors
    ///
    /// `Internal`：rerun 记录失败。
    pub fn clear(&self, entity: &str) -> Result<()> {
        self.rec
            .log(entity, &rerun::Clear::recursive())
            .map_err(viewer_err)
    }
}

fn viewer_err(e: rerun::RecordingStreamError) -> Error {
    Error::new(ErrorKind::Internal, "rerun log failed").with_source(e)
}
