//! 固定指纹的 INT8 ONNX 模型；生产会话要求 VSINPU 且所有图节点均可放置。
use std::path::{Path, PathBuf};

use firefly_landing::{Detection, Letterbox, decode_raw_detections};
use firefly_pubsub::camera::{DOWN_HEIGHT, DOWN_WIDTH};
use ort::{session::Session, value::Tensor};
use serde::Deserialize;
use sha2::{Digest, Sha256};

use crate::{Result, external, invalid};

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Backend {
    #[default]
    Vsinpu,
    Cpu,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    pub schema: u32,
    pub model: PathBuf,
    pub sha256: String,
    pub source_sha256: String,
    pub calibration_sha256: String,
    pub quantization: String,
    pub input: [usize; 4],
    /// 相机输出宽、高，须与 IPC 消息布局一致。
    pub camera: [usize; 2],
    pub output: String,
    pub classes: Vec<String>,
}

impl Manifest {
    pub fn validate(&self) -> Result<()> {
        if self.schema != 1
            || self.quantization != "int8_qdq"
            || self.camera != [DOWN_WIDTH, DOWN_HEIGHT]
            || self.input[0] != 1
            || self.input[1] != 3
            || self.input[2..]
                .iter()
                .any(|&n| n == 0 || n > 640 || n % 32 != 0)
            || self.output != "yolo_xywh_score"
            || self.classes != ["h_marker"]
            || [&self.sha256, &self.source_sha256, &self.calibration_sha256]
                .iter()
                .any(|s| s.len() != 64 || !s.bytes().all(|c| c.is_ascii_hexdigit()))
            || self.model.components().count() != 1
            || !self
                .model
                .components()
                .all(|part| matches!(part, std::path::Component::Normal(_)))
        {
            return Err(invalid("invalid quantized landing model manifest"));
        }
        Ok(())
    }
}

pub struct Model {
    session: Session,
    input: Tensor<f32>,
    transform: Letterbox,
}

/// ORT 环境由进程宿主持有；工作周期仅创建和释放 Session。
pub fn environment(runtime: Option<&Path>) -> Result<ort::environment::Environment> {
    #[cfg(target_os = "linux")]
    let environment = if let Some(runtime) = runtime {
        ort::init_from(runtime)
            .map_err(|e| external("landing.runtime.load", e))?
            .build()
            .map_err(|e| external("landing.runtime.create", e))?
    } else {
        ort::init()
            .build()
            .map_err(|e| external("landing.runtime.create", e))?
    };
    #[cfg(not(target_os = "linux"))]
    let environment = {
        if runtime.is_some() {
            return Err(invalid("runtime path is supported by the Linux build"));
        }
        ort::init()
            .build()
            .map_err(|e| external("landing.runtime.create", e))?
    };
    Ok(environment)
}

impl Model {
    pub fn load(
        manifest_path: &Path,
        backend: Backend,
        environment: &ort::environment::Environment,
    ) -> Result<Self> {
        let manifest: Manifest =
            serde_json::from_slice(&std::fs::read(manifest_path).map_err(|e| {
                firefly_error::Error::from(e)
                    .with_operation("landing.manifest.read")
                    .with_context("path", manifest_path.display())
            })?)
            .map_err(|e| invalid("invalid model manifest JSON").with_source(e))?;
        manifest.validate()?;
        let model_path = manifest_path
            .parent()
            .ok_or_else(|| invalid("manifest parent missing"))?
            .join(&manifest.model);
        let bytes = std::fs::read(&model_path).map_err(|e| {
            firefly_error::Error::from(e)
                .with_operation("landing.model.read")
                .with_context("path", model_path.display())
        })?;
        if format!("{:x}", Sha256::digest(&bytes)) != manifest.sha256 {
            return Err(invalid("landing model SHA256 mismatch"));
        }
        let builder = Session::builder(environment)
            .map_err(|e| external("landing.session.create", e))?
            .with_intra_threads(1)
            .map_err(|e| external("landing.session.configure", e))?
            .with_intra_op_spinning(false)
            .map_err(|e| external("landing.session.configure", e))?
            .with_auto_device(ort::session::builder::AutoDevicePolicy::PreferCPU)
            .map_err(|e| external("landing.session.configure", e))?;
        let mut builder = match backend {
            Backend::Cpu => builder,
            Backend::Vsinpu => {
                #[cfg(target_os = "linux")]
                {
                    builder
                        .with_execution_providers([ort::ep::VSINPU::default()
                            .build()
                            .error_on_failure()])
                        .map_err(|e| external("landing.vsinpu.register", e))?
                        .with_disable_cpu_fallback()
                        .map_err(|e| external("landing.vsinpu.configure", e))?
                }
                #[cfg(not(target_os = "linux"))]
                {
                    return Err(firefly_error::Error::new(firefly_error::ErrorKind::Unsupported, "VSINPU requires the A733 Linux runtime; use CPU only for explicit offline comparison").with_operation("landing.vsinpu.register"));
                }
            }
        };
        let session = builder
            .commit_from_memory(&bytes)
            .map_err(|e| external("landing.model.compile", e))?;
        let input = Tensor::from_array((manifest.input, vec![0.; manifest.input.iter().product()]))
            .map_err(|e| external("landing.tensor.create", e))?;
        let transform = Letterbox::new(manifest.camera, [manifest.input[3], manifest.input[2]])?;
        log::info!(
            "landing model ready: backend={backend:?}, sha256={}, input={:?}",
            manifest.sha256,
            manifest.input
        );
        Ok(Self {
            session,
            input,
            transform,
        })
    }

    pub fn detect(&mut self, gray: &[u8], threshold: f32) -> Result<Vec<Detection>> {
        self.transform.write_gray(
            gray,
            self.input
                .try_extract_tensor_mut::<f32>()
                .map_err(|e| external("landing.tensor.input", e))?
                .1,
        )?;
        let outputs = self
            .session
            .run(ort::inputs!["images" => &self.input])
            .map_err(|e| external("landing.inference", e))?;
        if outputs.len() != 1 {
            return Err(invalid("landing model must have one output"));
        }
        let (shape, values) = outputs[0]
            .try_extract_tensor::<f32>()
            .map_err(|e| external("landing.tensor.output", e))?;
        decode_raw_detections(shape, values, &self.transform, threshold)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn manifest_requires_quantization_fixed_input_and_local_artifact() {
        let mut manifest = Manifest {
            schema: 1,
            model: "model.onnx".into(),
            sha256: "a".repeat(64),
            source_sha256: "b".repeat(64),
            calibration_sha256: "c".repeat(64),
            quantization: "int8_qdq".into(),
            input: [1, 3, 288, 352],
            camera: [352, 288],
            output: "yolo_xywh_score".into(),
            classes: vec!["h_marker".into()],
        };
        manifest.validate().unwrap();
        manifest.model = "../model.onnx".into();
        assert!(manifest.validate().is_err());
        manifest.model = "model.onnx".into();
        manifest.quantization = "fp32".into();
        assert!(manifest.validate().is_err());
    }
}
