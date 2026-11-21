//! 下视 H 标记检测。桌面仿真使用 ONNX，板端使用预编译 NBG。
use std::path::{Path, PathBuf};

use firefly_landing::{Detection, Letterbox, decode_raw_detections};
use firefly_pubsub::camera::{DOWN_HEIGHT, DOWN_WIDTH};
use ort::{session::Session, value::Tensor};
use serde::Deserialize;

use crate::{Result, external, invalid};

#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Backend {
    #[default]
    Viplite,
    Cpu,
}

#[derive(Debug, Deserialize)]
pub struct Manifest {
    pub schema: u32,
    pub model: PathBuf,
    pub quantization: String,
    pub input: [usize; 4],
    /// 相机输出宽、高，须与 IPC 消息布局一致。
    pub camera: [usize; 2],
    pub output: String,
    pub classes: Vec<String>,
}

impl Manifest {
    pub fn validate(&self, backend: Backend) -> Result<()> {
        if backend == Backend::Viplite && self.input != [1, 3, DOWN_HEIGHT, DOWN_WIDTH] {
            return Err(invalid("VIPLite expects the down camera resolution"));
        }
        if self.schema != 1
            || self.quantization
                != match backend {
                    Backend::Cpu => "int8_qdq",
                    Backend::Viplite => "viplite_int8",
                }
            || self.camera != [DOWN_WIDTH, DOWN_HEIGHT]
            || self.input[0] != 1
            || self.input[1] != 3
            || self.input[2..]
                .iter()
                .any(|&n| n == 0 || n > 640 || n % 32 != 0)
            || self.output != "yolo_xywh_score"
            || self.classes != ["h_marker"]
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
    engine: Engine,
    transform: Letterbox,
}

enum Engine {
    Cpu {
        session: Session,
        input: Tensor<f32>,
    },
    #[cfg(target_os = "linux")]
    Viplite(Box<firefly_viplite::Network>),
}

pub enum Environment {
    Cpu(ort::environment::Environment),
    Viplite,
}

/// 桌面 ORT 环境跨精降周期复用。板端网络由当前精降周期持有。
pub fn environment(backend: Backend, runtime: Option<&Path>) -> Result<Environment> {
    if backend == Backend::Viplite {
        return Ok(Environment::Viplite);
    }
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
    Ok(Environment::Cpu(environment))
}

impl Model {
    pub fn load(manifest_path: &Path, environment: &Environment) -> Result<Self> {
        let manifest: Manifest =
            serde_json::from_slice(&std::fs::read(manifest_path).map_err(|e| {
                firefly_error::Error::from(e)
                    .with_operation("landing.manifest.read")
                    .with_context("path", manifest_path.display())
            })?)
            .map_err(|e| invalid("invalid model manifest JSON").with_source(e))?;
        let backend = match environment {
            Environment::Cpu(_) => Backend::Cpu,
            Environment::Viplite => Backend::Viplite,
        };
        manifest.validate(backend)?;
        let model_path = manifest_path
            .parent()
            .ok_or_else(|| invalid("manifest parent missing"))?
            .join(&manifest.model);
        let bytes = std::fs::read(&model_path).map_err(|e| {
            firefly_error::Error::from(e)
                .with_operation("landing.model.read")
                .with_context("path", model_path.display())
        })?;
        let engine = match environment {
            Environment::Cpu(environment) => {
                let session = Session::builder(environment)
                    .map_err(|e| external("landing.session.create", e))?
                    .with_intra_threads(1)
                    .map_err(|e| external("landing.session.configure", e))?
                    .with_intra_op_spinning(false)
                    .map_err(|e| external("landing.session.configure", e))?
                    .with_auto_device(ort::session::builder::AutoDevicePolicy::PreferCPU)
                    .map_err(|e| external("landing.session.configure", e))?
                    .commit_from_memory(&bytes)
                    .map_err(|e| external("landing.model.compile", e))?;
                let input =
                    Tensor::from_array((manifest.input, vec![0.; manifest.input.iter().product()]))
                        .map_err(|e| external("landing.tensor.create", e))?;
                Engine::Cpu { session, input }
            }
            Environment::Viplite => {
                #[cfg(target_os = "linux")]
                {
                    Engine::Viplite(Box::new(
                        firefly_viplite::Network::open(bytes, DOWN_WIDTH, DOWN_HEIGHT)
                            .map_err(|e| external("landing.viplite.open", e))?,
                    ))
                }
                #[cfg(not(target_os = "linux"))]
                {
                    return Err(invalid("VIPLite requires the A733 Linux runtime"));
                }
            }
        };
        let transform = Letterbox::new(manifest.camera, [manifest.input[3], manifest.input[2]])?;
        log::info!(
            "landing model ready: backend={backend:?}, model={}, input={:?}",
            model_path.display(),
            manifest.input
        );
        Ok(Self { engine, transform })
    }

    pub fn detect(&mut self, gray: &[u8], threshold: f32) -> Result<Vec<Detection>> {
        match &mut self.engine {
            Engine::Cpu { session, input } => {
                self.transform.write_gray(
                    gray,
                    input
                        .try_extract_tensor_mut::<f32>()
                        .map_err(|e| external("landing.tensor.input", e))?
                        .1,
                )?;
                let outputs = session
                    .run(ort::inputs!["images" => &*input])
                    .map_err(|e| external("landing.inference", e))?;
                if outputs.len() != 1 {
                    return Err(invalid("landing model must have one output"));
                }
                let (shape, values) = outputs[0]
                    .try_extract_tensor::<f32>()
                    .map_err(|e| external("landing.tensor.output", e))?;
                decode_raw_detections(shape, values, &self.transform, threshold)
            }
            #[cfg(target_os = "linux")]
            Engine::Viplite(network) => {
                let values = network
                    .run(gray)
                    .map_err(|e| external("landing.viplite.run", e))?;
                decode_raw_detections(
                    &[1, 5, (values.len() / 5) as i64],
                    values,
                    &self.transform,
                    threshold,
                )
            }
        }
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
            quantization: "int8_qdq".into(),
            input: [1, 3, 288, 352],
            camera: [352, 288],
            output: "yolo_xywh_score".into(),
            classes: vec!["h_marker".into()],
        };
        manifest.validate(Backend::Cpu).unwrap();
        assert!(manifest.validate(Backend::Viplite).is_err());
        manifest.model = "../model.onnx".into();
        assert!(manifest.validate(Backend::Cpu).is_err());
        manifest.model = "model.onnx".into();
        manifest.quantization = "viplite_int8".into();
        manifest.validate(Backend::Viplite).unwrap();
        assert!(manifest.validate(Backend::Cpu).is_err());
        manifest.input[2] = 320;
        assert!(manifest.validate(Backend::Viplite).is_err());
    }
}
