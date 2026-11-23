use crate::pixels::SIDE;
use firefly_config::paths::models;
use firefly_error::{Error, ErrorKind, Result};
use ort::{session::Session, value::Tensor};

#[track_caller]
fn inference_error(e: impl Into<ort::Error>) -> Error {
    Error::caused_by(ErrorKind::Internal, "run ONNX depth model", e.into())
}

pub enum Model {
    Cpu {
        session: Box<Session>,
        input: Tensor<f32>,
    },
    #[cfg(target_os = "linux")]
    Npu(Box<firefly_viplite::Network>),
}

impl Model {
    pub fn open(simulation: bool) -> Result<Self> {
        if simulation {
            let env = ort::init().build().map_err(inference_error)?;
            let path = models("yolo26n-depth/yolo26n-depth-320.onnx");
            let session = Session::builder(&env)
                .map_err(inference_error)?
                .with_intra_threads(1)
                .map_err(inference_error)?
                .with_intra_op_spinning(false)
                .map_err(inference_error)?
                .commit_from_file(&path)
                .map_err(|e| {
                    inference_error(e)
                        .with_operation("load depth model")
                        .with_context("path", path.display())
                })?;
            let input = Tensor::from_array(([1, 3, SIDE, SIDE], vec![0.; 3 * SIDE * SIDE]))
                .map_err(inference_error)?;
            return Ok(Self::Cpu {
                session: Box::new(session),
                input,
            });
        }
        #[cfg(target_os = "linux")]
        {
            use firefly_error::IoResultExt;
            let path = models("yolo26n-depth/yolo26n-depth-320-int8.nb");
            let bytes = std::fs::read(&path).at_path("read depth model", &path)?;
            Ok(Self::Npu(Box::new(firefly_viplite::Network::open_depth(
                bytes, SIDE,
            )?)))
        }
        #[cfg(not(target_os = "linux"))]
        Err(Error::new(
            ErrorKind::Unsupported,
            "VIPLite requires A733 Linux. Use --simulation on the SIL host",
        ))
    }

    pub fn run<T>(
        &mut self,
        gray: &[u8],
        consume: impl FnOnce(usize, &[f32]) -> Result<T>,
    ) -> Result<T> {
        match self {
            Self::Cpu { session, input } => {
                for channel in input
                    .try_extract_tensor_mut::<f32>()
                    .map_err(inference_error)?
                    .1
                    .chunks_exact_mut(gray.len())
                {
                    for (dst, &pixel) in channel.iter_mut().zip(gray) {
                        *dst = f32::from(pixel) / 255.;
                    }
                }
                let outputs = session
                    .run(ort::inputs!["images" => &*input])
                    .map_err(inference_error)?;
                let (shape, data) = outputs[0]
                    .try_extract_tensor::<f32>()
                    .map_err(inference_error)?;
                if !matches!(&shape[..], [1, 1, 320, 320] | [1, 320, 320]) {
                    return Err(Error::new(
                        ErrorKind::InvalidData,
                        format!("Unexpected ONNX depth shape {shape:?}"),
                    ));
                }
                consume(SIDE, data)
            }
            #[cfg(target_os = "linux")]
            Self::Npu(network) => consume(SIDE / 4, network.run(gray)?),
        }
    }
}
