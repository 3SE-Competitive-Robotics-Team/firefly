use crate::pixels::SIDE;
use firefly_config::paths::models;
use ort::{session::Session, value::Tensor};
use std::error::Error;

pub enum Model {
    Cpu {
        session: Box<Session>,
        input: Tensor<f32>,
    },
    #[cfg(target_os = "linux")]
    Npu(Box<firefly_viplite::Network>),
}

impl Model {
    pub fn open(simulation: bool) -> Result<Self, Box<dyn Error>> {
        if simulation {
            let env = ort::init().build()?;
            let session = Session::builder(&env)?
                .with_intra_threads(1)?
                .with_intra_op_spinning(false)?
                .commit_from_file(models("yolo26n-depth/yolo26n-depth-320.onnx"))?;
            let input = Tensor::from_array(([1, 3, SIDE, SIDE], vec![0.; 3 * SIDE * SIDE]))?;
            return Ok(Self::Cpu {
                session: Box::new(session),
                input,
            });
        }
        #[cfg(target_os = "linux")]
        {
            let bytes = std::fs::read(models("yolo26n-depth/yolo26n-depth-320-int8.nb"))?;
            Ok(Self::Npu(Box::new(firefly_viplite::Network::open_depth(
                bytes, SIDE,
            )?)))
        }
        #[cfg(not(target_os = "linux"))]
        Err("VIPLite requires A733 Linux. Use --simulation on the SIL host".into())
    }

    pub fn run<'a>(
        &'a mut self,
        gray: &[u8],
        cpu_output: &'a mut Vec<f32>,
    ) -> Result<(usize, &'a [f32]), Box<dyn Error>> {
        match self {
            Self::Cpu { session, input } => {
                for channel in input
                    .try_extract_tensor_mut::<f32>()?
                    .1
                    .chunks_exact_mut(gray.len())
                {
                    for (dst, &pixel) in channel.iter_mut().zip(gray) {
                        *dst = f32::from(pixel) / 255.;
                    }
                }
                let outputs = session.run(ort::inputs!["images" => &*input])?;
                let (shape, data) = outputs[0].try_extract_tensor::<f32>()?;
                if !matches!(&shape[..], [1, 1, 320, 320] | [1, 320, 320]) {
                    return Err(format!("Unexpected ONNX depth shape {shape:?}").into());
                }
                cpu_output.clear();
                cpu_output.extend_from_slice(data);
                Ok((SIDE, cpu_output))
            }
            #[cfg(target_os = "linux")]
            Self::Npu(network) => Ok((SIDE / 4, network.run(gray)?)),
        }
    }
}
