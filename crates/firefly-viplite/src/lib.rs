//! Synchronous VIPLite inference for A733 vision models.

// C ABI from vip_lite.h: keep C names, constants and the SDK's struct layout; the module's lints
// do not apply to either the generated declarations or the hand-maintained loader below.
#[allow(
    non_camel_case_types,
    non_snake_case,
    dead_code,
    unused_imports,
    unsafe_op_in_unsafe_fn,
    clippy::all
)]
mod ffi;

use ffi::*;
use firefly_error::{Error, ErrorKind, Result};
use std::{
    ffi::c_void,
    ptr, slice,
    sync::{Mutex, MutexGuard},
};

// VIPLite initialization and shutdown have process scope.
static DEVICE: Mutex<()> = Mutex::new(());

fn check(operation: &'static str, status: vip_status_e) -> Result<()> {
    if status == VIP_SUCCESS {
        Ok(())
    } else {
        Err(Error::new(ErrorKind::Io, "VIPLite call failed")
            .with_operation(operation)
            .with_context("status", status))
    }
}

struct Buffer {
    handle: vip_buffer,
    elements: usize,
    scale: f32,
    zero: i8,
}

pub struct Network {
    api: Vip,
    network: vip_network,
    prepared: bool,
    buffers: Vec<Buffer>,
    candidates: usize,
    lookup: [u8; 256],
    output: Vec<f32>,
    // The SDK may retain the NBG address until vip_destroy_network.
    model: Vec<u8>,
    _device: MutexGuard<'static, ()>,
}

#[derive(PartialEq, Eq)]
enum ModelKind {
    Detection,
    MonoDepth,
    Stereo,
}

/// Raw planar stereo input with a camera-sized FP32 relative disparity head.
pub struct StereoNetwork(Network);

impl StereoNetwork {
    pub fn open(model: Vec<u8>, width: usize, height: usize) -> Result<Self> {
        Network::create(model, width, height, ModelKind::Stereo).map(Self)
    }

    /// Run CPU correspondence while the device owns the input and output buffers.
    pub fn run(
        &mut self,
        left: &[u8],
        right: &[u8],
        correspond: impl FnOnce() -> Result<()>,
    ) -> Result<&[f32]> {
        let net = &mut self.0;
        let input = &net.buffers[0];
        if left.len() != net.output.len() || right.len() != left.len() {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "Stereo image dimensions differ from NBG",
            ));
        }
        // SAFETY: open validated both tensor sizes and formats. Mappings are scoped
        // to the copies; CPU correspondence has no access to the device buffers.
        unsafe {
            let dst = net.api.vip_map_buffer(input.handle).cast::<u8>();
            if dst.is_null() {
                return Err(Error::new(ErrorKind::Io, "Map stereo input"));
            }
            let (l, r) = slice::from_raw_parts_mut(dst, input.elements).split_at_mut(left.len());
            l.copy_from_slice(left);
            r.copy_from_slice(right);
            check("unmap stereo input", net.api.vip_unmap_buffer(input.handle))?;
            check(
                "flush stereo input",
                net.api
                    .vip_flush_buffer(input.handle, VIP_BUFFER_OPER_TYPE_FLUSH),
            )?;
            check(
                "submit stereo network",
                (net.api.vip_trigger_network)(net.network),
            )?;
            let matching = correspond();
            check(
                "wait for stereo network",
                (net.api.vip_wait_network)(net.network),
            )?;
            matching?;
            let output = net.buffers[1].handle;
            check(
                "invalidate relative disparity",
                net.api
                    .vip_flush_buffer(output, VIP_BUFFER_OPER_TYPE_INVALIDATE),
            )?;
            let src = net.api.vip_map_buffer(output).cast::<f32>();
            if src.is_null() {
                return Err(Error::new(ErrorKind::Io, "Map relative disparity"));
            }
            net.output
                .copy_from_slice(slice::from_raw_parts(src, net.buffers[1].elements));
            check("unmap relative disparity", net.api.vip_unmap_buffer(output))?;
        }
        Ok(&net.output)
    }
}

impl Network {
    pub fn open(model: Vec<u8>, width: usize, height: usize) -> Result<Self> {
        Self::create(model, width, height, ModelKind::Detection)
    }

    /// Compact depth networks expose a single FP16 metric head at quarter resolution.
    pub fn open_depth(model: Vec<u8>, side: usize) -> Result<Self> {
        Self::create(model, side, side, ModelKind::MonoDepth)
    }

    fn create(model: Vec<u8>, width: usize, height: usize, kind: ModelKind) -> Result<Self> {
        let size = u32::try_from(model.len()).map_err(|e| {
            Error::caused_by(ErrorKind::OutOfRange, "NBG exceeds the SDK size limit", e)
        })?;
        let device = DEVICE.try_lock().map_err(|e| match e {
            std::sync::TryLockError::WouldBlock => {
                Error::new(ErrorKind::Unavailable, "VIPLite is already in use")
            }
            std::sync::TryLockError::Poisoned(_) => {
                Error::new(ErrorKind::Internal, "VIPLite device lock is poisoned")
            }
        })?;
        // Load the BSP-provided C ABI. Each handle below is owned by this single-threaded session.
        let api = unsafe { Vip::new("libNBGlinker.so") }
            .map_err(|e| Error::caused_by(ErrorKind::Unavailable, "load libNBGlinker.so", e))?;
        check("vip_init", unsafe { api.vip_init() })?;
        let mut result = Self {
            api,
            network: ptr::null_mut(),
            prepared: false,
            buffers: Vec::new(),
            candidates: 0,
            lookup: [0; 256],
            output: Vec::new(),
            model,
            _device: device,
        };
        check("vip_create_network", unsafe {
            result.api.vip_create_network(
                result.model.as_ptr().cast(),
                size,
                VIP_CREATE_NETWORK_FROM_MEMORY,
                &mut result.network,
            )
        })?;
        for (property, expected) in [
            (VIP_NETWORK_PROP_INPUT_COUNT, 1),
            (
                VIP_NETWORK_PROP_OUTPUT_COUNT,
                if kind == ModelKind::Detection { 2 } else { 1 },
            ),
        ] {
            let mut count = 0_u32;
            check("vip_query_network", unsafe {
                result.api.vip_query_network(
                    result.network,
                    property as i32,
                    ptr::from_mut(&mut count).cast(),
                )
            })?;
            if count != expected {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    format!("Unexpected NBG port count {count}, expected {expected}"),
                ));
            }
        }
        let stereo = kind == ModelKind::Stereo;
        let input = result.bind(
            true,
            0,
            if stereo {
                VIP_BUFFER_FORMAT_UINT8
            } else {
                VIP_BUFFER_FORMAT_INT8
            },
        )?;
        if input != [width, height, if stereo { 2 } else { 3 }, 1] {
            return Err(Error::new(
                ErrorKind::InvalidData,
                format!("Unexpected NBG input {input:?}"),
            ));
        }
        if kind != ModelKind::Detection {
            let dims = result.bind(
                false,
                0,
                if stereo {
                    VIP_BUFFER_FORMAT_FP32
                } else {
                    VIP_BUFFER_FORMAT_FP16
                },
            )?;
            let divisor = if stereo { 1 } else { 4 };
            if dims.get(..2) != Some(&[width / divisor, height / divisor])
                || dims[2..].iter().any(|&d| d != 1)
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    format!("Unexpected depth head {dims:?}"),
                ));
            }
            result.output.resize(result.buffers[1].elements, 0.);
        } else {
            let boxes = result.bind(false, 0, VIP_BUFFER_FORMAT_INT8)?;
            let scores = result.bind(false, 1, VIP_BUFFER_FORMAT_INT8)?;
            let Some(&n) = boxes.first() else {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Missing boxes dimension",
                ));
            };
            if !(1..=10000).contains(&n)
                || boxes.get(1) != Some(&4)
                || boxes[2..].iter().any(|&d| d != 1)
                || scores.first() != Some(&n)
                || scores[1..].iter().any(|&d| d != 1)
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    format!("Unexpected NBG outputs {boxes:?}, {scores:?}"),
                ));
            }
            result.candidates = n;
            result.output.resize(5 * n, 0.0);
        }
        let input = &result.buffers[0];
        result.lookup = std::array::from_fn(|pixel| {
            let raw = ((pixel as f32 / 255.) / input.scale).round() + f32::from(input.zero);
            raw as i8 as u8
        });
        check("vip_prepare_network", unsafe {
            result.api.vip_prepare_network(result.network)
        })?;
        result.prepared = true;
        check("vip_set_input", unsafe {
            result
                .api
                .vip_set_input(result.network, 0, result.buffers[0].handle)
        })?;
        for (index, buffer) in result.buffers[1..].iter().enumerate() {
            check("vip_set_output", unsafe {
                result
                    .api
                    .vip_set_output(result.network, index as u32, buffer.handle)
            })?;
        }
        Ok(result)
    }

    fn bind(&mut self, input: bool, index: u32, format: u32) -> Result<Vec<usize>> {
        let query = if input {
            self.api.vip_query_input
        } else {
            self.api.vip_query_output
        };
        let read = |property, dst: *mut c_void| {
            check("vip_query_port", unsafe {
                query(self.network, index, property as i32, dst)
            })
        };
        // All fields are numeric C scalars. Query each property into the SDK's declared type.
        let mut p: vip_buffer_create_params_t = unsafe { std::mem::zeroed() };
        read(
            VIP_BUFFER_PROP_NUM_OF_DIMENSION,
            ptr::from_mut(&mut p.num_of_dims).cast(),
        )?;
        if !(1..=6).contains(&p.num_of_dims) {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "Invalid VIPLite tensor rank",
            ));
        }
        read(
            VIP_BUFFER_PROP_SIZES_OF_DIMENSION,
            p.sizes.as_mut_ptr().cast(),
        )?;
        read(
            VIP_BUFFER_PROP_DATA_FORMAT,
            ptr::from_mut(&mut p.data_format).cast(),
        )?;
        read(
            VIP_BUFFER_PROP_QUANT_FORMAT,
            ptr::from_mut(&mut p.quant_format).cast(),
        )?;
        let (scale, zero) = if format != VIP_BUFFER_FORMAT_INT8 {
            if p.data_format as u32 != format || p.quant_format as u32 != VIP_BUFFER_QUANTIZE_NONE {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    format!("Expected unquantized VIPLite format {format}"),
                ));
            }
            (1., 0)
        } else {
            if p.data_format as u32 != VIP_BUFFER_FORMAT_INT8
                || p.quant_format as u32 != VIP_BUFFER_QUANTIZE_TF_ASYMM
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    format!(
                        "Expected affine INT8 port, got format {}, quantization {}",
                        p.data_format, p.quant_format
                    ),
                ));
            }
            // quant_format selects the affine member of the SDK union.
            let affine = unsafe { &mut p.quant_data.affine };
            read(
                VIP_BUFFER_PROP_TF_SCALE,
                ptr::from_mut(&mut affine.scale).cast(),
            )?;
            read(
                VIP_BUFFER_PROP_TF_ZERO_POINT,
                ptr::from_mut(&mut affine.zeroPoint).cast(),
            )?;
            let scale = affine.scale;
            let zero = i8::try_from(affine.zeroPoint).map_err(|e| {
                Error::caused_by(ErrorKind::InvalidData, "invalid INT8 zero point", e)
            })?;
            if !scale.is_finite() || scale <= 0. {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Invalid tensor quantization scale",
                ));
            }
            (scale, zero)
        };
        let dims: Vec<_> = p.sizes[..p.num_of_dims as usize]
            .iter()
            .map(|&d| d as usize)
            .collect();
        let elements = dims
            .iter()
            .try_fold(1_usize, |n, d| n.checked_mul(*d))
            .ok_or_else(|| Error::new(ErrorKind::OutOfRange, "tensor size overflow"))?;
        let mut handle = ptr::null_mut();
        check("vip_create_buffer", unsafe {
            self.api.vip_create_buffer(&mut p, 0, &mut handle)
        })?;
        self.buffers.push(Buffer {
            handle,
            elements,
            scale,
            zero,
        });
        let available = unsafe { self.api.vip_get_buffer_size(handle) } as usize;
        let bytes_per_element = match format {
            VIP_BUFFER_FORMAT_FP32 => 4,
            VIP_BUFFER_FORMAT_FP16 => 2,
            _ => 1,
        };
        if elements == 0 || elements > available / bytes_per_element {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "Invalid VIPLite buffer size",
            ));
        }
        Ok(dims)
    }

    pub fn run(&mut self, gray: &[u8]) -> Result<&[f32]> {
        let input = &self.buffers[0];
        if gray.len().checked_mul(3) != Some(input.elements) {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "Camera dimensions differ from NBG",
            ));
        }
        unsafe {
            let mapped = self.api.vip_map_buffer(input.handle).cast::<u8>();
            if mapped.is_null() {
                return Err(Error::new(ErrorKind::Io, "Failed to map NPU input"));
            }
            let dst = slice::from_raw_parts_mut(mapped, input.elements);
            let (red, rest) = dst.split_at_mut(gray.len());
            let (green, blue) = rest.split_at_mut(gray.len());
            for (((r, g), b), &pixel) in red.iter_mut().zip(green).zip(blue).zip(gray) {
                let value = self.lookup[usize::from(pixel)];
                *r = value;
                *g = value;
                *b = value;
            }
            check("vip_unmap_input", self.api.vip_unmap_buffer(input.handle))?;
            check(
                "vip_flush_input",
                self.api
                    .vip_flush_buffer(input.handle, VIP_BUFFER_OPER_TYPE_FLUSH),
            )?;
            check("vip_run_network", self.api.vip_run_network(self.network))?;
        }
        if self.candidates == 0 {
            let buffer = &self.buffers[1];
            unsafe {
                check(
                    "vip_invalidate_output",
                    self.api
                        .vip_flush_buffer(buffer.handle, VIP_BUFFER_OPER_TYPE_INVALIDATE),
                )?;
                let mapped = self.api.vip_map_buffer(buffer.handle).cast::<half::f16>();
                if mapped.is_null() {
                    return Err(Error::new(ErrorKind::Io, "Failed to map depth output"));
                }
                half::slice::HalfFloatSliceExt::convert_to_f32_slice(
                    slice::from_raw_parts(mapped, buffer.elements),
                    &mut self.output,
                );
                check("vip_unmap_output", self.api.vip_unmap_buffer(buffer.handle))?;
            }
            return Ok(&self.output);
        }
        let (boxes, scores) = self.output.split_at_mut(4 * self.candidates);
        for (buffer, out) in self.buffers[1..].iter().zip([boxes, scores]) {
            unsafe {
                check(
                    "vip_invalidate_output",
                    self.api
                        .vip_flush_buffer(buffer.handle, VIP_BUFFER_OPER_TYPE_INVALIDATE),
                )?;
                let mapped = self.api.vip_map_buffer(buffer.handle);
                if mapped.is_null() {
                    return Err(Error::new(ErrorKind::Io, "Failed to map NPU output"));
                }
                for (value, &raw) in out
                    .iter_mut()
                    .zip(slice::from_raw_parts(mapped.cast::<i8>(), buffer.elements))
                {
                    *value = f32::from(i16::from(raw) - i16::from(buffer.zero)) * buffer.scale;
                }
                check("vip_unmap_output", self.api.vip_unmap_buffer(buffer.handle))?;
            }
        }
        Ok(&self.output)
    }
}

impl Drop for Network {
    fn drop(&mut self) {
        unsafe {
            if self.prepared {
                self.api.vip_finish_network(self.network);
            }
            if !self.network.is_null() {
                self.api.vip_destroy_network(self.network);
            }
            for buffer in &self.buffers {
                self.api.vip_destroy_buffer(buffer.handle);
            }
            self.api.vip_destroy();
        }
    }
}
