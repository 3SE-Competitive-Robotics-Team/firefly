//! Synchronous VIPLite inference for A733 vision models.

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
use std::{
    ffi::c_void,
    ptr, slice,
    sync::{Mutex, MutexGuard},
};

// VIPLite initialization and shutdown have process scope.
static DEVICE: Mutex<()> = Mutex::new(());

fn check(operation: &str, status: vip_status_e) -> Result<(), String> {
    if status == VIP_SUCCESS {
        Ok(())
    } else {
        Err(format!("{operation}: VIPLite status {status}"))
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
    plane: Vec<u8>,
    lookup: [u8; 256],
    output: Vec<f32>,
    // The SDK may retain the NBG address until vip_destroy_network.
    model: Vec<u8>,
    _device: MutexGuard<'static, ()>,
}

impl Network {
    pub fn open(model: Vec<u8>, width: usize, height: usize) -> Result<Self, String> {
        Self::create(model, width, height, false)
    }

    /// Compact depth networks expose a single FP16 metric head at quarter resolution.
    pub fn open_depth(model: Vec<u8>, side: usize) -> Result<Self, String> {
        Self::create(model, side, side, true)
    }

    fn create(model: Vec<u8>, width: usize, height: usize, depth: bool) -> Result<Self, String> {
        let size = u32::try_from(model.len()).map_err(|_| "NBG exceeds the SDK size limit")?;
        let device = DEVICE.try_lock().map_err(|_| "VIPLite is already in use")?;
        // Load the BSP-provided C ABI. Each handle below is owned by this single-threaded session.
        let api = unsafe { Vip::new("libNBGlinker.so") }.map_err(|e| e.to_string())?;
        check("vip_init", unsafe { api.vip_init() })?;
        let mut result = Self {
            api,
            network: ptr::null_mut(),
            prepared: false,
            buffers: Vec::new(),
            candidates: 0,
            plane: vec![0; width * height],
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
            (VIP_NETWORK_PROP_OUTPUT_COUNT, if depth { 1 } else { 2 }),
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
                return Err(format!(
                    "Unexpected NBG port count {count}, expected {expected}"
                ));
            }
        }
        let input = result.bind(true, 0, false)?;
        if input != [width, height, 3, 1] {
            return Err(format!("Unexpected NBG input {input:?}"));
        }
        if depth {
            let dims = result.bind(false, 0, true)?;
            if dims.get(..2) != Some(&[width / 4, height / 4]) || dims[2..].iter().any(|&d| d != 1)
            {
                return Err(format!("Unexpected depth head {dims:?}"));
            }
            result.output.resize(result.buffers[1].elements, 0.);
        } else {
            let boxes = result.bind(false, 0, false)?;
            let scores = result.bind(false, 1, false)?;
            let Some(&n) = boxes.first() else {
                return Err("Missing boxes dimension".into());
            };
            if !(1..=10000).contains(&n)
                || boxes.get(1) != Some(&4)
                || boxes[2..].iter().any(|&d| d != 1)
                || scores.first() != Some(&n)
                || scores[1..].iter().any(|&d| d != 1)
            {
                return Err(format!("Unexpected NBG outputs {boxes:?}, {scores:?}"));
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

    fn bind(&mut self, input: bool, index: u32, fp16: bool) -> Result<Vec<usize>, String> {
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
            return Err("Invalid VIPLite tensor rank".into());
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
        let (scale, zero) = if fp16 {
            if p.data_format as u32 != VIP_BUFFER_FORMAT_FP16
                || p.quant_format as u32 != VIP_BUFFER_QUANTIZE_NONE
            {
                return Err("Expected unquantized FP16 output".into());
            }
            (1., 0)
        } else {
            if p.data_format as u32 != VIP_BUFFER_FORMAT_INT8
                || p.quant_format as u32 != VIP_BUFFER_QUANTIZE_TF_ASYMM
            {
                return Err(format!(
                    "Expected affine INT8 port, got format {}, quantization {}",
                    p.data_format, p.quant_format
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
            let zero = i8::try_from(affine.zeroPoint).map_err(|_| "Invalid INT8 zero point")?;
            if !scale.is_finite() || scale <= 0. {
                return Err("Invalid tensor quantization scale".into());
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
            .ok_or("Tensor size overflow")?;
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
        if elements == 0 || elements > available / if fp16 { 2 } else { 1 } {
            return Err("Invalid VIPLite buffer size".into());
        }
        Ok(dims)
    }

    pub fn run(&mut self, gray: &[u8]) -> Result<&[f32], String> {
        let input = &self.buffers[0];
        if gray.len().checked_mul(3) != Some(input.elements) {
            return Err("Camera dimensions differ from NBG".into());
        }
        unsafe {
            let mapped = self.api.vip_map_buffer(input.handle).cast::<u8>();
            if mapped.is_null() {
                return Err("Failed to map NPU input".into());
            }
            let dst = slice::from_raw_parts_mut(mapped, input.elements);
            for (out, pixel) in self.plane.iter_mut().zip(gray) {
                *out = self.lookup[*pixel as usize];
            }
            for channel in dst.chunks_exact_mut(gray.len()) {
                channel.copy_from_slice(&self.plane);
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
                    return Err("Failed to map depth output".into());
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
                    return Err("Failed to map NPU output".into());
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
