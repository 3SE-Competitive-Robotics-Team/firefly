//! A733 ZipDepth inference with rectified stereo anchor correction.
#![cfg(all(target_os = "linux", target_arch = "aarch64"))]
use firefly_error::{Error, ErrorKind, Result};
use std::ffi::{CStr, c_char, c_int, c_void};
use std::ptr::NonNull;

const PIXELS: usize = 320 * 240;
unsafe extern "C" {
    fn depth_fusion_create(handle: *mut *mut c_void, error: *mut c_char, capacity: usize) -> c_int;
    fn depth_fusion_destroy(handle: *mut c_void);
    fn depth_fusion_match(
        handle: *mut c_void,
        left: *const u8,
        right: *const u8,
        error: *mut c_char,
        capacity: usize,
    ) -> c_int;
    fn depth_fusion_restore(
        handle: *mut c_void,
        left: *const u8,
        relative: *const f32,
        metres: *mut f32,
        error: *mut c_char,
        capacity: usize,
    ) -> c_int;
}

fn call(op: impl FnOnce(*mut c_char, usize) -> c_int) -> Result<()> {
    let mut error = [0; 256];
    if op(error.as_mut_ptr(), error.len()) != 0 {
        // SAFETY: the bridge writes a terminated snprintf message on every error.
        let message = unsafe { CStr::from_ptr(error.as_ptr()) }.to_string_lossy();
        return Err(Error::new(
            ErrorKind::Internal,
            format!("stereo depth: {message}"),
        ));
    }
    Ok(())
}
struct Fusion(NonNull<c_void>);
impl Drop for Fusion {
    fn drop(&mut self) {
        // SAFETY: this is the unique owner of the context returned by create.
        unsafe { depth_fusion_destroy(self.0.as_ptr()) };
    }
}
pub struct Stereo {
    network: firefly_viplite::StereoNetwork,
    fusion: Fusion,
    metres: Vec<f32>,
}
impl Stereo {
    pub fn open(model: Vec<u8>) -> Result<Self> {
        let network = firefly_viplite::StereoNetwork::open(model, 320, 240)?;
        let mut ptr = std::ptr::null_mut();
        // SAFETY: create initializes ptr on success and retains neither borrowed pointer.
        call(|error, len| unsafe { depth_fusion_create(&raw mut ptr, error, len) })?;
        let fusion = Fusion(
            NonNull::new(ptr)
                .ok_or_else(|| Error::new(ErrorKind::Internal, "Missing stereo context"))?,
        );
        Ok(Self {
            network,
            fusion,
            metres: vec![0.; PIXELS],
        })
    }
    pub fn run(&mut self, left: &[u8; PIXELS], right: &[u8; PIXELS]) -> Result<&[f32]> {
        let ctx = self.fusion.0.as_ptr();
        let relative = self.network.run(left, right, || {
            // SAFETY: both camera planes live through this synchronous matching call.
            call(|error, len| unsafe {
                depth_fusion_match(ctx, left.as_ptr(), right.as_ptr(), error, len)
            })
        })?;
        // SAFETY: the validated NBG output and both image arrays contain PIXELS elements.
        call(|error, len| unsafe {
            depth_fusion_restore(
                ctx,
                left.as_ptr(),
                relative.as_ptr(),
                self.metres.as_mut_ptr(),
                error,
                len,
            )
        })?;
        Ok(&self.metres)
    }
}
