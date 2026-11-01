// SPDX-License-Identifier: BSD-3-Clause
use crossbeam_channel::{SendTimeoutError, Sender};
use firefly_error::{Error, ErrorKind, Result};
use realsense_sys as rs;
use std::{
    collections::HashMap,
    ffi::{CStr, c_void},
    mem::MaybeUninit,
    panic::{AssertUnwindSafe, catch_unwind},
    ptr,
    sync::{
        Arc, Mutex, OnceLock,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
};

#[derive(Debug)]
pub struct VideoFrame {
    pub timestamp_ms: f64,
    pub exposure_us: i64,
    pub width: usize,
    pub height: usize,
    pub pixels: Vec<u8>,
}
#[derive(Debug)]
pub enum Payload {
    Gyro {
        timestamp_ms: f64,
        xyz: [f32; 3],
    },
    Accel {
        timestamp_ms: f64,
        xyz: [f32; 3],
    },
    Stereo([VideoFrame; 2]),
    Pose {
        timestamp_ms: f64,
        translation: [f32; 3],
        rotation_xyzw: [f32; 4],
    },
}

/// Extra raw bridge buffering; an implementation choice, not a Basalt parameter.
pub const RAW_QUEUE_CAPACITY: usize = 8;
pub type PayloadReceiver = crossbeam_channel::Receiver<Result<Payload>>;

fn failure(message: impl Into<String>) -> Error {
    Error::new(ErrorKind::InvalidArgument, message.into()).with_operation("basalt.realsense")
}
/// SDK functions return errors through their final pointer argument. All call
/// sites below provide handles owned by this module and release them with the
/// matching SDK destructor. Error strings are copied before freeing the error.
fn call<T>(invoke: impl FnOnce(*mut *mut rs::rs2_error) -> T) -> Result<T> {
    let mut error = ptr::null_mut();
    let value = invoke(&mut error);
    if error.is_null() {
        Ok(value)
    } else {
        // SAFETY: the SDK populated a live error object; its message lives until free.
        let message = unsafe {
            CStr::from_ptr(rs::rs2_get_error_message(error))
                .to_string_lossy()
                .into_owned()
        };
        unsafe { rs::rs2_free_error(error) };
        Err(failure(message))
    }
}
macro_rules! handle {
    ($name:ident,$raw:ty,$delete:path) => {
        struct $name(*mut $raw);
        impl Drop for $name {
            fn drop(&mut self) {
                if !self.0.is_null() {
                    unsafe { $delete(self.0) }
                }
            }
        }
    };
}
handle!(Context, rs::rs2_context, rs::rs2_delete_context);
handle!(Config, rs::rs2_config, rs::rs2_delete_config);
handle!(Pipeline, rs::rs2_pipeline, rs::rs2_delete_pipeline);
handle!(
    Profile,
    rs::rs2_pipeline_profile,
    rs::rs2_delete_pipeline_profile
);
handle!(Frame, rs::rs2_frame, rs::rs2_release_frame);
handle!(Devices, rs::rs2_device_list, rs::rs2_delete_device_list);
handle!(
    Streams,
    rs::rs2_stream_profile_list,
    rs::rs2_delete_stream_profiles_list
);
handle!(SdkDevice, rs::rs2_device, rs::rs2_delete_device);
handle!(Sensors, rs::rs2_sensor_list, rs::rs2_delete_sensor_list);
handle!(Sensor, rs::rs2_sensor, rs::rs2_delete_sensor);

#[derive(Debug)]
pub struct MotionCalibration {
    pub data: [[f32; 4]; 3],
    pub noise_variances: [f32; 3],
    pub bias_variances: [f32; 3],
}
#[derive(Debug)]
pub struct CameraCalibration {
    pub width: i32,
    pub height: i32,
    pub fx: f32,
    pub fy: f32,
    pub ppx: f32,
    pub ppy: f32,
    pub coefficients: [f32; 5],
    pub rotation: [f32; 9],
    pub translation: [f32; 3],
}
#[derive(Debug)]
pub struct Calibration {
    pub gyro: MotionCalibration,
    pub accel: MotionCalibration,
    pub cameras: [CameraCalibration; 2],
}

struct Callback {
    sender: Mutex<Option<Sender<Result<Payload>>>>,
    closed: AtomicBool,
}
impl Callback {
    fn new(sender: Sender<Result<Payload>>) -> Self {
        Self {
            sender: Mutex::new(Some(sender)),
            closed: AtomicBool::new(false),
        }
    }
    fn send(&self, mut event: Result<Payload>) {
        let sender = self.sender.lock().ok().and_then(|s| s.clone());
        let Some(sender) = sender else { return };
        while !self.closed.load(Ordering::Acquire) {
            match sender.send_timeout(event, std::time::Duration::from_millis(1)) {
                Ok(()) | Err(SendTimeoutError::Disconnected(_)) => return,
                Err(SendTimeoutError::Timeout(value)) => event = value,
            }
        }
    }
    fn finish(&self) {
        self.closed.store(true, Ordering::Release);
        if let Ok(mut sender) = self.sender.lock() {
            sender.take();
        }
    }
}

fn registry() -> &'static Mutex<HashMap<usize, Arc<Callback>>> {
    static REGISTRY: OnceLock<Mutex<HashMap<usize, Arc<Callback>>>> = OnceLock::new();
    REGISTRY.get_or_init(|| Mutex::new(HashMap::new()))
}
fn register(state: Arc<Callback>) -> Result<usize> {
    static NEXT: AtomicUsize = AtomicUsize::new(1);
    let mut token = NEXT.load(Ordering::Relaxed);
    loop {
        let next = token
            .checked_add(1)
            .ok_or_else(|| failure("Callback token space exhausted"))?;
        match NEXT.compare_exchange_weak(token, next, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => break,
            Err(current) => token = current,
        }
    }
    registry()
        .lock()
        .map_err(|_| failure("Poisoned callback registry"))?
        .insert(token, state);
    Ok(token)
}
fn lookup(token: usize) -> Option<Arc<Callback>> {
    registry().lock().ok()?.get(&token).cloned()
}
fn unregister(token: usize) {
    if let Ok(mut registry) = registry().lock() {
        registry.remove(&token);
    }
}

unsafe extern "C" fn callback(frame: *mut rs::rs2_frame, user: *mut c_void) {
    let owned = Frame(frame);
    // user is an opaque, never-reused integer token, not a Rust pointer. The
    // SDK may invoke a late callback after an exceptional stop. A cloned Arc
    // keeps already-dispatched state alive, while missing tokens drop the frame.
    let Some(state) = lookup(user as usize) else {
        return;
    };
    dispatch(&state, || decode(&owned));
}

fn dispatch(state: &Callback, decode: impl FnOnce() -> Result<Option<Payload>>) {
    let result = catch_unwind(AssertUnwindSafe(decode));
    match result {
        Ok(Ok(Some(payload))) => state.send(Ok(payload)),
        Ok(Ok(None)) => {}
        Ok(Err(error)) => state.send(Err(error)),
        Err(_) => state.send(Err(failure("Panic contained in SDK frame callback"))),
    }
}

fn extension(frame: &Frame, kind: rs::rs2_extension) -> Result<bool> {
    call(|e| unsafe { rs::rs2_is_frame_extendable_to(frame.0, kind, e) }).map(|v| v != 0)
}
fn decode(frame: &Frame) -> Result<Option<Payload>> {
    let timestamp = call(|e| unsafe { rs::rs2_get_frame_timestamp(frame.0, e) })?;
    if extension(frame, rs::rs2_extension_RS2_EXTENSION_MOTION_FRAME)? {
        let profile = call(|e| unsafe { rs::rs2_get_frame_stream_profile(frame.0, e) })?;
        let (mut stream, mut format, mut index, mut uid, mut fps) = (0, 0, 0, 0, 0);
        call(|e| unsafe {
            rs::rs2_get_stream_profile_data(
                profile,
                &mut stream,
                &mut format,
                &mut index,
                &mut uid,
                &mut fps,
                e,
            )
        })?;
        if format != rs::rs2_format_RS2_FORMAT_MOTION_XYZ32F {
            return Ok(None);
        }
        let len = call(|e| unsafe { rs::rs2_get_frame_data_size(frame.0, e) })?;
        if len < 12 {
            return Err(failure("Truncated SDK motion payload"));
        }
        let data = call(|e| unsafe { rs::rs2_get_frame_data(frame.0, e) })?;
        if data.is_null() {
            return Err(failure("Null SDK motion payload"));
        }
        // Motion XYZ32F is three native f32 values, potentially unaligned.
        let xyz = unsafe { ptr::read_unaligned(data.cast::<[f32; 3]>()) };
        return Ok(match stream {
            rs::rs2_stream_RS2_STREAM_GYRO => Some(Payload::Gyro {
                timestamp_ms: timestamp,
                xyz,
            }),
            rs::rs2_stream_RS2_STREAM_ACCEL => Some(Payload::Accel {
                timestamp_ms: timestamp,
                xyz,
            }),
            _ => None,
        });
    }
    if extension(frame, rs::rs2_extension_RS2_EXTENSION_COMPOSITE_FRAME)? {
        let count = call(|e| unsafe { rs::rs2_embedded_frames_count(frame.0, e) })?;
        if count != 2 {
            return Err(failure("Expected two fisheye frames"));
        }
        let mut output = Vec::with_capacity(2);
        for i in 0..2 {
            let child = Frame(call(|e| unsafe { rs::rs2_extract_frame(frame.0, i, e) })?);
            if !extension(&child, rs::rs2_extension_RS2_EXTENSION_VIDEO_FRAME)? {
                return Ok(None);
            }
            let timestamp_ms = call(|e| unsafe { rs::rs2_get_frame_timestamp(child.0, e) })?;
            let width = usize::try_from(call(|e| unsafe { rs::rs2_get_frame_width(child.0, e) })?)
                .map_err(|_| failure("Negative width"))?;
            let height =
                usize::try_from(call(|e| unsafe { rs::rs2_get_frame_height(child.0, e) })?)
                    .map_err(|_| failure("Negative height"))?;
            let count = width
                .checked_mul(height)
                .ok_or_else(|| failure("Image dimensions overflow"))?;
            let len = call(|e| unsafe { rs::rs2_get_frame_data_size(child.0, e) })?;
            if usize::try_from(len).map_or(true, |len| len < count) {
                return Err(failure("Truncated SDK image"));
            }
            let data = call(|e| unsafe { rs::rs2_get_frame_data(child.0, e) })?;
            if data.is_null() {
                return Err(failure("Null SDK image"));
            }
            let pixels = unsafe { std::slice::from_raw_parts(data.cast::<u8>(), count) }.to_vec();
            let exposure_us = call(|e| unsafe {
                rs::rs2_get_frame_metadata(
                    child.0,
                    rs::rs2_frame_metadata_value_RS2_FRAME_METADATA_ACTUAL_EXPOSURE,
                    e,
                )
            })?;
            output.push(VideoFrame {
                timestamp_ms,
                exposure_us,
                width,
                height,
                pixels,
            });
        }
        return Ok(Some(Payload::Stereo(
            output
                .try_into()
                .map_err(|_| failure("Invalid stereo frame count"))?,
        )));
    }
    if extension(frame, rs::rs2_extension_RS2_EXTENSION_POSE_FRAME)? {
        let mut pose = MaybeUninit::uninit();
        call(|e| unsafe { rs::rs2_pose_frame_get_pose_data(frame.0, pose.as_mut_ptr(), e) })?;
        let p = unsafe { pose.assume_init() };
        return Ok(Some(Payload::Pose {
            timestamp_ms: timestamp,
            translation: [p.translation.x, p.translation.y, p.translation.z],
            rotation_xyzw: [p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w],
        }));
    }
    Ok(None)
}

/// Owns the SDK pipeline on its creating thread. Its callback transfers owned
/// payloads through a bounded channel with cancellation-aware sends.
pub struct Device {
    pipeline: Option<Pipeline>,
    profile: Option<Profile>,
    callback: Option<Arc<Callback>>,
    callback_token: Option<usize>,
    config: Config,
    context: Context,
    running: bool,
}
impl Device {
    /// Resets the first enumerated device, matching the upstream constructor.
    /// The caller must allow SDK re-enumeration before starting the pipeline.
    pub fn reset_first_device(&self) -> Result<()> {
        if self.running {
            return Err(failure("Cannot reset a running pipeline"));
        }
        let devices = Devices(call(|e| unsafe {
            rs::rs2_query_devices(self.context.0, e)
        })?);
        let device = SdkDevice(call(|e| unsafe { rs::rs2_create_device(devices.0, 0, e) })?);
        call(|e| unsafe { rs::rs2_hardware_reset(device.0, e) })
    }
    pub fn calibration(&self) -> Result<Calibration> {
        let profile = self
            .profile
            .as_ref()
            .ok_or_else(|| failure("Pipeline is not running"))?;
        let streams = Streams(call(|e| unsafe {
            rs::rs2_pipeline_profile_get_streams(profile.0, e)
        })?);
        let count = call(|e| unsafe { rs::rs2_get_stream_profiles_count(streams.0, e) })?;
        let mut gyro = ptr::null();
        let mut accel = ptr::null();
        let mut cameras = [ptr::null(); 2];
        for i in 0..count {
            let stream = call(|e| unsafe { rs::rs2_get_stream_profile(streams.0, i, e) })?;
            let (mut kind, mut format, mut index, mut uid, mut fps) = (0, 0, 0, 0, 0);
            call(|e| unsafe {
                rs::rs2_get_stream_profile_data(
                    stream,
                    &mut kind,
                    &mut format,
                    &mut index,
                    &mut uid,
                    &mut fps,
                    e,
                )
            })?;
            match kind {
                rs::rs2_stream_RS2_STREAM_GYRO => gyro = stream,
                rs::rs2_stream_RS2_STREAM_ACCEL => accel = stream,
                rs::rs2_stream_RS2_STREAM_FISHEYE if (1..=2).contains(&index) => {
                    cameras[(index - 1) as usize] = stream
                }
                _ => {}
            }
        }
        if gyro.is_null() || accel.is_null() || cameras.iter().any(|p| p.is_null()) {
            return Err(failure("Missing T265 calibration stream"));
        }
        let motion = |stream| -> Result<MotionCalibration> {
            let mut value = MaybeUninit::uninit();
            call(|e| unsafe { rs::rs2_get_motion_intrinsics(stream, value.as_mut_ptr(), e) })?;
            let v = unsafe { value.assume_init() };
            Ok(MotionCalibration {
                data: v.data,
                noise_variances: v.noise_variances,
                bias_variances: v.bias_variances,
            })
        };
        let mut output = Vec::with_capacity(2);
        for camera in cameras {
            let mut intr = MaybeUninit::uninit();
            let mut ex = MaybeUninit::uninit();
            call(|e| unsafe { rs::rs2_get_video_stream_intrinsics(camera, intr.as_mut_ptr(), e) })?;
            call(|e| unsafe { rs::rs2_get_extrinsics(camera, gyro, ex.as_mut_ptr(), e) })?;
            let intr = unsafe { intr.assume_init() };
            let ex = unsafe { ex.assume_init() };
            output.push(CameraCalibration {
                width: intr.width,
                height: intr.height,
                fx: intr.fx,
                fy: intr.fy,
                ppx: intr.ppx,
                ppy: intr.ppy,
                coefficients: intr.coeffs,
                rotation: ex.rotation,
                translation: ex.translation,
            });
        }
        Ok(Calibration {
            gyro: motion(gyro)?,
            accel: motion(accel)?,
            cameras: output
                .try_into()
                .map_err(|_| failure("Expected two cameras"))?,
        })
    }
    /// Changes the same first sensor used by the upstream device controller.
    /// `None` enables automatic exposure; milliseconds are converted to microseconds.
    pub fn set_exposure(&self, milliseconds: Option<f64>) -> Result<()> {
        let device = if let Some(profile) = &self.profile {
            SdkDevice(call(|e| unsafe {
                rs::rs2_pipeline_profile_get_device(profile.0, e)
            })?)
        } else {
            let devices = Devices(call(|e| unsafe {
                rs::rs2_query_devices(self.context.0, e)
            })?);
            SdkDevice(call(|e| unsafe { rs::rs2_create_device(devices.0, 0, e) })?)
        };
        let sensors = Sensors(call(|e| unsafe { rs::rs2_query_sensors(device.0, e) })?);
        let sensor = Sensor(call(|e| unsafe { rs::rs2_create_sensor(sensors.0, 0, e) })?);
        call(|e| unsafe {
            rs::rs2_set_option(
                sensor.0.cast(),
                rs::rs2_option_RS2_OPTION_ENABLE_AUTO_EXPOSURE,
                if milliseconds.is_some() { 0. } else { 1. },
                e,
            )
        })?;
        if let Some(value) = milliseconds {
            call(|e| unsafe {
                rs::rs2_set_option(
                    sensor.0.cast(),
                    rs::rs2_option_RS2_OPTION_EXPOSURE,
                    (value * 1000.) as f32,
                    e,
                )
            })?;
        }
        Ok(())
    }
    pub fn new(manual_exposure: bool) -> Result<Self> {
        let version = call(|e| unsafe { rs::rs2_get_api_version(e) })?;
        if version / 100 != 251 {
            return Err(failure(format!(
                "Expected SDK 2.51.x for T265, found API {version}"
            )));
        }
        let context = Context(call(|e| unsafe { rs::rs2_create_context(version, e) })?);
        let config = Config(call(|e| unsafe { rs::rs2_create_config(e) })?);
        for (stream, index, format) in [
            (
                rs::rs2_stream_RS2_STREAM_ACCEL,
                -1,
                rs::rs2_format_RS2_FORMAT_MOTION_XYZ32F,
            ),
            (
                rs::rs2_stream_RS2_STREAM_GYRO,
                -1,
                rs::rs2_format_RS2_FORMAT_MOTION_XYZ32F,
            ),
            (
                rs::rs2_stream_RS2_STREAM_FISHEYE,
                1,
                rs::rs2_format_RS2_FORMAT_Y8,
            ),
            (
                rs::rs2_stream_RS2_STREAM_FISHEYE,
                2,
                rs::rs2_format_RS2_FORMAT_Y8,
            ),
        ] {
            call(|e| unsafe {
                rs::rs2_config_enable_stream(config.0, stream, index, 0, 0, format, 0, e)
            })?;
        }
        if !manual_exposure {
            call(|e| unsafe {
                rs::rs2_config_enable_stream(
                    config.0,
                    rs::rs2_stream_RS2_STREAM_POSE,
                    -1,
                    0,
                    0,
                    rs::rs2_format_RS2_FORMAT_6DOF,
                    0,
                    e,
                )
            })?;
        }
        Ok(Self {
            pipeline: None,
            profile: None,
            callback: None,
            callback_token: None,
            config,
            context,
            running: false,
        })
    }
    pub fn device_count(&self) -> Result<usize> {
        let devices = Devices(call(|e| unsafe {
            rs::rs2_query_devices(self.context.0, e)
        })?);
        usize::try_from(call(|e| unsafe { rs::rs2_get_device_count(devices.0, e) })?)
            .map_err(|_| failure("Negative device count"))
    }
    pub fn start(&mut self) -> Result<PayloadReceiver> {
        if self.running {
            return Err(failure("Pipeline is already running"));
        }
        self.pipeline = Some(Pipeline(call(|e| unsafe {
            rs::rs2_create_pipeline(self.context.0, e)
        })?));
        let (sender, receiver) = crossbeam_channel::bounded(RAW_QUEUE_CAPACITY);
        let state = Arc::new(Callback::new(sender));
        let token = register(Arc::clone(&state))?;
        self.callback = Some(state);
        self.callback_token = Some(token);
        let pipe = self.pipeline.as_ref().expect("pipeline allocated").0;
        let result = call(|e| unsafe {
            rs::rs2_pipeline_start_with_config_and_callback(
                pipe,
                self.config.0,
                Some(callback),
                token as *mut c_void,
                e,
            )
        });
        match result {
            Ok(profile) => {
                self.profile = Some(Profile(profile));
                self.running = true;
                Ok(receiver)
            }
            Err(error) => {
                if let Some(state) = &self.callback {
                    state.finish();
                }
                if let Some(token) = self.callback_token.take() {
                    unregister(token);
                }
                self.pipeline.take();
                self.callback.take();
                Err(error)
            }
        }
    }
    pub fn stop(&mut self) -> Result<()> {
        if !self.running {
            return Ok(());
        }
        self.running = false;
        // Cancel full-queue waits before SDK stop. The receiver drains all
        // accepted payloads, then observes disconnection once senders disappear.
        if let Some(state) = &self.callback {
            state.finish();
        }
        if let Some(token) = self.callback_token.take() {
            unregister(token);
        }
        let result = call(|e| unsafe {
            rs::rs2_pipeline_stop(self.pipeline.as_ref().expect("running pipeline").0, e)
        });
        self.pipeline.take();
        self.profile.take();
        self.callback.take();
        result
    }
}
impl Drop for Device {
    fn drop(&mut self) {
        let _ = self.stop();
        self.pipeline.take();
        if let Some(token) = self.callback_token.take() {
            unregister(token);
        }
        self.callback.take();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn consumed_abi_matches_sdk_251_headers() {
        use std::mem::{align_of, offset_of, size_of};
        assert_eq!(
            (
                size_of::<rs::rs2_intrinsics>(),
                align_of::<rs::rs2_intrinsics>()
            ),
            (48, 4)
        );
        assert_eq!(
            (
                size_of::<rs::rs2_motion_device_intrinsic>(),
                align_of::<rs::rs2_motion_device_intrinsic>()
            ),
            (72, 4)
        );
        assert_eq!(
            (
                size_of::<rs::rs2_extrinsics>(),
                align_of::<rs::rs2_extrinsics>()
            ),
            (48, 4)
        );
        assert_eq!(
            (size_of::<rs::rs2_pose>(), align_of::<rs::rs2_pose>()),
            (84, 4)
        );
        assert_eq!(offset_of!(rs::rs2_pose, rotation), 36);
        assert_eq!(offset_of!(rs::rs2_pose, tracker_confidence), 76);
        assert_eq!(
            offset_of!(rs::rs2_motion_device_intrinsic, noise_variances),
            48
        );
        assert_eq!(
            offset_of!(rs::rs2_motion_device_intrinsic, bias_variances),
            60
        );
        assert_eq!(offset_of!(rs::rs2_intrinsics, coeffs), 28);
        assert_eq!(offset_of!(rs::rs2_extrinsics, translation), 36);
        assert_eq!(
            [
                rs::rs2_extension_RS2_EXTENSION_VIDEO_FRAME,
                rs::rs2_extension_RS2_EXTENSION_MOTION_FRAME,
                rs::rs2_extension_RS2_EXTENSION_COMPOSITE_FRAME,
                rs::rs2_extension_RS2_EXTENSION_POSE_FRAME
            ],
            [8, 9, 10, 20]
        );
        assert_eq!(
            [
                rs::rs2_stream_RS2_STREAM_FISHEYE,
                rs::rs2_stream_RS2_STREAM_GYRO,
                rs::rs2_stream_RS2_STREAM_ACCEL,
                rs::rs2_stream_RS2_STREAM_POSE
            ],
            [4, 5, 6, 8]
        );
        assert_eq!(
            [
                rs::rs2_format_RS2_FORMAT_Y8,
                rs::rs2_format_RS2_FORMAT_MOTION_XYZ32F,
                rs::rs2_format_RS2_FORMAT_6DOF
            ],
            [9, 16, 18]
        );
        assert_eq!(
            [
                rs::rs2_frame_metadata_value_RS2_FRAME_METADATA_ACTUAL_EXPOSURE,
                rs::rs2_option_RS2_OPTION_EXPOSURE,
                rs::rs2_option_RS2_OPTION_ENABLE_AUTO_EXPOSURE
            ],
            [3, 3, 10]
        );
    }
    #[test]
    fn unregister_and_inflight_callbacks_disconnect_without_sending() {
        let (sender, receiver) = crossbeam_channel::bounded(1);
        let state = Arc::new(Callback::new(sender));
        let token = register(Arc::clone(&state)).unwrap();
        let inflight = lookup(token).unwrap();
        let barrier = Arc::new(std::sync::Barrier::new(2));
        let worker_barrier = Arc::clone(&barrier);
        let worker = std::thread::spawn(move || {
            worker_barrier.wait();
            dispatch(&inflight, || {
                Ok(Some(Payload::Gyro {
                    timestamp_ms: 1.,
                    xyz: [0.; 3],
                }))
            });
        });
        unregister(token);
        state.finish();
        state.finish();
        assert!(lookup(token).is_none());
        barrier.wait();
        worker.join().unwrap();
        assert!(matches!(
            receiver.try_recv(),
            Err(crossbeam_channel::TryRecvError::Disconnected)
        ));
        // A missing token never dereferences user or decodes a frame. Null is a
        // sentinel for this regression, and the RAII wrapper skips its release.
        unsafe { callback(ptr::null_mut(), token as *mut c_void) };
        let next = register(state).unwrap();
        assert!(next > token);
        unregister(next);
    }
    #[test]
    fn payload_dispatch_and_panic_boundary() {
        let (sender, receiver) = crossbeam_channel::bounded(1);
        let state = Callback::new(sender);
        dispatch(&state, || {
            Ok(Some(Payload::Gyro {
                timestamp_ms: 1.25,
                xyz: [1., 2., 3.],
            }))
        });
        match receiver.recv().unwrap().unwrap() {
            Payload::Gyro { timestamp_ms, xyz } => {
                assert_eq!(timestamp_ms, 1.25);
                assert_eq!(xyz, [1., 2., 3.]);
            }
            other => panic!("Unexpected {other:?}"),
        }
        dispatch(&state, || Ok(None));
        assert!(receiver.try_recv().is_err());
        dispatch(&state, || Err(failure("injected SDK error")));
        assert!(receiver.recv().unwrap().is_err());
        dispatch(&state, || panic!("injected callback panic"));
        assert!(receiver.recv().unwrap().is_err());
        drop(receiver);
        dispatch(&state, || {
            Ok(Some(Payload::Gyro {
                timestamp_ms: 0.,
                xyz: [0.; 3],
            }))
        });
    }
    #[test]
    fn full_queue_cancel_disconnects_after_drain() {
        let (sender, receiver) = crossbeam_channel::bounded(1);
        let state = Arc::new(Callback::new(sender));
        state.send(Ok(Payload::Gyro {
            timestamp_ms: 1.,
            xyz: [0.; 3],
        }));
        let inflight = Arc::clone(&state);
        let (done_tx, done_rx) = std::sync::mpsc::channel();
        let barrier = Arc::new(std::sync::Barrier::new(2));
        let worker_barrier = Arc::clone(&barrier);
        let worker = std::thread::spawn(move || {
            worker_barrier.wait();
            inflight.send(Ok(Payload::Gyro {
                timestamp_ms: 2.,
                xyz: [0.; 3],
            }));
            done_tx.send(()).unwrap();
        });
        barrier.wait();
        assert!(
            done_rx
                .recv_timeout(std::time::Duration::from_millis(10))
                .is_err()
        );
        state.finish();
        state.finish();
        done_rx
            .recv_timeout(std::time::Duration::from_secs(1))
            .unwrap();
        worker.join().unwrap();
        assert!(matches!(
            receiver.recv().unwrap().unwrap(),
            Payload::Gyro {
                timestamp_ms: 1.,
                ..
            }
        ));
        assert!(
            receiver
                .recv_timeout(std::time::Duration::from_secs(1))
                .is_err()
        );
    }
}
