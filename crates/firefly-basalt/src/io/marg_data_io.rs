// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// See LICENSE-BSD-3-Clause.
//! Native Cereal marginalization archives and asynchronous directory I/O.
//! Translation of official `io/marg_data_io.h/.cpp` at Basalt `0f3b2b52`.
use super::dataset_io::ImageData;
use crate::{
    optical_flow::{AffineCompact2, OpticalFlowInput, OpticalFlowResult},
    utils::{
        common_types::CerealBinary,
        imu_types::{AbsOrderMap, MargData},
    },
};
use firefly_basalt_headers::image::image::ManagedImage;
use firefly_error::{Error, ErrorKind, Result};
use nalgebra::{DMatrix, DVector};
use std::{
    any::Any,
    collections::{BTreeMap, BTreeSet, HashMap, HashSet},
    fs::{self, File},
    io::{BufReader, BufWriter, Read, Write},
    path::{Path, PathBuf},
    sync::{
        Arc,
        mpsc::{self, Receiver, SyncSender},
    },
    thread::{self, JoinHandle},
};

pub(crate) mod binary {
    use super::{Error, ErrorKind, Read, Result, Write};
    use nalgebra::{Isometry3, Quaternion, SVector, Translation3, UnitQuaternion};
    pub fn error(source: std::io::Error) -> Error {
        Error::new(
            ErrorKind::InvalidArgument,
            "Cannot process marginalization archive",
        )
        .with_operation("basalt.marg_data_io")
        .with_source(source)
    }
    pub fn invalid(message: &str) -> Error {
        error(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            message,
        ))
    }
    pub fn bytes<const N: usize>(r: &mut impl Read) -> Result<[u8; N]> {
        let mut out = [0; N];
        r.read_exact(&mut out).map_err(error)?;
        Ok(out)
    }
    pub fn i64(r: &mut impl Read) -> Result<i64> {
        Ok(i64::from_ne_bytes(bytes(r)?))
    }
    pub fn count(r: &mut impl Read) -> Result<usize> {
        usize::try_from(u64::from_ne_bytes(bytes(r)?)).map_err(|_| invalid("Archive size overflow"))
    }
    pub fn native_size(r: &mut impl Read) -> Result<usize> {
        Ok(usize::from_ne_bytes(bytes(r)?))
    }
    pub fn write_native_size(w: &mut impl Write, value: usize) -> Result<()> {
        w.write_all(&value.to_ne_bytes()).map_err(error)
    }
    pub fn scalar(r: &mut impl Read) -> Result<f64> {
        Ok(f64::from_ne_bytes(bytes(r)?))
    }
    pub fn boolean(r: &mut impl Read) -> Result<bool> {
        match bytes::<1>(r)?[0] {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(invalid("Invalid boolean")),
        }
    }
    pub fn write_i64(w: &mut impl Write, v: i64) -> Result<()> {
        w.write_all(&v.to_ne_bytes()).map_err(error)
    }
    pub fn write_count(w: &mut impl Write, v: usize) -> Result<()> {
        w.write_all(&(v as u64).to_ne_bytes()).map_err(error)
    }
    pub fn write_scalar(w: &mut impl Write, v: f64) -> Result<()> {
        w.write_all(&v.to_ne_bytes()).map_err(error)
    }
    pub fn write_bool(w: &mut impl Write, v: bool) -> Result<()> {
        w.write_all(&[u8::from(v)]).map_err(error)
    }
    pub fn vector<const N: usize>(r: &mut impl Read) -> Result<SVector<f64, N>> {
        let mut v = SVector::<f64, N>::zeros();
        for x in v.iter_mut() {
            *x = scalar(r)?;
        }
        Ok(v)
    }
    pub fn write_vector<const N: usize>(w: &mut impl Write, v: &SVector<f64, N>) -> Result<()> {
        for x in v {
            write_scalar(w, *x)?;
        }
        Ok(())
    }
    pub fn pose(r: &mut impl Read) -> Result<Isometry3<f64>> {
        let translation = vector::<3>(r)?;
        let q = vector::<4>(r)?;
        Ok(Isometry3::from_parts(
            Translation3::from(translation),
            UnitQuaternion::new_unchecked(Quaternion::new(q[3], q[0], q[1], q[2])),
        ))
    }
    pub fn write_pose(w: &mut impl Write, p: &Isometry3<f64>) -> Result<()> {
        write_vector(w, &p.translation.vector)?;
        write_vector(w, &p.rotation.quaternion().coords)
    }
}
use binary as b;

impl CerealBinary for AbsOrderMap {
    fn read_cereal_binary(r: &mut impl Read) -> Result<Self> {
        let total_size = b::native_size(r)?;
        let items = b::native_size(r)?;
        let size = b::count(r)?;
        let mut abs_order_map = BTreeMap::new();
        for _ in 0..size {
            let key = b::i64(r)?;
            let offset = i32::from_ne_bytes(b::bytes(r)?);
            let count = i32::from_ne_bytes(b::bytes(r)?);
            abs_order_map.insert(key, (offset, count));
        }
        Ok(Self {
            abs_order_map,
            items,
            total_size,
        })
    }
    fn write_cereal_binary(&self, w: &mut impl Write) -> Result<()> {
        b::write_native_size(w, self.total_size)?;
        b::write_native_size(w, self.items)?;
        b::write_count(w, self.abs_order_map.len())?;
        for (key, (offset, count)) in &self.abs_order_map {
            b::write_i64(w, *key)?;
            w.write_all(&offset.to_ne_bytes()).map_err(b::error)?;
            w.write_all(&count.to_ne_bytes()).map_err(b::error)?;
        }
        Ok(())
    }
}
fn read_map<T: CerealBinary>(r: &mut impl Read) -> Result<BTreeMap<i64, T>> {
    let count = b::count(r)?;
    let mut map = BTreeMap::new();
    for _ in 0..count {
        map.insert(b::i64(r)?, T::read_cereal_binary(r)?);
    }
    Ok(map)
}
fn write_map<T: CerealBinary>(w: &mut impl Write, map: &BTreeMap<i64, T>) -> Result<()> {
    b::write_count(w, map.len())?;
    for (key, v) in map {
        b::write_i64(w, *key)?;
        v.write_cereal_binary(w)?;
    }
    Ok(())
}
fn read_set(r: &mut impl Read) -> Result<BTreeSet<i64>> {
    let n = b::count(r)?;
    (0..n).map(|_| b::i64(r)).collect()
}
fn write_set(w: &mut impl Write, set: &BTreeSet<i64>) -> Result<()> {
    b::write_count(w, set.len())?;
    for key in set {
        b::write_i64(w, *key)?;
    }
    Ok(())
}
impl CerealBinary for MargData {
    fn read_cereal_binary(r: &mut impl Read) -> Result<Self> {
        let aom = AbsOrderMap::read_cereal_binary(r)?;
        let rows = b::count(r)?;
        let cols = b::count(r)?;
        rows.checked_mul(cols)
            .ok_or_else(|| b::invalid("Matrix dimensions overflow"))?;
        let mut abs_h = DMatrix::zeros(rows, cols);
        for i in 0..rows {
            for j in 0..cols {
                abs_h[(i, j)] = b::scalar(r)?;
            }
        }
        let size = b::count(r)?;
        let mut abs_b = DVector::zeros(size);
        for value in abs_b.iter_mut() {
            *value = b::scalar(r)?;
        }
        Ok(Self {
            aom,
            abs_h,
            abs_b,
            frame_poses: read_map(r)?,
            frame_states: read_map(r)?,
            kfs_all: read_set(r)?,
            kfs_to_marg: read_set(r)?,
            use_imu: b::boolean(r)?,
            opt_flow_res: Vec::new(),
        })
    }
    fn write_cereal_binary(&self, w: &mut impl Write) -> Result<()> {
        self.aom.write_cereal_binary(w)?;
        b::write_count(w, self.abs_h.nrows())?;
        b::write_count(w, self.abs_h.ncols())?;
        for i in 0..self.abs_h.nrows() {
            for j in 0..self.abs_h.ncols() {
                b::write_scalar(w, self.abs_h[(i, j)])?;
            }
        }
        b::write_count(w, self.abs_b.len())?;
        for value in &self.abs_b {
            b::write_scalar(w, *value)?;
        }
        write_map(w, &self.frame_poses)?;
        write_map(w, &self.frame_states)?;
        write_set(w, &self.kfs_all)?;
        write_set(w, &self.kfs_to_marg)?;
        b::write_bool(w, self.use_imu)
    }
}

/// Archive-local shared-pointer identities, including shared camera images.
#[derive(Default)]
pub struct FlowArchive {
    loaded: HashMap<u32, Arc<dyn Any + Send + Sync>>,
    saved: HashMap<usize, (u32, Arc<dyn Any + Send + Sync>)>,
}
impl FlowArchive {
    fn read_ptr<T: Any + Send + Sync>(
        &mut self,
        r: &mut impl Read,
        read: impl FnOnce(&mut Self, &mut dyn Read) -> Result<T>,
    ) -> Result<Option<Arc<T>>> {
        let id = u32::from_ne_bytes(b::bytes(r)?);
        if id == 0 {
            return Ok(None);
        }
        if id & 0x8000_0000 != 0 {
            let key = id & 0x7fff_ffff;
            if self.loaded.contains_key(&key) {
                return Err(b::invalid("Duplicate shared pointer ID"));
            }
            let value = Arc::new(read(self, r)?);
            self.loaded.insert(key, value.clone());
            Ok(Some(value))
        } else {
            self.loaded
                .get(&id)
                .cloned()
                .ok_or_else(|| b::invalid("Unknown shared pointer ID"))?
                .downcast::<T>()
                .map(Some)
                .map_err(|_| b::invalid("Shared pointer type mismatch"))
        }
    }
    fn write_ptr<T: Any + Send + Sync>(
        &mut self,
        w: &mut impl Write,
        value: Option<&Arc<T>>,
        write: impl FnOnce(&mut Self, &mut dyn Write, &T) -> Result<()>,
    ) -> Result<()> {
        let Some(value) = value else {
            return w.write_all(&0_u32.to_ne_bytes()).map_err(b::error);
        };
        let key = Arc::as_ptr(value) as usize;
        if let Some((id, _)) = self.saved.get(&key) {
            return w.write_all(&id.to_ne_bytes()).map_err(b::error);
        }
        let id = u32::try_from(self.saved.len() + 1)
            .map_err(|_| b::invalid("Too many shared pointers"))?;
        if id >= 0x8000_0000 {
            return Err(b::invalid("Too many shared pointers"));
        }
        self.saved.insert(key, (id, value.clone()));
        w.write_all(&(id | 0x8000_0000).to_ne_bytes())
            .map_err(b::error)?;
        write(self, w, value)
    }
    /// # Errors
    /// Returns invalid/truncated archive and pointer-reference errors.
    pub fn read_flow(&mut self, r: &mut impl Read) -> Result<Option<Arc<OpticalFlowResult>>> {
        self.read_ptr(r, |archive, mut r| {
            let t_ns = b::i64(&mut r)?;
            let cameras = b::count(&mut r)?;
            let mut observations = Vec::new();
            for _ in 0..cameras {
                let n = b::count(&mut r)?;
                let mut camera = BTreeMap::new();
                for _ in 0..n {
                    let id = b::native_size(&mut r)?;
                    let mut affine = AffineCompact2::default();
                    for row in 0..2 {
                        for col in 0..3 {
                            let v = f32::from_ne_bytes(b::bytes(&mut r)?);
                            if col == 2 {
                                affine.translation[row] = v;
                            } else {
                                affine.linear[(row, col)] = v;
                            }
                        }
                    }
                    camera.insert(id, affine);
                }
                observations.push(camera);
            }
            let input_images = archive.read_ptr(&mut r, |archive, mut r| {
                let t_ns = b::i64(&mut r)?;
                let n = b::count(&mut r)?;
                let mut img_data = Vec::new();
                for _ in 0..n {
                    let exposure = b::scalar(&mut r)?;
                    let img = archive.read_ptr(&mut r, |_, mut r| {
                        let w = b::native_size(&mut r)?;
                        let h = b::native_size(&mut r)?;
                        w.checked_mul(h)
                            .ok_or_else(|| b::invalid("Image dimensions overflow"))?;
                        let mut image = ManagedImage::<u16>::new(w, h);
                        for pixel in image.data_mut() {
                            *pixel = u16::from_ne_bytes(b::bytes(&mut r)?);
                        }
                        Ok(image)
                    })?;
                    img_data.push(ImageData { img, exposure });
                }
                Ok(OpticalFlowInput { t_ns, img_data })
            })?;
            Ok(OpticalFlowResult {
                t_ns,
                observations,
                input_images,
                pyramid_levels: Vec::new(),
            })
        })
    }
    /// # Errors
    /// Returns I/O and pointer-capacity errors.
    pub fn write_flow(
        &mut self,
        w: &mut impl Write,
        value: Option<&Arc<OpticalFlowResult>>,
    ) -> Result<()> {
        self.write_ptr(w, value, |archive, mut w, flow| {
            b::write_i64(&mut w, flow.t_ns)?;
            b::write_count(&mut w, flow.observations.len())?;
            for camera in &flow.observations {
                b::write_count(&mut w, camera.len())?;
                for (id, affine) in camera {
                    b::write_native_size(&mut w, *id)?;
                    for row in 0..2 {
                        for col in 0..3 {
                            let v = if col == 2 {
                                affine.translation[row]
                            } else {
                                affine.linear[(row, col)]
                            };
                            w.write_all(&v.to_ne_bytes()).map_err(b::error)?;
                        }
                    }
                }
            }
            archive.write_ptr(
                &mut w,
                flow.input_images.as_ref(),
                |archive, mut w, input| {
                    b::write_i64(&mut w, input.t_ns)?;
                    b::write_count(&mut w, input.img_data.len())?;
                    for image in &input.img_data {
                        b::write_scalar(&mut w, image.exposure)?;
                        archive.write_ptr(&mut w, image.img.as_ref(), |_, mut w, img| {
                            b::write_native_size(&mut w, img.w)?;
                            b::write_native_size(&mut w, img.h)?;
                            // Original binary_data writes w*h contiguous pixels, including padding.
                            for pixel in &img.data()[..img.w * img.h] {
                                w.write_all(&pixel.to_ne_bytes()).map_err(b::error)?;
                            }
                            Ok(())
                        })?;
                    }
                    Ok(())
                },
            )
        })
    }
}

fn save_marg(path: &Path, data: &MargData) -> Result<()> {
    let id = data
        .kfs_to_marg
        .first()
        .ok_or_else(|| b::invalid("Marginalization keyframe set is empty"))?;
    let mut file =
        BufWriter::new(File::create(path.join(format!("{id}.cereal"))).map_err(b::error)?);
    data.write_cereal_binary(&mut file)?;
    file.flush().map_err(b::error)
}
fn save_images(path: &Path, input: &Receiver<Option<Arc<OpticalFlowResult>>>) -> Result<()> {
    while let Ok(Some(data)) = input.recv() {
        let mut file = BufWriter::new(
            File::create(path.join(format!("{}.cereal", data.t_ns))).map_err(b::error)?,
        );
        FlowArchive::default().write_flow(&mut file, Some(&data))?;
        file.flush().map_err(b::error)?;
    }
    Ok(())
}
/// Two original worker queues: 1000 marginalizations and 300 image results.
pub struct MargDataSaver {
    pub in_marg_queue: SyncSender<Option<Arc<MargData>>>,
    worker: Option<JoinHandle<Result<()>>>,
}
impl MargDataSaver {
    /// Creates a new archive directory. Existing directories are refused to
    /// avoid the original constructor's unconditional recursive deletion.
    /// # Errors
    /// Returns directory creation or worker spawn errors.
    pub fn new(path: &Path) -> Result<Self> {
        fs::create_dir(path).map_err(b::error)?;
        let image_path = path.join("images");
        fs::create_dir(&image_path).map_err(b::error)?;
        let path = path.to_owned();
        let (in_marg_queue, input) = mpsc::sync_channel::<Option<Arc<MargData>>>(1000);
        let (images, image_input) = mpsc::sync_channel(300);
        let image_worker = thread::Builder::new()
            .name("basalt-marg-images".into())
            .spawn(move || save_images(&image_path, &image_input))
            .map_err(b::error)?;
        let worker = thread::Builder::new()
            .name("basalt-marg-save".into())
            .spawn(move || {
                let result = (|| {
                    let mut processed = HashSet::new();
                    while let Ok(Some(data)) = input.recv() {
                        save_marg(&path, &data)?;
                        for flow in &data.opt_flow_res {
                            if processed.insert(flow.t_ns) {
                                images
                                    .send(Some(flow.clone()))
                                    .map_err(|_| b::invalid("Image saver stopped"))?;
                            }
                        }
                    }
                    Ok(())
                })();
                drop(images);
                let images_result = image_worker
                    .join()
                    .map_err(|_| b::invalid("Image saver panicked"))?;
                result.and(images_result)
            })
            .map_err(b::error)?;
        Ok(Self {
            in_marg_queue,
            worker: Some(worker),
        })
    }
    /// # Errors
    /// Returns file I/O failures or worker panic errors.
    pub fn finish(mut self) -> Result<()> {
        self.join()
    }
    fn join(&mut self) -> Result<()> {
        let Some(worker) = self.worker.take() else {
            return Ok(());
        };
        let _ = self.in_marg_queue.send(None);
        worker
            .join()
            .map_err(|_| b::invalid("Marginalization saver panicked"))?
    }
}
impl Drop for MargDataSaver {
    fn drop(&mut self) {
        if let Err(error) = self.join() {
            log::error!("Marginalization saver shutdown: {error}");
        }
    }
}
fn load_directory(path: &Path, output: &SyncSender<Option<Arc<MargData>>>) -> Result<()> {
    let mut flows = BTreeMap::new();
    for entry in fs::read_dir(path.join("images")).map_err(b::error)? {
        let entry = entry.map_err(b::error)?;
        let mut file = BufReader::new(File::open(entry.path()).map_err(b::error)?);
        let flow = FlowArchive::default()
            .read_flow(&mut file)?
            .ok_or_else(|| b::invalid("Null archived flow"))?;
        flows.insert(flow.t_ns, flow);
    }
    let mut files = BTreeMap::<i64, PathBuf>::new();
    for entry in fs::read_dir(path).map_err(b::error)? {
        let entry = entry.map_err(b::error)?;
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if !name.as_bytes().first().is_some_and(u8::is_ascii_digit) {
            continue;
        }
        let raw = name
            .rsplit_once('.')
            .map_or(name.as_ref(), |(stem, _)| stem);
        let id = raw
            .parse()
            .map_err(|_| b::invalid("Invalid marginalization filename"))?;
        files.insert(id, entry.path());
    }
    for path in files.values() {
        let mut data =
            MargData::read_cereal_binary(&mut BufReader::new(File::open(path).map_err(b::error)?))?;
        for id in &data.kfs_all {
            data.opt_flow_res.push(
                flows
                    .get(id)
                    .cloned()
                    .ok_or_else(|| b::invalid("Missing archived keyframe image"))?,
            );
        }
        output
            .send(Some(Arc::new(data)))
            .map_err(|_| b::invalid("Marginalization consumer disconnected"))?;
    }
    Ok(())
}
pub struct MargDataLoader {
    worker: Option<JoinHandle<Result<()>>>,
}
impl MargDataLoader {
    /// # Errors
    /// Returns worker spawn failures; decoding failures are returned by `finish`.
    pub fn start(path: &Path, output: SyncSender<Option<Arc<MargData>>>) -> Result<Self> {
        let path = path.to_owned();
        let worker = thread::Builder::new()
            .name("basalt-marg-load".into())
            .spawn(move || {
                let result = load_directory(&path, &output);
                let _ = output.send(None);
                result
            })
            .map_err(b::error)?;
        Ok(Self {
            worker: Some(worker),
        })
    }
    /// # Errors
    /// Returns decoding, directory I/O or worker panic errors.
    pub fn finish(mut self) -> Result<()> {
        self.join()
    }
    fn join(&mut self) -> Result<()> {
        let Some(worker) = self.worker.take() else {
            return Ok(());
        };
        worker
            .join()
            .map_err(|_| b::invalid("Marginalization loader panicked"))?
    }
}
impl Drop for MargDataLoader {
    fn drop(&mut self) {
        if let Err(error) = self.join() {
            log::error!("Marginalization loader shutdown: {error}");
        }
    }
}
