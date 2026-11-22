use super::{Evidence, Vec3};
use re_log_encoding::{DecoderApp, read_rrd_footer};
use re_log_msg::LogMsg;
use re_log_types::external::arrow::array::{
    Array, DurationNanosecondArray, FixedSizeListArray, Float32Array, Float64Array, Int64Array,
    ListArray, RecordBatch, StringArray, StructArray, TimestampNanosecondArray, UInt32Array,
};
use std::{
    error::Error,
    fs::File,
    io::{self, BufReader},
    path::Path,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn cast<T: 'static>(a: &dyn Array) -> Result<&T> {
    a.as_any()
        .downcast_ref()
        .ok_or_else(|| format!("unexpected Arrow type {}", a.data_type()).into())
}
fn nanoseconds(column: &dyn Array) -> Result<&[i64]> {
    if column.null_count() != 0 {
        return Err("missing sim_time".into());
    }
    if let Some(times) = column.as_any().downcast_ref::<DurationNanosecondArray>() {
        return Ok(times.values());
    }
    if let Some(times) = column.as_any().downcast_ref::<TimestampNanosecondArray>() {
        return Ok(times.values());
    }
    Ok(cast::<Int64Array>(column)?.values())
}

fn vector<const N: usize>(a: &dyn Array) -> Result<[f64; N]> {
    let vectors = cast::<FixedSizeListArray>(a)?;
    let first = vectors.value(0);
    let v = cast::<Float32Array>(&*first)?;
    if v.len() != N {
        return Err("invalid pose dimension".into());
    }
    Ok(std::array::from_fn(|i| f64::from(v.value(i))))
}

pub(super) fn read(path: &Path) -> Result<Evidence> {
    let run = path
        .parent()
        .filter(|dir| dir.ends_with("rrd"))
        .and_then(Path::parent)
        .filter(|dir| firefly_archive::manifest_path(dir).is_file());
    let files = firefly_archive::reader::RecordingFiles::open(run.unwrap_or(path))?;
    if run.is_some() && !files.paths().iter().any(|file| file == path) {
        return Err("requested file is not a sealed recording segment".into());
    }
    let mut data = Evidence::default();
    let mut seen = std::collections::HashSet::new();
    for path in files.paths() {
        data.recording_complete = false;
        let file = File::open(path)?;
        let footer = futures_lite::future::block_on(read_rrd_footer(&file))?
            .ok_or("missing recording footer")?;
        let mut recordings = footer
            .manifests
            .into_keys()
            .filter(re_log_types::StoreId::is_recording);
        let recording = recordings.next().ok_or("missing recording manifest")?;
        if recordings.next().is_some() {
            return Err("expected one recording".into());
        }
        // Decode the full stream so damaged chunks also fail integrity verification.
        for msg in DecoderApp::decode_eager(BufReader::new(file))? {
            let msg = msg?;
            if msg.store_id().is_recording() && msg.store_id() != &recording {
                return Err("recording identity differs from the manifest".into());
            }
            if let LogMsg::ArrowMsg(id, msg) = msg
                && id == recording
                && seen.insert(msg.batch.chunk_id())
            {
                collect(&mut data, &msg.batch.entity_path().to_string(), &msg.batch)?;
            }
        }
    }
    data.events.sort_by(|a, b| a.0.total_cmp(&b.0));
    data.poses.sort_by(|a, b| a.0.total_cmp(&b.0));
    data.down_frames.sort_by(|a, b| a.0.total_cmp(&b.0));
    for rows in data.scalars.values_mut() {
        rows.sort_by(|a, b| a.0.total_cmp(&b.0));
    }
    Ok(data)
}

fn collect(data: &mut Evidence, path: &str, b: &RecordBatch) -> Result<()> {
    let name = match path {
        "/logs/flight" | "/logs/render" | "/logs/mimir" => "TextLog:text",
        "/gt/pose" => "Transform3D:translation",
        "/cameras/down/image" => "Image:buffer",
        "/fc/debug/state"
        | "/fc/debug/precision"
        | "/fc/imu/source"
        | "/fc/imu/health"
        | "/fc/imu/quaternion_xyzw"
        | "/landing/quality"
        | "/landing/runtime"
        | "/landing/target" => "Scalars:scalars",
        _ => return Ok(()),
    };
    let Some(col) = b.column_by_name(name) else {
        return Ok(());
    };
    let rows = cast::<ListArray>(col.as_ref())?;
    let times = b
        .column_by_name("sim_time")
        .map(|c| nanoseconds(c.as_ref()))
        .transpose()?;
    for i in 0..rows.len() {
        if rows.is_null(i) || rows.value_length(i) == 0 {
            continue;
        }
        let row = rows.value(i);
        if path == "/logs/mimir" {
            data.recording_complete |= cast::<StringArray>(&*row)?
                .iter()
                .flatten()
                .any(|s| s == "recording complete");
            continue;
        }
        let Some(times) = times else { continue };
        let t = times[i] as f64 / 1e9;
        match name {
            "TextLog:text" => {
                for s in cast::<StringArray>(&*row)?.iter().flatten() {
                    if path == "/logs/flight" && s.starts_with("mission {") {
                        data.events.push((t, serde_json::from_str(&s[8..])?));
                    }
                    if path == "/logs/render" && s.starts_with("capture {") {
                        let cfg = serde_json::from_str(&s[8..])?;
                        if !data.capture.is_null() && data.capture != cfg {
                            return Err(
                                "render capture configuration changed during recording".into()
                            );
                        }
                        data.capture = cfg;
                    }
                }
            }
            "Scalars:scalars" => data
                .scalars
                .entry(path.into())
                .or_default()
                .push((t, cast::<Float64Array>(&*row)?.values().to_vec())),
            "Transform3D:translation" => {
                let rotations = column(b, "Transform3D:quaternion")?;
                if rotations.is_valid(i) && rotations.value_length(i) > 0 {
                    data.poses.push((
                        t,
                        Vec3::from(vector::<3>(&*row)?),
                        vector::<4>(&*rotations.value(i))?,
                    ));
                }
            }
            _ => {
                let format = column(b, "Image:format")?;
                if row.len() == 1 && format.is_valid(i) && format.value_length(i) > 0 {
                    let f = format.value(i);
                    let fields = cast::<StructArray>(&*f)?;
                    let dimension = |name| -> Result<u32> {
                        Ok(cast::<UInt32Array>(
                            fields
                                .column_by_name(name)
                                .ok_or("missing image dimension")?
                                .as_ref(),
                        )?
                        .value(0))
                    };
                    data.down_frames.push((
                        t,
                        dimension("width")?,
                        dimension("height")?,
                        usize::try_from(cast::<ListArray>(&*row)?.value_length(0))?,
                    ));
                }
            }
        }
    }
    Ok(())
}
fn column<'a>(batch: &'a RecordBatch, name: &str) -> Result<&'a ListArray> {
    cast(
        batch
            .column_by_name(name)
            .ok_or_else(|| io::Error::other(format!("missing {name}")))?
            .as_ref(),
    )
}
