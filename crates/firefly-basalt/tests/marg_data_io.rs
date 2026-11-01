use firefly_basalt::{
    io::marg_data_io::{FlowArchive, MargDataLoader, MargDataSaver},
    utils::{common_types::CerealBinary, imu_types::MargData},
};
use std::{
    path::Path,
    sync::{Arc, mpsc},
    time::Duration,
};

#[test]
#[cfg(all(target_endian = "little", target_pointer_width = "64"))]
fn official_marginalization_bytes_preserve_frozen_and_current_states() {
    let reference = include_bytes!("reference/marg_data.cereal");
    let data = MargData::read_cereal_binary(&mut reference.as_slice()).unwrap();
    assert_eq!(data.abs_h.shape(), (2, 3));
    assert_eq!(data.abs_h[(1, 0)], -4.0);
    assert_eq!(data.frame_states.len(), 2);
    assert!(data.use_imu);
    assert!(data.frame_states[&100].is_linearized());
    assert_eq!(data.frame_states[&100].delta()[14], 0.015);
    assert_eq!(data.frame_poses[&100].delta()[0], 0.01);
    let mut bytes = Vec::new();
    data.write_cereal_binary(&mut bytes).unwrap();
    assert_eq!(&bytes, reference);
    for len in [0, 8, 32, reference.len() - 1] {
        assert!(MargData::read_cereal_binary(&mut &reference[..len]).is_err());
    }
}

#[test]
#[cfg(all(target_endian = "little", target_pointer_width = "64"))]
fn official_flow_bytes_preserve_shared_pointer_aliasing_and_nulls() {
    let reference = include_bytes!("reference/optical_flow.cereal");
    let mut input = reference.as_slice();
    let mut archive = FlowArchive::default();
    let first = archive.read_flow(&mut input).unwrap().unwrap();
    let second = archive.read_flow(&mut input).unwrap().unwrap();
    assert_eq!(input, [] as [u8; 0]);
    assert!(Arc::ptr_eq(&first, &second));
    let images = &first.input_images.as_ref().unwrap().img_data;
    assert!(Arc::ptr_eq(
        images[0].img.as_ref().unwrap(),
        images[1].img.as_ref().unwrap()
    ));
    assert!(images[2].img.is_none());
    assert_eq!(
        images[0].img.as_ref().unwrap().data(),
        [0, 11111, 22222, 33333, 44444, 55555]
    );
    assert_eq!(first.observations[0][&7].translation.x, 10.0);
    let mut bytes = Vec::new();
    let mut output = FlowArchive::default();
    output.write_flow(&mut bytes, Some(&first)).unwrap();
    output.write_flow(&mut bytes, Some(&second)).unwrap();
    assert_eq!(&bytes, reference);
    assert!(
        FlowArchive::default()
            .read_flow(&mut &1_u32.to_ne_bytes()[..])
            .is_err()
    );
}

#[test]
#[cfg(all(target_endian = "little", target_pointer_width = "64"))]
fn directory_workers_restore_images_and_signal_eos_on_decode_failure() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../logs")
        .join(format!("basalt-marg-test-{}", std::process::id()));
    let mut data =
        MargData::read_cereal_binary(&mut include_bytes!("reference/marg_data.cereal").as_slice())
            .unwrap();
    let flow = FlowArchive::default()
        .read_flow(&mut include_bytes!("reference/optical_flow.cereal").as_slice())
        .unwrap()
        .unwrap();
    data.kfs_all = [100].into();
    data.opt_flow_res = vec![flow.clone(), flow];
    let saver = MargDataSaver::new(&path).unwrap();
    saver.in_marg_queue.send(Some(Arc::new(data))).unwrap();
    saver.finish().unwrap();
    assert_eq!(std::fs::read_dir(path.join("images")).unwrap().count(), 1);
    let (tx, rx) = mpsc::sync_channel(2);
    let loader = MargDataLoader::start(&path, tx).unwrap();
    let restored = rx.recv_timeout(Duration::from_secs(3)).unwrap().unwrap();
    assert_eq!(restored.opt_flow_res.len(), 1);
    assert_eq!(restored.opt_flow_res[0].t_ns, 100);
    assert!(rx.recv_timeout(Duration::from_secs(3)).unwrap().is_none());
    loader.finish().unwrap();
    std::fs::remove_file(path.join("images/100.cereal")).unwrap();
    let (tx, rx) = mpsc::sync_channel(2);
    let loader = MargDataLoader::start(&path, tx).unwrap();
    assert!(rx.recv_timeout(Duration::from_secs(3)).unwrap().is_none());
    assert!(loader.finish().is_err());
    std::fs::remove_dir_all(path).unwrap();
}
