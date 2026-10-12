//! 每帧滤波诊断：标量保留曲线语义，联合协方差与逐特征贡献打包为矩阵。
use firefly_pubsub::viz::{VizMessage, VizPublisher, kind};
use firefly_vio::vio_manager::VioManager;

pub fn publish(viz_pub: &VizPublisher, vio: &VioManager) {
    let publish = |entity: &str, values: &[f64]| {
        for (part, values) in values.chunks(4).enumerate() {
            let path = if part == 0 {
                entity.to_owned()
            } else {
                format!("{entity}/{part}")
            };
            let mut msg = VizMessage::base(kind::SCALARS, vio.state.timestamp, &path);
            msg.scalars[..values.len()].copy_from_slice(values);
            msg.scalar_count = values.len() as u32;
            let _ = viz_pub.publish(msg);
        }
    };
    for update in &vio.state.update_diagnostics {
        let root = format!("vio/update/{}", update.source);
        publish(&format!("{root}/innovation"), &update.innovation);
        publish(&format!("{root}/correction"), &update.correction);
        for (name, snapshot) in [("before", &update.before), ("after", &update.after)] {
            publish(&format!("{root}/{name}/state"), &snapshot.value);
            publish(&format!("{root}/{name}/variance"), &snapshot.variance);
            let _ = viz_pub.publish(VizMessage::matrix(
                vio.state.timestamp,
                &format!("{root}/{name}/attitude_bias_covariance"),
                9,
                9,
                &snapshot.attitude_bias_covariance,
            ));
        }
    }
    for (batch, rows) in vio
        .state
        .slam_diagnostics
        .chunks(firefly_pubsub::viz::POINTS_MAX * 3 / 14)
        .enumerate()
    {
        let _ = viz_pub.publish(VizMessage::matrix(
            vio.state.timestamp,
            &format!("vio/update/slam/features/{batch}"),
            rows.len(),
            14,
            rows.as_flattened(),
        ));
    }
    publish(
        "vio/update/count",
        &[
            vio.state.update_diagnostics.len() as f64,
            vio.state.features_slam.len() as f64,
        ],
    );
}
