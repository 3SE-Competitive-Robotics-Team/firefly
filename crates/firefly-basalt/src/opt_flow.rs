// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Non-rendering input and result collection from original `src/opt_flow.cpp`.
use crate::{
    io::dataset_io::VioDataset,
    optical_flow::{OpticalFlowInput, OpticalFlowResult},
};
use std::{collections::BTreeMap, sync::Arc};

#[derive(Default)]
pub struct FlowStatistics {
    pub observations: BTreeMap<i64, OpticalFlowResult>,
    pub keypoint_stats: BTreeMap<usize, usize>,
}
impl FlowStatistics {
    /// Each camera observation contributes one, including repeated IDs in stereo.
    /// Duplicate timestamps preserve the first stored result, but still contribute
    /// observations to the statistics, matching C++ `emplace` followed by counting.
    pub fn read_result(&mut self, mut result: OpticalFlowResult) {
        result.input_images = None;
        for camera in &result.observations {
            for &id in camera.keys() {
                *self.keypoint_stats.entry(id).or_default() += 1;
            }
        }
        self.observations.entry(result.t_ns).or_insert(result);
    }
    #[must_use]
    pub fn mean_track_length(&self) -> f64 {
        self.keypoint_stats.values().map(|&n| n as f64).sum::<f64>()
            / self.keypoint_stats.len() as f64
    }
}

/// The caller feeds these images to the existing optical-flow worker and sends
/// its end-of-stream sentinel after this iterator is exhausted.
pub fn feed_images(
    dataset: &mut dyn VioDataset,
) -> impl Iterator<Item = firefly_error::Result<Arc<OpticalFlowInput>>> + '_ {
    dataset.image_timestamps().clone().into_iter().map(|t_ns| {
        Ok(Arc::new(OpticalFlowInput {
            t_ns,
            img_data: dataset.image_data(t_ns)?,
        }))
    })
}
