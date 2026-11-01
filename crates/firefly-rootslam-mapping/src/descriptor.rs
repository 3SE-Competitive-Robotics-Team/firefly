//! ORB on the existing tracked corners, as specified in `RootSLAM` §III-B.
use crate::map::{FeatureId, MapKeyframe};
use firefly_basalt::utils::{
    common_types::{Descriptor, KeypointsData},
    keypoints::{compute_angles, compute_descriptors},
};
use std::collections::BTreeMap;

#[must_use]
pub fn hamming(first: &Descriptor, second: &Descriptor) -> u32 {
    first
        .iter()
        .zip(second)
        .map(|(a, b)| (a ^ b).count_ones())
        .sum()
}

/// Existing ORB descriptors are retained. Corners lacking the original Basalt
/// ORB sampler's 19-pixel border are skipped rather than read out of bounds.
#[must_use]
pub fn compute_keyframe_descriptors(
    frame: &MapKeyframe,
) -> BTreeMap<(usize, FeatureId), Descriptor> {
    let mut output = BTreeMap::new();
    for (camera, features) in frame.features.iter().enumerate() {
        for (&id, feature) in features {
            if let Some(descriptor) = feature.descriptor {
                output.insert((camera, id), descriptor);
            }
        }
        let Some(image) = frame
            .images
            .as_ref()
            .and_then(|images| images.img_data.get(camera))
            .and_then(|data| data.img.as_ref())
        else {
            continue;
        };
        let view = image.view();
        let mut data = KeypointsData::default();
        let mut ids = Vec::new();
        for (&id, feature) in features {
            if feature.descriptor.is_none() && view.in_bounds(&feature.pixel, 19.) {
                ids.push(id);
                data.corners.push(feature.pixel);
            }
        }
        compute_angles(&view, &mut data, true);
        compute_descriptors(&view, &mut data);
        for (id, descriptor) in ids.into_iter().zip(data.corner_descriptors) {
            output.insert((camera, id), descriptor);
        }
    }
    output
}
