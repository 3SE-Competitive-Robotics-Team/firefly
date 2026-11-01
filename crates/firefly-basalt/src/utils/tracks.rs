// SPDX-License-Identifier: MPL-2.0
// Copyright (c) 2012, 2013 Pierre MOULON, 2018 Nikolaus DEMMEL.
// Source: VladyslavUsenko/basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc,
// include/basalt/utils/tracks.h (adapted from OpenMVG).
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. https://mozilla.org/MPL/2.0/
use super::{
    common_types::{FeatureTracks, ImageFeaturePair, Landmarks, Matches, TimeCamId, TrackId},
    union_find::UnionFind,
};
use std::collections::{BTreeMap, BTreeSet};

/// Builds connected feature tracks from inlier pairwise matches.
#[derive(Default)]
pub struct TrackBuilder {
    pub map_node_to_index: BTreeMap<ImageFeaturePair, TrackId>,
    pub uf_tree: UnionFind,
}
impl TrackBuilder {
    /// Adds the supplied correspondence graph to this builder's node map.
    /// # Panics
    /// Panics if the matches lock is poisoned or node indices exceed the forest.
    pub fn build(&mut self, pairwise_matches: &Matches) {
        let matches = pairwise_matches
            .read()
            .expect("pairwise matches lock poisoned");
        let mut features = BTreeSet::new();
        for (&(left, right), data) in matches.iter() {
            for &(a, b) in &data.inliers {
                features.insert((left, a));
                features.insert((right, b));
            }
        }
        for (index, feature) in features.into_iter().enumerate() {
            self.map_node_to_index
                .entry(feature)
                .or_insert(index as TrackId);
        }
        self.uf_tree.init_sets(self.map_node_to_index.len() as u32);
        for (&(left, right), data) in matches.iter() {
            for &(a, b) in &data.inliers {
                self.uf_tree.union(
                    self.map_node_to_index[&(left, a)] as u32,
                    self.map_node_to_index[&(right, b)] as u32,
                );
            }
        }
    }

    /// Rejects short components and components with multiple features from one image.
    /// The boolean return is always false, as in the source API.
    /// # Panics
    /// Panics if the public node map or forest contains invalid indices.
    pub fn filter(&mut self, minimum_track_length: usize) -> bool {
        let mut tracks: BTreeMap<TrackId, BTreeSet<TimeCamId>> = BTreeMap::new();
        let mut problematic = BTreeSet::new();
        for (&(image, _), &index) in &self.map_node_to_index {
            let track = TrackId::from(self.uf_tree.find(index as u32));
            if problematic.contains(&track) {
                continue;
            }
            if !tracks.entry(track).or_default().insert(image) {
                problematic.insert(track);
            }
        }
        for (track, images) in tracks {
            if images.len() < minimum_track_length {
                problematic.insert(track);
            }
        }
        for root in &mut self.uf_tree.cc_parent {
            if problematic.contains(&TrackId::from(*root)) {
                self.uf_tree.cc_size[*root as usize] = 1;
                *root = UnionFind::invalid_index();
            }
        }
        false
    }

    #[must_use]
    pub fn track_count(&self) -> usize {
        let mut parents: BTreeSet<_> = self.uf_tree.cc_parent.iter().copied().collect();
        parents.remove(&UnionFind::invalid_index());
        parents.len()
    }

    /// Replaces the output with every non-rejected component.
    /// # Panics
    /// Panics if the public node map or forest contains invalid indices.
    pub fn export(&mut self, tracks: &mut FeatureTracks) {
        tracks.clear();
        for (&feature, &index) in &self.map_node_to_index {
            let track = self.uf_tree.find(index as u32);
            if track != UnionFind::invalid_index() {
                tracks
                    .entry(TrackId::from(track))
                    .or_default()
                    .entry(feature.0)
                    .or_insert(feature.1);
            }
        }
    }
}

pub fn get_tracks_in_images(
    images: &BTreeSet<TimeCamId>,
    tracks: &FeatureTracks,
    shared: &mut Vec<TrackId>,
) -> bool {
    shared.clear();
    for (&id, track) in tracks {
        if images.iter().all(|image| track.contains_key(image)) {
            shared.push(id);
        }
    }
    !shared.is_empty()
}

pub fn get_tracks_in_image(
    image: TimeCamId,
    tracks: &FeatureTracks,
    shared: &mut Vec<TrackId>,
) -> bool {
    get_tracks_in_images(&BTreeSet::from([image]), tracks, shared)
}

/// Intersects a map's landmarks with tracks observed in the supplied image.
/// # Panics
/// Panics if a landmark's track is absent, matching the source's checked lookup.
pub fn get_shared_tracks(
    image: TimeCamId,
    tracks: &FeatureTracks,
    landmarks: &Landmarks,
    shared: &mut Vec<TrackId>,
) -> bool {
    shared.clear();
    for &id in landmarks.keys() {
        if tracks[&id].contains_key(&image) {
            shared.push(id);
        }
    }
    !shared.is_empty()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::utils::common_types::{Landmark, MatchData};
    use std::{collections::HashMap, sync::RwLock};

    #[test]
    fn matches_cpp_oracle() {
        let expected: serde_json::Value =
            serde_json::from_str(include_str!("../../tests/reference/utils.json")).unwrap();
        let matches = RwLock::new(HashMap::from([(
            (TimeCamId::new(100, 0), TimeCamId::new(200, 1)),
            MatchData {
                inliers: vec![(0, 10), (1, 11), (2, 12), (3, 12)],
                ..MatchData::default()
            },
        )]));
        let mut builder = TrackBuilder::default();
        let state = |sets: &UnionFind| serde_json::json!({"parent": sets.cc_parent, "rank": sets.cc_rank, "size": sets.cc_size});
        builder.build(&matches);
        assert_eq!(state(&builder.uf_tree), expected["tracks_before"]);
        assert_eq!(
            serde_json::json!(builder.filter(2)),
            expected["filter_return"]
        );
        assert_eq!(state(&builder.uf_tree), expected["tracks_after"]);
        assert_eq!(
            serde_json::json!(builder.track_count()),
            expected["track_count"]
        );
        let mut tracks = FeatureTracks::new();
        builder.export(&mut tracks);
        let sorted: BTreeMap<_, _> = tracks.into_iter().collect();
        let actual: Vec<_> = sorted
            .into_iter()
            .map(|(id, features)| {
                let rows: Vec<_> = features
                    .into_iter()
                    .map(|(image, feature)| {
                        serde_json::json!([image.frame_id, image.cam_id, feature])
                    })
                    .collect();
                serde_json::json!({"id":id,"features":rows})
            })
            .collect();
        assert_eq!(serde_json::json!(actual), expected["tracks"]);
    }

    #[test]
    fn valid_tracks_survive_and_image_collisions_are_removed() {
        let images = [
            TimeCamId::new(10, 0),
            TimeCamId::new(10, 1),
            TimeCamId::new(20, 0),
        ];
        let matches = RwLock::new(HashMap::from([
            (
                (images[0], images[1]),
                MatchData {
                    inliers: vec![(0, 1), (2, 3), (4, 5)],
                    ..MatchData::default()
                },
            ),
            (
                (images[1], images[2]),
                MatchData {
                    inliers: vec![(1, 2), (3, 6), (5, 7)],
                    ..MatchData::default()
                },
            ),
            (
                (images[0], images[2]),
                MatchData {
                    inliers: vec![(8, 6)],
                    ..MatchData::default()
                },
            ),
        ]));
        let mut builder = TrackBuilder::default();
        builder.build(&matches);
        assert!(!builder.filter(3));
        assert_eq!(builder.track_count(), 2);
        let mut tracks = FeatureTracks::new();
        builder.export(&mut tracks);
        let actual: BTreeSet<Vec<_>> = tracks
            .values()
            .map(|t| {
                t.iter()
                    .map(|(&image, &feature)| (image, feature))
                    .collect()
            })
            .collect();
        assert_eq!(
            actual,
            BTreeSet::from([
                vec![(images[0], 0), (images[1], 1), (images[2], 2)],
                vec![(images[0], 4), (images[1], 5), (images[2], 7)],
            ])
        );
        let mut found = vec![-1];
        assert!(get_tracks_in_images(&BTreeSet::new(), &tracks, &mut found));
        assert_eq!(found.len(), 2);
        assert!(!get_tracks_in_image(
            TimeCamId::new(999, 0),
            &tracks,
            &mut found
        ));
        assert_eq!(found, Vec::<TrackId>::new());
        let (&id, _) = tracks.iter().next().unwrap();
        assert!(get_shared_tracks(
            images[0],
            &tracks,
            &HashMap::from([(id, Landmark::default())]),
            &mut found
        ));
        assert_eq!(found, vec![id]);
    }

    #[test]
    fn short_and_empty_tracks_are_explicit() {
        let matches = RwLock::new(HashMap::from([(
            (TimeCamId::new(1, 0), TimeCamId::new(2, 0)),
            MatchData {
                inliers: vec![(1, 2)],
                ..MatchData::default()
            },
        )]));
        let mut builder = TrackBuilder::default();
        builder.build(&matches);
        builder.filter(3);
        let mut tracks = FeatureTracks::new();
        builder.export(&mut tracks);
        assert!(tracks.is_empty());
        assert_eq!(builder.track_count(), 0);
        let mut empty = TrackBuilder::default();
        empty.build(&Matches::default());
        assert!(!empty.filter(2));
        assert_eq!(empty.track_count(), 0);
    }
}
