// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
//! Original `utils/sim_utils.h`: simulated image observations.
use nalgebra::Vector2;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SimObservations {
    #[serde(rename = "value0", with = "positions")]
    pub pos: Vec<Vector2<f64>>,
    #[serde(rename = "value1")]
    pub id: Vec<i32>,
}

mod positions {
    use super::Vector2;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    pub fn serialize<S: Serializer>(
        values: &[Vector2<f64>],
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        values
            .iter()
            .map(|v| [v.x, v.y])
            .collect::<Vec<_>>()
            .serialize(serializer)
    }
    pub fn deserialize<'de, D: Deserializer<'de>>(
        deserializer: D,
    ) -> Result<Vec<Vector2<f64>>, D::Error> {
        Ok(Vec::<[f64; 2]>::deserialize(deserializer)?
            .into_iter()
            .map(Vector2::from)
            .collect())
    }
}
