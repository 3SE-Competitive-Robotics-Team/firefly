// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are in ../../LICENSE.
//! Native Cereal overloads from original `calibration_helper.h`.
use crate::{io::marg_data_io::binary, utils::common_types::CerealBinary};
use firefly_calibration::observation::{CalibCornerData, CalibInitPoseData};
use firefly_error::Result;
use std::io::{Read, Write};

pub(crate) fn read_map<T: CerealBinary>(
    reader: &mut impl Read,
) -> Result<std::collections::HashMap<firefly_calibration::observation::TimeCamId, T>> {
    let mut output = std::collections::HashMap::new();
    for _ in 0..binary::count(reader)? {
        let key = firefly_calibration::observation::TimeCamId::read_cereal_binary(reader)?;
        let value = T::read_cereal_binary(reader)?;
        output.entry(key).or_insert(value);
    }
    Ok(output)
}
pub(crate) fn write_map<T: CerealBinary>(
    writer: &mut impl Write,
    map: &std::collections::HashMap<firefly_calibration::observation::TimeCamId, T>,
) -> Result<()> {
    binary::write_count(writer, map.len())?;
    for (key, value) in map {
        key.write_cereal_binary(writer)?;
        value.write_cereal_binary(writer)?;
    }
    Ok(())
}
impl CerealBinary for CalibCornerData {
    fn read_cereal_binary(reader: &mut impl Read) -> Result<Self> {
        let mut corners = Vec::new();
        for _ in 0..binary::count(reader)? {
            corners.push(binary::vector(reader)?);
        }
        let mut corner_ids = Vec::new();
        for _ in 0..binary::count(reader)? {
            corner_ids.push(i32::from_ne_bytes(binary::bytes(reader)?));
        }
        let mut radii = Vec::new();
        for _ in 0..binary::count(reader)? {
            radii.push(binary::scalar(reader)?);
        }
        Ok(Self {
            corners,
            corner_ids,
            radii,
        })
    }
    fn write_cereal_binary(&self, writer: &mut impl Write) -> Result<()> {
        binary::write_count(writer, self.corners.len())?;
        for point in &self.corners {
            binary::write_vector(writer, point)?;
        }
        binary::write_count(writer, self.corner_ids.len())?;
        for id in &self.corner_ids {
            writer.write_all(&id.to_ne_bytes()).map_err(binary::error)?;
        }
        binary::write_count(writer, self.radii.len())?;
        for &radius in &self.radii {
            binary::write_scalar(writer, radius)?;
        }
        Ok(())
    }
}
impl CerealBinary for CalibInitPoseData {
    fn read_cereal_binary(reader: &mut impl Read) -> Result<Self> {
        let t_a_c = binary::pose(reader)?;
        let num_inliers = binary::count(reader)?;
        let mut reprojected_corners = Vec::new();
        for _ in 0..binary::count(reader)? {
            reprojected_corners.push(binary::vector(reader)?);
        }
        Ok(Self {
            t_a_c,
            num_inliers,
            reprojected_corners,
        })
    }
    fn write_cereal_binary(&self, writer: &mut impl Write) -> Result<()> {
        binary::write_pose(writer, &self.t_a_c)?;
        binary::write_count(writer, self.num_inliers)?;
        binary::write_count(writer, self.reprojected_corners.len())?;
        for point in &self.reprojected_corners {
            binary::write_vector(writer, point)?;
        }
        Ok(())
    }
}
