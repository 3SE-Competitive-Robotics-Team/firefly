// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// See docs/licenses/README.md. Native archive order from official imu_types.h.
use super::{PoseStateWithLin, PoseVelBiasStateWithLin};
use crate::imu::imu_types::PoseState;
use crate::imu::imu_types::PoseVelBiasState;
use crate::{io::marg_data_io::binary as b, utils::common_types::CerealBinary};
use firefly_error::Result;
use std::io::{Read, Write};

impl CerealBinary for PoseStateWithLin<f64> {
    fn read_cereal_binary(reader: &mut impl Read) -> Result<Self> {
        let pose = b::pose(reader)?;
        let current = b::pose(reader)?;
        let delta = b::vector(reader)?;
        let linearized = b::boolean(reader)?;
        let timestamp = b::i64(reader)?;
        Ok(Self {
            linearized,
            delta,
            pose_linearized: PoseState::new(timestamp, pose),
            pose_current: current,
            backup: None,
        })
    }
    fn write_cereal_binary(&self, writer: &mut impl Write) -> Result<()> {
        b::write_pose(writer, &self.pose_linearized.t_w_i)?;
        b::write_pose(writer, &self.pose_current)?;
        b::write_vector(writer, &self.delta)?;
        b::write_bool(writer, self.linearized)?;
        b::write_i64(writer, self.pose_linearized.t_ns)
    }
}
fn state(reader: &mut impl Read) -> Result<PoseVelBiasState<f64>> {
    Ok(PoseVelBiasState::new(
        0,
        b::pose(reader)?,
        b::vector(reader)?,
        b::vector(reader)?,
        b::vector(reader)?,
    ))
}
fn write_state(writer: &mut impl Write, state: &PoseVelBiasState<f64>) -> Result<()> {
    b::write_pose(writer, &state.state.pose.t_w_i)?;
    b::write_vector(writer, &state.state.vel_w_i)?;
    b::write_vector(writer, &state.bias_gyro)?;
    b::write_vector(writer, &state.bias_accel)
}
impl CerealBinary for PoseVelBiasStateWithLin<f64> {
    fn read_cereal_binary(reader: &mut impl Read) -> Result<Self> {
        let mut state_linearized = state(reader)?;
        let state_current = state(reader)?;
        let delta = b::vector(reader)?;
        let linearized = b::boolean(reader)?;
        state_linearized.state.pose.t_ns = b::i64(reader)?;
        // Cereal serializes only the linearized state's timestamp.
        Ok(Self {
            linearized,
            delta,
            state_linearized,
            state_current,
            backup: None,
        })
    }
    fn write_cereal_binary(&self, writer: &mut impl Write) -> Result<()> {
        write_state(writer, &self.state_linearized)?;
        write_state(writer, &self.state_current)?;
        b::write_vector(writer, &self.delta)?;
        b::write_bool(writer, self.linearized)?;
        b::write_i64(writer, self.t_ns())
    }
}
