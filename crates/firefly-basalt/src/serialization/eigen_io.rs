// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are indexed in docs/licenses/README.md.
//! Original `serialization/eigen_io.h`: row-major coefficient archives, regardless
//! of matrix storage order. Cereal native binary uses native scalar byte order.
//! Fully dynamic matrix JSON intentionally retains the upstream non-roundtrip
//! behavior: two size tags disappear on write and both read as the array length.
use crate::utils::sophus_utils::Sim3;
use nalgebra::{
    DefaultAllocator, Dim, Isometry3, OMatrix, Quaternion, Scalar, Translation3, UnitQuaternion,
    Vector3, allocator::Allocator,
};
use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error as _, ser::SerializeSeq};
use std::io::{self, Read, Write};

/// Native Cereal primitive coefficients accepted by the matrix adapter.
pub trait ArchiveScalar: Scalar + Copy {
    /// # Errors
    /// Propagates truncated input or I/O errors.
    fn read_native(input: &mut impl Read) -> io::Result<Self>;
    /// # Errors
    /// Propagates output I/O errors.
    fn write_native(self, output: &mut impl Write) -> io::Result<()>;
}
macro_rules! scalar {
    ($($type:ty),*)=>{$(impl ArchiveScalar for $type{
        fn read_native(input:&mut impl Read)->io::Result<Self>{let mut bytes=[0;std::mem::size_of::<Self>()];input.read_exact(&mut bytes)?;Ok(Self::from_ne_bytes(bytes))}
        fn write_native(self,output:&mut impl Write)->io::Result<()>{output.write_all(&self.to_ne_bytes())}
    })*};
}
scalar!(f32, f64, i8, u8, i16, u16, i32, u32, i64, u64, isize, usize);
impl ArchiveScalar for bool {
    fn read_native(input: &mut impl Read) -> io::Result<Self> {
        let byte = u8::read_native(input)?;
        match byte {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(invalid("Invalid native bool representation")),
        }
    }
    fn write_native(self, output: &mut impl Write) -> io::Result<()> {
        u8::from(self).write_native(output)
    }
}
fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}
fn read_size(input: &mut impl Read) -> io::Result<usize> {
    usize::try_from(u64::read_native(input)?).map_err(|_| invalid("Matrix size does not fit usize"))
}
fn write_size(output: &mut impl Write, size: usize) -> io::Result<()> {
    (size as u64).write_native(output)
}

/// Newtype enables the original archive schema without changing nalgebra serde.
#[derive(Clone, Debug, PartialEq)]
pub struct EigenMatrix<S: Scalar, R: Dim, C: Dim>(pub OMatrix<S, R, C>)
where
    DefaultAllocator: Allocator<R, C>;
impl<S: ArchiveScalar, R: Dim, C: Dim> EigenMatrix<S, R, C>
where
    DefaultAllocator: Allocator<R, C>,
{
    /// # Errors
    /// Reports unsupported zero fixed dimensions, invalid sizes or I/O errors.
    pub fn write_binary(&self, output: &mut impl Write) -> io::Result<()> {
        match (R::try_to_usize(), C::try_to_usize()) {
            (Some(0), _) | (_, Some(0)) => {
                return Err(invalid("Upstream has no zero fixed dimension archive"));
            }
            (Some(_), Some(_)) => {}
            (Some(_), None) | (None, Some(_)) => write_size(output, self.0.len())?,
            (None, None) => {
                write_size(output, self.0.nrows())?;
                write_size(output, self.0.ncols())?;
            }
        }
        for row in 0..self.0.nrows() {
            for col in 0..self.0.ncols() {
                self.0[(row, col)].write_native(output)?;
            }
        }
        Ok(())
    }
    /// # Errors
    /// Reports invalid dimensions, overflow, truncated input or I/O errors.
    pub fn read_binary(input: &mut impl Read) -> io::Result<Self> {
        let (rows, cols) = match (R::try_to_usize(), C::try_to_usize()) {
            (Some(0), _) | (_, Some(0)) => {
                return Err(invalid("Upstream has no zero fixed dimension archive"));
            }
            (Some(rows), Some(cols)) => (rows, cols),
            (Some(rows), None) => (rows, read_size(input)? / rows),
            (None, Some(cols)) => (read_size(input)? / cols, cols),
            (None, None) => (read_size(input)?, read_size(input)?),
        };
        let count = rows
            .checked_mul(cols)
            .ok_or_else(|| invalid("Matrix size overflow"))?;
        let mut coefficients = Vec::new();
        for _ in 0..count {
            coefficients.push(S::read_native(input)?);
        }
        Ok(Self(OMatrix::from_fn_generic(
            R::from_usize(rows),
            C::from_usize(cols),
            |row, col| coefficients[row * cols + col],
        )))
    }
}
impl<S: Scalar + Serialize, R: Dim, C: Dim> Serialize for EigenMatrix<S, R, C>
where
    DefaultAllocator: Allocator<R, C>,
{
    fn serialize<T: Serializer>(&self, serializer: T) -> Result<T::Ok, T::Error> {
        if R::try_to_usize() == Some(0) || C::try_to_usize() == Some(0) {
            return Err(serde::ser::Error::custom(
                "Upstream has no zero fixed dimension archive",
            ));
        }
        let mut output = serializer.serialize_seq(Some(self.0.len()))?;
        for row in 0..self.0.nrows() {
            for col in 0..self.0.ncols() {
                output.serialize_element(&self.0[(row, col)])?;
            }
        }
        output.end()
    }
}
impl<'de, S: Scalar + Deserialize<'de>, R: Dim, C: Dim> Deserialize<'de> for EigenMatrix<S, R, C>
where
    DefaultAllocator: Allocator<R, C>,
{
    fn deserialize<T: Deserializer<'de>>(deserializer: T) -> Result<Self, T::Error> {
        let values = Vec::<S>::deserialize(deserializer)?;
        let count = values.len();
        let (rows, cols) = match (R::try_to_usize(), C::try_to_usize()) {
            (Some(0), _) | (_, Some(0)) => {
                return Err(T::Error::custom(
                    "Upstream has no zero fixed dimension archive",
                ));
            }
            (Some(rows), Some(cols)) => {
                if count != rows * cols {
                    return Err(T::Error::custom("matrix has incorrect length"));
                }
                (rows, cols)
            }
            (Some(rows), None) => (rows, count / rows),
            (None, Some(cols)) => (count / cols, cols),
            (None, None) => {
                if count > 1 {
                    return Err(T::Error::custom(
                        "Upstream fully dynamic JSON matrix reads both dimensions as array length, exhausting input",
                    ));
                }
                (count, count)
            }
        };
        Ok(Self(OMatrix::from_fn_generic(
            R::from_usize(rows),
            C::from_usize(cols),
            |row, col| values[row * cols + col].clone(),
        )))
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
struct PoseCoefficients {
    px: f64,
    py: f64,
    pz: f64,
    qx: f64,
    qy: f64,
    qz: f64,
    qw: f64,
}
impl PoseCoefficients {
    fn new(translation: &Vector3<f64>, q: &Quaternion<f64>) -> Self {
        Self {
            px: translation.x,
            py: translation.y,
            pz: translation.z,
            qx: q.i,
            qy: q.j,
            qz: q.k,
            qw: q.w,
        }
    }
    fn translation(self) -> Vector3<f64> {
        Vector3::new(self.px, self.py, self.pz)
    }
    fn quaternion(self) -> Quaternion<f64> {
        Quaternion::new(self.qw, self.qx, self.qy, self.qz)
    }
    fn write(self, output: &mut impl Write) -> io::Result<()> {
        for value in [
            self.px, self.py, self.pz, self.qx, self.qy, self.qz, self.qw,
        ] {
            value.write_native(output)?;
        }
        Ok(())
    }
    fn read(input: &mut impl Read) -> io::Result<Self> {
        Ok(Self {
            px: f64::read_native(input)?,
            py: f64::read_native(input)?,
            pz: f64::read_native(input)?,
            qx: f64::read_native(input)?,
            qy: f64::read_native(input)?,
            qz: f64::read_native(input)?,
            qw: f64::read_native(input)?,
        })
    }
}
/// Exact `Sophus::SE3d` archive coefficients; reading does not renormalize.
#[derive(Clone, Debug)]
pub struct Se3Archive(pub Isometry3<f64>);
impl Se3Archive {
    fn coefficients(&self) -> PoseCoefficients {
        PoseCoefficients::new(&self.0.translation.vector, self.0.rotation.quaternion())
    }
    fn from_coefficients(value: PoseCoefficients) -> Self {
        Self(Isometry3::from_parts(
            Translation3::from(value.translation()),
            UnitQuaternion::new_unchecked(value.quaternion()),
        ))
    }
    /// # Errors
    /// Propagates I/O errors.
    pub fn write_binary(&self, output: &mut impl Write) -> io::Result<()> {
        self.coefficients().write(output)
    }
    /// # Errors
    /// Propagates truncated input and I/O errors.
    pub fn read_binary(input: &mut impl Read) -> io::Result<Self> {
        Ok(Self::from_coefficients(PoseCoefficients::read(input)?))
    }
}
impl Serialize for Se3Archive {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.coefficients().serialize(serializer)
    }
}
impl<'de> Deserialize<'de> for Se3Archive {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        Ok(Self::from_coefficients(PoseCoefficients::deserialize(
            deserializer,
        )?))
    }
}
/// Exact `Sophus::Sim3d` archive; quaternion norm squared encodes scale.
#[derive(Clone, Debug)]
pub struct Sim3Archive(pub Sim3<f64>);
impl Sim3Archive {
    fn coefficients(&self) -> PoseCoefficients {
        PoseCoefficients::new(&self.0.translation, self.0.scaled_rotation())
    }
    fn from_coefficients(value: PoseCoefficients) -> Self {
        Self(Sim3::from_scaled_quaternion_unchecked(
            value.quaternion(),
            value.translation(),
        ))
    }
    /// # Errors
    /// Propagates I/O errors.
    pub fn write_binary(&self, output: &mut impl Write) -> io::Result<()> {
        self.coefficients().write(output)
    }
    /// # Errors
    /// Propagates truncated input and I/O errors.
    pub fn read_binary(input: &mut impl Read) -> io::Result<Self> {
        Ok(Self::from_coefficients(PoseCoefficients::read(input)?))
    }
}
impl Serialize for Sim3Archive {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.coefficients().serialize(serializer)
    }
}
impl<'de> Deserialize<'de> for Sim3Archive {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        Ok(Self::from_coefficients(PoseCoefficients::deserialize(
            deserializer,
        )?))
    }
}
