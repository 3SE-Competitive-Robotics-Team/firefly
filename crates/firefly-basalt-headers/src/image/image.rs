// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// Copyright (c) 2011 Steven Lovegrove (Pangolin image container).
// SPDX-License-Identifier: BSD-3-Clause AND MIT
// License notices and conditions are in ../../LICENSE.
//! Translation of `include/basalt/image/image.h`, headers revision
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d` (original Basalt dependency).
//! Borrowed slices replace raw pointers; stride is in pixels rather than bytes.
//! Raw-pointer reinterpretation and allocator ownership transfer are intentionally
//! absent: Rust views preserve the element type and transfers use moves. Empty
//! reductions return `None`; deallocation resets dimensions instead of retaining
//! a shape with a null pointer. Signed overflow uses wrapping arithmetic where
//! the C++ operation has undefined behavior.

use nalgebra::{RealField, Vector2, Vector3};

/// The two upstream floating-point instantiations.
pub trait ImageScalar: RealField + Copy {
    fn as_f64(self) -> f64;
    /// Smallest positive normal value, matching C++ `numeric_limits::min()`.
    fn min_positive_value() -> Self;
}
impl ImageScalar for f32 {
    fn as_f64(self) -> f64 {
        f64::from(self)
    }
    fn min_positive_value() -> Self {
        Self::MIN_POSITIVE
    }
}
impl ImageScalar for f64 {
    fn as_f64(self) -> f64 {
        self
    }
    fn min_positive_value() -> Self {
        Self::MIN_POSITIVE
    }
}

/// Arithmetic pixel conversion used by the upstream interpolators.
pub trait Pixel: Copy {
    /// Whether C++ arithmetic promotes a float coordinate to double.
    const DOUBLE: bool = false;
    const LOWEST: Self;
    const MAXIMUM: Self;
    fn as_f64(self) -> f64;
}
macro_rules! pixels {
    ($($t:ty),*) => { $(impl Pixel for $t {
        const LOWEST:Self=Self::MIN;
        const MAXIMUM:Self=Self::MAX;
        fn as_f64(self) -> f64 { f64::from(self) }
    })* };
}
pixels!(u8, u16, u32, i8, i16, i32, f32);
impl Pixel for f64 {
    const DOUBLE: bool = true;
    const LOWEST: Self = Self::MIN;
    const MAXIMUM: Self = Self::MAX;
    fn as_f64(self) -> f64 {
        self
    }
}

/// Primitive arithmetic for image reductions and in-place differences.
pub trait ArithmeticPixel: Pixel + Default {
    #[must_use]
    fn add_pixel(self, other: Self) -> Self;
    #[must_use]
    fn subtract_pixel(self, other: Self) -> Self;
    fn repeated_byte(byte: u8) -> Self;
}
macro_rules! integer_arithmetic {
    ($($t:ty),*) => { $(impl ArithmeticPixel for $t {
        fn add_pixel(self,other:Self)->Self { self.wrapping_add(other) }
        fn subtract_pixel(self,other:Self)->Self { self.wrapping_sub(other) }
        fn repeated_byte(byte:u8)->Self { Self::from_ne_bytes([byte;size_of::<Self>()]) }
    })* };
}
integer_arithmetic!(u8, u16, u32, i8, i16, i32);
macro_rules! float_arithmetic {
    ($($t:ty),*) => { $(impl ArithmeticPixel for $t {
        fn add_pixel(self,other:Self)->Self { self+other }
        fn subtract_pixel(self,other:Self)->Self { self-other }
        fn repeated_byte(byte:u8)->Self { Self::from_ne_bytes([byte;size_of::<Self>()]) }
    })* };
}
float_arithmetic!(f32, f64);

/// Explicit numeric conversion corresponding to the upstream pixel cast.
/// Float-to-integer inputs must be finite and representable after truncation;
/// C++ has undefined behavior outside that domain.
pub trait PixelFrom<T>: Sized {
    fn pixel_from(value: T) -> Self;
}
macro_rules! pixel_conversions {
    ($destination:ty; $($source:ty),*) => { $(impl PixelFrom<$source> for $destination {
        #[allow(clippy::cast_possible_truncation,clippy::cast_sign_loss,clippy::cast_precision_loss,clippy::cast_lossless)]
        fn pixel_from(value:$source)->Self { value as Self }
    })* };
}
pixel_conversions!(u8;u8,u16,u32,i8,i16,i32,f32,f64);
pixel_conversions!(u16;u8,u16,u32,i8,i16,i32,f32,f64);
pixel_conversions!(u32;u8,u16,u32,i8,i16,i32,f32,f64);
pixel_conversions!(i8;u8,u16,u32,i8,i16,i32,f32,f64);
pixel_conversions!(i16;u8,u16,u32,i8,i16,i32,f32,f64);
pixel_conversions!(i32;u8,u16,u32,i8,i16,i32,f32,f64);
pixel_conversions!(f32;u8,u16,u32,i8,i16,i32,f32,f64);
pixel_conversions!(f64;u8,u16,u32,i8,i16,i32,f32,f64);

/// Horner evaluation of the four-sample cubic Hermite interpolant and derivative.
#[must_use]
// Preserve the reference's arithmetic order, including its overflow behavior.
#[allow(clippy::manual_midpoint)]
pub fn cub_hermite_spline(samples: [f64; 4], coordinate: f64) -> (f64, f64) {
    let [p0, p1, p2, p3] = samples;
    let cubic = 0.5 * (-p0 + 3. * p1 - 3. * p2 + p3);
    let quadratic = 0.5 * (2. * p0 - 5. * p1 + 4. * p2 - p3);
    let linear = 0.5 * (-p0 + p2);
    (
        p1 + coordinate * (linear + coordinate * (quadratic + coordinate * cubic)),
        linear + coordinate * (2. * quadratic + 3. * cubic * coordinate),
    )
}

/// Read-only pitched view. Padding is excluded from pixel reductions.
#[derive(Clone, Copy, Debug)]
pub struct Image<'a, T> {
    data: &'a [T],
    pub w: usize,
    pub h: usize,
    pub stride: usize,
}
impl<'a, T> Image<'a, T> {
    /// Raw borrowed storage, including pitch padding. This preserves pointer-based
    /// upstream operations that deliberately read a linear prefix of the image.
    #[must_use]
    pub fn storage(&self) -> &'a [T] {
        self.data
    }
    /// # Panics
    /// Panics if the slice cannot contain the requested pitched image.
    #[must_use]
    pub fn new(data: &'a [T], w: usize, h: usize, stride: usize) -> Self {
        assert!(stride >= w);
        let required = if h == 0 {
            0
        } else {
            (h - 1)
                .checked_mul(stride)
                .and_then(|n| n.checked_add(w))
                .expect("image dimensions overflow")
        };
        assert!(data.len() >= required);
        Self { data, w, h, stride }
    }
    #[must_use]
    pub fn area(&self) -> usize {
        self.w * self.h
    }
    #[must_use]
    pub fn is_contiguous(&self) -> bool {
        self.stride == self.w
    }
    #[must_use]
    pub fn size_bytes(&self) -> usize {
        self.stride * self.h * size_of::<T>()
    }
    /// Iterate visible pixels in row order, excluding pitch padding.
    pub fn pixels(&self) -> impl Iterator<Item = &'a T> + '_ {
        (0..self.h).flat_map(|y| self.row(y))
    }
    /// Column view preserving the parent pitch.
    /// # Panics
    /// Panics if the column is outside the image.
    #[must_use]
    pub fn col(&self, x: usize) -> Image<'a, T> {
        self.sub_image(x, 0, 1, self.h)
    }
    /// # Panics
    /// Panics if `y` is outside the view.
    #[must_use]
    pub fn row(&self, y: usize) -> &'a [T] {
        assert!(y < self.h);
        &self.data[y * self.stride..y * self.stride + self.w]
    }
    /// # Panics
    /// Panics if the requested pixel is outside the view.
    #[must_use]
    pub fn get(&self, x: usize, y: usize) -> &'a T {
        &self.row(y)[x]
    }
    /// # Panics
    /// Panics if the rectangle is outside the parent view.
    #[must_use]
    pub fn sub_image(&self, x: usize, y: usize, w: usize, h: usize) -> Image<'a, T> {
        assert!(x <= self.w && w <= self.w - x && y <= self.h && h <= self.h - y);
        let start = if h == 0 { 0 } else { y * self.stride + x };
        Image::new(&self.data[start..], w, h, self.stride)
    }
    #[must_use]
    pub fn in_bounds_integer(&self, x: i32, y: i32, border: i32) -> bool {
        border <= x
            && i64::from(x) < self.w as i64 - i64::from(border)
            && border <= y
            && i64::from(y) < self.h as i64 - i64::from(border)
    }
    /// Floating-point bounds require all four bilinear neighbors.
    /// # Panics
    /// Panics if the scalar type cannot represent the image dimensions.
    #[must_use]
    pub fn in_bounds<S: ImageScalar>(&self, p: &Vector2<S>, border: S) -> bool {
        border <= p.x
            && p.x < S::from_usize(self.w).unwrap() - border - S::one()
            && border <= p.y
            && p.y < S::from_usize(self.h).unwrap() - border - S::one()
    }
    pub fn accumulate<U>(&self, init: U, mut op: impl FnMut(U, &T) -> U) -> U {
        let mut result = init;
        for y in 0..self.h {
            for pixel in self.row(y) {
                result = op(result, pixel);
            }
        }
        result
    }
    #[must_use]
    pub fn min_max(&self) -> Option<(T, T)>
    where
        T: Pixel + PartialOrd,
    {
        if self.w == 0 || self.h == 0 {
            return None;
        }
        // C++ std::min/max take the new pixel as their first argument.
        Some(self.accumulate((T::MAXIMUM, T::LOWEST), |(lo, hi), &v| {
            (if lo < v { lo } else { v }, if v < hi { hi } else { v })
        }))
    }
}
impl<T: Pixel> Image<'_, T> {
    fn value<S: ImageScalar>(&self, x: usize, y: usize) -> S {
        S::from_f64(self.get(x, y).as_f64()).unwrap()
    }
    fn bilinear<S: ImageScalar>(&self, x: usize, y: usize, weights: [S; 4]) -> S {
        let [top_left, bottom_left, top_right, bottom_right] = weights;
        if T::DOUBLE {
            S::from_f64(
                top_left.as_f64() * self.get(x, y).as_f64()
                    + bottom_left.as_f64() * self.get(x, y + 1).as_f64()
                    + top_right.as_f64() * self.get(x + 1, y).as_f64()
                    + bottom_right.as_f64() * self.get(x + 1, y + 1).as_f64(),
            )
            .unwrap()
        } else {
            top_left * self.value(x, y)
                + bottom_left * self.value(x, y + 1)
                + top_right * self.value(x + 1, y)
                + bottom_right * self.value(x + 1, y + 1)
        }
    }
    fn weighted_pair<S: ImageScalar>(
        &self,
        a: S,
        b: S,
        first: (usize, usize),
        second: (usize, usize),
    ) -> f64 {
        if T::DOUBLE {
            a.as_f64() * self.get(first.0, first.1).as_f64()
                + b.as_f64() * self.get(second.0, second.1).as_f64()
        } else {
            (a * self.value(first.0, first.1) + b * self.value(second.0, second.1)).as_f64()
        }
    }
    /// Bilinear interpolation, preserving the upstream multiplication/addition order.
    /// # Panics
    /// Requires `in_bounds(p, 0)`.
    #[must_use]
    pub fn interp<S: ImageScalar>(&self, p: &Vector2<S>) -> S {
        assert!(self.in_bounds(p, S::zero()));
        let (ix, iy) = (p.x.as_f64() as usize, p.y.as_f64() as usize);
        let dx = p.x - S::from_usize(ix).unwrap();
        let dy = p.y - S::from_usize(iy).unwrap();
        let ddx = S::one() - dx;
        let ddy = S::one() - dy;
        self.bilinear(ix, iy, [ddx * ddy, ddx * dy, dx * ddy, dx * dy])
    }
    /// Bilinear value with bilinearly interpolated central-difference gradients.
    /// # Panics
    /// Requires `in_bounds(p, 1)`.
    #[must_use]
    pub fn interp_grad<S: ImageScalar>(&self, p: &Vector2<S>) -> Vector3<S> {
        assert!(self.in_bounds(p, S::one()));
        let (ix, iy) = (p.x.as_f64() as usize, p.y.as_f64() as usize);
        let dx = p.x - S::from_usize(ix).unwrap();
        let dy = p.y - S::from_usize(iy).unwrap();
        let ddx = S::one() - dx;
        let ddy = S::one() - dy;
        let bilinear = |x, y| self.bilinear(x, y, [ddx * ddy, ddx * dy, dx * ddy, dx * dy]);
        let half = S::from_f64(0.5).unwrap();
        Vector3::new(
            bilinear(ix, iy),
            half * (bilinear(ix + 1, iy) - bilinear(ix - 1, iy)),
            half * (bilinear(ix, iy + 1) - bilinear(ix, iy - 1)),
        )
    }
    /// Exact derivative of the bilinearly interpolated image.
    /// # Panics
    /// Requires `in_bounds(p, 0)`.
    #[must_use]
    pub fn interp_grad_bilinear_exact<S: ImageScalar>(&self, p: &Vector2<S>) -> Vector3<S> {
        assert!(self.in_bounds(p, S::zero()));
        let (ix, iy) = (p.x.as_f64() as usize, p.y.as_f64() as usize);
        let dx = p.x - S::from_usize(ix).unwrap();
        let dy = p.y - S::from_usize(iy).unwrap();
        let ddx = S::one() - dx;
        let ddy = S::one() - dy;
        let fxy0 = self.weighted_pair(ddx, dx, (ix, iy), (ix + 1, iy));
        let fxy1 = self.weighted_pair(ddx, dx, (ix, iy + 1), (ix + 1, iy + 1));
        let fx0y = self.weighted_pair(ddy, dy, (ix, iy), (ix, iy + 1));
        let fx1y = self.weighted_pair(ddy, dy, (ix + 1, iy), (ix + 1, iy + 1));
        Vector3::new(
            S::from_f64(fxy0 * ddy.as_f64() + fxy1 * dy.as_f64()).unwrap(),
            S::from_f64(fx1y - fx0y).unwrap(),
            S::from_f64(fxy1 - fxy0).unwrap(),
        )
    }
    /// Cubic Hermite value and exact derivatives with clamped border samples.
    /// # Panics
    /// Requires `in_bounds(p, 0)`.
    #[must_use]
    pub fn interp_grad_cubic_splines<S: ImageScalar>(&self, p: &Vector2<S>) -> Vector3<S> {
        assert!(self.in_bounds(p, S::zero()));
        let (ix, iy) = (p.x.as_f64() as usize, p.y.as_f64() as usize);
        let xs = [
            ix.saturating_sub(1),
            ix,
            (ix + 1).min(self.w - 1),
            (ix + 2).min(self.w - 1),
        ];
        let ys = [
            iy.saturating_sub(1),
            iy,
            (iy + 1).min(self.h - 1),
            (iy + 2).min(self.h - 1),
        ];
        let dx = (p.x - S::from_usize(ix).unwrap()).as_f64();
        let dy = (p.y - S::from_usize(iy).unwrap()).as_f64();
        let rows = ys.map(|y| cub_hermite_spline(xs.map(|x| self.get(x, y).as_f64()), dx));
        let (value, grad_y) = cub_hermite_spline(rows.map(|r| r.0), dy);
        let (grad_x, _) = cub_hermite_spline(rows.map(|r| r.1), dy);
        Vector3::new(
            S::from_f64(value).unwrap(),
            S::from_f64(grad_x).unwrap(),
            S::from_f64(grad_y).unwrap(),
        )
    }
    /// # Panics
    /// Requires `in_bounds(p, 0)`.
    #[must_use]
    pub fn interp_cubic_splines<S: ImageScalar>(&self, p: &Vector2<S>) -> f64 {
        // The upstream value-only overload returns double even for float coordinates.
        assert!(self.in_bounds(p, S::zero()));
        let q = Vector2::new(p.x.as_f64(), p.y.as_f64());
        self.interp_grad_cubic_splines(&q).x
    }
}

impl<T: ArithmeticPixel> Image<'_, T> {
    /// Accumulate in the pixel type, including integer wraparound.
    #[must_use]
    pub fn sum(&self) -> T {
        self.accumulate(T::default(), |sum, &pixel| sum.add_pixel(pixel))
    }
    /// Convert the pixel-type sum before dividing by the image area.
    /// # Panics
    /// Panics for an empty image or an unrepresentable scalar conversion.
    #[must_use]
    pub fn mean<S: ImageScalar>(&self) -> S {
        assert!(self.area() > 0);
        S::from_f64(self.sum().as_f64()).unwrap() / S::from_usize(self.area()).unwrap()
    }
}

/// Exclusively borrowed pitched image, with explicit immutable reborrowing.
pub struct ImageMut<'a, T> {
    data: &'a mut [T],
    pub w: usize,
    pub h: usize,
    pub stride: usize,
}
impl<T> ImageMut<'_, T> {
    #[must_use]
    pub fn view(&self) -> Image<'_, T> {
        Image::new(self.data, self.w, self.h, self.stride)
    }
    /// # Panics
    /// Panics for a row outside the view.
    pub fn row_mut(&mut self, y: usize) -> &mut [T] {
        assert!(y < self.h);
        &mut self.data[y * self.stride..y * self.stride + self.w]
    }
    pub fn transform(&mut self, mut f: impl FnMut(&T) -> T) {
        for y in 0..self.h {
            for p in self.row_mut(y) {
                *p = f(p);
            }
        }
    }
    pub fn fill(&mut self, value: T)
    where
        T: Copy,
    {
        self.transform(|_| value);
    }
    pub fn replace(&mut self, old: T, new: T)
    where
        T: Copy + PartialEq,
    {
        self.transform(|&v| if v == old { new } else { v });
    }
    /// Fill each complete pitched row with a repeated byte, including padding.
    /// # Panics
    /// Panics when the borrowed storage omits the final row's pitch padding.
    pub fn memset(&mut self, byte: u8)
    where
        T: ArithmeticPixel,
    {
        let length = self.stride * self.h;
        self.data[..length].fill(T::repeated_byte(byte));
    }
    /// # Panics
    /// Panics if the column is outside the image.
    pub fn col(&mut self, x: usize) -> ImageMut<'_, T> {
        self.sub_image(x, 0, 1, self.h)
    }
    /// # Panics
    /// Panics when the source dimensions exceed the destination.
    pub fn copy_from(&mut self, source: Image<'_, T>)
    where
        T: Copy,
    {
        assert!(self.w >= source.w && self.h >= source.h);
        for y in 0..source.h {
            self.row_mut(y)[..source.w].copy_from_slice(source.row(y));
        }
    }
    /// # Panics
    /// Panics if the rectangle is outside the parent view.
    pub fn sub_image(&mut self, x: usize, y: usize, w: usize, h: usize) -> ImageMut<'_, T> {
        assert!(x <= self.w && w <= self.w - x && y <= self.h && h <= self.h - y);
        let start = if h == 0 { 0 } else { y * self.stride + x };
        ImageMut {
            data: &mut self.data[start..],
            w,
            h,
            stride: self.stride,
        }
    }
}

/// Owning image. Rust moves and drops provide the upstream ownership transfer.
#[derive(Debug, Default)]
pub struct ManagedImage<T> {
    data: Vec<T>,
    pub w: usize,
    pub h: usize,
    pub stride: usize,
}
impl<T: Default + Clone> ManagedImage<T> {
    #[must_use]
    pub fn new(w: usize, h: usize) -> Self {
        Self::with_stride(w, h, w)
    }
    /// # Panics
    /// Panics for a stride smaller than width or overflowing dimensions.
    #[must_use]
    pub fn with_stride(w: usize, h: usize, stride: usize) -> Self {
        assert!(stride >= w);
        Self {
            data: vec![T::default(); stride.checked_mul(h).expect("image dimensions overflow")],
            w,
            h,
            stride,
        }
    }
    pub fn reinitialise(&mut self, w: usize, h: usize, stride: usize) {
        if self.w != w || self.h != h || self.stride != stride {
            *self = Self::with_stride(w, h, stride);
        }
    }
    pub fn copy_from(&mut self, source: Image<'_, T>)
    where
        T: Copy,
    {
        if self.w != source.w || self.h != source.h {
            *self = Self::new(source.w, source.h);
        }
        self.view_mut().copy_from(source);
    }
    /// Numerically convert visible source pixels; preserve an existing matching pitch.
    pub fn convert_from<U: Copy>(&mut self, source: Image<'_, U>)
    where
        T: PixelFrom<U>,
    {
        if self.w != source.w || self.h != source.h {
            *self = Self::new(source.w, source.h);
        }
        for y in 0..source.h {
            for x in 0..source.w {
                self.data[y * self.stride + x] = T::pixel_from(*source.get(x, y));
            }
        }
    }
}
impl<T> ManagedImage<T> {
    #[must_use]
    pub fn view(&self) -> Image<'_, T> {
        Image::new(&self.data, self.w, self.h, self.stride)
    }
    pub fn view_mut(&mut self) -> ImageMut<'_, T> {
        ImageMut {
            data: &mut self.data,
            w: self.w,
            h: self.h,
            stride: self.stride,
        }
    }
    pub fn data_mut(&mut self) -> &mut [T] {
        &mut self.data
    }
    #[must_use]
    pub fn data(&self) -> &[T] {
        &self.data
    }
    pub fn swap(&mut self, other: &mut Self) {
        std::mem::swap(self, other);
    }
    /// Release owned storage and reset dimensions to keep subsequent safe views valid.
    pub fn deallocate(&mut self) {
        self.data = Vec::new();
        self.w = 0;
        self.h = 0;
        self.stride = 0;
    }
}
impl<T: ArithmeticPixel> std::ops::SubAssign<&Self> for ManagedImage<T> {
    fn sub_assign(&mut self, other: &Self) {
        assert!(self.w >= other.w && self.h >= other.h);
        for y in 0..other.h {
            for x in 0..other.w {
                let index = y * self.stride + x;
                self.data[index] =
                    self.data[index].subtract_pixel(other.data[y * other.stride + x]);
            }
        }
    }
}
