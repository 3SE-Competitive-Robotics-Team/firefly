// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// License notices and conditions are in ../../LICENSE.
//! Translation of `include/basalt/image/image_pyr.h` at
//! `aa441ba3e51050c47ba1902537792a2e4db7e43d`.

use firefly_calibration::image::image::{Image, ImageMut, ManagedImage};

mod sealed {
    pub trait Sealed {}
    impl Sealed for u8 {}
    impl Sealed for u16 {}
}

/// The two integer pixel types accepted by the reference Gaussian pyramid.
pub trait PyramidPixel: sealed::Sealed + Copy + Default + Into<i32> {
    fn from_filtered(value: i32) -> Self;
}
impl PyramidPixel for u8 {
    fn from_filtered(value: i32) -> Self {
        value as Self
    }
}
impl PyramidPixel for u16 {
    fn from_filtered(value: i32) -> Self {
        value as Self
    }
}

/// Pyramid levels stored in a single mipmap, with level zero on the left.
#[derive(Debug, Default)]
pub struct ManagedImagePyr<T> {
    orig_w: usize,
    image: ManagedImage<T>,
}

/// Reflection at the upper image border, as used by the five-tap kernel.
#[must_use]
pub fn border101(coordinate: i32, size: i32) -> i32 {
    size - 1 - (size - 1 - coordinate).abs()
}

fn vertical<T: PyramidPixel>(image: Image<'_, T>, output_height: usize) -> ManagedImage<i32> {
    let mut accumulator = ManagedImage::<i32>::new(output_height, image.w);
    for row in 0..output_height {
        let center = 2 * row;
        let indices = [
            center.abs_diff(2),
            center.abs_diff(1),
            center,
            border101((center + 1) as i32, image.h as i32) as usize,
            border101((center + 2) as i32, image.h as i32) as usize,
        ];
        for column in 0..image.w {
            let values = indices.map(|y| Into::<i32>::into(*image.get(column, y)));
            accumulator.view_mut().row_mut(column)[row] =
                values[0] + 4 * values[1] + 6 * values[2] + 4 * values[3] + values[4];
        }
    }
    accumulator
}

fn horizontal<T: PyramidPixel>(accumulator: Image<'_, i32>, output: &mut ImageMut<'_, T>) {
    for column in 0..output.w {
        let center = 2 * column;
        let indices = [
            center.abs_diff(2),
            center.abs_diff(1),
            center,
            border101((center + 1) as i32, accumulator.h as i32) as usize,
            border101((center + 2) as i32, accumulator.h as i32) as usize,
        ];
        for row in 0..accumulator.w {
            let values = indices.map(|y| *accumulator.get(row, y));
            let sum = values[0] + 4 * values[1] + 6 * values[2] + 4 * values[3] + values[4];
            output.row_mut(row)[column] = T::from_filtered((sum + (1 << 7)) >> 8);
        }
    }
}

impl<T: PyramidPixel> ManagedImagePyr<T> {
    /// Creates level zero and `num_levels` successive half-resolution levels.
    /// # Panics
    /// Panics if dimensions overflow or a nonempty convolution requires samples
    /// outside the input (a dimension smaller than three pixels).
    #[must_use]
    pub fn new(source: &ManagedImage<T>, num_levels: usize) -> Self {
        let mut result = Self::default();
        result.set_from_image(source, num_levels);
        result
    }

    /// Copies level zero and computes Gaussian levels using integer rounding.
    /// # Panics
    /// Has the same dimension requirements as [`Self::new`].
    pub fn set_from_image(&mut self, source: &ManagedImage<T>, num_levels: usize) {
        self.orig_w = source.w;
        self.image.reinitialise(
            source
                .w
                .checked_add(source.w / 2)
                .expect("mipmap width overflow"),
            source.h,
            source
                .w
                .checked_add(source.w / 2)
                .expect("mipmap width overflow"),
        );
        self.image.view_mut().fill(T::default());
        self.image
            .view_mut()
            .sub_image(0, 0, source.w, source.h)
            .copy_from(source.view());
        for level in 0..num_levels {
            let input = self.lvl(level);
            let accumulator = vertical(input, input.h / 2);
            let [x, y] = self.lvl_offset(level + 1);
            let width = self.orig_w >> (level + 1);
            let height = self.image.h >> (level + 1);
            let mut view = self.image.view_mut();
            horizontal(accumulator.view(), &mut view.sub_image(x, y, width, height));
        }
    }

    /// Applies the separable `[1,4,6,4,1]` Gaussian and downsamples by two.
    /// # Panics
    /// Panics for mismatched output dimensions or out-of-bounds kernel samples.
    pub fn subsample(input: Image<'_, T>, output: &mut ImageMut<'_, T>) {
        assert_eq!((output.w, output.h), (input.w / 2, input.h / 2));
        let accumulator = vertical(input, output.h);
        horizontal(accumulator.view(), output);
    }
}

impl<T> ManagedImagePyr<T> {
    /// Returns the offset of a level within the mipmap.
    /// # Panics
    /// Panics if the level cannot be represented by a machine-word shift.
    #[must_use]
    pub fn lvl_offset(&self, level: usize) -> [usize; 2] {
        let x = if level == 0 { 0 } else { self.orig_w };
        let y = if level <= 1 {
            0
        } else {
            self.image.h - (self.image.h >> (level - 1))
        };
        [x, y]
    }

    /// Returns a borrowed level, including its shared mipmap pitch.
    /// # Panics
    /// Panics if the level cannot be represented by a machine-word shift.
    #[must_use]
    pub fn lvl(&self, level: usize) -> Image<'_, T> {
        let [x, y] = self.lvl_offset(level);
        self.image
            .view()
            .sub_image(x, y, self.orig_w >> level, self.image.h >> level)
    }

    #[must_use]
    pub fn mipmap(&self) -> Image<'_, T> {
        self.image.view()
    }
}
