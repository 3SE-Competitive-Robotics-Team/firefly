// SPDX-License-Identifier: BSD-4-Clause
// Copyright (c) 2014 Paul Furgale, Jérôme Maye, Jörn Rehder and Thomas Schneider.
// See LICENSE for the complete notice and advertising acknowledgement.
//! Original Gridder.h: stable grid iteration, newest object first within each cell.
pub struct Gridder<T> {
    x0: f32,
    y0: f32,
    width: usize,
    height: usize,
    cell_size: f32,
    cells: Vec<Vec<T>>,
}
impl<T: Copy> Gridder<T> {
    #[must_use]
    pub fn new(x0: f32, y0: f32, x1: f32, y1: f32, cell_size: f32) -> Self {
        let width = ((x1 - x0) / cell_size + 1.) as usize;
        let height = ((y1 - y0) / cell_size + 1.) as usize;
        Self {
            x0,
            y0,
            width,
            height,
            cell_size,
            cells: vec![Vec::new(); width * height],
        }
    }
    pub fn add(&mut self, x: f32, y: f32, object: T) {
        let ix = ((x - self.x0) / self.cell_size) as i32;
        let iy = ((y - self.y0) / self.cell_size) as i32;
        if ix >= 0 && iy >= 0 && (ix as usize) < self.width && (iy as usize) < self.height {
            self.cells[iy as usize * self.width + ix as usize].push(object);
        }
    }
    #[must_use]
    pub fn find(&self, x: f32, y: f32, range: f32) -> Vec<T> {
        let ix0 = (((x - range - self.x0) / self.cell_size) as i32).clamp(0, self.width as i32 - 1)
            as usize;
        let ix1 = (((x + range - self.x0) / self.cell_size) as i32).clamp(0, self.width as i32 - 1)
            as usize;
        let iy0 = (((y - range - self.y0) / self.cell_size) as i32).clamp(0, self.height as i32 - 1)
            as usize;
        let iy1 = (((y + range - self.y0) / self.cell_size) as i32).clamp(0, self.height as i32 - 1)
            as usize;
        let mut result = Vec::new();
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                result.extend(self.cells[iy * self.width + ix].iter().rev().copied());
            }
        }
        result
    }
}
