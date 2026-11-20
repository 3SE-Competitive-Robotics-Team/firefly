use firefly_pubsub::camera::{IMAGE_HEIGHT, IMAGE_SIZE, IMAGE_WIDTH};

pub const SIDE: usize = 320;

#[derive(Clone, Copy)]
struct Rows {
    a: usize,
    b: usize,
    mix: f32,
}

impl Rows {
    fn new(position: f32, size: usize) -> Self {
        let p = position.clamp(0., (size - 1) as f32);
        let a = p as usize;
        Self {
            a,
            b: (a + 1).min(size - 1),
            mix: p - a as f32,
        }
    }

    fn sample(self, a: f32, b: f32) -> f32 {
        a + (b - a) * self.mix
    }
}

pub struct Pixels {
    gray: Vec<u8>,
    square: Vec<f32>,
    restored: Vec<f32>,
    input_rows: Vec<Rows>,
    head_axis: Vec<Rows>,
    output_rows: Vec<Rows>,
}

impl Pixels {
    pub fn new() -> Self {
        let aligned = |size: usize, n: usize| {
            (0..n)
                .map(|i| Rows::new((i * (size - 1)) as f32 / (n - 1) as f32, size))
                .collect()
        };
        Self {
            gray: vec![0; SIDE * SIDE],
            square: vec![0.; SIDE * SIDE],
            restored: vec![0.; IMAGE_SIZE],
            input_rows: (0..SIDE)
                .map(|i| {
                    Rows::new(
                        (i as f32 + 0.5) * IMAGE_HEIGHT as f32 / SIDE as f32 - 0.5,
                        IMAGE_HEIGHT,
                    )
                })
                .collect(),
            head_axis: aligned(SIDE / 4, SIDE),
            output_rows: aligned(SIDE, IMAGE_HEIGHT),
        }
    }

    /// Stretch the entire left image with `OpenCV`'s half-pixel input coordinates.
    pub fn input(&mut self, frame: &[u8]) -> &[u8] {
        for (row, &r) in self
            .gray
            .as_chunks_mut::<SIDE>()
            .0
            .iter_mut()
            .zip(&self.input_rows)
        {
            let a = &frame[r.a * IMAGE_WIDTH..][..IMAGE_WIDTH];
            let b = &frame[r.b * IMAGE_WIDTH..][..IMAGE_WIDTH];
            for ((out, &a), &b) in row.iter_mut().zip(a).zip(b) {
                *out = r.sample(f32::from(a), f32::from(b)).round() as u8;
            }
        }
        &self.gray
    }

    /// Restore the compact NPU head to the ONNX output grid, then to camera pixels.
    pub fn restore(&mut self, side: usize, values: &[f32]) -> &[f32] {
        let full = if side == SIDE {
            values
        } else {
            for (row, &y) in self
                .square
                .as_chunks_mut::<SIDE>()
                .0
                .iter_mut()
                .zip(&self.head_axis)
            {
                let a = &values[y.a * side..][..side];
                let b = &values[y.b * side..][..side];
                for (out, &x) in row.iter_mut().zip(&self.head_axis) {
                    *out = y.sample(x.sample(a[x.a], a[x.b]), x.sample(b[x.a], b[x.b]));
                }
            }
            &self.square
        };
        for (row, &y) in self
            .restored
            .as_chunks_mut::<IMAGE_WIDTH>()
            .0
            .iter_mut()
            .zip(&self.output_rows)
        {
            for ((out, &a), &b) in row
                .iter_mut()
                .zip(&full[y.a * SIDE..])
                .zip(&full[y.b * SIDE..])
            {
                *out = y.sample(a, b);
            }
        }
        &self.restored
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn both_heads_preserve_metric_plane_and_camera_edges() {
        let mut pixels = Pixels::new();
        for side in [SIDE, SIDE / 4] {
            let plane: Vec<_> = (0..side * side)
                .map(|i| 1. + (i / side + i % side) as f32 / (side - 1) as f32)
                .collect();
            let restored = pixels.restore(side, &plane);
            for (i, &v) in restored.iter().enumerate() {
                let expected = 1.
                    + (i % IMAGE_WIDTH) as f32 / (IMAGE_WIDTH - 1) as f32
                    + (i / IMAGE_WIDTH) as f32 / (IMAGE_HEIGHT - 1) as f32;
                assert!((v - expected).abs() < 1e-5);
            }
        }
    }
}
