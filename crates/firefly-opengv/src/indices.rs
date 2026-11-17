/// Borrowed explicit indices or the complete contiguous correspondence range.
#[derive(Clone, Copy, Debug)]
pub enum Indices<'a> {
    Explicit(&'a [usize]),
    All(usize),
}
impl<'a> Indices<'a> {
    #[must_use]
    pub fn new(indices: &'a [usize]) -> Self {
        Self::Explicit(indices)
    }
    #[must_use]
    pub fn all(number_correspondences: usize) -> Self {
        Self::All(number_correspondences)
    }
    #[must_use]
    pub fn size(&self) -> usize {
        match self {
            Self::Explicit(i) => i.len(),
            Self::All(n) => *n,
        }
    }
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.size() == 0
    }
    #[must_use]
    pub fn get(&self, i: usize) -> usize {
        match self {
            Self::Explicit(indices) => indices[i],
            Self::All(_) => i,
        }
    }
}
impl<'a> From<&'a [usize]> for Indices<'a> {
    fn from(value: &'a [usize]) -> Self {
        Self::new(value)
    }
}
impl From<usize> for Indices<'_> {
    fn from(value: usize) -> Self {
        Self::all(value)
    }
}
