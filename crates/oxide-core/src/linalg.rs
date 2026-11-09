// SPDX-License-Identifier: AGPL-3.0-or-later
//! Small dense matrices on the stack, generic over the scalar type, for the
//! estimator's covariance maths (DESIGN §4.7). Heap-free and panic-free:
//! out-of-range access reads as zero and writes are ignored.

use crate::math::Real;

/// Dense row-major `R × C` matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat<T, const R: usize, const C: usize> {
    rows: [[T; C]; R],
}

impl<T: Real, const R: usize, const C: usize> Mat<T, R, C> {
    /// All zeros.
    pub fn zeros() -> Self {
        Self {
            rows: [[T::ZERO; C]; R],
        }
    }

    /// Ones on the leading diagonal, zeros elsewhere.
    pub fn identity() -> Self {
        let mut m = Self::zeros();
        for (r, row) in m.rows.iter_mut().enumerate() {
            if let Some(e) = row.get_mut(r) {
                *e = T::ONE;
            }
        }
        m
    }

    pub fn from_rows(rows: [[T; C]; R]) -> Self {
        Self { rows }
    }

    pub fn rows(&self) -> &[[T; C]; R] {
        &self.rows
    }

    /// Element `(r, c)`; zero when out of range.
    pub fn get(&self, r: usize, c: usize) -> T {
        self.rows
            .get(r)
            .and_then(|row| row.get(c))
            .copied()
            .unwrap_or(T::ZERO)
    }

    /// Sets element `(r, c)`; ignored when out of range.
    pub fn set(&mut self, r: usize, c: usize, v: T) {
        if let Some(e) = self.rows.get_mut(r).and_then(|row| row.get_mut(c)) {
            *e = v;
        }
    }

    /// Adds `v` to element `(r, c)`; ignored when out of range.
    pub fn add_at(&mut self, r: usize, c: usize, v: T) {
        if let Some(e) = self.rows.get_mut(r).and_then(|row| row.get_mut(c)) {
            *e += v;
        }
    }

    pub fn transpose(&self) -> Mat<T, C, R> {
        let mut out = Mat::<T, C, R>::zeros();
        for (r, row) in self.rows.iter().enumerate() {
            for (c, v) in row.iter().enumerate() {
                out.set(c, r, *v);
            }
        }
        out
    }

    /// Matrix product `self · o`.
    pub fn mul<const K: usize>(&self, o: &Mat<T, C, K>) -> Mat<T, R, K> {
        let mut out = Mat::<T, R, K>::zeros();
        for (orow, arow) in out.rows.iter_mut().zip(self.rows.iter()) {
            for (a, brow) in arow.iter().zip(o.rows.iter()) {
                for (acc, b) in orow.iter_mut().zip(brow.iter()) {
                    *acc += *a * *b;
                }
            }
        }
        out
    }

    fn zip_with(&self, o: &Self, f: impl Fn(T, T) -> T) -> Self {
        let mut out = *self;
        for (orow, brow) in out.rows.iter_mut().zip(o.rows.iter()) {
            for (a, b) in orow.iter_mut().zip(brow.iter()) {
                *a = f(*a, *b);
            }
        }
        out
    }

    pub fn add(&self, o: &Self) -> Self {
        self.zip_with(o, |a, b| a + b)
    }

    pub fn sub(&self, o: &Self) -> Self {
        self.zip_with(o, |a, b| a - b)
    }

    pub fn scale(&self, k: T) -> Self {
        let mut out = *self;
        for e in out.rows.iter_mut().flat_map(|row| row.iter_mut()) {
            *e *= k;
        }
        out
    }

    /// Matrix-vector product `self · v`.
    pub fn mul_vec(&self, v: &[T; C]) -> [T; R] {
        let mut out = [T::ZERO; R];
        for (o, row) in out.iter_mut().zip(self.rows.iter()) {
            let mut acc = T::ZERO;
            for (a, b) in row.iter().zip(v.iter()) {
                acc += *a * *b;
            }
            *o = acc;
        }
        out
    }

    /// True if every element is finite.
    pub fn is_finite(&self) -> bool {
        self.rows
            .iter()
            .all(|row| row.iter().all(|e| e.is_finite()))
    }

    /// The 3×3 block with top-left corner `(r0, c0)`; parts outside the
    /// matrix read as zero.
    pub fn block3(&self, r0: usize, c0: usize) -> [[T; 3]; 3] {
        let mut out = [[T::ZERO; 3]; 3];
        for (i, orow) in out.iter_mut().enumerate() {
            for (j, o) in orow.iter_mut().enumerate() {
                *o = self.get(r0.saturating_add(i), c0.saturating_add(j));
            }
        }
        out
    }

    /// Writes a 3×3 block with top-left corner `(r0, c0)`; parts outside the
    /// matrix are ignored.
    pub fn set_block3(&mut self, r0: usize, c0: usize, b: &[[T; 3]; 3]) {
        for (i, brow) in b.iter().enumerate() {
            for (j, v) in brow.iter().enumerate() {
                self.set(r0.saturating_add(i), c0.saturating_add(j), *v);
            }
        }
    }
}

impl<T: Real, const N: usize> Mat<T, N, N> {
    /// Replaces the matrix by `(P + Pᵀ)/2`.
    pub fn symmetrize(&mut self) {
        for r in 0..N {
            for c in (r + 1)..N {
                let avg = (self.get(r, c) + self.get(c, r)) * T::HALF;
                self.set(r, c, avg);
                self.set(c, r, avg);
            }
        }
    }

    pub fn diag(&self) -> [T; N] {
        let mut out = [T::ZERO; N];
        for (i, d) in out.iter_mut().enumerate() {
            *d = self.get(i, i);
        }
        out
    }

    pub fn trace(&self) -> T {
        self.diag().iter().fold(T::ZERO, |a, b| a + *b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    type M34 = Mat<f64, 3, 4>;
    type M42 = Mat<f64, 4, 2>;

    fn m34() -> M34 {
        Mat::from_rows([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
        ])
    }

    #[test]
    fn identity_and_basic_ops() {
        let i = Mat::<f64, 3, 3>::identity();
        assert_eq!(i.trace(), 3.0);
        assert_eq!(i.mul(&m34()), m34());
        assert_eq!(m34().mul(&Mat::<f64, 4, 4>::identity()), m34());
        let t = m34().transpose();
        assert_eq!(t.get(3, 2), 12.0);
        assert_eq!(m34().add(&m34()), m34().scale(2.0));
        assert_eq!(m34().sub(&m34()), M34::zeros());
        let v = m34().mul_vec(&[1.0, 0.0, 0.0, 1.0]);
        assert_eq!(v, [5.0, 13.0, 21.0]);
    }

    #[test]
    fn product_value() {
        let b: M42 = Mat::from_rows([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0], [2.0, 0.0]]);
        let p = m34().mul(&b);
        assert_eq!(p.rows(), &[[12.0, 5.0], [28.0, 13.0], [44.0, 21.0]]);
    }

    #[test]
    fn out_of_range_is_harmless() {
        let mut m = m34();
        assert_eq!(m.get(3, 0), 0.0);
        assert_eq!(m.get(0, 4), 0.0);
        assert_eq!(m.get(usize::MAX, usize::MAX), 0.0);
        m.set(9, 9, 1.0);
        m.add_at(0, 99, 1.0);
        assert_eq!(m, m34());
        // Blocks that hang over the edge read zeros and write nothing there.
        let b = m.block3(2, 2);
        assert_eq!(b[0], [11.0, 12.0, 0.0]);
        assert_eq!(b[1], [0.0; 3]);
        assert_eq!(m.block3(usize::MAX, usize::MAX), [[0.0; 3]; 3]);
        let mut z = Mat::<f64, 4, 4>::zeros();
        z.set_block3(2, 2, &[[1.0; 3]; 3]);
        assert_eq!(z.get(3, 3), 1.0);
        assert_eq!(z.trace(), 2.0);
        z.set_block3(usize::MAX, 0, &[[1.0; 3]; 3]);
        assert_eq!(z.trace(), 2.0);
    }

    #[test]
    fn symmetrize_and_finite() {
        let mut m = Mat::<f64, 2, 2>::from_rows([[1.0, 2.0], [4.0, 3.0]]);
        m.symmetrize();
        assert_eq!(m.get(0, 1), 3.0);
        assert_eq!(m.get(1, 0), 3.0);
        assert_eq!(m.diag(), [1.0, 3.0]);
        assert!(m.is_finite());
        m.set(0, 0, f64::NAN);
        assert!(!m.is_finite());
    }

    #[test]
    fn block_round_trip() {
        let mut m = Mat::<f64, 6, 6>::zeros();
        let b = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        m.set_block3(3, 0, &b);
        assert_eq!(m.block3(3, 0), b);
        assert_eq!(m.get(5, 2), 9.0);
    }

    fn mat_strategy<const R: usize, const C: usize>() -> impl Strategy<Value = Mat<f64, R, C>> {
        proptest::collection::vec(-10.0f64..10.0, R * C).prop_map(|v| {
            let mut m = Mat::<f64, R, C>::zeros();
            for (i, x) in v.into_iter().enumerate() {
                m.set(i / C, i % C, x);
            }
            m
        })
    }

    proptest! {
        #[test]
        fn identity_is_neutral(a in mat_strategy::<4, 3>()) {
            prop_assert_eq!(Mat::<f64, 4, 4>::identity().mul(&a), a);
            prop_assert_eq!(a.mul(&Mat::<f64, 3, 3>::identity()), a);
        }

        #[test]
        fn transpose_of_product(a in mat_strategy::<3, 4>(), b in mat_strategy::<4, 2>()) {
            let lhs = a.mul(&b).transpose();
            let rhs = b.transpose().mul(&a.transpose());
            prop_assert_eq!(lhs, rhs);
            prop_assert_eq!(a.transpose().transpose(), a);
        }

        #[test]
        fn associativity(
            a in mat_strategy::<3, 4>(),
            b in mat_strategy::<4, 2>(),
            c in mat_strategy::<2, 3>(),
        ) {
            let l = a.mul(&b).mul(&c);
            let r = a.mul(&b.mul(&c));
            for (lr, rr) in l.rows().iter().zip(r.rows()) {
                for (x, y) in lr.iter().zip(rr) {
                    prop_assert!((x - y).abs() <= 1e-9 * (1.0 + x.abs()));
                }
            }
        }

        #[test]
        fn out_of_range_never_panics(r in 0usize..usize::MAX, c in 0usize..usize::MAX, v in -1.0f64..1.0) {
            let mut m = Mat::<f64, 3, 3>::identity();
            let before = m;
            if r >= 3 || c >= 3 {
                prop_assert_eq!(m.get(r, c), 0.0);
                m.set(r, c, v);
                m.add_at(r, c, v);
                prop_assert_eq!(m, before);
            }
            let _ = m.block3(r, c);
            m.set_block3(r, c, &[[v; 3]; 3]);
        }
    }
}
