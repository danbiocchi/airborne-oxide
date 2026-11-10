// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fixed-capacity FIFO ring buffer for the delayed estimator horizon (no heap).

/// Fixed-capacity FIFO ring buffer (no heap). Pushing into a full buffer
/// drops the oldest element and returns it.
#[derive(Clone, Copy, Debug)]
pub struct Ring<T: Copy + Default, const CAP: usize> {
    buf: [T; CAP],
    /// Index in `buf` of the oldest element.
    head: usize,
    len: usize,
}

impl<T: Copy + Default, const CAP: usize> Default for Ring<T, CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Copy + Default, const CAP: usize> Ring<T, CAP> {
    pub fn new() -> Self {
        Self {
            buf: [T::default(); CAP],
            head: 0,
            len: 0,
        }
    }

    pub const fn capacity(&self) -> usize {
        CAP
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn is_full(&self) -> bool {
        self.len == CAP
    }

    /// Appends at the back. Returns the element dropped to make room, if any.
    /// With CAP == 0 the new element itself is returned.
    pub fn push(&mut self, v: T) -> Option<T> {
        if CAP == 0 {
            return Some(v);
        }
        if self.len == CAP {
            // Full: the oldest slot is overwritten and becomes the newest.
            let slot = self.buf.get_mut(self.head)?;
            let dropped = core::mem::replace(slot, v);
            self.head = (self.head + 1) % CAP;
            Some(dropped)
        } else {
            let idx = (self.head + self.len) % CAP;
            if let Some(slot) = self.buf.get_mut(idx) {
                *slot = v;
                self.len += 1;
            }
            None
        }
    }

    /// Removes and returns the oldest element.
    pub fn pop_front(&mut self) -> Option<T> {
        if self.len == 0 || CAP == 0 {
            return None;
        }
        let v = self.buf.get(self.head).copied()?;
        self.head = (self.head + 1) % CAP;
        self.len -= 1;
        Some(v)
    }

    pub fn front(&self) -> Option<&T> {
        self.get(0)
    }

    pub fn back(&self) -> Option<&T> {
        self.len.checked_sub(1).and_then(|i| self.get(i))
    }

    /// Element `i`, 0 = oldest.
    pub fn get(&self, i: usize) -> Option<&T> {
        if i >= self.len || CAP == 0 {
            return None;
        }
        self.buf.get((self.head + i) % CAP)
    }

    pub fn get_mut(&mut self, i: usize) -> Option<&mut T> {
        if i >= self.len || CAP == 0 {
            return None;
        }
        self.buf.get_mut((self.head + i) % CAP)
    }

    /// Oldest first.
    pub fn iter(&self) -> impl Iterator<Item = &T> + '_ {
        (0..self.len).filter_map(move |i| self.get(i))
    }

    /// Applies `f` to every element, oldest first.
    pub fn for_each_mut(&mut self, mut f: impl FnMut(&mut T)) {
        for i in 0..self.len {
            if let Some(e) = self.get_mut(i) {
                f(e);
            }
        }
    }

    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use std::collections::VecDeque;

    #[test]
    fn fifo_order_and_overflow() {
        let mut r: Ring<u32, 3> = Ring::new();
        assert!(r.is_empty());
        assert_eq!(r.capacity(), 3);
        assert_eq!(r.push(1), None);
        assert_eq!(r.push(2), None);
        assert_eq!(r.push(3), None);
        assert!(r.is_full());
        assert_eq!(r.push(4), Some(1));
        assert_eq!(r.iter().copied().collect::<Vec<_>>(), vec![2, 3, 4]);
        assert_eq!(r.front(), Some(&2));
        assert_eq!(r.back(), Some(&4));
        assert_eq!(r.get(1), Some(&3));
        assert_eq!(r.get(3), None);
        assert_eq!(r.pop_front(), Some(2));
        assert_eq!(r.len(), 2);
        assert_eq!(r.push(5), None);
        assert_eq!(r.iter().copied().collect::<Vec<_>>(), vec![3, 4, 5]);
    }

    #[test]
    fn get_mut_and_for_each() {
        let mut r: Ring<i32, 4> = Ring::default();
        for v in 0..6 {
            r.push(v);
        }
        // Holds 2, 3, 4, 5.
        if let Some(e) = r.get_mut(0) {
            *e = 20;
        }
        r.for_each_mut(|e| *e += 1);
        assert_eq!(r.iter().copied().collect::<Vec<_>>(), vec![21, 4, 5, 6]);
    }

    #[test]
    fn clear_empties() {
        let mut r: Ring<u8, 2> = Ring::new();
        r.push(1);
        r.push(2);
        r.clear();
        assert!(r.is_empty());
        assert_eq!(r.front(), None);
        assert_eq!(r.back(), None);
        assert_eq!(r.pop_front(), None);
        r.push(9);
        assert_eq!(r.front(), Some(&9));
    }

    #[test]
    fn zero_capacity_never_holds_anything() {
        let mut r: Ring<u8, 0> = Ring::new();
        assert_eq!(r.capacity(), 0);
        assert_eq!(r.push(7), Some(7));
        assert!(r.is_empty() && r.is_full());
        assert_eq!(r.get(0), None);
        assert_eq!(r.pop_front(), None);
        assert_eq!(r.back(), None);
        assert_eq!(r.iter().count(), 0);
        r.for_each_mut(|_| panic!("no elements"));
    }

    #[derive(Clone, Debug)]
    enum Op {
        Push(i32),
        Pop,
        Get(usize),
        Back,
    }

    fn op() -> impl Strategy<Value = Op> {
        prop_oneof![
            (any::<i32>()).prop_map(Op::Push),
            Just(Op::Pop),
            (0usize..8).prop_map(Op::Get),
            Just(Op::Back),
        ]
    }

    fn check<const CAP: usize>(ops: &[Op]) -> Result<(), TestCaseError> {
        let mut r: Ring<i32, CAP> = Ring::new();
        let mut model: VecDeque<i32> = VecDeque::new();
        for o in ops {
            match *o {
                Op::Push(v) => {
                    let got = r.push(v);
                    let want = if CAP == 0 {
                        Some(v)
                    } else if model.len() == CAP {
                        let d = model.pop_front();
                        model.push_back(v);
                        d
                    } else {
                        model.push_back(v);
                        None
                    };
                    prop_assert_eq!(got, want);
                }
                Op::Pop => prop_assert_eq!(r.pop_front(), model.pop_front()),
                Op::Get(i) => prop_assert_eq!(r.get(i), model.get(i)),
                Op::Back => prop_assert_eq!(r.back(), model.back()),
            }
            prop_assert_eq!(r.len(), model.len());
            prop_assert_eq!(r.is_empty(), model.is_empty());
            prop_assert_eq!(r.is_full(), model.len() == CAP);
            prop_assert_eq!(
                r.iter().copied().collect::<Vec<_>>(),
                model.iter().copied().collect::<Vec<_>>()
            );
        }
        Ok(())
    }

    proptest! {
        #[test]
        fn matches_vecdeque_cap0(ops in proptest::collection::vec(op(), 0..200)) {
            check::<0>(&ops)?;
        }

        #[test]
        fn matches_vecdeque_cap1(ops in proptest::collection::vec(op(), 0..200)) {
            check::<1>(&ops)?;
        }

        #[test]
        fn matches_vecdeque_cap5(ops in proptest::collection::vec(op(), 0..200)) {
            check::<5>(&ops)?;
        }
    }
}
