// SPDX-License-Identifier: AGPL-3.0-or-later
//! Tests of `Store::unmounted`/`remount` and of `ParamStore` (ADR-0004 §6).

use std::cell::Cell;

use crate::flash::{Flash, FlashError};
use crate::sim::SimFlash;
use crate::store::{Error, MountReport, Store};

/// A flash that fails every read while `failing` is set.
struct Flaky<F: Flash> {
    inner: F,
    failing: Cell<bool>,
}

impl<F: Flash> Flaky<F> {
    fn new(inner: F) -> Self {
        Self {
            inner,
            failing: Cell::new(false),
        }
    }
}

impl<F: Flash> Flash for Flaky<F> {
    const SECTOR_SIZE: usize = F::SECTOR_SIZE;
    fn read(&self, s: usize, o: usize, b: &mut [u8]) -> Result<(), FlashError> {
        if self.failing.get() {
            Err(FlashError::Device)
        } else {
            self.inner.read(s, o, b)
        }
    }
    fn program(&mut self, s: usize, o: usize, d: &[u8]) -> Result<(), FlashError> {
        self.inner.program(s, o, d)
    }
    fn erase(&mut self, s: usize) -> Result<(), FlashError> {
        self.inner.erase(s)
    }
}

#[test]
fn an_unmounted_store_holds_nothing_and_refuses_writes() {
    let mut s = Store::unmounted(SimFlash::<88>::new());
    assert_eq!(s.get(1), Ok(None));
    assert_eq!(s.iter().count(), 0);
    assert_eq!(s.set(1, 2), Err(Error::Corrupt));
}

#[test]
fn a_failed_remount_keeps_the_flash_and_a_later_one_works() {
    let mut s = Store::unmounted(Flaky::new(SimFlash::<88>::new()));
    s.flash_mut().failing.set(true);
    assert_eq!(s.remount(), Err(Error::Flash(FlashError::Device)));
    assert_eq!(s.set(1, 2), Err(Error::Corrupt), "not usable until mounted");
    s.flash_mut().failing.set(false);
    assert_eq!(s.remount(), Ok(MountReport::Fresh));
    s.set(1, 2).unwrap();
    assert_eq!(s.get(1), Ok(Some(2)));
    // And again on a populated store: the in-memory state is rebuilt.
    assert_eq!(s.remount(), Ok(MountReport::Clean));
    assert_eq!(s.get(1), Ok(Some(2)));
}
