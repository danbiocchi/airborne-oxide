// SPDX-License-Identifier: AGPL-3.0-or-later
//! The parameter policy of ADR-0004 §6, over a [`Store`], in one place the
//! firmware reuses.
//!
//! # What it does
//!
//! * [`ParamStore::boot`] mounts the store and applies what it holds onto the
//!   firmware's own configuration with [`params::load`]. It never fails: a
//!   store that cannot be mounted (corrupt) or read (a flash error, even
//!   half-way through) gives the firmware's own values untouched and a report
//!   with `unreadable` set. The report's `torn` counts the records in the live
//!   sector that failed their check and were skipped (a cut save, or damage:
//!   the two look alike); the core reports it (`ParamLoadRecovered`) without
//!   holding a fault. The caller hands the report to `Core::report_param_load`.
//! * [`ParamStore::service`] is called between ticks, from the main loop. It
//!   writes only when the core is disarmed, holds no parameter fault and has
//!   unsaved parameters. It then makes the store hold exactly
//!   `params::changed(core.config(), baseline)` with [`Store::replace_all`]:
//!   one record when one value differs from what the store holds, otherwise a
//!   fresh sector written and made valid in one program call. Then it tells
//!   the core the values are saved.
//!
//! # Consequences
//!
//! * **The store is never written in flight.** `service` does nothing while
//!   the core is armed, however much is unsaved (and `Core::clear_unsaved`
//!   itself does nothing while armed).
//! * **A store that was not fully understood is not modified until an
//!   operator acknowledges it.** Unknown keys, dropped values, a broken rule
//!   between stored values, or an unreadable store all leave the core holding a
//!   parameter fault; `service` is idle while it is held, so the flash keeps
//!   whatever it had (a newer firmware's values, say, which an older firmware
//!   must not erase by merely booting). The acknowledgement
//!   (`Inputs::param_ack`) clears the fault and marks the parameters unsaved;
//!   the next `service` then rewrites the store from the values in use,
//!   removing what was not understood. A corrupt store is formatted first.
//! * **A save of several parameters is all or nothing.** It is one
//!   `replace_all`: either one appended record, or a whole new sector that
//!   becomes the live one in a single program call. A power cut anywhere
//!   leaves the store holding entirely the old set or entirely the new one, so
//!   a pair of values that is valid only together can never be loaded half
//!   (the tests cut at every byte and inside every program call).
//! * **After a flash error** the store is mounted again by the next `service`
//!   (which also finishes or finds finished an interrupted save).
//!
//! The firmware still decides when to call `service` (a save can erase a
//! sector, which stalls the part for tens to hundreds of milliseconds on some
//! chips, so it belongs in the idle loop and never in a time-critical task),
//! and what to show the operator for a held fault.

use oxide_core::Core;
use oxide_core::config::VehicleConfig;
use oxide_core::params::{self, LoadReport};
use oxide_core::schedule::Profile;

use crate::flash::Flash;
use crate::store::{Error, MountReport, Replaced, Store};

/// What [`ParamStore::service`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Serviced {
    /// Nothing was written: armed, a parameter fault is held, or nothing is
    /// unsaved.
    Idle,
    /// The store now holds the values in use, and the core was told so.
    Saved(Replaced),
}

/// What the store needs before it can be written.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Health {
    /// Mounted and usable.
    Ready,
    /// A flash error left the in-memory state unreliable (or boot could not
    /// read the store): mount again before anything else.
    NeedsRemount,
    /// Mounted `Corrupt`: format before anything else.
    NeedsFormat,
}

/// A [`Store`] holding the parameters of a vehicle (see the module docs).
pub struct ParamStore<F: Flash, const N: usize> {
    store: Store<F>,
    health: Health,
    /// The firmware's own configuration, as `boot` was given it before it
    /// applied the stored values: what a save is measured against.
    baseline: VehicleConfig<N>,
    /// Saves that failed one after another (a success resets it).
    failed_saves: u8,
}

/// Failed saves in a row after which `service` stops touching the flash.
pub const MAX_FAILED_SAVES: u8 = 3;

impl<F: Flash, const N: usize> ParamStore<F, N> {
    /// Mounts the store and applies what it holds onto `cfg`, which holds the
    /// firmware's own values. Never fails: a store that cannot be mounted or
    /// read gives the firmware's values and a report with `unreadable` set.
    /// Hand the report to `Core::report_param_load`.
    ///
    /// Nothing is written, except by [`Store::mount`] itself when the flash
    /// holds nothing but headers (a first start, or an interrupted one).
    pub fn boot(flash: F, cfg: &mut VehicleConfig<N>, profile: &Profile) -> (Self, LoadReport) {
        let mut store = Store::unmounted(flash);
        let unreadable = LoadReport {
            unreadable: true,
            ..LoadReport::default()
        };
        let baseline = *cfg;
        let (health, report) = match store.remount() {
            Err(_) => (Health::NeedsRemount, unreadable),
            Ok(MountReport::Corrupt) => (Health::NeedsFormat, unreadable),
            Ok(_) => {
                // Load onto a copy: a read error half-way must not leave a
                // half-applied configuration.
                let mut next = *cfg;
                let mut failed = false;
                let mut report = params::load(
                    &mut next,
                    profile,
                    store.iter().map_while(|r| match r {
                        Ok(kv) => Some(kv),
                        Err(_) => {
                            failed = true;
                            None
                        }
                    }),
                );
                if failed {
                    (Health::NeedsRemount, unreadable)
                } else {
                    *cfg = next;
                    report.torn = u16::try_from(store.torn_records()).unwrap_or(u16::MAX);
                    (Health::Ready, report)
                }
            }
        };
        (
            Self {
                store,
                health,
                baseline,
                failed_saves: 0,
            },
            report,
        )
    }

    /// Call between ticks. Writes only when the core is disarmed, holds no
    /// parameter fault and reports unsaved parameters; then it makes the store
    /// hold exactly `params::changed(core.config(), self.baseline())` with
    /// [`Store::replace_all`] (remounting first after a flash error, and
    /// formatting first if the store was corrupt) and tells the core it is
    /// saved.
    ///
    /// The baseline is the firmware's own configuration, the one `boot` was
    /// given before it applied the stored values: the store keeps it, so a
    /// caller cannot pass the loaded configuration by mistake.
    ///
    /// # Errors
    /// A flash failure (the next call mounts again) or [`Error::Full`].
    /// `Core::params_unsaved` stays set, so the next call tries again, up to
    /// [`MAX_FAILED_SAVES`] failures in a row (a success resets the count):
    /// after that every call returns [`Error::GaveUp`] without touching the
    /// flash, until the next boot. (A save that cannot complete would
    /// otherwise erase a sector on every call from the idle loop.)
    pub fn service(&mut self, core: &mut Core<N>) -> Result<Serviced, Error> {
        if core.is_armed() || core.params_fault() || !core.params_unsaved() {
            return Ok(Serviced::Idle);
        }
        if self.gave_up() {
            return Err(Error::GaveUp);
        }
        let r = self.save(core);
        match r {
            Ok(_) => self.failed_saves = 0,
            Err(_) => self.failed_saves = self.failed_saves.saturating_add(1),
        }
        r
    }

    fn save(&mut self, core: &mut Core<N>) -> Result<Serviced, Error> {
        if self.health == Health::NeedsRemount {
            if self.store.remount()? == MountReport::Corrupt {
                self.health = Health::NeedsFormat;
            } else {
                self.health = Health::Ready;
            }
        }
        if self.health == Health::NeedsFormat {
            // An interrupted format leaves the store needing one again.
            self.store.format()?;
            self.health = Health::Ready;
        }
        match self
            .store
            .replace_all(params::changed(core.config(), &self.baseline))
        {
            Ok(replaced) => {
                core.clear_unsaved();
                Ok(Serviced::Saved(replaced))
            }
            Err(e) => {
                if matches!(e, Error::Flash(_) | Error::NeedsRemount) {
                    self.health = Health::NeedsRemount;
                }
                Err(e)
            }
        }
    }

    /// Whether `service` has given up saving until the next boot.
    #[must_use]
    pub fn gave_up(&self) -> bool {
        self.failed_saves >= MAX_FAILED_SAVES
    }

    /// The firmware's own configuration that `boot` was given, before the
    /// stored values were applied to it.
    #[must_use]
    pub fn baseline(&self) -> &VehicleConfig<N> {
        &self.baseline
    }

    /// The flash (for counters in tests and tools).
    #[must_use]
    pub fn flash(&self) -> &F {
        self.store.flash()
    }

    /// The flash, for tests that arm a power cut.
    pub fn flash_mut(&mut self) -> &mut F {
        self.store.flash_mut()
    }

    /// Gives the flash back, to boot again from it.
    #[must_use]
    pub fn into_flash(self) -> F {
        self.store.into_flash()
    }

    /// Sector erases the store has made in its life (the wear count).
    #[must_use]
    pub fn erase_count(&self) -> u32 {
        self.store.erase_count()
    }

    /// Torn records found in the live sector at boot (they are skipped).
    #[must_use]
    pub fn torn_records(&self) -> usize {
        self.store.torn_records()
    }
}
