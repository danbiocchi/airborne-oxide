// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pinned, stream-based random numbers (DESIGN §5.2 item 3).
//!
//! xoshiro256++ (Blackman and Vigna's public-domain reference algorithm),
//! seeded through SplitMix64. It is our own code rather than a crate, so the
//! sequence for a seed can never change under a dependency update. Normal
//! variates use Box–Muller through `oxide_core::math` (libm), so they are
//! bit-identical on every host.
//!
//! Every subsystem draws from its own [`Stream`], derived from the run seed.
//! Adding a draw in one subsystem therefore never shifts another's sequence.

use oxide_core::math::{Real, Vec3};

/// SplitMix64: used to expand seeds into generator state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SplitMix64(u64);

impl SplitMix64 {
    pub const fn new(seed: u64) -> Self {
        Self(seed)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
}

/// The subsystems that draw random numbers.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Stream {
    Imu,
    Gps,
    Baro,
    Mag,
    Battery,
    Wind,
    /// Scenario randomisation (wind strength, gust times, ...).
    Scenario,
}

impl Stream {
    /// Fixed identifier; never renumber an existing stream.
    pub const fn id(self) -> u64 {
        match self {
            Stream::Imu => 1,
            Stream::Gps => 2,
            Stream::Baro => 3,
            Stream::Mag => 4,
            Stream::Battery => 5,
            Stream::Wind => 6,
            Stream::Scenario => 7,
        }
    }
}

/// xoshiro256++ generator.
#[derive(Clone, Debug, PartialEq)]
pub struct Rng {
    s: [u64; 4],
    /// The second Box–Muller value, used by the next `normal` call.
    spare: Option<f64>,
}

impl Rng {
    /// Generator whose state is expanded from `seed` by SplitMix64.
    pub fn from_seed(seed: u64) -> Self {
        let mut sm = SplitMix64::new(seed);
        let mut s = [0_u64; 4];
        for v in &mut s {
            *v = sm.next_u64();
        }
        Self::from_state(s)
    }

    /// Generator with this exact state (for reference test vectors). An
    /// all-zero state, which would only ever produce zeros, is replaced.
    pub fn from_state(s: [u64; 4]) -> Self {
        let s = if s == [0; 4] {
            [0x9E37_79B9_7F4A_7C15, 0, 0, 0]
        } else {
            s
        };
        Self { s, spare: None }
    }

    /// The generator for one subsystem of one run.
    pub fn stream(run_seed: u64, stream: Stream) -> Self {
        let key = SplitMix64::new(stream.id()).next_u64();
        Self::from_seed(SplitMix64::new(run_seed ^ key).next_u64())
    }

    pub fn next_u64(&mut self) -> u64 {
        let [s0, s1, s2, s3] = &mut self.s;
        let result = s0.wrapping_add(*s3).rotate_left(23).wrapping_add(*s0);
        let t = *s1 << 17;
        *s2 ^= *s0;
        *s3 ^= *s1;
        *s1 ^= *s2;
        *s0 ^= *s3;
        *s2 ^= t;
        *s3 = s3.rotate_left(45);
        result
    }

    /// Uniform in `[0, 1)`, 53 random bits.
    #[allow(clippy::cast_precision_loss)]
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1_u64 << 53) as f64)
    }

    /// Uniform in `[lo, hi)`.
    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.uniform()
    }

    /// `true` with probability `p`.
    pub fn chance(&mut self, p: f64) -> bool {
        self.uniform() < p
    }

    /// Standard normal variate (Box–Muller; both values of each pair are
    /// used).
    pub fn normal(&mut self) -> f64 {
        if let Some(z) = self.spare.take() {
            return z;
        }
        // u1 in (0, 1], so ln(u1) is finite.
        let u1 = 1.0 - self.uniform();
        let u2 = self.uniform();
        let r = Real::sqrt(-2.0 * Real::ln(u1));
        let theta = 2.0 * core::f64::consts::PI * u2;
        self.spare = Some(r * Real::sin(theta));
        r * Real::cos(theta)
    }

    /// Zero-mean normal with standard deviation `sigma`.
    pub fn gaussian(&mut self, sigma: f64) -> f64 {
        sigma * self.normal()
    }

    /// Zero-mean normal vector with per-axis standard deviation `sigma`.
    pub fn gaussian_vec(&mut self, sigma: Vec3<f64>) -> Vec3<f64> {
        let x = self.gaussian(sigma.x);
        let y = self.gaussian(sigma.y);
        let z = self.gaussian(sigma.z);
        Vec3::new(x, y, z)
    }

    /// Unit vector in the horizontal (north-east) plane at a uniform random
    /// bearing.
    pub fn horizontal_direction(&mut self) -> Vec3<f64> {
        let a = self.range(0.0, 2.0 * core::f64::consts::PI);
        Vec3::new(Real::cos(a), Real::sin(a), 0.0)
    }
}

/// Seed of run `index` of the scenario called `scenario_id` (DESIGN §5.2
/// item 1).
pub fn run_seed(scenario_id: &str, index: u64) -> u64 {
    // FNV-1a over the name, then mixed with the index.
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in scenario_id.bytes() {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    SplitMix64::new(h ^ index.wrapping_mul(0x9E37_79B9_7F4A_7C15)).next_u64()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splitmix_reference_vector() {
        let mut sm = SplitMix64::new(0);
        assert_eq!(sm.next_u64(), 0xe220_a839_7b1d_cdaf);
        assert_eq!(sm.next_u64(), 0x6e78_9e6a_a1b9_65f4);
        assert_eq!(sm.next_u64(), 0x06c4_5d18_8009_454f);
    }

    #[test]
    fn xoshiro_reference_vector() {
        // From the reference implementation with state [1, 2, 3, 4].
        let mut r = Rng::from_state([1, 2, 3, 4]);
        assert_eq!(r.next_u64(), 41_943_041);
        assert_eq!(r.next_u64(), 58_720_359);
        assert_eq!(r.next_u64(), 3_588_806_011_781_223);
    }

    #[test]
    fn streams_differ_and_repeat() {
        let a: Vec<u64> = (0..4)
            .map({
                let mut r = Rng::stream(7, Stream::Imu);
                move |_| r.next_u64()
            })
            .collect();
        let mut again = Rng::stream(7, Stream::Imu);
        let b: Vec<u64> = (0..4).map(|_| again.next_u64()).collect();
        assert_eq!(a, b);
        let mut other = Rng::stream(7, Stream::Gps);
        assert_ne!(a[0], other.next_u64());
        let mut seed8 = Rng::stream(8, Stream::Imu);
        assert_ne!(a[0], seed8.next_u64());
    }

    #[test]
    fn normal_has_unit_moments() {
        let mut r = Rng::from_seed(42);
        let n = 200_000;
        let (mut sum, mut sum_sq) = (0.0, 0.0);
        for _ in 0..n {
            let z = r.normal();
            assert!(z.is_finite());
            sum += z;
            sum_sq += z * z;
        }
        let mean = sum / f64::from(n);
        let var = sum_sq / f64::from(n) - mean * mean;
        assert!(mean.abs() < 0.01, "mean {mean}");
        assert!((var - 1.0).abs() < 0.01, "variance {var}");
    }

    #[test]
    fn uniform_stays_in_range() {
        let mut r = Rng::from_seed(1);
        for _ in 0..10_000 {
            let u = r.uniform();
            assert!((0.0..1.0).contains(&u));
        }
    }

    #[test]
    fn run_seeds_are_distinct() {
        let a = run_seed("gust-hold", 0);
        assert_eq!(a, run_seed("gust-hold", 0));
        assert_ne!(a, run_seed("gust-hold", 1));
        assert_ne!(a, run_seed("hover-step", 0));
    }
}
