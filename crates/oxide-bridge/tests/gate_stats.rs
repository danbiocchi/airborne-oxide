// SPDX-License-Identifier: AGPL-3.0-or-later
//! The percentile and the median of the gate (ADR-0007 section 5), against
//! values worked by hand.

use oxide_bridge::gate::stats::{mean, median, percentile, percentile_95};

#[test]
fn one_value_is_every_percentile() {
    assert_eq!(percentile_95(&[0.25]), Some(0.25));
    assert_eq!(median(&[0.25]), Some(0.25));
}

#[test]
fn two_values_interpolate_linearly() {
    // position 0.95 * 1 = 0.95: a + 0.95 (b - a)
    let p = percentile_95(&[1.0, 2.0]).expect("a value");
    assert!((p - 1.95).abs() < 1e-12, "{p}");
    assert_eq!(median(&[1.0, 2.0]), Some(1.5));
    // the input need not be sorted
    assert_eq!(median(&[2.0, 1.0]), Some(1.5));
}

#[test]
fn twenty_values_put_the_95th_percentile_between_the_19th_and_the_20th() {
    // 1..=20: position 0.95 * 19 = 18.05 from 0: between the values 19 and 20.
    let v: Vec<f64> = (1..=20).map(f64::from).collect();
    let p = percentile_95(&v).expect("a value");
    assert!((p - 19.05).abs() < 1e-9, "{p}");
    assert_eq!(median(&v), Some(10.5));
    // not nearest rank (which would say 19 or 20)
    assert!(p > 19.0 && p < 20.0);
}

#[test]
fn five_hundred_values_put_it_at_position_474_05() {
    // value k (from 0) is k * 0.001: the position 0.95 * 499 = 474.05 gives
    // 474.05 * 0.001.
    let mut v: Vec<f64> = (0..500).map(|k| f64::from(k) * 0.001).collect();
    v.reverse();
    let p = percentile_95(&v).expect("a value");
    assert!((p - 0.474_05).abs() < 1e-12, "{p}");
    // The median of an even count is the mean of the two middle values.
    let m = median(&v).expect("a value");
    assert!((m - 0.2495).abs() < 1e-12, "{m}");
}

#[test]
fn a_value_of_the_sample_is_returned_exactly_not_recomputed() {
    let v = [0.1, 0.2, 0.3];
    assert_eq!(median(&v), Some(0.2));
    assert_eq!(percentile(&v, 1.0), Some(0.3));
    assert_eq!(percentile(&v, 0.0), Some(0.1));
}

#[test]
fn no_values_has_no_percentile_and_a_non_number_is_refused() {
    assert_eq!(median(&[]), None);
    assert_eq!(percentile_95(&[]), None);
    assert_eq!(median(&[1.0, f64::NAN]), None);
    assert_eq!(mean(&[]), None);
    assert_eq!(mean(&[1.0, 2.0, 6.0]), Some(3.0));
}

#[test]
fn the_median_of_an_even_count_is_a_plus_b_over_two_to_the_last_bit() {
    // For this pair a + 0.5 * (b - a) is 7.451453544440603 and (a + b) / 2 is
    // 7.451453544440604 (ADR-0007 sections 5 and 6: "(a + b) / 2").
    let (a, b) = (0.686_286_663_811_357_9_f64, 14.216_620_425_069_848_f64);
    assert_ne!(a + 0.5 * (b - a), (a + b) / 2.0);
    assert_eq!(median(&[b, a]), Some((a + b) / 2.0));
    assert_eq!(median(&[a, b, 20.0, 30.0]), Some((b + 20.0) / 2.0));
    // The 95th percentile is still the interpolation.
    assert_eq!(percentile(&[a, b], 0.5), Some(a + 0.5 * (b - a)));
}
