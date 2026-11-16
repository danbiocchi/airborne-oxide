// SPDX-License-Identifier: AGPL-3.0-or-later
//! What is written after each tick, and when (ADR-0005 §3 and §7).

use crate::header::{BuildInfo, Level};
use crate::logger::{Class, Logger, MessageWriter, Stats};
use crate::msg::{Error, Sink};
use crate::record::{self, RecordWriter, Topic};
use crate::topics::{self, Scales};
use oxide_core::config::VehicleConfig;
use oxide_core::flight::Telemetry;
use oxide_core::io::{
    BaroSample, BatterySample, ImuSample, Inputs, MagSample, Outputs, RcSample, Stamped,
};
use oxide_core::time::Micros;

/// Records what the core did, tick by tick, into a [`Logger`] ring.
#[derive(Debug)]
pub struct Recorder<const CAP: usize> {
    logger: Logger<CAP>,
    level: Level,
    scales: Option<Scales>,
    imu: Option<Stamped<ImuSample>>,
    imu_dt_us: u32,
    rc: Option<Stamped<RcSample>>,
    mag: Option<Stamped<MagSample>>,
    baro: Option<Stamped<BaroSample>>,
    battery: Option<Stamped<BatterySample>>,
}

impl<const CAP: usize> Default for Recorder<CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAP: usize> Recorder<CAP> {
    /// A recorder that has not started; a quarter of the ring is kept for
    /// events, parameters and status.
    pub const fn new() -> Self {
        Self {
            logger: Logger::new(CAP / 4),
            level: Level::Normal,
            scales: None,
            imu: None,
            imu_dt_us: 0,
            rc: None,
            mag: None,
            baro: None,
            battery: None,
        }
    }

    /// Writes the definitions to `sink` and makes the recorder ready.
    pub fn start<S: Sink + ?Sized, const N: usize>(
        &mut self,
        sink: &mut S,
        now: Micros,
        build: BuildInfo<'_>,
        profile: &str,
        level: Level,
        cfg: &VehicleConfig<N>,
    ) -> Result<(), Error> {
        let h = crate::header::Header {
            start_us: now.0,
            build,
            profile,
            level,
            nested: topics::NESTED,
        };
        crate::header::write_definitions(sink, &h, cfg, topics::NORMAL)?;
        self.level = level;
        self.scales = Some(Scales::new(cfg));
        Ok(())
    }

    /// Writes what is due after a tick.
    pub fn record<const N: usize>(
        &mut self,
        now: Micros,
        inputs: &Inputs,
        outputs: &Outputs<N>,
        telemetry: &Telemetry,
        cfg: &VehicleConfig<N>,
    ) {
        let Some(scales) = self.scales else { return };
        self.observe(now, inputs);
        let t = now.0;
        let t_sample = self.t_sample(now);
        let status = &outputs.status;
        let tel = telemetry;
        use topics::id;
        let imu = self.imu;
        let dt = self.imu_dt_us;
        if let Some(imu) = imu {
            self.stream(t, id::SENSOR_COMBINED, &topics::SENSOR_COMBINED, |w| {
                topics::sensor_combined(w, now, &imu, dt)
            });
        }
        if let Some(e) = tel.estimate {
            self.stream(
                t,
                id::VEHICLE_ANGULAR_VELOCITY,
                &topics::VEHICLE_ANGULAR_VELOCITY,
                |w| topics::vehicle_angular_velocity(w, now, t_sample, e.rate),
            );
        }
        self.stream(
            t,
            id::VEHICLE_RATES_SETPOINT,
            &topics::VEHICLE_RATES_SETPOINT,
            |w| {
                topics::vehicle_rates_setpoint(
                    w,
                    now,
                    tel.rate_target,
                    tel.thrust_target_n,
                    &scales,
                )
            },
        );
        self.stream(t, id::ACTUATOR_MOTORS, &topics::ACTUATOR_MOTORS, |w| {
            topics::actuator_motors(w, now, t_sample, &outputs.motors)
        });
        self.stream(
            t,
            id::VEHICLE_THRUST_SETPOINT,
            &topics::VEHICLE_THRUST_SETPOINT,
            |w| {
                topics::vehicle_thrust_setpoint(
                    w,
                    now,
                    t_sample,
                    tel.rate_command.thrust_n,
                    &scales,
                )
            },
        );
        self.stream(
            t,
            id::VEHICLE_TORQUE_SETPOINT,
            &topics::VEHICLE_TORQUE_SETPOINT,
            |w| {
                topics::vehicle_torque_setpoint(
                    w,
                    now,
                    t_sample,
                    tel.rate_command.torque_nm,
                    &scales,
                )
            },
        );
        if let Some(e) = tel.estimate {
            self.stream(t, id::VEHICLE_ATTITUDE, &topics::VEHICLE_ATTITUDE, |w| {
                topics::vehicle_attitude(w, now, t_sample, e.attitude)
            });
        }
        self.stream(
            t,
            id::VEHICLE_ATTITUDE_SETPOINT,
            &topics::VEHICLE_ATTITUDE_SETPOINT,
            |w| {
                topics::vehicle_attitude_setpoint(
                    w,
                    now,
                    tel.attitude_target,
                    tel.thrust_target_n,
                    &scales,
                )
            },
        );
        if let Some(e) = tel.estimate {
            let gps = status.estimator.gps_aiding;
            self.stream(
                t,
                id::VEHICLE_LOCAL_POSITION,
                &topics::VEHICLE_LOCAL_POSITION,
                |w| topics::vehicle_local_position(w, now, t_sample, &e, &tel.valid, gps),
            );
        }
        self.stream(
            t,
            id::VEHICLE_LOCAL_POSITION_SETPOINT,
            &topics::VEHICLE_LOCAL_POSITION_SETPOINT,
            |w| {
                topics::vehicle_local_position_setpoint(
                    w,
                    now,
                    tel.setpoints.as_ref(),
                    tel.attitude_target,
                    tel.thrust_target_n,
                    &scales,
                )
            },
        );
        if let Some(rc) = self.rc {
            self.stream(
                t,
                id::MANUAL_CONTROL_SETPOINT,
                &topics::MANUAL_CONTROL_SETPOINT,
                |w| topics::manual_control_setpoint(w, now, &rc),
            );
        }
        if let Some(mag) = self.mag {
            self.stream(
                t,
                id::VEHICLE_MAGNETOMETER,
                &topics::VEHICLE_MAGNETOMETER,
                |w| topics::vehicle_magnetometer(w, now, &mag),
            );
        }
        if let Some(baro) = self.baro {
            self.stream(t, id::VEHICLE_AIR_DATA, &topics::VEHICLE_AIR_DATA, |w| {
                topics::vehicle_air_data(w, now, &baro)
            });
        }
        if let Some(b) = self.battery {
            self.stream(t, id::BATTERY_STATUS, &topics::BATTERY_STATUS, |w| {
                topics::battery_status(
                    w,
                    now,
                    &b.value,
                    tel.battery_cell_v,
                    tel.battery_low,
                    tel.battery_critical,
                    &scales,
                )
            });
        }
        self.must(t, id::VEHICLE_STATUS, &topics::VEHICLE_STATUS, |w| {
            topics::vehicle_status(w, now, status)
        });
        self.must(t, id::FAILSAFE_FLAGS, &topics::FAILSAFE_FLAGS, |w| {
            topics::failsafe_flags(w, now, status)
        });
        let source = cfg.estimator.source;
        self.must(t, id::OXIDE_STATUS, &topics::OXIDE_STATUS, |w| {
            topics::oxide_status(
                w,
                now,
                status,
                source,
                tel.events_dropped,
                tel.landed,
                tel.ground_contact,
            )
        });
        self.stream(t, id::OXIDE_ESTIMATOR, &topics::OXIDE_ESTIMATOR, |w| {
            topics::oxide_estimator(
                w,
                now,
                &status.estimator,
                tel.estimate.is_some(),
                tel.gyro_filtered,
            )
        });
        let stats = self.logger.stats();
        self.must(t, id::OXIDE_LOGGER, &topics::OXIDE_LOGGER, |w| {
            topics::oxide_logger(w, now, &stats)
        });
    }

    /// Remembers the newest sample of each input that feeds a topic.
    fn observe(&mut self, now: Micros, inputs: &Inputs) {
        let _ = now;
        if let Some(s) = inputs.imu {
            self.imu_dt_us = match self.imu {
                Some(prev) => u32::try_from(s.t_sample.since(prev.t_sample)).unwrap_or(u32::MAX),
                None => 0,
            };
            self.imu = Some(s);
        }
        if let Some(s) = inputs.rc {
            self.rc = Some(s);
        }
        if let Some(s) = inputs.mag {
            self.mag = Some(s);
        }
        if let Some(s) = inputs.baro {
            self.baro = Some(s);
        }
        if let Some(s) = inputs.battery {
            self.battery = Some(s);
        }
    }

    /// The time of the data the estimate and the commands are based on: the
    /// newest IMU sample's, or the tick's before one has been seen.
    fn t_sample(&self, now: Micros) -> Micros {
        self.imu.map_or(now, |s| s.t_sample)
    }

    fn stream(
        &mut self,
        now_us: u64,
        id: u16,
        topic: &Topic,
        fill: impl FnOnce(&mut RecordWriter<'_, MessageWriter<'_>>),
    ) -> bool {
        self.logger
            .write(Class::Stream, now_us, |w| record::data(w, id, topic, fill))
    }

    fn must(
        &mut self,
        now_us: u64,
        id: u16,
        topic: &Topic,
        fill: impl FnOnce(&mut RecordWriter<'_, MessageWriter<'_>>),
    ) -> bool {
        self.logger
            .write(Class::Must, now_us, |w| record::data(w, id, topic, fill))
    }

    /// Hands the oldest bytes of the ring to `sink`; returns how many it took.
    pub fn drain<S: Sink + ?Sized>(&mut self, sink: &mut S) -> usize {
        self.logger.drain(|chunk| match sink.put(chunk) {
            Ok(()) => chunk.len(),
            Err(_) => 0,
        })
    }

    /// The logger's counters.
    pub fn stats(&self) -> Stats {
        self.logger.stats()
    }
}
