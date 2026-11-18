# Gate 1 report

**Verdict: NOT PASSED**

Commit `2c2fedb1dab3c65f8d52a431b38b369e9ffa4091`, clean tree.

## The criteria

### A1: HOLDS

- core-f405: 500 of 500 flights are safe and pass the scenario file's limits
- core-h743: 500 of 500 flights are safe and pass the scenario file's limits

### A2: FAILS

- ArduCopter: 0 of 500 flights are not safe and are left out of its statistics
- median: core-f405 0.17141266574579617 m, ArduCopter 0.1754460832792722 m (ours at most theirs)
- 95th percentile: core-f405 0.22612433204370755 m, ArduCopter 0.2208698311507864 m (ours above theirs)
- **fails:** the 95th percentile of core-f405 0.22612433204370755 is above ArduCopter's 0.2208698311507864

### B1: HOLDS

- gps-loss: core-f405 0 crash or fly-away, ArduCopter 0
- gps-loss-pilot: core-f405 0 crash or fly-away, ArduCopter 0
- gps-loss-recovers: core-f405 0 crash or fly-away, ArduCopter 0
- baro-loss: core-f405 0 crash or fly-away, ArduCopter 10
- mag-loss: core-f405 0 crash or fly-away, ArduCopter 0
- rc-loss-position: core-f405 0 crash or fly-away, ArduCopter 0
- rc-loss-override: core-f405 0 crash or fly-away, ArduCopter 0
- rc-loss-on-ground: core-f405 0 crash or fly-away, ArduCopter 0
- rc-loss-offboard-gcs: core-f405 0 crash or fly-away, ArduCopter 0
- gcs-loss: core-f405 0 crash or fly-away, ArduCopter 0
- no-human-link: core-f405 0 crash or fly-away, ArduCopter 0
- battery-low: core-f405 0 crash or fly-away, ArduCopter 0
- battery-critical: core-f405 0 crash or fly-away, ArduCopter 0
- geofence: core-f405 0 crash or fly-away, ArduCopter 0
- brain-crash: core-f405 0 crash or fly-away, ArduCopter 0
- brain-hang: core-f405 0 crash or fly-away, ArduCopter 0
- setpoint-gap: core-f405 0 crash or fly-away, ArduCopter 0
- pair-battery-critical+baro-loss: core-f405 0 crash or fly-away, ArduCopter 10
- pair-battery-critical+brain-crash: core-f405 0 crash or fly-away, ArduCopter 0
- pair-battery-critical+geofence: core-f405 0 crash or fly-away, ArduCopter 0
- pair-battery-low+baro-loss: core-f405 0 crash or fly-away, ArduCopter 10
- pair-battery-low+battery-critical: core-f405 0 crash or fly-away, ArduCopter 0
- pair-battery-low+brain-crash: core-f405 0 crash or fly-away, ArduCopter 0
- pair-battery-low+geofence: core-f405 0 crash or fly-away, ArduCopter 0
- pair-brain-crash+baro-loss: core-f405 0 crash or fly-away, ArduCopter 10
- pair-brain-crash+geofence: core-f405 0 crash or fly-away, ArduCopter 0
- pair-geofence+baro-loss: core-f405 0 crash or fly-away, ArduCopter 10
- pair-gps-loss+baro-loss: core-f405 0 crash or fly-away, ArduCopter 10
- pair-gps-loss+battery-critical: core-f405 0 crash or fly-away, ArduCopter 0
- pair-gps-loss+battery-low: core-f405 0 crash or fly-away, ArduCopter 0
- pair-gps-loss+brain-crash: core-f405 0 crash or fly-away, ArduCopter 0
- pair-gps-loss+geofence: core-f405 0 crash or fly-away, ArduCopter 0
- pair-rc-loss+baro-loss: core-f405 0 crash or fly-away, ArduCopter 0
- pair-rc-loss+battery-critical: core-f405 0 crash or fly-away, ArduCopter 0
- pair-rc-loss+battery-low: core-f405 0 crash or fly-away, ArduCopter 0
- pair-rc-loss+brain-crash: core-f405 0 crash or fly-away, ArduCopter 0
- pair-rc-loss+geofence: core-f405 0 crash or fly-away, ArduCopter 0
- pair-rc-loss+gps-loss: core-f405 0 crash or fly-away, ArduCopter 0

### B2: HOLDS

- gps-loss: core-f405 10 of 10 flights safe and pass
- gps-loss-pilot: core-f405 10 of 10 flights safe and pass
- gps-loss-recovers: core-f405 10 of 10 flights safe and pass
- baro-loss: core-f405 10 of 10 flights safe and pass
- mag-loss: core-f405 10 of 10 flights safe and pass
- rc-loss-position: core-f405 10 of 10 flights safe and pass
- rc-loss-override: core-f405 10 of 10 flights safe and pass
- rc-loss-on-ground: core-f405 10 of 10 flights safe and pass
- rc-loss-offboard-gcs: core-f405 10 of 10 flights safe and pass
- gcs-loss: core-f405 10 of 10 flights safe and pass
- no-human-link: core-f405 10 of 10 flights safe and pass
- battery-low: core-f405 10 of 10 flights safe and pass
- battery-critical: core-f405 10 of 10 flights safe and pass
- geofence: core-f405 10 of 10 flights safe and pass
- brain-crash: core-f405 10 of 10 flights safe and pass
- brain-hang: core-f405 10 of 10 flights safe and pass
- setpoint-gap: core-f405 10 of 10 flights safe and pass
- pair-battery-critical+baro-loss: core-f405 10 of 10 flights safe and pass
- pair-battery-critical+brain-crash: core-f405 10 of 10 flights safe and pass
- pair-battery-critical+geofence: core-f405 10 of 10 flights safe and pass
- pair-battery-low+baro-loss: core-f405 10 of 10 flights safe and pass
- pair-battery-low+battery-critical: core-f405 10 of 10 flights safe and pass
- pair-battery-low+brain-crash: core-f405 10 of 10 flights safe and pass
- pair-battery-low+geofence: core-f405 10 of 10 flights safe and pass
- pair-brain-crash+baro-loss: core-f405 10 of 10 flights safe and pass
- pair-brain-crash+geofence: core-f405 10 of 10 flights safe and pass
- pair-geofence+baro-loss: core-f405 10 of 10 flights safe and pass
- pair-gps-loss+baro-loss: core-f405 10 of 10 flights safe and pass
- pair-gps-loss+battery-critical: core-f405 10 of 10 flights safe and pass
- pair-gps-loss+battery-low: core-f405 10 of 10 flights safe and pass
- pair-gps-loss+brain-crash: core-f405 10 of 10 flights safe and pass
- pair-gps-loss+geofence: core-f405 10 of 10 flights safe and pass
- pair-rc-loss+baro-loss: core-f405 10 of 10 flights safe and pass
- pair-rc-loss+battery-critical: core-f405 10 of 10 flights safe and pass
- pair-rc-loss+battery-low: core-f405 10 of 10 flights safe and pass
- pair-rc-loss+brain-crash: core-f405 10 of 10 flights safe and pass
- pair-rc-loss+geofence: core-f405 10 of 10 flights safe and pass
- pair-rc-loss+gps-loss: core-f405 10 of 10 flights safe and pass

### B3: HOLDS

- motor-loss: median time from the failure to motors off, core-f405 0.9989999999999988 s, ArduCopter 14.999499999999998 s
- motor-loss: core-h743 (reported, decides nothing) 0.9997499999999988 s
- imu-loss: median time from the failure to motors off, core-f405 0.13899999999999935 s, ArduCopter 2.0015 s
- imu-loss: core-h743 (reported, decides nothing) 0.13974999999999937 s

### Repeats: HOLDS

- 10 flights flown twice, 10 repeat bit for bit and match the main table

## Criterion A: holding position in gusts

RMS horizontal error from the hold point, m. ArduCopter's statistics are over its safe flights only; the others are over all their flights.

| autopilot | flights | not safe | counted | median | 95th percentile | mean | worst | vertical RMS median | vertical RMS worst | tilt median (deg) | tilt worst (deg) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| arducopter | 500 | 0 | 500 | 0.1754 | 0.2209 | 0.1765 | 0.2907 | 0.0933 | 0.1834 | 6.6589 | 15.0249 |
| core-f405 | 500 | 0 | 500 | 0.1714 | 0.2261 | 0.1738 | 0.2966 | 0.0628 | 0.1739 | 9.3590 | 21.6564 |
| core-h743 | 500 | 0 | 500 | 0.1570 | 0.2116 | 0.1593 | 0.2502 | 0.0495 | 0.1565 | 8.7217 | 18.7231 |
| core-f405-truth | 500 | 0 | 500 | 0.0301 | 0.0435 | 0.0308 | 0.0573 | 0.0019 | 0.0096 | 5.9276 | 15.4402 |

Lower `rms_h_m`: core-f405 in 266 indices, ArduCopter in 234, equal in 0.

![cumulative curves of rms_h_m](gate-1-cumulative.svg)

![rms_h_m of each index, ArduCopter across, core-f405 up](gate-1-scatter.svg)

The curves and the points are over ArduCopter's safe flights: an index where its flight is not safe is not drawn.

## Criterion B: failures

Classes are safe / crash / fly-away. "Passed" counts the rows whose verdict column says pass.

| scenario | scored | core-f405 | core-f405 passed | core-h743 | core-h743 passed | ArduCopter |
|---|---|---|---|---|---|---|
| gps-loss | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| gps-loss-pilot | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| gps-loss-recovers | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| baro-loss | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 0 / 0 / 10 |
| mag-loss | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| rc-loss-position | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| rc-loss-override | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| rc-loss-on-ground | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| rc-loss-offboard-gcs | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| gcs-loss | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| no-human-link | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| battery-low | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| battery-critical | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| geofence | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| brain-crash | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| brain-hang | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| setpoint-gap | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-battery-critical+baro-loss | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 0 / 10 / 0 |
| pair-battery-critical+brain-crash | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-battery-critical+geofence | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-battery-low+baro-loss | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 0 / 0 / 10 |
| pair-battery-low+battery-critical | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-battery-low+brain-crash | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-battery-low+geofence | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-brain-crash+baro-loss | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 0 / 0 / 10 |
| pair-brain-crash+geofence | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-geofence+baro-loss | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 0 / 0 / 10 |
| pair-gps-loss+baro-loss | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 0 / 10 / 0 |
| pair-gps-loss+battery-critical | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-gps-loss+battery-low | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-gps-loss+brain-crash | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-gps-loss+geofence | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-rc-loss+baro-loss | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-rc-loss+battery-critical | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-rc-loss+battery-low | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-rc-loss+brain-crash | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-rc-loss+geofence | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| pair-rc-loss+gps-loss | on the outcome | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 | 10 of 10 | 10 / 0 / 0 |
| motor-loss | on the time to motors off | 0 / 10 / 0 | 10 of 10 | 0 / 10 / 0 | 10 of 10 | 0 / 10 / 0 |
| imu-loss | on the time to motors off | 0 / 10 / 0 | 10 of 10 | 0 / 10 / 0 | 10 of 10 | 0 / 10 / 0 |
| kill-in-rtl | not scored | 0 / 10 / 0 | 10 of 10 | 0 / 10 / 0 | 10 of 10 | 0 / 10 / 0 |
| hover-step | not scored | 1 / 0 / 0 | 1 of 1 | 1 / 0 / 0 | 1 of 1 | 1 / 0 / 0 |

## Time from the failure to motors off

| scenario | failure at (s) | core-f405 median (s) | core-h743 median (s) | ArduCopter median (s) |
|---|---|---|---|---|
| motor-loss | 15 | 0.9990 | 0.9997 | 14.9995 |
| imu-loss | 15 | 0.1390 | 0.1397 | 2.0015 |

## Reported for each scenario, deciding nothing

One row for each scenario but gust-hold and each autopilot, over every flight of that table: crashes and fly-aways are in these rows. "flights" is the rows of the table; "touched down" the rows that have a touchdown speed (m/s), and the median and worst (the largest) are over those. The end distance (m) is the horizontal distance of the end position from the arming point, sqrt(end north squared + end east squared), of every row that has both: where the flight ended, whether or not it landed. The greatest tilt (deg) is `max_tilt_deg` of every row that has one. "disarmed" counts the rows that have a disarm time, and the median and latest are over those; the times are flight time, and each scenario file names when its failure begins. The modes each flight went through are in its `audit.txt`. A `-` is a value that no row has. These numbers decide nothing.

| scenario | autopilot | flights | touched down | touchdown speed median (m/s) | touchdown speed worst (m/s) | end distance median (m) | end distance worst (m) | greatest tilt median (deg) | greatest tilt worst (deg) | disarmed | disarmed at median (s) | disarmed at latest (s) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| gps-loss | core-f405 | 10 | 10 | 0.7150 | 0.7603 | 6.6955 | 24.0677 | 7.4823 | 11.2100 | 10 | 35.1600 | 35.9800 |
| gps-loss | core-h743 | 10 | 10 | 0.7061 | 0.7292 | 8.6540 | 20.4933 | 7.7368 | 10.4967 | 10 | 35.2800 | 35.5200 |
| gps-loss | arducopter | 10 | 10 | 0.4997 | 0.5456 | 11.3494 | 27.6947 | 3.7362 | 7.0888 | 10 | 39.8335 | 40.1460 |
| gps-loss-pilot | core-f405 | 10 | 0 | - | - | 14.3935 | 31.8950 | 16.9780 | 29.1620 | 0 | - | - |
| gps-loss-pilot | core-h743 | 10 | 0 | - | - | 14.9150 | 27.3934 | 17.9803 | 28.4763 | 0 | - | - |
| gps-loss-pilot | arducopter | 10 | 10 | 0.4964 | 0.5607 | 6.3954 | 13.3621 | 3.4533 | 4.3773 | 10 | 41.0622 | 41.6935 |
| gps-loss-recovers | core-f405 | 10 | 0 | - | - | 0.1457 | 0.2274 | 10.1912 | 20.6294 | 0 | - | - |
| gps-loss-recovers | core-h743 | 10 | 0 | - | - | 0.1146 | 0.2385 | 10.7470 | 18.5752 | 0 | - | - |
| gps-loss-recovers | arducopter | 10 | 0 | - | - | 0.1489 | 0.2484 | 6.2822 | 7.7691 | 0 | - | - |
| baro-loss | core-f405 | 10 | 10 | 0.7011 | 0.7844 | 0.1103 | 0.3411 | 7.0710 | 10.7973 | 10 | 29.0100 | 30.0400 |
| baro-loss | core-h743 | 10 | 10 | 0.7003 | 0.7295 | 0.1395 | 0.4079 | 6.3822 | 11.3868 | 10 | 29.0100 | 29.4800 |
| baro-loss | arducopter | 10 | 0 | - | - | 1.2427 | 1.9525 | 3.6905 | 7.7535 | 0 | - | - |
| mag-loss | core-f405 | 10 | 10 | 0.6822 | 0.7428 | 3.0060 | 4.6154 | 7.9072 | 10.7302 | 10 | 53.9800 | 54.1600 |
| mag-loss | core-h743 | 10 | 10 | 0.7019 | 0.7249 | 3.3089 | 4.3908 | 7.8280 | 10.1394 | 10 | 53.9900 | 54.1200 |
| mag-loss | arducopter | 10 | 0 | - | - | 0.1624 | 0.3217 | 3.7378 | 5.5547 | 0 | - | - |
| rc-loss-position | core-f405 | 10 | 10 | 0.7024 | 0.7623 | 0.1692 | 0.3889 | 35.0587 | 35.6045 | 10 | 51.9900 | 52.5400 |
| rc-loss-position | core-h743 | 10 | 10 | 0.7125 | 0.7385 | 0.2135 | 0.3131 | 35.1429 | 35.4302 | 10 | 52.0000 | 52.2000 |
| rc-loss-position | arducopter | 10 | 10 | 0.4880 | 0.5142 | 0.1516 | 0.3308 | 20.5416 | 21.3972 | 10 | 73.5497 | 73.9485 |
| rc-loss-override | core-f405 | 10 | 0 | - | - | 7.0462 | 7.2691 | 35.1010 | 35.7940 | 0 | - | - |
| rc-loss-override | core-h743 | 10 | 0 | - | - | 6.9986 | 7.3054 | 35.0768 | 35.2618 | 0 | - | - |
| rc-loss-override | arducopter | 10 | 0 | - | - | 6.9585 | 7.0777 | 21.2248 | 23.1766 | 0 | - | - |
| rc-loss-on-ground | core-f405 | 10 | 0 | - | - | 0.0007 | 0.0019 | 0.0000 | 0.0029 | 10 | 6.0000 | 6.0000 |
| rc-loss-on-ground | core-h743 | 10 | 0 | - | - | 0.0008 | 0.0018 | 0.0002 | 0.0030 | 10 | 6.0000 | 6.0000 |
| rc-loss-on-ground | arducopter | 10 | 0 | - | - | 0.0005 | 0.0013 | 0.0000 | 0.0000 | 10 | 6.9835 | 6.9835 |
| rc-loss-offboard-gcs | core-f405 | 10 | 0 | - | - | 0.1305 | 0.2416 | 8.1938 | 17.0812 | 0 | - | - |
| rc-loss-offboard-gcs | core-h743 | 10 | 0 | - | - | 0.1270 | 0.3237 | 7.6164 | 13.7670 | 0 | - | - |
| rc-loss-offboard-gcs | arducopter | 10 | 10 | 0.5127 | 0.5355 | 0.1441 | 0.3814 | 4.0205 | 7.2355 | 10 | 35.9910 | 36.3285 |
| gcs-loss | core-f405 | 10 | 0 | - | - | 0.1926 | 0.2738 | 8.8744 | 12.7308 | 0 | - | - |
| gcs-loss | core-h743 | 10 | 0 | - | - | 0.1278 | 0.3128 | 8.4295 | 10.8872 | 0 | - | - |
| gcs-loss | arducopter | 10 | 10 | 0.5041 | 0.5166 | 0.1905 | 0.3774 | 4.6426 | 6.3612 | 10 | 39.1598 | 39.6635 |
| no-human-link | core-f405 | 10 | 10 | 0.7082 | 0.7648 | 0.3144 | 0.5286 | 9.1761 | 15.9370 | 10 | 40.7300 | 40.9000 |
| no-human-link | core-h743 | 10 | 10 | 0.7111 | 0.7354 | 0.2327 | 0.4381 | 9.0270 | 14.7371 | 10 | 40.7800 | 41.0400 |
| no-human-link | arducopter | 10 | 10 | 0.5220 | 0.5444 | 0.1796 | 0.2926 | 3.6961 | 5.0418 | 10 | 35.8885 | 36.3910 |
| battery-low | core-f405 | 10 | 10 | 0.7069 | 0.7466 | 0.1638 | 0.3360 | 35.0665 | 35.2330 | 10 | 47.6000 | 48.3000 |
| battery-low | core-h743 | 10 | 10 | 0.6975 | 0.7207 | 0.1896 | 0.3596 | 34.8663 | 35.1638 | 10 | 47.6800 | 48.0600 |
| battery-low | arducopter | 10 | 0 | - | - | 0.1484 | 0.4064 | 20.5032 | 22.8366 | 0 | - | - |
| battery-critical | core-f405 | 10 | 10 | 0.7070 | 0.7475 | 0.2434 | 0.3956 | 8.0459 | 11.0226 | 10 | 20.5400 | 20.8600 |
| battery-critical | core-h743 | 10 | 10 | 0.7003 | 0.7122 | 0.1246 | 0.4325 | 6.5393 | 10.5020 | 10 | 20.4200 | 20.7400 |
| battery-critical | arducopter | 10 | 10 | 0.5333 | 0.5649 | 0.1410 | 0.3727 | 3.4279 | 5.5151 | 10 | 43.6260 | 43.8785 |
| geofence | core-f405 | 10 | 10 | 0.7039 | 0.7318 | 0.1419 | 0.3417 | 34.9865 | 35.4074 | 10 | 40.2200 | 40.4200 |
| geofence | core-h743 | 10 | 10 | 0.7083 | 0.7271 | 0.2091 | 0.2797 | 34.9808 | 35.2727 | 10 | 40.0400 | 40.3200 |
| geofence | arducopter | 10 | 0 | - | - | 0.1127 | 0.2240 | 4.1071 | 7.0368 | 0 | - | - |
| brain-crash | core-f405 | 10 | 10 | 0.7036 | 0.7369 | 0.2165 | 0.5183 | 34.8737 | 35.1996 | 10 | 55.8500 | 56.7800 |
| brain-crash | core-h743 | 10 | 10 | 0.7090 | 0.7409 | 0.2351 | 0.3446 | 34.9358 | 34.9956 | 10 | 55.7600 | 56.0400 |
| brain-crash | arducopter | 10 | 0 | - | - | 15.0563 | 15.2503 | 20.8562 | 22.7500 | 0 | - | - |
| brain-hang | core-f405 | 10 | 10 | 0.7083 | 0.7549 | 0.2400 | 0.4064 | 34.9866 | 35.3472 | 10 | 55.8600 | 56.3600 |
| brain-hang | core-h743 | 10 | 10 | 0.7062 | 0.7381 | 0.1516 | 0.3543 | 34.9466 | 35.2703 | 10 | 55.7800 | 56.0600 |
| brain-hang | arducopter | 10 | 0 | - | - | 15.0259 | 15.1875 | 19.7308 | 21.2492 | 0 | - | - |
| setpoint-gap | core-f405 | 10 | 0 | - | - | 0.1568 | 0.3383 | 8.2814 | 15.0126 | 0 | - | - |
| setpoint-gap | core-h743 | 10 | 0 | - | - | 0.2020 | 0.3087 | 7.6685 | 12.9440 | 0 | - | - |
| setpoint-gap | arducopter | 10 | 0 | - | - | 0.1664 | 0.3380 | 3.7508 | 5.1350 | 0 | - | - |
| pair-battery-critical+baro-loss | core-f405 | 10 | 10 | 0.6990 | 0.7548 | 0.2285 | 0.4042 | 8.6082 | 10.9355 | 10 | 22.1600 | 22.8200 |
| pair-battery-critical+baro-loss | core-h743 | 10 | 10 | 0.6991 | 0.7467 | 0.2047 | 0.3654 | 8.3976 | 10.0532 | 10 | 22.1800 | 22.5400 |
| pair-battery-critical+baro-loss | arducopter | 10 | 10 | 3.0409 | 3.0560 | 3.3489 | 5.3446 | 3.5966 | 7.4188 | 10 | 47.2610 | 47.3910 |
| pair-battery-critical+brain-crash | core-f405 | 10 | 10 | 0.6837 | 0.7463 | 0.1720 | 0.3271 | 8.8686 | 15.0475 | 10 | 22.1400 | 22.3600 |
| pair-battery-critical+brain-crash | core-h743 | 10 | 10 | 0.6999 | 0.7209 | 0.1978 | 0.3805 | 8.9259 | 14.4328 | 10 | 22.1500 | 22.2400 |
| pair-battery-critical+brain-crash | arducopter | 10 | 10 | 0.4823 | 0.5047 | 0.1669 | 0.3869 | 3.8776 | 5.0248 | 10 | 45.1572 | 45.3185 |
| pair-battery-critical+geofence | core-f405 | 10 | 10 | 0.6982 | 0.7136 | 0.9139 | 1.2732 | 34.8373 | 35.2269 | 10 | 39.3000 | 39.8000 |
| pair-battery-critical+geofence | core-h743 | 10 | 10 | 0.7041 | 0.7200 | 0.8891 | 1.2079 | 34.8635 | 35.2614 | 10 | 39.1400 | 39.4400 |
| pair-battery-critical+geofence | arducopter | 10 | 10 | 0.5150 | 0.5374 | 0.1480 | 0.2614 | 3.3124 | 4.3137 | 10 | 49.9672 | 50.1360 |
| pair-battery-low+baro-loss | core-f405 | 10 | 10 | 0.7103 | 0.7335 | 0.1919 | 0.3829 | 7.1990 | 9.6299 | 10 | 28.1600 | 28.9800 |
| pair-battery-low+baro-loss | core-h743 | 10 | 10 | 0.6903 | 0.7183 | 0.2276 | 0.3382 | 6.7787 | 8.8695 | 10 | 28.1300 | 28.6600 |
| pair-battery-low+baro-loss | arducopter | 10 | 0 | - | - | 6.9809 | 11.6031 | 4.5293 | 7.7652 | 0 | - | - |
| pair-battery-low+battery-critical | core-f405 | 10 | 10 | 0.6887 | 0.7167 | 0.2616 | 0.3610 | 9.8124 | 12.1910 | 10 | 23.4700 | 24.1000 |
| pair-battery-low+battery-critical | core-h743 | 10 | 10 | 0.7116 | 0.7250 | 0.1941 | 0.4280 | 9.6430 | 10.8854 | 10 | 23.3700 | 23.8000 |
| pair-battery-low+battery-critical | arducopter | 10 | 10 | 0.4909 | 0.5038 | 0.1663 | 0.4302 | 3.8852 | 5.4482 | 10 | 46.2548 | 46.5385 |
| pair-battery-low+brain-crash | core-f405 | 10 | 10 | 0.7001 | 0.7487 | 0.1554 | 0.2589 | 8.2469 | 12.7747 | 10 | 23.3500 | 23.6400 |
| pair-battery-low+brain-crash | core-h743 | 10 | 10 | 0.7192 | 0.7444 | 0.1583 | 0.3214 | 7.1098 | 11.8750 | 10 | 23.2800 | 23.5600 |
| pair-battery-low+brain-crash | arducopter | 10 | 10 | 0.5224 | 0.5761 | 0.2458 | 0.3932 | 4.4205 | 6.2121 | 10 | 44.4910 | 44.6885 |
| pair-battery-low+geofence | core-f405 | 10 | 10 | 0.7124 | 0.7699 | 0.1802 | 0.3743 | 35.0360 | 35.4288 | 10 | 40.1700 | 40.7400 |
| pair-battery-low+geofence | core-h743 | 10 | 10 | 0.7017 | 0.7261 | 0.1532 | 0.4287 | 35.0754 | 35.3572 | 10 | 40.0800 | 40.2800 |
| pair-battery-low+geofence | arducopter | 10 | 10 | 0.5054 | 0.5239 | 0.2199 | 0.3434 | 4.1463 | 6.8141 | 10 | 49.2110 | 49.5635 |
| pair-brain-crash+baro-loss | core-f405 | 10 | 10 | 0.7122 | 0.7658 | 0.2058 | 0.3467 | 7.2797 | 12.7446 | 10 | 34.1300 | 34.4800 |
| pair-brain-crash+baro-loss | core-h743 | 10 | 10 | 0.7141 | 0.7315 | 0.1458 | 0.2767 | 6.3125 | 11.2726 | 10 | 33.8000 | 34.1400 |
| pair-brain-crash+baro-loss | arducopter | 10 | 0 | - | - | 1.0501 | 2.4897 | 3.3720 | 4.7409 | 0 | - | - |
| pair-brain-crash+geofence | core-f405 | 10 | 10 | 0.7008 | 0.7369 | 0.1343 | 0.3806 | 34.9323 | 35.2360 | 10 | 40.0700 | 40.5200 |
| pair-brain-crash+geofence | core-h743 | 10 | 10 | 0.6982 | 0.7129 | 0.1521 | 0.3584 | 34.9019 | 35.0959 | 10 | 39.9000 | 40.2200 |
| pair-brain-crash+geofence | arducopter | 10 | 0 | - | - | 0.1723 | 0.2602 | 4.2233 | 6.1171 | 0 | - | - |
| pair-geofence+baro-loss | core-f405 | 10 | 10 | 0.7012 | 0.7597 | 0.1693 | 0.3320 | 34.9312 | 35.2121 | 10 | 40.2200 | 41.4400 |
| pair-geofence+baro-loss | core-h743 | 10 | 10 | 0.6926 | 0.7506 | 0.1385 | 0.3993 | 34.9284 | 35.3192 | 10 | 40.3000 | 41.0400 |
| pair-geofence+baro-loss | arducopter | 10 | 0 | - | - | 1.1347 | 2.2176 | 3.4756 | 6.4357 | 0 | - | - |
| pair-gps-loss+baro-loss | core-f405 | 10 | 10 | 0.6720 | 0.7964 | 8.3042 | 12.8177 | 6.6731 | 12.8514 | 10 | 34.3300 | 35.1600 |
| pair-gps-loss+baro-loss | core-h743 | 10 | 10 | 0.7057 | 0.7885 | 7.2745 | 17.6757 | 6.1033 | 12.3165 | 10 | 33.8800 | 34.8600 |
| pair-gps-loss+baro-loss | arducopter | 10 | 10 | 2.6053 | 2.6835 | 10.4228 | 25.7453 | 4.1614 | 6.6825 | 10 | 38.6210 | 39.1610 |
| pair-gps-loss+battery-critical | core-f405 | 10 | 10 | 0.7047 | 0.7653 | 2.1950 | 4.6803 | 7.4400 | 13.2207 | 10 | 25.4400 | 25.7400 |
| pair-gps-loss+battery-critical | core-h743 | 10 | 10 | 0.7255 | 0.7596 | 0.9512 | 3.8226 | 6.9811 | 12.4552 | 10 | 25.1700 | 25.5400 |
| pair-gps-loss+battery-critical | arducopter | 10 | 10 | 0.5285 | 0.5756 | 15.5955 | 27.3858 | 3.6737 | 5.2343 | 10 | 39.3697 | 39.9335 |
| pair-gps-loss+battery-low | core-f405 | 10 | 10 | 0.7044 | 0.7347 | 12.3621 | 17.6047 | 8.7379 | 15.5118 | 10 | 35.2100 | 35.5600 |
| pair-gps-loss+battery-low | core-h743 | 10 | 10 | 0.7024 | 0.7429 | 9.5140 | 16.2308 | 8.0078 | 14.4412 | 10 | 35.3600 | 35.6600 |
| pair-gps-loss+battery-low | arducopter | 10 | 10 | 0.5068 | 0.5311 | 15.2506 | 21.8363 | 3.5458 | 5.9560 | 10 | 39.6972 | 40.4810 |
| pair-gps-loss+brain-crash | core-f405 | 10 | 10 | 0.6979 | 0.7299 | 11.6961 | 24.3253 | 8.7261 | 15.5136 | 10 | 35.2700 | 35.5200 |
| pair-gps-loss+brain-crash | core-h743 | 10 | 10 | 0.7110 | 0.7367 | 11.4099 | 20.5154 | 7.5539 | 15.0011 | 10 | 35.2300 | 35.4200 |
| pair-gps-loss+brain-crash | arducopter | 10 | 10 | 0.4998 | 0.5998 | 17.3791 | 30.8363 | 3.8546 | 7.3702 | 10 | 39.5147 | 40.0885 |
| pair-gps-loss+geofence | core-f405 | 10 | 10 | 0.7125 | 0.7498 | 14.1845 | 30.9095 | 35.0227 | 35.3226 | 10 | 47.4500 | 47.7400 |
| pair-gps-loss+geofence | core-h743 | 10 | 10 | 0.6978 | 0.7261 | 20.1534 | 32.0415 | 34.9116 | 35.1900 | 10 | 47.4700 | 47.6000 |
| pair-gps-loss+geofence | arducopter | 10 | 10 | 0.5128 | 0.5781 | 14.2868 | 20.8249 | 3.3144 | 6.8453 | 10 | 44.7360 | 45.2360 |
| pair-rc-loss+baro-loss | core-f405 | 10 | 10 | 0.6972 | 0.7534 | 0.2173 | 0.4598 | 6.9008 | 9.4246 | 10 | 28.7000 | 29.5000 |
| pair-rc-loss+baro-loss | core-h743 | 10 | 10 | 0.6893 | 0.7220 | 0.1554 | 0.3430 | 6.7062 | 9.1677 | 10 | 28.7900 | 29.0200 |
| pair-rc-loss+baro-loss | arducopter | 10 | 10 | 1.7293 | 1.7992 | 0.6584 | 1.0474 | 3.6420 | 5.7174 | 10 | 37.9147 | 38.0635 |
| pair-rc-loss+battery-critical | core-f405 | 10 | 10 | 0.7023 | 0.7440 | 0.2082 | 0.3612 | 9.3661 | 13.1706 | 10 | 22.2800 | 22.5600 |
| pair-rc-loss+battery-critical | core-h743 | 10 | 10 | 0.6991 | 0.7171 | 0.1911 | 0.3350 | 8.0912 | 11.1438 | 10 | 22.2200 | 22.4400 |
| pair-rc-loss+battery-critical | arducopter | 10 | 10 | 0.4949 | 0.5188 | 0.1556 | 0.2733 | 3.4989 | 5.3274 | 10 | 39.6260 | 40.3935 |
| pair-rc-loss+battery-low | core-f405 | 10 | 10 | 0.7131 | 0.7480 | 0.2203 | 0.4486 | 8.8588 | 12.7162 | 10 | 27.8700 | 28.3800 |
| pair-rc-loss+battery-low | core-h743 | 10 | 10 | 0.7118 | 0.7288 | 0.2090 | 0.3263 | 8.4727 | 11.6046 | 10 | 28.0100 | 28.5200 |
| pair-rc-loss+battery-low | arducopter | 10 | 10 | 0.5118 | 0.5464 | 0.1710 | 0.2382 | 4.8192 | 6.3453 | 10 | 35.9235 | 36.4735 |
| pair-rc-loss+brain-crash | core-f405 | 10 | 10 | 0.7030 | 0.7251 | 0.2074 | 0.3924 | 8.7704 | 14.5065 | 10 | 28.7200 | 29.1400 |
| pair-rc-loss+brain-crash | core-h743 | 10 | 10 | 0.7048 | 0.7303 | 0.2146 | 0.2869 | 8.6592 | 12.5174 | 10 | 28.7000 | 28.8200 |
| pair-rc-loss+brain-crash | arducopter | 10 | 10 | 0.5142 | 0.5390 | 0.1771 | 0.4077 | 3.8550 | 4.2364 | 10 | 35.9847 | 36.4510 |
| pair-rc-loss+geofence | core-f405 | 10 | 10 | 0.6897 | 0.7140 | 0.1664 | 0.3327 | 35.0190 | 35.2309 | 10 | 40.2700 | 40.7200 |
| pair-rc-loss+geofence | core-h743 | 10 | 10 | 0.7099 | 0.7327 | 0.1693 | 0.5421 | 34.8890 | 35.1805 | 10 | 40.1400 | 40.4200 |
| pair-rc-loss+geofence | arducopter | 10 | 10 | 0.4899 | 0.5269 | 0.1665 | 0.4023 | 3.9672 | 5.9401 | 10 | 40.7735 | 41.3685 |
| pair-rc-loss+gps-loss | core-f405 | 10 | 10 | 0.7049 | 0.7566 | 10.6099 | 19.3253 | 7.8059 | 12.4879 | 10 | 39.0100 | 39.2200 |
| pair-rc-loss+gps-loss | core-h743 | 10 | 10 | 0.7045 | 0.7320 | 9.7116 | 14.1717 | 7.2433 | 12.3472 | 10 | 39.0200 | 39.2400 |
| pair-rc-loss+gps-loss | arducopter | 10 | 10 | 0.4823 | 0.5528 | 7.0378 | 12.8821 | 4.2451 | 7.0083 | 10 | 40.2073 | 41.1835 |
| motor-loss | core-f405 | 10 | 10 | 7.8248 | 7.9997 | 4.6716 | 5.1043 | 143.4075 | 144.6172 | 10 | 15.9800 | 15.9800 |
| motor-loss | core-h743 | 10 | 10 | 7.6986 | 7.9298 | 4.8888 | 5.3000 | 143.3347 | 143.9493 | 10 | 15.9800 | 15.9800 |
| motor-loss | arducopter | 10 | 10 | 7.7129 | 8.5968 | 52.3306 | 259.4860 | 105.9597 | 179.5237 | 0 | - | - |
| imu-loss | core-f405 | 10 | 10 | 9.4853 | 9.6817 | 0.1214 | 0.2790 | 9.3116 | 15.1985 | 10 | 15.1200 | 15.1200 |
| imu-loss | core-h743 | 10 | 10 | 9.4715 | 9.5369 | 0.0789 | 0.2923 | 8.8850 | 13.9865 | 10 | 15.1200 | 15.1200 |
| imu-loss | arducopter | 10 | 10 | 11.0524 | 12.4632 | 5.1718 | 8.1264 | 178.2288 | 179.7101 | 10 | 17.0020 | 17.0020 |
| kill-in-rtl | core-f405 | 10 | 10 | 12.7680 | 12.8384 | 1.6355 | 2.0528 | 35.0947 | 35.5057 | 10 | 34.0400 | 34.0400 |
| kill-in-rtl | core-h743 | 10 | 10 | 12.7516 | 12.8249 | 1.6461 | 2.0993 | 34.9199 | 35.6191 | 10 | 34.0400 | 34.0400 |
| kill-in-rtl | arducopter | 10 | 10 | 15.4160 | 15.5849 | 20.2042 | 21.1899 | 20.0569 | 21.9980 | 10 | 34.2260 | 34.2260 |
| hover-step | core-f405 | 1 | 0 | - | - | 1.4171 | 1.4171 | 28.4244 | 28.4244 | 0 | - | - |
| hover-step | core-h743 | 1 | 0 | - | - | 1.4169 | 1.4169 | 27.3318 | 27.3318 | 0 | - | - |
| hover-step | arducopter | 1 | 0 | - | - | 1.3785 | 1.3785 | 12.0858 | 12.0858 | 0 | - | - |

## Flights flown twice

| scenario | index | repeats | hashes as in the main table |
|---|---|---|---|
| gust-hold | 0 | yes | yes |
| gust-hold | 1 | yes | yes |
| gust-hold | 2 | yes | yes |
| gust-hold | 3 | yes | yes |
| gust-hold | 4 | yes | yes |
| gps-loss | 0 | yes | yes |
| battery-low | 0 | yes | yes |
| rc-loss-offboard-gcs | 0 | yes | yes |
| imu-loss | 0 | yes | yes |
| pair-gps-loss+baro-loss | 0 | yes | yes |

## gate.txt

```text
oxide-bridge gate 1
commit 2c2fedb1dab3c65f8d52a431b38b369e9ffa4091
tree clean
scope full
binary sha256 caed35e4cb796ec06cac40b09566a18251771486b1cbac5e2bf16b7a5ed5045d
binary path /tmp/claude-0/-home-user-airborne-oxide/f85058bd-73c8-560d-8ff0-64221580022a/scratchpad/m1d/ardupilot-sitl-4.7.2/arducopter
command line oxide-bridge gate fly --scenarios scenarios --out /tmp/claude-0/-home-user-airborne-oxide/f85058bd-73c8-560d-8ff0-64221580022a/scratchpad/m1d/gate1 --binary /tmp/claude-0/-home-user-airborne-oxide/f85058bd-73c8-560d-8ff0-64221580022a/scratchpad/m1d/ardupilot-sitl-4.7.2/arducopter --jobs 3
first started 2026-11-18T14:05:27Z
started 2026-11-18T14:05:27Z
ended 2026-11-18T14:32:23Z
cores 4
param file base.parm sha256 ee7d4657a954e50ee258400d07f606ecab9a0fd7761879096aab8a57523b11a5
param file setup.parm sha256 0c5364d91dd6a73c751462096340f58eee29909bbc999535d346abec51e8a108
param file tuned-gains.parm sha256 e21a80edde40b189a8dc7a1b8f5575a7c76579f29608706ebff96dc09c6f69cc
param file failsafes.parm sha256 46cae574647ce9e10471c14f797830028044864abbe1826897b387582fd1bab0
scenario file gust-hold sha256 3482d54e847f3fbff1c9999884df507059e9eb5dc061ff48eb75a01dcb132798
scenario file gps-loss sha256 22dcc18486497d8b450ee6c5046337a2386712d1c37b60d8de21a95bdec91023
scenario file gps-loss-pilot sha256 dec00f25674c75f011dff37b5f53353c7628e4be0885829f8e9b704d0469cc0b
scenario file gps-loss-recovers sha256 82f2b46f16c78975f4ab5e47570ae3f50ee9ab6d24f2e2db852b75f1e0a6a1fd
scenario file baro-loss sha256 311fbfbb5f4cf757b8f8b02cface000cdcc8f6858375bdfc3ce9da17048e04fc
scenario file mag-loss sha256 14ea6bf404def353c6b84a2b643d581d85bb5d50c7a8cf68e67ffc3fd427bbe1
scenario file rc-loss-position sha256 b24b802472a0f71c26c2ed6f27b473f8ec975e9fdca00997e864d0f69b1c77cd
scenario file rc-loss-override sha256 2dfb3a602dca534acdcf4de2d555dbc25dda1990774a2a942cd700f9a6ccc273
scenario file rc-loss-on-ground sha256 53850c401fa99a7f3203c159c9c4b82be28ff7ed9eafd48bd5914099513ee00d
scenario file rc-loss-offboard-gcs sha256 c67cd761bf8d27713b47b4939c0eca2e792b537ed64f9050138045f187ecf12c
scenario file gcs-loss sha256 5cc64e04e94b63702dcb4bb6c5dd544d5c84b6d64cd81f27a57d0342bb37b7f9
scenario file no-human-link sha256 cd54047146fff3a6c1196eaf9c38d875f36df44230cb970ac0fc6a9a85cc99e4
scenario file battery-low sha256 d87b40d8660dc1b7a5d99392f55de1fd16e1758543da02266c79437df3e2cd84
scenario file battery-critical sha256 c7d26b82787a6125c0b14fa53a12c14d2072bda9c35e5f3a25fa1e1812e77475
scenario file geofence sha256 3b776affd16aca81dcf68f66d170371e57936a8a57367f557937e7bb850e718b
scenario file brain-crash sha256 8cdd0e4d72a53d8dde282854e14667f95b4099cc1e6e8b426989472ad543383b
scenario file brain-hang sha256 b2b012b86fa255d62c270fa5b569b65f7bc25b3b6c19f9470439598ecbe2ed2a
scenario file setpoint-gap sha256 c49cc1d13447bb21cfc11a101e190ed0a42a87cb06e9804fd8554395d8270987
scenario file pair-battery-critical+baro-loss sha256 3b09d29fdc7251671dedba1782d55438f8cb068565114c2ec355ec80982a5ad1
scenario file pair-battery-critical+brain-crash sha256 97072c0e3abc1405d867d47133717d64ba45a16b2f1c392bfecfdf9581b59d14
scenario file pair-battery-critical+geofence sha256 de6407ecfec042213e937fff8530d0844271782f172bea3eda3e4f064056c3d6
scenario file pair-battery-low+baro-loss sha256 950dd238982d6df4dfe7b451d0ad64589fa7a8908efce3d66b71e2775182625f
scenario file pair-battery-low+battery-critical sha256 a459580f118508b6b900f4f23fd74649738b7e78e9dbcc58832dabf8a49ee848
scenario file pair-battery-low+brain-crash sha256 6bbad8db9f55b444946f0512641be917561331f6142b91a2dfd967c25be3244d
scenario file pair-battery-low+geofence sha256 faa00d6b64475c4fac40d0157ac5e527cd2b923ec484f6fa6f659d1923bb858e
scenario file pair-brain-crash+baro-loss sha256 bc77db5368c3c3845bd4201338f987be6d64ec2ffc802c890a3274f65df969f4
scenario file pair-brain-crash+geofence sha256 8f76176ec9814ee2d49d61148418d142620db9e582ff1809d352014462ce6906
scenario file pair-geofence+baro-loss sha256 1d71a0c76afa4da026ac628bd1f49843d9ea35c7e981da5e5808e6934f6dd77d
scenario file pair-gps-loss+baro-loss sha256 4b177197169fdeb6afc1cf7d795b8d8160ece1893c95d50a269b9f8a7d74e68c
scenario file pair-gps-loss+battery-critical sha256 3475b75b938802c848c6e9c3aa503384d3b6c32406c2818110ee326f9ed7edcc
scenario file pair-gps-loss+battery-low sha256 f1af89afecfe2cc15d7a07b299eb8788b329b46b20cfc26c59cba70912712427
scenario file pair-gps-loss+brain-crash sha256 9993d376a440eeeb959aa212c95a3329de3b0e11caf389245345a8371844324b
scenario file pair-gps-loss+geofence sha256 f481bb997795159e64f09c1e1b655308321125cc1ff845c2418d8358cbda4cb4
scenario file pair-rc-loss+baro-loss sha256 396e119b6648fca70a9bd24f9a041471109bf3277fdc60d5ece61484f4477ff2
scenario file pair-rc-loss+battery-critical sha256 aba5b42f3eb037a4c2be71ddae08e29f1f527fed7e701d783c5efd83827b9fb6
scenario file pair-rc-loss+battery-low sha256 a0d7a1ef996b584cebb203ac87366b2a3ed697781daf6746040898d3e8f4c81d
scenario file pair-rc-loss+brain-crash sha256 9f9f07863445a659ccc4bd0a2a343311d05ae22d965573f74c9eb48c91727a05
scenario file pair-rc-loss+geofence sha256 d699e4ce2ea8c83e3767a9287f2d5f74886e98acee8b496e6427eb9cd0973ad5
scenario file pair-rc-loss+gps-loss sha256 9f76c5bef99f31e3e496762160fa1d0d8607a60408d933f955b7b991054f7577
scenario file motor-loss sha256 f63afaa7e0854922aa6420b23743f3fafc61cc11175f66cd42f7a00275fd20ca
scenario file imu-loss sha256 59d42bfc03860c50721af23f6a6700d32a6453129e3584781668e1d356889e89
scenario file kill-in-rtl sha256 50d79c1167dc94a8eb375f964b2b389dfee944bc75968076e7120a8e9d2dfbdb
scenario file hover-step sha256 3b9b44bb7e769a410ac70f035e2e9a9910821b2ddf7448d44f87d386e64d2f1d
plan gust gust-hold 500
plan outcome gps-loss 10
plan outcome gps-loss-pilot 10
plan outcome gps-loss-recovers 10
plan outcome baro-loss 10
plan outcome mag-loss 10
plan outcome rc-loss-position 10
plan outcome rc-loss-override 10
plan outcome rc-loss-on-ground 10
plan outcome rc-loss-offboard-gcs 10
plan outcome gcs-loss 10
plan outcome no-human-link 10
plan outcome battery-low 10
plan outcome battery-critical 10
plan outcome geofence 10
plan outcome brain-crash 10
plan outcome brain-hang 10
plan outcome setpoint-gap 10
plan outcome pair-battery-critical+baro-loss 10
plan outcome pair-battery-critical+brain-crash 10
plan outcome pair-battery-critical+geofence 10
plan outcome pair-battery-low+baro-loss 10
plan outcome pair-battery-low+battery-critical 10
plan outcome pair-battery-low+brain-crash 10
plan outcome pair-battery-low+geofence 10
plan outcome pair-brain-crash+baro-loss 10
plan outcome pair-brain-crash+geofence 10
plan outcome pair-geofence+baro-loss 10
plan outcome pair-gps-loss+baro-loss 10
plan outcome pair-gps-loss+battery-critical 10
plan outcome pair-gps-loss+battery-low 10
plan outcome pair-gps-loss+brain-crash 10
plan outcome pair-gps-loss+geofence 10
plan outcome pair-rc-loss+baro-loss 10
plan outcome pair-rc-loss+battery-critical 10
plan outcome pair-rc-loss+battery-low 10
plan outcome pair-rc-loss+brain-crash 10
plan outcome pair-rc-loss+geofence 10
plan outcome pair-rc-loss+gps-loss 10
plan motors-off motor-loss 10
plan motors-off imu-loss 10
plan not-scored kill-in-rtl 10
plan not-scored hover-step 1
twice gust-hold 5
twice gps-loss 1
twice battery-low 1
twice rc-loss-offboard-gcs 1
twice imu-loss 1
twice pair-gps-loss+baro-loss 1
pair pair-battery-critical+baro-loss baro-loss,battery
pair pair-battery-critical+brain-crash battery,brain-crash
pair pair-battery-critical+geofence battery,geofence
pair pair-battery-low+baro-loss baro-loss,battery
pair pair-battery-low+battery-critical battery
pair pair-battery-low+brain-crash battery,brain-crash
pair pair-battery-low+geofence battery,geofence
pair pair-brain-crash+baro-loss baro-loss,brain-crash
pair pair-brain-crash+geofence brain-crash,geofence
pair pair-geofence+baro-loss baro-loss,geofence
pair pair-gps-loss+baro-loss baro-loss,gps-loss
pair pair-gps-loss+battery-critical battery,gps-loss
pair pair-gps-loss+battery-low battery,gps-loss
pair pair-gps-loss+brain-crash brain-crash,gps-loss
pair pair-gps-loss+geofence geofence,gps-loss
pair pair-rc-loss+baro-loss baro-loss,rc-loss
pair pair-rc-loss+battery-critical battery,rc-loss
pair pair-rc-loss+battery-low battery,rc-loss
pair pair-rc-loss+brain-crash brain-crash,rc-loss
pair pair-rc-loss+geofence geofence,rc-loss
pair pair-rc-loss+gps-loss gps-loss,rc-loss
failure imu-loss 15
failure motor-loss 15
```
