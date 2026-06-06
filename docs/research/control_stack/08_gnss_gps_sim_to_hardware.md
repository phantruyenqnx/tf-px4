# GNSS / GPS in EKF2 — From Simulation to Real Hardware

> Companion to `02_ekf2.md`, `02c_ekf2_source_guide.md`, and `03_uwb_ekf2_implementation.md`.
> Focus: everything that touches `src/modules/ekf2/EKF/aid_sources/gnss/` plus the full pipeline that
> feeds it — from a fake fix in a unit test, through Gazebo/HIL SITL, to a real u-blox receiver on a UART.
>
> **Reference frames:** NED $\{W\}$, Body $\{B\}$ (FRD). Geodetic position is `(lat, lon, alt_MSL)`.
> Velocity is NED earth-frame. Heading/yaw is scalar-first Hamilton quaternion yaw, $\in[-\pi,\pi]$.

---

## 0. The one idea that makes everything simple

Whether GPS data comes from a real receiver, a Gazebo plugin, a MAVLink `HIL_GPS` packet, or a hand-coded
unit-test sample, **it all converges onto a single uORB topic and a single struct before EKF2 ever sees it.**

```
                          ┌─────────────────────────────────────────────┐
  REAL HARDWARE  ─────────┤                                             │
  (UART/SPI/CAN GPS)      │                                             │
                          │        sensor_gps  (uORB)                   │
  GAZEBO / GZ SITL ───────┤   ──►  vehicle_gps_position (uORB)  ──►  EKF2
  (plugin → HIL_GPS)      │        (after optional blending)            │
                          │                                             │
  UNIT-TEST SIM   ────────┤   (bypasses uORB: calls _ekf.setGpsData)    │
  (gnssSample direct)     │                                             │
                          └─────────────────────────────────────────────┘
```

So the EKF2 fusion code in `aid_sources/gnss/` is **source-agnostic**. It never knows or cares whether the
fix is real. That is why the same four `.cpp` files validate against unit tests, SITL flights, and real flights.

Two data types matter:

| Layer | Type | Defined in |
|---|---|---|
| Transport (driver/sim/CAN) | `sensor_gps_s` (uORB msg) | [`msg/SensorGps.msg`](../../../msg/SensorGps.msg) |
| EKF-internal | `gnssSample` (struct) | [`common.h:184`](../../../src/modules/ekf2/EKF/common.h#L184) |

The conversion from one to the other happens in exactly one place:
[`EKF2.cpp:2405-2451`](../../../src/modules/ekf2/EKF2.cpp#L2405) (`UpdateGpsSample`).

---

## 1. The two data structures

### 1.1 `sensor_gps_s` — the transport message

Published by **every** GPS source. Key fields (see [`msg/SensorGps.msg`](../../../msg/SensorGps.msg)):

| Field | Unit | Meaning |
|---|---|---|
| `timestamp` / `timestamp_sample` | µs | publish time / measurement time |
| `latitude_deg`, `longitude_deg` | deg (`float64`) | geodetic position |
| `altitude_msl_m`, `altitude_ellipsoid_m` | m (`float64`) | altitude above mean-sea-level / WGS-84 ellipsoid |
| `fix_type` | enum | 0/1 none, 2 = 2D, 3 = 3D, 4 = DGPS, 5 = RTK float, 6 = RTK fixed, 8 = extrapolated |
| `eph`, `epv` | m | 1-σ horizontal / vertical position error |
| `hdop`, `vdop` | – | horizontal / vertical dilution of precision |
| `s_variance_m_s` | m/s | speed accuracy (1-σ) |
| `c_variance_rad` | rad | course accuracy |
| `vel_n/e/d_m_s`, `vel_m_s` | m/s | NED velocity + ground speed |
| `vel_ned_valid` | bool | NED velocity usable |
| `cog_rad` | rad | course over ground |
| `satellites_used` | – | satellite count |
| `heading`, `heading_offset`, `heading_accuracy` | rad | dual-antenna yaw (NaN if none) |
| `spoofing_state`, `jamming_state`, `jamming_indicator` | enum | interference / spoof detection |
| `noise_per_ms`, `automatic_gain_control` | – | RF health |
| `rtcm_injection_rate`, `selected_rtcm_instance`, `rtcm_crc_failed` | – | RTK correction feedback |

### 1.2 `gnssSample` — what EKF2 actually fuses

[`common.h:184-200`](../../../src/modules/ekf2/EKF/common.h#L184) — a slimmed, EKF-ready subset:

```cpp
struct gnssSample {
    uint64_t time_us;   // measurement timestamp (µs)
    double   lat, lon;  // degrees
    float    alt;       // GNSS altitude above MSL (m)
    Vector3f vel;       // NED velocity (m/s)
    float    hacc, vacc;// 1-σ horizontal / vertical position error (m)
    float    sacc;      // 1-σ speed error (m/s)
    uint8_t  fix_type;  // 0-1 none, 2 = 2D, 3 = 3D, 4 = code-diff, 5 = RTK
    uint8_t  nsats;     // satellites used
    float    pdop;      // position dilution of precision
    float    yaw;       // dual-antenna yaw (NaN if none), rad [-PI, PI]
    float    yaw_acc;   // 1-σ yaw error (rad)
    float    yaw_offset;// dual-antenna mounting offset (rad)
    bool     spoofed;   // spoofing flag
};
```

### 1.3 The conversion ([`EKF2.cpp:2405-2451`](../../../src/modules/ekf2/EKF2.cpp#L2405))

Noteworthy transforms applied at the boundary:

- **PDOP is synthesised** from HDOP & VDOP: `pdop = sqrt(hdop² + vdop²)`.
- **`sacc = s_variance_m_s`** — the speed accuracy field is reused as speed 1-σ.
- **`spoofed`** is only `true` when `spoofing_state == SPOOFING_STATE_MULTIPLE` (the strongest level).
- **Velocity** only forwarded when `vel_ned_valid`.
- **Heading offset** back-fill: if the receiver gives a `heading` but no `heading_offset`, EKF2 applies the
  `EKF2_GPS_YAW_OFF` parameter and de-rotates the heading so the stored `yaw` is the *body* heading.

After conversion, `_ekf.setGpsData(gnss_sample)` pushes the sample into a ring buffer (`_gps_buffer`),
time-stamped so it can later be popped at the **fusion time horizon** (the IMU-delayed timestamp).

---

## 2. The real-hardware path

### 2.1 Serial / SPI GPS driver — [`src/drivers/gps/gps.cpp`](../../../src/drivers/gps/gps.cpp)

- Runs as a `ModuleBase` task; supports a **Main** and a **Secondary** instance (dual receivers) via
  `PublicationMulti<sensor_gps_s>` → `sensor_gps` instances 0 and 1.
- **Auto-baud + auto-protocol** detection. Baud candidates include 9600…921600. Protocol probe order:
  UBX → MTK → ASHTECH → EMLIDREACH → FEMTOMES → (NMEA) and back.
- Reads bytes, hands them to the active protocol parser, which fills `sensor_gps_s` and publishes.
- Handles RTK: pulls `gps_inject_data` (RTCM3 corrections from a base/NTRIP) and injects to the device,
  reports `rtcm_injection_rate`.

### 2.2 Protocol parsers — [`src/drivers/gps/devices/src/`](../../../src/drivers/gps/devices/src/)

All derive from `GPSHelper` ([`gps_helper.h`](../../../src/drivers/gps/devices/src/gps_helper.h)), implementing
`configure()` and `receive()`:

| Parser | Notes |
|---|---|
| **`ubx`** | u-blox 6/7/8/9; most feature-complete; RTK (F9P), moving-baseline heading, CFG-VALSET |
| **`nmea`** | universal text fallback; limited accuracy/heading fields |
| **`mtk`** | MediaTek chipsets |
| **`ashtech` / `sbf`** | Ashtech / Septentrio binary, survey grade, heading |
| **`emlid_reach`** | Emlid Reach RTK |
| **`femtomes`, `unicore`** | multi-constellation RTK |
| **`rtcm`, `crc`, `base_station`** | RTK correction parsing / shared helpers |

Example (UBX NAV-PVT): `lat = msg.lat·1e-7`, `alt = msg.hMSL·1e-3`, `eph = msg.hAcc·1e-3`,
`vel_n = msg.velN·1e-3`, `satellites_used = msg.numSV`, etc.

### 2.3 CAN GPS

- **DroneCAN / UAVCAN:** [`src/drivers/uavcan/sensors/gnss.cpp`](../../../src/drivers/uavcan/sensors/gnss.cpp)
  subscribes to `gnss::Fix`, `Fix2`, `Auxiliary`, ArduPilot `RelPosHeading`; maps fix mode/sub-mode to
  `fix_type` (DGPS=4, RTK-float=5, RTK-fixed=6) and publishes `sensor_gps`.
- **Cyphal:** [`src/drivers/cyphal/Subscribers/udral/Gnss.hpp`](../../../src/drivers/cyphal/Subscribers/udral/Gnss.hpp)
  subscribes to `reg.drone.physics.kinematics.geodetic.Point` → `sensor_gps`.

### 2.4 Multi-receiver blending — `sensors/vehicle_gps_position/`

When more than one `sensor_gps` instance exists, the
[`VehicleGPSPosition`](../../../src/modules/sensors/vehicle_gps_position/) module selects or blends them and
publishes the single `vehicle_gps_position` topic that EKF2 consumes:

- **Selection:** best `fix_type`, then most satellites, optionally pinned to a primary instance.
- **Blending:** weighted average by accuracy (speed / h-pos / v-pos), low-pass position-offset correction.
- A receiver is dropped after a ~2 s timeout.

Controlled by `SENS_GPS_MASK` (enable + weighting), `SENS_GPS_TAU` (blend time constant),
`SENS_GPS_PRIME` (primary index).

> **Key takeaway:** all of §2 stops at `vehicle_gps_position`. EKF2 only ever subscribes to that topic.

---

## 3. The simulation paths

There are **three distinct simulation entry points**, at decreasing levels of realism.

### 3.1 SITL with a flight simulator (Gazebo / gz / jMAVSim)

```
Simulator physics (groundtruth lat/lon/alt/vel)
   │
   ├─ Gazebo GPS plugin  ── adds random-walk + noise-density ──►  HIL_GPS MAVLink msg
   │     gazebo_gps_plugin.cpp (rate ~5 Hz, XY/Z random walk, vel noise, ~120 ms delay buffer)
   │
   ▼
SimulatorMavlink::handle_message_hil_gps()   src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp
   │  converts HIL_GPS → sensor_gps_s:
   │    lat = lat/1e7, alt = alt/1e3, eph = eph·1e-2, vel_* = v*/100,
   │    s_variance = 0.25, c_variance = 0.5, cog = atan2(ve, vn)
   ▼
sensor_gps (uORB, up to MAX_GPS=3 instances; _gps_blocked can drop them for failure injection)
   ▼
vehicle_gps_position ──► EKF2     (identical to real hardware from here on)
```

This is the **highest-fidelity** sim: it exercises the *entire* real pipeline except the UART bytes and the
protocol parser.

### 3.2 Pure-PX4 SITL without a GPS plugin — `sensor_gps_sim`

[`src/modules/simulation/sensor_gps_sim/`](../../../src/modules/simulation/sensor_gps_sim/) synthesises GPS
directly from PX4's own groundtruth topics, bypassing MAVLink:

- **Subscribes:** `vehicle_global_position_groundtruth`, `vehicle_local_position_groundtruth`.
- **Adds Gaussian noise:** position (~0.2 m h, ~0.5 m v), NED velocity (≈0.06/0.08/0.16 m/s).
- **Publishes** `sensor_gps` at ~8 Hz.
- **`SIM_GPS_USED`** (default 10) sets satellite count: `≥4` → 3D fix with good accuracies; `<4` → no fix
  with large (≈100) error estimates. This is the knob for "fly without GPS" testing.

### 3.3 EKF2 unit-test simulator — `test/sensor_simulator/gps.cpp`

The fastest, most controllable path — used by the GoogleTest suite. It **does not touch uORB at all**; it
builds a `gnssSample` and calls `_ekf->setGpsData()` directly.

- [`test/sensor_simulator/gps.h`](../../../src/modules/ekf2/test/sensor_simulator/gps.h) /
  [`gps.cpp`](../../../src/modules/ekf2/test/sensor_simulator/gps.cpp).
- Setters: `setLatitude/Longitude/Altitude`, `setVelocity`, `setFixType`, `setNumberOfSatellites`,
  `setPdop`, `setYaw`, `setYawOffset`, `setPositionRateNED`.
- Default fix (Zurich): lat ≈ 47.3566, lon ≈ 8.5190, alt ≈ 422 m, fix 3D, 16 sats, hacc 0.5 m,
  vacc 0.8 m, sacc 0.2 m/s.

### 3.4 Replay

[`test/replay_data/iris_gps.csv`](../../../src/modules/ekf2/test/replay_data/iris_gps.csv) +
[`test/change_indication/iris_gps.csv`](../../../src/modules/ekf2/test/change_indication/iris_gps.csv) drive
a recorded-data regression test (`test_EKF_withReplayData.cpp`) — real logged GPS replayed deterministically,
with the change-indication file pinning the expected state evolution.

---

## 4. EKF2 GNSS fusion — the heart of `aid_sources/gnss/`

Four source files, ~1,100 lines total:

| File | Role |
|---|---|
| [`gps_control.cpp`](../../../src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp) | top-level scheduler; velocity & horizontal-position fusion; resets; EKF-GSF yaw |
| [`gps_checks.cpp`](../../../src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp) | pre-flight / in-flight quality gating |
| [`gnss_height_control.cpp`](../../../src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp) | vertical-position (altitude) fusion with bias estimator |
| [`gnss_yaw_control.cpp`](../../../src/modules/ekf2/EKF/aid_sources/gnss/gnss_yaw_control.cpp) | dual-antenna heading fusion |

### 4.1 The scheduler — `controlGpsFusion()` ([`gps_control.cpp:42`](../../../src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L42))

Per IMU-delayed update:

1. **Early-out** if no buffer or `EKF2_GPS_CTRL == 0` → `stopGnssFusion()`.
2. **Always run the EKF-GSF yaw estimator** `_yawEstimator.predict(...)` (the IMU-only emergency yaw fallback,
   see §4.6) — this runs even when GNSS is not being fused.
3. Mark `_gps_intermittent` if no buffer push within `2 × GNSS_MAX_INTERVAL`.
4. Pop the sample at the fusion horizon: `_gps_buffer->pop_first_older_than(imu.time_us, …)`.
5. If a sample is ready → **run quality checks** (`runGnssChecks`). Checks must hold continuously for
   `EKF2_REQ_GPS_H` seconds before `_gps_checks_passed` latches `true`.
6. Build aid-source status for **position** (`updateGnssPos`) and **velocity** (`updateGnssVel`).
7. Then run the four control functions: yaw (`controlGnssYawFusion`), yaw-estimator update, velocity
   (`controlGnssVelFusion`), horizontal position (`controlGnssPosFusion`). Height is driven separately from
   the EKF2 height-source scheduler.

Each control function follows the EKF2 **start/continue/stop** idiom seen throughout the codebase:

```
if (currently_fusing) {
    if (continuing_conditions_passing) fuse(); else stop();
} else {
    if (starting_conditions_passing)  { reset_state_to_gnss(); start(); }
}
```

`continuing_conditions` = control bit set + `tilt_align` + `yaw_align`.
`starting_conditions` = continuing + `_gps_checks_passed`.

### 4.2 Quality checks — `runGnssChecks()` ([`gps_checks.cpp:58`](../../../src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp#L58))

Each check sets a bit in `_gps_check_fail_status`; only checks **enabled in `EKF2_GPS_CHECK`** can cause
rejection. Bit layout (`MASK_GPS_*`):

| Bit | Mask | Check | Threshold param |
|---|---|---|---|
| 0 | `NSATS` | satellites `< req` | `EKF2_REQ_NSATS` (6) |
| 1 | `PDOP` | `pdop > req` | `EKF2_REQ_PDOP` (2.5) |
| 2 | `HACC` | `hacc > req` | `EKF2_REQ_EPH` (3.0 m) |
| 3 | `VACC` | `vacc > req` | `EKF2_REQ_EPV` (5.0 m) |
| 4 | `SACC` | `sacc > req` | `EKF2_REQ_SACC` (0.5 m/s) |
| 5 | `HDRIFT` | horizontal drift while stationary | `EKF2_REQ_HDRIFT` (0.1 m/s) |
| 6 | `VDRIFT` | vertical drift while stationary | `EKF2_REQ_VDRIFT` (0.2 m/s) |
| 7 | `HSPD` | filtered horizontal speed while stationary | `EKF2_REQ_HDRIFT` |
| 8 | `VSPD` | filtered vertical speed while stationary | `EKF2_REQ_VDRIFT` |
| 9 | `SPOOFED` | receiver reports spoofing | — |

Always-on (independent of mask): `fix` check fails when `fix_type < 3`.

**Drift / speed checks only run when on-ground and at rest.** They low-pass the position derivative
(`filt_time_const = 10 s`) and compare against the limits. In-air, these four are forced to pass. An absolute
hard limit (`velocity_limit`) forces hspeed/vspeed failure regardless of state.

Pass/fail bookkeeping uses `_last_gps_fail_us` / `_last_gps_pass_us`; the latch in `controlGpsFusion`
requires the pass condition to hold for `EKF2_REQ_GPS_H` before fusion may start, and triggers the
`gps_checks_passed` information event.

### 4.3 Velocity fusion — `updateGnssVel` / `controlGnssVelFusion`

- **Lever-arm correction:** GNSS antenna offset (`EKF2_GPS_POS_{X,Y,Z}` − IMU offset) crossed with body
  angular rate → velocity offset rotated to earth frame and subtracted
  ([`gps_control.cpp:209`](../../../src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp#L209)).
- **Observation variance:** $\sigma_v^2 = \max(\texttt{sacc}, \texttt{EKF2\_GPS\_V\_NOISE}, 0.01)^2$;
  the **vertical** component is inflated by $1.5^2$.
- **Innovation:** $\nu = \hat{v} - z$; gate `EKF2_GPS_V_GATE` (σ).
- **Bad-vertical-accel special case:** if the IMU has flagged bad vertical accel and only `vz` is rejected
  (vx, vy accepted) and `sacc < EKF2_REQ_SACC`, the vertical innovation is *clamped* rather than rejected —
  keeps a trustworthy GNSS vz from being thrown away during accel faults.
- On start (and on forced reset/timeout) the velocity state is hard-reset to the GNSS observation
  (`resetVelocityToGnss`).

### 4.4 Horizontal position fusion — `updateGnssPos` / `controlGnssPosFusion`

- **Lever-arm correction** applied to lat/lon/alt via `LatLonAlt`.
- **Observation noise:** $\max(\texttt{hacc}, \texttt{EKF2\_GPS\_P\_NOISE})$, but **capped at
  `pos_noaid_noise`** when GNSS is the *only* horizontal aiding source (so a bad-but-large hacc cannot let
  attitude errors grow unbounded).
- Innovation in the local tangent plane; gate `EKF2_GPS_P_GATE`.
- Start resets horizontal position to GNSS. There is also a **global-origin initialisation** branch: even
  before position fusion formally starts, if checks pass and the local origin is uninitialised, the position
  is seeded from GNSS.

### 4.5 Height fusion — `controlGnssHeightFusion()` ([`gnss_height_control.cpp:41`](../../../src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp#L41))

Distinct from horizontal because GNSS altitude is noisy and biased:

- Runs only when `EKF2_GPS_CTRL` bit 1 (Altitude) is set; gated by `_gps_checks_passed` + initialised origin.
- **Vertical bias estimator** (`HeightBiasEstimator _gps_hgt_b_est`): a small auxiliary filter tracking the
  slow offset between GNSS altitude and the EKF's height datum. Process noise spectral density set by
  `EKF2_GPS_HGT_B_NSD`; the bias is fused *before* the main height innovation is formed.
- Observation noise: `max(vacc, 1.5 × EKF2_GPS_P_NOISE)`, again capped at `pos_noaid_noise` when GNSS is the
  sole vertical reference.
- Sign convention: NED-down, so the innovation uses `-(measurement − bias)`.
- If `EKF2_HGT_REF == GNSS`, starting GNSS height resets the altitude datum and makes GNSS the height
  reference; otherwise GNSS height is fused as a secondary source with its bias initialised so it doesn't jump.
- Includes the standard fallback logic: if all height sources fail it force-resets to GNSS; if another source
  is healthy it simply stops GNSS height. (`gpsHgtToBaroFallback` test covers this.)

### 4.6 Yaw — two completely different mechanisms

**(a) Dual-antenna GNSS yaw** — `gnss_yaw_control.cpp`. A *direct measurement* of heading from a
moving-baseline / dual-antenna receiver:

- Active only when `EKF2_GPS_CTRL` bit 3 set and not `gnss_yaw_fault`.
- `updateGnssYaw` builds the predicted antenna-array heading via generated symforce code
  [`compute_gnss_yaw_pred_innov_var_and_h.h`](../../../src/modules/ekf2/EKF/python/ekf_derivation/generated/compute_gnss_yaw_pred_innov_var_and_h.h);
  observation = `wrap_pi(yaw + yaw_offset)`; noise = `max(yaw_acc, EKF2_GPS_YAW_NOISE)`; gate
  `EKF2_HDG_GATE`.
- `resetYawToGnss` refuses if the antenna array is within 30° of vertical (unreliable heading).
- `fuseGnssYaw` guards against an ill-conditioned innovation variance (covariance reset + abort), and resets
  the gyro-Z bias variance if a large persistent test-ratio suggests a wrong gyro bias on the ground.

**(b) EKF-GSF emergency yaw estimator** — `_yawEstimator` (driven from `gps_control.cpp`). A bank of
$N$ simple AHRS filters that derive yaw purely from IMU + GNSS *velocity* (no magnetometer). It is the
recovery mechanism for a bad yaw in flight:

- `controlGnssYawEstimator` feeds GNSS horizontal velocity into the estimator when `sacc < EKF2_REQ_SACC`.
- Before `yaw_align`, a converged estimate is used to align yaw straight from IMU+GPS
  (`resetYawToEKFGSF`) — this is how a multicopter can take off and align heading **without a magnetometer**.
- `isYawFailure()` (>25° disagreement) + sustained horizontal-velocity innovation failures →
  `tryYawEmergencyReset()`: snaps yaw to the GSF estimate and marks mag / GNSS-yaw / EV-yaw as faulty so they
  stop dragging the solution. Tuned by `EKF2_GSF_TAS` and `EKF2_EKFGSF_*` params.

### 4.7 Resets & dead-reckoning — `shouldResetGpsFusion()` / `stopGnssFusion()`

- `shouldResetGpsFusion()` decides a forced reset when horizontal aiding has timed out
  (`EKF2_NOAID_TOUT` / `reset_timeout_max`), with an optical-flow carve-out, and detects an in-flight
  navigation failure (both pos & vel timed out since takeoff).
- A forced reset hard-resets velocity and/or position back to the GNSS observation instead of continuing to
  fuse innovations.
- `stopGnssFusion()` tears down all four sub-fusions (vel, pos, height, yaw) and resets the GSF estimator.

---

## 5. End-to-end fusion math (quick reference)

For a generic linear measurement $z = Hx + \nu,\ \nu\sim\mathcal N(0,R)$:

$$
\nu = z - H\hat{x}, \qquad
S = HPH^\top + R, \qquad
K = PH^\top S^{-1}
$$
$$
\hat{x}^+ = \hat{x} + K\nu, \qquad
P^+ = (I - KH)P
$$

GNSS uses this with:

| Aiding | $z$ | $R$ (variance) | Gate |
|---|---|---|---|
| Horizontal pos | lat/lon (tangent-plane) | $\max(\texttt{hacc}, \texttt{EKF2\_GPS\_P\_NOISE})^2$, capped | `EKF2_GPS_P_GATE` |
| Velocity | NED vel | $\max(\texttt{sacc}, \texttt{EKF2\_GPS\_V\_NOISE})^2$, vz ×$1.5^2$ | `EKF2_GPS_V_GATE` |
| Height | alt (−down) | $\max(\texttt{vacc}, 1.5\,\texttt{EKF2\_GPS\_P\_NOISE})^2$ + bias var | `EKF2_GPS_P_GATE` |
| Yaw (dual-ant.) | `wrap_pi(yaw+offset)` | $\max(\texttt{yaw\_acc}, \texttt{EKF2\_GPS\_YAW\_NOISE})^2$ | `EKF2_HDG_GATE` |

The **innovation consistency (gate) test** rejects a measurement when
$\nu^2 > \gamma^2\, S$ (γ = gate in σ). For position/velocity $H$ is trivial (identity rows); for yaw $H$ is
the symforce-generated Jacobian.

---

## 6. Parameter reference (`params_gnss.yaml`)

| Parameter | Default | Unit | Purpose |
|---|---|---|---|
| `EKF2_GPS_CTRL` | 7 | bitmask | enable: b0 lon/lat, b1 alt, b2 3D vel, b3 dual-ant heading |
| `EKF2_GPS_DELAY` | 110 | ms | GPS-vs-IMU measurement delay (reboot) |
| `EKF2_GPS_P_NOISE` | 0.5 | m | base position measurement noise |
| `EKF2_GPS_P_GATE` | 5.0 | σ | position innovation gate |
| `EKF2_GPS_V_NOISE` | 0.3 | m/s | base velocity measurement noise |
| `EKF2_GPS_V_GATE` | 5.0 | σ | velocity innovation gate |
| `EKF2_GPS_YAW_OFF` | 0.0 | deg | dual-antenna mounting offset |
| `EKF2_GPS_POS_X/Y/Z` | 0.0 | m | GPS antenna lever arm (body frame) |
| `EKF2_GPS_CHECK` | 1023 | bitmask | which quality checks gate fusion (see §4.2) |
| `EKF2_REQ_EPH` | 3.0 | m | max horizontal pos error |
| `EKF2_REQ_EPV` | 5.0 | m | max vertical pos error |
| `EKF2_REQ_SACC` | 0.5 | m/s | max speed accuracy |
| `EKF2_REQ_NSATS` | 6 | – | min satellites |
| `EKF2_REQ_PDOP` | 2.5 | – | max PDOP |
| `EKF2_REQ_HDRIFT` | 0.1 | m/s | max stationary horizontal drift/speed |
| `EKF2_REQ_VDRIFT` | 0.2 | m/s | max stationary vertical drift/speed |
| `EKF2_REQ_GPS_H` | 10.0 | s | continuous-health time before fusion may start (reboot) |
| `EKF2_GSF_TAS` | 15.0 | m/s | assumed airspeed for EKF-GSF centripetal compensation |

> Names in `params_gnss.yaml` use the `EKF2_REQ_EPH/EPV` form; the check bit comments in the YAML still refer
> to them by their threshold params. Height-bias (`EKF2_GPS_HGT_B_NSD`), yaw-noise (`EKF2_GPS_YAW_NOISE`),
> heading gate (`EKF2_HDG_GATE`) and EKF-GSF params live in neighbouring EKF2 param files.

Transport-layer params (driver / blending, **not** in `params_gnss.yaml`): `GPS_1_CONFIG`/`GPS_2_CONFIG`,
`GPS_1_GNSS`/`GPS_2_GNSS` (constellation mask), `GPS_UBX_DYNMODEL`, `GPS_UBX_MODE`, `GPS_YAW_OFFSET`,
`SENS_GPS_MASK`, `SENS_GPS_TAU`, `SENS_GPS_PRIME`, `SIM_GPS_USED`.

---

## 7. Tests — what proves it works

[`test/test_EKF_gps.cpp`](../../../src/modules/ekf2/test/test_EKF_gps.cpp):

| Test | Asserts |
|---|---|
| `gpsTimeout` | fusion stops when sats drop below minimum; recovers when restored |
| `gpsFixLoss` | `fix_type=0` → dead-reckoning → GNSS deactivates; local pos invalidates |
| `resetToGpsVelocity` | velocity state snaps to injected GNSS velocity; reset counter increments |
| `resetToGpsPosition` | position state snaps to a 20 m injected jump |
| `gpsHgtToBaroFallback` | seamless height-source handover GNSS→baro when GPS stops |
| `altitudeDrift` | EKF tolerates a slow GNSS altitude bias without baro innovation blowing up |

[`test/test_EKF_gnss_yaw.cpp`](../../../src/modules/ekf2/test/test_EKF_gnss_yaw.cpp):
`fusionStartWithReset`, `yawConvergence`, fixed-angle cases (`yaw0/60/180/-120/-30`), `fallBackToMag`,
`fallBackToYawEmergencyEstimator`.

Run them: `make tests TESTFILTER=EKF_gps` / `TESTFILTER=EKF_gnss_yaw`.

---

## 8. Putting it together — the full journey of one fix

```
 1. Receiver measures pseudoranges → solves PVT
 2. Protocol parser (ubx/nmea/…) or HIL_GPS or sim module fills sensor_gps_s
 3. (multi-GPS) VehicleGPSPosition selects/blends → vehicle_gps_position
 4. EKF2.cpp UpdateGpsSample: sensor_gps_s → gnssSample, derive pdop, de-rotate heading
 5. _ekf.setGpsData → ring buffer (_gps_buffer)
 6. controlGpsFusion pops sample at IMU fusion horizon
 7. runGnssChecks → must pass EKF2_REQ_GPS_H seconds → _gps_checks_passed latches
 8. updateGnssPos / updateGnssVel build innovations (lever-arm + noise model)
 9. controlGnss{Vel,Pos,Height,Yaw}Fusion: start (reset) / continue (fuse) / stop
10. EKF-GSF yaw estimator runs in parallel as emergency heading fallback
11. Corrected state → estimator_local_position / vehicle_global_position / attitude
```

Steps 4–11 are **byte-for-byte identical** in SITL and on real hardware. Only steps 1–3 differ — which is
exactly why a unit test, a Gazebo flight, and a real flight all exercise the same fusion code.

---

## 9. Keywords for deeper study

EKF aiding source · innovation consistency / Mahalanobis gating · lever-arm compensation · GNSS
moving-baseline heading · EKF-GSF (Gaussian Sum Filter) emergency yaw · GPS spoofing/jamming detection ·
RTK / RTCM3 injection · DOP (PDOP/HDOP/VDOP) · dead-reckoning timeout · height bias estimator ·
sensor fusion time horizon & delayed-time EKF · HIL_GPS / SITL noise models · multi-receiver GPS blending.

---

## 10. File map

| Concern | Path |
|---|---|
| Fusion scheduler, vel & h-pos, resets, EKF-GSF | `src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp` |
| Quality checks | `src/modules/ekf2/EKF/aid_sources/gnss/gps_checks.cpp` |
| Height fusion + bias estimator | `src/modules/ekf2/EKF/aid_sources/gnss/gnss_height_control.cpp` |
| Dual-antenna yaw fusion | `src/modules/ekf2/EKF/aid_sources/gnss/gnss_yaw_control.cpp` |
| Parameters | `src/modules/ekf2/params_gnss.yaml` |
| sensor_gps→gnssSample bridge | `src/modules/ekf2/EKF2.cpp` (`UpdateGpsSample`, ~L2405) |
| `gnssSample` struct | `src/modules/ekf2/EKF/common.h:184` |
| Transport message | `msg/SensorGps.msg` |
| Real driver + protocols | `src/drivers/gps/gps.cpp`, `src/drivers/gps/devices/src/` |
| CAN GPS | `src/drivers/uavcan/sensors/gnss.cpp`, `src/drivers/cyphal/.../udral/Gnss.hpp` |
| Multi-GPS blending | `src/modules/sensors/vehicle_gps_position/` |
| SITL HIL bridge | `src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp` |
| Pure-SITL GPS synth | `src/modules/simulation/sensor_gps_sim/` |
| Unit-test GPS simulator | `src/modules/ekf2/test/sensor_simulator/gps.{h,cpp}` |
| Unit tests | `src/modules/ekf2/test/test_EKF_gps.cpp`, `test_EKF_gnss_yaw.cpp` |
| Replay data | `src/modules/ekf2/test/replay_data/iris_gps.csv` |
</content>
</invoke>
