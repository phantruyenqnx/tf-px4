# 2c. EKF2 Source Code Reading Guide — Quadcopter Path Only

> **Purpose**: File-by-file, function-by-function map for developers new to the EKF2 source code. Only covers what is relevant to a typical GPS/baro/mag quadcopter — fixed-wing, VTOL, and multi-IMU selector are omitted.
>
> Read after `02_ekf2.md` (theory) and `02b_kf_to_ekf2_theory.md` (KF → EKF2). Notation is consistent with `00_index.md`.

---

## Table of Contents

1. [Two-Layer Architecture](#1-two-layer-architecture)
2. [State Vector — Read First](#2-state-vector--read-first)
3. [Main Loop: `Ekf::update()`](#3-main-loop-ekfupdate)
4. [`predictState()` — IMU Integration](#4-predictstate--imu-integration)
5. [`predictCovariance()` + Generated Code](#5-predictcovariance--generated-code)
6. [`controlFusionModes()` — Dispatcher](#6-controlfusionmodes--dispatcher)
7. [Pattern for Reading Any Aid Source](#7-pattern-for-reading-any-aid-source)
8. [Core Kalman Functions: `fuse()` + `measurementUpdate()`](#8-core-kalman-functions-fuse--measurementupdate)
9. [Output Predictor — IMU Delay Compensation](#9-output-predictor--imu-delay-compensation)
10. [File Table: Read / Skip](#10-file-table-read--skip)
11. [Quick Paths by Question](#11-quick-paths-by-question)

---

## 1. Two-Layer Architecture

```
src/modules/ekf2/
├── EKF2.cpp           ← Layer 1: uORB module — feed sensor, publish results
├── EKF2Selector.cpp   ← SKIP (multi-IMU redundancy)
└── EKF/
    ├── ekf.cpp            ← Layer 2: main EKF loop
    ├── ekf.h              ← Full API + member state (read for the map)
    ├── covariance.cpp     ← Predict P (calls generated code)
    ├── control.cpp        ← Dispatcher: enable/disable each aid source
    ├── ekf_helper.cpp     ← Core Kalman functions (fuse, measurementUpdate)
    ├── output_predictor/  ← IMU delay compensation → output for controller
    └── aid_sources/       ← Each sensor has its own folder
```

**Reading rule**: Focus on `EKF/` (algorithm). Only read `EKF2.cpp` when you need to know how sensor data is fed in.

The two most important APIs between the two layers:

| Direction | API | Meaning |
|---|---|---|
| Sensor → EKF | `_ekf.setImuData()`, `setGpsData()`, `setBaroData()`, `setMagData()`, `setOpticalFlowData()` | Push sample into ring buffer |
| EKF → controller | `_attitude_pub` (`vehicle_attitude`), `_local_position_pub` (`vehicle_local_position`) | Output after output predictor |

---

## 2. State Vector — Read First

**File**: `@/home/frank/tf-px4/src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h`
(auto-generated from SymPy — **do not edit manually**)

```cpp
struct StateSample {
    matrix::Quaternion<float> quat_nominal{};  // attitude body→NED
    matrix::Vector3<float>    vel{};            // velocity NED (m/s)
    matrix::Vector3<float>    pos{};            // position NED — z only (m)
    matrix::Vector3<float>    gyro_bias{};      // gyro bias (rad/s)
    matrix::Vector3<float>    accel_bias{};     // accel bias (m/s²)
    matrix::Vector3<float>    mag_I{};          // Earth mag field (Gauss)
    matrix::Vector3<float>    mag_B{};          // body mag bias (Gauss)
    matrix::Vector2<float>    wind_vel{};       // wind NE (m/s)
    float                     terrain{};        // terrain altitude (m)
};

namespace State {
    static constexpr IdxDof quat_nominal{0, 3};  // index in P, dof=3 (error-state)
    static constexpr IdxDof vel        {3, 3};
    static constexpr IdxDof pos        {6, 3};
    static constexpr IdxDof gyro_bias  {9, 3};
    static constexpr IdxDof accel_bias {12, 3};
    static constexpr IdxDof mag_I      {15, 3};
    static constexpr IdxDof mag_B      {18, 3};
    static constexpr IdxDof wind_vel   {21, 2};
    static constexpr IdxDof terrain    {23, 1};
    static constexpr uint8_t size{24};           // P is 24×24
}
```

**Key points about error-state**:
- `StateSample` stores `quat_nominal` with 4 elements, but `State::quat_nominal.dof = 3`.
- Covariance `P` is `SquareMatrix<float, 24>` — **not** 25×25.
- When reading `P.slice<S.dof, 1>(S.idx, 0)` or `K.slice<S.dof, 1>(S.idx, 0)`, remember that the quaternion part maps through the attitude error vector $\delta\boldsymbol{\theta}\in\mathbb{R}^3$ (see §3 and §8).
- `_state` is the `StateSample` member of class `Ekf`. Covariance is `P` (public member in `EstimatorInterface`).
- Position X, Y are **not** stored in `_state.pos` but in `_gpos` (type `LatLonAlt`) to avoid float accumulation error when flying far.

---

## 3. Main Loop: `Ekf::update()`

**File**: `@/home/frank/tf-px4/src/modules/ekf2/EKF/ekf.cpp` — function `update()` line ~138

```
Ekf::update()  ← called by EKF2::Run() each time new IMU data arrives
  │
  ├─ (first call) initialiseFilter()
  │     ├─ initialiseTilt()         ← quaternion initialized from accel LPF
  │     └─ initialiseCovariance()   ← P = diag(sigma²_init)
  │
  ├─ imu_sample_delayed = _imu_buffer.get_oldest()
  │                                 ← KEY: EKF runs at DELAYED TIME HORIZON
  │
  ├─ predictCovariance(imu_delayed) ← P(k|k-1)
  ├─ predictState(imu_delayed)      ← x(k|k-1)
  │
  ├─ controlFusionModes(imu_delayed) ← EKF update: select sensor to fuse
  │
  └─ output_predictor.correctOutputStates()
                                    ← compensate back to real-time for controller
```

These are the **4 most important lines** in the entire EKF2:

```cpp
predictCovariance(imu_sample_delayed);          // P predict
predictState(imu_sample_delayed);               // x predict
controlFusionModes(imu_sample_delayed);         // sensor fusion
_output_predictor.correctOutputStates(...);     // delay compensation
```

---

## 4. `predictState()` — IMU Integration

**File**: `@/home/frank/tf-px4/src/modules/ekf2/EKF/ekf.cpp` — function `predictState()` line ~231

Four IMU integration steps:

```
// Step 1: subtract bias + Earth rate → true angular rate
corrected_delta_ang = delta_ang - gyro_bias*dt
                    - R_body2ned.T * earth_rate_NED * dt

// Step 2: propagate quaternion
dq = AxisAngle(corrected_delta_ang)
quat_nominal = (quat_nominal * dq).normalized()
R_to_earth   = DCM(quat_nominal)               // update DCM

// Step 3: integrate velocity NED
corrected_delta_vel_ef = R_to_earth * (delta_vel - accel_bias*dt)
vel += corrected_delta_vel_ef
vel += (gravity + coriolis + transport_rate) * dt

// Step 4: integrate position (trapezoidal)
_gpos += (vel_prev + vel) * dt * 0.5
_state.pos(2) = -_gpos.altitude()              // only z stored in state
```

**Code reading notes**:
- `delta_ang` and `delta_vel` are already *integrated* (not raw): $\Delta\boldsymbol{\theta} = \int\boldsymbol{\omega}\,dt$, $\Delta\boldsymbol{v}=\int\boldsymbol{a}\,dt$.
- `getGyroBias()` / `getAccelBias()` return `_state.gyro_bias` / `_state.accel_bias`.
- `coriolis_acceleration = -2 * earth_rate_NED × vel` — small at low speed, but fully computed.
- Position X, Y are accumulated into `_gpos` (global position), not into `_state.pos`.

---

## 5. `predictCovariance()` + Generated Code

**File**: `@/home/frank/tf-px4/src/modules/ekf2/EKF/covariance.cpp`

`covariance.cpp` **does not contain** the actual Jacobian formulas. It only `#include`s and calls:

```cpp
#include <ekf_derivation/generated/predict_covariance.h>
// → sym::PredictCovariance(...) functions are inlined
```

The entire Jacobian $\boldsymbol{F}$ and $\boldsymbol{G}\boldsymbol{Q}\boldsymbol{G}^\top$ are **generated from SymPy**:

```
EKF/python/ekf_derivation/derivation.py   ← source of symbolic math
  → generates:
EKF/python/ekf_derivation/generated/
  ├── predict_covariance.h   ← function to predict P (inlined C++)
  ├── compute_*_h.h          ← Jacobian H for each sensor
  └── state.h                ← StateSample + State namespace
```

**How to read**: To understand Jacobian $F_{ij}$, **read `derivation.py`** (SymPy symbolic, clear) — do not read the generated C++ (unreadable). `derivation.py` declares each Jacobian element explicitly using mathematical notation.

---

## 6. `controlFusionModes()` — Dispatcher

**File**: `@/home/frank/tf-px4/src/modules/ekf2/EKF/control.cpp`

This is the **dispatcher** that calls each sensor's `control*Fusion()` function every tick:

```cpp
void Ekf::controlFusionModes(imu_delayed) {
    controlMagFusion()           // yaw/heading from magnetometer
    controlOpticalFlowFusion()   // optical flow (if available)
    controlGpsFusion()           // GPS vel + pos (horizontal)
    controlAirDataFusion()       // ← SKIP: fixed-wing airspeed
    controlBetaFusion()          // ← SKIP: fixed-wing sideslip
    controlDragFusion()          // ← SKIP: fixed-wing drag model
    controlHeightFusion()        // altitude (baro / GPS / range / EV)
    controlGravityFusion()       // gravity vector
    controlExternalVisionFusion()// EV pos/vel/yaw (if using VIO)
    controlFakePosFusion()       // fallback when no sensor available
    ...
}
```

**For a typical GPS quadcopter**, only follow 4 branches:

| Branch | File |
|---|---|
| `controlGpsFusion()` | `aid_sources/gnss/gps_control.cpp` |
| `controlMagFusion()` | `aid_sources/magnetometer/mag_control.cpp` |
| `controlHeightFusion()` → `controlBaroHeightFusion()` | `aid_sources/barometer/baro_height_control.cpp` |
| `controlOpticalFlowFusion()` | `aid_sources/optical_flow/optical_flow_control.cpp` |

Each `control*Fusion()` function does 3 things: check whether data is in the buffer → compute innovation → decide whether to call fuse.

---

## 7. Pattern for Reading Any Aid Source

Every aid source follows the **same pattern**. Example: GPS velocity (file `gps_control.cpp`):

```
controlGpsFusion(imu_delayed)
  ├─ pop gnssSample from buffer (already delay-matched with IMU)
  │
  ├─ updateGnssVel(imu_sample, gnss_sample, aid_src)
  │    innovation = gnss_vel - _state.vel         ← z = y - h(x)
  │    innov_var  = diag(P)[vel] + R_gps_vel       ← S = HPH + R
  │    test_ratio = innov² / (gate² * innov_var)   ← chi-square
  │
  ├─ if test_ratio < 1.0:                          ← innovation gate pass
  │    controlGnssVelFusion(aid_src)
  │      └─ velocity_fusion.cpp::fuseVelocity()
  │           K = P.col(vel_idx) / innov_var        ← K = PH/S (sequential)
  │           measurementUpdate(K, H, R, innov)
  │             ├─ update P (Joseph stabilized form)
  │             └─ fuse(K, innov) → update _state
  │
  └─ else: mark innovation_rejected in aid_src
```

**3 invariant steps**:
1. `update*()` — compute innovation, innov_var, test_ratio → store in `estimator_aid_source*d_s`.
2. Gate check — if pass, call fuse.
3. `measurementUpdate()` + `fuse()` — the two core functions (see §8).

**Struct `estimator_aid_source*d_s`** stores the full state of one sensor: `innovation`, `innovation_variance`, `innovation_rejected`, `fused`, `test_ratio`, `timestamp_sample`. This is also the topic published for logging/debug.

---

## 8. Core Kalman Functions: `fuse()` + `measurementUpdate()`

**File**: `@/home/frank/tf-px4/src/modules/ekf2/EKF/ekf_helper.cpp`

### `fuse(K, innovation)` — line ~736

Applies correction to each field of `_state`:

```cpp
void Ekf::fuse(const VectorState &K, float innovation) {
    // Quaternion: multiplicative correction
    Quatf dq(AxisAnglef(K.slice<3,1>(State::quat_nominal.idx, 0) * (-innovation)));
    _state.quat_nominal = (dq * _state.quat_nominal).normalized();

    // Velocity: additive
    _state.vel -= K.slice<3,1>(State::vel.idx, 0) * innovation;

    // Position: add to global position
    _gpos += K.slice<3,1>(State::pos.idx, 0) * (-innovation);
    _state.pos(2) = -_gpos.altitude();

    // Gyro bias, accel bias: additive + clamp
    _state.gyro_bias  -= K.slice<3,1>(State::gyro_bias.idx,  0) * innovation;
    _state.accel_bias -= K.slice<3,1>(State::accel_bias.idx, 0) * innovation;

    // mag_I, mag_B: only when _control_status.flags.mag == true
    // wind_vel:     only when _control_status.flags.wind == true
    ...
}
```

**Key note**: Quaternion uses **multiplicative** $\boldsymbol{q}\leftarrow\exp(\delta\boldsymbol{\theta}/2)\otimes\boldsymbol{q}$, not additive. Other states are additively corrected.

### `measurementUpdate(K, H, R, innovation)` — line ~1089

Joseph stabilized covariance update:

$$
\boldsymbol{P} \leftarrow (\boldsymbol{I}-\boldsymbol{K}\boldsymbol{H})\boldsymbol{P}(\boldsymbol{I}-\boldsymbol{K}\boldsymbol{H})^\top + \boldsymbol{K}R\boldsymbol{K}^\top
$$

Code performs **2 passes** to avoid additional memory allocation:

```
// Pass 1: P_temp = (I - K*H) * P  (conventional update, P temporarily loses symmetry)
PH = P * H
P(i,j) -= K(i) * PH(j)   ← for all i,j

// Pass 2: P = P_temp * (I - H.T * K.T) + K*R*K.T  (stabilize)
PH = P * H                ← recompute with P_temp
P(i,j) = P(i,j) - PH(i)*K(j) + K(i)*R*K(j)   ← lower triangle only, then mirror
```

Then calls `constrainStateVariances()` — clamps diagonal P to prevent divergence.

**This is the only function that updates P**. Learn it once, understand every sensor.

---

## 9. Output Predictor — IMU Delay Compensation

**File**: `@/home/frank/tf-px4/src/modules/ekf2/EKF/output_predictor/output_predictor.cpp`

### Why it is needed

The EKF runs at the **delayed time horizon** (~100 ms lag) to wait for sensor data synchronization. But the controller needs attitude/velocity immediately (latency < 5 ms).

```
Timeline:
  ──────────────────────────────────────────────────────→ time
  IMU sample    ... IMU sample    IMU sample (now)
  |                 |             |
  delayed horizon   |             now
  EKF corrects here |             controller reads here
  ←── ~100ms lag ───→
```

### Mechanism

```
Each new IMU sample (not delayed):
  output_predictor.calculateOutputStates(imu_now)
    → integrate quat_out, vel_out using latest IMU
    → store in output_buffer[now]

When EKF finishes one tick (delayed):
  output_predictor.correctOutputStates(imu_delayed.time_us, q_ekf, vel_ekf, ...)
    → compute delta between EKF state and output state at delayed time
    → add delta (tracking error) to entire output_buffer
    → result: output_buffer[now] reflects EKF correction + latest IMU integration
```

**Output** is read by `EKF2.cpp` to publish `vehicle_attitude` and `vehicle_local_position`.

---

## 10. File Table: Read / Skip

### Core Layer (EKF/) — read in order

| File | Read? | Reason |
|---|---|---|
| `EKF/python/ekf_derivation/generated/state.h` | **Read first** | State vector layout + P index |
| `EKF/ekf.cpp::update()` | **Read carefully** | Main loop, 4 steps |
| `EKF/ekf.cpp::predictState()` | **Read carefully** | IMU integration ~50 lines, clear |
| `EKF/ekf.cpp::initialiseTilt()` | Skim | Quaternion initialization from accel |
| `EKF/ekf_helper.cpp::fuse()` | **Read carefully** | State correction, understand once |
| `EKF/ekf_helper.cpp::measurementUpdate()` | **Read carefully** | Joseph form, used for every sensor |
| `EKF/control.cpp::controlFusionModes()` | Skim | Dispatcher, see what is enabled/disabled |
| `EKF/covariance.cpp` | Skim | Only calls generated code |
| `EKF/python/ekf_derivation/derivation.py` | When Jacobian needed | Symbolic math, source of covariance |
| `EKF/output_predictor/output_predictor.cpp` | Read conceptually | Understand delay compensation |

### Aid Sources — typical GPS quadcopter

| File | Read? | Reason |
|---|---|---|
| `aid_sources/gnss/gps_control.cpp` | Read | GPS vel + pos fusion |
| `aid_sources/magnetometer/mag_control.cpp` | Read | Yaw/heading |
| `aid_sources/barometer/baro_height_control.cpp` | Read | Altitude |
| `aid_sources/optical_flow/optical_flow_control.cpp` | Optional | If flow sensor is present |
| `velocity_fusion.cpp`, `position_fusion.cpp` | Skim | Compute K and call measurementUpdate |
| `yaw_estimator/EKFGSF_yaw.cpp` | Optional | GSF fallback when mag fails |

### Skip Entirely

| File/Folder | Reason to skip |
|---|---|
| `EKF2Selector.cpp` | Multi-IMU redundancy |
| `aid_sources/airspeed/` | Fixed-wing airspeed |
| `aid_sources/sideslip/` | Fixed-wing sideslip (beta) |
| `aid_sources/drag/` | Fixed-wing drag model |
| `aid_sources/ZeroVelocityUpdate*` | Only when landed/stationary check |
| `aid_sources/ZeroGyroUpdate*` | Only when `EKF2_IMU_CTRL` enables GyroBias |

### Wrapper Layer (EKF2.cpp) — need-to-know only

| Point | Location | Meaning |
|---|---|---|
| Trigger | `_sensor_combined_sub.registerCallback()` | Run on new IMU |
| Load IMU | `EKF2.cpp` ~line 603 | `imu_sample_new` → `_ekf.setImuData()` |
| Load GPS | `EKF2.cpp` ~line 2451 | `_ekf.setGpsData(gnss_sample)` |
| Load Baro | `EKF2.cpp` ~line 2173 | `_ekf.setBaroData(...)` |
| Load Mag | `EKF2.cpp` ~line 2511 | `_ekf.setMagData(...)` |
| Call EKF | `EKF2.cpp` ~line 700 | `_ekf.update()` |
| Publish att | `EKF2.cpp` ~line 1021 | `_attitude_pub.publish(att)` |
| Publish pos | `EKF2.cpp` ~line 1540 | `_local_position_pub.publish(lpos)` |

---

## 11. Quick Paths by Question

### "How is GPS velocity fused?"

1. `control.cpp:115` → `controlGpsFusion()` is called
2. `gnss/gps_control.cpp:126` → `controlGnssVelFusion()` → `updateGnssVel()` computes innovation
3. `velocity_fusion.cpp` → `fuseVelocity()` → compute K (sequential per axis)
4. `ekf_helper.cpp:1089` → `measurementUpdate(K, H, R, innov)` → update P
5. `ekf_helper.cpp:736` → `fuse(K, innov)` → update `_state.vel`

### "How does gyro bias anti-windup work?"

1. `ekf_helper.cpp::fuse()` — see `gyro_bias -= K[...] * innov` with clamp `±getGyroBiasLimit()`
2. `ekf.h::clearInhibitedStateKalmanGains()` — zeroes out K for inhibited states
3. `ekf.h` member `_gyro_bias_inhibit[3]` — set when accel clipping or bad accel

### "When GPS is lost, what does EKF do?"

1. `gnss/gps_control.cpp::stopGnssFusion()` — clears `_control_status.flags.gps`
2. `control.cpp::controlFakePosFusion()` — activates fake pos constraint to prevent drift
3. `ekf.h` field `_time_last_horizontal_aiding` → `updateDeadReckoningStatus()` → publishes dead reckoning warning

### "What time does the output predictor get attitude from?"

1. `output_predictor.cpp::calculateOutputStates(imu_now)` — runs every new IMU sample (high rate)
2. `output_predictor.cpp::correctOutputStates(...)` — runs after `Ekf::update()` (delayed rate)
3. `EKF2.cpp::publishAttitude()` → reads from `_ekf.output_predictor().getState()` (newest output buffer)

### "What is the Jacobian H for GPS position?"

1. `aid_sources/gnss/gps_control.cpp::updateGnssPos()` — find how H is computed
2. If H is simple (identity on pos states): H vector all zeros except column `State::pos.idx`
3. If more complex: look for `#include <ekf_derivation/generated/compute_*_h.h>`

---

## Summary Mental Model

```
Each tick (new IMU):
  ┌─ PREDICT ─────────────────────────────────────────────┐
  │  predictState()       : x(k|k-1) via IMU integration  │
  │  predictCovariance()  : P(k|k-1) via generated F,Q    │
  └───────────────────────────────────────────────────────┘
  ┌─ UPDATE ──────────────────────────────────────────────┐
  │  controlFusionModes() : for each available sensor:    │
  │    update*()     → z, S, test_ratio                   │
  │    if gate pass  → measurementUpdate(K, H, R, z)      │
  │                      ├── update P (Joseph form)       │
  │                      └── fuse(K, z) → update _state   │
  └───────────────────────────────────────────────────────┘
  ┌─ OUTPUT ──────────────────────────────────────────────┐
  │  output_predictor.correctOutputStates()               │
  │  → q, vel, pos at "now" (not delayed horizon)         │
  │  → publish vehicle_attitude, vehicle_local_position   │
  └───────────────────────────────────────────────────────┘
```
