# EKF2 Unified Reference — Source Guide + ESKF Theory

> **Structure**:
> - **Part 1** — *Source Navigation*: file map, read order, debugging paths. No theory.
> - **Part 2** — *ESKF Theory with Code*: every derivation is immediately followed by its verified source implementation. No duplicate file tables.
>
> All code snippets verified against the actual source tree.

---

## Table of Contents

- [Part 1 — Source Navigation Guide](#part-1--source-navigation-guide)
  - [1.1 Two-Layer Architecture](#11-two-layer-architecture)
  - [1.2 File Table: Read / Skip](#12-file-table-read--skip)
  - [1.3 Function Map with Line Numbers](#13-function-map-with-line-numbers)
  - [1.4 Quick Paths by Question](#14-quick-paths-by-question)
  - [1.5 imuSample — Fields and Data Flow](#15-imusample--fields-and-data-flow)
- [Part 2 — ESKF Theory with Code](#part-2--eskf-theory-with-code)
  - [2.1 Architecture Overview](#21-architecture-overview)
  - [2.2 Why Error-State EKF](#22-why-error-state-ekf)
  - [2.3 The 24-State Error Vector](#23-the-24-state-error-vector)
  - [2.4 PREDICT — Nominal State Propagation](#24-predict--nominal-state-propagation)
  - [2.5 PREDICT — Covariance Propagation](#25-predict--covariance-propagation)
  - [2.6 UPDATE — Observation Models](#26-update--observation-models)
  - [2.7 UPDATE — Kalman Gain and Covariance](#27-update--kalman-gain-and-covariance)
  - [2.8 UPDATE — State Injection](#28-update--state-injection)
  - [2.9 Output Predictor](#29-output-predictor)
  - [2.10 Why Linearization Still Works](#210-why-linearization-still-works)
  - [2.11 Theory-to-Code Summary Table](#211-theory-to-code-summary-table)

---

# Part 1 — Source Navigation Guide

## 1.1 Two-Layer Architecture

```
src/modules/ekf2/
├── EKF2.cpp              ← Layer 1: uORB wrapper — feeds sensors, publishes results
├── EKF2Selector.cpp      ← SKIP (multi-IMU redundancy, not relevant for single-IMU)
└── EKF/
    ├── ekf.cpp               ← Layer 2 entry: Ekf::update(), predictState()
    ├── ekf.h                 ← Full class API + member layout (read for orientation)
    ├── covariance.cpp        ← predictCovariance() — calls SymPy-generated code
    ├── control.cpp           ← controlFusionModes() — dispatcher for all aid sources
    ├── ekf_helper.cpp        ← fuse(), measurementUpdate() — the two Kalman core funcs
    ├── output_predictor/     ← real-time output bridge (delayed → now)
    └── aid_sources/
        ├── gnss/gps_control.cpp
        ├── magnetometer/mag_control.cpp
        ├── barometer/baro_height_control.cpp
        ├── optical_flow/optical_flow_control.cpp
        └── ...
```

**Reading rule**: Stay in `EKF/`. Only open `EKF2.cpp` when you need to trace how sensor data enters the buffer.

**The two critical inter-layer APIs**:

| Direction | Call | Purpose |
|---|---|---|
| Sensor → EKF | `_ekf.setImuData()`, `setGpsData()`, `setBaroData()`, `setMagData()` | Push sample into time-ordered ring buffer |
| EKF → controller | `_attitude_pub`, `_local_position_pub` | Post-output-predictor state for real-time use |

---

## 1.2 File Table: Read / Skip

### Core algorithm — read in this order

| File | Priority | Why |
|---|---|---|
| `EKF/python/ekf_derivation/generated/state.h` | **Read first** | State vector layout, P index offsets; all other code references these constants |
| `EKF/ekf.cpp` — `update()` line 138 | **Read carefully** | The 4-line main loop; sets execution order |
| `EKF/ekf.cpp` — `predictState()` line 231 | **Read carefully** | ~50 lines of IMU integration; each line maps to a kinematics equation |
| `EKF/ekf_helper.cpp` — `fuse()` line 736 | **Read carefully** | State correction; quaternion is left-multiplicative, all others additive |
| `EKF/ekf_helper.cpp` — `measurementUpdate()` line 1089 | **Read carefully** | Joseph-stabilized covariance update; used by every single sensor |
| `EKF/control.cpp` — `controlFusionModes()` line 46 | Skim | Dispatcher; note the `#if CONFIG_EKF2_*` compile guards |
| `EKF/covariance.cpp` — `predictCovariance()` line 113 | Skim | Sets up noise vars, calls `sym::PredictCovariance`, then adds per-group process noise |
| `EKF/python/ekf_derivation/derivation.py` | When you need Jacobians | SymPy source for F and H; do not read the generated C++ directly |
| `EKF/output_predictor/output_predictor.cpp` | Read conceptually | Understand the delayed vs. real-time horizon split |

### Aid sources — GPS quadrotor

| File | Read? | What it does |
|---|---|---|
| `aid_sources/gnss/gps_control.cpp` | Read | GPS velocity + position fusion |
| `aid_sources/magnetometer/mag_control.cpp` | Read | Yaw / heading from mag |
| `aid_sources/barometer/baro_height_control.cpp` | Read | Barometric altitude |
| `aid_sources/optical_flow/optical_flow_control.cpp` | Optional | If flow sensor present |
| `velocity_fusion.cpp`, `position_fusion.cpp` | Skim | Compute K vector, call `measurementUpdate` |

### Skip entirely

| File / Folder | Reason |
|---|---|
| `EKF2Selector.cpp` | Multi-IMU selection — unrelated to filter math |
| `aid_sources/airspeed/` | Fixed-wing only |
| `aid_sources/sideslip/` | Fixed-wing only |
| `aid_sources/drag/` | Fixed-wing only |
| `aid_sources/ZeroVelocityUpdate*` | Only active when vehicle is detected as stationary |
| `aid_sources/ZeroGyroUpdate*` | Only when `EKF2_IMU_CTRL` enables gyro-bias zero update |

### EKF2.cpp wrapper — need-to-know only

| What | Location | Notes |
|---|---|---|
| Run trigger | `_sensor_combined_sub.registerCallback()` | Fires on new IMU |
| IMU push | ~line 603 | `_ekf.setImuData(imu_sample_new)` |
| GPS push | ~line 2451 | `_ekf.setGpsData(gnss_sample)` |
| Call EKF | ~line 700 | `_ekf.update()` |
| Publish attitude | ~line 1021 | `_attitude_pub.publish(att)` from output predictor |
| Publish position | ~line 1540 | `_local_position_pub.publish(lpos)` |

---

## 1.3 Function Map with Line Numbers

```
Ekf::update()                              ekf.cpp:138
  │
  ├─ predictCovariance(imu_delayed)        covariance.cpp:113   ← runs BEFORE predictState
  ├─ predictState(imu_delayed)             ekf.cpp:231
  ├─ controlFusionModes(imu_delayed)       control.cpp:46
  │   ├─ controlMagFusion(imu_delayed)     mag_control.cpp
  │   ├─ controlOpticalFlowFusion(...)     optical_flow_control.cpp
  │   ├─ controlGpsFusion(imu_delayed)     gps_control.cpp
  │   ├─ controlAirDataFusion(...)         [skip: fixed-wing]
  │   ├─ controlHeightFusion(imu_delayed)  height_control.cpp  ← always compiled, no guard
  │   ├─ controlGravityFusion(...)         gravity_fusion.cpp
  │   └─ controlExternalVisionFusion(...)  ev_control.cpp
  │
  └─ output_predictor.correctOutputStates(...)   output_predictor.cpp
```

**Key invariant**: every aid source follows the same 3-step pattern:

```
1. updateX()      — compute innovation ν = z − h(x_nom), innov_var S, test_ratio
2. gate check     — if test_ratio < 1.0, proceed
3. measurementUpdate(K, H, R, ν) → fuse(K, ν)
```

The struct `estimator_aid_source*d_s` carries `innovation`, `innovation_variance`, `test_ratio`, `fused`, `innovation_rejected` — also published as uORB topics for logging.

---

## 1.4 Quick Paths by Question

**"How is GPS velocity fused?"**
1. `control.cpp:115` → `controlGpsFusion()` (inside `#if CONFIG_EKF2_GNSS`)
2. `gps_control.cpp` → `updateGnssVel()` computes innovation
3. `velocity_fusion.cpp` → `fuseVelocity()` computes K (one axis at a time)
4. `ekf_helper.cpp:1089` → `measurementUpdate(K, H, R, innov)` updates P
5. `ekf_helper.cpp:736` → `fuse(K, innov)` updates `_state.vel`

**"How does gyro bias anti-windup work?"**
1. `ekf_helper.cpp::fuse()` — `gyro_bias -= K[...] * innov` then clamped to `±getGyroBiasLimit()`
2. `ekf.h::clearInhibitedStateKalmanGains()` — zeroes K rows for inhibited states
3. Member `_gyro_bias_inhibit[3]` — set when accel clipping or bad accel detected

**"When GPS is lost, what happens?"**
1. `gps_control.cpp::stopGnssFusion()` — clears `_control_status.flags.gps`
2. `control.cpp` — `controlFakePosFusion()` activates to prevent horizontal drift
3. `ekf.h` — `_time_last_horizontal_aiding` → `updateDeadReckoningStatus()` → publishes dead-reckoning warning

**"Where does the output predictor get real-time attitude from?"**
1. `output_predictor.cpp::calculateOutputStates(imu_now)` — runs every new IMU (high rate)
2. `output_predictor.cpp::correctOutputStates(...)` — runs after `Ekf::update()` (delayed rate)
3. `EKF2.cpp::publishAttitude()` → reads `_ekf.output_predictor().getState()` (newest buffer entry)

**"Where is Jacobian H for GPS position?"**
1. `gps_control.cpp::updateGnssPos()` — H is implicitly identity on `State::pos` rows
2. K is computed as `P.slice<3,3>(State::pos.idx, State::pos.idx)` / innov_var (sequential scalar)
3. More complex H (magnetometer): `#include <ekf_derivation/generated/compute_mag_innov_var_and_hx.h>`

---

## 1.5 `imuSample` — Fields and Data Flow

Every function in the EKF core takes `const imuSample &imu_delayed` as its primary argument. Understanding what each field is — and is not — is essential before reading any of these functions.

**Definition**: [src/modules/ekf2/EKF/common.h:175](../../../src/modules/ekf2/EKF/common.h)

```cpp
struct imuSample {
    uint64_t time_us{};               ///< timestamp of the measurement (μs)
    Vector3f delta_ang{};             ///< delta angle,  body frame — integrated gyro   (rad)
    Vector3f delta_vel{};             ///< delta velocity, body frame — integrated accel (m/s)
    float    delta_ang_dt{};          ///< integration period for delta_ang (s)
    float    delta_vel_dt{};          ///< integration period for delta_vel (s)
    bool     delta_vel_clipping[3]{}; ///< true per axis if accel saturated during this sample
};
```

### Why deltas, not raw rates?

IMU hardware does not output instantaneous rad/s and m/s². It outputs **pre-integrated** increments over a sampling window — the integration is done inside the IMU firmware or the driver:

$$\Delta\boldsymbol{\theta} = \int_{t_0}^{t_1}\boldsymbol{\omega}\,dt \quad\text{[rad]}, \qquad \Delta\mathbf{v} = \int_{t_0}^{t_1}\mathbf{a}\,dt \quad\text{[m/s]}$$

`delta_ang` is a rotation vector (rad), not an angular velocity. `delta_vel` is a velocity increment (m/s), not an acceleration.

Two hardware paths in [EKF2.cpp:603–668](../../../src/modules/ekf2/EKF2.cpp):

```cpp
// Path A — hardware already provides pre-integrated deltas (vehicle_imu_s)
imu_sample_new.delta_ang    = Vector3f{imu.delta_angle};           // rad
imu_sample_new.delta_ang_dt = imu.delta_angle_dt * 1e-6f;          // s
imu_sample_new.delta_vel    = Vector3f{imu.delta_velocity};        // m/s
imu_sample_new.delta_vel_dt = imu.delta_velocity_dt * 1e-6f;       // s

// Path B — driver provides raw rate/accel; driver multiplies by dt
imu_sample_new.delta_ang    = Vector3f{sensor_combined.gyro_rad}
                              * imu_sample_new.delta_ang_dt;        // rad/s × s = rad
imu_sample_new.delta_vel    = Vector3f{sensor_combined.accelerometer_m_s2}
                              * imu_sample_new.delta_vel_dt;        // m/s² × s = m/s
```

### Field-by-field reference

| Field | Unit | Mathematical meaning | Used in |
|---|---|---|---|
| `time_us` | μs | Timestamp of this sample | Time-aligning all sensor buffers |
| `delta_ang` | rad | $\Delta\boldsymbol{\theta}=\int\boldsymbol{\omega}\,dt$ — body-frame rotation this step | `predictState()` F1: `Quatf dq(AxisAnglef{corrected_delta_ang})` |
| `delta_ang_dt` | s | Integration window for `delta_ang` | Bias scale: `getGyroBias() * delta_ang_dt`; $\Delta t$ for F_θbg |
| `delta_vel` | m/s | $\Delta\mathbf{v}=\int\mathbf{a}\,dt$ — body-frame velocity increment this step | `predictState()` F2: `_state.vel += _R_to_earth * corrected_delta_vel` |
| `delta_vel_dt` | s | Integration window for `delta_vel` | Bias scale: `getAccelBias() * delta_vel_dt`; $\Delta t$ for gravity/coriolis |
| `delta_vel_clipping[3]` | bool | Accel saturated on axis x/y/z | `predictCovariance()`: inflate `accel_var` to `sq(BADACC_BIAS_PNOISE)` |

### The delta vs. rate distinction in the EKF

The same sample is used two different ways depending on context:

**`predictState()` — use `delta_ang` directly** (correct: it is already the rotation vector to apply):

```cpp
// ekf.cpp:245
// corrected_delta_ang = delta_ang - bias*dt - earth_rate_correction
const Quatf dq(AxisAnglef{corrected_delta_ang});   // delta_ang in rad → quaternion increment
_state.quat_nominal = (_state.quat_nominal * dq).normalized();
```

**`predictCovariance()` — divide back to get rate for Jacobian F** (F is evaluated at current angular rate, not at the angle):

```cpp
// covariance.cpp:137
// sym::PredictCovariance needs instantaneous rate to evaluate F_θθ = I - [ω·Δt]×
P = sym::PredictCovariance(
    _state.vector(), P,
    imu_delayed.delta_vel / imu_delayed.delta_vel_dt,   // m/s ÷ s = m/s²  (accel rate for F_vθ)
    accel_var,
    imu_delayed.delta_ang / imu_delayed.delta_ang_dt,   // rad ÷ s = rad/s  (ω for F_θθ)
    gyro_var,
    dt);
```

### Why `imu_delayed` and not the latest IMU?

`imu_delayed = _imu_buffer.get_oldest()` — the **oldest** entry in the ring buffer, not the newest. The buffer holds `_imu_buffer_length` downsampled samples:

```
_time_delayed_us = _imu_buffer.get_oldest().time_us

Lag ≈ imu_buffer_length × filter_update_period_s
    ≈ 10–20 samples × 4 ms = 40–80 ms
```

This lag exists so that GPS (10 Hz), mag (100 Hz), and baro (50 Hz) data can all arrive and be placed in their own observation buffers before the EKF tries to fuse them. The EKF fuses all sensors at the **same delayed time horizon**, ensuring consistent time-alignment. The output predictor then forward-propagates the corrected estimate from `t_delayed` to `t_now` using the buffered fresh IMU samples that arrived in the meantime.

```
Raw IMU at t_now ──────────────────────────────────────►
                                                        output_predictor
                                                        (calculateOutputStates)

Downsampled IMU → ring buffer → imu_delayed = oldest
                                     │
                                     ▼
                              Ekf::update()
                          predictCovariance + predictState + controlFusionModes
```

The `output_predictor.calculateOutputStates()` is called on every **raw** (non-downsampled) IMU sample in `EstimatorInterface::setIMUData()` at line 89 — before the downsampler. This is what gives the real-time output its low latency.

---

# Part 2 — ESKF Theory with Code

## 2.1 Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────┐
│                   ERROR-STATE EKF (ESKF) — OVERVIEW                  │
│                                                                      │
│  Core identity:  x_true = x_nom ⊞ δx                                │
│                                                                      │
│  ┌─────────────────────────────┐   ┌──────────────────────────────┐  │
│  │     NOMINAL STATE x_nom     │   │    ERROR STATE δx ∈ R^24     │  │
│  │     lives on the manifold   │   │    lives in flat tangent     │  │
│  │                             │   │    space — standard KF       │  │
│  │  q ∈ SO(3)  (unit sphere)   │   │    δθ ∈ R^3  (rotation vec) │  │
│  │  v, p ∈ R^3                 │   │    δv, δp ∈ R^3             │  │
│  │  biases, mag, wind, terrain │   │    δb_g, δb_a, δm_I, …      │  │
│  │                             │   │                              │  │
│  │  Updated by:                │   │  Covariance P (24×24)        │  │
│  │  full nonlinear kinematics  │   │  maintained by linear KF     │  │
│  └─────────────────────────────┘   └──────────────────────────────┘  │
│                                                                      │
│  PREDICT                                                             │
│    x_nom ← f(x_nom)         [ekf.cpp: predictState()]               │
│    P     ← F·P·F^T + Q      [covariance.cpp: predictCovariance()]   │
│                                                                      │
│  UPDATE  (per sensor)                                                │
│    ν = z − h(x_nom)         [aid_sources/*/control.cpp]             │
│    K = P·H^T·(H·P·H^T+R)^-1 [ekf_helper.cpp: measurementUpdate()]  │
│    x_nom ← x_nom ⊞ K·ν      [ekf_helper.cpp: fuse()]               │
│    P ← (I−KH)P(I−KH)^T+KRK^T                                       │
│                                                                      │
│  OUTPUT BRIDGE                                                       │
│    propagate corrected state from t_delayed → t_now                 │
│    [output_predictor/output_predictor.cpp]                           │
└──────────────────────────────────────────────────────────────────────┘

Time horizons:
  IMU now ─────────────────────────────► t_now   (controller reads here)
  EKF delayed ──► t_delayed (~100 ms)             (EKF corrects here)
```

The `Ekf::update()` loop maps exactly to this diagram:

```cpp
// ekf.cpp:138 — Ekf::update()  (verified against source)
predictCovariance(imu_sample_delayed);            // P predict  ← runs FIRST
predictState(imu_sample_delayed);                 // x predict  ← runs SECOND
controlFusionModes(imu_sample_delayed);           // UPDATE for each sensor
_output_predictor.correctOutputStates(...);       // bridge delayed → now
```

> **Note on order**: `predictCovariance` runs before `predictState`. The Jacobian F is evaluated at the *previous* nominal state, then the nominal state is updated. This is the standard EKF convention: covariance propagation uses the pre-step Jacobian.

---

## 2.2 Why Error-State EKF

### Problem 1 — Nonlinear dynamics

The KF's closure guarantee (Gaussian → Gaussian through the filter) requires **Property 1: linear transform of a Gaussian is Gaussian**. Real drone dynamics break this:

$$\mathbf{q}_{k+1} = \mathbf{q}_k \otimes \Delta\mathbf{q}(\boldsymbol{\omega})\,, \qquad \mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{R}(\mathbf{q}_k)\,\mathbf{a}\,\Delta t + \mathbf{g}\,\Delta t$$

The rotation matrix $\mathbf{R}(\mathbf{q})$ is quadratic in the quaternion components — after passing a Gaussian $\mathbf{q}$ through $\mathbf{R}(\cdot)$, the result is non-Gaussian. The EKF solution is a first-order Taylor expansion at each step.

### Problem 2 — SO(3) manifold

Attitude lives on the **Special Orthogonal group** SO(3), not $\mathbb{R}^n$. A unit quaternion $\mathbf{q} = (q_w,q_x,q_y,q_z)$ with $\|\mathbf{q}\|=1$ has:
- 4 stored numbers, but only 3 degrees of freedom
- Composition by multiplication $\mathbf{q}_1\otimes\mathbf{q}_2$, not addition

Naively adding a Gaussian perturbation $\mathbf{q}+\delta\mathbf{q}$ violates the unit-norm constraint: the result is no longer a valid rotation. Consequence: if you force the KF to operate on the 4-component quaternion directly, the covariance is rank-deficient, the Kalman gain corrupts the norm, and renormalizing destroys the probabilistic meaning of $P$.

### ESKF solution

Split the state:

$$\mathbf{x}_\text{true} = \mathbf{x}_\text{nom} \boxplus \delta\mathbf{x}$$

| Component | Space | Update rule | Role |
|---|---|---|---|
| $\mathbf{x}_\text{nom}$ | SO(3) × $\mathbb{R}^{21}$ | Full nonlinear kinematics | Best current guess |
| $\delta\mathbf{x} \in \mathbb{R}^{24}$ | Flat tangent space | Linear Gaussian KF | Uncertainty around guess |

For attitude specifically:

$$\mathbf{q}_\text{true} = \mathbf{q}_\text{nom} \otimes \delta\mathbf{q}(\delta\boldsymbol{\theta})\,, \qquad \delta\mathbf{q}(\delta\boldsymbol{\theta}) \approx \begin{bmatrix}1\\\delta\boldsymbol{\theta}/2\end{bmatrix}$$

$\delta\boldsymbol{\theta} \in \mathbb{R}^3$ is a rotation vector — always small by design (reset to zero after every update) — so the linearization of the error dynamics is always accurate regardless of the nominal attitude magnitude.

| Aspect | Naive EKF on $\mathbf{q}\in\mathbb{R}^4$ | ESKF on $\delta\boldsymbol{\theta}\in\mathbb{R}^3$ |
|---|---|---|
| Covariance | 4×4, rank-deficient | 3×3, full rank |
| Norm constraint | Violated by KF update | Enforced: $\mathbf{q}_\text{nom}$ never leaves the sphere |
| Linearization point | Around current (possibly large) $\mathbf{q}$ | Always around $\delta\mathbf{x}=0$ (always small) |

---

## 2.3 The 24-State Error Vector

The error state $\delta\mathbf{x}\in\mathbb{R}^{24}$ is the vector whose covariance matrix $\mathbf{P}$ (24×24) the filter maintains. The nominal state $\mathbf{x}_\text{nom}$ is a parallel `StateSample` struct in physical units.

$$\delta\mathbf{x} = \begin{bmatrix}
\delta\boldsymbol{\theta} \\ \delta\mathbf{v} \\ \delta\mathbf{p} \\ \delta\mathbf{b}_g \\ \delta\mathbf{b}_a \\ \delta\mathbf{m}_I \\ \delta\mathbf{m}_B \\ \delta\mathbf{w} \\ \delta h
\end{bmatrix} \in\mathbb{R}^{24}$$

| Component | Indices | DoF | Physical meaning | Why included |
|---|---|---|---|---|
| $\delta\boldsymbol{\theta}$ | 0–2 | 3 | Attitude error (rotation vector, body frame) | Minimum for dead-reckoning |
| $\delta\mathbf{v}$ | 3–5 | 3 | NED velocity error (m/s) | Minimum for dead-reckoning |
| $\delta\mathbf{p}$ | 6–8 | 3 | NED position error (m) | Minimum for dead-reckoning |
| $\delta\mathbf{b}_g$ | 9–11 | 3 | Gyroscope bias error (rad/s) | Without it: attitude drifts unboundedly |
| $\delta\mathbf{b}_a$ | 12–14 | 3 | Accelerometer bias error (m/s²) | Without it: position error ~ ½·b·t² |
| $\delta\mathbf{m}_I$ | 15–17 | 3 | Earth magnetic field error (Gauss) | Location-dependent; unknown a priori |
| $\delta\mathbf{m}_B$ | 18–20 | 3 | Body magnetic bias (Gauss) | Motors/ESCs distort local field |
| $\delta\mathbf{w}$ | 21–22 | 2 | Wind velocity error NE (m/s) | Needed to reconcile airspeed vs ground speed |
| $\delta h$ | 23 | 1 | Terrain height AGL error (m) | Needed for rangefinder altitude fusion |
| **Total** | | **24** | | $24^2 = 576$ floats for P (2.25 KB at float32) |

> **File**: [src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h](../../../src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h) — auto-generated, do not edit manually.

```cpp
// state.h (auto-generated from derivation.py via SymPy)
struct StateSample {
    matrix::Quaternion<float> quat_nominal{};  // 4 floats stored, 3 DoF in error state
    matrix::Vector3<float>    vel{};
    matrix::Vector3<float>    pos{};           // only z stored here; x,y live in _gpos
    matrix::Vector3<float>    gyro_bias{};
    matrix::Vector3<float>    accel_bias{};
    matrix::Vector3<float>    mag_I{};
    matrix::Vector3<float>    mag_B{};
    matrix::Vector2<float>    wind_vel{};
    float                     terrain{};
};

namespace State {
    static constexpr IdxDof quat_nominal{0,  3};  // idx=0, dof=3 in P (not 4)
    static constexpr IdxDof vel        {3,  3};
    static constexpr IdxDof pos        {6,  3};
    static constexpr IdxDof gyro_bias  {9,  3};
    static constexpr IdxDof accel_bias {12, 3};
    static constexpr IdxDof mag_I      {15, 3};
    static constexpr IdxDof mag_B      {18, 3};
    static constexpr IdxDof wind_vel   {21, 2};
    static constexpr IdxDof terrain    {23, 1};
    static constexpr uint8_t size{24};   // P is SquareMatrix<float,24>
};
```

**Why `quat_nominal.dof = 3` despite 4 stored floats**: The quaternion norm constraint removes one degree of freedom. The covariance $P$ tracks the 3-dimensional rotation vector error $\delta\boldsymbol{\theta}$, not the 4-component quaternion. Position x,y are stored in `_gpos` (type `LatLonAlt`) to avoid floating-point accumulation error at large distances.

---

## 2.4 PREDICT — Nominal State Propagation

The nominal state is propagated by the full nonlinear kinematics — no linearization here. This is `predictState()` in [ekf.cpp:231](../../../src/modules/ekf2/EKF/ekf.cpp).

The corrected IMU measurements (after bias removal):

$$\boldsymbol{\omega}_c = \tilde{\boldsymbol{\omega}} - \hat{\mathbf{b}}_g - \mathbf{R}^\top\boldsymbol{\omega}_e\,, \qquad \mathbf{a}_c = \tilde{\mathbf{a}} - \hat{\mathbf{b}}_a$$

where $\boldsymbol{\omega}_e$ is the Earth rotation rate in NED frame (computed and cached when latitude changes by > 1°).

---

### Eq. F1 — Attitude Kinematics

$$\boxed{\hat{\mathbf{q}}_{k+1} = \hat{\mathbf{q}}_k \otimes \Delta\mathbf{q}(\boldsymbol{\omega}_c\,\Delta t)}$$

where $\Delta\mathbf{q}(\boldsymbol{\phi}) = \begin{bmatrix}\cos(\|\boldsymbol{\phi}\|/2)\\\sin(\|\boldsymbol{\phi}\|/2)\,\hat{\boldsymbol{\phi}}\end{bmatrix}$; for small $\|\boldsymbol{\phi}\|$: $\Delta\mathbf{q}\approx\begin{bmatrix}1\\\boldsymbol{\phi}/2\end{bmatrix}$.

| Symbol | Meaning |
|---|---|
| $\hat{\mathbf{q}}_k\in\mathbb{H}_1$ | Nominal quaternion at step $k$, $\|\mathbf{q}\|=1$ |
| $\otimes$ | Quaternion multiplication (Hamilton product) |
| $\boldsymbol{\omega}_c$ | Angular rate with gyro bias and Earth rate removed |
| $\Delta t$ | `imu_delayed.delta_ang_dt` |

> **File**: [ekf.cpp:238–249](../../../src/modules/ekf2/EKF/ekf.cpp)

```cpp
// ekf.cpp:238 — predictState()
// Eq. F1: q_nom ← q_nom ⊗ Δq(ω_corrected · Δt)

// Step 1: remove bias and Earth rate from gyro measurement
const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;
corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_delayed.delta_ang_dt;

// Step 2: build Δq from rotation vector, then right-multiply
const Quatf dq(AxisAnglef{corrected_delta_ang});
_state.quat_nominal = (_state.quat_nominal * dq).normalized();
_R_to_earth = Dcmf(_state.quat_nominal);   // update DCM immediately
```

---

### Eq. F2 — Velocity Kinematics

$$\boxed{\hat{\mathbf{v}}_{k+1} = \hat{\mathbf{v}}_k + \mathbf{R}(\hat{\mathbf{q}}_k)\,\mathbf{a}_c\,\Delta t + \mathbf{g}\,\Delta t}$$

The rotation matrix $\mathbf{R}(\mathbf{q})$ in terms of quaternion components $[q_w,q_x,q_y,q_z]$:

$$\mathbf{R}(\mathbf{q}) = \begin{bmatrix}
1-2(q_y^2+q_z^2) & 2(q_xq_y-q_wq_z) & 2(q_xq_z+q_wq_y) \\
2(q_xq_y+q_wq_z) & 1-2(q_x^2+q_z^2) & 2(q_yq_z-q_wq_x) \\
2(q_xq_z-q_wq_y) & 2(q_yq_z+q_wq_x) & 1-2(q_x^2+q_y^2)
\end{bmatrix}$$

This is the primary source of nonlinearity — $\mathbf{R}$ is quadratic in the quaternion components.

| Symbol | Meaning |
|---|---|
| $\mathbf{R}(\hat{\mathbf{q}}_k) = $ `_R_to_earth` | DCM body→NED, updated at the end of F1 |
| $\mathbf{a}_c$ | Corrected specific force in body frame |
| $\mathbf{g}=[0,0,+g]^\top$ | NED convention: $+Z$ is down, $g\approx 9.81$ m/s² |
| `gravity_acceleration` | `(0, 0, CONSTANTS_ONE_G)` — note: "simplistic model" comment in source |

> **File**: [ekf.cpp:252–266](../../../src/modules/ekf2/EKF/ekf.cpp)

```cpp
// ekf.cpp:252 — predictState() continued
// Eq. F2: v ← v + R(q)·a_corrected·Δt + g·Δt + coriolis + transport_rate

const Vector3f delta_vel_bias_scaled  = getAccelBias() * imu_delayed.delta_vel_dt;
const Vector3f corrected_delta_vel    = imu_delayed.delta_vel - delta_vel_bias_scaled;
const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel; // body → NED

const Vector3f vel_last = _state.vel;          // saved for trapezoidal position update
_state.vel += corrected_delta_vel_ef;          // R(q)·a·Δt

// Additional corrections (all small; fully computed)
const Vector3f gravity_acceleration(0.f, 0.f, CONSTANTS_ONE_G); // simplistic model
const Vector3f coriolis_acceleration = -2.f * _earth_rate_NED.cross(vel_last);
const Vector3f transport_rate = -_gpos.computeAngularRateNavFrame(vel_last).cross(vel_last);
_state.vel += (gravity_acceleration + coriolis_acceleration + transport_rate) * imu_delayed.delta_vel_dt;

// Velocity is clamped to ±_params.velocity_limit after this
```

---

### Eq. F3 — Position Kinematics

$$\boxed{\hat{\mathbf{p}}_{k+1} = \hat{\mathbf{p}}_k + \tfrac{1}{2}(\hat{\mathbf{v}}_k + \hat{\mathbf{v}}_{k+1})\,\Delta t}$$

Trapezoidal integration — more accurate than Euler for velocity integration over one IMU step.

| Symbol | Meaning |
|---|---|
| `_gpos` | `LatLonAlt` object accumulating global x,y; avoids float precision loss at long range |
| `_state.pos(2)` | Only z (Down) is stored in the state; x,y are in `_gpos` |

> **File**: [ekf.cpp:268–270](../../../src/modules/ekf2/EKF/ekf.cpp)

```cpp
// ekf.cpp:268 — predictState() continued
// Eq. F3: p ← p + ½(v_prev + v_new)·Δt  (trapezoidal)
_gpos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;
_state.pos(2) = -_gpos.altitude();   // Down = negative altitude
```

---

### Eq. F4–F8 — Bias and Auxiliary States

All biases and auxiliary states are **random walks** — no deterministic evolution:

$$\hat{\mathbf{b}}_{g,k+1} = \hat{\mathbf{b}}_{g,k}\,, \quad \hat{\mathbf{b}}_{a,k+1} = \hat{\mathbf{b}}_{a,k}\,, \quad \hat{\mathbf{m}}_{I,k+1} = \hat{\mathbf{m}}_{I,k}\,, \quad \ldots$$

Their uncertainty growth is captured entirely by the process noise $\mathbf{Q}$ added in the covariance step. `predictState()` does not touch these fields.

---

## 2.5 PREDICT — Covariance Propagation

The linearized error dynamics around $\delta\mathbf{x}=0$:

$$\delta\mathbf{x}_{k+1} \approx \mathbf{F}_k\,\delta\mathbf{x}_k + \mathbf{G}_k\,\mathbf{n}_k$$

By Property 1 + Property 2 (linear transform + sum of independent Gaussians):

$$\boxed{\mathbf{P}_{k+1} = \mathbf{F}_k\,\mathbf{P}_k\,\mathbf{F}_k^\top + \mathbf{Q}}$$

| Symbol | Size | Meaning |
|---|---|---|
| $\mathbf{F}_k$ | $24\times 24$ | State transition Jacobian; most off-diagonal blocks are zero |
| $\mathbf{Q}$ | $24\times 24$ | Discrete process noise; includes IMU noise + random-walk noise |
| $\mathbf{P}$ | $24\times 24$ | Error covariance; symmetric positive semi-definite |

### Jacobian F — block structure

$\mathbf{F}_k$ is sparse. The non-zero off-diagonal blocks derive from the kinematic equations:

#### $\mathbf{F}_{\theta\theta}$ — attitude self-rotation correction

From error quaternion propagation ($\delta\mathbf{q}$ conjugated by the nominal rotation $\Delta\mathbf{q}$):

$$\boxed{\mathbf{F}_{\theta\theta} = \mathbf{I} - [\Delta\boldsymbol{\phi}]_\times\,, \qquad \Delta\boldsymbol{\phi} = \boldsymbol{\omega}_c\,\Delta t}$$

| Symbol | Meaning |
|---|---|
| $[\Delta\boldsymbol{\phi}]_\times$ | Skew-symmetric matrix of $\Delta\boldsymbol{\phi}$; $[\mathbf{u}]_\times\mathbf{v}=\mathbf{u}\times\mathbf{v}$ |
| $\Delta\boldsymbol{\phi}$ | Nominal rotation vector over one step; small at ≥250 Hz |

**Physical meaning**: The attitude error rotates with the nominal trajectory. The correction $-[\Delta\boldsymbol{\phi}]_\times$ is at most 4% of identity at 250 Hz with typical drone rates.

#### $\mathbf{F}_{\theta b_g}$ — gyro bias drives attitude drift

$$\boxed{\mathbf{F}_{\theta b_g} = -\Delta t\,\mathbf{I}_{3\times 3}}$$

**Physical meaning**: A gyro bias error of 1 rad/s accumulates 1·Δt rad of attitude error per step. This is the dominant long-term drift source.

#### $\mathbf{F}_{v\theta}$ — attitude error drives velocity error

From the rotation-matrix perturbation $\mathbf{R}(\mathbf{q}_\text{true})\approx\mathbf{R}_\text{nom}(\mathbf{I}+[\delta\boldsymbol{\theta}]_\times)$ and the identity $[\mathbf{u}]_\times\mathbf{v}=-[\mathbf{v}]_\times\mathbf{u}$:

$$\boxed{\mathbf{F}_{v\theta} = -\mathbf{R}_\text{nom}\,[\mathbf{a}_b]_\times\,\Delta t\,, \qquad \mathbf{a}_b = \tilde{\mathbf{a}} - \hat{\mathbf{b}}_a}$$

| Symbol | Meaning |
|---|---|
| $\mathbf{R}_\text{nom}$ | DCM body→NED at current nominal attitude |
| $[\mathbf{a}_b]_\times$ | Skew-symmetric matrix of corrected specific force |

**Physical meaning**: An attitude error mis-projects the accelerometer into the wrong NED direction, creating a spurious velocity increment. The larger the specific force (aggressive maneuver), the stronger this coupling.

#### $\mathbf{F}_{vb_a}$ — accel bias drives velocity drift

$$\boxed{\mathbf{F}_{vb_a} = -\mathbf{R}_\text{nom}\,\Delta t}$$

**Physical meaning**: An accelerometer bias of $b_a$ m/s² causes velocity to drift at $\mathbf{R}_\text{nom}\,b_a$ m/s (after rotation to NED). Without estimating $b_a$, position error grows as $\frac{1}{2}b_a t^2$ — at 0.05 m/s² bias, that is 2.5 m in 10 seconds.

#### $\mathbf{F}_{pv}$ — velocity error drives position drift

$$\boxed{\mathbf{F}_{pv} = \Delta t\,\mathbf{I}_{3\times 3}}$$

#### All other blocks

All diagonal blocks: $\mathbf{I}$ (state carries forward). All remaining off-diagonal blocks: $\mathbf{0}$ (no direct coupling). The full $24\times 24$ structure:

$$\mathbf{F}_k = \begin{bmatrix}
(\mathbf{I}-[\Delta\boldsymbol{\phi}]_\times) & 0 & 0 & -\Delta t\mathbf{I} & 0 & 0 & 0 & 0 & 0 \\
-\mathbf{R}[\mathbf{a}_b]_\times\Delta t & \mathbf{I} & 0 & 0 & -\mathbf{R}\Delta t & 0 & 0 & 0 & 0 \\
0 & \Delta t\mathbf{I} & \mathbf{I} & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & \mathbf{I} & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & \mathbf{I} & 0 & 0 & 0 & 0 \\
\vdots & & & & & \mathbf{I} & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & \mathbf{I} & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & \mathbf{I} & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}$$

Rows/columns: $[\delta\boldsymbol{\theta}\;|\;\delta\mathbf{v}\;|\;\delta\mathbf{p}\;|\;\delta\mathbf{b}_g\;|\;\delta\mathbf{b}_a\;|\;\delta\mathbf{m}_I\;|\;\delta\mathbf{m}_B\;|\;\delta\mathbf{w}\;|\;\delta h]$, $\mathbf{R}\equiv\mathbf{R}_\text{nom}$.

### How PX4 computes $\mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$

Rather than materializing the 24×24 $\mathbf{F}$ and doing two matrix products ($24^3 = 13824$ multiply-adds), SymPy symbolically expands $\mathbf{F}\mathbf{P}\mathbf{F}^\top$ at derivation time and emits optimized C++ that directly writes each element $P(i,j)$ using common-subexpression elimination.

> **Symbolic source**: [src/modules/ekf2/EKF/python/ekf_derivation/derivation.py](../../../src/modules/ekf2/EKF/python/ekf_derivation/derivation.py)
> **Generated output**: [src/modules/ekf2/EKF/python/ekf_derivation/generated/predict_covariance.h](../../../src/modules/ekf2/EKF/python/ekf_derivation/generated/predict_covariance.h)
> **Caller**: [src/modules/ekf2/EKF/covariance.cpp:113](../../../src/modules/ekf2/EKF/covariance.cpp)

```cpp
// covariance.cpp:113 — predictCovariance() (verified against source)
void Ekf::predictCovariance(const imuSample &imu_delayed)
{
    const float dt = 0.5f * (imu_delayed.delta_vel_dt + imu_delayed.delta_ang_dt);

    // gyro and accel noise variances (accel inflated if bad data detected)
    float gyro_var = sq(_params.gyro_noise);
    Vector3f accel_var;
    for (unsigned i = 0; i < 3; i++) {
        accel_var(i) = (_fault_status.flags.bad_acc_vertical || imu_delayed.delta_vel_clipping[i])
                       ? sq(BADACC_BIAS_PNOISE) : sq(_params.accel_noise);
    }

    // CORE: generated symbolic code updates every P(i,j) in place
    // Note: passes physical rates (delta/dt), not raw deltas
    P = sym::PredictCovariance(
        _state.vector(), P,
        imu_delayed.delta_vel / imu_delayed.delta_vel_dt,  // accel in m/s²
        accel_var,
        imu_delayed.delta_ang / imu_delayed.delta_ang_dt,  // angular rate in rad/s
        gyro_var,
        dt);

    // Process noise for random-walk states added SEPARATELY (not inside sym::)
    // gyro bias
    const float gyro_bias_pn = sq(dt * _params.gyro_bias_p_noise);
    for (unsigned i = State::gyro_bias.idx; i < State::gyro_bias.idx + State::gyro_bias.dof; i++)
        if (P(i, i) < gyro_var) P(i, i) += gyro_bias_pn;

    // accel bias
    const float accel_bias_pn = sq(dt * _params.accel_bias_p_noise);
    for (unsigned i = State::accel_bias.idx; i < State::accel_bias.idx + State::accel_bias.dof; i++)
        if (P(i, i) < accel_var(i - State::accel_bias.idx)) P(i, i) += accel_bias_pn;

    // mag_I, mag_B, wind_vel, terrain: similar pattern (guarded by CONFIG_EKF2_* flags)
    // ...
    constrainStateVariances();  // clamp diagonal; fires only on numerical pathology
}
```

**Process noise parameters** (configured via QGroundControl):

| Parameter | Type | Feeds into |
|---|---|---|
| `EKF2_GYR_NOISE` | $\sigma_g$ (rad/s/√Hz) | $Q_{\theta\theta}$ via `sym::PredictCovariance` |
| `EKF2_ACC_NOISE` | $\sigma_a$ (m/s²/√Hz) | $Q_{vv}$ via `sym::PredictCovariance` |
| `EKF2_GYR_B_NOISE` | $\sigma_{bg}$ (rad/s²/√Hz) | $Q_{b_g b_g}$ added after |
| `EKF2_ACC_B_NOISE` | $\sigma_{ba}$ (m/s³/√Hz) | $Q_{b_a b_a}$ added after |
| `EKF2_MAG_NOISE` | $\sigma_m$ (Gauss/√Hz) | $Q_{m_I},Q_{m_B}$ added after |

---

## 2.6 UPDATE — Observation Models

For each sensor, the UPDATE step requires: (1) an observation function $h(\mathbf{x}_\text{nom})$ that predicts the reading, and (2) its Jacobian $\mathbf{H}=\partial h/\partial\delta\mathbf{x}$ mapping the error state to measurement error.

PX4 fuses **one scalar at a time** (sequential scalar fusion). For a GPS with 3 position components, `measurementUpdate()` is called 3 times. Each call: $\mathbf{H}\in\mathbb{R}^{1\times 24}$, $\mathbf{K}\in\mathbb{R}^{24\times 1}$, $S = \mathbf{H}\mathbf{P}\mathbf{H}^\top + R$ is a scalar. This eliminates the $m\times m$ matrix inversion, reducing cost from $O(m^3)$ to $O(1)$ per measurement.

### GPS Position

Predicted measurement: $\hat{\mathbf{z}} = \hat{\mathbf{p}}_\text{nom}$ (position directly from nominal state).

$$\mathbf{H}_\text{GPS\text{-}pos} = \begin{bmatrix}\mathbf{0}_{3\times 6} & \mathbf{I}_3 & \mathbf{0}_{3\times 15}\end{bmatrix} \in \mathbb{R}^{3\times 24}$$

Columns 6–8 (position error $\delta\mathbf{p}$) are identity; all others zero.

Innovation: $\boldsymbol{\nu} = \mathbf{z}_\text{GPS} - \hat{\mathbf{p}}_\text{nom}$

> **File**: [src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp](../../../src/modules/ekf2/EKF/aid_sources/gnss/gps_control.cpp) — `updateGnssPos()`

```cpp
// gps_control.cpp — GPS position: H is implicit (identity on pos rows)
// For scalar fusion, Kalman gain K_i = P.col(State::pos.idx + i) / S
// where S = P(State::pos.idx+i, State::pos.idx+i) + R_pos
// Then measurementUpdate(K, H_vec, R_pos, innov) is called per axis
```

### GPS Velocity

Predicted measurement: $\hat{\mathbf{z}} = \hat{\mathbf{v}}_\text{nom}$ (velocity directly).

$$\mathbf{H}_\text{GPS\text{-}vel} = \begin{bmatrix}\mathbf{0}_{3\times 3} & \mathbf{I}_3 & \mathbf{0}_{3\times 18}\end{bmatrix} \in \mathbb{R}^{3\times 24}$$

Columns 3–5 (velocity error $\delta\mathbf{v}$) are identity.

### Barometer Altitude

Predicted measurement: $\hat{z} = \hat{p}_D$ (Down component of position, scalar).

$$\mathbf{H}_\text{baro} = \begin{bmatrix}0,\ldots,0,\underbrace{1}_{\text{index 8}},0,\ldots,0\end{bmatrix} \in \mathbb{R}^{1\times 24}$$

> **File**: [src/modules/ekf2/EKF/aid_sources/barometer/baro_height_control.cpp](../../../src/modules/ekf2/EKF/aid_sources/barometer/baro_height_control.cpp)

### Magnetometer

Predicted measurement: $\hat{\mathbf{z}} = \mathbf{R}(\hat{\mathbf{q}})^\top\hat{\mathbf{m}}_I + \hat{\mathbf{m}}_B$ (field in body frame). The Jacobian couples three error-state groups:

$$\mathbf{H}_\text{mag}\big|_{\delta\boldsymbol{\theta}} = [\mathbf{R}^\top\mathbf{m}_I]_\times \quad(3\times 3,\;\text{cols 0–2})$$

$$\mathbf{H}_\text{mag}\big|_{\delta\mathbf{m}_I} = \mathbf{R}^\top \quad(\text{cols 15–17})\,, \qquad \mathbf{H}_\text{mag}\big|_{\delta\mathbf{m}_B} = \mathbf{I} \quad(\text{cols 18–20})$$

> **File**: [src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp](../../../src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp) — calls generated Jacobian:

```cpp
// mag_control.cpp — H and innovation variance from generated SymPy code
#include <ekf_derivation/generated/compute_mag_innov_var_and_hx.h>
// sym::ComputeMagInnovVarAndHx(q, P, mag_I, mag_B, R_mag, &H, &innov_var)
```

---

## 2.7 UPDATE — Kalman Gain and Covariance

When measurement $\mathbf{z}_k$ arrives:

**Innovation:**

$$\boxed{\boldsymbol{\nu}_k = \mathbf{z}_k - h(\mathbf{x}_\text{nom,k})}$$

**Innovation covariance** (Property 4 — joint Gaussian):

$$\boxed{\mathbf{S}_k = \mathbf{H}_k\,\mathbf{P}_k\,\mathbf{H}_k^\top + \mathbf{R}}$$

$\mathbf{H}\mathbf{P}\mathbf{H}^\top$: state uncertainty projected to sensor space. $\mathbf{R}$: sensor noise. For scalar fusion, $S$ is a single float — no matrix inversion, just division.

**Kalman Gain** (Property 5 — Woodbury identity):

$$\boxed{\mathbf{K}_k = \mathbf{P}_k\,\mathbf{H}_k^\top\,S_k^{-1}} \quad (24\times 1\text{ for scalar fusion})$$

**Joseph-stabilized covariance update:**

$$\boxed{\mathbf{P}_{k|k} = (\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)\,\mathbf{P}_{k|k-1}\,(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)^\top + \mathbf{K}_k\,R\,\mathbf{K}_k^\top}$$

The simple form $(\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}$ is theoretically correct only when $\mathbf{K}$ is optimal. When some Kalman gains are zeroed (e.g., for unobservable states via `clearInhibitedStateKalmanGains`), the simple form can produce a non-symmetric or indefinite $\mathbf{P}$. The Joseph form guarantees positive semi-definiteness regardless.

| Symbol | Size | Meaning |
|---|---|---|
| $\boldsymbol{\nu}_k$ | $1\times 1$ (scalar fusion) | Innovation: observed minus predicted |
| $S_k$ | $1\times 1$ | Innovation variance; denominator of K |
| $\mathbf{K}_k$ | $24\times 1$ | Kalman gain: maps scalar innovation to 24D state correction |

> **File**: [src/modules/ekf2/EKF/ekf_helper.cpp:1089](../../../src/modules/ekf2/EKF/ekf_helper.cpp) — `measurementUpdate()`

```cpp
// ekf_helper.cpp:1089 — measurementUpdate()  (verified against source)
// Arguments: K (24×1 gain), H (24×1 Jacobian stored as column), R (scalar noise), innovation (scalar)
bool Ekf::measurementUpdate(VectorState &K, const VectorState &H, const float R, const float innovation)
{
    clearInhibitedStateKalmanGains(K);  // zero K rows for inhibited states (e.g., accel clipping)

    // Joseph form — implemented in 2 passes to avoid allocating an extra 24×24 matrix.
    // Reference: Bierman (1977), "Factorization Methods for Discrete Sequential Estimation"
    //
    // P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
    //   = P_temp * (I - H^T*K^T) + K*R*K^T

    // Pass 1: P_temp = P - K*(P*H)^T  (conventional update; P temporarily asymmetric)
    // P is symmetric → P*H == H^T*P^T == H^T*P; taking the row product is faster (row-major storage)
    VectorState PH = P * H;
    for (unsigned i = 0; i < State::size; i++)
        for (unsigned j = 0; j < State::size; j++)
            P(i, j) -= K(i) * PH(j);

    // Pass 2: P = P_temp - P_temp*H^T*K^T + K*R*K^T  (restore symmetry)
    PH = P * H;
    for (unsigned i = 0; i < State::size; i++)
        for (unsigned j = 0; j <= i; j++) {
            P(i, j) = P(i, j) - PH(i) * K(j) + K(i) * R * K(j);
            P(j, i) = P(i, j);   // enforce lower-triangle symmetry
        }

    constrainStateVariances();  // diagonal clamp — fires only on numerical pathology

    fuse(K, innovation);   // apply δx̂ = K·ν to the nominal state
    return true;
}
```

---

## 2.8 UPDATE — State Injection

After computing $\delta\hat{\mathbf{x}} = \mathbf{K}\cdot\boldsymbol{\nu}$, the correction is injected into the nominal state via $\mathbf{x}_\text{nom} \leftarrow \mathbf{x}_\text{nom} \boxplus \delta\hat{\mathbf{x}}$:

$$\hat{\mathbf{q}}_{k|k} = \delta\mathbf{q}(\delta\hat{\boldsymbol{\theta}}) \otimes \hat{\mathbf{q}}_{k|k-1} \tag{multiplicative — stays on sphere}$$

$$\hat{\mathbf{v}}_{k|k} = \hat{\mathbf{v}}_{k|k-1} - K_v \cdot \nu\,, \quad \hat{\mathbf{p}}_{k|k} = \hat{\mathbf{p}}_{k|k-1} - K_p \cdot \nu\,, \quad \ldots \tag{additive}$$

| Rule | Applied to | Reason |
|---|---|---|
| **Multiplicative** (left-multiply) | $\mathbf{q}$ | Quaternion composition; `delta_quat * q_nom` keeps norm = 1 |
| **Additive** | All other states | They live in $\mathbb{R}^n$; standard vector addition |

> **File**: [src/modules/ekf2/EKF/ekf_helper.cpp:736](../../../src/modules/ekf2/EKF/ekf_helper.cpp) — `fuse()`

```cpp
// ekf_helper.cpp:736 — fuse()  (verified against source)
void Ekf::fuse(const VectorState &K, float innovation)
{
    // ATTITUDE — LEFT-multiplicative correction
    // δθ = K[0:3] * innovation  (rotation vector)
    // δq = AxisAngle(δθ) → unit quaternion  (valid, stays on sphere)
    // q ← δq ⊗ q_nom   (LEFT multiply — note: not right-multiply)
    Quatf delta_quat(matrix::AxisAnglef(
        K.slice<State::quat_nominal.dof, 1>(State::quat_nominal.idx, 0) * (-1.f * innovation)));
    _state.quat_nominal = delta_quat * _state.quat_nominal;   // ← LEFT multiply
    _state.quat_nominal.normalize();
    _R_to_earth = Dcmf(_state.quat_nominal);

    // VELOCITY — additive (clamped to ±1000 m/s)
    _state.vel = matrix::constrain(
        _state.vel - K.slice<State::vel.dof, 1>(State::vel.idx, 0) * innovation,
        -1.e3f, 1.e3f);

    // POSITION — additive into _gpos; _state.pos is zeroed (only z retained)
    const Vector3f pos_correction = K.slice<State::pos.dof, 1>(State::pos.idx, 0) * (-innovation);
    _gpos += pos_correction;
    _state.pos.zero();                       // x,y discarded from state; live in _gpos
    _state.pos(2) = -_gpos.altitude();      // z retained from global position

    // GYRO BIAS — additive + clamped
    _state.gyro_bias = matrix::constrain(
        _state.gyro_bias - K.slice<State::gyro_bias.dof, 1>(State::gyro_bias.idx, 0) * innovation,
        -getGyroBiasLimit(), getGyroBiasLimit());

    // ACCEL BIAS — additive + clamped
    _state.accel_bias = matrix::constrain(
        _state.accel_bias - K.slice<State::accel_bias.dof, 1>(State::accel_bias.idx, 0) * innovation,
        -getAccelBiasLimit(), getAccelBiasLimit());

    // MAG — additive, only when flag is set
    if (_control_status.flags.mag) {
        _state.mag_I = matrix::constrain(
            _state.mag_I - K.slice<State::mag_I.dof, 1>(State::mag_I.idx, 0) * innovation,
            -1.f, 1.f);
        _state.mag_B = matrix::constrain(
            _state.mag_B - K.slice<State::mag_B.dof, 1>(State::mag_B.idx, 0) * innovation,
            -getMagBiasLimit(), getMagBiasLimit());
    }

    // WIND — additive, only when flag is set (CONFIG_EKF2_WIND)
    // TERRAIN — additive (CONFIG_EKF2_TERRAIN)
}
```

**Three source-verified corrections vs. prior docs:**
1. Quaternion correction is **left-multiply** `delta_quat * _state.quat_nominal`, not right-multiply.
2. `_state.pos.zero()` is called after GPS correction — the x,y components of `_state.pos` are always zeroed; only z is maintained.
3. The sign convention on attitude: `K.slice(...) * (-1.f * innovation)` — note the `-1.f` factor.

---

## 2.9 Output Predictor

> **File**: [src/modules/ekf2/EKF/output_predictor/output_predictor.cpp](../../../src/modules/ekf2/EKF/output_predictor/output_predictor.cpp)

**Why it exists**: The EKF runs at the **delayed time horizon** (~100 ms lag) to allow all sensor data to arrive and be time-aligned in the buffer. But the flight controller needs attitude and velocity now (latency < 5 ms).

```
Timeline:
  ──────────────────────────────────────────────────────────► t
  │              │              │               │
  t-100ms        t-50ms         t_delayed       t_now
                                    │               │
                                EKF::update()   controller
                                corrects here   reads here
```

**Two concurrent loops:**

```
High-rate loop (every new IMU sample, no delay):
  output_predictor.calculateOutputStates(imu_now)
    → integrate q_out, v_out using latest IMU
    → store in ring buffer at t_now

Low-rate loop (after each Ekf::update(), at delayed horizon):
  output_predictor.correctOutputStates(t_delayed, q_ekf, v_ekf, gpos_ekf, b_g, b_a)
    → find output buffer entry at t_delayed
    → compute delta: EKF estimate − output state at that time
    → propagate delta forward through the entire buffer to t_now
    → result: output_buffer[t_now] = EKF-corrected + latest IMU integration
```

```cpp
// ekf.cpp:171 — the correctOutputStates call in Ekf::update()
_output_predictor.correctOutputStates(
    imu_sample_delayed.time_us,
    _state.quat_nominal, _state.vel, _gpos,
    _state.gyro_bias, _state.accel_bias);
```

The developer TODO comment in `output_predictor.cpp:281` — *"there is no guarantee that data is at delayed fusion horizon"* — is a known limitation: the time alignment is approximate under high sensor load.

---

## 2.10 Why Linearization Still Works

**What the approximation discards**: The true covariance propagation is $\int \mathbf{f}(\mathbf{x})\mathbf{f}(\mathbf{x})^\top p(\mathbf{x})d\mathbf{x}$ — no closed form for nonlinear $\mathbf{f}$. The Taylor approximation gives $\mathbf{F}\mathbf{P}\mathbf{F}^\top$, discarding terms $O(\|\mathbf{P}\|^2)$.

**Three failure modes**: (1) Overconfidence — P underestimates true uncertainty. (2) Inconsistency — reported P < actual MSE. (3) Divergence — when P is large (poor initial conditions or large sudden maneuver).

**Why EKF2 avoids these**:

| Factor | Value | Effect |
|---|---|---|
| IMU rate | 200–1000 Hz | $\Delta t \leq 5$ ms → $\|\Delta\boldsymbol{\phi}\| \leq 0.05$ rad/step (mildly nonlinear) |
| Post-convergence P | Small | Second-order terms are $O(\mathbf{P}^2) \ll O(\mathbf{P})$ |
| ESKF design | $\delta\boldsymbol{\theta}$ always reset | Linearization around $\delta\mathbf{x}=0$ is always accurate, even at large nominal attitude |

---

## 2.11 Theory-to-Code Summary Table

| ESKF concept | Equation / formula | Code location | Function |
|---|---|---|---|
| Error state dimension | $\delta\mathbf{x}\in\mathbb{R}^{24}$ | [state.h](../../../src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h) | `State::size = 24` |
| Nominal state | `StateSample` struct | [state.h](../../../src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h) | q, vel, pos, biases |
| Main loop order | predictCov → predictState → update → output | [ekf.cpp:138](../../../src/modules/ekf2/EKF/ekf.cpp) | `Ekf::update()` |
| F1 quaternion | $\mathbf{q}\leftarrow\mathbf{q}\otimes\Delta\mathbf{q}$ | [ekf.cpp:245–249](../../../src/modules/ekf2/EKF/ekf.cpp) | `predictState()` |
| F2 velocity | $\mathbf{v}\leftarrow\mathbf{v}+\mathbf{R}\mathbf{a}\Delta t+\mathbf{g}\Delta t$ | [ekf.cpp:254–266](../../../src/modules/ekf2/EKF/ekf.cpp) | `predictState()` |
| F3 position | $\mathbf{p}\leftarrow\mathbf{p}+\frac{1}{2}(\mathbf{v}+\mathbf{v}')\Delta t$ | [ekf.cpp:269–270](../../../src/modules/ekf2/EKF/ekf.cpp) | `predictState()` |
| Jacobian F (symbolic) | $\mathbf{F}=\partial\delta\mathbf{x}'/\partial\delta\mathbf{x}$ | [derivation.py](../../../src/modules/ekf2/EKF/python/ekf_derivation/derivation.py) | SymPy derivation |
| Covariance predict | $\mathbf{P}\leftarrow\mathbf{F}\mathbf{P}\mathbf{F}^\top+\mathbf{Q}$ | [covariance.cpp:137](../../../src/modules/ekf2/EKF/covariance.cpp) | `sym::PredictCovariance()` |
| Process noise (biases) | Added separately per group | [covariance.cpp:146–230](../../../src/modules/ekf2/EKF/covariance.cpp) | After `sym::PredictCovariance` |
| Sensor dispatcher | `controlFusionModes()` | [control.cpp:46](../../../src/modules/ekf2/EKF/control.cpp) | All guarded by `#if CONFIG_*` |
| Innovation | $\boldsymbol{\nu}=\mathbf{z}-h(\mathbf{x}_\text{nom})$ | `aid_sources/*/control.cpp` | Per-sensor `update*()` |
| Kalman gain | $\mathbf{K}=\mathbf{P}\mathbf{H}^\top S^{-1}$ | [ekf_helper.cpp:1089](../../../src/modules/ekf2/EKF/ekf_helper.cpp) | `measurementUpdate()` |
| Joseph form | $\mathbf{P}=(\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I}-\mathbf{K}\mathbf{H})^\top+\mathbf{K}R\mathbf{K}^\top$ | [ekf_helper.cpp:1104–1128](../../../src/modules/ekf2/EKF/ekf_helper.cpp) | `measurementUpdate()` |
| Attitude injection | $\mathbf{q}\leftarrow\delta\mathbf{q}\otimes\mathbf{q}$ (left-multiply) | [ekf_helper.cpp:739–742](../../../src/modules/ekf2/EKF/ekf_helper.cpp) | `fuse()` |
| Other state injection | additive + clamp | [ekf_helper.cpp:746–791](../../../src/modules/ekf2/EKF/ekf_helper.cpp) | `fuse()` |
| Error state reset | implicit — nominal updated, next step starts from 0 error | after `fuse()` | — |
| Real-time bridging | propagate past delayed horizon | [output_predictor.cpp](../../../src/modules/ekf2/EKF/output_predictor/output_predictor.cpp) | `correctOutputStates()` |

---
