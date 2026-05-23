# 2d. EKF2 Limitations & Improvement Directions

> **Purpose**: Document specific limitations of PX4 EKF2 (this codebase), with exact code references showing *where* each limitation manifests, and scientific literature pointing to better approaches.
>
> Read after `02_ekf2.md` and `02c_ekf2_source_guide.md`. All file paths point into `src/modules/ekf2/EKF/`.

---

## Table of Contents

1. [First-Order Linearization (EKF vs UKF/iEKF)](#1-first-order-linearization)
2. [EKF Inconsistency — Overconfident Covariance](#2-ekf-inconsistency--overconfident-covariance)
3. [IMU Integration — Simplistic Gravity Model + Downsampling](#3-imu-integration--simplistic-model--downsampling)
4. [Output Predictor — Unguaranteed Time Alignment](#4-output-predictor--unguaranteed-time-alignment)
5. [Sequential Scalar Fusion (not Batch/Iterated)](#5-sequential-scalar-fusion)
6. [Wind State — Unobservable on Quadrotor](#6-wind-state--unobservable-on-quadrotor)
7. [Covariance Clamping as Last Resort](#7-covariance-clamping-as-last-resort)
8. [Fixed Process Noise Model](#8-fixed-process-noise-model)
9. [No Formal Integrity Monitoring / FDI](#9-no-formal-integrity-monitoring--fdi)
10. [Magnetic Heading Fragility](#10-magnetic-heading-fragility)
11. [Summary Table](#11-summary-table)

---

## 1. First-Order Linearization

### What it is

EKF2 uses **Error-State EKF (ESKF)** — a first-order Taylor approximation of the nonlinear process model around the current estimate. The Jacobian $\boldsymbol{F} = \partial f / \partial \delta\boldsymbol{x}$ is recomputed every tick at the **current** estimate. When the true state diverges from the estimate (e.g., during aggressive maneuvers, large attitude errors at initialization, or GPS outage followed by re-acquisition), the linearization point is wrong and the covariance propagation becomes inaccurate.

### Where in code

The Jacobian is computed in generated code called from `covariance.cpp`:

```cpp
// covariance.cpp:38-46
#include <ekf_derivation/generated/predict_covariance.h>

// The Jacobian F is re-linearized every call at the CURRENT _state.quat_nominal
// (not at the first estimate — this is the root cause of EKF inconsistency, see §2)
void Ekf::predictCovariance(const imuSample &imu_delayed)
{
    // ... calls sym::PredictCovariance(_state.quat_nominal, _state.vel,
    //                                  _state.gyro_bias, _state.accel_bias, ...)
    // Re-evaluates F at current estimate each tick
    constrainStateVariances();
}
```

The `predictState()` integration is exact (no linearization), but the **covariance prediction** $\boldsymbol{P}_{k+1} = \boldsymbol{F}\boldsymbol{P}_k\boldsymbol{F}^\top + \boldsymbol{G}\boldsymbol{Q}\boldsymbol{G}^\top$ uses this linearized $\boldsymbol{F}$.

### Impact

- Filter may **diverge** during flip maneuvers or rapid attitude changes where $\lVert\delta\boldsymbol{\theta}\rVert$ is large.
- Covariance $\boldsymbol{P}$ shrinks faster than the true error — filter becomes overconfident (see §2).
- For typical quadcopter hover/cruise: negligible. For aggressive acrobatics: significant.

### Improvement

| Method | Description | Key Paper |
|---|---|---|
| **UKF (Unscented KF)** | Uses sigma points to propagate mean + covariance through nonlinear $f$ exactly to 3rd order | Wan & Merwe, AISTATS 2000 |
| **iEKF (Iterated EKF)** | Re-linearizes at updated estimate during measurement update | Bell & Cathey, IEEE TAC 1993 |
| **Particle Filter** | Full nonparametric distribution, handles multimodal | Thrun et al., *Probabilistic Robotics* 2005 |

UKF has been prototyped for UAV attitude and shows ~30% smaller RMSE in attitude under fast rotations (Kraft 2003), at ~3× higher compute cost.

---

## 2. EKF Inconsistency — Overconfident Covariance

### What it is

EKF re-linearizes at the **current estimate** each step. This violates the statistical consistency requirement: the true state should lie within the confidence ellipsoid with the correct probability. The result is $\boldsymbol{P}$ underestimates the true error — the filter is "overconfident" and down-weights new measurements, allowing bias to accumulate uncorrected.

This is a known problem in EKF-based SLAM and VIO and applies equally here.

### Where in code

The symptom is visible in `measurementUpdate()`:

```cpp
// ekf_helper.cpp:1113-1117
// P is now not symmetrical if K is not optimal (e.g.: some gains have been zeroed)
for (unsigned i = 0; i < State::size; i++) {
    for (unsigned j = 0; j < State::size; j++) {
        P(i, j) -= K(i) * PH(j);
    }
}
```

The comment "K is not optimal" acknowledges that `clearInhibitedStateKalmanGains()` zeros out gains for inhibited states (mag, wind, accel_bias). When K is not the true optimal Kalman gain, the Joseph form still runs but `P` may not accurately represent the true covariance.

The `constrainStateVariances()` clamp (§7) is the fallback when $\boldsymbol{P}$ diverges — itself evidence that consistency is not guaranteed.

### Improvement

| Method | Description | Key Paper |
|---|---|---|
| **FEJ-EKF** (First-Estimates Jacobian) | Fix $\boldsymbol{F}$ and $\boldsymbol{H}$ at the *first* linearization point — eliminates spurious information gain | Li & Mourikis, IJRR 2013 |
| **OC-EKF** (Observability-Constrained) | Modifies Jacobians to preserve the correct observability structure of the system | Huang, Kaess, Leonard, IJRR 2010 |
| **InEKF** (Invariant EKF) | Uses Lie group structure of SE(3) — linearization error is second-order, not first-order | Barrau & Bonnabel, IEEE TAC 2017 |

InEKF is particularly promising for IMU-integrated navigation: the error dynamics are **group-affine**, making the EKF consistent by construction on $SE_2(3)$.

---

## 3. IMU Integration — Simplistic Model + Downsampling

### What it is

Two issues in `predictState()`:

**3a. Gravity model is scalar constant:**

```cpp
// ekf.cpp:263
const Vector3f gravity_acceleration(0.f, 0.f, CONSTANTS_ONE_G); // simplistic model
```

`CONSTANTS_ONE_G = 9.80665 m/s²` is a fixed constant. No variation with altitude (≈ 3 ppm/km), latitude (≈ 0.5% pole-to-equator), or local anomalies. For sub-200m operation this is negligible, but the comment in the code itself calls it "simplistic".

**3b. IMU downsampling loses high-frequency information:**

```cpp
// estimator_interface.cpp:93-107
if (_imu_down_sampler.update(imu_sample)) {
    _imu_updated = true;
    imuSample imu_downsampled = _imu_down_sampler.getDownSampledImuAndTriggerReset();

    // constrain integration delta time to prevent numerical problems
    imu_downsampled.delta_ang_dt = math::constrain(imu_downsampled.delta_ang_dt, imu_min_dt, imu_max_dt);
    imu_downsampled.delta_vel_dt = math::constrain(imu_downsampled.delta_vel_dt, imu_min_dt, imu_max_dt);

    _imu_buffer.push(imu_downsampled);
```

The EKF operates at `filter_update_interval_us = 10000 µs` (100 Hz default), downsampled from raw IMU (~1 kHz on most flight controllers). Downsampling by simple accumulation loses the **coning and sculling corrections** needed for accurate integration under high angular rate + vibration. For hover/cruise this is fine. For high-speed flips, the rotation vector attitude error can be 0.1–1 deg per second of aggressive flight.

**3c. Trapezoidal integration is first-order:**

```cpp
// ekf.cpp:268-269
_gpos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;
_state.pos(2) = -_gpos.altitude();
```

Trapezoidal rule is $O(dt^2)$ per step, $O(dt)$ accumulated. At 100 Hz ($dt = 10$ ms), position error is bounded but non-zero for jerky motion.

### Improvement

| Method | Description | Key Paper |
|---|---|---|
| **IMU Preintegration on SO(3)** | Accumulate delta rotation as rotation vector with exact BCH formula; handles multiple IMU samples between fusion epochs | Forster et al., TRO 2017 |
| **Coning + Sculling correction** | Second-order corrections to rotation vector integration under vibration | Savage, JGCD 1998 |
| **Full gravity model** | WGS-84 normal gravity formula | NIMA TR8350.2 |

Forster's preintegration is the foundation of modern VIO (VINS-Mono, OpenVINS, OKVIS) and reduces pose drift by 10–40% under vibration compared to simple accumulation.

---

## 4. Output Predictor — Unguaranteed Time Alignment

### What it is

The output predictor (`output_predictor/output_predictor.cpp`) bridges the ~100 ms EKF delay to the controller's "now". Two `TODO` comments in the code directly acknowledge design gaps:

```cpp
// output_predictor.cpp:281-282
// get the oldest INS state data from the ring buffer
// this data will be at the EKF fusion time horizon
// TODO: there is no guarantee that data is at delayed fusion horizon
//       Shouldn't we use pop_first_older_than?
const outputSample &output_delayed = _output_buffer.get_oldest();
```

```cpp
// ekf.cpp:153-154
// get the oldest IMU data from the buffer
// TODO: explicitly pop at desired time horizon
const imuSample imu_sample_delayed = _imu_buffer.get_oldest();
```

This means the delayed sample used for correction is **assumed** to align with the fusion time horizon, but is not **guaranteed** to. Under high IMU jitter or scheduling latency, the correction is applied to the wrong time offset.

### Correction mechanism

The correction is a first-order complementary filter, not a true time-reversal:

```cpp
// output_predictor.cpp:296-302
const float time_delay = fmaxf((time_latest_us - time_delayed_us) * 1e-6f, _dt_update_states_avg);
const float att_gain = 0.5f * _dt_update_states_avg / time_delay;

// calculate a correction to the delta angle that will cause the INS to track the EKF quaternions
_delta_angle_corr = delta_ang_error * att_gain;

// velocity/position: complementary filter with tunable time constants
const float vel_gain = _dt_correct_states_avg / math::constrain(_vel_tau, _dt_correct_states_avg, 10.f);
const float pos_gain = _dt_correct_states_avg / math::constrain(_pos_tau, _dt_correct_states_avg, 10.f);
```

The `att_gain`, `vel_gain`, `pos_gain` are computed heuristically. There is also:

```cpp
// output_predictor.cpp:101 (in reset path)
// TODO: who resets the output buffer content?
```

### Impact

- Under IMU scheduling jitter > 1–2 ms, the time alignment error introduces a systematic attitude bias of ~0.01–0.1 deg depending on angular rate.
- The complementary filter gains are fixed — not adaptive to flight dynamics.

### Improvement

Proper **Out-Of-Sequence Measurement (OOSM)** handling or an optimal smoother (Rauch-Tung-Striebel) would eliminate the heuristic complementary filter. Computationally expensive but exact. Alternatively, zero-delay EKF architectures (Mourikis & Roumeliotis, ICRA 2007) avoid the problem entirely by fusing at "now" rather than at a delayed horizon.

---

## 5. Sequential Scalar Fusion

### What it is

`fuseVelocity()` fuses each velocity axis **independently**, one scalar at a time:

```cpp
// velocity_fusion.cpp:57-77
bool Ekf::fuseVelocity(estimator_aid_source3d_s &aid_src)
{
    if (!aid_src.innovation_rejected) {
        for (unsigned i = 0; i < 3; i++) {
            // fuse vx, vy, vz as 3 independent scalar measurements
            fuseDirectStateMeasurement(
                aid_src.innovation[i],
                aid_src.innovation_variance[i],
                aid_src.observation_variance[i],
                State::vel.idx + i);
        }
        // ...
    }
}
```

This is mathematically equivalent to batch fusion only when the observation noise matrix $\boldsymbol{R}$ is **diagonal** (uncorrelated noise between axes). GPS velocity noise is typically diagonal in practice, so the approximation is valid here. However, this sequential approach:

1. Updates $\boldsymbol{P}$ three times in a row — the second and third fusions use an already-partially-updated $\boldsymbol{P}$, leading to slightly suboptimal gains.
2. Does not handle correlated measurement noise correctly if $\boldsymbol{R}$ is non-diagonal (e.g., vision odometry with correlated position/velocity noise).

### Improvement

**Iterated EKF (iEKF)** or **batch fusion**: compute the full $\boldsymbol{H}$ (3×24 for 3D velocity) and perform a single matrix solve. For the specific case of GPS velocity (diagonal $\boldsymbol{R}$), the difference is negligible. For vision odometry with correlated noise, the difference can be significant.

**Paper**: Sibley, Matthies, Sukhatme (2010) *"Sliding window filter with application to planetary landing"* — discusses batch vs sequential fusion tradeoffs.

---

## 6. Wind State — Unobservable on Quadrotor

### What it is

The state vector includes `wind_vel` (2D wind NE). However, `_control_status.flags.wind` is only set `true` inside `controlAirDataFusion()`, which requires an **airspeed sensor** or drag model:

```cpp
// aid_sources/airspeed/airspeed_fusion.cpp:57-62
// If both airspeed and sideslip fusion have timed out and we are not
// using a drag observation model then we no longer have valid wind estimates
const bool airspeed_timed_out  = isTimedOut(_aid_src_airspeed.time_last_fuse, (uint64_t)10e6);
const bool sideslip_timed_out  = isTimedOut(_aid_src_sideslip.time_last_fuse, (uint64_t)10e6);

if (_control_status.flags.fake_pos || (airspeed_timed_out && sideslip_timed_out && (_params.drag_ctrl == 0))) {
    _control_status.flags.wind = false;
}
```

Airspeed sensors and sideslip fusion (`controlBetaFusion`) are fixed-wing features. The drag model (`CONFIG_EKF2_DRAG_FUSION`) is available for multirotors but requires accurate motor/rotor drag coefficients and is rarely enabled.

Consequence: in `clearInhibitedStateKalmanGains()`, the Kalman gain rows for `wind_vel` are zeroed out when the flag is false:

```cpp
// ekf_helper.cpp:1263-1271
#if defined(CONFIG_EKF2_WIND)
    if (!_control_status.flags.wind) {
        for (unsigned i = 0; i < State::wind_vel.dof; i++) {
            K(State::wind_vel.idx + i) = 0.f;  // wind never updated on a plain quadrotor
        }
    }
#endif // CONFIG_EKF2_WIND
```

The wind state is in the state vector, consuming 2 rows/columns of the 24×24 $\boldsymbol{P}$ matrix, but is **never meaningfully estimated** on a standard GPS quadrotor.

### Impact

- Wind-compensated position hold in strong wind relies purely on the velocity/position feedback loop in `mc_pos_control`, not on a wind estimate.
- Dead-reckoning during GPS outage ignores wind effects entirely.

### Improvement

| Method | Description | Key Paper |
|---|---|---|
| **Motor RPM + drag model** | Back-compute aerodynamic force from RPM → wind estimate | Neumann & Bartholmai, Measurement 2015 |
| **Rotor thrust matching** | Compare expected vs actual acceleration; residual ≈ wind force | Sikkel et al., IROS 2021 |
| **Extended wind estimation** | Full $\boldsymbol{C_D}$ identification + wind via recursive LS | Abichandani et al., Sensors 2020 |

---

## 7. Covariance Clamping as Last Resort

### What it is

`constrainStateVariances()` runs after every `predictCovariance()` and every `measurementUpdate()`. The comment in the code is explicit:

```cpp
// covariance.cpp:246-248
// NOTE: This limiting is a last resort and should not be relied on
// TODO: Split covariance prediction into separate F*P*transpose(F) and Q contributions
//       and set corresponding entries in Q to zero when states exceed 50% of the limit
```

The actual clamping logic for position and velocity:

```cpp
// covariance.cpp:252-256
constrainStateVar(State::quat_nominal, 1e-9f, 1.f);
constrainStateVar(State::vel,          1e-6f, 1e6f);
constrainStateVar(State::pos,          1e-6f, 1e6f);
constrainStateVarLimitRatio(State::gyro_bias,  kGyroBiasVarianceMin,  1.f);
constrainStateVarLimitRatio(State::accel_bias, kAccelBiasVarianceMin, 1.f);
```

When `P(i,i)` exceeds the upper limit, it is **not simply clipped** (which would break cross-correlations) — instead a zero-innovation measurement is fused to reduce it:

```cpp
// covariance.cpp:287-293
} else if (P(i, i) > max) {
    // Constrain variance growth by fusing zero innovation as clipping the variance
    // would artificially increase the correlation between states and destabilize the filter.
    const float innov = 0.f;
    const float R = 10.f * P(i, i); // reduces variance by ~10% as K = P / (P + R)
    const float innov_var = P(i, i) + R;
    fuseDirectStateMeasurement(innov, innov_var, R, i);
}
```

This is a workaround for numerical instability in $\boldsymbol{P}$. The TODO comment acknowledges the real fix: structure the Q injection to prevent growth in the first place.

### Impact

- Under prolonged GPS outage + high vibration, $P(\text{pos}, \text{pos})$ can inflate until the clamp triggers. This indicates the filter is in "coast mode" — state estimates are drifting.
- The zero-innovation fusion trick changes cross-correlations between states in a non-physical way.

---

## 8. Fixed Process Noise Model

### What it is

The IMU noise densities are fixed parameters:

```cpp
// common.h:282-287
float gyro_noise{1.5e-2f};          ///< IMU angular rate noise (rad/sec)
float accel_noise{3.5e-1f};         ///< IMU acceleration noise (m/sec**2)
float gyro_bias_p_noise{1.0e-3f};   ///< process noise for gyro bias (rad/sec**2)
float accel_bias_p_noise{1.0e-2f};  ///< process noise for accel bias (m/sec**3)
```

These are set once at boot from parameters (`EKF2_GYR_NOISE`, `EKF2_ACC_NOISE`, etc.) and never updated. In reality:

- IMU noise is **motor-speed-dependent** — increases nonlinearly with rotor RPM due to vibration coupling.
- Accel noise is **attitude-dependent** — centrifugal forces in hard turns add structured noise not captured by the white-noise model.
- Temperature affects IMU bias drift rates significantly.

The **Hover Thrust Estimator** (§7 in the control stack docs) does implement adaptive measurement noise for its scalar EKF, but the main 24-state EKF does not.

### Improvement

| Method | Description | Key Paper |
|---|---|---|
| **Sage-Husa adaptive filter** | Online estimation of $\boldsymbol{Q}$ and $\boldsymbol{R}$ from innovation statistics | Sage & Husa, IEEE TAC 1969 |
| **Innovation-based adaptive** | Estimate $\boldsymbol{Q}$/$\boldsymbol{R}$ from windowed innovation covariance | Mehra, IEEE TAC 1972 |
| **RPM-coupled noise model** | Scale `gyro_noise` / `accel_noise` with estimated motor RPM | Mohamed & Schwarz, J. Geodesy 1999 |

The Hover Thrust Estimator in `zero_order_hover_thrust_ekf.cpp` already implements adaptive $R$ via `updateMeasurementNoise()` — the same pattern could be applied to the main EKF's accel noise.

---

## 9. No Formal Integrity Monitoring / FDI

### What it is

EKF2 uses chi-square innovation gating to reject outliers:

```cpp
// (pattern in every aid source, e.g. gps_control.cpp)
const float test_ratio = sq(innovation) / (sq(innovation_gate) * innovation_variance);
// if test_ratio > 1.0: innovation_rejected = true → do not fuse
```

This is **threshold-based rejection**, not formal integrity monitoring. Limitations:

1. The gate is symmetric — a **slowly-drifting** GPS signal (spoofing, multipath) can pass the gate indefinitely because each individual innovation is small.
2. There is no **cross-sensor consistency check** — if GPS and baro both drift in a correlated way, neither is rejected.
3. No formal **Protection Level (PL)** computation. Aviation-grade GNSS requires bounding the position error at a given integrity risk — EKF2 provides no such bound.

The GPS spoofing detection comment:

```cpp
// common.h:194 (gnssSample)
bool spoofed{};  ///< true if GNSS data is spoofed
```

The `spoofed` flag exists in the data structure, but its actual use depends on the GPS driver — EKF2 itself does no spoofing detection.

### Improvement

| Method | Description | Key Paper |
|---|---|---|
| **RAIM** (Receiver Autonomous Integrity Monitoring) | Statistical test on GNSS pseudoranges; compute Protection Level | Blanch et al., ION GNSS+ 2015 |
| **Multiple-hypothesis EKF** | Run N filter hypotheses with different sensor subsets; compare | Roumeliotis & Bekey, ICRA 2000 |
| **Interacting Multiple Models (IMM)** | Bayesian fusion of M motion models; detects sensor faults | Mazor et al., IEEE Trans AES 1998 |

---

## 10. Magnetic Heading Fragility

### What it is

Magnetometer fusion (`mag_control.cpp`, `mag_fusion.cpp`) estimates both heading and the Earth's magnetic field vector $\boldsymbol{m}_I$ + body hard-iron bias $\boldsymbol{m}_B$. However:

- `mag_B` (hard-iron) is estimated online — good.
- **Soft-iron distortion** (scaling + rotation of the magnetic field, modeled as a 3×3 matrix) is **not estimated** online. The only compensation is via offline calibration stored in parameters.
- When flying near ferrous structures (vehicles, buildings), soft-iron changes mid-flight and cannot be corrected.

The 3D mag fusion switches to heading-only (`mag_hdg`) in certain conditions, further degrading accuracy:

```cpp
// ekf.h:249-256
const bool is_using_mag = (_control_status.flags.mag_3D || _control_status.flags.mag_hdg);

if (is_using_mag) {
    return _control_status.flags.yaw_align
        && _control_status.flags.mag_aligned_in_flight
        // ...
}
return _control_status.flags.yaw_align;
```

The EKF-GSF yaw estimator (`yaw_estimator/EKFGSF_yaw.cpp`) is the fallback when mag fails — it uses GPS velocity to estimate yaw. But it requires GPS and takes several seconds to converge.

### Improvement

| Method | Description | Key Paper |
|---|---|---|
| **Full soft-iron online calibration** | Estimate 3×3 soft-iron matrix online (9 extra states) | Gebre-Egziabher et al., J. Guidance 2006 |
| **Dual-antenna GNSS yaw** | Two GPS antennas → precise heading without magnetometer | PX4 already supports via `gnss_yaw_control.cpp` |
| **Visual compass** | Optical flow or feature tracking → yaw from appearance | Engel et al., ECCV 2014 |

---

## 11. Summary Table

| # | Limitation | Root Cause in Code | Impact Level | Best Fix |
|---|---|---|---|---|
| 1 | First-order linearization | `covariance.cpp`: Jacobian re-evaluated at current estimate | Medium (aggressive flight) | UKF / InEKF |
| 2 | EKF inconsistency | `measurementUpdate()`: K zeroed for inhibited states | Medium (long-term drift) | FEJ-EKF / OC-EKF |
| 3a | Simplistic gravity model | `ekf.cpp:263`: `// simplistic model` | Low (< 500 m altitude) | WGS-84 gravity |
| 3b | IMU downsampling | `estimator_interface.cpp:93`: `imu_down_sampler` | Low–Medium | Preintegration |
| 4 | Output predictor alignment | `output_predictor.cpp:281`: `// TODO: no guarantee` | Low–Medium | OOSM / smoother |
| 5 | Sequential scalar fusion | `velocity_fusion.cpp:62`: loop over 3 axes independently | Low (GPS diagonal R) | Batch iEKF |
| 6 | Wind unobservable | `ekf_helper.cpp:1265`: K zeroed when `!flags.wind` | High (wind reject) | Motor-RPM drag model |
| 7 | Covariance clamping | `covariance.cpp:246`: `// last resort` | Medium (GPS outage) | Structured Q |
| 8 | Fixed process noise | `common.h:282`: static noise params | Medium (high vibration) | Sage-Husa / adaptive |
| 9 | No formal FDI | Innovation gate only; `gnssSample::spoofed` unused by EKF | High (spoofing) | RAIM / multi-hypothesis |
| 10 | Mag soft-iron not online | `mag_B` only estimates hard-iron | Medium (near metal) | Dual-antenna GPS / 9-state mag |

---

## Key References

| Paper | Relevance |
|---|---|
| Sola (2017), *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508 | Foundation of EKF2 design; §6 discusses when ESKF fails |
| Wan & Merwe (2000), *The Unscented Kalman Filter*, AISTATS | UKF as fix for limitation 1 |
| Barrau & Bonnabel (2017), *The Invariant Extended Kalman Filter as a Stable Observer*, IEEE TAC | InEKF — consistency by construction on Lie groups |
| Li & Mourikis (2013), *High-precision consistent EKF-based VIO*, IJRR | FEJ-EKF for limitation 2 |
| Huang, Kaess, Leonard (2010), *Observability-based rules for EKF SLAM*, IJRR | OC-EKF for limitation 2 |
| Forster et al. (2017), *On-Manifold Preintegration for Real-Time VIO*, TRO | IMU preintegration for limitation 3 |
| Mehra (1972), *Approaches to Adaptive Filtering*, IEEE TAC | Adaptive Q/R for limitation 8 |
| Blanch et al. (2015), *RAIM with Optimal Integrity Allocations*, ION GNSS+ | Formal integrity for limitation 9 |
| Gebre-Egziabher et al. (2006), *Calibration of strapdown magnetometers*, J. Guidance | Online soft-iron for limitation 10 |
