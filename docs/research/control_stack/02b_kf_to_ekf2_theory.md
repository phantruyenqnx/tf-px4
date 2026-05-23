# From Standard Kalman Filter to EKF2 in PX4 — Theory and Practice

> **Purpose**: Explains the development chain KF → EKF → Error-State EKF → EKF2 PX4, highlighting each place where the implementation **differs** from textbook theory and why.
>
> Read after `02_ekf2.md`. Notation is consistent with `00_index.md`.

---

## Table of Contents

1. [Standard Linear KF](#1-standard-linear-kf)
2. [EKF — Extension to Nonlinear Systems](#2-ekf--extension-to-nonlinear-systems)
3. [Error-State EKF (ESKF / MEKF)](#3-error-state-ekf-eskf--mekf)
4. [From Theoretical ESKF → PX4 EKF2](#4-from-theoretical-eskf--px4-ekf2)
5. [Theory vs. Practice Comparison Table](#5-theory-vs-practice-comparison-table)
6. [Mental Model — How to Read EKF2 Code](#6-mental-model--how-to-read-ekf2-code)

---

## 1. Standard Linear KF

### 1.1. Basic Assumptions

The Kalman Filter (1960) is optimal when **all** 3 conditions are satisfied:

| Condition | Symbol |
|---|---|
| **Linear** system | $\boldsymbol{x}_{k+1}=\boldsymbol{F}\boldsymbol{x}_k+\boldsymbol{B}\boldsymbol{u}_k+\boldsymbol{w}_k$ |
| **White Gaussian** process noise | $\boldsymbol{w}_k\sim\mathcal{N}(0,\boldsymbol{Q})$ |
| **White Gaussian** measurement noise | $\boldsymbol{v}_k\sim\mathcal{N}(0,\boldsymbol{R})$ |

Measurement model: $\boldsymbol{y}_k = \boldsymbol{H}\boldsymbol{x}_k + \boldsymbol{v}_k$.

### 1.2. Two Steps of KF

**Predict step** (propagate state + covariance before a new measurement arrives):

$$
\hat{\boldsymbol{x}}_{k+1|k} = \boldsymbol{F}\hat{\boldsymbol{x}}_{k|k} + \boldsymbol{B}\boldsymbol{u}_k
$$

$$
\boldsymbol{P}_{k+1|k} = \boldsymbol{F}\boldsymbol{P}_{k|k}\boldsymbol{F}^\top + \boldsymbol{Q}
$$

**Update step** (upon receiving measurement $\boldsymbol{y}_{k+1}$):

$$
\boldsymbol{z} = \boldsymbol{y}_{k+1} - \boldsymbol{H}\hat{\boldsymbol{x}}_{k+1|k} \quad\text{(innovation)}
$$

$$
\boldsymbol{S} = \boldsymbol{H}\boldsymbol{P}_{k+1|k}\boldsymbol{H}^\top + \boldsymbol{R}
$$

$$
\boldsymbol{K} = \boldsymbol{P}_{k+1|k}\boldsymbol{H}^\top\boldsymbol{S}^{-1} \quad\text{(Kalman gain)}
$$

$$
\hat{\boldsymbol{x}}_{k+1|k+1} = \hat{\boldsymbol{x}}_{k+1|k} + \boldsymbol{K}\boldsymbol{z}
$$

$$
\boldsymbol{P}_{k+1|k+1} = (\boldsymbol{I}-\boldsymbol{K}\boldsymbol{H})\boldsymbol{P}_{k+1|k}
$$

KF provides the **MMSE estimate** (minimum mean-square error) when the 3 conditions above hold → it is the best possible filter.

### 1.3. Why KF Cannot Be Applied Directly to a Drone?

A drone is a **strongly nonlinear** system:

- Rotational dynamics: $\dot{\boldsymbol{q}} = \frac{1}{2}\boldsymbol{q}\otimes\boldsymbol{\omega}$ — **nonlinear** in $\boldsymbol{q}$.
- Acceleration transformation: $\boldsymbol{a}_W = \boldsymbol{R}(\boldsymbol{q})(\boldsymbol{a}_m - \boldsymbol{b}_a) + \boldsymbol{g}_W$ — $\boldsymbol{R}(\boldsymbol{q})$ is a trigonometric function of $\boldsymbol{q}$.
- Magnetic field observation: $h_{mag}(\boldsymbol{x}) = \boldsymbol{R}^\top(\boldsymbol{q})\boldsymbol{m}_I + \boldsymbol{m}_B$ — product of two states.
- Optical flow observation: $h_{flow} = -\frac{1}{h_t}\boldsymbol{R}^\top\boldsymbol{v}_W + \boldsymbol{\omega}$ — divides by state $h_t$.

If $\boldsymbol{F},\boldsymbol{H}$ are forced to be constants, the linearization error will be very large.

---

## 2. EKF — Extension to Nonlinear Systems

### 2.1. Core Idea

EKF (Extended Kalman Filter, Jazwinski 1970) **linearizes** the nonlinear system around the current estimate using a first-order Taylor expansion:

$$
f(\boldsymbol{x}) \approx f(\hat{\boldsymbol{x}}) + \underbrace{\frac{\partial f}{\partial \boldsymbol{x}}\bigg|_{\hat{\boldsymbol{x}}}}_{\boldsymbol{F}}\delta\boldsymbol{x}
$$

$$
h(\boldsymbol{x}) \approx h(\hat{\boldsymbol{x}}) + \underbrace{\frac{\partial h}{\partial \boldsymbol{x}}\bigg|_{\hat{\boldsymbol{x}}}}_{\boldsymbol{H}}\delta\boldsymbol{x}
$$

Both Jacobians $\boldsymbol{F},\boldsymbol{H}$ **must be recomputed at each step** using the current value of $\hat{\boldsymbol{x}}$.

### 2.2. EKF Algorithm

**Predict** (use nonlinear function $f$ for state, Jacobian $\boldsymbol{F}$ for covariance):

$$
\hat{\boldsymbol{x}}_{k+1|k} = f(\hat{\boldsymbol{x}}_{k|k}, \boldsymbol{u}_k)
$$

$$
\boldsymbol{P}_{k+1|k} = \boldsymbol{F}_k\boldsymbol{P}_{k|k}\boldsymbol{F}_k^\top + \boldsymbol{G}_k\boldsymbol{Q}\boldsymbol{G}_k^\top
$$

**Update** (use nonlinear function $h$ for innovation, Jacobian $\boldsymbol{H}$ for gain):

$$
\boldsymbol{z} = \boldsymbol{y} - h(\hat{\boldsymbol{x}}_{k+1|k})
$$

$$
\boldsymbol{S} = \boldsymbol{H}\boldsymbol{P}_{k+1|k}\boldsymbol{H}^\top + \boldsymbol{R}, \quad
\boldsymbol{K} = \boldsymbol{P}_{k+1|k}\boldsymbol{H}^\top\boldsymbol{S}^{-1}
$$

$$
\hat{\boldsymbol{x}}_{k+1|k+1} = \hat{\boldsymbol{x}}_{k+1|k} + \boldsymbol{K}\boldsymbol{z}
$$

$$
\boldsymbol{P}_{k+1|k+1} = (\boldsymbol{I}-\boldsymbol{K}\boldsymbol{H})\boldsymbol{P}_{k+1|k}
$$

### 2.3. Problems with Applying Classic EKF Directly to Attitude

The "classic" EKF (above) treats **all** of $\boldsymbol{x}$ as a real vector and applies the correction directly:

$$
\hat{\boldsymbol{x}} \leftarrow \hat{\boldsymbol{x}} + \boldsymbol{K}\boldsymbol{z}
$$

This is **incorrect** for quaternions because:

1. **Quaternion is not a vector space**: $\boldsymbol{q}+\delta\boldsymbol{q}$ may no longer be a unit quaternion ($\lVert\boldsymbol{q}\rVert\neq 1$).
2. **Over-parameterization**: 4 real numbers represent 3 DoF — a 4×4 covariance is singular (rank 3).
3. **Poor linearization**: for large angles, $\partial(\boldsymbol{q}_{k+1})/\partial\boldsymbol{q}_k$ poorly represents large rotations.

Two common (and weak) workarounds:
- Use Euler angles: encounter singularity (gimbal lock at pitch ±90°).
- Normalize $\boldsymbol{q}$ after each update: treats the symptom, not the root cause.

---

## 3. Error-State EKF (ESKF / MEKF)

This is the foundational theory actually used by PX4 EKF2. See the original reference: **Sola, arXiv:1711.02508**.

### 3.1. Splitting State into Nominal + Error

$$
\boldsymbol{x} = \hat{\boldsymbol{x}} \boxplus \delta\boldsymbol{x}
$$

| Component | Role |
|---|---|
| **Nominal state** $\hat{\boldsymbol{x}}$ | Dynamics integrated from measurements (no covariance) — fast propagation |
| **Error state** $\delta\boldsymbol{x}$ | Small errors, well-linearized around **zero** — this is what the KF tracks |

The $\boxplus$ operator depends on the state type:
- Regular vectors ($\boldsymbol{v}$, $\boldsymbol{p}$, bias): $\hat{\boldsymbol{v}}\boxplus\delta\boldsymbol{v} = \hat{\boldsymbol{v}}+\delta\boldsymbol{v}$ (ordinary addition).
- Quaternion: $\hat{\boldsymbol{q}}\boxplus\delta\boldsymbol{\theta} = \hat{\boldsymbol{q}}\otimes\exp_{\boldsymbol{q}}(\delta\boldsymbol{\theta}/2)$ (multiplicative update).

### 3.2. Error State Definition for Rotation

The attitude error $\delta\boldsymbol{\theta}\in\mathbb{R}^3$ is a **rotation vector** (axis × angle):

$$
\boldsymbol{q}_{true} = \hat{\boldsymbol{q}}\otimes\delta\boldsymbol{q}, \quad
\delta\boldsymbol{q} = \exp_{\boldsymbol{q}}(\delta\boldsymbol{\theta}/2) \approx \begin{pmatrix}1\\ \delta\boldsymbol{\theta}/2\end{pmatrix} \text{ (when small)}
$$

Advantages:
- $\delta\boldsymbol{\theta}\in\mathbb{R}^3$ is a true vector → addition and covariance computation are straightforward.
- Linearization around **zero** is always better than around a large value.
- No singularity.
- $\boldsymbol{P}\in\mathbb{R}^{24\times24}$ (not 25×25), non-singular.

### 3.3. ESKF Loop

```
1. [Predict nominal]  x̂ ← f(x̂, u)         (integrate IMU, no Jacobian needed)
2. [Predict error]    P ← F·P·F^T + G·Q·G^T  (Jacobian evaluated at x̂)
3. [Update]           z = y - h(x̂)           (innovation uses nominal)
                      K = P·H^T·(H·P·H^T + R)^{-1}
                      δx = K·z
4. [Inject]           x̂ ← x̂ ⊞ δx           (apply correction to nominal on the manifold)
5. [Reset]            δx ← 0, P ← (I-K·H)·P  (reset error to 0, update covariance)
```

The **Reset** step (5) is the biggest difference from standard EKF: the error state returns to zero after each update, and the nominal state absorbs all corrections.

---

## 4. From Theoretical ESKF → PX4 EKF2

This is where **practice differs from theory**. PX4 EKF2 uses ESKF as its foundation but has many important technical differences.

---

### 4.1. Delayed Time Horizon + Output Predictor

**Theory**: KF/EKF assumes measurements arrive **at exactly** step $k$ → fused immediately.

**PX4 Reality**:
- IMU arrives at 1 kHz, GPS arrives at 5–10 Hz with a delay of ~100–200 ms (transmission + processing), baro 50 Hz, mag 100 Hz, vision 30 Hz.
- If fused in arrival order, GPS timestamp < current IMU timestamp → **Out-Of-Sequence Measurement (OOSM)**.

**Solution**:

```
EKF runs at "delayed horizon" T_delay ≈ 100 ms:
  ┌─ IMU ring buffer (past) ────────────────────────────┐
  │  t-100ms ... t-50ms ... t-20ms ... t-5ms ... t_now  │
  │       ↑                                              │
  │  EKF runs here                                Output predictor
  └──────────────────────────────────────────────────────┘
```

- **Ring buffer** (`EKF2.cpp`): stores IMU, GPS, baro, mag, vision ≈ 300 ms. When GPS arrives late, it is placed at the correct timestamp position in the buffer.
- **EKF core** processes at `t - T_delay` → all sensors have had time to arrive.
- **Output predictor** (`output_predictor/`): uses the latest IMU to propagate from the delayed state to `t_now`. A **tracking error correction loop** pulls `q_out` toward `q_EKF` each time the delayed EKF produces a new estimate.

**Consequence**: the final output (`vehicle_local_position`, `vehicle_attitude`) is the state at **real time** but smoothed from the delayed EKF, not directly from the EKF core. The two values are **not exactly identical**.

---

### 4.2. Jacobians Auto-Generated with SymPy

**Theory**: Jacobian $\boldsymbol{F} = \partial f/\partial\delta\boldsymbol{x}$ is derived by hand, then coded manually.

**PX4 Reality**:

```
src/modules/ekf2/EKF/python/ekf_derivation/
  ├── ekf.py         ← define state, dynamics using SymPy
  ├── generate_code.py
  └── generated/
      ├── predict_covariance.h   ← F·P·F^T + Q fully automated
      ├── compute_mag_innov_var_and_h.h
      ├── compute_flow_xy_innov_var_and_h.h
      └── ...
```

Each Jacobian is a C++ header file containing the fully expanded expression (typically 200–1000 lines) generated from SymPy `cse()` (common subexpression elimination). Developers **do not compute** $\boldsymbol{F}$ by hand; they write `f(x)` in SymPy and run the generator.

**Implication**: no algebraic errors in the Jacobians; but difficult to debug by inspection since the output is mechanical code. Adding a new state requires modifying `ekf.py` and re-running the generator.

---

### 4.3. Joseph Form for Covariance Update

**Theory**: simplified form:

$$
\boldsymbol{P}_{new} = (\boldsymbol{I}-\boldsymbol{K}\boldsymbol{H})\boldsymbol{P}
$$

Proven optimal when $\boldsymbol{K}$ is exactly the optimal gain.

**Practical problem**: with 32-bit float, $\boldsymbol{P}$ at 24×24, after many steps accumulated round-off → $\boldsymbol{P}$ loses **symmetry** and **positive-semi-definiteness (PSD)** → negative covariance → incorrect Kalman gain → filter divergence.

**PX4 uses the Joseph form**:

$$
\boldsymbol{P}_{new} = (\boldsymbol{I}-\boldsymbol{K}\boldsymbol{H})\boldsymbol{P}(\boldsymbol{I}-\boldsymbol{K}\boldsymbol{H})^\top + \boldsymbol{K}\boldsymbol{R}\boldsymbol{K}^\top
$$

- Twice as many matrix multiplications.
- **Always guarantees symmetry** (of the form $A\cdot A^\top$).
- **Always PSD** (sum of two positive semi-definite forms).
- Robust to round-off → filter does not diverge due to numerical issues.

---

### 4.4. Innovation Gating (Chi-square Test)

**Standard theory**: fuse all measurements unconditionally.

**PX4 Reality**: GPS multipath, magnetic interference, vision jumps → sudden measurement outliers.

**Solution**: check the **Mahalanobis distance** of the innovation:

$$
d^2 = \boldsymbol{z}^\top \boldsymbol{S}^{-1} \boldsymbol{z} \overset{?}{<} \gamma^2
$$

- $d^2$ follows a $\chi^2$ distribution with degrees of freedom equal to the dimension of $\boldsymbol{z}$ (when the filter is consistent).
- $\gamma$ = `EKF2_GPS_P_GATE`, `EKF2_MAG_GATE`, etc. (default 5.0 → 5σ region).
- If $d^2 \geq \gamma^2$: reject sample, no update (state and $\boldsymbol{P}$ unchanged).

**Consequence**: PX4 reports `innovation_check_flags` when a gate fails → used to diagnose GPS/mag/vision issues during flight.

---

### 4.5. Asynchronous Multi-Sensor Fusion

**Theory**: classic KF assumes a single measurement type, synchronous with the predict cycle.

**PX4 Reality**: 7+ sensor types, different frequencies, arriving at any time:

```
IMU      ████████████████████████████████  (1000 Hz)
GPS      █               █               (5–10 Hz)
Baro     ████████████                     (50 Hz)
Mag      ████████████████                 (100 Hz)
Vision   ████████                         (30 Hz)
Flow     ████████                         (30–100 Hz)
Range    ████                             (20–50 Hz)
```

**Mechanism**:
- Each sensor has its **own ring buffer** (timestamp-indexed).
- Each EKF core step: predict to sensor timestamp, fuse, predict further.
- Fusion is implemented **sequentially** (one sensor at a time, updating $\boldsymbol{P}$ each time) rather than in batch → simpler code, good approximation for independent sensors.

**Difference from theory**: theoretical multi-rate KF (Larsen extrapolation, OOSM) is considerably more complex. PX4 solves this with the simpler delayed-horizon approach, which is sufficient in practice.

---

### 4.6. Bias Random Walk Model (Instead of Fixed Bias)

**Simple theory**: bias is a constant, estimated once and done.

**IMU Reality**: bias **drifts over time** due to temperature, aging, and vibration. This is known as **bias instability** / **in-run bias**.

**PX4 Model**: bias is modeled as a **random walk** (Wiener process):

$$
\dot{\boldsymbol{b}}_g = \boldsymbol{n}_{bg}, \quad \boldsymbol{n}_{bg}\sim\mathcal{N}(0, \sigma_{bg}^2\boldsymbol{I})
$$

Discretized: $\boldsymbol{b}_{g,k+1} = \boldsymbol{b}_{g,k} + \boldsymbol{w}_{bg,k}$ with variance $\sigma_{bg}^2\Delta t$.

- `EKF2_GYR_B_NOISE` ($\sigma_{bg}$): process noise for gyro bias (rad/s/√s → variance/step).
- `EKF2_ACC_B_NOISE` ($\sigma_{ba}$): same for accel.

**Consequence**: the bias state **never converges to zero**; the $\boldsymbol{P}$ of bias always has a lower bound $\geq\sigma_{bg}^2\Delta t$. The EKF continuously tracks bias and does not "lock in" old values. If `EKF2_GYR_B_NOISE = 0`, bias becomes constant → filter cannot adapt → drift after a few minutes of flight.

---

### 4.7. Multi-EKF Selector (Fault-Tolerant Estimation)

**Standard theory**: a single EKF.

**Reality**: high-end airframes have 2–3 IMUs (Pixhawk 6X: ICM-42688-P + ICM-20649 + IIM-42652). A single IMU may experience:
- Abnormal shock/vibration → gyro saturation.
- Thermal runaway → bias spike.
- Uneven rotor magnetic interference.

**PX4 Solution** (`EKF2Selector.cpp`):

```
IMU 0 ──→ EKF instance 0 ──→ test_ratio_0
IMU 1 ──→ EKF instance 1 ──→ test_ratio_1  ──→ Selector ──→ best output
IMU 2 ──→ EKF instance 2 ──→ test_ratio_2
```

`combined_test_ratio` = aggregated innovation ratio across all sensors for that EKF instance. The EKF with the lowest ratio (best innovation quality) is selected as output.

**Switch logic**: hysteresis to prevent chattering; on switch, `EKF2Selector` publishes `sensor_selection` to inform the controller.

---

### 4.8. Reset Events with Delta Compensation

**Theory**: EKF does not reset; estimation is continuous.

**Reality**: some situations force EKF to reset state abruptly:
- First GPS lock after flying indoors (position jumps tens of meters).
- Vision setpoint changes the reference frame origin.
- Large glitch exceeding the gate → accumulated error.

**Problem**: if position/velocity jumps, the controller (holding the old setpoint) will command a sudden thrust → crash.

**Solution**: EKF2 publishes **delta values** on reset:
- `vehicle_local_position.delta_xy`, `delta_z`, `delta_vxy`, `delta_vz`
- `vehicle_attitude.delta_q_reset`

The controller (`MulticopterPositionControl::adjustSetpointForEKFResets`) **adds the delta to the setpoint** immediately upon receiving the reset event → setpoint shifts with the estimate → error = 0 → no thrust spike.

**Difference from theory**: standard theory does not address this mechanism; it is a PX4 engineering practice.

---

### 4.9. Partial State Activation (Aiding Source ON/OFF)

**Theory**: KF has a fixed state vector.

**Reality**: GPS or range finder may not always be available. PX4 manages:

| Parameter | Meaning |
|---|---|
| `EKF2_AID_MASK` | enable/disable GPS, vision, flow, etc. |
| `EKF2_HGT_REF` | primary height source (baro/GPS/range/vision) |
| `EKF2_GPS_CTRL` | enable/disable GPS position and velocity independently |

When no aiding source is available for horizontal position:
- `vel` and `pos` states are still predicted (IMU propagation).
- Covariance $\boldsymbol{P}_{pos}$ grows over time (unconstrained).
- EKF will **dead-reckon** but drift over time.

When optical flow + range finder are enabled: EKF also fuses the `terrain` state (terrain height $h_t$) — a state that is only active when the appropriate sensor is available.

---

## 5. Theory vs. Practice Comparison Table

| Aspect | KF/EKF Theory | PX4 EKF2 Practice | Code File |
|---|---|---|---|
| **State architecture** | State vector $\in\mathbb{R}^n$ directly | 24-dim error-state + 25-elem nominal state | `state.h`, `ekf.h` |
| **Quaternion update** | Add $\hat{\boldsymbol{x}}+\boldsymbol{K}\boldsymbol{z}$ directly | Multiplicative: $\hat{\boldsymbol{q}}\otimes\exp(\delta\boldsymbol{\theta}/2)$ | `ekf.cpp` §Inject |
| **Measurement timing** | Synchronous with predict | Delayed horizon ~100 ms + ring buffer + output predictor | `output_predictor/` |
| **Jacobian** | Computed by hand or numerically | Auto-generated from SymPy | `generated/*.h` |
| **Covariance update** | $(I-KH)P$ simplified | Joseph form $(I-KH)P(I-KH)^\top + KRK^\top$ | `covariance.cpp` |
| **Outliers** | Fuse all | Chi-square gate $d^2 < \gamma^2$ | `aid_sources/*.cpp` |
| **Multi-rate sensors** | Single measurement type | Sequential multi-rate fusion, separate ring buffers | `EKF2.cpp` |
| **Bias model** | Constant or none | Random walk (Wiener) with `EKF2_*_B_NOISE` | `covariance.cpp` |
| **Fault tolerance** | Single EKF | Multi-EKF per IMU + selector for best innovation | `EKF2Selector.cpp` |
| **State discontinuity** | Does not occur | Reset events + delta compensation for controller | `EKF2.cpp::publish_*` |
| **Fixed state** | Always n states | Some states only active when sensor is available | `aid_sources/` |

---

## 6. Mental Model — How to Read EKF2 Code

When reading `src/modules/ekf2/`, picture 4 nested loops:

```
[1000 Hz] IMU loop
  → Integrator.hpp: integrate delta-angle, delta-velocity
  → ekf.cpp::predictState(): update nominal state (q, v, p)
  → covariance.cpp::predictCovariance(): update P (using generated Jacobian)

[5–100 Hz] Sensor fusion loop (each time a new sensor arrives in the ring buffer)
  → aid_sources/xxx_control.cpp: check gate, compute innovation
  → compute_xxx_innov_var_and_h.h: compute H, S (auto-generated)
  → ekf.cpp::fuseXxx(): K, δx, Joseph update P
  → Inject δx into nominal state (boxplus)

[1000 Hz] Output predictor loop
  → output_predictor/: propagate q/v/p from delayed → now
  → Correction from delayed EKF each time a new estimate is available

[1–10 Hz] Selector loop (multi-IMU)
  → EKF2Selector.cpp: compare test_ratio across instances
  → Switch if primary fails
```

When tracing a bug, e.g. "GPS fuse incorrect":
1. `aid_sources/gnss/` → does the gate pass?
2. `compute_gnss_pos_innov_var_and_h.h` → is H correct?
3. `covariance.cpp` → is P reasonable?
4. `output_predictor/` → is delay compensation correct?

---

## Reference Documents (required reading, in order)

1. **Sola, *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508** — all of §3 in this file is based on this paper; read it before reading the code.
2. **Trawny & Roumeliotis, *Indirect Kalman Filter for 3D Attitude Estimation*, UMN TR 2005-002** — more rigorous proof of MEKF.
3. **Simon, *Optimal State Estimation* (Wiley 2006)** — KF/EKF foundations §3–§7, Joseph form §5.3.
4. **PX4 ECL EKF docs**: <https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html> — parameter reference.
5. **Markley & Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control* (2014)** — MEKF for aerospace §7.
