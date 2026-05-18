# Detailed Mathematics of the PX4 Quadcopter Control Stack

This document explains **every formula** in the PX4 multicopter control stack, tracing the flow from **sensor → EKF2 → Position Control → Attitude Control → Rate Control → Control Allocation → Motor**. All symbols are used consistently throughout the document.

> Read alongside `quadcopter_control_flow.md` (overview document).
> All code references map to `src/` in the `tf-px4` repo.

---

## 0. Symbol Conventions (used throughout)

### 0.1. Reference Frames

| Symbol | Frame | Notes |
|---|---|---|
| $\{W\}$ | World / NED | x-North, y-East, z-Down |
| $\{B\}$ | Body | x-Forward, y-Right, z-Down |
| $\boldsymbol{R}_{WB}=\boldsymbol{R}\in SO(3)$ | DCM body→world | column 3 = $\boldsymbol{z}_B$ in $\{W\}$ |
| $\boldsymbol{q}=(q_w, q_x, q_y, q_z)$ | Hamilton quaternion, scalar-first | $\boldsymbol{R}=\boldsymbol{R}(\boldsymbol{q})$ |

### 0.2. Kinematic Quantities

| Symbol | Meaning | Frame |
|---|---|---|
| $\boldsymbol{p}=(p_x, p_y, p_z)^\top$ | position | $\{W\}$ |
| $\boldsymbol{v}$ | linear velocity | $\{W\}$ |
| $\boldsymbol{a}$ | linear acceleration | $\{W\}$ |
| $\boldsymbol{\omega}=(p,q,r)^\top$ | angular rate | $\{B\}$ |
| $\dot{\boldsymbol{\omega}}=\boldsymbol{\alpha}$ | angular acceleration | $\{B\}$ |
| $\psi$ | yaw (Euler 3-2-1) | rotation about $\boldsymbol{z}_W$ |
| $g=9.80665$ | gravitational acceleration | constant |

Append `_sp` for setpoint (desired): $\boldsymbol{p}_{sp},\boldsymbol{v}_{sp},\boldsymbol{a}_{sp},\boldsymbol{q}_{sp},\boldsymbol{\omega}_{sp}$.

### 0.3. Force / Control Quantities

| Symbol | Meaning |
|---|---|
| $\boldsymbol{T}\in\mathbb{R}^3$ | **normalized** thrust vector [-1,1] (PX4 convention: negative body Z = upward thrust) |
| $T_z$ (collective) | scalar thrust along $\boldsymbol{z}_B$, code symbol: `thrust_body[2]` |
| $\boldsymbol{\tau}=(\tau_x,\tau_y,\tau_z)^\top$ | normalized torque about body axes |
| $T_h\in[0,1]$ | hover thrust (`MPC_THR_HOVER` or HTE) |
| $\boldsymbol{u}\in\mathbb{R}^{n_m}$ | per-motor setpoint [0,1] |
| $\boldsymbol{B}\in\mathbb{R}^{6\times n_m}$ | effectiveness matrix (allocation) |

### 0.4. Operators
- $\odot$ = element-wise multiplication (Hadamard); in matrix code `.emult()`.
- $\boldsymbol{a}\times\boldsymbol{b}$ = cross product.
- $\lfloor x\rfloor_{[a,b]} = \mathrm{clip}(x,a,b)$.
- $[\boldsymbol{x}]_\times$ = skew-symmetric matrix corresponding to cross product.
- $\boldsymbol{q}_1\otimes\boldsymbol{q}_2$ = Hamilton quaternion multiplication.
- $\mathrm{Im}(\boldsymbol{q}) = (q_x,q_y,q_z)^\top$.

---

## 1. Sensor Layer

File: `src/modules/sensors/`, `src/lib/sensor_calibration/`, `src/lib/mathlib/math/filter/`.

### 1.1. IMU (gyroscope + accelerometer)

Measurement model:
$$
\boldsymbol{\omega}_m = \boldsymbol{\omega} + \boldsymbol{b}_g + \boldsymbol{n}_g, \qquad
\boldsymbol{a}_m = \boldsymbol{R}^\top(\boldsymbol{a} - \boldsymbol{g}_W) + \boldsymbol{b}_a + \boldsymbol{n}_a
$$
with $\boldsymbol{g}_W=(0,0,g)^\top$ in NED. Pipeline:

1. **Calibration**: subtract offline bias + rotation/scale matrix → `sensor_gyro`, `sensor_accel`.
2. **Low-pass filter** 2nd-order Butterworth ($f_c$ = `IMU_GYRO_CUTOFF` ≈ 30 Hz default).
3. **Notch filter** to remove rotor vibration (`IMU_GYRO_NF*`):
$$
H_{notch}(s)=\frac{s^2+\omega_n^2}{s^2+\frac{\omega_n}{Q}s+\omega_n^2}
$$
4. Coning/sculling integration at 1 kHz → publish `vehicle_angular_velocity` containing $\boldsymbol{\omega}$ and $\dot{\boldsymbol{\omega}}$ (derivative estimated by finite difference + 1st-order LPF):
$$
\dot{\omega}_k = \alpha\,\dot{\omega}_{k-1} + (1-\alpha)\,\frac{\omega_k-\omega_{k-1}}{\Delta t},\quad \alpha=e^{-\Delta t/\tau}.
$$

### 1.2. Magnetometer / Barometer / GPS / Range / Flow / Vision
All are calibrated (bias and scale subtracted) then pushed into EKF2 as "aiding sources" (see `EKF/aid_sources/*`). For each sensor $i$, the measurement is $\boldsymbol{y}_i = h_i(\boldsymbol{x}) + \boldsymbol{n}_i$ with covariance $\boldsymbol{R}_i$.

---

## 2. EKF2 — State Estimation

Main file: `src/modules/ekf2/EKF2.cpp`, `src/modules/ekf2/EKF/ekf.h`, state vector at `src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h`.

### 2.1. State Vector (24 DoF, 25-element representation)

```
@/home/frank/tf-px4/src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h:12-41
```

$$
\boldsymbol{x} = \begin{bmatrix}
\boldsymbol{q} & \boldsymbol{v}_W & \boldsymbol{p}_W &
\boldsymbol{b}_g & \boldsymbol{b}_a &
\boldsymbol{m}_I & \boldsymbol{m}_B & \boldsymbol{w}_W & h_t
\end{bmatrix}^\top
$$

Where: $\boldsymbol{m}_I$ = world magnetic field, $\boldsymbol{m}_B$ = body mag bias, $\boldsymbol{w}_W=(w_n,w_e)$ = wind, $h_t$ = terrain height. Quaternion (4 elements) only occupies 3 DoF thanks to error-state formulation: covariance size $24\times24$.

### 2.2. Prediction Step

At each new IMU sample ($\Delta t \approx 1\text{–}4$ ms, "delayed horizon"):

**Quaternion update** (right multiply, error in body frame):
$$
\hat{\boldsymbol{q}}_{k+1} = \hat{\boldsymbol{q}}_{k}\otimes \exp\!\left(\tfrac{1}{2}(\boldsymbol{\omega}_m-\hat{\boldsymbol{b}}_g)\,\Delta t\right)
$$
with $\exp(\boldsymbol{\theta}/2) = (\cos\|\boldsymbol{\theta}\|/2,\ \mathrm{sinc}(\|\boldsymbol{\theta}\|/2)\,\boldsymbol{\theta}/2)$.

**Velocity/position update** (Euler integration):
$$
\boldsymbol{a}_W = \boldsymbol{R}(\hat{\boldsymbol{q}})(\boldsymbol{a}_m-\hat{\boldsymbol{b}}_a) + \boldsymbol{g}_W
$$
$$
\hat{\boldsymbol{v}}_{k+1} = \hat{\boldsymbol{v}}_k + \boldsymbol{a}_W\,\Delta t,\qquad
\hat{\boldsymbol{p}}_{k+1} = \hat{\boldsymbol{p}}_k + \hat{\boldsymbol{v}}_k\Delta t + \tfrac{1}{2}\boldsymbol{a}_W\Delta t^2.
$$

**Covariance update** (error-state, 24×24):
$$
\boldsymbol{P}_{k+1} = \boldsymbol{F}\boldsymbol{P}_k\boldsymbol{F}^\top + \boldsymbol{G}\boldsymbol{Q}\boldsymbol{G}^\top
$$
with $\boldsymbol{F}=\partial f/\partial\delta\boldsymbol{x}$ and $\boldsymbol{G}$ the Jacobian with respect to noise (gyro/accel noise + bias random walk). The expression for $\boldsymbol{F}$ is auto-generated using SymPy: see `EKF/python/ekf_derivation/generated/predict_covariance.h`.

### 2.3. Measurement Update Step

For each sensor with residual $\boldsymbol{y}_i - h_i(\hat{\boldsymbol{x}})$:
$$
\boldsymbol{S} = \boldsymbol{H}\boldsymbol{P}\boldsymbol{H}^\top + \boldsymbol{R},\quad
\boldsymbol{K} = \boldsymbol{P}\boldsymbol{H}^\top\boldsymbol{S}^{-1}
$$
$$
\delta\boldsymbol{x} = \boldsymbol{K}(\boldsymbol{y}-h(\hat{\boldsymbol{x}})),\quad
\hat{\boldsymbol{x}}\leftarrow \hat{\boldsymbol{x}}\boxplus\delta\boldsymbol{x}
$$
$$
\boldsymbol{P}\leftarrow (\boldsymbol{I}-\boldsymbol{K}\boldsymbol{H})\boldsymbol{P}.
$$

Where $\boxplus$ is the error-state addition operator: for the quaternion part $\hat{\boldsymbol{q}}\leftarrow \hat{\boldsymbol{q}}\otimes\exp(\delta\boldsymbol{\theta}/2)$; the rest is ordinary addition. Innovation gate test:
$$
\boldsymbol{r}^\top\boldsymbol{S}^{-1}\boldsymbol{r} < \gamma^2 \quad(\text{e.g. }\gamma=5)
$$

### 2.4. Output (Output Predictor)
Because the EKF runs at a **delayed horizon**, an "output predictor" runs in real time, propagating $\boldsymbol{q},\boldsymbol{v},\boldsymbol{p}$ using the latest IMU samples and then publishing:
- `vehicle_attitude` ($\hat{\boldsymbol{q}}$, $\Delta\boldsymbol{q}_{reset}$)
- `vehicle_local_position` ($\boldsymbol{p},\boldsymbol{v}$ NED, validity flags)
- `vehicle_angular_velocity` ($\boldsymbol{\omega}-\hat{\boldsymbol{b}}_g$, $\dot{\boldsymbol{\omega}}$)

These topics are the inputs for all downstream controller layers.

---

## 3. Position Control (outermost loop)

Core: `@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp`.

Setpoint input: `trajectory_setpoint` from `flight_mode_manager`, containing $\boldsymbol{p}_{sp},\boldsymbol{v}_{sp},\boldsymbol{a}_{sp},\psi_{sp},\dot{\psi}_{sp}$ (each field may be NaN = not controlled).

### 3.1. Position P-controller (`_positionControl`)

Function `PositionControl::_positionControl`:
$$
\boldsymbol{v}_{sp,P} = \boldsymbol{K}_p^{pos}\odot(\boldsymbol{p}_{sp}-\boldsymbol{p})
$$
$$
\boldsymbol{v}_{sp} \leftarrow \boldsymbol{v}_{sp,P} + \boldsymbol{v}_{sp,FF}\quad(\text{NaN-selective addition})
$$

Parameters: `MPC_XY_P` (default 0.95), `MPC_Z_P` (default 1.0).

**Lateral limit prioritizing P-term over FF** (`ControlMath::constrainXY`): given $\boldsymbol{v}_0=\boldsymbol{v}_{sp,P}^{xy}$ with priority, $\boldsymbol{v}_1=\boldsymbol{v}_{sp,FF}^{xy}$, constraint $\|\boldsymbol{v}_0+s\hat{\boldsymbol{v}}_1\|\le V_{max}$. Quadratic solve:
$$
s = -\hat{\boldsymbol{v}}_1\!\cdot\!\boldsymbol{v}_0 + \sqrt{(\hat{\boldsymbol{v}}_1\!\cdot\!\boldsymbol{v}_0)^2 - (\|\boldsymbol{v}_0\|^2-V_{max}^2)}.
$$

Z-axis: $v_{sp,z}\leftarrow \mathrm{clip}(v_{sp,z}, -V_{up}, V_{down})$.

### 3.2. Velocity PID-controller (`_velocityControl`)

$$
\boldsymbol{e}_v = \boldsymbol{v}_{sp}-\boldsymbol{v},\quad
\boldsymbol{a}_{sp,PID} = \boldsymbol{K}_p^v\odot\boldsymbol{e}_v + \boldsymbol{I}_v - \boldsymbol{K}_d^v\odot\dot{\boldsymbol{v}}
$$
$$
\boldsymbol{a}_{sp} \leftarrow \boldsymbol{a}_{sp,PID} + \boldsymbol{a}_{sp,FF}
$$

(Note: the code uses $\dot{\boldsymbol{v}}$ — velocity derivative — rather than the error derivative $\dot{\boldsymbol{e}}_v$, to avoid amplifying the error spike when $\boldsymbol{v}_{sp}$ steps.)

Integral $\boldsymbol{I}_v$ is accumulated *at the end* of the function after saturation is determined:
$$
\boldsymbol{I}_v^{(k+1)} = \boldsymbol{I}_v^{(k)} + \boldsymbol{K}_i^v\odot\boldsymbol{e}_v\,\Delta t
$$

**Anti-windup on Z-axis**: if Z thrust is saturated and the error has the same sign, set $e_{v,z}=0$ before integration.

**Lateral tracking anti-windup (Rundqwist 1990)**: let $\boldsymbol{a}_{prod}^{xy}$ be the acceleration *actually produced* (after thrust saturation); when $\|\boldsymbol{a}_{sp}^{xy}\|>\|\boldsymbol{a}_{prod}^{xy}\|$:
$$
\boldsymbol{e}_v^{xy}\leftarrow\boldsymbol{e}_v^{xy} - K_{arw}(\boldsymbol{a}_{sp}^{xy}-\boldsymbol{a}_{prod}^{xy}),\quad K_{arw}=\frac{2}{K_p^{v,x}}.
$$

Parameters: `MPC_{XY,Z}_VEL_{P,I,D}_ACC`.

### 3.3. Acceleration → Thrust Vector (`_accelerationControl`)

Thrust ↔ acceleration relationship assumes the model:
$$
\boldsymbol{a}_{cmd} = \boldsymbol{a}_{sp} - \boldsymbol{g}_W = \boldsymbol{a}_{sp} - g\hat{\boldsymbol{z}}_W
$$
("specific force" to be produced). Desired body Z-axis direction in NED:
$$
\hat{\boldsymbol{z}}_B^* = -\boldsymbol{a}_{cmd}/\|\boldsymbol{a}_{cmd}\|
$$
(negative sign because NED z is down, thrust pointing up = $-\hat{\boldsymbol{z}}_W$).

**Decouple flag** (`MPC_ACC_DECOUPLE`): if enabled, uses fixed $z_{spec}=-g$, ignoring $a_{sp,z}$ when determining tilt → avoids tilt error during vertical acceleration.

**Tilt limit** (`ControlMath::limitTilt`): given $\hat{\boldsymbol{z}}_B^*$, $\hat{\boldsymbol{z}}_W=(0,0,1)$, $\theta_{max}$:
$$
\theta = \min(\arccos(\hat{\boldsymbol{z}}_B^*\!\cdot\!\hat{\boldsymbol{z}}_W),\theta_{max})
$$
$$
\hat{\boldsymbol{r}} = \frac{\hat{\boldsymbol{z}}_B^*-(\hat{\boldsymbol{z}}_B^*\!\cdot\!\hat{\boldsymbol{z}}_W)\hat{\boldsymbol{z}}_W}{\|\cdot\|},\quad
\hat{\boldsymbol{z}}_B^\dagger = \cos\theta\,\hat{\boldsymbol{z}}_W + \sin\theta\,\hat{\boldsymbol{r}}.
$$

**Acceleration → thrust conversion** (normalized via hover thrust):
$$
T_z^{NED} = a_{sp,z}\frac{T_h}{g} - T_h
$$
(derived from: at hover $a_{sp,z}=0$ ⇒ $T_z^{NED}=-T_h$.) Then projected onto the tilt-limited body axis:
$$
T_{coll} = \min\!\left(\frac{T_z^{NED}}{\hat{\boldsymbol{z}}_W\!\cdot\!\hat{\boldsymbol{z}}_B^\dagger},\ -T_{min}\right),\quad
\boldsymbol{T} = T_{coll}\,\hat{\boldsymbol{z}}_B^\dagger.
$$

**Thrust saturation** (prioritize vertical, maintain lateral margin `MPC_THR_XY_MARG`):
$$
T_{xy,allocated} = \min(\|\boldsymbol{T}^{xy}\|,M_{xy}),\quad
T_z\ge-\sqrt{T_{max}^2-T_{xy,allocated}^2}
$$
$$
T_{xy,max} = \sqrt{T_{max}^2-T_z^2},\qquad
\boldsymbol{T}^{xy}\leftarrow \frac{\boldsymbol{T}^{xy}}{\|\boldsymbol{T}^{xy}\|}T_{xy,max}\ \text{(if exceeded)}.
$$

### 3.4. Thrust Vector → Attitude Setpoint (`ControlMath::thrustToAttitude`)

Given thrust vector $\boldsymbol{T}$ (NED) and yaw setpoint $\psi_{sp}$. Define body axes in $\{W\}$:
$$
\hat{\boldsymbol{z}}_B = -\boldsymbol{T}/\|\boldsymbol{T}\|
$$
$$
\boldsymbol{y}_C = (-\sin\psi_{sp},\ \cos\psi_{sp},\ 0)^\top\quad(\text{y-axis of the yaw-only "C" frame})
$$
$$
\hat{\boldsymbol{x}}_B = \frac{\boldsymbol{y}_C\times\hat{\boldsymbol{z}}_B}{\|\boldsymbol{y}_C\times\hat{\boldsymbol{z}}_B\|},\quad
\hat{\boldsymbol{y}}_B = \hat{\boldsymbol{z}}_B\times\hat{\boldsymbol{x}}_B
$$
$$
\boldsymbol{R}_{sp} = [\hat{\boldsymbol{x}}_B\ \hat{\boldsymbol{y}}_B\ \hat{\boldsymbol{z}}_B]\quad\Rightarrow\quad \boldsymbol{q}_{sp} = q(\boldsymbol{R}_{sp}).
$$

Also `thrust_body[2]` = $-\|\boldsymbol{T}\|$ (collective in body, negative = upward thrust).

Output topic `vehicle_attitude_setpoint`: $(\boldsymbol{q}_{sp},\ \boldsymbol{T}^{body}=(0,0,-\|\boldsymbol{T}\|),\ \dot{\psi}_{sp})$.

---

## 4. Attitude Control

Core: `@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`. Based on Brescianini, Hehn, D'Andrea (ETH 2013).

Input: $\boldsymbol{q}$ (current), $\boldsymbol{q}_{sp}$, $\dot{\psi}_{sp}$.

### 4.1. Yaw / Tilt Decoupling (Reduced Attitude)

Goal: bring $\hat{\boldsymbol{z}}_B\to\hat{\boldsymbol{z}}_B^{sp}$ with *full priority*, while yaw is controlled with *weight* $w_\psi=$ `MC_YAW_WEIGHT` (default 0.4).

Define:
$$
\boldsymbol{e}_z = \hat{\boldsymbol{z}}_B = \boldsymbol{R}(\boldsymbol{q})\hat{\boldsymbol{z}}_W,\qquad
\boldsymbol{e}_z^{sp} = \boldsymbol{R}(\boldsymbol{q}_{sp})\hat{\boldsymbol{z}}_W.
$$

Quaternion rotating from $\boldsymbol{e}_z\to\boldsymbol{e}_z^{sp}$ (minimal rotation between 2 axes):
$$
\boldsymbol{q}_{red}^{(W)} = \mathrm{quat\_from\_two\_vectors}(\boldsymbol{e}_z,\boldsymbol{e}_z^{sp}).
$$

Since $\boldsymbol{q}_{red}^{(W)}$ is a delta in world frame, **right-multiply** with $\boldsymbol{q}$ to get the "reduced desired attitude":
$$
\boldsymbol{q}_{red} = \boldsymbol{q}_{red}^{(W)}\otimes\boldsymbol{q}.
$$

### 4.2. Yaw Error Component

$$
\boldsymbol{q}_{\delta\psi} = \boldsymbol{q}_{red}^{-1}\otimes\boldsymbol{q}_{sp}
$$
By definition $\boldsymbol{q}_{\delta\psi}$ has the form $(\cos(\alpha/2), 0, 0, \sin(\alpha/2))$. Apply yaw weight:
$$
\boldsymbol{q}_{\delta\psi}^{(w)} = (\cos(w_\psi\arccos q_{\delta\psi,w}),\ 0,\ 0,\ \sin(w_\psi\arcsin q_{\delta\psi,z})).
$$

Blended desired attitude:
$$
\boldsymbol{q}_d = \boldsymbol{q}_{red}\otimes\boldsymbol{q}_{\delta\psi}^{(w)}.
$$

### 4.3. Quaternion Attitude Error → Rate Setpoint

$$
\boldsymbol{q}_e = \boldsymbol{q}^{-1}\otimes\boldsymbol{q}_d,\quad \text{canonicalize: }\boldsymbol{q}_e\leftarrow\mathrm{sign}(q_{e,w})\boldsymbol{q}_e
$$

Theorem: for a unit quaternion, $\mathrm{Im}(\boldsymbol{q}_e)=\sin(\alpha/2)\hat{\boldsymbol{r}}$. Proportional control law (proportional to rotation axis × small angle):
$$
\boldsymbol{\omega}_{sp} = 2\,\boldsymbol{K}_p^{att}\odot\mathrm{Im}(\boldsymbol{q}_e)
$$
with $\boldsymbol{K}_p^{att}=(K_p^\phi, K_p^\theta, K_p^\psi/w_\psi)$ — yaw gain is inversely scaled to cancel the effect of $w_\psi$.

### 4.4. Yaw Rate Feed-Forward

$\dot{\psi}_{sp}$ is defined in world frame (around $\hat{\boldsymbol{z}}_W$), needs to be expressed in body frame:
$$
\boldsymbol{\omega}_{sp} \mathrel{+}= \boldsymbol{R}^\top(\boldsymbol{q})\hat{\boldsymbol{z}}_W\cdot\dot{\psi}_{sp}.
$$

### 4.5. Rate Limiting
$$
\omega_{sp,i}\leftarrow\mathrm{clip}(\omega_{sp,i},-\omega_{max,i},\omega_{max,i})
$$
with $\omega_{max,i}=$ `MC_{ROLL,PITCH,YAW}RATE_MAX`.

Output `vehicle_rates_setpoint`: $(\boldsymbol{\omega}_{sp},\ \boldsymbol{T}^{body})$ (thrust pass-through from position controller).

---

## 5. Rate Control (inner loop, runs ~1 kHz)

Core: `@/home/frank/tf-px4/src/lib/rate_control/rate_control.cpp`. Module wrapper: `MulticopterRateControl.cpp` (callback registered on `vehicle_angular_velocity`).

### 5.1. PID + Feed-Forward Law

$$
\boldsymbol{e}_\omega = \boldsymbol{\omega}_{sp}-\boldsymbol{\omega}
$$
$$
\boxed{\ \boldsymbol{\tau} = \boldsymbol{K}_p^\omega\odot\boldsymbol{e}_\omega + \boldsymbol{I}_\omega - \boldsymbol{K}_d^\omega\odot\dot{\boldsymbol{\omega}} + \boldsymbol{K}_{ff}^\omega\odot\boldsymbol{\omega}_{sp}\ }
$$

Note: the D-term acts on $\dot{\boldsymbol{\omega}}$ (measured directly from EKF/gyro derivative) rather than $\dot{\boldsymbol{e}}_\omega$ — avoids "derivative kick" when the setpoint steps.

Parameters: `MC_{ROLL,PITCH,YAW}RATE_{K,P,I,D,FF}`. The "actual" gains are multiplied by an additional factor $K$ (ideal form):
$$
K_p = K\cdot p,\quad K_i = K\cdot i,\quad K_d = K\cdot d.
$$

### 5.2. Integration with Nonlinear Anti-Windup

Before integration, apply 3 checks per axis $i$:

**(a) Saturation feedback from allocator** (`setSaturationStatus`): if axis $i$ is positively saturated ($s_i^+ = 1$) then:
$$
e_{\omega,i}\leftarrow\min(e_{\omega,i},0)
$$
(analogously for negative saturation).

**(b) Nonlinear $i$-factor** reduces I gain when error is too large (normalized to 400°):
$$
i_{f,i} = \max\!\left(0,\ 1-\left(\frac{e_{\omega,i}}{400^\circ}\right)^2\right)
$$

**(c) Euler integration + clamp**:
$$
I_{\omega,i}\leftarrow \mathrm{clip}\!\left(I_{\omega,i} + i_{f,i}\,K_i^\omega\,e_{\omega,i}\,\Delta t,\ -I_{lim,i},\ I_{lim,i}\right).
$$

`I_lim` = `MC_{R,P,Y}R_INT_LIM`. When `landed` or disarmed, $\boldsymbol{I}_\omega\leftarrow 0$.

### 5.3. Yaw Torque LPF
To reduce vibration from rotor acceleration:
$$
\tau_z\leftarrow \mathrm{LPF}_{f_c=\text{MC\_YAW\_TQ\_CUTOFF}}(\tau_z).
$$

### 5.4. ACRO Mode (manual without attitude control)

$\boldsymbol{\omega}_{sp}$ is generated directly from sticks with superexpo:
$$
\mathrm{superexpo}(x,e,s) = (1-e)\,x + e\,x^3\quad\text{then multiplied by smoothing factor} \frac{1-s}{1-s|x|}.
$$
Then multiplied by $\boldsymbol{\omega}_{max}^{acro}=$ `MC_ACRO_{R,P,Y}_MAX`.

### 5.5. Battery Scaling (optional `MC_BAT_SCALE_EN`)

$$
\boldsymbol{\tau}\leftarrow\mathrm{clip}(s_{bat}\boldsymbol{\tau},-1,1),\quad\boldsymbol{T}\leftarrow\mathrm{clip}(s_{bat}\boldsymbol{T},-1,1)
$$
with $s_{bat}=V_{nom}/V_{batt}$ — compensates for battery voltage drop to maintain consistent response.

Output: `vehicle_torque_setpoint` $= \boldsymbol{\tau}$, `vehicle_thrust_setpoint` $= \boldsymbol{T}^{body}$, both ∈ [-1,1].

---

## 6. Control Allocation (motor distribution)

Core: `@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp`. Wrapper: `ControlAllocator.cpp`.

### 6.1. Problem

Command vector (6 axes): $\boldsymbol{c}=[\tau_x,\tau_y,\tau_z,T_x,T_y,T_z]^\top\in\mathbb{R}^6$. Output per motor $\boldsymbol{u}\in\mathbb{R}^{n_m}$. Linear model:
$$
\boldsymbol{c} = \boldsymbol{B}\boldsymbol{u}
$$
$\boldsymbol{B}\in\mathbb{R}^{6\times n_m}$ = effectiveness matrix, generated from airframe geometry (motor positions, thrust directions, spin directions) in `VehicleActuatorEffectiveness/`. Example for quad X (motor $i$ at angle $\theta_i$, arm length $r$, drag moment coefficient $c_m$):
$$
\boldsymbol{B}_{:,i} = \begin{bmatrix}
-r\sin\theta_i \\ r\cos\theta_i \\ \pm c_m \\ 0 \\ 0 \\ -1
\end{bmatrix}
$$
(sign of $c_m$ depends on CW/CCW of the motor.)

### 6.2. Moore–Penrose Pseudo-inverse Solution

$$
\boldsymbol{B}^+ = \boldsymbol{B}^\top(\boldsymbol{B}\boldsymbol{B}^\top)^{-1}\quad\text{(when }n_m\ge 6\text{ and full rank)}
$$
$$
\boxed{\ \boldsymbol{u} = \boldsymbol{u}_{trim} + \boldsymbol{B}^+(\boldsymbol{c}-\boldsymbol{c}_{trim})\ }
$$

In code (`allocate()`):
```
_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
```
with `_mix` = normalized $\boldsymbol{B}^+$.

### 6.3. Column Normalization of $\boldsymbol{B}^+$

To ensure the same value $\tau_x=1$ produces the same total motor deviation regardless of motor count, each column of $\boldsymbol{B}^+$ is divided by:
- Roll/Pitch: $\sqrt{\|\boldsymbol{B}^+_{:,roll}\|^2 / (n_{nz}/2)}$ — maintains common scale.
- Yaw: $\max_i |B^+_{i,yaw}|$.
- Thrust axis: $\frac{1}{n_{nz}}\sum_i |B^+_{i,thrust}|$.

### 6.4. Sequential Desaturation (SD) — when $n_m=4$ lacks sufficient DoF

File: `ControlAllocationSequentialDesaturation.cpp`. When pseudo-inverse yields $u_i\notin[u_{min},u_{max}]$, the algorithm sacrifices axes in priority order (default for multicopter): **yaw < tilt-roll/pitch < thrust**. For each sacrificed axis:
$$
\boldsymbol{u}\leftarrow\boldsymbol{u} + s\,\boldsymbol{B}^+_{:,k},\qquad
s = \arg\min_s\sum_i \mathbb{1}[u_i+s\,B^+_{i,k}\notin[u_{min},u_{max}]]\cdot\|\cdot\|
$$
(informally: find $s$ to push $\boldsymbol{u}$ back to the feasible region, then clip $\boldsymbol{u}\leftarrow\mathrm{clip}(\boldsymbol{u},u_{min},u_{max})$).

`unallocated_torque` $= \boldsymbol{c}-\boldsymbol{B}\boldsymbol{u}$ is published in `control_allocator_status` for the Rate Controller to use as anti-windup in §5.2(a).

### 6.5. Per-Motor Slew-Rate

Before output:
$$
u_i^{(k)}\leftarrow u_i^{(k-1)} + \mathrm{clip}(u_i^{(k)}-u_i^{(k-1)},-r_i\Delta t,\,r_i\Delta t)
$$
with $r_i=$ `CA_R{i}_SLEW`.

Output: `actuator_motors.control[i]` $\in[0,1]$ → mixer/driver PWM/DShot/UAVCAN ESC.

---

## 7. Hover Thrust Estimator (HTE) — adaptive

Module `mc_hover_thrust_estimator`. Theory: scalar RLS, model
$$
a_z^W = g\left(\frac{T_{cmd}}{T_h}-1\right)+\eta
$$
with $T_{cmd}$ = current collective thrust, $a_z^W$ = vertical acceleration in NED. Hidden parameter $T_h$, estimated by:
$$
\hat{T}_h^{(k+1)} = \hat{T}_h^{(k)} + K_k\big(a_z - h(\hat{T}_h^{(k)})\big)
$$
$$
K_k = \frac{P_k H_k}{\lambda + H_k^2 P_k},\quad
P_{k+1} = (1-K_k H_k)P_k/\lambda + Q
$$
with $H_k=\partial h/\partial T_h$. When `MPC_USE_HTE=1`, $T_h$ in §3 is updated smoothly via `PositionControl::updateHoverThrust` (pushed directly into $\boldsymbol{I}_v$ to avoid output discontinuity):
$$
I_{v,z}\leftarrow I_{v,z} + (a_{sp,z}-g)\frac{T_h^{old}}{T_h^{new}} + g - a_{sp,z}.
$$

---

## 8. Summary Table: Who Produces, Who Consumes

| Topic | Mathematical Content | Publisher | Consumer |
|---|---|---|---|
| `vehicle_angular_velocity` | $\boldsymbol{\omega},\dot{\boldsymbol{\omega}}$ | EKF2 / sensors | Rate ctrl |
| `vehicle_attitude` | $\boldsymbol{q}$ | EKF2 | Att ctrl |
| `vehicle_local_position` | $\boldsymbol{p},\boldsymbol{v}$ NED | EKF2 | Pos ctrl, FMM |
| `trajectory_setpoint` | $\boldsymbol{p}_{sp},\boldsymbol{v}_{sp},\boldsymbol{a}_{sp},\psi_{sp}$ | FMM | Pos ctrl |
| `vehicle_attitude_setpoint` | $\boldsymbol{q}_{sp},\boldsymbol{T}^{body}$ | Pos ctrl | Att ctrl |
| `vehicle_rates_setpoint` | $\boldsymbol{\omega}_{sp},\boldsymbol{T}^{body}$ | Att ctrl | Rate ctrl |
| `vehicle_torque_setpoint` | $\boldsymbol{\tau}\in[-1,1]^3$ | Rate ctrl | Allocator |
| `vehicle_thrust_setpoint` | $\boldsymbol{T}\in[-1,1]^3$ | Rate ctrl | Allocator |
| `actuator_motors` | $\boldsymbol{u}\in[0,1]^{n_m}$ | Allocator | PWM/DShot |
| `control_allocator_status` | $\boldsymbol{c}-\boldsymbol{B}\boldsymbol{u}$, sat flags | Allocator | Rate ctrl (anti-windup) |
| `hover_thrust_estimate` | $T_h$ | HTE | Pos ctrl, Att ctrl (stick scaling) |

---

## 9. End-to-End Mathematical Pipeline (one-page)

$$
\underbrace{\boldsymbol{y}_{IMU,GPS,...}}_{\text{sensors}}
\xrightarrow{\text{EKF: }\boldsymbol{x}\boxplus \boldsymbol{K}(\boldsymbol{y}-h(\boldsymbol{x}))}
\underbrace{\boldsymbol{p},\boldsymbol{v},\boldsymbol{q},\boldsymbol{\omega}}_{\text{state}}
$$
$$
\xrightarrow[\text{FMM}]{\text{flight task}}
\boldsymbol{p}_{sp},\boldsymbol{v}_{sp},\psi_{sp}
\xrightarrow[\text{P+PID}]{\boldsymbol{e}_p,\boldsymbol{e}_v}
\boldsymbol{a}_{sp}
\xrightarrow[\text{geometric}]{\boldsymbol{a}_{sp}-g\hat{\boldsymbol{z}}_W}
\boldsymbol{T},\boldsymbol{q}_{sp}
$$
$$
\xrightarrow[\text{quat P}]{\boldsymbol{q}_e=\boldsymbol{q}^{-1}\boldsymbol{q}_d}
\boldsymbol{\omega}_{sp}
\xrightarrow[\text{PID+FF}]{\boldsymbol{e}_\omega}
\boldsymbol{\tau}
\xrightarrow[\text{pseudo-inv + SD}]{\boldsymbol{u}=\boldsymbol{u}_{trim}+\boldsymbol{B}^+(\boldsymbol{c}-\boldsymbol{c}_{trim})}
\boldsymbol{u}\to\text{ESC}\to\text{rotors}.
$$

The feedback loop closes via IMU/GPS/Mag/Baro → EKF2.

---

## 10. Quick Cross-Reference to Code

| Layer | File | Main Function |
|---|---|---|
| EKF predict | `src/modules/ekf2/EKF/ekf.cpp` | `Ekf::predictState`, `predictCovariance` |
| EKF update | `src/modules/ekf2/EKF/aid_sources/*` | one `fuseXxx()` per sensor |
| Pos P/PID | `mc_pos_control/PositionControl/PositionControl.cpp` | `_positionControl`, `_velocityControl`, `_accelerationControl` |
| Thrust→Att | `mc_pos_control/PositionControl/ControlMath.cpp` | `thrustToAttitude`, `bodyzToAttitude`, `limitTilt`, `constrainXY` |
| Quat Att | `mc_att_control/AttitudeControl/AttitudeControl.cpp` | `update(q)` |
| Rate PID | `lib/rate_control/rate_control.cpp` | `update`, `updateIntegral` |
| Allocation | `lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp` | `updatePseudoInverse`, `allocate` |
| SD | `…/ControlAllocationSequentialDesaturation.cpp` | `desaturate` |
| HTE | `modules/mc_hover_thrust_estimator/` | `HoverThrustEstimator::update` |

All formulas above have been verified directly against the code in the `tf-px4` repo.
