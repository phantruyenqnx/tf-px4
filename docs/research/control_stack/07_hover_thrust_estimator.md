# 7. Hover Thrust Estimator (Adaptive Scalar EKF)

## 7.1. Role

The parameter $T_h$ — "hover thrust" = normalized thrust [0,1] sufficient to hover (i.e. counteract gravity) — depends on mass, payload, air density, and motor lift capacity (battery voltage drop). The position controller uses $T_h$ in §3 to convert the acceleration setpoint to a thrust setpoint:
$$
T_z^{NED}=a_{sp,z}\frac{T_h}{g}-T_h
$$
An incorrect $T_h$ → biased thrust → the velocity controller integrator must compensate → poor vertical tracking, oscillations during takeoff/landing.

HTE is a **scalar zero-order EKF** that estimates $T_h$ online from acceleration + thrust output.

## 7.2. Main Code

| Role | File |
|---|---|
| Module wrapper | `@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/MulticopterHoverThrustEstimator.cpp` |
| **Scalar EKF core** | `@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp` |
| EKF header | `@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.hpp` |
| Params | `hover_thrust_estimator_params.c` |
| Test | `zero_order_hover_thrust_ekf_test.cpp` |

Functions to read:
- `ZeroOrderHoverThrustEkf::predict(dt)` — simple, only grows the covariance.
- `ZeroOrderHoverThrustEkf::fuseAccZ(acc_z, thrust)` — full single Kalman step.
- `MulticopterHoverThrustEstimator::Run()` — orchestrator (reads NED accel + thrust setpoint).

Consumer on the position control side: `PositionControl::updateHoverThrust(hover_thrust_new)` — bumpless update into the integrator (see §7.4 below).

## 7.3. Symbol Table

### State + Observation

| Symbol | Code | Meaning |
|---|---|---|
| $T_h\in[T_{h,min},T_{h,max}]$ | `_hover_thr` | normalized hover thrust (state, scalar) |
| $u\in[0,1]$ | `thrust` (input) | current normalized thrust setpoint (output of §3, input to allocator) |
| $a_z^W$ | `acc_z` (input) | vertical acceleration measured in NED (from EKF, gravity subtracted) |
| $g=9.80665$ | `CONSTANTS_ONE_G` | gravitational acceleration |

### Covariance + Kalman

| Symbol | Code | Meaning |
|---|---|---|
| $P$ | `_state_var` | state covariance ($T_h$ variance) |
| $\sigma_p^2$ | `_process_var` / `HTE_HT_NOISE` | process noise (rate of change of $T_h$ per second) |
| $R=\sigma_a^2 s_R$ | `_acc_var * _acc_var_scale` | measurement noise (adaptive) |
| $H=\partial h/\partial T_h$ | `computeH(thrust)` | measurement Jacobian |
| $S=HPH+R$ | `innov_var` | innovation covariance |
| $K=PH/S$ | `K` | Kalman gain |
| $y=a_z^W-h(\hat T_h)$ | `innov` | innovation |
| $\gamma$ | `_gate_size` / `HTE_ACC_GATE` | gate factor (sigma) |
| $r=y^2/(\gamma^2 S)$ | `innov_test_ratio` | chi-square test ratio |

### Adaptive Noise

| Symbol | Code | Meaning |
|---|---|---|
| $\bar y$ | `_residual_lpf` | LPF of residual (removes bias) |
| $\bar r$ | `_signed_innov_test_ratio_lpf` | LPF of signed test ratio (to detect "prolonged rejection") |
| $\tau_{lpf}$ | `_lpf_time_constant` | LPF residual time constant |
| $\tau_{noise}$ | `_noise_learning_time_constant` | time constant for learning $\sigma_a^2$ |
| $\sigma_a^2$ | `_acc_var` | adaptively learned accelerometer noise variance |

---

## 7.4. Mathematical Model

### State
$x = T_h$ (scalar). Process: zero-order (assumes $T_h$ changes very slowly relative to $\Delta t$).

### Predict

$$
\hat{T}_h^{(k+1)}=\hat{T}_h^{(k)},\qquad P_{k+1} = P_k + \sigma_p^2\Delta t^2
$$

**Variable explanation**:
- State $T_h$ does NOT change ("zero-order" / "random walk"): the EKF assumes hover thrust changes slowly relative to the update rate.
- Covariance grows over time × process noise: the longer without an update, the less "confident".
- $\sigma_p^2$ = `HTE_HT_NOISE` ($\approx 0.001\sim 0.01$ /s²) — increase if faster EKF adaptation is desired (e.g. drone frequently dropping payload).

```@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp:44-50
void ZeroOrderHoverThrustEkf::predict(const float dt)
{
	// State is constant
	// Predict state covariance only
	_state_var += _process_var * dt * dt;
	_dt = dt;
}
```

### Measurement Model

Physical relationship: for the current normalized thrust $u\in[0,1]$, the measured vertical acceleration in NED is:
$$
a_z^W = g\frac{u}{T_h} - g + \eta
$$
(at hover $u=T_h\Rightarrow a_z=0$). Measurement function:
$$
h(T_h) = g\frac{u}{T_h} - g
$$
Jacobian:
$$
H = \frac{\partial h}{\partial T_h} = -g\frac{u}{T_h^2}
$$

**Variable explanation**:
- $u$: current thrust setpoint (allocator input, normalized).
- At hover, the motor produces exactly $g$ → $u\equiv T_h$ → $a_z^W=0$ (no acceleration).
- At maximum thrust ($u=1$), the drone accelerates upward: $a_z^W = g(1/T_h-1)>0$ in upward-NED (note: NED z points down → upward acceleration = $a_z^W<0$; in the actual PX4 code the sign is corrected in `MulticopterHoverThrustEstimator`).
- $\eta\sim\mathcal{N}(0,R)$: measurement noise (mechanical vibration, frame misalignment).
- $H<0$: increasing $T_h$ (heavier vehicle) → predicted $a_z^W$ decreases → EKF subtracts $K\cdot y\cdot H$ vector → converges correctly.

```@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp:84-87
inline float ZeroOrderHoverThrustEkf::computeH(const float thrust) const
{
	return -CONSTANTS_ONE_G * thrust / (_hover_thr * _hover_thr);
}
```

```@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp:96-105
float ZeroOrderHoverThrustEkf::computeInnov(const float acc_z, const float thrust) const
{
	const float predicted_acc_z = computePredictedAccZ(thrust);
	return acc_z - predicted_acc_z;
}

float ZeroOrderHoverThrustEkf::computePredictedAccZ(const float thrust) const
{
	return CONSTANTS_ONE_G * thrust / _hover_thr - CONSTANTS_ONE_G;
}
```

### Update (Scalar Kalman)

$$
y = a_z^{W,measured} - h(\hat{T}_h)
$$
$$
S = H P H + R,\quad R = \sigma_a^2\cdot s_{R}
$$
$$
K = \frac{P\cdot H}{S}
$$
$$
\hat{T}_h \leftarrow \mathrm{clip}(\hat{T}_h + K y,\ T_{h,min},\ T_{h,max})
$$
$$
P\leftarrow \mathrm{clip}((1-KH)P,\ 10^{-10},\ 1)
$$

**Variable explanation**:
- $y$: innovation — difference between measured and predicted acceleration.
- $S$: variance of $y$ — "uncertainty" of the innovation.
- $K$: gain — approaches 0 when $R\gg HPH$ (distrust sensor), approaches $1/H$ when $P\gg R$ (trust sensor).
- $\mathrm{clip}\,T_h\in[T_{h,min},T_{h,max}]$ = `[0.1, 0.9]` by default — prevents EKF from going to physically unrealistic values.
- $\mathrm{clip}\,P\in[10^{-10},1]$ — floor prevents the EKF from "freezing" (P → 0); ceiling prevents divergence.

Corresponding sub-functions:

```@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp:89-115
inline float ZeroOrderHoverThrustEkf::computeInnovVar(const float H) const
{
	const float R = _acc_var * _acc_var_scale;
	const float P = _state_var;
	return math::max(H * P * H + R, R);
}

// ...

inline float ZeroOrderHoverThrustEkf::computeKalmanGain(const float H, const float innov_var) const
{
	return _state_var * H / innov_var;
}

inline float ZeroOrderHoverThrustEkf::computeInnovTestRatio(const float innov, const float innov_var) const
{
	return innov * innov / (_gate_size * _gate_size * innov_var);
}
```

```@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp:122-130
inline void ZeroOrderHoverThrustEkf::updateState(const float K, const float innov)
{
	_hover_thr = math::constrain(_hover_thr + K * innov, _hover_thr_min, _hover_thr_max);
}

inline void ZeroOrderHoverThrustEkf::updateStateCovariance(const float K, const float H)
{
	_state_var = math::constrain((1.f - K * H) * _state_var, 1e-10f, 1.f);
}
```

Full fuse step in `fuseAccZ`:

```@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp:52-82
void ZeroOrderHoverThrustEkf::fuseAccZ(const float acc_z, const float thrust)
{
	const float H = computeH(thrust);
	const float innov_var = computeInnovVar(H);
	const float innov = computeInnov(acc_z, thrust);
	const float K = computeKalmanGain(H, innov_var);
	const float innov_test_ratio = computeInnovTestRatio(innov, innov_var);

	float residual = innov;

	if (isTestRatioPassing(innov_test_ratio)) {
		updateState(K, innov);
		updateStateCovariance(K, H);
		residual = computeInnov(acc_z, thrust); // residual != innovation since the hover thrust changed

	} else if (isLargeOffsetDetected()) {
		// Rejecting all the measurements for some time,
		// it means that the hover thrust suddenly changed or that the EKF
		// is diverging. To recover, we bump the state variance
		bumpStateVariance();
	}

	const float signed_innov_test_ratio = sign(innov) * innov_test_ratio;
	updateLpf(residual, signed_innov_test_ratio);
	updateMeasurementNoise(residual, H);

	// save for logging
	_innov = innov;
	_innov_var = innov_var;
	_innov_test_ratio = innov_test_ratio;
}
```

### Innovation Gating

$$
r = \frac{y^2}{\gamma^2 S} \quad (\gamma=\text{HTE\_HT\_GATE})
$$

**Variable explanation**:
- $r$: chi-square test ratio (removing $\gamma$: $y^2/S\sim\chi^2_1$).
- $\gamma$ = `HTE_ACC_GATE` (default 3 sigma).
- $r<1$ ⇔ $|y|<\gamma\sqrt{S}$ → innovation within $\gamma$-sigma, update accepted.
- $\bar r$ = LPF of $\mathrm{sign}(y)\cdot r$ — indicates "which direction is being rejected" (if many samples reject in the same direction → EKF is biased to one side).
- When $|\bar r|>0.2$ persists: "bump" $P\mathrel{+}= 10^3\sigma_p^2\Delta t^2$ — increase uncertainty → EKF "dares" to chase $T_h$ faster → recovery after payload drop, motor burnout, ...

```@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp:117-140
inline bool ZeroOrderHoverThrustEkf::isTestRatioPassing(const float innov_test_ratio) const
{
	return innov_test_ratio < 1.f;
}

// ...

inline bool ZeroOrderHoverThrustEkf::isLargeOffsetDetected() const
{
	return fabsf(_signed_innov_test_ratio_lpf) > 0.2f;
}

inline void ZeroOrderHoverThrustEkf::bumpStateVariance()
{
	_state_var += 1e3f * _process_var * _dt * _dt;
}
```

### Adaptive Measurement Noise

LPF the residual then learn $\sigma_a^2$:
$$
\bar y\leftarrow(1-\alpha)\bar y+\alpha y,\quad \alpha=\frac{\Delta t}{\tau_{lpf}+\Delta t}
$$
$$
\sigma_a^2\leftarrow\mathrm{clip}\!\left((1-\alpha')\sigma_a^2 + \alpha'((y-\bar y)^2+H P H),\ 1,\ 400\right)
$$

**Variable explanation**:
- $\bar y$: LPF-filtered mean of the residual — estimates the DC bias of the acceleration (e.g. tilt causing projection error).
- $y-\bar y$: residual after removing bias — the "purely random" component.
- $(y-\bar y)^2 + HPH$: in theory, $\mathbb{E}[(y-\bar y)^2]=\sigma_a^2 + HPH$, so subtracting $HPH$ gives $\hat\sigma_a^2$. However the code ADDS (does not subtract) because the practical meaning is: total noise (sensor + state uncertainty) — the EKF is "conservative".
- $\alpha,\alpha'$: LPF weights, proportional to $\Delta t/\tau$. $\tau_{lpf}<\tau_{noise}$ → bias is learned quickly, noise is learned slowly.
- Clamp $[1,400]$ m²/s⁴: keeps values in a physically reasonable range.

→ Automatically widens $R$ during high vibration (noisy rotors) and tightens it when the vehicle is calm.

```@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp:142-156
inline void ZeroOrderHoverThrustEkf::updateLpf(const float residual, const float signed_innov_test_ratio)
{
	const float alpha = _dt / (_lpf_time_constant + _dt);
	_residual_lpf = (1.f - alpha) * _residual_lpf + alpha * residual;
	_signed_innov_test_ratio_lpf = (1.f - alpha) * _signed_innov_test_ratio_lpf + alpha * math::constrain(
				       signed_innov_test_ratio, -1.f, 1.f);
}

inline void ZeroOrderHoverThrustEkf::updateMeasurementNoise(const float residual, const float H)
{
	const float alpha = _dt / (_noise_learning_time_constant + _dt);
	const float res_no_bias = residual - _residual_lpf;
	const float P = _state_var;
	_acc_var = math::constrain((1.f - alpha) * _acc_var  + alpha * (res_no_bias * res_no_bias + H * P * H), 1.f, 400.f);
}
```

## 7.5. Bumpless Integration into the Position Controller

When $T_h$ changes, applying it directly to the formula $T_z^{NED}=a_{sp,z}T_h/g - T_h$ causes a thrust jump and a vehicle jolt. The difference must be "absorbed" into the velocity controller integrator so that the output remains unchanged:

Derivation: to keep $T'_z=T_z$ when $T_h\to T_h'$:
$$
a_{sp,z}'\frac{T_h'}{g}-T_h' = a_{sp,z}\frac{T_h}{g}-T_h
$$
$$
\Rightarrow a_{sp,z}' = (a_{sp,z}-g)\frac{T_h}{T_h'}+g
$$

Since $a_{sp,z}=I_{v,z}+(\text{other terms})$, push the difference $\Delta a = a_{sp,z}'-a_{sp,z}$ into $I_{v,z}$:
$$
I_{v,z}\leftarrow I_{v,z}+(a_{sp,z}-g)\frac{T_h^{old}}{T_h^{new}}+g-a_{sp,z}
$$

**Variable explanation**:
- $T_z$: thrust output to the allocator (unchanged across the HTE update).
- $a_{sp,z}'$: new acceleration value required so that the thrust output is unchanged with $T_h^{new}$.
- $a_{sp,z}=I_{v,z}+(\text{P-term, FF})$ — only the accumulated integrator portion; P/FF will be updated automatically in the next cycle.
- By pushing the difference into $I_{v,z}$, the controller "accounts for" the $T_h$ change while keeping the output consistent → "bumpless transfer" — no vehicle jolt.

Cross-reference with code:

```@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp:75-89
void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}
```

## 7.6. When HTE Runs / Is "Frozen"

- Requires `vehicle_local_position.az_valid` and the vehicle must be armed and in flight (not landed).
- Requires thrust setpoint to not be saturated (near `MPC_THR_MAX` or `MPC_THR_MIN` → poor Jacobian information).
- Requires low $|\dot{\boldsymbol{\omega}}|$ and low tilt — vertical acceleration Z must not be distorted by lateral acceleration during rotation.

If conditions are not met → `hover_thrust_estimate.valid=false`, position controller falls back to `MPC_THR_HOVER`.

## 7.7. Theoretical Background & Keywords

| Topic | Keywords |
|---|---|
| Adaptive control | `recursive least squares RLS`, `Kalman filter for parameter identification`, `system identification online` |
| Online mass estimation | `online mass estimation UAV`, `quadcopter payload estimation`, `adaptive thrust mapping` |
| Innovation gating + bump | `chi-square test innovation`, `outlier rejection`, `covariance inflation` |
| Adaptive R / Q | `Sage-Husa adaptive Kalman`, `innovation-based adaptive estimation IAE`, `Mehra adaptive Kalman 1972` |
| Bumpless transfer | `bumpless transfer control`, `integrator preloading`, `controller switching` |
| ESC/motor model | `propeller momentum theory`, `thrust coefficient $C_T$`, `rotor disk model` |
| Sensor frame | `specific force vs acceleration`, `accelerometer dynamic model` |

### Papers / References
- Bresciani, Ferrari et al. — original HTE PX4 paper (see comment in `zero_order_hover_thrust_ekf.cpp`, author Mathieu Bresciani).
- Mehra (1972) — *Approaches to Adaptive Filtering*. IEEE TAC.
- Aström & Wittenmark, *Adaptive Control* — RLS chapter.
- Simon, *Optimal State Estimation* — adaptive Kalman chapter.

## 7.8. PX4 Parameters

| Parameter | Meaning |
|---|---|
| `MPC_USE_HTE` | Enable HTE (if off, uses fixed MPC_THR_HOVER) |
| `HTE_HT_NOISE` | Process noise $\sigma_p$ (rate of change of $T_h$) |
| `HTE_ACC_GATE` | Innovation gate |
| `HTE_HT_ERR_INIT` | Initial std-dev of $T_h$ |
| `MPC_THR_HOVER` | Fallback value when HTE is not valid |

## 7.9. Debug Tips
- Log topic `hover_thrust_estimate`: check `hover_thrust`, `hover_thrust_var`, `valid`, `accel_innov`.
- If HTE does not converge: check whether vertical acceleration Z has large noise (strong vibration), or whether the thrust setpoint is frequently saturated.
- When replacing payload in flight (cargo drop), expect a spike in `accel_innov` followed by variance bump recovery → `hover_thrust` converges to the new value in ~5–10 s.
