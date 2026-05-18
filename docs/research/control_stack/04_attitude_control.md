# 4. Attitude Control (Quaternion P-controller, tilt-prioritized)

## 4.1. Role

The middle loop of the cascade. Receives `vehicle_attitude_setpoint` ($\boldsymbol{q}_{sp}, \boldsymbol{T}^{body}, \dot{\psi}_{sp}$) from the position controller, along with `vehicle_attitude` ($\boldsymbol{q}$) from EKF2. Output is `vehicle_rates_setpoint` ($\boldsymbol{\omega}_{sp}, \boldsymbol{T}^{body}$) for the rate controller.

Philosophy: **pure P-controller** (no I/D); bias compensation is handled by the rate controller below. The subtle point is **separating yaw from tilt** because thrust depends only on tilt — yaw error is less dangerous than tilt error.

## 4.2. Main Code

| Role | File |
|---|---|
| uORB wrapper + manual stick | `@/home/frank/tf-px4/src/modules/mc_att_control/mc_att_control_main.cpp` |
| Parameter header | `@/home/frank/tf-px4/src/modules/mc_att_control/mc_att_control.hpp` |
| **Algorithm core** | `@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp` |
| Math helper (VTOL tilt correction) | `AttitudeControl/AttitudeControlMath.hpp` |
| Yaw stick handler | `@/home/frank/tf-px4/src/lib/stick_yaw/` |
| Test | `AttitudeControl/AttitudeControlTest.cpp` |

Functions to read:
- `MulticopterAttitudeControl::Run()` — orchestrator (line 206 onward).
- `MulticopterAttitudeControl::generate_attitude_setpoint(q, dt)` — generates $\boldsymbol{q}_{sp}$ from stick (Stabilized mode).
- `AttitudeControl::update(q)` — main control law.

## 4.3. Symbol Table

### Input / state

| Symbol | Code | Meaning |
|---|---|---|
| $\boldsymbol{q}$ | `q` | current attitude quaternion (body→world, Hamilton scalar-first) |
| $\boldsymbol{q}_{sp}$ | `_attitude_setpoint_q` / `qd` | attitude setpoint from position control |
| $\dot{\psi}_{sp}$ | `_yawspeed_setpoint` | feed-forward yaw rate (rad/s, in $\{W\}$) |
| $\boldsymbol{e}_z=\boldsymbol{R}(\boldsymbol{q})\hat{\boldsymbol{z}}_W$ | `e_z` (`q.dcm_z()`) | body z-axis expressed in $\{W\}$ |
| $\boldsymbol{e}_z^{sp}=\boldsymbol{R}(\boldsymbol{q}_{sp})\hat{\boldsymbol{z}}_W$ | `e_z_d` | desired body z-axis in $\{W\}$ |

### Intermediate

| Symbol | Code | Meaning |
|---|---|---|
| $\boldsymbol{q}_{red}$ | `qd_red` | reduced desired attitude — cares only about tilt, ignores yaw |
| $\boldsymbol{q}_{\delta\psi}$ | `qd_dyaw` | remaining yaw component $\boldsymbol{q}_{red}^{-1}\!\otimes\!\boldsymbol{q}_{sp}$ |
| $\boldsymbol{q}_d$ | `qd` (after scaling) | blended desired attitude (full tilt + weighted yaw) |
| $\boldsymbol{q}_e=\boldsymbol{q}^{-1}\!\otimes\!\boldsymbol{q}_d$ | `qe` | attitude error quaternion |
| $\boldsymbol{e}_q=2\,\mathrm{Im}(\boldsymbol{q}_e^{canonical})$ | `eq` | rotation vector error (≈ $\alpha\hat{\boldsymbol{r}}$ for small $\alpha$) |
| $\boldsymbol{\omega}_{sp}$ | `rate_setpoint` (output) | rate setpoint output to rate controller |

### Gains & parameters

| Symbol | Code / param | Meaning |
|---|---|---|
| $\boldsymbol{K}_p^{att}$ | `_proportional_gain` | (`MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P/`$w_\psi$) |
| $w_\psi\in[0,1]$ | `_yaw_w` / `MC_YAW_WEIGHT` | yaw vs tilt weight (default 0.4) |
| $\omega_{max,i}$ | `_rate_limit` / `MC_*RATE_MAX` | rate output limit |
| $\theta_{max}$ | `MPC_MAN_TILT_MAX` | maximum tilt in manual mode |

### Operators

| Symbol | Meaning |
|---|---|
| $\otimes$ | Hamilton quaternion multiplication |
| $\mathrm{Im}(\boldsymbol{q})$ | vector part $(q_x,q_y,q_z)$ |
| $\mathrm{canonical}(\boldsymbol{q})$ | choose the solution with $q_w\ge 0$ (avoid antipodal) |
| `q.dcm_z()` | column 3 of $\boldsymbol{R}(\boldsymbol{q})$ — which is $\boldsymbol{R}\hat{\boldsymbol{z}}_W$ |

---

## 4.4. Detailed Formulas

The full control law is in the `update` function:

```@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:55-114
matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	Quatf qd = _attitude_setpoint_q;

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		qd_red = qd;

	} else {
		// Transform rotation from current to desired thrust vector into a world frame reduced desired attitude.
		// This is a right multiplication as the tilt error quaternion is obtained from two Z vectors expressed in the world frame.
		qd_red *= q;
	}

	// With a full desired attitude given by: qd = qd_red * qd_dyaw, extract the delta yaw component.
	// By definition, the delta yaw quaternion has the form (cos(angle/2), 0, 0, sin(angle/2))
	Quatf qd_dyaw = qd_red.inversed() * qd;
	qd_dyaw.canonicalize();
	// catch numerical problems with the domain of acosf and asinf
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	// scale the delta yaw angle and re-combine the desired attitude
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	const Quatf qe = q.inversed() * qd;

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	const Vector3f eq = 2.f * qe.canonical().imag();

	// calculate angular rates setpoint
	Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// Feed forward the yaw setpoint rate.
	if (std::isfinite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
```

Each step is broken down below and mapped to its formula.

### Step 1 — Reduced attitude (tilt-only correction)

Current and desired body Z-axis (both expressed in $\{W\}$):
$$
\boldsymbol{e}_z = \boldsymbol{R}(\boldsymbol{q})\hat{\boldsymbol{z}}_W,\qquad
\boldsymbol{e}_z^{sp}=\boldsymbol{R}(\boldsymbol{q}_{sp})\hat{\boldsymbol{z}}_W
$$

Minimum-rotation quaternion between the two axes:
$$
\boldsymbol{q}_{red}^{(W)} = \mathrm{quat\_from\_two\_vectors}(\boldsymbol{e}_z,\boldsymbol{e}_z^{sp})
$$

This is a rotation in the world frame; right-multiply by $\boldsymbol{q}$:
$$
\boldsymbol{q}_{red}=\boldsymbol{q}_{red}^{(W)}\otimes\boldsymbol{q}
$$

**Variable explanation**:
- $\hat{\boldsymbol{z}}_W=(0,0,1)^\top$ (NED, pointing down).
- $\boldsymbol{e}_z,\boldsymbol{e}_z^{sp}$: indicate where the body vertical axis currently points versus where it should point. If they coincide → tilt is OK.
- $\boldsymbol{q}_{red}^{(W)}$: the minimum rotation (axis perpendicular to both vectors) that takes $\boldsymbol{e}_z\to\boldsymbol{e}_z^{sp}$ — rotates tilt only, does not change yaw.
- Right-multiply by $\boldsymbol{q}$: combines the tilt rotation (in $\{W\}$) with the current attitude → produces reduced desired attitude $\boldsymbol{q}_{red}$ (its yaw equals the yaw of $\boldsymbol{q}$ — i.e., "keeps current yaw").

Code:

```@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:60-74
const Vector3f e_z = q.dcm_z();
const Vector3f e_z_d = qd.dcm_z();
Quatf qd_red(e_z, e_z_d);

if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
	qd_red = qd;
} else {
	qd_red *= q;
}
```
$\boldsymbol{q}_{red}$ = "desired attitude if only tilt is considered" — aligns the body Z-axis with the setpoint, leaving yaw free.

**Edge case**: when $\boldsymbol{e}_z\approx-\boldsymbol{e}_z^{sp}$ (180° axis divergence), the code falls back to `q_red = q_sp` to avoid singularity (singular axis).

### Step 2 — Yaw delta component + scaling by $w_\psi$

$$
\boldsymbol{q}_{\delta\psi} = \boldsymbol{q}_{red}^{-1}\otimes\boldsymbol{q}_{sp}
$$

By the decomposition theorem, $\boldsymbol{q}_{\delta\psi}$ contains only yaw around $\hat{\boldsymbol{z}}_B$ → has the form $(\cos(\alpha/2),0,0,\sin(\alpha/2))$.

Apply weight $w_\psi$ = `MC_YAW_WEIGHT` (default 0.4 — set < 1 so that tilt is still prioritized when yaw deviation is large):
$$
\boldsymbol{q}_{\delta\psi}^{(w)} = \big(\cos(w_\psi\arccos q_{\delta\psi,w}),\ 0,\ 0,\ \sin(w_\psi\arcsin q_{\delta\psi,z})\big)
$$

Blended desired attitude (full tilt priority + weighted yaw):
$$
\boldsymbol{q}_d = \boldsymbol{q}_{red}\otimes\boldsymbol{q}_{\delta\psi}^{(w)}
$$

**Variable explanation**:
- $q_{\delta\psi,w}=\cos(\alpha/2)$, $q_{\delta\psi,z}=\sin(\alpha/2)$ where $\alpha$ = yaw deviation angle.
- $\arccos q_{\delta\psi,w}=\alpha/2$ → multiply by $w_\psi$ → $\cos(w_\psi\alpha/2)$ = scales yaw error to $w_\psi\alpha$ ("moves slower" compared to tilt).
- Using `acosf(qd_dyaw(0))` and `asinf(qd_dyaw(3))` separately improves numerical stability near $\alpha\approx\pi$ (acos loses precision at $\pm 1$, asin loses precision near 0).
- $\boldsymbol{q}_d$: "shifts" from $\boldsymbol{q}_{red}$ adding scaled yaw → prioritizes tilt (full gain) over yaw (gain * $w_\psi$).

Code:

```@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:78-85
Quatf qd_dyaw = qd_red.inversed() * qd;
qd_dyaw.canonicalize();
// catch numerical problems with the domain of acosf and asinf
qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

// scale the delta yaw angle and re-combine the desired attitude
qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))), 0.f, 0.f, sinf(_yaw_w * asinf(qd_dyaw(3))));
```

### Step 3 — Quaternion error → angular rate setpoint

$$
\boldsymbol{q}_e = \boldsymbol{q}^{-1}\otimes\boldsymbol{q}_d,\quad \boldsymbol{q}_e\leftarrow\mathrm{sign}(q_{e,w})\boldsymbol{q}_e\quad(\text{canonicalize})
$$

Theorem: for small rotations, $\mathrm{Im}(\boldsymbol{q}_e)=\sin(\alpha/2)\hat{\boldsymbol{r}}\approx(\alpha/2)\hat{\boldsymbol{r}}$. P law:
$$
\boxed{\ \boldsymbol{\omega}_{sp} = 2\,\boldsymbol{K}_p^{att}\odot\mathrm{Im}(\boldsymbol{q}_e)\ }
$$

**Variable explanation**:
- $\boldsymbol{q}_e$ is in the **body frame** (because $\boldsymbol{q}^{-1}\otimes\boldsymbol{q}_d$) → $\mathrm{Im}(\boldsymbol{q}_e)$ is directly used to command rate in the body frame.
- `canonicalize`: flips the sign of the entire quaternion if $q_{e,w}<0$ — since $\boldsymbol{q}$ and $-\boldsymbol{q}$ represent the same rotation, this selects the "short path" solution.
- The factor $2$ comes from $\sin(\alpha/2)\to\alpha/2$: multiplied by 2 to get rotation vector $\alpha\hat{\boldsymbol{r}}$ (unit: radians).
- $\boldsymbol{K}_p^{att}=(K_p^\phi, K_p^\theta, K_p^\psi/w_\psi)$: yaw gain is *divided back* by $w_\psi$ so that the final rate response is equivalent even though the yaw error was scaled in Step 2 (inverse compensation).

Code:

```@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:87-95
// quaternion attitude control law, qe is rotation from q to qd
const Quatf qe = q.inversed() * qd;

// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
// also taking care of the antipodal unit quaternion ambiguity
const Vector3f eq = 2.f * qe.canonical().imag();

// calculate angular rates setpoint
Vector3f rate_setpoint = eq.emult(_proportional_gain);
```

Note: the inverse division of yaw gain by $w_\psi$ happens in `setProportionalGain`:

```@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:44-53
void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}
}
```

### Step 4 — Feed-forward yaw rate

$\dot{\psi}_{sp}$ is the angular rate around $\hat{\boldsymbol{z}}_W$. To add it to $\boldsymbol{\omega}_{sp}$ (body frame), project $\hat{\boldsymbol{z}}_W$ into the body:
$$
\boldsymbol{\omega}_{sp}\mathrel{+}= \boldsymbol{R}^\top(\boldsymbol{q})\hat{\boldsymbol{z}}_W\,\dot{\psi}_{sp}
$$

**Variable explanation**:
- $\boldsymbol{R}(\boldsymbol{q})=\boldsymbol{R}_{WB}$ → $\boldsymbol{R}^\top=\boldsymbol{R}_{BW}$ transforms a vector from $\{W\}$ to $\{B\}$.
- $\boldsymbol{R}^\top\hat{\boldsymbol{z}}_W$ = column 3 of $\boldsymbol{R}^\top$ = $\hat{\boldsymbol{z}}_W$ expressed in the body. When tilt = 0: $=\hat{\boldsymbol{z}}_B=(0,0,1)$ → yaw rate is added only to the body r-axis. When tilted: all three components are affected.
- This formula ensures yaw FF correctly represents *world-frame yaw rate*, without error at high tilt (fixes the gimbal lock issue of Euler-rate FF).

Code:

```@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:104-106
if (std::isfinite(_yawspeed_setpoint)) {
	rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
}
```

### Step 5 — Rate limiting
$$
\omega_{sp,i}\leftarrow\mathrm{clip}(\omega_{sp,i},-\omega_{max,i},\omega_{max,i})
$$
- $\omega_{max,i}$: per-axis rate limit (`MC_{ROLL,PITCH,YAW}RATE_MAX`, rad/s).
- Protects the rate controller from unreasonable angular velocity commands (e.g. when attitude error > 180°).

```@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:108-111
// limit rates
for (int i = 0; i < 3; i++) {
	rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
}
```

## 4.5. Generate attitude setpoint from stick (Stabilized mode)

When there is no position control, $\boldsymbol{q}_{sp}$ is generated from RC. See `mc_att_control_main.cpp:136-203`.

Tilt from stick:
$$
\boldsymbol{v}=(roll\cdot\theta_{max},\ -pitch\cdot\theta_{max})\quad\text{after passing through filter time-constant }\tau_{tilt}
$$
$$
\|\boldsymbol{v}\|>\theta_{max}\Rightarrow \boldsymbol{v}\leftarrow\boldsymbol{v}\frac{\theta_{max}}{\|\boldsymbol{v}\|}
$$

**Variable explanation**:
- $roll, pitch\in[-1,1]$: stick values (with expo + deadzone applied).
- $\theta_{max}$ = `MPC_MAN_TILT_MAX` (rad).
- $\boldsymbol{v}=(v_x,v_y)$: 2D tilt rotation vector — magnitude = tilt angle, direction = tilt rotation axis in horizontal $\{W\}$.
- The negative sign on pitch because positive pitch stick = nose down = rotation around $-\hat{\boldsymbol{e}}_y$.
- $\tau_{tilt}$ = `MC_MAN_TILT_TAU` — low-pass filter to avoid tilt shock when stick is jerked.

Roll-pitch quaternion in axis-angle form:
$$
\boldsymbol{q}_{rp} = \mathrm{AxisAngle}(v_x,v_y,0)
$$
- $\mathrm{AxisAngle}(\boldsymbol{u})=(\cos(\|\boldsymbol{u}\|/2),\ \mathrm{sinc}(\|\boldsymbol{u}\|/2)\boldsymbol{u}/2)$ — no yaw component because $u_z=0$.

Yaw from stick (see `StickYaw` lib): integrate yaw stick with expo + deadzone:
$$
\boldsymbol{q}_{yaw}=(\cos(\psi_{sp}/2),0,0,\sin(\psi_{sp}/2))
$$
- $\psi_{sp}$ is integrated from yaw stick rate × $\Delta t$ → "hold last yaw" when stick = 0.

Final setpoint:
$$
\boldsymbol{q}_{sp}=\boldsymbol{q}_{yaw}\otimes\boldsymbol{q}_{rp}
$$
- Order: yaw first (around $\hat{\boldsymbol{z}}_W$), then tilt — equivalent to rotating in intermediate yawed $\{W\}$ frame.

Throttle stick → `thrust_body[2] = -throttle_curve(stick)` with `MPC_THR_CURVE` selecting the curve shape (linear/HTE-rescaled).

## 4.6. EKF Reset Handling

When EKF resets yaw (`quat_reset_counter` increments), receive `delta_q_reset`:
$$
\psi_{sp,stab}\leftarrow\mathrm{wrap}_\pi(\psi_{sp,stab}+\delta\psi)
$$
$$
\boldsymbol{q}_{sp}\leftarrow \delta\boldsymbol{q}_{reset}\otimes\boldsymbol{q}_{sp}
$$
→ avoids jerk when EKF jumps yaw.

## 4.7. Theoretical Background & Keywords

| Topic | Keywords |
|---|---|
| Quaternion attitude control | `Brescianini Hehn D'Andrea ETH 2013 nonlinear quadrocopter attitude control`, `quaternion P-controller` |
| Tilt-prioritized | `tilt prioritization`, `reduced attitude control`, `attitude decomposition tilt-yaw` |
| Geometric SO(3) control | `Lee SE(3) geometric control`, `Bullo Lewis geometric mechanics`, `Mahony Hua Hamel SO(3) control` |
| Quaternion math | `Hamilton convention`, `right vs left composition`, `axis-angle exponential`, `slerp` |
| Singularity avoidance | `gimbal lock`, `quaternion antipodal`, `canonicalization` |

### Foundational Papers
- Brescianini, Hehn, D'Andrea (2013) — *Nonlinear Quadrocopter Attitude Control*. ETH tech report — **this is precisely the algorithm PX4 currently uses**. Original link in the comment at the top of `mc_att_control_main.cpp`.
- Mahony, Hua, Hamel (2011) — *Multirotor Aerial Vehicles: Modeling, Estimation, and Control*. IEEE RAM.
- Fresk & Nikolakopoulos (2013) — *Full Quaternion Based Attitude Control for a Quadrotor*.

### Further Reading
- Markley & Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control* — quaternion chapter.
- Sola tutorial (referenced in §2) for quaternion decomposition.

## 4.8. PX4 Parameters

| Parameter | Meaning |
|---|---|
| `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P` | Attitude P gains |
| `MC_YAW_WEIGHT` | Yaw vs tilt weight (default 0.4) |
| `MC_ROLLRATE_MAX`, `MC_PITCHRATE_MAX`, `MC_YAWRATE_MAX` | Rate output limits |
| `MPC_MAN_TILT_MAX`, `MC_MAN_TILT_TAU` | Max tilt + filter constant for stick |
| `MPC_THR_CURVE`, `MPC_THR_HOVER` | Manual throttle curve |

## 4.9. Debug Tips
- Log: `vehicle_attitude_setpoint.q_d` vs `vehicle_attitude.q` → compute error angle with $2\arccos|q_e\cdot q|$.
- If yaw is slow: increase `MC_YAW_P` or decrease `MC_YAW_WEIGHT` (paradox — smaller weight consumes less yaw axis budget but response is slower).
- If drone wobbles at high tilt: usually because tilt-priority is active → normal, do NOT increase att gain.
