# 5. Rate Control (PID + FF + Saturation-aware Anti-Windup)

## 5.1. Role

The innermost loop. Runs on gyro updates (~1 kHz). Receives `vehicle_rates_setpoint` ($\boldsymbol{\omega}_{sp}$) from the attitude controller, gyro $\boldsymbol{\omega}$ and derivative $\dot{\boldsymbol{\omega}}$ from EKF/sensors. Output is `vehicle_torque_setpoint` ($\boldsymbol{\tau}\in[-1,1]^3$) for the control allocator.

This is the **most stability-critical loop**: high frequency, simple, and "hard". Most UAV tuning revolves around this stage.

## 5.2. Main Code

| Role | File |
|---|---|
| uORB wrapper + gyro-scheduled execution | `@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.cpp` |
| Parameter header | `@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.hpp` |
| **PID algorithm core** | `@/home/frank/tf-px4/src/lib/rate_control/rate_control.cpp` |
| Lib header (must read) | `@/home/frank/tf-px4/src/lib/rate_control/rate_control.hpp` |
| Test | `@/home/frank/tf-px4/src/lib/rate_control/rate_control_test.cpp` |

Functions to read:
- `MulticopterRateControl::Run()` — orchestrator (registers gyro callback).
- `MulticopterRateControl::parameters_updated()` — maps `K*P/I/D` parameters → ideal form.
- `RateControl::update(rate, rate_sp, angular_accel, dt, landed)` — main PID law.
- `RateControl::updateIntegral(rate_error, dt)` — nonlinear anti-windup.

## 5.3. Symbol Table

### Input / output

| Symbol | Code | Meaning |
|---|---|---|
| $\boldsymbol{\omega}_{sp}=(p,q,r)_{sp}^\top$ | `_rates_setpoint` / `rate_sp` | rate setpoint from attitude controller (rad/s, body) |
| $\boldsymbol{\omega}=(p,q,r)^\top$ | `rates` | measured gyro (rad/s, body) |
| $\dot{\boldsymbol{\omega}}$ | `angular_accel` | angular acceleration (filtered gyro derivative, from sensor pipeline) |
| $\boldsymbol{\tau}\in[-1,1]^3$ | `torque` / output | normalized torque setpoint (per body axis) |
| $\boldsymbol{T}\in\mathbb{R}^3$ | `_thrust_setpoint` | normalized thrust setpoint (body) |

### Intermediate

| Symbol | Code | Meaning |
|---|---|---|
| $\boldsymbol{e}_\omega=\boldsymbol{\omega}_{sp}-\boldsymbol{\omega}$ | `rate_error` | rate error (rad/s) |
| $\boldsymbol{I}_\omega$ | `_rate_int` | integrator state |
| $i_{f,i}\in[0,1]$ | `i_factor` | nonlinear coefficient that reduces I-gain when error is large |

### Gains & limits

| Symbol | Code / param | Meaning |
|---|---|---|
| $\boldsymbol{K}$ | `MC_{R,P,Y}RATE_K` | master gain (ideal form) |
| $\boldsymbol{p},\boldsymbol{i},\boldsymbol{d},\boldsymbol{ff}$ | `MC_{R,P,Y}RATE_{P,I,D,FF}` | parallel-form parameters |
| $\boldsymbol{K}_p^\omega=\boldsymbol{K}\odot\boldsymbol{p}$ | `_gain_p` | rate P-gain (after conversion) |
| $\boldsymbol{K}_i^\omega,\boldsymbol{K}_d^\omega,\boldsymbol{K}_{ff}^\omega$ | `_gain_{i,d,ff}` | rate I/D/FF gains |
| $I_{lim,i}$ | `_lim_int` / `MC_{R,P,Y}R_INT_LIM` | integrator bounds |
| $\theta_{ref}=400\deg=6.98$ rad/s | hard-coded | scale of nonlinear $i$-factor |
| $f_c^{yaw}$ | `MC_YAW_TQ_CUTOFF` | yaw torque LPF |
| $s_{bat}=V_{nom}/V_{batt}$ | `_battery_status_scale` | battery voltage compensation scale |

### Saturation flags from allocator

| Symbol | Code | Meaning |
|---|---|---|
| $s_i^+\in\{0,1\}$ | `_control_allocator_saturation_positive(i)` | axis $i$ saturated in positive direction |
| $s_i^-\in\{0,1\}$ | `_control_allocator_saturation_negative(i)` | axis $i$ saturated in negative direction |

---

## 5.4. Detailed Formulas

### Step 1 — Parameter conversion to ideal form

The `K*` parameter allows expressing the controller in *parallel* form (`P + I/s + sD`) ↔ *ideal* form (`K(1+1/sTi+sTd)`):

$$
K_p^\omega = \boldsymbol{K}\odot\boldsymbol{p},\quad K_i^\omega=\boldsymbol{K}\odot\boldsymbol{i},\quad K_d^\omega=\boldsymbol{K}\odot\boldsymbol{d}
$$

**Variable explanation**:
- $\boldsymbol{K}$ = `MC_{R,P,Y}RATE_K`: master gain in ideal form ($K$ in $K(1+1/sTi+sTd)$).
- $\boldsymbol{p},\boldsymbol{i},\boldsymbol{d}$ = `MC_{R,P,Y}RATE_{P,I,D}`: parallel-form P/I/D ratios (tuners are typically more familiar with this form).
- Effect: tuning only $\boldsymbol{K}$ → scales all P/I/D uniformly, preserving the resonant balance (PID dynamics unchanged, system is simply "sped up").

```@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.cpp:78-92
// rate control parameters
// The controller gain K is used to convert the parallel (P + I/s + sD) form
// to the ideal (K * [1 + 1/sTi + sTd]) form
const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

_rate_control.setPidGains(
	rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
	rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
	rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

_rate_control.setIntegratorLimit(
	Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

_rate_control.setFeedForwardGain(
	Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));
```

### Step 2 — PID law + Feed-forward

$$
\boldsymbol{e}_\omega = \boldsymbol{\omega}_{sp}-\boldsymbol{\omega}
$$

$$
\boxed{\ \boldsymbol{\tau} = \boldsymbol{K}_p^\omega\odot\boldsymbol{e}_\omega + \boldsymbol{I}_\omega - \boldsymbol{K}_d^\omega\odot\dot{\boldsymbol{\omega}} + \boldsymbol{K}_{ff}^\omega\odot\boldsymbol{\omega}_{sp}\ }
$$

**Variable explanation**:
- $\boldsymbol{e}_\omega$: rate error (rad/s).
- $\boldsymbol{I}_\omega$: accumulated integrator, updated via `updateIntegral` (see Step 3).
- $\dot{\boldsymbol{\omega}}$: actual gyro derivative (NOT $\dot{\boldsymbol{e}}_\omega$) → **minus** sign + applied directly to measurement → **derivative on measurement**.
- $\boldsymbol{\omega}_{sp}$ in the FF-term goes directly to output ("feed-forward through"), bypasses the P loop → increases response speed for smooth maneuvers.

```@/home/frank/tf-px4/src/lib/rate_control/rate_control.cpp:71-86
Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}
```

Notable characteristics:
- D-term acts on $\dot{\boldsymbol{\omega}}$ directly (`angular_accel`, gyro derivative from sensor pipeline §1.5.5) — prevents "derivative kick" when $\boldsymbol{\omega}_{sp}$ steps.
- FF-term acts directly on $\boldsymbol{\omega}_{sp}$ (`rate_sp`) — bypasses P, enabling fast tracking of smooth maneuvers.

### Step 3 — Anti-Windup Integration

Full `updateIntegral` function:

```@/home/frank/tf-px4/src/lib/rate_control/rate_control.cpp:88-118
void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}
```

Each mechanism is broken down below:

#### (a) Saturation feedback from Allocator

If the allocator has saturated the motors in the positive direction:

$$
e_{\omega,i}\leftarrow\min(e_{\omega,i},0),\quad\text{or }e_{\omega,i}\leftarrow\max(e_{\omega,i},0)\text{ for negative direction}
$$

**Variable explanation**:
- $s_i^+ = 1$: axis $i$ saturated in the positive direction (motor already at max in the direction producing $\tau_i>0$).
- By clipping $e_{\omega,i}\le 0$, the integrator can only decrease in the negative direction → does not wind up further in the already-saturated direction.
- Same applies for $s_i^- = 1$ in the negative direction.

This flag is set by `MulticopterRateControl::Run()` from the topic `control_allocator_status.unallocated_torque`:

```@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.cpp:197-216
// update saturation status from control allocation feedback
control_allocator_status_s control_allocator_status;

if (_control_allocator_status_sub.update(&control_allocator_status)) {
	Vector<bool, 3> saturation_positive;
	Vector<bool, 3> saturation_negative;

	if (!control_allocator_status.torque_setpoint_achieved) {
		for (size_t i = 0; i < 3; i++) {
			if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
				saturation_positive(i) = true;

			} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
				saturation_negative(i) = true;
			}
		}
	}

	// TODO: send the unallocated value directly for better anti-windup
	_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
}
```

#### (b) Nonlinear $i$-factor (reduce integration when error is large)

$$
i_{f,i} = \max\!\left(0,\ 1-\left(\frac{e_{\omega,i}}{\theta_{ref}}\right)^2\right),\quad \theta_{ref}=400°=6.98\,\text{rad/s}
$$

**Variable explanation**:
- $i_{f,i}\in[0,1]$: coefficient multiplied into the I-gain before integration.
- $\theta_{ref}$ = error threshold at which $i_f=0$ (hard-coded `math::radians(400.f)` $\approx 6.98$ rad/s).
- Parabolic shape $1-x^2$ → "soft" relationship: for $|e_\omega|<100°$, $i_f\approx 1$ (nearly no effect); for $|e_\omega|=200°$, $i_f\approx 0.75$; for $|e_\omega|\ge400°$, $i_f=0$.
- Purpose: avoid "bounce-back" after a flip — when error is very large, the integrator should not build up (a form of supplementary "clamping" anti-windup).

```@/home/frank/tf-px4/src/lib/rate_control/rate_control.cpp:107-108
float i_factor = rate_error(i) / math::radians(400.f);
i_factor = math::max(0.0f, 1.f - i_factor * i_factor);
```

#### (c) Euler integration + clamp

$$
I_{\omega,i}\leftarrow\mathrm{clip}\!\left(I_{\omega,i}+i_{f,i}\,K_i^\omega\,e_{\omega,i}\Delta t,\ -I_{lim,i},\ I_{lim,i}\right)
$$

**Variable explanation**:
- $i_{f,i}\,K_i^\omega\,e_{\omega,i}\Delta t$: single Euler step integration amount, with nonlinear factor applied.
- $I_{lim,i}$ = `MC_{R,P,Y}R_INT_LIM`: absolute clamp bound — prevents the integrator from consuming the entire torque output budget.
- When `landed` or disarmed → reset $\boldsymbol{I}_\omega=\mathbf{0}$ (outside `updateIntegral`, from the wrapper).

```@/home/frank/tf-px4/src/lib/rate_control/rate_control.cpp:111-116
float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

// do not propagate the result if out of range or invalid
if (PX4_ISFINITE(rate_i)) {
	_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
}
```

### Step 4 — Yaw torque LPF

$$
\tau_z\leftarrow \mathrm{LPF}_{f_c=\text{MC\_YAW\_TQ\_CUTOFF}}(\tau_z)
$$

**Variable explanation**:
- $\tau_z$: yaw component of the torque setpoint.
- $\mathrm{LPF}_{f_c}$: 1st-order alpha filter (see §1.5.5) with cutoff at $f_c$ = `MC_YAW_TQ_CUTOFF` Hz.
- Applied only to the yaw axis, roll/pitch are unaffected.
- Purpose: yaw torque is generated by the RPM differential between CW/CCW motors (reaction-torque coefficient $c_m$ is small, ~1/100 of thrust) → requires large amplitude → amplifies noise. LPF suppresses vibration.

```@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.cpp:218-223
// run rate controller
Vector3f torque_setpoint =
	_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);

// apply low-pass filtering on yaw axis to reduce high frequency torque caused by rotor acceleration
torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);
```

### Step 5 — Battery scaling (optional)

When `MC_BAT_SCALE_EN=1`:

$$
s_{bat}=\frac{V_{nom}}{V_{batt}},\quad
\boldsymbol{\tau}\leftarrow\mathrm{clip}(s_{bat}\boldsymbol{\tau},-1,1),\ \boldsymbol{T}\leftarrow\mathrm{clip}(s_{bat}\boldsymbol{T},-1,1)
$$

**Variable explanation**:
- $V_{nom}$: nominal battery voltage (e.g. 16.8 V for a full 4S).
- $V_{batt}$: measured voltage (from `battery_status`).
- $s_{bat}\ge 1$: battery voltage drops → higher PWM needed to produce the same force (because motor force is proportional to $V^2$).
- Clip to $[-1,1]$ prevents overflow.
- Battery voltage compensation → maintains consistent response throughout the flight (gain stability not affected by battery SoC drift).

```@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.cpp:241-256
// scale setpoints by battery status if enabled
if (_param_mc_bat_scale_en.get()) {
	if (_battery_status_sub.updated()) {
		battery_status_s battery_status;

		if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
			_battery_status_scale = battery_status.scale;
		}
	}

	if (_battery_status_scale > 0.f) {
		for (int i = 0; i < 3; i++) {
			vehicle_thrust_setpoint.xyz[i] = math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
			vehicle_torque_setpoint.xyz[i] = math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
		}
	}
}
```

## 5.5. ACRO mode (manual without attitude loop)

When `flag_control_manual_enabled && !flag_control_attitude_enabled`:

```@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.cpp:154-177
if (_vehicle_control_mode.flag_control_manual_enabled && !_vehicle_control_mode.flag_control_attitude_enabled) {
	// generate the rate setpoint from sticks
	manual_control_setpoint_s manual_control_setpoint;

	if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		// manual rates control - ACRO mode
		const Vector3f man_rate_sp{
			math::superexpo(manual_control_setpoint.roll, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
			math::superexpo(-manual_control_setpoint.pitch, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
			math::superexpo(manual_control_setpoint.yaw, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

		_rates_setpoint = man_rate_sp.emult(_acro_rate_max);
		_thrust_setpoint(2) = -(manual_control_setpoint.throttle + 1.f) * .5f;
		_thrust_setpoint(0) = _thrust_setpoint(1) = 0.f;

		// publish rate setpoint
		vehicle_rates_setpoint.roll = _rates_setpoint(0);
		vehicle_rates_setpoint.pitch = _rates_setpoint(1);
		vehicle_rates_setpoint.yaw = _rates_setpoint(2);
		_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
		vehicle_rates_setpoint.timestamp = hrt_absolute_time();

		_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
	}
}
```

$$
x_{shape}=\mathrm{superexpo}(x,e_{exp},s_{sup})=(1-e_{exp})x+e_{exp}x^3,\ \text{then}\ \frac{1-s_{sup}}{1-s_{sup}|x|}
$$

$$
\boldsymbol{\omega}_{sp} = \mathrm{shape}(\text{stick})\odot \boldsymbol{\omega}_{max}^{acro}
$$

**Variable explanation**:
- $x\in[-1,1]$: stick value.
- $e_{exp}$ = `MC_ACRO_EXPO*`: curve curvature between stick and output (0 = linear, 1 = very curved in the middle).
- $s_{sup}$ = `MC_ACRO_SUPEXPO*`: super-expo factor — increases sensitivity at the extremes.
- $\boldsymbol{\omega}_{max}^{acro}$ = `MC_ACRO_{R,P,Y}_MAX` (rad/s): maximum rate when stick = $\pm 1$.
- Purpose: gives the pilot an "easy to control" zone in the middle, while still achieving high rates when stick is at the extreme.

## 5.6. Theoretical Background & Keywords

| Topic | Keywords |
|---|---|
| PID parallel vs ideal | `parallel form vs ideal form PID`, `Astrom Hagglund PID textbook` |
| Anti-windup | `back-calculation`, `tracking anti-windup`, `conditional integration`, `clamping anti-windup`, `Aström Rundqwist 1989` |
| Saturation-aware control | `command-aware integrator`, `pilot-induced oscillation prevention` |
| Derivative kick | `derivative on measurement vs error`, `setpoint weighting PID` |
| Feed-forward in rate loop | `2-DOF rate controller`, `inverse model FF` |
| Body-rate aggressive flight | `Faessler Falanga Scaramuzza body rate control 2018` |
| Notch on D-term | `D-term filtering`, `Betaflight RPM filter`, `gyro feedback shaping` |
| Battery compensation | `voltage compensation ESC`, `thrust normalization`, `motor RPM model` |

### Papers / Books
- Astrom & Hagglund, *PID Controllers: Theory, Design, and Tuning* — Chapter 3 (anti-windup).
- Faessler, Falanga, Scaramuzza (2018) — *Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight*. RAL.
- Pounds, Mahony, Corke (2010) — *Modelling and Control of a Large Quadrotor Robot*.
- Furrer et al. — RotorS Gazebo paper (motor + ESC model).

## 5.7. PX4 Parameters

| Parameter | Meaning |
|---|---|
| `MC_ROLLRATE_K`, `MC_PITCHRATE_K`, `MC_YAWRATE_K` | Master gain (scales all P/I/D per axis) |
| `MC_*RATE_P`, `MC_*RATE_I`, `MC_*RATE_D`, `MC_*RATE_FF` | PID + FF gains |
| `MC_RR_INT_LIM`, `MC_PR_INT_LIM`, `MC_YR_INT_LIM` | Integrator limits |
| `MC_YAW_TQ_CUTOFF` | Yaw torque LPF |
| `MC_BAT_SCALE_EN` | Enable battery scaling |
| `MC_ACRO_*_MAX`, `MC_ACRO_EXPO*`, `MC_ACRO_SUPEXPO*` | ACRO shaping |
| `IMU_DGYRO_CUTOFF` | LPF for $\dot{\boldsymbol{\omega}}$ (D-term) |

## 5.8. Tuning Tips
1. Set `K=1`, P/I/D per airframe defaults.
2. Increase `MC_*RATE_K` until high-frequency oscillation appears (~> 30 Hz) → reduce by 30%.
3. Increase `MC_*RATE_I/MC_*RATE_K` until steady-state error disappears.
4. Increase D-term if overshoot occurs, but be careful of noise amplification.
5. Log `rate_ctrl_status.{rollspeed_integ,...}` to check whether the integrator saturates.
