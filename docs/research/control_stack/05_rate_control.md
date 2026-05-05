# 5. Rate Control (PID + FF + Saturation-aware Anti-Windup)

## 5.1. Vai trò

Vòng trong cùng. Chạy theo gyro update (~1 kHz). Nhận `vehicle_rates_setpoint` ($\boldsymbol{\omega}_{sp}$) từ attitude controller, gyro $\boldsymbol{\omega}$ và đạo hàm $\dot{\boldsymbol{\omega}}$ từ EKF/sensors. Output là `vehicle_torque_setpoint` ($\boldsymbol{\tau}\in[-1,1]^3$) cho control allocator.

Đây là **vòng quan trọng nhất về độ ổn định**: tần số cao, đơn giản, "cứng". Hầu hết tunning UAV xoay quanh tầng này.

## 5.2. Code chính

| Vai trò | File |
|---|---|
| Wrapper uORB + lập lịch theo gyro | `@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.cpp` |
| Header tham số | `@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.hpp` |
| **Lõi thuật toán PID** | `@/home/frank/tf-px4/src/lib/rate_control/rate_control.cpp` |
| Header lib (cần đọc) | `@/home/frank/tf-px4/src/lib/rate_control/rate_control.hpp` |
| Test | `@/home/frank/tf-px4/src/lib/rate_control/rate_control_test.cpp` |

Hàm cần đọc:
- `MulticopterRateControl::Run()` — orchestrator (đăng ký callback gyro).
- `MulticopterRateControl::parameters_updated()` — map tham số `K*P/I/D` → ideal form.
- `RateControl::update(rate, rate_sp, angular_accel, dt, landed)` — luật PID chính.
- `RateControl::updateIntegral(rate_error, dt)` — anti-windup nonlinear.

## 5.3. Bảng ký hiệu

### Input / output

| Ký hiệu | Code | Nghĩa |
|---|---|---|
| $\boldsymbol{\omega}_{sp}=(p,q,r)_{sp}^\top$ | `_rates_setpoint` / `rate_sp` | rate setpoint từ attitude controller (rad/s, body) |
| $\boldsymbol{\omega}=(p,q,r)^\top$ | `rates` | gyro đo thực (rad/s, body) |
| $\dot{\boldsymbol{\omega}}$ | `angular_accel` | gia tốc góc (đạo hàm gyro filtered, từ sensor pipeline) |
| $\boldsymbol{\tau}\in[-1,1]^3$ | `torque` / output | torque setpoint normalized (cho mỗi trục body) |
| $\boldsymbol{T}\in\mathbb{R}^3$ | `_thrust_setpoint` | thrust setpoint normalized (body) |

### Trung gian

| Ký hiệu | Code | Nghĩa |
|---|---|---|
| $\boldsymbol{e}_\omega=\boldsymbol{\omega}_{sp}-\boldsymbol{\omega}$ | `rate_error` | rate error (rad/s) |
| $\boldsymbol{I}_\omega$ | `_rate_int` | integrator state |
| $i_{f,i}\in[0,1]$ | `i_factor` | hệ số nonlinear giảm I-gain khi error lớn |

### Gain & giới hạn

| Ký hiệu | Code / param | Nghĩa |
|---|---|---|
| $\boldsymbol{K}$ | `MC_{R,P,Y}RATE_K` | master gain ideal-form |
| $\boldsymbol{p},\boldsymbol{i},\boldsymbol{d},\boldsymbol{ff}$ | `MC_{R,P,Y}RATE_{P,I,D,FF}` | tham số parallel-form |
| $\boldsymbol{K}_p^\omega=\boldsymbol{K}\odot\boldsymbol{p}$ | `_gain_p` | P-gain rate (sau quy đổi) |
| $\boldsymbol{K}_i^\omega,\boldsymbol{K}_d^\omega,\boldsymbol{K}_{ff}^\omega$ | `_gain_{i,d,ff}` | I/D/FF gain rate |
| $I_{lim,i}$ | `_lim_int` / `MC_{R,P,Y}R_INT_LIM` | biên integrator |
| $\theta_{ref}=400\deg=6.98$ rad/s | hard-coded | scale của $i$-factor nonlinear |
| $f_c^{yaw}$ | `MC_YAW_TQ_CUTOFF` | LPF yaw torque |
| $s_{bat}=V_{nom}/V_{batt}$ | `_battery_status_scale` | scale bù áp pin |

### Cờ saturation từ allocator

| Ký hiệu | Code | Nghĩa |
|---|---|---|
| $s_i^+\in\{0,1\}$ | `_control_allocator_saturation_positive(i)` | trục $i$ saturate phía dương |
| $s_i^-\in\{0,1\}$ | `_control_allocator_saturation_negative(i)` | trục $i$ saturate phía âm |

---

## 5.4. Công thức chi tiết

### Bước 1 — Quy đổi tham số ideal form

Tham số `K*` cho phép biểu diễn dạng *parallel* (`P + I/s + sD`) ↔ *ideal* (`K(1+1/sTi+sTd)`):
$$
K_p^\omega = \boldsymbol{K}\odot\boldsymbol{p},\quad K_i^\omega=\boldsymbol{K}\odot\boldsymbol{i},\quad K_d^\omega=\boldsymbol{K}\odot\boldsymbol{d}
$$

**Giải thích biến**:
- $\boldsymbol{K}$ = `MC_{R,P,Y}RATE_K`: master gain ideal form ($K$ trong $K(1+1/sTi+sTd)$).
- $\boldsymbol{p},\boldsymbol{i},\boldsymbol{d}$ = `MC_{R,P,Y}RATE_{P,I,D}`: tỉ lệ P/I/D parallel form (người tune thường quen với dạng này).
- Tác dụng: tune chỉ $\boldsymbol{K}$ → scale toàn bộ P/I/D đồng đều, giữ nguyên bình đổng hưởng (PID dynamics không đổi, chỉ "speed up" hệ thống).

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

### Bước 2 — Luật PID + Feed-forward

$$
\boldsymbol{e}_\omega = \boldsymbol{\omega}_{sp}-\boldsymbol{\omega}
$$
$$
\boxed{\ \boldsymbol{\tau} = \boldsymbol{K}_p^\omega\odot\boldsymbol{e}_\omega + \boldsymbol{I}_\omega - \boldsymbol{K}_d^\omega\odot\dot{\boldsymbol{\omega}} + \boldsymbol{K}_{ff}^\omega\odot\boldsymbol{\omega}_{sp}\ }
$$

**Giải thích biến**:
- $\boldsymbol{e}_\omega$: rate error (rad/s).
- $\boldsymbol{I}_\omega$: integrator tích lũy, cập nhật qua `updateIntegral` (xem Bước 3).
- $\dot{\boldsymbol{\omega}}$: gyro derivative thực (KHÔNG phải $\dot{\boldsymbol{e}}_\omega$) → dấu **trừ** + lên thẳng measurement → **derivative on measurement**.
- $\boldsymbol{\omega}_{sp}$ trong FF-term đi thẳng vào output ("feed-forward through"), không qua P loop → tăng response speed cho maneuver mượt.

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

⚠️ Đặc điểm:
- D-term tác động lên $\dot{\boldsymbol{\omega}}$ trực tiếp (`angular_accel`, gyro derivative từ sensor pipeline §1.5.5) — chống "derivative kick" khi $\boldsymbol{\omega}_{sp}$ nhảy bậc.
- FF-term lên thẳng $\boldsymbol{\omega}_{sp}$ (`rate_sp`) — không bị qua P, giúp tracking nhanh các maneuver mượt.

### Bước 3 — Anti-Windup tích phân

Toàn bộ hàm `updateIntegral`:

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

Tách từng cơ chế:

#### (a) Saturation feedback từ Allocator

Nếu allocator đã bão hòa motor cho hướng dương:
$$
e_{\omega,i}\leftarrow\min(e_{\omega,i},0),\quad\text{hoặc }e_{\omega,i}\leftarrow\max(e_{\omega,i},0)\text{ cho hướng âm}
$$

**Giải thích biến**:
- $s_i^+ = 1$: trục $i$ đã saturate phía dương (motor đã max trong hướng tạo $\tau_i>0$).
- Bằng cách clip $e_{\omega,i}\le 0$, integrator chỉ được giảm theo hướng âm → không "đập mà" wind-up theo hướng đã bão hòa.
- Tương tự cho $s_i^- = 1$ ở phía âm.

Cờ này được set bởi `MulticopterRateControl::Run()` từ topic `control_allocator_status.unallocated_torque`:

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

#### (b) Nonlinear $i$-factor (giảm tích phân khi sai số lớn)

$$
i_{f,i} = \max\!\left(0,\ 1-\left(\frac{e_{\omega,i}}{\theta_{ref}}\right)^2\right),\quad \theta_{ref}=400°=6.98\,\text{rad/s}
$$

**Giải thích biến**:
- $i_{f,i}\in[0,1]$: hệ số nhân vào I-gain trước khi tích phân.
- $\theta_{ref}$ = ngưỡng error tại đó $i_f=0$ (hard-coded `math::radians(400.f)` $\approx 6.98$ rad/s).
- Dạng parabol $1-x^2$ → quan hệ "mềm": với $|e_\omega|<100°$, $i_f\approx 1$ (gần như không ảnh hưởng); với $|e_\omega|=200°$, $i_f\approx 0.75$; với $|e_\omega|\ge400°$, $i_f=0$.
- Mục đích: tránh "bounce-back" sau flip — khi error cực lớn, integrator không nên build up (phong cách "clamping" anti-windup bổ sung).

```@/home/frank/tf-px4/src/lib/rate_control/rate_control.cpp:107-108
float i_factor = rate_error(i) / math::radians(400.f);
i_factor = math::max(0.0f, 1.f - i_factor * i_factor);
```

#### (c) Tích phân Euler + clamp

$$
I_{\omega,i}\leftarrow\mathrm{clip}\!\left(I_{\omega,i}+i_{f,i}\,K_i^\omega\,e_{\omega,i}\Delta t,\ -I_{lim,i},\ I_{lim,i}\right)
$$

**Giải thích biến**:
- $i_{f,i}\,K_i^\omega\,e_{\omega,i}\Delta t$: lượng tích phân một bước Euler, có nhân nonlinear factor.
- $I_{lim,i}$ = `MC_{R,P,Y}R_INT_LIM`: biên clamp tuyệt đối — ngăn integrator chiếm hết ngân sách torque output.
- Khi `landed` hoặc disarmed → reset $\boldsymbol{I}_\omega=\mathbf{0}$ (ngoài `updateIntegral`, từ wrapper).

```@/home/frank/tf-px4/src/lib/rate_control/rate_control.cpp:111-116
float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

// do not propagate the result if out of range or invalid
if (PX4_ISFINITE(rate_i)) {
	_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
}
```

### Bước 4 — Yaw torque LPF
$$
\tau_z\leftarrow \mathrm{LPF}_{f_c=\text{MC\_YAW\_TQ\_CUTOFF}}(\tau_z)
$$

**Giải thích biến**:
- $\tau_z$: thành phần yaw của torque setpoint.
- $\mathrm{LPF}_{f_c}$: 1ˢᵗ-order alpha filter (xem §1.5.5) cắt ở $f_c$ = `MC_YAW_TQ_CUTOFF` Hz.
- Chỉ chạy lên trục yaw, không động vào roll/pitch.
- Mục đích: yaw torque được sinh nhờ sự chênh lệch RPM giữa CW/CCW motors (hệ số reaction-torque $c_m$ nhỏ, ~ 1/100 của thrust) → cần biên độ lớn → khuếch đại noise. LPF chống rung.

```@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.cpp:218-223
// run rate controller
Vector3f torque_setpoint =
	_rate_control.update(rates, _rates_setpoint, angular_accel, dt, _maybe_landed || _landed);

// apply low-pass filtering on yaw axis to reduce high frequency torque caused by rotor acceleration
torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);
```

### Bước 5 — Battery scaling (tùy chọn)

Khi `MC_BAT_SCALE_EN=1`:
$$
s_{bat}=\frac{V_{nom}}{V_{batt}},\quad
\boldsymbol{\tau}\leftarrow\mathrm{clip}(s_{bat}\boldsymbol{\tau},-1,1),\ \boldsymbol{T}\leftarrow\mathrm{clip}(s_{bat}\boldsymbol{T},-1,1)
$$

**Giải thích biến**:
- $V_{nom}$: áp pin danh định (vd. 16.8 V cho 4S full).
- $V_{batt}$: áp đo thực (từ `battery_status`).
- $s_{bat}\ge 1$: pin tuụt điện áp → cần PWM cao hơn để sinh cùng lực (vì lực motor tỉ lệ với $V^2$).
- Clip về $[-1,1]$ tránh tràn.
- Bù sụt áp pin → giữ phản hồi nhất quán suốt chuyến bay (gain ổn định không bị dịch theo SoC pin).

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

## 5.5. ACRO mode (manual không có attitude loop)

Khi `flag_control_manual_enabled && !flag_control_attitude_enabled`:

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
x_{shape}=\mathrm{superexpo}(x,e_{exp},s_{sup})=(1-e_{exp})x+e_{exp}x^3,\ \text{rồi}\ \frac{1-s_{sup}}{1-s_{sup}|x|}
$$
$$
\boldsymbol{\omega}_{sp} = \mathrm{shape}(\text{stick})\odot \boldsymbol{\omega}_{max}^{acro}
$$

**Giải thích biến**:
- $x\in[-1,1]$: stick value.
- $e_{exp}$ = `MC_ACRO_EXPO*`: độ cong của curve giữa stick và output (0 = linear, 1 = cực củng ở giữa).
- $s_{sup}$ = `MC_ACRO_SUPEXPO*`: super-expo factor — tăng độ nhạy ở cực đại.
- $\boldsymbol{\omega}_{max}^{acro}$ = `MC_ACRO_{R,P,Y}_MAX` (rad/s): rate tối đa khi stick = $\pm 1$.
- Mục đích: cho phi công có vùng "dễ control" ở trừ middle, nhưng vẫn đạt rate cao khi stick cùng cực.

## 5.6. Cơ sở lý thuyết & keywords

| Chủ đề | Keywords |
|---|---|
| PID parallel vs ideal | `parallel form vs ideal form PID`, `Astrom Hagglund PID textbook` |
| Anti-windup | `back-calculation`, `tracking anti-windup`, `conditional integration`, `clamping anti-windup`, `Aström Rundqwist 1989` |
| Saturation-aware control | `command-aware integrator`, `pilot-induced oscillation prevention` |
| Derivative kick | `derivative on measurement vs error`, `setpoint weighting PID` |
| Feed-forward in rate loop | `2-DOF rate controller`, `inverse model FF` |
| Body-rate aggressive flight | `Faessler Falanga Scaramuzza body rate control 2018` |
| Notch on D-term | `D-term filtering`, `Betaflight RPM filter`, `gyro feedback shaping` |
| Battery compensation | `voltage compensation ESC`, `thrust normalization`, `motor RPM model` |

### Bài báo / sách
- Astrom & Hagglund, *PID Controllers: Theory, Design, and Tuning* — chương 3 (anti-windup).
- Faessler, Falanga, Scaramuzza (2018) — *Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight*. RAL.
- Pounds, Mahony, Corke (2010) — *Modelling and Control of a Large Quadrotor Robot*.
- Furrer et al. — RotorS Gazebo paper (mô hình motor + ESC).

## 5.7. Tham số PX4

| Tham số | Ý nghĩa |
|---|---|
| `MC_ROLLRATE_K`, `MC_PITCHRATE_K`, `MC_YAWRATE_K` | Master gain (scale tất cả P/I/D theo trục) |
| `MC_*RATE_P`, `MC_*RATE_I`, `MC_*RATE_D`, `MC_*RATE_FF` | PID + FF gain |
| `MC_RR_INT_LIM`, `MC_PR_INT_LIM`, `MC_YR_INT_LIM` | Integrator limit |
| `MC_YAW_TQ_CUTOFF` | LPF yaw torque |
| `MC_BAT_SCALE_EN` | Bật battery scaling |
| `MC_ACRO_*_MAX`, `MC_ACRO_EXPO*`, `MC_ACRO_SUPEXPO*` | ACRO shaping |
| `IMU_DGYRO_CUTOFF` | LPF cho $\dot{\boldsymbol{\omega}}$ (D-term) |

## 5.8. Tip tuning
1. Set `K=1`, P/I/D theo defaults airframe.
2. Tăng `MC_*RATE_K` đến khi thấy oscillation cao tần (~> 30 Hz) → giảm 30%.
3. Tăng `MC_*RATE_I/MC_*RATE_K` đến khi steady-state error mất.
4. D-term tăng nếu overshoot, nhưng cẩn thận khuếch đại noise.
5. Log `rate_ctrl_status.{rollspeed_integ,...}` xem integrator có saturate không.
