# 4. Attitude Control (Quaternion P-controller, tilt-prioritized)

## 4.1. Vai trò

Vòng giữa của cascade. Nhận `vehicle_attitude_setpoint` ($\boldsymbol{q}_{sp}, \boldsymbol{T}^{body}, \dot{\psi}_{sp}$) từ position controller, cùng `vehicle_attitude` ($\boldsymbol{q}$) từ EKF2. Output là `vehicle_rates_setpoint` ($\boldsymbol{\omega}_{sp}, \boldsymbol{T}^{body}$) cho rate controller.

Triết lý: **chỉ là P-controller** (không I/D); việc bù bias do rate controller phía dưới làm. Nhưng tinh tế ở chỗ **tách yaw khỏi tilt** vì lực đẩy chỉ phụ thuộc tilt — sai yaw không nguy hiểm bằng sai tilt.

## 4.2. Code chính

| Vai trò | File |
|---|---|
| Wrapper uORB + manual stick | `@/home/frank/tf-px4/src/modules/mc_att_control/mc_att_control_main.cpp` |
| Header tham số | `@/home/frank/tf-px4/src/modules/mc_att_control/mc_att_control.hpp` |
| **Lõi thuật toán** | `@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp` |
| Helper toán (VTOL tilt correction) | `AttitudeControl/AttitudeControlMath.hpp` |
| Yaw stick handler | `@/home/frank/tf-px4/src/lib/stick_yaw/` |
| Test | `AttitudeControl/AttitudeControlTest.cpp` |

Hàm cần đọc:
- `MulticopterAttitudeControl::Run()` — orchestrator (line 206 trở đi).
- `MulticopterAttitudeControl::generate_attitude_setpoint(q, dt)` — sinh $\boldsymbol{q}_{sp}$ từ stick (Stabilized mode).
- `AttitudeControl::update(q)` — luật điều khiển chính.

## 4.3. Bảng ký hiệu

### Input / state

| Ký hiệu | Code | Nghĩa |
|---|---|---|
| $\boldsymbol{q}$ | `q` | quaternion tư thế hiện tại (body→world, Hamilton scalar-first) |
| $\boldsymbol{q}_{sp}$ | `_attitude_setpoint_q` / `qd` | quaternion setpoint từ pos control |
| $\dot{\psi}_{sp}$ | `_yawspeed_setpoint` | feed-forward yaw rate (rad/s, trong $\{W\}$) |
| $\boldsymbol{e}_z=\boldsymbol{R}(\boldsymbol{q})\hat{\boldsymbol{z}}_W$ | `e_z` (`q.dcm_z()`) | trục z body biểu diễn trong $\{W\}$ |
| $\boldsymbol{e}_z^{sp}=\boldsymbol{R}(\boldsymbol{q}_{sp})\hat{\boldsymbol{z}}_W$ | `e_z_d` | trục z body mong muốn trong $\{W\}$ |

### Trung gian

| Ký hiệu | Code | Nghĩa |
|---|---|---|
| $\boldsymbol{q}_{red}$ | `qd_red` | reduced desired attitude — chỉ lo tilt, bỏ qua yaw |
| $\boldsymbol{q}_{\delta\psi}$ | `qd_dyaw` | phần yaw còn lại $\boldsymbol{q}_{red}^{-1}\!\otimes\!\boldsymbol{q}_{sp}$ |
| $\boldsymbol{q}_d$ | `qd` (sau scale) | desired attitude lai (full tilt + weighted yaw) |
| $\boldsymbol{q}_e=\boldsymbol{q}^{-1}\!\otimes\!\boldsymbol{q}_d$ | `qe` | quaternion error |
| $\boldsymbol{e}_q=2\,\mathrm{Im}(\boldsymbol{q}_e^{canonical})$ | `eq` | rotation vector error (≈ $\alpha\hat{\boldsymbol{r}}$ với $\alpha$ nhỏ) |
| $\boldsymbol{\omega}_{sp}$ | `rate_setpoint` (output) | rate setpoint xuất cho rate controller |

### Gain & tham số

| Ký hiệu | Code / param | Nghĩa |
|---|---|---|
| $\boldsymbol{K}_p^{att}$ | `_proportional_gain` | (`MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P/`$w_\psi$) |
| $w_\psi\in[0,1]$ | `_yaw_w` / `MC_YAW_WEIGHT` | trọng số yaw vs tilt (mặc định 0.4) |
| $\omega_{max,i}$ | `_rate_limit` / `MC_*RATE_MAX` | giới hạn rate output |
| $\theta_{max}$ | `MPC_MAN_TILT_MAX` | tilt max khi ở manual |

### Toán tử

| Ký hiệu | Nghĩa |
|---|---|
| $\otimes$ | nhân quaternion Hamilton |
| $\mathrm{Im}(\boldsymbol{q})$ | phần vector $(q_x,q_y,q_z)$ |
| $\mathrm{canonical}(\boldsymbol{q})$ | chọn nghiệm có $q_w\ge 0$ (tránh antipodal) |
| `q.dcm_z()` | cột 3 của $\boldsymbol{R}(\boldsymbol{q})$ — chính là $\boldsymbol{R}\hat{\boldsymbol{z}}_W$ |

---

## 4.4. Công thức chi tiết

Toàn bộ luật điều khiển nằm trong hàm `update`:

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

Bên dưới tách nhỏ từng bước ánh xạ với công thức.

### Bước 1 — Reduced attitude (chỉ điều chỉnh tilt)

Trục Z body hiện tại và mong muốn (cùng biểu diễn trong $\{W\}$):
$$
\boldsymbol{e}_z = \boldsymbol{R}(\boldsymbol{q})\hat{\boldsymbol{z}}_W,\qquad
\boldsymbol{e}_z^{sp}=\boldsymbol{R}(\boldsymbol{q}_{sp})\hat{\boldsymbol{z}}_W
$$

Quaternion quay tối thiểu giữa hai trục:
$$
\boldsymbol{q}_{red}^{(W)} = \mathrm{quat\_from\_two\_vectors}(\boldsymbol{e}_z,\boldsymbol{e}_z^{sp})
$$

Đây là rotation trong world frame; right-multiply với $\boldsymbol{q}$:
$$
\boldsymbol{q}_{red}=\boldsymbol{q}_{red}^{(W)}\otimes\boldsymbol{q}
$$

**Giải thích biến**:
- $\hat{\boldsymbol{z}}_W=(0,0,1)^\top$ (NED, hướng xuống).
- $\boldsymbol{e}_z,\boldsymbol{e}_z^{sp}$: cho biết trục dọc body đang chỉ đâu so với mong muốn. Nếu chúng trùng nhau → tilt OK.
- $\boldsymbol{q}_{red}^{(W)}$: rotation "ít" nhất (axis vuông góc cả hai vector) đem $\boldsymbol{e}_z\to\boldsymbol{e}_z^{sp}$ — chỉ quay tilt, không đổi yaw.
- Right-multiply với $\boldsymbol{q}$: ghép rotation tilt (trong $\{W\}$) với attitude hiện tại → ra reduced desired attitude $\boldsymbol{q}_{red}$ (yaw của nó = yaw của $\boldsymbol{q}$ — tức "giữ nguyên yaw hiện tại").

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
$\boldsymbol{q}_{red}$ = "desired attitude nếu chỉ quan tâm tilt" — đem trục Z body trùng setpoint, để yaw tự do.

**Edge case**: khi $\boldsymbol{e}_z\approx-\boldsymbol{e}_z^{sp}$ (180° lệch trục), code chuyển sang `q_red = q_sp` để tránh kỳ dị (singular axis).

### Bước 2 — Phần yaw chênh lệch + scale theo $w_\psi$

$$
\boldsymbol{q}_{\delta\psi} = \boldsymbol{q}_{red}^{-1}\otimes\boldsymbol{q}_{sp}
$$

Theo định lý phân rã, $\boldsymbol{q}_{\delta\psi}$ chỉ chứa yaw quanh $\hat{\boldsymbol{z}}_B$ → có dạng $(\cos(\alpha/2),0,0,\sin(\alpha/2))$.

Áp trọng số $w_\psi$ = `MC_YAW_WEIGHT` (mặc định 0.4 — đặt < 1 để khi yaw lệch nhiều, tilt vẫn được ưu tiên):
$$
\boldsymbol{q}_{\delta\psi}^{(w)} = \big(\cos(w_\psi\arccos q_{\delta\psi,w}),\ 0,\ 0,\ \sin(w_\psi\arcsin q_{\delta\psi,z})\big)
$$

Desired attitude lai (mix tilt full priority + yaw weighted):
$$
\boldsymbol{q}_d = \boldsymbol{q}_{red}\otimes\boldsymbol{q}_{\delta\psi}^{(w)}
$$

**Giải thích biến**:
- $q_{\delta\psi,w}=\cos(\alpha/2)$, $q_{\delta\psi,z}=\sin(\alpha/2)$ với $\alpha$ = góc yaw lech.
- $\arccos q_{\delta\psi,w}=\alpha/2$ → nhân $w_\psi$ → $\cos(w_\psi\alpha/2)$ = scale yaw error về phần $w_\psi\alpha$ ("đi chậm hơn" so với tilt).
- Việc dùng `acosf(qd_dyaw(0))` và `asinf(qd_dyaw(3))` riêng nhau là để bền số quanh $\alpha\approx\pi$ (acos mất độ chính xác ở $\pm 1$, asin mất độ chính xác ở 0).
- $\boldsymbol{q}_d$: "dịch" từ $\boldsymbol{q}_{red}$ thêm phần yaw scaled → ưu tiên tilt (full gain) hơn yaw (gain * $w_\psi$).

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

### Bước 3 — Quaternion error → angular rate setpoint

$$
\boldsymbol{q}_e = \boldsymbol{q}^{-1}\otimes\boldsymbol{q}_d,\quad \boldsymbol{q}_e\leftarrow\mathrm{sign}(q_{e,w})\boldsymbol{q}_e\quad(\text{canonicalize})
$$

Định lý: với rotation nhỏ, $\mathrm{Im}(\boldsymbol{q}_e)=\sin(\alpha/2)\hat{\boldsymbol{r}}\approx(\alpha/2)\hat{\boldsymbol{r}}$. Luật P:
$$
\boxed{\ \boldsymbol{\omega}_{sp} = 2\,\boldsymbol{K}_p^{att}\odot\mathrm{Im}(\boldsymbol{q}_e)\ }
$$

**Giải thích biến**:
- $\boldsymbol{q}_e$ trong **body frame** (vì $\boldsymbol{q}^{-1}\otimes\boldsymbol{q}_d$) → $\mathrm{Im}(\boldsymbol{q}_e)$ trực tiếp dùng để cấp rate trong body frame.
- `canonicalize`: đảo dấu toàn bộ quaternion nếu $q_{e,w}<0$ — vì $\boldsymbol{q}$ và $-\boldsymbol{q}$ biểu diễn cùng rotation, chọn nghiệm "qua đường ngắn".
- Hệ số $2$ đến từ $\sin(\alpha/2)\to\alpha/2$: nhân 2 để ra rotation vector $\alpha\hat{\boldsymbol{r}}$ (đơn vị radian).
- $\boldsymbol{K}_p^{att}=(K_p^\phi, K_p^\theta, K_p^\psi/w_\psi)$: yaw gain được *chia ngược* cho $w_\psi$ để kết quả cuối có rate response tương đương dù đã scale yaw error ở Bước 2 (bù ngược).

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

Lưu ý: việc chia ngược yaw gain cho $w_\psi$ xảy ra trong `setProportionalGain`:

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

### Bước 4 — Feed-forward yaw rate

$\dot{\psi}_{sp}$ là tốc độ quay quanh $\hat{\boldsymbol{z}}_W$. Để cộng vào $\boldsymbol{\omega}_{sp}$ (body frame), chiếu trục $\hat{\boldsymbol{z}}_W$ về body:
$$
\boldsymbol{\omega}_{sp}\mathrel{+}= \boldsymbol{R}^\top(\boldsymbol{q})\hat{\boldsymbol{z}}_W\,\dot{\psi}_{sp}
$$

**Giải thích biến**:
- $\boldsymbol{R}(\boldsymbol{q})=\boldsymbol{R}_{WB}$ → $\boldsymbol{R}^\top=\boldsymbol{R}_{BW}$ chuyển vector từ $\{W\}$ về $\{B\}$.
- $\boldsymbol{R}^\top\hat{\boldsymbol{z}}_W$ = cột 3 của $\boldsymbol{R}^\top$ = $\hat{\boldsymbol{z}}_W$ biểu diễn trong body. Khi tilt = 0: $=\hat{\boldsymbol{z}}_B=(0,0,1)$ → yaw rate chỉ cộng vào trục r body. Khi tilt: thay cả ba thành phần.
- Công thức đảm bảo yaw FF đúng nghĩa *world-frame yaw rate*, không bị sai khi tilt cao (khắc phục gimbal lock của Euler-rate FF).

Code:

```@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:104-106
if (std::isfinite(_yawspeed_setpoint)) {
	rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
}
```

### Bước 5 — Giới hạn rate
$$
\omega_{sp,i}\leftarrow\mathrm{clip}(\omega_{sp,i},-\omega_{max,i},\omega_{max,i})
$$
- $\omega_{max,i}$: giới hạn rate từng trục (`MC_{ROLL,PITCH,YAW}RATE_MAX`, rad/s).
- Bảo vệ rate controller khỏi cấp tốc độ góc vô lý (vd. khi error attitude > 180°).

```@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:108-111
// limit rates
for (int i = 0; i < 3; i++) {
	rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
}
```

## 4.5. Generate attitude setpoint từ stick (Stabilized mode)

Khi không có pos control, tự sinh $\boldsymbol{q}_{sp}$ từ RC. Xem `mc_att_control_main.cpp:136-203`.

Tilt từ stick:
$$
\boldsymbol{v}=(roll\cdot\theta_{max},\ -pitch\cdot\theta_{max})\quad\text{sau khi qua filter time-constant }\tau_{tilt}
$$
$$
\|\boldsymbol{v}\|>\theta_{max}\Rightarrow \boldsymbol{v}\leftarrow\boldsymbol{v}\frac{\theta_{max}}{\|\boldsymbol{v}\|}
$$

**Giải thích biến**:
- $roll, pitch\in[-1,1]$: stick value (đã expo + deadzone).
- $\theta_{max}$ = `MPC_MAN_TILT_MAX` (rad).
- $\boldsymbol{v}=(v_x,v_y)$: rotation vector tilt 2D — độ dài = góc tilt, hướng = trục quay tilt trong $\{W\}$ ngang.
- Dấu trừ trên pitch vì stick pitch dương = nghề mũi xuống = quay quanh $-\hat{\boldsymbol{e}}_y$.
- $\tau_{tilt}$ = `MC_MAN_TILT_TAU` — lowpass filter tránh shock tilt khi stick giật.

Quaternion roll-pitch dưới dạng axis-angle:
$$
\boldsymbol{q}_{rp} = \mathrm{AxisAngle}(v_x,v_y,0)
$$
- $\mathrm{AxisAngle}(\boldsymbol{u})=(\cos(\|\boldsymbol{u}\|/2),\ \mathrm{sinc}(\|\boldsymbol{u}\|/2)\boldsymbol{u}/2)$ — không có thành phần yaw vì $u_z=0$.

Yaw từ stick (xem `StickYaw` lib): integrate yaw stick với expo + deadzone:
$$
\boldsymbol{q}_{yaw}=(\cos(\psi_{sp}/2),0,0,\sin(\psi_{sp}/2))
$$
- $\psi_{sp}$ được tích phân từ yaw stick rate × $\Delta t$ → “hold last yaw” khi stick = 0.

Setpoint cuối:
$$
\boldsymbol{q}_{sp}=\boldsymbol{q}_{yaw}\otimes\boldsymbol{q}_{rp}
$$
- Thứ tự: yaw trước (lên $\hat{\boldsymbol{z}}_W$), rồi tilt — tương đương quay trong frame $\{W\}$ trung gian đã yaw.

Throttle stick → `thrust_body[2] = -throttle_curve(stick)` với `MPC_THR_CURVE` chọn dạng curve (linear/HTE-rescaled).

## 4.6. EKF reset handling

Khi EKF reset yaw (`quat_reset_counter` tăng), nhận `delta_q_reset`:
$$
\psi_{sp,stab}\leftarrow\mathrm{wrap}_\pi(\psi_{sp,stab}+\delta\psi)
$$
$$
\boldsymbol{q}_{sp}\leftarrow \delta\boldsymbol{q}_{reset}\otimes\boldsymbol{q}_{sp}
$$
→ tránh giật khi EKF nhảy yaw.

## 4.7. Cơ sở lý thuyết & keywords

| Chủ đề | Keywords |
|---|---|
| Quaternion attitude control | `Brescianini Hehn D'Andrea ETH 2013 nonlinear quadrocopter attitude control`, `quaternion P-controller` |
| Tilt-prioritized | `tilt prioritization`, `reduced attitude control`, `attitude decomposition tilt-yaw` |
| Geometric SO(3) control | `Lee SE(3) geometric control`, `Bullo Lewis geometric mechanics`, `Mahony Hua Hamel SO(3) control` |
| Quaternion math | `Hamilton convention`, `right vs left composition`, `axis-angle exponential`, `slerp` |
| Singularity avoidance | `gimbal lock`, `quaternion antipodal`, `canonicalization` |

### Bài báo nền tảng
- Brescianini, Hehn, D'Andrea (2013) — *Nonlinear Quadrocopter Attitude Control*. ETH tech report — **chính xác là thuật toán PX4 đang dùng**. Link gốc trong comment đầu file `mc_att_control_main.cpp`.
- Mahony, Hua, Hamel (2011) — *Multirotor Aerial Vehicles: Modeling, Estimation, and Control*. IEEE RAM.
- Fresk & Nikolakopoulos (2013) — *Full Quaternion Based Attitude Control for a Quadrotor*.

### Tài liệu mở rộng
- Markley & Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control* — chương quaternion.
- Sola tutorial (đã nhắc ở §2) cho phép phân rã quaternion.

## 4.8. Tham số PX4

| Tham số | Ý nghĩa |
|---|---|
| `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P` | Attitude P gains |
| `MC_YAW_WEIGHT` | Trọng số yaw vs tilt (mặc định 0.4) |
| `MC_ROLLRATE_MAX`, `MC_PITCHRATE_MAX`, `MC_YAWRATE_MAX` | Giới hạn rate output |
| `MPC_MAN_TILT_MAX`, `MC_MAN_TILT_TAU` | Tilt max + filter constant cho stick |
| `MPC_THR_CURVE`, `MPC_THR_HOVER` | Throttle curve manual |

## 4.9. Tip debug
- Log: `vehicle_attitude_setpoint.q_d` vs `vehicle_attitude.q` → tính error angle bằng $2\arccos|q_e\cdot q|$.
- Nếu yaw chậm: tăng `MC_YAW_P` hoặc giảm `MC_YAW_WEIGHT` (paradox — weight nhỏ tiêu thụ ít trục yaw nhưng response chậm).
- Nếu drone wobble lúc tilt cao: thường do tilt-priority hoạt động → bình thường, KHÔNG tăng gain att.
