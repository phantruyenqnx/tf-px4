# 7. Hover Thrust Estimator (Adaptive Scalar EKF)

## 7.1. Vai trò

Tham số $T_h$ — "hover thrust" = thrust normalized [0,1] đủ để hover (= bù trọng lực) — phụ thuộc khối lượng, tải, mật độ không khí, sức nâng motor (sụt áp pin). Position controller dùng $T_h$ ở §3 để chuyển gia tốc setpoint sang thrust setpoint:
$$
T_z^{NED}=a_{sp,z}\frac{T_h}{g}-T_h
$$
Sai $T_h$ → biased thrust → integrator của velocity controller phải bù → tracking dọc xấu, dao động khi cất cánh/đáp.

HTE là một **scalar EKF zero-order** ước lượng $T_h$ trực tuyến từ accel + thrust output.

## 7.2. Code chính

| Vai trò | File |
|---|---|
| Module wrapper | `@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/MulticopterHoverThrustEstimator.cpp` |
| **Lõi EKF scalar** | `@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp` |
| Header EKF | `@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.hpp` |
| Params | `hover_thrust_estimator_params.c` |
| Test | `zero_order_hover_thrust_ekf_test.cpp` |

Hàm cần đọc:
- `ZeroOrderHoverThrustEkf::predict(dt)` — đơn giản, chỉ tăng covariance.
- `ZeroOrderHoverThrustEkf::fuseAccZ(acc_z, thrust)` — toàn bộ một bước Kalman.
- `MulticopterHoverThrustEstimator::Run()` — orchestrator (đọc accel NED + thrust setpoint).

Consumer phía pos control: `PositionControl::updateHoverThrust(hover_thrust_new)` — bumpless update vào integrator (xem mục §7.4 dưới).

## 7.3. Bảng ký hiệu

### State + observation

| Ký hiệu | Code | Nghĩa |
|---|---|---|
| $T_h\in[T_{h,min},T_{h,max}]$ | `_hover_thr` | hover thrust normalized (state, scalar) |
| $u\in[0,1]$ | `thrust` (input) | thrust setpoint normalized hiện tại (cuối §3, đầu allocator) |
| $a_z^W$ | `acc_z` (input) | gia tốc dọc đo trong NED (từ EKF, đã trừ $g$) |
| $g=9.80665$ | `CONSTANTS_ONE_G` | trọng trường |

### Covariance + Kalman

| Ký hiệu | Code | Nghĩa |
|---|---|---|
| $P$ | `_state_var` | covariance state ($T_h$ variance) |
| $\sigma_p^2$ | `_process_var` / `HTE_HT_NOISE` | process noise (độ thay đổi $T_h$ trong 1s) |
| $R=\sigma_a^2 s_R$ | `_acc_var * _acc_var_scale` | measurement noise (adaptive) |
| $H=\partial h/\partial T_h$ | `computeH(thrust)` | Jacobian đo |
| $S=HPH+R$ | `innov_var` | innovation covariance |
| $K=PH/S$ | `K` | Kalman gain |
| $y=a_z^W-h(\hat T_h)$ | `innov` | innovation |
| $\gamma$ | `_gate_size` / `HTE_ACC_GATE` | gate factor (sigma) |
| $r=y^2/(\gamma^2 S)$ | `innov_test_ratio` | chi-square test ratio |

### Adaptive noise

| Ký hiệu | Code | Nghĩa |
|---|---|---|
| $\bar y$ | `_residual_lpf` | LPF của residual (loại bias) |
| $\bar r$ | `_signed_innov_test_ratio_lpf` | LPF của signed test ratio (để dò "reject lâu") |
| $\tau_{lpf}$ | `_lpf_time_constant` | time constant LPF residual |
| $\tau_{noise}$ | `_noise_learning_time_constant` | time constant học $\sigma_a^2$ |
| $\sigma_a^2$ | `_acc_var` | variance noise accel được học thích nghi |

---

## 7.4. Mô hình toán

### State
$x = T_h$ (scalar). Process: zero-order (giả định $T_h$ thay đổi rất chậm so với $\Delta t$).

### Predict

$$
\hat{T}_h^{(k+1)}=\hat{T}_h^{(k)},\qquad P_{k+1} = P_k + \sigma_p^2\Delta t^2
$$

**Giải thích biến**:
- State $T_h$ KHÔNG đổi ("zero-order" / "random walk"): EKF giả định hover thrust thay đổi chậm so với tần số update.
- Covariance phình ra theo thời gian × process noise: càng lâu không update, càng "thiếu tự tin".
- $\sigma_p^2$ = `HTE_HT_NOISE` ($\approx 0.001\sim 0.01$ /s²) — tăng nếu muốn EKF thích nghi nhanh (vd. drone drop payload nhiều).

```@/home/frank/tf-px4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp:44-50
void ZeroOrderHoverThrustEkf::predict(const float dt)
{
	// State is constant
	// Predict state covariance only
	_state_var += _process_var * dt * dt;
	_dt = dt;
}
```

### Measurement model

Quan hệ vật lý: cho thrust normalized hiện tại $u\in[0,1]$, gia tốc dọc đo được trong NED là:
$$
a_z^W = g\frac{u}{T_h} - g + \eta
$$
(tại hover $u=T_h\Rightarrow a_z=0$). Hàm đo:
$$
h(T_h) = g\frac{u}{T_h} - g
$$
Jacobian:
$$
H = \frac{\partial h}{\partial T_h} = -g\frac{u}{T_h^2}
$$

**Giải thích biến**:
- $u$: thrust setpoint hiện tại (đầu vào của allocator, normalized).
- Tại hover, motor sinh đúng $g$ → $u\equiv T_h$ → $a_z^W=0$ (không gia tốc).
- Khi thrust max ($u=1$), drone tăng tốc lên: $a_z^W = g(1/T_h-1)>0$ trong NED-ngức (note: NED z hướng xuống → tăng tốc lên = $a_z^W<0$; trong code thực PX4 có sửa dấu tại `MulticopterHoverThrustEstimator`).
- $\eta\sim\mathcal{N}(0,R)$: noise đo (rung cơ, sai tính frame).
- $H<0$: tăng $T_h$ (máy nặng hơn) → $a_z^W$ dự đoán giảm → EKF trừ $K\cdot y\cdot H$ vector → đuổi về đúng.

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

### Update (Kalman scalar)

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

**Giải thích biến**:
- $y$: innovation — sai giữa accel đo và dự đoán.
- $S$: variance của $y$ — "độ bất định" của innovation.
- $K$: gain — tiệm cận 0 khi $R\gg HPH$ (không tin sensor), tiệm cận $1/H$ khi $P\gg R$ (tin sensor).
- $\mathrm{clip}\,T_h\in[T_{h,min},T_{h,max}]$ = `[0.1, 0.9]` macđịnh — ngăn EKF đi ra vùng phi vật lý.
- $\mathrm{clip}\,P\in[10^{-10},1]$ — floor để EKF không "đóng băng" (P quyền lực $\to$ 0); ceil tránh phân kỳ.

Các hàm con tương ứng:

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

Toàn bộ một bước fuse ở `fuseAccZ`:

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

### Innovation gating

$$
r = \frac{y^2}{\gamma^2 S} \quad (\gamma=\text{HTE\_HT\_GATE})
$$

**Giải thích biến**:
- $r$: chi-square test ratio (bỏ $\gamma$ ra: $y^2/S\sim\chi^2_1$).
- $\gamma$ = `HTE_ACC_GATE` (mặc định 3 sigma).
- $r<1$ ⇔ $|y|<\gamma\sqrt{S}$ → innovation trong $\gamma$-sigma, chấp nhận update.
- $\bar r$ = LPF của $\mathrm{sign}(y)\cdot r$ — cho biết "reject phía nào" (nếu nhiều sample reject cùng hướng → EKF lệch sai bên).
- Khi $|\bar r|>0.2$ kéo dài: "bump" $P\mathrel{+}= 10^3\sigma_p^2\Delta t^2$ — tăng độ bất định → EKF "dám" đuổi $T_h$ nhanh → recover sau drop payload, cháy motor, ...

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

### Adaptive measurement noise

LPF residual rồi học $\sigma_a^2$:
$$
\bar y\leftarrow(1-\alpha)\bar y+\alpha y,\quad \alpha=\frac{\Delta t}{\tau_{lpf}+\Delta t}
$$
$$
\sigma_a^2\leftarrow\mathrm{clip}\!\left((1-\alpha')\sigma_a^2 + \alpha'((y-\bar y)^2+H P H),\ 1,\ 400\right)
$$

**Giải thích biến**:
- $\bar y$: trung bình chắt của residual qua LPF — ước lượng bias DC của accel (vd. tilt làm chiếu sai).
- $y-\bar y$: residual sau khi trừ bias — phần "thuần random".
- $(y-\bar y)^2 + HPH$: trong lý thuyết, $\mathbb{E}[(y-\bar y)^2]=\sigma_a^2 + HPH$, nên trừ $HPH$ để ra $\hat\sigma_a^2$. Nhưng code CỘNG (không trừ) vì nghĩa thực dụng: noise tổng (sensor + state uncertainty) — EKF "thụn trọng".
- $\alpha,\alpha'$: weights LPF, tỉ lệ với $\Delta t/\tau$. $\tau_{lpf}<\tau_{noise}$ → bias học nhanh, noise học chậm.
- Clip $[1,400]$ m²/s⁴: giữ trong vùng vật lý hợp lý.

→ tự động nới $R$ khi rung cao (rotor ồn) và siết lại khi máy bay yên.

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

## 7.5. Bumpless integration vào Position Controller

Khi $T_h$ thay đổi, nếu áp ngay vào công thức $T_z^{NED}=a_{sp,z}T_h/g - T_h$ thì thrust output sẽ nhảy, máy bay giật. Cần "đẩy" chênh lệch vào integrator velocity controller để output không đổi:

Suy luận: muốn $T'_z=T_z$ với $T_h\to T_h'$:
$$
a_{sp,z}'\frac{T_h'}{g}-T_h' = a_{sp,z}\frac{T_h}{g}-T_h
$$
$$
\Rightarrow a_{sp,z}' = (a_{sp,z}-g)\frac{T_h}{T_h'}+g
$$

Vì $a_{sp,z}=I_{v,z}+(\text{phần khác})$, đẩy chênh lệch $\Delta a = a_{sp,z}'-a_{sp,z}$ vào $I_{v,z}$:
$$
I_{v,z}\leftarrow I_{v,z}+(a_{sp,z}-g)\frac{T_h^{old}}{T_h^{new}}+g-a_{sp,z}
$$

**Giải thích biến**:
- $T_z$: thrust output cho allocator (giữ nguyên qua trước/sau update HTE).
- $a_{sp,z}'$: giá trị gia tốc mới cần có để thrust output không đổi với $T_h^{new}$.
- $a_{sp,z}=I_{v,z}+(\text{P-term, FF})$ — chỉ phần integrator tích lũy; P/FF sẽ tự động update ở chu kỳ sau.
- Bằng cách đẩy chênh lệch vào $I_{v,z}$, controller "quên" chính $T_h$ đã thay nhưng vẫn cho output thống nhất → "bumpless transfer" — máy bay không giật.

Đối chiếu code:

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

## 7.6. Khi nào HTE chạy / bị "freeze"

- Cần `vehicle_local_position.az_valid` và máy bay đang armed + bay (không landed).
- Cần thrust setpoint không bão hòa (gần `MPC_THR_MAX` hoặc `MPC_THR_MIN` → thông tin Jacobian kém).
- Cần $|\dot{\boldsymbol{\omega}}|$ và tilt thấp — accel Z không bị méo bởi gia tốc ngang khi quay.

Nếu không đủ điều kiện → `hover_thrust_estimate.valid=false`, position controller fallback về `MPC_THR_HOVER`.

## 7.7. Cơ sở lý thuyết & keywords

| Chủ đề | Keywords |
|---|---|
| Adaptive control | `recursive least squares RLS`, `Kalman filter for parameter identification`, `system identification online` |
| Online mass estimation | `online mass estimation UAV`, `quadcopter payload estimation`, `adaptive thrust mapping` |
| Innovation gating + bump | `chi-square test innovation`, `outlier rejection`, `covariance inflation` |
| Adaptive R / Q | `Sage-Husa adaptive Kalman`, `innovation-based adaptive estimation IAE`, `Mehra adaptive Kalman 1972` |
| Bumpless transfer | `bumpless transfer control`, `integrator preloading`, `controller switching` |
| ESC/motor model | `propeller momentum theory`, `thrust coefficient $C_T$`, `rotor disk model` |
| Sensor frame | `specific force vs acceleration`, `accelerometer dynamic model` |

### Bài báo / tài liệu
- Bresciani, Ferrari et al. — bài báo gốc HTE PX4 (xem comment `zero_order_hover_thrust_ekf.cpp`, tác giả Mathieu Bresciani).
- Mehra (1972) — *Approaches to Adaptive Filtering*. IEEE TAC.
- Aström & Wittenmark, *Adaptive Control* — chương RLS.
- Simon, *Optimal State Estimation* — chương adaptive Kalman.

## 7.8. Tham số PX4

| Tham số | Ý nghĩa |
|---|---|
| `MPC_USE_HTE` | Bật HTE (nếu off, dùng MPC_THR_HOVER cố định) |
| `HTE_HT_NOISE` | Process noise $\sigma_p$ (rate of change của $T_h$) |
| `HTE_ACC_GATE` | Gate innovation |
| `HTE_HT_ERR_INIT` | Std-dev khởi tạo của $T_h$ |
| `MPC_THR_HOVER` | Giá trị fallback khi HTE không valid |

## 7.9. Tip debug
- Log topic `hover_thrust_estimate`: kiểm tra `hover_thrust`, `hover_thrust_var`, `valid`, `accel_innov`.
- Nếu HTE không hội tụ: kiểm tra accel Z có nhiễu lớn không (rung mạnh), hoặc thrust setpoint thường xuyên saturate.
- Khi thay payload trên không (drop cargo), mong đợi spike trong `accel_innov` rồi bump variance recovery → `hover_thrust` hội tụ về giá trị mới sau ~5–10 s.
