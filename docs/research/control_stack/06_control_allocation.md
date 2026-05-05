# 6. Control Allocation (Mixer toán học)

## 6.1. Vai trò

Chuyển vector lệnh 6-trục $\boldsymbol{c}=[\boldsymbol{\tau};\boldsymbol{T}]\in\mathbb{R}^6$ (3 torque + 3 thrust component) → tín hiệu $n_m$ motor $\boldsymbol{u}\in[0,1]^{n_m}$. Đây là **mixer thế hệ mới** của PX4 thay cho `mixer_module` cũ — tách biệt thuật toán phân phối khỏi cấu hình hình học.

## 6.2. Code chính

| Vai trò | File |
|---|---|
| Module wrapper | `@/home/frank/tf-px4/src/modules/control_allocator/ControlAllocator.cpp` |
| Header | `@/home/frank/tf-px4/src/modules/control_allocator/ControlAllocator.hpp` |
| Effectiveness theo airframe | `@/home/frank/tf-px4/src/modules/control_allocator/VehicleActuatorEffectiveness/` |
| Base allocation | `@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocation.cpp` |
| **Pseudo-inverse** | `@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp` |
| **Sequential desaturation** | `@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp` |
| Effectiveness lib | `@/home/frank/tf-px4/src/lib/control_allocation/actuator_effectiveness/` |
| Generic inverse (Greville/Moore-Penrose) | `@/home/frank/tf-px4/src/lib/matrix/matrix/PseudoInverse.hpp` (`matrix::geninv`) |

## 6.3. Bảng ký hiệu

### Biến chính

| Ký hiệu | Code | Nghĩa |
|---|---|---|
| $\boldsymbol{c}=[\boldsymbol{\tau};\boldsymbol{T}]\in\mathbb{R}^6$ | `_control_sp` | lệnh đầu vào (3 torque + 3 thrust comp.), normalized $\in[-1,1]$ |
| $\boldsymbol{u}\in\mathbb{R}^{n_m}$ | `_actuator_sp` | output mỗi actuator $\in[u_{min},u_{max}]$ |
| $n_m$ | `_num_actuators` | số motor/control surface |
| $\boldsymbol{B}\in\mathbb{R}^{6\times n_m}$ | `_effectiveness` | effectiveness matrix (sinh từ hình học airframe) |
| $\boldsymbol{B}^+\in\mathbb{R}^{n_m\times 6}$ | `_mix` | pseudo-inverse đã chuẩn hóa |
| $\boldsymbol{u}_{trim}$ | `_actuator_trim` | working point (vd. `0.5` cho hover quad) |
| $\boldsymbol{c}_{trim}$ | `_control_trim` | lệnh tương ứng với $\boldsymbol{u}_{trim}$ |

### Tham số mô hình actuator

| Ký hiệu | Nghĩa |
|---|---|
| $r$ | bán kính cánh tay đòn motor (m) |
| $\theta_i$ | góc vị trí motor $i$ trong mặt phẳng x-y body |
| $c_m$ | hệ số mo-men cản (yaw / thrust ratio, $\sim 0.01$) |
| $\sigma_i\in\{+1,-1\}$ | chiều quay rotor (CW=$-$, CCW=$+$) |

### Saturation handling

| Ký hiệu | Code | Nghĩa |
|---|---|---|
| $\boldsymbol{d}\in\mathbb{R}^{n_m}$ | `desaturation_vector` | hướng "dịch chuyển" trong nullspace của $\boldsymbol{B}$ (không đổi $\boldsymbol{c}$) |
| $k$ | `gain` | gain SD tìm được để đẩy $\boldsymbol{u}$ về feasible |
| $u_{min,i},u_{max,i}$ | `_actuator_min/max` | biên motor $i$ |
| $\boldsymbol{c}_{achieved}=\boldsymbol{B}\boldsymbol{u}_{clipped}$ | derived | lệnh thực sự sinh được sau khi clip |
| $\boldsymbol{c}_{unalloc}=\boldsymbol{c}-\boldsymbol{c}_{achieved}$ | `unallocated_torque/thrust` | lệnh KHÔNG sinh được, feedback về rate controller (§5) |

### Toán tử

| Ký hiệu | Nghĩa |
|---|---|
| $\boldsymbol{B}^+$ | Moore-Penrose pseudo-inverse |
| $\mathrm{geninv}$ | Greville recursive PI (handle rank-deficient) |
| $\mathrm{Null}(\boldsymbol{B})$ | nullspace = $\{\boldsymbol{u}:\boldsymbol{B}\boldsymbol{u}=\boldsymbol{0}\}$ |

---

## 6.4. Bài toán toán học

Vector lệnh chuẩn hóa:
$$
\boldsymbol{c}=\begin{bmatrix}\tau_x\\ \tau_y\\ \tau_z\\ T_x\\ T_y\\ T_z\end{bmatrix}\in[-1,1]^6
$$

Output mỗi motor $\boldsymbol{u}\in[u_{min},u_{max}]^{n_m}$ (mặc định $[0,1]$).

Mô hình actuator tuyến tính:
$$
\boldsymbol{c} = \boldsymbol{B}\boldsymbol{u}
$$
$\boldsymbol{B}\in\mathbb{R}^{6\times n_m}$ là **effectiveness matrix** sinh từ hình học airframe.

**Giải thích biến**:
- $\boldsymbol{c}=(\tau_x,\tau_y,\tau_z,T_x,T_y,T_z)^\top$: lệnh 6-DoF đã normalized về $[-1,1]$.
- $\boldsymbol{u}=(u_0,\dots,u_{n_m-1})^\top$: giá trị motor (PWM-equivalent), mặc định $\in[0,1]$.
- $B_{ji}$: trục $j$ (trong 6 trục) thay đổi bao nhiêu khi motor $i$ tăng 1 đơn vị — đánh giá "ai ảnh hưởng đến ai".

### Ví dụ: Quadcopter X (motor đánh số 0..3 ở 4 góc)

Cho:
- Vị trí motor $i$: $(r\cos\theta_i, r\sin\theta_i, 0)$ trong body, $\theta_i\in\{45°,135°,225°,315°\}$.
- Hướng đẩy: $-\hat{\boldsymbol{z}}_B$.
- Mô-men cản (yaw): hệ số $c_m$, dấu CW/CCW xen kẽ.

Cột $i$ của $\boldsymbol{B}$:
$$
\boldsymbol{B}_{:,i} = \begin{bmatrix}
-r\sin\theta_i & \text{(roll)}\\
\phantom{-}r\cos\theta_i & \text{(pitch)}\\
\sigma_i\,c_m & \text{(yaw, }\sigma_i=\pm1\text{)}\\
0 & \text{(T_x)}\\
0 & \text{(T_y)}\\
-1 & \text{(T_z, đẩy lên)}
\end{bmatrix}
$$

**Giải thích từng hàng**:
- Hàng 1 (roll = $\tau_x$): motor càng xa trục x (lớn $|\sin\theta_i|$) càng tạo roll torque, dấu theo $-\sin\theta_i$ vì thrust hướng $-\hat{z}_B$ × cánh tay $\hat{y}\sin\theta_i$.
- Hàng 2 (pitch): tương tự với trục y, dấu $+\cos\theta_i$.
- Hàng 3 (yaw = $\tau_z$): mo-men phản lực không khí từ rotor, dấu theo chiều quay $\sigma_i$, độ lớn = $c_m$ (~ 1/100 của thrust).
- Hàng 4-5 (T_x, T_y): = 0 vì quad chỉ đẩy theo $\hat{z}_B$, không có lateral thrust (khác với tilt-rotor).
- Hàng 6 (T_z): = $-1$ — thrust theo $-\hat{z}_B$ ($T_z$ âm trong NED-like body z hướng xuống).

$\boldsymbol{B}$ được tạo ở `ActuatorEffectivenessRotors.cpp`.

## 6.5. Pseudo-inverse Moore–Penrose

Khi $n_m\ge 6$ và $\boldsymbol{B}$ rank đầy:
$$
\boldsymbol{B}^+ = \boldsymbol{B}^\top(\boldsymbol{B}\boldsymbol{B}^\top)^{-1}
$$

**Giải thích biến**:
- $\boldsymbol{B}\boldsymbol{B}^\top\in\mathbb{R}^{6\times 6}$: invertible khi $\boldsymbol{B}$ full row-rank (đủ motor để control 6 trục).
- $\boldsymbol{B}^+$: ma trận "đảo" sao cho $\boldsymbol{B}\boldsymbol{B}^+ = \boldsymbol{I}_6$ (trái chỉ khi over-actuated).
- Dạng này áp dụng cho hex/octo (≥ 6 motor).

Với $n_m<6$ (vd. quad có 4 motor < 6 trục), code dùng `matrix::geninv` — Greville's recursive Moore-Penrose, hoạt động cho cả rank thiếu.

```@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp:61-78
void
ControlAllocationPseudoInverse::updatePseudoInverse()
{
	if (_mix_update_needed) {
		matrix::geninv(_effectiveness, _mix);

		if (!_metric_allocation) {
			if (_normalization_needs_update && !_had_actuator_failure) {
				updateControlAllocationMatrixScale();
				_normalization_needs_update = false;
			}

			normalizeControlAllocationMatrix();
		}

		_mix_update_needed = false;
	}
}
```

Luật phân phối:
$$
\boxed{\ \boldsymbol{u} = \boldsymbol{u}_{trim} + \boldsymbol{B}^+(\boldsymbol{c}-\boldsymbol{c}_{trim})\ }
$$

**Giải thích biến**:
- $\boldsymbol{u}_{trim}$: working point — cho quad hover, tương đương hệ số hover thrust (~ 0.5 cho default).
- $\boldsymbol{c}_{trim}$: lệnh sinh ra tại $\boldsymbol{u}_{trim}$ (giá trị $T_z=-T_h$, các cái khác = 0).
- $\boldsymbol{c}-\boldsymbol{c}_{trim}$: "lệnh dịch chuyển" so với trim → nhân $\boldsymbol{B}^+$ để ra lượng motor cần thay đổi.
- Tổng cộng với $\boldsymbol{u}_{trim}$ → PWM cuối.
- Lợi ích trim: tuyến tính hóa quanh hover → phần dịch nhỏ, gain nằm đúng độ nhạy ESC.

Đối chiếu code (`allocate()`):

```@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp:179-189
void
ControlAllocationPseudoInverse::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	// Allocate
	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
}
```

`_mix` = $\boldsymbol{B}^+$ đã chuẩn hóa.

### Tính chất Moore-Penrose
$\boldsymbol{u}=\boldsymbol{B}^+\boldsymbol{c}$ là nghiệm **năng lượng tối thiểu** (chuẩn $\ell_2$) trong không gian:
$$
\min_\boldsymbol{u}\|\boldsymbol{u}\|^2\quad\text{s.t.}\quad \boldsymbol{B}\boldsymbol{u}=\boldsymbol{c}
$$

- Nếu under-actuated ($n_m\ge 6$, $\boldsymbol{B}$ full row-rank): ​vô số nghiệm, PI chọn cái có $\|\boldsymbol{u}\|$ nhỏ nhất (ít sử dụng motor effort).
- Khi quá xác định ($n_m<6$, rank<6) → KHÔNG có nghiệm chính xác, PI cho **least-squares** approximation $\min\|\boldsymbol{B}\boldsymbol{u}-\boldsymbol{c}\|^2$ — giả như "theo đuổi sát nhất" lệnh, phần lệch được đánh dấu $\boldsymbol{c}_{unalloc}$.

## 6.6. Chuẩn hóa cột $\boldsymbol{B}^+$

Mục đích: cùng giá trị $\tau_x=1$ phải tạo cùng "tổng độ lệch motor" bất kể số motor (4-quad vs 6-hex).

- Roll/Pitch (cùng scale): $\sqrt{\|\boldsymbol{B}^+_{:,0}\|^2/(n_{nz}/2)}$.
- Yaw: $\max_i|B^+_{i,2}|$.
- Thrust mỗi trục: $\frac{1}{n_{nz}}\sum_i|B^+_{i,3+axis}|$.

**Giải thích biến**:
- $\boldsymbol{B}^+_{:,j}$: cột $j$ của $\boldsymbol{B}^+$ — cho thấy lệnh trục $j$ (vd. roll) phân phối đến từng motor thế nào.
- $n_{nz}$: số motor có effectiveness khác 0 cho trục đó.
- Roll/pitch được dùng RMS-norm để chuẩn hóa (cho đẹp đối xứng); yaw dùng max-norm vì $c_m$ nhỏ cần biên lớn.
- Thrust dùng mean-norm để tổng motor command luôn có độ lớn bằng với $T_z$.

Sau đó zero-out các phần tử $|B^+_{ij}|<10^{-3}$ để tránh nhiễu số.

```@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp:100-122
float roll_norm_scale = 1.f;

if (num_non_zero_roll_torque > 0) {
	roll_norm_scale = sqrtf(_mix.col(0).norm_squared() / (num_non_zero_roll_torque / 2.f));
}

float pitch_norm_scale = 1.f;

if (num_non_zero_pitch_torque > 0) {
	pitch_norm_scale = sqrtf(_mix.col(1).norm_squared() / (num_non_zero_pitch_torque / 2.f));
}

_control_allocation_scale(0) = fmaxf(roll_norm_scale, pitch_norm_scale);
_control_allocation_scale(1) = _control_allocation_scale(0);

// Scale yaw separately
_control_allocation_scale(2) = _mix.col(2).max();
```

```@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp:168-176
// Set all the small elements to 0 to avoid issues
// in the control allocation algorithms
for (int i = 0; i < _num_actuators; i++) {
	for (int j = 0; j < NUM_AXES; j++) {
		if (fabsf(_mix(i, j)) < 1e-3f) {
			_mix(i, j) = 0.f;
		}
	}
}
```

## 6.7. Sequential Desaturation (SD) — quad/hex bị thiếu DoF

File: `ControlAllocationSequentialDesaturation.cpp`. Ý tưởng: thêm vector trong nullspace của $\boldsymbol{B}$ vào $\boldsymbol{u}$ (không thay đổi $\boldsymbol{c}=\boldsymbol{B}\boldsymbol{u}$) sao cho $\boldsymbol{u}$ về vùng feasible $[u_{min},u_{max}]$. Khi không thể, **hi sinh** trục theo thứ tự ưu tiên.

### Hàm `computeDesaturationGain`

Cho desaturation vector $\boldsymbol{d}$ (thường là cột thrust, hoặc cột nullspace), tìm gain $k$:

Với mỗi motor saturate $u_i<u_{min,i}$:
$$
k_i = \frac{u_{min,i}-u_i}{d_i}
$$
Tương tự cho $u_i>u_{max,i}$. Lấy:
$$
k = k_{min}+k_{max}\quad(\text{giảm tổng saturation}).
$$

**Giải thích biến**:
- $\boldsymbol{d}$: vector chỉ hướng "đẩy" $\boldsymbol{u}$ — nếu $\boldsymbol{d}\in\mathrm{Null}(\boldsymbol{B})$ thì $\boldsymbol{c}$ không đổi ("free move"), nếu không (vd. cột thrust) thì hy sinh trục $T_z$ để cứu tilt.
- $k_i$: gain cần thiết để đưa motor $i$ về đúng biên (`min` hoặc `max`).
- $k_{min},k_{max}$: gain âm nhỏ nhất và dương lớn nhất qua mọi motor — trung hòa hai chiều (nếu một motor saturate cao còn cái khác saturate thấp, trù trung bình).
- Bỏ qua actuator có $|d_i|<0.2$ (effectiveness yếu) — tránh chia xấp xỉ 0 gây gain đột biến.

Áp:
$$
\boldsymbol{u}\leftarrow\boldsymbol{u}+k\boldsymbol{d}
$$
Lặp 1 lần với $k\leftarrow 0.5\,k_{new}$ để hội tụ ổn định ("two-step bisection").

```@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp:67-86
void ControlAllocationSequentialDesaturation::desaturateActuators(
	ActuatorVector &actuator_sp,
	const ActuatorVector &desaturation_vector, bool increase_only)
{
	float gain = computeDesaturationGain(desaturation_vector, actuator_sp);

	if (increase_only && gain < 0.f) {
		return;
	}

	for (int i = 0; i < _num_actuators; i++) {
		actuator_sp(i) += gain * desaturation_vector(i);
	}

	gain = 0.5f * computeDesaturationGain(desaturation_vector, actuator_sp);

	for (int i = 0; i < _num_actuators; i++) {
		actuator_sp(i) += gain * desaturation_vector(i);
	}
}
```

```@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp:88-119
float ControlAllocationSequentialDesaturation::computeDesaturationGain(const ActuatorVector &desaturation_vector,
		const ActuatorVector &actuator_sp)
{
	float k_min = 0.f;
	float k_max = 0.f;

	for (int i = 0; i < _num_actuators; i++) {
		// Do not use try to desaturate using an actuator with weak effectiveness to avoid large desaturation gains
		if (fabsf(desaturation_vector(i)) < 0.2f) {
			continue;
		}

		if (actuator_sp(i) < _actuator_min(i)) {
			float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);

			if (k < k_min) { k_min = k; }

			if (k > k_max) { k_max = k; }
		}

		if (actuator_sp(i) > _actuator_max(i)) {
			float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);

			if (k < k_min) { k_min = k; }

			if (k > k_max) { k_max = k; }
		}
	}

	// Reduce the saturation as much as possible
	return k_min + k_max;
}
```

### Airmode (chế độ ưu tiên)

`MC_AIRMODE` chọn:
- `0` (disabled): khi mất thrust, *giảm tilt response* (ưu tiên thrust). Drone "dập đất" mềm hơn.
- `1` (Roll/Pitch): cho phép thrust dao động để giữ tilt response.
- `2` (Roll/Pitch/Yaw): cho phép cả yaw — aggressive nhất, dùng cho acro/race.

Nội dung mỗi `mixAirmode*` là một chuỗi `desaturateActuators` với desaturation vector khác nhau (cột yaw, cột thrust, ...).

```@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp:44-65
void
ControlAllocationSequentialDesaturation::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	switch (_param_mc_airmode.get()) {
	case 1:
		mixAirmodeRP();
		break;

	case 2:
		mixAirmodeRPY();
		break;

	default:
		mixAirmodeDisabled();
		break;
	}
}
```

### Output saturation feedback

Sau allocation, tính:
$$
\boldsymbol{c}_{achieved} = \boldsymbol{B}\boldsymbol{u}_{clipped}
$$
$$
\boldsymbol{c}_{unalloc} = \boldsymbol{c}-\boldsymbol{c}_{achieved}
$$

**Giải thích biến**:
- $\boldsymbol{u}_{clipped}=\mathrm{clip}(\boldsymbol{u},u_{min},u_{max})$: motor command sau khi clip về vùng cho phép (có thể bị hạ $\boldsymbol{u}$).
- $\boldsymbol{c}_{achieved}$: lệnh thực sự sinh được — nhân lại $\boldsymbol{B}$ để kiểm đối.
- $\boldsymbol{c}_{unalloc}\ne 0$ nghĩa là saturation xảy ra — đầu vào feedback cho §5 anti-windup.

Publish `control_allocator_status.unallocated_torque/thrust` → rate controller dùng làm anti-windup §5.4(a).

## 6.8. Slew-rate giới hạn motor

Trước khi gửi ra:
$$
u_i^{(k)}\leftarrow u_i^{(k-1)}+\mathrm{clip}\!\left(u_i^{(k)}-u_i^{(k-1)},-r_i\Delta t,\ r_i\Delta t\right)
$$

**Giải thích biến**:
- $u_i^{(k)},u_i^{(k-1)}$: motor command ở bước hiện tại và trước.
- $r_i$ = `CA_R{i}_SLEW`: tốc độ thay đổi max (1/s, vd. 0.4/s).
- $r_i\Delta t$: ngưỡng change mỗi bước — step lớn hơn sẽ bị clip.
- Bảo vệ ESC khỏi nhảy bậc, tránh cộng hưởng cơ, giảm amp spike.

## 6.9. Cơ sở lý thuyết & keywords

| Chủ đề | Keywords |
|---|---|
| Pseudo-inverse | `Moore-Penrose pseudoinverse`, `Greville recursive algorithm`, `weighted least squares` |
| Control allocation | `Bodson 2002 control allocation evaluation`, `Härkegård quadratic programming allocation`, `daisy chain allocation`, `direct allocation` |
| Sequential desaturation | `prioritized control allocation`, `bisection desaturation`, `null space redistribution` |
| Effectiveness matrix | `B-matrix UAV`, `actuator effectiveness model`, `motor torque coefficient` |
| Aggressive flight allocation | `Brescianini Hehn D'Andrea quadcopter trajectory tracking`, `Faessler thrust mixing 2017` |
| Aerospace allocation | `aircraft pseudo-inverse mixing`, `thrust vectoring allocation`, `redundant actuator management` |
| Saturation handling | `saturation-aware MPC`, `anti-windup with allocation feedback` |

### Bài báo nền tảng
- Bodson (2002) — *Evaluation of Optimization Methods for Control Allocation*. JGCD.
- Härkegård (2002) — *Efficient Active Set Algorithms for Solving Constrained Least Squares Problems in Aircraft Control Allocation*.
- Johansen & Fossen (2013) — *Control Allocation - A Survey*. Automatica (đọc nếu muốn tổng quan).
- Faessler, Falanga, Scaramuzza (2017) — *Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight*.

### Sách
- Oppenheimer, Doman, Bolender, *Control Allocation* — chương trong *The Control Handbook*.

## 6.10. Tham số PX4

| Tham số | Ý nghĩa |
|---|---|
| `CA_AIRFRAME` | Loại airframe (quad, hex, ...) |
| `CA_METHOD` | Allocation method (0=PseudoInverse, 1=SequentialDesaturation) |
| `CA_R{0..N}_*` | Tham số motor i (vị trí, góc tilt, ...) |
| `CA_R{i}_SLEW` | Slew-rate cho motor i |
| `MC_AIRMODE` | Chiến lược desaturation |

## 6.11. Tip debug
- Log `actuator_motors.control[i]` xem motor có saturate không.
- Log `control_allocator_status.unallocated_torque/thrust` để phát hiện thiếu authority.
- Nếu yaw "yếu" lúc thrust cao: mặc định airmode 0 ưu tiên thrust → đổi sang 1/2 nếu cần.
- Hiệu chuẩn `c_m` (mô-men cản): nếu yaw drift, có thể `CA_R*_KM` chưa đúng.
