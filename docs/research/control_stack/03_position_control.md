# 3. Position Control

## 3.1. Vai trò

Vòng ngoài cùng của cascaded control. Nhận `trajectory_setpoint` ($\boldsymbol{p}_{sp},\boldsymbol{v}_{sp},\boldsymbol{a}_{sp},\psi_{sp},\dot{\psi}_{sp}$) từ `flight_mode_manager`, cùng state ($\boldsymbol{p},\boldsymbol{v},\boldsymbol{a}_{est},\psi$) từ EKF2. Output là `vehicle_attitude_setpoint` (quaternion + collective thrust) cho tầng attitude.

Cấu trúc: **P (vị trí) → PID (vận tốc) → Acc → Thrust vector → Quaternion**.

## 3.2. Code chính

| Vai trò | File |
|---|---|
| Wrapper uORB + lập lịch | `@/home/frank/tf-px4/src/modules/mc_pos_control/MulticopterPositionControl.cpp` |
| Header tham số | `@/home/frank/tf-px4/src/modules/mc_pos_control/MulticopterPositionControl.hpp` |
| **Lõi thuật toán** | `@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp` |
| Thrust→Attitude geometry | `@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/ControlMath.cpp` |
| Smooth takeoff state machine | `Takeoff/Takeoff.cpp` |
| GoTo smoother | `GotoControl/GotoControl.cpp` |

Hàm cần đọc kỹ:
- `PositionControl::update(dt)` — orchestrator, gọi 3 hàm con.
- `PositionControl::_positionControl()` — P-loop vị trí.
- `PositionControl::_velocityControl(dt)` — PID vận tốc + 2 anti-windup.
- `PositionControl::_accelerationControl()` — gia tốc → thrust vector.
- `ControlMath::thrustToAttitude` & `bodyzToAttitude` — thrust vector → quaternion.
- `ControlMath::limitTilt`, `constrainXY` — saturation hình học.

## 3.3. Bảng ký hiệu

### Setpoint & state (input)

| Ký hiệu | Code | Nghĩa |
|---|---|---|
| $\boldsymbol{p}_{sp},\boldsymbol{v}_{sp,FF},\boldsymbol{a}_{sp,FF}$ | `_pos_sp`, FF parts | setpoint vị trí + feed-forward vận tốc/gia tốc (NED) |
| $\psi_{sp},\dot{\psi}_{sp}$ | `_yaw_sp` | yaw setpoint + yaw rate FF |
| $\boldsymbol{p},\boldsymbol{v}$ | `_pos`, `_vel` | vị trí + vận tốc thực (từ EKF2) |
| $\dot{\boldsymbol{v}}$ | `_vel_dot` | đạo hàm vận tốc (≈ accel filtered) |
| $T_h$ | `_hover_thrust` | hover thrust normalized $\in[0,1]$, từ HTE hoặc `MPC_THR_HOVER` |

### Trung gian (cascade output)

| Ký hiệu | Code | Nghĩa |
|---|---|---|
| $\boldsymbol{v}_{sp}$ | `_vel_sp` | velocity setpoint sau P-pos |
| $\boldsymbol{e}_v=\boldsymbol{v}_{sp}-\boldsymbol{v}$ | `vel_error` | velocity error |
| $\boldsymbol{I}_v$ | `_vel_int` | integrator velocity |
| $\boldsymbol{a}_{sp}$ | `_acc_sp` | acceleration setpoint (m/s²) |
| $\boldsymbol{a}_{cmd}=\boldsymbol{a}_{sp}-\boldsymbol{g}_W$ | derived | gia tốc cần tạo (specific force NED) |
| $\boldsymbol{T}\in\mathbb{R}^3$ | `_thr_sp` | thrust vector normalized (NED) |
| $\hat{\boldsymbol{z}}_B^*,\hat{\boldsymbol{z}}_B^\dagger$ | `body_z` | trục z body mong muốn (raw / sau limit tilt) |
| $\boldsymbol{q}_{sp}$ | `att_sp.q_d` | quaternion setpoint xuất ra |

### Gain & giới hạn

| Ký hiệu | Code / param | Nghĩa |
|---|---|---|
| $\boldsymbol{K}_p^{pos}$ | `_gain_pos_p` / `MPC_{XY,Z}_P` | P-gain vị trí |
| $\boldsymbol{K}_p^v,\boldsymbol{K}_i^v,\boldsymbol{K}_d^v$ | `_gain_vel_{p,i,d}` / `MPC_*_VEL_{P,I,D}_ACC` | PID gains vận tốc |
| $V_{max},V_{up},V_{down}$ | `_lim_vel_*` / `MPC_*_VEL_MAX*` | giới hạn vận tốc |
| $\theta_{max}$ | `_lim_tilt` / `MPC_TILTMAX_AIR` | tilt cực đại (rad) |
| $T_{min},T_{max}$ | `_lim_thr_*` / `MPC_THR_MIN/MAX` | biên thrust |
| $K_{arw}=2/K_p^{v,x}$ | `arw_gain` | ARW gain (Rundqwist 1990) |

### Toán tử

| Ký hiệu | Nghĩa |
|---|---|
| $\odot$ | nhân từng phần (Hadamard / `emult` trong matrix lib) |
| $\boldsymbol{v}^{xy}$ | thành phần ngang $(v_x,v_y)$ |
| $\hat{\boldsymbol{e}}$ | unit vector |
| $\hat{\boldsymbol{z}}_W=(0,0,1)^\top$ | trục z world (NED, hướng xuống) |

---

## 3.4. Công thức chi tiết

### Bước 1 — P-controller vị trí

$$
\boldsymbol{v}_{sp,P} = \boldsymbol{K}_p^{pos}\odot(\boldsymbol{p}_{sp}-\boldsymbol{p})
$$
$$
\boldsymbol{v}_{sp} \leftarrow \boldsymbol{v}_{sp,P} + \boldsymbol{v}_{sp,FF}
$$

**Giải thích biến**:
- $\boldsymbol{p}_{sp}-\boldsymbol{p}$: position error (NED, m).
- $\boldsymbol{K}_p^{pos}=\mathrm{diag}(K_{xy},K_{xy},K_z)$: gain P diagonal.
- $\boldsymbol{v}_{sp,FF}$: feed-forward velocity (từ trajectory generator) — đi thẳng vào output, không qua P.

Đối chiếu code:

```@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp:124-138
void PositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}
```

Tham số: `MPC_XY_P` (~0.95), `MPC_Z_P` (~1.0).

**Giới hạn tốc độ ngang ưu tiên P-term hơn FF** (`constrainXY`): cho $\boldsymbol{v}_0=\boldsymbol{v}_{sp,P}^{xy}$ ưu tiên, tìm $s\ge 0$ sao cho $\|\boldsymbol{v}_0+s\hat{\boldsymbol{v}}_{1}\|\le V_{max}$:
$$
s = -\hat{\boldsymbol{v}}_1\!\cdot\!\boldsymbol{v}_0 + \sqrt{(\hat{\boldsymbol{v}}_1\!\cdot\!\boldsymbol{v}_0)^2 - (\|\boldsymbol{v}_0\|^2-V_{max}^2)}
$$
(nghiệm dương phương trình bậc 2). Code:

```@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/ControlMath.cpp:172-176
Vector2f u1 = v1.normalized();
float m = u1.dot(v0);
float c = v0.dot(v0) - max * max;
float s = -m + sqrtf(m * m - c);
return v0 + u1 * s;
```

Trục Z: $v_{sp,z}\leftarrow\mathrm{clip}(v_{sp,z},-V_{up},V_{down})$.

### Bước 2 — PID-controller vận tốc

$$
\boldsymbol{e}_v=\boldsymbol{v}_{sp}-\boldsymbol{v}
$$
$$
\boldsymbol{a}_{sp,PID} = \boldsymbol{K}_p^v\odot\boldsymbol{e}_v + \boldsymbol{I}_v - \boldsymbol{K}_d^v\odot\dot{\boldsymbol{v}}
$$
$$
\boldsymbol{a}_{sp}\leftarrow \boldsymbol{a}_{sp,PID}+\boldsymbol{a}_{sp,FF}
$$

**Giải thích biến**:
- $\boldsymbol{e}_v$: velocity error (m/s).
- $\boldsymbol{I}_v$: integrator state — tích lũy $\boldsymbol{K}_i^v\odot\boldsymbol{e}_v\Delta t$ qua thời gian.
- $\dot{\boldsymbol{v}}$: đạo hàm vận tốc (≈ gia tốc filtered, lấy từ EKF).
- $\boldsymbol{a}_{sp,FF}$: feed-forward acceleration từ trajectory.

⚠️ D-term tác động lên **đạo hàm vận tốc** $\dot{\boldsymbol{v}}$ (do EKF cung cấp), không phải $\dot{\boldsymbol{e}}_v$ → tránh derivative kick (khi $\boldsymbol{v}_{sp}$ nhảy bậc sẽ không tạo spike $\dot{\boldsymbol{e}}_v$). Code:

```@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp:145-152
// PID velocity control
Vector3f vel_error = _vel_sp - _vel;
Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

// No control input from setpoints or corresponding states which are NAN
ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

_accelerationControl();
```

**Anti-windup Z** (giản đơn): nếu thrust Z đụng giới hạn và sai số cùng dấu → đặt $e_{v,z}=0$ trước khi tích phân.

```@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp:154-158
// Integrator anti-windup in vertical direction
if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
	vel_error(2) = 0.f;
}
```

**Anti-Reset Windup ngang (Rundqwist 1990)**:
$$
\boldsymbol{a}_{prod}^{xy} = \boldsymbol{T}^{xy}\frac{g}{T_h}
$$
$$
\text{Khi }\|\boldsymbol{a}_{sp}^{xy}\| > \|\boldsymbol{a}_{prod}^{xy}\|:\quad
\boldsymbol{e}_v^{xy}\leftarrow\boldsymbol{e}_v^{xy}-K_{arw}(\boldsymbol{a}_{sp}^{xy}-\boldsymbol{a}_{prod}^{xy}),\ K_{arw}=\frac{2}{K_p^{v,x}}.
$$

**Giải thích biến**:
- $\boldsymbol{a}_{prod}^{xy}$: gia tốc ngang **thực sự tạo ra được** với thrust hiện tại (sau khi đã bị saturate bởi tilt-max + thrust-max).
- $\boldsymbol{a}_{sp}^{xy}-\boldsymbol{a}_{prod}^{xy}$: phần gia tốc bị "cắt" do saturation — ARW "đá ngược" lượng này vào $\boldsymbol{e}_v$ để integrator không wind-up.
- $K_{arw}=2/K_p^{v,x}$: gain back-calculation theo công thức chuẩn Rundqwist (~$2/K_p$).
- Khi không saturated ($\boldsymbol{a}_{sp}^{xy}\le\boldsymbol{a}_{prod}^{xy}$): không chạy ARW, integrator tích phân bình thường.

Code:

```@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp:185-201
// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
// The ARW loop needs to run if the signal is saturated only.
if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
	const float arw_gain = 2.f / _gain_vel_p(0);
	const Vector2f acc_sp_xy = _acc_sp.xy();

	vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
}

// Make sure integral doesn't get NAN
ControlMath::setZeroIfNanVector3f(vel_error);
// Update integral part of velocity control
_vel_int += vel_error.emult(_gain_vel_i) * dt;
```

Tích phân Euler:
$$
\boldsymbol{I}_v^{(k+1)}=\boldsymbol{I}_v^{(k)}+\boldsymbol{K}_i^v\odot\boldsymbol{e}_v\Delta t,\quad I_{v,z}\in[-g,g].
$$

- Lưu ý: $\boldsymbol{e}_v$ ở đây là **đã bị sửa** bởi anti-windup Z (set 0 khi saturate) và ARW ngang (trừ back-calc) trước khi tích phân.
- Clamp $|I_{v,z}|\le g$ ngăn integrator chiếm toàn bộ ngân sách thrust.

Tham số: `MPC_{XY,Z}_VEL_{P,I,D}_ACC`.

### Bước 3 — Acceleration → Thrust vector

Gia tốc cần tạo (specific force):
$$
\boldsymbol{a}_{cmd}=\boldsymbol{a}_{sp}-\boldsymbol{g}_W=\begin{bmatrix}a_{sp,x}\\a_{sp,y}\\a_{sp,z}+g\end{bmatrix}
$$

**Giải thích biến**:
- $\boldsymbol{a}_{sp}$: gia tốc mong muốn trong $\{W\}$ NED.
- $\boldsymbol{g}_W=(0,0,g)^\top$ — trong NED, $g$ dương (hướng xuống).
- $\boldsymbol{a}_{cmd}$: specific force = lực không phải trọng trường / khối lượng — cái mà thrust phải sinh ra.
- Dấu trừ trên trục z chuyển thành cộng vì $-(g\cdot 1)= -g$ → hover ($a_{sp,z}=0$): $a_{cmd,z}=-g<0$, hướng lên.

Trục body Z mong muốn (NED, hướng đẩy lên = $-\hat{\boldsymbol{z}}_W$):
$$
\hat{\boldsymbol{z}}_B^* = -\boldsymbol{a}_{cmd}/\|\boldsymbol{a}_{cmd}\|
$$
- Dấu trừ vì thrust đẩy theo $-\hat{\boldsymbol{z}}_B$ (rotor đẩy xuống → phản lực đẩy body lên).
- $\hat{\boldsymbol{z}}_B^*$: trục z body raw (chưa limit tilt).

**Decouple flag** (`MPC_ACC_DECOUPLE`): nếu bật, dùng $z_{spec}=-g$ cố định, bỏ $a_{sp,z}$ khi tính tilt → tránh tilt theo gia tốc dọc.

**Limit tilt** (`limitTilt`):
$$
\theta = \min(\arccos(\hat{\boldsymbol{z}}_B^*\!\cdot\!\hat{\boldsymbol{z}}_W), \theta_{max})
$$
$$
\hat{\boldsymbol{z}}_B^\dagger = \cos\theta\,\hat{\boldsymbol{z}}_W + \sin\theta\,\hat{\boldsymbol{r}},\quad
\hat{\boldsymbol{r}}=\frac{\hat{\boldsymbol{z}}_B^*-(\hat{\boldsymbol{z}}_B^*\!\cdot\!\hat{\boldsymbol{z}}_W)\hat{\boldsymbol{z}}_W}{\|\cdot\|}
$$

- $\theta$: tổng góc tilt (rad), đã clip về $\theta_{max}$.
- $\hat{\boldsymbol{r}}$: thành phần ngang (vuông góc $\hat{\boldsymbol{z}}_W$) của $\hat{\boldsymbol{z}}_B^*$, normalized — trục quay tilt.
- $\hat{\boldsymbol{z}}_B^\dagger$: trục z body sau limit, giữ nguyên hướng tilt nhưng khống chế biên độ.

**Chuyển a → thrust** qua hover thrust:
$$
T_z^{NED} = a_{sp,z}\frac{T_h}{g} - T_h
$$

- $T_h$: hover thrust normalized (= $T$ ứng với $a_z=0$).
- Tại hover ($a_{sp,z}=0$): $T_z^{NED}=-T_h$ (âm vì thrust ngược $\hat{\boldsymbol{z}}_W$).
- Công thức đối xứng quanh hover: gia tốc dương ($a_{sp,z}>0$, muốn đi xuống) → $|T_z^{NED}|<T_h$ (giảm thrust); ngược lại.

Chiếu lên trục body đã giới hạn:
$$
T_{coll}=\min\!\left(\frac{T_z^{NED}}{\hat{\boldsymbol{z}}_W\!\cdot\!\hat{\boldsymbol{z}}_B^\dagger},-T_{min}\right),\quad
\boldsymbol{T}=T_{coll}\,\hat{\boldsymbol{z}}_B^\dagger
$$

- $\hat{\boldsymbol{z}}_W\!\cdot\!\hat{\boldsymbol{z}}_B^\dagger=\cos\theta$: hiệu quả chiếu thrust lên trục dọc (giảm theo cosine khi tilt).
- Chia cho $\cos\theta$: tăng độ lớn thrust để giữ lực dọc như yeu cầu khi tilt.
- $\min(\dots,-T_{min})$: đảm bảo thrust đủ lớn (âm theo chiều NED), không cho hạ dưới $T_{min}$.

Code (toàn bộ `_accelerationControl`):

```@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp:204-222
void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) {
		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		z_specific_force += _acc_sp(2);
	}

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}
```

**Saturation thrust ưu tiên dọc**:
$$
T_{xy,alloc}=\min(\|\boldsymbol{T}^{xy}\|,M_{xy}),\quad T_z\ge -\sqrt{T_{max}^2-T_{xy,alloc}^2}
$$
$$
T_{xy,max}=\sqrt{T_{max}^2-T_z^2}
$$
$$
\boldsymbol{T}^{xy}\leftarrow \boldsymbol{T}^{xy}\frac{T_{xy,max}}{\|\boldsymbol{T}^{xy}\|}\quad(\text{nếu vượt})
$$

- $M_{xy}$: margin ngang (`MPC_THR_XY_MARG`).
- $T_{xy,alloc}$: thrust ngang được "dành" trước.
- $T_z$ được cấp đủ để $\|\boldsymbol{T}\|\le T_{max}$ (giữ altitude tracking là ưu tiên số 1).
- Sau khi $T_z$ chốt, $T_{xy,max}$ được chia từ bở ngân sách còn lại — thrust ngang bị scale xuống nếu cần.

### Bước 4 — Thrust vector → Quaternion (`ControlMath::thrustToAttitude`)

$$
\hat{\boldsymbol{z}}_B = -\boldsymbol{T}/\|\boldsymbol{T}\|
$$
$$
\boldsymbol{y}_C=(-\sin\psi_{sp},\cos\psi_{sp},0)^\top
$$
$$
\hat{\boldsymbol{x}}_B = \frac{\boldsymbol{y}_C\times\hat{\boldsymbol{z}}_B}{\|\boldsymbol{y}_C\times\hat{\boldsymbol{z}}_B\|},\quad
\hat{\boldsymbol{y}}_B = \hat{\boldsymbol{z}}_B\times\hat{\boldsymbol{x}}_B
$$
$$
\boldsymbol{R}_{sp}=[\hat{\boldsymbol{x}}_B\ \hat{\boldsymbol{y}}_B\ \hat{\boldsymbol{z}}_B]\Rightarrow\boldsymbol{q}_{sp}
$$

**Giải thích biến**:
- $\hat{\boldsymbol{z}}_B$: trục z body cuối cùng (sau saturation), đảo dấu với thrust vốn hướng $-\hat{\boldsymbol{z}}_B$ trong NED.
- $\boldsymbol{y}_C$: vector $\hat{\boldsymbol{y}}$ trong **C-frame** (yaw-only intermediate frame) — mặt phẳng ngang, xoay theo $\psi_{sp}$. Công thức $(-\sin\psi,\cos\psi,0)$ chính là quay $\hat{\boldsymbol{e}}_y$ quanh $\hat{\boldsymbol{z}}_W$ góc $\psi_{sp}$.
- $\hat{\boldsymbol{x}}_B$: vuông góc cả $\boldsymbol{y}_C$ và $\hat{\boldsymbol{z}}_B$ — mũi trước body, hướng theo yaw mong muốn.
- $\hat{\boldsymbol{y}}_B$: hoàn thiện hệ trực thuận bằng cross product.
- $\boldsymbol{R}_{sp}=[\hat{\boldsymbol{x}}_B\ \hat{\boldsymbol{y}}_B\ \hat{\boldsymbol{z}}_B]$ — cột là các trục body biểu diễn trong $\{W\}$ ⇒ đó chính là $\boldsymbol{R}_{WB}$.

Code:

```@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/ControlMath.cpp:47-51
void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
	att_sp.thrust_body[2] = -thr_sp.length();
}
```

```@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/ControlMath.cpp:70-114
void bodyzToAttitude(Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	// zero vector, no direction, set safe level value
	if (body_z.norm_squared() < FLT_EPSILON) {
		body_z(2) = 1.f;
	}

	body_z.normalize();

	// vector of desired yaw direction in XY plane, rotated by PI/2
	const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

	// desired body_x axis, orthogonal to body_z
	Vector3f body_x = y_C % body_z;

	// keep nose to front while inverted upside down
	if (body_z(2) < 0.f) {
		body_x = -body_x;
	}

	if (fabsf(body_z(2)) < 0.000001f) {
		// desired thrust is in XY plane, set X downside to construct correct matrix,
		// but yaw component will not be used actually
		body_x.zero();
		body_x(2) = 1.f;
	}

	body_x.normalize();

	// desired body_y axis
	const Vector3f body_y = body_z % body_x;

	Dcmf R_sp;

	// fill rotation matrix
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// copy quaternion setpoint to attitude setpoint topic
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);
}
```

**Output**: `vehicle_attitude_setpoint = (q_d=q_sp, thrust_body=(0,0,-||T||), yaw_sp_move_rate=ψ̇_sp)`.

## 3.5. Cơ sở lý thuyết & keywords

| Chủ đề | Keywords |
|---|---|
| Cascaded P/PID | `cascade control`, `inner-outer loop tuning`, `Skogestad SIMC` |
| Anti-windup | `back-calculation anti-windup`, `Rundqwist 1990 anti-reset windup`, `tracking anti-windup`, `conditional integration` |
| Geometric control | `Lee Leok McClamroch geometric tracking control SE(3)`, `flatness quadrotor`, `Mellinger Kumar minimum-snap` |
| Thrust vectoring | `differential flatness`, `thrust direction control`, `tilt-prioritized control` |
| Feed-forward design | `2-DOF controller`, `feed-forward + feedback structure` |
| Saturation handling | `prioritized control allocation`, `command shaping`, `reference governor` |

### Bài báo nền tảng
- Lee, Leok, McClamroch (2010) — *Geometric Tracking Control of a Quadrotor UAV on SE(3)*. arXiv:1003.2005.
- Mellinger & Kumar (2011) — *Minimum Snap Trajectory Generation and Control for Quadrotors*.
- L. Rundqwist (1990) — *Anti-reset Windup for PID Controllers*.
- Faessler, Falanga, Scaramuzza (2018) — *Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight*.

### Tài liệu mở rộng
- PX4 doc: <https://docs.px4.io/main/en/flight_stack/controller_diagrams.html>.
- Astrom & Hagglund, *PID Controllers: Theory, Design, and Tuning* (chương anti-windup).

## 3.6. Tham số PX4 quan trọng

| Tham số | Ý nghĩa |
|---|---|
| `MPC_XY_P`, `MPC_Z_P` | Gain P vị trí |
| `MPC_XY_VEL_P/I/D_ACC`, `MPC_Z_VEL_P/I/D_ACC` | Gain PID vận tốc |
| `MPC_XY_VEL_MAX`, `MPC_Z_VEL_MAX_UP/DN` | Giới hạn vận tốc |
| `MPC_TILTMAX_AIR`, `MPC_TILTMAX_LND` | Giới hạn tilt |
| `MPC_THR_HOVER`, `MPC_THR_MIN/MAX` | Hover thrust + biên thrust |
| `MPC_THR_XY_MARG` | Margin ngang khi ưu tiên dọc |
| `MPC_USE_HTE` | Bật adaptive hover thrust |
| `MPC_ACC_DECOUPLE` | Decouple tilt khỏi a_z |

## 3.7. Tip debug
- Log: `vehicle_local_position_setpoint` cho thấy mỗi thành phần (x_sp, vx_sp, ax_sp, thrust) → so sánh với `vehicle_local_position` để kiểm tra tracking error.
- Nếu tracking ngang kém: kiểm tra `MPC_THR_HOVER`/HTE trước khi tăng gain.
- Nếu rung dọc khi đáp: thường do D-term + nhiễu accel — giảm `MPC_Z_VEL_D_ACC` hoặc tăng `IMU_ACCEL_CUTOFF`.
