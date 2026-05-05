# Toán học chi tiết của tầng Control Quadcopter PX4

Tài liệu này diễn giải **từng công thức** trong stack điều khiển PX4 dành cho multicopter, đi ngược dòng từ **sensor → EKF2 → Position Control → Attitude Control → Rate Control → Control Allocation → Motor**. Tất cả ký hiệu được dùng nhất quán trên toàn tài liệu.

> Đọc cùng với `quadcopter_control_flow.md` (file tổng quan).
> Mọi tham chiếu code đều ánh xạ tới `src/` trong repo `tf-px4`.

---

## 0. Quy ước ký hiệu (dùng xuyên suốt)

### 0.1. Khung tham chiếu

| Ký hiệu | Khung | Ghi chú |
|---|---|---|
| $\{W\}$ | World / NED | x-North, y-East, z-Down |
| $\{B\}$ | Body | x-Forward, y-Right, z-Down |
| $\boldsymbol{R}_{WB}=\boldsymbol{R}\in SO(3)$ | DCM body→world | cột 3 = $\boldsymbol{z}_B$ trong $\{W\}$ |
| $\boldsymbol{q}=(q_w, q_x, q_y, q_z)$ | quaternion Hamilton, scalar-first | $\boldsymbol{R}=\boldsymbol{R}(\boldsymbol{q})$ |

### 0.2. Đại lượng động học

| Ký hiệu | Ý nghĩa | Khung |
|---|---|---|
| $\boldsymbol{p}=(p_x, p_y, p_z)^\top$ | vị trí | $\{W\}$ |
| $\boldsymbol{v}$ | vận tốc tuyến tính | $\{W\}$ |
| $\boldsymbol{a}$ | gia tốc tuyến tính | $\{W\}$ |
| $\boldsymbol{\omega}=(p,q,r)^\top$ | tốc độ góc | $\{B\}$ |
| $\dot{\boldsymbol{\omega}}=\boldsymbol{\alpha}$ | gia tốc góc | $\{B\}$ |
| $\psi$ | yaw (Euler 3-2-1) | quay quanh $\boldsymbol{z}_W$ |
| $g=9.80665$ | gia tốc trọng trường | hằng số |

Đặt thêm `_sp` cho setpoint (mong muốn): $\boldsymbol{p}_{sp},\boldsymbol{v}_{sp},\boldsymbol{a}_{sp},\boldsymbol{q}_{sp},\boldsymbol{\omega}_{sp}$.

### 0.3. Đại lượng lực / điều khiển

| Ký hiệu | Ý nghĩa |
|---|---|
| $\boldsymbol{T}\in\mathbb{R}^3$ | vector thrust **chuẩn hóa** [-1,1] (PX4 quy ước Z body âm = đẩy lên) |
| $T_z$ (collective) | scalar thrust theo $\boldsymbol{z}_B$, ký hiệu code: `thrust_body[2]` |
| $\boldsymbol{\tau}=(\tau_x,\tau_y,\tau_z)^\top$ | torque chuẩn hóa quanh trục body |
| $T_h\in[0,1]$ | hover thrust (`MPC_THR_HOVER` hoặc HTE) |
| $\boldsymbol{u}\in\mathbb{R}^{n_m}$ | setpoint từng motor [0,1] |
| $\boldsymbol{B}\in\mathbb{R}^{6\times n_m}$ | effectiveness matrix (allocation) |

### 0.4. Toán tử
- $\odot$ = nhân từng phần (Hadamard); trong code matrix là `.emult()`.
- $\boldsymbol{a}\times\boldsymbol{b}$ = tích có hướng.
- $\lfloor x\rfloor_{[a,b]} = \mathrm{clip}(x,a,b)$.
- $[\boldsymbol{x}]_\times$ = ma trận đối xứng-lệch tương ứng tích chéo.
- $\boldsymbol{q}_1\otimes\boldsymbol{q}_2$ = nhân quaternion Hamilton.
- $\mathrm{Im}(\boldsymbol{q}) = (q_x,q_y,q_z)^\top$.

---

## 1. Tầng cảm biến (Sensors)

File: `src/modules/sensors/`, `src/lib/sensor_calibration/`, `src/lib/mathlib/math/filter/`.

### 1.1. IMU (gyroscope + accelerometer)

Mô hình đo lường:
$$
\boldsymbol{\omega}_m = \boldsymbol{\omega} + \boldsymbol{b}_g + \boldsymbol{n}_g, \qquad
\boldsymbol{a}_m = \boldsymbol{R}^\top(\boldsymbol{a} - \boldsymbol{g}_W) + \boldsymbol{b}_a + \boldsymbol{n}_a
$$
với $\boldsymbol{g}_W=(0,0,g)^\top$ trong NED. Pipeline:

1. **Hiệu chuẩn**: trừ bias offline + ma trận xoay/scale → `sensor_gyro`, `sensor_accel`.
2. **Lọc thông thấp** Butterworth bậc 2 ($f_c$ = `IMU_GYRO_CUTOFF` ≈ 30 Hz mặc định).
3. **Notch filter** loại bỏ rung rotor (`IMU_GYRO_NF*`):
$$
H_{notch}(s)=\frac{s^2+\omega_n^2}{s^2+\frac{\omega_n}{Q}s+\omega_n^2}
$$
4. Tích phân coning/sculling 1 kHz → publish `vehicle_angular_velocity` chứa $\boldsymbol{\omega}$ và $\dot{\boldsymbol{\omega}}$ (ước lượng đạo hàm bằng chênh lệch hữu hạn + LPF bậc 1):
$$
\dot{\omega}_k = \alpha\,\dot{\omega}_{k-1} + (1-\alpha)\,\frac{\omega_k-\omega_{k-1}}{\Delta t},\quad \alpha=e^{-\Delta t/\tau}.
$$

### 1.2. Magnetometer / Barometer / GPS / Range / Flow / Vision
Đều được hiệu chuẩn (trừ bias, scale) rồi push vào EKF2 dưới dạng "aiding source" (xem `EKF/aid_sources/*`). Với mỗi cảm biến $i$, giá trị đo là $\boldsymbol{y}_i = h_i(\boldsymbol{x}) + \boldsymbol{n}_i$ với covariance $\boldsymbol{R}_i$.

---

## 2. EKF2 — ước lượng trạng thái

File chính: `src/modules/ekf2/EKF2.cpp`, `src/modules/ekf2/EKF/ekf.h`, state vector tại `src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h`.

### 2.1. Vector trạng thái (24 DoF, biểu diễn 25-element)

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

Trong đó: $\boldsymbol{m}_I$ = từ trường thế giới, $\boldsymbol{m}_B$ = bias mag body, $\boldsymbol{w}_W=(w_n,w_e)$ = gió, $h_t$ = độ cao địa hình. Quaternion (4 phần tử) chỉ chiếm 3 DoF nhờ error-state formulation: covariance kích thước $24\times24$.

### 2.2. Bước dự đoán (prediction)

Vào lúc IMU sample mới ($\Delta t \approx 1\text{–}4$ ms, "delayed horizon"):

**Cập nhật quaternion** (right multiply, error in body frame):
$$
\hat{\boldsymbol{q}}_{k+1} = \hat{\boldsymbol{q}}_{k}\otimes \exp\!\left(\tfrac{1}{2}(\boldsymbol{\omega}_m-\hat{\boldsymbol{b}}_g)\,\Delta t\right)
$$
với $\exp(\boldsymbol{\theta}/2) = (\cos\|\boldsymbol{\theta}\|/2,\ \mathrm{sinc}(\|\boldsymbol{\theta}\|/2)\,\boldsymbol{\theta}/2)$.

**Cập nhật vận tốc/vị trí** (Euler tích phân):
$$
\boldsymbol{a}_W = \boldsymbol{R}(\hat{\boldsymbol{q}})(\boldsymbol{a}_m-\hat{\boldsymbol{b}}_a) + \boldsymbol{g}_W
$$
$$
\hat{\boldsymbol{v}}_{k+1} = \hat{\boldsymbol{v}}_k + \boldsymbol{a}_W\,\Delta t,\qquad
\hat{\boldsymbol{p}}_{k+1} = \hat{\boldsymbol{p}}_k + \hat{\boldsymbol{v}}_k\Delta t + \tfrac{1}{2}\boldsymbol{a}_W\Delta t^2.
$$

**Cập nhật hiệp phương sai** (error-state, 24×24):
$$
\boldsymbol{P}_{k+1} = \boldsymbol{F}\boldsymbol{P}_k\boldsymbol{F}^\top + \boldsymbol{G}\boldsymbol{Q}\boldsymbol{G}^\top
$$
với $\boldsymbol{F}=\partial f/\partial\delta\boldsymbol{x}$ và $\boldsymbol{G}$ Jacobian theo nhiễu (gyro/accel noise + bias random walk). Biểu thức $\boldsymbol{F}$ được auto-generate bằng SymPy: xem `EKF/python/ekf_derivation/generated/predict_covariance.h`.

### 2.3. Bước cập nhật (measurement update)

Với mỗi sensor có residual $\boldsymbol{y}_i - h_i(\hat{\boldsymbol{x}})$:
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

Trong đó $\boxplus$ là phép cộng error-state: với phần quaternion $\hat{\boldsymbol{q}}\leftarrow \hat{\boldsymbol{q}}\otimes\exp(\delta\boldsymbol{\theta}/2)$; phần còn lại là cộng thường. Test innovation gate:
$$
\boldsymbol{r}^\top\boldsymbol{S}^{-1}\boldsymbol{r} < \gamma^2 \quad(\text{vd. }\gamma=5)
$$

### 2.4. Đầu ra (Output predictor)
Vì EKF chạy ở **delayed horizon**, có một bộ "output predictor" chạy ở thời gian thực, propagate $\boldsymbol{q},\boldsymbol{v},\boldsymbol{p}$ bằng IMU mới nhất rồi publish:
- `vehicle_attitude` ($\hat{\boldsymbol{q}}$, $\Delta\boldsymbol{q}_{reset}$)
- `vehicle_local_position` ($\boldsymbol{p},\boldsymbol{v}$ NED, các flag valid)
- `vehicle_angular_velocity` ($\boldsymbol{\omega}-\hat{\boldsymbol{b}}_g$, $\dot{\boldsymbol{\omega}}$)

Các topic này là đầu vào cho mọi tầng controller phía dưới.

---

## 3. Position Control (vòng ngoài cùng)

Lõi: `@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp`.

Setpoint vào: `trajectory_setpoint` từ `flight_mode_manager`, gồm $\boldsymbol{p}_{sp},\boldsymbol{v}_{sp},\boldsymbol{a}_{sp},\psi_{sp},\dot{\psi}_{sp}$ (mỗi trường có thể NaN = không điều khiển).

### 3.1. P-controller vị trí (`_positionControl`)

Hàm `PositionControl::_positionControl`:
$$
\boldsymbol{v}_{sp,P} = \boldsymbol{K}_p^{pos}\odot(\boldsymbol{p}_{sp}-\boldsymbol{p})
$$
$$
\boldsymbol{v}_{sp} \leftarrow \boldsymbol{v}_{sp,P} + \boldsymbol{v}_{sp,FF}\quad(\text{cộng có chọn lọc bỏ NaN})
$$

Tham số: `MPC_XY_P` (mặc định 0.95), `MPC_Z_P` (mặc định 1.0).

**Giới hạn ngang ưu tiên P-term hơn FF** (`ControlMath::constrainXY`): cho $\boldsymbol{v}_0=\boldsymbol{v}_{sp,P}^{xy}$ ưu tiên, $\boldsymbol{v}_1=\boldsymbol{v}_{sp,FF}^{xy}$, ràng buộc $\|\boldsymbol{v}_0+s\hat{\boldsymbol{v}}_1\|\le V_{max}$. Giải bậc 2:
$$
s = -\hat{\boldsymbol{v}}_1\!\cdot\!\boldsymbol{v}_0 + \sqrt{(\hat{\boldsymbol{v}}_1\!\cdot\!\boldsymbol{v}_0)^2 - (\|\boldsymbol{v}_0\|^2-V_{max}^2)}.
$$

Trục Z: $v_{sp,z}\leftarrow \mathrm{clip}(v_{sp,z}, -V_{up}, V_{down})$.

### 3.2. PID-controller vận tốc (`_velocityControl`)

$$
\boldsymbol{e}_v = \boldsymbol{v}_{sp}-\boldsymbol{v},\quad
\boldsymbol{a}_{sp,PID} = \boldsymbol{K}_p^v\odot\boldsymbol{e}_v + \boldsymbol{I}_v - \boldsymbol{K}_d^v\odot\dot{\boldsymbol{v}}
$$
$$
\boldsymbol{a}_{sp} \leftarrow \boldsymbol{a}_{sp,PID} + \boldsymbol{a}_{sp,FF}
$$

(Lưu ý: code dùng $\dot{\boldsymbol{v}}$ — đạo hàm vận tốc — chứ không dùng đạo hàm sai số $\dot{\boldsymbol{e}}_v$, để tránh khuếch đại đỉnh sai số khi $\boldsymbol{v}_{sp}$ nhảy bậc.)

Tích phân $\boldsymbol{I}_v$ được tích lũy *cuối* hàm sau khi xác định saturation:
$$
\boldsymbol{I}_v^{(k+1)} = \boldsymbol{I}_v^{(k)} + \boldsymbol{K}_i^v\odot\boldsymbol{e}_v\,\Delta t
$$

**Anti-windup theo trục Z**: nếu thrust Z đã bão hòa và sai số cùng dấu thì gán $e_{v,z}=0$ trước khi tích phân.

**Anti-windup tracking ngang (Rundqwist 1990)**: gọi $\boldsymbol{a}_{prod}^{xy}$ là gia tốc *thực sự* tạo ra (sau saturation thrust), thì khi $\|\boldsymbol{a}_{sp}^{xy}\|>\|\boldsymbol{a}_{prod}^{xy}\|$:
$$
\boldsymbol{e}_v^{xy}\leftarrow\boldsymbol{e}_v^{xy} - K_{arw}(\boldsymbol{a}_{sp}^{xy}-\boldsymbol{a}_{prod}^{xy}),\quad K_{arw}=\frac{2}{K_p^{v,x}}.
$$

Tham số: `MPC_{XY,Z}_VEL_{P,I,D}_ACC`.

### 3.3. Acceleration → Thrust vector (`_accelerationControl`)

Quan hệ thrust ↔ gia tốc giả định mô hình:
$$
\boldsymbol{a}_{cmd} = \boldsymbol{a}_{sp} - \boldsymbol{g}_W = \boldsymbol{a}_{sp} - g\hat{\boldsymbol{z}}_W
$$
("specific force" cần tạo). Hướng trục Z body mong muốn trong NED:
$$
\hat{\boldsymbol{z}}_B^* = -\boldsymbol{a}_{cmd}/\|\boldsymbol{a}_{cmd}\|
$$
(dấu trừ vì NED z-down, thrust hướng lên = $-\hat{\boldsymbol{z}}_W$).

**Decouple flag** (`MPC_ACC_DECOUPLE`): nếu bật, dùng $z_{spec}=-g$ cố định, bỏ qua $a_{sp,z}$ khi xác định tilt → tránh tilt-sai khi đang gia tốc dọc.

**Limit tilt** (`ControlMath::limitTilt`): cho $\hat{\boldsymbol{z}}_B^*$, $\hat{\boldsymbol{z}}_W=(0,0,1)$, $\theta_{max}$:
$$
\theta = \min(\arccos(\hat{\boldsymbol{z}}_B^*\!\cdot\!\hat{\boldsymbol{z}}_W),\theta_{max})
$$
$$
\hat{\boldsymbol{r}} = \frac{\hat{\boldsymbol{z}}_B^*-(\hat{\boldsymbol{z}}_B^*\!\cdot\!\hat{\boldsymbol{z}}_W)\hat{\boldsymbol{z}}_W}{\|\cdot\|},\quad
\hat{\boldsymbol{z}}_B^\dagger = \cos\theta\,\hat{\boldsymbol{z}}_W + \sin\theta\,\hat{\boldsymbol{r}}.
$$

**Chuyển a → thrust** (chuẩn hóa qua hover thrust):
$$
T_z^{NED} = a_{sp,z}\frac{T_h}{g} - T_h
$$
(suy từ định nghĩa: tại hover $a_{sp,z}=0$ ⇒ $T_z^{NED}=-T_h$.) Sau đó chiếu lên trục body đã được giới hạn tilt:
$$
T_{coll} = \min\!\left(\frac{T_z^{NED}}{\hat{\boldsymbol{z}}_W\!\cdot\!\hat{\boldsymbol{z}}_B^\dagger},\ -T_{min}\right),\quad
\boldsymbol{T} = T_{coll}\,\hat{\boldsymbol{z}}_B^\dagger.
$$

**Saturation thrust** (ưu tiên dọc, giữ margin ngang `MPC_THR_XY_MARG`):
$$
T_{xy,allocated} = \min(\|\boldsymbol{T}^{xy}\|,M_{xy}),\quad
T_z\ge-\sqrt{T_{max}^2-T_{xy,allocated}^2}
$$
$$
T_{xy,max} = \sqrt{T_{max}^2-T_z^2},\qquad
\boldsymbol{T}^{xy}\leftarrow \frac{\boldsymbol{T}^{xy}}{\|\boldsymbol{T}^{xy}\|}T_{xy,max}\ \text{(nếu vượt)}.
$$

### 3.4. Thrust vector → Attitude setpoint (`ControlMath::thrustToAttitude`)

Cho thrust vector $\boldsymbol{T}$ (NED) và yaw setpoint $\psi_{sp}$. Định nghĩa các trục body trong $\{W\}$:
$$
\hat{\boldsymbol{z}}_B = -\boldsymbol{T}/\|\boldsymbol{T}\|
$$
$$
\boldsymbol{y}_C = (-\sin\psi_{sp},\ \cos\psi_{sp},\ 0)^\top\quad(\text{trục y của khung "C" yaw-only})
$$
$$
\hat{\boldsymbol{x}}_B = \frac{\boldsymbol{y}_C\times\hat{\boldsymbol{z}}_B}{\|\boldsymbol{y}_C\times\hat{\boldsymbol{z}}_B\|},\quad
\hat{\boldsymbol{y}}_B = \hat{\boldsymbol{z}}_B\times\hat{\boldsymbol{x}}_B
$$
$$
\boldsymbol{R}_{sp} = [\hat{\boldsymbol{x}}_B\ \hat{\boldsymbol{y}}_B\ \hat{\boldsymbol{z}}_B]\quad\Rightarrow\quad \boldsymbol{q}_{sp} = q(\boldsymbol{R}_{sp}).
$$

Đồng thời `thrust_body[2]` = $-\|\boldsymbol{T}\|$ (collective theo body, âm = đẩy lên).

Output topic `vehicle_attitude_setpoint`: $(\boldsymbol{q}_{sp},\ \boldsymbol{T}^{body}=(0,0,-\|\boldsymbol{T}\|),\ \dot{\psi}_{sp})$.

---

## 4. Attitude Control

Lõi: `@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`. Theo bài báo Brescianini, Hehn, D'Andrea (ETH 2013).

Vào: $\boldsymbol{q}$ (current), $\boldsymbol{q}_{sp}$, $\dot{\psi}_{sp}$.

### 4.1. Tách yaw / tilt (Reduced Attitude)

Mục tiêu: đưa $\hat{\boldsymbol{z}}_B\to\hat{\boldsymbol{z}}_B^{sp}$ với *full priority*, yaw được điều khiển với *trọng số* $w_\psi=$ `MC_YAW_WEIGHT` (mặc định 0.4).

Đặt:
$$
\boldsymbol{e}_z = \hat{\boldsymbol{z}}_B = \boldsymbol{R}(\boldsymbol{q})\hat{\boldsymbol{z}}_W,\qquad
\boldsymbol{e}_z^{sp} = \boldsymbol{R}(\boldsymbol{q}_{sp})\hat{\boldsymbol{z}}_W.
$$

Quaternion quay từ $\boldsymbol{e}_z\to\boldsymbol{e}_z^{sp}$ (rotation tối thiểu giữa 2 trục):
$$
\boldsymbol{q}_{red}^{(W)} = \mathrm{quat\_from\_two\_vectors}(\boldsymbol{e}_z,\boldsymbol{e}_z^{sp}).
$$

Vì $\boldsymbol{q}_{red}^{(W)}$ là delta trong world frame, **right-multiply** với $\boldsymbol{q}$ để có "desired attitude rút gọn":
$$
\boldsymbol{q}_{red} = \boldsymbol{q}_{red}^{(W)}\otimes\boldsymbol{q}.
$$

### 4.2. Phần yaw chênh lệch

$$
\boldsymbol{q}_{\delta\psi} = \boldsymbol{q}_{red}^{-1}\otimes\boldsymbol{q}_{sp}
$$
Theo định nghĩa $\boldsymbol{q}_{\delta\psi}$ chỉ có dạng $(\cos(\alpha/2), 0, 0, \sin(\alpha/2))$. Áp trọng số yaw:
$$
\boldsymbol{q}_{\delta\psi}^{(w)} = (\cos(w_\psi\arccos q_{\delta\psi,w}),\ 0,\ 0,\ \sin(w_\psi\arcsin q_{\delta\psi,z})).
$$

Desired attitude lai:
$$
\boldsymbol{q}_d = \boldsymbol{q}_{red}\otimes\boldsymbol{q}_{\delta\psi}^{(w)}.
$$

### 4.3. Quaternion attitude error → rate setpoint

$$
\boldsymbol{q}_e = \boldsymbol{q}^{-1}\otimes\boldsymbol{q}_d,\quad \text{canonicalize: }\boldsymbol{q}_e\leftarrow\mathrm{sign}(q_{e,w})\boldsymbol{q}_e
$$

Định lý: với quaternion đơn vị, $\mathrm{Im}(\boldsymbol{q}_e)=\sin(\alpha/2)\hat{\boldsymbol{r}}$. Luật điều khiển P (proportional theo trục xoay × góc nhỏ):
$$
\boldsymbol{\omega}_{sp} = 2\,\boldsymbol{K}_p^{att}\odot\mathrm{Im}(\boldsymbol{q}_e)
$$
với $\boldsymbol{K}_p^{att}=(K_p^\phi, K_p^\theta, K_p^\psi/w_\psi)$ — yaw gain được scale ngược lại để khử ảnh hưởng của $w_\psi$.

### 4.4. Feed-forward yaw rate

$\dot{\psi}_{sp}$ xác định trong frame world (quanh $\hat{\boldsymbol{z}}_W$), cần biểu diễn trong body:
$$
\boldsymbol{\omega}_{sp} \mathrel{+}= \boldsymbol{R}^\top(\boldsymbol{q})\hat{\boldsymbol{z}}_W\cdot\dot{\psi}_{sp}.
$$

### 4.5. Giới hạn rate
$$
\omega_{sp,i}\leftarrow\mathrm{clip}(\omega_{sp,i},-\omega_{max,i},\omega_{max,i})
$$
với $\omega_{max,i}=$ `MC_{ROLL,PITCH,YAW}RATE_MAX`.

Output `vehicle_rates_setpoint`: $(\boldsymbol{\omega}_{sp},\ \boldsymbol{T}^{body})$ (thrust pass-through từ position controller).

---

## 5. Rate Control (vòng trong, chạy ~1 kHz)

Lõi: `@/home/frank/tf-px4/src/lib/rate_control/rate_control.cpp`. Module wrapper: `MulticopterRateControl.cpp` (gắn callback theo `vehicle_angular_velocity`).

### 5.1. Luật PID + Feed-forward

$$
\boldsymbol{e}_\omega = \boldsymbol{\omega}_{sp}-\boldsymbol{\omega}
$$
$$
\boxed{\ \boldsymbol{\tau} = \boldsymbol{K}_p^\omega\odot\boldsymbol{e}_\omega + \boldsymbol{I}_\omega - \boldsymbol{K}_d^\omega\odot\dot{\boldsymbol{\omega}} + \boldsymbol{K}_{ff}^\omega\odot\boldsymbol{\omega}_{sp}\ }
$$

Lưu ý: D-term tác động lên $\dot{\boldsymbol{\omega}}$ (trực tiếp đo từ EKF/gyro derivative) chứ không phải $\dot{\boldsymbol{e}}_\omega$ — chống "derivative kick" khi setpoint nhảy.

Tham số: `MC_{ROLL,PITCH,YAW}RATE_{K,P,I,D,FF}`. Các gain "thực" được nhân thêm hệ số $K$ (ideal form):
$$
K_p = K\cdot p,\quad K_i = K\cdot i,\quad K_d = K\cdot d.
$$

### 5.2. Tích phân với chống windup phi tuyến

Trước khi tích phân, áp 3 kiểm tra trên từng trục $i$:

**(a) Saturation feedback từ allocator** (`setSaturationStatus`): nếu trục $i$ đã bão hòa dương ($s_i^+ = 1$) thì:
$$
e_{\omega,i}\leftarrow\min(e_{\omega,i},0)
$$
(tương tự cho âm).

**(b) Nonlinear $i$-factor** giảm gain I khi sai số quá lớn (đã quy ước 400° rad):
$$
i_{f,i} = \max\!\left(0,\ 1-\left(\frac{e_{\omega,i}}{400^\circ}\right)^2\right)
$$

**(c) Tích phân Euler + clamp**:
$$
I_{\omega,i}\leftarrow \mathrm{clip}\!\left(I_{\omega,i} + i_{f,i}\,K_i^\omega\,e_{\omega,i}\,\Delta t,\ -I_{lim,i},\ I_{lim,i}\right).
$$

`I_lim` = `MC_{R,P,Y}R_INT_LIM`. Khi `landed` hoặc disarmed, $\boldsymbol{I}_\omega\leftarrow 0$.

### 5.3. Yaw torque LPF
Để giảm rung do gia tốc rotor:
$$
\tau_z\leftarrow \mathrm{LPF}_{f_c=\text{MC\_YAW\_TQ\_CUTOFF}}(\tau_z).
$$

### 5.4. ACRO mode (manual không có attitude control)

$\boldsymbol{\omega}_{sp}$ được sinh thẳng từ stick với superexpo:
$$
\mathrm{superexpo}(x,e,s) = (1-e)\,x + e\,x^3\quad\text{rồi nhân hệ số mượt} \frac{1-s}{1-s|x|}.
$$
Sau đó nhân với $\boldsymbol{\omega}_{max}^{acro}=$ `MC_ACRO_{R,P,Y}_MAX`.

### 5.5. Battery scaling (tùy chọn `MC_BAT_SCALE_EN`)

$$
\boldsymbol{\tau}\leftarrow\mathrm{clip}(s_{bat}\boldsymbol{\tau},-1,1),\quad\boldsymbol{T}\leftarrow\mathrm{clip}(s_{bat}\boldsymbol{T},-1,1)
$$
với $s_{bat}=V_{nom}/V_{batt}$ — bù sụt áp pin để giữ phản hồi nhất quán.

Output: `vehicle_torque_setpoint` $= \boldsymbol{\tau}$, `vehicle_thrust_setpoint` $= \boldsymbol{T}^{body}$, đều ∈ [-1,1].

---

## 6. Control Allocation (phân phối motor)

Lõi: `@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp`. Wrapper: `ControlAllocator.cpp`.

### 6.1. Bài toán

Vector lệnh (6 trục): $\boldsymbol{c}=[\tau_x,\tau_y,\tau_z,T_x,T_y,T_z]^\top\in\mathbb{R}^6$. Output mỗi motor $\boldsymbol{u}\in\mathbb{R}^{n_m}$. Mô hình tuyến tính:
$$
\boldsymbol{c} = \boldsymbol{B}\boldsymbol{u}
$$
$\boldsymbol{B}\in\mathbb{R}^{6\times n_m}$ = effectiveness matrix, được sinh từ hình học airframe (vị trí motor, hướng trục đẩy, hướng quay) trong `VehicleActuatorEffectiveness/`. Ví dụ với quad X (motor $i$ ở góc $\theta_i$, cách tâm $r$, hệ số mô-men cản $c_m$):
$$
\boldsymbol{B}_{:,i} = \begin{bmatrix}
-r\sin\theta_i \\ r\cos\theta_i \\ \pm c_m \\ 0 \\ 0 \\ -1
\end{bmatrix}
$$
(dấu của $c_m$ phụ thuộc CW/CCW của motor.)

### 6.2. Giải bằng pseudo-inverse Moore–Penrose

$$
\boldsymbol{B}^+ = \boldsymbol{B}^\top(\boldsymbol{B}\boldsymbol{B}^\top)^{-1}\quad\text{(khi }n_m\ge 6\text{ và rank đầy)}
$$
$$
\boxed{\ \boldsymbol{u} = \boldsymbol{u}_{trim} + \boldsymbol{B}^+(\boldsymbol{c}-\boldsymbol{c}_{trim})\ }
$$

Trong code (`allocate()`):
```
_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
```
với `_mix` = $\boldsymbol{B}^+$ đã chuẩn hóa.

### 6.3. Chuẩn hóa cột $\boldsymbol{B}^+$

Để cùng giá trị $\tau_x=1$ tạo tổng lệch motor như nhau bất kể số motor, mỗi cột của $\boldsymbol{B}^+$ được chia bởi:
- Roll/Pitch: $\sqrt{\|\boldsymbol{B}^+_{:,roll}\|^2 / (n_{nz}/2)}$ — giữ chung scale.
- Yaw: $\max_i |B^+_{i,yaw}|$.
- Thrust trục: $\frac{1}{n_{nz}}\sum_i |B^+_{i,thrust}|$.

### 6.4. Sequential Desaturation (SD) — khi $n_m=4$ không đủ DoF

File: `ControlAllocationSequentialDesaturation.cpp`. Khi pseudo-inverse cho ra $u_i\notin[u_{min},u_{max}]$, thuật toán hi sinh các trục theo thứ tự ưu tiên (mặc định cho multicopter): **yaw < tilt-roll/pitch < thrust**. Với mỗi trục bị hi sinh:
$$
\boldsymbol{u}\leftarrow\boldsymbol{u} + s\,\boldsymbol{B}^+_{:,k},\qquad
s = \arg\min_s\sum_i \mathbb{1}[u_i+s\,B^+_{i,k}\notin[u_{min},u_{max}]]\cdot\|\cdot\|
$$
(nói nôm: tìm $s$ để đẩy $\boldsymbol{u}$ về vùng feasible, sau đó kẹp $\boldsymbol{u}\leftarrow\mathrm{clip}(\boldsymbol{u},u_{min},u_{max})$).

`unallocated_torque` $= \boldsymbol{c}-\boldsymbol{B}\boldsymbol{u}$ được publish trong `control_allocator_status` để Rate Controller làm anti-windup ở §5.2(a).

### 6.5. Slew-rate per motor

Trước khi xuất:
$$
u_i^{(k)}\leftarrow u_i^{(k-1)} + \mathrm{clip}(u_i^{(k)}-u_i^{(k-1)},-r_i\Delta t,\,r_i\Delta t)
$$
với $r_i=$ `CA_R{i}_SLEW`.

Output: `actuator_motors.control[i]` $\in[0,1]$ → mixer/driver PWM/DShot/UAVCAN ESC.

---

## 7. Hover Thrust Estimator (HTE) — adaptive

Module `mc_hover_thrust_estimator`. Lý thuyết: RLS scalar, mô hình
$$
a_z^W = g\left(\frac{T_{cmd}}{T_h}-1\right)+\eta
$$
với $T_{cmd}$ = collective thrust hiện tại, $a_z^W$ = gia tốc dọc trong NED. Tham số ẩn $T_h$, ước lượng bởi:
$$
\hat{T}_h^{(k+1)} = \hat{T}_h^{(k)} + K_k\big(a_z - h(\hat{T}_h^{(k)})\big)
$$
$$
K_k = \frac{P_k H_k}{\lambda + H_k^2 P_k},\quad
P_{k+1} = (1-K_k H_k)P_k/\lambda + Q
$$
với $H_k=\partial h/\partial T_h$. Khi `MPC_USE_HTE=1`, $T_h$ trong §3 được cập nhật mượt qua `PositionControl::updateHoverThrust` (đẩy thẳng vào $\boldsymbol{I}_v$ để không gây gián đoạn output):
$$
I_{v,z}\leftarrow I_{v,z} + (a_{sp,z}-g)\frac{T_h^{old}}{T_h^{new}} + g - a_{sp,z}.
$$

---

## 8. Bảng tổng kết: ai sản xuất ai tiêu thụ

| Topic | Nội dung toán | Ai publish | Ai consume |
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

## 9. Pipeline toán học từ đầu đến cuối (one-page)

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

Vòng feedback đóng lại qua IMU/GPS/Mag/Baro → EKF2.

---

## 10. Tham chiếu chéo nhanh tới code

| Tầng | File | Hàm chính |
|---|---|---|
| EKF predict | `src/modules/ekf2/EKF/ekf.cpp` | `Ekf::predictState`, `predictCovariance` |
| EKF update | `src/modules/ekf2/EKF/aid_sources/*` | mỗi sensor một `fuseXxx()` |
| Pos P/PID | `mc_pos_control/PositionControl/PositionControl.cpp` | `_positionControl`, `_velocityControl`, `_accelerationControl` |
| Thrust→Att | `mc_pos_control/PositionControl/ControlMath.cpp` | `thrustToAttitude`, `bodyzToAttitude`, `limitTilt`, `constrainXY` |
| Quat Att | `mc_att_control/AttitudeControl/AttitudeControl.cpp` | `update(q)` |
| Rate PID | `lib/rate_control/rate_control.cpp` | `update`, `updateIntegral` |
| Allocation | `lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp` | `updatePseudoInverse`, `allocate` |
| SD | `…/ControlAllocationSequentialDesaturation.cpp` | `desaturate` |
| HTE | `modules/mc_hover_thrust_estimator/` | `HoverThrustEstimator::update` |

Tất cả công thức trên đã được kiểm chứng trực tiếp với code trong repo `tf-px4`.
