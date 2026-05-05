# 1. Sensors & IMU Pipeline

## 1.1. Vai trò trong stack

Tầng cảm biến chuyển dữ liệu thô từ chip (gyro, accel, mag, baro, GPS, range, optical flow, vision) → các topic uORB chuẩn hóa (đã hiệu chuẩn, lọc, vote) cấp cho **EKF2**. EKF2 mới là người gộp tất cả thành state estimate. Tầng này **không chứa controller**, nhưng chất lượng của nó quyết định trần hiệu năng của toàn bộ control stack.

```
chip raw → driver → sensor_xxx → vehicle_xxx (calibrated, filtered, voted) → EKF2
```

## 1.2. Code chính

| Vai trò | File |
|---|---|
| Module wrapper | `@/home/frank/tf-px4/src/modules/sensors/sensors.cpp` |
| Vote multiple IMU | `@/home/frank/tf-px4/src/modules/sensors/voted_sensors_update.cpp` |
| Gộp gyro + accel theo timestamp | `@/home/frank/tf-px4/src/modules/sensors/vehicle_imu/` |
| Lọc gyro (LPF + notch + dynamic notch) | `@/home/frank/tf-px4/src/modules/sensors/vehicle_angular_velocity/` |
| Lọc accel | `@/home/frank/tf-px4/src/modules/sensors/vehicle_acceleration/` |
| Baro / GPS / Mag / Flow | `vehicle_air_data/`, `vehicle_gps_position/`, `vehicle_magnetometer/`, `vehicle_optical_flow/` |
| Hiệu chuẩn (offline) | `@/home/frank/tf-px4/src/lib/sensor_calibration/` |
| Tích phân coning/sculling | `@/home/frank/tf-px4/src/modules/sensors/Integrator.hpp` |
| FFT động (gyro_fft) | `@/home/frank/tf-px4/src/modules/gyro_fft/` |
| Thư viện filter | `@/home/frank/tf-px4/src/lib/mathlib/math/filter/` (LowPassFilter2pVector3f, NotchFilter, AlphaFilter) |

## 1.3. Bảng ký hiệu dùng trong file này

Để khỏi phải lật về `00_index.md` mỗi lần. Đồng nhất với toàn bộ control stack.

### Khung tham chiếu

| Ký hiệu | Nghĩa |
|---|---|
| $\{W\}$ | World frame, **NED** (x-North, y-East, z-Down) |
| $\{B\}$ | Body frame, **FRD** (x-Forward, y-Right, z-Down) |
| $\{S\}$ | Sensor frame (mỗi chip có thể lệch trục so với body) |
| $\boldsymbol{R}_{WB}$ | DCM 3×3 quay $\{B\}\to\{W\}$ (cột 3 = $\hat{\boldsymbol{z}}_B$ biểu diễn trong $\{W\}$) |
| $\boldsymbol{R}_{BW}=\boldsymbol{R}_{WB}^\top$ | DCM ngược, quay $\{W\}\to\{B\}$ |
| $\boldsymbol{R}_{BS}$ | DCM cố định cấu hình phần cứng (sensor mounting), `_rotation` trong code calibration |
| $\boldsymbol{q}$ | Quaternion Hamilton scalar-first $(q_w,q_x,q_y,q_z)$ |

### Đại lượng động học

| Ký hiệu | Nghĩa | Đơn vị |
|---|---|---|
| $\boldsymbol{p}=(p_x,p_y,p_z)^\top$ | vị trí trong $\{W\}$ | m |
| $\boldsymbol{v}$ | vận tốc tuyến tính trong $\{W\}$ | m/s |
| $\boldsymbol{a}$ | gia tốc tuyến tính (vật lý, không có g) trong $\{W\}$ | m/s² |
| $\boldsymbol{g}_W=(0,0,g)^\top$ | trọng trường, $g=9.80665$ | m/s² |
| $\boldsymbol{\omega}=(p,q,r)^\top$ | tốc độ góc body | rad/s |
| $\dot{\boldsymbol{\omega}}$ | gia tốc góc body | rad/s² |

### Đo lường (raw từ chip)

| Ký hiệu | Nghĩa |
|---|---|
| $\boldsymbol{\omega}_m$ | gyro raw (sau hiệu chuẩn nhà máy) — `sensor_gyro` |
| $\boldsymbol{a}_m$ | accel raw — `sensor_accel` |
| $\boldsymbol{m}_m$ | mag raw — `sensor_mag` |
| $p$ (vô hướng) | áp suất tĩnh — `sensor_baro` |
| $\boldsymbol{p}_{GPS},\boldsymbol{v}_{GPS}$ | đo từ GPS đã chuyển sang NED |
| $\boldsymbol{\omega}_{flow}$ | flow rate camera — `sensor_optical_flow` |

### Tham số sai số cảm biến

| Ký hiệu | Nghĩa | Code |
|---|---|---|
| $\boldsymbol{b}_{g}$ | bias gyro (drift) | `_offset` (Gyroscope.hpp) |
| $\boldsymbol{b}_{a}$ | bias accel | `_offset` (Accelerometer.hpp) |
| $\boldsymbol{b}_{th}$ | thermal offset | `_thermal_offset` |
| $\boldsymbol{S}_{g,a}$ | scale + cross-axis (3×3) | `_scale` (vector cho accel; ma trận cho mag) |
| $\boldsymbol{b}_m^{hard}$ | hard-iron mag offset | `_offset` (Magnetometer.hpp) |
| $\boldsymbol{D}$ | soft-iron mag (3×3) | `_scale` (Magnetometer) |
| $\boldsymbol{n}_*$ | nhiễu Gauss $\sim\mathcal{N}(0,\sigma^2)$ | `EKF2_*_NOISE` |

### Tham số filter

| Ký hiệu | Nghĩa | Tham số PX4 |
|---|---|---|
| $f_s$ | sample frequency | tự động theo IMU |
| $f_c$ | cutoff frequency LPF | `IMU_GYRO_CUTOFF`, `IMU_DGYRO_CUTOFF`, `IMU_ACCEL_CUTOFF` |
| $f_n$, $\omega_n=2\pi f_n$ | notch frequency | `IMU_GYRO_NF*_FRQ` |
| $BW$ | notch bandwidth (-3 dB) | `IMU_GYRO_NF*_BW` |
| $Q=f_n/BW$ | notch quality factor | dẫn xuất |
| $\alpha\in[0,1]$ | hệ số AlphaFilter (1 = giữ nguyên input) | `_alpha`, $\alpha=\Delta t/(\tau+\Delta t)$ |
| $\tau$ | time constant LPF bậc 1 | dẫn xuất từ $f_c$: $\tau=1/(2\pi f_c)$ |
| $h_t$ | terrain height (cho optical flow) | EKF2 state |

---

## 1.4. Mô hình toán + code hiệu chuẩn

### 1.4.1. IMU (gyro + accel)

#### Mô hình đo

$$
\boldsymbol{\omega}_m = \boldsymbol{\omega} + \boldsymbol{b}_g + \boldsymbol{S}_g\boldsymbol{\omega} + \boldsymbol{n}_g
$$
$$
\boldsymbol{a}_m = \boldsymbol{R}_{BW}(\boldsymbol{a}-\boldsymbol{g}_W) + \boldsymbol{b}_a + \boldsymbol{S}_a\boldsymbol{a} + \boldsymbol{n}_a
$$

**Giải thích từng biến**:
- $\boldsymbol{\omega}_m$: gyro raw (rad/s), sensor đo trong $\{S\}$.
- $\boldsymbol{\omega}$: tốc độ góc thật (cái EKF muốn ước lượng).
- $\boldsymbol{b}_g$: **bias gyro** — drift chậm theo nhiệt độ + thời gian. EKF2 ước lượng online (state `gyro_bias`).
- $\boldsymbol{S}_g$: ma trận scale + cross-axis 3×3, gần đường chéo. PX4 hiện chỉ lưu vector đường chéo cho accel (`_scale: Vector3f`), bỏ qua cross-axis.
- $\boldsymbol{n}_g\sim\mathcal{N}(0,\sigma_g^2 I)$: nhiễu trắng Gauss (giả định EKF).
- $\boldsymbol{a}_m$: accel raw (m/s²) — đo **specific force** = lực không phải trọng lực chia khối lượng.
- $\boldsymbol{R}_{BW}(\boldsymbol{a}-\boldsymbol{g}_W)$: chuyển gia tốc thật trong $\{W\}$ về $\{B\}$ rồi trừ $\boldsymbol{g}_W$ (vì specific force).
- $\boldsymbol{b}_a$: **bias accel** — offset, EKF2 state `accel_bias`.

#### Code hiệu chỉnh (đảo công thức trên để lấy lại $\boldsymbol{\omega},\boldsymbol{a}$)

Gyro:

```@/home/frank/tf-px4/src/lib/sensor_calibration/Gyroscope.hpp:79-89
// apply offsets and scale
// rotate corrected measurements from sensor to body frame
inline matrix::Vector3f Correct(const matrix::Vector3f &data) const
{
	return _rotation * matrix::Vector3f{data - _thermal_offset - _offset};
}

inline matrix::Vector3f Uncorrect(const matrix::Vector3f &corrected_data) const
{
	return (_rotation.T() * corrected_data) + _thermal_offset + _offset;
}
```

Đối chiếu công thức: $\boldsymbol{\omega}_{corr} = \boldsymbol{R}_{BS}(\boldsymbol{\omega}_m - \boldsymbol{b}_{th} - \boldsymbol{b}_g)$.
- `_rotation` = $\boldsymbol{R}_{BS}$ (sensor-to-body).
- `_thermal_offset` = $\boldsymbol{b}_{th}$ (hiệu chỉnh nhiệt từ `sensor_correction` topic).
- `_offset` = $\boldsymbol{b}_g$ (lưu trong tham số `CAL_GYROx_*OFF`).

Accel (có thêm scale):

```@/home/frank/tf-px4/src/lib/sensor_calibration/Accelerometer.hpp:80-85
// apply offsets and scale
// rotate corrected measurements from sensor to body frame
inline matrix::Vector3f Correct(const matrix::Vector3f &data) const
{
	return _rotation * matrix::Vector3f{(data - _thermal_offset - _offset).emult(_scale)};
}
```

$\boldsymbol{a}_{corr}=\boldsymbol{R}_{BS}\big(\mathrm{diag}(\boldsymbol{S}_a)(\boldsymbol{a}_m-\boldsymbol{b}_{th}-\boldsymbol{b}_a)\big)$.

> **Lưu ý**: bias online ($\hat{\boldsymbol{b}}_g,\hat{\boldsymbol{b}}_a$ EKF2 ước lượng) được trừ tiếp trong EKF2 (file 02), KHÔNG ở đây. `_offset` chỉ là phần hiệu chuẩn offline lưu vào parameter.

---

### 1.4.2. Magnetometer

#### Mô hình đo

$$
\boldsymbol{m}_m = \boldsymbol{D}\,\boldsymbol{R}_{BW}\boldsymbol{m}_W + \boldsymbol{b}_m^{hard} + \boldsymbol{n}_m
$$

**Giải thích biến**:
- $\boldsymbol{m}_m$: vector từ trường raw (gauss hoặc tesla).
- $\boldsymbol{m}_W$: từ trường địa lý tại vị trí bay (lookup từ World Magnetic Model — `world_magnetic_model/`).
- $\boldsymbol{R}_{BW}$: chuyển về body.
- $\boldsymbol{D}$ (3×3): **soft-iron** — biến dạng do vật liệu sắt từ gần cảm biến (khung, motor) làm méo vector từ trường thành ellipsoid. Lưu trong `_scale: Matrix3f`.
- $\boldsymbol{b}_m^{hard}$: **hard-iron** — offset cố định do nam châm vĩnh cửu (motor, loa). Lưu trong `_offset`.
- $\boldsymbol{n}_m$: nhiễu (đến từ EMI motor là chính).

#### Code hiệu chỉnh

```@/home/frank/tf-px4/src/lib/sensor_calibration/Magnetometer.hpp:96-101
// apply offsets and scale
// rotate corrected measurements from sensor to body frame
inline matrix::Vector3f Correct(const matrix::Vector3f &data) const
{
	return _rotation * (_scale * ((data + _power * _power_compensation) - _offset));
}
```

Đảo mô hình: $\boldsymbol{m}_{corr}=\boldsymbol{R}_{BS}\,\boldsymbol{D}^{-1}(\boldsymbol{m}_m+P\boldsymbol{c}_P-\boldsymbol{b}_m^{hard})$.
- `_scale` = $\boldsymbol{D}^{-1}$ (PX4 lưu thẳng inverse cho rẻ tính).
- `_offset` = $\boldsymbol{b}_m^{hard}$.
- `_power * _power_compensation` = bù dòng điện motor: $\boldsymbol{c}_P\cdot I$ (mag bị nhiễu tỉ lệ với dòng — `MagPowerCompensation`).

#### Hiệu chuẩn = ellipsoid fit

Khi xoay drone đủ hết hướng (4π sr), tập điểm $\boldsymbol{m}_m$ phải nằm trên một **mặt cầu** bán kính $\|\boldsymbol{m}_W\|$ tâm $\boldsymbol{0}$. Do hard/soft-iron, thực tế nó là **ellipsoid lệch tâm**. Ellipsoid fit giải bài toán:
$$
\min_{\boldsymbol{D},\boldsymbol{b}}\sum_k\left|\,\|\boldsymbol{D}^{-1}(\boldsymbol{m}_{m,k}-\boldsymbol{b})\|^2 - 1\,\right|^2
$$
→ tìm $\boldsymbol{D},\boldsymbol{b}$ làm tập điểm thành mặt cầu đơn vị. Code calibration nằm trong `commander/calibration/mag_calibration.cpp`.

---

### 1.4.3. Barometer (height)

#### Công thức ICAO atmosphere

$$
h = \frac{T_0}{L}\left[1-\left(\frac{p}{p_0}\right)^{LR/g}\right]
$$

**Giải thích biến**:
- $h$: độ cao so với mức tham chiếu (m).
- $p$: áp suất đo (Pa).
- $p_0$: áp suất tham chiếu (Pa) — tại home, hoặc 1013.25 hPa nếu dùng MSL.
- $T_0=288.15$ K: nhiệt độ chuẩn ICAO ở MSL.
- $L=-0.0065$ K/m: lapse rate (suất giảm nhiệt độ theo độ cao).
- $R=287.05$ J/(kg·K): hằng số khí riêng cho không khí khô.
- $g=9.80665$ m/s².

> **Quan trọng**: do $T_0$ ở mẫu thực không bằng 288.15 K, công thức cho **altitude tuyệt đối sai** ±20–50 m. Nhưng **delta height** (chênh lệch ngắn hạn) thì rất chính xác (~10 cm) vì sai số $T_0$ triệt tiêu — đó là lý do baro chỉ dùng làm aiding cho **z** trong EKF.

Code: `src/lib/atmosphere/atmosphere.cpp` (hàm `getAltitudeFromPressure`).

---

### 1.4.4. GPS

PX4 nhận LLA (lat/lon/alt) + ECEF velocity từ module GPS. Chuyển sang local NED bằng **azimuthal equidistant projection** quanh điểm home:
$$
\boldsymbol{p}_{GPS}^{NED} = \mathrm{geo\_project}(\mathrm{lat},\mathrm{lon},\mathrm{alt};\ \mathrm{lat}_0,\mathrm{lon}_0,\mathrm{alt}_0)
$$

EKF observation:
$$
\boldsymbol{p}_{GPS} = \boldsymbol{p}_W + \boldsymbol{n}_{GPS},\qquad \boldsymbol{v}_{GPS}=\boldsymbol{v}_W + \boldsymbol{n}_v
$$

**Biến**:
- $\boldsymbol{p}_{GPS}$: vị trí GPS đo (NED), $\boldsymbol{n}_{GPS}\sim\mathcal{N}(0,\sigma_{GPS}^2)$ với $\sigma_{GPS}$ lấy từ `eph` GPS module gửi lên.
- $\boldsymbol{v}_{GPS}$: vận tốc, độc lập với position (GPS đo Doppler), thường chính xác hơn position. $\sigma_v$ lấy từ `epv`.

Code: `src/lib/geo/geo.cpp::project`.

---

### 1.4.5. Optical Flow

#### Mô hình flow rate

Camera nhìn xuống đất; pixel di chuyển trong khung hình do hai nguồn: (a) máy bay tịnh tiến trên mặt đất, (b) máy bay quay (xoay camera).

$$
\boldsymbol{\omega}_{flow} = -\frac{1}{h_t}(\boldsymbol{R}_{BW}\boldsymbol{v}_W)\times\hat{\boldsymbol{z}}_B + \boldsymbol{\omega} + \boldsymbol{n}_{flow}
$$

**Biến**:
- $\boldsymbol{\omega}_{flow}$: "flow rate" 2D đo được (rad/s ở pixel quy đổi qua FOV camera).
- $h_t$: **terrain height** (khoảng cách camera tới mặt đất). EKF2 có state `terrain` ước lượng từ range finder + flow.
- $\boldsymbol{R}_{BW}\boldsymbol{v}_W$: vận tốc body (chiếu world velocity về body).
- $\hat{\boldsymbol{z}}_B = (0,0,1)^\top$: trục dọc body (camera nhìn theo chiều này).
- $\boldsymbol{\omega}$: tốc độ góc thật (cộng vào vì xoay drone cũng làm pixel chuyển động).
- Tích chéo $\times\hat{\boldsymbol{z}}_B$ chỉ lấy 2 thành phần ngang (vì flow chỉ đo 2D).

EKF dùng innovation = $\boldsymbol{\omega}_{flow,measured} - \boldsymbol{\omega}_{flow,predicted}$ để cập nhật $\boldsymbol{v}_W$ và $h_t$.

Code: `src/modules/ekf2/EKF/aid_sources/optical_flow/optical_flow_control.cpp`.

---

## 1.5. Pipeline lọc gyro (chi tiết với code)

Trong `vehicle_angular_velocity/`, thứ tự xử lý: **calibrate → coning integrate → notch (static + dynamic) → LPF → derivative LPF → publish**.

### 1.5.1. Coning integrator (Bortz / Savage)

#### Công thức

Khi gộp các sample gyro tốc độ cao thành tăng góc $\Delta\boldsymbol{\theta}_k$ trên một chu kỳ, nếu chỉ tích phân thường $\int\boldsymbol{\omega}\,dt$ sẽ sai khi trục quay không trùng với trục thân (hiện tượng *coning*, sinh sai số DC). Hiệu chỉnh Bortz:

$$
\Delta\boldsymbol{\theta}_k = \boldsymbol{\alpha}_k + \boldsymbol{\beta}_k
$$
$$
\boldsymbol{\alpha}_k = \int_{t_{k-1}}^{t_k}\boldsymbol{\omega}\,dt\quad(\text{tích phân hình thang})
$$
$$
\boldsymbol{\beta}_k = \tfrac{1}{2}\sum_i\Big(\boldsymbol{\alpha}_{prev} + \tfrac{1}{6}\Delta\boldsymbol{\alpha}_{prev}\Big)\times \Delta\boldsymbol{\alpha}_i
$$

**Biến**:
- $\boldsymbol{\omega}$: gyro đã hiệu chuẩn ở §1.4.1.
- $\boldsymbol{\alpha}_k$: tích phân thô (góc quay vector, rad).
- $\boldsymbol{\beta}_k$: số hạng coning correction (lũy kế qua các sub-sample).
- $\Delta\boldsymbol{\alpha}_i$: tăng tích phân giữa 2 sub-sample liên tiếp.
- $\Delta\boldsymbol{\theta}_k$: **delta-angle** xuất ra ($\approx\boldsymbol{\omega}\cdot\Delta t$ + correction).

#### Code

Tích phân hình thang (lớp gốc):

```@/home/frank/tf-px4/src/modules/sensors/Integrator.hpp:130-139
inline matrix::Vector3f integrate(const matrix::Vector3f &val, const float dt)
{
	// Use trapezoidal integration to calculate the delta integral
	_integrated_samples++;
	_integral_dt += dt;
	const matrix::Vector3f delta_alpha{(val + _last_val) *dt * 0.5f};
	_last_val = val;

	return delta_alpha;
}
```
$\Delta\boldsymbol{\alpha} = \tfrac{1}{2}(\boldsymbol{\omega}_k+\boldsymbol{\omega}_{k-1})\Delta t$ — đó là `(val + _last_val)*dt*0.5f`.

Coning correction (lớp dẫn xuất `IntegratorConing`):

```@/home/frank/tf-px4/src/modules/sensors/Integrator.hpp:164-187
inline void put(const matrix::Vector3f &val, const float dt)
{
	if ((dt > DT_MIN) && (_integral_dt + dt < DT_MAX)) {
		// Use trapezoidal integration to calculate the delta integral
		const matrix::Vector3f delta_alpha{integrate(val, dt)};

		// Calculate coning corrections
		// Coning compensation derived by Paul Riseborough and Jonathan Challinger,
		// following:
		// Strapdown Inertial Navigation Integration Algorithm Design Part 1: Attitude Algorithms
		// Sourced: https://arc.aiaa.org/doi/pdf/10.2514/2.4228
		_beta += ((_last_alpha + _last_delta_alpha * (1.f / 6.f)) % delta_alpha) * 0.5f;
		_last_delta_alpha = delta_alpha;
		_last_alpha = _alpha;

		// accumulate delta integrals
		_alpha += delta_alpha;

	} else {
		reset();
		_last_val = val;
	}
}
```
- `_alpha` = $\boldsymbol{\alpha}$ tích lũy.
- `_beta` = $\boldsymbol{\beta}$ tích lũy.
- `_last_alpha`, `_last_delta_alpha` = giá trị chu kỳ trước.
- Toán tử `%` trong matrix lib PX4 = tích chéo $\times$.

Khi reset: trả về $\boldsymbol{\alpha}+\boldsymbol{\beta}$ (line 207).

---

### 1.5.2. Low-Pass Filter (Butterworth bậc 2, biquad Direct Form II)

#### Hàm truyền liên tục

$$
H(s)=\frac{\omega_c^2}{s^2+\sqrt{2}\,\omega_c\,s+\omega_c^2},\quad \omega_c=2\pi f_c
$$

**Biến**:
- $\omega_c$: cutoff angular frequency (rad/s).
- $f_c$: cutoff (Hz), tham số `IMU_GYRO_CUTOFF`.
- Hệ số damping $\zeta=\sqrt{2}/2$ → đáp ứng Butterworth (phẳng nhất ở passband).

Rời rạc hóa bằng **bilinear transform** $s\to\omega_c\tan(\pi/(f_s/f_c))\cdot\frac{z-1}{z+1}$ thành biquad:
$$
y_k = b_0 x_k + b_1 x_{k-1} + b_2 x_{k-2} - a_1 y_{k-1} - a_2 y_{k-2}
$$

#### Code tính hệ số

```@/home/frank/tf-px4/src/lib/mathlib/math/filter/LowPassFilter2p.hpp:74-90
_cutoff_freq = math::max(cutoff_freq, sample_freq * 0.001f);
_sample_freq = sample_freq;

const float fr = _sample_freq / _cutoff_freq;
const float ohm = tanf(M_PI_F / fr);
const float c = 1.f + 2.f * cosf(M_PI_F / 4.f) * ohm + ohm * ohm;

_b0 = ohm * ohm / c;
_b1 = 2.f * _b0;
_b2 = _b0;

_a1 = 2.f * (ohm * ohm - 1.f) / c;
_a2 = (1.f - 2.f * cosf(M_PI_F / 4.f) * ohm + ohm * ohm) / c;
```

- `ohm` = $\Omega = \tan(\pi f_c/f_s)$ (warping factor của bilinear transform).
- `cosf(M_PI_F/4.f)` = $\cos(\pi/4)=\sqrt{2}/2$ = damping Butterworth.
- `c` = chuẩn hóa $a_0$ về 1.

#### Code áp filter (Direct Form II)

```@/home/frank/tf-px4/src/lib/mathlib/math/filter/LowPassFilter2p.hpp:98-109
inline T apply(const T &sample)
{
	// Direct Form II implementation
	T delay_element_0{sample - _delay_element_1 *_a1 - _delay_element_2 * _a2};

	const T output{delay_element_0 *_b0 + _delay_element_1 *_b1 + _delay_element_2 * _b2};

	_delay_element_2 = _delay_element_1;
	_delay_element_1 = delay_element_0;

	return output;
}
```
Direct Form II tiết kiệm bộ nhớ (chỉ 2 phần tử trễ thay vì 4 như Form I).

---

### 1.5.3. Notch Filter (cố định + động)

#### Hàm truyền

$$
H_{notch}(s)=\frac{s^2+\omega_n^2}{s^2+\frac{\omega_n}{Q}s+\omega_n^2},\quad Q=\frac{f_n}{BW},\ \omega_n=2\pi f_n
$$

**Biến**:
- $f_n$: tần số notch cần cắt (Hz). Cố định: `IMU_GYRO_NF0_FRQ`. Động: lấy từ FFT realtime.
- $BW$: bandwidth -3 dB (Hz). `IMU_GYRO_NF0_BW`.
- $Q$: quality factor — $Q$ lớn → notch hẹp/sâu, $Q$ nhỏ → rộng/nông.

Tại $\omega=\omega_n$: $|H|=0$ (zero hoàn hảo). Xa $\omega_n$: $|H|\to 1$ (pass-through).

#### Code tính hệ số

```@/home/frank/tf-px4/src/lib/mathlib/math/filter/NotchFilter.hpp:267-279
_sample_freq = sample_freq;
_notch_freq = notch_freq_new;
_bandwidth = bandwidth_new;

const float alpha = tanf(M_PI_F * _bandwidth / _sample_freq);
const float beta = -cosf(2.f * M_PI_F * _notch_freq / _sample_freq);
const float a0_inv = 1.f / (alpha + 1.f);

_b0 = a0_inv;
_b1 = 2.f * beta * a0_inv;
_b2 = a0_inv;

_a1 = _b1;
```
- `alpha` = $\tan(\pi BW/f_s)$.
- `beta` = $-\cos(2\pi f_n/f_s)$.
- Note: `_a2 = (1 - alpha) * a0_inv` (xem dòng kế tiếp trong file gốc, không in vào đây để gọn).

#### Code áp (Direct Form I)

```@/home/frank/tf-px4/src/lib/mathlib/math/filter/NotchFilter.hpp:173-188
inline T applyInternal(const T &sample)
{
	// Direct Form I implementation
	T output = _b0 * sample + _b1 * _delay_element_1 + _b2 * _delay_element_2 - _a1 * _delay_element_output_1 - _a2 *
		   _delay_element_output_2;

	// shift inputs
	_delay_element_2 = _delay_element_1;
	_delay_element_1 = sample;

	// shift outputs
	_delay_element_output_2 = _delay_element_output_1;
	_delay_element_output_1 = output;

	return output;
}
```
Direct Form I (chứ không phải II như LPF) vì cần giữ history khi dynamically đổi $f_n$ (notch động) — Form II sẽ phá history khi đổi hệ số.

---

### 1.5.4. Dynamic Notch (gyro_fft)

#### Cơ chế

1. Đẩy mẫu gyro vào ring buffer ~ 512 mẫu.
2. Áp Hann window rồi FFT thực ⇒ phổ.
3. Tìm 3 đỉnh phổ mạnh nhất trong dải [`IMU_GYRO_FFT_MIN`, `IMU_GYRO_FFT_MAX`] Hz.
4. Smooth bằng AlphaFilter để tránh giật → set vào notch realtime.

#### Công thức peak picking
$$
f_{peak,i} = \arg\max_{f\in[f_{min},f_{max}]\setminus\{\text{near previous peaks}\}} |X(f)|
$$
với $X(f)$ = magnitude spectrum, $i=1,2,3$.

Code: `src/modules/gyro_fft/GyroFFT.cpp` (cần xem riêng nếu muốn đào sâu).

---

### 1.5.5. Đạo hàm $\dot{\boldsymbol{\omega}}$ — AlphaFilter

#### Công thức

$$
\dot{\boldsymbol{\omega}}_k^{raw} = \frac{\boldsymbol{\omega}_k - \boldsymbol{\omega}_{k-1}}{\Delta t}
$$
$$
\dot{\boldsymbol{\omega}}_k = \dot{\boldsymbol{\omega}}_{k-1} + \alpha(\dot{\boldsymbol{\omega}}_k^{raw}-\dot{\boldsymbol{\omega}}_{k-1}),\quad \alpha=\frac{\Delta t}{\tau+\Delta t}
$$

**Biến**:
- $\dot{\boldsymbol{\omega}}_k^{raw}$: đạo hàm thô (chênh lệch hữu hạn) — rất nhiễu.
- $\alpha$: weight cho input mới. $\tau=1/(2\pi f_c)$ với $f_c$ = `IMU_DGYRO_CUTOFF`.
- Khi $\alpha=1$ → không lọc; $\alpha=0$ → output đứng yên.

Đây chính là **leaky integrator** / first-order IIR / "exponentially weighted moving average".

#### Code

```@/home/frank/tf-px4/src/lib/mathlib/math/filter/AlphaFilter.hpp:144-145
template <typename T>
T AlphaFilter<T>::updateCalculation(const T &sample) { return _filter_state + _alpha * (sample - _filter_state); }
```

```@/home/frank/tf-px4/src/lib/mathlib/math/filter/AlphaFilter.hpp:68-77
void setParameters(float sample_interval, float time_constant)
{
	const float denominator = time_constant + sample_interval;

	if (denominator > FLT_EPSILON) {
		setAlpha(sample_interval / denominator);
	}

	_time_constant = time_constant;
}
```

Đối chiếu: `_filter_state + _alpha*(sample - _filter_state)` chính là $y_{k-1}+\alpha(x_k-y_{k-1})$.

---

### 1.5.6. Voting nhiều IMU

Khi airframe có 2-3 IMU, mỗi IMU chạy đường ống §1.4-§1.5 độc lập. `data_validator/DataValidatorGroup` tính error metric cho từng cảm biến:
$$
e_i = \alpha_e\,e_i^{(prev)} + (1-\alpha_e)\,(\boldsymbol{y}_i - \mathrm{median}_j\boldsymbol{y}_j)^2
$$

**Biến**:
- $e_i$: error tích lũy cho IMU $i$.
- $\alpha_e$: smoothing factor (~0.95).
- $\mathrm{median}_j\boldsymbol{y}_j$: trung vị của các IMU khác.

Sensor có $e_i$ thấp nhất + priority cao nhất được chọn làm primary. Code: `src/lib/systemlib/data_validator/DataValidatorGroup.cpp`.

---

## 1.6. Cơ sở lý thuyết & keywords

| Chủ đề | Keywords để tra cứu |
|---|---|
| Coning/sculling integration | `Bortz strapdown attitude algorithm`, `Savage strapdown algorithm`, `coning compensation` |
| IMU error model | `Allan variance`, `noise density`, `random walk gyro`, `bias instability`, `IEEE-952 gyro spec` |
| Hiệu chuẩn IMU | `multi-position calibration`, `TRIAD method`, `least-squares calibration` |
| Mag calibration | `ellipsoid fitting`, `hard-iron soft-iron correction`, `Merayo's algorithm` |
| Notch / IIR filter | `biquad filter`, `Butterworth`, `bilinear transform`, `Direct Form II` |
| Dynamic notch | `motor RPM tracking notch`, `Betaflight dynamic filter`, `peak detection FFT` |
| Sensor voting | `triple modular redundancy`, `fault detection isolation FDI`, `chi-square innovation gating` |
| Optical flow model | `image-plane optical flow`, `Lucas-Kanade`, `egomotion` |
| Barometric altitude | `ICAO standard atmosphere`, `barometric formula`, `temperature compensation` |

### Tài liệu nên đọc
- Titterton & Weston, *Strapdown Inertial Navigation Technology* (chương 11 — coning, sculling).
- Farrell, *Aided Navigation*.
- Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation*.
- IEEE Std 952-1997 (gyro performance specification).
- Renaudin et al. — hiệu chuẩn mag bằng ellipsoid fitting.

## 1.7. Tham số PX4 cần biết
- `IMU_GYRO_CUTOFF`, `IMU_DGYRO_CUTOFF`, `IMU_ACCEL_CUTOFF`
- `IMU_GYRO_NF0_FRQ/BW`, `IMU_GYRO_NF1_FRQ/BW`
- `IMU_GYRO_DYN_NOTCH` (bật dynamic notch)
- `CAL_GYRO*_ID`, `CAL_ACC*_ID`, `CAL_MAG*_ID` — id sau hiệu chuẩn.
- `EKF2_IMU_CTRL` — cờ bật bias estimation.
