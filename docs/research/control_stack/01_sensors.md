# 1. Sensors & IMU Pipeline

## 1.1. Role in the Stack

The sensor layer converts raw chip data (gyro, accel, mag, baro, GPS, range, optical flow, vision) → standardized uORB topics (calibrated, filtered, voted) to feed into **EKF2**. EKF2 is then responsible for fusing everything into a state estimate. This layer **contains no controller**, but its quality determines the performance ceiling of the entire control stack.

```
chip raw → driver → sensor_xxx → vehicle_xxx (calibrated, filtered, voted) → EKF2
```

## 1.2. Key Code

| Role | File |
|---|---|
| Module wrapper | `@/home/frank/tf-px4/src/modules/sensors/sensors.cpp` |
| Vote multiple IMUs | `@/home/frank/tf-px4/src/modules/sensors/voted_sensors_update.cpp` |
| Merge gyro + accel by timestamp | `@/home/frank/tf-px4/src/modules/sensors/vehicle_imu/` |
| Gyro filtering (LPF + notch + dynamic notch) | `@/home/frank/tf-px4/src/modules/sensors/vehicle_angular_velocity/` |
| Accel filtering | `@/home/frank/tf-px4/src/modules/sensors/vehicle_acceleration/` |
| Baro / GPS / Mag / Flow | `vehicle_air_data/`, `vehicle_gps_position/`, `vehicle_magnetometer/`, `vehicle_optical_flow/` |
| Calibration (offline) | `@/home/frank/tf-px4/src/lib/sensor_calibration/` |
| Coning/sculling integration | `@/home/frank/tf-px4/src/modules/sensors/Integrator.hpp` |
| Dynamic FFT (gyro_fft) | `@/home/frank/tf-px4/src/modules/gyro_fft/` |
| Filter library | `@/home/frank/tf-px4/src/lib/mathlib/math/filter/` (LowPassFilter2pVector3f, NotchFilter, AlphaFilter) |

## 1.3. Notation Table Used in This File

For quick reference without switching back to `00_index.md`. Consistent with the entire control stack.

### Reference Frames

| Symbol | Meaning |
|---|---|
| $\{W\}$ | World frame, **NED** (x-North, y-East, z-Down) |
| $\{B\}$ | Body frame, **FRD** (x-Forward, y-Right, z-Down) |
| $\{S\}$ | Sensor frame (each chip may have a different axis alignment relative to body) |
| $\boldsymbol{R}_{WB}$ | DCM 3×3 rotating $\{B\}\to\{W\}$ (column 3 = $\hat{\boldsymbol{z}}_B$ expressed in $\{W\}$) |
| $\boldsymbol{R}_{BW}=\boldsymbol{R}_{WB}^\top$ | Inverse DCM, rotating $\{W\}\to\{B\}$ |
| $\boldsymbol{R}_{BS}$ | Fixed DCM for hardware mounting (sensor mounting), `_rotation` in calibration code |
| $\boldsymbol{q}$ | Hamilton scalar-first quaternion $(q_w,q_x,q_y,q_z)$ |

### Kinematic Quantities

| Symbol | Meaning | Unit |
|---|---|---|
| $\boldsymbol{p}=(p_x,p_y,p_z)^\top$ | position in $\{W\}$ | m |
| $\boldsymbol{v}$ | linear velocity in $\{W\}$ | m/s |
| $\boldsymbol{a}$ | linear acceleration (physical, gravity-free) in $\{W\}$ | m/s² |
| $\boldsymbol{g}_W=(0,0,g)^\top$ | gravitational field, $g=9.80665$ | m/s² |
| $\boldsymbol{\omega}=(p,q,r)^\top$ | body angular rate | rad/s |
| $\dot{\boldsymbol{\omega}}$ | body angular acceleration | rad/s² |

### Measurements (raw from chip)

| Symbol | Meaning |
|---|---|
| $\boldsymbol{\omega}_m$ | gyro raw (after factory calibration) — `sensor_gyro` |
| $\boldsymbol{a}_m$ | accel raw — `sensor_accel` |
| $\boldsymbol{m}_m$ | mag raw — `sensor_mag` |
| $p$ (scalar) | static pressure — `sensor_baro` |
| $\boldsymbol{p}_{GPS},\boldsymbol{v}_{GPS}$ | GPS measurements converted to NED |
| $\boldsymbol{\omega}_{flow}$ | camera flow rate — `sensor_optical_flow` |

### Sensor Error Parameters

| Symbol | Meaning | Code |
|---|---|---|
| $\boldsymbol{b}_{g}$ | gyro bias (drift) | `_offset` (Gyroscope.hpp) |
| $\boldsymbol{b}_{a}$ | accel bias | `_offset` (Accelerometer.hpp) |
| $\boldsymbol{b}_{th}$ | thermal offset | `_thermal_offset` |
| $\boldsymbol{S}_{g,a}$ | scale + cross-axis (3×3) | `_scale` (vector for accel; matrix for mag) |
| $\boldsymbol{b}_m^{hard}$ | hard-iron mag offset | `_offset` (Magnetometer.hpp) |
| $\boldsymbol{D}$ | soft-iron mag (3×3) | `_scale` (Magnetometer) |
| $\boldsymbol{n}_*$ | Gaussian noise $\sim\mathcal{N}(0,\sigma^2)$ | `EKF2_*_NOISE` |

### Filter Parameters

| Symbol | Meaning | PX4 Parameter |
|---|---|---|
| $f_s$ | sample frequency | auto-detected from IMU |
| $f_c$ | LPF cutoff frequency | `IMU_GYRO_CUTOFF`, `IMU_DGYRO_CUTOFF`, `IMU_ACCEL_CUTOFF` |
| $f_n$, $\omega_n=2\pi f_n$ | notch frequency | `IMU_GYRO_NF*_FRQ` |
| $BW$ | notch bandwidth (-3 dB) | `IMU_GYRO_NF*_BW` |
| $Q=f_n/BW$ | notch quality factor | derived |
| $\alpha\in[0,1]$ | AlphaFilter coefficient (1 = pass input through) | `_alpha`, $\alpha=\Delta t/(\tau+\Delta t)$ |
| $\tau$ | first-order LPF time constant | derived from $f_c$: $\tau=1/(2\pi f_c)$ |
| $h_t$ | terrain height (for optical flow) | EKF2 state |

---

## 1.4. Math Models + Calibration Code

### 1.4.1. IMU (gyro + accel)

#### Measurement Model

$$
\boldsymbol{\omega}_m = \boldsymbol{\omega} + \boldsymbol{b}_g + \boldsymbol{S}_g\boldsymbol{\omega} + \boldsymbol{n}_g
$$

$$
\boldsymbol{a}_m = \boldsymbol{R}_{BW}(\boldsymbol{a}-\boldsymbol{g}_W) + \boldsymbol{b}_a + \boldsymbol{S}_a\boldsymbol{a} + \boldsymbol{n}_a
$$

**Variable descriptions**:
- $\boldsymbol{\omega}_m$: gyro raw (rad/s), sensor measures in $\{S\}$.
- $\boldsymbol{\omega}$: true angular rate (what EKF wants to estimate).
- $\boldsymbol{b}_g$: **gyro bias** — slow drift with temperature + time. EKF2 estimates online (state `gyro_bias`).
- $\boldsymbol{S}_g$: 3×3 scale + cross-axis matrix, nearly diagonal. PX4 currently only stores the diagonal vector for accel (`_scale: Vector3f`), ignoring cross-axis.
- $\boldsymbol{n}_g\sim\mathcal{N}(0,\sigma_g^2 I)$: Gaussian white noise (EKF assumption).
- $\boldsymbol{a}_m$: accel raw (m/s²) — measures **specific force** = non-gravitational force divided by mass.
- $\boldsymbol{R}_{BW}(\boldsymbol{a}-\boldsymbol{g}_W)$: transforms true acceleration in $\{W\}$ to $\{B\}$ then subtracts $\boldsymbol{g}_W$ (due to specific force).
- $\boldsymbol{b}_a$: **accel bias** — offset, EKF2 state `accel_bias`.

#### Correction Code (inverting the above to recover $\boldsymbol{\omega},\boldsymbol{a}$)

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

Cross-referencing the formula: $\boldsymbol{\omega}_{corr} = \boldsymbol{R}_{BS}(\boldsymbol{\omega}_m - \boldsymbol{b}_{th} - \boldsymbol{b}_g)$.
- `_rotation` = $\boldsymbol{R}_{BS}$ (sensor-to-body).
- `_thermal_offset` = $\boldsymbol{b}_{th}$ (thermal correction from `sensor_correction` topic).
- `_offset` = $\boldsymbol{b}_g$ (stored in parameter `CAL_GYROx_*OFF`).

Accel (with additional scale):

```@/home/frank/tf-px4/src/lib/sensor_calibration/Accelerometer.hpp:80-85
// apply offsets and scale
// rotate corrected measurements from sensor to body frame
inline matrix::Vector3f Correct(const matrix::Vector3f &data) const
{
	return _rotation * matrix::Vector3f{(data - _thermal_offset - _offset).emult(_scale)};
}
```

$\boldsymbol{a}_{corr}=\boldsymbol{R}_{BS}\big(\mathrm{diag}(\boldsymbol{S}_a)(\boldsymbol{a}_m-\boldsymbol{b}_{th}-\boldsymbol{b}_a)\big)$.

> **Note**: online bias ($\hat{\boldsymbol{b}}_g,\hat{\boldsymbol{b}}_a$ estimated by EKF2) is subtracted further inside EKF2 (file 02), NOT here. `_offset` is only the offline calibration portion stored in parameters.

---

### 1.4.2. Magnetometer

#### Measurement Model

$$
\boldsymbol{m}_m = \boldsymbol{D}\,\boldsymbol{R}_{BW}\boldsymbol{m}_W + \boldsymbol{b}_m^{hard} + \boldsymbol{n}_m
$$

**Variable descriptions**:
- $\boldsymbol{m}_m$: raw magnetic field vector (gauss or tesla).
- $\boldsymbol{m}_W$: Earth's magnetic field at the flight location (looked up from the World Magnetic Model — `world_magnetic_model/`).
- $\boldsymbol{R}_{BW}$: transforms to body frame.
- $\boldsymbol{D}$ (3×3): **soft-iron** — distortion caused by ferromagnetic materials near the sensor (frame, motors) that distort the field vector into an ellipsoid. Stored in `_scale: Matrix3f`.
- $\boldsymbol{b}_m^{hard}$: **hard-iron** — fixed offset due to permanent magnets (motors, speakers). Stored in `_offset`.
- $\boldsymbol{n}_m$: noise (primarily from motor EMI).

#### Correction Code

```@/home/frank/tf-px4/src/lib/sensor_calibration/Magnetometer.hpp:96-101
// apply offsets and scale
// rotate corrected measurements from sensor to body frame
inline matrix::Vector3f Correct(const matrix::Vector3f &data) const
{
	return _rotation * (_scale * ((data + _power * _power_compensation) - _offset));
}
```

Inverting the model: $\boldsymbol{m}_{corr}=\boldsymbol{R}_{BS}\,\boldsymbol{D}^{-1}(\boldsymbol{m}_m+P\boldsymbol{c}_P-\boldsymbol{b}_m^{hard})$.
- `_scale` = $\boldsymbol{D}^{-1}$ (PX4 stores the inverse directly for computational efficiency).
- `_offset` = $\boldsymbol{b}_m^{hard}$.
- `_power * _power_compensation` = motor current compensation: $\boldsymbol{c}_P\cdot I$ (mag is disturbed proportionally to current — `MagPowerCompensation`).

#### Calibration = Ellipsoid Fit

When the drone is rotated through all orientations (4π sr), the point cloud $\boldsymbol{m}_m$ should lie on a **sphere** of radius $\lVert\boldsymbol{m}_W\rVert$ centered at $\boldsymbol{0}$. Due to hard/soft-iron effects, it is in practice an **off-center ellipsoid**. Ellipsoid fitting solves the problem:

$$
\min_{\boldsymbol{D},\boldsymbol{b}}\sum_k\left|\,\lVert\boldsymbol{D}^{-1}(\boldsymbol{m}_{m,k}-\boldsymbol{b})\rVert^2 - 1\,\right|^2
$$

→ finds $\boldsymbol{D},\boldsymbol{b}$ that transform the point cloud back to a unit sphere. Calibration code is in `commander/calibration/mag_calibration.cpp`.

---

### 1.4.3. Barometer (height)

#### ICAO Atmosphere Formula

$$
h = \frac{T_0}{L}\left[1-\left(\frac{p}{p_0}\right)^{LR/g}\right]
$$

**Variable descriptions**:
- $h$: altitude above reference level (m).
- $p$: measured pressure (Pa).
- $p_0$: reference pressure (Pa) — at home position, or 1013.25 hPa if using MSL.
- $T_0=288.15$ K: ICAO standard temperature at MSL.
- $L=-0.0065$ K/m: lapse rate (temperature decrease with altitude).
- $R=287.05$ J/(kg·K): specific gas constant for dry air.
- $g=9.80665$ m/s².

> **Important**: because the actual $T_0$ differs from 288.15 K, the formula gives **absolute altitude error** of ±20–50 m. However, **delta height** (short-term difference) is very accurate (~10 cm) because the $T_0$ error cancels out — this is why baro is only used as aiding for **z** in EKF.

Code: `src/lib/atmosphere/atmosphere.cpp` (function `getAltitudeFromPressure`).

---

### 1.4.4. GPS

PX4 receives LLA (lat/lon/alt) + ECEF velocity from the GPS module. Converts to local NED using **azimuthal equidistant projection** centered on the home point:

$$
\boldsymbol{p}_{GPS}^{NED} = \mathrm{geo\_project}(\mathrm{lat},\mathrm{lon},\mathrm{alt};\ \mathrm{lat}_0,\mathrm{lon}_0,\mathrm{alt}_0)
$$

EKF observation:

$$
\boldsymbol{p}_{GPS} = \boldsymbol{p}_W + \boldsymbol{n}_{GPS},\qquad \boldsymbol{v}_{GPS}=\boldsymbol{v}_W + \boldsymbol{n}_v
$$

**Variables**:
- $\boldsymbol{p}_{GPS}$: GPS position measurement (NED), $\boldsymbol{n}_{GPS}\sim\mathcal{N}(0,\sigma_{GPS}^2)$ where $\sigma_{GPS}$ is taken from the `eph` field sent by the GPS module.
- $\boldsymbol{v}_{GPS}$: velocity, independent of position (GPS measures Doppler), typically more accurate than position. $\sigma_v$ taken from `epv`.

Code: `src/lib/geo/geo.cpp::project`.

---

### 1.4.5. Optical Flow

#### Flow Rate Model

A downward-facing camera; pixels move in the frame due to two sources: (a) translational motion of the aircraft over the ground, (b) rotation of the aircraft (spinning the camera).

$$
\boldsymbol{\omega}_{flow} = -\frac{1}{h_t}(\boldsymbol{R}_{BW}\boldsymbol{v}_W)\times\hat{\boldsymbol{z}}_B + \boldsymbol{\omega} + \boldsymbol{n}_{flow}
$$

**Variables**:
- $\boldsymbol{\omega}_{flow}$: measured 2D "flow rate" (rad/s in pixels converted through camera FOV).
- $h_t$: **terrain height** (distance from camera to ground). EKF2 has a `terrain` state estimated from range finder + flow.
- $\boldsymbol{R}_{BW}\boldsymbol{v}_W$: body velocity (world velocity projected onto body frame).
- $\hat{\boldsymbol{z}}_B = (0,0,1)^\top$: body vertical axis (camera points in this direction).
- $\boldsymbol{\omega}$: true angular rate (added because rotating the drone also causes pixel motion).
- Cross product $\times\hat{\boldsymbol{z}}_B$ extracts only the 2 horizontal components (since flow only measures 2D).

EKF uses innovation = $\boldsymbol{\omega}_{flow,measured} - \boldsymbol{\omega}_{flow,predicted}$ to update $\boldsymbol{v}_W$ and $h_t$.

Code: `src/modules/ekf2/EKF/aid_sources/optical_flow/optical_flow_control.cpp`.

---

## 1.5. Gyro Filtering Pipeline (detailed with code)

In `vehicle_angular_velocity/`, the processing order is: **calibrate → coning integrate → notch (static + dynamic) → LPF → derivative LPF → publish**.

### 1.5.1. Coning Integrator (Bortz / Savage)

#### Formula

When accumulating high-rate gyro samples into an angular increment $\Delta\boldsymbol{\theta}_k$ over one cycle, simple integration $\int\boldsymbol{\omega}\,dt$ is erroneous when the rotation axis is not aligned with the body axis (the *coning* phenomenon, which generates DC error). Bortz correction:

$$
\Delta\boldsymbol{\theta}_k = \boldsymbol{\alpha}_k + \boldsymbol{\beta}_k
$$

$$
\boldsymbol{\alpha}_k = \int_{t_{k-1}}^{t_k}\boldsymbol{\omega}\,dt\quad(\text{trapezoidal integration})
$$

$$
\boldsymbol{\beta}_k = \tfrac{1}{2}\sum_i\Big(\boldsymbol{\alpha}_{prev} + \tfrac{1}{6}\Delta\boldsymbol{\alpha}_{prev}\Big)\times \Delta\boldsymbol{\alpha}_i
$$

**Variables**:
- $\boldsymbol{\omega}$: calibrated gyro from §1.4.1.
- $\boldsymbol{\alpha}_k$: raw integral (rotation vector, rad).
- $\boldsymbol{\beta}_k$: coning correction term (accumulated over sub-samples).
- $\Delta\boldsymbol{\alpha}_i$: integration increment between two consecutive sub-samples.
- $\Delta\boldsymbol{\theta}_k$: output **delta-angle** ($\approx\boldsymbol{\omega}\cdot\Delta t$ + correction).

#### Code

Trapezoidal integration (base class):

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
$\Delta\boldsymbol{\alpha} = \tfrac{1}{2}(\boldsymbol{\omega}_k+\boldsymbol{\omega}_{k-1})\Delta t$ — that is `(val + _last_val)*dt*0.5f`.

Coning correction (derived class `IntegratorConing`):

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
- `_alpha` = accumulated $\boldsymbol{\alpha}$.
- `_beta` = accumulated $\boldsymbol{\beta}$.
- `_last_alpha`, `_last_delta_alpha` = values from previous cycle.
- Operator `%` in PX4's matrix lib = cross product $\times$.

On reset: returns $\boldsymbol{\alpha}+\boldsymbol{\beta}$ (line 207).

---

### 1.5.2. Low-Pass Filter (2nd-order Butterworth, biquad Direct Form II)

#### Continuous Transfer Function

$$
H(s)=\frac{\omega_c^2}{s^2+\sqrt{2}\,\omega_c\,s+\omega_c^2},\quad \omega_c=2\pi f_c
$$

**Variables**:
- $\omega_c$: cutoff angular frequency (rad/s).
- $f_c$: cutoff (Hz), parameter `IMU_GYRO_CUTOFF`.
- Damping coefficient $\zeta=\sqrt{2}/2$ → Butterworth response (maximally flat passband).

Discretized using **bilinear transform** $s\to\omega_c\tan(\pi/(f_s/f_c))\cdot\frac{z-1}{z+1}$ into a biquad:

$$
y_k = b_0 x_k + b_1 x_{k-1} + b_2 x_{k-2} - a_1 y_{k-1} - a_2 y_{k-2}
$$

#### Coefficient Computation Code

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

- `ohm` = $\Omega = \tan(\pi f_c/f_s)$ (bilinear transform warping factor).
- `cosf(M_PI_F/4.f)` = $\cos(\pi/4)=\sqrt{2}/2$ = Butterworth damping.
- `c` = normalizes $a_0$ to 1.

#### Filter Application Code (Direct Form II)

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
Direct Form II is memory-efficient (only 2 delay elements instead of 4 as in Form I).

---

### 1.5.3. Notch Filter (static + dynamic)

#### Transfer Function

$$
H_{notch}(s)=\frac{s^2+\omega_n^2}{s^2+\frac{\omega_n}{Q}s+\omega_n^2},\quad Q=\frac{f_n}{BW},\ \omega_n=2\pi f_n
$$

**Variables**:
- $f_n$: notch frequency to suppress (Hz). Static: `IMU_GYRO_NF0_FRQ`. Dynamic: taken from realtime FFT.
- $BW$: -3 dB bandwidth (Hz). `IMU_GYRO_NF0_BW`.
- $Q$: quality factor — large $Q$ → narrow/deep notch, small $Q$ → wide/shallow notch.

At $\omega=\omega_n$: $|H|=0$ (perfect zero). Away from $\omega_n$: $|H|\to 1$ (pass-through).

#### Coefficient Computation Code

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
- Note: `_a2 = (1 - alpha) * a0_inv` (see next line in the source file, omitted here for brevity).

#### Application Code (Direct Form I)

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
Direct Form I (rather than Form II as in LPF) is used because history must be preserved when dynamically changing $f_n$ (dynamic notch) — Form II would corrupt history when coefficients change.

---

### 1.5.4. Dynamic Notch (gyro_fft)

#### Mechanism

1. Push gyro samples into a ring buffer of ~512 samples.
2. Apply Hann window then real FFT ⇒ spectrum.
3. Find the 3 strongest spectral peaks in the range [`IMU_GYRO_FFT_MIN`, `IMU_GYRO_FFT_MAX`] Hz.
4. Smooth using AlphaFilter to avoid jitter → set into notch realtime.

#### Peak Picking Formula

$$
f_{peak,i} = \arg\max_{f\in[f_{min},f_{max}]\setminus\{\text{near previous peaks}\}} |X(f)|
$$

where $X(f)$ = magnitude spectrum, $i=1,2,3$.

Code: `src/modules/gyro_fft/GyroFFT.cpp` (see separately for deeper investigation).

---

### 1.5.5. Derivative $\dot{\boldsymbol{\omega}}$ — AlphaFilter

#### Formula

$$
\dot{\boldsymbol{\omega}}_k^{raw} = \frac{\boldsymbol{\omega}_k - \boldsymbol{\omega}_{k-1}}{\Delta t}
$$

$$
\dot{\boldsymbol{\omega}}_k = \dot{\boldsymbol{\omega}}_{k-1} + \alpha(\dot{\boldsymbol{\omega}}_k^{raw}-\dot{\boldsymbol{\omega}}_{k-1}),\quad \alpha=\frac{\Delta t}{\tau+\Delta t}
$$

**Variables**:
- $\dot{\boldsymbol{\omega}}_k^{raw}$: raw derivative (finite difference) — very noisy.
- $\alpha$: weight for new input. $\tau=1/(2\pi f_c)$ where $f_c$ = `IMU_DGYRO_CUTOFF`.
- When $\alpha=1$ → no filtering; $\alpha=0$ → output frozen.

This is a **leaky integrator** / first-order IIR / "exponentially weighted moving average".

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

Cross-reference: `_filter_state + _alpha*(sample - _filter_state)` is exactly $y_{k-1}+\alpha(x_k-y_{k-1})$.

---

### 1.5.6. Multi-IMU Voting

When the airframe has 2–3 IMUs, each IMU runs its own pipeline from §1.4–§1.5 independently. `data_validator/DataValidatorGroup` computes an error metric for each sensor:

$$
e_i = \alpha_e\,e_i^{(prev)} + (1-\alpha_e)\,(\boldsymbol{y}_i - \mathrm{median}_j\boldsymbol{y}_j)^2
$$

**Variables**:
- $e_i$: accumulated error for IMU $i$.
- $\alpha_e$: smoothing factor (~0.95).
- $\mathrm{median}_j\boldsymbol{y}_j$: median of the other IMUs.

The sensor with the lowest $e_i$ and highest priority is selected as primary. Code: `src/lib/systemlib/data_validator/DataValidatorGroup.cpp`.

---

## 1.6. Theoretical Background & Keywords

| Topic | Keywords for Reference |
|---|---|
| Coning/sculling integration | `Bortz strapdown attitude algorithm`, `Savage strapdown algorithm`, `coning compensation` |
| IMU error model | `Allan variance`, `noise density`, `random walk gyro`, `bias instability`, `IEEE-952 gyro spec` |
| IMU calibration | `multi-position calibration`, `TRIAD method`, `least-squares calibration` |
| Mag calibration | `ellipsoid fitting`, `hard-iron soft-iron correction`, `Merayo's algorithm` |
| Notch / IIR filter | `biquad filter`, `Butterworth`, `bilinear transform`, `Direct Form II` |
| Dynamic notch | `motor RPM tracking notch`, `Betaflight dynamic filter`, `peak detection FFT` |
| Sensor voting | `triple modular redundancy`, `fault detection isolation FDI`, `chi-square innovation gating` |
| Optical flow model | `image-plane optical flow`, `Lucas-Kanade`, `egomotion` |
| Barometric altitude | `ICAO standard atmosphere`, `barometric formula`, `temperature compensation` |

### Recommended Reading
- Titterton & Weston, *Strapdown Inertial Navigation Technology* (chapter 11 — coning, sculling).
- Farrell, *Aided Navigation*.
- Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation*.
- IEEE Std 952-1997 (gyro performance specification).
- Renaudin et al. — mag calibration using ellipsoid fitting.

## 1.7. PX4 Parameters to Know
- `IMU_GYRO_CUTOFF`, `IMU_DGYRO_CUTOFF`, `IMU_ACCEL_CUTOFF`
- `IMU_GYRO_NF0_FRQ/BW`, `IMU_GYRO_NF1_FRQ/BW`
- `IMU_GYRO_DYN_NOTCH` (enable dynamic notch)
- `CAL_GYRO*_ID`, `CAL_ACC*_ID`, `CAL_MAG*_ID` — IDs after calibration.
- `EKF2_IMU_CTRL` — flag to enable bias estimation.
