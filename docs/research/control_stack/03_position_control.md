# 3. Position Control

## 3.1. Role

The outermost loop of the cascaded control stack. Receives `trajectory_setpoint` ($\boldsymbol{p}_{sp},\boldsymbol{v}_{sp},\boldsymbol{a}_{sp},\psi_{sp},\dot{\psi}_{sp}$) from `flight_mode_manager`, together with state ($\boldsymbol{p},\boldsymbol{v},\boldsymbol{a}_{est},\psi$) from EKF2. Output is `vehicle_attitude_setpoint` (quaternion + collective thrust) for the attitude layer.

Structure: **P (position) → PID (velocity) → Acc → Thrust vector → Quaternion**.

## 3.2. Main Code

| Role | File |
|---|---|
| uORB wrapper + scheduling | `@/home/frank/tf-px4/src/modules/mc_pos_control/MulticopterPositionControl.cpp` |
| Parameter header | `@/home/frank/tf-px4/src/modules/mc_pos_control/MulticopterPositionControl.hpp` |
| **Algorithm core** | `@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp` |
| Thrust→Attitude geometry | `@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/ControlMath.cpp` |
| Smooth takeoff state machine | `Takeoff/Takeoff.cpp` |
| GoTo smoother | `GotoControl/GotoControl.cpp` |

Functions to read carefully:
- `PositionControl::update(dt)` — orchestrator, calls 3 sub-functions.
- `PositionControl::_positionControl()` — position P-loop.
- `PositionControl::_velocityControl(dt)` — velocity PID + 2 anti-windup schemes.
- `PositionControl::_accelerationControl()` — acceleration → thrust vector.
- `ControlMath::thrustToAttitude` & `bodyzToAttitude` — thrust vector → quaternion.
- `ControlMath::limitTilt`, `constrainXY` — geometric saturation.

## 3.3. Notation Table

### Setpoint & state (input)

| Symbol | Code | Meaning |
|---|---|---|
| $\boldsymbol{p}_{sp},\boldsymbol{v}_{sp,FF},\boldsymbol{a}_{sp,FF}$ | `_pos_sp`, FF parts | position setpoint + feed-forward velocity/acceleration (NED) |
| $\psi_{sp},\dot{\psi}_{sp}$ | `_yaw_sp` | yaw setpoint + yaw rate FF |
| $\boldsymbol{p},\boldsymbol{v}$ | `_pos`, `_vel` | actual position + velocity (from EKF2) |
| $\dot{\boldsymbol{v}}$ | `_vel_dot` | velocity derivative (≈ filtered accel) |
| $T_h$ | `_hover_thrust` | normalized hover thrust $\in[0,1]$, from HTE or `MPC_THR_HOVER` |

### Intermediate (cascade output)

| Symbol | Code | Meaning |
|---|---|---|
| $\boldsymbol{v}_{sp}$ | `_vel_sp` | velocity setpoint after position P |
| $\boldsymbol{e}_v=\boldsymbol{v}_{sp}-\boldsymbol{v}$ | `vel_error` | velocity error |
| $\boldsymbol{I}_v$ | `_vel_int` | velocity integrator |
| $\boldsymbol{a}_{sp}$ | `_acc_sp` | acceleration setpoint (m/s²) |
| $\boldsymbol{a}_{cmd}=\boldsymbol{a}_{sp}-\boldsymbol{g}_W$ | derived | required acceleration (specific force NED) |
| $\boldsymbol{T}\in\mathbb{R}^3$ | `_thr_sp` | normalized thrust vector (NED) |
| $\hat{\boldsymbol{z}}_B^*,\hat{\boldsymbol{z}}_B^\dagger$ | `body_z` | desired body z axis (raw / after tilt limit) |
| $\boldsymbol{q}_{sp}$ | `att_sp.q_d` | output quaternion setpoint |

### Gains & limits

| Symbol | Code / param | Meaning |
|---|---|---|
| $\boldsymbol{K}_p^{pos}$ | `_gain_pos_p` / `MPC_{XY,Z}_P` | position P-gain |
| $\boldsymbol{K}_p^v,\boldsymbol{K}_i^v,\boldsymbol{K}_d^v$ | `_gain_vel_{p,i,d}` / `MPC_*_VEL_{P,I,D}_ACC` | velocity PID gains |
| $V_{max},V_{up},V_{down}$ | `_lim_vel_*` / `MPC_*_VEL_MAX*` | velocity limits |
| $\theta_{max}$ | `_lim_tilt` / `MPC_TILTMAX_AIR` | maximum tilt (rad) |
| $T_{min},T_{max}$ | `_lim_thr_*` / `MPC_THR_MIN/MAX` | thrust bounds |
| $K_{arw}=2/K_p^{v,x}$ | `arw_gain` | ARW gain (Rundqwist 1990) |

### Operators

| Symbol | Meaning |
|---|---|
| $\odot$ | element-wise multiplication (Hadamard / `emult` in matrix lib) |
| $\boldsymbol{v}^{xy}$ | horizontal component $(v_x,v_y)$ |
| $\hat{\boldsymbol{e}}$ | unit vector |
| $\hat{\boldsymbol{z}}_W=(0,0,1)^\top$ | world z axis (NED, pointing down) |

---

## 3.4. Detailed Formulas

### Step 1 — Position P-controller

$$
\boldsymbol{v}_{sp,P} = \boldsymbol{K}_p^{pos}\odot(\boldsymbol{p}_{sp}-\boldsymbol{p})
$$

$$
\boldsymbol{v}_{sp} \leftarrow \boldsymbol{v}_{sp,P} + \boldsymbol{v}_{sp,FF}
$$

**Variable explanation**:
- $\boldsymbol{p}_{sp}-\boldsymbol{p}$: position error (NED, m).
- $\boldsymbol{K}_p^{pos}=\mathrm{diag}(K_{xy},K_{xy},K_z)$: diagonal P-gain.
- $\boldsymbol{v}_{sp,FF}$: feed-forward velocity (from trajectory generator) — goes directly to output, bypassing P.

Code reference:

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

Parameters: `MPC_XY_P` (~0.95), `MPC_Z_P` (~1.0).

**Horizontal speed limit prioritizes P-term over FF** (`constrainXY`): given $\boldsymbol{v}_0=\boldsymbol{v}_{sp,P}^{xy}$ as priority, find $s\ge 0$ such that $\lVert\boldsymbol{v}_0+s\hat{\boldsymbol{v}}_{1}\rVert\le V_{max}$:

$$
s = -\hat{\boldsymbol{v}}_1\!\cdot\!\boldsymbol{v}_0 + \sqrt{(\hat{\boldsymbol{v}}_1\!\cdot\!\boldsymbol{v}_0)^2 - (\lVert\boldsymbol{v}_0\rVert^2-V_{max}^2)}
$$

(positive root of quadratic equation). Code:

```@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/ControlMath.cpp:172-176
Vector2f u1 = v1.normalized();
float m = u1.dot(v0);
float c = v0.dot(v0) - max * max;
float s = -m + sqrtf(m * m - c);
return v0 + u1 * s;
```

Z axis: $v_{sp,z}\leftarrow\mathrm{clip}(v_{sp,z},-V_{up},V_{down})$.

### Step 2 — Velocity PID-controller

$$
\boldsymbol{e}_v=\boldsymbol{v}_{sp}-\boldsymbol{v}
$$

$$
\boldsymbol{a}_{sp,PID} = \boldsymbol{K}_p^v\odot\boldsymbol{e}_v + \boldsymbol{I}_v - \boldsymbol{K}_d^v\odot\dot{\boldsymbol{v}}
$$

$$
\boldsymbol{a}_{sp}\leftarrow \boldsymbol{a}_{sp,PID}+\boldsymbol{a}_{sp,FF}
$$

**Variable explanation**:
- $\boldsymbol{e}_v$: velocity error (m/s).
- $\boldsymbol{I}_v$: integrator state — accumulates $\boldsymbol{K}_i^v\odot\boldsymbol{e}_v\Delta t$ over time.
- $\dot{\boldsymbol{v}}$: velocity derivative (≈ filtered acceleration, taken from EKF).
- $\boldsymbol{a}_{sp,FF}$: feed-forward acceleration from trajectory.

⚠️ D-term acts on **velocity derivative** $\dot{\boldsymbol{v}}$ (provided by EKF), not on $\dot{\boldsymbol{e}}_v$ → avoids derivative kick (when $\boldsymbol{v}_{sp}$ steps, no $\dot{\boldsymbol{e}}_v$ spike). Code:

```@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp:145-152
// PID velocity control
Vector3f vel_error = _vel_sp - _vel;
Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

// No control input from setpoints or corresponding states which are NAN
ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

_accelerationControl();
```

**Anti-windup Z** (simple): if Z thrust hits its limit and error has the same sign → set $e_{v,z}=0$ before integrating.

```@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp:154-158
// Integrator anti-windup in vertical direction
if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
	vel_error(2) = 0.f;
}
```

**Horizontal Anti-Reset Windup (Rundqwist 1990)**:

$$
\boldsymbol{a}_{prod}^{xy} = \boldsymbol{T}^{xy}\frac{g}{T_h}
$$

$$
\text{When }\lVert\boldsymbol{a}_{sp}^{xy}\rVert > \lVert\boldsymbol{a}_{prod}^{xy}\rVert:\quad
\boldsymbol{e}_v^{xy}\leftarrow\boldsymbol{e}_v^{xy}-K_{arw}(\boldsymbol{a}_{sp}^{xy}-\boldsymbol{a}_{prod}^{xy}),\ K_{arw}=\frac{2}{K_p^{v,x}}.
$$

**Variable explanation**:
- $\boldsymbol{a}_{prod}^{xy}$: horizontal acceleration **actually producible** with the current thrust (after being saturated by tilt-max + thrust-max).
- $\boldsymbol{a}_{sp}^{xy}-\boldsymbol{a}_{prod}^{xy}$: the portion of acceleration "cut off" by saturation — ARW feeds this back into $\boldsymbol{e}_v$ to prevent integrator wind-up.
- $K_{arw}=2/K_p^{v,x}$: back-calculation gain per the standard Rundqwist formula (~$2/K_p$).
- When not saturated ($\boldsymbol{a}_{sp}^{xy}\le\boldsymbol{a}_{prod}^{xy}$): ARW does not run, integrator accumulates normally.

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

Euler integration:

$$
\boldsymbol{I}_v^{(k+1)}=\boldsymbol{I}_v^{(k)}+\boldsymbol{K}_i^v\odot\boldsymbol{e}_v\Delta t,\quad I_{v,z}\in[-g,g].
$$

- Note: $\boldsymbol{e}_v$ here is **already modified** by Z anti-windup (set to 0 when saturated) and horizontal ARW (subtracted back-calculation) before integration.
- Clamp $|I_{v,z}|\le g$ prevents the integrator from consuming the entire thrust budget.

Parameters: `MPC_{XY,Z}_VEL_{P,I,D}_ACC`.

### Step 3 — Acceleration → Thrust Vector

Required acceleration (specific force):

$$
\boldsymbol{a}_{cmd}=\boldsymbol{a}_{sp}-\boldsymbol{g}_W=\begin{bmatrix}a_{sp,x}\\a_{sp,y}\\a_{sp,z}+g\end{bmatrix}
$$

**Variable explanation**:
- $\boldsymbol{a}_{sp}$: desired acceleration in $\{W\}$ NED.
- $\boldsymbol{g}_W=(0,0,g)^\top$ — in NED, $g$ is positive (pointing down).
- $\boldsymbol{a}_{cmd}$: specific force = non-gravitational force / mass — what thrust must produce.
- The minus sign on the z axis becomes plus because $-(g\cdot 1)= -g$ → at hover ($a_{sp,z}=0$): $a_{cmd,z}=-g<0$, pointing up.

Desired body Z axis (NED, thrust direction upward = $-\hat{\boldsymbol{z}}_W$):

$$
\hat{\boldsymbol{z}}_B^* = -\boldsymbol{a}_{cmd}/\lVert\boldsymbol{a}_{cmd}\rVert
$$

- Minus sign because thrust pushes along $-\hat{\boldsymbol{z}}_B$ (rotor pushes down → reaction force pushes body up).
- $\hat{\boldsymbol{z}}_B^*$: raw body z axis (before tilt limit).

**Decouple flag** (`MPC_ACC_DECOUPLE`): if enabled, uses fixed $z_{spec}=-g$, ignores $a_{sp,z}$ when computing tilt → avoids tilting due to vertical acceleration.

**Tilt limit** (`limitTilt`):

$$
\theta = \min(\arccos(\hat{\boldsymbol{z}}_B^*\!\cdot\!\hat{\boldsymbol{z}}_W), \theta_{max})
$$

$$
\hat{\boldsymbol{z}}_B^\dagger = \cos\theta\,\hat{\boldsymbol{z}}_W + \sin\theta\,\hat{\boldsymbol{r}},\quad
\hat{\boldsymbol{r}}=\frac{\hat{\boldsymbol{z}}_B^*-(\hat{\boldsymbol{z}}_B^*\!\cdot\!\hat{\boldsymbol{z}}_W)\hat{\boldsymbol{z}}_W}{\lVert\cdot\rVert}
$$

- $\theta$: total tilt angle (rad), clipped to $\theta_{max}$.
- $\hat{\boldsymbol{r}}$: horizontal component (orthogonal to $\hat{\boldsymbol{z}}_W$) of $\hat{\boldsymbol{z}}_B^*$, normalized — the tilt rotation axis.
- $\hat{\boldsymbol{z}}_B^\dagger$: body z axis after limit, preserving tilt direction but constraining magnitude.

**Convert acceleration → thrust** via hover thrust:

$$
T_z^{NED} = a_{sp,z}\frac{T_h}{g} - T_h
$$

- $T_h$: normalized hover thrust (= $T$ when $a_z=0$).
- At hover ($a_{sp,z}=0$): $T_z^{NED}=-T_h$ (negative because thrust opposes $\hat{\boldsymbol{z}}_W$).
- Formula is symmetric around hover: positive acceleration ($a_{sp,z}>0$, wanting to descend) → $|T_z^{NED}|<T_h$ (reduced thrust); and vice versa.

Project onto the limited body axis:

$$
T_{coll}=\min\!\left(\frac{T_z^{NED}}{\hat{\boldsymbol{z}}_W\!\cdot\!\hat{\boldsymbol{z}}_B^\dagger},-T_{min}\right),\quad
\boldsymbol{T}=T_{coll}\,\hat{\boldsymbol{z}}_B^\dagger
$$

- $\hat{\boldsymbol{z}}_W\!\cdot\!\hat{\boldsymbol{z}}_B^\dagger=\cos\theta$: thrust projection efficiency onto the vertical axis (decreases with cosine as tilt increases).
- Dividing by $\cos\theta$: increases thrust magnitude to maintain the required vertical force when tilted.
- $\min(\dots,-T_{min})$: ensures thrust is large enough (negative in NED direction), preventing it from falling below $T_{min}$.

Code (full `_accelerationControl`):

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

**Thrust saturation prioritizes vertical**:

$$
T_{xy,alloc}=\min(\lVert\boldsymbol{T}^{xy}\rVert,M_{xy}),\quad T_z\ge -\sqrt{T_{max}^2-T_{xy,alloc}^2}
$$

$$
T_{xy,max}=\sqrt{T_{max}^2-T_z^2}
$$

$$
\boldsymbol{T}^{xy}\leftarrow \boldsymbol{T}^{xy}\frac{T_{xy,max}}{\lVert\boldsymbol{T}^{xy}\rVert}\quad(\text{if exceeded})
$$

- $M_{xy}$: horizontal margin (`MPC_THR_XY_MARG`).
- $T_{xy,alloc}$: horizontal thrust pre-allocated.
- $T_z$ is given enough budget so that $\lVert\boldsymbol{T}\rVert\le T_{max}$ (altitude tracking is priority #1).
- Once $T_z$ is fixed, $T_{xy,max}$ is allocated from the remaining budget — horizontal thrust is scaled down if needed.

### Step 4 — Thrust Vector → Quaternion (`ControlMath::thrustToAttitude`)

$$
\hat{\boldsymbol{z}}_B = -\boldsymbol{T}/\lVert\boldsymbol{T}\rVert
$$

$$
\boldsymbol{y}_C=(-\sin\psi_{sp},\cos\psi_{sp},0)^\top
$$

$$
\hat{\boldsymbol{x}}_B = \frac{\boldsymbol{y}_C\times\hat{\boldsymbol{z}}_B}{\lVert\boldsymbol{y}_C\times\hat{\boldsymbol{z}}_B\rVert},\quad
\hat{\boldsymbol{y}}_B = \hat{\boldsymbol{z}}_B\times\hat{\boldsymbol{x}}_B
$$

$$
\boldsymbol{R}_{sp}=[\hat{\boldsymbol{x}}_B\ \hat{\boldsymbol{y}}_B\ \hat{\boldsymbol{z}}_B]\Rightarrow\boldsymbol{q}_{sp}
$$

**Variable explanation**:
- $\hat{\boldsymbol{z}}_B$: final body z axis (after saturation), sign-flipped from thrust which points along $-\hat{\boldsymbol{z}}_B$ in NED.
- $\boldsymbol{y}_C$: $\hat{\boldsymbol{y}}$ vector in the **C-frame** (yaw-only intermediate frame) — horizontal plane, rotated by $\psi_{sp}$. The formula $(-\sin\psi,\cos\psi,0)$ is exactly $\hat{\boldsymbol{e}}_y$ rotated around $\hat{\boldsymbol{z}}_W$ by angle $\psi_{sp}$.
- $\hat{\boldsymbol{x}}_B$: orthogonal to both $\boldsymbol{y}_C$ and $\hat{\boldsymbol{z}}_B$ — the body nose forward direction, aligned with the desired yaw.
- $\hat{\boldsymbol{y}}_B$: completes the right-handed frame via cross product.
- $\boldsymbol{R}_{sp}=[\hat{\boldsymbol{x}}_B\ \hat{\boldsymbol{y}}_B\ \hat{\boldsymbol{z}}_B]$ — columns are body axes expressed in $\{W\}$ ⇒ this is $\boldsymbol{R}_{WB}$.

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

## 3.5. Theoretical Background & Keywords

| Topic | Keywords |
|---|---|
| Cascaded P/PID | `cascade control`, `inner-outer loop tuning`, `Skogestad SIMC` |
| Anti-windup | `back-calculation anti-windup`, `Rundqwist 1990 anti-reset windup`, `tracking anti-windup`, `conditional integration` |
| Geometric control | `Lee Leok McClamroch geometric tracking control SE(3)`, `flatness quadrotor`, `Mellinger Kumar minimum-snap` |
| Thrust vectoring | `differential flatness`, `thrust direction control`, `tilt-prioritized control` |
| Feed-forward design | `2-DOF controller`, `feed-forward + feedback structure` |
| Saturation handling | `prioritized control allocation`, `command shaping`, `reference governor` |

### Foundational Papers
- Lee, Leok, McClamroch (2010) — *Geometric Tracking Control of a Quadrotor UAV on SE(3)*. arXiv:1003.2005.
- Mellinger & Kumar (2011) — *Minimum Snap Trajectory Generation and Control for Quadrotors*.
- L. Rundqwist (1990) — *Anti-reset Windup for PID Controllers*.
- Faessler, Falanga, Scaramuzza (2018) — *Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight*.

### Extended References
- PX4 doc: <https://docs.px4.io/main/en/flight_stack/controller_diagrams.html>.
- Astrom & Hagglund, *PID Controllers: Theory, Design, and Tuning* (anti-windup chapter).

## 3.6. Important PX4 Parameters

| Parameter | Meaning |
|---|---|
| `MPC_XY_P`, `MPC_Z_P` | Position P-gain |
| `MPC_XY_VEL_P/I/D_ACC`, `MPC_Z_VEL_P/I/D_ACC` | Velocity PID gains |
| `MPC_XY_VEL_MAX`, `MPC_Z_VEL_MAX_UP/DN` | Velocity limits |
| `MPC_TILTMAX_AIR`, `MPC_TILTMAX_LND` | Tilt limits |
| `MPC_THR_HOVER`, `MPC_THR_MIN/MAX` | Hover thrust + thrust bounds |
| `MPC_THR_XY_MARG` | Horizontal margin when prioritizing vertical |
| `MPC_USE_HTE` | Enable adaptive hover thrust |
| `MPC_ACC_DECOUPLE` | Decouple tilt from a_z |

## 3.7. Debug Tips
- Log: `vehicle_local_position_setpoint` shows each component (x_sp, vx_sp, ax_sp, thrust) → compare with `vehicle_local_position` to check tracking error.
- If horizontal tracking is poor: check `MPC_THR_HOVER`/HTE before increasing gain.
- If vertical oscillation on landing: usually D-term + accel noise — reduce `MPC_Z_VEL_D_ACC` or increase `IMU_ACCEL_CUTOFF`.
