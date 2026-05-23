# 6. Control Allocation (Mathematical Mixer)

## 6.1. Role

Converts the 6-axis command vector $\boldsymbol{c}=[\boldsymbol{\tau};\boldsymbol{T}]\in\mathbb{R}^6$ (3 torque + 3 thrust components) into $n_m$ motor signals $\boldsymbol{u}\in[0,1]^{n_m}$. This is the **next-generation mixer** in PX4 replacing the old `mixer_module` — it separates the allocation algorithm from the geometric configuration.

## 6.2. Main Code

| Role | File |
|---|---|
| Module wrapper | `@/home/frank/tf-px4/src/modules/control_allocator/ControlAllocator.cpp` |
| Header | `@/home/frank/tf-px4/src/modules/control_allocator/ControlAllocator.hpp` |
| Effectiveness by airframe | `@/home/frank/tf-px4/src/modules/control_allocator/VehicleActuatorEffectiveness/` |
| Base allocation | `@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocation.cpp` |
| **Pseudo-inverse** | `@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp` |
| **Sequential desaturation** | `@/home/frank/tf-px4/src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp` |
| Effectiveness lib | `@/home/frank/tf-px4/src/lib/control_allocation/actuator_effectiveness/` |
| Generic inverse (Greville/Moore-Penrose) | `@/home/frank/tf-px4/src/lib/matrix/matrix/PseudoInverse.hpp` (`matrix::geninv`) |

## 6.3. Symbol Table

### Main Variables

| Symbol | Code | Meaning |
|---|---|---|
| $\boldsymbol{c}=[\boldsymbol{\tau};\boldsymbol{T}]\in\mathbb{R}^6$ | `_control_sp` | input command (3 torque + 3 thrust comp.), normalized $\in[-1,1]$ |
| $\boldsymbol{u}\in\mathbb{R}^{n_m}$ | `_actuator_sp` | output per actuator $\in[u_{min},u_{max}]$ |
| $n_m$ | `_num_actuators` | number of motors/control surfaces |
| $\boldsymbol{B}\in\mathbb{R}^{6\times n_m}$ | `_effectiveness` | effectiveness matrix (generated from airframe geometry) |
| $\boldsymbol{B}^+\in\mathbb{R}^{n_m\times 6}$ | `_mix` | normalized pseudo-inverse |
| $\boldsymbol{u}_{trim}$ | `_actuator_trim` | working point (e.g. `0.5` for hover quad) |
| $\boldsymbol{c}_{trim}$ | `_control_trim` | command corresponding to $\boldsymbol{u}_{trim}$ |

### Actuator Model Parameters

| Symbol | Meaning |
|---|---|
| $r$ | motor arm radius (m) |
| $\theta_i$ | angular position of motor $i$ in the body x-y plane |
| $c_m$ | drag moment coefficient (yaw / thrust ratio, $\sim 0.01$) |
| $\sigma_i\in\{+1,-1\}$ | rotor spin direction (CW=$-$, CCW=$+$) |

### Saturation Handling

| Symbol | Code | Meaning |
|---|---|---|
| $\boldsymbol{d}\in\mathbb{R}^{n_m}$ | `desaturation_vector` | "shift" direction in the nullspace of $\boldsymbol{B}$ (does not change $\boldsymbol{c}$) |
| $k$ | `gain` | SD gain found to push $\boldsymbol{u}$ back to feasible |
| $u_{min,i},u_{max,i}$ | `_actuator_min/max` | bounds for motor $i$ |
| $\boldsymbol{c}_{achieved}=\boldsymbol{B}\boldsymbol{u}_{clipped}$ | derived | command actually produced after clipping |
| $\boldsymbol{c}_{unalloc}=\boldsymbol{c}-\boldsymbol{c}_{achieved}$ | `unallocated_torque/thrust` | command that CANNOT be produced, fed back to rate controller (§5) |

### Operators

| Symbol | Meaning |
|---|---|
| $\boldsymbol{B}^+$ | Moore-Penrose pseudo-inverse |
| $\mathrm{geninv}$ | Greville recursive PI (handles rank-deficient cases) |
| $\mathrm{Null}(\boldsymbol{B})$ | nullspace = $\{\boldsymbol{u}:\boldsymbol{B}\boldsymbol{u}=\boldsymbol{0}\}$ |

---

## 6.4. Mathematical Problem

Normalized command vector:

$$
\boldsymbol{c}=\begin{bmatrix}\tau_x\\ \tau_y\\ \tau_z\\ T_x\\ T_y\\ T_z\end{bmatrix}\in[-1,1]^6
$$

Output per motor $\boldsymbol{u}\in[u_{min},u_{max}]^{n_m}$ (default $[0,1]$).

Linear actuator model:

$$
\boldsymbol{c} = \boldsymbol{B}\boldsymbol{u}
$$

$\boldsymbol{B}\in\mathbb{R}^{6\times n_m}$ is the **effectiveness matrix** generated from the airframe geometry.

**Variable explanation**:
- $\boldsymbol{c}=(\tau_x,\tau_y,\tau_z,T_x,T_y,T_z)^\top$: 6-DoF command normalized to $[-1,1]$.
- $\boldsymbol{u}=(u_0,\dots,u_{n_m-1})^\top$: motor values (PWM-equivalent), default $\in[0,1]$.
- $B_{ji}$: how much axis $j$ (among the 6 axes) changes when motor $i$ increases by 1 unit — quantifies "who affects what".

### Example: Quadcopter X (motors numbered 0..3 at 4 corners)

Given:
- Motor $i$ position: $(r\cos\theta_i, r\sin\theta_i, 0)$ in body, $\theta_i\in\{45°,135°,225°,315°\}$.
- Thrust direction: $-\hat{\boldsymbol{z}}_B$.
- Drag moment (yaw): coefficient $c_m$, alternating CW/CCW sign.

Column $i$ of $\boldsymbol{B}$:

$$
\boldsymbol{B}_{:,i} = \begin{bmatrix}
-r\sin\theta_i & \text{(roll)}\\
\phantom{-}r\cos\theta_i & \text{(pitch)}\\
\sigma_i\,c_m & \text{(yaw, }\sigma_i=\pm1\text{)}\\
0 & \text{(T_x)}\\
0 & \text{(T_y)}\\
-1 & \text{(T_z, upward thrust)}
\end{bmatrix}
$$

**Row-by-row explanation**:
- Row 1 (roll = $\tau_x$): motors farther from the x-axis (larger $|\sin\theta_i|$) generate more roll torque; sign follows $-\sin\theta_i$ because thrust is in the $-\hat{z}_B$ direction × moment arm $\hat{y}\sin\theta_i$.
- Row 2 (pitch): similar, about the y-axis, sign $+\cos\theta_i$.
- Row 3 (yaw = $\tau_z$): aerodynamic reaction moment from the rotor, sign follows spin direction $\sigma_i$, magnitude = $c_m$ (~ 1/100 of thrust).
- Rows 4-5 (T_x, T_y): = 0 because the quad only thrusts along $\hat{z}_B$, no lateral thrust (unlike tilt-rotors).
- Row 6 (T_z): = $-1$ — thrust along $-\hat{z}_B$ ($T_z$ negative in NED-like body z pointing down).

$\boldsymbol{B}$ is generated in `ActuatorEffectivenessRotors.cpp`.

## 6.5. Moore–Penrose Pseudo-inverse

When $n_m\ge 6$ and $\boldsymbol{B}$ has full rank:

$$
\boldsymbol{B}^+ = \boldsymbol{B}^\top(\boldsymbol{B}\boldsymbol{B}^\top)^{-1}
$$

**Variable explanation**:
- $\boldsymbol{B}\boldsymbol{B}^\top\in\mathbb{R}^{6\times 6}$: invertible when $\boldsymbol{B}$ is full row-rank (enough motors to control 6 axes).
- $\boldsymbol{B}^+$: "inverse" matrix such that $\boldsymbol{B}\boldsymbol{B}^+ = \boldsymbol{I}_6$ (left-inverse only when over-actuated).
- This form applies to hex/octo (≥ 6 motors).

For $n_m<6$ (e.g. quad with 4 motors < 6 axes), the code uses `matrix::geninv` — Greville's recursive Moore-Penrose, which works even for rank-deficient cases.

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

Allocation law:

$$
\boxed{\ \boldsymbol{u} = \boldsymbol{u}_{trim} + \boldsymbol{B}^+(\boldsymbol{c}-\boldsymbol{c}_{trim})\ }
$$

**Variable explanation**:
- $\boldsymbol{u}_{trim}$: working point — for a hovering quad, equivalent to the hover thrust coefficient (~ 0.5 by default).
- $\boldsymbol{c}_{trim}$: command produced at $\boldsymbol{u}_{trim}$ (value $T_z=-T_h$, others = 0).
- $\boldsymbol{c}-\boldsymbol{c}_{trim}$: "delta command" relative to trim → multiplied by $\boldsymbol{B}^+$ to get the required motor change.
- Added to $\boldsymbol{u}_{trim}$ → final PWM.
- Trim benefit: linearizes around hover → small delta, gain aligns with correct ESC sensitivity.

Cross-reference with code (`allocate()`):

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

`_mix` = normalized $\boldsymbol{B}^+$.

### Moore-Penrose Properties
$\boldsymbol{u}=\boldsymbol{B}^+\boldsymbol{c}$ is the **minimum-energy solution** ($\ell_2$ norm) in the space:

$$
\min_\boldsymbol{u}\lVert\boldsymbol{u}\rVert^2\quad\text{s.t.}\quad \boldsymbol{B}\boldsymbol{u}=\boldsymbol{c}
$$

- If under-actuated ($n_m\ge 6$, $\boldsymbol{B}$ full row-rank): infinitely many solutions, PI selects the one with smallest $\lVert\boldsymbol{u}\rVert$ (minimum motor effort).
- When over-determined ($n_m<6$, rank<6) → NO exact solution, PI gives a **least-squares** approximation $\min\lVert\boldsymbol{B}\boldsymbol{u}-\boldsymbol{c}\rVert^2$ — "best pursuit" of the command, with the residual marked as $\boldsymbol{c}_{unalloc}$.

## 6.6. Column Normalization of $\boldsymbol{B}^+$

Purpose: the same value $\tau_x=1$ must produce the same "total motor deviation" regardless of the number of motors (4-quad vs 6-hex).

- Roll/Pitch (same scale): $\sqrt{\lVert\boldsymbol{B}^+_{:,0}\rVert^2/(n_{nz}/2)}$.
- Yaw: $\max_i|B^+_{i,2}|$.
- Thrust per axis: $\frac{1}{n_{nz}}\sum_i|B^+_{i,3+axis}|$.

**Variable explanation**:
- $\boldsymbol{B}^+_{:,j}$: column $j$ of $\boldsymbol{B}^+$ — shows how the command on axis $j$ (e.g. roll) is distributed to each motor.
- $n_{nz}$: number of motors with nonzero effectiveness for that axis.
- Roll/pitch use RMS-norm for normalization (for clean symmetry); yaw uses max-norm because $c_m$ is small and needs a larger range.
- Thrust uses mean-norm so that the total motor command always has the same magnitude as $T_z$.

Then zero out elements where $|B^+_{ij}|<10^{-3}$ to avoid numerical noise.

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

## 6.7. Sequential Desaturation (SD) — quad/hex with insufficient DoF

File: `ControlAllocationSequentialDesaturation.cpp`. Idea: add a vector in the nullspace of $\boldsymbol{B}$ to $\boldsymbol{u}$ (without changing $\boldsymbol{c}=\boldsymbol{B}\boldsymbol{u}$) so that $\boldsymbol{u}$ returns to the feasible region $[u_{min},u_{max}]$. When this is not possible, **sacrifice** axes in priority order.

### `computeDesaturationGain` Function

Given desaturation vector $\boldsymbol{d}$ (typically the thrust column, or a nullspace column), find gain $k$:

For each saturated motor $u_i<u_{min,i}$:

$$
k_i = \frac{u_{min,i}-u_i}{d_i}
$$

Similarly for $u_i>u_{max,i}$. Take:

$$
k = k_{min}+k_{max}\quad(\text{reduce total saturation}).
$$

**Variable explanation**:
- $\boldsymbol{d}$: vector indicating the "push" direction for $\boldsymbol{u}$ — if $\boldsymbol{d}\in\mathrm{Null}(\boldsymbol{B})$ then $\boldsymbol{c}$ remains unchanged ("free move"); otherwise (e.g. thrust column) it sacrifices the $T_z$ axis to rescue tilt.
- $k_i$: gain needed to bring motor $i$ exactly to its bound (`min` or `max`).
- $k_{min},k_{max}$: smallest negative and largest positive gain across all motors — balances both directions (if one motor saturates high while another saturates low, averages them out).
- Ignore actuators with $|d_i|<0.2$ (weak effectiveness) — avoids near-zero division causing gain spikes.

Apply:

$$
\boldsymbol{u}\leftarrow\boldsymbol{u}+k\boldsymbol{d}
$$

Repeat once with $k\leftarrow 0.5\,k_{new}$ for stable convergence ("two-step bisection").

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

### Airmode (priority mode)

`MC_AIRMODE` selects:
- `0` (disabled): when thrust is insufficient, *reduce tilt response* (prioritize thrust). Drone lands more softly.
- `1` (Roll/Pitch): allows thrust to vary in order to maintain tilt response.
- `2` (Roll/Pitch/Yaw): also allows yaw — most aggressive, used for acro/race.

Each `mixAirmode*` function consists of a sequence of `desaturateActuators` calls with different desaturation vectors (yaw column, thrust column, ...).

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

After allocation, compute:

$$
\boldsymbol{c}_{achieved} = \boldsymbol{B}\boldsymbol{u}_{clipped}
$$

$$
\boldsymbol{c}_{unalloc} = \boldsymbol{c}-\boldsymbol{c}_{achieved}
$$

**Variable explanation**:
- $\boldsymbol{u}_{clipped}=\mathrm{clip}(\boldsymbol{u},u_{min},u_{max})$: motor command after clipping to the allowed range (may reduce $\boldsymbol{u}$).
- $\boldsymbol{c}_{achieved}$: command actually produced — multiplied back by $\boldsymbol{B}$ for verification.
- $\boldsymbol{c}_{unalloc}\ne 0$ means saturation occurred — fed back to §5 anti-windup.

Publish `control_allocator_status.unallocated_torque/thrust` → rate controller uses this for anti-windup §5.4(a).

## 6.8. Motor Slew-Rate Limiting

Before outputting:

$$
u_i^{(k)}\leftarrow u_i^{(k-1)}+\mathrm{clip}\!\left(u_i^{(k)}-u_i^{(k-1)},-r_i\Delta t,\ r_i\Delta t\right)
$$

**Variable explanation**:
- $u_i^{(k)},u_i^{(k-1)}$: motor command at current and previous step.
- $r_i$ = `CA_R{i}_SLEW`: maximum rate of change (1/s, e.g. 0.4/s).
- $r_i\Delta t$: maximum change per step — larger steps are clipped.
- Protects ESCs from step jumps, avoids mechanical resonance, reduces current spikes.

## 6.9. Theoretical Background & Keywords

| Topic | Keywords |
|---|---|
| Pseudo-inverse | `Moore-Penrose pseudoinverse`, `Greville recursive algorithm`, `weighted least squares` |
| Control allocation | `Bodson 2002 control allocation evaluation`, `Härkegård quadratic programming allocation`, `daisy chain allocation`, `direct allocation` |
| Sequential desaturation | `prioritized control allocation`, `bisection desaturation`, `null space redistribution` |
| Effectiveness matrix | `B-matrix UAV`, `actuator effectiveness model`, `motor torque coefficient` |
| Aggressive flight allocation | `Brescianini Hehn D'Andrea quadcopter trajectory tracking`, `Faessler thrust mixing 2017` |
| Aerospace allocation | `aircraft pseudo-inverse mixing`, `thrust vectoring allocation`, `redundant actuator management` |
| Saturation handling | `saturation-aware MPC`, `anti-windup with allocation feedback` |

### Foundational Papers
- Bodson (2002) — *Evaluation of Optimization Methods for Control Allocation*. JGCD.
- Härkegård (2002) — *Efficient Active Set Algorithms for Solving Constrained Least Squares Problems in Aircraft Control Allocation*.
- Johansen & Fossen (2013) — *Control Allocation - A Survey*. Automatica (read for an overview).
- Faessler, Falanga, Scaramuzza (2017) — *Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight*.

### Books
- Oppenheimer, Doman, Bolender, *Control Allocation* — chapter in *The Control Handbook*.

## 6.10. PX4 Parameters

| Parameter | Meaning |
|---|---|
| `CA_AIRFRAME` | Airframe type (quad, hex, ...) |
| `CA_METHOD` | Allocation method (0=PseudoInverse, 1=SequentialDesaturation) |
| `CA_R{0..N}_*` | Motor i parameters (position, tilt angle, ...) |
| `CA_R{i}_SLEW` | Slew-rate for motor i |
| `MC_AIRMODE` | Desaturation strategy |

## 6.11. Debug Tips
- Log `actuator_motors.control[i]` to check if any motor is saturating.
- Log `control_allocator_status.unallocated_torque/thrust` to detect insufficient authority.
- If yaw is "weak" at high thrust: default airmode 0 prioritizes thrust → switch to 1/2 if needed.
- Calibrate `c_m` (drag moment coefficient): if yaw drifts, `CA_R*_KM` may be incorrect.
