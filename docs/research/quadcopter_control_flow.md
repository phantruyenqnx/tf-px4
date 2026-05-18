# Quadcopter Controller Architecture in PX4 (tf-px4)

This document describes the complete control stack of a multicopter in the PX4 `src/` codebase. Goal: understand the **flow** from "user intent / mission" to **PWM commands to motors**, know the root files of each layer, and grasp the core algorithm of each block.

---

## 1. Architecture Overview

PX4 uses a **cascaded control** architecture running on uORB pub/sub. Each layer is a separate module, running on a **work queue** (event-driven, not a dedicated thread), triggered by a topic from the layer above or by a new sensor sample.

```
┌─────────────────────────────────────────────────────────────────────┐
│  User / GCS / Mission                                                │
│   - manual_control_setpoint (RC)                                     │
│   - vehicle_command (MAVLink / offboard)                             │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  navigator (mission, RTL, loiter, takeoff/land auto)                 │
│  src/modules/navigator                                               │
│  → publish: position_setpoint_triplet                                │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  flight_mode_manager  (selects FlightTask based on nav_state)        │
│  src/modules/flight_mode_manager/FlightModeManager.cpp               │
│  Tasks: ManualPosition, ManualAltitude, Auto, Orbit, Descend, ...    │
│  → publish: trajectory_setpoint, vehicle_constraints                 │
└──────────────────────────┬──────────────────────────────────────────┘
                           │ trajectory_setpoint (pos/vel/acc/yaw)
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  mc_pos_control  (Position + Velocity P/PID, outermost loop)         │
│  src/modules/mc_pos_control/MulticopterPositionControl.cpp           │
│   └─ core: PositionControl/PositionControl.cpp                       │
│  Input : vehicle_local_position + trajectory_setpoint                │
│  Output: vehicle_attitude_setpoint (q_d, thrust_body)                │
└──────────────────────────┬──────────────────────────────────────────┘
                           │ vehicle_attitude_setpoint
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  mc_att_control  (Attitude P-controller, quaternion)                 │
│  src/modules/mc_att_control/mc_att_control_main.cpp                  │
│   └─ core: AttitudeControl/AttitudeControl.cpp                       │
│  Input : vehicle_attitude (q) + vehicle_attitude_setpoint            │
│  Output: vehicle_rates_setpoint (ω_d, thrust_body)                   │
└──────────────────────────┬──────────────────────────────────────────┘
                           │ vehicle_rates_setpoint
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  mc_rate_control (Rate PID, innermost loop - runs fastest)           │
│  src/modules/mc_rate_control/MulticopterRateControl.cpp              │
│   └─ core: src/lib/rate_control/rate_control.cpp                     │
│  Input : vehicle_angular_velocity (gyro) + vehicle_rates_setpoint    │
│  Output: vehicle_torque_setpoint, vehicle_thrust_setpoint            │
└──────────────────────────┬──────────────────────────────────────────┘
                           │ torque + thrust setpoints (normalised [-1,1])
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  control_allocator  (mixer: torque/thrust → individual motors)       │
│  src/modules/control_allocator/ControlAllocator.cpp                  │
│   └─ algo: src/lib/control_allocation/* (pseudo-inverse + clipping)  │
│  Output: actuator_motors, actuator_servos                            │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  pwm_out / dshot / uavcan driver (src/lib/mixer_module + drivers/)   │
│  → PWM/DShot signals to ESC → motors                                 │
└─────────────────────────────────────────────────────────────────────┘
```

The feedback branch (state estimation) runs in parallel:

```
sensors (gyro, accel, mag, baro, GPS)
   → src/modules/sensors  (filtering, calibration)
   → src/modules/ekf2     (EKF: vehicle_attitude, vehicle_local_position,
                                  vehicle_angular_velocity, ...)
```

EKF2 is the "source of truth" about the vehicle state; all controller layers read its topics.

---

## 2. Flow from Root to Leaf (file by file)

### 2.1. Command Sources
- **RC / manual**: `src/modules/manual_control` reads raw RC, publishes `manual_control_setpoint`.
- **Mission / offboard**: `src/modules/mavlink` (MAVLink) or `src/modules/uxrce_dds_client` (uXRCE-DDS / ROS 2) → publish `vehicle_command`, offboard `trajectory_setpoint`, ...
- **Automatic mission**: `src/modules/navigator` processes mission items, RTL, loiter; outputs `position_setpoint_triplet`.

### 2.2. `flight_mode_manager` — setpoint generation method selection
Main files:
- `@/home/frank/tf-px4/src/modules/flight_mode_manager/FlightModeManager.cpp`
- `@/home/frank/tf-px4/src/modules/flight_mode_manager/FlightModeManager.hpp`

Mechanism: based on `vehicle_status.nav_state` and `vehicle_control_mode`, FMM selects a `FlightTask` (state pattern). Each task lives in `src/modules/flight_mode_manager/tasks/`:

- `FlightTask/` — base class.
- `ManualAltitude/`, `ManualPosition/`, `ManualAcceleration/` — manual modes (Stabilized/Altitude/Position).
- `Auto/`, `AutoFollowTarget/`, `Orbit/`, `Descend/`, `Failsafe/`, `Transition/` — autonomous modes.
- `Utility/` — trajectory smoothing (jerk-limited), waypoint handling, ...

Each task `update(dt)` produces a `trajectory_setpoint_s` (pos, vel, acc, yaw, yawspeed — any unused field set to `NaN`). FMM publishes the `trajectory_setpoint` and `vehicle_constraints` topics. This is the **core protocol** between the "setpoint generation layer" and the "setpoint tracking layer".

Smoothing is performed in `src/lib/motion_planning/` (jerk-limited trajectories) — the algorithmic basis for smooth throttle/stick input.

### 2.3. `mc_pos_control` — outer loop (Position + Velocity)

Files:
- Module wrapper (uORB I/O): `@/home/frank/tf-px4/src/modules/mc_pos_control/MulticopterPositionControl.cpp`
- Header: `@/home/frank/tf-px4/src/modules/mc_pos_control/MulticopterPositionControl.hpp`
- **Algorithm core**: `@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp`

Support:
- `Takeoff/Takeoff.cpp` — state machine for spool-up + thrust ramp during takeoff.
- `GotoControl/` — handles smooth "goto setpoint".
- `PositionControl/ControlMath.cpp` — converts thrust vector → attitude quaternion.

Algorithm (see `PositionControl::update` function):
1. **Position P-controller** (`_positionControl`): `v_sp = Kp_pos * (p_sp - p) + v_ff`, then clamp to velocity limits.
2. **Velocity PID-controller** (`_velocityControl`): `a_sp = Kp_v*(v_sp - v) + Ki_v*∫err dt - Kd_v*ȧ + a_ff` with anti-windup when thrust limit is hit.
3. **Acceleration → Thrust → Attitude** (`_accelerationControl` + `ControlMath::thrustToAttitude`):
   - Desired thrust force (vector in world frame): `T = m*(a_sp + g·ẑ)`, expressed as *normalized thrust* via the `MPC_THR_HOVER` parameter (or the `mc_hover_thrust_estimator`).
   - Tilt is limited (`MPC_TILTMAX_AIR`).
   - Attitude quaternion `q_d` is constructed so that the body Z-axis aligns with the thrust vector direction, combined with `yaw_sp`.
4. Output: `vehicle_attitude_setpoint` (contains `q_d` + `thrust_body[3]`) and `vehicle_local_position_setpoint` (for logging/land detector).

Core concept: **cascaded P (position) → PID (velocity) → vector thrust → quaternion**.

### 2.4. `mc_att_control` — middle loop (Attitude)

Files:
- Wrapper: `@/home/frank/tf-px4/src/modules/mc_att_control/mc_att_control_main.cpp`
- Header: `@/home/frank/tf-px4/src/modules/mc_att_control/mc_att_control.hpp`
- **Core**: `@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`

In Manual/Stabilized mode (no pos/vel control), this module generates `vehicle_attitude_setpoint` directly from sticks (`generate_attitude_setpoint`). When position control is active, it receives the setpoint from `mc_pos_control`.

Core algorithm (`AttitudeControl::update(q)`):
- **Quaternion attitude controller** based on the paper *"Nonlinear Quadrocopter Attitude Control"* — Brescianini, Hehn, D'Andrea, ETH 2013.
- Idea: decouple *tilt* (roll/pitch) priority from *yaw* so that when yaw is far off, the drone still maintains the correct thrust direction (avoids altitude loss).
  1. Compute `q_red` that corrects only the Z-axis (tilt) — full priority.
  2. Compute `q_dyaw` for the yaw component, scaled by `MC_YAW_WEIGHT < 1` → blended desired attitude `q_mix = q_red * q_yaw_scaled`.
  3. Error `q_e = q^{-1} * q_mix`; rate setpoint: `ω_sp = 2 * sign(q_e0) * Kp ⊗ q_e.imag` (sin(α/2)·axis).
  4. Add feed-forward `yawspeed_setpoint` (projected into body frame).
  5. Clamp to `MC_{ROLL,PITCH,YAW}RATE_MAX`.
- Output: `vehicle_rates_setpoint` (ω_d) — this is a **P-controller only**, no I/D at this layer.

### 2.5. `mc_rate_control` — inner loop (Rate, runs fastest)

Files:
- Wrapper: `@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.cpp`
- Header: `@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.hpp`
- **Core**: `@/home/frank/tf-px4/src/lib/rate_control/rate_control.cpp` (shared lib used by both MC and FW).

This module is a **gyro work item** (`_vehicle_angular_velocity_sub.registerCallback()`), meaning it runs on every new gyro sample (~ 1 kHz depending on IMU). This is the fastest and most stability-critical loop.

Algorithm (`RateControl::update`):
- **Parallel-form PID with feed-forward**:
  ```
  τ = Kp ⊙ (ω_sp - ω) + ∫Ki·err dt − Kd ⊙ α  + Kff ⊙ ω_sp
  ```
  where `α = ω̇` (angular rate derivative, provided by EKF as `xyz_derivative`).
- Parameters are mapped through an "ideal" form (`K * [1 + 1/(Ti·s) + Td·s]`) for easier manual tuning (`MC_*RATE_K/P/I/D`).
- **Two-layer anti-windup**:
  1. Saturation feedback from `control_allocator_status` (if a motor is saturated on a given axis, stop integrating in that direction).
  2. Reduce I gain when rate error is large (`i_factor = max(0, 1 − (err/400°)²)`) to prevent bounce-back after a flip.
- Yaw is low-pass filtered (`MC_YAW_TQ_CUTOFF`) to reduce vibration from rotor acceleration.
- In **ACRO mode** (manual + no attitude control), this module generates `ω_sp` directly from sticks with expo/superexpo curves.

Output: `vehicle_torque_setpoint` (3 axes, [-1,1]) + `vehicle_thrust_setpoint` (3 axes, [-1,1]).

### 2.6. `control_allocator` — motor distribution

File: `@/home/frank/tf-px4/src/modules/control_allocator/ControlAllocator.cpp`

Algorithm library: `src/lib/control_allocation/` — includes `ControlAllocationPseudoInverse`, `ControlAllocationSequentialDesaturation`, ...

Effectiveness matrix per airframe: `src/modules/control_allocator/VehicleActuatorEffectiveness/` (e.g. `ActuatorEffectivenessMultirotor`, `Tilt`, `Tiltrotor`, `Rover`, ...).

Core algorithm (for a quad):
- Solves the linear problem `B·u = [τ; T_z]` (B is the effectiveness matrix 4×n_motor) using pseudo-inverse `u = B⁺·[τ; T_z]`.
- Clips `u ∈ [0,1]` per motor; if saturation occurs, sequential desaturation reduces the lower-priority components (typically yaw is sacrificed before thrust).
- Publishes `actuator_motors` (each element = setpoint for 1 motor) + feedback `control_allocator_status` (sent back to rate controller for anti-windup).

### 2.7. PWM/DShot Drivers
- `src/lib/mixer_module/` — common output module (`MixingOutput`).
- `src/drivers/pwm_out/`, `src/drivers/dshot/`, `src/drivers/uavcan/` — drivers that execute hardware/CAN signals.

---

## 3. Summary: "Where is the algorithmic root"

| Layer | Control Type | Core File | Theoretical Reference |
|---|---|---|---|
| Position | P (position) | `mc_pos_control/PositionControl/PositionControl.cpp::_positionControl` | Classic PID |
| Velocity | PID + anti-windup | `..._velocityControl` | PID + back-calculation |
| Thrust→Att | geometric vector | `mc_pos_control/PositionControl/ControlMath.cpp::thrustToAttitude` | tilt-prioritized |
| Attitude | quaternion P | `mc_att_control/AttitudeControl/AttitudeControl.cpp::update` | Brescianini et al., ETH 2013 |
| Rate | PID + FF + saturation-aware | `lib/rate_control/rate_control.cpp::update` | parallel-form PID + nonlinear i_factor |
| Allocation | least-squares + desaturation | `lib/control_allocation/ControlAllocationPseudoInverse.cpp` | Moore-Penrose pseudo-inverse |
| Hover thrust | recursive least-squares (RLS) | `modules/mc_hover_thrust_estimator/` | RLS adaptive |
| Setpoint smoothing | jerk-limited trajectory | `lib/motion_planning/VelocitySmoothing.cpp` | bang-bang jerk |

**Philosophy**: cascaded control where each inner loop runs ~10× faster than the outer loop (gyro ~1 kHz → rate ~1 kHz → attitude ~250 Hz → position ~50 Hz). The inner loop should be as simple and "stiff" as possible; the outer loop handles path planning and smoothing.

---

## 4. uORB Topics as "Interfaces"

| Topic | Producer | Consumer | Meaning |
|---|---|---|---|
| `manual_control_setpoint` | `manual_control` | `flight_mode_manager`, `mc_att_control`, `mc_rate_control` | Normalized RC sticks |
| `vehicle_local_position` | `ekf2` | `mc_pos_control`, `flight_mode_manager` | x,y,z + vx,vy,vz local NED |
| `vehicle_attitude` | `ekf2` | `mc_att_control` | quaternion |
| `vehicle_angular_velocity` | `ekf2` (filtered gyro) | `mc_rate_control` | ω + ω̇ |
| `trajectory_setpoint` | `flight_mode_manager` | `mc_pos_control` | pos/vel/acc/yaw setpoint |
| `vehicle_attitude_setpoint` | `mc_pos_control` (or `mc_att_control` in Stabilized) | `mc_att_control` | q_d + thrust_body |
| `vehicle_rates_setpoint` | `mc_att_control` (or `mc_rate_control` in Acro) | `mc_rate_control` | ω_d + thrust_body |
| `vehicle_torque_setpoint`, `vehicle_thrust_setpoint` | `mc_rate_control` | `control_allocator` | controller output, normalized |
| `control_allocator_status` | `control_allocator` | `mc_rate_control` | saturation feedback (anti-windup) |
| `actuator_motors` | `control_allocator` | output drivers | per-motor setpoint [0,1] |

---

## 5. Quick Path for Reading Code

If you want to follow the flow of a "fly to point X" command:
1. `src/modules/navigator/Navigator.cpp` — generates `position_setpoint_triplet`.
2. `src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp` — converts triplet into a smooth `trajectory_setpoint`.
3. `src/modules/mc_pos_control/MulticopterPositionControl.cpp::Run()` → `PositionControl::update()` → `vehicle_attitude_setpoint`.
4. `src/modules/mc_att_control/mc_att_control_main.cpp::Run()` → `AttitudeControl::update()` → `vehicle_rates_setpoint`.
5. `src/modules/mc_rate_control/MulticopterRateControl.cpp::Run()` → `RateControl::update()` → torque/thrust setpoint.
6. `src/modules/control_allocator/ControlAllocator.cpp::Run()` → `actuator_motors`.
7. `src/lib/mixer_module/MixingOutput.cpp` → PWM driver.

That is the complete "control stack" of a quadcopter in this PX4 codebase.
