# Kiến trúc bộ điều khiển Quadcopter trong PX4 (tf-px4)

Tài liệu này mô tả toàn bộ tầng điều khiển (control stack) của một multicopter trong codebase `src/` của PX4. Mục tiêu: hiểu **flow** từ "ý định người dùng / mission" cho tới **lệnh PWM ra động cơ**, biết file gốc rễ của từng tầng, và nắm thuật toán cốt lõi của mỗi khối.

---

## 1. Tổng quan kiến trúc

PX4 dùng kiến trúc **cascaded control** (điều khiển nối tầng) chạy trên uORB pub/sub. Mỗi tầng là một module riêng, chạy theo **work queue** (event-driven, không phải thread riêng), kích hoạt bởi topic phía trên hoặc bởi sensor mới.

```
┌─────────────────────────────────────────────────────────────────────┐
│  Người dùng / GCS / Mission                                          │
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
│  flight_mode_manager  (chọn FlightTask theo nav_state)               │
│  src/modules/flight_mode_manager/FlightModeManager.cpp               │
│  Tasks: ManualPosition, ManualAltitude, Auto, Orbit, Descend, ...    │
│  → publish: trajectory_setpoint, vehicle_constraints                 │
└──────────────────────────┬──────────────────────────────────────────┘
                           │ trajectory_setpoint (pos/vel/acc/yaw)
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  mc_pos_control  (Position + Velocity P/PID, ngoài cùng)             │
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
│  mc_rate_control (Rate PID, trong cùng - chạy nhanh nhất)            │
│  src/modules/mc_rate_control/MulticopterRateControl.cpp              │
│   └─ core: src/lib/rate_control/rate_control.cpp                     │
│  Input : vehicle_angular_velocity (gyro) + vehicle_rates_setpoint    │
│  Output: vehicle_torque_setpoint, vehicle_thrust_setpoint            │
└──────────────────────────┬──────────────────────────────────────────┘
                           │ torque + thrust setpoints (normalised [-1,1])
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  control_allocator  (mixer: torque/thrust → từng motor)              │
│  src/modules/control_allocator/ControlAllocator.cpp                  │
│   └─ algo: src/lib/control_allocation/* (pseudo-inverse + clipping)  │
│  Output: actuator_motors, actuator_servos                            │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  pwm_out / dshot / uavcan driver (src/lib/mixer_module + drivers/)   │
│  → tín hiệu PWM/DShot tới ESC → động cơ                              │
└─────────────────────────────────────────────────────────────────────┘
```

Nhánh feedback (state estimation) chạy song song:

```
sensors (gyro, accel, mag, baro, GPS)
   → src/modules/sensors  (lọc, hiệu chuẩn)
   → src/modules/ekf2     (EKF: vehicle_attitude, vehicle_local_position,
                                  vehicle_angular_velocity, ...)
```

EKF2 là "nguồn sự thật" về trạng thái máy bay; mọi tầng điều khiển đọc topic của nó.

---

## 2. Flow từ gốc đến ngọn (file by file)

### 2.1. Nguồn lệnh
- **RC / manual**: `src/modules/manual_control` đọc RC raw, publish `manual_control_setpoint`.
- **Mission / offboard**: `src/modules/mavlink` (MAVLink) hoặc `src/modules/uxrce_dds_client` (uXRCE-DDS / ROS 2) → publish `vehicle_command`, `trajectory_setpoint` offboard, ...
- **Mission tự động**: `src/modules/navigator` xử lý mission items, RTL, loiter; output `position_setpoint_triplet`.

### 2.2. `flight_mode_manager` — chọn cách sinh setpoint
File chính:
- `@/home/frank/tf-px4/src/modules/flight_mode_manager/FlightModeManager.cpp`
- `@/home/frank/tf-px4/src/modules/flight_mode_manager/FlightModeManager.hpp`

Cơ chế: dựa vào `vehicle_status.nav_state` và `vehicle_control_mode`, FMM chọn một `FlightTask` (state pattern). Mỗi task nằm trong `src/modules/flight_mode_manager/tasks/`:

- `FlightTask/` — base class.
- `ManualAltitude/`, `ManualPosition/`, `ManualAcceleration/` — chế độ tay (Stabilized/Altitude/Position).
- `Auto/`, `AutoFollowTarget/`, `Orbit/`, `Descend/`, `Failsafe/`, `Transition/` — chế độ tự động.
- `Utility/` — smoothing trajectory (jerk-limited), waypoint handling, ...

Mỗi task `update(dt)` sẽ tạo `trajectory_setpoint_s` (pos, vel, acc, yaw, yawspeed — bất kỳ trường nào không dùng đặt `NaN`). FMM publish topic `trajectory_setpoint` và `vehicle_constraints`. Đây là **giao thức cốt lõi** giữa "tầng generate setpoint" và "tầng track setpoint".

Smoothing được thực hiện trong `src/lib/motion_planning/` (jerk-limited trajectories) — gốc thuật toán làm tay ga / cần lái mượt mà.

### 2.3. `mc_pos_control` — vòng ngoài (Position + Velocity)

File:
- Module wrapper (uORB I/O): `@/home/frank/tf-px4/src/modules/mc_pos_control/MulticopterPositionControl.cpp`
- Header: `@/home/frank/tf-px4/src/modules/mc_pos_control/MulticopterPositionControl.hpp`
- **Lõi thuật toán**: `@/home/frank/tf-px4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp`

Hỗ trợ:
- `Takeoff/Takeoff.cpp` — state machine spool-up + ramp thrust khi cất cánh.
- `GotoControl/` — xử lý "goto setpoint" mượt.
- `PositionControl/ControlMath.cpp` — chuyển vector lực đẩy → quaternion thái độ.

Thuật toán (xem hàm `PositionControl::update`):
1. **P-controller vị trí** (`_positionControl`): `v_sp = Kp_pos * (p_sp - p) + v_ff`, sau đó clamp theo giới hạn vận tốc.
2. **PID-controller vận tốc** (`_velocityControl`): `a_sp = Kp_v*(v_sp - v) + Ki_v*∫err dt - Kd_v*ȧ + a_ff` với anti-windup khi đụng giới hạn thrust.
3. **Acceleration → Thrust → Attitude** (`_accelerationControl` + `ControlMath::thrustToAttitude`):
   - Lực đẩy mong muốn (vector trong frame world): `T = m*(a_sp + g·ẑ)`, biểu diễn dưới dạng *normalized thrust* qua tham số `MPC_THR_HOVER` (hoặc estimator `mc_hover_thrust_estimator`).
   - Tilt được giới hạn (`MPC_TILTMAX_AIR`).
   - Quaternion thái độ `q_d` được dựng sao cho trục Z body khớp hướng vector lực, kết hợp với `yaw_sp`.
4. Output: `vehicle_attitude_setpoint` (gồm `q_d` + `thrust_body[3]`) và `vehicle_local_position_setpoint` (cho logging/landdetector).

Cốt lõi: **cascaded P (position) → PID (velocity) → vector thrust → quaternion**.

### 2.4. `mc_att_control` — vòng giữa (Attitude)

File:
- Wrapper: `@/home/frank/tf-px4/src/modules/mc_att_control/mc_att_control_main.cpp`
- Header: `@/home/frank/tf-px4/src/modules/mc_att_control/mc_att_control.hpp`
- **Lõi**: `@/home/frank/tf-px4/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`

Khi ở Manual/Stabilized (không có pos/vel control), module này tự sinh `vehicle_attitude_setpoint` từ stick (`generate_attitude_setpoint`). Khi có pos control, nó nhận setpoint từ `mc_pos_control`.

Thuật toán cốt lõi (`AttitudeControl::update(q)`):
- **Quaternion attitude controller** theo bài báo *"Nonlinear Quadrocopter Attitude Control"* — Brescianini, Hehn, D'Andrea, ETH 2013.
- Ý tưởng: tách ưu tiên *tilt* (roll/pitch) khỏi *yaw* để khi yaw lệch nhiều, drone vẫn giữ được hướng lực đẩy đúng (tránh mất độ cao).
  1. Tính `q_red` chỉ chỉnh trục Z (tilt) — full priority.
  2. Tính `q_dyaw` phần yaw, scale bằng `MC_YAW_WEIGHT < 1` → desired attitude lai `q_mix = q_red * q_yaw_scaled`.
  3. Sai số `q_e = q^{-1} * q_mix`; tốc độ góc đặt: `ω_sp = 2 * sign(q_e0) * Kp ⊗ q_e.imag` (sin(α/2)·trục).
  4. Thêm feed-forward `yawspeed_setpoint` (chiếu sang body frame).
  5. Clamp theo `MC_{ROLL,PITCH,YAW}RATE_MAX`.
- Output: `vehicle_rates_setpoint` (ω_d) — chỉ là **P-controller**, không có I/D ở tầng này.

### 2.5. `mc_rate_control` — vòng trong (Rate, chạy nhanh nhất)

File:
- Wrapper: `@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.cpp`
- Header: `@/home/frank/tf-px4/src/modules/mc_rate_control/MulticopterRateControl.hpp`
- **Lõi**: `@/home/frank/tf-px4/src/lib/rate_control/rate_control.cpp` (lib dùng chung cho cả MC và FW).

Module này là **work item theo gyro** (`_vehicle_angular_velocity_sub.registerCallback()`), tức là chạy mỗi khi có sample gyro mới (~ 1 kHz tùy IMU). Đây là vòng nhanh nhất, quan trọng nhất về độ ổn định.

Thuật toán (`RateControl::update`):
- PID **dạng song song có feed-forward**:
  ```
  τ = Kp ⊙ (ω_sp - ω) + ∫Ki·err dt − Kd ⊙ α  + Kff ⊙ ω_sp
  ```
  với `α = ω̇` (đạo hàm tốc độ góc, do EKF cung cấp dưới dạng `xyz_derivative`).
- Tham số được map qua dạng "ideal" (`K * [1 + 1/(Ti·s) + Td·s]`) để quen thuộc với người chỉnh tay (`MC_*RATE_K/P/I/D`).
- **Anti-windup hai lớp**:
  1. Saturation feedback từ `control_allocator_status` (nếu motor đã bão hòa theo trục nào, dừng tích phân theo hướng đó).
  2. Giảm gain I khi sai số rate lớn (`i_factor = max(0, 1 − (err/400°)²)`) để chống bounce-back sau flip.
- Yaw được lọc thông thấp (`MC_YAW_TQ_CUTOFF`) để giảm rung do gia tốc rotor.
- Khi ở **ACRO mode** (manual + không có attitude control), module này tự sinh `ω_sp` từ stick với expo/superexpo curve.

Output: `vehicle_torque_setpoint` (3 trục, [-1,1]) + `vehicle_thrust_setpoint` (3 trục, [-1,1]).

### 2.6. `control_allocator` — phân phối tới động cơ

File: `@/home/frank/tf-px4/src/modules/control_allocator/ControlAllocator.cpp`

Thư viện thuật toán: `src/lib/control_allocation/` — gồm `ControlAllocationPseudoInverse`, `ControlAllocationSequentialDesaturation`, ...

Effectiveness matrix theo từng airframe: `src/modules/control_allocator/VehicleActuatorEffectiveness/` (ví dụ `ActuatorEffectivenessMultirotor`, `Tilt`, `Tiltrotor`, `Rover`, ...).

Thuật toán cốt lõi (cho quad):
- Giải bài toán tuyến tính `B·u = [τ; T_z]` (B là effectiveness matrix 4×n_motor) bằng pseudo-inverse `u = B⁺·[τ; T_z]`.
- Clip `u ∈ [0,1]` cho từng motor; nếu bão hòa, sequential desaturation giảm các thành phần ưu tiên thấp (thường yaw bị hi sinh trước thrust).
- Phát `actuator_motors` (mỗi phần tử = setpoint cho 1 motor) + feedback `control_allocator_status` (báo về cho rate controller để anti-windup).

### 2.7. Drivers PWM/DShot
- `src/lib/mixer_module/` — common output module (`MixingOutput`).
- `src/drivers/pwm_out/`, `src/drivers/dshot/`, `src/drivers/uavcan/` — driver thực thi tín hiệu phần cứng/CAN.

---

## 3. Tổng kết "đâu là gốc rễ thuật toán"

| Tầng | Loại điều khiển | File lõi | Tham chiếu lý thuyết |
|---|---|---|---|
| Position | P (vị trí) | `mc_pos_control/PositionControl/PositionControl.cpp::_positionControl` | PID kinh điển |
| Velocity | PID + anti-windup | `..._velocityControl` | PID + back-calculation |
| Thrust→Att | hình học vector | `mc_pos_control/PositionControl/ControlMath.cpp::thrustToAttitude` | tilt-prioritized |
| Attitude | quaternion P | `mc_att_control/AttitudeControl/AttitudeControl.cpp::update` | Brescianini et al., ETH 2013 |
| Rate | PID + FF + saturation-aware | `lib/rate_control/rate_control.cpp::update` | PID dạng song song + nonlinear i_factor |
| Allocation | least-squares + desaturation | `lib/control_allocation/ControlAllocationPseudoInverse.cpp` | Moore-Penrose pseudo-inverse |
| Hover thrust | recursive least-squares (RLS) | `modules/mc_hover_thrust_estimator/` | RLS adaptive |
| Smoothing setpoint | jerk-limited trajectory | `lib/motion_planning/VelocitySmoothing.cpp` | bang-bang jerk |

**Triết lý**: cascaded control với mỗi vòng nhanh hơn vòng ngoài ~10× (gyro ~1 kHz → rate ~1 kHz → att ~250 Hz → pos ~50 Hz). Vòng trong càng đơn giản và càng "cứng" càng tốt; vòng ngoài lo đường đi và mượt hóa.

---

## 4. Bảng các uORB topic làm "khớp nối"

| Topic | Producer | Consumer | Ý nghĩa |
|---|---|---|---|
| `manual_control_setpoint` | `manual_control` | `flight_mode_manager`, `mc_att_control`, `mc_rate_control` | RC sticks đã chuẩn hóa |
| `vehicle_local_position` | `ekf2` | `mc_pos_control`, `flight_mode_manager` | x,y,z + vx,vy,vz local NED |
| `vehicle_attitude` | `ekf2` | `mc_att_control` | quaternion |
| `vehicle_angular_velocity` | `ekf2` (filtered gyro) | `mc_rate_control` | ω + ω̇ |
| `trajectory_setpoint` | `flight_mode_manager` | `mc_pos_control` | pos/vel/acc/yaw setpoint |
| `vehicle_attitude_setpoint` | `mc_pos_control` (hoặc `mc_att_control` ở Stabilized) | `mc_att_control` | q_d + thrust_body |
| `vehicle_rates_setpoint` | `mc_att_control` (hoặc `mc_rate_control` ở Acro) | `mc_rate_control` | ω_d + thrust_body |
| `vehicle_torque_setpoint`, `vehicle_thrust_setpoint` | `mc_rate_control` | `control_allocator` | đầu ra controller, normalized |
| `control_allocator_status` | `control_allocator` | `mc_rate_control` | saturation feedback (anti-windup) |
| `actuator_motors` | `control_allocator` | output drivers | setpoint từng motor [0,1] |

---

## 5. Đường nhanh để đọc code

Nếu bạn muốn theo flow một lệnh "fly to point X":
1. `src/modules/navigator/Navigator.cpp` — sinh `position_setpoint_triplet`.
2. `src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp` — biến triplet thành `trajectory_setpoint` mượt.
3. `src/modules/mc_pos_control/MulticopterPositionControl.cpp::Run()` → `PositionControl::update()` → `vehicle_attitude_setpoint`.
4. `src/modules/mc_att_control/mc_att_control_main.cpp::Run()` → `AttitudeControl::update()` → `vehicle_rates_setpoint`.
5. `src/modules/mc_rate_control/MulticopterRateControl.cpp::Run()` → `RateControl::update()` → torque/thrust setpoint.
6. `src/modules/control_allocator/ControlAllocator.cpp::Run()` → `actuator_motors`.
7. `src/lib/mixer_module/MixingOutput.cpp` → driver PWM.

Đó là toàn bộ "tầng control" của quadcopter trên PX4 codebase này.
