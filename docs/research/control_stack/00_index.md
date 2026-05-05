# Control Stack — chỉ mục 7 mục

Mỗi file dưới đây diễn giải **một tầng** của control stack PX4 multicopter: công thức ↔ code ↔ cơ sở lý thuyết ↔ keywords để học sâu.

| # | Mục | File |
|---|---|---|
| 1 | Sensors & IMU pipeline | `01_sensors.md` |
| 2 | EKF2 — State Estimation | `02_ekf2.md` |
| 3 | Position Control | `03_position_control.md` |
| 4 | Attitude Control | `04_attitude_control.md` |
| 5 | Rate Control | `05_rate_control.md` |
| 6 | Control Allocation | `06_control_allocation.md` |
| 7 | Hover Thrust Estimator | `07_hover_thrust_estimator.md` |

Quy ước ký hiệu giữ nhất quán với `../quadcopter_control_math.md` (§0).

Khung tham chiếu: NED $\{W\}$ và Body $\{B\}$ (FRD). Quaternion Hamilton scalar-first $\boldsymbol{q}=(q_w,q_x,q_y,q_z)$. Nhân Hamming-cộng từng phần $\odot$, nhân quaternion $\otimes$, error-state addition $\boxplus$.

Đọc theo thứ tự: 1 → 2 → 3 → 4 → 5 → 6 → 7 (đi từ cảm biến đến motor). Mục 7 (HTE) ăn ngược lên mục 3.
