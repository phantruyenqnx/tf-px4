# Control Stack — Index of 7 Topics

Each file below explains **one layer** of the PX4 multicopter control stack: formulas ↔ code ↔ theoretical basis ↔ keywords for deeper study.

| # | Topic | File |
|---|---|---|
| 1 | Sensors & IMU pipeline | `01_sensors.md` |
| 2 | EKF2 — State Estimation | `02_ekf2.md` |
| 2b | KF → EKF2: theory vs practice | `02b_kf_to_ekf2_theory.md` |
| 2c | Guide to reading EKF2 source (quadcopter path) | `02c_ekf2_source_guide.md` |
| 2d | EKF2 limitations & improvement directions | `02d_ekf2_limitations.md` |
| 2e | Gaussian → KF → EKF → ESKF → EKF2 PX4 (full derivation) | `02e_gaussian_to_ekf2_full.md` |
| 3 | Position Control | `03_position_control.md` |
| 4 | Attitude Control | `04_attitude_control.md` |
| 5 | Rate Control | `05_rate_control.md` |
| 6 | Control Allocation | `06_control_allocation.md` |
| 7 | Hover Thrust Estimator | `07_hover_thrust_estimator.md` |

Notation conventions are kept consistent with `../quadcopter_control_math.md` (§0).

Reference frames: NED $\{W\}$ and Body $\{B\}$ (FRD). Hamilton scalar-first quaternion $\boldsymbol{q}=(q_w,q_x,q_y,q_z)$. Element-wise product $\odot$, quaternion product $\otimes$, error-state addition $\boxplus$.

Read in order: 1 → 2 → 3 → 4 → 5 → 6 → 7 (from sensors to motors). Topic 7 (HTE) feeds back up into topic 3.
