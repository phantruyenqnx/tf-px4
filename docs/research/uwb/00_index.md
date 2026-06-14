# UWB → EKF2 Fusion & Precision Landing — Index

Research + implementation docs for fusing UWB ranging into PX4 EKF2, and applying it to
GPS-degraded precision landing. Read in order.

| # | Topic | File |
|---|---|---|
| 1 | Literature review — tight vs loose coupling, observability | `01_literature_review.md` |
| 2 | Tightly-coupled approach + PX4 EKF2 integration (paper → code) | `02_tightly_coupled_px4.md` |
| 3 | v1 implementation plan (Pattern B, self-contained class) — done | `03_v1_implementation_plan.md` |
| 4 | v2 plan — native aiding refactor + control_status + GPS-denied (3 modes) — done | `04_v2_native_plan.md` |
| 5 | Precision landing in GPS-degraded zones — scenario, fairness, test plan | `05_precision_landing.md` |
| 6 | Precision-landing implementation plan (task-by-task) | `06_precision_landing_plan.md` |

**Implementation status:** v1 + v2 fusion implemented and verified in SITL (3 modes: UWB+GPS,
GPS-only, UWB-only incl. trilateration cold-start). Precision-landing application is in planning
(docs 05/06).

**Related:** EKF2 internals/ESKF derivation live in [`../control_stack/02f_ekf2_unified.md`](../control_stack/02f_ekf2_unified.md);
notation in [`../quadcopter_control_math.md`](../quadcopter_control_math.md).
