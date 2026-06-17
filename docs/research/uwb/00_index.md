# UWB → EKF2 Fusion & Precision Landing — Index

Research + implementation docs for fusing UWB ranging into PX4 EKF2, and applying it to
GPS-degraded precision landing. Read in order.

| # | Topic | File |
|---|---|---|
| 1 | Literature review — tight vs loose coupling, observability | `01_literature_review.md` |
| 2 | Tightly-coupled approach + PX4 EKF2 integration (paper → code) | `02_tightly_coupled_px4.md` |
| 3 | v1 implementation plan (Pattern B, self-contained class) — done | `03_v1_implementation_plan.md` |
| 4 | v2 plan — native aiding refactor + control_status + GPS-denied (3 modes) — done | `04_v2_native_plan.md` |
| 5 | Precision landing in GPS-degraded zones — methodology + task-by-task implementation plan | `05_precision_landing.md` |
| 6 | Reference bibliography — verified GPS+UWB fusion & UWB-landing papers (links checked) | `06_references.md` |
| 7 | Theory grounding — GPS+UWB EKF math from 2 cloned papers, mapped to our EKF2 code | `07_gps_uwb_theory_grounding.md` |
| 8 | Deep audit, fix & verification (PX4 uORB→buffer→fusion→switching); root-cause **B1 fixed** | `08_fix_plan.md` |

**Implementation status:** v1 + v2 fusion implemented and verified in SITL (3 modes: UWB+GPS,
GPS-only, UWB-only incl. trilateration cold-start). **GPS+UWB precision landing done & SITL-verified**
— root-cause bug **B1** (multi-anchor buffer drop) fixed; 3-seed A/B = 0.03 m vs 1.5 m (98 %); see
`08_fix_plan.md`. Open: B6 (scale anchors), B4/B5 (field hardening), indoor UWB-only (later).

**Related:** EKF2 internals/ESKF derivation live in [`../control_stack/02f_ekf2_unified.md`](../control_stack/02f_ekf2_unified.md);
notation in [`../quadcopter_control_math.md`](../quadcopter_control_math.md).
