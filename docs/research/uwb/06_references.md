# GPS + UWB Fusion & UWB Precision Landing — Reference Bibliography

> Curated from a multi-source research pass (fan-out web search → fetch → 3-vote adversarial
> verification). **Every entry below was checked to be a real paper with a resolvable link** — titles,
> authors, venue/year and DOI/arXiv were confirmed against the primary source plus at least one
> independent index (IEEE Xplore / PMC / Semantic Scholar / NASA ADS). Entries that could not be
> verified were dropped.
>
> Two papers (marked **📥 cloned**) are downloaded into [`papers/`](papers/) and analysed from their
> actual equations in [`07_gps_uwb_theory_grounding.md`](07_gps_uwb_theory_grounding.md). This is the
> theory backbone for fusing UWB into PX4 EKF2 (GPS outdoors, UWB priority near the pad).
>
> Complements the older reference lists in [`01_literature_review.md`](01_literature_review.md) §Refs
> and [`05_precision_landing.md`](05_precision_landing.md) §8 — no duplication of those entry IDs.

---

## A. GPS / GNSS + UWB fusion (the core of this work)

**[A1] 📥 Multi-GNSS Precise Point Positioning with UWB Tightly Coupled Integration**
Z. Huang, S. Jin, K. Su, X. Tang. *Sensors* 2022, **22(6):2232**. DOI: 10.3390/s22062232
🔗 https://www.mdpi.com/1424-8220/22/6/2232 · OA full text: https://pmc.ncbi.nlm.nih.gov/articles/PMC8950949/
*Tightly-coupled multi-GNSS (GPS/GLONASS/BDS/Galileo) PPP + UWB **TDOA** in one EKF. Raw pseudorange/
carrier-phase + UWB TDOA stacked in a single measurement vector.* → cloned & analysed (theory of how
UWB enters the measurement model). **Caveat:** tight PPP on raw observables, *not* EKF2's structure.

**[A2] 📥 Research on Kinematic and Static Filtering of the ESKF Based on INS/GNSS/UWB**
Z. Ren, S. Liu, J. Dai, Y. Lv, Y. Fan. *Sensors* 2023, **23(11):5193**. OA: https://pmc.ncbi.nlm.nih.gov/articles/PMC10222699/
*Error-State KF fusing INS + GNSS + UWB **loosely-coupled** — UWB position aids the filter with the
**same observation matrix as GPS**. Sequential "kinematic then static" update = process GNSS/INS then
UWB/INS. +21.98% vs GNSS/INS, +13.03% vs UWB/INS.* → cloned & analysed: **structural match to PX4
EKF2** (error-state, position/velocity aiding). **Caveat:** platform is a UGV, not a UAV.

**[A3] GPS/UWB/MEMS-IMU tightly coupled navigation with improved robust Kalman filter**
*Advances in Space Research* 2016, **58(11)**. DOI: 10.1016/j.asr.2016.07.028
🔗 https://www.sciencedirect.com/science/article/abs/pii/S0273117716303982 *(Elsevier paywall — stub only)*
*UWB range injected into GPS/INS tightly-coupled nav; robust KF uses variance of squared Mahalanobis
distance over a moving window to reject UWB gross errors / NLOS outliers.* Directly relevant to
GPS↔UWB hand-off robustness. **Could not clone (paywalled).**

**[A4] Seamless Outdoor–Indoor Pedestrian Positioning with GNSS/UWB/IMU Fusion: A Comparison of EKF, FGO, and PF**
J. Zhang, X. Yu, S. Ha, et al. (Turku). arXiv 2025. 🔗 https://arxiv.org/abs/2512.10480
*Unified GNSS/UWB/IMU framework: GNSS absolute updates outdoors, UWB absolute updates indoors, IMU-PDR
backbone — exactly the outdoor↔indoor hand-off topology. Compares ESKF vs factor-graph vs particle
filter; ESKF most consistent overall.* **Caveat:** pedestrian (PDR), not UAV; arXiv preprint.

---

## B. UWB for UAV precision landing / docking (the application)

**[B1] UWB and IMU-Based UAV's Assistance System for Autonomous Landing on a Platform** ⭐
A. Ochoa-de-Eribe-Landaberea, L. Zamora-Cadenas, O. Peñagaricano-Muñoa, I. Velez. *Sensors* 2022,
**22(6):2347**. DOI: 10.3390/s22062347
🔗 https://www.mdpi.com/1424-8220/22/6/2347 · OA: https://pmc.ncbi.nlm.nih.gov/articles/PMC8948988/
*8 anchors around a 2×2 m pad, **2 tags on the drone**, per-tag EKF then fused. RMSE 0.410 → 0.208 m
(~50%).* Already cited as [05 ref1]; closest published analogue to the landing application.

**[B2] GPS-Free Wireless Precise Positioning System for Automatic Flying and Landing of Shipborne UAV** ⭐
T.-Y. Lo, J.-Y. Chang, et al., S.-G. Mao. *Sensors* 2024, **24(2):550**. DOI: 10.3390/s24020550
🔗 https://www.mdpi.com/1424-8220/24/2/550 · OA: https://pmc.ncbi.nlm.nih.gov/articles/PMC10821099/
*UAV autonomously flies from 200 m, approaches and lands on a **moving** shipborne platform without
GPS; all data over the UWB link (cm-level).* Demonstrates the full approach→land profile on UWB.

**[B3] Autonomous Dynamic Docking of UAV Based on UWB-Vision in GPS-Denied Environment**
C. Cheng, X. Li, L. Xie, L. Li. *Journal of the Franklin Institute* 2022, **359(7):2788–2809**.
DOI: 10.1016/j.jfranklin.2022.03.005 🔗 https://www.sciencedirect.com/science/article/abs/pii/S0016003222001569
*UWB + vision, 3 phases (hover/approach/landing); EKF for relative pose in hover; precision landing on
a mobile platform.*

**[B4] Integrated UWB-Vision Approach for Autonomous Docking of UAVs in GPS-Denied Environments**
T.-M. Nguyen, T. H. Nguyen, M. Cao, Z. Qiu, L. Xie (NTU). *IEEE ICRA* 2019, pp. 9603–9609.
DOI: 10.1109/ICRA.2019.8793851 🔗 https://ieeexplore.ieee.org/document/8793851
*Canonical NTU UWB+vision UAV docking paper.*

---

## C. UWB + IMU / multi-sensor EKF for UAV localization (GPS-denied) — technique base

**[C1] Accurate 3D Localization for MAV Swarms by UWB and IMU Fusion**
J. Li, Y. Bi, K. Li, K. Wang, F. Lin, B. M. Chen. *IEEE ICCA* 2018. arXiv:1807.10913
🔗 https://arxiv.org/abs/1807.10913 · code: https://github.com/lijx10/uwb-localization
*EKF fusing IMU + UWB, 80 Hz 3D, low latency.*

**[C2] UWB-based System for UAV Localization in GNSS-Denied Environments: Characterization and Dataset**
J. Peña Queralta, C. Martínez Almansa, F. Schiano, D. Floreano, T. Westerlund. *IEEE/RSJ IROS* 2020.
arXiv:2003.04380 🔗 https://arxiv.org/abs/2003.04380 · dataset: https://github.com/TIERS/uwb-drone-dataset
*Decawave DWM1001; indoor UAV localization + open dataset.*

**[C3] Error-State EKF Multi-Sensor Fusion for UAV Localization in GPS- and Magnetometer-Denied Indoor Environments**
L. Markovic, M. Kovac, R. Milijas, M. Car, S. Bogdan (LARICS, Zagreb). *ICUAS* 2022. arXiv:2109.04908
🔗 https://arxiv.org/abs/2109.04908
*ES-EKF fusing IMU + LiDAR SLAM + VIO + Pozyx UWB; validated vs Optitrack.*

**[C4] An Indoor UAV Localization Framework with ESKF Tightly-Coupled Fusion and Multi-Epoch UWB Outlier Rejection**
J. Zhao, Z. Deng, E. Hu, W. Su, B. Lou, Y. Liu. *Sensors* 2025, **25(24):7673**.
🔗 https://www.mdpi.com/1424-8220/25/24/7673 *(also cited as [05 ref5])*
*ESKF tightly-coupled: IMU prediction + raw UWB ranges + VIO + TFmini altitude; multi-epoch NLOS
rejection.*

**[C5] An Improved UWB Indoor Positioning Approach for UAVs Based on the Dual-Anchor Model**
*Sensors* 2025, **25(4):1052**. DOI: 10.3390/s25041052 🔗 https://www.mdpi.com/1424-8220/25/4/1052
*Only 2 anchors (rangefinder gives altitude → 3D collapses to 2D); UWB+rangefinder+baro+accel fused by
UKF; decimeter-level.* Relevant to the few-anchor problem (cf. memory: queue-depth anchor issue).

**[C6] An Improved UWB/IMU Tightly Coupled Positioning Algorithm Study**
A. Zou, W. Hu, Y. Luo, P. Jiang (Hunan Agri. Univ.). *Sensors* 2023, **23(13):5918**.
DOI: 10.3390/s23135918 🔗 https://www.mdpi.com/1424-8220/23/13/5918
*EKF tightly-coupled UWB/IMU, difference-of-measurements as innovation.*

---

## How to read this for the project

| Need | Start here |
|---|---|
| **GPS + UWB in one EKF — measurement model** | **A1** (TDOA stacking), **A2** (loose position aiding) |
| **Structure that matches PX4 EKF2** | **A2** (error-state, pos/vel aiding, same H as GPS) |
| **Outdoor↔indoor / GPS↔UWB hand-off** | **A4** (topology), **A3** (NLOS robust rejection at the seam) |
| **Precision-landing application** | **B1**, **B2** (full approach→land on UWB) |
| **UWB-only indoor (later)** | **C1–C6** |

> ⚠️ Honesty notes carried from verification: **A2** is a UGV, **A4** is pedestrian — cite them for the
> *filter structure*, not as UAV flight evidence. No single verified paper does exactly "GPS outdoors →
> UWB-priority near pad → precision landing on a UAV"; that combination is **A2's loose-coupling
> structure** + **B1/B2's landing profile**. That gap is the novel part of this work.
