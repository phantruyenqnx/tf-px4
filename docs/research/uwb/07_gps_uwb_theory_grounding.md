# GPS + UWB Fusion in EKF2 — Theory Grounding (from cloned papers)

> **Purpose.** Ground the project's UWB→EKF2 fusion and the *GPS-outdoors / UWB-priority-near-pad*
> precision-landing design in the **actual equations** of two GPS/GNSS+UWB papers that were
> **downloaded into [`papers/`](papers/)** (not paraphrased from memory). Each claim below cites either
> a paper equation (file + line in the extracted text) or a source line in our EKF2 tree.
>
> **Cloned sources** (full text + MathML→text extraction in `papers/`):
> - **[A1]** Huang et al., *Multi-GNSS PPP with UWB Tightly Coupled Integration*, Sensors 2022 22(6):2232
>   → `papers/gnss-ppp-uwb_sensors2022.txt` (OA mirror PMC8950949).
> - **[A2]** Ren et al., *Kinematic & Static Filtering of the ESKF Based on INS/GNSS/UWB*, Sensors 2023
>   → `papers/eskf-ins-gnss-uwb_sensors2023.txt` (OA mirror PMC10222699).
>
> A3 (GPS/UWB/MEMS-IMU robust KF, ASR 2016) is paywalled — only the abstract/stub was retrievable;
> it is used qualitatively, **not** quoted. See [`06_references.md`](06_references.md).

---

## 0. TL;DR — what the theory says about your design

Your target: **outdoors fly on GPS; inside UWB range (≈ pad) let UWB dominate; hand off both ways; use
it for precision landing.** The two papers map to this cleanly:

| Your design element | Grounded in | Where |
|---|---|---|
| UWB and GPS in one filter | Both A1 (tight) and A2 (loose) fuse GNSS + UWB in a single KF | A1 §2.3.1, A2 §2.3 |
| **UWB "higher priority" near pad** | = **smaller measurement covariance R** for UWB than GPS → larger Kalman gain on UWB. A2 sets UWB σ=0.8 m vs GPS σ=1.0 m and UWB wins | A2 §4.2, Eqs (13)–(15) |
| GPS↔UWB hand-off (in/out of range) | A1 explicitly: *"when both GNSS and UWB are available… the tightly-coupled model is implemented"*; after a GNSS outage the GNSS/UWB filter "continues to work well" and re-converges faster | A1 line 797, 979 |
| Why UWB fixes landing error | UWB is a **bias-free local-frame** constraint; GPS carries slow drift. Near the pad UWB pulls the estimate off the drifted GPS frame onto the surveyed-anchor frame | A1 line 880 (64–78% horiz. RMS ↓); verified A/B = 0.03 m vs 1.5 m (08) |
| Outdoor↔indoor seam robustness | NLOS/gross-range rejection needed at the seam (A3 Mahalanobis-window robust KF; your code's re-acquisition gate) | A3; `uwb_range_control.cpp:214` |

**Bottom line:** the design is theoretically sound and matches published GNSS+UWB practice. The one
place you are ahead of / off to the side of the literature is the *mechanism* of "UWB priority" — you
do it with **R-inflation on GPS + a hard trilateration reset**, whereas the papers do it with **a
lower fixed R on UWB** (A2) or **adaptive R + robust gating** (A3/A4). See §4 for the gap.

---

## 1. Two coupling styles — and which one your code actually is

The literature splits GPS+UWB into **loosely** vs **tightly** coupled. A1's intro states the trade
directly (`papers/gnss-ppp-uwb_sensors2022.txt:729`):

> "The loosely coupled integration… is computationally low when GPS is working, **but it fails when
> GPS measurements are not available.** … the GPS/UWB tightly coupled integration… can increase the
> **redundancy of observations and improve the reliability**."

- **Loosely coupled (A2).** UWB is first solved to a **position** (TDOA → x,y,z), then that position
  aids the filter exactly like a GPS fix. A2's UWB observation (`...sensors2023.txt:837`):

  ```
  L_k^UWB = [ v_INS − v_UWB ; p_INS − p_UWB ] = H_k^UWB · x_k + v_k^UWB
  H_k^UWB = [0  I  0  0  0 ;  0  0  I  0  0]      ← identical structure to H_k^GNSS (line 832)
  ```

  i.e. **UWB enters with the same measurement Jacobian as GPS** — it's just another pos/vel source.

- **Tightly coupled (A1) — and this is what your tree does.** UWB is fused as **raw range / TDOA**,
  never pre-solved. A1's UWB TDOA residual (`...sensors2022.txt:788`):

  ```
  d^uwb = −v · δp + ε_d          (v = LOS-difference unit vectors;  δp = position increment)
  ```

  Our EKF2 uses the simpler **direct range** (not TDOA-differenced) but the same idea —
  `aid_sources/uwb/uwb_range_control.cpp:1`:

  ```
  h(x) = ||p − a_i||                         predicted range to anchor a_i
  H[pos] = (p − a_i)^T / ||p − a_i||         unit LOS direction   (code line 45–50)
  ```

  Each tag→anchor range is one independent scalar update (`uwb_range_control.cpp:23,74`).

**So:** our implementation is the **tightly-coupled (A1) family**, not the loose (A2) family. A1 is
therefore the *primary* theory anchor; A2 is the *alternative* and the source of the "same-H-as-GPS"
intuition.

### Why tight (your choice) is the right call here
A1's own results justify it (`...sensors2022.txt:880, 923, 979`):
- With only **3 GPS satellites visible** (degraded sky), a GPS-only solution can't be formed at all,
  but GNSS/UWB still has "reasonable PDOP" and holds cm-level horizontal — *tight coupling adds usable
  observations even when GPS alone is under-determined.*
- **Each range is an independent constraint**, so you don't need ≥3 simultaneous anchors *per epoch*
  to get value (loose TDOA solve does). This matters directly for us — see the memory note about the
  system seeing ~1 anchor at a time ([[gz-uwb-plugin-one-anchor]]); a tight per-range update degrades
  gracefully where a loose 3-anchor solve would simply fail.

---

## 2. The EKF, side by side (paper ↔ our EKF2)

Both papers are **error-state** filters, which is exactly PX4 ECL/EKF2's formulation.

A2 state & error-state (`...sensors2023.txt:762,764,767`):

```
X       = [ q, v, p, a_b, ω_b ]^T          nominal (att, vel, pos, accel-bias, gyro-bias)
X_truth = X + δX                            error-state injection
δX      = [ δq, δv, δp, δa_b, δω_b ]^T
```

This is PX4 EKF2's state to a tee (quaternion error, vel, pos, IMU biases). The predict step
(`...sensors2023.txt:810,822`) is the standard `δx_{k+1}=Φ δx_k + Γ w`, `Σ=ΦΣΦ^T+ΓQΓ^T`; the update
(`...sensors2023.txt:844`) is the standard Joseph-form gain. Our `measurementUpdate(K,H,R,innov)`
(`uwb_range_control.cpp:74`) is the same update with the **scalar** range innovation:

```
innovation     = predicted − z = ||p−a_i|| − range          (uwb_range_control.cpp:58)
innovation_var = H P H^T + R                                  (uwb_range_control.cpp:59)
K              = P H^T / innovation_var                       (uwb_range_control.cpp:73)
```

A1's combined GNSS/UWB measurement stack (`...sensors2022.txt:804`) is the tight-coupling target —
GPS pseudorange/carrier-phase rows **and** the UWB row in one `z_k`:

```
z_k = [ p_m^G, φ_m^G, …, d_i^uwb, … ]^T  =  H_k x_k + ε_k
```

We don't stack — we fuse GPS and each UWB range **sequentially** (PX4 fuses every aid source
sequentially). A2 formalizes exactly this sequential style as **"kinematic (GNSS/INS) then static
(UWB/INS)"** (`...sensors2023.txt:864,893`): GNSS updates first, the result + covariance is handed to
the UWB update as its prior. Mathematically sequential scalar updates ≡ a stacked update when the
measurements are independent — so our per-source sequential fusion is the standard, correct reduction
of A1's stacked `z_k`.

---

## 3. "UWB priority near the pad" — the formal meaning

"Priority" in a Kalman filter is **not** a mode switch; it's **relative measurement covariance**. The
gain on a source scales with `P H^T / (H P H^T + R)` — **smaller R ⇒ larger gain ⇒ that source pulls
the state more.** The papers make this concrete:

- **A2 weights UWB above GPS by giving it a smaller R**: GPS position σ = 1.0 m, **UWB position σ =
  0.8 m** (`...sensors2023.txt:Table 1`, `:837` measurement-noise vectors `v_k^UWB=[0.4,0.4,0.4,0.8,0.8,0.8]`
  vs `v_k^GNSS=[0.5,…,1,1,1]`). Result: +13% accuracy from the UWB stage on top of GNSS.
- **A1 sets UWB σ = 0.1 m** against pseudorange σ = 0.3 m (`...sensors2022.txt:832`) — UWB heavily
  trusted, giving the 64–78% horizontal RMS improvement (`:880`).

### How our tree expresses priority — and where it diverges
Our knob is **`EKF2_UWB_GPS`** ("UWB-dominant near the pad"). Two mechanisms in code:

1. **R-inflation on GNSS** when UWB should dominate (commit `db618a4` / 05 §B; the
   `gps_control` path inflates GPS noise so its gain shrinks). This is the *complement* of the papers'
   "lower R on UWB" — same end effect (UWB wins), achieved by raising GPS R instead of lowering UWB R.
   ✔ Theoretically equivalent for the gain ratio.
2. **Hard re-acquisition reset** (`uwb_range_control.cpp:214–224`):

   > "after flying out of UWB range the EKF drifts on GPS; when the ranges return the UWB innovations
   > are large and get gated out (chicken-and-egg, ~73% rejected)… snap the horizontal position to the
   > UWB trilateration solution so UWB re-locks."

   This is **not** in either paper. It's a pragmatic fix for a real seam problem (see §4).

---

## 4. The seam problem (GPS→UWB re-entry) — where you're off the paper trail

The single most important theory-vs-code gap is the **re-entry transient**. When the drone returns
into UWB range, the GPS-drifted estimate is far from the true (anchor-frame) position, so the first
UWB ranges produce **huge innovations** and the χ² gate rejects them — UWB can never get a foot in the
door. Your code solves it by **hard-resetting** position to the trilateration fix (`:219`).

The literature solves the *same* large-innovation/NLOS problem more principledly:

- **A3 (ASR 2016, robust KF).** Computes the **variance of the squared Mahalanobis distance over a
  moving window** as the gross-error judgment factor, *de-weighting* (not hard-rejecting) suspect UWB
  ranges. This both (a) avoids the chicken-and-egg lock-out and (b) avoids a discontinuous state jump.
- **A4 (Turku 2025) / [05 ref2] (Osaka 2025).** Adaptive R from DOP/UERE — R grows with geometry/NLOS
  risk, so the seam is handled by smoothly *trusting UWB less when it's unreliable* and more as it
  settles, instead of a binary gate.

**Recommendation (theory-backed):** keep the trilateration reset as a *cold-start / fallback*, but for
the re-entry transient add a **temporary innovation-gate widening + R taper** (large R that decays as
consecutive UWB ranges pass the gate). That is A3/A4's mechanism and it removes the state
discontinuity that a hard reset injects right before a precision landing — exactly the phase where you
least want a position jump. The current ~73%-reject → snap behavior works, but a tapered re-lock is
the published, smoother route and is a low-risk change to `controlUwbRangeFusion`.

> **⚠️ Update (verified) — this upgrade turned out to be unnecessary.** The ~73% reject storm was a
> *symptom of B1* (only one anchor fusing → loose estimate → gating). After the per-anchor-buffer fix,
> SITL shows re-entry reject ≈ 2% and **no position reset at all** — the hard-reset never fires. So the
> R-taper is **not needed** for GPS+UWB; the trilateration reset stays as an inert fallback. See
> [`08_fix_plan.md`](08_fix_plan.md) B3.

---

## 5. Anchor survey & observability (tight-coupling's price)

Tight raw-range fusion (our path) buys redundancy but **pays in anchor survey accuracy**: `h(x)`
depends on `a_i` (anchor NED in params, `uwb_range_control.cpp:12–21`). A survey error in `a_i` biases
every fix by ~the same amount — there is no averaging-out. A1/A2 both assume **known base-station
coordinates** (A1 `:826` "UWB data include the positions of the base stations"; A2 `:Figure 3` fixed
red-circle anchors). So:

- Your surveyed-anchor accuracy is a **hard floor** on landing accuracy. For a ~0.03 m landing (08),
  anchors must be surveyed to well under that.
- **Vertical observability** is real in our model because anchors are non-coplanar
  (`uwb_range_control.cpp:42–44`), so the LOS Jacobian has a vertical component — good. But the cold-
  start trilateration deliberately solves **N/E only** and leaves height on baro
  (`uwb_range_control.cpp:85–87`), which is the safe choice when anchors are near-coplanar.

---

## 6. Concrete, theory-grounded checklist for the GPS+UWB outdoor → landing path

1. **Keep tight raw-range fusion** (A1-justified; degrades gracefully to 1–2 anchors, unlike a loose
   TDOA solve). ✔ already done.
2. **Express UWB priority as a covariance ratio, and log it.** Whether you inflate GPS R or shrink UWB
   R, the *gain ratio* is what matters (§3). Record both R's so the priority is auditable.
3. ~~**Replace the hard re-entry snap with an R-taper + temporary gate-widen** (A3/A4).~~ **Verified
   unnecessary** after the B1 fix — re-entry shows no reset in SITL (§4 update; `08_fix_plan.md` B3).
4. **Survey anchors to ≪ target landing CEP**; treat survey error as a bias floor (§5).
5. **Sanity-check against A1's numbers:** horizontal RMS should improve markedly once UWB engages;
   vertical gains are smaller (A1 up-component improved least, `:931`) — expect baro/anchor-height to
   dominate vertical near touchdown.

---

## 7. Honesty / limits of this grounding

- **A2 is a UGV, A4 is pedestrian.** Their *filter structure and R-weighting* transfer to a UAV; their
  *flight performance numbers* do not. Cite them for mechanism, not for "UAV proves it."
- **A1 is tightly-coupled PPP on raw GNSS observables.** PX4 EKF2 ingests GPS **position/velocity**
  (loose on the GPS side), so A1's GNSS half is *not* what EKF2 does — only A1's **UWB raw-range half**
  is the direct analogue to our `fuseUwbRange`. Don't over-claim PPP-level (cm) GPS behavior for EKF2.
- **A3 could not be cloned** (Elsevier paywall) — its robust-KF method is described from the verified
  abstract, not read in full. Treat §4's A3 mechanism as a *direction to implement & test*, not a
  reproduced result.
- No verified paper does *exactly* "GPS outdoors → UWB-priority near pad → UAV precision landing." That
  specific composition (A2-style priority weighting + A1-style tight range + B1/B2 landing profile) is
  **the novel contribution of this work** — which is why the SITL A/B result in
  [`05_precision_landing.md`](05_precision_landing.md) is the evidence that matters most.
