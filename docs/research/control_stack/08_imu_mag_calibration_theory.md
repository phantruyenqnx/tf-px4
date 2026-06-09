# 8. Calibration of IMU & Magnetometer — Theory

> **Scope.** This file explains the *theory* behind how PX4 calibrates the
> inertial measurement unit (gyroscope + accelerometer) and the magnetometer.
> It is grounded in what the PX4 codebase actually does, but contains **no code
> and no source citations** — only the mathematical models, the estimation
> methods, the procedures, and the reasoning. For the signal-conditioning
> (filtering) side of the pipeline see [09 — Filtering theory](09_imu_mag_filtering_theory.md);
> for where these corrected signals are *fused* see [02 — EKF2](02_ekf2.md).

Notation follows [01 — Sensors](01_sensors.md) §1.3 and the index (§0): world frame
$\{W\}$ = NED, body frame $\{B\}$ = FRD, sensor frame $\{S\}$, rotation
$\boldsymbol{R}_{BS}$ from sensor to body. $g = 9.80665\,\text{m/s}^2$.

---

## 8.1. Why Calibration Exists — the Sensor Error Budget

EKF2 assumes its inputs are **unbiased measurements corrupted only by zero-mean
white Gaussian noise**. Real MEMS sensors violate this in three structured ways
that no amount of statistical filtering can remove, because they are *not*
zero-mean:

| Error class | Physical cause | Effect on estimate |
|---|---|---|
| **Bias / offset** | Fabrication asymmetry, mounting stress, temperature | Constant drift; integrates into unbounded attitude/position error |
| **Scale factor** | Sensitivity ≠ nominal | Output proportional-but-wrong; grows with signal amplitude |
| **Cross-axis / misalignment** | Imperfect orthogonality of the 3 sense axes; chip not aligned to the board | Leakage between axes |
| **Environmental distortion** | Ferromagnetic material, motor currents (mag); thermal drift (all) | State- or input-dependent corruption |

Calibration is the process of **identifying** these deterministic error
parameters offline (or slowly online) so they can be *subtracted out*, leaving a
residual that genuinely looks like the white noise EKF2 expects. The split of
labour is deliberate:

- **Offline / stored calibration** removes the large, slowly-varying structured
  errors (factory bias, scale, hard/soft-iron). Stored in `CAL_*` parameters.
- **Online estimation inside EKF2** tracks the *remaining* slow drift of gyro and
  accel bias as filter states ([02](02_ekf2.md)). This is *not* calibration in
  the sense of this document — it is state estimation.
- **Online auxiliary estimators** (gyro bias learner, magnetometer bias learner)
  refine the stored offsets between/within flights without a manual procedure.

The general correction applied to every triad is the same affine inverse model:

$$
\boldsymbol{x}_{\text{corr}} = \boldsymbol{R}_{BS}\,\boldsymbol{M}\,(\boldsymbol{x}_{\text{raw}} - \boldsymbol{b} - \boldsymbol{b}_{\text{th}})
$$

where $\boldsymbol{b}$ is the bias, $\boldsymbol{b}_{\text{th}}$ the temperature
offset, $\boldsymbol{M}$ a scale (and in principle cross-axis) matrix, and
$\boldsymbol{R}_{BS}$ the fixed mounting rotation. Calibration is the act of
finding $\boldsymbol{b}$, $\boldsymbol{M}$, $\boldsymbol{b}_{\text{th}}$. The
sensors differ only in *which* terms are non-trivial and *how* they are
identified.

---

## 8.2. Gyroscope Calibration

### 8.2.1. Error model

$$
\boldsymbol{\omega}_m = (\boldsymbol{I} + \boldsymbol{S}_g)\,\boldsymbol{\omega} + \boldsymbol{b}_g(T) + \boldsymbol{n}_g
$$

- $\boldsymbol{b}_g(T)$ — **bias**, the dominant gyro error and the one that
  matters most: it is integrated directly into attitude, so a $0.01\,\text{rad/s}$
  bias becomes a $0.6^\circ/\text{s}$ attitude drift. Strongly temperature
  dependent.
- $\boldsymbol{S}_g$ — scale + cross-axis. For MEMS rate gyros this is small;
  PX4 does **not** estimate it in the field. Only the bias is calibrated.
- $\boldsymbol{n}_g$ — white noise, left for EKF2 to absorb.

The key insight that makes gyro bias *observable without any reference* is that
**a stationary vehicle has $\boldsymbol{\omega} = \boldsymbol{0}$**. Therefore any
non-zero average output *is* the bias. No external truth (no gravity, no field
model) is needed — gravity gives the accelerometer a reference, the Earth field
gives the magnetometer one, but the gyro's reference is simply "zero motion."

### 8.2.2. Offline (preflight) bias estimate

The manual "gyro calibration" is the simplest estimator in the stack:

$$
\hat{\boldsymbol{b}}_g = \frac{1}{N}\sum_{k=1}^{N}\boldsymbol{\omega}_{m,k},
\qquad \boldsymbol{\omega}\equiv\boldsymbol{0}
$$

A fixed batch (a few hundred samples) is averaged while the vehicle is held
still. Two robustness mechanisms wrap this trivial mean:

1. **Outlier rejection** via a short sliding **median** of the stream, so a
   single bump does not corrupt the average.
2. **Motion detection**: the batch mean is compared against the median; if the
   vehicle was disturbed during collection (difference beyond a small angular
   threshold, on the order of a fraction of a degree per second) the run is
   rejected and retried. This guards the $\boldsymbol{\omega}=\boldsymbol{0}$
   assumption that the whole method rests on.

### 8.2.3. Online bias learning (continuous, while disarmed)

Because gyro bias drifts with temperature and time, PX4 also re-estimates it
continuously whenever the vehicle is **disarmed and still**, using a streaming
mean/variance estimator (Welford's algorithm — numerically stable single-pass
computation of running mean *and* variance). The theory it adds over a plain
average:

- **Gating on stillness.** Accumulation resets if the temperature moves more than
  ~1 °C or the measured acceleration vector shifts more than a small threshold —
  both indicate the $\boldsymbol{\omega}=\boldsymbol{0}$ premise is broken.
- **Confidence from variance.** An update to the stored offset is accepted only
  when the running variance is small (the window was genuinely quiet) *and* a
  minimum sample count is reached.
- **Hysteresis on commit.** The new offset replaces the old one only if it
  changed by **more than one standard deviation** of the estimate, or if the new
  window's variance is markedly better than the variance behind the currently
  stored value. This prevents the parameter from chattering on noise while still
  capturing genuine thermal drift.

This is conceptually a **per-axis Bayesian update**: a new measurement of the
bias is fused only when it is both precise (low variance) and significantly
different from the current belief.

### 8.2.4. Thermal compensation

Gyro bias is a strong, repeatable function of die temperature. PX4 models it as a
**polynomial in temperature** identified by a dedicated thermal-calibration run
(see §8.6). At runtime the polynomial output $\boldsymbol{b}_{\text{th}}(T)$ is
subtracted *before* the static offset, so the residual the online learner sees is
already temperature-flattened.

---

## 8.3. Accelerometer Calibration

### 8.3.1. Error model and the gravity reference

$$
\boldsymbol{a}_m = \boldsymbol{T}^{-1}\boldsymbol{a}_{\text{corr}} + \boldsymbol{b}_a + \boldsymbol{n}_a
\quad\Longleftrightarrow\quad
\boldsymbol{a}_{\text{corr}} = \boldsymbol{T}\,(\boldsymbol{a}_m - \boldsymbol{b}_a)
$$

- $\boldsymbol{b}_a$ — bias (offset).
- $\boldsymbol{T}$ — a $3\times3$ transform combining **scale factor** and, in
  principle, **cross-axis/misalignment**.

The reference that makes both observable: **a stationary accelerometer measures
specific force, whose magnitude must equal $g$** in every orientation. The locus
of the raw vectors over all attitudes is therefore (ideally) a sphere of radius
$g$ centred at the origin; bias shifts the centre, scale/misalignment deform it
into an ellipsoid. Calibration restores the sphere — exactly the same geometric
idea as the magnetometer (§8.4), but with a *known* radius $g$.

### 8.3.2. The 6-orientation procedure

Rather than a free-form ellipsoid fit, the accelerometer uses a **structured
6-position method** because the six axis-aligned attitudes make the algebra
exact and the user instructions unambiguous (level, on each side, nose up, nose
down, upside down). At each face one axis reads $\pm g$ and the others ~0. Many
samples per face are averaged to beat down noise.

**Bias** comes from the two *opposite* faces of each axis. If axis $i$ reads
$a^{+}_i$ pointing up and $a^{-}_i$ pointing down, the bias is the midpoint —
gravity cancels by symmetry:

$$
b_{a,i} = \tfrac{1}{2}\left(a^{+}_i + a^{-}_i\right)
$$

**Scale/transform** comes from solving, per axis, the linear system that maps the
three bias-removed "down" measurements to the known gravity vectors. Stacking the
three measured-minus-offset row vectors into a matrix $\boldsymbol{A}$:

$$
\boldsymbol{T} = \boldsymbol{A}^{-1}\, g
$$

This is a closed-form least-information solution: 6 positions give exactly the 12
unknowns (9 in $\boldsymbol{T}$ + 3 in $\boldsymbol{b}_a$), so the system is solved
by matrix inversion rather than over-determined least squares. (The broader
literature frames the same problem as a least-squares minimisation of
$\big|\,\lVert\boldsymbol{a}_m-\boldsymbol{b}\rVert - g\,\big|$ over many random
orientations; PX4 chooses the structured, exactly-determined variant for
operational simplicity.)

**Frame handling.** The transform is solved in the calibration (level) frame and
then rotated into the sensor frame by a similarity transform with the board
rotation $\boldsymbol{R}$:

$$
\boldsymbol{b}_a^{S} = \boldsymbol{R}^\top \boldsymbol{b}_a,
\qquad
\boldsymbol{T}^{S} = \boldsymbol{R}^\top \boldsymbol{T}\,\boldsymbol{R}
$$

### 8.3.3. A deliberate simplification: diagonal scale only

Although the 6-position solution produces a **full $3\times3$** transform
$\boldsymbol{T}^{S}$ (which *could* capture cross-axis misalignment), PX4 stores
**only its diagonal** as the per-axis scale factor; the off-diagonal misalignment
terms are discarded. The runtime correction is therefore an *element-wise* scale,
not a matrix product:

$$
\boldsymbol{a}_{\text{corr}} = \boldsymbol{R}_{BS}\,\big(\operatorname{diag}(\boldsymbol{s}_a)\odot(\boldsymbol{a}_m - \boldsymbol{b}_a - \boldsymbol{b}_{\text{th}})\big)
$$

The rationale: MEMS accelerometer cross-axis errors are small, the residual is
well within EKF2's accel noise budget, and a diagonal model is robust against an
imperfect 6-position procedure that would otherwise inject spurious off-diagonal
terms. The accelerometer, like the gyro, also has a **temperature-polynomial**
offset $\boldsymbol{b}_{\text{th}}(T)$ and its slowly-varying residual bias is
tracked by EKF2 in flight.

---

## 8.4. Magnetometer Calibration

The magnetometer is the most involved because its environment, not just the chip,
distorts the measurement — and that environment is fixed to the airframe, so it
*can* be calibrated out.

### 8.4.1. Hard-iron and soft-iron error model

$$
\boldsymbol{m}_m = \boldsymbol{D}\,\boldsymbol{R}_{BW}\,\boldsymbol{m}_W + \boldsymbol{b}_m^{\text{hard}} + \boldsymbol{n}_m
$$

- $\boldsymbol{m}_W$ — the **Earth field** at the current location: known magnitude
  and inclination from the World Magnetic Model (WMM), looked up by GPS lat/lon.
- $\boldsymbol{b}_m^{\text{hard}}$ — **hard-iron** offset. A *constant* additive
  field from permanently magnetised material rigidly attached to the frame
  (magnets in motors/speakers, magnetised screws). Shifts the field-vector cloud
  off-centre. This is the dominant and most important mag error.
- $\boldsymbol{D}$ — **soft-iron** matrix. Ferromagnetic material that is *not*
  itself magnetised but *re-shapes* the ambient field (re-routes flux),
  multiplying the true field by a $3\times3$ matrix. Turns the sphere into a
  tilted ellipsoid. Includes the sensor's own scale/misalignment.

### 8.4.2. Geometry: sphere on the inside, ellipsoid in practice

With no errors, rotating the vehicle through all attitudes traces the tip of
$\boldsymbol{m}_m$ over a **sphere** of radius $\lVert\boldsymbol{m}_W\rVert$
centred at the origin (rotation preserves magnitude). Hard-iron translates the
sphere; soft-iron deforms and tilts it into an **ellipsoid**. Calibration finds
the affine map that sends the measured ellipsoid back to a centred sphere:

$$
\boldsymbol{m}_{\text{corr}} = \boldsymbol{D}^{-1}\big(\boldsymbol{m}_m - \boldsymbol{b}_m^{\text{hard}}\big)
$$

so that $\lVert\boldsymbol{m}_{\text{corr}}\rVert \approx \lVert\boldsymbol{m}_W\rVert$
for every orientation. PX4 stores the **inverse** soft-iron directly (diagonal
$\boldsymbol{s}$ + off-diagonal terms) plus the hard-iron offset, so the runtime
correction is one offset subtraction and one matrix multiply.

### 8.4.3. Two-stage fit: sphere then ellipsoid, by Levenberg–Marquardt

PX4 does **not** use a closed-form algebraic ellipsoid fit. It minimises a
geometric residual with **Levenberg–Marquardt (LM)** — the standard
trust-region blend between Gauss–Newton (fast near the solution) and gradient
descent (robust far from it), interpolated by a damping factor $\lambda$ that is
raised when a step fails and lowered when it succeeds.

The amount of data available decides the model order, which avoids
over-fitting a poorly-excited dataset:

- **Sphere fit (offset only)** when the rotation coverage is limited
  (≈ two sides excited). Parameters: centre $\boldsymbol{b}$ (hard-iron) and
  radius $r$. The soft-iron is left at identity.
- **Ellipsoid fit (offset + scale)** when coverage is rich (three or more sides).
  Parameters: offset (3) + diagonal scale (3) + off-diagonal (3) = 9.

The cost minimised is the **radial residual** — distance from each transformed
sample to the fitted surface:

$$
J(\boldsymbol{\theta}) = \sum_k \Big(r - \big\lVert \boldsymbol{M}(\boldsymbol{m}_{m,k} - \boldsymbol{b}) \big\rVert\Big)^2
$$

with $\boldsymbol{M}=\boldsymbol{I}$ for the sphere stage and the lower-triangular
(diag + off-diag) scale for the ellipsoid stage. LM needs the Jacobian of each
residual w.r.t. the parameters; PX4 derives these analytically (cheaper and more
stable than finite differences on an embedded target).

**Convergence / acceptance criteria** combine several guards so a bad dataset is
rejected rather than blindly trusted:

- minimum and maximum iteration counts (run long enough, but bounded);
- a cost-improvement threshold and a step-size threshold (stop when either the
  fit stops improving or the parameter step becomes negligible);
- a **physical sanity bound on the fitted radius**: the sphere radius must lie in
  the plausible Earth-field range (~0.2–0.7 Gauss). A fit outside this is
  rejected, catching divergence and grossly disturbed data.

**Data hygiene during collection.** Samples are accepted only after enough
rotation has occurred (measured by integrating the gyro), and new samples are
**spatially thinned** — a candidate too close to an already-stored point is
dropped. This keeps the point cloud roughly uniform over the sphere so the fit is
not dominated by whatever orientation the operator dwelt in.

**Reference radius from the WMM.** When a GPS fix is available the target radius
$\lVert\boldsymbol{m}_W\rVert$ is the WMM field strength at the current location;
otherwise a default is used. Using the true local magnitude lets the fit also
correct overall scale, not just shape.

### 8.4.4. Power / current compensation

Motor currents create a magnetic field proportional to the current drawn, which
is *not* attitude-dependent and so cannot be captured by hard/soft-iron. PX4
models it as an additional term proportional to a power proxy $P$ (throttle, or
measured battery current):

$$
\boldsymbol{m}_{\text{corr}} = \boldsymbol{D}^{-1}\big(\boldsymbol{m}_m + P\,\boldsymbol{c}_P - \boldsymbol{b}_m^{\text{hard}}\big)
$$

The compensation vector $\boldsymbol{c}_P$ is identified separately (e.g. a
high-current ground run). This matters because the current-induced field can
otherwise masquerade as a yaw error that scales with throttle.

### 8.4.5. Online magnetometer bias learning — the angular-rate method

A standout piece of theory in PX4: the magnetometer **hard-iron bias is estimated
online using the gyroscope**, with **no knowledge of the Earth field's direction
or magnitude required**. The method follows Troni & Whitcomb's adaptive bias
estimator for 3-D field sensors aided by angular rate.

The principle: the *true* field is constant in the world frame, so in the body
frame it rotates rigidly with the vehicle. Its body-frame derivative is therefore
purely kinematic — the cross product of angular rate with the field — and is
**independent of any additive bias** only if the bias is correct. Concretely, the
true (bias-free) field $\boldsymbol{f}=\boldsymbol{m}-\boldsymbol{b}$ obeys

$$
\dot{\boldsymbol{f}} = -\,\boldsymbol{\omega}\times\boldsymbol{f}
$$

PX4 propagates a predicted field with this kinematic model and drives an adaptive
law that pushes the bias estimate to make the prediction match the measurement:

$$
\hat{\boldsymbol{f}}_{n+1} = \hat{\boldsymbol{f}}_n + \big(-\boldsymbol{\omega}\times(\hat{\boldsymbol{f}}_n - \hat{\boldsymbol{b}}_n)\big)\Delta t,
\qquad
\hat{\boldsymbol{b}}_{n+1} = \hat{\boldsymbol{b}}_n + k\,\big(-\boldsymbol{\omega}\times \boldsymbol{\nu}\big)\Delta t
$$

with innovation $\boldsymbol{\nu} = \boldsymbol{m} - \hat{\boldsymbol{f}}$ and
learning gain $k$. **Observability requires rotation** — the bias is only
excited when $\boldsymbol{\omega}\neq\boldsymbol{0}$, exactly why the manual
procedure asks you to spin the vehicle and why the online learner gates on motion.
Acceptance is gated on the estimate being significant, on a **fitness ratio**
between angular motion and bias-change rate, and on a sustained convergence
window before the learned offset is written back. (Separately, EKF2 also estimates
mag bias as part of its state — these are complementary refinements of the same
quantity.)

---

## 8.5. The Sensor-Mounting Rotation $\boldsymbol{R}_{BS}$

Independent of the numeric calibrations above, each sensor carries a discrete
**board/sensor rotation** (one of a finite enumeration of 90°/45° mountings, plus
fine roll/pitch/yaw for external mags). This is not estimated from data; it is a
configuration choice describing how the chip is physically mounted relative to
the FRD body axes. It is applied *last* in every correction so that offsets and
scales are identified in the sensor's native frame and only then rotated into the
body frame EKF2 works in.

---

## 8.6. Temperature Compensation (all inertial sensors + baro)

MEMS bias drifts substantially with die temperature, and most of that drift is
**repeatable**, hence calibratable. PX4 fits a **low-order polynomial** in
temperature for the offset of each axis of each sensor:

$$
\boldsymbol{b}_{\text{th}}(T) = \sum_{j=0}^{n} \boldsymbol{c}_j\,(T - T_{\text{ref}})^{\,j}
$$

- Identified by **ordinary least squares** (normal equations on the Vandermonde
  matrix of relative temperatures) during a dedicated thermal-soak run that ramps
  the board across a temperature range while logging bias.
- Expressed relative to a reference temperature $T_{\text{ref}}$ (midpoint of the
  soak) so the constant term is well-conditioned.
- The run enforces a **minimum temperature rise** before it accepts the fit, so
  the polynomial is supported by real spread rather than extrapolated from a
  narrow band.

At runtime $\boldsymbol{b}_{\text{th}}(T)$ is subtracted ahead of the static
calibration, so the static offset and the online learners only deal with the
temperature-flattened residual.

---

## 8.7. What PX4 Estimates vs. What It Leaves to EKF2

| Quantity | Method | When | Where it lives |
|---|---|---|---|
| Gyro bias (static) | average at rest, motion-rejected | manual / online (Welford, disarmed) | `CAL_GYRO*_OFF` |
| Gyro scale/cross-axis | **not calibrated** | — | — |
| Gyro bias (residual drift) | EKF2 state | in flight | filter state |
| Accel bias | 6-position midpoints | manual | `CAL_ACC*_OFF` |
| Accel scale (diagonal) | 6-position $\boldsymbol{A}^{-1}g$, diagonal kept | manual | `CAL_ACC*_SCALE` |
| Accel cross-axis | computed then **discarded** | — | — |
| Accel bias (residual) | EKF2 state | in flight | filter state |
| Mag hard-iron | LM sphere/ellipsoid fit; online angular-rate learner; EKF2 state | manual + online | `CAL_MAG*_OFF` |
| Mag soft-iron (diag + off-diag) | LM ellipsoid fit | manual | `CAL_MAG*_SCALE`, `CAL_MAG*_ODIAG` |
| Mag current comp | proportional-to-current fit | dedicated run | `CAL_MAG*_COMP` |
| Thermal offsets (gyro/accel/baro) | least-squares polynomial in $T$ | thermal soak | `TC_*` |
| Mounting rotation | configuration, not estimated | setup | `CAL_*_ROT` |

The throughline: **PX4 calibrates exactly the structured errors it can observe
from a free, available reference** — zero motion for gyro bias, gravity for the
accelerometer, the Earth field (and rigid-rotation kinematics) for the
magnetometer — and consciously *omits* the small, hard-to-observe terms (gyro
scale, accel cross-axis) rather than risk fitting noise. Everything that remains
slowly-varying is handed to EKF2 as an estimated state.

---

## 8.8. Theoretical Background & Keywords

| Topic | Keywords for deeper study |
|---|---|
| IMU error characterisation | `Allan variance`, `angle random walk`, `bias instability`, `rate random walk`, `noise density`, `IEEE Std 952` |
| Accelerometer calibration | `multi-position calibration`, `six-position method`, `least-squares scale/bias`, `gravity-norm constraint` |
| Magnetometer model | `hard-iron soft-iron`, `ellipsoid fitting`, `sphere fit`, `Levenberg–Marquardt`, `Merayo algorithm`, `eCompass calibration` |
| Online mag bias | `angular-rate-aided bias estimation`, `Troni–Whitcomb`, `adaptive observer`, `field-sensor bias` |
| Earth field reference | `World Magnetic Model (WMM)`, `magnetic inclination/declination` |
| Thermal calibration | `temperature compensation`, `polynomial bias model`, `thermal soak`, `Vandermonde least squares` |
| Reference frames | `sensor-to-body rotation`, `similarity transform`, `DCM` |

### Recommended reading
- Troni, G. & Whitcomb, L. — *Adaptive Estimation of Measurement Bias in
  Three-Dimensional Field Sensors with Angular-Rate Sensors: Theory and
  Comparative Experimental Evaluation*, RSS 2013.
  [paper (PDF)](https://www.roboticsproceedings.org/rss09/p50.pdf)
- NXP AN4246 — *Calibrating an eCompass in the Presence of Hard and Soft Iron
  Interference*.
  [application note (PDF)](https://www.nxp.com/docs/en/application-note/AN4246.pdf)
- Elkaim, G. — *Calibration of Strapdown Magnetometers in the Magnetic Field
  Domain*.
  [paper (PDF)](https://users.soe.ucsc.edu/~elkaim/Documents/magcal.pdf)
- Wu et al. — *An improved magnetometer calibration and compensation method based
  on Levenberg–Marquardt for multirotor UAV*, 2020.
  [article](https://journals.sagepub.com/doi/full/10.1177/0020294019890627)
- *Inertial Sensor Noise Analysis Using Allan Variance* — MathWorks.
  [guide](https://www.mathworks.com/help/fusion/ug/inertial-sensor-noise-analysis-using-allan-variance.html)
- Titterton & Weston, *Strapdown Inertial Navigation Technology* (sensor error
  models, calibration); Groves, *Principles of GNSS, Inertial, and Multisensor
  Integrated Navigation*.

> **Next:** [09 — Filtering of IMU & Mag before EKF2](09_imu_mag_filtering_theory.md)
> covers what happens to these *calibrated* signals — coning/sculling integration,
> low-pass, static & dynamic notch — before they reach the estimator.
