# UWB EKF2 v2 — Native Aiding Source + GPS-Denied Operation (Implementation Plan)

> Plan for v2 of UWB→EKF2 fusion. Ordered, independently buildable/testable tasks; each ends in a commit verified at runtime in SITL before committing. Checkboxes (`- [ ]`) track progress.

**Goal:** Make UWB a **first-class horizontal-position aiding source** in EKF2 (like GPS), so the three modes work:
1. **UWB-only** (indoor, GPS-denied) — UWB initializes and holds position.
2. **UWB + GPS** — both fuse together.
3. **No UWB** — GPS as before; UWB simply inactive.

**Starting point (v1, committed):** UWB range fusion works as a self-contained `UwbRange` class (Pattern B, AuxGlobalPosition-style), verified `fused=true`, innovation ~1 cm. But it (a) is module-coupled (`MODULE_NAME`, not unit-testable), and (b) does **not** register as a recognized aiding source — so the EKF's `dead_reckoning`/position-validity logic ignores it, which blocks GPS-denied flight.

**Approach (two design changes):**
- **Pattern A refactor** — move the working scalar-range fusion from the `UwbRange` class into the *native* EKF2 pattern (the one GPS/auxvel use): uORB-free `Ekf::controlUwbRangeFusion()` + `setUwbData()` + ring buffer in `EstimatorInterface`, with the `EKF2` module wrapper doing the uORB I/O and params. This makes the core uORB-free (unit-testable) and lets the fusion touch core control state directly.
- **First-class aiding** — add a `uwb` flag to `filter_control_status_u`, count it in the horizontal-position-aiding helpers and `updateHorizontalDeadReckoningstatus()`, set `_time_last_hor_pos_fuse` on each successful fuse, and add start/stop + a trilateration-based position **initialization** so UWB can bring up position with no GPS.

**Tech stack:** PX4 EKF2 (ESKF), uORB, CMake/Kconfig, SITL (`make px4_sitl gz_f450-uwb_uwb`). Reuses the existing `sensor_uwb` topic and `EKF2_UWB_*` params from v1.

**Verification reality:** primary check is build + SITL `listener`/`ekf2 status` + groundtruth, **confirmed by the user before each commit**. pxh shell has **no `;` chaining** — one `param set` per line. Phase 1 also unlocks a uORB-free gtest (optional).

**Scope guard:** ideal LOS, ≤4 surveyed anchors (params), 25 Hz. NLOS/adaptive-R and anchor self-survey remain out of scope.

---

## Reference patterns (verbatim anchors from the current tree)

- `filter_control_status_u` union — `EKF/common.h:565-627` (hand-written, mirrored in `msg/EstimatorStatusFlags.msg`). Free bits 45-63.
- `getNumberOfActiveHorizontalPositionAidingSources()` — `estimator_interface.cpp:620` (`gnss_pos + ev_pos + aux_gpos`).
- `updateHorizontalDeadReckoningstatus()` position-aiding check — `ekf_helper.cpp:813`.
- Native plumbing template (auxvel): `setAuxVelData` `estimator_interface.cpp:403`; `controlAuxVelFusion` `aid_sources/auxvel/auxvel_fusion.cpp:36`; wrapper `EKF2::UpdateAuxVelSample` `EKF2.cpp:2120`.
- GPS start/stop/origin: `aid_sources/gnss/gps_control.cpp` (`controlGnssPosFusion` ~171, `stopGnssFusion`); `resetHorizontalPositionTo(Vector2f,Vector2f)` `position_fusion.cpp:167`; `resetAidSourceStatusZeroInnovation(1d)` `ekf_helper.cpp:1140`.
- EV runs **without** a global origin (local frame): `startEvPosFusion` resets via `resetHorizontalPositionTo(measurement,var)` with no origin — the model for UWB-only.

---

## PHASE 1 — Refactor Pattern B → Pattern A (behavior-preserving)

End state: identical runtime behavior to v1 (per-anchor scalar fusion, `fused=true`, innovation ~cm), but the fusion lives in the EKF core (uORB-free) and the `EKF2` wrapper does uORB I/O. The `UwbRange` class is deleted.

### Task 1: `common.h` — sample struct + params struct fields

**Files:** Modify `src/modules/ekf2/EKF/common.h`

- [ ] **Step 1: Add the sample struct** (near `auxVelSample`, under guard)
```cpp
#if defined(CONFIG_EKF2_UWB)
struct uwbSample {
	uint64_t time_us{};    ///< timestamp of the measurement (uSec)
	uint8_t  anchor_id{};  ///< 0-indexed anchor slot
	float    range{};      ///< measured distance (m)
	float    range_var{};  ///< measurement variance (m^2); 0 = use EKF2_UWB_NOISE^2
};
#endif // CONFIG_EKF2_UWB
```

- [ ] **Step 2: Add params** inside `struct parameters { ... }` (near the GNSS block)
```cpp
#if defined(CONFIG_EKF2_UWB)
	int32_t  uwb_ctrl{0};                 ///< 1=enable UWB range fusion
	float    uwb_delay_ms{50.f};          ///< UWB measurement delay relative to IMU (ms)
	float    uwb_noise{0.05f};            ///< baseline range sigma (m)
	float    uwb_innov_gate{5.f};         ///< range innovation gate (sigma)
	int32_t  uwb_n_anchors{4};            ///< number of configured anchors (1-4)
	float    uwb_anchor_n[4]{};           ///< anchor North positions (m), EKF NED origin frame
	float    uwb_anchor_e[4]{};           ///< anchor East positions (m)
	float    uwb_anchor_d[4]{};           ///< anchor Down positions (m)
#endif // CONFIG_EKF2_UWB
```

- [ ] **Step 3: Build** `make px4_sitl_default 2>&1 | tail -5` → clean (unused so far).
- [ ] **Step 4: Commit** `ekf2: add uwbSample + UWB params to EKF core (v2 refactor)`

---

### Task 2: `EstimatorInterface` — `setUwbData()` + ring buffer

**Files:** Modify `src/modules/ekf2/EKF/estimator_interface.h`, `estimator_interface.cpp`

- [ ] **Step 1: estimator_interface.h** — declaration (public) + buffer member (protected), mirror auxvel
```cpp
#if defined(CONFIG_EKF2_UWB)
	void setUwbData(const uwbSample &uwb_sample);
#endif // CONFIG_EKF2_UWB
```
```cpp
#if defined(CONFIG_EKF2_UWB)
	RingBuffer<uwbSample> *_uwb_buffer {nullptr};
	uint64_t _time_last_uwb_buffer_push{0};
#endif // CONFIG_EKF2_UWB
```

- [ ] **Step 2: estimator_interface.cpp** — implementation (mirror `setAuxVelData`)
```cpp
#if defined(CONFIG_EKF2_UWB)
void EstimatorInterface::setUwbData(const uwbSample &uwb_sample)
{
	if (!_initialised) {
		return;
	}

	if (_uwb_buffer == nullptr) {
		_uwb_buffer = new RingBuffer<uwbSample>(_obs_buffer_length);

		if (_uwb_buffer == nullptr || !_uwb_buffer->valid()) {
			delete _uwb_buffer;
			_uwb_buffer = nullptr;
			printBufferAllocationFailed("UWB");
			return;
		}
	}

	const int64_t time_us = uwb_sample.time_us
				- static_cast<int64_t>(_params.uwb_delay_ms * 1000)
				- static_cast<int64_t>(_dt_ekf_avg * 5e5f);

	if (time_us >= static_cast<int64_t>(_uwb_buffer->get_newest().time_us + _min_obs_interval_us)) {
		uwbSample sample = uwb_sample;
		sample.time_us = time_us;
		_uwb_buffer->push(sample);
		_time_last_uwb_buffer_push = _time_latest_us;
	}
}
#endif // CONFIG_EKF2_UWB
```

- [ ] **Step 3: Build** → clean. **Step 4: Commit** `ekf2: add setUwbData + ring buffer in EstimatorInterface`

---

### Task 3: Core fusion as `Ekf` methods (delete the class)

**Files:** Create `src/modules/ekf2/EKF/aid_sources/uwb/uwb_range_control.cpp`; Modify `ekf.h`; **Delete** `aid_sources/uwb/uwb_range.{hpp,cpp}`

- [ ] **Step 1: ekf.h** — replace the v1 friend/include/member with native decls.
  Remove: `#include "aid_sources/uwb/uwb_range.hpp"`, `friend class UwbRange;`, `UwbRange _uwb_range {};`.
  Add (control method group, under `#if defined(CONFIG_EKF2_UWB)`):
```cpp
#if defined(CONFIG_EKF2_UWB)
	void controlUwbRangeFusion(const imuSample &imu_delayed);
	bool fuseUwbRange(const uwbSample &sample, estimator_aid_source1d_s &aid_src);
	matrix::Vector3f getUwbAnchorPos(uint8_t anchor_id) const;
	void stopUwbFusion();

	estimator_aid_source1d_s _aid_src_uwb[4] {};
	uint64_t _time_last_uwb_fuse{0};

	const auto &aid_src_uwb() const { return _aid_src_uwb; }
#endif // CONFIG_EKF2_UWB
```

- [ ] **Step 2: Create `aid_sources/uwb/uwb_range_control.cpp`** (uORB-free; moves the v1 fuse logic, now an `Ekf` member so `VectorState`/`State`/`P`/`_gpos` are in scope — no `Ekf::` qualifier, no friend)
```cpp
#include "ekf.h"

matrix::Vector3f Ekf::getUwbAnchorPos(uint8_t anchor_id) const
{
	switch (anchor_id) {
	case 0: return matrix::Vector3f(_params.uwb_anchor_n[0], _params.uwb_anchor_e[0], _params.uwb_anchor_d[0]);
	case 1: return matrix::Vector3f(_params.uwb_anchor_n[1], _params.uwb_anchor_e[1], _params.uwb_anchor_d[1]);
	case 2: return matrix::Vector3f(_params.uwb_anchor_n[2], _params.uwb_anchor_e[2], _params.uwb_anchor_d[2]);
	case 3: return matrix::Vector3f(_params.uwb_anchor_n[3], _params.uwb_anchor_e[3], _params.uwb_anchor_d[3]);
	default: return matrix::Vector3f();
	}
}

bool Ekf::fuseUwbRange(const uwbSample &sample, estimator_aid_source1d_s &aid_src)
{
	// vehicle NED position relative to the EKF origin at the delayed horizon
	float pn, pe;
	if (_local_origin_lat_lon.isInitialized()) {
		_local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg(), pn, pe);
	} else {
		// local-frame-only (GPS-denied): position is already in the anchor/local frame
		const Vector2f p = getLocalHorizontalPosition();
		pn = p(0); pe = p(1);
	}
	const float pd = -getLocalVerticalPosition(); // see note; or -( _gpos.altitude() - originAlt )

	const Vector3f pos_ned(pn, pe, pd);
	const Vector3f anchor = getUwbAnchorPos(sample.anchor_id);
	const Vector3f diff = pos_ned - anchor;
	const float predicted = diff.norm();

	if (predicted < 0.1f) { return false; }

	const Vector3f los = diff / predicted;
	VectorState H;
	H.setZero();
	H(State::pos.idx + 0) = los(0);
	H(State::pos.idx + 1) = los(1);
	H(State::pos.idx + 2) = los(2);

	float R = math::max(sq(_params.uwb_noise), 1e-4f);
	if (sample.range_var > 0.f && sample.range_var < 100.f) { R = sample.range_var; }

	const float innovation = predicted - sample.range;     // h(x) - z (PX4 convention)
	const float innovation_var = H.dot(P * H) + R;

	updateAidSourceStatus(aid_src, sample.time_us, sample.range, R, innovation, innovation_var,
			      math::max(_params.uwb_innov_gate, 1.f));

	if (aid_src.innovation_rejected) { return false; }

	VectorState K = P * H / aid_src.innovation_variance;
	measurementUpdate(K, H, R, aid_src.innovation);

	aid_src.fused = true;
	aid_src.time_last_fuse = _time_delayed_us;
	_time_last_uwb_fuse = _time_delayed_us;
	_time_last_hor_pos_fuse = _time_delayed_us;   // <-- Phase 2 makes this matter
	return true;
}

void Ekf::stopUwbFusion()
{
	if (_control_status.flags.uwb) {            // flag added in Phase 2
		_control_status.flags.uwb = false;
		ECL_INFO("stopping UWB fusion");
	}
}

void Ekf::controlUwbRangeFusion(const imuSample &imu_delayed)
{
	if (_uwb_buffer == nullptr) { return; }

	if (_params.uwb_ctrl == 0) {
		stopUwbFusion();
		return;
	}

	uwbSample sample;
	while (_uwb_buffer->pop_first_older_than(imu_delayed.time_us, &sample)) {
		if (!_control_status.flags.tilt_align) { continue; }
		if (sample.anchor_id >= (uint8_t)_params.uwb_n_anchors) { continue; }

		// Phase 3 adds: init position via trilateration when no horizontal aiding yet.
		// Phase 1: require an existing position origin (GPS) as in v1.
		if (!isHorizontalAidingActive()) { continue; }

		fuseUwbRange(sample, _aid_src_uwb[sample.anchor_id]);
	}
}
```
> **Note (position accessors):** confirm the exact getters for delayed-horizon local NED. `getLocalHorizontalPosition()` exists (used by GPS/EV). For vertical, use the same approach GPS uses for `_gpos.altitude()` vs origin altitude. Keep the v1-proven projection (`_local_origin_lat_lon.project(_gpos…)` + origin-alt) as the GPS-active path; the local-only branch is for Phase 3.

- [ ] **Step 3: Delete** `aid_sources/uwb/uwb_range.hpp` and `uwb_range.cpp` (`git rm`).
- [ ] **Step 4: Revert the v1 `Ekf::updateParameters()` hook** (`_uwb_range.updateParameters()` in `ekf.cpp`) — no longer needed; params now live in `_params` (refreshed by EKF2 wrapper, Task 5).
- [ ] **Step 5: Build** (expect errors until Tasks 4-6 wire the rest; OK to defer build to Task 6). **Commit** with Task 6.

---

### Task 4: `control.cpp` — native call (no MODULE_NAME guard)

**Files:** Modify `src/modules/ekf2/EKF/control.cpp`

- [ ] **Step 1:** Replace the v1 block
```cpp
#if defined(CONFIG_EKF2_UWB) && defined(MODULE_NAME)
	_uwb_range.update(*this, imu_delayed);
#endif
```
with (next to `controlGpsFusion`, like a native source):
```cpp
#if defined(CONFIG_EKF2_UWB)
	controlUwbRangeFusion(imu_delayed);
#endif // CONFIG_EKF2_UWB
```

---

### Task 5: `EKF2` wrapper — subscription, feed, params, publish

**Files:** Modify `src/modules/ekf2/EKF2.hpp`, `EKF2.cpp`

- [ ] **Step 1: EKF2.hpp** — replace v1's `DEFINE_PARAMETERS` ownership; now ParamExt bound to `_params` (mirror GNSS):
```cpp
#if defined(CONFIG_EKF2_UWB)
	uORB::Subscription _sensor_uwb_sub{ORB_ID(sensor_uwb)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_uwb_pub{ORB_ID(estimator_aid_src_uwb)};
	hrt_abstime _status_uwb_pub_last{0};
	void UpdateUwbSample(ekf2_timestamps_s &ekf2_timestamps);
#endif // CONFIG_EKF2_UWB
```
DEFINE_PARAMETERS (ParamExt → _params):
```cpp
#if defined(CONFIG_EKF2_UWB)
	(ParamExtInt<px4::params::EKF2_UWB_CTRL>)   _param_ekf2_uwb_ctrl,
	(ParamExtFloat<px4::params::EKF2_UWB_DELAY>) _param_ekf2_uwb_delay,
	(ParamExtFloat<px4::params::EKF2_UWB_NOISE>) _param_ekf2_uwb_noise,
	(ParamExtFloat<px4::params::EKF2_UWB_GATE>)  _param_ekf2_uwb_gate,
	(ParamExtInt<px4::params::EKF2_UWB_N_ANCH>)  _param_ekf2_uwb_n_anch,
	(ParamExtFloat<px4::params::EKF2_UWB_A0_N>) _param_ekf2_uwb_a0_n,
	(ParamExtFloat<px4::params::EKF2_UWB_A0_E>) _param_ekf2_uwb_a0_e,
	(ParamExtFloat<px4::params::EKF2_UWB_A0_D>) _param_ekf2_uwb_a0_d,
	/* A1_N..A3_D — 9 more, same pattern */
#endif // CONFIG_EKF2_UWB
```

- [ ] **Step 2: EKF2.cpp ctor** — bind ParamExt to `_params` (mirror GNSS block):
```cpp
#if defined(CONFIG_EKF2_UWB)
	_param_ekf2_uwb_ctrl(_params->uwb_ctrl),
	_param_ekf2_uwb_delay(_params->uwb_delay_ms),
	_param_ekf2_uwb_noise(_params->uwb_noise),
	_param_ekf2_uwb_gate(_params->uwb_innov_gate),
	_param_ekf2_uwb_n_anch(_params->uwb_n_anchors),
	_param_ekf2_uwb_a0_n(_params->uwb_anchor_n[0]), _param_ekf2_uwb_a0_e(_params->uwb_anchor_e[0]), _param_ekf2_uwb_a0_d(_params->uwb_anchor_d[0]),
	/* A1..A3 likewise */
#endif // CONFIG_EKF2_UWB
```

- [ ] **Step 3: EKF2.cpp** — `UpdateUwbSample` (mirror `UpdateAuxVelSample`)
```cpp
#if defined(CONFIG_EKF2_UWB)
void EKF2::UpdateUwbSample(ekf2_timestamps_s &ekf2_timestamps)
{
	sensor_uwb_s uwb;

	while (_sensor_uwb_sub.update(&uwb)) {
		if (uwb.mac >= (uint16_t)_param_ekf2_uwb_n_anch.get()) { continue; }
		if (!PX4_ISFINITE(uwb.distance) || uwb.distance <= 0.f) { continue; }

		uwbSample sample{};
		sample.time_us   = uwb.timestamp;       // sensor_uwb has no timestamp_sample
		sample.anchor_id = (uint8_t)uwb.mac;
		sample.range     = uwb.distance;
		sample.range_var = 0.f;
		_ekf.setUwbData(sample);
	}
}
#endif // CONFIG_EKF2_UWB
```

- [ ] **Step 4: EKF2.cpp Run()** — call `UpdateUwbSample(ekf2_timestamps);` next to `UpdateGpsSample`.
- [ ] **Step 5: EKF2.cpp PublishAidSourceStatus()** — publish per anchor:
```cpp
#if defined(CONFIG_EKF2_UWB)
	for (int i = 0; i < math::min(_param_ekf2_uwb_n_anch.get(), (int32_t)4); i++) {
		PublishAidSourceStatus(_ekf.aid_src_uwb()[i], _status_uwb_pub_last, _estimator_aid_src_uwb_pub);
	}
#endif // CONFIG_EKF2_UWB
```

---

### Task 6: CMake + build + verify Phase 1 (parity with v1)

**Files:** Modify `src/modules/ekf2/EKF/CMakeLists.txt`, `src/modules/ekf2/CMakeLists.txt`

- [ ] **Step 1:** In **both** CMakeLists, replace `uwb/uwb_range.cpp` with `uwb/uwb_range_control.cpp` (EKF/ uses `aid_sources/uwb/uwb_range_control.cpp`; module uses `EKF/aid_sources/uwb/uwb_range_control.cpp`). Keep `params_uwb.yaml`.
- [ ] **Step 2: Build** `make px4_sitl_default 2>&1 | tail -20` → clean link.
- [ ] **Step 3: USER runtime verify (parity):** run SITL, set anchors (one per line) + `EKF2_UWB_CTRL 1`, `listener estimator_aid_src_uwb 12` → `fused=true`, innovation ~cm, stable (same as v1).
- [ ] **Step 4: Commit** (after user OK) `ekf2: refactor UWB fusion to native Ekf pattern (uORB-free core)`

---

## PHASE 2 — `control_status.flags.uwb` (first-class horizontal aiding)

End state: with UWB fusing, EKF recognizes it as horizontal-position aiding → `dead_reckoning` false, position valid, `ekf2 status` shows the flag. (Still requires an origin from GPS to start — UWB-only init is Phase 3.)

### Task 7: Add the `uwb` control-status flag

**Files:** `EKF/common.h`, `msg/EstimatorStatusFlags.msg`, `estimator_interface.h`, `estimator_interface.cpp`, `ekf_helper.cpp`, `EKF2.cpp`

- [ ] **Step 1: common.h** — add to `filter_control_status_u.flags` (next free bit, 45):
```cpp
		uint64_t uwb                     : 1; ///< 45 - true if UWB range fusion is intended
```
- [ ] **Step 2: msg/EstimatorStatusFlags.msg** — mirror: add `bool cs_uwb` (keep order/bit in sync) and bump any count fields if present.
- [ ] **Step 3: estimator_interface.h** — helpers:
```cpp
void enableControlStatusUwb()  { _control_status.flags.uwb = true; }
void disableControlStatusUwb() { _control_status.flags.uwb = false; }
```
- [ ] **Step 4: estimator_interface.cpp** — count it:
```cpp
int EstimatorInterface::getNumberOfActiveHorizontalPositionAidingSources() const
{
	return int(_control_status.flags.gnss_pos)
	       + int(_control_status.flags.ev_pos)
	       + int(_control_status.flags.aux_gpos)
	       + int(_control_status.flags.uwb);
}
```
and add `+ int(_control_status.flags.uwb)` to `getNumberOfActiveHorizontalAidingSources()`.
- [ ] **Step 5: ekf_helper.cpp** `updateHorizontalDeadReckoningstatus()` — add to the position-aiding check:
```cpp
	if ((_control_status.flags.gnss_pos || _control_status.flags.ev_pos
	     || _control_status.flags.aux_gpos || _control_status.flags.uwb)
	    && isRecent(_time_last_hor_pos_fuse, _params.no_aid_timeout_max)) {
		inertial_dead_reckoning = false;
	}
```
- [ ] **Step 6: EKF2.cpp** `PublishEstimatorStatus()` — `status_flags.cs_uwb = _ekf.control_status_flags().uwb;`
- [ ] **Step 7: Build** → clean (regenerates EstimatorStatusFlags).

### Task 8: Set/clear the flag in the control loop

**Files:** `aid_sources/uwb/uwb_range_control.cpp`

- [ ] **Step 1:** In `controlUwbRangeFusion`, after a successful fuse of at least one anchor this cycle, set the flag; add a timeout-stop:
```cpp
	bool any_fused = false;
	uwbSample sample;
	while (_uwb_buffer->pop_first_older_than(imu_delayed.time_us, &sample)) {
		if (!_control_status.flags.tilt_align) { continue; }
		if (sample.anchor_id >= (uint8_t)_params.uwb_n_anchors) { continue; }
		if (!isHorizontalAidingActive() && !_control_status.flags.uwb) { continue; } // Phase 3 relaxes this
		any_fused |= fuseUwbRange(sample, _aid_src_uwb[sample.anchor_id]);
	}

	if (any_fused && !_control_status.flags.uwb) {
		ECL_INFO("starting UWB fusion");
		_control_status.flags.uwb = true;
	}

	if (_control_status.flags.uwb && isTimedOut(_time_last_uwb_fuse, _params.reset_timeout_max)) {
		stopUwbFusion();
	}
```
- [ ] **Step 2: Build → USER verify:** GPS on + UWB → `ekf2 status` / `estimator_status_flags` shows `cs_uwb=true`; turn `EKF2_UWB_CTRL 0` or block ranges → flag clears after timeout. **Commit** (after OK) `ekf2: register UWB as a horizontal-position aiding source (control_status.flags.uwb)`.

---

## PHASE 3 — UWB-only initialization (GPS-denied) + 3-mode validation

End state: with **no GPS**, UWB trilaterates an initial position, resets the EKF to it, sets `flags.uwb`, and holds position (no dead_reckoning). All three modes validated.

### Task 9: Trilateration init + reset

**Files:** `ekf.h`, `aid_sources/uwb/uwb_range_control.cpp`

- [ ] **Step 1: ekf.h** — declare:
```cpp
#if defined(CONFIG_EKF2_UWB)
	bool tryInitUwb();   // returns true once position has been initialized from ranges
	uwbSample _uwb_latest[4] {};   // most-recent range per anchor for init
	bool _uwb_initialized{false};
#endif
```
- [ ] **Step 2:** Implement linear-least-squares trilateration (anchor 0 as reference):
```cpp
bool Ekf::tryInitUwb()
{
	const int n = math::min((int)_params.uwb_n_anchors, 4);
	if (n < 4) { return false; }   // need 4 for a 3-D fix (3-D position observable)

	// require a recent range from every anchor
	for (int i = 0; i < n; i++) {
		if (!isRecent(_uwb_latest[i].time_us, 5e5) || _uwb_latest[i].range <= 0.f) {
			return false;
		}
	}

	// 2*(a_i - a_0)^T p = (|a_i|^2 - |a_0|^2) - (r_i^2 - r_0^2),  i = 1..3  -> A(3x3) p = b
	const Vector3f a0 = getUwbAnchorPos(0);
	const float r0 = _uwb_latest[0].range;
	matrix::SquareMatrix<float, 3> A;
	matrix::Vector3f b;
	for (int i = 1; i < 4; i++) {
		const Vector3f ai = getUwbAnchorPos(i);
		const Vector3f d = 2.f * (ai - a0);
		A.row(i - 1) = matrix::Matrix<float, 1, 3>(d.data());
		b(i - 1) = (ai.dot(ai) - a0.dot(a0)) - (sq(_uwb_latest[i].range) - sq(r0));
	}
	const auto Ainv = matrix::inv(A);
	if (!Ainv.isFinite()) { return false; }
	const Vector3f p = Ainv * b;   // NED position in the anchor/local frame

	// initialize EKF horizontal position (local frame; no global origin needed)
	const float var = sq(math::max(_params.uwb_noise, 0.1f)) * 4.f;
	resetHorizontalPositionTo(Vector2f(p(0), p(1)), Vector2f(var, var));
	// (height: keep current baro/height source; optionally reset z from p(2))

	_uwb_initialized = true;
	_control_status.flags.uwb = true;
	ECL_INFO("UWB position initialized by trilateration");
	return true;
}
```
- [ ] **Step 3:** In `UpdateUwbSample` (EKF2.cpp) keep latest per anchor → `_ekf` needs them: simplest is to stash in `setUwbData` (store into `_uwb_latest[anchor_id]` before/after push). Add that store in `setUwbData` or in `fuseUwbRange`'s caller.
- [ ] **Step 4:** In `controlUwbRangeFusion`, replace the Phase-1 gate with the init path:
```cpp
	if (!isHorizontalAidingActive() && !_control_status.flags.uwb) {
		// GPS-denied bring-up: trilaterate once, then fuse normally
		if (!tryInitUwb()) { return; }
	}
```
- [ ] **Step 5: Build** → clean.

### Task 10: 3-mode SITL validation

**Files:** none (runtime) + record results in this doc.

- [ ] **Mode UWB+GPS:** GPS on, `EKF2_UWB_CTRL 1`, anchors set → `cs_uwb=true` AND `cs_gnss_pos=true`; both fuse; `dead_reckoning=false`.
- [ ] **Mode GPS-only:** `EKF2_UWB_CTRL 0` → `cs_uwb=false`, GPS normal.
- [ ] **Mode UWB-only (GPS-denied):** start with GPS, let it converge, then `param set EKF2_GPS_CTRL 0`. Expect: `cs_uwb` stays true, `cs_gnss_pos` clears, `dead_reckoning=false`, `vehicle_local_position` tracks groundtruth (~0.1–0.3 m), `commander takeoff` + position hold works on UWB alone.
- [ ] **Cold UWB-only:** boot with `EKF2_GPS_CTRL 0` from the start → `tryInitUwb()` brings up position from ranges; verify `vehicle_local_position` initializes near truth.
- [ ] **Robustness:** block one anchor → 3 remaining keep fusing; UWB dropout > timeout → `stopUwbFusion`, falls back to dead_reckoning/fake-pos; UWB returns → re-inits/resumes.
- [ ] **Commit** results note.

---

## Implementation status (verified in SITL)

| Phase / Task | Commits | Result |
|---|---|---|
| Phase 1 — refactor B→A (uORB-free core) | `42f7047e97`, `6cc68ddb31`, `64d9297339` | parity with v1 (`fused=true`, innov ~2 cm) |
| Phase 2 Task 7 — add `control_status.flags.uwb` | `315f23a9e6` | `cs_uwb` published |
| Phase 2 Task 8 — set/clear + credit horizontal aiding | `567d3c47af` | GPS-on: `cs_uwb=True`; after GPS off: `cs_uwb=True`, `dead_reckoning=False`, pos holds |
| Phase 3 Task 9 — UWB-only 2-D trilateration cold-start | `c996565498` | boot with `EKF2_GPS_CTRL=0`: trilateration init, `cs_uwb=True`, `xy_valid=True`, no global origin |

**Three modes verified:**
- **UWB + GPS** — both `cs_gnss_pos` and `cs_uwb` true, both fuse.
- **GPS-only** (`EKF2_UWB_CTRL=0`) — GPS as before, `cs_uwb=False`.
- **UWB-only** — (a) GPS drops mid-flight → UWB holds (Task 8); (b) cold boot with no GPS → trilateration brings up position (Task 9). `ref_lat/lon=nan`, `xy_global=False` confirm GPS is never fused.

**Key fix found during validation:** the 4 sim anchors are **coplanar** (all `D=-2.0`), so a 3-D trilateration solve is singular. Task 9 uses **2-D** trilateration (solve N/E only, remove the vertical leg with the baro height) — correct since we only `resetHorizontalPositionTo()`. Needs ≥3 anchors.

**Remaining (optional) robustness sweep (Task 10):** block one anchor (3 remaining still fuse); UWB dropout > `reset_timeout_max` → `stopUwbFusion` then re-init on return; GPS re-enable transition. `z`/height accuracy is baro-driven (anchor frame assumed to share the height origin).

---

## Self-Review Notes (for the reviewer)

- **Highest-risk task = Task 9 (trilateration + reset).** The frame must be consistent: anchor params are NED relative to the EKF origin; in GPS-denied mode the EKF local frame *is* the anchor frame (origin at anchor-frame origin). Confirm `resetHorizontalPositionTo(Vector2f,Vector2f)` with no global origin behaves like the EV path. Height/z init left to baro initially.
- **`_time_last_hor_pos_fuse` is the linchpin** (Task 3 sets it on each fuse). Without it, Phase 2's dead-reckoning check won't credit UWB. Verify it's the field `updateHorizontalDeadReckoningstatus` reads.
- **Flag bit + msg sync:** `filter_control_status_u` (common.h) and `EstimatorStatusFlags.msg` are hand-kept in sync — add `uwb` to both at the same bit index; rebuild regenerates the msg.
- **ParamExt vs ModuleParams:** Phase 1 moves params into `_params` (ParamExt in EKF2), so the v1 `_uwb_range.updateParameters()` hook is removed; EKF2's existing `updateParams()` refreshes `_params` live.
- **matrix API for trilateration:** confirm `matrix::inv`, `SquareMatrix::row`, `.isFinite()` signatures in `src/lib/matrix`; adjust if names differ.
- **min anchors:** trilateration needs 4 for 3-D; with `EKF2_UWB_N_ANCH < 4`, UWB-only cold-start is disabled (fusion still works once GPS/other sets position). Document this.
- **Position accessors (Task 3):** verify `getLocalHorizontalPosition()` / vertical accessor names against the tree before coding; keep the v1-proven `_gpos` projection for the GPS-active path.
- **No `;` in pxh:** all SITL param-set steps are one-per-line.

---

*Plan v2. Builds on v1 (commits `7478550675`…`5ed64770ad`). Approach/theory: [`01_uwb_tightly_coupled_px4.md`](02_tightly_coupled_px4.md); v1 task plan: [`03_uwb_ekf2_implementation_plan.md`](03_v1_implementation_plan.md).*
