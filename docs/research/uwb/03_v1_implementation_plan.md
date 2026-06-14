# UWB Tightly-Coupled EKF2 Fusion — Implementation Plan

> Implementation plan for the UWB tightly-coupled EKF2 fusion. Tasks are ordered and independently buildable/testable; each ends in a commit. Checkboxes (`- [ ]`) track progress.

**Goal:** Fuse simulated UWB tag→anchor ranges into PX4 EKF2 as a tightly-coupled scalar position-aiding source, end-to-end from the `gtec_uwb_plugin` gz topic to a converging `vehicle_local_position` with GPS off — on a code path that the future DWM3000 hardware driver reuses unchanged.

**Architecture:** Follow PX4's self-contained aiding-source class template (`AuxGlobalPosition`). All UWB fusion lives in one class `UwbRange` (`aid_sources/uwb/uwb_range.{hpp,cpp}`) that self-subscribes to the **existing** `sensor_uwb` uORB topic, owns its `EKF2_UWB_*` params, buffers samples, fuses each range as a **scalar** measurement `z=‖p−a‖` (Jacobian `H_pos = (p−a)/‖p−a‖`) via the public `ekf.measurementUpdate()`, and publishes its own aid-source status. It plugs into the filter as a `friend class` with one member + one call. Upstream, `gtec_uwb_plugin` already publishes `px4::msgs::Ranging`; a `gz_bridge` callback converts it to `sensor_uwb`. Design + citations: [`docs/research/control_stack/01_uwb_tightly_coupled_px4.md`](02_tightly_coupled_px4.md).

**Tech Stack:** PX4 EKF2 (ESKF, 24-state), uORB, gz-transport (Harmonic) + `gz_bridge`, protobuf (`px4_gz_msgs`), CMake/Kconfig, SITL (`make px4_sitl gz_f450-uwb_uwb`).

**Key decision — reuse `sensor_uwb`, no new message.** `msg/SensorUwb.msg` already exists (output of the `uwb_sr150` driver). We reuse it so SITL and real UWB hardware share one fusion path:
- `distance` (m) → the range `z`.
- `mac` (uint16) → carries the **0-based anchor index** (sim bridge sets `mac = anchor_id`; a future driver sets the index or maps real MACs via params — no schema change).
- `timestamp` → measurement time; the missing `timestamp_sample` is covered by `timestamp − EKF2_UWB_DELAY`.
- range noise: `sensor_uwb` has no `range_variance`, so R comes from `EKF2_UWB_NOISE²` (a **fixed R**, which is exactly what Mueller et al. ICRA 2015 use — not a workaround).
- `offset_x/y/z` + `orientation` already carry the antenna lever-arm used by the ESKF Jacobian (Zhao et al. Sensors 2025) — available for a later refinement; v1 ignores it.
- `nlos` available for later adaptive-R; v1 (ideal LOS) uses constant R.

**DWM3000 open path:** `UwbRange` is hardware-agnostic — it only reads `sensor_uwb`. The future DWM3000 driver (TWR ranging, distance-per-anchor, no AoA) just publishes `sensor_uwb` the same way the bridge does; the fusion code is untouched. Build SITL first; the driver is a separate later effort.

**Testing reality:** EKF2 has no quick per-fusion unit harness; primary verification is **build + SITL `listener`/`ekf2 status` + groundtruth comparison**. Each task states its check.

**Scope (v1):** ideal LOS, 4 fixed anchors at known NED positions (params), UWB as **position aid only** (heading still from mag/GPS-yaw). UWB **requires an existing position origin** (GPS or another aid sets it); pure UWB-only cold-start trilateration is out of scope.

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `msg/SensorUwb.msg` | **Reuse (no change)** | existing UWB topic: `distance`, `mac`, `timestamp`, `nlos`, `offset_*` |
| `msg/EstimatorAidSource1d.msg` | Modify | register `estimator_aid_src_uwb` topic |
| `src/modules/simulation/gz_bridge/GZBridge.hpp` | Modify | proto include, callback decl, publication member |
| `src/modules/simulation/gz_bridge/GZBridge.cpp` | Modify | subscribe `/gtec/toa/ranging`, `rangeCallback` → publish `sensor_uwb` |
| `src/modules/ekf2/Kconfig` | Modify | `menuconfig EKF2_UWB` |
| `src/modules/ekf2/params_uwb.yaml` | Create | `EKF2_UWB_*` param definitions |
| `src/modules/ekf2/EKF/aid_sources/uwb/uwb_range.hpp` | Create | `UwbRange` class (sub, buffer, params, aid src, pub) |
| `src/modules/ekf2/EKF/aid_sources/uwb/uwb_range.cpp` | Create | `update()`, `anchorPos()`, `fuse()` |
| `src/modules/ekf2/EKF/ekf.h` | Modify | `#include`, `friend class UwbRange;`, `UwbRange _uwb_range{};` member |
| `src/modules/ekf2/EKF/control.cpp` | Modify | `_uwb_range.update(*this, imu_delayed);` |
| `src/modules/ekf2/EKF/CMakeLists.txt` | Modify | compile `uwb/uwb_range.cpp` into `ecl_EKF` |
| `src/modules/ekf2/CMakeLists.txt` | Modify | compile `EKF/aid_sources/uwb/uwb_range.cpp` into the `ekf2` **module** (MODULE_NAME) — required, else `undefined reference` |
| `boards/px4/sitl/default.px4board` | Modify | `CONFIG_EKF2_UWB=y` |

**Untouched:** `common.h`, `estimator_interface.{h,cpp}`, `EKF2.{hpp,cpp}` — class is self-contained. **No new uORB message.**

---

## PHASE 1 — UWB ranges into uORB (sim)

End state: `listener sensor_uwb` shows 4 anchors (via `mac` 0..3) streaming in SITL.

### Task 1: gz_bridge `rangeCallback` → publish `sensor_uwb`

**Files:**
- Modify: `src/modules/simulation/gz_bridge/GZBridge.hpp` (top includes; proto include ~line 83; method decl ~line 120; publication member ~line 143)
- Modify: `src/modules/simulation/gz_bridge/GZBridge.cpp` (subscribe in `init()` near optical-flow block ~line 147; callback near ~line 229)

- [ ] **Step 1: GZBridge.hpp — includes, decl, member**

Add the uORB topic + proto includes (mirror the optical-flow ones):
```cpp
#include <uORB/topics/sensor_uwb.h>
```
```cpp
#include <ranging.pb.h>
```
Private method declaration (near `void opticalFlowCallback(...)`):
```cpp
	void rangeCallback(const px4::msgs::Ranging &msg);
```
Publication member (near `_optical_flow_pub`):
```cpp
	uORB::PublicationMulti<sensor_uwb_s> _uwb_pub{ORB_ID(sensor_uwb)};
```

- [ ] **Step 2: GZBridge.cpp — subscribe in `init()`**

After the optical-flow `Subscribe` block:
```cpp
	// UWB ranging (gtec_uwb_plugin publishes on a fixed, non-namespaced topic)
	if (!_node.Subscribe("/gtec/toa/ranging", &GZBridge::rangeCallback, this)) {
		PX4_ERR("failed to subscribe to /gtec/toa/ranging");
		return PX4_ERROR;
	}
```

- [ ] **Step 3: GZBridge.cpp — implement the callback**

```cpp
void GZBridge::rangeCallback(const px4::msgs::Ranging &msg)
{
	sensor_uwb_s report{};

	report.timestamp = hrt_absolute_time();
	report.mac = (uint16_t)msg.anchor_id();       // anchor index carried in mac (0..3 in sim)
	report.distance = (float)msg.range() * 1e-3f; // proto range is millimetres -> metres
	report.nlos = 0;                              // ideal LOS sim
	// aoa_*, fom, offset_*, orientation left 0 (unused by v1 fusion)

	_uwb_pub.publish(report);
}
```

- [ ] **Step 4: Build SITL**

Run: `make px4_sitl gz_f450-uwb_uwb 2>&1 | tail -15`
Expected: clean (gz_bridge already links `px4_gz_msgs`; `ranging.pb.h` resolves; `sensor_uwb` already exists).

- [ ] **Step 5: Verify ranges reach uORB**

Run sim (Terminal 1), press Play. In `pxh>`: `listener sensor_uwb 8`
Expected: `mac` 0..3, `distance` ≈ 3.9–4.1 m, ~25 Hz.

- [ ] **Step 6: Commit**
```bash
git add src/modules/simulation/gz_bridge/GZBridge.hpp src/modules/simulation/gz_bridge/GZBridge.cpp
git commit -m "gz_bridge: bridge px4::msgs::Ranging to sensor_uwb (mac=anchor_id)"
```

---

## PHASE 2 — `UwbRange` aiding class

End state: `CONFIG_EKF2_UWB=y` builds; the class subscribes to `sensor_uwb`, buffers, and fuses per-anchor ranges; `estimator_aid_src_uwb` shows sane innovations.

### Task 2: Kconfig flag, params, aid-source topic

**Files:**
- Modify: `src/modules/ekf2/Kconfig` (after `menuconfig EKF2_GNSS`)
- Modify: `boards/px4/sitl/default.px4board`
- Create: `src/modules/ekf2/params_uwb.yaml`
- Modify: `msg/EstimatorAidSource1d.msg`

- [ ] **Step 1: Kconfig**
```kconfig
menuconfig EKF2_UWB
depends on MODULES_EKF2
	bool "UWB range fusion support"
	default n
	---help---
		EKF2 tightly-coupled UWB range fusion (position aiding).
		Reuses the sensor_uwb topic. Requires anchors with known NED
		positions (EKF2_UWB_A*_*).
```

- [ ] **Step 2: Enable in SITL board**

Run: `grep -rln "CONFIG_EKF2_GNSS=y" boards/px4/sitl/`
Add `CONFIG_EKF2_UWB=y` to that file (likely `boards/px4/sitl/default.px4board`).

- [ ] **Step 3: Register the aid-source topic**

In `msg/EstimatorAidSource1d.msg`, add a `# TOPICS` line:
```
# TOPICS estimator_aid_src_uwb
```

- [ ] **Step 4: Create `params_uwb.yaml`**

`src/modules/ekf2/params_uwb.yaml`:
```yaml
module_name: ekf2
parameters:
- group: EKF2
  definitions:
    EKF2_UWB_CTRL:
      description: { short: UWB range fusion control }
      type: int32
      default: 0
      min: 0
      max: 1
      reboot_required: true
    EKF2_UWB_DELAY:
      description: { short: UWB range measurement delay relative to IMU }
      type: float
      default: 50.0
      min: 0.0
      max: 300.0
      unit: ms
      reboot_required: true
      decimal: 1
    EKF2_UWB_NOISE:
      description: { short: UWB range measurement noise (1-sigma) }
      type: float
      default: 0.05
      min: 0.01
      max: 1.0
      unit: m
      decimal: 2
    EKF2_UWB_GATE:
      description: { short: UWB range innovation gate size (1-sigma) }
      type: float
      default: 5.0
      min: 1.0
      max: 10.0
    EKF2_UWB_N_ANCH:
      description: { short: Number of configured UWB anchors }
      type: int32
      default: 4
      min: 1
      max: 4
      reboot_required: true
    EKF2_UWB_A0_N: { description: { short: UWB anchor 0 North (NED m) }, type: float, default: 0.0, unit: m, decimal: 2 }
    EKF2_UWB_A0_E: { description: { short: UWB anchor 0 East (NED m) },  type: float, default: 0.0, unit: m, decimal: 2 }
    EKF2_UWB_A0_D: { description: { short: UWB anchor 0 Down (NED m) },  type: float, default: 0.0, unit: m, decimal: 2 }
    EKF2_UWB_A1_N: { description: { short: UWB anchor 1 North (NED m) }, type: float, default: 0.0, unit: m, decimal: 2 }
    EKF2_UWB_A1_E: { description: { short: UWB anchor 1 East (NED m) },  type: float, default: 0.0, unit: m, decimal: 2 }
    EKF2_UWB_A1_D: { description: { short: UWB anchor 1 Down (NED m) },  type: float, default: 0.0, unit: m, decimal: 2 }
    EKF2_UWB_A2_N: { description: { short: UWB anchor 2 North (NED m) }, type: float, default: 0.0, unit: m, decimal: 2 }
    EKF2_UWB_A2_E: { description: { short: UWB anchor 2 East (NED m) },  type: float, default: 0.0, unit: m, decimal: 2 }
    EKF2_UWB_A2_D: { description: { short: UWB anchor 2 Down (NED m) },  type: float, default: 0.0, unit: m, decimal: 2 }
    EKF2_UWB_A3_N: { description: { short: UWB anchor 3 North (NED m) }, type: float, default: 0.0, unit: m, decimal: 2 }
    EKF2_UWB_A3_E: { description: { short: UWB anchor 3 East (NED m) },  type: float, default: 0.0, unit: m, decimal: 2 }
    EKF2_UWB_A3_D: { description: { short: UWB anchor 3 Down (NED m) },  type: float, default: 0.0, unit: m, decimal: 2 }
```

- [ ] **Step 5: Confirm params yaml discovery**

Run: `grep -rn "params_.*\.yaml\|module.yaml" src/modules/ekf2/CMakeLists.txt`
If the list is explicit, add `params_uwb.yaml` the way `params_gnss.yaml` is referenced; if globbed, no change.

- [ ] **Step 6: Build**

Run: `make px4_sitl gz_f450-uwb_uwb 2>&1 | tail -8`
Expected: builds; `estimator_aid_src_uwb` topic generates.
Runtime check (in `pxh>`): `param set EKF2_UWB_NOISE 0.07` succeeds (proves the param is registered), then `param show -c EKF2_UWB*` lists it. NOTE: plain `param show EKF2_UWB*` shows **nothing** here — `param show <pattern>` only lists *used* params (`do_show` passes `only_used=true`), and EKF2_UWB_* stay "unused" until Task 5's `UwbRange` references them via `DEFINE_PARAMETERS`. Use `param set`/`param show -c`/`param show -a` to verify before Task 5.

- [ ] **Step 7: Commit**
```bash
git add src/modules/ekf2/Kconfig boards/px4/sitl/default.px4board src/modules/ekf2/params_uwb.yaml msg/EstimatorAidSource1d.msg
git commit -m "ekf2: add EKF2_UWB Kconfig + params + estimator_aid_src_uwb topic"
```

---

### Task 3: Scaffold `UwbRange` class + wire into `Ekf`

End state: class compiles and is called every control cycle; `update()` is a no-op. Proves friend/member/CMake wiring.

**Files:**
- Create: `src/modules/ekf2/EKF/aid_sources/uwb/uwb_range.hpp`
- Create: `src/modules/ekf2/EKF/aid_sources/uwb/uwb_range.cpp`
- Modify: `src/modules/ekf2/EKF/ekf.h` (include, friend, member)
- Modify: `src/modules/ekf2/EKF/control.cpp` (call)
- Modify: `src/modules/ekf2/EKF/CMakeLists.txt`

- [ ] **Step 1: Create `uwb_range.hpp` (mirrors AuxGlobalPosition)**

```cpp
#ifndef EKF_AID_SOURCES_UWB_RANGE_HPP
#define EKF_AID_SOURCES_UWB_RANGE_HPP

#include <px4_platform_common/module_params.h>
#include <matrix/math.hpp>

#include "common.h"
#include "RingBuffer.h"

#if defined(MODULE_NAME)
# include <uORB/Subscription.hpp>
# include <uORB/PublicationMulti.hpp>
# include <uORB/topics/sensor_uwb.h>
# include <uORB/topics/estimator_aid_source1d.h>
#endif // MODULE_NAME

class Ekf;

class UwbRange : public ModuleParams
{
public:
	UwbRange() : ModuleParams(nullptr)
	{
#if defined(MODULE_NAME)
		_aid_src_uwb_pub.advertise();
#endif // MODULE_NAME
	}

	~UwbRange() = default;

	void update(Ekf &ekf, const estimator::imuSample &imu_delayed);

	void updateParameters() { updateParams(); }

private:
	static constexpr uint8_t kMaxAnchors = 4;

	struct UwbSample {
		uint64_t time_us{};    ///< measurement time (uSec), already delay-adjusted
		uint8_t  anchor_id{};
		float    range{};      ///< measured distance (m)
		float    range_var{};  ///< measurement variance (m^2); 0 = use EKF2_UWB_NOISE^2
	};

	matrix::Vector3f anchorPos(uint8_t anchor_id) const;
	bool fuse(Ekf &ekf, const UwbSample &sample, estimator_aid_source1d_s &aid_src);

	RingBuffer<UwbSample> _uwb_buffer{20};
	uint64_t _time_last_buffer_push{0};
	estimator_aid_source1d_s _aid_src_uwb[kMaxAnchors] {};

#if defined(MODULE_NAME)
	uORB::Subscription _sensor_uwb_sub{ORB_ID(sensor_uwb)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _aid_src_uwb_pub{ORB_ID(estimator_aid_src_uwb)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::EKF2_UWB_CTRL>)    _param_ekf2_uwb_ctrl,
		(ParamFloat<px4::params::EKF2_UWB_DELAY>) _param_ekf2_uwb_delay,
		(ParamFloat<px4::params::EKF2_UWB_NOISE>) _param_ekf2_uwb_noise,
		(ParamFloat<px4::params::EKF2_UWB_GATE>)  _param_ekf2_uwb_gate,
		(ParamInt<px4::params::EKF2_UWB_N_ANCH>)  _param_ekf2_uwb_n_anch,
		(ParamFloat<px4::params::EKF2_UWB_A0_N>) _param_a0_n,
		(ParamFloat<px4::params::EKF2_UWB_A0_E>) _param_a0_e,
		(ParamFloat<px4::params::EKF2_UWB_A0_D>) _param_a0_d,
		(ParamFloat<px4::params::EKF2_UWB_A1_N>) _param_a1_n,
		(ParamFloat<px4::params::EKF2_UWB_A1_E>) _param_a1_e,
		(ParamFloat<px4::params::EKF2_UWB_A1_D>) _param_a1_d,
		(ParamFloat<px4::params::EKF2_UWB_A2_N>) _param_a2_n,
		(ParamFloat<px4::params::EKF2_UWB_A2_E>) _param_a2_e,
		(ParamFloat<px4::params::EKF2_UWB_A2_D>) _param_a2_d,
		(ParamFloat<px4::params::EKF2_UWB_A3_N>) _param_a3_n,
		(ParamFloat<px4::params::EKF2_UWB_A3_E>) _param_a3_e,
		(ParamFloat<px4::params::EKF2_UWB_A3_D>) _param_a3_d
	)
#endif // MODULE_NAME
};

#endif // EKF_AID_SOURCES_UWB_RANGE_HPP
```

- [ ] **Step 2: Create `uwb_range.cpp` (no-op body for now)**

```cpp
#include "ekf.h"
#include "aid_sources/uwb/uwb_range.hpp"

void UwbRange::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{
	(void)ekf;
	(void)imu_delayed;
	// implemented in Task 4/5
}
```

- [ ] **Step 3: ekf.h — include, friend, member**

Near the aid-source includes (with `aux_global_position.hpp`):
```cpp
#if defined(CONFIG_EKF2_UWB)
# include "aid_sources/uwb/uwb_range.hpp"
#endif // CONFIG_EKF2_UWB
```
Next to `friend class AuxGlobalPosition;` (~line 420):
```cpp
#if defined(CONFIG_EKF2_UWB)
	friend class UwbRange;
#endif // CONFIG_EKF2_UWB
```
Next to `AuxGlobalPosition _aux_global_position {};` (~line 1144):
```cpp
#if defined(CONFIG_EKF2_UWB)
	UwbRange _uwb_range {};
#endif // CONFIG_EKF2_UWB
```

- [ ] **Step 4: control.cpp — call in `controlFusionModes`**

Next to `_aux_global_position.update(*this, imu_delayed);` (~line 119):
```cpp
#if defined(CONFIG_EKF2_UWB) && defined(MODULE_NAME)
	_uwb_range.update(*this, imu_delayed);
#endif // CONFIG_EKF2_UWB && MODULE_NAME
```

- [ ] **Step 5: conditional compile — BOTH CMakeLists**

EKF aid sources are compiled twice: once into the standalone `ecl_EKF` lib (no `MODULE_NAME`) and once into the `ekf2` module (with `MODULE_NAME`, where `update()` is actually defined). The source must be listed in **both** or the module build hits `undefined reference to UwbRange::update`.

`src/modules/ekf2/EKF/CMakeLists.txt` (ecl_EKF), next to the aux_global block:
```cmake
if(CONFIG_EKF2_UWB)
	list(APPEND EKF_SRCS aid_sources/uwb/uwb_range.cpp)
endif()
```
`src/modules/ekf2/CMakeLists.txt` (module — merge with the params block from Task 2; note the `EKF/` path prefix):
```cmake
if(CONFIG_EKF2_UWB)
	list(APPEND EKF_SRCS EKF/aid_sources/uwb/uwb_range.cpp)
	list(APPEND EKF_MODULE_PARAMS params_uwb.yaml)
endif()
```

- [ ] **Step 6: Build**

Run: `make px4_sitl_default 2>&1 | tail -8`  (compile-only; avoids launching the sim)
Expected: clean link, no `undefined reference`.

- [ ] **Step 7: Commit**
```bash
git add src/modules/ekf2/EKF/aid_sources/uwb/ src/modules/ekf2/EKF/ekf.h src/modules/ekf2/EKF/control.cpp src/modules/ekf2/EKF/CMakeLists.txt
git commit -m "ekf2: scaffold UwbRange aiding class wired via friend + control loop"
```

---

### Task 4: `update()` — subscribe `sensor_uwb`, buffer at delayed horizon

End state: samples flow from `sensor_uwb` into the internal ring buffer and pop at the delayed horizon. No fusion yet.

**Files:**
- Modify: `src/modules/ekf2/EKF/aid_sources/uwb/uwb_range.cpp`

- [ ] **Step 1: Replace `update()` body**

```cpp
void UwbRange::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{
#if defined(MODULE_NAME)
	// 1) ingest sensor_uwb into the delay buffer
	sensor_uwb_s msg;

	while (_sensor_uwb_sub.update(&msg)) {
		// anchor index carried in mac; sim sets mac = anchor_id (0..3)
		if (msg.mac >= (uint16_t)_param_ekf2_uwb_n_anch.get()) {
			continue;
		}

		if (!PX4_ISFINITE(msg.distance) || msg.distance <= 0.f) {
			continue;
		}

		UwbSample sample{};
		// sensor_uwb has no timestamp_sample -> use timestamp minus a fixed delay
		sample.time_us   = msg.timestamp - (uint64_t)(_param_ekf2_uwb_delay.get() * 1000.f);
		sample.anchor_id = (uint8_t)msg.mac;
		sample.range     = msg.distance;
		sample.range_var = 0.f; // sensor_uwb carries no variance -> fuse() uses EKF2_UWB_NOISE^2

		_uwb_buffer.push(sample);
		_time_last_buffer_push = imu_delayed.time_us;
	}

	if (_param_ekf2_uwb_ctrl.get() == 0) {
		return;
	}

	// 2) fuse everything aligned with the current delayed horizon
	UwbSample sample;

	while (_uwb_buffer.pop_first_older_than(imu_delayed.time_us, &sample)) {
		if (!ekf.control_status_flags().tilt_align) {
			continue;
		}

		// v1: needs a position origin from another source (GPS/EV) before fusing
		if (!ekf._local_origin_lat_lon.isInitialized()) {
			continue;
		}

		fuse(ekf, sample, _aid_src_uwb[sample.anchor_id]); // implemented in Task 5
	}
#else
	(void)ekf;
	(void)imu_delayed;
#endif // MODULE_NAME
}
```

- [ ] **Step 2: Add a temporary stub `fuse()` so it links (replaced in Task 5)**

Under `#if defined(MODULE_NAME)`:
```cpp
bool UwbRange::fuse(Ekf &ekf, const UwbSample &sample, estimator_aid_source1d_s &aid_src)
{
	(void)ekf; (void)aid_src;
	PX4_DEBUG("UWB pop anchor %d range %.2f", sample.anchor_id, (double)sample.range);
	return false;
}
```

- [ ] **Step 3: Build + verify buffering**

Run: `make px4_sitl gz_f450-uwb_uwb 2>&1 | tail -10`
In SITL: `param set EKF2_UWB_CTRL 1` (temporarily raise the print to `PX4_INFO` if needed).
Expected: "UWB pop anchor 0..3" once GPS sets the origin.

- [ ] **Step 4: Commit**
```bash
git add src/modules/ekf2/EKF/aid_sources/uwb/uwb_range.cpp
git commit -m "ekf2: UwbRange ingests sensor_uwb into delay buffer"
```

---

### Task 5: Fusion + publish aid source

End state: per-anchor scalar range fusion runs; `estimator_aid_src_uwb` shows innovations.

**Files:**
- Modify: `src/modules/ekf2/EKF/aid_sources/uwb/uwb_range.cpp`

- [ ] **Step 1: Implement `anchorPos()` (under `#if defined(MODULE_NAME)`)**
```cpp
matrix::Vector3f UwbRange::anchorPos(uint8_t anchor_id) const
{
	switch (anchor_id) {
	case 0: return matrix::Vector3f(_param_a0_n.get(), _param_a0_e.get(), _param_a0_d.get());
	case 1: return matrix::Vector3f(_param_a1_n.get(), _param_a1_e.get(), _param_a1_d.get());
	case 2: return matrix::Vector3f(_param_a2_n.get(), _param_a2_e.get(), _param_a2_d.get());
	case 3: return matrix::Vector3f(_param_a3_n.get(), _param_a3_e.get(), _param_a3_d.get());
	default: return matrix::Vector3f();
	}
}
```

- [ ] **Step 2: Replace the stub `fuse()` with the real one**
```cpp
bool UwbRange::fuse(Ekf &ekf, const UwbSample &sample, estimator_aid_source1d_s &aid_src)
{
	// vehicle local NED position at the delayed (fusion) horizon
	float pn;
	float pe;
	ekf._local_origin_lat_lon.project(ekf._gpos.latitude_deg(), ekf._gpos.longitude_deg(), pn, pe);
	const matrix::Vector3f pos_ned(pn, pe, -ekf._gpos.altitude());

	const matrix::Vector3f anchor = anchorPos(sample.anchor_id);
	const matrix::Vector3f diff = pos_ned - anchor;
	const float predicted = diff.norm();

	if (predicted < 0.1f) {
		return false; // singular Jacobian (vehicle on top of anchor)
	}

	// measurement Jacobian: only the position block is non-zero (unit LOS direction)
	const matrix::Vector3f e = diff / predicted;
	VectorState H;
	H.setZero();
	H(State::pos.idx + 0) = e(0);
	H(State::pos.idx + 1) = e(1);
	H(State::pos.idx + 2) = e(2);

	// measurement noise: fixed baseline (sensor_uwb carries no variance)
	float R = math::max(sq(_param_ekf2_uwb_noise.get()), 1e-4f);

	if (sample.range_var > 0.f && sample.range_var < 100.f) {
		R = sample.range_var;
	}

	const float innovation = sample.range - predicted;        // z - h(x)
	const float innovation_var = H.dot(ekf.P * H) + R;        // H P H^T + R

	ekf.updateAidSourceStatus(aid_src,
				  sample.time_us,
				  sample.range,
				  R,
				  innovation,
				  innovation_var,
				  math::max(_param_ekf2_uwb_gate.get(), 1.f));

	if (aid_src.innovation_rejected) {
		return false;
	}

	VectorState K = ekf.P * H / aid_src.innovation_variance;
	ekf.measurementUpdate(K, H, R, aid_src.innovation);

	aid_src.fused = true;
	aid_src.time_last_fuse = ekf._time_delayed_us;
	return true;
}
```

- [ ] **Step 3: Publish the aid source after fusing (in `update()` pop loop)**
```cpp
		fuse(ekf, sample, _aid_src_uwb[sample.anchor_id]);
		_aid_src_uwb_pub.publish(_aid_src_uwb[sample.anchor_id]);
```

- [ ] **Step 4: Build**

Run: `make px4_sitl gz_f450-uwb_uwb 2>&1 | tail -15`
Expected: clean. (`VectorState`, `State`, `sq`, `math::max` via `ekf.h`; `ekf.P`, `ekf._gpos`, `ekf._local_origin_lat_lon`, `ekf.updateAidSourceStatus`, `ekf._time_delayed_us` via `friend class UwbRange`; `ekf.measurementUpdate` is public.)

- [ ] **Step 5: Verify fusion in SITL**

Run sim; set anchor params (Task 6 Step 1), `param set EKF2_UWB_CTRL 1`, reboot, Play.
Run: `listener estimator_aid_src_uwb 8`
Expected: `observation` ≈ range, `innovation` small (~cm), `test_ratio` < 1, `fused=true`.

- [ ] **Step 6: Commit**
```bash
git add src/modules/ekf2/EKF/aid_sources/uwb/uwb_range.cpp
git commit -m "ekf2: tightly-coupled UWB scalar range fusion + aid-source publish"
```

---

## PHASE 3 — Validation

### Task 6: SITL convergence test

**Files:** none (runtime). Anchor NED from `worlds/uwb.sdf` (ENU→NED: N=y, E=x, D=−z).

- [ ] **Step 1: Configure anchors + enable**

In `pxh>` (verify the ENU→NED mapping against the world file first). NOTE: the pxh shell
does **not** support `;` command chaining — enter one `param set` per line, or only the first
on each line is applied (a real trap: it silently leaves E/D at 0 → huge innovation, rejected).
Params apply live (updateParameters is wired), so no reboot is needed.
```
param set EKF2_UWB_CTRL 1
param set EKF2_UWB_A0_N -2.5
param set EKF2_UWB_A0_E -2.5
param set EKF2_UWB_A0_D -2.0
param set EKF2_UWB_A1_N -2.5
param set EKF2_UWB_A1_E 2.5
param set EKF2_UWB_A1_D -2.0
param set EKF2_UWB_A2_N 2.5
param set EKF2_UWB_A2_E 2.5
param set EKF2_UWB_A2_D -2.0
param set EKF2_UWB_A3_N 2.5
param set EKF2_UWB_A3_E -2.5
param set EKF2_UWB_A3_D -2.0
```
Verify with `param show -c EKF2_UWB*` that all 12 N/E/D are set (none stray at 0). Press Play.

- [ ] **Step 2: Baseline (GPS on) — fusion consistency**

Run: `listener estimator_aid_src_uwb 12`
Expected: all 4 anchors `fused=true`, `test_ratio` < 1, innovations zero-mean (~cm).

- [ ] **Step 3: GPS-denied — UWB holds position**

`pxh>`: `param set EKF2_GPS_CTRL 0`, `commander takeoff`. Compare:
Run: `listener vehicle_local_position` vs `listener vehicle_local_position_groundtruth`
Expected: position stays bounded (~0.1–0.3 m) on UWB alone.

- [ ] **Step 4: Robustness — drop one anchor**

Make one anchor wrong/NLOS: remaining 3 keep `fused=true`, position degrades gracefully.

- [ ] **Step 5: Record results + commit**

Append innovation stats + GPS-off RMSE to [`01_uwb_tightly_coupled_px4.md`](02_tightly_coupled_px4.md) §9.
```bash
git add docs/research/control_stack/01_uwb_tightly_coupled_px4.md
git commit -m "docs(research): record UWB EKF2 SITL validation results"
```

---

## Self-Review Notes (for the reviewer)

- **Spec coverage:** bridge → `sensor_uwb` → class (subscribe/buffer/fuse/publish) → validate. Matches approach doc §6/§8.
- **`sensor_uwb` reuse — field mapping:** `distance`→range, `mac`→anchor index, `timestamp`(−`EKF2_UWB_DELAY`)→sample time, `nlos`→(future adaptive R). R from `EKF2_UWB_NOISE²` (fixed R, matching Mueller ICRA 2015). No schema change; `offset_*`/`orientation` (lever-arm) reserved for a later refinement.
- **Soft spots to scrutinize before coding:**
  1. **`mac` as anchor index:** sim bridge sets `mac = anchor_id`. For real hardware later, either the driver writes the index into `mac`, or add a `mac→index` param map (still no msg change). Confirm `mac` is `uint16` and indices 0..3 fit (they do).
  2. **`_gpos` projection (Task 5):** plan projects `ekf._gpos` (delayed-horizon global pos) via `ekf._local_origin_lat_lon`. Confirm `_gpos` is delayed-horizon and that projecting it matches how other delayed aiding gets position.
  3. **`control_status_flags()` accessor (Task 4):** confirm exact method name used by `AuxGlobalPosition`; gate uses `.tilt_align`.
  4. **params yaml discovery (Task 2 Step 5):** glob vs explicit list. Verify with `param set EKF2_UWB_NOISE 0.07` + `param show -c EKF2_UWB*` (NOT plain `param show EKF2_UWB*` — it hides unused params until Task 5 references them).
  5. **`MODULE_NAME` guards:** all uORB + param access under `#if defined(MODULE_NAME)`, matching `AuxGlobalPosition`, so the EKF lib still builds for unit tests (where `update()` is a no-op; `fuse`/`anchorPos` defined only under `MODULE_NAME` and only called there → no undefined-reference).
  6. **`RingBuffer<UwbSample>{20}`:** 4 anchors × 25 Hz vs EKF delayed horizon; mirrors `AuxGlobalPosition`'s TODO. Bump if pops miss.
  7. **ENU→NED anchor mapping (Task 6):** gz world is ENU, PX4 is NED — verify the conversion before trusting numbers.
- **DWM3000 follow-up (out of scope here):** a separate driver publishing `sensor_uwb` (TWR distance per anchor; set `mac`=index or add the param map). The fusion class is unchanged.
- **No EKF gtest in v1:** validation is SITL-based by design; a synthetic-range gtest fixture is a reasonable follow-up.

---

*Plan saved 2026-06-13 (rev 3 — reuse `sensor_uwb`, DWM3000-ready, AuxGlobalPosition self-contained-class template). Approach + citations: [`docs/research/control_stack/01_uwb_tightly_coupled_px4.md`](02_tightly_coupled_px4.md).*
