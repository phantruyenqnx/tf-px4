# UWB Tightly-Coupled Fusion — PX4 EKF2 Implementation Guide

> **What this file covers:** HOW to implement — exact file paths, line-level diff guidance, and
> verified code templates drawn from the actual source tree.
>
> **Read first:** `01_uwb_ekf2_spec.md` for WHAT and WHY, `02f_ekf2_unified.md` for EKF2
> internals (ESKF theory + source navigation).

---

## Table of Contents

- [0. Pre-flight: What Already Exists](#0-pre-flight-what-already-exists)
- [0.5 Phase 0 — DWM3000 UART Driver](#05-phase-0--dwm3000-uart-driver)
- [1. Phase 1 — uORB Topics](#1-phase-1--uorb-topics)
- [2. Phase 2 — Sample Struct + Ring Buffer](#2-phase-2--sample-struct--ring-buffer)
- [3. Phase 3 — Parameters](#3-phase-3--parameters)
- [4. Phase 4 — EstimatorInterface: setUwbData()](#4-phase-4--estimatorinterface-setuwbdata)
- [5. Phase 5 — Ekf Class: Headers](#5-phase-5--ekf-class-headers)
- [6. Phase 6 — Core Fusion: uwb_range_fusion.cpp](#6-phase-6--core-fusion-uwb_range_fusioncpp)
- [7. Phase 7 — Wire into controlFusionModes()](#7-phase-7--wire-into-controlfusionmodes)
- [8. Phase 8 — EKF2 Wrapper: Subscription + Feed](#8-phase-8--ekf2-wrapper-subscription--feed)
- [9. Phase 9 — Build System](#9-phase-9--build-system)
- [10. Critical Implementation Notes](#10-critical-implementation-notes)

---

## 0. Pre-flight: What Already Exists

Before writing any code, verify these existing pieces — they shape the approach.

### 0.1 Existing UWB uORB topic

`msg/SensorUwb.msg` already exists with fields:
```
uint64  timestamp
uint32  sessionid
uint16  mac          # Initiator (drone) MAC
uint16  mac_dest     # Responder (anchor) MAC  ← anchor identity
uint16  status
uint8   nlos         # 0=LOS, 1=NLOS
float32 distance     # measured range (m)
float32 aoa_*        # angle of arrival fields (not used for ranging)
uint8   orientation
float32 offset_x/y/z # antenna offset in body frame
```

**Key issue:** no `range_variance`, no `quality` score, no `timestamp_sample` (separate from `timestamp`).
The existing message is designed for the UWB driver's raw output. We have two options:

- **Option A (Recommended):** Add a new message `msg/SensorUwbRange.msg` tailored for EKF2 fusion.
- **Option B:** Reuse `sensor_uwb` and derive variance from a parameter baseline.

This guide uses **Option A** because Option B forces us to handle the missing `timestamp_sample`
in an ugly way, and the variance field is essential for adaptive R.

### 0.2 Aid source message that exists

`msg/EstimatorAidSource1d.msg` — use this for per-anchor logging. It has:
```
timestamp, timestamp_sample, observation, observation_variance,
innovation, innovation_variance, test_ratio, innovation_rejected, fused, time_last_fuse
```
PX4 convention is to publish one `estimator_aid_source1d` per anchor per update.

### 0.3 Functions we will reuse unchanged

| Function | File | Role |
|---|---|---|
| `measurementUpdate(K, H, R, innov)` | `ekf_helper.cpp:1089` | Joseph-form P update + state injection |
| `fuse(K, innov)` | `ekf_helper.cpp:736` | Apply K·innov to all 24 states |
| `updateAidSourceStatus(...)` | `ekf_helper.cpp:1155` | Compute innovation gate test ratio |
| `RingBuffer<T>` | `EKF/RingBuffer.h` | Template ring buffer, used by all sensors |

**Critical:** UWB has a nonlinear measurement model (`‖p - a‖`), so H is NOT a standard basis
vector. We **cannot** use `fuseDirectStateMeasurement()` (which hardcodes `H(state_index) = 1`).
We must call `measurementUpdate(K, H, R, innov)` with an explicitly computed H.

---

## 0.5 Phase 0 — DWM3000 UART Driver

The EKF2 changes below only define how EKF consumes UWB range samples. With a self-developed
DWM3000 tag connected to Pixhawk over UART, there is one missing layer before Phase 1:

```
DWM3000 tag firmware
    → UART frame on Pixhawk TELEM/GPS/UART port
    → PX4 driver parses frame
    → publish one uORB range sample per anchor
    → EKF2 subscribes and feeds setUwbData()
```

### 0.5.1 What current source already has

PX4 already contains `src/drivers/uwb/uwb_sr150/`, but that driver is specific to the NXP
SR150/MK UWB Shield frame format:

- `uwb_sr150.cpp` opens a serial port, configures termios, reads a fixed `distance_msg_t`,
  and publishes `sensor_uwb`.
- `uwb_sr150.h` defines only one `UWB_range_meas_t measurements` inside each `distance_msg_t`.
- `msg/SensorUwb.msg` carries `mac_dest`, `status`, `nlos`, and `distance`, but no
  `timestamp_sample`, no variance, and no `anchor_id`.

So for DWM3000 + 4 anchors, do **not** try to feed EKF directly from the existing SR150 parser.
Use it as a PX4 driver style reference, then add a DWM3000-specific serial driver that publishes
the EKF-oriented topic from Phase 1.

### 0.5.2 Recommended driver location

Create a new driver next to the existing UWB driver:

```
src/drivers/uwb/dwm3000_uart/
    CMakeLists.txt
    Kconfig
    module.yaml
    dwm3000_uart.cpp
    dwm3000_uart.hpp
```

Then register it:

```cmake
# src/drivers/uwb/CMakeLists.txt
add_subdirectory(uwb_sr150)
add_subdirectory(dwm3000_uart)
```

```kconfig
# src/drivers/uwb/Kconfig
menuconfig COMMON_UWB
    bool "common UWB Drivers"
    default n
    select DRIVERS_UWB_UWB_SR150
    select DRIVERS_UWB_DWM3000_UART
```

and in `src/drivers/uwb/dwm3000_uart/Kconfig`:

```kconfig
menuconfig DRIVERS_UWB_DWM3000_UART
    bool "dwm3000_uart"
    default n
    ---help---
        Enable support for a DWM3000 UART tag publishing ranges to anchors.
```

### 0.5.3 Driver skeleton follows PX4 serial driver patterns

Use the same base pattern as `TFMINI` and `uwb_sr150`:

- inherit `ModuleBase`, `ModuleParams`, `px4::ScheduledWorkItem`;
- schedule on `px4::serial_port_to_wq(port)`;
- open with `::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK)`;
- configure 8N1 termios (`CLOCAL | CREAD`, `CS8`, no parity, one stop bit, no flow control);
- read all available bytes each cycle;
- keep parser state across cycles because UART frames can be split;
- publish **up to 4 uORB messages per ranging round**, one per anchor.

The publication should target the new Phase 1 topic:

```cpp
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_uwb_range.h>

uORB::Publication<sensor_uwb_range_s> _sensor_uwb_range_pub{ORB_ID(sensor_uwb_range)};
```

For each valid anchor in a decoded DWM3000 frame:

```cpp
sensor_uwb_range_s msg{};
msg.timestamp = hrt_absolute_time();          // publish/receive time
msg.timestamp_sample = sample_time_us;        // measurement time, see 0.5.5
msg.anchor_id = anchor_id;                    // 0..3 after MAC/address mapping
msg.range = range_mm * 0.001f;
msg.range_variance = range_variance_m2;       // 0 if unknown; EKF uses EKF2_UWB_NOISE^2
msg.nlos_flag = nlos_flag;
msg.quality = quality;                        // 0..100, or 0 if unknown
_sensor_uwb_range_pub.publish(msg);
```

Do not publish the four distances as one custom array if EKF2 expects `sensor_uwb_range`.
The ring buffer and sequential fusion in later phases are simpler and more PX4-like when each
anchor range is one sample.

### 0.5.4 UART frame contract between tag firmware and PX4

The exact binary protocol is yours, but it should include enough metadata for EKF:

```cpp
struct Dwm3000Range {
    uint16_t anchor_mac;      // or compact anchor slot id
    uint32_t range_mm;
    uint16_t variance_mm2;    // optional; 0 means unknown
    uint8_t  nlos;            // 0 LOS, 1 possible NLOS, 2 confirmed NLOS
    uint8_t  quality;         // 0..100
} __attribute__((packed));

struct Dwm3000Frame {
    uint16_t magic;           // e.g. 0xD300
    uint8_t  version;
    uint8_t  n_ranges;        // normally 4, allow fewer if an anchor is missing
    uint32_t seq;
    uint64_t measurement_time_us; // preferred if tag is time-synced to PX4
    Dwm3000Range ranges[4];
    uint16_t crc16;
} __attribute__((packed));
```

Minimum validation before publishing:

- frame header/version/length/CRC valid;
- `n_ranges <= 4`;
- anchor MAC maps to a known `anchor_id`;
- range finite and inside expected operating limits, for example `0.1 m .. 200 m`;
- duplicate anchors in one frame are either rejected or last-sample-wins;
- stale sequence numbers are dropped.

Anchor mapping can be done in the driver (`anchor_mac → anchor_id`) or in the tag firmware
(send `anchor_id` directly). If the DWM3000 firmware sends MACs, add driver params such as
`UWB_A0_MAC ... UWB_A3_MAC`, or keep a small static mapping while prototyping.

### 0.5.5 Timestamp rule

For EKF, `timestamp_sample` is more important than `timestamp`.

Best case:

```
measurement_time_us = PX4-synchronized hardware measurement time
timestamp_sample    = measurement_time_us
timestamp           = hrt_absolute_time() when PX4 publishes
```

If the DWM3000 tag is not time-synced to PX4, set:

```cpp
const hrt_abstime now = hrt_absolute_time();
msg.timestamp = now;
msg.timestamp_sample = now - estimated_uart_and_processing_latency_us;
```

Then put the remaining systematic delay into `EKF2_UWB_DELAY`. Avoid setting
`timestamp_sample` to zero or to a tag-local clock that PX4 cannot compare with `hrt_absolute_time()`.

### 0.5.6 `module.yaml` serial port integration

Mirror the existing `uwb_sr150/module.yaml` so the driver can be assigned to a Pixhawk UART
through a parameter:

```yaml
module_name: DWM3000 UART UWB range driver
serial_config:
  - command: dwm3000_uart start -d ${SERIAL_DEV} -b p:${BAUD_PARAM}
    port_config_param:
      name: DWM3000_CFG
      group: UWB
```

The start command should also accept manual use:

```sh
dwm3000_uart start -d /dev/ttyS2 -b 115200
dwm3000_uart status
dwm3000_uart stop
```

### 0.5.7 Alternative: bridge from existing `sensor_uwb`

If you want the smallest short-term change, adapt the existing `uwb_sr150` style and publish
`sensor_uwb` first, then add a small bridge module:

```
sensor_uwb → map mac_dest to anchor_id → fill timestamp_sample/range_variance → sensor_uwb_range
```

This is useful for quick logging, but the cleaner implementation is for the DWM3000 driver to
publish `sensor_uwb_range` directly. That keeps EKF2 independent of a vendor-specific raw UWB
message format.

---

## 1. Phase 1 — uORB Topics

### 1.1 New sensor input topic

**File to create:** `msg/SensorUwbRange.msg`

```
# UWB range measurement for EKF2 tightly-coupled fusion.
# timestamp_sample is the hardware measurement time (not publish time).

uint64 timestamp           # message publish time (us)
uint64 timestamp_sample    # when the chip actually measured (us) — critical for time-alignment

uint8  anchor_id           # 0-indexed slot, mapped from mac_dest via parameter
float32 range              # measured distance (m)
float32 range_variance     # measurement variance (m^2); 0 → use EKF2_UWB_NOISE^2
uint8  nlos_flag           # 0=LOS, 1=possible NLOS, 2=confirmed NLOS
uint8  quality             # 0-100; 0 → unknown
```

**File to create:** `msg/EstimatorAidSourceUwbRange.msg`  
Use the existing `EstimatorAidSource1d` structure — no new msg needed. Just add
an `anchor_id` for convenience:

```
uint64 timestamp
uint64 timestamp_sample
uint8  anchor_id
float32 observation              # range (m)
float32 observation_variance     # R after adaptive adjustment
float32 innovation               # z - h(x)
float32 innovation_variance      # S = H·P·H^T + R
float32 test_ratio               # innov^2 / (gate^2 * S)
bool   innovation_rejected
bool   fused
uint64 time_last_fuse
```

Register both in `msg/CMakeLists.txt` under the `set(msg_files ...)` list.

---

## 2. Phase 2 — Sample Struct + Ring Buffer

### 2.1 Add `uwbRangeSample` to common.h

**File:** `src/modules/ekf2/EKF/common.h`

Add after the `auxVelSample` block (around line 255), inside the `#if defined(CONFIG_EKF2_UWB)` guard:

```cpp
#if defined(CONFIG_EKF2_UWB)
struct uwbRangeSample {
    uint64_t time_us{};         ///< timestamp of measurement (uSec) — from timestamp_sample
    uint8_t  anchor_id{};       ///< 0-indexed anchor slot
    float    range{};           ///< measured distance (m)
    float    range_var{};       ///< measurement variance (m^2); 0 = use param baseline
    uint8_t  nlos_flag{};       ///< 0=LOS, 1=possible, 2=confirmed
    uint8_t  quality{};         ///< 0-100
};
#endif // CONFIG_EKF2_UWB
```

### 2.2 Add UWB parameters to `struct parameters`

**File:** `src/modules/ekf2/EKF/common.h`, inside `struct parameters { ... }`:

```cpp
#if defined(CONFIG_EKF2_UWB)
    int32_t  uwb_ctrl{0};                 ///< UWB fusion enable (0=off, 1=on)
    float    uwb_delay_ms{50.0f};         ///< sensor delay relative to IMU (ms)
    float    uwb_range_noise{0.05f};      ///< baseline range sigma (m)
    float    uwb_innov_gate{5.0f};        ///< innovation gate (sigma multiples)
    int32_t  uwb_n_anchors{4};            ///< number of configured anchors (1-8)

    // Anchor NED positions [m]. Origin = EKF home position.
    // Max 8 anchors; entries above uwb_n_anchors are ignored.
    float    uwb_anchor_n[8]{};           ///< anchor North position (m)
    float    uwb_anchor_e[8]{};           ///< anchor East position (m)
    float    uwb_anchor_d[8]{};           ///< anchor Down position (m)
#endif // CONFIG_EKF2_UWB
```

### 2.3 Add ring buffer in EstimatorInterface

**File:** `src/modules/ekf2/EKF/estimator_interface.h`, inside the `protected:` section.

Find the block of `RingBuffer<*> *_xxx_buffer {nullptr};` declarations and add:

```cpp
#if defined(CONFIG_EKF2_UWB)
    RingBuffer<uwbRangeSample> *_uwb_buffer{nullptr};
    uint64_t _time_last_uwb_buffer_push{0};
#endif // CONFIG_EKF2_UWB
```

---

## 3. Phase 3 — Parameters

### 3.1 Parameter YAML

**File to create:** `src/modules/ekf2/params_uwb.yaml`

Follow the pattern of `params_gnss.yaml`. Key entries:

```yaml
parameters:
- group: EKF2
  definitions:
    EKF2_UWB_CTRL:
      description:
        short: UWB range fusion control
      type: int32
      default: 0
      min: 0
      max: 1
      reboot_required: true

    EKF2_UWB_DELAY:
      description:
        short: UWB range measurement delay relative to IMU (ms)
      type: float
      default: 50.0
      min: 0.0
      max: 300.0
      unit: ms

    EKF2_UWB_NOISE:
      description:
        short: UWB range measurement baseline sigma (m)
      type: float
      default: 0.05
      min: 0.01
      max: 1.0
      unit: m

    EKF2_UWB_GATE:
      description:
        short: UWB innovation gate (sigma)
      type: float
      default: 5.0
      min: 1.0
      max: 10.0

    EKF2_UWB_N_ANCH:
      description:
        short: Number of configured UWB anchors (1-8)
      type: int32
      default: 4
      min: 1
      max: 8
      reboot_required: true

    # Repeat for each anchor index 0-7:
    EKF2_UWB_A0_N:
      description:
        short: UWB anchor 0 North position (m) in EKF NED frame
      type: float
      default: 0.0
    # ... A0_E, A0_D, A1_N, A1_E, A1_D, ...
```

Include this yaml in `module.yaml` by adding `params_uwb.yaml` to the `parameters` list.

### 3.2 Parameter binding in EKF2.cpp constructor

**File:** `src/modules/ekf2/EKF2.cpp`, in the `EKF2::EKF2()` initializer list, inside
`#if defined(CONFIG_EKF2_UWB)`:

```cpp
#if defined(CONFIG_EKF2_UWB)
_param_ekf2_uwb_ctrl(_params->uwb_ctrl),
_param_ekf2_uwb_delay(_params->uwb_delay_ms),
_param_ekf2_uwb_noise(_params->uwb_range_noise),
_param_ekf2_uwb_gate(_params->uwb_innov_gate),
_param_ekf2_uwb_n_anch(_params->uwb_n_anchors),
// anchor positions wired similarly
#endif
```

Declare these as `DEFINE_PARAMETERS(...)` entries in `EKF2.hpp` (see Section 8).

---

## 4. Phase 4 — EstimatorInterface: setUwbData()

### 4.1 Declaration

**File:** `src/modules/ekf2/EKF/estimator_interface.h`, inside `class EstimatorInterface`:

```cpp
#if defined(CONFIG_EKF2_UWB)
    void setUwbData(const uwbRangeSample &uwb_sample);
#endif // CONFIG_EKF2_UWB
```

### 4.2 Implementation

**File:** `src/modules/ekf2/EKF/estimator_interface.cpp`

Pattern is identical to `setGpsData()`. Add at the end of the file:

```cpp
#if defined(CONFIG_EKF2_UWB)
void EstimatorInterface::setUwbData(const uwbRangeSample &uwb_sample)
{
    if (!_initialised) { return; }

    // Allocate buffer on first call (lazy init, same pattern as GPS)
    if (_uwb_buffer == nullptr) {
        _uwb_buffer = new RingBuffer<uwbRangeSample>();

        if (_uwb_buffer == nullptr || !_uwb_buffer->allocate(_obs_buffer_length)) {
            printBufferAllocationFailed("UWB");
            delete _uwb_buffer;
            _uwb_buffer = nullptr;
            return;
        }
    }

    const int64_t time_us = (int64_t)uwb_sample.time_us
                            - (int64_t)(_params.uwb_delay_ms * 1000.f);

    if (time_us < 0) { return; }

    uwbRangeSample sample = uwb_sample;
    sample.time_us = (uint64_t)time_us;

    _uwb_buffer->push(sample);
    _time_last_uwb_buffer_push = uwb_sample.time_us;
}
#endif // CONFIG_EKF2_UWB
```

**What this does:**
1. Lazy-allocates the ring buffer on first call (same as GPS/baro pattern).
2. Applies the configured delay to time-stamp align UWB to the IMU delayed horizon.
3. Pushes to buffer — `controlUwbFusion()` will pop it at the right time.

---

## 5. Phase 5 — Ekf Class: Headers

### 5.1 Add methods and members to `ekf.h`

**File:** `src/modules/ekf2/EKF/ekf.h`

Inside `class Ekf final : public EstimatorInterface`, add inside
`#if defined(CONFIG_EKF2_UWB)`:

```cpp
#if defined(CONFIG_EKF2_UWB)
    // Aid source status (one per anchor, max 8)
    estimator_aid_source1d_s _aid_src_uwb_range[8]{};

    void controlUwbFusion(const imuSample &imu_delayed);
    bool fuseUwbRange(const uwbRangeSample &sample);
    bool tryInitFromUwb();

    // Anchor positions in NED [m]
    Vector3f getAnchorPos(uint8_t anchor_id) const;

    // State tracking
    uint64_t _time_last_uwb_fuse{0};
    bool     _uwb_initialized{false};
    uint8_t  _uwb_consecutive_rejects{0};

private:
    float computeAdaptiveR(const uwbRangeSample &sample) const;
#endif // CONFIG_EKF2_UWB
```

**Accessor for publish** (in the public section):
```cpp
#if defined(CONFIG_EKF2_UWB)
    const auto &aid_src_uwb_range() const { return _aid_src_uwb_range; }
#endif // CONFIG_EKF2_UWB
```

---

## 6. Phase 6 — Core Fusion: uwb_range_fusion.cpp

**File to create:** `src/modules/ekf2/EKF/aid_sources/uwb/uwb_range_fusion.cpp`

This is the main implementation file. Below is the complete template with all key logic.

```cpp
/**
 * @file uwb_range_fusion.cpp
 * UWB range tightly-coupled fusion for EKF2.
 * Measurement model: h(x) = ‖p - a_i‖
 * Jacobian:          H[6:9] = (p - a_i)^T / ‖p - a_i‖  (unit vector, position block only)
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

// ─── Helper: read anchor NED position from parameters ────────────────────────

Vector3f Ekf::getAnchorPos(uint8_t anchor_id) const
{
    if (anchor_id >= 8) { return Vector3f{0.f, 0.f, 0.f}; }
    return Vector3f{
        _params.uwb_anchor_n[anchor_id],
        _params.uwb_anchor_e[anchor_id],
        _params.uwb_anchor_d[anchor_id]
    };
}

// ─── Adaptive R ──────────────────────────────────────────────────────────────

float Ekf::computeAdaptiveR(const uwbRangeSample &sample) const
{
    // Baseline variance from parameter (sigma → variance)
    float R = math::max(sq(_params.uwb_range_noise), 1e-4f);

    // Use chip-provided variance if available and sane
    if (sample.range_var > 0.f && sample.range_var < 100.f) {
        R = sample.range_var;
    }

    // Quality degradation (chip reports 0-100; <80 inflates R)
    if (sample.quality > 0 && sample.quality < 80) {
        R *= (80.f / math::max((float)sample.quality, 10.f));
    }

    // NLOS penalty
    if (sample.nlos_flag == 1) { R *= 4.f; }
    if (sample.nlos_flag == 2) { R *= 100.f; } // near-reject

    // Distance-dependent noise growth (empirical: beyond 20m)
    if (sample.range > 20.f) {
        R *= (sample.range / 20.f);
    }

    return R;
}

// ─── Single-range fusion ──────────────────────────────────────────────────────

bool Ekf::fuseUwbRange(const uwbRangeSample &sample)
{
    // Sanity checks
    if (sample.anchor_id >= (uint8_t)_params.uwb_n_anchors) { return false; }
    if (!PX4_ISFINITE(sample.range) || sample.range < 0.1f || sample.range > 200.f) {
        return false;
    }

    const Vector3f anchor_ned = getAnchorPos(sample.anchor_id);

    // ── Step 1: predicted range using current NED position ────────────────────
    //
    //  EKF internal position _state.pos is always zeroed after each update.
    //  True NED position lives in _gpos (LatLonAlt).
    //  Use the EKF local position helper to get NED relative to origin.
    //
    const Vector3f pos_ned(
        (float)(_gpos.latitude_deg()  - _local_origin_lat_lon.getProjectionReferenceLat()),
        (float)(_gpos.longitude_deg() - _local_origin_lat_lon.getProjectionReferenceLon()),
        -_gpos.altitude()
    );
    // Better: use the projection directly
    // Vector2f pos_ne; _local_origin_lat_lon.project(_gpos.latitude_deg(), _gpos.longitude_deg(), pos_ne(0), pos_ne(1));
    // Vector3f pos_ned(pos_ne(0), pos_ne(1), -_gpos.altitude());

    const Vector3f diff = pos_ned - anchor_ned;
    const float predicted_range = diff.norm();

    if (predicted_range < 0.1f) {
        // Drone is on top of anchor — Jacobian is singular
        return false;
    }

    // ── Step 2: measurement Jacobian H (1×24, stored as VectorState column) ──
    //
    //  H[0:3]   = 0  (attitude — range doesn't directly depend on attitude)
    //  H[3:6]   = 0  (velocity)
    //  H[6:9]   = (p - a)^T / ‖p - a‖  = unit vector from anchor to drone
    //  H[9:24]  = 0  (biases, mag, wind, terrain)
    //
    VectorState H;
    H.setZero();
    const Vector3f unit_vec = diff / predicted_range;
    H(State::pos.idx + 0) = unit_vec(0); // North component
    H(State::pos.idx + 1) = unit_vec(1); // East component
    H(State::pos.idx + 2) = unit_vec(2); // Down component

    // ── Step 3: innovation and innovation variance ─────────────────────────────
    const float R = computeAdaptiveR(sample);
    const float innovation = sample.range - predicted_range;  // z - h(x)

    // S = H * P * H^T + R  (scalar — PX4 computes this as P*H first)
    const VectorState PH = P * H;  // 24×1
    float innovation_var = H.dot(PH) + R;

    if (innovation_var < R) {
        // Numerical issue — reset to safe value
        innovation_var = R;
    }

    // ── Step 4: Update aid source status + innovation gate ────────────────────
    auto &aid_src = _aid_src_uwb_range[sample.anchor_id];

    updateAidSourceStatus(aid_src,
                          sample.time_us,          // timestamp_sample
                          sample.range,            // observation
                          R,                       // observation_variance
                          innovation,              // innovation
                          innovation_var,          // innovation_variance
                          math::max(_params.uwb_innov_gate, 1.f)); // gate

    if (aid_src.innovation_rejected) {
        _uwb_consecutive_rejects++;
        return false;
    }

    _uwb_consecutive_rejects = 0;

    // ── Step 5: Kalman gain K = P * H^T / S ───────────────────────────────────
    VectorState K = PH / innovation_var;

    // ── Step 6: Joseph-form covariance update + state injection ───────────────
    //  measurementUpdate() calls fuse() internally → updates all 24 states.
    //  Position correction: _gpos += K[6:9] * innovation; _state.pos.zero()
    measurementUpdate(K, H, R, innovation);

    aid_src.fused = true;
    aid_src.time_last_fuse = _time_delayed_us;
    _time_last_uwb_fuse = _time_delayed_us;

    return true;
}

// ─── Control loop dispatcher ──────────────────────────────────────────────────

void Ekf::controlUwbFusion(const imuSample &imu_delayed)
{
    if (!_uwb_buffer || (_params.uwb_ctrl == 0)) { return; }

    // ── Check if UWB has timed out ──────────────────────────────────────────
    if (_uwb_initialized && isTimedOut(_time_last_uwb_fuse, (uint64_t)2e6)) {
        ECL_WARN("UWB timeout — stopping fusion");
        _control_status.flags.uwb = false;
        // Let filter predict-only; P will grow, self-converges when UWB returns
    }

    // ── Pop all samples that align with the delayed IMU horizon ─────────────
    uwbRangeSample sample{};

    while (_uwb_buffer->pop_first_older_than(imu_delayed.time_us, &sample)) {

        if (!_control_status.flags.tilt_align) { continue; }  // not ready

        // Try to initialize position from UWB if not yet done by another source
        if (!_uwb_initialized && !isHorizontalAidingActive()) {
            // Collect a few samples then trilaterate (see tryInitFromUwb())
            // For now: skip fusion until we have a position estimate
            continue;
        }

        const bool fused = fuseUwbRange(sample);

        if (fused && !_control_status.flags.uwb) {
            ECL_INFO("UWB: starting range fusion");
            _control_status.flags.uwb = true;
        }
    }

    // ── Divergence guard: hard reset if too many consecutive rejects ─────────
    if (_uwb_consecutive_rejects >= 10) {
        ECL_WARN("UWB: %d consecutive rejections — hard reset", _uwb_consecutive_rejects);
        _uwb_consecutive_rejects = 0;
        _uwb_initialized = false;
        _control_status.flags.uwb = false;
    }
}

// ─── Cold-start trilateration (simplified) ───────────────────────────────────

bool Ekf::tryInitFromUwb()
{
    // Collect one range per anchor (use most recent from buffer)
    // For N=4 anchors, subtract equation i=0 from i=1,2,3 to linearize:
    //   2*(a0 - ai)^T * p = (‖a0‖^2 - ‖ai‖^2) - (r0^2 - ri^2)
    // Stack into A*p = b, solve p = A^{-1}*b

    // This is only called once at init; implementation detail below.
    // Skip if fewer than 3 anchors have recent measurements.

    // NOTE: This function is optional if GPS provides initial position.
    // In that case, uwb_initialized is set to true immediately.
    return false; // placeholder — implement when no GPS available
}
```

### 6.1 Key implementation detail: position access

The current nominal position is **not** in `_state.pos` (which is zeroed after every fuse).
True NED position lives in `_gpos` (a `LatLonAlt` object). To convert to local NED:

```cpp
// Get NED [m] from _gpos relative to EKF origin
float pos_n, pos_e;
_local_origin_lat_lon.project(
    _gpos.latitude_deg(),
    _gpos.longitude_deg(),
    pos_n, pos_e
);
const float pos_d = -_gpos.altitude();
const Vector3f pos_ned(pos_n, pos_e, pos_d);
```

Check `MapProjection::project()` in `lib/lat_lon_alt/` for the exact API.

---

## 7. Phase 7 — Wire into controlFusionModes()

**File:** `src/modules/ekf2/EKF/control.cpp`

Inside `Ekf::controlFusionModes()`, after the `controlGpsFusion` block and before
`controlHeightFusion`:

```cpp
#if defined(CONFIG_EKF2_UWB)
    controlUwbFusion(imu_delayed);
#endif // CONFIG_EKF2_UWB
```

**Exact insertion point** (between lines 116 and 134 in the current source):
```cpp
#if defined(CONFIG_EKF2_GNSS)
    controlGpsFusion(imu_delayed);                // ← line 116
#endif // CONFIG_EKF2_GNSS

#if defined(CONFIG_EKF2_UWB)
    controlUwbFusion(imu_delayed);                // ← INSERT HERE
#endif // CONFIG_EKF2_UWB

    controlHeightFusion(imu_delayed);             // ← line 134
```

**Why this position:** UWB is a position aid like GPS. It must run after tilt alignment check
(already done at the top of `controlFusionModes`) and before height fusion (which may depend
on position state being stable).

---

## 8. Phase 8 — EKF2 Wrapper: Subscription + Feed

### 8.1 Add subscription in EKF2.hpp

**File:** `src/modules/ekf2/EKF2.hpp`, inside `class EKF2`:

```cpp
#if defined(CONFIG_EKF2_UWB)
    // Subscription
    uORB::Subscription _sensor_uwb_range_sub{ORB_ID(sensor_uwb_range)};

    // Publication (one per anchor, up to 8)
    uORB::PublicationMulti<estimator_aid_source_uwb_range_s> _estimator_aid_src_uwb_range_pub[8]
        {{ORB_ID(estimator_aid_source_uwb_range)}, /* x8 */};

    // Parameters
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::EKF2_UWB_CTRL>)   _param_ekf2_uwb_ctrl,
        (ParamFloat<px4::params::EKF2_UWB_DELAY>) _param_ekf2_uwb_delay,
        (ParamFloat<px4::params::EKF2_UWB_NOISE>) _param_ekf2_uwb_noise,
        (ParamFloat<px4::params::EKF2_UWB_GATE>)  _param_ekf2_uwb_gate,
        (ParamInt<px4::params::EKF2_UWB_N_ANCH>)  _param_ekf2_uwb_n_anch
        // Add anchor position params similarly
    )

    void UpdateUwbSample(const hrt_abstime &timestamp);
    void PublishUwbAidSourceStatus(const hrt_abstime &timestamp);
#endif // CONFIG_EKF2_UWB
```

### 8.2 UpdateUwbSample in EKF2.cpp

**File:** `src/modules/ekf2/EKF2.cpp`

```cpp
#if defined(CONFIG_EKF2_UWB)
void EKF2::UpdateUwbSample(const hrt_abstime &timestamp)
{
    sensor_uwb_range_s sensor_uwb_range;

    while (_sensor_uwb_range_sub.update(&sensor_uwb_range)) {
        // Map uORB message to EKF internal sample
        uwbRangeSample uwb_sample{};

        // Use timestamp_sample (hardware time) for time-alignment,
        // NOT the publish timestamp
        uwb_sample.time_us   = sensor_uwb_range.timestamp_sample;
        uwb_sample.anchor_id = sensor_uwb_range.anchor_id;
        uwb_sample.range     = sensor_uwb_range.range;
        uwb_sample.range_var = sensor_uwb_range.range_variance;
        uwb_sample.nlos_flag = sensor_uwb_range.nlos_flag;
        uwb_sample.quality   = sensor_uwb_range.quality;

        // Sanity: anchor_id must be within configured range
        if (uwb_sample.anchor_id >= (uint8_t)_param_ekf2_uwb_n_anch.get()) {
            continue;
        }

        // Sanity: range must be positive and finite
        if (!PX4_ISFINITE(uwb_sample.range) || uwb_sample.range <= 0.f) {
            continue;
        }

        _ekf.setUwbData(uwb_sample);
    }
}
#endif // CONFIG_EKF2_UWB
```

### 8.3 Call site in EKF2::Run()

**File:** `src/modules/ekf2/EKF2.cpp`, inside `EKF2::Run()`.

Find where other sensors are updated (e.g., `UpdateGpsSample(now)`) and add:

```cpp
#if defined(CONFIG_EKF2_UWB)
    UpdateUwbSample(now);
#endif // CONFIG_EKF2_UWB
```

### 8.4 Publish aid source status

**File:** `src/modules/ekf2/EKF2.cpp`, inside `PublishAidSourceStatus()`:

```cpp
#if defined(CONFIG_EKF2_UWB)
    // Publish per-anchor aid source status for logging
    const int n_anchors = _param_ekf2_uwb_n_anch.get();
    for (int i = 0; i < n_anchors && i < 8; i++) {
        const auto &aid_src = _ekf.aid_src_uwb_range()[i];
        if (aid_src.timestamp_sample > 0) {
            estimator_aid_source_uwb_range_s msg{};
            // fill msg from aid_src...
            msg.timestamp = hrt_absolute_time();
            msg.anchor_id = i;
            msg.observation = aid_src.observation;
            msg.innovation  = aid_src.innovation;
            // ...
            _estimator_aid_src_uwb_range_pub[i].publish(msg);
        }
    }
#endif // CONFIG_EKF2_UWB
```

---

## 9. Phase 9 — Build System

### 9.1 Kconfig

**File:** `src/modules/ekf2/Kconfig`

Add after the `EKF2_AUXVEL` block:

```kconfig
menuconfig EKF2_UWB
depends on MODULES_EKF2
    bool "UWB range fusion support"
    default n
    ---help---
        EKF2 UWB tightly-coupled range fusion support.
        Requires at least 1 anchor with known NED position.
```

### 9.2 EKF/CMakeLists.txt

**File:** `src/modules/ekf2/EKF/CMakeLists.txt`

After the `if(CONFIG_EKF2_AUXVEL)` block, add:

```cmake
if(CONFIG_EKF2_UWB)
    list(APPEND EKF_SRCS aid_sources/uwb/uwb_range_fusion.cpp)
endif()
```

### 9.3 Top-level CMakeLists

**File:** `src/modules/ekf2/CMakeLists.txt`

The top-level CMakeLists usually just builds the EKF library and the EKF2 module.
No structural changes needed if `uwb_range_fusion.cpp` is inside the `ecl_EKF` library
target (handled by the EKF/CMakeLists.txt above).

If you add a new msg file, also register it in `msg/CMakeLists.txt`.

### 9.4 DWM3000 UART driver build hooks

The EKF build hooks above do not build the UART reader. For the driver from Phase 0, add:

```cmake
# src/drivers/uwb/CMakeLists.txt
add_subdirectory(dwm3000_uart)
```

```cmake
# src/drivers/uwb/dwm3000_uart/CMakeLists.txt
px4_add_module(
    MODULE drivers__dwm3000_uart
    MAIN dwm3000_uart
    SRCS
        dwm3000_uart.cpp
        dwm3000_uart.hpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        px4_work_queue
)
```

Also add `src/drivers/uwb/dwm3000_uart/Kconfig` and include/select it from
`src/drivers/uwb/Kconfig`, following the existing `uwb_sr150` layout. On board configs,
enable both:

```text
CONFIG_DRIVERS_UWB_DWM3000_UART=y
CONFIG_EKF2_UWB=y
```

Runtime data flow to verify:

```sh
dwm3000_uart start -d /dev/ttyS2 -b 115200
listener sensor_uwb_range
ekf2 status
```

---

## 10. Critical Implementation Notes

### 10.1 The `_state.pos.zero()` trap

After every `fuse()` call, PX4 zeroes `_state.pos` (x, y) — see `ekf_helper.cpp:753`.
The true position lives in `_gpos`. **Never read `_state.pos` to compute the predicted
range** — it will always be zero after the first GPS/UWB fusion.

Use `_local_origin_lat_lon.project(...)` on `_gpos` instead. Verify:
```
grep -n "_gpos" src/modules/ekf2/EKF/ekf_helper.cpp
```

### 10.2 `measurementUpdate` vs `fuseDirectStateMeasurement`

| Function | H shape | When to use |
|---|---|---|
| `fuseDirectStateMeasurement(innov, innov_var, R, state_idx)` | Basis vector (1 entry) | GPS pos X, GPS pos Y, baro height |
| `measurementUpdate(K, H, R, innov)` | Arbitrary VectorState | **UWB range** — H has 3 nonzero entries |

UWB **must** use `measurementUpdate` because `H = (p - a) / ‖p - a‖` has contributions
at positions [6], [7], [8] simultaneously with different weights.

### 10.3 Sequential fusion order matters

When multiple anchors have data in the same IMU step, fuse them sequentially (one at a time).
The `pop_first_older_than` loop in `controlUwbFusion` already handles this — each call to
`fuseUwbRange` modifies `_state` and `P` before the next anchor is processed.

This is statistically optimal when anchor measurement noises are uncorrelated (they are,
because TWR is independent per-pair).

### 10.4 Innovation sign convention

PX4 convention (verified in `ekf_helper.cpp:736`):

```
fuse(K, innovation)   →  _state -= K * innovation
```

So `innovation = measurement - predicted` (`z - h(x)`) is the **correct** sign.

If range is larger than predicted → positive innovation → position pushed toward anchor.
This is correct: if the anchor reports a longer distance, the drone is farther from the
anchor than expected → pull position away (but the Jacobian unit vector points FROM
anchor TO drone, so `K * innov` moves position in that direction). Double-check with
a simple unit test (Section 8.1 of the spec).

### 10.5 `filter_control_status_u` — adding the `uwb` flag

The `filter_control_status_u` union is defined in a generated uORB header. To add a
`uwb` flag:

1. Find `msg/EstimatorStatusFlags.msg` (or similar) — this drives the bitfield.
2. Add `bool uwb` to the appropriate section.
3. Regenerate with `make uorb_headers`.
4. If modifying the shared message is too invasive, use a module-local boolean
   `_uwb_active` in the `Ekf` class instead — same functional effect.

### 10.6 Time-alignment via `timestamp_sample`

The EKF runs on the **delayed IMU horizon** (~50-80 ms in the past). All sensors must
be time-aligned to this horizon via their ring buffers.

The critical path:
```
UWB chip measures at time T_hw
Driver receives at time T_recv  (T_recv ≈ T_hw + UART latency)
Driver publishes with:
    timestamp_sample = T_hw      ← set by driver
    timestamp        = T_recv    ← set by hrt_absolute_time()

EKF2::UpdateUwbSample():
    uwb_sample.time_us = sensor_uwb_range.timestamp_sample  ← must use THIS

EstimatorInterface::setUwbData():
    adjusted_time = time_us - uwb_delay_ms * 1000           ← apply param delay

RingBuffer::push(sample)
    → stored at adjusted_time

controlUwbFusion():
    pop_first_older_than(imu_delayed.time_us)               ← pops when aligned
```

If the UWB driver does not set `timestamp_sample` correctly (uses `hrt_absolute_time()`
instead of hardware timestamp), add the UART latency to `EKF2_UWB_DELAY` as compensation.

### 10.7 `isHorizontalAidingActive()` check

`isHorizontalAidingActive()` returns true if GPS vel, GPS pos, EV vel, EV pos,
or optical flow is active. If UWB is the **only** position source (indoor, no GPS),
this check will return false until UWB itself becomes active — creating a deadlock.

Solution: separate the initialization path from the fusion path in `controlUwbFusion()`:
```
If no horizontal aiding:
    → attempt trilateration to get initial position
    → once position initialized, set _uwb_initialized = true
    → on next iteration, start fusing normally
Else (GPS/EV already active):
    → set _uwb_initialized = true immediately
    → start UWB fusion immediately (it enhances existing position estimate)
```

---

## Appendix A: File Change Summary

| File | Action | Description |
|---|---|---|
| `msg/SensorUwbRange.msg` | **CREATE** | Input sensor topic |
| `msg/EstimatorAidSourceUwbRange.msg` | **CREATE** | Debug/log output per anchor |
| `msg/CMakeLists.txt` | **EDIT** | Register new msg files |
| `EKF/common.h` | **EDIT** | Add `uwbRangeSample` struct + UWB params |
| `EKF/estimator_interface.h` | **EDIT** | Add `setUwbData()` + ring buffer member |
| `EKF/estimator_interface.cpp` | **EDIT** | Implement `setUwbData()` |
| `EKF/ekf.h` | **EDIT** | Add methods + members under `CONFIG_EKF2_UWB` |
| `EKF/aid_sources/uwb/uwb_range_fusion.cpp` | **CREATE** | All fusion logic |
| `EKF/control.cpp` | **EDIT** | Add `controlUwbFusion()` call |
| `EKF/CMakeLists.txt` | **EDIT** | Conditionally compile new .cpp |
| `EKF2.hpp` | **EDIT** | Add subscription, DEFINE_PARAMETERS |
| `EKF2.cpp` | **EDIT** | Add `UpdateUwbSample()`, publish aid source |
| `Kconfig` | **EDIT** | Add `EKF2_UWB` menuconfig entry |
| `params_uwb.yaml` | **CREATE** | All UWB parameters |
| `module.yaml` | **EDIT** | Include `params_uwb.yaml` |

## Appendix B: Verification Checklist

```
□ sensor_uwb_range topic publishes at expected rate (listener sensor_uwb_range)
□ uwb_buffer allocates without error (check PX4 console logs)
□ pop_first_older_than returns true (innovation_rejected never, test_ratio > 0)
□ aid_src_uwb_range[i].fused = true for each anchor
□ vehicle_local_position converges with UWB active, no GPS
□ Blocking one anchor → 3 remaining anchors still fuse (test_ratio normal)
□ Killing all UWB → timeout fires after 2s, _control_status.flags.uwb = false
□ UWB returns → filter re-converges within 2-3s
□ Covariance P decreases after fusion, increases during predict-only
```
