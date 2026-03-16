# Gz Harmonic HIL Implementation

## Status

> In development — branch `tf-px4-1.16`

PX4 1.16 with Gazebo Harmonic has **no HIL support**. This document defines the
implementation plan to add it. All new files are organized so they can be extracted
into a standalone `px4-gz-hil` repo later.

---

## Architecture Decision

**GZHILBridge is a Gz Harmonic System plugin** (standalone `.so`), not a PX4 uORB module.

Reasons:
- No PX4 source modification required for end users — just add plugin to world SDF
- Exact same pattern as Gz Classic's `libgazebo_mavlink_interface.so`
- Subscribes to Gz sensor topics directly (IMU, GPS, Baro, Mag) — no uORB dependency
- Drives Gz motors via `gz::transport` after receiving actuator commands from board
- Naturally extractable to standalone repo: only depends on `gz-sim8`, `gz-transport13`, mavlink headers

> This supersedes the earlier "Method 2 via uORB bridge" design. The Gz System plugin
> approach is cleaner, has zero PX4 runtime dependency, and is truly plug-and-play.

---

## Confirmed Data Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                        LINUX HOST (WSL)                             │
│                                                                     │
│  Gz Harmonic physics engine                                         │
│    IMU / GPS / Baro / Mag sensor plugins                            │
│       │                                                             │
│       │  gz::transport topics (internal Gz IPC)                     │
│       ▼                                                             │
│  libGZHILBridge.so   ← NEW Gz System plugin                        │
│  ISystemPostUpdate():                                               │
│    - reads ECM for IMU, GPS, Baro, Mag data                         │
│    - encodes → MAVLink HIL_SENSOR + HIL_GPS                         │
│    - writes → USB serial → Board                                    │
│  Reader thread:                                                     │
│    - reads USB serial ← Board                                       │
│    - decodes HIL_ACTUATOR_CONTROLS                                  │
│    - publishes gz::msgs::Actuators → Gz motor joints                │
└──────────────────────────┬──────────────────────────────────────────┘
                           │  USB Serial /dev/ttyACM0 @ 921600 baud
                           │  MAVLink protocol
┌──────────────────────────▼──────────────────────────────────────────┐
│                        BOARD (NuttX / FMU-v6x)                      │
│                                                                     │
│  SimulatorMavlink  [existing, unchanged]                            │
│    handle_message_hil_sensor() → uORB: sensor_accel/gyro/mag/baro  │
│    handle_message_hil_gps()    → uORB: sensor_gps                  │
│         │                                                           │
│         ▼  uORB                                                     │
│    EKF2 → state estimation                                          │
│         │                                                           │
│         ▼  uORB                                                     │
│    MC_ATT_CONTROL → MC_RATE_CONTROL → CONTROL_ALLOCATOR            │
│         │                                                           │
│         ▼  uORB: actuator_outputs_sim                               │
│    SimulatorMavlink                                                 │
│    send_controls() → MAVLink HIL_ACTUATOR_CONTROLS                 │
└─────────────────────────────────────────────────────────────────────┘
```

---

## File Layout

All new files are under a self-contained subtree. To extract to standalone repo later:
`git subtree split --prefix=src/modules/simulation/gz_bridge/hil`

```
tf-px4/
│
├── src/modules/simulation/gz_bridge/
│   ├── hil/                                    ← NEW — self-contained subtree
│   │   ├── GZHILBridge.hpp                     ← Gz System plugin header
│   │   ├── GZHILBridge.cpp                     ← Gz System plugin implementation
│   │   ├── CMakeLists.txt                      ← Standalone build (gz-sim8 only)
│   │   └── Kconfig                             ← PLATFORM_POSIX guard
│   ├── GZBridge.cpp                            ← MODIFY: skip sensor publish in HIL mode
│   └── CMakeLists.txt                          ← MODIFY: add_subdirectory(hil)
│
├── Tools/simulation/gz/
│   ├── models/
│   │   └── x500_hitl/                          ← NEW — HIL-ready model
│   │       ├── model.config
│   │       └── model.sdf                       ← x500 + GZHILBridge plugin
│   └── worlds/
│       └── hitl_default.sdf                    ← NEW — HIL world
│
├── ROMFS/px4fmu_common/init.d-posix/
│   ├── px4-rc.gzhil                            ← NEW — HIL startup script
│   └── px4-rc.simulator                        ← MODIFY: add HIL branch
│
└── boards/px4/fmu-v6x/
    └── tf.px4board                             ← MODIFY: add HIL config flag
```

---

## Tasks

### TASK 1 — `hil/CMakeLists.txt`

**File:** `src/modules/simulation/gz_bridge/hil/CMakeLists.txt`

Standalone CMake — builds `libGZHILBridge.so`. No PX4 module system needed.
Links only against gz-sim8, gz-transport13, gz-plugin2, mavlink headers.

```cmake
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(GZHILBridge SHARED
    GZHILBridge.cpp
)
target_include_directories(GZHILBridge PRIVATE
    ${CMAKE_BINARY_DIR}/mavlink
    ${CMAKE_BINARY_DIR}/mavlink/${CONFIG_MAVLINK_DIALECT}
)
target_link_libraries(GZHILBridge
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-plugin2::gz-plugin2
)
set_target_properties(GZHILBridge PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build_gazebo-harmonic
)
```

### TASK 2 — `hil/Kconfig`

**File:** `src/modules/simulation/gz_bridge/hil/Kconfig`

```kconfig
menuconfig MODULES_SIMULATION_GZ_HIL_BRIDGE
    bool "gz_hil_bridge"
    default n
    depends on PLATFORM_POSIX && MODULES_SIMULATION_GZ_MSGS
    ---help---
        Gazebo Harmonic HIL bridge plugin.
        Connects real PX4 hardware to Gz Harmonic simulation via USB serial.
```

### TASK 3 — `hil/GZHILBridge.hpp`

**File:** `src/modules/simulation/gz_bridge/hil/GZHILBridge.hpp`

Gz System plugin class. Interfaces:
- `ISystemConfigure` — read SDF params, open serial, start reader thread
- `ISystemPostUpdate` — read ECM sensor data, encode MAVLink, send to board

Key members:
```cpp
// Serial
int _serial_fd{-1};
std::string _serial_device{"/dev/ttyACM0"};
int _baud_rate{921600};

// Threading
std::thread _reader_thread;
std::mutex _actuator_mutex;
std::atomic<bool> _running{false};

// Actuator output to Gz
gz::transport::Node _node;
gz::transport::Node::Publisher _actuators_pub;
std::string _model_name;

// Latest actuator values from board
float _actuator_controls[16]{};
bool _armed{false};
```

### TASK 4 — `hil/GZHILBridge.cpp`

**File:** `src/modules/simulation/gz_bridge/hil/GZHILBridge.cpp`

Three logical sections:

**Section A — Configure():**
- Read `<serial_device>`, `<baud_rate>`, `<model_name>` from SDF
- Open serial fd with `open()` + `termios` setup
- Advertise `gz::msgs::Actuators` to `/<model_name>/command/motor_speed`
- Start `_reader_thread`

**Section B — PostUpdate():**
- Read IMU component from ECM → encode `HIL_SENSOR` (accel+gyro)
- Read Magnetometer → add to `HIL_SENSOR` (mag fields)
- Read AirPressure → add to `HIL_SENSOR` (baro)
- Read NavSat → encode `HIL_GPS`
- `write()` both messages to `_serial_fd`

**Section C — Reader thread:**
- `read()` loop on `_serial_fd`
- `mavlink_parse_char()` parse loop
- On `MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS`: decode, lock mutex, store controls, set armed flag
- Publish `gz::msgs::Actuators` to Gz

**Reference files:**
| What | Reference |
|---|---|
| Serial open + termios | `SimulatorMavlink.cpp` `do_serial_read()` |
| HIL_SENSOR encode | `mavlink_interface.cpp` `SendSensorMessages()` |
| HIL_GPS encode | `mavlink_interface.cpp` `SendGpsMessages()` |
| Actuator decode | `mavlink_interface.cpp` `handle_actuator_controls()` |
| Gz actuator publish | `GZMixingInterfaceESC.cpp` `updateOutputs()` |
| ECM sensor read | `gz_plugins/moving_platform_controller/MovingPlatformController.cpp` |
| Thread + I/O pattern | `gz_plugins/gstreamer/GstCameraSystem.cpp` |
| Plugin registration | `gz_plugins/template_plugin/TemplateSystem.cpp` |

### TASK 5 — Wire into build system

**File:** `src/modules/simulation/gz_bridge/CMakeLists.txt`

Add inside the existing `if(gz-transport_FOUND)` block:
```cmake
add_subdirectory(hil)
```

**File:** `boards/px4/fmu-v6x/tf.px4board`

```
CONFIG_MODULES_SIMULATION_GZ_HIL_BRIDGE=y
```

### TASK 6 — GZBridge HIL mode switch

**File:** `src/modules/simulation/gz_bridge/GZBridge.cpp`

When `SYS_HITL=1`, skip publishing sensor uORB topics and disable mixing interfaces.
GZHILBridge plugin handles all sensor → board and board → actuator paths directly.

```cpp
// Add member to GZBridge.hpp
int32_t _hil_mode{0};

// In GZBridge::Run()
param_get(param_find("SYS_HITL"), &_hil_mode);

// In imuCallback(), magnetometerCallback(), barometerCallback(), navSatCallback()
if (_hil_mode) { return; }

// In Run(), disable mixing interfaces
if (_hil_mode) {
    _mixing_interface_esc.stop();
    _mixing_interface_servo.stop();
}
```

### TASK 7 — Simulation assets

**File:** `Tools/simulation/gz/models/x500_hitl/model.sdf`

Copy `x500/model.sdf`, add GZHILBridge plugin block:
```xml
<plugin filename="libGZHILBridge.so" name="custom::GZHILBridge">
  <serial_device>/dev/ttyACM0</serial_device>
  <baud_rate>921600</baud_rate>
  <model_name>x500_hitl</model_name>
</plugin>
```

**File:** `Tools/simulation/gz/worlds/hitl_default.sdf`

Copy `default.sdf`, replace model include with `x500_hitl`.

### TASK 8 — Startup script

**File:** `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzhil`

```sh
#!/bin/sh
# Gz Harmonic HIL startup
# GZHILBridge plugin is loaded by Gz world SDF — no separate process needed
# Just launch gz sim with the HIL world

export GZ_SIM_SYSTEM_PLUGIN_PATH=${PX4_DIR}/build/px4_sitl_default/build_gazebo-harmonic

gz sim -r ${PX4_DIR}/Tools/simulation/gz/worlds/hitl_default.sdf &
```

**File:** `ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator`

Add before existing `elif [ "$PX4_SIMULATOR" = "gz" ]` check:
```sh
if [ "$PX4_SIMULATOR" = "gz" ] && [ "$(param show -q SYS_HITL)" = "1" ]; then
    . px4-rc.gzhil
elif [ "$PX4_SIMULATOR" = "gz" ] || ...
```

---

## Test File Layout

Test files live inside `hil/` so they travel with the standalone repo:

```
src/modules/simulation/gz_bridge/hil/
├── GZHILBridge.hpp
├── GZHILBridge.cpp
├── CMakeLists.txt
├── Kconfig
└── test/
    ├── CMakeLists.txt
    ├── test_mavlink_encoding.cpp     # Unit: HIL_SENSOR / HIL_GPS encode correctness
    ├── test_serial_framing.cpp       # Unit: MAVLink byte framing + parse round-trip
    ├── test_actuator_decode.cpp      # Unit: HIL_ACTUATOR_CONTROLS decode + armed flag
    ├── test_frame_conversion.cpp     # Unit: FLU/ENU → FRD/NED rotation correctness
    └── mock/
        ├── MockSerial.hpp            # Fake serial fd — write() captures bytes in buffer
        └── MockGzNode.hpp            # Fake gz::transport::Node — records published msgs
```

### What each test covers

| File | Type | Tests |
|---|---|---|
| `test_mavlink_encoding.cpp` | Unit | Given sensor values → encoded HIL_SENSOR bytes → decode back → values match |
| `test_serial_framing.cpp` | Unit | MAVLink parse loop handles partial reads, multi-message buffers, corrupt bytes |
| `test_actuator_decode.cpp` | Unit | HIL_ACTUATOR_CONTROLS decoded correctly, armed flag extracted from `mode` field |
| `test_frame_conversion.cpp` | Unit | IMU accel/gyro FLU→FRD rotation, GPS ENU→NED velocity rotation |

### Mock strategy

- `MockSerial` replaces `_serial_fd` — `write()` stores bytes in `std::vector<uint8_t>`,
  `read()` returns pre-loaded bytes. No real hardware needed in CI.
- `MockGzNode` replaces `gz::transport::Node::Publisher` — records last published
  `gz::msgs::Actuators` for assertion.

### Running tests

```bash
# Build tests
cmake --build build/px4_sitl_default --target gz_hil_tests

# Run
ctest --test-dir build/px4_sitl_default/src/modules/simulation/gz_bridge/hil/test -V
```

---

## Implementation Order

### Step 1 — Build skeleton
**Tasks:** TASK 1 + TASK 2 + empty `GZHILBridge.cpp/hpp`
**Test:** `cmake --build ... --target GZHILBridge` → `libGZHILBridge.so` produced, no errors

### Step 2 — Wire into PX4 build
**Tasks:** TASK 5
**Test:** `make px4_fmu-v6x_tf` and `make px4_sitl_default` both complete cleanly

### Step 3 — MAVLink encoding (with unit tests)
**Tasks:** TASK 4 Section B encode logic + `test_mavlink_encoding.cpp` + `test_frame_conversion.cpp`
**Test:** `ctest` passes all encoding unit tests before touching real hardware

### Step 4 — Serial framing (with unit tests)
**Tasks:** TASK 4 Section C reader thread + `test_serial_framing.cpp` + `test_actuator_decode.cpp`
**Test:** `ctest` passes — round-trip encode→send→receive→decode verified with MockSerial

### Step 5 — Hardware integration
**Tasks:** TASK 3 full `Configure()` with real serial open
**Test:** Board connected → plugin logs "Opened /dev/ttyACM0" → `listener sensor_accel` on board shows data

### Step 6 — Full flight test
**Tasks:** TASK 6 + TASK 7 + TASK 8
**Test:** Gz window shows x500_hitl model, QGC connects, arm → rotors spin, takeoff → stable hover

---

## Board Parameters (set once via QGC before HIL)

| Parameter | Value | Reason |
|---|---|---|
| `SYS_HITL` | `1` | Disable real sensor drivers, enable HIL receive |
| `SYS_AUTOSTART` | `1001` | Generic quadrotor airframe |
| `MAV_USEHILGPS` | `1` | Accept GPS from HIL_GPS MAVLink message |
| `CBRK_SUPPLY_CHK` | `894281` | Bypass power check (no real power module) |
| `CBRK_USB_CHK` | `197848` | Allow arming over USB |

---

## MAVLink Messages Reference

### HIL_SENSOR — Gz plugin → Board (msg ID 107)
| Field | Type | Unit |
|---|---|---|
| `time_usec` | uint64 | microseconds |
| `xacc/yacc/zacc` | float | m/s² body frame FRD |
| `xgyro/ygyro/zgyro` | float | rad/s |
| `xmag/ymag/zmag` | float | Gauss |
| `abs_pressure` | float | hPa |
| `diff_pressure` | float | hPa |
| `pressure_alt` | float | meters |
| `temperature` | float | °C |
| `fields_updated` | uint32 | bitmask of valid fields |
| `id` | uint8 | sensor instance (0–2) |

### HIL_GPS — Gz plugin → Board (msg ID 113)
| Field | Type | Unit |
|---|---|---|
| `time_usec` | uint64 | microseconds |
| `lat/lon` | int32 | degrees × 1e7 |
| `alt` | int32 | mm MSL |
| `vel` | uint16 | cm/s ground speed |
| `vn/ve/vd` | int16 | cm/s NED velocity |
| `cog` | uint16 | centidegrees |
| `satellites_visible` | uint8 | |
| `id` | uint8 | GPS instance |

### HIL_ACTUATOR_CONTROLS — Board → Gz plugin (msg ID 93)
| Field | Type | Notes |
|---|---|---|
| `time_usec` | uint64 | microseconds |
| `controls[16]` | float | normalized, motors: [0,1] |
| `mode` | uint8 | bit7 = armed |
| `flags` | uint8 | bit0 = lockstep |

---

## Coordinate Frames

Gz Harmonic uses **FLU/ENU**. PX4 uses **FRD/NED**.

The Gz sensor plugins output data in their native frame. `GZHILBridge` must convert
before encoding MAVLink — same rotation that `GZBridge.cpp` applies:

```cpp
// FLU → FRD
static const gz::math::Quaterniond q_FLU_to_FRD(0, 1, 0, 0);
// ENU → NED
static const gz::math::Quaterniond q_ENU_to_NED(0, 0.70711, 0.70711, 0);
```

Reference: `GZBridge.cpp` `imuCallback()` and `navSatCallback()`.

---

## Future: Extracting to Standalone Repo

When ready to publish as `px4-gz-hil`:

```bash
# Extract hil/ subtree
git subtree split --prefix=src/modules/simulation/gz_bridge/hil -b hil-only

# Copy assets
cp -r Tools/simulation/gz/models/x500_hitl  px4-gz-hil/models/
cp -r Tools/simulation/gz/worlds/hitl_default.sdf  px4-gz-hil/worlds/

# The CMakeLists.txt in hil/ is already standalone — no changes needed
```

The plugin `.so` will work with any PX4 1.15+ firmware without recompiling PX4.
