# HITL with Gazebo Harmonic

Hardware-in-the-Loop (HITL) simulation using [Gazebo Harmonic](https://gazebosim.org/docs/harmonic) runs real PX4 firmware on a physical flight controller while Gazebo provides simulated sensor data and visualizes motor outputs.

This implementation adds HITL support for Gazebo Harmonic to PX4, filling a gap that currently only existed for Gazebo Classic.
It is implemented as a Gazebo System plugin (`GZHILBridge`) that lives entirely in `src/modules/simulation/gz_bridge/hil/`.

## Overview

```
┌──────────────────────────────────────────────────────────────┐
│  Development Machine (Ubuntu 22.04 + Gz Harmonic)            │
│                                                               │
│  ┌─────────────┐  HIL_SENSOR     ┌───────────────────────┐  │
│  │  Gz Harmonic│ ─────────────► │                       │  │
│  │  (physics + │                 │   mavlink-routerd     │──┼──► QGC (UDP)
│  │   sensors)  │ ◄───────────── │   (serial ↔ TCP)      │  │
│  │             │  HIL_ACTUATOR   │                       │  │
│  │ GZHILBridge │  _CONTROLS      └──────────┬────────────┘  │
│  │  plugin     │                            │ USB serial     │
│  └─────────────┘                            │               │
└─────────────────────────────────────────────┼───────────────┘
                                              │
                                    ┌─────────▼──────────┐
                                    │  PX4 Flight         │
                                    │  Controller         │
                                    │  (real hardware,    │
                                    │   SYS_HITL=1)       │
                                    └────────────────────┘
```

**Data flow:**
1. Gazebo physics steps at 250 Hz
2. `GZHILBridge` subscribes to Gz sensor topics (IMU, mag, baro, GPS)
3. Encodes MAVLink `HIL_SENSOR` + `HIL_GPS` messages → sends to board via `mavlink-routerd` TCP
4. `mavlink-routerd` forwards to board via USB serial (`/dev/ttyACM0`)
5. Board flight controller runs real EKF, position control, etc.
6. Board sends `HIL_ACTUATOR_CONTROLS` → back through routerd → `GZHILBridge`
7. `GZHILBridge` publishes motor rad/s to Gazebo motor joints

## Supported Configurations

| Vehicle   | Gz Model    | World              |
|-----------|-------------|--------------------|
| x500 quad | `x500_hitl` | `hitl_default.sdf` |

## Prerequisites

### Hardware

- PX4 flight controller connected via USB (tested: Pixhawk 6X / `px4_fmu-v6x`)
- Verify: `ls /dev/ttyACM*` should show `/dev/ttyACM0`
- Add user to dialout group (once): `sudo usermod -aG dialout $USER` then re-login

### Software

Install Gazebo Harmonic and dependencies:

```bash
bash src/modules/simulation/gz_bridge/hil/setup_dev_env.sh
```

Install `mavlink-routerd`:

```bash
cd /tmp
git clone https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router && git submodule update --init --recursive
meson setup build && ninja -C build
sudo ninja -C build install
```

Install `pyserial` (used by the launch script for board readiness check):

```bash
pip3 install pyserial
```

### Board Setup (one-time)

In QGC, set these parameters on the board and reboot:

| Parameter        | Value     | Purpose                              |
|------------------|-----------|--------------------------------------|
| `SYS_HITL`       | `1`       | Enable HIL mode                      |
| `MAV_USEHILGPS`  | `1`       | Use HIL GPS instead of real GPS      |
| `CBRK_SUPPLY_CHK`| `894281`  | Bypass power supply check            |

After setting `SYS_HITL=1`, the board will not fly normally until it is reset to `0`.

## Build

`libGZHILBridge.so` is built automatically by `hil_launch.py` on first run if not present.
To build manually:

```bash
cmake -B build/px4_sitl_default -S .
cmake --build build/px4_sitl_default --target mavlink_c_generate
cmake --build build/px4_sitl_default --target GZHILBridge
```

Output: `build/px4_sitl_default/src/modules/simulation/gz_bridge/hil/libGZHILBridge.so`

## Running

```bash
make hil
```

With options:

```bash
PX4_HIL_DEVICE=/dev/ttyACM0 PX4_HIL_QGC_IP=172.17.128.1 make hil
```

Or directly:

```bash
python3 src/modules/simulation/gz_bridge/hil/hil_launch.py --qgc-ip 172.17.128.1
```

### What `make hil` does

1. Builds `libGZHILBridge.so` if not already built
2. Verifies board device exists and `mavlink-routerd` is in PATH
3. Starts `gz sim -s hitl_default.sdf` (server) + `gz sim -g` (GUI)
4. Waits for Gazebo world to be ready, then resumes simulation
5. Starts `mavlink-routerd` watchdog — auto-restarts on USB disconnect/board reboot
6. `GZHILBridge` plugin auto-reconnects TCP when `mavlink-routerd` restarts

### Environment variables

| Variable           | Default         | Description                              |
|--------------------|-----------------|------------------------------------------|
| `PX4_HIL_DEVICE`   | `/dev/ttyACM0`  | Serial device to board                   |
| `PX4_HIL_BAUD`     | `921600`        | Baud rate                                |
| `PX4_HIL_QGC_IP`   | `127.0.0.1`     | QGC host IP for UDP telemetry            |
| `PX4_HIL_QGC_PORT` | `14550`         | QGC UDP port                             |
| `PX4_GZ_WORLD`     | `hitl_default`  | Gazebo world name (without `.sdf`)       |
| `PX4_HIL_BUILD`    | `build/px4_sitl_default` | PX4 build directory             |
| `HEADLESS`         | unset           | Set to any value to skip Gz GUI          |

Example — QGC on Windows host (WSL2):

```bash
PX4_HIL_QGC_IP=172.17.128.1 make hil
```

### Reconnect behavior

When the USB cable is unplugged or the board reboots:

1. `mavlink-routerd` exits (serial port lost)
2. `GZHILBridge` detects TCP connection closed → retries every 2 s
3. Watchdog waits for board to re-enumerate **and** send bytes (confirms firmware running)
4. `mavlink-routerd` restarts → `GZHILBridge` reconnects TCP automatically
5. No Gazebo restart needed — model stays in world, plugin stays loaded

## Implementation

### Files

| File | Purpose |
|------|---------|
| `src/modules/simulation/gz_bridge/hil/GZHILBridge.hpp` | Plugin class declaration |
| `src/modules/simulation/gz_bridge/hil/GZHILBridge.cpp` | Plugin implementation |
| `src/modules/simulation/gz_bridge/hil/CMakeLists.txt` | Build definition |
| `src/modules/simulation/gz_bridge/hil/hil_launch.py` | Launch script (Gz + routerd watchdog) |
| `src/modules/simulation/gz_bridge/hil/setup_dev_env.sh` | Install Gz Harmonic + deps |
| `Tools/simulation/gz/worlds/hitl_default.sdf` | HIL world — pre-spawns `x500_hitl` |
| `Tools/simulation/gz/models/x500_hitl/` | x500 model with GZHILBridge plugin |

### GZHILBridge plugin

A Gazebo Harmonic System plugin implementing `ISystemConfigure` + `ISystemPostUpdate`.

**Sensor encoding** (Gz → board):

| Gz topic              | MAVLink message  | Frame conversion         |
|-----------------------|------------------|--------------------------|
| `imu_sensor/imu`      | `HIL_SENSOR`     | FLU → FRD (negate Y, Z)  |
| `magnetometer_sensor` | `HIL_SENSOR`     | Gz left-hand → FRD       |
| `air_pressure_sensor` | `HIL_SENSOR`     | Pa → hPa, ISA altitude   |
| `navsat_sensor/navsat`| `HIL_GPS`        | ENU velocity → NED       |

**Actuator decoding** (board → Gz):

`HIL_ACTUATOR_CONTROLS` normalized [0, 1] → scaled to `[0, 1000]` rad/s → published to `/x500_hitl/command/motor_speed`.

**Transport modes** (configured via model SDF params):

- **TCP mode** (default): `GZHILBridge` connects as TCP client to `mavlink-routerd` port 5760. Allows QGC and GZHILBridge to share one serial connection. Auto-reconnects every 2 s when TCP drops.
- **Serial mode**: `GZHILBridge` owns the serial port directly (no `mavlink-routerd`, no QGC).

**Thread safety:**

A `_fd_mutex` protects all fd reads/writes across three concurrent threads:
- Gz sensor callbacks (write MAVLink to TCP)
- `readerThread` (read `HIL_ACTUATOR_CONTROLS` from TCP)
- `PostUpdate` (reconnect retry + heartbeat)

### hil_launch.py

Python script that manages all processes from a single entry point:

- Builds plugin if missing (runs cmake targets `mavlink_c_generate` + `GZHILBridge`)
- Starts Gz server + GUI with correct environment variables
- Waits for world ready via `gz service /world/.../scene/info`
- Runs `RouterdWatchdog` thread: starts `mavlink-routerd`, detects exit, waits for board to be
  truly ready (reads bytes from serial — not just device node present), restarts routerd
- On routerd restart: logs reconnect info — no model respawn needed, plugin reconnects TCP on its own

### Build integration

`GZHILBridge` is built as a standalone cmake target (not part of the main PX4 firmware binary).
CMake detects `gz-sim` and `gz-plugin` automatically; if not found the target is skipped with a status message.

## Troubleshooting

**`[ERROR] Build succeeded but plugin not found`**
- Plugin is built to `build/px4_sitl_default/src/modules/simulation/gz_bridge/hil/libGZHILBridge.so`
- Delete the build directory and retry: `rm -rf build/px4_sitl_default && make hil`

**No sensor data in QGC after connecting**
- Verify `SYS_HITL=1` is set on the board and it has been rebooted
- Check routerd is running: `ps aux | grep mavlink-routerd`
- Check TCP connection: routerd log should show `TCP [N]dynamic: Connection accepted`
- Verify `GZ_SIM_SERVER_CONFIG_PATH` points to `src/modules/simulation/gz_bridge/server.config`

**`TCP Server: Could not bind to tcp socket (Address already in use)`**
- Old routerd instance still running: `pkill mavlink-routerd`
- Or a previous GZHILBridge TCP connection has not yet closed — wait 3–5 s and retry

**Board not detected after reboot**
- WSL2 keeps `/dev/ttyACM0` device node present even during board reboot
- The watchdog reads actual bytes from the port to confirm firmware is running before restarting routerd
- If board takes >30 s to boot, watchdog will wait indefinitely (by design)

**Gz sim crashes with ODE assertion**
- Physics explosion from control divergence — wait for EKF to fully initialize before arming
- Watch QGC for GPS fix (green icon) and no IMU bias warnings before takeoff

**`[GZHILBridge] Configure() called again — ignoring`**
- Normal with `merge='true'` in SDF — only the first call is used

**`libOpticalFlowSystem.so / libGstCameraSystem.so` not found**
- Harmless warnings — those plugins are not needed for HIL

**Gz GUI crash on WSL2 (NVIDIA / D3D12 segfault)**
- Set `LIBGL_ALWAYS_SOFTWARE=1` or use `HEADLESS=1 make hil`

## Related

- [PX4 Issue #22431](https://github.com/PX4/PX4-Autopilot/issues/22431) — upstream request for HITL support in new Gz
- [PX4 HITL docs](https://docs.px4.io/main/en/simulation/hitl.html) — upstream docs (Gazebo Classic only)
- `src/modules/simulation/gz_bridge/hil/GZHILBridge.hpp` — plugin API
