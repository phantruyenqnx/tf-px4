# HIL Simulation with tf.px4board on Gazebo Classic

Hardware-in-the-Loop (HIL) simulation runs the actual PX4 firmware on the real Pixhawk v6X board while Gazebo Classic provides simulated sensor data (IMU, GPS, barometer) and renders the vehicle physics. No real sensors or motors are used during flight.

## Architecture

```
Board (SYS_HITL=1)
  ↕ USB/Serial (ttyACMx, 921600 baud)
Gazebo Classic (WSL)
  - libgazebo_mavlink_interface.so reads serial
  - simulates physics, sends HIL_SENSOR / HIL_GPS to board
  - receives ACTUATOR_OUTPUTS from board, spins virtual motors
  ↕ UDP 14550
QGroundControl (Windows)
  - auto-detects UDP 14550 broadcast from Gazebo plugin
```

---

## Prerequisites

### 1. Build tf firmware

```bash
cd ~/tf-px4
make px4_fmu-v6x_tf
```

Flash the resulting `build/px4_fmu-v6x_tf/px4_fmu-v6x_tf.px4` to the board via QGC.

### 2. Build Gazebo Classic plugins

The HIL world requires `libgazebo_mavlink_interface.so` built from the PX4 sitl_gazebo-classic submodule.

```bash
cd ~/tf-px4
make px4_sitl_default sitl_gazebo-classic
```

Output plugins are placed in `build/px4_sitl_default/build_gazebo-classic/`.

### 3. Install Gazebo Classic 11 (Ubuntu 22.04)

Ubuntu 22.04's default PX4 setup script installs Gz Harmonic, which does **not** support HIL worlds. Install Gazebo Classic 11 manually:

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

sudo apt-get update
sudo apt-get install gazebo libgazebo-dev
```

Verify: `gzserver --version` should print `version 11.x`.

### 4. Install mavlink-router (for QGC on Windows)

Used to forward MAVLink from WSL to QGC running on Windows over UDP.

```bash
# Dependencies
sudo apt install meson ninja-build pkg-config gcc g++

# Build
git clone https://github.com/mavlink-router/mavlink-router.git ~/mavlink-router
cd ~/mavlink-router
git submodule update --init --recursive
meson setup build .
ninja -C build
sudo ninja -C build install
```

### 5. Fix serial device (udev rule)

Each USB reconnect may assign a different `/dev/ttyACMx` number. Create a persistent symlink:

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="3185", ATTRS{idProduct}=="0035", SYMLINK+="px4fmu"' \
  | sudo tee /etc/udev/rules.d/99-px4.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
```

The board will always be available as `/dev/px4fmu` after replug.

Update the HIL model to use the symlink:

```
Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_hitl/iris_hitl.sdf
```

```xml
<serialDevice>/dev/px4fmu</serialDevice>
<baudRate>921600</baudRate>
```

---

## Board Parameters

Load or manually set the following parameters via QGC (Vehicle Setup → Parameters):

| Parameter | Value | Notes |
|---|---|---|
| `SYS_HITL` | `1` | Enable HIL mode, disables real sensors |
| `SYS_AUTOSTART` | `1001` | Iris quadrotor airframe |
| `CBRK_SUPPLY_CHK` | `894281` | Bypass power supply check |
| `CBRK_USB_CHK` | `197848` | Allow arming over USB |
| `CBRK_IO_SAFETY` | `22027` | Bypass IO safety switch |
| `MAV_USEHILGPS` | `1` | Use GPS data from Gazebo HIL |
| `MAV_0_CONFIG` | `101` | MAVLink on USB (TEL1) |
| `HIL_ACT_FUNC1` | `101` | Motor 1 → actuator output 1 |
| `HIL_ACT_FUNC2` | `102` | Motor 2 → actuator output 2 |
| `HIL_ACT_FUNC3` | `103` | Motor 3 → actuator output 3 |
| `HIL_ACT_FUNC4` | `104` | Motor 4 → actuator output 4 |

> After setting all parameters, reboot the board.

A pre-saved parameter file is available at:
`QGroundControl/Parameters/HIL1.params`

---

## Running HIL Simulation

All steps must be run in the **same terminal session** so environment variables are inherited.

### Step 1 — Connect the board

Plug in the Pixhawk v6X via USB. Verify it appears:

```bash
lsusb | grep Auterion
# Expected: ID 3185:0035 Auterion PX4 FMU v6X.x

ls /dev/px4fmu   # or /dev/ttyACMx if udev rule not set
```

### Step 2 — Source Gazebo environment

```bash
cd ~/tf-px4
source Tools/simulation/gazebo-classic/setup_gazebo.bash \
  $(pwd) $(pwd)/build/px4_sitl_default

# Verify
echo $GAZEBO_PLUGIN_PATH   # should point to build/px4_sitl_default/build_gazebo-classic
echo $GAZEBO_MODEL_PATH    # should point to sitl_gazebo-classic/models
```

### Step 3 — Launch Gazebo

```bash
gzserver Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world &
gzclient
```

Expected output:
```
Opened serial device /dev/px4fmu
```

The Iris drone model should appear in the Gazebo window.

### Step 4 — Forward MAVLink to QGC on Windows

Get the Windows host IP from WSL:
```bash
ip route show default
# e.g.: default via 172.17.128.1 dev eth0
```

Run mavlink-router:
```bash
mavlink-routerd -e 172.17.128.1:14550 0.0.0.0:14550
```

### Step 5 — Connect QGC

In QGroundControl on Windows:
- **Application Settings → Comm Links** — UDP auto-connect on port `14550` should detect the vehicle automatically.
- Ensure USB auto-connect is **disabled** (the board is connected through WSL, not Windows COM port).

---

## Troubleshooting

### `Error opening serial device: open: No such file or directory`

Device number changed after reconnect. Check:
```bash
ls /dev/ttyACM*
```
Either update `<serialDevice>` in `iris_hitl.sdf` or set up the udev rule above.

### `QGC UDP bind failed: Cannot assign requested address`

The `<qgc_addr>` in `iris_hitl.sdf` is set to a Windows IP that does not exist inside WSL. Set it to:
```xml
<qgc_addr>INADDR_ANY</qgc_addr>
```
Then use mavlink-router to forward UDP to Windows.

### Drone position jumps erratically in QGC

Likely cause: `MAV_USEHILGPS = 0` — EKF2 is not receiving GPS from Gazebo. Set `MAV_USEHILGPS = 1` and reboot.

### Gazebo model does not appear / plugin not loaded

`setup_gazebo.bash` was not sourced before launching gzserver. `GAZEBO_PLUGIN_PATH` must be set in the same shell session. Kill gzserver and restart from Step 2.

### `Serial port closed!` loop in gzserver output

Board is connected but Gazebo cannot open the serial port. Possible causes:
- User not in `dialout` group: `sudo usermod -aG dialout $USER` (logout/login required)
- Another process (mavlink-router) already has the port open — kill it first

---

## Notes

- **Gazebo Classic vs Gz Harmonic**: The HIL world (`hitl_iris.world`) and `libgazebo_mavlink_interface.so` only work with Gazebo Classic 11. Gz Harmonic (default on Ubuntu 22.04) does not support HIL mode.
- **mavlink-router is not needed if only using Gazebo**: The Gazebo plugin connects directly to the serial port and also broadcasts QGC data on UDP 14550. mavlink-router is only needed to forward that UDP traffic to a Windows host.
- **Board firmware**: Use `px4_fmu-v6x_tf` build, not `px4_fmu-v6x_default`. The `tf.px4board` config removes unused drivers (UAVCAN, Ethernet, camera, FW/VTOL modules) to reduce firmware size.
