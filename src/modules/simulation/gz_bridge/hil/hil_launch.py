#!/usr/bin/env python3
"""
HIL (Hardware-in-the-Loop) launcher for PX4 + Gazebo Harmonic.

Manages:
  - mavlink-routerd watchdog (auto-restart on USB disconnect/board reboot)
  - Gazebo server + GUI (started once; model respawned on board reconnect)
  - Clean shutdown of all child processes on Ctrl-C

Usage:
    python3 hil_launch.py [options]
    python3 hil_launch.py --device /dev/ttyACM1 --qgc-ip 192.168.1.100

Environment variable overrides (same as Makefile):
    PX4_HIL_DEVICE, PX4_HIL_BAUD, PX4_HIL_QGC_IP, PX4_HIL_QGC_PORT,
    PX4_GZ_WORLD, PX4_HIL_BUILD, HEADLESS
"""

import argparse
import os
import signal
import socket
import subprocess
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Resolve repo root (this file lives at <repo>/src/modules/simulation/gz_bridge/hil/)
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_DIR   = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", "..", ".."))

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="PX4 HIL launcher — Gazebo Harmonic + mavlink-routerd + real board"
    )
    p.add_argument("--device",    default=os.environ.get("PX4_HIL_DEVICE",   "/dev/ttyACM0"),
                   help="Serial device of the board (default: /dev/ttyACM0)")
    p.add_argument("--baud",      default=int(os.environ.get("PX4_HIL_BAUD",  "921600")),
                   type=int,      help="Baud rate (default: 921600)")
    p.add_argument("--qgc-ip",    default=os.environ.get("PX4_HIL_QGC_IP",   "127.0.0.1"),
                   dest="qgc_ip", help="QGroundControl IP (default: 127.0.0.1)")
    p.add_argument("--qgc-port",  default=int(os.environ.get("PX4_HIL_QGC_PORT", "14550")),
                   type=int, dest="qgc_port",
                   help="QGroundControl UDP port (default: 14550)")
    p.add_argument("--world",     default=os.environ.get("PX4_GZ_WORLD",     "hitl_default"),
                   help="Gazebo world name (default: hitl_default)")
    p.add_argument("--build",     default=os.environ.get("PX4_HIL_BUILD",
                                                          os.path.join(REPO_DIR, "build", "px4_sitl_default")),
                   help="PX4 build directory")
    p.add_argument("--headless",  action="store_true",
                   default=bool(os.environ.get("HEADLESS", "")),
                   help="Do not launch Gazebo GUI")
    p.add_argument("--tcp-port",  default=5760, type=int, dest="tcp_port",
                   help="mavlink-routerd TCP port (default: 5760)")
    return p.parse_args()

# ---------------------------------------------------------------------------
# Logging helpers
# ---------------------------------------------------------------------------

def log(tag, msg):
    print(f"[{tag}] {msg}", flush=True)

def info(msg):  log("INFO ", msg)
def warn(msg):  log("WARN ", msg)
def error(msg): log("ERROR", msg)

# ---------------------------------------------------------------------------
# Plugin / build check
# ---------------------------------------------------------------------------

def ensure_plugin_built(build_dir, repo_dir):
    plugin = os.path.join(build_dir, "src", "modules", "simulation", "gz_bridge", "hil", "libGZHILBridge.so")
    if os.path.isfile(plugin):
        info(f"GZHILBridge.so found at {plugin}")
        return plugin

    info("GZHILBridge.so not found — building...")
    def run(cmd):
        result = subprocess.run(cmd, cwd=repo_dir)
        if result.returncode != 0:
            error(f"Build step failed: {' '.join(cmd)}")
            sys.exit(1)

    run(["cmake", "-B", build_dir, "-S", repo_dir])
    run(["cmake", "--build", build_dir, "--target", "mavlink_c_generate"])
    run(["cmake", "--build", build_dir, "--target", "GZHILBridge"])

    if not os.path.isfile(plugin):
        error("Build succeeded but plugin not found — check cmake output")
        sys.exit(1)

    info("Build complete")
    return plugin

# ---------------------------------------------------------------------------
# Board readiness check
# ---------------------------------------------------------------------------

def board_is_ready(device, baud, timeout_s=4):
    """Return True if the serial device exists and is actively sending bytes."""
    if not os.path.exists(device):
        return False
    try:
        import serial
        s = serial.Serial(device, baud, timeout=timeout_s)
        data = s.read(64)
        s.close()
        return len(data) > 0
    except Exception:
        return False

# ---------------------------------------------------------------------------
# TCP readiness check
# ---------------------------------------------------------------------------

def tcp_port_open(host, port, timeout=1.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except (OSError, ConnectionRefusedError):
        return False

# ---------------------------------------------------------------------------
# Gazebo helpers
# ---------------------------------------------------------------------------

def gz_service_available(world):
    """Check if the Gazebo world scene/info service is responding."""
    result = subprocess.run(
        ["gz", "service", "-i", "--service", f"/world/{world}/scene/info"],
        capture_output=True, text=True, timeout=3
    )
    return "Service providers" in result.stdout

def gz_resume(world):
    subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/control",
         "--reqtype", "gz.msgs.WorldControl",
         "--reptype", "gz.msgs.Boolean",
         "--timeout", "5000",
         "--req", "pause: false"],
        capture_output=True
    )

def gz_respawn_model(world, model_name="x500_hitl"):
    """Remove the existing model and re-include it from the SDF URI."""
    info(f"Respawning model {model_name} in world {world}")

    # Delete the existing model
    subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/remove",
         "--reqtype", "gz.msgs.Entity",
         "--reptype", "gz.msgs.Boolean",
         "--timeout", "5000",
         "--req", f'name: "{model_name}" type: MODEL'],
        capture_output=True
    )
    time.sleep(0.5)

    # Re-create it via the create service (spawn from model URI)
    sdf_spawn = (
        f'<sdf version="1.9">'
        f'<include>'
        f'<uri>model://{model_name}</uri>'
        f'<name>{model_name}</name>'
        f'<pose>0 0 0.2 0 0 0</pose>'
        f'</include>'
        f'</sdf>'
    )
    subprocess.run(
        ["gz", "service", "-s", f"/world/{world}/create",
         "--reqtype", "gz.msgs.EntityFactory",
         "--reptype", "gz.msgs.Boolean",
         "--timeout", "5000",
         "--req", f'sdf: "{sdf_spawn}"'],
        capture_output=True
    )

# ---------------------------------------------------------------------------
# Process group — kills all children on exit
# ---------------------------------------------------------------------------

_child_procs = []
_procs_lock  = threading.Lock()
_shutdown    = threading.Event()

def register_proc(proc):
    with _procs_lock:
        _child_procs.append(proc)

def kill_all():
    # Kill every registered child, then kill the entire process group
    with _procs_lock:
        for p in _child_procs:
            try: p.terminate()
            except Exception: pass
        for p in _child_procs:
            try: p.wait(timeout=2)
            except Exception:
                try: p.kill()
                except Exception: pass

    # Also send SIGTERM to our own process group to catch any stray children
    try:
        os.killpg(os.getpgrp(), signal.SIGTERM)
    except Exception:
        pass

def _signal_handler(sig, frame):
    if _shutdown.is_set():
        return  # ignore repeated Ctrl+C
    info("Shutting down...")
    _shutdown.set()

# ---------------------------------------------------------------------------
# mavlink-routerd watchdog thread
# ---------------------------------------------------------------------------

class RouterdWatchdog(threading.Thread):
    def __init__(self, device, baud, qgc_ip, qgc_port, tcp_port,
                 gz_world, on_reconnect_cb):
        super().__init__(daemon=True, name="routerd-watchdog")
        self.device       = device
        self.baud         = baud
        self.qgc_ip       = qgc_ip
        self.qgc_port     = qgc_port
        self.tcp_port     = tcp_port
        self.gz_world     = gz_world
        self.on_reconnect = on_reconnect_cb  # called after routerd restarts and TCP is up
        self._stop        = threading.Event()
        self._proc        = None

    def stop(self):
        self._stop.set()
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()

    def _start_routerd(self):
        cmd = [
            "mavlink-routerd",
            "-e", f"{self.qgc_ip}:{self.qgc_port}",
            f"{self.device}:{self.baud}",
        ]
        info(f"Starting mavlink-routerd: {' '.join(cmd)}")
        proc = subprocess.Popen(cmd)
        register_proc(proc)
        self._proc = proc
        return proc

    def _wait_for_board(self):
        """Wait until board is physically present and sending bytes."""
        info("Waiting for board to re-enumerate and boot...")
        while not self._stop.is_set() and not _shutdown.is_set():
            if board_is_ready(self.device, self.baud):
                info("Board is ready")
                return True
            warn(f"Board not ready at {self.device}, retrying in 2s...")
            time.sleep(2)
        return False

    def _wait_for_tcp(self):
        """Wait until mavlink-routerd TCP port is accepting connections."""
        info(f"Waiting for mavlink-routerd TCP :{self.tcp_port}...")
        for _ in range(15):
            if self._stop.is_set() or _shutdown.is_set():
                return False
            if tcp_port_open("127.0.0.1", self.tcp_port):
                info(f"mavlink-routerd TCP :{self.tcp_port} ready")
                return True
            time.sleep(1)
        error("Timeout waiting for mavlink-routerd TCP")
        return False

    def run(self):
        is_restart = False
        while not self._stop.is_set() and not _shutdown.is_set():
            if is_restart:
                # routerd exited — board rebooted or USB disconnected
                warn("mavlink-routerd exited")
                # Wait for lingering TCP connections to close before routerd rebinds port
                time.sleep(3)
                if not self._wait_for_board():
                    break

            proc = self._start_routerd()

            if not self._wait_for_tcp():
                proc.terminate()
                is_restart = True
                continue

            # On restart: respawn model to reset drone position.
            # On first start: world SDF already includes the model — do nothing.
            if is_restart:
                self.on_reconnect()

            is_restart = True
            # Block until routerd exits
            proc.wait()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    info(f"Repo:    {REPO_DIR}")
    info(f"Build:   {args.build}")
    info(f"Device:  {args.device} @ {args.baud}")
    info(f"QGC:     {args.qgc_ip}:{args.qgc_port}")
    info(f"World:   {args.world}")
    info(f"Headless:{args.headless}")

    # 1. Build plugin if needed
    plugin = ensure_plugin_built(args.build, REPO_DIR)
    plugin_dir = os.path.dirname(plugin)

    # 2. Check board present before starting
    if not os.path.exists(args.device):
        error(f"Board not found at {args.device} — plug in board and retry")
        sys.exit(1)

    # 3. Check mavlink-routerd available
    if subprocess.run(["which", "mavlink-routerd"], capture_output=True).returncode != 0:
        error("mavlink-routerd not found in PATH")
        sys.exit(1)

    # 4. Set up Gazebo environment
    # HIL assets (world + model) live in the gz submodule (Tools/simulation/gz).
    gz_submodule = os.path.join(REPO_DIR, "Tools", "simulation", "gz")

    env = os.environ.copy()
    env["GZ_SIM_RESOURCE_PATH"] = (
        f"{gz_submodule}/models:"
        f"{gz_submodule}/worlds"
    )
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = plugin_dir  # .../gz_bridge/hil/
    env["GZ_SIM_SERVER_CONFIG_PATH"] = (
        f"{REPO_DIR}/src/modules/simulation/gz_bridge/server.config"
    )

    world_sdf = os.path.join(gz_submodule, "worlds", f"{args.world}.sdf")
    if not os.path.isfile(world_sdf):
        error(f"World SDF not found: {args.world}.sdf")
        sys.exit(1)

    # 5. Start Gazebo server
    info(f"Starting Gazebo server: {args.world}")
    gz_server = subprocess.Popen(["gz", "sim", "-s", world_sdf], env=env)
    register_proc(gz_server)

    # 6. Start Gazebo GUI (unless headless)
    if not args.headless:
        info("Starting Gazebo GUI")
        gz_gui = subprocess.Popen(["gz", "sim", "-g"], env=env)
        register_proc(gz_gui)

    # 7. Wait for Gazebo world to be ready
    info("Waiting for Gazebo world to be ready...")
    for i in range(30):
        try:
            if gz_service_available(args.world):
                info("Gazebo world ready")
                break
        except Exception:
            pass
        time.sleep(1)
    else:
        error("Timed out waiting for Gazebo world")
        kill_all()
        sys.exit(1)

    time.sleep(1)
    info("Resuming simulation")
    gz_resume(args.world)

    # 8. on_reconnect callback — called when routerd restarts (not first start)
    # GZHILBridge plugin stays loaded in Gz and auto-reconnects TCP on its own (2s retry).
    # No model respawn needed — respawning unloads the plugin and breaks the connection.
    def on_routerd_reconnect():
        info("mavlink-routerd restarted — GZHILBridge will reconnect TCP automatically")

    # 9. Start routerd watchdog
    watchdog = RouterdWatchdog(
        device=args.device,
        baud=args.baud,
        qgc_ip=args.qgc_ip,
        qgc_port=args.qgc_port,
        tcp_port=args.tcp_port,
        gz_world=args.world,
        on_reconnect_cb=on_routerd_reconnect,
    )
    watchdog.start()

    # 10. Wait — block until Ctrl+C or Gazebo server exits
    try:
        while not _shutdown.is_set():
            if gz_server.poll() is not None:
                info("Gazebo server exited")
                break
            _shutdown.wait(timeout=1)
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown.set()
        info("Shutting down all processes...")
        watchdog.stop()
        kill_all()
        sys.exit(0)


if __name__ == "__main__":
    main()
