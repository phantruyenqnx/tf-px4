#!/usr/bin/env python3
"""UWB precision-landing mission runner (pymavlink).

Drives ONE flight on an already-running PX4 SITL (`make px4_sitl gz_f450-uwb_uwb`):
  set scenario params -> upload an AUTO mission (takeoff -> ~box laps far from the pad so the
  GPS bias drifts uncorrected -> waypoint over the pad -> NAV_LAND at home) -> arm -> start ->
  wait until landed -> record the EKF landing position + timing -> disarm -> append a CSV row.

The TRUE landing error (vs the pad ground-truth) is computed later by analyze.py from the ulog
(ground-truth is not on the MAVLink link). Per-seed runs are driven by a launcher that reboots
SITL per SIM_GPS_SEED (that param is consumed at boot); this script handles the runtime params.

Scenarios:
  A   GPS-only baseline            EKF2_UWB_CTRL=0
  B1  GPS+UWB passive              EKF2_UWB_CTRL=1, EKF2_UWB_GPS=0
  B2  GPS+UWB + GPS R-inflation    EKF2_UWB_CTRL=1, EKF2_UWB_GPS=1   (needs Task 6 feature)

Usage:
  python3 mission.py --scenario A --out runs.csv
"""
import argparse
import csv
import math
import os
import struct
import sys
import time

from pymavlink import mavutil

# PX4 transmits INT params bit-cast into the float param_value field (NOT as a numeric float).
# Sending a numeric float for an INT param corrupts it (e.g. set 7 -> float 7.0 -> PX4 reads the
# bits as int = 1088421888). So INT params must be packed/unpacked by reinterpreting the 4 bytes.
_REAL_TYPES = {mavutil.mavlink.MAV_PARAM_TYPE_REAL32, mavutil.mavlink.MAV_PARAM_TYPE_REAL64}


def _is_int_type(ptype):
    return ptype not in _REAL_TYPES


def _bits_to_int(f):
    return struct.unpack('<i', struct.pack('<f', f))[0]


def _int_to_bits(i):
    return struct.unpack('<f', struct.pack('<i', int(i)))[0]

# MAVLink enums (kept explicit so the script is self-documenting)
FRAME_GLOBAL_REL_ALT = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
FRAME_MISSION = mavutil.mavlink.MAV_FRAME_MISSION
CMD_WAYPOINT = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
CMD_TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
CMD_LAND = mavutil.mavlink.MAV_CMD_NAV_LAND
CMD_RTL = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
MISSION_TYPE = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
LANDED_ON_GROUND = mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
LANDED_IN_AIR = mavutil.mavlink.MAV_LANDED_STATE_IN_AIR

# PX4 custom mode (AUTO / MISSION)
PX4_MAIN_AUTO = 4
PX4_SUB_AUTO_MISSION = 4

_MAIN_MODE = {1: 'MANUAL', 2: 'ALTCTL', 3: 'POSCTL', 4: 'AUTO', 5: 'ACRO', 6: 'OFFBOARD', 7: 'STAB'}
_AUTO_SUB = {1: 'READY', 2: 'TAKEOFF', 3: 'LOITER', 4: 'MISSION', 5: 'RTL', 6: 'LAND',
             8: 'FOLLOW', 9: 'PRECLAND'}


def mode_name(custom_mode):
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    if main == 4:
        return f"AUTO.{_AUTO_SUB.get(sub, sub)}"
    return _MAIN_MODE.get(main, str(main))


SCENARIOS = {
    'A': {'EKF2_UWB_CTRL': 0},                          # GPS only (baseline)
    'B': {'EKF2_UWB_CTRL': 1, 'EKF2_UWB_GPS': 1},       # GPS + UWB, UWB-dominant near the pad
}

EARTH_R = 6378137.0


def log(msg):
    print(f"[mission] {msg}", flush=True)


def offset_ll(lat, lon, dn, de):
    """Add a north/east offset (m) to a lat/lon (deg)."""
    dlat = dn / EARTH_R
    dlon = de / (EARTH_R * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


class Mission:
    def __init__(self, conn):
        self.m = mavutil.mavlink_connection(conn)
        log(f"waiting for autopilot heartbeat on {conn} ...")
        ap = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1  # = 1; ignore GCS / system-0 heartbeats
        end = time.time() + 60
        hb = None
        while time.time() < end:
            h = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if h and h.get_srcSystem() != 0 and h.get_srcComponent() == ap:
                hb = h
                break
        if hb is None:
            raise TimeoutError("no autopilot heartbeat (is the sim up? close other GCS?)")
        self.tsys, self.tcomp = hb.get_srcSystem(), hb.get_srcComponent()
        # pymavlink uses these as defaults for *_send target fields
        self.m.target_system, self.m.target_component = self.tsys, self.tcomp
        log(f"connected: system {self.tsys} component {self.tcomp}")

    # ---- params -------------------------------------------------------------
    @staticmethod
    def _pid(msg):
        pid = msg.param_id
        return pid.decode() if isinstance(pid, bytes) else pid

    def get_param(self, name):
        """Return (value, ptype); INT params decoded from the bit-cast float field."""
        self.m.mav.param_request_read_send(self.tsys, self.tcomp, name.encode(), -1)
        msg = self._wait('PARAM_VALUE', 2.0, match=lambda x: self._pid(x).rstrip('\x00') == name)
        if msg is None:
            return None, None
        val = _bits_to_int(msg.param_value) if _is_int_type(msg.param_type) else msg.param_value
        return val, msg.param_type

    def set_param(self, name, value, ptype):
        pv = _int_to_bits(value) if _is_int_type(ptype) else float(value)
        for _ in range(5):
            self.m.mav.param_set_send(self.tsys, self.tcomp, name.encode(), pv, ptype)
            cur, _ = self.get_param(name)
            if cur is not None and abs(cur - float(value)) < 1e-3:
                log(f"param {name} = {value}")
                return True
        log(f"WARN could not confirm param {name}={value}")
        return False

    def ensure_param(self, name, value):
        """Set a param only if it differs (a redundant param_set still triggers a
        parameter_update that resets EKF GPS aiding ~6 s). Returns True if changed."""
        cur, ptype = self.get_param(name)
        if ptype is None:
            ptype = (mavutil.mavlink.MAV_PARAM_TYPE_INT32 if float(value).is_integer()
                     else mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        if cur is not None and abs(cur - float(value)) < 1e-3:
            log(f"param {name} already {value} (skip)")
            return False
        ok = self.set_param(name, value, ptype)
        if not ok:
            log(f"param {name} not available — skipping (feature not built?)")
        return ok  # 'changed' only if actually set

    # ---- streams ------------------------------------------------------------
    def request_stream(self, msg_id, hz):
        self.m.mav.command_long_send(
            self.tsys, self.tcomp, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    # ---- helpers ------------------------------------------------------------
    def _wait(self, mtype, timeout, match=None):
        end = time.time() + timeout
        while time.time() < end:
            msg = self.m.recv_match(type=mtype, blocking=True, timeout=end - time.time())
            if msg is None:
                return None
            if match is None or match(msg):
                return msg
        return None

    def wait_global_pos(self, timeout=120):
        log("waiting for valid global position (EKF/GPS ready) ...")
        end = time.time() + timeout
        while time.time() < end:
            msg = self.m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if msg and msg.lat != 0:
                return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0
        raise TimeoutError("no global position")

    def wait_position_stable(self, timeout=40, settle=5.0, jump_int=50):
        """Wait until GLOBAL_POSITION_INT stops jumping (EKF settled on GPS, not
        dead-reckoning / resetting). jump_int ~ 0.55 m in 1e-7 deg units."""
        log("waiting for EKF position to settle (no resets) ...")
        end = time.time() + timeout
        last = None
        stable_since = None
        while time.time() < end:
            m = self.m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if not m or m.lat == 0:
                stable_since = None
                continue
            p = (m.lat, m.lon)
            if last is not None and abs(p[0] - last[0]) + abs(p[1] - last[1]) > jump_int:
                stable_since = None  # a reset/jump -> not settled
            if stable_since is None:
                stable_since = time.time()
            last = p
            if time.time() - stable_since >= settle:
                log("position settled")
                return True
        log("WARN position did not settle within timeout (continuing)")
        return False

    # ---- mission upload -----------------------------------------------------
    def upload(self, items):
        self.m.mav.mission_clear_all_send(self.tsys, self.tcomp, MISSION_TYPE)
        time.sleep(0.5)
        self.m.mav.mission_count_send(self.tsys, self.tcomp, len(items), MISSION_TYPE)
        sent = set()
        end = time.time() + 30
        while len(sent) < len(items) and time.time() < end:
            req = self.m.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                                    blocking=True, timeout=5)
            if req is None:
                continue
            seq = req.seq
            self.m.mav.send(items[seq])
            sent.add(seq)
        ack = self._wait('MISSION_ACK', 5)
        ok = ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
        log(f"mission upload: {len(sent)}/{len(items)} items, ack={'OK' if ok else ack}")
        return ok

    def _item(self, seq, cmd, lat, lon, alt, current=0, p1=0.0, frame=FRAME_GLOBAL_REL_ALT):
        return self.m.mav.mission_item_int_encode(
            self.tsys, self.tcomp, seq, frame, cmd,
            current, 1, p1, 0.0, 0.0, float('nan'),
            int(lat * 1e7), int(lon * 1e7), float(alt), MISSION_TYPE)

    def get_global_origin(self, timeout=10):
        """EKF local-frame origin (lat/lon). A global setpoint maps to local via this FIXED
        origin, so it is the right reference for 'land at the pad' (bias cancels in the
        global->local conversion)."""
        self.m.mav.command_long_send(
            self.tsys, self.tcomp, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, 0, 0, 0, 0, 0, 0)
        msg = self._wait('GPS_GLOBAL_ORIGIN', timeout)
        return (msg.latitude / 1e7, msg.longitude / 1e7) if msg else None

    def build(self, lat_pad, lon_pad, alt, box, laps, approach_alt):
        """Take off, fly a box >14 m from the pad, return over the pad and AUTO.LAND on it.
        The pad is the UWB anchor centroid expressed via the EKF origin, so NAV_LAND maps to the
        local anchor centroid: with UWB the estimate is pad-accurate -> lands on the pad; GPS-only
        -> lands off by the GPS error. (Not RTL: RTL targets the GPS-biased home, not the pad.)"""
        h = box / 2.0
        corners = [(h, h), (h, -h), (-h, -h), (-h, h)]  # NE, SE, SW, NW (N,E)
        items, seq = [], 0
        items.append(self._item(seq, CMD_TAKEOFF, lat_pad, lon_pad, alt, current=1)); seq += 1
        for _ in range(laps):
            for dn, de in corners:
                la, lo = offset_ll(lat_pad, lon_pad, dn, de)
                items.append(self._item(seq, CMD_WAYPOINT, la, lo, alt)); seq += 1
        # Low approach waypoint over the pad: descend into UWB range and let UWB + the full-authority
        # position controller centre on the pad (small NAV_ACC_RAD) BEFORE the short final descent,
        # so the drone doesn't touch down while still flying back to (0,0).
        items.append(self._item(seq, CMD_WAYPOINT, lat_pad, lon_pad, approach_alt)); seq += 1
        items.append(self._item(seq, CMD_LAND, lat_pad, lon_pad, 0.0)); seq += 1      # land on pad
        return items

    # ---- arm / mode ---------------------------------------------------------
    def arm(self, timeout=30):
        end = time.time() + timeout
        while time.time() < end:
            self.m.mav.command_long_send(
                self.tsys, self.tcomp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0)
            ack = self._wait('COMMAND_ACK', 2,
                             match=lambda x: x.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                log("armed")
                return True
            time.sleep(1)
        return False

    def start_mission(self):
        custom = (1 << 0)  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED via DO_SET_MODE
        self.m.mav.command_long_send(
            self.tsys, self.tcomp, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            PX4_MAIN_AUTO, PX4_SUB_AUTO_MISSION, 0, 0, 0, 0)
        time.sleep(1)
        self.m.mav.command_long_send(
            self.tsys, self.tcomp, mavutil.mavlink.MAV_CMD_MISSION_START, 0,
            0, 0, 0, 0, 0, 0, 0)
        log("mission started (AUTO.MISSION)")

    def wait_landed(self, timeout=600):
        """Wait until the vehicle has flown and then landed; return EKF landing x,y."""
        end = time.time() + timeout
        been_airborne = False
        last_xy = (float('nan'), float('nan'))
        last_seq = -1
        last_mode = None
        while time.time() < end:
            msg = self.m.recv_match(
                type=['EXTENDED_SYS_STATE', 'LOCAL_POSITION_NED', 'MISSION_CURRENT', 'HEARTBEAT'],
                blocking=True, timeout=5)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'HEARTBEAT':
                if msg.get_srcComponent() != self.tcomp:
                    continue  # ignore GCS / other components
                nm = mode_name(msg.custom_mode)
                if nm != last_mode:
                    last_mode = nm
                    log(f"mode -> {nm}")
            elif t == 'LOCAL_POSITION_NED':
                last_xy = (msg.x, msg.y)
            elif t == 'MISSION_CURRENT' and msg.seq != last_seq:
                last_seq = msg.seq
                log(f"mission waypoint {msg.seq}")
            elif t == 'EXTENDED_SYS_STATE':
                if msg.landed_state == LANDED_IN_AIR:
                    been_airborne = True
                elif msg.landed_state == LANDED_ON_GROUND and been_airborne:
                    log(f"landed (EKF x={last_xy[0]:.2f} y={last_xy[1]:.2f})")
                    return last_xy
        raise TimeoutError("did not land within timeout")

    def disarm(self):
        self.m.mav.command_long_send(
            self.tsys, self.tcomp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', choices=SCENARIOS, default='A',
                    help='A = GPS-only baseline; B = GPS+UWB (UWB-dominant near the pad)')
    ap.add_argument('--connect', default='udpin:0.0.0.0:14540')
    ap.add_argument('--alt', type=float, default=30.0, help='mission altitude [m]')
    ap.add_argument('--box', type=float, default=50.0, help='box edge [m] (>28 keeps >14 m from pad)')
    ap.add_argument('--laps', type=int, default=2)
    ap.add_argument('--approach-alt', type=float, default=6.0,
                    help='low approach altitude over the pad [m] (within UWB range, centre then land)')
    ap.add_argument('--acc-rad', type=float, default=0.5, help='waypoint acceptance radius [m]')
    ap.add_argument('--out', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'results', 'runs.csv'),
        help='CSV to append run results to (default: Tools/uwb_landing/results/runs.csv)')
    ap.add_argument('--seed', type=int, default=-1, help='SIM_GPS_SEED the sim booted with (for the CSV)')
    ap.add_argument('--timeout', type=float, default=600.0)
    args = ap.parse_args()

    mis = Mission(args.connect)
    for ms in (33, 32, 245, 42):  # GLOBAL_POSITION_INT, LOCAL_POSITION_NED, EXTENDED_SYS_STATE, MISSION_CURRENT
        mis.request_stream(ms, 5)

    # EKF aiding params: a redundant param_set still resets GPS aiding -> read-before-set + wait.
    # (EKF2_HGT_REF=baro is set at BOOT by the f450-uwb airframe -- a runtime change doesn't
    # re-reference the height, so it must not be done here.)
    changed = mis.ensure_param('EKF2_GPS_CTRL', 7)
    for k, v in SCENARIOS[args.scenario].items():
        changed = mis.ensure_param(k, v) or changed
    # Tight waypoint acceptance so the drone centres on the pad (UWB) before the final descent.
    mis.ensure_param('NAV_ACC_RAD', args.acc_rad)
    if changed:
        log("EKF param(s) changed -> waiting 15 s for GPS aiding to re-converge")
        time.sleep(15)

    mis.wait_global_pos()
    mis.wait_position_stable()

    # Pad = the take-off point = EKF local origin (local 0,0). Landing at the EKF origin global
    # maps back to local (0,0) regardless of GPS bias (it cancels in the global<->local conversion).
    # With UWB the drone's local position is accurate in the anchor frame, so it returns precisely
    # to where it took off; GPS-only drifts and lands off. (The take-off point need not be the
    # anchor centroid — UWB is accurate anywhere inside the anchor field with >=3 anchors.)
    origin = mis.get_global_origin()
    if origin:
        lat_pad, lon_pad = origin
        log(f"pad = take-off point (EKF local origin) -> {lat_pad:.7f},{lon_pad:.7f}")
    else:
        lat_pad, lon_pad, _ = mis.wait_global_pos()
        log(f"WARN no GPS_GLOBAL_ORIGIN; using current fix as pad {lat_pad:.7f},{lon_pad:.7f}")

    items = mis.build(lat_pad, lon_pad, args.alt, args.box, args.laps, args.approach_alt)
    if not mis.upload(items):
        log("ERROR mission upload failed"); sys.exit(1)
    if not mis.arm():
        log("ERROR could not arm (pre-flight checks?)"); sys.exit(1)
    mis.start_mission()

    t_start = time.time()
    try:
        x, y = mis.wait_landed(args.timeout)
    except TimeoutError as e:
        log(f"ERROR {e}"); sys.exit(1)
    dur = time.time() - t_start
    mis.disarm()

    row = {'scenario': args.scenario, 'seed': args.seed,
           'ekf_land_x': round(x, 3), 'ekf_land_y': round(y, 3),
           'flight_s': round(dur, 1), 'wall_time': int(time.time())}
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    new = not os.path.exists(args.out)
    with open(args.out, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(row))
        if new:
            w.writeheader()
        w.writerow(row)
    log(f"appended run to {args.out}: {row}")
    log("NOTE true landing error (vs pad ground-truth) -> analyze.py on the ulog")


if __name__ == '__main__':
    main()
