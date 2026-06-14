#!/usr/bin/env python3
"""Analyze UWB precision-landing runs from PX4 ulogs.

For each ulog: detect touchdown (last arm->disarm), compute the TRUE landing error =
horizontal distance of the ground-truth landing point from the pad (ground-truth origin,
where the UWB anchors are), label the scenario from the EKF params active at flight time,
and collect time series. Across runs it reports per-scenario landing error / CEP / RMSE and
draws the §6 plots (touchdown scatter + CEP circles, position error vs time, trajectory).

Usage:
  python3 analyze.py <run1.ulg> [run2.ulg ...]
  python3 analyze.py build/px4_sitl_default/rootfs/log/2026-06-14/*.ulg --out plots.png
"""
import argparse
import os
import sys

import numpy as np
from pyulog import ULog

import matplotlib
# Show an interactive window by default (use the window's Save button to pick where to save);
# fall back to headless Agg only when there's no display or --no-show is passed.
if '--no-show' in sys.argv or not (os.environ.get('DISPLAY') or os.name == 'nt'):
    matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCEN_COLOR = {'A': 'tab:red', 'B': 'tab:green', '?': 'gray'}
SCEN_LABEL = {'A': 'A: GPS-only', 'B': 'B: GPS+UWB', '?': 'unknown'}


def topic(u, name):
    for m in u.data_list:
        if m.name == name:
            return m.data
    return None


def quats_to_euler(w, x, y, z):
    """Body→NED quaternion arrays -> roll, pitch, yaw in degrees."""
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def param_at(u, name, t_abs):
    """Param value active at absolute time t_abs (handles runtime param changes)."""
    val = u.initial_parameters.get(name)
    for ts, n, v in getattr(u, 'changed_parameters', []):
        if n == name and ts <= t_abs:
            val = v
    return val


def scenario_of(u, t_abs):
    uwb = param_at(u, 'EKF2_UWB_CTRL', t_abs)
    return 'A' if uwb in (None, 0) else 'B'


def analyze_one(path):
    u = ULog(path, ['vehicle_status', 'vehicle_local_position',
                    'vehicle_local_position_groundtruth', 'vehicle_attitude'])
    vs = topic(u, 'vehicle_status')
    lp = topic(u, 'vehicle_local_position')
    gt = topic(u, 'vehicle_local_position_groundtruth')
    att = topic(u, 'vehicle_attitude')
    if not (vs and lp and gt):
        print(f"  skip {path}: missing topics")
        return None

    t = np.asarray(vs['timestamp'], float)
    arm = np.asarray(vs['arming_state'])
    dis = np.where((arm[:-1] == 2) & (arm[1:] == 1))[0]
    if len(dis) == 0:
        print(f"  skip {path}: no disarm (did it fly?)")
        return None
    t_land = t[dis[-1]]                       # touchdown ~ last disarm (abs µs)

    tg = np.asarray(gt['timestamp'], float)
    gx = np.interp(t_land, tg, np.asarray(gt['x'], float))
    gy = np.interp(t_land, tg, np.asarray(gt['y'], float))
    tl = np.asarray(lp['timestamp'], float)
    ex = np.interp(t_land, tl, np.asarray(lp['x'], float))
    ey = np.interp(t_land, tl, np.asarray(lp['y'], float))

    scen = scenario_of(u, t_land)
    # series for plots (whole flight), all on the local-position time base tl
    t0 = tl[0]
    series = dict(
        t=(tl - t0) * 1e-6,
        ex=np.asarray(lp['x'], float), ey=np.asarray(lp['y'], float),
        ez=np.asarray(lp['z'], float),
        gx=np.interp(tl, tg, np.asarray(gt['x'], float)),
        gy=np.interp(tl, tg, np.asarray(gt['y'], float)),
        gz=np.interp(tl, tg, np.asarray(gt['z'], float)),
        vx=np.asarray(lp['vx'], float), vy=np.asarray(lp['vy'], float),
        vz=np.asarray(lp['vz'], float))
    if att and 'q[0]' in att:
        ta = np.asarray(att['timestamp'], float)
        roll, pitch, yaw = quats_to_euler(*(np.asarray(att[f'q[{i}]'], float) for i in range(4)))
        series['roll'] = np.interp(tl, ta, roll)
        series['pitch'] = np.interp(tl, ta, pitch)
        series['yaw'] = np.interp(tl, ta, yaw)
    return dict(path=path, scen=scen,
                true_xy=(float(gx), float(gy)), true_err=float(np.hypot(gx, gy)),
                ekf_xy=(float(ex), float(ey)),
                gps_bias=float(np.hypot(ex - gx, ey - gy)), series=series)


def stats(radii):
    r = np.asarray(radii, float)
    return dict(n=len(r), mean=r.mean(), cep=np.median(r),
                rmse=np.sqrt((r ** 2).mean()), worst=r.max())


def plots(runs, out, show):
    fig = plt.figure(figsize=(18, 13))

    # ---- row 1: summary across all runs ----
    a = fig.add_subplot(3, 3, 1)
    a.plot(0, 0, 'k*', ms=16, label='pad (anchors)')
    for scen in ['A', 'B']:
        pts = [r['true_xy'] for r in runs if r['scen'] == scen]
        if not pts:
            continue
        xs, ys = zip(*pts)
        a.scatter(ys, xs, c=SCEN_COLOR[scen], label=SCEN_LABEL[scen], alpha=0.7, zorder=3)
        cep = np.median([np.hypot(x, y) for x, y in pts])
        a.add_patch(plt.Circle((0, 0), cep, color=SCEN_COLOR[scen], fill=False, ls='--', alpha=0.6))
    a.set_aspect('equal'); a.grid(alpha=0.3); a.legend(fontsize=8)
    a.set_xlabel('East [m]'); a.set_ylabel('North [m]'); a.set_title('Touchdown scatter + CEP')

    a = fig.add_subplot(3, 3, 2)
    done = set()
    for r in runs:
        if r['scen'] in done:
            continue
        done.add(r['scen'])
        s = r['series']
        err = np.hypot(s['ex'] - s['gx'], s['ey'] - s['gy'])
        a.plot(s['t'], err, color=SCEN_COLOR[r['scen']], label=SCEN_LABEL[r['scen']])
    a.grid(alpha=0.3); a.legend(fontsize=8)
    a.set_xlabel('time [s]'); a.set_ylabel('EKF − truth horiz err [m]')
    a.set_title('Position error vs time')

    a = fig.add_subplot(3, 3, 3)
    s = runs[0]['series']
    a.plot(s['gy'], s['gx'], 'b-', lw=0.8, label='ground-truth')
    a.plot(s['ey'], s['ex'], 'r-', lw=0.8, alpha=0.6, label='EKF')
    a.plot(0, 0, 'k*', ms=14)
    a.set_aspect('equal'); a.grid(alpha=0.3); a.legend(fontsize=8)
    a.set_xlabel('East [m]'); a.set_ylabel('North [m]'); a.set_title('Trajectory 2D (top-down)')

    # ---- rows 2-3: detail for the representative (first) run ----
    s = runs[0]['series']
    up, gup = -s['ez'], -s['gz']

    a = fig.add_subplot(3, 3, 4, projection='3d')
    a.plot(s['gy'], s['gx'], gup, 'b-', lw=0.7, label='truth')
    a.plot(s['ey'], s['ex'], up, 'r-', lw=0.7, alpha=0.6, label='EKF')
    a.scatter([0], [0], [0], c='k', marker='*', s=80)
    a.set_xlabel('E [m]'); a.set_ylabel('N [m]'); a.set_zlabel('Up [m]')
    a.set_title('3D trajectory'); a.legend(fontsize=7)

    a = fig.add_subplot(3, 3, 5)
    a.plot(s['t'], s['ex'], label='N (x)')
    a.plot(s['t'], s['ey'], label='E (y)')
    a.plot(s['t'], up, label='Up (−z)')
    a.grid(alpha=0.3); a.legend(fontsize=8)
    a.set_xlabel('time [s]'); a.set_ylabel('[m]'); a.set_title('Position X/Y/Z vs time')

    a = fig.add_subplot(3, 3, 6)
    if 'roll' in s:
        a.plot(s['t'], s['roll'], label='roll')
        a.plot(s['t'], s['pitch'], label='pitch')
        a.plot(s['t'], s['yaw'], label='yaw')
        a.legend(fontsize=8)
    a.grid(alpha=0.3)
    a.set_xlabel('time [s]'); a.set_ylabel('[deg]'); a.set_title('Roll/Pitch/Yaw vs time')

    a = fig.add_subplot(3, 3, 7)
    a.plot(s['t'], s['vx'], label='Vx (N)')
    a.plot(s['t'], s['vy'], label='Vy (E)')
    a.plot(s['t'], s['vz'], label='Vz (D)')
    a.grid(alpha=0.3); a.legend(fontsize=8)
    a.set_xlabel('time [s]'); a.set_ylabel('[m/s]'); a.set_title('Velocity Vx/Vy/Vz vs time')

    fig.suptitle(f"UWB precision-landing — detail run: [{runs[0]['scen']}] "
                 f"{runs[0]['path'].split('/')[-1]}", fontsize=12)
    fig.tight_layout()
    if out:
        fig.savefig(out, dpi=120)
        print(f"\nsaved plots -> {out}")
    if show:
        print("\nshowing plot window — use its toolbar Save button to choose where to save")
        plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ulogs', nargs='+')
    ap.add_argument('--out', default=None,
                    help='also save the figure to this path (default: only show the window)')
    ap.add_argument('--no-show', action='store_true', help='headless: do not open a window')
    args = ap.parse_args()

    runs = []
    print("per-run landing error (true = ground-truth vs pad):")
    for p in args.ulogs:
        try:
            r = analyze_one(p)
        except Exception as e:
            print(f"  skip {p}: {e}")
            continue
        if r:
            runs.append(r)
            print(f"  [{r['scen']}] {p.split('/')[-1]}: true={r['true_err']:.2f} m "
                  f"at ({r['true_xy'][0]:.2f},{r['true_xy'][1]:.2f})  gps_bias={r['gps_bias']:.2f} m")
    if not runs:
        print("no valid runs"); sys.exit(1)

    print("\nper-scenario summary (true landing error):")
    print(f"  {'scen':<4} {'n':>3} {'mean':>6} {'CEP50':>6} {'RMSE':>6} {'worst':>6}  [m]")
    for scen in ['A', 'B', '?']:
        radii = [r['true_err'] for r in runs if r['scen'] == scen]
        if not radii:
            continue
        s = stats(radii)
        print(f"  {scen:<4} {s['n']:>3} {s['mean']:>6.2f} {s['cep']:>6.2f} "
              f"{s['rmse']:>6.2f} {s['worst']:>6.2f}")

    plots(runs, args.out, show=not args.no_show)


if __name__ == '__main__':
    main()
