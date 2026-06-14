#!/usr/bin/env python3
"""Validate the simulated NEO-M9N GPS error model (gz_bridge::addGpsNoise) from a SITL ulog.

Two views:
  RAW  = sensor_gps vs vehicle_global_position_groundtruth  -> the model output directly
         (raw error = full Gauss-Markov bias + white jitter). Best to validate the model.
  EKF  = vehicle_local_position vs ..._groundtruth          -> confirms the EKF accepts/uses it
         (xy_valid, not dead_reckoning). EKF error = bias drift since origin (smoothed).

Checks: realistic M9N magnitude (~0.5-3 m), a SMALL white jitter + SLOW bias (not random jumps),
EKF accepts it, reproducible per seed, seed-sensitive, and ~0 when SIM_GPS_NSC=0.

Usage:
  _gps_noise_check.py realism  <ulg>
  _gps_noise_check.py zero     <ulg>
  _gps_noise_check.py repro    <ulg_a> <ulg_b>
  _gps_noise_check.py seeddiff <ulg_seed1> <ulg_seed2>
Exit 0 = PASS, 1 = FAIL.
"""
import sys
import numpy as np
from pyulog import ULog

WIN_FRAC = 0.45          # analyse the settled tail
DEG2M = np.pi / 180.0 * 6371000.0   # m per degree latitude


def _data(path, topics):
    u = ULog(path, topics)
    return {m.name: m.data for m in u.data_list}


def load_raw(path):
    """Raw GPS error (sensor_gps vs groundtruth global), in local N/E metres."""
    d = _data(path, ['sensor_gps', 'vehicle_global_position_groundtruth'])
    g, gt = d['sensor_gps'], d['vehicle_global_position_groundtruth']
    t = np.asarray(g['timestamp'], float) * 1e-6
    t -= t[0]
    lat = np.asarray(g['latitude_deg'], float)
    lon = np.asarray(g['longitude_deg'], float)
    tg = np.asarray(gt['timestamp'], float) * 1e-6
    tg -= tg[0]
    glat = np.interp(t, tg, np.asarray(gt['lat'], float))
    glon = np.interp(t, tg, np.asarray(gt['lon'], float))
    lat0 = float(np.median(glat))
    en = (lat - glat) * DEG2M
    ee = (lon - glon) * DEG2M * np.cos(np.deg2rad(lat0))
    return dict(t=t, en=en, ee=ee, herr=np.hypot(en, ee))


def load_ekf(path):
    d = _data(path, ['vehicle_local_position', 'vehicle_local_position_groundtruth'])
    lp, gt = d['vehicle_local_position'], d['vehicle_local_position_groundtruth']
    t = np.asarray(lp['timestamp'], float) * 1e-6
    t -= t[0]
    tg = np.asarray(gt['timestamp'], float) * 1e-6
    tg -= tg[0]
    ex = np.asarray(lp['x'], float) - np.interp(t, tg, np.asarray(gt['x'], float))
    ey = np.asarray(lp['y'], float) - np.interp(t, tg, np.asarray(gt['y'], float))
    return dict(t=t, ex=ex, ey=ey, herr=np.hypot(ex, ey),
                xyv=np.asarray(lp['xy_valid']), dr=np.asarray(lp['dead_reckoning']))


def _drift_speed(e):
    """Median speed of the EKF position error, on a 1 s grid (quantifies the slow 'trôi')."""
    j = _tail(e)
    tg = np.arange(e['t'][j], e['t'][-1], 1.0)
    if len(tg) < 4:
        return float('nan')
    ex = np.interp(tg, e['t'], e['ex'])
    ey = np.interp(tg, e['t'], e['ey'])
    return float(np.median(np.hypot(np.diff(ex), np.diff(ey))))  # per 1 s == m/s


def _tail(s):
    return int(len(s['t']) * WIN_FRAC)


def _movavg(a, w):
    w = max(3, w | 1)  # odd
    k = np.ones(w) / w
    return np.convolve(a, k, mode='same')


def _white_est(s):
    """White-noise sigma per axis = std of raw error after removing the slow bias
    (moving average over ~6 s). Rate-independent: separates fix-to-fix white jitter
    from the slow Gauss-Markov drift, so it catches a too-large white term ('jumps')."""
    dt = np.median(np.diff(s['t']))
    w = int(round(6.0 / dt)) if dt > 0 else 7
    i, j = _tail(s), max(4, (int(round(6.0 / dt)) if dt > 0 else 7))
    rn = (s['en'] - _movavg(s['en'], w))[i + j:-j]
    re = (s['ee'] - _movavg(s['ee'], w))[i + j:-j]
    if len(rn) < 5:
        return float('nan')
    return float(np.sqrt(0.5 * (np.var(rn) + np.var(re))))


def _resample(s, tg):
    return np.interp(tg, s['t'], s['en']), np.interp(tg, s['t'], s['ee'])


def p(ok, msg):
    print(f"  [{'PASS' if ok else 'FAIL'}] {msg}")
    return ok


def realism(path):
    r = load_raw(path)
    e = load_ekf(path)
    i = _tail(r)
    med = float(np.median(r['herr'][i:]))
    white = _white_est(r)
    j = _tail(e)
    ekf_p99 = float(np.percentile(np.abs(np.diff(e['herr'][j:])), 99))
    drift = _drift_speed(e)
    accepted = float(np.mean((e['xyv'][j:] == 1) & (e['dr'][j:] == 0)))
    print(f"realism  {path}")
    print(f"  duration={r['t'][-1]:.0f}s  (raw gps n={len(r['t'])} @~{1/np.median(np.diff(r['t'])):.0f}Hz, ekf n={len(e['t'])})")
    print(f"  RAW gps horizontal error: median={med:.2f} m  max={r['herr'][i:].max():.2f} m")
    print(f"  EKF horizontal error (tail): median={float(np.median(e['herr'][j:])):.2f} m")
    print(f"  EKF error drift speed (the 'trôi'): {drift*100:.1f} cm/s")
    print(f"  EKF output fix-to-fix step p99: {ekf_p99*100:.2f} cm  (smooth = no jumps)")
    print(f"  EKF GPS-accepted (xy_valid & !dead_reckoning): {accepted*100:.1f}%")
    print(f"  [info] raw high-freq jitter (bias removed): {white*100:.1f} cm/axis")
    ok = True
    ok &= p(0.4 <= med <= 3.5, f"M9N-realistic GPS error magnitude 0.4-3.5 m (got {med:.2f})")
    ok &= p(accepted > 0.95, f"EKF accepts GPS, not rejected as drift (>95%, got {accepted*100:.1f}%)")
    ok &= p(0.0 < drift < 0.15, f"slow smooth drift, not erratic (<15 cm/s, got {drift*100:.1f})")
    ok &= p(ekf_p99 < 0.10, f"EKF output smooth, no jumps (p99 step <10 cm, got {ekf_p99*100:.2f})")
    return ok


def zero(path):
    r = load_raw(path)
    i = _tail(r)
    med = float(np.median(r['herr'][i:]))
    print(f"zero(SIM_GPS_NSC=0)  {path}\n  RAW gps horizontal error: median={med:.2f} m")
    return p(med < 0.20, f"~0 error with noise off (<20 cm, got {med*100:.1f} cm)")


def _trace_diff(a, b):
    sa, sb = load_raw(a), load_raw(b)
    tg = np.linspace(5, min(sa['t'][-1], sb['t'][-1]), 400)
    ax, ay = _resample(sa, tg)
    bx, by = _resample(sb, tg)
    return np.hypot(ax - bx, ay - by)


def repro(a, b):
    diff = _trace_diff(a, b)
    mad = float(np.mean(diff))
    print(f"repro  {a}\n  vs  {b}\n  mean |trace diff| = {mad:.3f} m (max {diff.max():.3f})")
    return p(mad < 0.35, f"same seed -> near-identical trace (<0.35 m, got {mad:.3f})")


def seeddiff(a, b):
    mad = float(np.mean(_trace_diff(a, b)))
    print(f"seeddiff  {a}\n  vs  {b}\n  mean |trace diff| = {mad:.3f} m")
    return p(mad > 0.30, f"different seed -> different trace (>0.30 m, got {mad:.3f})")


def main():
    mode = sys.argv[1]
    fn = dict(realism=realism, zero=zero, repro=repro, seeddiff=seeddiff)[mode]
    ok = fn(*sys.argv[2:])
    print(f"==> {mode}: {'PASS' if ok else 'FAIL'}\n")
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
