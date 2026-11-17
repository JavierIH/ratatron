#!/usr/bin/env python3
"""The real robot's IR readings in a recorded speed run (CAL RUN, software/calib_data/*_run.csv) against what the
simulator's sensor model predicts there: does the Sharp read the walls in curves as the model says?

The trajectory comes from the encoders (dead reckoning from the start cell's centre, facing north), the readings
from the raw ADC through the firmware's own calibration, the prediction from the practice maze layout
(docs/mazes.md) with the simulator's rays. Residuals (measured - predicted) are reported on straights and in
curves, for the readings within the sensors' range.

    ratatron sim real-ir-check LAYOUT run.csv ...     (LAYOUT: C, D, E, F of docs/mazes.md)
"""
import sys as _sys  # USAGE_GUARD
if len(_sys.argv) < 3 or any(a in ('-h', '--help') for a in _sys.argv[1:]):
    print(__doc__.strip())
    raise SystemExit(0)
import math
import os
import re
import statistics
import sys

import numpy as np

from .curve_ir_info import RANGE_MM, predict, wall_segments
from . import config as C
from .maze import E, N, Maze

LAYOUTS = {     # docs/mazes.md: (east of cells), (north of cells)
    'C': ([(0, 0), (1, 1), (2, 1)], [(0, 1), (2, 1)]),
    'D': ([(0, 0), (0, 1), (1, 1), (2, 1)], [(2, 1)]),
    'E': ([(0, 0), (1, 0), (0, 1), (2, 1), (1, 2)], [(2, 1)]),
    'F': ([(0, 1), (1, 1), (2, 1)], [(1, 0), (1, 1)]),
}
DELAY_MS = {'FL': 22, 'FR': 22, 'SL': 6, 'SR': 6}   # IR_DELAY_MS (fronts), 4-8 ms (sides, docs/faults/centring.md)


def practice_maze(name):
    m = Maze(16)
    for x in range(16):
        for y in range(16):
            m.walls[x][y] = 0
    for x in range(4):                  # the 4x3 border
        m.set(x, 0, 4)
        m.set(x, 2, N)
    for y in range(3):
        m.set(0, y, 8)
        m.set(3, y, E)
    east, north = LAYOUTS[name]
    for x, y in east:
        m.set(x, y, E)
    for x, y in north:
        m.set(x, y, N)
    return m


def sim_offsets():
    """Reading = beam length + offset, as sim/robot.py sets them (the firmware's reference readings)."""
    half = 0.5 * C.FW['LANE_WIDTH_MM']
    front_ref, skew = C.FW['FRONT_WALL_REF_MM'], C.FW['FRONT_SQUARE_OFFSET_MM']
    want = {'SL': C.FW['SIDE_CENTER_L_MM'], 'SR': C.FW['SIDE_CENTER_R_MM'],
            'FL': front_ref + skew / 2, 'FR': front_ref - skew / 2}
    out = {}
    for name, (_, _, (x, y, _), deg) in C.IR_SENSORS.items():
        length = half - y if name in ('FL', 'FR') else (half - abs(x)) / abs(math.cos(math.radians(deg)))
        out[name] = want[name] - length
    return out


def load(path):
    text = open(path).read()
    info = dict(re.findall(r'(\w+)=("[^"]*"|\S+)', ' '.join(l for l in text.splitlines() if l.startswith('#'))))
    cal = {k: [float(v.rstrip('f')) for v in info[f'ir_cal_{k.lower()}'].strip('"').split(',')]
           for k in ('FL', 'FR', 'SL', 'SR')}
    rows = [list(map(float, l.split(','))) for l in text.splitlines() if l[:1].isdigit()]
    return info, cal, np.array(rows)


def analyse(layout, path):
    info, cal, d = load(path)
    tpm = float(info['ticks_per_mm'])
    wd = float(info.get('wheel_diff_ppm', 0)) * 1e-6
    half_track = float(info['turn_ticks']) / tpm / (math.pi / 2)
    t, el, er = d[:, 0], d[:, 1], d[:, 2]
    raw = {'FL': d[:, 5], 'FR': d[:, 6], 'SL': d[:, 7], 'SR': d[:, 8]}
    ref_rot = d[:, 10] / 100.0
    # dead reckoning: x right (east), y north, yaw deg + left (0 = north)
    x, y, yaw = [90.0], [90.0], [0.0]
    for i in range(1, len(t)):
        l = (el[i] - el[i - 1]) / tpm * (1.0 + wd)
        r = (er[i] - er[i - 1]) / tpm
        ds, dth = 0.5 * (l + r), (r - l) / (2.0 * half_track)
        h = math.radians(yaw[-1]) + 0.5 * dth
        x.append(x[-1] - ds * math.sin(h))
        y.append(y[-1] + ds * math.cos(h))
        yaw.append(yaw[-1] + math.degrees(dth))
    x, y, yaw = np.array(x), np.array(y), np.array(yaw)
    rate = np.gradient(ref_rot, t / 1000.0)
    segs = wall_segments(practice_maze(layout))
    offs = sim_offsets()
    out = {}
    for name, coef in cal.items():
        a, b, c, e = coef
        mm = np.clip(((a * raw[name] + b) * raw[name] + c) * raw[name] + e, 0, 400)
        tp = t - DELAY_MS[name]             # the reading reports where the robot was
        poses = np.stack([np.interp(tp, t, x), np.interp(tp, t, y), np.interp(tp, t, yaw)], axis=1)
        pred = predict(segs, poses, [name] * len(t), offs)
        ok = (mm < RANGE_MM) & (pred < RANGE_MM) & (t > 50)
        curve = np.abs(np.interp(tp, t, rate)) > 100
        out[name] = (mm - pred, ok & ~curve, ok & curve)
    return out


def main():
    layout, files = sys.argv[1], sys.argv[2:]
    acc = {n: ([], []) for n in C.IR_SENSORS}
    for f in files:
        res = analyse(layout, f)
        for n, (r, s, c) in res.items():
            acc[n][0].extend(r[s])
            acc[n][1].extend(r[c])
    print(f'{len(files)} runs, layout {layout}: residual = measured - predicted (mm), readings within {RANGE_MM:.0f} mm')
    for n, (s, c) in acc.items():
        def stat(v):
            if len(v) < 5:
                return f'{len(v):4d} readings'
            med = statistics.median(v)
            mad = statistics.median(abs(e - med) for e in v)
            return f'{len(v):4d} readings, median {med:+6.1f}, spread (MAD) {mad:5.1f}, 90 % within {np.percentile(np.abs(v), 90):5.1f}'
        print(f'  {n} straight: {stat(s)}')
        print(f'  {n} curve:    {stat(c)}')


if __name__ == '__main__':
    main()
