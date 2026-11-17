#!/usr/bin/env python3
"""How much the IR readings taken during a curve tell about the robot's position (step 1: the simulator's own
ideal sensors; step 2 would be the real robot's CAL RUN recordings).

A race runs; for every curve the readings each sensor gave (its 16 ms samples, noise included, only those within
the Sharp's 150 mm) and the true poses at those instants are kept. Then, curve by curve, the true trajectory is
moved by a known error (along the corridor, across it, heading) and a least-squares fit of the predicted readings
(the maze's walls and posts ray-cast as the simulator does) against the measured ones tries to find it again.
The spread of what it finds is how well the curve's readings fix the position.

    sim/.venv/bin/python sim/tools/curve_ir_info.py MAZE FLASH RACE [--trials 20]
"""
import argparse
import math
import os
import random
import re
import shutil
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ratsim import config as C  # noqa: E402
from ratsim.firmware import build  # noqa: E402
from ratsim.maze import Maze  # noqa: E402
from ratsim.robot import IR_FAR_MM, IR_PERIOD_MS, IR_SPREAD_DEG, Robot  # noqa: E402

RANGE_MM = 150.0        # GP2Y0A51SK0F: 2-15 cm


def wall_segments(maze):
    """The faces of every wall and post (mm): rows of x0, y0, x1, y1."""
    cell, t, sw, sd = C.CELL_MM, C.WALL_MM, C.SLOT_W_MM, C.SLOT_D_MM
    boxes = []
    for px in range(maze.size + 1):
        for py in range(maze.size + 1):
            boxes.append((px * cell, py * cell, t, t))      # the slots are ignored (3 mm, a few readings)
    for x, y, kind in maze.segments():
        if kind == 'h':
            boxes.append(((x + 0.5) * cell, y * cell, cell - t, t))
        else:
            boxes.append((x * cell, (y + 0.5) * cell, t, cell - t))
    segs = []
    for cx, cy, sx, sy in boxes:
        x0, x1, y0, y1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
        segs += [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]
    return np.array(segs, dtype=float)


def cast(segs, ox, oy, dx, dy):
    """Distance along unit rays (arrays) to the nearest segment, IR_FAR_MM if none."""
    sx, sy = segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1]
    qx, qy = segs[:, 0][None, :] - ox[:, None], segs[:, 1][None, :] - oy[:, None]
    den = dx[:, None] * sy[None, :] - dy[:, None] * sx[None, :]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (qx * sy[None, :] - qy * sx[None, :]) / den
        u = (qx * dy[:, None] - qy * dx[:, None]) / den
    ok = (np.abs(den) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1)
    t = np.where(ok, t, np.inf)
    d = t.min(axis=1)
    return np.where(np.isfinite(d), d, IR_FAR_MM)


def predict(segs, poses, sensors, offsets):
    """Readings (mm) of sensors[i] at poses[i] = (x, y, yaw deg; yaw 0 north, + left), as the simulator reads."""
    x, y, yaw = poses[:, 0], poses[:, 1], np.radians(poses[:, 2])
    inv = np.zeros(len(poses))
    for spread in (-IR_SPREAD_DEG, 0.0, IR_SPREAD_DEG):
        ox, oy, dx, dy = [], [], [], []
        for i, name in enumerate(sensors):
            (_, _, (sx, sy, _), deg) = C.IR_SENSORS[name]
            c, s = math.cos(yaw[i]), math.sin(yaw[i])
            # robot frame: x right, y forward; world: rotate by yaw (0 = north)
            ox.append(x[i] + sx * c - sy * s)
            oy.append(y[i] + sx * s + sy * c)
            a = math.radians(deg + spread)
            bx, by = math.cos(a), math.sin(a)
            dx.append(bx * c - by * s)
            dy.append(bx * s + by * c)
        d = cast(segs, np.array(ox), np.array(oy), np.array(dx), np.array(dy))
        d = np.minimum(d + np.array([offsets[n] for n in sensors]), IR_FAR_MM)
        inv += 1.0 / np.maximum(d, 1.0)
    return 3.0 / inv


def shifted(poses, along, across, dyaw, heading_in):
    """The trajectory moved by an error in the curve's entry frame (heading_in: yaw of the corridor in)."""
    h = math.radians(heading_in)
    fx, fy = -math.sin(h), math.cos(h)      # forward
    rx, ry = math.cos(h), math.sin(h)       # right
    out = poses.copy()
    out[:, 0] += along * fx + across * rx
    out[:, 1] += along * fy + across * ry
    out[:, 2] += dyaw
    return out


def fit(segs, poses, sensors, offsets, measured, heading_in):
    """The error (along, across, dyaw) that best explains the readings: Gauss-Newton from zero."""
    p = np.zeros(3)
    steps = np.array([1.0, 1.0, 0.2])
    for _ in range(12):
        base = predict(segs, shifted(poses, *p, heading_in), sensors, offsets)
        J = np.zeros((len(base), 3))
        for k in range(3):
            q = p.copy()
            q[k] += steps[k]
            J[:, k] = (predict(segs, shifted(poses, *q, heading_in), sensors, offsets) - base) / steps[k]
        res = measured - base
        # readings that cannot be compared (out of range now and predicted far) weigh nothing
        w = ((measured < RANGE_MM) | (base < RANGE_MM)).astype(float)
        A = J.T @ (J * w[:, None]) + np.diag([1e-3, 1e-3, 1e-2])
        dp = np.linalg.solve(A, J.T @ (res * w))
        p += np.clip(dp, -15, 15)
        if np.abs(dp).max() < 0.05:
            break
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('maze')
    ap.add_argument('flash')
    ap.add_argument('race')
    ap.add_argument('--trials', type=int, default=20)
    a = ap.parse_args()
    maze = Maze.load(a.maze)
    tmp = tempfile.NamedTemporaryFile(suffix='.bin', delete=False).name
    shutil.copy(a.flash, tmp)
    r = Robot(maze, build(), tmp)
    for _ in range(300):
        r.step()
    r.fw.send(f'MODE 2 {a.race}')
    for _ in range(100):
        r.step()
    r.fw.send('START')
    log, samples, prev_yaw = '', [], None
    while r.fw.time_ms() < 120000:
        t = r.fw.time_ms()
        held_before = dict(r.ir_held)
        r.step()
        log += r.fw.received() if t % 50 == 0 else ''
        if 'Goal reached in the speed run' in log or 'End:' in log or r.crashed():
            break
        x, y, yaw = r.pose()
        rate = 0.0 if prev_yaw is None else ((yaw - prev_yaw + 180) % 360 - 180) * 1000
        prev_yaw = yaw
        for name in C.IR_SENSORS:
            if r.ir_held[name] != held_before[name]:            # a new 16 ms sample of this sensor
                samples.append((t, name, r.ir_held[name], x, y, yaw, rate))
    os.unlink(tmp)
    # curves: runs of samples with |yaw rate| > 60 deg/s
    curves, cur = [], []
    for s in samples:
        if abs(s[6]) > 60:
            cur.append(s)
        elif cur:
            if len(cur) > 8:
                curves.append(cur)
            cur = []
    segs = wall_segments(maze)
    rng = random.Random(1)
    print(f'{len(curves)} curves, {sum(len(c) for c in curves) / max(len(curves), 1):.0f} readings each '
          f'({sum(1 for c in curves for s in c if s[2] < RANGE_MM) / max(sum(len(c) for c in curves), 1) * 100:.0f} % '
          f'within {RANGE_MM:.0f} mm)')
    errs = []
    for c in curves:
        poses = np.array([[s[3], s[4], s[5]] for s in c])
        sensors = [s[1] for s in c]
        measured = np.array([s[2] for s in c])
        heading_in = round(c[0][5] / 90) * 90
        # the true trajectory explains the readings: fitting it must give ~0 (the model's own bias)
        bias = fit(segs, poses, sensors, r.ir_offset, measured, heading_in)
        for _ in range(a.trials):
            e = np.array([rng.uniform(-25, 25), rng.uniform(-12, 12), rng.uniform(-3, 3)])
            believed = shifted(poses, *(-e), heading_in)         # the robot believes it is e behind the truth
            found = fit(segs, believed, sensors, r.ir_offset, measured, heading_in)
            errs.append((found - e, bias))
    if not errs:
        print('no curves')
        return
    miss = np.array([x[0] for x in errs])
    bias = np.array([x[1] for x in errs])
    for k, name, unit in ((0, 'along', 'mm'), (1, 'across', 'mm'), (2, 'heading', 'deg')):
        print(f'{name:8s}: error left after the fit  rms {np.sqrt((miss[:, k] ** 2).mean()):5.1f} {unit}, '
              f'90 % within {np.percentile(np.abs(miss[:, k]), 90):5.1f}   (the true path fits at '
              f'{np.abs(bias[:, k]).mean():4.1f})')


if __name__ == '__main__':
    main()
