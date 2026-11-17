#!/usr/bin/env python3
"""Every curve of a speed run measured against the cell geometry: lateral offset and heading error where the curve
should begin (85.5 mm before the corner cell's centre) and where it should end (85.5 mm after), and the speed.

    ratatron sim curves MAZE FLASH RACE [--lib LIB] [--csv OUT]

FLASH: a flash image with the maze's map (a search's); RACE: the race menu entry (4, 5, 6).
The route is taken from the firmware's own line; only the run to the goal is measured.
"""
import argparse
import csv
import math
import os
import re
import shutil
import sys
import tempfile

from .firmware import build
from .maze import Maze
from .robot import Robot

CELL, LEG = 180.0, 85.5         # the curve's legs along each axis (CURVE_RADIUS_MM 70 + ramps, robot_config.h)
D = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # N E S W
YAW = [0.0, -90.0, 180.0, 90.0]         # robot.pose() yaw of each heading


def corners(route, start=(0, 0), heading=0):
    """The corner cells of a route text ("7R3L4..."): (cell, heading in, heading out)."""
    x, y = start
    h = heading
    out = []
    for n, t in re.findall(r'(\d+)([RL]?)', route):
        for _ in range(int(n)):
            x, y = x + D[h][0], y + D[h][1]
        if t:
            h2 = (h + (1 if t == 'R' else -1)) % 4
            out.append(((x, y), h, h2))
            h = h2
    return out


def corners_by_maze(maze, known):
    """The known corners, then those of the maze's shortest path from the last one to the centre goal."""
    from collections import deque
    goal = {(7, 7), (7, 8), (8, 7), (8, 8)}
    prev = {(0, 0): None}
    q = deque([(0, 0)])
    bits = [1, 2, 4, 8]
    while q:
        c = q.popleft()
        if c in goal:
            break
        for h, (dx, dy) in enumerate(D):
            n = (c[0] + dx, c[1] + dy)
            if not maze.has(c[0], c[1], bits[h]) and n not in prev:
                prev[n] = c
                q.append(n)
    path = [c]
    while prev[path[-1]] is not None:
        path.append(prev[path[-1]])
    path.reverse()
    out = []
    for i in range(1, len(path) - 1):
        h0 = D.index((path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]))
        h1 = D.index((path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]))
        if h0 != h1:
            out.append((path[i], h0, h1))
    assert out[:len(known)] == known, 'the route is not the maze\'s shortest path'
    return out


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('maze')
    p.add_argument('flash')
    p.add_argument('race')
    p.add_argument('--lib', help='firmware library (default: the simulator one, built if needed)')
    p.add_argument('--csv')
    a = p.parse_args()
    tmp = tempfile.NamedTemporaryFile(suffix='.bin', delete=False).name
    shutil.copy(a.flash, tmp)
    r = Robot(Maze.load(a.maze), a.lib or build(), tmp)
    for _ in range(300):
        r.step()
    r.fw.send(f'MODE 2 {a.race}')
    for _ in range(100):
        r.step()
    r.fw.send('START')
    log = ''
    while 'route' not in log and r.fw.time_ms() < 20000:
        r.step()
        log += r.fw.received()
    m = re.search(r'FAST \(0,0\)N turn 0 \+ route (\S+)', log)
    # The log line cuts long routes ("+"): the rest comes from following the map is not needed, the robot's
    # own run tells; measure the corners we know, and stop at the first touch or the goal.
    cs = corners(m.group(1).rstrip('+'))
    if m.group(1).endswith('+'):        # the log cuts long routes: the rest by the maze's shortest path (unique paths)
        cs = corners_by_maze(r.maze, cs)
    rows, i, phase, prev = [], 0, 'entry', None
    entry = None
    while i < len(cs) and r.fw.time_ms() < 600000:
        r.step()
        log += r.fw.received()
        if 'Goal reached in the speed run' in log or 'End:' in log:
            break
        (cx, cy), h0, h1 = cs[i]
        x, y, yaw = r.pose()
        ccx, ccy = (cx + 0.5) * CELL, (cy + 0.5) * CELL
        vx, vy = r.data.qvel[r.free_dof:r.free_dof + 2] * 1000
        hy = math.radians(yaw)
        v = -vx * math.sin(hy) + vy * math.cos(hy)        # forward speed (the sideways slip left out)
        slip = vx * math.cos(hy) + vy * math.sin(hy)      # sideways, > 0 to the right
        slip_max = max(globals().get('slip_max', 0.0), abs(slip)) if phase == 'exit' else 0.0
        globals()['slip_max'] = slip_max
        if phase == 'entry':
            d = D[h0]
            along = (x - ccx) * d[0] + (y - ccy) * d[1]           # < 0 before the centre
            if along >= -LEG:
                lat = (x - ccx) * d[1] - (y - ccy) * d[0]           # > 0 to the right of the line
                entry = (lat, wrap(yaw - YAW[h0]), v)
                phase = 'exit'
        else:
            d = D[h1]
            along = (x - ccx) * d[0] + (y - ccy) * d[1]
            if along >= LEG:
                lat = (x - ccx) * d[1] - (y - ccy) * d[0]
                turn = 'R' if (h1 - h0) % 4 == 1 else 'L'
                rows.append({'n': i, 'cell': f'{cx},{cy}', 'turn': turn, 'gap': '',
                             'v_in': round(entry[2]), 'lat_in': round(entry[0], 1), 'yaw_in': round(entry[1], 2),
                             'v_out': round(v), 'slip': round(slip_max), 'lat_out': round(lat, 1), 'yaw_out': round(wrap(yaw - YAW[h1]), 2)})
                i += 1
                phase = 'entry'
        if r.crashed():
            rows.append({'n': i, 'cell': f'{cx},{cy}', 'turn': 'TOUCH', 'gap': '', 'v_in': round(v)})
            break
    # cells of straight before each curve
    for k, row in enumerate(rows):
        if k < len(cs) and row['turn'] in 'RL':
            pc = cs[k - 1][0] if k else (0, 0)
            row['gap'] = abs(cs[k][0][0] - pc[0]) + abs(cs[k][0][1] - pc[1])
    os.unlink(tmp)
    keys = ['n', 'cell', 'turn', 'gap', 'v_in', 'lat_in', 'yaw_in', 'v_out', 'slip', 'lat_out', 'yaw_out']
    for row in rows:
        print('  '.join(f'{k}={row.get(k, "")}' for k in keys))
    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, keys)
            w.writeheader()
            w.writerows(rows)


if __name__ == '__main__':
    main()
