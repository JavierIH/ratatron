#!/usr/bin/env python3
"""The firmware's map (from the telemetry of a run.py --log) against the real maze: walls it believes that are not
there (phantoms) and walls it believes open that are there (missed), each with the pose it was first seen from.

    ratatron sim mapdiff maze.txt run.log
"""
import sys as _sys  # USAGE_GUARD
if len(_sys.argv) < 3 or any(a in ('-h', '--help') for a in _sys.argv[1:]):
    print(__doc__.strip())
    raise SystemExit(0)
import os
import sys

from .maze import E, N, S, W, Maze

SIDES = {'N': N, 'E': E, 'S': S, 'W': W}
STEP = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}


def key(x, y, d):
    """One name per wall: (x, y, 'N') or (x, y, 'E') of the cell south / west of it."""
    if d == 'S':
        return (x, y - 1, 'N')
    if d == 'W':
        return (x - 1, y, 'E')
    return (x, y, d)


def main():
    maze = Maze.load(sys.argv[1])
    first_seen = {}             # wall -> (line number, pose, state) of the first '#' / open sighting
    state = {}
    rows = {}
    for i, line in enumerate(open(sys.argv[2], errors='replace'), 1):
        line = line.strip()
        if line.startswith('@C') and len(line) >= 9:
            x, y, h = int(line[2], 16), int(line[3], 16), line[4]
            for d, c in zip('NESW', line[5:9]):
                k = key(x, y, d)
                if c in '#.o' and (k not in first_seen or first_seen[k][2] != c):
                    first_seen.setdefault(k, (i, f'({x},{y}){h}', c))
        elif line.startswith('@R') and len(line) >= 19:
            rows[int(line[2], 16)] = line[3:19]
    for y, cells in rows.items():
        for x, ch in enumerate(cells):
            v = int(ch, 32)
            state[(x, y, 'N')] = v & 3
            state[(x, y, 'E')] = (v >> 2) & 3
    phantom, missed = [], []
    for (x, y, d), v in state.items():
        if not (0 <= x < 16 and 0 <= y < 16):
            continue
        if (d == 'N' and y == 15) or (d == 'E' and x == 15):
            continue
        real = maze.has(x, y, SIDES[d])
        if v == 1 and not real:
            phantom.append((x, y, d))
        elif v in (2, 3) and real:
            missed.append((x, y, d, v))
    print(f'{len(phantom)} phantom walls, {len(missed)} missed walls (map rows: {len(rows)})')
    for x, y, d in sorted(phantom):
        print(f'  phantom {d} of ({x},{y})   first seen from {first_seen.get((x, y, d), "?")}')
    for x, y, d, v in sorted(missed):
        print(f'  missed  {d} of ({x},{y}) {"verified" if v == 3 else "once"}   first seen from '
              f'{first_seen.get((x, y, d), "?")}')


if __name__ == '__main__':
    main()
