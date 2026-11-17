#!/usr/bin/env python3
"""The failed runs of a batch (tools/batch.py OUT_DIR), one line each: phase, race, how far along its route, why.

    sim/.venv/bin/python sim/tools/failures.py OUT_DIR
"""
import glob
import os
import re
import sys

out = sys.argv[1]
rows = []
for log in sorted(glob.glob(os.path.join(out, '*.log'))):
    text = open(log, errors='replace').read()
    name = os.path.basename(log)[:-4]
    race = 'search'
    for line in text.splitlines():
        m = re.search(r'START mode (\S+)', line)
        if m:
            race = m.group(1)
        m = re.search(r'route (\d+) cells, (\d+) curves: end=(\w+) dist=([\d.]+)/([\d.]+)mm v=(\d+)/(\d+)', line)
        if m and m.group(3) not in ('ENC', 'IR'):
            last = m
        m2 = re.search(r'!! (FAST|RETURN|GOAL|OPTIM|SEARCH)?:? ?(\w+) at \((\d+),(\d+)\)(\w)', line)
        if m2 and 'Position lost' in line:
            frac = float(last.group(4)) / float(last.group(5)) if last else float('nan')
            rows.append((name, race, m2.group(1) or '?', last.group(3) if last else '?', f'{frac:.2f}',
                         f'({m2.group(3)},{m2.group(4)}){m2.group(5)}', last.group(6) + '/' + last.group(7) if last else ''))
            last = None
        if 'target unreachable at' in line:
            rows.append((name, race, 'SEARCH', 'UNREACHABLE', '', '', ''))
for r in rows:
    print('  '.join(f'{c:<26}' if i == 0 else f'{c:<8}' for i, c in enumerate(r)))
from collections import Counter
print(Counter((r[1], r[2]) for r in rows))
