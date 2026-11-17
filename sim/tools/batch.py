#!/usr/bin/env python3
"""The firmware on many mazes: per maze a search, then races 2.4 and 2.6, in parallel; a summary per maze (CSV)
and the incidents (pose lost, the hull touching a wall, failed runs, "!!" lines) for reading the logs.

    sim/.venv/bin/python sim/tools/batch.py OUT_DIR maze.txt ... [--workers 10] [--races "2 4,2 6"]
"""
import argparse
import csv
import os
import re
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PY = os.path.join(SIM, '.venv', 'bin', 'python')


def run_one(maze, out, races, max_s, maps=None):
    name = os.path.splitext(os.path.basename(maze))[0]
    log = os.path.join(out, name + '.log')
    flash = os.path.join(out, name + '.bin')
    cmd = [PY, os.path.join(SIM, 'run.py'), '--headless', '--speed', '0', '--no-pty', '--flash', flash,
           '--maze', maze, '--timestamps', '--max-s', str(max_s), '--until-idle']
    if maps:        # the map a search saved before: only the races (for changes the search does not use)
        shutil.copy(os.path.join(maps, name + '.bin'), flash)
        steps = []
    else:
        cmd += ['--erase-flash']
        steps = ['MODE 1', 'START']
    for r in races:
        if steps:
            steps.append(':wait-idle')
        steps += [f'MODE {r}', 'START']
    for c in steps:
        cmd += ['--cmd', c]
    t0 = time.monotonic()
    with open(log, 'w') as f:
        p = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=max_s * 2 + 600)
    return summarize(name, open(log).read(), time.monotonic() - t0, p.returncode)


def summarize(name, text, wall, rc):
    def first(pattern, group=1, cast=float):
        m = re.search(pattern, text)
        return cast(m.group(group)) if m else ''
    ends = re.findall(r'End: (\w+)', text)
    return {
        'maze': name,
        'goal_s': first(r'\[\s*([\d.]+)\] Goal reached at'),
        'goal_actions': first(r'Goal reached at \(\d+,\d+\) after (\d+) actions', cast=int),
        'search_end_s': first(r'\[\s*([\d.]+)\] At the start after'),
        'cells': first(r'Map saved \((\d+) cells visited\)\n(?![\s\S]*Map saved)', cast=int),
        'cost': first(r'Fast path verified: cost (\d+)', cast=int),
        'ends': ' '.join(ends),
        'failed': sum(e != 'OK' for e in ends),
        'runs': len(ends),
        'routes': ' '.join(f'{v}:{e}:{int(t) / 1000:.2f}' for e, v, t in
                           re.findall(r'route \d+ cells, \d+ curves: end=(\w+) .*?v=(\d+/\d+) .*?t=(\d+)ms', text)),
        'pose_lost': text.count('POSE LOST'),
        'touches': text.count('touches a wall'),
        'late': sum(int(n) for n in re.findall(r'decisions on the way: \d+, .*?late (\d+)', text)),
        'bang': len(re.findall(r'^\[[\s\d.]+\] !! (?!decisions on the way)', text, re.M)),
        'halted': int('## firmware' in text),
        'sim_s': first(r'## end: ([\d.]+) s simulated'),
        'wall_s': round(wall),
        'rc': rc,
    }


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('out')
    p.add_argument('mazes', nargs='+')
    p.add_argument('--workers', type=int, default=10)
    p.add_argument('--races', default='2 4,2 6', help='race menu entries after the search, comma separated')
    p.add_argument('--max-s', type=int, default=1500, help='simulated seconds per maze at most')
    p.add_argument('--maps', help='a batch dir whose saved maps (*.bin) replace the search')
    a = p.parse_args()
    os.makedirs(a.out, exist_ok=True)
    subprocess.run(['make', '-s', '-C', os.path.join(SIM, 'fw')], check=True)
    races = [r.strip() for r in a.races.split(',') if r.strip()]
    rows, t0 = [], time.monotonic()
    with ThreadPoolExecutor(a.workers) as ex:
        futures = [ex.submit(run_one, m, a.out, races, a.max_s, a.maps) for m in a.mazes]
        for i, f in enumerate(futures, 1):
            try:
                row = f.result()
            except Exception as e:                  # noqa: BLE001  a crash of the runner is a result too
                row = {'maze': os.path.basename(a.mazes[i - 1]), 'ends': f'runner: {e}', 'failed': 1}
            rows.append(row)
            bad = row.get('failed') or row.get('pose_lost') or row.get('touches') or row.get('bang') or row.get('halted')
            print(f'[{i}/{len(a.mazes)} {time.monotonic() - t0:5.0f}s] {row["maze"]:28s} {row.get("ends", "")}'
                  f'{"  <-- " + str({k: row[k] for k in ("pose_lost", "touches", "bang", "halted") if row.get(k)}) if bad else ""}',
                  flush=True)
    keys = list(rows[0].keys()) if rows else []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(os.path.join(a.out, 'summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows(rows)
    n = len(rows)
    print(f'\n{n} mazes: {sum(1 for r in rows if not r.get("failed") and r.get("runs") == len(races) + 1)} all runs OK, '
          f'{sum(1 for r in rows if r.get("failed"))} with a failed run, {sum(1 for r in rows if r.get("pose_lost"))} '
          f'with a pose lost, {sum(1 for r in rows if r.get("touches"))} touching, '
          f'{sum(1 for r in rows if r.get("bang"))} with "!!", {sum(1 for r in rows if r.get("halted"))} halted')


if __name__ == '__main__':
    sys.exit(main())
