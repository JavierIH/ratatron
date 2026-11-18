"""ratatron: the micromouse's tools.

    ratatron connect            hold the robot's link, record it, share it (--sim: the simulator, ratasim)
    ratatron send CMD ...       commands through the shared link, and the answers
    ratatron monitor            the live maze monitor and console
    ratatron calib CSV ...      reports from calibration recordings
    ratatron cycles ELF         the planner's cost on an emulated Cortex-M3
    ratatron dashboard          the diag_test firmware's panel
    ratatron flash ENV          build and flash the robot (competition, practice, dev, virtual)
    ratatron test               every test: firmware on the host, tools

`ratatron COMMAND -h` for each one's options. Reference: docs/tools.md.
The simulator (the robot in MuJoCo with its firmware) is ratasim: https://github.com/JavierIH/ratasim
"""
import os
import runpy
import shutil
import subprocess
import sys

from . import paths

COMMANDS = {
    'connect': 'ratatron.link.hub',
    'send': 'ratatron.link.send',
    'monitor': 'ratatron.monitor.monitor',
    'calib': 'ratatron.calib.analyze',
    'cycles': 'ratatron.calib.plan_cycles',
    'dashboard': 'ratatron.dashboard',
}
ENVS = ('competition', 'practice', 'dev', 'virtual')


def run_module(module, name, args):
    sys.argv = [name] + args
    runpy.run_module(module, run_name='__main__')     # sys.argv[0] stays the command's name (argparse prog)


def pio():
    found = shutil.which('pio') or os.path.expanduser('~/.platformio/penv/bin/pio')
    if not os.path.exists(found):
        sys.exit('!! PlatformIO not found (pip install platformio)')
    return found


def flash(args):
    if len(args) != 1 or args[0] not in ENVS:
        sys.exit('usage: ratatron flash {%s}   (the robot keeps its map unless the goal or the record layout '
                 'changes: AGENTS.md)' % ','.join(ENVS))
    return subprocess.call([pio(), 'run', '-d', paths.FIRMWARE, '-e', args[0], '-t', 'upload'])


def test(args):
    host = os.path.join(paths.FIRMWARE, 'test', 'host')
    steps = [['make', '-s', '-C', host], ['make', '-s', '-C', host, 'small-queue'],
             [sys.executable, '-m', 'unittest', 'discover', '-s', os.path.join(paths.SOFTWARE, 'tests')]]
    for s in steps:
        print('==', ' '.join(os.path.relpath(x) if os.path.isabs(x) else x for x in s), flush=True)
        if subprocess.call(s, cwd=paths.SOFTWARE):
            return 1
    return 0


def main():
    argv = sys.argv[1:]
    if not argv or argv[0] in ('-h', '--help', 'help'):
        print(__doc__.strip())
        return 0
    cmd, args = argv[0], argv[1:]
    if cmd == 'flash':
        return flash(args)
    if cmd == 'test':
        return test(args)
    if cmd == 'sim':
        sys.exit('!! the simulator is now ratasim: https://github.com/JavierIH/ratasim')
    if cmd not in COMMANDS:
        sys.exit("!! no command %r: `ratatron help`" % cmd)
    return run_module(COMMANDS[cmd], 'ratatron ' + cmd, args)


if __name__ == '__main__':
    sys.exit(main())
