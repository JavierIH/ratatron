"""ratatron send: commands to the robot through `ratatron connect`, and what it answers.

    ratatron send STATUS
    ratatron send -w "data saved" -t 25 "CAL TURN 2"      # wait until a line matches, at most 25 s

Prints the robot's text lines after the commands (telemetry '@' lines left out unless --telemetry).
"""
import argparse
import re
import socket
import sys
import time

from .. import paths


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratatron send', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('commands', nargs='+', help='console lines (MODE 1, START, STATUS, ...)')
    p.add_argument('-w', '--wait', metavar='REGEX', help='wait until a line matches')
    p.add_argument('-t', '--timeout', type=float, default=3.0, help='seconds to wait (default 3)')
    p.add_argument('--gap', type=float, default=1.2, help='seconds between commands (default 1.2)')
    p.add_argument('--telemetry', action='store_true', help="show the '@' lines too")
    a = p.parse_args(argv)
    try:
        s = socket.create_connection(paths.HUB_ADDRESS, timeout=2)
    except OSError:
        sys.exit('!! no link: start `ratatron connect` (or `ratatron connect --sim`) first')
    pattern = re.compile(a.wait) if a.wait else None
    buf, done = b'', False

    def show(until):
        nonlocal buf, done
        while time.monotonic() < until and not done:
            s.settimeout(max(0.05, until - time.monotonic()))
            try:
                data = s.recv(4096)
            except socket.timeout:
                break
            if not data:
                sys.exit('!! the link closed')
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('utf-8', 'replace').rstrip('\r')
                if a.telemetry or not text.startswith('@'):
                    print(text, flush=True)
                if pattern and pattern.search(text):
                    done = True
    for i, c in enumerate(a.commands):
        s.sendall((c + '\n').encode())
        show(time.monotonic() + (a.gap if i + 1 < len(a.commands) else 0))
    show(time.monotonic() + a.timeout)
    s.close()
    if pattern and not done:
        sys.exit(1)


if __name__ == "__main__":
    main()
