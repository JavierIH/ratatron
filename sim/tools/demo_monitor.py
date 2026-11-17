#!/usr/bin/env python3
"""A video of the simulator and the robot monitor working together, with no screen: the 3D view on the left, the
monitor (software/robot_monitor.py, in a pseudo-terminal, read back by a terminal emulator) on the right. The
commands are typed into the monitor's console: MODE 1, START (the search), then MODE 2 6, START (race 2.6).

    sim/.venv/bin/python sim/tools/demo_monitor.py demo.mp4      (needs pyte, pillow, ffmpeg)
"""
import fcntl
import os
import pty
import struct
import subprocess
import sys
import termios
import time

import mujoco
import pyte
from PIL import Image, ImageDraw, ImageFont

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(SIM)
sys.path.insert(0, SIM)
import run  # noqa: E402

COLS, ROWS, CW, CH = 150, 48, 8, 15         # the monitor's terminal and its character cell in pixels
VIEW = ROWS * CH                            # the 3D view, square
W, H = VIEW + COLS * CW, ROWS * CH
FPS = 30
FONT = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 13)
NAMED = {'black': (0, 0, 0), 'red': (205, 49, 49), 'green': (13, 188, 121), 'brown': (229, 229, 16),
         'blue': (36, 114, 200), 'magenta': (188, 63, 188), 'cyan': (17, 168, 205), 'white': (229, 229, 229)}
FG, BG = (215, 215, 215), (14, 14, 16)


def colour(c, default):
    if c == 'default':
        return default
    if c in NAMED:
        return NAMED[c]
    if c.startswith('bright') and c[6:] in NAMED:
        return tuple(min(255, v + 50) for v in NAMED[c[6:]])
    try:
        return tuple(int(c[i:i + 2], 16) for i in (0, 2, 4))
    except ValueError:
        return default


def block_cells(ch):
    """The filled sub-cells (columns, rows, [(x, y)]) of a block or sextant character, else None."""
    o = ord(ch)
    if 0x1FB00 <= o <= 0x1FB3B:             # sextants: 2 x 3, bit 0 top left ... bit 5 bottom right
        p = o - 0x1FB00 + 1
        p += p >= 21
        p += p >= 42
        return 2, 3, [(i % 2, i // 2) for i in range(6) if p >> i & 1]
    return {'█': (1, 1, [(0, 0)]), '▀': (1, 2, [(0, 0)]), '▄': (1, 2, [(0, 1)]),
            '▌': (2, 1, [(0, 0)]), '▐': (2, 1, [(1, 0)])}.get(ch)


class Terminal:
    def __init__(self):
        self.screen = pyte.Screen(COLS, ROWS)
        self.stream = pyte.ByteStream(self.screen)
        self.image = Image.new('RGB', (COLS * CW, ROWS * CH), BG)
        self.draw = ImageDraw.Draw(self.image)

    def feed(self, data):
        self.stream.feed(data)

    def render(self):
        for y in self.screen.dirty:
            line = self.screen.buffer[y]
            for x in range(COLS):
                c = line[x]
                fg, bg = colour(c.fg, FG), colour(c.bg, BG)
                if c.reverse:
                    fg, bg = bg, fg
                x0, y0 = x * CW, y * CH
                self.draw.rectangle((x0, y0, x0 + CW - 1, y0 + CH - 1), fill=bg)
                if c.data in (' ', ''):
                    continue
                cells = block_cells(c.data)
                if cells:
                    nx, ny, on = cells
                    for cx, cy in on:
                        self.draw.rectangle((x0 + cx * CW // nx, y0 + cy * CH // ny,
                                             x0 + (cx + 1) * CW // nx - 1, y0 + (cy + 1) * CH // ny - 1), fill=fg)
                else:
                    self.draw.text((x0, y0), c.data, font=FONT, fill=fg)
        self.screen.dirty.clear()
        return self.image


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(SIM, 'build', 'demo_monitor.mp4')
    args = run.make_parser().parse_args(['--headless', '--erase-flash'])
    s = run.Session(args)
    run.open_port(s, args.port)
    r = s.robot

    pid, fd = pty.fork()
    if pid == 0:
        os.chdir(ROOT)
        os.environ.update(TERM='xterm-256color', COLORTERM='truecolor')
        os.execvp('python3', ['python3', 'software/robot_monitor.py', '--port', os.path.relpath(args.port, ROOT),
                                         '--no-record', '--full'])
    fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack('HHHH', ROWS, COLS, 0, 0))
    os.set_blocking(fd, False)
    term = Terminal()

    view = mujoco.Renderer(r.model, VIEW, VIEW)
    top = mujoco.Renderer(r.model, 220, 220)
    cam, tcam = mujoco.MjvCamera(), mujoco.MjvCamera()
    side = s.maze.size * 0.18
    tcam.lookat[:] = (side / 2, side / 2, 0)
    tcam.distance, tcam.elevation, tcam.azimuth = side * 1.75, -90, 90
    cam.distance, cam.elevation, cam.azimuth = 0.7, -52, 90
    cam.lookat[:] = (0.09, 0.09, 0)
    ff = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
                           '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '24', out],
                          stdin=subprocess.PIPE)

    keys = []                   # (sim ms, bytes) typed into the monitor
    def type_text(at_ms, text):
        for i, ch in enumerate(text):
            keys.append((at_ms + 70 * i, ch.encode()))
    type_text(3000, 'MODE 1\r')
    type_text(5000, 'START\r')
    ends, last_status, finish_at, next_frame = 0, '', None, 0
    wall0 = time.monotonic()
    while True:
        s.step()
        t = r.fw.time_ms()
        while keys and keys[0][0] <= t:
            os.write(fd, keys.pop(0)[1])
        if s.status != last_status:
            last_status = s.status
            if s.status.startswith('End:'):
                ends += 1
                if ends == 1:
                    type_text(t + 3000, 'MODE 2 6\r')
                    type_text(t + 5000, 'START\r')
                else:
                    finish_at = t + 5000
        if finish_at and t >= finish_at:
            break
        try:
            term.feed(os.read(fd, 1 << 16))
        except (BlockingIOError, OSError):
            pass
        if t < next_frame:
            continue
        searching = r.fw.lib.app_run_active() and ends == 0
        speed = 2 if searching else 1       # the search at 2x, the race and the console at 1x
        next_frame = t + 1000 * speed // FPS
        x, y, _ = r.pose()
        for i, v in enumerate((x / 1000, y / 1000, 0.0)):
            cam.lookat[i] = 0.85 * cam.lookat[i] + 0.15 * v
        view.update_scene(r.data, cam)
        left = Image.fromarray(view.render())
        top.update_scene(r.data, tcam)
        left.paste(Image.fromarray(top.render()), (VIEW - 230, 10))
        d = ImageDraw.Draw(left)
        d.rectangle((0, 0, 250, 50), fill=(0, 0, 0))
        d.text((10, 8), f'MuJoCo  t = {t / 1000:6.1f} s  x{speed}', font=FONT, fill=(255, 255, 255))
        leds = r.fw.leds()
        for i in range(6):
            d.ellipse((10 + i * 18, 30, 22 + i * 18, 42), fill=(255, 40, 20) if leds >> (5 - i) & 1 else (60, 20, 20))
        frame = Image.new('RGB', (W, H))
        frame.paste(left, (0, 0))
        frame.paste(term.render(), (VIEW, 0))
        ff.stdin.write(frame.tobytes())
        ahead = t / 1000 - (time.monotonic() - wall0)   # the monitor expects real time: never faster
        if ahead > 0:
            time.sleep(ahead)
    os.write(fd, b'\x1b')
    time.sleep(0.5)
    os.kill(pid, 9)
    ff.stdin.close()
    ff.wait()
    try:
        os.remove(args.port)
    except OSError:
        pass
    print(f'{out}: {t / 1000:.0f} s simulated, {ends} runs ended', flush=True)


if __name__ == '__main__':
    main()
