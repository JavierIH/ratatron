#!/usr/bin/env python3
"""RATATRON in MuJoCo: the robot's firmware (firmware/src, unchanged) driving the CAD robot in a 16x16 maze.

    ratatron sim                          # random maze (seed 1), viewer, console on stdin
    ratatron sim --maze oshwdem2026                     # a maze of sim/mazes, or any maze file (micromouseonline)
    ratatron sim --headless --speed 0 --cmd "MODE 1" --cmd START --until-idle

Console: every line goes to the firmware as over Bluetooth (MODE 1, START, STOP, MAP, STATUS, ...), except
    :start :select      press a button          :place x y [N|E|S|W]   put the robot at a cell centre
    :reset              power cycle (map kept)   :erase-flash           power cycle with blank flash
    :speed f            real-time factor (0 = as fast as possible)       :quit
Viewer keys: Page Up = SELECT, Page Down = START, End = back to the start cell.
The monitor, as with the robot: `ratatron connect --sim`, then `ratatron monitor` (the console, SYNC, the map
and the route all work through it).
"""
import argparse
import os
import queue
import sys
import threading
import time

from .. import paths
from .firmware import STATE_NAMES, build
from .maze import Maze
from .robot import Robot

KEY_PAGE_UP, KEY_PAGE_DOWN, KEY_END = 266, 267, 269
MAZES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mazes')


def maze_path(name):
    """A maze file, or the name of one in sim/mazes (`oshwdem2026`)."""
    if os.path.exists(name):
        return name
    bundled = os.path.join(MAZES, name if name.endswith('.txt') else name + '.txt')
    if os.path.exists(bundled):
        return bundled
    raise SystemExit(f'no maze {name!r} (the bundled ones: {", ".join(sorted(f[:-4] for f in os.listdir(MAZES)))})')


class Session:
    def __init__(self, args):
        self.args = args
        self.maze = Maze.load(maze_path(args.maze)) if args.maze else Maze.generate(seed=args.seed, loops=args.loops)
        os.makedirs(os.path.dirname(args.flash), exist_ok=True)
        if args.erase_flash and os.path.exists(args.flash):
            os.remove(args.flash)
        self.robot = Robot(self.maze, build(args.practice), args.flash, seed=args.seed, slip_deg=args.slip)
        self.inbox = queue.Queue()
        self.speed = args.speed
        self.quit = False
        self.pty_fd = None
        self.log = open(args.log, 'a') if args.log else None
        self.idle_since = None
        self.started = False
        self.crashes = 0
        self.wait = None
        self.status = ''

    # ---- input ------------------------------------------------------------------------------------------------
    def stdin_reader(self):
        for line in sys.stdin:
            self.inbox.put(line.strip())
        self.inbox.put(':eof')

    def pty_reader(self):
        import select
        buf = b''
        while not self.quit:
            select.select([self.pty_fd], [], [], 0.1)
            try:
                chunk = os.read(self.pty_fd, 256)
            except OSError:         # nothing yet, or no program on the other end
                time.sleep(0.05)
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self.inbox.put(line.decode(errors='replace').strip())

    def handle(self, line):
        r = self.robot
        if not line:
            return
        if not line.startswith(':'):
            r.fw.send(line)
            if line.upper().startswith(('START', 'MODE')):
                self.started = True
            return
        words = line[1:].split()
        cmd = words[0].lower() if words else ''
        if cmd in ('start', 'select'):
            r.press(cmd)
            self.started = True
        elif cmd == 'place' and len(words) >= 3:
            r.place(int(words[1]), int(words[2]), (words[3] if len(words) > 3 else 'N').upper())
        elif cmd in ('reset', 'erase-flash'):
            if cmd == 'erase-flash' and os.path.exists(self.args.flash):
                os.remove(self.args.flash)
            r.place(0, 0, 'N')
            r.boot('power-on')
        elif cmd == 'wait-idle':        # scripts: the next lines wait until the run started before has ended
            self.wait = 'start'
        elif cmd == 'speed' and len(words) > 1:
            self.speed = float(words[1])
        elif cmd in ('quit', 'eof'):
            if cmd == 'quit' or not self.args.headless:
                self.quit = cmd == 'quit'
        else:
            print(f'?? {line}', flush=True)

    def key(self, keycode):
        if keycode == KEY_PAGE_UP:
            self.inbox.put(':select')
        elif keycode == KEY_PAGE_DOWN:
            self.inbox.put(':start')
        elif keycode == KEY_END:
            self.inbox.put(':place 0 0 N')

    # ---- output -----------------------------------------------------------------------------------------------
    def output(self, text):
        if not text:
            return
        if self.pty_fd is not None:
            try:        # non-blocking: with no monitor reading, what does not fit is lost (as over Bluetooth)
                os.write(self.pty_fd, text.replace('\n', '\r\n').encode())
            except OSError:
                pass
        if self.log:
            self.log.write(text)
        for line in text.splitlines(keepends=True):
            if line.startswith(('== ', 'Goal reached', 'Optimal fast', 'At the start', 'End:', 'START mode')):
                self.status = line.strip()[:48]
            if line.startswith('@P') and len(line) >= 5:
                self.check_pose(line[2:5])
            if self.args.telemetry or not line.startswith('@'):
                t = self.robot.fw.time_ms() / 1000.0
                sys.stdout.write(f'[{t:8.3f}] {line}' if self.args.timestamps else line)
        sys.stdout.flush()
        self.last_text = text

    def check_pose(self, p):
        """The firmware's pose (@P: x, y hex digits and heading) against where the robot really is."""
        try:
            fx, fy, fh = int(p[0], 16), int(p[1], 16), p[2]
        except ValueError:
            return
        x, y, yaw = self.robot.pose()
        cx, cy = int(x // 180), int(y // 180)
        heading = 'NWSE'[int(((yaw % 360) + 45) // 90) % 4]
        off_x, off_y = x - (cx + 0.5) * 180, y - (cy + 0.5) * 180
        yaw_err = (yaw - {'N': 0, 'W': 90, 'S': 180, 'E': -90}[fh] + 180) % 360 - 180
        wrong = (cx, cy, heading) != (fx, fy, fh)
        self.pose_checks = getattr(self, 'pose_checks', 0) + 1
        if wrong or self.args.poses:
            print(f'## {"POSE LOST" if wrong else "pose"}: firmware ({fx},{fy}){fh}, robot ({cx},{cy}){heading} '
                  f'off {off_x:+.0f},{off_y:+.0f} mm yaw {yaw_err:+.1f} deg', flush=True)
        if wrong:
            self.pose_lost = getattr(self, 'pose_lost', 0) + 1

    # ---- loop ---------------------------------------------------------------------------------------------------
    def step(self):
        r = self.robot
        if self.wait:
            active = r.fw.lib.app_run_active()
            if self.wait == 'start' and active:
                self.wait = 'end'
            elif self.wait == 'end' and not active:
                self.wait, self.wait_until = 'settle', r.fw.time_ms() + 2000
            elif self.wait == 'settle' and r.fw.time_ms() >= self.wait_until:
                self.wait = None
        while not self.wait and not self.inbox.empty():
            self.handle(self.inbox.get())
        state = r.step()
        if state == 1:          # RESET: the chip restarts, the robot stays where it is
            self.output(r.fw.received())
            r.boot('RESET (software)')
        elif state >= 2 and not getattr(self, '_halted', False):
            self._halted = True
            self.output(r.fw.received() + f'\n## firmware {STATE_NAMES[state]}\n')
        if r.fw.time_ms() % 10 == 0:
            text = r.fw.received()
            self.output(text)
            touched = r.crashed()
            if touched:
                self.crashes += 1
                if self.crashes == 1 or self.crashes % 500 == 0:
                    x, y, h = r.pose()
                    print(f'## [{r.fw.time_ms() / 1000:8.3f}] the robot touches a wall at x={x:.0f} y={y:.0f} mm, heading {h:.1f} deg ({touched})',
                          flush=True)
            if self.args.until_idle and self.started:
                busy = r.fw.lib.app_run_active()
                if busy:
                    self.idle_since = None
                elif self.idle_since is None:
                    self.idle_since = r.fw.time_ms()
                elif r.fw.time_ms() - self.idle_since > 8000:
                    self.quit = True
        if self.args.max_s and r.fw.time_ms() > self.args.max_s * 1000:
            self.quit = True

    def run_headless(self):
        wall0, sim0 = time.monotonic(), self.robot.fw.time_ms()
        rec = Recorder(self, self.args.record) if self.args.record else None
        while not self.quit:
            self.step()
            if rec:
                rec.maybe_frame()
            if self.speed > 0:
                ahead = (self.robot.fw.time_ms() - sim0) / 1000.0 / self.speed - (time.monotonic() - wall0)
                if ahead > 0.01:
                    time.sleep(ahead)
        if rec:
            rec.close()

    def run_viewer(self):
        import mujoco.viewer
        r = self.robot
        with mujoco.viewer.launch_passive(r.model, r.data, key_callback=self.key) as v:
            v.cam.lookat[:] = (r.maze.size * 0.09, r.maze.size * 0.09, 0)
            v.cam.distance, v.cam.elevation, v.cam.azimuth = 3.6, -70, 90
            wall0, sim0, last_sync = time.monotonic(), r.fw.time_ms(), 0.0
            while v.is_running() and not self.quit:
                with v.lock():
                    for _ in range(10):
                        self.step()
                now = time.monotonic()
                if now - last_sync > 1 / 60:
                    if self.args.follow:
                        x, y, _ = r.pose()
                        v.cam.lookat[:] = (x / 1000, y / 1000, 0)
                    v.sync()
                    last_sync = now
                if self.speed > 0:
                    ahead = (r.fw.time_ms() - sim0) / 1000.0 / self.speed - (time.monotonic() - wall0)
                    if ahead > 0.002:
                        time.sleep(ahead)
                    elif ahead < -0.5:      # slower than asked: do not try to catch up
                        wall0, sim0 = time.monotonic(), r.fw.time_ms()


class Recorder:
    """An mp4 (ffmpeg) of the run: a camera following the robot, the whole maze in a corner, time, mode and LEDs.
    Runs (the robot moving) at --record-speed, the waits between them at 8x."""

    def __init__(self, session, path, width=1280, height=720, fps=30):
        import subprocess
        import mujoco
        from PIL import Image, ImageDraw     # noqa: F401  (checked here: needed for the overlay)
        self.s, self.fps = session, fps
        m = session.robot.model
        self.mujoco = mujoco
        self.big = mujoco.Renderer(m, height, width)
        self.small = mujoco.Renderer(m, 300, 300)
        self.cam, self.top = mujoco.MjvCamera(), mujoco.MjvCamera()
        side = session.maze.size * 0.18
        self.top.lookat[:] = (side / 2, side / 2, 0)
        self.top.distance, self.top.elevation, self.top.azimuth = side * 1.75, -90, 90
        self.cam.distance, self.cam.elevation, self.cam.azimuth = 0.75, -50, 90
        self.next_ms = session.robot.fw.time_ms()
        self.w, self.h = width, height
        self.ff = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                                    '-s', f'{width}x{height}', '-r', str(fps), '-i', '-', '-c:v', 'libx264',
                                    '-pix_fmt', 'yuv420p', '-crf', '23', path], stdin=subprocess.PIPE)

    def maybe_frame(self):
        r = self.s.robot
        t = r.fw.time_ms()
        if t < self.next_ms:
            return
        moving = r.fw.lib.app_run_active()
        speed = self.s.args.record_speed if moving else 8.0
        self.next_ms = t + max(1, int(1000 * speed / self.fps))
        from PIL import Image, ImageDraw
        x, y, _ = r.pose()
        look, target = self.cam.lookat, (x / 1000, y / 1000, 0.0)
        for i in range(3):
            look[i] = 0.85 * look[i] + 0.15 * target[i] if look.any() else target[i]
        self.big.update_scene(r.data, self.cam)
        frame = Image.fromarray(self.big.render())
        self.small.update_scene(r.data, self.top)
        inset = Image.fromarray(self.small.render())
        frame.paste(inset, (self.w - 310, 10))
        d = ImageDraw.Draw(frame)
        leds = r.fw.leds()
        d.rectangle((0, 0, 330, 60), fill=(0, 0, 0))
        d.text((10, 8), f't = {t / 1000:6.1f} s   x{speed:g}', fill=(255, 255, 255))
        d.text((10, 30), self.s.status, fill=(255, 220, 60))
        for i in range(6):
            on = leds >> (5 - i) & 1
            d.ellipse((200 + i * 20, 8, 212 + i * 20, 20), fill=(255, 40, 20) if on else (60, 20, 20))
        self.ff.stdin.write(frame.tobytes())

    def close(self):
        self.ff.stdin.close()
        self.ff.wait()


def make_parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', help='maze file (micromouseonline text format); default: generated')
    p.add_argument('--seed', type=int, default=1, help='generated maze and sensor noise seed')
    p.add_argument('--loops', type=float, default=0.08, help='generated maze: share of walls removed (loops)')
    p.add_argument('--practice', action='store_true', help='the practice build (goal 3,2)')
    p.add_argument('--slip', type=float, default=0.0,
                   help='encoder slip: deg of encoder heading (+ left) in the first 100 mm of every move from rest')
    p.add_argument('--flash', default=os.path.join(paths.SIM_BUILD, 'flash.bin'),
                   help='the robot flash (saved map) between runs')
    p.add_argument('--erase-flash', action='store_true', help='start with blank flash (no saved map)')
    p.add_argument('--headless', action='store_true', help='no viewer')
    p.add_argument('--speed', type=float, default=1.0, help='real-time factor (0: as fast as possible)')
    p.add_argument('--cmd', action='append', default=[], help='console line to send at the start (repeatable)')
    p.add_argument('--until-idle', action='store_true', help='quit 8 s after the run ends')
    p.add_argument('--max-s', type=float, default=0, help='quit after this many simulated seconds')
    p.add_argument('--telemetry', action='store_true', help='show the @ telemetry lines too')
    p.add_argument('--timestamps', action='store_true', help='simulated time before every line')
    p.add_argument('--log', help='append everything the firmware sends to this file')
    p.add_argument('--port', default=paths.SIM_TTY,
                   help=f'the serial port the monitor opens (a link to a pseudo-terminal; default {paths.SIM_TTY})')
    p.add_argument('--no-pty', action='store_true', help='no serial port for the monitor')
    p.add_argument('--poses', action='store_true', help='print the true pose at every firmware pose report')
    p.add_argument('--follow', action='store_true', help='the viewer camera follows the robot')
    p.add_argument('--record', metavar='MP4', help='headless: record a video (ffmpeg)')
    p.add_argument('--record-speed', type=float, default=2.0, help='video speed while the robot runs (default 2x)')
    p.add_argument('--save-maze', help='write the maze to this file and go on')
    return p


def open_port(s, path):
    """The robot's Bluetooth port for the monitor: a pseudo-terminal linked at `path`. The slave stays open here, so
    the port survives monitors coming and going; non-blocking: with no monitor the output is lost, not the run."""
    import pty
    import tty
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    s.pty_fd = master
    os.makedirs(os.path.dirname(path), exist_ok=True)
    try:
        os.remove(path)
    except OSError:
        pass
    os.symlink(os.ttyname(slave), path)
    threading.Thread(target=s.pty_reader, daemon=True).start()
    return slave


def main():
    args = make_parser().parse_args()
    s = Session(args)
    if args.save_maze:
        with open(args.save_maze, 'w') as f:
            f.write(s.maze.text())
    link = None
    if not args.no_pty:
        link = args.port
        open_port(s, link)
        print(f'## the robot\'s Bluetooth port: {os.path.relpath(link)}  '
              f'(ratatron monitor --port {os.path.relpath(link)})', flush=True)
    for c in args.cmd:
        s.inbox.put(c)
    if not args.headless or not args.cmd:
        threading.Thread(target=s.stdin_reader, daemon=True).start()
    try:
        s.run_headless() if args.headless else s.run_viewer()
    except KeyboardInterrupt:
        pass
    s.output(s.robot.fw.received())
    x, y, h = s.robot.pose()
    print(f'## end: {s.robot.fw.time_ms() / 1000:.1f} s simulated, robot at x={x:.0f} y={y:.0f} mm heading {h:.1f} deg',
          flush=True)
    s.robot.fw.close()
    if link:
        try:
            os.remove(link)
        except OSError:
            pass


if __name__ == '__main__':
    main()
