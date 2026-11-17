"""ratatron connect: the one program that holds the robot's link, shared with every tool.

It opens the robot (/dev/rfcomm0 if bound, else an RFCOMM socket to the HC-05: no root needed; or the simulator's
port with --sim), reconnects by itself when the link drops, records the whole session to software/logs/ (the
simulator's to its build folder), the @D dumps as CSV to software/calib_data/, and serves the link on
127.0.0.1:7770: any number of tools attach at once
(ratatron monitor, ratatron send, a pyserial URL socket://127.0.0.1:7770). Each gets every line the robot sends;
each line a tool sends goes to the robot (`/note <text>` annotates the last dump instead).
"""
import argparse
import os
import select
import socket
import sys
import time

from .. import paths
from ..monitor.monitor import CalibrationCapture


class BtSocket:
    """The HC-05 through an RFCOMM socket, with the calls of a serial port used here."""

    def __init__(self, mac):
        self.s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.s.settimeout(15)
        self.s.connect((mac, 1))
        self.s.setblocking(False)
        self.fileno, self.close = self.s.fileno, self.s.close

    def write(self, data):
        self.s.sendall(data)

    def read(self, n):
        data = self.s.recv(n)
        if not data:
            raise OSError('closed')
        return data


class Hub:
    def __init__(self, args):
        self.args = args
        logs = os.path.join(paths.SIM_BUILD, 'logs') if args.sim else paths.LOGS    # only the robot's are kept
        os.makedirs(logs, exist_ok=True)
        os.makedirs(paths.RUNTIME, exist_ok=True)
        self.server = socket.create_server(paths.HUB_ADDRESS)
        self.server.setblocking(False)
        stamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        self.log_path = os.path.join(logs, stamp + '.log')
        self.log = open(self.log_path, 'w', encoding='utf-8', buffering=1)
        with open(os.path.join(paths.RUNTIME, 'log_path'), 'w') as f:
            f.write(self.log_path + '\n')
        self.t0 = time.monotonic()
        self.capture = CalibrationCapture(paths.CALIB_DATA)
        self.clients = {}           # socket -> pending input bytes
        self.port = None
        self.rx = b''

    def record(self, direction, text):
        self.log.write('%.3f\t%s\t%s\n' % (time.monotonic() - self.t0, direction, text))

    def say(self, text):
        self.record('!', text)
        print(time.strftime('%H:%M:%S'), text, flush=True)

    def open_port(self):
        import serial
        a = self.args
        try:
            if a.sim:
                p = serial.Serial(paths.SIM_TTY, 9600, timeout=0)
                where = 'the simulator'
            elif a.port or os.path.exists('/dev/rfcomm0'):
                p = serial.Serial(a.port or '/dev/rfcomm0', 9600, timeout=0)
                where = a.port or '/dev/rfcomm0'
            else:
                p = BtSocket(a.mac)
                where = 'HC-05 ' + a.mac
            self.say('connected to ' + where)
            return p
        except (OSError, ValueError) as exc:    # serial.SerialException is an OSError
            self.say('no connection: %s' % exc)
            return None

    def to_clients(self, data):
        for c in list(self.clients):
            try:
                c.sendall(data)
            except OSError:
                self.drop(c)

    def drop(self, c):
        self.clients.pop(c, None)
        c.close()
        self.say('tool left (%d attached)' % len(self.clients))

    def from_client(self, c, data):
        self.clients[c] += data
        while b'\n' in self.clients[c]:
            line, self.clients[c] = self.clients[c].split(b'\n', 1)
            text = line.decode('utf-8', 'replace').strip()
            if not text:
                continue
            if text.startswith('/note '):
                msg = self.capture.note(text[6:])[1]
                self.record('!', msg)
                c.sendall((msg + '\n').encode())
                continue
            if not self.port:
                self.record('!', 'not sent (no link): ' + text)
                c.sendall(b'!! not sent: the robot is not connected\n')
                continue
            try:
                self.port.write((text + '\n').encode())
                self.record('>', text)
            except OSError:
                self.record('!', 'not sent (disconnected): ' + text)
            time.sleep(0.05)

    def from_robot(self, data):
        self.rx += data
        self.to_clients(data)
        while b'\n' in self.rx:
            raw, self.rx = self.rx.split(b'\n', 1)
            text = raw.decode('utf-8', 'replace').rstrip('\r')
            self.record('<', text)
            if text.startswith('@D'):
                result = self.capture.feed(text)
                if result and result[0] in ('ok', 'bad'):
                    self.say(result[1])
            elif self.args.echo and not text.startswith('@'):
                print(text, flush=True)

    def run(self):
        self.say('recording to %s; tools attach at %s:%d' % ((os.path.relpath(self.log_path),) + paths.HUB_ADDRESS))
        retry_at = 0.0
        while True:
            if not self.port and time.monotonic() >= retry_at:
                self.port = self.open_port()
                retry_at = time.monotonic() + 2.0
            fds = [self.server] + list(self.clients) + ([self.port] if self.port else [])
            ready, _, _ = select.select(fds, [], [], 0.5)
            for fd in ready:
                if fd is self.server:
                    c, _ = self.server.accept()
                    c.setblocking(True)
                    self.clients[c] = b''
                    self.say('tool attached (%d attached)' % len(self.clients))
                elif fd in self.clients:
                    try:
                        data = fd.recv(4096)
                    except OSError:
                        data = b''
                    if data:
                        self.from_client(fd, data)
                    else:
                        self.drop(fd)
                elif fd is self.port:
                    try:
                        data = self.port.read(4096)
                        if not data and not isinstance(self.port, BtSocket):
                            raise OSError('no data')    # a serial port readable with nothing: the link is gone
                        self.from_robot(data)
                    except OSError:
                        self.say('disconnected')
                        self.port.close()
                        self.port = None
                        retry_at = time.monotonic() + 1.0


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratatron connect', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', help='serial port of the robot (default /dev/rfcomm0 if bound, else Bluetooth directly)')
    p.add_argument('--mac', default=paths.HC05, help='the HC-05 to reach without /dev/rfcomm0 (default %(default)s)')
    p.add_argument('--sim', action='store_true', help='the simulator (ratatron sim) instead of the robot')
    p.add_argument('--echo', action='store_true', help="print the robot's text lines here too")
    args = p.parse_args(argv)
    try:
        hub = Hub(args)
    except OSError as exc:
        sys.exit('!! %s:%d is taken: is another `ratatron connect` running? (%s)' % (paths.HUB_ADDRESS + (exc,)))
    try:
        hub.run()
    except KeyboardInterrupt:
        hub.say('stopped')


if __name__ == "__main__":
    main()
