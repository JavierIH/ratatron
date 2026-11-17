"""Where things are: the tools run from the repository (an editable install, `pip install -e software`)."""
import os

PACKAGE = os.path.dirname(os.path.abspath(__file__))
SOFTWARE = os.path.dirname(PACKAGE)
REPO = os.path.dirname(SOFTWARE)
FIRMWARE = os.path.join(REPO, 'firmware')
LOGS = os.path.join(SOFTWARE, 'logs')               # the robot's sessions (commit them)
CALIB_DATA = os.path.join(SOFTWARE, 'calib_data')   # @D dumps as CSV
SIM = os.path.join(PACKAGE, 'sim')
SIM_BUILD = os.path.join(SIM, 'build')              # flash images, the simulator's serial port (not committed)
RUNTIME = os.path.join(os.environ.get('XDG_RUNTIME_DIR') or '/tmp', 'ratatron')   # the link hub's files
HUB_ADDRESS = ('127.0.0.1', 7770)                   # ratatron connect: where the tools attach
HC05 = '20:13:09:11:04:32'                          # the robot's Bluetooth module, RFCOMM channel 1
SIM_TTY = os.path.join(RUNTIME, 'sim-tty')          # the simulator's serial port (a link to its pseudo-terminal)
