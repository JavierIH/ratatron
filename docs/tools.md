# The `ratatron` tools

`software/install.sh` installs the `ratatron` command (a virtual environment in `software/.venv`, editable: the
tools run from this repository; `--no-sim` leaves MuJoCo out). `ratatron COMMAND -h` lists each one's options.

| Command | What |
|---|---|
| `ratatron connect` | holds the robot's link (`/dev/rfcomm0`, else Bluetooth straight to the HC-05, no root), reconnects, records the session to `software/logs/` and `@D` dumps to `software/calib_data/`, shares the link on `127.0.0.1:7770`; `--sim`: the simulator instead |
| `ratatron send [-w REGEX] [-t S] CMD ...` | console commands through the shared link, and the answers |
| `ratatron monitor` | the live maze monitor and console (below) |
| `ratatron sim` | the robot in MuJoCo with its real firmware ([simulator.md](simulator.md)) |
| `ratatron sim batch\|curves\|failures\|mapdiff\|demo\|ir-info\|real-ir` | the simulator's studies |
| `ratatron calib CSV ...` | reports and suggested constants from calibration recordings ([usage.md](usage.md)) |
| `ratatron cycles ELF [maze]` | the planner's cost on an emulated Cortex-M3 (`pip install -e "software[cycles]"`) |
| `ratatron dashboard` | the `diag_test` firmware's LED, IR and encoder panel |
| `ratatron flash ENV` | builds and flashes `competition`, `practice`, `dev` or `virtual` |
| `ratatron test [--sim]` | host tests (also with a 16-state planner queue), the tools' tests, a simulator search |

Typical session: `ratatron connect &`, then `ratatron monitor` and, from scripts, `ratatron send`; the simulator the
same way with `ratatron sim` and `ratatron connect --sim`.

## Live monitor

```
ratatron monitor                 # the link `ratatron connect` shares, else /dev/rfcomm0 at 9600 baud
ratatron monitor --replay software/logs/<session>.log --speed 4
```

Draws the maze while the robot explores it, as pixels (six to a
character): the walls it confirmed in neon, dots where none was seen yet,
the floor of the visited cells coloured by their distance to the goal, the
robot gliding from cell to cell with this run's trail behind it, the
**route it is about to take** (worked out with the firmware's own planner
and costs), the candidate cells while it optimises and, stopped, the
verified fast path; the goal lit, hex coordinates on the axes. During a
run only the robot moves on the map; stopped, the goal pulses and light
flows along the fast path. `--effects` (or `/effects`) makes the map move
during the runs too: a sweep over what is not seen yet, a ripple where a
cell is discovered, the route flowing, the goal pulsing, the candidates
blinking. Banners mark the goal and the end of each run; a
`!!` line turns the frames red. Next to it, the run panel (its phases, the
clock in large digits, the speed runs' times), a status panel (pose,
explored share, fast path, link, RX activity), the coloured log and the
console; a splash screen until the first telemetry arrives. Needs a UTF-8
terminal with 24-bit colour (256 colours work, coarser) that draws the
sextant characters of Unicode 13 (VS Code, GNOME Terminal, kitty do);
where they show as boxes, `--pixels half` (or `/pixels`) draws the maze
with half blocks, two pixels to a character.

- A speed run sends no pose until it ends: the robot is drawn along the
  route at an estimated pace, then where it reports. The run's time is the
  robot's own (the `t=` of its `route` line, `LOG 2`), else the time
  between the lines as they arrive.
- Keys: Enter sends, Up/Down history, PgUp/PgDn scroll the log, Tab shows
  the help, **Ctrl+X sends STOP**, Esc quits. Local commands: `/full` (always
  16x16), `/pixels`, `/effects`, `/clear`, `/note`.
- Every session is recorded in `software/logs/` and can be replayed with
  `--replay`.
- The robot sends its telemetry (`@...` lines of ~20 bytes) only between
  actions; the monitor only transmits when you type (plus a `SYNC` on
  connecting). It does not slow the robot down. `TELEM OFF` turns it off.
- Through `ratatron connect` it shares the link (the hub records the session); on a port of its own it opens it
  exclusively and records the session itself.

## Build, flash and test by hand

`ratatron flash ENV` and `ratatron test` wrap these.

`pio` runs inside `firmware/` (or from the root with `-d firmware`); the
rest from the root.

```
cd firmware
pio run                  # robot firmware, 16x16 goal, no CAL (competition)
pio run -t upload        # flash it (ST-Link with openocd, see AGENTS.md)
pio run -e practice -t upload   # 4x3 practice maze, with CAL and STATUS registers
pio run -e dev -t upload        # 16x16 goal with CAL and STATUS registers
cd ..
make -C firmware/test/host        # PC tests: planner, strategies and simulator
firmware/test/host/build/host_tests --demo   # log and map of a simulated search
python3 -m unittest discover -s software/tests              # monitor and analysis
```

Hardware test environments: `pio run -e uart_test` and `pio run -e
diag_test` (the latter with `ratatron dashboard`). `pio run -e virtual -t
upload`: the virtual robot, the robot firmware with a built-in 16x16 maze;
search and races run through it at real speed, the search saving to the
real flash (to test saves without a maze). The wheels stay still unless `TUNE WHEELS 1`
(robot on a stand only: they turn with every move). Every race starts with
the virtual robot at the start. Back on the real firmware its map is
ignored (send `ERASE`). Architecture and conventions: [AGENTS.md](AGENTS.md);
design notes and investigations: [docs/](docs/).
