# Open issues

One entry per issue: state and next step, one or two lines each. Update the
entry before ending a session; move it to "Closed" with its result and
commit when done. Details go in the docs it points to, not here.

## Next session

At the robot (`ratatron flash practice`; everything below is in `develop`):
- Validate the firmware of 11-17, so far only in the simulator: the
  curves keep at most 2 deg of the centring's offset, chains of 3+
  curves at 400, the arrival check of the races (walls vs the map), the
  RAM savings (`STATUS`: the stack never used, ~2.7 KB expected). A
  search and races 2.4-2.6 on a practice layout with chained curves (E).
- Measure for the simulator (issue 24), then write them in
  `software/ratatron/sim/config.py`: the mass (a scale), the centre of
  mass along the robot (a knife-edge balance; its height if possible),
  the skids' gap to the floor (feeler gauge). Then check that race 2.6
  fails in the simulator as on the robot.
- The IR at an angle (issue 24, ~10 min): in a corner cell, the robot by
  hand at 0, 15, 30, 45, 60, 75, 90 deg about the centre, `CAL NOISE` at
  each; `ratatron sim real-ir` style comparison with the model.
- The new tools with the real link: `ratatron connect` (rfcomm0 and the
  direct RFCOMM socket), `ratatron monitor` and `ratatron send` attached
  at once; the monitor's new look live.

Without the robot:
- The 3D viewer (`ratatron sim` with a screen) is untested.
- Better 3D files of the robot from the user: the FreeCAD file holds
  several design iterations (the parts were picked by hand,
  `software/ratatron/sim/export_cad.py`), shows 4 LEDs of the board's 6
  and no clear skid; with clean, current parts (and the real skids) the
  simulator's geometry and collisions can be exact.
- The competition gives 7 minutes for the search and the races
  together (the user, 11-13): the OPTIM phase stays as it is.
- Next by `docs/roadmap.md`: the large maze (issue 11, deferred until a
  long stretch is available), then one way to search and the legs at 600.

## Open

- Issue 22, review of 10-22 and audit of 11-11 (`docs/review.md`, last
  two sections). The audit (A1-A6, the doubtful front) is validated on
  the robot (11-13, `docs/testing.md`). Question 5 done 11-16
  (`CURVE_PRE`/`CURVE_POST` deleted, next flash); question 4 closed
  11-16 (the SysTick peak is the CAL recorder's halving, the normal tick
  ~265-325 us: emulated, no change). Left: question 3 of 10-22 (only if
  the robot shows it); the plan is `docs/roadmap.md`.

- Issue 19, work plan of the review (`docs/review.md`): done 10-06 items 2
  (flash defences removed), 3 (`DEV_TOOLS`: competition build lean) and 7
  (LEDs); 5 (split `motion.c`) and 6 (hardware, 11-16) declined. Left: 1
  (large stretch), 4 (HC-05 at 115200: the module in AT mode, on the
  robot; the user, 11-16: later, with the next flash). Item 8 declined
  11-16: the monitor keeps its planner port (the transcript tests'
  cross-check of the firmware). Ideas
  from issue 15: a SysTick stop if the following error runs away while the
  main loop blocks (no such failure seen: only with one).

- Issue 11, 16x16 competition readiness (`docs/testing.md`): tests 2-5
  done 09-27 (results there). Left: test 1 on a large stretch (review item
  1), deferred 11-16: no long stretch available, done when there is one
  (the long-corridor centring check of OSHWDEM goes with it). The
  baseboard oscillation check is dropped (11-16: no means to do it).

- Issue 24, MuJoCo simulator (`ratatron sim`, `docs/simulator.md`): the firmware
  (unchanged) on a simulated chip drives the CAD robot in a 16x16 maze;
  search and races 2.4-2.6 clean on a generated maze (11-17). Next, at
  the robot: weigh it and measure its centre of mass (along the axis, a
  knife-edge balance), then write them in `software/ratatron/sim/config.py` (`MASS_KG`,
  `COM_MM`, now estimated: the fast curves depend on them) and check that
  2.6 fails in the simulator as on the robot. The viewer is untested (no
  monitor in that session). 11-17: 525 real mazes (`ratatron sim batch`):
  searches 523/525 (2 unreachable goals), races 2.4 fail 13.6 % and 2.6
  20 %, almost all touching in curves. Causes found: the centring's
  heading offset kept through a curve (a curve begun mid-correction turns
  wide; chained curves turn it into an along-track error), and the
  centring turning in place while braking (the goal reached 11 deg off).
  Branch `sim-steer-ds`: the offset clamped to 2 deg at each curve, 2.4
  fails 8 %, 2.6 17.5 % (simulator only). Curves (`ratatron sim curves`,
  mazes spiral, stairs): single curves clean up to 480; chains of curves
  drift sideways every curve with no feedback (stairs: 2.6 crashes at
  the 11th). Prototype C (same branch): routes with 3+ chained curves at
  400 at most. Race-only, same maps: 2.4 fails 12.4 -> 7.5 %, 2.6 16.8 ->
  7.4 %, 2.6 median time to the goal 13.3 -> 14.5 s; C breaks 4 host tests
  (the planner still prices chains at 480). IR in curves
  (`curve_ir_info.py`): ideal sensors fix the position to ~1 mm per curve;
  next, CAL RUN curves on the robot to see if the real Sharp agrees.
  Branch `sim-curve-fixes` now limits only the chained curves (each at
  its own speed): 2.4 fails 7.5 %, 2.6 10.1 %, 2.6 median +0.5 s (all
  curves at 400: 7.4 %, +1.25 s); the planner keeps its costs (--costs:
  no pair better by more than 0.3 %).
  The real CAL RUN recordings of 09-26 (`ratatron sim real-ir`,
  layouts C and E): readings off the model by 5-15 mm (MAD), biases up
  to 45 mm, but the trajectory is dead reckoning and the layout guessed
  by the time: not conclusive. Next at the robot (~10 min): in a corner
  cell, the robot placed by hand at 0, 15, 30, 45, 60, 75 and 90 deg
  about the cell centre, `CAL NOISE` at each: the side and front
  readings against the angle with known geometry.
- Branch `ram-savings` (11-17): 1.1 KB of RAM freed (planner queue 512
  with an exact overflow, IR trail 32 ms, centring average 16):
  competition 47.2 %, dev 78.9 %. Host tests (also with a 16-state
  queue, in CI) and the simulator pass; to merge after a robot check.
  Then the robot (mass, centre of mass, skid gap) before trusting any of it.

## Closed

- 2026-11-16 One repo for the whole robot (issue 23): the firmware repo
  (`rat_sw`) merged into the hardware's (`ratatron`), every commit with
  its hash, as `electronics/`, `mechanics/`, `firmware/`, `software/`,
  `docs/`; pushed with the tags, CI green. The hardware is CC BY-SA 4.0
  (the user's choice), the code GPLv3. `rat_sw` has an orphan `main`
  (the default) with a README pointing here; archiving it on GitHub is
  the user's. The branch `jev-remote` stays local, in the old layout
  (the user: it may never be pushed).

- 2026-11-16 OSHWDEM 2026 (issue 17; `docs/faults/oshwdem2026.md`, section
  7): the race 2.4 crash was the old heading-based centring (parallel but
  off-centre on long corridors), replaced and validated on short
  straights 11-13; no race starting after it, the HSI wedge (fixed 10-05)
  or A1 (fixed 11-11). The long-corridor check goes with issue 11.

- 2026-11-13 Robot session after the flash of `practice`
  (`docs/testing.md`, session of 11-13), steps 1-6 all passed. Centring
  at wall ends (GitHub issue #1, `docs/faults/centring.md`, section 12):
  the 10-08 `SLIPPED` case 1.7 deg, 3-cell straights at 300-900 1.8-2.6
  deg (4-12 before), crooked starts still centre. A search, races 2.4
  (x3) and 2.5 (x2) clean; races read only (a wall on the route:
  `BLOCKED`, map and flash untouched); sides with `CONT OFF` right; the
  one-sensor front `?`; the follower left the map alone. Battery ~5 %
  under 10-08: `pace=90-95%` at 900. `ratatron connect` no longer needs
  `/dev/rfcomm0` (an RFCOMM socket, no root).

- 2026-11-11 Lighting check (issue 21; `docs/lighting.md`): `CHECK`
  removed. On the robot (10-08) it gave FAIL at home under good light:
  the shifts it resolved (a few mm) are the size of the placement's and
  the maze's tolerances. Flash -1.7 KB. The competition checklist reads
  the light with `IR` (`docs/testing.md`); the IR still have no ambient
  subtraction (hardware), so a strong lamp can still make an open right
  side a wall: shade it.

- 2026-10-08 The IR delay (issue 20; `docs/measurements.md`): 8 wall
  approaches at 100-900 mm/s (two at each) gave FL 16-24 / FR 24-32 ms at 600-900 (the
  same ms at both speeds: a delay, no slide), in-place turns 0-8 ms on all
  four. `IR_DELAY_MS` 50 -> 22 (the user's call: stops alike, ~40 ms less
  crawl at the end), goes with the next flash; `SEARCH_LEG_SPEED_MAX`
  stays 450 (robust before fast).

- 2026-10-06 Review of 09-30 (issue 15): (a) map repairs only at rest, their
  first pass keeping the robot's cell's walls (test_repair_at_rest), and (b)
  the shorter STATUS "flash:" line, in the firmware since the 10-05 flash;
  software/ratatron/calib/analyze.py unwraps its int16 columns (long CAL RUN recordings); the
  README's CAL table rewritten. Ideas left for the review's plan (issue 19):
  cap the race return's curve speed, a SysTick stop if the following error
  runs away while the main loop blocks, measure the SysTick load.

- 2026-10-05 The flash wedge (issue 18; `docs/faults/oshwdem2026.md`, `docs/faults/freezes.md`):
  hard motor reversals leave the running HSI crawling (~8000x slow flash
  operations, a reset that does not boot). Fixed by stopping the HSI except
  during flash operations (e3c5b21), validated on a stand: control wedged
  after 20 reversals, the fix clean over 600, reset boots, saves normal.
  Watch STATUS "HSI n" (should stay 0) and "!! flash" lines in real runs.

- 2026-09-30 Simplification (issue 16; the user found the code too
  complicated): removed `TURN_CARRY`, `SIDE_LEVER`, the old centring
  integral (`TUNE OBSERVER 0`, `BIAS_WIN`) and `SENSE_SETTLE`, all neutral
  after their tests (4ceda32: host transcripts identical, RAM -200 B,
  flash -544 B; next flash, keeps map and parameters). `robot_config.h`
  one line per constant (9c0ff65: 358 -> 157 lines, firmware identical
  but the build time), measurements in `docs/design.md`; the rule for
  new knobs in AGENTS.md. `CURVE_PRE`/`CURVE_POST` stay: `software/ratatron/calib/analyze.py`
  suggests them after `CAL CURVE` (a new floor).

- 2026-09-27 Centring on straights (issue 14, layout I): no weave at
  300-600; the slow convergence was the observer learning a turn's yaw
  (KI 8: -1.5..-3 mm to the end of 3 cells). KI 16 is the new default
  (f95a56d, next flash; until then send `KI 16` after a reset): ~0 from the
  second cell; 24 weaved. At 800 `STEER_VREF` 400-600 no difference; a 2-4
  mm step at the two-wall to one-wall post. `docs/design.md`, centring.

- 2026-09-27 Race 2.5/2.6 curves and the goal on G (issue 12): the curves
  at 480 slide outwards (grip limit, ~3.3 m/s^2 lateral) and two left ones
  in a row left mode 5 77 mm short of the goal, then crashed on the return.
  At 400 (race 2.5, `FAST_MID_*` 900/400) two full cycles on G clean both
  ways, every end `end=IR` (goal: FL 96/FR 122 squared and aligned, then
  85/103), distances +-18 mm between runs, absorbed by the IR. Race 2.6
  (900/480) stays in the menu at the grip limit: not repeatable, use it
  knowing that.

- 2026-09-27 New mode menu (issue 13): 1 search, 2 races (2.1/2.2 left/right
  followers, 2.3 no curves, 2.4 800/300, 2.5 900/400, 2.6 900/480), 3 erase.
  Validated on G: the left follower reported the goal after 11 actions and
  went on 17 legs until STOP (no "!!"); race 2.3 there and back clean (5 + 5
  legs, all `end=IR`, err <= 3 mm). Its save hit the flash wedge and the HSI
  restart cleared it (`docs/faults/freezes.md`, n=2).

- 2026-09-27 Side-sensor calibration (issue 6, roadmap): no flash needed.
  `software/ratatron/calib/analyze.py` now fits rounds of `CAL NOISE` + `CAL TURN 1` x4 (the
  fronts facing the side walls measure the offset): slopes SL 0.95, SR 0.99,
  centred 88.1/75.3 vs 89/76 in firmware (0.1 mm of centring), kept. The
  180 deg pair needs the robot centred front to back (warns now).

- 2026-09-27 Flash of the followers' straights and the boot sweep (issue
  10): sweep and SELECT OK; on E mode 2 reached the goal in 9 actions,
  mode 3 in 11 (dead end at (1,0) included), 2-cell legs, no "!!".
  `TURN_CARRY` stays 0 (would need its own in-place scale; no gain on D).

- 2026-09-27 Flash of the new modes (issue 9): LEDs and SELECT OK, mode 2
  to the goal (9 actions), modes 4 and 5 on E clean both ways (route
  `2R1R1L1R1L1L2` in one leg, no "!!"). Mode 3 moves to issue 10.

- 2026-09-27 `!! the program stalled 3000 ms` while mode 4 (now 6)
  waited for the erase confirmation: the wait now reports health (next
  flash).

- 2026-09-27 Speed-run curves at 480 off-centre on staircases (issue 1):
  the 180's over-turn fixed (`TURNTICKS` 403), wheel drift trimmed
  (`WHEEL_DIFF` -0.0025), and the curves' sideways slip compensated with
  `CURVE_PRE_SLIP` 7 above `CURVE_PRE_V0` 300: layout E's last straight
  went from +21 mm left to +4.6/0.0 at 480, 0.0 at 400, +0.4 at 300
  (`docs/design.md`, curve slip). `TURN_CARRY`, `SIDE_LEVER`: no clear
  gain, left off.

- 2026-09-27 Flash wedge again (09-27 00:26, `docs/faults/freezes.md`): a restart
  of the HSI cleared it (n=1), so a slow halfword now restarts the HSI and
  the write goes on (a bad record is rewritten in the next slot); only a
  second slow one blocks the store. Goes with the next flash (keeps map and
  params); if a "!! flash: 2 bytes en ..." line appears, copy it into
  `docs/faults/freezes.md`.

- 2026-09-26 Search legs on long straights (layout D, 450 mm/s): two
  searches (16:16, 16:23), both identical: 13 actions, the legs 2 N, 3 E,
  3 W, 2 S decided each next cell in time and stopped on the front wall
  (`end=IR`) 0.3-0.5 mm short, err <= 1.7 mm / 2.7 deg; map right, no "!!".

- 2026-09-26 Freezes, the flash wedging (`docs/faults/freezes.md`): fix 452756d
  validated on layout D (search, 4 speed runs, `SAVE`s to a full log and
  its compaction: 56-63 us a halfword, no "!!"). The wedge never showed in
  those writes; if a "!! flash" line appears, reopen with it.

- 2026-09-26 Robot checks of the 10:48 firmware: 1-cell search legs leave
  no doubtful sides (only rest readings do, by design); step mode does not
  pause before a speed run; `CLOCK HSI` (64 MHz), `CAL TURN 4`, `RESET`
  (boots, loads the map, back on the crystal) all work.
- 2026-09-26 Layout C scrape: the centring's integral learned an
  off-centre start as heading; replaced by an observer (1472d96). Squaring
  limited to 12 deg (93370fc). Stall reports only printed in mode 5
  (0221cd6). `CAL RUN` records a speed run (04c0caf, a6d7fac).
