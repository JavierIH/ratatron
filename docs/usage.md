# Using the robot

Buttons, modes and LEDs; the Bluetooth console; the calibration tests.

## Buttons, modes, LEDs

| Button | Stopped | During a run |
|---|---|---|
| SELECT | after boot (LEDs sweeping) picks mode 1; then cycles the modes (1: LEDs 1-2, 2: 3-4, 3: 5-6); in the race menu, cycles the races (LEDs 1..n) | — |
| START | launches the mode after a 5 s countdown, every LED blinking fast (ignored until a mode is picked); on mode 2 it opens the race menu, and there it launches the race | stops the run |

Modes: 1 search, 2 race, 3 erase the map and go back to the build's default
goal (press START again within 3 s). The race menu (slow blink; only a reset
leaves it): 1 left wall follower, 2 right wall follower (past the goal they
go on until STOP; they explore on a blank map of their own, the map of the
search is back when they end), 3 no curves (verified path, straights at `FAST` 800,
turns in place; there and back), 4 `FAST 800 CURVE 300`, 5 `FAST 900 CURVE
400`, 6 `FAST 900 CURVE 480`. Picking a race sets `FAST` and `CURVE`; the
console can change them before START. No race changes the map or writes
the flash.

At a competition: `ERASE` (or mode 3) if the maze is new → mode 1 → race
2.4, and the faster ones once the previous one goes well.

LEDs: the mode picked (a blink a second = alive); on the straights, 1-3 =
centring on the left wall, 4-6 = on the right one; map saved in flash = 3
slow blinks; failed run = 3 fast blinks; **map not saved** (only in RAM:
do not reset or power off) = the left and right halves alternating fast;
START refused (no verified path) = LEDs 3 and 4 blinking 3 times;
continuous fast blink = `Error_Handler`; continuous slow blink = CPU fault
(motors stopped).

## Bluetooth console

One command per line (in the monitor or any serial terminal), upper or
lower case. The firmware has no `HELP` (it saves flash): this table is the
reference. "Stopped" = refused during a run.

| Command | What it does | Stopped |
|---|---|---|
| `STATUS` | state, parameters, stack, the slowest SysTick, reset cause, clock and flash registers and chip (`practice`/`dev` builds), the flash's free slots and its last write's times; after a search, the worst decision on the way (also printed when the run ends) | |
| `MODE n [k]` | picks the mode: 1 search, 2 k race k (1/2 left/right follower, 3 no curves, 4 800/300, 5 900/400, 6 900/480), 3 erase map | yes |
| `START` | launches the mode picked after 5 s (as the button) | |
| `STOP` | stops the run (as START during a run) | |
| `PAUSE`, `RESUME` | brakes and waits; goes on after a pause or a step | |
| `STEP ON\|OFF` (or `DEBUG`) | step mode: a pause after every action | |
| `SPD n` | mm/s cruise of the search and of the speed run's return | |
| `FAST n` | mm/s cruise of the speed run's straights | |
| `CURVE n` | mm/s through the speed run's curves (the motors cap it at ~480) | |
| `ACCEL n` | mm/s² speeding up and braking on straights | |
| `TURN n`, `TACCEL n` | deg/s peak and deg/s² of the turns in place | |
| `TURNTICKS n` | encoder ticks of a 90° turn (fewer = turns less) | |
| `KP f` | centring: deg/s of turn per mm off-centre | |
| `KD f` | centring: damping, deg of turn per mm the error changes | |
| `TUNE [name value]` | sets a control constant live (not saved); `TUNE` alone lists them | |
| `LOG 0-2` | log detail | |
| `TELEM ON\|OFF` | `@` lines for the monitor's live map | |
| `CONT ON\|OFF` | search with straights without stopping (ON, the default) or stopping in every cell; until reset | yes |
| `DEFAULTS` | back to the default parameters | |
| `IR` | sensors in mm and raw, and the encoders | |
| `WALLS` | senses the walls right now | yes |
| `MAP` | draws the ASCII map with the fast path | yes |
| `GOAL x y [x1 y1]` | goal cells (e.g. `GOAL 7 7 8 8` for 16x16) | yes |
| `SAVE` | saves map, goal and parameters (compacts the flash if it has no free slot) | yes |
| `ERASE` | erases the map in RAM and flash and goes back to the build's default goal (16x16: 7 7 8 8) | yes |
| `HOME` | "the robot is at the start facing north" | yes |
| `SYNC` | sends map and state to the monitor again | yes |
| `CAL ...` | calibration tests (below; `practice` and `dev` builds) | yes |
| `RESET` | resets the micro | |

## Calibration data

Only in the `practice` and `dev` builds (the competition one leaves the
recorder out). During a test the robot records the encoders, the PWM applied and the 4 raw
IR every 2-10 ms (if they do not fit, it spaces the samples out instead of
cutting the end) and dumps them when it ends; the monitor saves them as CSV
in `software/calib_data/` with every firmware constant. The tests that move the
robot wait 2 s (START or STOP cancel).

| Test | What to do | Calibrates |
|---|---|---|
| `CAL NOISE [ms]` | robot still | sensor noise; square to a wall and centred in a cell: `FRONT_SQUARE_OFFSET_MM` |
| `CAL STRAIGHT [cells] [mm/s]` | in a corridor; then `/note measured <mm> mm` | distance per cell, centring |
| `CAL TURN [±quarters]` | in place; then `/note angle <deg>` | `TURNTICKS`, overshoot |
| `CAL CURVE [±1] [mm/s]` | a cell, a smooth curve, a cell | exit lateral and stop, `CURVE_ANGLE` |
| `CAL STEP [pwm] [ms]` | free space ahead | motor model, braking |
| `CAL IR [mm]` | against a wall ahead; `/note start <mm> mm` | the front IR curve |
| `CAL RUN` | before a run | records its next continuous move |
| rounds of `CAL NOISE` + `CAL TURN 1` (x4) | in a cell closed on 3 sides, centred front to back; a round per position, moving the robot 10-15 mm sideways between rounds | slope and centre (`SIDE_CENTER_L/R_MM`) of the side IR |
| `CAL DUMP` | — | sends the last recording again |

```
ratatron calib software/calib_data/*.csv
```

summarises every file and, with the notes, suggests concrete values (for
example new IR coefficients ready for `infrared.c`). `--delay` measures every IR's
delay from in-place turns and wall approaches (docs/testing.md).
