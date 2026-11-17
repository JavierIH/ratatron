# The simulator

The robot's firmware (`firmware/src`, unchanged) driving the CAD robot in MuJoCo, in a 16x16 maze by the rules:
black floor, white walls 50 x 12 mm with red tops, 180 mm cells, 12 mm posts with a 3 x 3 mm slot in every face.
Code: `software/ratatron/sim/`.

```
ratatron sim                         # viewer + console on stdin; a generated maze (seed 1)
ratatron sim --maze oshwdem2026      # a maze of sim/mazes, or any micromouseonline maze file
ratatron sim --headless --speed 0 --erase-flash --cmd "MODE 1" --cmd START --until-idle
ratatron connect --sim               # then ratatron monitor / ratatron send, as with the robot
```

- Console lines go to the firmware as over Bluetooth; `:start`, `:select` press the buttons, `:place x y N`,
  `:reset`, `:erase-flash`, `:speed f`, `:wait-idle` (scripts), `:quit`. Viewer: Page Up = SELECT,
  Page Down = START, End = back to the start. The viewer needs a screen: else `--headless` (`--record out.mp4`).
- `--poses` prints the true pose at every pose the firmware reports: "POSE LOST" when they differ, "touches a
  wall" when the hull does. The flash (saved map) persists in `software/ratatron/sim/build/flash.bin`.
- `--slip DEG` (also in `batch`): in the first 100 mm of every move from rest one encoder counts DEG of encoder
  heading (+ left) more than its wheel rolls; `--slip 6` gives the robot's run-up slip at `ACCEL` 3000 (11-17).
- Studies: `ratatron sim batch` (many mazes; `--maps` reuses saved searches for race-only changes, `--practice`),
  `curves` (every curve against the cells), `failures`, `mapdiff`, `ir-info` / `real-ir` (the IR in curves),
  `demo` (a video of the simulator and the monitor together).

## How it works

- `fw/`: the firmware as a shared library: the application files and the `gpio.c`/`infrared.c` drivers compile
  as they are on a fake HAL; `sim_drivers.c` replaces motors, encoders, UART, flash, clock and health. `main()`
  runs in a coroutine that yields in `HAL_GetTick()`/`HAL_Delay()`; every simulated ms runs SysTick.
  Computation takes no simulated time; the UART never drops.
- Every ms: wheel angles to encoder ticks (`WHEEL_TICKS_PER_MM`, `WHEEL_DIFF`), IR rays to ADC counts through
  the firmware's own calibration, 1 ms of firmware, PWM to motor torque (the firmware's motor model,
  `MOTOR_TAU_S`), two physics steps of 0.5 ms.
- IR (Sharp GP2Y0A51SK0F where the CAD puts them): three rays, a reading every 16 ms held, 0.8 mm noise, an
  offset per sensor so the firmware's reference readings hold centred in a cell.
- Robot: CAD meshes (`export_cad.py`, FreeCAD headless: every part visible in
  `mechanics/freecad/ratatron.fcstd`, one mesh per colour); collisions: 2 mm tire bands at the track that gives a
  90 deg turn its 403 ticks, a box hull, the CAD's skids (R 13 mm caps at y +26 and -23 mm, 0.3 mm off the
  floor).

## Calibrated, assumed, open

- Matched to the robot: front stops (median -5.4 mm vs -5.3 on 44 real stops), in-place turns, curve slip
  (~2 deg per 90 at 500 mm/s).
- **Assumed, to be measured** (issue 24): mass 100 g, centre of mass on the axle (from the CAD by parts), skid
  gap 0.3 mm (the CAD's, tires unloaded), tire friction 0.6. Fast curves depend on them: the simulator is more optimistic than the robot.
- The 525 real mazes (11-17): searches 523/525 (2 goals unreachable), and what the races taught is in
  `docs/ISSUES.md`, issue 24.
