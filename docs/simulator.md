# RATATRON in MuJoCo

The robot's firmware (`firmware/src`, unchanged) driving the CAD robot in a 16x16 micromouse maze:
black floor, white walls 50 mm high and 12 mm thick, red tops, 180 mm cells; 12 mm posts with a 3 x 3 mm slot
in every face, the walls ending in a 3 x 3 mm tongue that fits it (an empty slot reads 3 mm deeper on the IR).

```
python3 -m venv sim/.venv && sim/.venv/bin/pip install -r sim/requirements.txt
sim/.venv/bin/python sim/run.py                     # viewer + console (stdin); random maze, seed 1
sim/.venv/bin/python sim/run.py --maze file.txt     # a maze file (github.com/micromouseonline/mazefiles format)
sim/.venv/bin/python sim/run.py --headless --speed 0 --erase-flash \
    --cmd "MODE 1" --cmd START --cmd :wait-idle --cmd "MODE 2 6" --cmd START --until-idle
sim/.venv/bin/python sim/run.py --headless --speed 0 ... --record run.mp4   # video (ffmpeg)
```

The library builds itself (`make -C sim/fw`, also `PRACTICE=1`) when a firmware file changed.
Console lines go to the firmware as over Bluetooth (`MODE 1`, `START`, `STOP`, `MAP`, `STATUS`, `CAL ...`);
`:start` / `:select` press the buttons, `:place x y N`, `:reset`, `:erase-flash`, `:speed f`, `:wait-idle`
(scripts), `:quit`. Viewer: Page Up = SELECT, Page Down = START, End = back to the start.
The monitor works as with the robot: `python3 software/robot_monitor.py --port sim/build/tty` (a link to a
pseudo-terminal the simulator opens; `--no-pty` without it). `--poses` prints the true pose at every
pose the firmware reports; "POSE LOST" whenever they disagree, "touches a wall" when the hull does.
The flash (saved map) lives in `sim/build/flash.bin` between runs, as on the robot.

`sim/tools/demo_monitor.py out.mp4` records the simulator and the monitor together with no screen (the monitor in
a pseudo-terminal, read back with pyte; its commands typed into its console).

The viewer needs a monitor: with none (a remote session) GLFW aborts; use `--headless` (and `--record`).

## How it works

- `fw/`: the firmware as a shared library. `main.c`, `motion.c`, `control.c`, `path.c`, the strategy files and
  the `gpio.c` / `infrared.c` drivers compile as they are against a fake HAL (`fw/hal/`); `sim_drivers.c`
  replaces the motor, encoder, UART, flash, clock and health drivers. `main()` runs in a coroutine: it yields
  every 4 calls of `HAL_GetTick()` (every waiting loop calls it) and in `HAL_Delay()`, then the simulator steps
  1 ms and runs the SysTick handler (`app_systick()`). Computation takes no simulated time. A reset loads a new
  copy of the library (statics from zero). The UART never drops (the real one does beyond its queue).
- `ratsim/robot.py`, every ms: wheel angles -> encoder ticks (`WHEEL_TICKS_PER_MM`, `WHEEL_DIFF`); IR rays ->
  readings -> ADC counts by inverting the firmware's own calibration (`ir_mm_of`) into the DMA buffer;
  firmware 1 ms; PWM -> motor torque; two physics steps of 0.5 ms.
- Motors: torque `b (w_target - w)`, `w_target` from the firmware's own model (`MOTOR_KV_*`, `MOTOR_KS_PWM`),
  `b` giving the loaded robot `MOTOR_TAU_S`; below 85 PWM a still wheel stays still (as `control_sim.c`).
- IR (Sharp GP2Y0A51SK0F, positions and directions from the CAD): three rays (the spot, 2.5 deg apart, mixed in
  1/d), reading = beam length + an offset per sensor so that the robot centred in a cell reads the firmware's
  references (`SIDE_CENTER_*`, `FRONT_WALL_REF_MM`, `FRONT_SQUARE_OFFSET_MM`); each sensor measures every 16 ms
  and holds, noise 0.8 mm.
- Robot (`ratsim/config.py`, `ratsim/model.py`): CAD meshes in the assembly frame
  (`tools/export_cad.py`, FreeCAD headless). Collisions: two 2 mm tire bands at the contact track that gives a
  90 deg in-place turn its 403 ticks (`TURNTICKS`), a box hull, front and back skids at the chassis' lowest points.

## Calibrated against the robot, assumed, open

| Item | Value | Source |
|---|---|---|
| Front stop (`front aligned err`) | median -5.4 mm (sim) vs -5.3 (44 real stops) | IR hold only; an extra delay of 6-14 ms gave -7..-11 |
| In-place turn | body = encoders | tire bands `condim 3` (torsional friction under-turned 20 deg) |
| Curve slip | ~2 deg per 90 deg at 500 mm/s (open loop) | firmware `CURVE_SLIP_DEG` 2 at 480 |
| Mass | 100 g | **assumed**: to be weighed |
| Centre of mass | on the axle (y +0.5, z +8 mm) | **estimated** from the CAD by parts: to be measured. 6 mm ahead made the 2.5/2.6 races crash |
| Tire friction | 0.6 | assumed; 0.8-1.2 changed nothing in the races |

Results (generated maze, seed 1): search to the centre, OPTIM, back, saved, no pose lost (yaw within 3.8 deg);
races 2.4, 2.5 and 2.6 clean there and back. On the robot 2.6 is not safe (`docs/testing.md`): the simulator is
more optimistic in fast curves until the mass and centre of mass are measured.
