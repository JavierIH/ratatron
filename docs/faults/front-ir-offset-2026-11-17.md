# The front IR read ~6 mm short of their reference (found 2026-11-17)

The front pair reads ~6 mm less than `FRONT_WALL_REF_MM` (94) says a robot
centred in a cell reads. So every stop aimed by the front IR ends ~6 mm before
the cell centre, farther from the wall. Not validated on the robot yet. The
analysis below is from recordings only.

## 1. How it showed up

The IR sweep of 11-17 (`docs/testing.md`, session of 11-17, point 6;
`software/calib_data/2026-11-17_22-51-15_turn.csv` and `22-52-25`): the robot
in (3,0) of layout E, placed by hand, one slow 90 deg turn from north to the
east wall and back. Against the simulator's sensor model at the cell centre:
SL/SR within 3 mm, FL/FR a steady -6 to -9 mm at 30-90 deg.

## 2. Position or calibration

- **The sweep alone** (the hand placement fitted on a 1 mm grid from some
  sensors through `real_ir_check`'s `load` and `curve_ir_info.predict`, then
  the others' residuals at that pose). From the sides: the robot 1 mm east,
  7 mm south of the centre, then FL -6.7, FR -7.3. Explaining the fronts by
  position instead puts the robot 7 mm east and 15 mm south, and then SL and SR
  must be off by -7 and +8 in opposite directions. That would run every
  corridor 7 mm off the centre line, which the centring tests do not show.
- **Without the placement** (`CHECK`, the lighting routine, 10-08, log
  `software/logs/2026-10-08_19-00-58.log`): it reads each front sensor facing
  the east and the west walls of the start cell. Their mean does not depend on
  where the robot stands. Three runs: FL -3.8 / -1.9 / -2.9 mm, FR -9.9 / -8.8 /
  -7.2. The pair's mean is -5.8, so a centred robot reads ~88, not 94. Sides,
  the same way: SL -1.1 to -3.0, SR +1.9 to +4.8. The FL/FR split depends on
  the yaw the turns leave (89.2 deg each); the mean does not.
- **Both agree.** The sweep's -6 to -9 is the -5.8 of the calibration plus the
  placement: SR on the east wall puts the robot 1-2.5 mm east of the centre.
- **The race ends** (11-17, `end=IR`, at rest): FL 83 / FR 99, mean 91,
  aimed at `FRONT_TRACK_REF_MM` 92. In the sweep, facing the same kind of wall
  from 1-2.5 mm east of the centre: FL 78 / FR 92, mean 85. So a race ends
  ~4 mm before the centre, which fits a centred reading of ~88.
- **The cubics' scale** (the slow approaches `CAL STRAIGHT 3` at 100 and 300
  mm/s of 10-08, five runs; each reading, 22 ms late, against the final
  reading plus the encoder distance still to go, a line fitted): from 85 to 150 mm FL has a slope of 0.95-1.00, FR of 1.05-1.12.
  FR's cubic is ~10 % steep: 40 mm farther than the stop, it reads 4-6 mm
  long. This is also why FR's apparent IR delay is the longer one (24-32 ms
  against FL's 16-24, `docs/measurements.md`).
- **Where the error comes from.** The cubics in `infrared.c` date from
  2024-10-16 and nothing in the repo records how they were fitted.
  `firmware/src/calib.txt` is not their data: the cubics miss its tables by
  up to 17 mm at 70-110 mm, so it is obsolete (kept, marked to be ignored).
  The CAD (`software/ratatron/sim/config.py`) puts the sensors' faces 54.9 mm
  from the wall at the centre. The readings carry per-sensor offsets of
  +5 to +38 mm on top of that, so the calibration's zero is not the face. The
  references absorb those offsets, but only where they were taken.

## 3. What it does to the robot

- The search's front alignment (`motion_align_front()`, aimed at 94) and every
  `end=IR` stop (aimed at 92) leave the robot ~4-6 mm before the centre. The
  next in-place turn turns that into a lateral offset, which the centring
  removes during the next cell.
- A race started after the return starts ~4 mm ahead of the centre. The
  route's curves then come ~4 mm late, on the outer side, the opposite of the
  inner-post hits, which are slips (`race-slip-2026-11-17.md`).
- The simulator builds its sensor offsets from the same references, so it
  has no such error and cannot show it.

## 4. Reproduce and check on the robot

- With a ruler: stop facing a wall with `end=IR`, or after a search's front
  alignment, then measure the gap from the nose to the wall. The CAD's nose
  is 45 mm ahead of the axle and the axle sits 84 mm from the wall at the
  centre, so a correct reference leaves 39 mm after the alignment (94) and
  37 mm after an `end=IR` stop (92). A front pair reading 6 mm short leaves
  ~45 and ~43 mm.
- Or the placement-free turn: in the start cell facing north, `CAL TURN 1`
  four times (north, east, south, west), `CAL DUMP` after each. The front
  mean facing east and facing west is the centred reading.

## 5. Fix, once confirmed

- `FRONT_WALL_REF_MM` 94 to ~88 and `FRONT_TRACK_REF_MM` 92 to ~86. Their
  difference stays as measured. `SIDE_CLOSE_DOUBT_MM` and the doubt rules are
  relative to the reference.
- Then check on the robot that the stops, ~6 mm nearer the wall, do not bring
  the side sensors' phantom walls back. The side beams leave the cell by the
  post when the robot stands too far forward (AGENTS.md, hardware).
- FR's scale: refit both front cubics from a ruler-measured approach,
  readings every 10 mm from 40 to 170 mm. Only if the stops still scatter
  after the reference change.
