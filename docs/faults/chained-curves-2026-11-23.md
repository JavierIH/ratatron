# Chained curves and short final straights: the centring (simulator, 2026-11-23)

The firmware of develop `ed3d518` (no firmware change since 11-18) in
ratasim `f616de5`, to find the speed run's common trajectory errors and
fix them. Simulator only: nothing here has run on the robot yet (section 7).

## 1. Method

- 108 mazes: every fifth of ratasim's 525 classic ones (sorted by name),
  plus `oshwdem2026`, `spiral` and `stairs`. Per maze a search, then races
  2.3-2.6, each run with `--trace` (the true pose every 10 ms) besides the log.
- From the trace and the maze: the hull's distance to the walls and posts
  (the collision box, 72 x 74 mm, -29 to +45 mm from the axle), the lateral
  offset and yaw on straights, every race curve at its gates (85.5 mm before
  and after its cell's centre, the cells taken from the robot's own path),
  and the pose when the firmware says "Goal reached in the speed run".
- Variants run only the races on the maps the search saved (`ratasim batch
  --maps`). A race that fails ends that maze's sequence, so counts are
  failed of run. The control, the same firmware run that way, failed 17 of
  303 races where the full run had failed 14 of 305: two runs of one
  firmware differ by a few failures, so only bigger differences count.
  `001` and `001-anomaly-test` have no path to the goal: not counted.

## 2. What the trajectory showed

- The search and race 2.3 (no curves) are clean: straights |lateral| p95
  2-3 mm, the hull never nearer a wall than 22 mm (p5 of each run's minimum).
- The straights re-centre within one cell: 9 mm off at a curve's exit, 5.7
  at +90 mm, 0.5 at +180. A curve after one cell of straight or more is
  entered within 1.2-2 mm (p95).
- Curves in consecutive cells (staircases, zigzags: half of all race
  curves) have no straight between them (9 mm), and the centring was blind
  there: in a curve, until the delayed IR read the new corridor, and faded out
  over the last 40 mm before the next curve. Each curve's error (entry yaw,
  slip) carried into the next: |lateral| entering the k-th curve of a chain,
  median 0.3 mm at the first, 3-5 at the 2nd-4th, 8 at the 10th; p95 1.3,
  14-16, 35.
- Every hull contact (15 runs) was in or just after a chain. Most "failed
  returns" were outbound crashes the firmware did not notice: pushed against
  a wall, the wheels slipping, the path slowed to its minimum pace (34 %),
  the encoders completed the route and it reported the goal; the return then
  started from the wrong cell.
- Stops: when the last curve is one cell before the stop (94.5 mm), the
  robot arrived yawed, |yaw| p90 3.6 deg (up to 29); with two cells or more
  0.5. The centring begins a correction there that the stop cuts in the
  middle (KP 8 turns ~10 deg in 0.14 s for 10 mm), and the next move starts
  crooked: in `stairs` the return's first curve then `SLIPPED`.

## 3. Tried and dropped

| Variant | failed 2.4 / 2.5 / 2.6 | of | contacts |
|---|---|---|---|
| control | 6 / 6 / 5 = 17 | 303 | 32 |
| offset held into a curve 0 deg (2) | 6 / 11 / 6 = 23 | 298 | 31 |
| offset held, no clamp | 8 / 12 / 2 = 22 | 293 | 44 |
| KP 12, KD 0.75 | 6 / 6 / 3 = 15 | 303 | 31 |
| KP 16, KD 0.85 | 5 / 11 / 8 = 24 | 300 | 35 |

The offset the centring holds through a curve does carry real drift (0 deg
is no better). Higher gains re-centre the straights faster (+90 mm: 4.4 and
3.5 mm instead of 5.7) without fewer failures: the straights were not the
problem.

## 4. The changes (behaviour, no new offset)

1. The centring also runs in the windows between chained curves: over a
   curve's first and last `STEER_CURVE_EDGE_MM` (the clothoid is there less
   than 1 deg off the axis, the side IR read as on a straight), and with no
   fade before a curve in the next cell (the few mm between two curves are
   all it gets). The window's size:

   | `STEER_CURVE_EDGE_MM` | failed 2.4 / 2.5 / 2.6 | contacts | chain entry \|lat\| p95, 10th curve |
   |---|---|---|---|
   | (control) | 6 / 6 / 5 = 17 | 32 | 35 mm |
   | 8 | 0 / 2 / 2 = 4 | 4 | 17 mm |
   | 15 | 1 / 4 / 2 = 7 | 8 | 18 mm |
   | 25 | 4 / 6 / 1 = 11 | 10 | 22 mm |

   8 mm: as good as 15 and the least into the curve. Times to the goal
   unchanged (paired, the same mazes: +0.01 %).
2. After the route's last curve, a straight shorter than a cell (the
   distance a correction takes, above) only aligns with the walls: no P
   term, the D term turns the robot parallel. Arrival |yaw| p90 in 2.4 / 2.5
   / 2.6: control 3.4 / 3.7 / 4.4 deg, with change 1 alone 2.0 / 2.4 / 4.2,
   with both 0.8 / 1.4 / 1.9; arrivals over 4 deg in 2.6: 12, 13, 1. The
   lateral offset at the stop grows a little (p90 in 2.6: 12, 10, 14 mm),
   harmless: the next move starts its own centring. Both changes: failed 2
   (0 / 2 / 0), poses lost 26 (control) -> 5 -> 2.

Flash +172 B (competition 83.7 -> 84.0 %, practice 92.6 -> 92.8 %), RAM
unchanged.

## 5. Validation

The two changes against develop, the same maps (528: every classic maze,
the maps of one search with this firmware; the races of develop with
`--maps`):

| Run | Firmware | failed 2.4 / 2.5 / 2.6 | of | % | contacts | poses lost |
|---|---|---|---|---|---|---|
| 528 mazes | develop | 29 / 33 / 18 = 80 | 1487 | 5.4 | 142 | 108 |
| | this | 0 / 9 / 3 = 12 | 1569 | 0.8 | 18 | 15 |
| 108, `--slip 6` | develop | 26 / 4 / 8 = 38 | 265 | 14.3 | 58 | 44 |
| | this | 3 / 2 / 3 = 8 | 313 | 2.6 | 13 | 10 |

- Times to the goal, paired (445 mazes where both reach it): 2.4 -0.02 %,
  2.5 +0.01 %, 2.6 0.00 %.
- Arrivals (528): |yaw| p90 develop 3.5 / 3.8 / 4.5 deg, this 0.8 / 1.3 /
  2.0; over 4 deg 42 / 41 / 60 against 3 / 5 / 5.
- Chained curves: |lateral| entering, p95, 15.6 / 16.0 / 17.1 mm against
  8.3 / 10.2 / 11.0; at the exit of every curve 14.2 / 14.6 / 15.5 against
  8.5 / 9.5 / 10.8.
- `--slip 6` (the run-up of 11-17: the encoder heading 6 deg off the walls
  in the first straight) rehearses the open question of
  `race-slip-2026-11-17.md` 4, the 2 deg clamp with a real encoder heading
  error: the centring in the windows takes most of it out (develop's 2.4
  failed a quarter of its races). The clamp itself was not varied with the
  slip.
- Develop's first contacts (47 of its failed races): 42 in a chain of
  curves or right after one.

## 6. What remains (528 mazes, 12 failed races)

11 of the 12 mazes fail in develop too (at the same race or before).

- 5 in chains of 4-6 curves: the inner post, mid-curve (`hitel52`,
  `mont88a`, `shiba`, `test-pathological`, `us01`).
- 5 on open straights after a curve: 5-9 cells with no side wall
  (`namae`, `taiwan2017f`, `kilkai`, `killer`; the return of `japan2000`).
  The curve leaves ~2 deg (`kilkai`: entered centred and parallel, left 28
  mm to the inside and -2 deg), and with no wall to centre on that is dead
  reckoning: 46 mm in 1.4 m at 900, into a post. A behaviour to look at:
  how a single curve at 400 ends that far inside (along-track error at its
  start, or slip).
- 2 returns that start from a bad arrival (`portugal-2017-final-expert`:
  at the goal 34 mm off and 9 deg; `japan2013ef`).

## 7. Next, at the robot

Layout E (`docs/mazes.md`), the staircase of six curves where 2.4 hit the
inner post on 11-17 (`race-slip-2026-11-17.md`): races 2.4 and 2.5 with
`CAL RUN`, the lateral entering the last straight (`in`) against that table,
and the yaw at the goal (`FL - FR` after the front alignment).
