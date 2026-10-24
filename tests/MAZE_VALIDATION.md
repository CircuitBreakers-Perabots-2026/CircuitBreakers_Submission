# Maze speed validation

Tested in Webots R2025a with the saved `worlds/MazeTest.wbt` starting pose,
`MODE = "maze"`, and a 16 ms controller step. Times below are simulation
time measured by the controller when it first activates the final LED;
they exclude time spent holding at the finish. Fast simulation playback
does not change these measurements.

| Controller | Completion time | Cells explored | Blue / green | Final state | Peak absolute roll or pitch |
| --- | ---: | ---: | --- | --- | ---: |
| Before speed tuning | 3155.632 s | 36 | 2 / 3 | End cell (3,8), green LED | 0.03 degrees |
| Faster profile, run 1 | 996.672 s | 36 | 2 / 3 | End cell (3,8), green LED | 0.03 degrees |
| Faster profile, reset and repeat | 996.672 s | 36 | 2 / 3 | End cell (3,8), green LED | 0.03 degrees |

The measured completion speedup is 3.17 times (68.4% less simulation time).
The baseline used the committed floor-detection fix plus the same timing
and tilt instrumentation. The faster runs also avoid returning to the
entrance once every grid cell has been scanned.

## Settings

- Forward wheel-speed cap: 2 rad/s, or 0.04 m/s with the fixed 0.02 m wheel radius.
- Wheel-command acceleration: 1 rad/s squared.
- Turn wheel-speed cap: 1 rad/s, with a 0.25 s initial settling period.
- Both actions limit approach speed using remaining stopping distance/angle.
- The existing distance/heading/tilt fault checks, 2 mm movement tolerance,
  2 degree turn tolerance, and stopped hold periods remain enabled.
- No chassis, wheel, motor or physics properties were changed by this tuning.

## Repeating the checks

1. Open `worlds/MazeTest.wbt` and reset to its saved starting pose.
2. Run until the console prints `FINISHED` followed by `RUN RESULT`.
3. Confirm both counts, end-cell placement, green LED, elapsed time and peak tilt.
4. Reset and repeat.

Run automated checks from the repository root:

```sh
python3 -B -m unittest discover -s tests
```

All 49 tests pass. Motion tests cover braking to a cell centre, acceleration
limits, left/right turns across yaw wrap, and immediate fault stops at
16, 32 and 64 ms steps. Their wheel/heading model is ideal; the real Webots
runs above check the supplied development maze. Other maze layouts and
the Section 1 obstacle course were not simulated in this validation.
