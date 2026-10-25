# Section 1 / Section 2 integration validation

Validated on 25 October 2026 with Webots R2025a, a 16ms controller step,
the saved starting poses and default `full` mode. Times are measured
simulation time, independent of fast playback.

## Full-course runs

| Layout | Section 1 time | Section 1 distance | Maze time | Cells / blue / green | Final LED | Conservative final response |
| --- | ---: | ---: | ---: | --- | --- | ---: |
| Supplied Arena.wbt, reduced scanning | 228.544 s | 3.576 m | 995.920 s | 36 / 2 / 3 | Green, stopped at end | 2.752 s |
| Separate practice obstacle arrangement | 245.968 s | 4.466 m | 995.920 s | 36 / 2 / 3 | Green, stopped at end | 2.752 s |

Both runs verified lateral centring, heading, and the first maze-cell
position before the autonomous handoff. Peak Section 1 tilt was 0.04
degrees; peak Section 2 tilt was 0.03 degrees.

A separate `MazeTest.wbt` run, using unchanged default `full` mode,
automatically selected `maze` at startup and completed in 995.920 seconds.
It visited all 36 cells, counted 2 blue / 3 green, stopped with the green
LED on, and reported a 2.752-second final response and 0.03-degree peak tilt.

Before reducing repeated scans, the supplied field took 457.600 seconds.
The final version takes 228.544 seconds, about half as long. It uses one
initial quarter-turn scan, retains reachable destinations when replanning,
and performs additional scans only if planning or recovery needs them.

The practice project was created under `/private/tmp/` with 13 separate
10cm cubes arranged in three alternating barriers. It used the same
controller and the unmodified supplied Arena.proto, with a separate
PracticeObstacles prototype. It is a test fixture, not part of the
submitted project. Neither supplied locked prototype was edited.

The response timer begins at 0.160m encoder progress on the final hop,
earlier than nominal full-body entry at approximately 0.171m. This allows
about 1cm of longitudinal positioning uncertainty. The final 0.220m hop
stops about 3cm short of the cell centre, leaving the whole body inside
the clear area and time to stop and activate the LED within three seconds.
This is an encoder/geometry check, not a Supervisor measurement of pose.

## Automated checks

All 64 tests pass:

```sh
python3 -B -m unittest discover -s tests
```

The dedicated-maze startup, failed handoff gate, absolute entry heading,
different opening positions, A* obstacle avoidance, nonzero clearance,
actual sensor offsets, quarter-turn beam coverage, and end-cell timing
have explicit regression coverage. Motion limits are tested at 16, 32,
and 64ms controller steps.

## Preserved assets and history

SHA-256 of the locked project assets, matching the pre-integration main:

```text
Arena.proto      23bb0016ce182e6b71597df623da742a37c9525f13dede044a327745a4f9e82b
Obstacles.proto  9c5aee2f03eddd2698fa6b2a5212e3f21fa500027721b4f310cb9bc8508b5401
```

The merge retains upstream Section 1 commit
`ffcfbcee353edae30a103f42b2539c28cbfaaea8` unchanged, including authorship.
The integration adapts its mapping/planning algorithms and keeps the
previously tested maze code. No history is squashed or force-pushed.

These tests cover the supplied maze and one changed obstacle layout.
They do not establish performance for every possible official layout.
The declared starting-pose/frame assumptions are documented in README.md.
