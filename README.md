# CircuitBreakers PeraBots controller

The controller combines autonomous obstacle navigation and maze exploration
in the required `controllers/main_controller/` folder.

## Run in Webots R2025a

1. Open `worlds/Arena.wbt`.
2. Reload or reset the world to its saved starting pose.
3. Run the simulation. The default `full` mode selects the starting section
   from the floor camera. No code changes are needed between attempts.
4. Section 1 explores a range-sensor occupancy map, plans with A*, detects
   the transition opening, centres between its walls, and measures the
   red-to-black floor edge to position the robot in the first maze cell.
5. Only a verified entrance permits the continuous handoff to Section 2.
6. Section 2 maps and explores the maze, counts each coloured cell once,
   plans its final route, and stops fully inside the end cell with the
   majority LED on.

For a dedicated Section 2 test, open `worlds/MazeTest.wbt` and run. Its black
starting floor selects maze mode automatically. The starting robot must be
at the designated first-cell centre facing into the maze.

Optional `controllerArgs` on the robot, for development:

```vrml
controllerArgs [ "--mode=section1" ]
```

Modes are `full` (default, including automatic section selection), `maze`
(explicit dedicated maze attempt), and `section1` (stop after verifying
the entrance).

## Navigation and rulebook constraints

- Section 1 assumes the supplied starting-zone centre, with the robot
  facing along the long arena side and the field to its right. Known
  arena dimensions and starting-zone size define an odometry frame.
  No obstacle positions, transition coordinate, or fixed route are stored.
- Six permitted distance sensors supply actual obstacle observations.
  A short initial scan fills gaps between beams. Further scans occur only
  when planning or recovery needs new information.
- The planner retains its destination while it remains reachable and
  keeps robot clearance at every fallback level.
- Entrance alignment uses the sensed corridor and camera edge, with
  encoder/IMU measurements. Failures stop the robot and prevent a false
  successful handoff.
- The final stop is 30 mm before the end-cell centre, within its clear
  internal area. This avoids waiting until the centre after the entire
  robot has entered the cell. A conservative encoder-based timer starts
  before nominal full-body entry and checks the three-second final
  stop/LED budget.
- The supplied red end marker is detected by the camera; blue and green
  locations and totals are learned during the run. A tie or missing end
  observation produces a diagnostic stop rather than a guessed LED.
- Arena.proto and Obstacles.proto retain their original contents.
  The integration adds only permitted sensors in extensionSlot; it does
  not change locked mechanics, LEDs, collision shapes, or physics.
  The previous small balance adjustment to the drive-wheel positions is
  retained, as permitted by rulebook Section 4.4.
- Runtime control uses no GPS, Supervisor, emitter, receiver, downloaded
  map or external command. All helper modules ship in main_controller.

## Team history

Section 1 mapping, inflation, A* and path smoothing were adapted from
[Perabots-section01](https://github.com/CircuitBreakers-Perabots-2026/Perabots-section01),
commit `ffcfbcee353edae30a103f42b2539c28cbfaaea8` (Dewmi).

The integration is a merge with both repositories as parents. The original
teammate commit retains its author, date, message, tree and hash; it has
not been squashed or rewritten. Integration adaptations preserve the
tested Section 2 base and correct sensor geometry, transition discovery,
handoff validation and repeated scanning.

## Automated checks

```sh
python3 -B -m unittest discover -s tests
```

The tests cover colour detection, mapping, shortest paths, motion and
braking, section selection, successful and failed handoffs, sensor mount
offsets, openings at different positions, obstacle clearance, scan
coverage, and the final-cell timing budget. See `tests/VALIDATION.md`
for measured Webots results and their scope.

## Submission

Use Webots R2025a. Keep the required filenames and folder layout:

```text
CircuitBreakers_Submission/
  controllers/main_controller/main_controller.py
  controllers/main_controller/  (all helper modules)
  protos/Arena.proto
  protos/Obstacles.proto
  protos/TwoWheelRobot.proto
  worlds/Arena.wbt
  worlds/MazeTest.wbt
```

Create an archive from committed main, without caches or local preferences:

```sh
mkdir -p dist
git archive --format=zip --prefix=CircuitBreakers_Submission/ --output=dist/CircuitBreakers_Submission.zip main
```

Extract the archive into a fresh folder and check Arena.wbt and MazeTest.wbt
before uploading. The default controller detects which section it starts in;
no controller edits are needed for dedicated maze attempts.

The rulebook lists 28 October as the Phase 1 deadline. It says the final
archive format, outer filename and submission link will be announced through
the official PeraBots channels. Check that notice for the closing time and
any naming/format requirements, then upload through the official submission
form and keep its confirmation. Internal filenames must remain unchanged.
