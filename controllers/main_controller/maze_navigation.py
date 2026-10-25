"""Maze exploration using sensor mapping and DFS backtracking."""

import math
from statistics import median

from floor_detection import classify_floor_rgb, read_floor_rgb
from floor_map import (
    FLOOR_SAMPLES,
    FloorMap,
    FloorObservationConflict,
    DuplicateEndCell,
    bfs_path,
    consensus,
)
from movement import MoveForward
from turning import Turn90, angle_difference


ROWS = 4
COLUMNS = 9
CELL_DISTANCE = 0.25
# The final stop is 30mm before the cell centre, safely inside the 238mm
# clear area. This avoids spending the 3-second LED deadline creeping all
# the way to the centre after the whole body has already entered the cell.
FINAL_CELL_DISTANCE = 0.22
# Start the deadline conservatively, allowing roughly 1cm of odometry error.
FINAL_ENTRY_DISTANCE = 0.16

# Heading order: north, east, south, west.
# Row zero is the entrance-side row.
DIRECTIONS = [
    (1, 0),
    (0, 1),
    (-1, 0),
    (0, -1),
]

HEADING_NAMES = ("N", "E", "S", "W")

MAX_TILT = math.radians(10)
SCAN_DELAY = 0.3
SCAN_SAMPLES = 12
FLOOR_SCAN_ATTEMPTS = 3
FLOOR_RETRY_DELAY = 0.25


class CardinalTurn(Turn90):
    """Use the existing turn controller with an absolute heading target."""

    def __init__(self, direction, start_time, target_yaw):
        super().__init__(direction, start_time)
        self.absolute_target = target_yaw

    def update(self, now, yaw):
        if (
            self.target_yaw is None
            and now - self.start_time >= self.settle_seconds
            and math.isfinite(yaw)
        ):
            self.start_yaw = yaw
            self.target_yaw = self.absolute_target
            self.turn_started = now

        return super().update(now, yaw)


def inside(cell):
    row, column = cell
    return (
        0 <= row < ROWS
        and 0 <= column < COLUMNS
    )


def neighbour(cell, heading):
    row_change, column_change = DIRECTIONS[heading]
    return (
        cell[0] + row_change,
        cell[1] + column_change,
    )


def classify_wall(distance):
    if distance < 0.12:
        return True

    if distance > 0.20:
        return False

    raise RuntimeError(
        f"Uncertain wall distance: {distance:.3f} m. "
        "Check cell centering and sensor alignment."
    )


def run_maze(robot, time_step=16, *, entry_heading=None):
    left_motor = robot.getDevice("left_wheel_motor")
    right_motor = robot.getDevice("right_wheel_motor")

    for motor in (left_motor, right_motor):
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    robot.getDevice("led_blue").set(0)
    robot.getDevice("led_green").set(0)

    sensors = {
        "front": robot.getDevice("front_distance"),
        "left": robot.getDevice("left_distance"),
        "right": robot.getDevice("right_distance"),
    }

    for sensor in sensors.values():
        sensor.enable(time_step)

    left_encoder = robot.getDevice("left_wheel_sensor")
    right_encoder = robot.getDevice("right_wheel_sensor")
    left_encoder.enable(time_step)
    right_encoder.enable(time_step)

    imu = robot.getDevice("imu")
    imu.enable(time_step)

    # Downward camera for floor-colour classification.
    camera = robot.getDevice("floor_camera")
    camera.enable(time_step)

    floor_map = FloorMap(ROWS, COLUMNS)
    floor_samples = []
    floor_scan_attempts = 0

    # Return-leg state.
    return_path = []   # BFS hops remaining (cells after current, in order)
    returning = False
    led_done = False
    final_entry_started = None

    # Handles for the LEDs (same devices initialised below).
    led_blue = robot.getDevice("led_blue")
    led_green = robot.getDevice("led_green")

    # Each cell stores N/E/S/W walls:
    # None = unknown, True = wall, False = opening.
    walls = {
        (row, column): [None, None, None, None]
        for row in range(ROWS)
        for column in range(COLUMNS)
    }

    visited = set()

    current = (0, 0)
    heading = 0
    north_yaw = None

    # Active DFS path from the entrance to the current cell.
    path = [current]

    destination = None
    destination_heading = None
    backtracking = False

    turn_heading = None
    action = None

    state = "SETTLING"
    state_started = robot.getTime()
    run_started = state_started
    peak_tilt = 0.0
    scan_samples = []
    last_report = -1.0

    def expected_yaw(direction):
        return angle_difference(
            north_yaw - direction * math.pi / 2,
            0.0,
        )

    def record_wall(cell, direction, blocked):
        updates = [(cell, direction)]
        adjacent = neighbour(cell, direction)

        if inside(adjacent):
            updates.append(
                (adjacent, (direction + 2) % 4)
            )

        # Detect contradictions instead of silently overwriting.
        for target_cell, target_direction in updates:
            previous = walls[target_cell][target_direction]

            if previous is not None and previous != blocked:
                raise RuntimeError(
                    f"Map contradiction at {target_cell}, "
                    f"direction {HEADING_NAMES[target_direction]}"
                )

        for target_cell, target_direction in updates:
            walls[target_cell][target_direction] = blocked

    print("Starting maze exploration at cell (0, 0), facing N.")

    try:
        while robot.step(time_step) != -1:
            now = robot.getTime()

            readings = {
                name: sensor.getValue()
                for name, sensor in sensors.items()
            }

            roll, pitch, yaw = imu.getRollPitchYaw()
            if math.isfinite(roll) and math.isfinite(pitch):
                peak_tilt = max(peak_tilt, abs(roll), abs(pitch))
            left_position = left_encoder.getValue()
            right_position = right_encoder.getValue()

            left_speed = 0.0
            right_speed = 0.0

            if state != "SETTLING":
                values = [
                    roll,
                    pitch,
                    yaw,
                    left_position,
                    right_position,
                    *readings.values(),
                ]

                if not all(math.isfinite(v) for v in values):
                    raise RuntimeError("Invalid sensor reading")

                if not all(
                    0.0 <= value <= 1.0
                    for value in readings.values()
                ):
                    raise RuntimeError("Invalid distance reading")

                if (
                    abs(pitch) > MAX_TILT
                    or abs(roll) > MAX_TILT
                ):
                    raise RuntimeError(
                        "Excessive tilt: "
                        f"pitch={math.degrees(pitch):.1f}, "
                        f"roll={math.degrees(roll):.1f}"
                    )

            if state == "SETTLING":
                if now - state_started >= 1.0:
                    if not math.isfinite(yaw):
                        raise RuntimeError("Invalid starting heading")

                    # The robot must initially face maze north.
                    north_yaw = yaw if entry_heading is None else entry_heading
                    if not math.isfinite(north_yaw):
                        raise RuntimeError("Invalid maze-entry heading")
                    state = "SCAN"
                    state_started = now
                    scan_samples = []

            elif state == "SCAN":
                if now - state_started >= SCAN_DELAY:
                    scan_samples.append(readings)

                    if len(scan_samples) >= SCAN_SAMPLES:
                        relative_directions = {
                            "front": heading,
                            "left": (heading - 1) % 4,
                            "right": (heading + 1) % 4,
                        }

                        for name, direction in relative_directions.items():
                            distance = median(
                                sample[name]
                                for sample in scan_samples
                            )

                            record_wall(
                                current,
                                direction,
                                classify_wall(distance),
                            )

                        visited.add(current)

                        print(
                            f"SCANNED {current}, "
                            f"heading={HEADING_NAMES[heading]}, "
                            f"walls[N,E,S,W]={walls[current]}"
                        )

                        floor_samples = []
                        floor_scan_attempts = 0
                        state = "SCAN_FLOOR"
                        state_started = now

            elif state == "SCAN_FLOOR":
                # A short stationary retry allows transient frames to clear.
                if now - state_started < FLOOR_RETRY_DELAY:
                    continue

                rgb = read_floor_rgb(camera)
                floor_samples.append(classify_floor_rgb(rgb))

                if len(floor_samples) >= FLOOR_SAMPLES:
                    label = consensus(floor_samples)
                    floor_scan_attempts += 1
                    rgb_report = (
                        None if rgb is None
                        else tuple(round(v, 1) for v in rgb)
                    )

                    print(
                        f"FLOOR | cell={current} | "
                        f"samples={floor_samples} | "
                        f"confirmed={label} | "
                        f"rgb={rgb_report} | "
                        f"attempt={floor_scan_attempts}/{FLOOR_SCAN_ATTEMPTS}"
                    )

                    if label is None:
                        if floor_scan_attempts < FLOOR_SCAN_ATTEMPTS:
                            floor_samples = []
                            state_started = now
                            print(
                                f"Retrying floor scan at {current}; "
                                "holding stopped."
                            )
                            continue
                        raise RuntimeError(
                            f"Cannot confirm floor colour at {current} after "
                            f"{FLOOR_SCAN_ATTEMPTS} scans; RGB={rgb}. "
                            "Check the floor-camera image and colour thresholds."
                        )

                    try:
                        floor_map.record(current, label)
                    except (
                        FloorObservationConflict,
                        DuplicateEndCell,
                    ) as exc:
                        raise RuntimeError(str(exc)) from exc

                    print(
                        f"TILES | {floor_map.counts()} | "
                        f"end={floor_map.end_cell}"
                    )

                    state = "PLAN"

            elif state == "PLAN":
                destination = None
                backtracking = False

                # Prefer forward, left, right, then behind.
                priorities = [
                    heading,
                    (heading - 1) % 4,
                    (heading + 1) % 4,
                    (heading + 2) % 4,
                ]

                for direction in priorities:
                    candidate = neighbour(current, direction)

                    if (
                        inside(candidate)
                        and walls[current][direction] is False
                        and candidate not in visited
                    ):
                        destination = candidate
                        destination_heading = direction
                        break

                if destination is None:
                    if len(path) == 1 or len(visited) == ROWS * COLUMNS:
                        # Once every grid cell has been scanned, plan from
                        # here rather than backtracking to the entrance first.
                        print(
                            "EXPLORATION COMPLETE: "
                            f"visited {len(visited)} cells; "
                            f"planning final route from {current}."
                        )

                        # Determine majority before starting the return leg so
                        # the LED result is ready at arrival.
                        majority = floor_map.majority()
                        end = floor_map.end_cell
                        print(
                            f"EXPLORATION RESULT | tiles={floor_map.counts()} | "
                            f"end={end}"
                        )

                        if end is None:
                            print(
                                "MAZE STOPPED: No end-cell candidate found. "
                                "No LED will be lit."
                            )
                            return

                        route = bfs_path(walls, current, end, ROWS, COLUMNS)
                        if route is None:
                            print(
                                f"MAZE STOPPED: End cell {end} is unreachable "
                                "via confirmed passages. No LED will be lit."
                            )
                            return

                        if majority is None:
                            print(
                                "MAZE STOPPED: Tile counts are tied or zero. "
                                "No LED will be lit."
                            )
                            return

                        print(
                            f"RETURNING to end cell {end}; "
                            f"majority={majority}; "
                            f"route length={len(route)}"
                        )

                        # If the robot is already at the end cell,
                        # the BFS path is empty; go straight to FINISHED.
                        if not route:
                            state = "FINISHED"
                        else:
                            returning = True
                            return_path = list(route)
                            destination = return_path.pop(0)
                            diff = (
                                destination[0] - current[0],
                                destination[1] - current[1],
                            )
                            destination_heading = DIRECTIONS.index(diff)
                            state = "ALIGN"
                        continue

                    destination = path[-2]
                    backtracking = True

                    difference = (
                        destination[0] - current[0],
                        destination[1] - current[1],
                    )

                    destination_heading = DIRECTIONS.index(
                        difference
                    )

                    if (
                        walls[current][destination_heading]
                        is not False
                    ):
                        raise RuntimeError(
                            "Backtracking passage is not confirmed open"
                        )

                print(
                    f"Planning {current} -> {destination}; "
                    f"backtracking={backtracking}"
                )

                state = "ALIGN"

            elif state == "ALIGN":
                if heading == destination_heading:
                    action = MoveForward(
                        target_distance=(
                            FINAL_CELL_DISTANCE
                            if returning and not return_path
                            else CELL_DISTANCE
                        ),
                        start_time=now,
                        left_position=left_position,
                        right_position=right_position,
                        yaw=yaw,
                    )

                    # Hold the original cardinal reference,
                    # rather than accumulating relative-turn error.
                    action.target_yaw = expected_yaw(heading)
                    state = "MOVE"

                else:
                    difference = (
                        destination_heading - heading
                    ) % 4

                    # A 180-degree change uses two right turns.
                    if difference in (1, 2):
                        direction = "right"
                        turn_heading = (heading + 1) % 4
                    else:
                        direction = "left"
                        turn_heading = (heading - 1) % 4

                    action = CardinalTurn(
                        direction,
                        now,
                        expected_yaw(turn_heading),
                    )

                    state = "TURN"

            elif state == "TURN":
                left_speed, right_speed = action.update(
                    now, yaw
                )

                if action.fault:
                    raise RuntimeError(action.fault)

                if action.complete:
                    heading = turn_heading
                    action = None
                    state = "ALIGN"
                    left_speed = 0.0
                    right_speed = 0.0

            elif state == "MOVE":
                left_speed, right_speed = action.update(
                    now=now,
                    left_position=left_position,
                    right_position=right_position,
                    yaw=yaw,
                    front_distance=readings["front"],
                )

                if action.fault:
                    # Position may be between cells.
                    # Do not advance the map coordinates.
                    raise RuntimeError(action.fault)

                if (returning and not return_path and final_entry_started is None
                        and action.distance >= FINAL_ENTRY_DISTANCE):
                    final_entry_started = now

                if action.complete:
                    record_wall(current, heading, False)
                    current = destination

                    if returning:
                        # DFS stack is finished; don't modify path.
                        print(f"RETURN ARRIVED at {current}")
                        action = None
                        left_speed = 0.0
                        right_speed = 0.0

                        if return_path:
                            # Advance to the next hop in the BFS route.
                            destination = return_path.pop(0)
                            diff = (
                                destination[0] - current[0],
                                destination[1] - current[1],
                            )
                            destination_heading = DIRECTIONS.index(diff)
                            state = "ALIGN"
                        else:
                            # Reached the end cell.
                            state = "FINISHED"

                    elif backtracking:
                        path.pop()
                        print(f"ARRIVED at {current}")
                        action = None
                        scan_samples = []
                        state = "SCAN"
                        state_started = now
                        left_speed = 0.0
                        right_speed = 0.0

                    else:
                        path.append(current)
                        print(f"ARRIVED at {current}")
                        action = None
                        scan_samples = []
                        state = "SCAN"
                        state_started = now
                        left_speed = 0.0
                        right_speed = 0.0

            elif state == "FINISHED":
                # Motors stay at zero (no speed assignments here).
                if not led_done:
                    response_time = (
                        0.0 if final_entry_started is None else now - final_entry_started
                    )
                    if response_time > 3.0:
                        raise RuntimeError("End-cell stop exceeded the 3-second deadline")
                    led_blue.set(0)
                    led_green.set(0)
                    if majority == "blue":
                        led_blue.set(1)
                        print("FINISHED: majority=blue; led_blue ON.")
                    elif majority == "green":
                        led_green.set(1)
                        print("FINISHED: majority=green; led_green ON.")
                    else:
                        print("FINISHED: no valid majority; both LEDs off.")
                    led_done = True
                    print(
                        f"RUN RESULT | elapsed={now - run_started:.3f} s | "
                        f"visited={len(visited)} | tiles={floor_map.counts()} | "
                        f"peak_tilt={math.degrees(peak_tilt):.2f} deg | "
                        f"final_response={response_time:.3f} s"
                    )

            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

            if now - last_report >= 0.5:
                print(
                    f"{state} | cell={current} | "
                    f"heading={HEADING_NAMES[heading]} | "
                    f"visited={len(visited)} | "
                    f"pitch={math.degrees(pitch):.1f}"
                )
                last_report = now

    except RuntimeError as error:
        print(f"MAZE STOPPED: {error}")
        print(
            f"Last confirmed cell={current}, "
            f"heading={HEADING_NAMES[heading]}"
        )

    finally:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
