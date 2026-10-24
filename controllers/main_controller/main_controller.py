from controller import Robot
from turning import Turn90
from movement import MoveForward
from sequence import ActionSequence
import math

TIME_STEP = 16

# Available modes:
# "navigate"   : approach, stop, choose a side, turn, and continue
# "read_only"  : report sensor readings without moving
# "approach"   : approach one obstacle, then remain stopped
# "turn_left"  : perform one 90-degree left turn
# "turn_right" : perform one 90-degree right turn
# "move_cell"  : move forward by a specified distance
# "sequence"   : run a sequence of actions
# "maze"       : navigate through a maze
# "section1"   : cross the obstacle field and stop at the red transition zone
# "full"       : section1, then straight into the maze
MODE = "full"  # Change this to test different modes

SETTLE_SECONDS = 1.0
REPORT_INTERVAL = 0.25

MAX_SPEED = 0.5
MIN_APPROACH_SPEED = 0.15
COMMAND_ACCELERATION = 0.25

STOP_DISTANCE = 0.12
SLOW_DISTANCE = 0.22
RESUME_DISTANCE = 0.16

# Starting threshold for deciding whether a side is open.
SIDE_CLEARANCE = 0.15

STOP_HOLD_SECONDS = 1.0
RECHECK_SECONDS = 0.5

# Prevent repeated turns when no usable direction is found.
MAX_TURN_ATTEMPTS = 2

MAX_TILT = math.radians(10)

VALID_MODES = {
    "navigate",
    "read_only",
    "approach",
    "turn_left",
    "turn_right",
    "move_cell",
    "sequence",
    "maze",
    "section1",
    "full",
}
if MODE not in VALID_MODES:
    raise ValueError(f"Unknown mode: {MODE}")


def move_toward(current, target, maximum_change):
    """Limit the change in a wheel-speed command."""
    difference = target - current
    return current + max(
        -maximum_change,
        min(maximum_change, difference)
    )


def valid_distance(value):
    return math.isfinite(value) and 0.0 <= value <= 1.0


robot = Robot()

if MODE == "maze":
    from maze_navigation import run_maze

    run_maze(robot, TIME_STEP)
    raise SystemExit

if MODE == "section1":
    from obstacle_navigation import run_obstacle_field

    run_obstacle_field(robot, TIME_STEP)
    raise SystemExit

if MODE == "full":
    from obstacle_navigation import run_obstacle_field
    from maze_navigation import run_maze

    # run_obstacle_field returns True only if it reached the red zone.
    if run_obstacle_field(robot, TIME_STEP):
        run_maze(robot, TIME_STEP)
    raise SystemExit

# Motors
left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")

for motor in (left_motor, right_motor):
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

# Required decision LEDs remain off during this exercise.
robot.getDevice("led_blue").set(0)
robot.getDevice("led_green").set(0)

# Distance sensors
distance_sensors = {
    "front": robot.getDevice("front_distance"),
    "left": robot.getDevice("left_distance"),
    "right": robot.getDevice("right_distance"),
}

for sensor in distance_sensors.values():
    sensor.enable(TIME_STEP)

# Heading sensor
imu = robot.getDevice("imu")
imu.enable(TIME_STEP)

# Encoders
left_encoder = robot.getDevice("left_wheel_sensor")
right_encoder = robot.getDevice("right_wheel_sensor")

left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

dt = TIME_STEP / 1000.0
max_speed_change = COMMAND_ACCELERATION * dt

state = "SETTLING"
state_started = robot.getTime()
last_report = -REPORT_INTERVAL

command_left = 0.0
command_right = 0.0

turn = None
move = None
sequence = None

turn_attempts = 0
fault_reason = ""

print(f"Starting controller. Mode: {MODE}")

while robot.step(TIME_STEP) != -1:
    now = robot.getTime()

    distances = {
        name: sensor.getValue()
        for name, sensor in distance_sensors.items()
    }

    front_distance = distances["front"]
    left_distance = distances["left"]
    right_distance = distances["right"]

    roll, pitch, yaw = imu.getRollPitchYaw()

    distances_valid = all(
        valid_distance(value)
        for value in distances.values()
    )

    heading_valid = all(
        math.isfinite(value)
        for value in (roll, pitch, yaw)
    )

    desired_left = 0.0
    desired_right = 0.0

    # Turn90 already limits its wheel-command changes.
    direct_action_commands = False

    # Stop on invalid readings after the initial settling period.
    if state not in ("SETTLING", "READ_ONLY", "DONE", "FAULT"):
        if not distances_valid:
            state = "FAULT"
            fault_reason = "Invalid distance-sensor reading"

        elif not heading_valid:
            state = "FAULT"
            fault_reason = "Invalid IMU reading"

        elif abs(pitch) > MAX_TILT or abs(roll) > MAX_TILT:
            state = "FAULT"
            fault_reason = (
                "Excessive body tilt: "
                f"pitch={math.degrees(pitch):.1f} deg, "
                f"roll={math.degrees(roll):.1f} deg"
            )

    if state == "SETTLING":
        if now - state_started >= SETTLE_SECONDS:
            if MODE == "read_only":
                state = "READ_ONLY"

            elif MODE in ("turn_left", "turn_right"):
                direction = (
                    "left" if MODE == "turn_left" else "right"
                )
                turn = Turn90(direction, now)
                state = "TURNING"
            elif MODE == "move_cell":
                move = MoveForward(
                    target_distance=0.25,
                    start_time=now,
                    left_position=left_encoder.getValue(),
                    right_position=right_encoder.getValue(),
                    yaw=yaw,
                )
                state = "MOVE_CELL"
            elif MODE == "sequence":
                sequence = ActionSequence(
                    steps=[
                        ("move", 0.25),
                        ("turn", "right"),
                        ("move", 0.25),
                    ],
                    start_time=now,
                )
                state = "SEQUENCE"
            else:
                state = "FORWARD"

            state_started = now

    elif state == "READ_ONLY":
        # Motors remain stopped.
        pass

    elif state == "SEQUENCE":
        desired_left, desired_right = sequence.update(
            now=now,
            left_position=left_encoder.getValue(),
            right_position=right_encoder.getValue(),
            yaw=yaw,
            front_distance=front_distance,
        )

        direct_action_commands = True

        if sequence.fault:
            state = "FAULT"
            fault_reason = sequence.fault

        elif sequence.complete:
            state = "DONE"
            print("SEQUENCE COMPLETE. Holding stopped.")

    elif state == "MOVE_CELL":
        desired_left, desired_right = move.update(
            now=now,
            left_position=left_encoder.getValue(),
            right_position=right_encoder.getValue(),
            yaw=yaw,
            front_distance=front_distance,
        )

        direct_action_commands = True

        if move.fault:
            state = "FAULT"
            fault_reason = move.fault

        elif move.complete:
            state = "DONE"

            print(
                "MOVE COMPLETE: "
                f"distance={move.distance:.3f} m"
            )

    elif state == "FORWARD":
        if front_distance <= STOP_DISTANCE:
            if MODE == "approach":
                state = "DONE"
                print("Approach complete. Holding stopped.")
            else:
                state = "STOPPING"
                state_started = now
                print("Obstacle detected. Stopping.")

        else:
            fraction = (
                (front_distance - STOP_DISTANCE)
                / (SLOW_DISTANCE - STOP_DISTANCE)
            )

            speed = min(
                MAX_SPEED,
                max(
                    MIN_APPROACH_SPEED,
                    MAX_SPEED * fraction
                )
            )

            desired_left = speed
            desired_right = speed

    elif state == "STOPPING":
        commands_stopped = (
            abs(command_left) <= 0.02
            and abs(command_right) <= 0.02
        )

        if (
            commands_stopped
            and now - state_started >= STOP_HOLD_SECONDS
        ):
            state = "CHOOSE_TURN"
            state_started = now

    elif state == "CHOOSE_TURN":
        left_clear = left_distance > SIDE_CLEARANCE
        right_clear = right_distance > SIDE_CLEARANCE

        if turn_attempts >= MAX_TURN_ATTEMPTS:
            state = "FAULT"
            fault_reason = (
                "No clear forward route after repeated turns"
            )

        elif not left_clear and not right_clear:
            state = "FAULT"
            fault_reason = (
                "Both sides blocked; dead-end recovery "
                "is not implemented yet"
            )

        else:
            if left_clear and not right_clear:
                direction = "left"

            elif right_clear and not left_clear:
                direction = "right"

            else:
                direction = (
                    "left"
                    if left_distance >= right_distance
                    else "right"
                )

            # Create the action once, when entering TURNING.
            turn = Turn90(direction, now)
            turn_attempts += 1

            state = "TURNING"
            state_started = now

            print(
                f"Choosing {direction}: "
                f"left={left_distance:.3f} m, "
                f"right={right_distance:.3f} m"
            )

    elif state == "TURNING":
        desired_left, desired_right = turn.update(now, yaw)
        direct_action_commands = True

        if turn.fault:
            state = "FAULT"
            fault_reason = turn.fault
            desired_left = 0.0
            desired_right = 0.0

        elif turn.complete:
            desired_left = 0.0
            desired_right = 0.0

            print(
                "Turn complete: "
                f"change={math.degrees(turn.change):.1f} deg, "
                f"error={math.degrees(turn.error):.1f} deg"
            )

            if MODE in ("turn_left", "turn_right"):
                state = "DONE"
            else:
                state = "RECHECK"

            state_started = now

    elif state == "RECHECK":
        if now - state_started >= RECHECK_SECONDS:
            if front_distance > RESUME_DISTANCE:
                state = "FORWARD"
                turn_attempts = 0
                print("Front clear. Continuing forward.")
            else:
                state = "CHOOSE_TURN"
                print("Front still blocked. Rechecking sides.")

            state_started = now

    elif state in ("DONE", "FAULT"):
        # Remain stopped until the simulation is reset.
        pass

    # Apply the final wheel commands once per iteration.
    if state in ("READ_ONLY", "DONE", "FAULT"):
        command_left = 0.0
        command_right = 0.0

    elif direct_action_commands:
        command_left = desired_left
        command_right = desired_right

    else:
        command_left = move_toward(
            command_left,
            desired_left,
            max_speed_change
        )

        command_right = move_toward(
            command_right,
            desired_right,
            max_speed_change
        )

    # Set the wheel speeds for this iteration.
    left_motor.setVelocity(command_left)
    right_motor.setVelocity(command_right)

    # Report the current state and sensor readings at a fixed interval.
    if now - last_report >= REPORT_INTERVAL:
        print(
            f"{state} | "
            f"front={front_distance:.3f} m | "
            f"left={left_distance:.3f} m | "
            f"right={right_distance:.3f} m | "
            f"yaw={math.degrees(yaw):.1f} deg | "
            f"pitch={math.degrees(pitch):.1f} deg | "
            f"motors=({command_left:.2f}, "
            f"{command_right:.2f})"
        )


        if state == "FAULT":
            print(f"Reason: {fault_reason}")

        if move is not None:
            print(
                f"Movement: {move.status} | "
                f"distance={move.distance:.3f} m | "
                f"target={move.target_distance:.3f} m"
            )

        if sequence is not None:
            print(f"Sequence: {sequence.status}")

        last_report = now