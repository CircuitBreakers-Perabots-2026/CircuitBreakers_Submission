from controller import Robot
import math
from turning import Turn90

TIME_STEP = 64

MODE = "turn_right"

SETTLE_SECONDS = 1.0
REPORT_INTERVAL = 0.25

MAX_SPEED = 2.0 # Wheel angular speed, rad/s.
MIN_APPROACH_SPEED = 0.25
COMMAND_ACCELERATION = 0.5  # Command change limit, rad/s².

STOP_DISTANCE = 0.08  # Metres from the sensor.
SLOW_DISTANCE = 0.18  # Begin slowing within this distance.

if MODE not in ("read_only", "approach", "turn_left", "turn_right"):
    raise ValueError("Unknown exercise mode: " + MODE)

robot = Robot()

# Motors
left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")

for motor in (left_motor, right_motor):
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

# LEDs
led_blue = robot.getDevice("led_blue")
led_green = robot.getDevice("led_green")
led_blue.set(0)
led_green.set(0)

# Distnace Sensors
front_sensor = robot.getDevice("front_distance")
left_sensor = robot.getDevice("left_distance")
right_sensor = robot.getDevice("right_distance")

for sensor in (front_sensor, left_sensor, right_sensor):
    sensor.enable(TIME_STEP)

# Inertial Sensor
imu = robot.getDevice("imu")
imu.enable(TIME_STEP)

if MODE in ("turn_left", "turn_right"):
    direction = "left" if MODE == "turn_left" else "right"
    turn = Turn90(direction, robot.getTime())
    last_report = -REPORT_INTERVAL
    previous_status = None
    while robot.step(TIME_STEP) != -1:
        now = robot.getTime()
        roll, pitch, yaw = imu.getRollPitchYaw()
        left_speed, right_speed = turn.update(now, yaw)
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        if turn.status != previous_status or now - last_report >= REPORT_INTERVAL:
            error_text = (
                f"{math.degrees(turn.error):.1f} deg"
                if turn.error is not None else "waiting"
            )
            print(
                f"{turn.status} | direction={direction} | "
                f"yaw={math.degrees(yaw):.1f} deg | "
                f"change={math.degrees(turn.change):.1f} deg | "
                f"error={error_text} | pitch={math.degrees(pitch):.1f} deg"
            )
            previous_status = turn.status
            last_report = now

    raise SystemExit

start_time = robot.getTime()
last_report = -REPORT_INTERVAL

command_speed = 0.0
stop_reason = None

dt = TIME_STEP / 1000.0
max_speed_change = COMMAND_ACCELERATION * dt

print(f"Starting sensor exercise. Mode: {MODE}")

while robot.step(TIME_STEP) != -1:
    now = robot.getTime()
    elapsed = now - start_time
    distance = front_sensor.getValue()

    valid_reading = (
        math.isfinite(distance)
        and 0.0 <= distance <= 1.0
    )

    target_speed = 0.0
    immediate_stop = False

    roll, pitch, yaw = imu.getRollPitchYaw()

    print(
    f"Heading: {math.degrees(yaw):.1f} degrees | "
    f"Pitch: {math.degrees(pitch):.1f} degrees"
    )

    if elapsed < SETTLE_SECONDS:
        state = "Settling"

    elif not valid_reading:
        stop_reason = "Invalid sensor reading"
        state = stop_reason
        immediate_stop = True

    elif MODE == "read_only":
        state = "Reading sensor; motors stopped"

    elif stop_reason is not None:
        state = stop_reason

    elif distance <= STOP_DISTANCE:
        stop_reason = "Obstacle reached stopping threshold"
        state = stop_reason
        immediate_stop = True
        print(stop_reason)

    else:
        state = "Approaching obstacle"

        # Scale the desired speed as the obstacle gets closer.
        fraction = (
            (distance - STOP_DISTANCE)
            / (SLOW_DISTANCE - STOP_DISTANCE)
        )

        target_speed = min(
            MAX_SPEED,
            max(MIN_APPROACH_SPEED, MAX_SPEED * fraction)
        )

    if immediate_stop:
        command_speed = 0.0
    else:
        # Gradually change the speed command.
        difference = target_speed - command_speed
        change = max(
            -max_speed_change,
            min(max_speed_change, difference)
        )
        command_speed += change

    left_motor.setVelocity(command_speed)
    right_motor.setVelocity(command_speed)

    if now - last_report >= REPORT_INTERVAL:
        left_distance = left_sensor.getValue()
        right_distance = right_sensor.getValue()
        distance_text = (
            f"{distance:.3f} m"
            if valid_reading
            else "INVALID"
        )

        print(
            f"{state} | "
            f"front={distance_text} | "
            f"left={left_distance:.3f} m | "
            f"right={right_distance:.3f} m | "
            f"wheel command={command_speed:.2f} rad/s"
        )
        last_report = now
