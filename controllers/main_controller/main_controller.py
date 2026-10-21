from controller import Robot

TIME_STEP = 64

# Test "forward" first, then "left".
TEST = "forward"
PEAK_SPEED = 2.0  # Wheel angular speed, radians/second.

robot = Robot()

left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")

for motor in (left_motor, right_motor):
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

robot.getDevice("led_blue").set(0)
robot.getDevice("led_green").set(0)

start_time = robot.getTime()
previous_stage = None

while robot.step(TIME_STEP) != -1:
    elapsed = robot.getTime() - start_time

    if elapsed < 1.0:
        stage = "Holding still"
        speed = 0.0
    elif elapsed < 3.0:
        stage = "Accelerating"
        speed = PEAK_SPEED * (elapsed - 1.0) / 2.0
    elif elapsed < 5.0:
        stage = "Steady movement"
        speed = PEAK_SPEED
    elif elapsed < 7.0:
        stage = "Decelerating"
        speed = PEAK_SPEED * (7.0 - elapsed) / 2.0
    else:
        stage = "Stopped"
        speed = 0.0

    if stage != previous_stage:
        print(stage)
        previous_stage = stage

    if TEST == "forward":
        left_motor.setVelocity(speed)
        right_motor.setVelocity(speed)
    elif TEST == "left":
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(speed)
    else:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)