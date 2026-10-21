"""
main_controller.py

The simplest possible starter controller for TwoWheelRobot.
It does NOT try to avoid walls - it just drives around randomly and will
bump into whatever is in its way. Both LEDs are also switched on so you
can visually confirm they work. This is meant purely as a placeholder /
example for teams to build on and edit themselves.

File placement:
  controllers/main_controller/main_controller.py
(folder name and file name must match exactly)
"""

from controller import Robot
import random

TIME_STEP = 64
MAX_SPEED = 6.0

robot = Robot()

left_motor = robot.getDevice('left_wheel_motor')
right_motor = robot.getDevice('right_wheel_motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# --- LED test: turn both on and leave them on the whole time ---
led_blue = robot.getDevice('led_blue')
led_green = robot.getDevice('led_green')
led_blue.set(1)
led_green.set(1)

# how many simulation steps to keep doing the current action before
# picking a new random one
steps_left = 0
left_speed = MAX_SPEED
right_speed = MAX_SPEED

while robot.step(TIME_STEP) != -1:
    if steps_left <= 0:
        # pick a new random action: mostly drive forward, sometimes turn
        action = random.choice(['forward', 'forward', 'forward', 'turn_left', 'turn_right'])

        if action == 'forward':
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED
            steps_left = random.randint(20, 60)   # keep going straight a while
        elif action == 'turn_left':
            left_speed = -MAX_SPEED * 0.5
            right_speed = MAX_SPEED * 0.5
            steps_left = random.randint(5, 15)
        else:  # turn_right
            left_speed = MAX_SPEED * 0.5
            right_speed = -MAX_SPEED * 0.5
            steps_left = random.randint(5, 15)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    steps_left -= 1
