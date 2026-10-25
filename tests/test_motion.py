"""Motion-profile checks with ideal wheel encoders and differential-drive yaw.

These check braking, command limits and heading wrap; real traction and
body stability are verified separately in the Webots maze.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), "..", "controllers", "main_controller"
))

from movement import MoveForward
from turning import Turn90, angle_difference
from maze_navigation import CardinalTurn


class MotionTests(unittest.TestCase):
    def test_final_cell_stop_and_led_budget_after_full_body_entry(self):
        for dt in (0.016, 0.032, 0.064):
            with self.subTest(dt=dt):
                move = MoveForward(0.22, 0, 0, 0, 0)
                now = position = 0.0
                entered_at = None
                while now < 12:
                    now += dt
                    commands = move.update(now, position, position, 0, 1)
                    # Include 11mm of longitudinal positioning uncertainty:
                    # nominal whole-body entry is at about 0.171m.
                    if position * 0.02 >= 0.16 and entered_at is None:
                        entered_at = now
                    position += sum(commands) / 2 * dt
                    if move.complete or move.fault:
                        break
                self.assertTrue(move.complete)
                self.assertIsNone(move.fault)
                self.assertIsNotNone(entered_at)
                self.assertLess(now - entered_at + dt, 3)
                self.assertLess(abs(position * 0.02 - 0.25) + 0.04, 0.119)

    def test_cell_move_brakes_to_target_with_bounded_acceleration(self):
        for dt in (0.016, 0.032, 0.064):
            with self.subTest(dt=dt):
                move = MoveForward(0.25, 0, 0, 0, 0)
                position = now = peak_speed = 0.0
                previous = (0.0, 0.0)
                while now < 12:
                    now += dt
                    commands = move.update(now, position, position, 0, 1)
                    for command, old in zip(commands, previous):
                        self.assertLessEqual(abs(command), move.max_speed)
                        self.assertLessEqual(
                            abs(command - old), move.acceleration * dt + 1e-9
                        )
                    peak_speed = max(peak_speed, *commands)
                    position += sum(commands) / 2 * dt
                    previous = commands
                    if move.complete or move.fault:
                        break
                self.assertIsNone(move.fault)
                self.assertTrue(move.complete)
                self.assertLess(now, 10)
                self.assertGreater(peak_speed, 1.9)
                self.assertAlmostEqual(position * 0.02, 0.25, delta=0.002)
                self.assertEqual(commands, (0.0, 0.0))

    def test_fast_turns_both_directions_across_yaw_wrap(self):
        for dt in (0.016, 0.032, 0.064):
            for direction, sign in (("left", 1), ("right", -1)):
                with self.subTest(dt=dt, direction=direction):
                    yaw = sign * math.radians(179)
                    target = angle_difference(yaw + sign * math.pi / 2, 0)
                    turn = Turn90(direction, 0)
                    now = 0.0
                    previous = (0.0, 0.0)
                    while now < 12:
                        now += dt
                        commands = turn.update(now, yaw)
                        for command, old in zip(commands, previous):
                            self.assertLessEqual(abs(command), turn.max_speed)
                            self.assertLessEqual(
                                abs(command - old), turn.acceleration * dt + 1e-9
                            )
                        left, right = commands
                        yaw = angle_difference(
                            yaw + (right - left) * 0.02 / 0.064 * dt, 0
                        )
                        previous = commands
                        if turn.complete or turn.fault:
                            break
                    self.assertIsNone(turn.fault)
                    self.assertTrue(turn.complete)
                    self.assertLess(now, 5)
                    self.assertLessEqual(abs(angle_difference(target, yaw)),
                                         math.radians(2))
                    self.assertEqual(commands, (0.0, 0.0))

    def test_cardinal_turn_uses_absolute_target_after_shorter_settling(self):
        target = -math.pi / 2
        turn = CardinalTurn("right", 0, target)
        turn.update(0.1, 0.02)
        self.assertIsNone(turn.target_yaw)
        commands = turn.update(0.26, 0.02)
        self.assertEqual(turn.target_yaw, target)
        self.assertGreater(commands[0], 0)
        self.assertLess(commands[1], 0)

    def test_blocked_or_invalid_reading_still_stops_immediately(self):
        for front, yaw in ((0.04, 0), (1, float("nan")),
                           (1, math.radians(20))):
            with self.subTest(front=front, yaw=yaw):
                move = MoveForward(0.25, 0, 0, 0, 0)
                move.left_command = move.right_command = 2
                self.assertEqual(move.update(0.1, 0, 0, yaw, front), (0, 0))
                self.assertTrue(move.fault)


if __name__ == "__main__":
    unittest.main()
