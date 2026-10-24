"""Regression tests for shaded tiles and the maze's floor-scan recovery."""

import contextlib
import io
import math
import os
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), "..", "controllers", "main_controller"
))

from floor_detection import classify_floor_rgb, read_floor, read_floor_rgb
import maze_navigation as maze


class Camera:
    """Small BGRA camera buffer with the same channel access as Webots."""

    def __init__(self, rgb=None):
        self.rgb = rgb

    def enable(self, period):
        pass

    def getWidth(self):
        return 8

    def getHeight(self):
        return 8

    def getImage(self):
        if self.rgb is None:
            return None
        r, g, b = self.rgb
        return bytes((b, g, r, 255)) * 64

    @staticmethod
    def imageGetRed(image, width, x, y):
        return image[4 * (y * width + x) + 2]

    @staticmethod
    def imageGetGreen(image, width, x, y):
        return image[4 * (y * width + x) + 1]

    @staticmethod
    def imageGetBlue(image, width, x, y):
        return image[4 * (y * width + x)]


class FloorDetectionTests(unittest.TestCase):
    def test_shaded_tiles_do_not_become_unknown_or_black(self):
        for rgb, label in [
            ((0, 59, 0), "green"),  # Old 50..60 brightness gap.
            ((2, 35, 3), "green"),  # Below old black threshold.
            ((3, 2, 35), "blue"),
            ((45, 2, 3), "red"),
            ((0, 150, 0), "green"),
            ((0, 0, 150), "blue"),
        ]:
            with self.subTest(rgb=rgb):
                self.assertEqual(read_floor(Camera(rgb)), label)

    def test_neutral_floor_and_near_black_noise(self):
        for rgb in [(0, 0, 0), (22, 22, 22), (29, 30, 28), (2, 8, 3)]:
            with self.subTest(rgb=rgb):
                self.assertEqual(read_floor(Camera(rgb)), "black")
        self.assertEqual(read_floor(Camera((220, 222, 219))), "white")

    def test_orange_chassis_and_ambiguous_colours_are_not_end_tiles(self):
        for rgb in [(243, 160, 0), (255, 140, 0), (100, 100, 100),
                    (90, 90, 0), (30, 45, 30)]:
            with self.subTest(rgb=rgb):
                self.assertEqual(read_floor(Camera(rgb)), "unknown")

    def test_missing_or_invalid_images_do_not_confirm_a_tile(self):
        self.assertIsNone(read_floor_rgb(Camera()))
        self.assertEqual(read_floor(Camera()), "unknown")
        for rgb in [(float("nan"), 0, 0), (-1, 20, 0), (256, 0, 0)]:
            self.assertEqual(classify_floor_rgb(rgb), "unknown")

    def test_rgb_diagnostics_use_the_same_pixels_as_classification(self):
        self.assertEqual(read_floor_rgb(Camera((2, 59, 3))), (2, 59, 3))


class Device:
    def __init__(self, value=0.0):
        self.value = value

    def enable(self, period):
        pass

    def setPosition(self, position):
        pass

    def setVelocity(self, value):
        self.value = value

    def set(self, value):
        self.value = value

    def getValue(self):
        return self.value


class MazeRobot:
    """Sensor-driven navigation fixture; actions finish at cell centres.

    This tests the real state machine, classifier, map, BFS and LEDs,
    substituting only wheel-motion/turn execution for Webots physics.
    The small test layout and colours never enter the controller.
    """

    def __init__(self, unknown_frames=0):
        self.cell = (0, 0)
        self.heading = 0
        self.yaw = math.pi / 2
        self.time = 0.0
        self.unknown_frames = unknown_frames
        self.camera_reads = 0
        self.cells_seen = set()
        self.led_events = []
        self.devices = {
            name: Device() for name in (
                "left_wheel_motor", "right_wheel_motor", "left_wheel_sensor",
                "right_wheel_sensor", "led_blue", "led_green",
            )
        }
        self.camera = Camera()
        self.devices["floor_camera"] = self.camera
        original_image = self.camera.getImage

        def get_image():
            # Every scan and retry must take place with both motors stopped.
            for name in ("left_wheel_motor", "right_wheel_motor"):
                if self.devices[name].value != 0:
                    raise AssertionError("Floor sampled while moving")
            self.camera_reads += 1
            self.cells_seen.add(self.cell)
            self.camera.rgb = (
                (100, 100, 0) if self.camera_reads <= self.unknown_frames else
                {
                    (0, 0): (22, 22, 22),
                    (0, 1): (2, 59, 2),
                    (0, 2): (0, 0, 35),
                    (1, 0): (0, 35, 0),
                    (1, 1): (22, 22, 22),
                    (1, 2): (120, 0, 0),
                }[self.cell]
            )
            return original_image()

        self.camera.getImage = get_image
        for name, offset in [("front", 0), ("left", -1), ("right", 1)]:
            device = Device()
            device.getValue = lambda offset=offset: self.distance(offset)
            self.devices[name + "_distance"] = device
        imu = Device()
        imu.getRollPitchYaw = lambda: (0, 0, self.yaw)
        self.devices["imu"] = imu

        for name in ("led_blue", "led_green"):
            def set_led(value, name=name):
                self.devices[name].value = value
                if value:
                    self.led_events.append((name, self.cell))
            self.devices[name].set = set_led

    def distance(self, offset):
        neighbour = maze.neighbour(self.cell, (self.heading + offset) % 4)
        return 0.5 if maze.inside(neighbour) else 0.08

    def getDevice(self, name):
        return self.devices[name]

    def getTime(self):
        return self.time

    def step(self, period):
        self.time += period / 1000.0
        if self.time > 200:
            raise AssertionError("Navigation did not finish within the test budget")
        return -1 if self.led_events else 0


class MazeFloorIntegrationTests(unittest.TestCase):
    def run_fixture(self, robot):
        class Move:
            complete = False
            fault = None

            def __init__(self, **kwargs):
                pass

            def update(self, **kwargs):
                robot.cell = maze.neighbour(robot.cell, robot.heading)
                if not maze.inside(robot.cell):
                    raise AssertionError("Robot left the fixture grid")
                self.complete = True
                return (0.0, 0.0)

        class Turn:
            complete = False
            fault = None

            def __init__(self, direction, now, target_yaw):
                self.target = target_yaw

            def update(self, now, yaw):
                robot.yaw = self.target
                robot.heading = round((math.pi / 2 - self.target) / (math.pi / 2)) % 4
                self.complete = True
                return (0.0, 0.0)

        output = io.StringIO()
        with patch.multiple(maze, ROWS=2, COLUMNS=3, MoveForward=Move,
                            CardinalTurn=Turn), contextlib.redirect_stdout(output):
            maze.run_maze(robot, 100)
        self.assertEqual(robot.devices["left_wheel_motor"].value, 0)
        self.assertEqual(robot.devices["right_wheel_motor"].value, 0)
        return output.getvalue()

    def test_shaded_tiles_count_and_return_to_end_with_correct_led(self):
        robot = MazeRobot()
        log = self.run_fixture(robot)
        self.assertEqual(len(robot.cells_seen), 6)
        self.assertEqual(robot.led_events, [("led_green", (1, 2))])
        self.assertIn("tiles={'blue': 1, 'green': 2}", log)
        self.assertIn("RETURNING to end cell (1, 2)", log)
        self.assertNotIn("MAZE STOPPED", log)

    def test_temporary_uncertainty_retries_then_completes(self):
        robot = MazeRobot(unknown_frames=5)
        log = self.run_fixture(robot)
        self.assertIn("Retrying floor scan at (0, 0)", log)
        self.assertIn("attempt=2/3", log)
        self.assertEqual(robot.led_events, [("led_green", (1, 2))])

    def test_permanent_uncertainty_stops_without_inventing_counts_or_led(self):
        robot = MazeRobot(unknown_frames=100)
        log = self.run_fixture(robot)
        self.assertEqual(robot.camera_reads, 15)
        # Includes initial settling and wall scanning at a coarse 100ms step.
        self.assertLess(robot.time, 6)
        self.assertIn("after 3 scans; RGB=", log)
        self.assertEqual(robot.led_events, [])
        self.assertEqual(robot.cell, (0, 0))


if __name__ == "__main__":
    unittest.main()
