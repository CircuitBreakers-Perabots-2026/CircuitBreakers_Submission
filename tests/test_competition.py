"""Integration boundaries: opening discovery, safe planning and section dispatch."""

import contextlib
import io
import math
import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), "..", "controllers", "main_controller"
))

import competition
import obstacle_navigation as section1
from test_floor_detection import Camera, Device
import main_controller


class StartingRobot:
    def __init__(self, rgb):
        self.devices = {name: Device() for name in (
            "left_wheel_motor", "right_wheel_motor", "led_blue", "led_green"
        )}
        self.devices["floor_camera"] = Camera(rgb)

    def getDevice(self, name):
        return self.devices[name]

    def step(self, period):
        return 0


class CompetitionTests(unittest.TestCase):
    def test_entrypoint_dispatches_each_competition_mode(self):
        for mode in ("full", "section1", "maze"):
            with self.subTest(mode=mode):
                robot = object()
                with patch.dict(sys.modules, {"controller": SimpleNamespace(Robot=lambda: robot)}), \
                        patch.object(sys, "argv", ["main_controller.py", "--mode", mode]), \
                        patch.object(main_controller, "run_course") as run:
                    main_controller.main()
                    run.assert_called_once_with(robot, 16, mode)

    def test_removed_exercise_mode_is_rejected_before_robot_startup(self):
        with patch.object(sys, "argv", ["main_controller.py", "--mode", "sequence"]), \
                contextlib.redirect_stderr(io.StringIO()), \
                patch.object(main_controller, "run_course") as run:
            with self.assertRaises(SystemExit):
                main_controller.main()
            run.assert_not_called()

    def test_auto_selection_handles_section1_and_dedicated_maze_attempts(self):
        for rgb, expected in [((120, 0, 0), "section1"),
                              ((110, 112, 111), "section1"),
                              ((22, 22, 22), "maze"),
                              ((0, 45, 0), "maze")]:
            with self.subTest(rgb=rgb):
                self.assertEqual(competition.starting_section(StartingRobot(rgb), 16),
                                 expected)

    def test_camera_fault_does_not_choose_a_section(self):
        with self.assertRaises(RuntimeError):
            competition.starting_section(StartingRobot(None), 16)

    def test_failed_or_incomplete_entrance_never_starts_maze(self):
        with patch("obstacle_navigation.run_obstacle_field", return_value=None), \
                patch("maze_navigation.run_maze") as maze, \
                contextlib.redirect_stdout(io.StringIO()):
            robot = StartingRobot((120, 0, 0))
            competition.run_course(robot)
            maze.assert_not_called()
            self.assertEqual(robot.devices["left_wheel_motor"].value, 0)
            self.assertEqual(robot.devices["right_wheel_motor"].value, 0)

    def test_verified_handoff_passes_absolute_heading_to_maze(self):
        entry = section1.MazeEntrance(math.pi / 2, 100, 3)
        robot = StartingRobot((120, 0, 0))
        with patch("obstacle_navigation.run_obstacle_field", return_value=entry), \
                patch("maze_navigation.run_maze") as maze, \
                contextlib.redirect_stdout(io.StringIO()):
            competition.run_course(robot)
            maze.assert_called_once_with(robot, 16, entry_heading=math.pi / 2)

    def test_section1_mode_stops_without_starting_maze(self):
        entry = section1.MazeEntrance(math.pi / 2, 100, 3)
        with patch("obstacle_navigation.run_obstacle_field", return_value=entry), \
                patch("maze_navigation.run_maze") as maze, \
                contextlib.redirect_stdout(io.StringIO()):
            competition.run_course(StartingRobot((120, 0, 0)), mode="section1")
            maze.assert_not_called()

    def test_dedicated_maze_start_does_not_call_section1(self):
        robot = StartingRobot((22, 22, 22))
        with patch("obstacle_navigation.run_obstacle_field") as field, \
                patch("maze_navigation.run_maze") as maze, \
                contextlib.redirect_stdout(io.StringIO()):
            competition.run_course(robot)
            field.assert_not_called()
            maze.assert_called_once_with(robot, 16)

    def test_controller_argument_override_keeps_default_autonomous(self):
        self.assertEqual(competition.parse_mode("full", []), "full")
        self.assertEqual(competition.parse_mode("full", ["--mode", "maze"]), "maze")


class PlannerTests(unittest.TestCase):
    def test_six_beams_need_only_a_quarter_turn_for_scan_coverage(self):
        angles = sorted(angle % (2 * math.pi) for _, _, angle in section1.BEAMS.values())
        gaps = [b - a for a, b in zip(angles, angles[1:] + [angles[0] + 2 * math.pi])]
        self.assertLessEqual(max(gaps), math.pi / 2 + 1e-8)

        class Scan:
            yaw = 0.0

            def hold(self, *args):
                pass

            def wheels(self, left, right):
                pass

            def frame(self):
                self.yaw += 0.05
                if self.yaw > 2 * math.pi:
                    raise AssertionError("Scan failed to finish")

        scan = Scan()
        section1.NavigationIO.scan(scan)
        self.assertGreaterEqual(scan.yaw, math.pi / 2)
        self.assertLess(scan.yaw, math.pi / 2 + 0.06)

    def test_sensor_hit_uses_its_actual_offset(self):
        grid = section1.OccupancyGrid()
        grid.integrate(0, 0, 0, math.pi / 2, 0.2, (0.003, 0.045))
        self.assertIn(section1.cell_of(0.003, 0.245), grid.occupied)
        self.assertLess(grid.logodds[section1.cell_of(0.003, 0.10)], 0)

    def test_robot_clearance_is_retained_in_every_planning_level(self):
        self.assertTrue(all(radius >= 0.06 for radius in section1.LEVEL_RADII))
        grid = section1.OccupancyGrid()
        grid.force_occupied(0.5, -0.5)
        for level in range(len(section1.RADII_CELLS)):
            self.assertFalse(section1.walkable(grid, section1.cell_of(0.525, -0.5), level))

    def test_outside_space_requires_sensor_evidence(self):
        grid = section1.OccupancyGrid()
        cell = section1.cell_of(0.8, section1.ARENA_Y[0] - 0.06)
        self.assertFalse(section1.walkable(grid, cell, 0))
        grid._apply(cell, -0.5)
        self.assertTrue(section1.walkable(grid, cell, 0))

    def test_obstacle_is_avoided_by_astar_and_smoothing(self):
        grid = section1.OccupancyGrid()
        grid.force_occupied(0.5, -0.5)
        route, level = section1.find_route(grid, (0.2, -0.5), (0.8, -0.5), 0.03)
        self.assertIsNotNone(route)
        self.assertGreater(len(route), 2)
        self.assertTrue(section1.path_clear(grid, (0.2, -0.5), route, level))

    def test_opening_is_discovered_at_different_boundary_locations(self):
        for centre in (0.6, 1.4):
            with self.subTest(centre=centre):
                grid = section1.OccupancyGrid()
                self.assertEqual(section1.boundary_openings(grid), [])
                # A measured opening on the far side, rather than a fixed goal.
                lo, hi = section1.ARENA_X
                for i in range(math.ceil((hi - lo) / section1.RES) + 1):
                    x = lo + i * section1.RES
                    if abs(x - centre) <= 0.10:
                        grid._apply(section1.cell_of(x, section1.ARENA_Y[0]), -0.5)
                openings = section1.boundary_openings(grid)
                self.assertEqual(len(openings), 1)
                goal, normal = openings[0]
                self.assertAlmostEqual(goal[0], centre, delta=0.03)
                self.assertAlmostEqual(normal, -math.pi / 2)
                self.assertLess(goal[1], section1.ARENA_Y[0])

    def test_unverified_corridor_does_not_produce_an_entrance(self):
        class BadCorridor:
            travelled = 0
            readings = {"left": 1, "right": 1, "front": 0.04}

            def hold(self, *args):
                pass

            def turn_to(self, *args):
                pass

            def frame(self):
                pass

        with self.assertRaisesRegex(RuntimeError, "verify the transition corridor"):
            section1.entrance_handoff(BadCorridor(), -math.pi / 2)


if __name__ == "__main__":
    unittest.main()
