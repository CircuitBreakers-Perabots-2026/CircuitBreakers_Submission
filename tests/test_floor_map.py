"""Pure-logic tests for floor_map: consensus, unique tile records,
majority outcomes, and BFS return-path planning.

Run from the repository root:
    python -m unittest discover tests
"""

import os
import sys
import unittest


sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "..", "controllers",
                 "main_controller"),
)

from floor_map import (
    DuplicateEndCell,
    FloorMap,
    FloorObservationConflict,
    bfs_path,
    consensus,
)


ROWS = 4
COLUMNS = 9


def open_grid(rows=ROWS, columns=COLUMNS):
    """A wall map with every in-grid passage confirmed open."""
    walls = {}
    for row in range(rows):
        for column in range(columns):
            walls[(row, column)] = [False, False, False, False]
    return walls


def close_boundary(walls, rows=ROWS, columns=COLUMNS):
    """Confirm the outer boundary of the grid as walled.

    Directions are N/E/S/W (0-3); N increases the row, so the outside
    neighbour of row 0 is to the SOUTH and of column 0 is to the WEST.
    """
    for row in range(rows):
        for column in range(columns):
            cell = (row, column)
            if row == rows - 1:
                walls[cell][0] = True   # N edge
            if column == columns - 1:
                walls[cell][1] = True   # E edge
            if row == 0:
                walls[cell][2] = True   # S edge
            if column == 0:
                walls[cell][3] = True   # W edge
    return walls


class ConsensusTests(unittest.TestCase):
    def test_unanimous_five_frames(self):
        self.assertEqual(
            consensus(["blue"] * 5),
            "blue",
        )

    def test_quorum_of_four(self):
        self.assertEqual(
            consensus(["blue", "blue", "blue", "blue", "unknown"]),
            "blue",
        )

    def test_three_to_two_split_is_ambiguous(self):
        self.assertIsNone(
            consensus(["blue", "blue", "blue", "green", "green"])
        )

    def test_two_to_two_split_is_ambiguous(self):
        self.assertIsNone(
            consensus(["blue", "blue", "green", "green", "unknown"])
        )

    def test_all_unknown_never_confirms(self):
        self.assertIsNone(consensus(["unknown"] * 5))

    def test_unknown_mixed_never_wins(self):
        # 3 unknown + 2 known is below quorum anyway, and unknown
        # must never be returned even when it would "lead".
        self.assertIsNone(consensus(["unknown", "unknown",
                                     "unknown", "blue", "blue"]))

    def test_below_quorum_is_not_confirmed(self):
        # 4/5 agreeing would pass at the default quorum of 4;
        # a stricter quorum must reject it.
        samples = ["green", "green", "green", "green", "white"]
        self.assertEqual(consensus(samples), "green")
        self.assertIsNone(consensus(samples, quorum=5))

    def test_empty_input(self):
        self.assertIsNone(consensus([]))

    def test_unrecognised_label_rejected(self):
        with self.assertRaises(ValueError):
            consensus(["magenta"] * 5)

    def test_black_and_white_are_confirmable(self):
        self.assertEqual(consensus(["black"] * 5), "black")
        self.assertEqual(consensus(["white"] * 5), "white")
        self.assertEqual(consensus(["red"] * 5), "red")


class FloorMapTests(unittest.TestCase):
    def make_map(self):
        return FloorMap(ROWS, COLUMNS)

    def test_single_cell_record(self):
        floor_map = self.make_map()
        floor_map.record((0, 0), "green")
        self.assertEqual(floor_map.label_of((0, 0)), "green")
        self.assertIsNone(floor_map.label_of((0, 1)))

    def test_revisit_does_not_increase_count(self):
        floor_map = self.make_map()
        floor_map.record((0, 0), "green")
        floor_map.record((0, 0), "green")
        floor_map.record((0, 0), "green")
        floor_map.record((1, 1), "blue")
        floor_map.record((1, 1), "blue")
        counts = floor_map.counts()
        self.assertEqual(counts["green"], 1)
        self.assertEqual(counts["blue"], 1)

    def test_counts_ignored_outside_grid(self):
        floor_map = self.make_map()
        floor_map.record((-1, 0), "blue")
        floor_map.record((ROWS, 0), "green")
        self.assertEqual(floor_map.counts(), {"blue": 0, "green": 0})

    def test_contradiction_raises(self):
        floor_map = self.make_map()
        floor_map.record((2, 3), "blue")
        with self.assertRaises(FloorObservationConflict):
            floor_map.record((2, 3), "green")

    def test_unconfirmed_label_rejected(self):
        floor_map = self.make_map()
        with self.assertRaises(ValueError):
            floor_map.record((0, 0), "unknown")

    def test_neutral_colours_do_not_affect_counts(self):
        floor_map = self.make_map()
        floor_map.record((0, 0), "black")
        floor_map.record((0, 1), "white")
        floor_map.record((0, 2), "blue")
        counts = floor_map.counts()
        self.assertEqual(counts["blue"], 1)
        self.assertEqual(counts["green"], 0)

    def test_first_red_cell_recorded_as_end(self):
        floor_map = self.make_map()
        floor_map.record((3, 8), "red")
        self.assertEqual(floor_map.end_cell, (3, 8))
        # Red is not a tile colour: it never enters the counts.
        self.assertEqual(floor_map.counts(), {"blue": 0, "green": 0})

    def test_second_different_red_cell_raises(self):
        floor_map = self.make_map()
        floor_map.record((3, 8), "red")
        with self.assertRaises(DuplicateEndCell):
            floor_map.record((0, 1), "red")

    def test_re_red_cell_is_noop(self):
        floor_map = self.make_map()
        floor_map.record((3, 8), "red")
        floor_map.record((3, 8), "red")
        self.assertEqual(floor_map.end_cell, (3, 8))

    def test_red_outside_grid_not_an_end_candidate(self):
        floor_map = self.make_map()
        floor_map.record((-1, 0), "red")
        self.assertIsNone(floor_map.end_cell)

    def test_majority_blue(self):
        floor_map = self.make_map()
        floor_map.record((0, 0), "blue")
        floor_map.record((1, 1), "blue")
        floor_map.record((2, 2), "green")
        self.assertEqual(floor_map.majority(), "blue")

    def test_majority_green(self):
        floor_map = self.make_map()
        floor_map.record((0, 0), "green")
        floor_map.record((1, 1), "green")
        floor_map.record((1, 2), "blue")
        self.assertEqual(floor_map.majority(), "green")

    def test_tie_has_no_majority(self):
        floor_map = self.make_map()
        floor_map.record((0, 0), "blue")
        floor_map.record((1, 1), "green")
        self.assertIsNone(floor_map.majority())

    def test_no_tiles_has_no_majority(self):
        floor_map = self.make_map()
        self.assertIsNone(floor_map.majority())

    def test_reset_clears_records(self):
        floor_map = self.make_map()
        floor_map.record((0, 0), "green")
        floor_map.record((3, 8), "red")
        floor_map.reset()
        self.assertIsNone(floor_map.end_cell)
        self.assertEqual(floor_map.counts(), {"blue": 0, "green": 0})


class BfsPathTests(unittest.TestCase):
    def test_direct_route(self):
        walls = close_boundary(open_grid())
        path = bfs_path(walls, (0, 0), (0, 2))
        self.assertEqual(path, [(0, 1), (0, 2)])

    def test_detour_around_confirmed_wall(self):
        walls = close_boundary(open_grid())
        # Block the direct (0,0)->(0,1) passage on both sides.
        walls[(0, 0)][1] = True
        walls[(0, 1)][3] = True
        path = bfs_path(walls, (0, 0), (0, 1))
        self.assertEqual(path, [(1, 0), (1, 1), (0, 1)])

    def test_shortest_path_is_chosen(self):
        walls = close_boundary(open_grid())
        # Direct two-step route along row 0 beats any detour.
        path = bfs_path(walls, (0, 0), (0, 2))
        self.assertEqual(len(path), 2)

    def test_unknown_passage_never_traversed(self):
        # 1x2 grid: the only route between the cells is unscanned.
        walls = {
            (0, 0): [None, None, None, None],
            (0, 1): [None, None, None, None],
        }
        self.assertIsNone(bfs_path(walls, (0, 0), (0, 1), 1, 2))

        walls[(0, 0)][1] = False
        walls[(0, 1)][3] = False
        self.assertEqual(
            bfs_path(walls, (0, 0), (0, 1), 1, 2),
            [(0, 1)],
        )

    def test_detour_uses_only_confirmed_passages(self):
        walls = close_boundary(open_grid())
        walls[(0, 0)][1] = None  # direct route unscanned
        walls[(0, 1)][3] = None
        path = bfs_path(walls, (0, 0), (0, 1))
        self.assertEqual(path, [(1, 0), (1, 1), (0, 1)])

    def test_confirmed_wall_never_traversed(self):
        # 1x3 grid: middle passage confirmed walled; no detour exists.
        walls = {
            (0, 0): [True, True, True, True],
            (0, 1): [True, True, True, True],
            (0, 2): [True, True, True, True],
        }
        self.assertIsNone(bfs_path(walls, (0, 0), (0, 2), 1, 3))

        walls[(0, 0)][1] = False
        walls[(0, 1)][3] = False
        walls[(0, 1)][1] = True
        walls[(0, 2)][3] = True
        self.assertIsNone(bfs_path(walls, (0, 0), (0, 2), 1, 3))

    def test_one_sided_opening_rejected(self):
        # The neighbour disagrees: (0,0) says open east, (0,1) says
        # wall west. Both sides must confirm; hop is refused.
        walls = {
            (0, 0): [None, False, None, None],
            (0, 1): [None, None, None, True],
        }
        self.assertIsNone(bfs_path(walls, (0, 0), (0, 1), 1, 2))

    def test_unreachable_goal_returns_none(self):
        walls = close_boundary(open_grid())
        # Fully seal (0,1) with confirmed walls on all four sides.
        walls[(0, 1)][0] = True
        walls[(0, 1)][1] = True
        walls[(0, 1)][2] = True
        walls[(0, 1)][3] = True
        self.assertIsNone(bfs_path(walls, (0, 0), (0, 1)))

    def test_start_equals_goal(self):
        walls = open_grid()
        self.assertEqual(bfs_path(walls, (0, 0), (0, 0)), [])

    def test_goal_outside_grid_returns_none(self):
        walls = open_grid()
        self.assertIsNone(bfs_path(walls, (0, 0), (5, 5)))

    def test_start_outside_grid_returns_none(self):
        walls = open_grid()
        self.assertIsNone(bfs_path(walls, (-1, 0), (0, 1)))

    def test_missing_cell_entry_treated_as_unknown(self):
        # A neighbour missing from the wall map behaves like None
        # (unknown), so the hop is refused.
        walls = {(0, 0): [None, False, None, None]}
        self.assertIsNone(bfs_path(walls, (0, 0), (0, 1), 1, 2))


if __name__ == "__main__":
    unittest.main()
