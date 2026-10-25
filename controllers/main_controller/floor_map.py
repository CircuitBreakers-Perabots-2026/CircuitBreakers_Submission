"""Confirmed floor-colour records and return-path planning.

Pure logic: no Webots imports, so it can be unit-tested without a
simulation. The maze state machine collects raw labels from
``read_floor`` while stopped over a cell and hands them here; this
module only stores, counts, and plans from *confirmed* observations.

Development-arena convention (not a competition rule): a confirmed
**red** cell is treated as the end-cell candidate. No coordinates are
hardcoded; the red surface the robot actually scans is recorded.
"""

from collections import deque


# Labels that read_floor may return; "unknown" is never confirmable.
LABELS = ("red", "green", "blue", "black", "white", "unknown")

# Labels that may be stored as a confirmed cell colour.
CONFIRMABLE = ("red", "green", "blue", "black", "white")

# Tile labels that are counted for the final majority decision.
TILE_LABELS = ("blue", "green")

# Development-arena end-cell marker label.
END_LABEL = "red"

# Multi-frame consensus thresholds.
FLOOR_SAMPLES = 5   # raw frames collected while stopped over a cell
FLOOR_QUORUM = 4    # agreeing frames required for a confirmed label


class FloorObservationConflict(RuntimeError):
    """A cell already has a different confirmed colour."""


class DuplicateEndCell(RuntimeError):
    """A second, different confirmed end-cell candidate was recorded."""


def consensus(samples, quorum=FLOOR_QUORUM):
    """
    Reduce raw floor labels to one confirmed label, or None.

    A label is confirmed when at least ``quorum`` of the samples match
    it and no other label (including "unknown") matches as many.
    "unknown" itself is never returned.

    Returns:
        One of CONFIRMABLE, or None when the frames are ambiguous
        (splits, all-unknown, or below quorum).
    """
    if not samples:
        return None

    counts = {label: 0 for label in LABELS}
    for label in samples:
        if label not in LABELS:
            raise ValueError(f"Unrecognised floor label: {label!r}")
        counts[label] += 1

    best_label = None
    best_count = 0
    for label in sorted(counts):
        if counts[label] > best_count:
            best_label = label
            best_count = counts[label]

    if best_label is None or best_label == "unknown":
        return None

    # Ties (with any other label) are ambiguous: no confirmed colour.
    if any(
        counts[label] == best_count
        for label in LABELS
        if label != best_label
    ):
        return None

    if best_count < quorum:
        return None

    return best_label


def inside_cell(cell, rows, columns):
    row, column = cell
    return 0 <= row < rows and 0 <= column < columns


class FloorMap:
    """Confirmed floor colours per maze cell, plus tile counts and the
    end-cell candidate.

    Coordinates use the existing navigation convention: (row, column)
    tuples with row zero on the entrance side.
    """

    def __init__(self, rows, columns):
        self.rows = rows
        self.columns = columns
        self._cells = {}  # cell -> confirmed label
        self._end_cell = None

    def record(self, cell, label):
        """
        Store a confirmed colour for a cell.

        Re-recording the same label is a no-op (revisits never change
        counts). A different confirmed label for the same cell raises
        FloorObservationConflict; a second, different red cell raises
        DuplicateEndCell.
        """
        if label not in CONFIRMABLE:
            raise ValueError(f"Unconfirmed label cannot be stored: {label!r}")

        previous = self._cells.get(cell)
        if previous is not None:
            if previous != label:
                raise FloorObservationConflict(
                    f"Cell {cell}: confirmed {previous!r}, "
                    f"observed {label!r}"
                )
            return

        if (
            label == END_LABEL
            and not inside_cell(cell, self.rows, self.columns)
        ):
            # Red surfaces outside the maze grid are not end candidates.
            return

        if label == END_LABEL and self._end_cell is not None:
            raise DuplicateEndCell(
                f"Second end-cell candidate {cell}; "
                f"first was {self._end_cell}"
            )

        self._cells[cell] = label
        if label == END_LABEL:
            self._end_cell = cell

    def label_of(self, cell):
        """The confirmed label for a cell, or None if unconfirmed."""
        return self._cells.get(cell)

    def counts(self):
        """Unique blue/green tile counts, derived from stored cells."""
        result = {label: 0 for label in TILE_LABELS}
        for cell, label in self._cells.items():
            if label in TILE_LABELS and inside_cell(
                cell, self.rows, self.columns
            ):
                result[label] += 1
        return result

    @property
    def end_cell(self):
        """The confirmed in-grid red cell, or None."""
        return self._end_cell

    def majority(self):
        """
        "blue" or "green" from the unique tile counts.

        Returns None for a tie, including when no tiles were recorded.
        """
        blue, green = (self.counts()[label] for label in TILE_LABELS)
        if blue > green:
            return "blue"
        if green > blue:
            return "green"
        return None

    def reset(self):
        """Clear all records (e.g. between simulation runs)."""
        self._cells = {}
        self._end_cell = None


def _step(cell, direction):
    row_change, column_change = (
        (1, 0), (0, 1), (-1, 0), (0, -1)
    )[direction]
    return (cell[0] + row_change, cell[1] + column_change)


def bfs_path(walls, start, goal, rows=4, columns=9):
    """
    Shortest route through confirmed open passages, or None.

    ``walls`` uses the existing representation: walls[cell][d] is
    None (unknown), True (wall), or False (opening), for directions
    N/E/S/W (0-3). A hop is only allowed when BOTH cells confirm the
    passage as open; unknown (None) or confirmed-walled connections are
    never traversed.

    Returns:
        The list of cells after ``start`` (empty when start == goal),
        or None when the goal is not inside the grid or is unreachable.
    """
    if not (
        inside_cell(start, rows, columns)
        and inside_cell(goal, rows, columns)
    ):
        return None

    if start == goal:
        return []

    came_from = {start: None}
    queue = deque([start])

    while queue:
        cell = queue.popleft()
        for direction in range(4):
            if walls.get(cell, [None] * 4)[direction] is not False:
                continue

            adjacent = _step(cell, direction)
            if not inside_cell(adjacent, rows, columns):
                continue

            if walls.get(adjacent, [None] * 4)[(direction + 2) % 4] is not False:
                continue

            if adjacent in came_from:
                continue

            came_from[adjacent] = cell
            if adjacent == goal:
                path = []
                cursor = adjacent
                while cursor != start:
                    path.append(cursor)
                    cursor = came_from[cursor]
                path.reverse()
                return path

            queue.append(adjacent)

    return None
